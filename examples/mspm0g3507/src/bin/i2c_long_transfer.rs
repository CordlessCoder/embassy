//! Blocking I2C transfers longer than the controller's FIFO, against the U575 `i2c_target_long`.
//!
//! Wiring: `PB2` J1.9 SCL to `PB8` Arduino D15, `PB3` J1.10 SDA to `PB9` Arduino D14, shared ground,
//! 4.7 kΩ from each line to 3V3. Analyser `D0` on SCL, `D1` on SDA, `D2` on `PB13` J4.35.
//!
//! A transfer is one burst. `CCTR.CBLEN` is 12 bits, so the ceiling is 4095 bytes and the FIFO — eight
//! on this part — is a buffer the bytes are fed through, not the unit they move in. The controller
//! stretches SCL whenever the transmit FIFO runs dry or the receive FIFO fills (SLAU846 25.2.3.8), so
//! software being late costs bus time and not data.
//!
//! # A: lengths
//!
//! Write, read and write-read at 1, 7, 8, 9, 16, 64 and 256 bytes. Eight is the FIFO, so 7/8/9 straddle
//! the length the driver used to refuse outright. The target answers a ramp, so every byte is checked.
//!
//! # B: the ceiling
//!
//! 4096 is refused with [`Error::TransferLengthIsOverLimit`] before anything reaches the bus. Pure
//! software — the guard is on the length, so no target is involved and the 4 kB buffer is never sent.
//!
//! # C: does it stretch at all?
//!
//! `PB13` is driven high for the duration of one 256-byte write, so the capture can be cut to exactly
//! that transfer. **The question is whether SCL is ever held low longer than a bit period.**
//!
//! Expect it not to be, and that is the useful answer rather than a disappointing one: a *polling*
//! refill cannot fall behind. At 32 MHz a 400 kHz byte is ~720 cycles and the loop is a handful of
//! instructions, so the FIFO never empties and the stretch the design relies on is never exercised.
//! **What this phase establishes is that the safety net is not load-bearing on this path** — which is
//! why the interesting version of it is the asynchronous one, where the refill waits on an interrupt
//! that other handlers can delay. Re-run this phase when the async path moves to single-burst.
//!
//! # D: the SCL-low watchdog against a long transfer
//!
//! `Config::clock_low_timeout_us` programs `TIMEOUT_CTL` counter A to catch a target that stops
//! clocking. If it counts SCL low without caring who holds it, then any stretch we cause reports
//! [`Error::Timeout`] on a healthy bus and the two features cannot be combined.
//!
//! Phase C says we do not stretch here, so a pass is weak evidence — it says the pair composes *at this
//! transfer length and clock*, not that the counter ignores our own stretching. A failure would be
//! strong. Either way the result belongs in `ti_data_source_gaps.md`, which already records this counter
//! counting 520 functional clocks against a documented `(1 + TPR) x 12`.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::i2c::{Config, Error, I2c};
use embassy_time::Timer;
use panic_halt as _;

/// The address `i2c_target_long` answers on.
const TARGET_ADDR: u8 = 0x48;

/// Longest transfer the target can hold, from its own buffer.
const MAX_LEN: usize = 256;

/// Lengths phase A walks. 8 is the FIFO on this part, so 7/8/9 straddle the old limit.
const LENGTHS: [usize; 7] = [1, 7, 8, 9, 16, 64, MAX_LEN];

/// One past the burst-length field's ceiling, for phase B.
const OVER_CEILING: usize = 4096;

/// SCL-low watchdog for phase D. Far above one byte at any supported rate, far below a wedged bus.
const CLOCK_LOW_TIMEOUT_US: u32 = 5_000;

/// Let the target finish offering the read half a write is answered with.
const SETTLE_MS: u64 = 100;

/// Backing store for phase B's over-long slice. Never sent — the guard rejects it first.
static OVERSIZED: [u8; OVER_CEILING] = [0; OVER_CEILING];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // Brackets phase C's transfer so the capture can be cut to it.
    let mut marker = Output::new(p.PB13, Level::Low);

    let mut i2c = unwrap!(I2c::new_blocking(p.I2C1, p.PB2, p.PB3, Config::default()));

    // The ramp the target expects and answers with.
    let mut ramp = [0u8; MAX_LEN];
    for (i, byte) in ramp.iter_mut().enumerate() {
        *byte = i as u8;
    }
    let mut got = [0u8; MAX_LEN];

    let mut fail = 0u32;

    // ---- A: lengths ----
    info!("A: lengths");
    for len in LENGTHS {
        Timer::after_millis(SETTLE_MS).await;
        match i2c.blocking_write(TARGET_ADDR, &ramp[..len]) {
            Ok(()) => info!("  write {}: ok", len),
            Err(e) => {
                fail += 1;
                error!("  write {}: {}", len, e);
            }
        }

        Timer::after_millis(SETTLE_MS).await;
        got[..len].fill(0xAA);
        match i2c.blocking_read(TARGET_ADDR, &mut got[..len]) {
            Ok(()) => match first_mismatch(&got[..len]) {
                None => info!("  read {}: ok", len),
                Some(i) => {
                    fail += 1;
                    error!("  read {}: byte {} is {}, expected {}", len, i, got[i], i as u8);
                }
            },
            Err(e) => {
                fail += 1;
                error!("  read {}: {}", len, e);
            }
        }

        Timer::after_millis(SETTLE_MS).await;
        got[..len].fill(0xAA);
        match i2c.blocking_write_read(TARGET_ADDR, &ramp[..1], &mut got[..len]) {
            Ok(()) => match first_mismatch(&got[..len]) {
                None => info!("  write_read {}: ok", len),
                Some(i) => {
                    fail += 1;
                    error!("  write_read {}: byte {} is {}, expected {}", len, i, got[i], i as u8);
                }
            },
            Err(e) => {
                fail += 1;
                error!("  write_read {}: {}", len, e);
            }
        }
    }

    // ---- B: the ceiling ----
    info!("B: the ceiling");
    match i2c.blocking_write(TARGET_ADDR, &OVERSIZED) {
        Err(Error::TransferLengthIsOverLimit) => info!("  {} refused, as it should be", OVER_CEILING),
        Ok(()) => {
            fail += 1;
            error!("  {} was accepted; the length guard is not doing its job", OVER_CEILING);
        }
        Err(e) => {
            fail += 1;
            error!("  {} refused, but as {} rather than a length error", OVER_CEILING, e);
        }
    }

    // ---- C: does it stretch at all? ----
    info!("C: one {}-byte write, bracketed on PB13", MAX_LEN);
    Timer::after_millis(SETTLE_MS).await;

    marker.set_high();
    let result = i2c.blocking_write(TARGET_ADDR, &ramp[..MAX_LEN]);
    marker.set_low();

    match result {
        Ok(()) => info!("  ok — measure SCL low time inside the marker window"),
        Err(e) => {
            fail += 1;
            error!("  {}", e);
        }
    }

    // ---- D: the SCL-low watchdog against a long transfer ----
    info!("D: the same write with the SCL-low watchdog on");
    Timer::after_millis(SETTLE_MS).await;

    let mut watched = Config::default();
    watched.clock_low_timeout_us = Some(CLOCK_LOW_TIMEOUT_US);
    match i2c.set_config(watched) {
        Ok(()) => match i2c.blocking_write(TARGET_ADDR, &ramp[..MAX_LEN]) {
            Ok(()) => info!("  ok — the pair composes at this length and clock"),
            Err(Error::Timeout) => {
                fail += 1;
                error!(
                    "  Timeout on a healthy bus — the watchdog counts SCL low whoever holds it, so it \
                     cannot be combined with a transfer that outruns the FIFO"
                );
            }
            Err(e) => {
                fail += 1;
                error!("  unexpected: {}", e);
            }
        },
        Err(e) => {
            fail += 1;
            error!("  could not enable the watchdog: {}", e);
        }
    }

    if fail == 0 {
        info!("all phases ok");
    } else {
        error!("{} failures", fail);
    }

    loop {
        Timer::after_millis(1000).await;
    }
}

/// First index where `got` departs from the ramp.
fn first_mismatch(got: &[u8]) -> Option<usize> {
    got.iter().enumerate().find(|(i, b)| **b != *i as u8).map(|(i, _)| i)
}
