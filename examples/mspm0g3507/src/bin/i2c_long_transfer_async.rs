//! The asynchronous twin of `i2c_long_transfer`, against the U575 `i2c_target_long`.
//!
//! Wiring: `PB2` J1.9 SCL to `PB8` Arduino D15, `PB3` J1.10 SDA to `PB9` Arduino D14, shared ground,
//! 4.7 kΩ from each line to 3V3. Analyser `D0` on SCL, `D1` on SDA, `D2` on `PB13` J4.35.
//!
//! Same single-burst design as the blocking path — `CCTR.CBLEN` covers the whole transfer and the FIFO
//! is fed through it — but the refill runs from `CTXFIFOTRG`/`CRXFIFOTRG` instead of a poll loop.
//!
//! `TESTING.md` C19 found the blocking path never stretches SCL, because a poll loop cannot fall behind.
//! This binary was written expecting the interrupt-fed path to stretch instead — the transmit trigger is
//! `Empty`, so the FIFO is drained before a refill is even requested.
//!
//! **It does not, and that is the result.** Measured at 100 kHz the two paths are within 0.03 ms over a
//! 256-byte write, and the worst single byte is 84.36 us against a mean of 83.11. The refill latency
//! fits inside the SCL-low phase of the byte already in flight, so it never becomes bus time. **The
//! clock-stretching backpressure this design rests on is not exercised by either path at 100 kHz**.
//! It has since been reached the other way, by loading the CPU so the refill cannot keep up — see
//! `TESTING.md` C22, where the controller holds SCL low for 14.25 ms and loses nothing.
//!
//! # A: lengths
//!
//! As C19's phase A, at 1, 7, 8, 9, 16, 64 and 256 bytes, checking every byte against the ramp.
//!
//! # B: the ceiling
//!
//! 4096 is refused with [`Error::TransferLengthIsOverLimit`] before anything reaches the bus.
//!
//! # C: the stretch
//!
//! One 256-byte write and one 256-byte read, each bracketed on `PB13`, compared against C19's 22.951 ms
//! for the same write from the blocking path. Measures what interrupt-driven refill costs; the answer so
//! far is nothing, and see above.
//!
//! # D: the SCL-low watchdog against a stretching transfer
//!
//! `Config::clock_low_timeout_us` programs `TIMEOUT_CTL` counter A, the SCL-low watchdog. The worry was
//! that it counts SCL low without caring who holds it, so that our own refill latency would report
//! [`Error::Timeout`] on a healthy bus.
//!
//! **The counter's granularity settles this without needing the run.** Its step is `8320 / clock_hz`,
//! two to 255 of them, so from MFCLK the shortest timeout representable is **4.2 ms** — against a
//! worst-case refill stretch of about a microsecond. The watchdog is three and a half thousand times too
//! coarse to see it. The phase runs anyway, near the floor, to confirm nothing else intervenes.
//!
//! # E: cancellation part-way through a long transfer
//!
//! Single-burst makes this case bigger than it was: a cancel used to abandon at most a FIFO load, and now
//! it abandons a burst the controller is still running, anywhere inside it.
//!
//! Three steps, in this order, because the third only means something against the first:
//!
//! 1. **A short read cancelled**, then a short read — the control. `TESTING.md` C4 already covers this
//!    (760 cancels, none wedged), so a failure here means the rig, not the change.
//! 2. **A 256-byte read cancelled mid-burst**, then a short read.
//! 3. If that read reports [`Error::BusStuck`], **[`I2c::recover_stuck_bus`] and try once more.** Note
//!    the U575 is already on record not being rescued by nine clocks — it shifts data out and keeps
//!    holding SDA — so an unrecovered bus here is a statement about this target, not about the driver.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::i2c::{Config, Error, I2c};
use embassy_mspm0::{bind_interrupts, i2c, peripherals};
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C1 => i2c::InterruptHandler<peripherals::I2C1>;
});

/// The address `i2c_target_long` answers on.
const TARGET_ADDR: u8 = 0x48;

/// Longest transfer the target can hold, from its own buffer.
const MAX_LEN: usize = 256;

/// Lengths phase A walks. 8 is the FIFO on this part, so 7/8/9 straddle the old limit.
const LENGTHS: [usize; 7] = [1, 7, 8, 9, 16, 64, MAX_LEN];

/// One past the burst-length field's ceiling, for phase B.
const OVER_CEILING: usize = 4096;

/// SCL-low watchdog for phase D.
///
/// The counter's step is `8320 / clock_hz`, so from MFCLK the representable range is 4.2 ms to 530 ms and
/// this is very near the floor. That floor is the phase's real finding — see the module docs.
const CLOCK_LOW_TIMEOUT_US: u32 = 5_000;

/// How far into the 256-byte read phase E cancels. A 256-byte transfer takes ~23 ms, so this lands
/// solidly mid-burst.
const CANCEL_AFTER_MS: u64 = 5;

/// Let the target finish offering the read half a write is answered with.
const SETTLE_MS: u64 = 100;

/// Backing store for phase B's over-long slice. Never sent — the guard rejects it first.
static OVERSIZED: [u8; OVER_CEILING] = [0; OVER_CEILING];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut marker = Output::new(p.PB13, Level::Low);
    let mut i2c = unwrap!(I2c::new_async(p.I2C1, p.PB2, p.PB3, Irqs, Config::default()));

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
        match i2c.async_write(TARGET_ADDR, &ramp[..len]).await {
            Ok(()) => info!("  write {}: ok", len),
            Err(e) => {
                fail += 1;
                error!("  write {}: {}", len, e);
            }
        }

        Timer::after_millis(SETTLE_MS).await;
        got[..len].fill(0xAA);
        match i2c.async_read(TARGET_ADDR, &mut got[..len]).await {
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
        match i2c.async_write_read(TARGET_ADDR, &ramp[..1], &mut got[..len]).await {
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
    match i2c.async_write(TARGET_ADDR, &OVERSIZED).await {
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

    // ---- C: the stretch ----
    info!("C: {}-byte write then read, each bracketed on PB13", MAX_LEN);
    Timer::after_millis(SETTLE_MS).await;

    marker.set_high();
    let wrote = i2c.async_write(TARGET_ADDR, &ramp[..MAX_LEN]).await;
    marker.set_low();
    match wrote {
        Ok(()) => info!("  write ok — measure SCL low inside the first marker window"),
        Err(e) => {
            fail += 1;
            error!("  write: {}", e);
        }
    }

    Timer::after_millis(SETTLE_MS).await;
    got.fill(0xAA);
    marker.set_high();
    let readback = i2c.async_read(TARGET_ADDR, &mut got).await;
    marker.set_low();
    match readback {
        Ok(()) => match first_mismatch(&got) {
            None => info!("  read ok — second marker window"),
            Some(i) => {
                fail += 1;
                error!("  read: byte {} is {}, expected {}", i, got[i], i as u8);
            }
        },
        Err(e) => {
            fail += 1;
            error!("  read: {}", e);
        }
    }

    // ---- D: the SCL-low watchdog against a stretching transfer ----
    info!("D: the same write with the SCL-low watchdog on");
    Timer::after_millis(SETTLE_MS).await;

    let mut watched = Config::default();
    watched.clock_low_timeout_us = Some(CLOCK_LOW_TIMEOUT_US);
    match i2c.set_config(watched) {
        Ok(()) => match i2c.async_write(TARGET_ADDR, &ramp[..MAX_LEN]).await {
            Ok(()) => info!("  ok — counter A does not count our own stretching"),
            Err(Error::Timeout) => {
                fail += 1;
                error!(
                    "  Timeout on a healthy bus — counter A counts SCL low whoever holds it, so \
                     clock_low_timeout_us cannot be combined with an interrupt-fed transfer"
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

    // Back to the default before phase E, so a cancellation is not also a timeout test.
    if let Err(e) = i2c.set_config(Config::default()) {
        fail += 1;
        error!("  could not restore the default config: {}", e);
    }

    // ---- E: cancellation part-way through ----
    info!("E: cancellation");

    // The control: a short read cancelled is the case C4 already covers.
    Timer::after_millis(SETTLE_MS).await;
    got.fill(0xAA);
    match select(i2c.async_read(TARGET_ADDR, &mut got[..8]), Timer::after_micros(200)).await {
        Either::First(r) => info!("  short read finished before the cancel landed: {}", r),
        Either::Second(()) => info!("  short read cancelled"),
    }
    Timer::after_millis(SETTLE_MS).await;
    match read_back(&mut i2c, &mut got).await {
        Ok(()) => info!("  control: recovered from a short cancel"),
        Err(e) => {
            fail += 1;
            error!("  control: a short cancel did not recover: {}", e);
        }
    }

    // The case single-burst makes new: a cancel landing deep inside one burst.
    info!("  now a {}-byte read cancelled after {} ms", MAX_LEN, CANCEL_AFTER_MS);
    Timer::after_millis(SETTLE_MS).await;
    got.fill(0xAA);
    match select(
        i2c.async_read(TARGET_ADDR, &mut got),
        Timer::after_millis(CANCEL_AFTER_MS),
    )
    .await
    {
        Either::First(r) => warn!("  the long read finished before the cancel landed: {}", r),
        Either::Second(()) => info!("  cancelled mid-burst"),
    }

    Timer::after_millis(SETTLE_MS).await;
    match read_back(&mut i2c, &mut got).await {
        Ok(()) => info!("  recovered from a mid-burst cancel with no help"),
        Err(Error::BusStuck) => {
            warn!("  BusStuck after a mid-burst cancel; asking recover_stuck_bus");
            match i2c.recover_stuck_bus() {
                Ok(()) => {
                    Timer::after_millis(SETTLE_MS).await;
                    match read_back(&mut i2c, &mut got).await {
                        Ok(()) => info!("  recover_stuck_bus cleared it"),
                        Err(e) => {
                            fail += 1;
                            error!("  still stuck after recover_stuck_bus: {}", e);
                        }
                    }
                }
                Err(e) => warn!(
                    "  recover_stuck_bus could not clear it ({}) — expected against this target, which \
                     keeps holding SDA through nine clocks",
                    e
                ),
            }
        }
        Err(e) => {
            fail += 1;
            error!("  the read after a mid-burst cancel failed: {}", e);
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

/// An eight-byte read, checked against the ramp.
async fn read_back(i2c: &mut I2c<'static, embassy_mspm0::mode::Async>, got: &mut [u8]) -> Result<(), Error> {
    got[..8].fill(0xAA);
    i2c.async_read(TARGET_ADDR, &mut got[..8]).await?;
    match first_mismatch(&got[..8]) {
        None => Ok(()),
        Some(_) => Err(Error::Bus),
    }
}

/// First index where `got` departs from the ramp.
fn first_mismatch(got: &[u8]) -> Option<usize> {
    got.iter().enumerate().find(|(i, b)| **b != *i as u8).map(|(i, _)| i)
}
