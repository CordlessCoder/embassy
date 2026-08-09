//! Does a read the controller cut short leave its unsent bytes to answer the next one?
//!
//! The target half of a pair; `examples/stm32u575/src/bin/i2c_short_read.rs` supplies the traffic and
//! **holds the verdict**, because this end cannot see what actually reached the bus. Rig D:
//!
//! | Net | MSPM0 G3507 | U575 |
//! |---|---|---|
//! | SCL | `PB2` J1.9, `I2C1` | `PB8`, Arduino D15 |
//! | SDA | `PB3` J1.10, `I2C1` | `PB9`, Arduino D14 |
//! | GND | J1.20 | Arduino power header |
//!
//! 4.7 kΩ from each line to 3V3, once for the bus. Markers, on the same two pins the write-side
//! example uses:
//!
//! | Pin | Analyser | Meaning |
//! |---|---|---|
//! | `PB13` J4.35 | D2 | high across `listen` |
//! | `PB0` J2.12 | D3 | toggled once per completed round, so a capture can be cut at a failure |
//!
//! **Flash this end first and leave it listening**, then start the U575.
//!
//! # What is under test
//!
//! A controller that reads fewer bytes than the target offered leaves the rest in the transmit FIFO.
//! Nothing in `respond_to_read` removes them, so without the fix they are what the *next* read
//! returns — corruption that surfaces one transaction after the one that caused it.
//!
//! The fix is not a software flush between commands. That races a controller which starts reading
//! immediately after the STOP, and loses. `TCTR.TXWAIT_STALE_TXFIFO` makes the transmit state machine
//! treat the FIFO as empty at every STOP so the surplus is never shifted out at all, and the next
//! `respond_to_read` clears it while the bus is stretched — SLAU846 §25.2.3.13.1.
//!
//! # The pass
//!
//! Read it off the U575's log, or off an I2C decode of the bus, which is the better witness: every
//! second read must carry `C1 C2`. A read carrying `A2 A3` is the defect, and the byte says how far
//! into the previous answer the target had got.
//!
//! A third phase cancels a `respond_to_read` by dropping it mid-answer, which leaves the FIFO part
//! full with nothing reporting it — the case a flush on the return path could not have covered.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::i2c::{Address, ClockDiv, ClockSel, Config, Timing};
use embassy_mspm0::i2c_target::{Command, Config as TargetConfig, I2cTarget, ReadStatus};
use embassy_mspm0::peripherals::I2C1;
use embassy_mspm0::sysctl::clock;
use embassy_mspm0::{bind_interrupts, i2c};
use embassy_time::{Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C1 => i2c::InterruptHandler<I2C1>;
});

const TIMING: Timing = match Timing::solve(&clock::RESET_SETUP.clocks(), ClockSel::MfClk, ClockDiv::DivBy1, 100_000) {
    Some(timing) => timing,
    None => core::panic!("100 kHz is not reachable from MFCLK"),
};

/// Offered in full to the short read, which takes two. The remaining six are what must not survive.
///
/// Ascending from `0xA0` so a stale byte says how far the previous answer had got.
const ANSWER: [u8; 8] = [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7];

/// The next read's answer, unlike anything in `ANSWER` so a leftover is unmistakable.
const CANARY: [u8; 2] = [0xC1, 0xC2];

/// Unused — the controller only reads — but `listen` needs somewhere to put a write it never gets.
const BUFFER: usize = 8;

/// How long a `respond_to_read` waits before being dropped, cycled so the drop lands in different
/// places: before the controller has taken anything, and part way through the answer.
const CANCEL_AFTER: [Duration; 3] = [
    Duration::from_micros(80),
    Duration::from_micros(300),
    Duration::from_micros(700),
];

/// Cancel one answer in this many, so the cancellation path is exercised without starving the rest.
const CANCEL_EVERY: u32 = 7;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut in_listen = Output::new(p.PB13, Level::Low);
    let mut judged = Output::new(p.PB0, Level::Low);

    let config = Config::default().with_timing(TIMING);
    let mut target_config = TargetConfig::default();
    target_config.target_addr = Address::SevenBit(0x48);
    let mut i2c = I2cTarget::new(p.I2C1, p.PB2, p.PB3, Irqs, config, target_config).unwrap();

    let mut buffer = [0u8; BUFFER];

    let mut reads = 0u32;
    let mut leftover = 0u32;
    let mut done = 0u32;
    let mut cancelled = 0u32;
    let mut other = 0u32;

    info!("listening on 0x48, offering {} bytes to every other read", ANSWER.len());

    loop {
        in_listen.set_high();
        let command = i2c.listen(&mut buffer).await;
        in_listen.set_low();

        match command {
            Ok(Command::Read) => {
                reads += 1;

                // The controller alternates: a short read of the long answer, then a read of the
                // canary. Odd reads offer the surplus, even ones prove it did not survive.
                let response: &[u8] = if reads % 2 == 1 { &ANSWER } else { &CANARY };

                if reads % CANCEL_EVERY == 0 {
                    let after = CANCEL_AFTER[(reads / CANCEL_EVERY) as usize % CANCEL_AFTER.len()];

                    match select(i2c.respond_to_read(response), Timer::after(after)).await {
                        Either::First(_) => {}
                        Either::Second(()) => {
                            cancelled += 1;
                            if cancelled <= 3 {
                                warn!("read {}: dropped an answer after {} us", reads, after.as_micros());
                            }
                        }
                    }
                } else {
                    match i2c.respond_to_read(response).await {
                        // Expected on every short read, and the count is how many bytes were left.
                        Ok(ReadStatus::LeftoverBytes(_)) => leftover += 1,
                        Ok(_) => done += 1,
                        Err(_) => other += 1,
                    }
                }

                // One edge per completed round, so a capture can be cut at the round the wire decode
                // shows going wrong. It is not the verdict — this end cannot see what it sent.
                if reads % 2 == 0 {
                    judged.toggle();
                }
            }

            Ok(_) => other += 1,
            Err(_) => other += 1,
        }

        if reads % 200 == 0 && reads > 0 {
            info!(
                "{} reads: {} left bytes behind, {} completed, {} cancelled, {} other",
                reads, leftover, done, cancelled, other
            );
        }
    }
}
