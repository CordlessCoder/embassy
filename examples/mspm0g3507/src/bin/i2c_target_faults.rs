//! Does an over-length write leave bytes behind for the next transaction?
//!
//! The target half of a pair; `examples/stm32u575/src/bin/i2c_overlong.rs` supplies the traffic. Rig D:
//!
//! | Net | MSPM0 G3507 | U575 |
//! |---|---|---|
//! | SCL | `PB2` J1.9, `I2C1` | `PB8`, Arduino D15 |
//! | SDA | `PB3` J1.10, `I2C1` | `PB9`, Arduino D14 |
//! | GND | J1.20 | Arduino power header |
//!
//! 4.7 kΩ from each line to 3V3, once for the bus. Markers, for a capture that wants to see where the
//! driver was rather than only what crossed the wire:
//!
//! | Pin | Meaning |
//! |---|---|
//! | `PB13` J4.35 | high across `listen` |
//! | `PB0` J2.12 | toggled when a round is judged, so a capture can be cut at a failure |
//!
//! **Flash this end first and leave it listening**, then start the U575.
//!
//! # What is under test
//!
//! `listen` returns `PartialWrite` when the controller sends more than the buffer holds. Both of its
//! early returns skip the interrupt-mask cleanup that every other terminating path performs, and neither
//! drains what is left in the receive FIFO. The bytes that did not fit therefore stay there, and the next
//! `listen` delivers them as the head of the following write — corruption that surfaces one transaction
//! *after* the one that caused it, which is what makes it worth a dedicated example.
//!
//! A second phase cancels a `listen` by dropping it, which the driver has no `OnDrop` guard for, and then
//! checks the target still answers.
//!
//! # The pass
//!
//! Every canary write arrives as exactly `C1 C2`. Anything longer, or anything starting with a byte from
//! the over-length write, is the defect.
//!
//! **Read the verdict off `PB0`, not the log.** It toggles once per *clean* canary, so an edge count
//! equal to the number the controller sent means none were corrupted. RTT is not dependable here: a
//! burst of messages desynchronises defmt's framing and everything after it silently fails to decode,
//! which reads exactly like a target that has stopped. That misdiagnosis was made three times before the
//! pin count settled it.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::i2c::{Address, ClockDiv, ClockSel, Config, Timing};
use embassy_mspm0::i2c_target::{Command, Config as TargetConfig, Error, I2cTarget};
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

/// Deliberately smaller than the controller's over-length write, which is what produces `PartialWrite`.
const BUFFER: usize = 4;

/// What the controller sends between over-length writes.
const CANARY: [u8; 2] = [0xC1, 0xC2];

/// Answer to a read, so the controller's read phase completes.
const ANSWER: [u8; 2] = [0xA0, 0xA1];

/// How long a cancelled `listen` waits before being dropped, cycled so the drop lands in different
/// places.
///
/// The controller offers a transaction about every 2 ms and a 12-byte write at 100 kHz occupies rather
/// over 1 ms of that, so these straddle both cases: some cancels land on an idle bus, some in the middle
/// of a transfer the target is already receiving. **A timeout longer than the gap fires never** — the
/// first version of this used 3 ms and reported 0 cancellations out of 2000 while looking like it
/// worked.
const CANCEL_AFTER: [Duration; 3] = [
    Duration::from_micros(150),
    Duration::from_micros(600),
    Duration::from_micros(1100),
];

/// Cancel one `listen` in this many, so the cancellation path is exercised without starving the rest.
const CANCEL_EVERY: u32 = 5;

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

    let mut partials = 0u32;
    let mut canaries = 0u32;
    let mut corrupt = 0u32;
    let mut cancelled = 0u32;
    let mut reads = 0u32;
    let mut listens = 0u32;
    let mut other = 0u32;

    info!("listening on 0x48, buffer {} bytes", BUFFER);

    loop {
        listens += 1;

        in_listen.set_high();

        // Every `CANCEL_EVERY`th listen is dropped mid-flight. The driver takes no `OnDrop`, so if the
        // cancellation leaves interrupts armed or the bus stretched, the rounds after this one are where
        // it shows.
        let command = if listens % CANCEL_EVERY == 0 {
            let after = CANCEL_AFTER[(listens / CANCEL_EVERY) as usize % CANCEL_AFTER.len()];

            match select(i2c.listen(&mut buffer), Timer::after(after)).await {
                Either::First(command) => command,
                Either::Second(()) => {
                    cancelled += 1;
                    if cancelled <= 3 {
                        warn!("round {}: cancelled a listen after {} us", listens, after.as_micros());
                    }
                    in_listen.set_low();
                    continue;
                }
            }
        } else {
            i2c.listen(&mut buffer).await
        };

        in_listen.set_low();

        match command {
            Ok(Command::Write(len)) => {
                // The only writes on this bus are the canary, so anything else is the leftovers of an
                // over-length write arriving one transaction late.
                if len != CANARY.len() || buffer[..len] != CANARY[..] {
                    corrupt += 1;
                    if corrupt <= 5 {
                        error!(
                            "round {}: write of {} bytes {:?}, expected {:?}",
                            listens,
                            len,
                            buffer[..len.min(BUFFER)],
                            CANARY
                        );
                    }
                } else {
                    canaries += 1;

                    // Toggled only for a *clean* canary, so the analyser alone settles the verdict:
                    // one edge per canary the controller sent means none were corrupted. RTT is not
                    // trustworthy on this path — defmt-rtt runs non-blocking here and drops whatever
                    // does not fit, which has already read as a stalled target twice today.
                    judged.toggle();
                }
            }

            Ok(Command::Read) => {
                reads += 1;
                let _ = i2c.respond_to_read(&ANSWER).await;
            }

            Ok(Command::WriteRead(_)) | Ok(Command::GeneralCall(_)) => other += 1,

            Err(Error::PartialWrite(_)) => partials += 1,

            Err(_) => other += 1,
        }

        if listens % 500 == 0 {
            if corrupt == 0 {
                info!(
                    "{} listens: {} partial, {} canaries clean, {} reads, {} cancelled, {} other -- ok",
                    listens, partials, canaries, reads, cancelled, other
                );
            } else {
                error!(
                    "{} listens: {} partial, {} canaries clean, {} CORRUPT, {} reads, {} cancelled",
                    listens, partials, canaries, corrupt, reads, cancelled
                );
            }
        }
    }
}
