//! Drives an I2C target on another vendor's HAL, and checks the answers.
//!
//! Other side of the U575 `i2c_target` example, which answers a read with `[8, 8]` and a write-read with
//! `[9, 9]` — the same protocol our own `i2c_target` example serves, so the expectations here hold
//! whichever side is ours. This one tests our **controller**: the timing solved by `i2c::Timing`, the
//! restart between the write and read halves of a write-read, and the FIFO handling under async.
//!
//! Wiring, with both boards sharing a ground and **one pull-up per line** — neither board fits them, so
//! 4.7 kΩ to 3V3 on each of SCL and SDA:
//!
//! | MSPM0G3507 | U575  | Signal |
//! |------------|-------|--------|
//! | `PB2`      | `PB8` | SCL    |
//! | `PB3`      | `PB9` | SDA    |
//!
//! # What to check
//!
//! - **Every pass reports clean.** A wrong `TPR` shows up as an error or a NACK rather than as wrong
//!   data, so a clean pass is evidence about the timing as well as the data path.
//! - **SCL on an analyser is 100 kHz**, and the low period is at least 4.7 µs. `Timing::solve` picks a
//!   period from the clock it is told about; if the clock tree it was told about is not the one running,
//!   the bus is out by that ratio and this is where it shows.
//! - A run against the MSPM0 `i2c_target` example on the other board instead of the U575 should give
//!   exactly the same log. If it does and the U575 run does not, our two sides agree with each other and
//!   not with the standard, which is the failure this pair exists to catch.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::i2c::{ClockDiv, ClockSel, Config, I2c, InterruptHandler, Timing};
use embassy_mspm0::peripherals::I2C1;
use embassy_mspm0::sysctl::clock;
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C1 => InterruptHandler<I2C1>;
});

/// The address both target examples answer on.
const TARGET_ADDR: u8 = 0x48;

/// What a plain read should return.
const EXPECT_READ: [u8; 2] = [8, 8];

/// What a write-read should return.
const EXPECT_WRITE_READ: [u8; 2] = [9, 9];

/// 100 kHz off the bus clock, solved here rather than divided for on the device.
///
/// The bus clock rather than MFCLK, so the solver has a rate with enough resolution to hit 100 kHz
/// closely — which is what makes the analyser check above meaningful. The timing carries the source it
/// was solved for, so `with_timing` programs `CLKSEL` to match and the two cannot drift apart.
const TIMING: Timing = match Timing::solve(
    &clock::RESET_SETUP.clocks(),
    ClockSel::BusClk,
    ClockDiv::DivBy1,
    100_000,
) {
    Some(timing) => timing,
    None => core::panic!("100 kHz is not reachable from the bus clock"),
};

/// Gap after a plain write. The U575 target answers a write by offering a read as well, which has to
/// time out before it will listen again.
const AFTER_WRITE_MS: u64 = 100;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let config = Config::default().with_timing(TIMING);

    let mut i2c = unwrap!(I2c::new_async(p.I2C1, p.PB2, p.PB3, Irqs, config));

    info!("driving the target at {:#x}", TARGET_ADDR);

    let mut pass = 0u32;

    loop {
        pass += 1;
        let mut failures = 0;

        // A plain write: one transaction, no restart.
        if let Err(e) = i2c.async_write(TARGET_ADDR, &[0xAA, 0x55]).await {
            error!("write failed: {:?}", e);
            failures += 1;
        }
        Timer::after_millis(AFTER_WRITE_MS).await;

        let mut read = [0u8; 2];
        match i2c.async_read(TARGET_ADDR, &mut read).await {
            Ok(()) if read == EXPECT_READ => debug!("read: {:?}", read),
            Ok(()) => {
                error!("read gave {:?}, expected {:?}", read, EXPECT_READ);
                failures += 1;
            }
            Err(e) => {
                error!("read failed: {:?}", e);
                failures += 1;
            }
        }

        // Write then read in one transaction, which is the restart path.
        let mut write_read = [0u8; 2];
        match i2c.async_write_read(TARGET_ADDR, &[0x01], &mut write_read).await {
            Ok(()) if write_read == EXPECT_WRITE_READ => debug!("write_read: {:?}", write_read),
            Ok(()) => {
                error!("write_read gave {:?}, expected {:?}", write_read, EXPECT_WRITE_READ);
                failures += 1;
            }
            Err(e) => {
                error!("write_read failed: {:?}", e);
                failures += 1;
            }
        }

        if failures == 0 {
            info!("pass {} clean", pass);
        } else {
            error!("pass {} had {} failures", pass, failures);
        }

        Timer::after_millis(500).await;
    }
}
