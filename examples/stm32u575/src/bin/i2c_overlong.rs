//! Drives an MSPM0 target with writes longer than its buffer, to see whether the leftovers reappear.
//!
//! The counterpart is `examples/mspm0g3507/src/bin/i2c_target_faults.rs`, which is where the pass or
//! failure is reported — this end only supplies the traffic. Rig D:
//!
//! | Net | U575 | MSPM0 G3507 |
//! |---|---|---|
//! | SCL | `PB8`, Arduino D15 | `PB2` J1.9 |
//! | SDA | `PB9`, Arduino D14 | `PB3` J1.10 |
//! | GND | Arduino power header | J1.20 |
//!
//! 4.7 kΩ from each line to 3V3, once for the bus.
//!
//! # The cycle
//!
//! 1. An **over-length write** — more bytes than the target's buffer, so its `listen` returns
//!    `PartialWrite` and has to decide what to do with the rest.
//! 2. A **canary write**, short and distinctive. The target checks it arrives exactly as sent. If the
//!    over-length write left bytes in the receive FIFO, they arrive glued to the front of this one.
//! 3. A **read**, so the target's answer path is exercised between rounds.
//!
//! Start this end second: the target has to be listening before the first write.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::Timer;
use panic_probe as _;

const TARGET_ADDR: u8 = 0x48;
const FREQUENCY: Hertz = Hertz(100_000);

/// Longer than the target's receive buffer, which is what makes it a partial write.
const OVERLONG: [u8; 12] = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C];

/// Short and unlike anything in the over-length write, so a leftover byte is unmistakable.
const CANARY: [u8; 2] = [0xC1, 0xC2];

const ROUNDS: u32 = 500;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = Config::default();

    config.rcc.hsi = true;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul10,
        divp: None,
        divq: None,
        divr: Some(PllDiv::Div1),
    });
    config.rcc.sys = Sysclk::Pll1R;
    config.rcc.voltage_range = VoltageScale::Range1;

    let p = embassy_stm32::init(config);

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = FREQUENCY;

    let mut i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c_config);

    info!("driving {:#x}: {} rounds of overlong, canary, read", TARGET_ADDR, ROUNDS);

    let mut overlong_err = 0u32;
    let mut canary_err = 0u32;
    let mut read_err = 0u32;

    for round in 0..ROUNDS {
        // The target NACKs or truncates this; either is a legal answer to more data than it can hold, so
        // an error here is not itself a failure.
        if i2c.blocking_write(TARGET_ADDR, &OVERLONG).is_err() {
            overlong_err += 1;
        }

        // Long enough for the target to come back round to `listen`, short enough that 500 rounds is not
        // a coffee break.
        Timer::after_millis(2).await;

        if i2c.blocking_write(TARGET_ADDR, &CANARY).is_err() {
            canary_err += 1;
        }

        Timer::after_millis(2).await;

        let mut answer = [0u8; 2];
        if i2c.blocking_read(TARGET_ADDR, &mut answer).is_err() {
            read_err += 1;
        }

        Timer::after_millis(2).await;

        if round % 100 == 99 {
            info!(
                "{} rounds: {} overlong err, {} canary err, {} read err",
                round + 1,
                overlong_err,
                canary_err,
                read_err
            );
        }
    }

    info!("done -- read the target's log for the verdict");

    loop {
        Timer::after_secs(60).await;
    }
}
