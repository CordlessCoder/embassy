//! Drives an MSPM0 target on a 10-bit address.
//!
//! Other side of the MSPM0 `i2c_target_10bit` example. An `embassy-stm32` controller is an independent
//! implementation, so what it gets back is evidence about our target's `TOAR.TMODE` rather than about
//! one HAL agreeing with itself.
//!
//! Wiring, with both boards sharing a ground and **one pull-up per line** — neither board fits them, so
//! 4.7 kΩ to 3V3 on each of SCL and SDA:
//!
//! | U575  | MSPM0G3507 (`I2C1`) | Signal |
//! |-------|---------------------|--------|
//! | `PB8` | `PB2`               | SCL    |
//! | `PB9` | `PB3`               | SDA    |
//!
//! # What each step exercises
//!
//! A write, a read and a write-read on `0x148`, which together need `TOAR.TMODE` to match a 10-bit own
//! address and the target's answers to survive the extra address-write frame a 10-bit read carries. The
//! second own address is tested by `i2c_controller` instead, its register only being compared while the
//! target is in 7-bit mode.
//!
//! Note what this needs from **this** end: `CR2.SADD` holds a 7-bit address in bits 7:1 but a 10-bit one
//! in bits 9:0, so a driver that shifts both puts every 10-bit address out one bit too far left. A run
//! against a target that answers only its configured address is what catches that.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{Address, I2c};
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, i2c};
use embassy_time::Timer;
use panic_probe as _;

/// The target's 10-bit address.
const TARGET_10BIT: Address = Address::TenBit(0x148);

/// What a plain read should return.
const EXPECT_READ: [u8; 2] = [8, 8];

/// What a write-read should return.
const EXPECT_WRITE_READ: [u8; 2] = [9, 9];

/// 100 kHz, which is what the target example solves its timing for.
const FREQUENCY: Hertz = Hertz::khz(100);

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = Config::default();

    // 16 MHz HSI multiplied to 160 MHz, keeping this end identical to the other examples in this crate.
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

    info!("driving the target on 10-bit {:#x}", TARGET_10BIT.addr());

    let mut pass = 0u32;

    loop {
        pass += 1;
        let mut failures = 0;

        if let Err(e) = i2c.blocking_write(TARGET_10BIT, &[0xAA, 0x55]) {
            error!("10-bit write failed: {}", e);
            failures += 1;
        }

        let mut read = [0u8; 2];
        match i2c.blocking_read(TARGET_10BIT, &mut read) {
            Ok(()) if read == EXPECT_READ => debug!("10-bit read: {:?}", read),
            Ok(()) => {
                error!("10-bit read gave {:?}, expected {:?}", read, EXPECT_READ);
                failures += 1;
            }
            Err(e) => {
                error!("10-bit read failed: {}", e);
                failures += 1;
            }
        }

        let mut write_read = [0u8; 2];
        match i2c.blocking_write_read(TARGET_10BIT, &[0x01], &mut write_read) {
            Ok(()) if write_read == EXPECT_WRITE_READ => debug!("10-bit write_read: {:?}", write_read),
            Ok(()) => {
                error!(
                    "10-bit write_read gave {:?}, expected {:?}",
                    write_read, EXPECT_WRITE_READ
                );
                failures += 1;
            }
            Err(e) => {
                error!("10-bit write_read failed: {}", e);
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
