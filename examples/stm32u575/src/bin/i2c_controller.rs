//! Drives an MSPM0 I2C target from a known-good controller, and checks what comes back.
//!
//! Other side of the MSPM0 `i2c_target` example — it answers a read with `[8, 8]`, a write-read with
//! `[9, 9]` padded with `0xFE`, and has the general call enabled. Those fixed answers are what makes it
//! testable from here: an `embassy-stm32` controller is an independent implementation, so agreement is
//! evidence about our target driver rather than about one HAL agreeing with itself.
//!
//! Wiring, with both boards sharing a ground and **one pull-up per line** — neither board fits them, so
//! 4.7 kΩ to 3V3 on each of SCL and SDA:
//!
//! | U575  | MSPM0L1306 (`I2C0`) | MSPM0G3507 (`I2C1`) | Signal |
//! |-------|---------------------|---------------------|--------|
//! | `PB8` | `PA1`               | `PB2`               | SCL    |
//! | `PB9` | `PA0`               | `PB3`               | SDA    |
//!
//! Each board's pins are the ones its own `i2c_target` example already uses. On the L1306, `PA0` is also
//! LED1: take its jumper off, or the LED loads SDA.
//!
//! # What each step exercises
//!
//! | Step | On the target |
//! |------|---------------|
//! | write of 2 bytes | `Command::Write`, and the RX FIFO path |
//! | read of 2 | `Command::Read` answered exactly, `ReadStatus::Done` |
//! | write-read of 2 | `Command::WriteRead` and `respond_and_fill` |
//! | write-read of 4 | the fill: the target only offers two bytes, so the rest must come back `0xFE` |
//! | read of 4 | `ReadStatus::NeedMoreBytes` and the target's `reset()` — not asserted, only reported |
//! | general call | address 0 matching with `general_call = true` |
//! | read of `0x50`..`0x53` | `TOAR2` with `OAR2_MASK`, and `matched_address()` reporting which one |
//! | write to `0x54` | that the mask stops where it should — this one must NACK |
//!
//! Everything is asserted bar two. The read of 4 is logged rather than checked: what a target sends
//! after it runs out of bytes is not defined by the driver, and the interesting part is that it
//! recovers, which shows up as the next pass passing. The general call is logged at the far end only.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::i2c::I2c;
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, i2c};
use embassy_time::Timer;
use panic_probe as _;

/// The address the MSPM0 `i2c_target` example answers on.
const TARGET_ADDR: u8 = 0x48;

/// What that example answers a plain read with.
const EXPECT_READ: [u8; 2] = [8, 8];

/// What it answers a write-read with, and the byte it pads a longer one with.
const EXPECT_WRITE_READ: [u8; 2] = [9, 9];
const EXPECT_FILL: u8 = 0xFE;

/// The target's masked second address covers these, and answers a read on each with the address itself.
const IN_RANGE: [u8; 4] = [0x50, 0x51, 0x52, 0x53];

/// Just past the mask, so it must not be answered at all.
const OUT_OF_RANGE: u8 = 0x54;

/// 100 kHz, which is what the target example solves its timing for.
const FREQUENCY: Hertz = Hertz::khz(100);

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = Config::default();

    // 16 MHz HSI multiplied to 160 MHz. Not needed for 100 kHz, but it keeps this end identical to the
    // other examples in this crate so a difference is never the clock setup.
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

    info!("driving the target at {:#x}", TARGET_ADDR);

    let mut pass = 0u32;

    loop {
        pass += 1;
        let mut failures = 0;

        // A plain write. The target logs it and keeps the bytes; nothing comes back to check.
        if let Err(e) = i2c.blocking_write(TARGET_ADDR, &[0xAA, 0x55]) {
            error!("write failed: {}", e);
            failures += 1;
        }

        let mut read = [0u8; 2];
        match i2c.blocking_read(TARGET_ADDR, &mut read) {
            Ok(()) if read == EXPECT_READ => debug!("read: {:?}", read),
            Ok(()) => {
                error!("read gave {:?}, expected {:?}", read, EXPECT_READ);
                failures += 1;
            }
            Err(e) => {
                error!("read failed: {}", e);
                failures += 1;
            }
        }

        let mut write_read = [0u8; 2];
        match i2c.blocking_write_read(TARGET_ADDR, &[0x01], &mut write_read) {
            Ok(()) if write_read == EXPECT_WRITE_READ => debug!("write_read: {:?}", write_read),
            Ok(()) => {
                error!("write_read gave {:?}, expected {:?}", write_read, EXPECT_WRITE_READ);
                failures += 1;
            }
            Err(e) => {
                error!("write_read failed: {}", e);
                failures += 1;
            }
        }

        // Two more bytes than the target offers, so the rest has to be the fill byte.
        let mut filled = [0u8; 4];
        let expected_fill = [EXPECT_WRITE_READ[0], EXPECT_WRITE_READ[1], EXPECT_FILL, EXPECT_FILL];
        match i2c.blocking_write_read(TARGET_ADDR, &[0x01], &mut filled) {
            Ok(()) if filled == expected_fill => debug!("filled write_read: {:?}", filled),
            Ok(()) => {
                error!("filled write_read gave {:?}, expected {:?}", filled, expected_fill);
                failures += 1;
            }
            Err(e) => {
                error!("filled write_read failed: {}", e);
                failures += 1;
            }
        }

        // Past the end of what the target has to send: it should report `NeedMoreBytes` and reset. What
        // it puts on the wire meanwhile is undefined, so this is reported rather than checked.
        let mut overrun = [0u8; 4];
        match i2c.blocking_read(TARGET_ADDR, &mut overrun) {
            Ok(()) => info!("over-long read gave {:?}", overrun),
            Err(e) => info!("over-long read failed with {}, which is also an acceptable answer", e),
        }

        // The target's second address, which covers a masked range. Every address in the range has to be
        // answered, and a read from one has to come back as that address — the target sends whatever
        // matched, so this checks the range and the reporting of it in one step.
        for addr in IN_RANGE {
            let mut matched = [0u8; 2];
            match i2c.blocking_read(addr, &mut matched) {
                Ok(()) if matched == [addr; 2] => debug!("{:#x} answered as itself", addr),
                Ok(()) => {
                    error!(
                        "{:#x} answered as {:?}, so the wrong address was reported",
                        addr, matched
                    );
                    failures += 1;
                }
                Err(e) => {
                    error!("read from {:#x} in the masked range failed: {}", addr, e);
                    failures += 1;
                }
            }
        }

        if i2c.blocking_write(OUT_OF_RANGE, &[OUT_OF_RANGE]).is_ok() {
            error!("{:#x} was answered, so the mask is wider than asked for", OUT_OF_RANGE);
            failures += 1;
        }

        // Address 0. The target example enables the general call, so it should log one.
        if let Err(e) = i2c.blocking_write(0x00u8, &[0x06]) {
            info!("general call failed with {}", e);
        }

        if failures == 0 {
            info!("pass {} clean", pass);
        } else {
            error!("pass {} had {} failures", pass, failures);
        }

        Timer::after_millis(500).await;
    }
}
