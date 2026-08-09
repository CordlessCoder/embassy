//! Reads fewer bytes than an MSPM0 target offers, then reads again to see what comes back.
//!
//! The counterpart is `examples/mspm0g3507/src/bin/i2c_target_stale_tx.rs`. **Unlike the write-side
//! pair, the verdict is at this end**: the target cannot see what it actually put on the wire, and
//! this is the end that receives the canary. Rig D:
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
//! 1. A **short read** — two bytes of the eight the target offers, so six are left unsent and the
//!    target's `respond_to_read` returns `LeftoverBytes(6)`.
//! 2. A **canary read**, which the target answers with two distinctive bytes. If the surplus from the
//!    short read is still queued ahead of them, this read returns the surplus instead.
//!
//! A round passes when the canary read is exactly `C1 C2`. Getting `A2 A3` — the third and fourth
//! bytes of the long answer — is the defect, and it names itself: the number that comes back says how
//! far into the previous answer the target had got.
//!
//! # Also worth watching on the wire
//!
//! The analyser decode is an independent witness here, and a better one than either log, because the
//! corruption is on the bus rather than in a buffer. D0 on SCL, D1 on SDA.
//!
//! Start this end second: the target has to be listening before the first read.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use panic_probe as _;

const TARGET_ADDR: u8 = 0x48;
const FREQUENCY: Hertz = Hertz(100_000);

/// What the canary read must return, byte for byte.
const CANARY: [u8; 2] = [0xC1, 0xC2];

/// First byte of the long answer, so a corrupt canary can be reported as an offset into it rather than
/// as an opaque wrong value.
const ANSWER_BASE: u8 = 0xA0;

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

    info!(
        "driving {:#x}: {} rounds of short read, canary read",
        TARGET_ADDR, ROUNDS
    );

    let mut short_err = 0u32;
    let mut canary_err = 0u32;
    let mut stale = 0u32;
    let mut first_stale = 0u8;

    for round in 0..ROUNDS {
        // Two of the eight on offer. An error is not itself a failure — what matters is the next read.
        let mut discard = [0u8; 2];
        if i2c.blocking_read(TARGET_ADDR, &mut discard).is_err() {
            short_err += 1;
        }

        // Long enough for the target to come back round to `listen`, short enough that 500 rounds is
        // not a coffee break.
        Timer::after_millis(2).await;

        let mut answer = [0u8; 2];
        if i2c.blocking_read(TARGET_ADDR, &mut answer).is_err() {
            canary_err += 1;
        } else if answer != CANARY {
            stale += 1;
            if stale == 1 {
                first_stale = answer[0];
            }
        }

        Timer::after_millis(2).await;

        if round % 100 == 99 {
            info!(
                "{} rounds: {} stale, {} short err, {} canary err",
                round + 1,
                stale,
                short_err,
                canary_err
            );
        }
    }

    if stale == 0 {
        info!("PASS: {} rounds, every canary read was C1 C2", ROUNDS);
    } else {
        // `first_stale - ANSWER_BASE` is how many bytes of the long answer the target had sent before
        // the short read stopped it, which is the number the fix has to drive to nothing.
        error!(
            "FAIL: {} of {} canary reads returned the previous answer, first at offset {}",
            stale,
            ROUNDS,
            first_stale.wrapping_sub(ANSWER_BASE)
        );
    }

    loop {
        Timer::after_secs(60).await;
    }
}
