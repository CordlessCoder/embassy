//! Addresses a target with a 10-bit address, and proves all ten bits reach the wire.
//!
//! Other side of the U575 `i2c_target_10bit` example, which answers a 10-bit address on `OA1` and a
//! 7-bit one on `OA2` at the same time. Both are live throughout, so this switches addressing mode
//! between transfers on one instance and one wiring.
//!
//! Wiring, with both boards sharing a ground and **one pull-up per line** — neither board fits them, so
//! 4.7 kΩ to 3V3 on each of SCL and SDA:
//!
//! | MSPM0G3507 | U575  | Signal |
//! |------------|-------|--------|
//! | `PB2`      | `PB8` | SCL    |
//! | `PB3`      | `PB9` | SDA    |
//!
//! # What it establishes
//!
//! Four phases, in the order they stop being about addressing at all:
//!
//! - **7-bit still works**, in the same binary and on the same instance as the 10-bit transfers. A mode
//!   left set from the previous transfer shows up here rather than as a mystery later.
//! - **10-bit is answered** for a write, a plain read and a write-read. The plain read is the
//!   interesting one: SLAU846 §25.2.3.4 says a 10-bit read is only possible as a combined transfer —
//!   the address sent in a write frame, then a repeated START carrying the header again with R/W=1 —
//!   and does not say whether the peripheral sequences that itself. Measured on the wire, **it does**:
//!   one START with `CMODE` set to 10-bit and `DIR` receiving produces the whole
//!   `[header 2nd-byte] Sr [header R]` sequence with nothing from software.
//!
//!   It also re-sends the address between the halves of a write-read, so that goes out as
//!   `[header 2nd-byte data] Sr [header 2nd-byte] Sr [header R]` — the address three times in one
//!   transaction. Legal, and not something a register can turn off, but it means the **target cannot
//!   tell a 10-bit read from a 10-bit write-read**: both end in an address-write frame followed by a
//!   restart into a read. Which of the two answers comes back is therefore up to the target, and only
//!   the 7-bit phase can insist on a particular one.
//! - **The two high bits are transmitted**, checked by addressing `0x048`, whose low byte is the same
//!   as the target's `0x148`. Anything that drops or mangles the header bits is answered here instead of
//!   NACKed.
//! - **The second address byte is transmitted**, checked by addressing `0x149`.
//!
//! The last two phases pass by being refused, so `NackAddress` is the expected result and any other
//! outcome — including success — is the failure.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::i2c::{Address, ClockDiv, ClockSel, Config, Error, I2c, InterruptHandler, Timing};
use embassy_mspm0::peripherals::I2C1;
use embassy_mspm0::sysctl::clock;
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C1 => InterruptHandler<I2C1>;
});

/// The 10-bit address the target answers on.
const TARGET_10BIT: Address = Address::TenBit(0x148);

/// The 7-bit address the same target answers on, through its second address register.
const TARGET_7BIT: Address = Address::SevenBit(0x48);

/// Same low byte as [`TARGET_10BIT`], different high bits: answered only if the header went out wrong.
const WRONG_HIGH_BITS: Address = Address::TenBit(0x048);

/// Same high bits as [`TARGET_10BIT`], different low byte: answered only if the second byte went out
/// wrong.
const WRONG_LOW_BYTE: Address = Address::TenBit(0x149);

/// What a plain read should return.
const EXPECT_READ: [u8; 2] = [8, 8];

/// What a write-read should return.
const EXPECT_WRITE_READ: [u8; 2] = [9, 9];

/// 100 kHz off the bus clock, solved here rather than divided for on the device.
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

/// Drive one address through a write, a read and a write-read, counting what did not work.
///
/// `exact` insists that the read and the write-read return their own answer, which only holds in 7-bit
/// mode; a 10-bit target sees the same thing either way and answers whichever it likes. Both cases
/// still reject an answer that is neither.
async fn exercise(i2c: &mut I2c<'static, embassy_mspm0::mode::Async>, address: Address, exact: bool) -> u32 {
    let mut failures = 0;

    if let Err(e) = i2c.async_write(address, &[0xAA, 0x55]).await {
        error!("{}: write failed: {:?}", address, e);
        failures += 1;
    }
    Timer::after_millis(AFTER_WRITE_MS).await;

    let mut read = [0u8; 2];
    match i2c.async_read(address, &mut read).await {
        Ok(()) if answered(&read, &EXPECT_READ, exact) => debug!("{}: read {:?}", address, read),
        Ok(()) => {
            error!("{}: read gave {:?}, expected {:?}", address, read, EXPECT_READ);
            failures += 1;
        }
        Err(e) => {
            error!("{}: read failed: {:?}", address, e);
            failures += 1;
        }
    }

    let mut write_read = [0u8; 2];
    match i2c.async_write_read(address, &[0x01], &mut write_read).await {
        Ok(()) if answered(&write_read, &EXPECT_WRITE_READ, exact) => {
            debug!("{}: write_read {:?}", address, write_read)
        }
        Ok(()) => {
            error!(
                "{}: write_read gave {:?}, expected {:?}",
                address, write_read, EXPECT_WRITE_READ
            );
            failures += 1;
        }
        Err(e) => {
            error!("{}: write_read failed: {:?}", address, e);
            failures += 1;
        }
    }

    failures
}

/// Whether the target's answer is acceptable: the one this transfer asks for, or — when the target
/// cannot tell which transfer it was — either of the two it knows.
fn answered(got: &[u8; 2], want: &[u8; 2], exact: bool) -> bool {
    if exact {
        got == want
    } else {
        *got == EXPECT_READ || *got == EXPECT_WRITE_READ
    }
}

/// Check that an address nothing answers on is refused, which is how a mistransmitted address shows up.
async fn expect_nack(i2c: &mut I2c<'static, embassy_mspm0::mode::Async>, address: Address) -> u32 {
    match i2c.async_write(address, &[0xAA]).await {
        Err(Error::NackAddress) => {
            debug!("{}: refused, as it should be", address);
            0
        }
        Err(e) => {
            error!("{}: refused with {:?}, expected NackAddress", address, e);
            1
        }
        Ok(()) => {
            error!("{}: answered, so the address did not reach the wire intact", address);
            1
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let config = Config::default().with_timing(TIMING);

    let mut i2c = unwrap!(I2c::new_async(p.I2C1, p.PB2, p.PB3, Irqs, config));

    info!("driving {} and {}", TARGET_7BIT, TARGET_10BIT);

    let mut pass = 0u32;

    loop {
        pass += 1;

        let mut failures = exercise(&mut i2c, TARGET_7BIT, true).await;
        failures += exercise(&mut i2c, TARGET_10BIT, false).await;
        failures += expect_nack(&mut i2c, WRONG_HIGH_BITS).await;
        failures += expect_nack(&mut i2c, WRONG_LOW_BYTE).await;

        if failures == 0 {
            info!("pass {} clean", pass);
        } else {
            error!("pass {} had {} failures", pass, failures);
        }

        Timer::after_millis(500).await;
    }
}
