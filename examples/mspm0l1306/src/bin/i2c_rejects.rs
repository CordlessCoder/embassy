//! Checks that the configurations and addresses the I2C driver is supposed to refuse are refused.
//!
//! **Needs no wiring and no target.** Every case here is settled before the peripheral touches the bus:
//! `Address::checked` runs at the top of each transfer method, and the target's own address checks run
//! inside `init`, which `new` reaches through `reset`. So this can run on a bare board, and on either MSPM0
//! — nothing in it is part-specific beyond the instance and pins.
//!
//! # Why bother
//!
//! These are the paths that exist precisely so that a mistake cannot reach the wire, which is what makes
//! them easy to leave untested: nothing fails visibly if a rejection quietly stops happening. Two of them
//! guard against silence in particular:
//!
//! - **`SecondAddressWith10Bit`.** `OAR2` is only compared while the target is in 7-bit mode, measured, so
//!   a second address alongside a 10-bit primary would be configured and then answer nothing at all.
//! - **`InvalidAddress` and `InvalidTargetAddress`.** Both `Address` variants are wider than the address
//!   they carry, and both are constructible directly, past the `From` impls that assert. Unchecked, the
//!   peripheral truncates to its field width and talks to — or answers as — a different device.
//!
//! # What a pass looks like
//!
//! `all N checks passed`, and nothing else. Every check names itself on failure.
//!
//! The **positive controls matter as much as the rejections**: a driver that refused everything would pass a
//! test made only of rejections. So a legal target configuration must be accepted, and a legal address must
//! get as far as failing on the bus rather than failing validation.
//!
//! Not covered here: the `embedded_hal::i2c::I2c<TenBitAddress>` impls. Reaching this same validation
//! through the trait would prove little, and proving the trait drives a bus correctly needs a target — so it
//! belongs with the wired tests, and no example crate depends on `embedded-hal` yet.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::i2c::{Address, ClockDiv, ClockSel, Config, ConfigError, Error, I2c, Timing};
use embassy_mspm0::i2c_target::{Config as TargetConfig, I2cTarget, SecondAddress};
use embassy_mspm0::peripherals::I2C0;
use embassy_mspm0::sysctl::clock;
use embassy_mspm0::{bind_interrupts, i2c};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C0 => i2c::InterruptHandler<I2C0>;
});

/// A legal 7-bit second address with a mask covering `0x50`..`0x53`.
const SECOND: SecondAddress = SecondAddress { addr: 0x50, mask: 0x03 };

const TIMING: Timing = match Timing::solve(&clock::RESET_SETUP.clocks(), ClockSel::MfClk, ClockDiv::DivBy1, 100_000) {
    Some(timing) => timing,
    None => core::panic!("100 kHz is not reachable from MFCLK"),
};

/// Counts checks so a pass is a number rather than an absence of complaints.
struct Checks {
    run: u32,
    failed: u32,
}

impl Checks {
    fn expect(&mut self, name: &str, ok: bool) {
        self.run += 1;
        if ok {
            debug!("  {} ok", name);
        } else {
            self.failed += 1;
            error!("  {} FAILED", name);
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut p = embassy_mspm0::init(Default::default());

    let config = Config::default().with_timing(TIMING);
    let mut checks = Checks { run: 0, failed: 0 };

    // ---- the controller's address checks, which run before the bus is touched ----
    //
    // Done first, and in a scope of its own, because the positive control below has to actually
    // reach the bus: an instance that has already been an `I2cTarget` does not drive one as a
    // controller, so with this after the target checks it passed without transmitting anything.
    {
        let mut i2c = unwrap!(I2c::new_blocking(
            p.I2C0.reborrow(),
            p.PA1.reborrow(),
            p.PA0.reborrow(),
            config
        ));

        checks.expect(
            "a 10-bit address above 0x3ff is refused",
            i2c.blocking_write(Address::TenBit(0x400), &[0]) == Err(Error::InvalidAddress),
        );
        checks.expect(
            "a 7-bit address above 0x7f is refused",
            i2c.blocking_write(Address::SevenBit(0x80), &[0]) == Err(Error::InvalidAddress),
        );
        checks.expect(
            "a read to an out-of-range address is refused",
            i2c.blocking_read(Address::TenBit(0x400), &mut [0]) == Err(Error::InvalidAddress),
        );
        checks.expect(
            "a write-read to an out-of-range address is refused",
            i2c.blocking_write_read(Address::TenBit(0x400), &[0], &mut [0]) == Err(Error::InvalidAddress),
        );

        // The positive control for the controller: a legal address must get past validation and fail on the
        // bus instead. With nothing wired that is a bus or timeout error — any of them will do, so long as it
        // is not the rejection above.
        let legal_address = i2c.blocking_write(Address::TenBit(0x148), &[0]);

        checks.expect(
            "a legal address reaches the bus rather than being refused",
            legal_address != Err(Error::InvalidAddress),
        );
        debug!("  (it came back as {:?}, which is the bus talking)", legal_address);
    }

    // ---- the target's configuration checks, all of them inside `init` ----

    // Reborrowed every time because each attempt consumes the instance and both pins, and a rejected
    // configuration has to leave them usable for the next one.
    let mut target = |target_config: TargetConfig| -> Result<(), ConfigError> {
        I2cTarget::new(
            p.I2C0.reborrow(),
            p.PA1.reborrow(),
            p.PA0.reborrow(),
            Irqs,
            config,
            target_config,
        )
        .map(|_| ())
    };

    let mut legal = TargetConfig::default();
    legal.target_addr = Address::SevenBit(0x48);
    legal.second_addr = Some(SECOND);
    checks.expect(
        "a 7-bit primary with a masked second address is accepted",
        target(legal).is_ok(),
    );

    let mut ten_bit_only = TargetConfig::default();
    ten_bit_only.target_addr = Address::TenBit(0x148);
    checks.expect("a 10-bit primary alone is accepted", target(ten_bit_only).is_ok());

    let mut both = TargetConfig::default();
    both.target_addr = Address::TenBit(0x148);
    both.second_addr = Some(SECOND);
    checks.expect(
        "a second address alongside a 10-bit primary is refused",
        target(both) == Err(ConfigError::SecondAddressWith10Bit),
    );

    let mut wide_primary = TargetConfig::default();
    wide_primary.target_addr = Address::TenBit(0x400);
    checks.expect(
        "a 10-bit primary above 0x3ff is refused",
        target(wide_primary) == Err(ConfigError::InvalidTargetAddress),
    );

    let mut wide_seven_bit = TargetConfig::default();
    wide_seven_bit.target_addr = Address::SevenBit(0x80);
    checks.expect(
        "a 7-bit primary above 0x7f is refused",
        target(wide_seven_bit) == Err(ConfigError::InvalidTargetAddress),
    );

    let mut wide_second = TargetConfig::default();
    wide_second.second_addr = Some(SecondAddress { addr: 0x80, mask: 0 });
    checks.expect(
        "a second address above 0x7f is refused",
        target(wide_second) == Err(ConfigError::InvalidTargetAddress),
    );

    let mut wide_mask = TargetConfig::default();
    wide_mask.second_addr = Some(SecondAddress { addr: 0x50, mask: 0x80 });
    checks.expect(
        "a second address mask above 0x7f is refused",
        target(wide_mask) == Err(ConfigError::InvalidTargetAddress),
    );

    if checks.failed == 0 {
        info!("all {} checks passed", checks.run);
    } else {
        error!("{} of {} checks FAILED", checks.failed, checks.run);
    }

    loop {
        embassy_time::Timer::after_secs(60).await;
    }
}
