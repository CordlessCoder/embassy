//! Two failure modes of the blocking I2C controller, against the U575 `i2c_target` example.
//!
//! Same wiring as `i2c_crosscheck`: `PB2`→`PB8` SCL, `PB3`→`PB9` SDA, shared ground, 4.7 kΩ from each
//! line to 3V3.
//!
//! # A: what a failed transfer leaves behind
//!
//! Good transfer, then one that must fail, then the same good transfer again. The third step is the
//! measurement — a driver that leaves a byte in the FIFO after an error answers it with the previous
//! transfer's data, so the damage shows up one transaction *after* the one that went wrong. That is what
//! makes it hard to see: the error is reported against a transfer that succeeded.
//!
//! The forced failure is an address nobody answers, so it NACKs on the address byte and needs nothing
//! from the target.
//!
//! # B: what a long transfer leaves behind
//!
//! The U575 target stretches SCL while its application is not inside `listen()`, and after answering a
//! write it offers a read that has to time out before it listens again. `i2c_crosscheck` leaves 100 ms
//! between transactions to stay clear of that. This does the opposite on purpose: back-to-back transfers
//! with no gap, so the target stretches for as long as it likes.
//!
//! Every wait in the driver is an unbounded `while` on `busy()`, `idle()` or `busbsy()`, and an error
//! path that returns without issuing STOP leaves the bus so that IDLE never arrives. The symptom is not
//! a wrong answer, it is that this binary stops logging. **The last line printed names the step that
//! hung**, which is why every step announces itself first.
//!
//! Blocking only — `blocking_*` exists on `I2c<Blocking>` and `async_*` on `I2c<Async>`, so one instance
//! cannot do both, and the fix under discussion is for the blocking path.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::i2c::{ClockDiv, ClockSel, Config, Error, I2c, Timing};
use embassy_mspm0::sysctl::clock;
use embassy_time::Timer;
use panic_halt as _;

/// The address the U575 target answers on.
const TARGET_ADDR: u8 = 0x48;

/// Nothing lives here, so addressing it NACKs.
const ABSENT_ADDR: u8 = 0x50;

/// What a plain read returns.
const EXPECT_READ: [u8; 2] = [8, 8];

/// What a write-read returns.
const EXPECT_WRITE_READ: [u8; 2] = [9, 9];

/// `BusClk` at 100 kHz, for comparing against the default.
///
/// `Config::default` runs from MFCLK and is what the faults below need; pass this to `with_timing` and
/// they all go away, which is why every other I2C test on this bench passed.
#[allow(dead_code)]
const BUSCLK_TIMING: Timing = match Timing::solve(
    &clock::RESET_SETUP.clocks(),
    ClockSel::BusClk,
    ClockDiv::DivBy1,
    100_000,
) {
    Some(timing) => timing,
    None => core::panic!("100 kHz is not reachable from the bus clock"),
};

/// Long enough for the U575 to finish offering the read it answers a write with.
const SETTLE_MS: u64 = 100;

/// Back-to-back transfers in phase B, with no gap between them.
const HAMMER: u32 = 20;

/// Consecutive failures before phase A checks recovery.
///
/// One is not enough: an address NACK moves no data, so there is nothing to strand. The report this
/// chases showed about ten in a row — a target rebooting mid-conversation — before the readback came
/// back offset, so the damage needs to accumulate.
const NACK_BURST: u32 = 12;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // The shipped default, which is MFCLK. `with_timing(TIMING)` solved for the same source wedges
    // identically, so the solved-timing path is not the variable — this is just the smaller reproduction.
    let config = Config::default();
    let mut i2c = unwrap!(I2c::new_blocking(p.I2C1, p.PB2, p.PB3, config));

    info!("target {:#x}, forcing NACKs against {:#x}", TARGET_ADDR, ABSENT_ADDR);

    let mut pass = 0u32;
    let mut stale = 0u32;

    loop {
        pass += 1;

        // --- A: recovery after a NACK ---
        let mut buf = [0u8; 2];
        if let Err(e) = i2c.blocking_write_read(TARGET_ADDR, &[1], &mut buf) {
            error!("pass {}: baseline write-read failed: {}", pass, e);
        } else if buf != EXPECT_WRITE_READ {
            error!("pass {}: baseline write-read gave {:?}", pass, buf);
        }
        Timer::after_millis(SETTLE_MS).await;

        for _ in 0..NACK_BURST {
            match i2c.blocking_write_read(ABSENT_ADDR, &[1], &mut [0u8; 2]) {
                Err(Error::NackAddress) => {}
                Err(e) => warn!("pass {}: absent address gave {} rather than NackAddress", pass, e),
                Ok(()) => error!("pass {}: absent address answered", pass),
            }
        }

        let mut after = [0u8; 2];
        match i2c.blocking_write_read(TARGET_ADDR, &[1], &mut after) {
            Ok(()) if after == EXPECT_WRITE_READ => {}
            Ok(()) => {
                stale += 1;
                error!(
                    "pass {}: STALE — write-read after a NACK gave {:?}, wanted {:?}",
                    pass, after, EXPECT_WRITE_READ
                );
            }
            Err(e) => {
                stale += 1;
                error!("pass {}: write-read after a NACK failed: {}", pass, e);
            }
        }
        Timer::after_millis(SETTLE_MS).await;

        let mut read = [0u8; 2];
        let _ = i2c.blocking_read(TARGET_ADDR, &mut read);
        for _ in 0..NACK_BURST {
            let _ = i2c.blocking_read(ABSENT_ADDR, &mut [0u8; 2]);
        }
        match i2c.blocking_read(TARGET_ADDR, &mut read) {
            Ok(()) if read == EXPECT_READ => {}
            Ok(()) => {
                stale += 1;
                error!("pass {}: STALE — read after a NACK gave {:?}", pass, read);
            }
            Err(e) => {
                stale += 1;
                error!("pass {}: read after a NACK failed: {}", pass, e);
            }
        }

        info!("pass {}: phase A done, {} stale so far", pass, stale);
        Timer::after_millis(SETTLE_MS).await;

        // --- B: no gap, so the target stretches as long as it likes ---
        for i in 0..HAMMER {
            info!("pass {} hammer {}: starting", pass, i);
            let mut b = [0u8; 2];
            match i2c.blocking_write_read(TARGET_ADDR, &[1], &mut b) {
                Ok(()) => {}
                Err(e) => info!("pass {} hammer {}: {}", pass, i, e),
            }
            info!("pass {} hammer {}: returned", pass, i);
        }

        info!("pass {}: phase B survived", pass);
        Timer::after_millis(SETTLE_MS).await;
    }
}
