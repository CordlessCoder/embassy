//! What `embedded_hal::i2c::I2c::transaction` puts on the wire, against the U575 `i2c_target_long`.
//!
//! Wiring: `PB2` J1.9 SCL to `PB8` Arduino D15, `PB3` J1.10 SDA to `PB9` Arduino D14, shared ground,
//! 4.7 kΩ from each line to 3V3. Analyser `D0` on SCL, `D1` on SDA, `D2` on `PB13` J4.35.
//!
//! **The log cannot tell you whether this passes.** Every case here returns `Ok` both before and after
//! the driver is made to conform: what changes is the number of address phases on the bus, and only a
//! decode shows that. `PB13` is driven high for the duration of each case so the capture can be cut to
//! it, and the case number is pulsed out on the same pin first — *n* short pulses, then the case.
//!
//! # What the contract requires
//!
//! `embedded-hal` merges **consecutive operations of the same type**: no repeated START and no address
//! between them, with the data flowing continuously. A change of type gets a repeated START and the
//! address again. One STOP ends the lot.
//!
//! So the address-phase count per case is the measurement:
//!
//! | case | operations | address phases | note |
//! |---|---|---|---|
//! | 1 | `[W]` | 1 | the trivial one |
//! | 2 | `[W, W]` | **1** | merged; two before the fix |
//! | 3 | `[R, R]` | **1** | merged; two before the fix |
//! | 4 | `[W, R]` | 2 | a type change, so a restart |
//! | 5 | `[W, R, W]` | 3 | two type changes |
//! | 6 | `[W, W, R]` | **2** | the first two merge, then a restart |
//! | 7 | `[W(0), W]` | **1** | an empty operation is skipped, not addressed |
//!
//! Case 8 asks for a merged run longer than one burst can carry and expects
//! [`Error::TransferLengthIsOverLimit`] rather than a truncated transfer. That one the log *can* judge.
//!
//! # Reading the capture
//!
//! Count `start` and `address` records between the marker's rising and falling edge. A conforming run
//! shows the table's count; the pre-fix driver shows one address phase per operation in every case.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::i2c::{Config, Error, I2c};
use embassy_time::Timer;
use embedded_hal::i2c::{I2c as _, Operation};
use panic_halt as _;

/// The address `i2c_target_long` answers on.
const TARGET_ADDR: u8 = 0x48;

/// Longest transfer the target holds.
const MAX_LEN: usize = 256;

/// One past what a single burst carries, for case 8. Two operations that each fit and together do not.
const OVER_CEILING_EACH: usize = 2100;

/// Let the target finish offering the read half a write is answered with.
const SETTLE_MS: u64 = 100;

/// Marker pulse either side, short enough not to be confused with a case window.
const PULSE_US: u64 = 200;

static BIG: [u8; OVER_CEILING_EACH] = [0; OVER_CEILING_EACH];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut marker = Output::new(p.PB13, Level::Low);
    let mut i2c = unwrap!(I2c::new_blocking(p.I2C1, p.PB2, p.PB3, Config::default()));

    let mut ramp = [0u8; MAX_LEN];
    for (i, byte) in ramp.iter_mut().enumerate() {
        *byte = i as u8;
    }
    let (head, tail) = ramp.split_at(8);

    let mut got = [0u8; 32];
    let mut fail = 0u32;

    info!("i2c_transaction: each case is bracketed on PB13, preceded by its number in pulses");

    // 1: [W] — one address phase.
    run(&mut marker, 1, &mut fail, "W", || {
        i2c.transaction(TARGET_ADDR, &mut [Operation::Write(head)])
    })
    .await;

    // 2: [W, W] — must merge into one.
    run(&mut marker, 2, &mut fail, "W,W", || {
        i2c.transaction(TARGET_ADDR, &mut [Operation::Write(head), Operation::Write(&tail[..8])])
    })
    .await;

    // 3: [R, R] — must merge into one.
    {
        let (a, b) = got.split_at_mut(8);
        run(&mut marker, 3, &mut fail, "R,R", || {
            i2c.transaction(TARGET_ADDR, &mut [Operation::Read(a), Operation::Read(&mut b[..8])])
        })
        .await;
    }

    // 4: [W, R] — a type change, so two.
    run(&mut marker, 4, &mut fail, "W,R", || {
        i2c.transaction(
            TARGET_ADDR,
            &mut [Operation::Write(head), Operation::Read(&mut got[..8])],
        )
    })
    .await;

    // 5: [W, R, W] — three.
    run(&mut marker, 5, &mut fail, "W,R,W", || {
        i2c.transaction(
            TARGET_ADDR,
            &mut [
                Operation::Write(head),
                Operation::Read(&mut got[..8]),
                Operation::Write(&tail[..4]),
            ],
        )
    })
    .await;

    // 6: [W, W, R] — the writes merge, then a restart: two.
    run(&mut marker, 6, &mut fail, "W,W,R", || {
        i2c.transaction(
            TARGET_ADDR,
            &mut [
                Operation::Write(head),
                Operation::Write(&tail[..4]),
                Operation::Read(&mut got[..8]),
            ],
        )
    })
    .await;

    // 7: an empty operation is skipped rather than addressed.
    run(&mut marker, 7, &mut fail, "W(0),W", || {
        i2c.transaction(TARGET_ADDR, &mut [Operation::Write(&[]), Operation::Write(head)])
    })
    .await;

    // 8: a merged run past one burst's reach. The log judges this one.
    info!("8: a merged run longer than a burst");
    Timer::after_millis(SETTLE_MS).await;
    match i2c.transaction(TARGET_ADDR, &mut [Operation::Write(&BIG), Operation::Write(&BIG)]) {
        Err(Error::TransferLengthIsOverLimit) => info!("  refused, as it should be"),
        Ok(()) => {
            fail += 1;
            error!("  accepted; a merged run past the burst length was not caught");
        }
        Err(e) => {
            fail += 1;
            error!("  refused as {} rather than a length error", e);
        }
    }

    if fail == 0 {
        info!("every case returned Ok — now count the address phases in the capture");
    } else {
        error!("{} cases failed outright", fail);
    }

    loop {
        Timer::after_millis(1000).await;
    }
}

/// Pulse `case` out on the marker, then bracket the operation with it.
async fn run<F>(marker: &mut Output<'static>, case: u8, fail: &mut u32, what: &str, op: F)
where
    F: FnOnce() -> Result<(), Error>,
{
    info!("{}: {}", case, what);
    Timer::after_millis(SETTLE_MS).await;

    for _ in 0..case {
        marker.set_high();
        Timer::after_micros(PULSE_US).await;
        marker.set_low();
        Timer::after_micros(PULSE_US).await;
    }

    Timer::after_micros(PULSE_US * 4).await;

    marker.set_high();
    let result = op();
    marker.set_low();

    match result {
        Ok(()) => debug!("  ok"),
        Err(e) => {
            *fail += 1;
            error!("  {}", e);
        }
    }
}
