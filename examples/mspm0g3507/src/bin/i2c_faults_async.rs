//! `i2c_faults` for the async controller, which is a different error path.
//!
//! Same wiring and same battery as the blocking copy: `PB2`→`PB8` SCL, `PB3`→`PB9` SDA, shared ground,
//! 4.7 kΩ from each line to 3V3. `blocking_*` lives on `I2c<Blocking>` and `async_*` on `I2c<Async>`, so
//! one instance cannot run both and this has to be its own binary.
//!
//! The point is whether `I2C_ERR_13` reaches here. It should not on the erratum's wording — it is about
//! polling `BUSY` to wait for completion, and this path waits on the transfer-done interrupt instead. But
//! the entry guards are still `while busbsy() {}` spins, and a NACK still has to leave the controller fit
//! for the next transfer, so it is worth measuring rather than assuming.
//!
//! Runs on MFCLK, which is `Config::default` and the setting the blocking path failed under.
//!
//! # Reading phase C
//!
//! `stale` and `wedged` are defects: bytes from the wrong transfer, or a transfer that never returned.
//! `target would not release SDA` is not — nine clocks do not rescue every target, and this one keeps
//! shifting data out and holding the line, so `recover_stuck_bus` reports it rather than pretending.
//! A pass is `0 stale, 0 wedged`, whatever the last figure says.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::i2c::{Config, Error, I2c, InterruptHandler};
use embassy_mspm0::peripherals::I2C1;
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C1 => InterruptHandler<I2C1>;
});

/// The address the U575 target answers on.
const TARGET_ADDR: u8 = 0x48;

/// Nothing lives here, so addressing it NACKs.
const ABSENT_ADDR: u8 = 0x50;

/// What a plain read returns.
const EXPECT_READ: [u8; 2] = [8, 8];

/// What a write-read returns.
const EXPECT_WRITE_READ: [u8; 2] = [9, 9];

/// Long enough for the U575 to finish offering the read it answers a write with.
const SETTLE_MS: u64 = 100;

/// Back-to-back transfers with no gap between them.
const HAMMER: u32 = 20;

/// Consecutive failures before recovery is checked.
const NACK_BURST: u32 = 12;

/// SCL-low timeout to configure, or `None` for the unbounded behaviour.
///
/// It does not rescue phase C: the wedge there is the target holding **SDA** low with SCL high, and the
/// counter watches SCL low. Set anyway, because it is what a real caller would do and it bounds the other
/// phases.
const TIMEOUT_US: Option<u32> = Some(20_000);

/// Budget for the transfer that follows a cancellation, in milliseconds.
///
/// Bounded by the target, not by us — see phase C.
const RECOVER_MS: u64 = 300;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut config = Config::default();
    config.clock_low_timeout_us = TIMEOUT_US;
    let mut i2c = unwrap!(I2c::new_async(p.I2C1, p.PB2, p.PB3, Irqs, config));

    info!(
        "async: target {:#x}, forcing NACKs against {:#x}",
        TARGET_ADDR, ABSENT_ADDR
    );

    let mut pass = 0u32;
    let mut stale = 0u32;
    let mut false_ok = 0u32;
    let mut cancels = 0u32;
    let mut wedged = 0u32;
    // Counted apart from `stale`: a target that will not release SDA is the target's limit, not the
    // driver returning another transfer's bytes. Conflated, a clean run reads as dozens of failures.
    let mut unrecoverable = 0u32;

    loop {
        pass += 1;

        let mut buf = [0u8; 2];
        if let Err(e) = i2c.async_write_read(TARGET_ADDR, &[1], &mut buf).await {
            error!("pass {}: baseline write-read failed: {}", pass, e);
        } else if buf != EXPECT_WRITE_READ {
            error!("pass {}: baseline write-read gave {:?}", pass, buf);
        }
        Timer::after_millis(SETTLE_MS).await;

        for n in 0..NACK_BURST {
            info!("pass {}: nack-burst {} entering", pass, n);
            match i2c.async_write_read(ABSENT_ADDR, &[1], &mut [0u8; 2]).await {
                Err(Error::NackAddress) => {}
                Err(e) => warn!("pass {}: absent address gave {} rather than NackAddress", pass, e),
                Ok(()) => {
                    false_ok += 1;
                    error!("pass {}: FALSE OK — absent address answered", pass);
                }
            }
            info!("pass {}: nack-burst {} returned", pass, n);
        }

        let mut after = [0u8; 2];
        match i2c.async_write_read(TARGET_ADDR, &[1], &mut after).await {
            Ok(()) if after == EXPECT_WRITE_READ => {}
            Ok(()) => {
                stale += 1;
                error!("pass {}: STALE — write-read after a NACK gave {:?}", pass, after);
            }
            Err(e) => {
                stale += 1;
                error!("pass {}: write-read after a NACK failed: {}", pass, e);
            }
        }
        Timer::after_millis(SETTLE_MS).await;

        let mut read = [0u8; 2];
        let _ = i2c.async_read(TARGET_ADDR, &mut read).await;
        for _ in 0..NACK_BURST {
            let _ = i2c.async_read(ABSENT_ADDR, &mut [0u8; 2]).await;
        }
        match i2c.async_read(TARGET_ADDR, &mut read).await {
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

        info!("pass {}: phase A done, {} stale, {} false ok", pass, stale, false_ok);
        Timer::after_millis(SETTLE_MS).await;

        for i in 0..HAMMER {
            info!("pass {} hammer {}: starting", pass, i);
            let mut b = [0u8; 2];
            match i2c.async_write_read(TARGET_ADDR, &[1], &mut b).await {
                Ok(()) => {}
                Err(e) => info!("pass {} hammer {}: {}", pass, i, e),
            }
            info!("pass {} hammer {}: returned", pass, i);
        }

        info!("pass {}: phase B survived", pass);
        Timer::after_millis(SETTLE_MS).await;

        // --- C: cancel transfers part-way, then check the bus still works ---
        for i in 0..HAMMER {
            let mut b = [0u8; 2];
            // A timeout shorter than a transfer, so the future is dropped mid-flight rather than
            // completing. Racing it against a timer is how a real caller cancels.
            let cancelled = matches!(
                select(
                    i2c.async_write_read(TARGET_ADDR, &[1], &mut b),
                    Timer::after_micros(20 + (i as u64 * 7) % 400),
                )
                .await,
                Either::Second(()),
            );
            if !cancelled {
                continue;
            }
            cancels += 1;

            // Generous, because recovery is bounded by the *target*: a truncated transaction leaves the
            // U575 stretching SCL, measured at tens of milliseconds, and the controller cannot finish the
            // abandoned burst until it lets go. A bound at all is what makes a wedge distinguishable from
            // a slow target.
            let mut after = [0u8; 2];
            match select(
                i2c.async_write_read(TARGET_ADDR, &[1], &mut after),
                Timer::after_millis(RECOVER_MS),
            )
            .await
            {
                Either::First(Ok(())) if after == EXPECT_WRITE_READ => {}
                Either::First(Ok(())) => {
                    stale += 1;
                    error!("pass {} cancel {}: recovery gave {:?}", pass, i, after);
                }
                // The intended pattern for a stuck bus: the driver reports it rather than clocking the
                // lines behind the caller's back, and the caller decides whether to do it and retry.
                Either::First(Err(Error::BusStuck)) => {
                    match i2c.recover_stuck_bus() {
                        Ok(()) => info!("pass {} cancel {}: bus was stuck, recovered", pass, i),
                        Err(e) => {
                            warn!("pass {} cancel {}: bus stuck and unrecoverable: {}", pass, i, e);
                            unrecoverable += 1;
                            continue;
                        }
                    }

                    let mut retry = [0u8; 2];
                    match select(
                        i2c.async_write_read(TARGET_ADDR, &[1], &mut retry),
                        Timer::after_millis(RECOVER_MS),
                    )
                    .await
                    {
                        Either::First(Ok(())) if retry == EXPECT_WRITE_READ => {}
                        Either::First(Ok(())) => {
                            stale += 1;
                            error!("pass {} cancel {}: retry gave {:?}", pass, i, retry);
                        }
                        Either::First(Err(e)) => {
                            stale += 1;
                            error!("pass {} cancel {}: retry failed: {}", pass, i, e);
                        }
                        Either::Second(()) => {
                            wedged += 1;
                            error!("pass {} cancel {}: retry still running", pass, i);
                        }
                    }
                }
                Either::First(Err(e)) => {
                    stale += 1;
                    error!("pass {} cancel {}: recovery failed: {}", pass, i, e);
                }
                Either::Second(()) => {
                    wedged += 1;
                    error!(
                        "pass {} cancel {}: recovery still running after {} ms",
                        pass, i, RECOVER_MS
                    );
                }
            }
            Timer::after_millis(SETTLE_MS).await;
        }

        info!(
            "pass {}: phase C done, {} cancelled, {} stale, {} wedged, {} target would not release SDA",
            pass, cancels, stale, wedged, unrecoverable
        );
        Timer::after_millis(SETTLE_MS).await;
    }
}
