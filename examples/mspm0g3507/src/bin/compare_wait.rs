//! Awaiting a compare match, on two channels of one timer.
//!
//! **No wiring.** A compare match is a counter event, not a pin one, so the channels need no pins and
//! nothing outside the chip is involved. That is what makes this a test of the driver rather than of a
//! rig.
//!
//! # What it checks
//!
//! Each channel has its own waker, and the handler has to wake the one whose channel matched. Two
//! channels are armed a half period apart and awaited in turn, so the driver is asked to tell them
//! apart on every cycle:
//!
//! - **Waking the wrong channel stalls it.** The await for the other one never completes, no line is
//!   logged, and the run reports nothing rather than reporting something wrong.
//! - **Waking at the wrong time shows up as the interval.** `Ch0` is armed a quarter of the way through
//!   the period and `Ch1` three quarters, so both gaps are half a period whichever way round they are
//!   read.
//!
//! A pass is a steady stream of `ok` with the measured interval within [`TOLERANCE_MS`] of half the
//! period. Silence is a failure — see above.
//!
//! `TIMG6` has two channels, so this also covers the case where the driver's waker array is sized to
//! fewer than the four a `Channel` can name: a handler that walked all four would run off the end of it.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::peripherals::TIMG6;
use embassy_mspm0::tim::Channel;
use embassy_mspm0::tim::compare::{Compare, Config as CompareConfig, InterruptHandler};
use embassy_time::Instant;
use panic_probe as _;

/// Counter period. Long enough that the interval is comfortably above the time source's resolution,
/// short enough that a stall is obvious rather than something to wait out.
const PERIOD_MS: u64 = 100;

/// How far the measured half period may be from nominal before it is called a failure. The counter and
/// `embassy_time` run from different clocks, so exact agreement is not on offer.
const TOLERANCE_MS: u64 = 3;

bind_interrupts!(struct Irqs {
    TIMG6 => InterruptHandler<TIMG6>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // Divided hard, because the period has to fit a 16-bit counter: 32 MHz undivided wraps every 2 ms.
    let mut compare = Compare::new_2ch(
        p.TIMG6,
        None,
        None,
        Irqs,
        CompareConfig::new().with_divider(8).with_prescaler(256),
    );

    let tick_hz = compare.timer().tick_frequency();
    let period = (tick_hz as u64 * PERIOD_MS / 1000) as u32;
    unwrap!(compare.timer_mut().set_load_value(period));

    // A quarter and three quarters of the way through, so the two gaps are equal and neither lands near
    // the wrap, where a match and a reload would be hard to tell apart.
    let first = (period / 4) as u16;
    let second = (period * 3 / 4) as u16;

    info!(
        "TIMG6 at {} Hz, {} ticks per {} ms period; Ch0 at {}, Ch1 at {}",
        tick_hz, period, PERIOD_MS, first, second
    );

    compare.start();

    let expected = PERIOD_MS / 2;
    let mut previous: Option<Instant> = None;
    let mut checked = 0u32;
    let mut failed = 0u32;

    loop {
        // Whichever channel is next round the cycle. Arming the one that has just fired would wait a
        // whole period, so they alternate.
        for (channel, value) in [(Channel::Ch0, first), (Channel::Ch1, second)] {
            compare.channel(channel).wait_until(value).await;

            let now = Instant::now();

            if let Some(previous) = previous {
                let elapsed = (now - previous).as_millis();
                let error = elapsed as i64 - expected as i64;
                checked += 1;

                if error.unsigned_abs() > TOLERANCE_MS {
                    failed += 1;
                    error!(
                        "{:?}: {} ms between matches, {} off the expected {}",
                        channel, elapsed, error, expected
                    );
                } else if checked % 20 == 0 {
                    info!(
                        "{} checked, {} failed -- {} ms between matches, {} off -- ok",
                        checked, failed, elapsed, error
                    );
                }
            }

            previous = Some(now);
        }
    }
}
