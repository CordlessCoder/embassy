//! Checks that `Config::min_sleep` keeps short sleeps out of a deep-sleep mode, by measuring what each
//! sleep costs.
//!
//! The L-series half of the G3507 `min_sleep_gate` check. Same measurement, different SYSCTL family and
//! different published latencies — this part gives 14 µs for STOP1, 13 for STOP2 and 15 for both
//! STANDBY modes, and **no figure at all for STOP0**, which is why the HAL scales `min_sleep` from the
//! deepest published figure rather than looking up the mode it is about to enter.
//!
//! Walks a set of sleep lengths from under the threshold to well over it, timing a batch of a thousand
//! sleeps at each. What is measured is the *excess* over what was asked for: a sleep pays its mode's
//! wake-up latency once on the way out, so a batch the gate allows into STANDBY should run about
//! `MAX_WAKE_NS` longer per sleep than one it rejects into `WFI`. On this part that is 15 µs, half a
//! tick — invisible in one sleep, about 490 ticks over a batch.
//!
//! Nothing but the sleep length changes between cases, so the step in the per-sleep cost is the gate.
//!
//! | Pin    | Signal                                                                  |
//! |--------|-------------------------------------------------------------------------|
//! | `PA10` | high for the batches the gate should allow, low for the ones it rejects  |
//! | `PA0`  | LED1, toggled once per pass as a liveness check                          |
//!
//! `PA10` is what a current probe needs: supply current should drop to the STANDBY figure exactly while
//! it is high, and sit at RUN current while it is low. It pulses ten times at startup, so a flat line
//! means the pin is not reaching the probe rather than the run being broken. **Unfit the LED1 jumper
//! before measuring current** — a lit LED is milliamps and swamps everything being compared here.
//!
//! With `DEFMT_LOG=trace` the HAL says so itself, once per rejected sleep:
//! `Waking in 4 ticks, under the 8 tick minimum`. That is the decision; the timing below is its cost.
//!
//! Two things to expect in the numbers:
//!
//! - **The case at the threshold can land either way.** The window is measured when the executor goes
//!   idle, a fraction of a tick after the deadline was set, so an 8 tick sleep is often 7 ticks away by
//!   then and gets rejected.
//! - **Every case carries a fixed overhead** on top of the wake-up latency: a `Timer` deadline is
//!   rounded up to a whole tick, and polling the task back costs a few microseconds. It applies to all
//!   cases equally, so compare the cases against each other rather than against zero.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::low_power::{DEFAULT_MIN_SLEEP, MAX_WAKE_NS};
use embassy_time::{Duration, Instant, TICK_HZ, Timer};
use panic_halt as _;

/// The threshold under test, in ticks. Well above the two ticks this part defaults to, so there is room
/// for cases on both sides of it.
const MIN_SLEEP_TICKS: u64 = 8;

/// Sleep lengths to walk, in ticks. Under [`MIN_SLEEP_TICKS`] the gate rejects them, over it the gate
/// allows them, and at it the answer depends on how much of the tick the executor spends getting to
/// idle.
const CASES: &[u64] = &[2, 4, 7, 8, 10, 16, 32];

/// Sleeps per case. One wake-up latency is under a tick, so it only shows up once enough of them have
/// accumulated to move the tick count.
const SLEEPS: u32 = 1000;

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut config = embassy_mspm0::Config::default();
    config.min_sleep = Duration::from_ticks(MIN_SLEEP_TICKS);
    let p = embassy_mspm0::init(config);

    info!(
        "min_sleep {} ticks; this part wakes in {} ns and would default to {} ticks",
        MIN_SLEEP_TICKS,
        MAX_WAKE_NS,
        DEFAULT_MIN_SLEEP.as_ticks(),
    );

    let mut allowed_pin = Output::new(p.PA10, Level::Low);
    let mut led = Output::new(p.PA0, Level::Low);

    // Proves the pin reaches the probe before a current trace is aligned against it.
    for _ in 0..10 {
        allowed_pin.set_high();
        Timer::after_millis(50).await;
        allowed_pin.set_low();
        Timer::after_millis(50).await;
    }

    loop {
        led.toggle();

        for &ticks in CASES {
            let allowed = ticks >= MIN_SLEEP_TICKS;
            info!(
                "{} ticks x {}, gate should {}",
                ticks,
                SLEEPS,
                if allowed { "allow" } else { "reject" }
            );

            allowed_pin.set_level(allowed.into());

            // Timed with nothing else in the loop: a log line here would cost more than the latency
            // being measured.
            let start = Instant::now();
            for _ in 0..SLEEPS {
                Timer::after_ticks(ticks).await;
            }
            let elapsed = start.elapsed().as_ticks();

            allowed_pin.set_low();

            // A timer never fires early, so the excess cannot go negative.
            let excess = elapsed - ticks * SLEEPS as u64;
            let per_sleep_ns = excess * 1_000_000_000 / (TICK_HZ * SLEEPS as u64);

            info!(
                "  {} ticks elapsed, {} over, {} ns per sleep",
                elapsed, excess, per_sleep_ns
            );
        }
    }
}
