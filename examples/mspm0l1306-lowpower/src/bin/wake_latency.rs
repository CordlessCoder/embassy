//! Answers a wake edge as fast as it can, at every sleep level, so a G3507 can time the answer.
//!
//! The L1306 half of `wake_latency`. The G3507 version is the same measurement on the other family; this
//! one exists because the two parts have different SYSCTL blocks, and the sleep levels the HAL offers are
//! the HAL's names, not the silicon's. What each one costs here is a separate question from what it costs
//! there. Pair it with `wake_latency_host` on a G3507, which drives the edge and times the reply.
//!
//! The task waits on a rising edge, so the executor idles into the deepest mode the case allows and the
//! edge is what brings the chip back. What the host measures is the wake-up latency plus the GPIO
//! interrupt and one executor poll.
//!
//! Wiring, with both boards sharing a ground:
//!
//! | L1306         | G3507          | Signal                                        |
//! |---------------|----------------|-----------------------------------------------|
//! | `PA10` (in)   | `PB7`  (out)   | wake — the host drives it high, then low again |
//! | `PA1`  (out)  | `PB2`  (in)    | ack — driven high as soon as the task resumes  |
//!
//! Those are the same two wires the G3507-as-target version uses, with each end doing the other's job, so
//! swapping which board is under test needs no rewiring.
//!
//! No liveness LED: `PA0` is open-drain with J9 shipped open on this board, so it reads as floating
//! rather than as a level, and it is this part's `I2C0` SDA besides. The two nets show the loop running.
//!
//! # What to check
//!
//! - **The latency against this device's table.** SLASEX0D gives 14 µs for STOP1, 13 for STOP2 and 15 for
//!   STANDBY, all typicals. It publishes **no STOP0 figure and one STANDBY figure**, not two, so two of
//!   the six cases here have nothing to check against — what they cost is the measurement's own answer.
//! - **STOP2 faster than STOP1** (13 against 14), the same non-monotonic pair the G3507 has.
//! - **`wfi` as the control.** It never leaves RUN, so it is the software path on its own: GPIO interrupt,
//!   waker, one executor poll. Subtract it to get silicon.
//! - **Cases that come out equal.** The HAL's five levels are not obliged to be five distinct states on
//!   this part. Two adjacent cases agreeing to a few nanoseconds is a result, not a fault.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Input, Level, Output, Pull};
use embassy_mspm0::low_power::{DEFAULT_MIN_SLEEP, MAX_WAKE_NS};
use embassy_mspm0::sysctl::{SleepLevel, WakeGuard};
use embassy_time::Timer;
use panic_halt as _;

/// Each case guards one level deeper than the sleep it wants, since a guard blocks its own level and
/// everything below it. The first blocks all deep sleep, leaving a plain `WFI`; the last blocks nothing.
const CASES: &[(Option<SleepLevel>, &str)] = &[
    (Some(SleepLevel::Stop0), "wfi"),
    (Some(SleepLevel::Stop1), "stop0"),
    (Some(SleepLevel::Stop2), "stop1"),
    (Some(SleepLevel::Standby0), "stop2"),
    (Some(SleepLevel::Standby1), "standby0"),
    (None, "standby1"),
];

/// Wakes per case, so the spread at each level is visible rather than one sample of it.
const WAKES_PER_CASE: u32 = 8;

/// How long `ack` is held high. Long enough for the host's polling loop and an analyser to catch it
/// without ambiguity; the measurement is of the edge, so the width does not enter it.
const ACK_HOLD_MS: u64 = 1;

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // Deep sleep can drop the debug connection, so leave a window to re-flash through.
    info!("re-flash window, starting in 5s");
    Timer::after_secs(5).await;

    info!(
        "the table says {} ns for the deepest mode, and min_sleep defaults to {} ticks",
        MAX_WAKE_NS,
        DEFAULT_MIN_SLEEP.as_ticks(),
    );

    let mut wake = Input::new(p.PA10, Pull::Down);
    let mut ack = Output::new(p.PA1, Level::Low);

    loop {
        for (guard, name) in CASES {
            info!("{}: {} wakes", name, WAKES_PER_CASE);

            let _guard = guard.map(WakeGuard::new);

            for _ in 0..WAKES_PER_CASE {
                // The stimulus has to be idle before arming. `GPIO_ERR_01` applies here too, and an
                // already-low pin returns from this immediately.
                wake.wait_for_low().await;

                // The sleep happens here: nothing else is pending, so the executor idles into the
                // deepest mode this case allows and the edge is what ends it.
                wake.wait_for_rising_edge().await;

                ack.set_high();
                Timer::after_millis(ACK_HOLD_MS).await;
                ack.set_low();
            }
        }
    }
}
