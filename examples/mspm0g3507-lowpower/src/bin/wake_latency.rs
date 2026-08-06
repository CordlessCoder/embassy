//! Answers a wake edge as fast as it can, at every sleep level, so the L1306 can time the answer.
//!
//! This is the measurement `Config::min_sleep` is derived from. The default threshold is four times
//! `low_power::MAX_WAKE_NS`, which comes from a datasheet table of typical figures that nobody has
//! checked — this checks it. Pair it with `wake_latency_host` on an L1306, which drives the edge and
//! times the reply to 31 ns.
//!
//! The task waits on a rising edge, so the executor idles into the deepest mode the case allows and
//! the edge is what brings the chip back. What the host measures is therefore the wake-up latency plus
//! the GPIO interrupt and one executor poll, which is a few hundred nanoseconds of the total.
//!
//! Wiring, with both boards sharing a ground:
//!
//! | G3507         | L1306          | Signal                                       |
//! |---------------|----------------|----------------------------------------------|
//! | `PB7`  (in)   | `PA10` (out)   | wake — the host drives it high, then low again |
//! | `PB2`  (out)  | `PA1`  (in)    | ack — driven high as soon as the task resumes  |
//!
//! `PA0` drives the on-board LED once per case as a liveness check.
//!
//! # What to check
//!
//! - **The latency the host reports against this device's table.** For an MSPM0G3507 the datasheet
//!   gives 12.1 µs for STOP0, 13.5 for STOP1, 12.9 for STOP2 and 15.2 for both STANDBY modes. The
//!   `wfi` case is the control: it never leaves RUN, so it should come back in well under a
//!   microsecond and shows what the interrupt and the poll cost on their own.
//! - **The figures are typical, not ceilings** — TI publishes one number per mode spanning the MIN,
//!   TYP and MAX columns. A measurement a little over the table is not a contradiction. One several
//!   times over it means the mode is not what this thinks it is.
//! - **STOP2 coming back faster than STOP1**, which the table claims (12.9 against 13.5). It is the
//!   one non-monotonic pair on this part, and the reason the HAL does not assume deeper costs more.
//! - **`PMCU_ERR_08`** (this family): an edge that arrives while the chip is still on its way into the
//!   mode adds about 3 µs, with no workaround. Occasional high outliers are expected.
//! - **Rare fast outliers** are the `min_sleep` gate doing its job: the time driver ticks twice per
//!   counter period, so a sleep armed within `min_sleep` of one of those is left as a plain `WFI`.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_group_interrupts;
use embassy_mspm0::gpio::{self, Input, Level, Output, Pull};
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

/// Wakes per case, so the spread at each level is visible rather than one sample of it. The host logs
/// one line per wake and this logs one per case, so the two streams stay in step.
const WAKES_PER_CASE: u32 = 8;

/// How long `ack` is held high. Long enough for the host's polling loop and an analyser to catch it
/// without ambiguity; the measurement is of the edge, so the width does not enter it.
const ACK_HOLD_MS: u64 = 1;

// Every port has to be bound, because which one a pin belongs to is not known until run time.
bind_group_interrupts!(struct Irqs {
    GPIOA => gpio::InterruptHandler;
    GPIOB => gpio::InterruptHandler;
});

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

    let mut wake = Input::new_async(p.PB7, Pull::Down, Irqs);
    let mut ack = Output::new(p.PB2, Level::Low);
    let mut led = Output::new(p.PA0, Level::High);
    led.set_inversion(true);

    loop {
        for (guard, name) in CASES {
            led.toggle();
            info!("{}: {} wakes", name, WAKES_PER_CASE);

            let _guard = guard.map(WakeGuard::new);

            for _ in 0..WAKES_PER_CASE {
                // The stimulus has to be idle before arming. `GPIO_ERR_01` on this family loses the
                // next edge if the pin is still asserted when the chip goes back to sleep, and an
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
