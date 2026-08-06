//! `wake_latency` with marker pins across the wake path, to find where an L1306 wake's time goes.
//!
//! The L1306 half of the G3507's `wake_latency_probe`. Same markers, same segments, different family — so
//! the two can be read side by side and the parts of the path that are silicon can be told from the parts
//! that are the HAL.
//!
//! # What the markers give
//!
//! `embassy-mspm0`'s `_probe` feature brackets three stages of a wake with pins. Together with the wake
//! edge and the task's own `ack` they cut one wake into measurable pieces:
//!
//! | Segment | What is in it |
//! |---|---|
//! | wake edge → `PA16` rise | sleep-mode wake, `low_power::sleep`'s post-wake work, interrupt entry, dispatch |
//! | `PA16` rise → `PA17` rise | the handler's register work before the wake: `MIS` and `DIN31_0` reads, `ICLR` |
//! | `PA17` rise → fall | reporting the edge to the waiter alone |
//! | `PA17` fall → `PA16` fall | the `IMASK` read-modify-write that closes the handler |
//! | `PA16` fall → `PA18` rise | interrupt return, thread-mode resume, the executor loop |
//! | `PA18` rise → `ack` rise | the executor dispatching to the task, and the task up to its pin write |
//!
//! `PA18` falls after `ack` rises, because the task's write happens inside the poll it brackets.
//!
//! Subtract the `wfi` case from a deep-sleep case *within one segment* to get that mode's silicon cost.
//! Each marker's own store sits inside the segment that precedes it, making that segment pessimistic by a
//! few cycles — constant, so it cancels out of any differential.
//!
//! **The absolutes here are inflated by the marker writes and are not quotable.** Run `wake_latency` for a
//! wake latency; run this for where it went.
//!
//! # Analyser
//!
//! The `wake_latency` rig plus three leads, all on one contiguous run of J2. None of the three pins has a
//! jumper, an LED or a sensor on it, unlike most of what this board breaks out.
//!
//! | Channel | Pin | Header | Marker |
//! |---|---|---|---|
//! | D2 | L1306 `PA10` | J4.36 | wake in |
//! | D3 | L1306 `PA1` | J1.9 | ack out |
//! | D4 | L1306 `PA16` | J2.24 | `Marker::Handler` |
//! | D5 | L1306 `PA17` | J2.25 | `Marker::Waker` |
//! | D6 | L1306 `PA18` | J2.26 | `Marker::Poll` |
//!
//! Wiring to the G3507 is unchanged, and `wake_latency_host` runs there as before.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Input, Level, Output, Port, Pull};
use embassy_mspm0::low_power::{DEFAULT_MIN_SLEEP, MAX_WAKE_NS};
use embassy_mspm0::probe::{self, Marker};
use embassy_mspm0::sysctl::{SleepLevel, WakeGuard};
use embassy_time::Timer;
use panic_halt as _;

/// Same list and order as `wake_latency`, so the phase recovery works the same way and the numbers are
/// directly comparable.
const CASES: &[(Option<SleepLevel>, &str)] = &[
    (Some(SleepLevel::Stop0), "wfi"),
    (Some(SleepLevel::Stop1), "stop0"),
    (Some(SleepLevel::Stop2), "stop1"),
    (Some(SleepLevel::Standby0), "stop2"),
    (Some(SleepLevel::Standby1), "standby0"),
    (None, "standby1"),
];

const WAKES_PER_CASE: u32 = 8;

const ACK_HOLD_MS: u64 = 1;

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    info!("re-flash window, starting in 5s");
    Timer::after_secs(5).await;

    info!(
        "the table says {} ns for the deepest mode, and min_sleep defaults to {} ticks",
        MAX_WAKE_NS,
        DEFAULT_MIN_SLEEP.as_ticks(),
    );

    let mut wake = Input::new(p.PA10, Pull::Down);
    let mut ack = Output::new(p.PA1, Level::Low);

    // Claimed so nothing else can drive them, then handed to the HAL by port and pin — `probe` writes the
    // registers directly, because the code it instruments cannot borrow an `Output`.
    let _handler = Output::new(p.PA16, Level::Low);
    let _waker = Output::new(p.PA17, Level::Low);
    let _poll = Output::new(p.PA18, Level::Low);

    probe::arm(Marker::Handler, Port::PortA, 16);
    probe::arm(Marker::Waker, Port::PortA, 17);
    probe::arm(Marker::Poll, Port::PortA, 18);

    loop {
        for (guard, name) in CASES {
            info!("{}: {} wakes", name, WAKES_PER_CASE);

            let _guard = guard.map(WakeGuard::new);

            for _ in 0..WAKES_PER_CASE {
                wake.wait_for_low().await;
                wake.wait_for_rising_edge().await;

                ack.set_high();
                Timer::after_millis(ACK_HOLD_MS).await;
                ack.set_low();
            }
        }
    }
}
