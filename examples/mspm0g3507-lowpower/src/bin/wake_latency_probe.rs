//! `wake_latency` with a marker pin across the GPIO interrupt handler, to find where a wake's time goes.
//!
//! B3 measured 35-49 µs from wake edge to the task's answer, of which only 11-14 µs is the sleep mode
//! itself: the `wfi` case, which never leaves RUN, still costs ~29 µs. B3b ruled out the executor hand-off
//! by moving the task to an interrupt-mode executor and getting nothing back. This splits what is left.
//!
//! # What the markers give
//!
//! `embassy-mspm0`'s `_probe` feature brackets three stages of a wake with pins. Together with the wake
//! edge and the task's own `ack` they cut one wake into five measurable pieces:
//!
//! | Segment | What is in it |
//! |---|---|
//! | wake edge → `PB13` rise | sleep-mode wake, `low_power::sleep`'s post-wake work, critical-section exit, interrupt entry, `GROUP1` demux |
//! | `PB13` rise → `PB0` rise | the handler's register work before the wake: `MIS` and `DIN31_0` reads, `ICLR` |
//! | `PB0` rise → fall | the `WaitMap` wake alone |
//! | `PB0` fall → `PB13` fall | the `IMASK` read-modify-write that closes the handler |
//! | `PB13` fall → `PB1` rise | interrupt return, thread-mode resume, leaving the critical section, the executor loop |
//! | `PB1` rise → `ack` rise | the executor dispatching to the task, and the task up to its pin write |
//!
//! `PB1` falls after `ack` rises, because the task's write happens inside the poll it brackets.
//!
//! The first segment is the answer to "how long to the interrupt, without the runtime" — the marker is the
//! handler's earliest instruction, so no HAL dispatch or executor is inside it. It cannot be made smaller
//! from an example: `GROUP1` and the `GPIOB` group vector are both `no_mangle` in the HAL, so an
//! application cannot substitute its own handler.
//!
//! Subtract the `wfi` case from a deep-sleep case *within one segment* to get that mode's silicon cost, the
//! same way B3 does. Each marker's own store sits inside the segment that precedes it, making that segment
//! pessimistic by a few cycles — constant, so it cancels out of any differential.
//!
//! # Analyser
//!
//! B3's rig plus three leads. All three pins are free in this example.
//!
//! | Channel | Pin | Header | Marker |
//! |---|---|---|---|
//! | D0 | G3507 `PB7` | J2.14 | wake in |
//! | D1 | G3507 `PB2` | J1.9 | ack out |
//! | D4 | G3507 `PB13` | J4.35 | `Marker::GpioHandler` |
//! | D5 | G3507 `PB0` | J2.12 | `Marker::GpioWaker` |
//! | D6 | G3507 `PB1` | J4.39 | `Marker::ExecutorPoll` |
//!
//! Wiring to the L1306 is unchanged, and `wake_latency_host` runs there as before.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_group_interrupts;
use embassy_mspm0::gpio::{self, Input, Level, Output, Port, Pull};
use embassy_mspm0::low_power::{DEFAULT_MIN_SLEEP, MAX_WAKE_NS};
use embassy_mspm0::probe::{self, Marker};
use embassy_mspm0::sysctl::{SleepLevel, WakeGuard};
use embassy_time::Timer;
use panic_halt as _;

/// Same list and order as `wake_latency`, so the phase-recovery trick still works and the numbers are
/// directly comparable: the two STANDBY cases are the only adjacent pair the datasheet gives one figure.
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

/// How long `ack` is held high. The measurement is of the edge, so the width does not enter it.
const ACK_HOLD_MS: u64 = 1;

// Every port has to be bound, because which one a pin belongs to is not known until run time.
bind_group_interrupts!(struct Irqs {
    GPIOA => gpio::InterruptHandler;
    GPIOB => gpio::InterruptHandler;
});

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    info!(
        "the table says {} ns for the deepest mode, and min_sleep defaults to {} ticks",
        MAX_WAKE_NS,
        DEFAULT_MIN_SLEEP.as_ticks(),
    );

    // Held for the program's life: dropping one would return its pin to the reset state while the HAL is
    // still writing to it.
    let _handler = Output::new(p.PB13, Level::Low);
    let _waker = Output::new(p.PB0, Level::Low);
    let _poll = Output::new(p.PB1, Level::Low);

    probe::arm(Marker::GpioHandler, Port::PortB, 13);
    probe::arm(Marker::GpioWaker, Port::PortB, 0);
    probe::arm(Marker::ExecutorPoll, Port::PortB, 1);

    let mut wake = Input::new_async(p.PB7, Pull::Down, Irqs);
    let mut ack = Output::new(p.PB2, Level::Low);

    loop {
        for (guard, name) in CASES {
            info!("{}: {} wakes", name, WAKES_PER_CASE);

            let _guard = guard.map(WakeGuard::new);

            for _ in 0..WAKES_PER_CASE {
                // The stimulus has to be idle before arming. `GPIO_ERR_01` on this family loses the next
                // edge if the pin is still asserted when the chip goes back to sleep, and an already-low
                // pin returns from this immediately.
                wake.wait_for_low().await;

                wake.wait_for_rising_edge().await;

                ack.set_high();
                Timer::after_millis(ACK_HOLD_MS).await;
                ack.set_low();
            }
        }
    }
}
