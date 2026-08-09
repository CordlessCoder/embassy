//! `wake_latency`, with the waking task on an interrupt-mode executor instead of the thread one.
//!
//! Pair it with the same `wake_latency_host` on an L1306 and the same wiring, so the two binaries are a
//! direct A/B: identical cases, identical wake count, identical ack hold. Flash one, capture, flash the
//! other, capture, compare per case.
//!
//! # What differs
//!
//! On the thread executor the answer to an edge travels: GPIO interrupt → waker → return to thread mode
//! → the executor's loop → poll the task → drive `ack`. Here the pender pends [`EXEC_IRQ`] instead, so
//! the NVIC tail-chains from the GPIO handler straight into the executor and the task is polled without
//! thread mode ever running.
//!
//! The thread executor still exists and still owns the sleeping: it has nothing to poll, so it idles into
//! [`low_power::sleep`](embassy_mspm0::low_power::sleep) exactly as before, and the `WakeGuard`s the task
//! holds still decide how deep. What changes is only who polls the task on the way out.
//!
//! Both interrupts sit at [`Priority::P0`], which is what makes the hand-off a tail-chain rather than a
//! return to thread mode and a fresh dispatch.
//!
//! # What to expect
//!
//! Read this before reading a result as a win. `wake_latency`'s `wfi` case — the one that never leaves
//! RUN, so its figure is pure software — costs **28.8 µs**. Whatever that is, it is *also* in every
//! deep-sleep case, and the thread-mode return this example removes is only one part of it. So the honest
//! prediction is a modest saving, not the whole pedestal.
//!
//! What makes the comparison worth running anyway is that it splits the pedestal in two: the part that is
//! the executor hand-off, and the part that is the GPIO interrupt path and `low_power::sleep`'s own
//! post-wake work. Those cannot be separated from the thread-executor number alone.
//!
//! The per-case *differentials* should not move at all — they are silicon. If they do, something about
//! this arrangement changed the sleep depth, which would be a bug rather than a result.
//!
//! Wiring, with both boards sharing a ground:
//!
//! | G3507         | L1306          | Signal                                         |
//! |---------------|----------------|------------------------------------------------|
//! | `PB7`  (in)   | `PA10` (out)   | wake — the host drives it high, then low again |
//! | `PB2`  (out)  | `PA1`  (in)    | ack — driven high as soon as the task resumes  |

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::executor::InterruptExecutor;
use embassy_mspm0::gpio::{self, Input, Level, Output, Pull};
use embassy_mspm0::interrupt::{Interrupt, InterruptExt, Priority};
use embassy_mspm0::low_power::{DEFAULT_MIN_SLEEP, MAX_WAKE_NS};
use embassy_mspm0::mode::Async;
use embassy_mspm0::sysctl::{SleepLevel, WakeGuard};
use embassy_mspm0::{bind_group_interrupts, interrupt};
use embassy_time::Timer;
use panic_halt as _;

/// Each case guards one level deeper than the sleep it wants, since a guard blocks its own level and
/// everything below it. The first blocks all deep sleep, leaving a plain `WFI`; the last blocks nothing.
///
/// Same list and same order as `wake_latency`, so the phase-recovery trick still works: the two STANDBY
/// cases are the only adjacent pair the datasheet gives the same figure, which fixes the offset.
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

/// The interrupt the executor is polled from.
///
/// MSPM0 has no interrupt reserved for software, so this borrows one from a peripheral the example never
/// touches. `AES` is a safe pick here: nothing in this binary can raise it, so every entry is the pender's.
const EXEC_IRQ: Interrupt = Interrupt::AES;

static EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn AES() {
    unsafe { EXECUTOR.on_interrupt() }
}

/// Answers the edge. Runs in [`EXEC_IRQ`], not thread mode.
#[embassy_executor::task]
async fn answer(mut wake: Input<'static, Async>, mut ack: Output<'static>) -> ! {
    loop {
        for (guard, name) in CASES {
            info!("{}: {} wakes", name, WAKES_PER_CASE);

            let _guard = guard.map(WakeGuard::new);

            for _ in 0..WAKES_PER_CASE {
                // The stimulus has to be idle before arming. `GPIO_ERR_01` on this family loses the next
                // edge if the pin is still asserted when the chip goes back to sleep, and an already-low
                // pin returns from this immediately.
                wake.wait_for_low().await;

                // Returning Pending here is what lets the thread executor reach its idle and sleep.
                wake.wait_for_rising_edge().await;

                ack.set_high();
                Timer::after_millis(ACK_HOLD_MS).await;
                ack.set_low();
            }
        }
    }
}

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

    let wake = Input::new_async(p.PB7, Pull::Down, Irqs);
    let ack = Output::new(p.PB2, Level::Low);

    // Must be set before `start`, which unmasks. P0 matches the GPIO group interrupt that pends this one,
    // so the hand-off is a tail-chain instead of a return to thread mode.
    EXEC_IRQ.set_priority(Priority::P0);
    // `SpawnError` has no `defmt::Format`, and a static message is cheaper than deriving one.
    let Ok(token) = answer(wake, ack) else {
        core::panic!("the answer task was already spawned")
    };
    EXECUTOR.start(EXEC_IRQ).spawn(token);

    // Nothing else belongs on the thread executor: leaving it with no ready task is what sends it into
    // `low_power::sleep` and keeps this a measurement of the interrupt path.
    core::future::pending().await
}
