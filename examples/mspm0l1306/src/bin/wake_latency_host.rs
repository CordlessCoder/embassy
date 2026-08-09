//! Times how long a G3507 takes to answer a wake edge, to about half a microsecond.
//!
//! Other side of the G3507 `wake_latency` check, which sleeps at each level in turn and drives `ack` as
//! soon as its task resumes. This drives the edge and times the reply against a 32 MHz counter, so the
//! wake-up latencies the HAL's `min_sleep` default is scaled from get measured rather than trusted.
//!
//! Wiring, with both boards sharing a ground:
//!
//! | L1306         | G3507         | Signal                                       |
//! |---------------|---------------|----------------------------------------------|
//! | `PA10` (out)  | `PB7`  (in)   | wake — driven high, then low once acked      |
//! | `PA1`  (in)   | `PB2`  (out)  | ack — the target's reply                     |
//!
//! # What it measures
//!
//! `TIMG2` free-runs from the 32 MHz bus clock, so one tick is 31.25 ns. The edge goes out, a polling
//! loop watches `ack`, and the counter is read on the way out. The loop itself is the resolution limit:
//! it reads two registers per turn, so the answer is good to something like half a microsecond against
//! latencies of 8 to 20 µs. A logic analyser on the two pins is the finer instrument — this is the one
//! that needs no instrument.
//!
//! The target's own reply path — GPIO interrupt, waker, one executor poll — is inside the number and is
//! a few hundred nanoseconds of it. Its `wfi` case is the control that shows how much.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Input, Level, Output, Pull};
use embassy_mspm0::tim::{ClockSel, low_level};
use embassy_time::Timer;
use panic_halt as _;

/// How long to wait for a reply. Kept well inside the counter's 2.048 ms wrap at 32 MHz so a timeout
/// cannot be mistaken for a fast reply, and well over the 252 µs the deepest mode on the target could
/// plausibly need.
const TIMEOUT_US: u32 = 1_000;

/// Gap between edges. The target logs, re-arms and gets back to sleep in this window; too short and it
/// is measured on its way in rather than at rest.
const PERIOD_MS: u64 = 200;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut wake = Output::new(p.PA10, Level::Low);
    let ack = Input::new(p.PA1, Pull::Down);
    let mut led = Output::new(p.PA0, Level::Low);

    let counter = low_level::Timer::new(
        p.TIMG2,
        low_level::Config {
            clock: ClockSel::BusClk,
            // Undivided: the whole point is to resolve microseconds.
            prescaler: 1,
            counter_on_enable: low_level::CounterOnEnable::Preserve,
            free_run_in_debug: true,
            ..Default::default()
        },
    );
    counter.start();

    let tick_hz = counter.tick_frequency();
    let timeout_ticks = (tick_hz as u64 * TIMEOUT_US as u64 / 1_000_000) as u32;

    // A counter reloading from its maximum wraps modulo `max + 1`.
    let wrap: u32 = counter.max_count().into();

    info!(
        "counting at {} Hz, {} ns a tick, timing out after {} ticks",
        tick_hz,
        1_000_000_000 / tick_hz,
        timeout_ticks,
    );
    defmt::assert!(timeout_ticks < wrap / 2, "the timeout outlasts the counter");

    loop {
        led.toggle();

        // A reply still asserted from last time would read as an instant wake.
        if ack.is_high() {
            warn!("ack is still high, skipping this edge");
            Timer::after_millis(PERIOD_MS).await;
            continue;
        }

        let start: u32 = counter.counter().into();
        wake.set_high();

        // Polling rather than an interrupt: an edge waiter would add its own wake path to a
        // measurement of somebody else's.
        let elapsed = loop {
            let elapsed = (u32::from(counter.counter()).wrapping_sub(start)) & wrap;

            if ack.is_high() {
                break Some(elapsed);
            }

            if elapsed > timeout_ticks {
                break None;
            }
        };

        wake.set_low();

        match elapsed {
            Some(ticks) => info!(
                "acked after {} ticks, {} ns",
                ticks,
                ticks as u64 * 1_000_000_000 / tick_hz as u64
            ),
            None => warn!("no ack within {} us", TIMEOUT_US),
        }

        Timer::after_millis(PERIOD_MS).await;
    }
}
