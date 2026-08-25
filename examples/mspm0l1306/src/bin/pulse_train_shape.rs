//! A pulse train's polarity, its resting state, and where it stops.
//!
//! **One analyser channel on BoosterPack 36 (PA10), and a ground.** Nothing else is wired, and
//! nothing else on the board drives the pin.
//!
//! `pulse_train.rs` covers the widths and the counting. This one covers the shape of the ends, which
//! is what decides whether an arbitrary sequence is reachable. TIMG4 again, ticking at 1 MHz, so
//! every width below is in microseconds.
//!
//! Every phase emits the same three elements — 10, 20 and 30 us high with a 20 us gap — so the only
//! thing that changes is the shape of the ends.
//!
//! **The first two phases rest high on purpose, and that is what makes them tell apart.** With a low
//! resting level a dropped trailing low is invisible: the train ends low and stays low either way,
//! and the only difference is 20 us of a gap that also contains the driver being rebuilt. Resting
//! high turns the same 20 us into a notch between the last pulse and the resting level, which is
//! either present or absent.
//!
//! - **complete, resting high** — `H10 L20 H20 L20 H30` then a **20 us low**, then rests high.
//! - **at the final compare, resting high** — `H10 L20 H20 L20 H30` then rests high **with no notch**.
//!   Three highs and two lows: the odd sequence a whole number of periods cannot express.
//! - **inverted** — the same elements with the polarity flipped: `L10 H20 L20 H20 L30 H20`, so the
//!   train begins with a low period and ends with a high one, and it rests low.
//! - **released** — the pin is left undriven between trains. With no pull that reads as an
//!   indeterminate level rather than a clean one; with the internal pull-up it settles high. **The
//!   pin is still driven for the handler's own latency** after the closing boundary, before the
//!   release lands.
//!
//! Each driver is held alive across its own gap. Dropping it disconnects the pin, so a driver
//! released early would make every resting level read as a floating one — which is what the first
//! version of this example did, and it could not have failed.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::gpio::Pull;
use embassy_mspm0::peripherals::TIMG4;
use embassy_mspm0::tim::pulse_train::{Config as TrainConfig, End, Idle, InterruptHandler, Pulse, PulseTrain};
use embassy_mspm0::tim::simple_pwm::Polarity;
use embassy_time::{Duration, Timer};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    TIMG4 => InterruptHandler<TIMG4>;
});

/// Three rising widths, so the order is readable off one capture and an off-by-one is a missing
/// pulse rather than a subtle width error.
const RAMP: [Pulse; 3] = [
    Pulse { high: 10, low: 20 },
    Pulse { high: 20, low: 20 },
    Pulse { high: 30, low: 20 },
];

/// 32 MHz / 8 / 4, so one tick is one microsecond.
const fn base() -> TrainConfig {
    TrainConfig::new().with_divider(8).with_prescaler(4)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut timer = p.TIMG4;
    let mut pin = p.PA10;

    loop {
        for (name, config) in [
            ("complete, resting high", base().with_idle(Idle::High)),
            (
                "at the final compare, resting high",
                base().with_idle(Idle::High).with_end(End::AtFinalCompare),
            ),
            (
                "inverted",
                base().with_idle(Idle::Low).with_polarity(Polarity::ActiveLow),
            ),
            ("released", base().with_idle(Idle::HighImpedance)),
        ] {
            // Rebuilt per phase because the shape is configuration rather than a per-train argument.
            let mut train = PulseTrain::new(timer.reborrow(), pin.reborrow(), Pull::None, Irqs, config);

            train.emit(&RAMP).await;

            info!("{} done", name);

            // Held alive across the gap: dropping it disconnects the pin, and the resting level is
            // half of what each phase is here to show.
            Timer::after(Duration::from_millis(5)).await;
        }

        Timer::after(Duration::from_millis(20)).await;
    }
}
