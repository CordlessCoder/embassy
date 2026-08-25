//! A pulse train's polarity, its resting state, and where it stops.
//!
//! **One analyser channel on BoosterPack 36 (PA10), and a ground.** Nothing else is wired, and
//! nothing else on the board drives the pin.
//!
//! `pulse_train.rs` covers the widths and the counting. This one covers the shape of the ends, which
//! is what decides whether an arbitrary sequence is reachable. TIMG4 again, ticking at 1 MHz, so
//! every width below is in microseconds.
//!
//! Four phases repeat, each separated by a long idle gap. Every one emits the same three elements,
//! so what changes between them is only the shape:
//!
//! - **plain** — active high, ends after the last element's low. Three highs, three lows.
//! - **stop at the compare** — active high, ends at the last element's compare match. Three highs
//!   and **two** lows, and the train is shorter by the final low. This is the odd sequence a whole
//!   number of periods cannot express.
//! - **inverted** — the same three elements with the polarity flipped, so the train begins with a
//!   low period and ends with a high one, and the pin rests high.
//! - **released** — active high, and the pin is left undriven between trains. On an analyser with no
//!   pull that reads as an indeterminate level rather than a clean one; with the internal pull-up on
//!   it settles high. **The pin is still driven for the handler's own latency** after the closing
//!   boundary, before the release lands.
//!
//! A pass is: the counts above, every high 10, 20 and 30 us in order, every low 20 us, and the
//! inverted phase's first edge falling rather than rising.

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
            ("plain", base().with_idle(Idle::Low)),
            (
                "stop at the compare",
                base().with_idle(Idle::Low).with_end(End::AtFinalCompare),
            ),
            (
                "inverted",
                base().with_idle(Idle::High).with_polarity(Polarity::ActiveLow),
            ),
            ("released", base().with_idle(Idle::HighImpedance)),
        ] {
            // Rebuilt per phase because the shape is configuration rather than a per-train argument.
            let mut train = PulseTrain::new(timer.reborrow(), pin.reborrow(), Pull::None, Irqs, config);

            train.emit(&RAMP).await;

            info!("{} done", name);

            // Dropped here rather than released: `release` hands back an erased pin, and the next
            // phase wants the concrete one to name its channel again.
            drop(train);

            Timer::after(Duration::from_millis(5)).await;
        }

        Timer::after(Duration::from_millis(20)).await;
    }
}
