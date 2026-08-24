//! Pulse trains of differing widths, emitted back to back on one pin.
//!
//! **One analyser channel on BoosterPack 36 (PA10), and a ground.** Nothing else is wired, and
//! nothing else on the board drives the pin.
//!
//! TIMG4 is the only instance on this device with both shadow registers, so it is the only one that
//! can do this. The counter ticks at 1 MHz, so every width below is in microseconds.
//!
//! Three trains repeat with a long idle gap between them, chosen so an off-by-one in the driver's
//! element counting is visible rather than subtle:
//!
//! - a five-element ramp, 10 through 50 us high with a 20 us gap each — the order and the count are
//!   both readable off one capture;
//! - a single 80 us pulse, which is the shortest train the driver accepts and the one where no
//!   second element is ever queued;
//! - a two-element pair, 15 then 60, where the second element is the last one queued before the
//!   train ends.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::gpio::{Level, Pull};
use embassy_mspm0::peripherals::TIMG4;
use embassy_mspm0::tim::pulse_train::{Config as TrainConfig, InterruptHandler, Pulse, PulseTrain};
use embassy_time::{Duration, Timer};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    TIMG4 => InterruptHandler<TIMG4>;
});

const RAMP: [Pulse; 5] = [
    Pulse { high: 10, low: 20 },
    Pulse { high: 20, low: 20 },
    Pulse { high: 30, low: 20 },
    Pulse { high: 40, low: 20 },
    Pulse { high: 50, low: 20 },
];

const SINGLE: [Pulse; 1] = [Pulse { high: 80, low: 20 }];

const PAIR: [Pulse; 2] = [Pulse { high: 15, low: 20 }, Pulse { high: 60, low: 20 }];

/// Long enough that abandoning it lands inside an element rather than on a boundary.
const CANCEL: [Pulse; 5] = [Pulse { high: 100, low: 100 }; 5];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut buffer = [Pulse { high: 1, low: 1 }; CANCEL.len()];

    let mut train = PulseTrain::new(
        p.TIMG4,
        p.PA10,
        Pull::None,
        &mut buffer,
        Irqs,
        TrainConfig {
            // 32 MHz / 8 / 4, so one tick is one microsecond.
            divider: 8,
            prescaler: 4,
            idle: Level::Low,
            ..Default::default()
        },
    );

    info!("emitting at {} Hz", train.timer().tick_frequency());

    loop {
        for (name, pulses) in [("ramp", &RAMP[..]), ("single", &SINGLE[..]), ("pair", &PAIR[..])] {
            train.emit(pulses).await;

            info!("{} done, {} elements", name, pulses.len());

            Timer::after(Duration::from_millis(2)).await;
        }

        // A train dropped part way through. Five 200 us elements is 1 ms of work, abandoned after
        // 300 us, so the pin is cut mid-element rather than at a boundary — the case where a driver
        // that parks lazily leaves a stuck level or a runt behind.
        match select(train.emit(&CANCEL), Timer::after(Duration::from_micros(300))).await {
            Either::First(()) => warn!("the cancelled train finished, which it should not have"),
            Either::Second(()) => info!("cancelled part way"),
        }

        Timer::after(Duration::from_millis(10)).await;
    }
}
