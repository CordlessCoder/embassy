//! Does the compare-driven edge run late, and does the clock source decide it?
//!
//! **One analyser channel on BoosterPack 12 (PA18), and a ground.** Nothing else is wired.
//!
//! `TESTING.md` C41. A low phase ends at the counter's zero event and a high phase ends at a compare
//! match, which are two different paths through the output generator. If one of them ran late the
//! widths would still be self-consistent, so a train measured on its own cannot show it — what shows
//! it is fitting the rate on the low phases and reading the high against that fit.
//!
//! Phases 1 and 2 differ only in [`ClockSel`]. Both tick at 1 MHz, so every width is microseconds.
//! The high is fixed and the low sweeps eightfold, which is what lets a rate be fitted on the lows
//! alone and the high's residue read off against it.
//!
//! - **offset on MFCLK's high and not on BusClk's** — the clock path, and a caveat for a train run
//!   from MFCLK.
//! - **offset on both** — the compare path, and every `PulseTrain` user has it.
//! - **offset on neither** — the driver is clean and whatever prompted the question is elsewhere.
//!
//! Phases 3 and 4 are the cancellation cases, which have never run. Both cut a train part way
//! through and ask what the pin does *while the driver repairs the output generator's latch*.
//!
//! - **resting high, cut mid-high** — `ODIS` holds the output low before the conditional inversion
//!   (SLAU846E 28.3.32), so an unrepaired driver drives the pin at the opposite of its resting level
//!   for the width of the repair. Pass is a pin that stays high through the cancellation.
//! - **released, cut mid-low** — the pin is pulled up, so undriven reads high and driven reads low.
//!   Pass is a rise at the cancellation. A rise delayed by the repair's width means the line was
//!   still being driven after the caller let go of it.
//!
//! The cancellations land through `embassy-time`, whose tick is about 30 us, so each is placed in
//! the middle of a 200 us phase rather than near its edge.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::gpio::Pull;
use embassy_mspm0::peripherals::TIMG4;
use embassy_mspm0::tim::ClockSel;
use embassy_mspm0::tim::pulse_train::{Config as TrainConfig, Idle, InterruptHandler, Pulse, PulseTrain};
use embassy_time::{Duration, Timer};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    TIMG4 => InterruptHandler<TIMG4>;
});

/// A fixed high against an eightfold low sweep. The fifth element is sacrificial: its low runs into
/// the gap after the train, so it is the one that cannot be measured.
const SWEEP: [Pulse; 5] = [
    Pulse { high: 50, low: 10 },
    Pulse { high: 50, low: 20 },
    Pulse { high: 50, low: 40 },
    Pulse { high: 50, low: 80 },
    Pulse { high: 50, low: 200 },
];

/// Long enough that a cancellation lands inside an element rather than on a boundary, and that
/// `embassy-time`'s 30 us granularity cannot move it into the neighbouring phase.
const CANCEL: [Pulse; 5] = [Pulse { high: 200, low: 200 }; 5];

/// Repeats per clock phase. Four clean lows a train, so this is the sample count per width.
const TRAINS: usize = 20;

/// 32 MHz / 8 / 4 and 4 MHz / 4 / 1 both land on 1 MHz.
const fn bus() -> TrainConfig {
    TrainConfig::new()
        .with_clock(ClockSel::BusClk)
        .with_divider(8)
        .with_prescaler(4)
}

const fn mfclk() -> TrainConfig {
    TrainConfig::new()
        .with_clock(ClockSel::MfClk)
        .with_divider(CONTROL_DIVIDER)
        .with_prescaler(1)
}

/// 4 for a 1 MHz tick, which matches the bus phase and is what the measurement wants.
///
/// **Set to 1 to prove the mux moved.** MFCLK then ticks at 4 MHz and every width comes out a
/// quarter of its nominal; a `ClockSel` that silently did nothing leaves the counter on the 32 MHz
/// bus clock, where the same divider gives an eighth. Two clocks that agree are the expected result
/// here, and they are also what a dead mux looks like.
const CONTROL_DIVIDER: u8 = 4;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut timer = p.TIMG4;
    let mut pin = p.PA18;

    loop {
        // The trailing gap differs per phase so a capture can be cut into phases by gap length
        // alone. Every gap inside a phase is 500 us, so anything longer is a boundary, and no two
        // boundaries are the same length.
        for (name, config, gap) in [("busclk", bus(), 3), ("mfclk", mfclk(), 10)] {
            let mut train = PulseTrain::new(timer.reborrow(), pin.reborrow(), Pull::None, Irqs, config);

            info!("{} sweep at {} Hz", name, train.timer().tick_frequency());

            for _ in 0..TRAINS {
                train.emit(&SWEEP).await;

                // Clear of the longest low in the sweep, so the sacrificial element's low is the
                // only one that merges with it.
                Timer::after(Duration::from_micros(500)).await;
            }

            Timer::after(Duration::from_millis(gap)).await;
        }

        // Resting high and cut inside a high phase, where a repair that drives the pin low is a
        // notch against the resting level rather than something hidden in a gap.
        {
            let mut train = PulseTrain::new(
                timer.reborrow(),
                pin.reborrow(),
                Pull::None,
                Irqs,
                bus().with_idle(Idle::High),
            );

            match select(train.emit(&CANCEL), Timer::after(Duration::from_micros(100))).await {
                Either::First(()) => warn!("the cancelled train finished, which it should not have"),
                Either::Second(()) => info!("cut mid-high, resting high"),
            }

            Timer::after(Duration::from_millis(20)).await;
        }

        // Released between trains, and cut inside a *high* phase. Cutting inside a low phase
        // cannot answer anything: `set_as_disconnected` drops the pull, so a released pin floats,
        // and a floating line reads low exactly like a driven one. Cutting while the pin is high
        // separates them — letting go leaves the line to decay on its own, where a repair that
        // keeps driving it pulls it down at once.
        {
            let mut train = PulseTrain::new(
                timer.reborrow(),
                pin.reborrow(),
                Pull::Up,
                Irqs,
                bus().with_idle(Idle::HighImpedance),
            );

            match select(train.emit(&CANCEL), Timer::after(Duration::from_micros(100))).await {
                Either::First(()) => warn!("the cancelled train finished, which it should not have"),
                Either::Second(()) => info!("cut mid-high, released"),
            }
        }

        Timer::after(Duration::from_millis(50)).await;
    }
}
