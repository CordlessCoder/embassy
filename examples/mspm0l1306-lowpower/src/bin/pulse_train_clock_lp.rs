//! The same train, the same two clocks, with `low-power` on.
//!
//! **One analyser channel on BoosterPack 12 (PA18), and a ground.** Nothing else is wired.
//!
//! `TESTING.md` C42, and the control is `examples/mspm0l1306`'s `pulse_train_clock`, which is this
//! measurement with the feature off and measures clean on both clocks.
//!
//! The two clocks are not symmetric under this feature, and the asymmetry is the driver's own.
//! `PowerDomain::floor_to_keep_running` puts a PD0 timer clocked at 4 MHz or less at `Stop2`, so a
//! train on MFCLK leaves `Stop1` reachable and the executor may sleep there **while the counter is
//! running**. The same timer on a 32 MHz bus clock floors at `Stop0` and blocks every STOP level, so
//! it never sleeps at all. Only one of the two phases below can therefore sleep mid-train.
//!
//! Widths are fitted the same way: the high is fixed and the low sweeps eightfold, so a rate comes
//! off the low phases and the high is read against it.
//!
//! Measured: BusClk −0.021% with a −1.3 ns residue, MFCLK **+0.726% with a +207.5 ns residue** and
//! six times the jitter. Setting `BLOCK_STOP1` holds the part out of STOP1 and nothing else, and
//! brings MFCLK back to +0.018% — so the level is what does it, and the driver's own sleep floor is
//! what lets a train on MFCLK reach it.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::gpio::Pull;
use embassy_mspm0::peripherals::TIMG4;
use embassy_mspm0::tim::ClockSel;
use embassy_mspm0::sysctl::{SleepLevel, WakeGuard};
use embassy_mspm0::tim::pulse_train::{Config as TrainConfig, InterruptHandler, Pulse, PulseTrain};
use embassy_time::{Duration, Timer};
use panic_halt as _;

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

const TRAINS: usize = 20;

/// Whether to hold the part out of STOP1. The measurement wants both runs.
const BLOCK_STOP1: bool = false;

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
        .with_divider(4)
        .with_prescaler(1)
}

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // Hold the executor out of STOP1 and deeper, leaving STOP0 and `WFI`. `SleepLevel::Stop1` is
    // documented as limiting SYSOSC to 4 MHz, and the TRM takes MFCLK straight off SYSOSC rather
    // than through the divider once SYSOSC is itself at 4 MHz — so if the depth is what shortens
    // the widths, this is the one level that has to be blocked to see them come back.
    let _guard = BLOCK_STOP1.then(|| WakeGuard::new(SleepLevel::Stop1));

    let mut timer = p.TIMG4;
    let mut pin = p.PA18;

    loop {
        // Trailing gaps differ per phase so a capture can be cut into phases by gap length alone.
        // Every gap inside a phase is 500 us, so anything longer is a boundary.
        for (name, config, gap) in [("busclk", bus(), 3), ("mfclk", mfclk(), 10)] {
            let mut train = PulseTrain::new(timer.reborrow(), pin.reborrow(), Pull::None, Irqs, config);

            info!("{} sweep at {} Hz", name, train.timer().tick_frequency());

            for _ in 0..TRAINS {
                train.emit(&SWEEP).await;

                // Long enough that the executor reaches a sleep between trains as well as inside
                // one, so a phase that can sleep does so on both paths.
                Timer::after(Duration::from_micros(500)).await;
            }

            Timer::after(Duration::from_millis(gap)).await;
        }

        Timer::after(Duration::from_millis(50)).await;
    }
}
