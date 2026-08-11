//! The brown-out supervisor's warning levels, and that a deep sleep does not quietly lose one.
//!
//! `BOR0` is the reset threshold every device boots at. Above it sit three warning levels which
//! interrupt instead of resetting, so an application can hear that the supply is sagging before it
//! goes. This arms each of them, checks the supervisor really took it, then sleeps and checks the
//! level survived the round trip.
//!
//! **The sleep is the point.** On this part `PMCU_ERR_03` says the warning levels do not work in
//! STANDBY, so the HAL drops to `BOR0` on the way in and puts the level back on the way out. If that
//! restore ever stops happening, an application gets its warning level for exactly one sleep and
//! then silently runs without it for ever — which nothing else would report.
//!
//! No wiring, no instrument.
//!
//! # What this cannot check
//!
//! **That the supervisor actually fires.** That needs the supply taken below a threshold, from a
//! bench supply, while the part is in STANDBY — which is the erratum's own failure and the only way
//! to see it. What runs here is the bookkeeping around it.
//!
//! Nor can it see the level *during* the sleep, the core not running to be asked.
//!
//! # Reading a failure at `arm`
//!
//! The thresholds are supply voltages: on this part `BOR1-` is about 2.15 V, `BOR2-` 2.74 V and
//! `BOR3-` 2.94 V. Asking for a level the supply is already under leaves the supervisor at `BOR0`
//! and is reported as an error rather than a silent no-op — so a board running near 2.9 V failing to
//! arm `Bor3` is the check working, not the driver failing.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::sysctl::{self, BorThreshold};
use embassy_time::Timer;
use panic_halt as _;

/// Long enough that `Config::min_sleep` lets the executor into a deep-sleep mode rather than a plain
/// `WFI` — the threshold is a few ticks and this is thousands.
const SLEEP: embassy_time::Duration = embassy_time::Duration::from_millis(200);

/// Sleeps to take. One would prove the restore happens; several prove it keeps happening, which is
/// the shape a bug in the save half would take.
const ROUNDS: u32 = 5;

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let _p = embassy_mspm0::init(Default::default());

    let mut fails = 0;

    // Every MSPM0 boots at the reset level, whatever the application later asks for.
    match sysctl::bor_threshold() {
        BorThreshold::Bor0 => info!("boot level: Bor0 ok"),
        other => {
            error!("boot level: {}, want Bor0", other);
            fails += 1;
        }
    }

    // Each warning level in turn, lowest first. All three sit below a 3.3 V supply, so all three
    // should arm; the readback inside `set_bor_threshold` is what says the two-register sequence and
    // its settling time are right.
    for threshold in [BorThreshold::Bor1, BorThreshold::Bor2, BorThreshold::Bor3] {
        match sysctl::set_bor_threshold(threshold) {
            Ok(()) => info!("arm {}: ok", threshold),
            Err(active) => {
                error!("arm {}: supervisor stayed at {}", threshold, active);
                fails += 1;
            }
        }
    }

    // Left at the highest, which is the one a sagging supply reaches first and so the one whose loss
    // would matter soonest.
    let armed = BorThreshold::Bor3;

    for round in 0..ROUNDS {
        Timer::after(SLEEP).await;

        match sysctl::bor_threshold() {
            level if level == armed => info!("round {}: still {} ok", round, level),
            level => {
                error!(
                    "round {}: {} after a sleep, want {} — the level was not put back",
                    round, level, armed
                );
                fails += 1;

                // Put it back by hand, so the remaining rounds test the restore again rather than
                // reporting the same loss repeatedly.
                let _ = sysctl::set_bor_threshold(armed);
            }
        }
    }

    // Back to the reset level, which is where an application that no longer wants the warning should
    // leave it — a warning level costs a little current and disarms itself when it fires.
    match sysctl::set_bor_threshold(BorThreshold::Bor0) {
        Ok(()) => info!("back to Bor0: ok"),
        Err(active) => {
            error!("back to Bor0: supervisor stayed at {}", active);
            fails += 1;
        }
    }

    if fails == 0 {
        info!("bor: all ok");
    } else {
        error!("bor: {} failed", fails);
    }
}
