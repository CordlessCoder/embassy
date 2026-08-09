//! An RTIC application that sleeps between events instead of spinning in `wfi`.
//!
//! RTIC has no idle policy beyond whatever `#[idle]` does, and a bare `wfi` there only ever reaches
//! the shallowest mode. [`low_power::sleep`](embassy_mspm0::low_power::sleep) is the same call the
//! low-power executor makes on idle, so an RTIC application that makes it gets the same sleep depths —
//! the deepest level the active `WakeGuard`s permit, capped by
//! [`Config::min_sleep`](embassy_mspm0::Config::min_sleep) when the next wake is too close to be worth
//! it.
//!
//! It needs `low-power` on the HAL, and `critical-section` as a direct dependency for the token.
//!
//! # `#[idle]` and nowhere else
//!
//! `sleep` is `unsafe` for one reason: it must run in **thread mode**. A `WFI` inside a handler is
//! only woken by an interrupt of higher priority than the one running, so at the lowest priority it
//! never returns. RTIC's software tasks run in their dispatcher's handler, which makes `#[idle]` the
//! only valid caller in an RTIC application — the same call from a `#[task]` compiles and hangs.
//!
//! # Measuring it
//!
//! **Detached.** `probe-rs` holds the device out of deep sleep for as long as RTT is attached, which
//! turns a current measurement into a measurement of nothing. Flash, disconnect, then read the supply
//! across the J101 3V3 jumper. **Unfit the LED1 jumper too** — a lit LED is milliamps and swamps the
//! difference being looked for.
//!
//! | Pin | Signal |
//! |---|---|
//! | `PA0` | LED1, toggled once per wake as a liveness check |

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0, I2C1])]
mod app {
    use defmt::info;
    use embassy_mspm0::gpio::{Level, Output};
    use embassy_mspm0::{Config, low_power};
    use embassy_time::Timer;

    /// Long enough that the sleep dominates the time spent awake, and long enough to watch.
    const PERIOD_SECS: u64 = 2;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let p = embassy_mspm0::init(Config::default());

        info!("sleeping between blinks");

        blink::spawn(Output::new(p.PA0, Level::Low)).map_err(|_| ()).unwrap();

        (Shared {}, Local {})
    }

    /// Thread mode, below every task — the one place `sleep` may be called from.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            critical_section::with(|cs| unsafe { low_power::sleep(cs) });
        }
    }

    #[task(priority = 1)]
    async fn blink(_cx: blink::Context, mut led: Output<'static>) {
        loop {
            led.toggle();
            Timer::after_secs(PERIOD_SECS).await;
        }
    }
}
