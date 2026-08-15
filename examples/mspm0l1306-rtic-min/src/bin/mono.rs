//! An RTIC monotonic on an MSPM0 timer, sleeping between deadlines.
//!
//! `Mono` is what an RTIC application uses instead of `embassy-time`, and this crate is where that
//! claim is checkable: `cargo tree` shows no `embassy-time`, `embassy-time-driver` or
//! `embassy-time-queue-utils` in the build. The timer runs from LFCLK, which survives every
//! deep-sleep mode, so `idle` takes the device down between blinks and the compare brings it back.
//!
//! `Config::interrupts` is `External` because nothing here binds a group source, so `init` has no
//! group line to enable.
//!
//! Wiring: none. `LED1` is the LaunchPad's own on `PA0`.

#![no_std]
#![no_main]

use panic_halt as _;

embassy_mspm0::rtic_monotonic!(Mono, TIMG0);

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0, I2C1])]
mod app {
    use embassy_mspm0::fugit::ExtU64;
    use embassy_mspm0::gpio::{Level, Output};
    use embassy_mspm0::rtic_time::Monotonic;
    use embassy_mspm0::{Config, InterruptPolicy};

    use crate::Mono;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let mut config = Config::default();
        config.interrupts = InterruptPolicy::External;

        let p = embassy_mspm0::init(config);

        Mono::start(p.TIMG0);

        let mut led = Output::new(p.PA0, Level::Low);
        // LED1 is active low.
        led.set_high();

        blink::spawn(led).map_err(|_| ()).unwrap();

        (Shared {}, Local {})
    }

    /// Thread mode, below every task — the one place a sleep may be entered from.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            critical_section::with(embassy_mspm0::idle);
        }
    }

    #[task(priority = 1)]
    async fn blink(_: blink::Context, mut led: Output<'static>) {
        loop {
            Mono::delay(500.millis()).await;
            led.toggle();
        }
    }
}
