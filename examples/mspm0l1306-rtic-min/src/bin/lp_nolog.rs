//! Deep sleep from RTIC's `#[idle]` with no time driver: `low-power` on its own.
//!
//! Without a time driver nothing schedules a wake, so the duration gate has nothing to weigh and the
//! `WakeGuard`s alone decide how deep the chip goes. That is the configuration an interrupt-driven
//! application wants — it sleeps until a peripheral wakes it, and pays for no timer.
//!
//! `sleep` may only be called from `#[idle]`. It must run in thread mode, and RTIC's software tasks
//! run in their dispatcher's handler, where a `WFI` at the lowest priority never returns.
//!
//! Press `S2` (`PA14`); `LED1` (`PA0`) toggles. Between edges the chip is in a deep-sleep mode — to
//! measure that, run detached, since an attached RTT session holds it out of deep sleep.
#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0, I2C1])]
mod app {
    use embassy_mspm0::gpio::{self, Input, Level, Output, Pull};
    use embassy_mspm0::mode::Async;
    use embassy_mspm0::{Config, bind_group_interrupts};

    bind_group_interrupts!(struct Irqs {
        GPIOA => gpio::InterruptHandler;
    });

    #[shared]
    struct Shared {}
    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let p = embassy_mspm0::init(Config::default());
        let mut led = Output::new(p.PA0, Level::Low);
        led.set_high();

        watch::spawn(Input::new_async(p.PA14, Irqs, Pull::Up), led)
            .map_err(|_| ())
            .unwrap();

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            critical_section::with(embassy_mspm0::idle);
        }
    }

    #[task(priority = 1)]
    async fn watch(_cx: watch::Context, mut button: Input<'static, Async>, mut led: Output<'static>) {
        loop {
            button.wait_for_falling_edge().await;
            led.toggle();
        }
    }
}
