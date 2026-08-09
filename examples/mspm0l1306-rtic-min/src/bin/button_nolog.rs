//! The same application with no logging: the size floor for an async HAL driver on RTIC.
//!
//! Worth building whenever a size claim is made about this HAL. Logging is the largest single item in
//! any of these binaries by a wide margin — far more than the driver it is used to observe — so a
//! figure taken from a binary that links `defmt-rtt` says more about the logger than the HAL.
//!
//! Press `S2` (`PA14`); `LED1` (`PA0`) toggles. There is nothing else to see, by design.
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

        watch::spawn(Input::new_async(p.PA14, Pull::Up, Irqs), led)
            .map_err(|_| ())
            .unwrap();

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
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
