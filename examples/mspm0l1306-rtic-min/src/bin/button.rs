//! An `embassy-mspm0` async driver on RTIC, with no embassy runtime crate in the build.
//!
//! `Input<Async>` parks on the GPIO interrupt through the HAL's own waiter list, and RTIC's dispatcher
//! polls the task back. Nothing in that path wants a time source or an executor, so the only embassy
//! crates here are the HAL and the no-std support crates it is built from.
//!
//! Press `S2` (`PA14`); `LED1` (`PA0`) toggles and each edge is logged.
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0, I2C1])]
mod app {
    use defmt::info;
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
        let mut n = 0u32;
        loop {
            button.wait_for_falling_edge().await;
            led.toggle();
            n += 1;
            info!("edge {}", n);
        }
    }
}
