//! RTIC layered on top of an ordinary `embassy-mspm0` application.
//!
//! `rt` stays on and the HAL keeps its own vector table, so RTIC gets the whole driver set,
//! `embassy_time::Timer` and the async APIs without any of them being reimplemented. RTIC claims only
//! the interrupts it names — here the two dispatchers its software tasks run on.
//!
//! The dispatchers must be interrupts nothing else uses. `SPI0` and `I2C1` are free on this part as
//! long as the application drives neither, and `time-driver-any` never lands on them.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0, I2C1])]
mod app {
    use defmt::info;
    use embassy_mspm0::gpio::{Level, Output};
    use embassy_mspm0::{Config, Peri, peripherals};
    use embassy_time::Timer;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        info!("Hello world!");

        let p = embassy_mspm0::init(Config::default());
        blink::spawn(p.PA0).map_err(|_| ()).unwrap();

        (Shared {}, Local {})
    }

    #[task(priority = 1)]
    async fn blink(_cx: blink::Context, pin: Peri<'static, peripherals::PA0>) {
        let mut led = Output::new(pin, Level::Low);
        led.set_inversion(true);

        loop {
            Timer::after_millis(500).await;

            info!("Toggle");
            led.toggle();
        }
    }
}
