//! Window watchdog petted from its own task, leaving the main task free to do work.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::wwdt::{Config, Timeout, Watchdog};
use embassy_time::Timer;
use panic_halt as _;

#[embassy_executor::task]
async fn watchdog_task(wdt: Watchdog<'static>) -> ! {
    wdt.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut config = Config::default();
    config.timeout = Timeout::Sec1;
    // Keep the count where it was over a deep sleep, so sleeping longer than the timeout is allowed.
    config.stop_in_sleep = true;

    let wdt = Watchdog::new(p.WWDT0, config);
    info!("petting every {} us", wdt.config().pet_interval_micros());
    spawner.spawn(watchdog_task(wdt).unwrap());

    let mut led = Output::new(p.PA0, Level::High);
    led.set_inversion(true);

    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}
