#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{self, Input, Level, Output, Pull};
use embassy_mspm0::{Config, bind_group_interrupts};
use panic_halt as _;

// Every port has to be bound, because which one a pin belongs to is not known until run time.
bind_group_interrupts!(struct Irqs {
    GPIOA => gpio::InterruptHandler;
    GPIOB => gpio::InterruptHandler;
    GPIOC => gpio::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Hello world!");

    let p = embassy_mspm0::init(Config::default());

    let led1 = p.PA0;
    let s2 = p.PB8;

    let mut led1 = Output::new(led1, Level::Low);

    let mut s2 = Input::new_async(s2, Pull::Up, Irqs);

    // led1 is active low
    led1.set_high();

    loop {
        s2.wait_for_falling_edge().await;

        info!("Switch 2 was pressed");

        led1.toggle();
    }
}
