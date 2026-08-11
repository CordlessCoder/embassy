//! Clock witness: toggles PA16 with a fixed cycle delay, so the half-period reveals the core rate.
//! Scratch, not for committing. Identical source on both HAL arms.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mspm0::Config;
use embassy_mspm0::gpio::{Level, Output};
use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Config::default());
    let mut out = Output::new(p.PA16, Level::Low);

    loop {
        out.toggle();
        cortex_m::asm::delay(32_000);
    }
}
