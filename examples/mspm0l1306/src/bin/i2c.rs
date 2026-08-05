//! This example uses FIFO with polling, and the maximum FIFO size is 8.
//! Refer to async example to handle larger packets.
//!
//! This example controls AD5171 digital potentiometer via I2C with the LP-MSPM0L1306 board.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::i2c::{ClockDiv, ClockSel, Config, I2c, Timing};
use embassy_mspm0::sysctl::clock;
use embassy_time::Timer;
use panic_halt as _;

const ADDRESS: u8 = 0x2c;

/// The default 100 kHz bus off MFCLK, solved here rather than divided for on the device.
const TIMING: Timing = match Timing::solve(&clock::RESET_SETUP.clocks(), ClockSel::MfClk, ClockDiv::DivBy1, 100_000) {
    Some(timing) => timing,
    None => core::panic!("100 kHz is not reachable from MFCLK"),
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let instance = p.I2C0;
    let scl = p.PA1;
    let sda = p.PA0;

    let mut i2c = unwrap!(I2c::new_blocking(
        instance,
        scl,
        sda,
        Config::default().with_timing(TIMING)
    ));

    let mut pot_value: u8 = 0;

    loop {
        let to_write = [0u8, pot_value];

        match i2c.blocking_write(ADDRESS, &to_write) {
            Ok(()) => info!("New potentioemter value: {}", pot_value),
            Err(e) => error!("I2c Error: {:?}", e),
        }

        pot_value += 1;
        // if reached 64th position (max)
        // start over from lowest value
        if pot_value == 64 {
            pot_value = 0;
        }
        Timer::after_millis(500).await;
    }
}
