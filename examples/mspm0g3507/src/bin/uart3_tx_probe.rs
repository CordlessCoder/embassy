//! Control for the `uart3_retention` check: transmits on `UART3` without ever sleeping.
//!
//! If the L1306 running `uart3_retention_host` sees these lines, the `PB2` -> `PA1` link is good and any
//! silence in the real test is about deep sleep. If it sees nothing here either, the problem is the
//! wiring or the pins, not retention.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::sysctl::clock;
use embassy_mspm0::uart::{Baud, ClockSel, Config, UartTx};
use embassy_time::Timer;
use panic_halt as _;

/// The UART runs from MFCLK under the default clock tree, so the baud divider is solved here
/// rather than searched for on the device.
const BAUD: Baud = match Baud::solve(ClockSel::MfClk, clock::RESET_SETUP.clocks().mfclk, 9600) {
    Some(baud) => baud,
    None => core::panic!("this baud rate is not reachable from MFCLK"),
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let config = Config::default().with_baud(BAUD);

    let mut uart = unwrap!(UartTx::new_blocking(p.UART3, p.PB2, config));
    info!("transmitting on UART3/PB2 at 9600");

    loop {
        unwrap!(uart.blocking_write(b"boot\n"));
        Timer::after_millis(500).await;
    }
}
