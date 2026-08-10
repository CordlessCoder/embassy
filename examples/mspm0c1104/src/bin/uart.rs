//! Example of using blocking uart
//!
//! This uses the virtual COM port provided on the LP-MSPM0C1104 board.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::sysctl::clock;
use embassy_mspm0::uart::{Baud, ClockSel, Config, Uart};
use panic_halt as _;

/// The UART runs from MFCLK under the default clock tree, so its rate is a constant and the
/// baud divider can be solved now rather than searched for on the device.
const BAUD: Baud = match Baud::solve(ClockSel::MfClk, clock::RESET_SETUP.clocks().mfclk, 115200) {
    Some(baud) => baud,
    None => core::panic!("this baud rate is not reachable from MFCLK"),
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Hello world!");

    let p = embassy_mspm0::init(Default::default());

    let instance = p.UART0;
    let tx = p.PA27;
    let rx = p.PA26;

    let config = Config::default().with_baud(BAUD);
    let mut uart = unwrap!(Uart::new_blocking(instance, rx, tx, config));

    unwrap!(uart.begin_blocking_write().write(b"Hello Embassy World!\r\n"));
    info!("wrote Hello, starting echo");

    let mut buf = [0u8; 1];

    loop {
        unwrap!(uart.blocking_read(&mut buf));
        unwrap!(uart.begin_blocking_write().write(&buf));
    }
}
