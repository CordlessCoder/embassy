//! Unbuffered async UART echo.
//!
//! Waits on the receive FIFO rather than draining it into a software ring, so the driver carries no
//! buffer of its own. The FIFO is four entries deep, which is the whole of the slack: a task that
//! stays away longer than four character times overruns, and the byte that caused it comes back as
//! [`Error::Overrun`]. Use `BufferedUart` where the consumer cannot promise to come back that fast.
//!
//! This uses the virtual COM port provided on the LP-MSPM0L1306 board.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::sysctl::clock;
use embassy_mspm0::uart::{Baud, ClockSel, Config, InterruptHandler, Uart};
use embassy_mspm0::{bind_interrupts, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    UART0 => InterruptHandler<peripherals::UART0>;
});

/// The UART runs from MFCLK under the default clock tree, so its rate is a constant and the
/// baud divider can be solved now rather than searched for on the device.
const BAUD: Baud = match Baud::solve(ClockSel::MfClk, clock::RESET_SETUP.clocks().mfclk, 115200) {
    Some(baud) => baud,
    None => core::panic!("this baud rate is not reachable from MFCLK"),
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let config = Config::default().with_baud(BAUD);
    let mut uart = unwrap!(Uart::new(p.UART0, p.PA9, p.PA8, Irqs, config));

    unwrap!(uart.write(b"Hello Embassy World!\r\n").await);
    unwrap!(uart.flush().await);
    info!("wrote hello, starting echo");

    let mut buf = [0u8; 1];

    loop {
        match uart.read(&mut buf).await {
            Ok(()) => unwrap!(uart.write(&buf).await),
            Err(err) => warn!("receive error: {}", err),
        }
    }
}
