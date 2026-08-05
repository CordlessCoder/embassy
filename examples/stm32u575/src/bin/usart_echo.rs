//! Echoes every byte back, so an MSPM0 can check its UART against a known-good one.
//!
//! Other side of the MSPM0 `uart_crosscheck` example, which sends patterns and verifies what comes
//! back. Everything interesting happens over there — this end only has to be trustworthy, which is the
//! point of running it on a different HAL and a different silicon vendor.
//!
//! Wiring, with both boards sharing a ground:
//!
//! | U575              | MSPM0G3507   | Signal              |
//! |-------------------|--------------|---------------------|
//! | `PA3` (RX), pin A0 | `PB6` (TX)  | MSPM0 out, U575 in  |
//! | `PA2` (TX), pin A1 | `PB7` (RX)  | U575 out, MSPM0 in  |
//!
//! USART2's alternate mapping, not the `PD5`/`PD6` one: those reach only the ST morpho headers, which
//! ship unsoldered. `PA2`/`PA3` are A1/A0 on the Arduino header, which is populated.
//!
//! `BAUD` has to match the other side. 115200 is the default there too; the rates worth walking are
//! 9600, 115200, 921600 and 1000000, the last two because that is where a divider that rounds the wrong
//! way starts producing framing errors rather than just jitter.
//!
//! The MSPM0's clock tree decides how well it can hit a rate. This end runs from a 160 MHz PLL, which
//! divides exactly enough for any of them, so a mismatch is the other side's.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale};
use embassy_stm32::usart::{BufferedUart, Config};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embedded_io_async::{Read, Write};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

/// Must match the MSPM0 side.
const BAUD: u32 = 115_200;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();

    // 16 MHz HSI multiplied to 160 MHz, so the baud divider is never the reason for a mismatch.
    config.rcc.hsi = true;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul10,
        divp: None,
        divq: None,
        divr: Some(PllDiv::Div1),
    });
    config.rcc.sys = Sysclk::Pll1R;
    config.rcc.voltage_range = VoltageScale::Range1;

    let p = embassy_stm32::init(config);

    let mut uart_config = Config::default();
    uart_config.baudrate = BAUD;

    let mut tx_buf = [0u8; 256];
    let mut rx_buf = [0u8; 256];
    let mut usart = unwrap!(BufferedUart::new(
        p.USART2,
        p.PA3,
        p.PA2,
        &mut tx_buf,
        &mut rx_buf,
        Irqs,
        uart_config,
    ));

    info!("echoing at {} baud", BAUD);

    let mut buf = [0u8; 64];
    let mut echoed = 0u32;

    loop {
        // Echo whatever arrives rather than a fixed block, so the other side chooses the chunking.
        let read = match usart.read(&mut buf).await {
            Ok(read) => read,
            Err(e) => {
                error!("read error: {}", e);
                continue;
            }
        };

        if let Err(e) = usart.write_all(&buf[..read]).await {
            error!("write error: {}", e);
            continue;
        }

        echoed += read as u32;
        debug!("echoed {} bytes, {} total", read, echoed);
    }
}
