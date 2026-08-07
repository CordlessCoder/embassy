//! Transmits a rolling counter continuously, so an MSPM0 receiver can be measured against a sender that
//! does not slow down for it.
//!
//! Other side of the MSPM0 `uart_rx_ceiling` example. `usart_echo` cannot answer what this answers: an
//! echo is paced by whatever the MSPM0 manages to send, and a loopback on the MSPM0 is paced by its own
//! starved CPU. Both are self-throttling, so neither can overrun the receiver. **This end never reads and
//! never waits**, so the line runs at exactly `BAUD` regardless of what the far end is coping with, and
//! whatever the MSPM0 fails to collect is a real loss.
//!
//! Wiring, sharing a ground. Only the second row is needed; the first is left in the table because the
//! same rig serves `usart_echo`.
//!
//! | U575               | MSPM0G3507   | Signal              |
//! |--------------------|--------------|---------------------|
//! | `PA3` (RX), pin A0 | `PB6` (TX)   | unused here         |
//! | `PA2` (TX), pin A1 | `PB7` (RX)   | **the stream**      |
//!
//! USART2's alternate mapping, not the `PD5`/`PD6` one: those reach only the ST morpho headers, which
//! ship unsoldered. `PA2`/`PA3` are A1/A0 on the Arduino header, which is populated.
//!
//! `BAUD` has to match the other side, and both are `const`, so a sweep means reflashing both ends per
//! rate. Flash and reset this end **detached** — `probe-rs download` then `probe-rs reset` — and stream
//! RTT from the MSPM0 only; probe-rs leaves this board halted when it exits, which would stop the flood
//! mid-measurement.
//!
//! The counter wraps every 256 bytes, which is what lets the far end resynchronise on a loss and report
//! how many bytes went missing rather than one error repeated for the rest of the run.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale};
use embassy_stm32::usart::{BufferedUart, Config};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embedded_io_async::Write;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

/// Must match the MSPM0 side.
const BAUD: u32 = 460_800;

/// Bytes per write. Large enough that this end is never the bottleneck.
const BLOCK: usize = 64;

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

    info!("flooding at {} baud", BAUD);

    let mut next = 0u8;

    loop {
        let mut block = [0u8; BLOCK];
        for byte in block.iter_mut() {
            *byte = next;
            next = next.wrapping_add(1);
        }

        if let Err(e) = usart.write_all(&block).await {
            error!("write error: {}", e);
        }
    }
}
