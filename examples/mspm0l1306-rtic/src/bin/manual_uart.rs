//! An echo served entirely from an RTIC hardware task, with no async machinery anywhere.
//!
//! [`uart::low_level::Uart`] powers the instance up, claims the pins and applies a `Config`, then
//! stops. There is no waker, no future and no interrupt handler from the HAL — the task below is the
//! handler, and RTIC owns the vector.
//!
//! # What it demonstrates
//!
//! **Backpressure without a software buffer.** When the transmit FIFO fills with bytes still
//! arriving, the receive sources are masked and the transmit source unmasked. The unread bytes stay
//! in the receive FIFO and the sender is held off by the hardware, which is the same thing a ring
//! buffer would have been standing in for.
//!
//! **Clearing before draining.** `RIS` is sticky and a byte that lands mid-drain sets it again, so
//! clearing afterwards would discard that flag with the byte still in the FIFO.
//!
//! # Wiring
//!
//! None. PA8 and PA9 are the LaunchPad's XDS110 backchannel UART, so a terminal on the board's
//! virtual COM port at 115200 8N1 sees everything it types come back.
//!
//! An overrun is not handled: type faster than the transmitter drains for long enough and the
//! receiver drops bytes silently. [`Event::Overrun`] is where that would be caught.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0])]
mod app {
    use embassy_mspm0::Config;
    use embassy_mspm0::uart::low_level::{Event, Uart};
    use embassy_mspm0::uart::{self, Baud, ClockSel};

    /// Solved at compile time, so the divider search never reaches the binary.
    const BAUD: Baud = match Baud::solve(
        ClockSel::BusClk,
        embassy_mspm0::sysctl::clock::RESET_SETUP.clocks().ulpclk,
        115_200,
    ) {
        Some(baud) => baud,
        None => core::panic!("115200 is not reachable from the reset bus clock"),
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        uart: Uart<'static>,
    }

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let p = embassy_mspm0::init(Config::default());

        let mut uart = Uart::new(p.UART0, p.PA8, p.PA9, uart::Config::new().with_baud(BAUD)).unwrap();

        // The NVIC line is already unmasked: RTIC does that for anything a task binds. These are the
        // sources within the peripheral, which nothing else touches.
        uart.enable_interrupt(Event::Rx, true);
        uart.enable_interrupt(Event::RxTimeout, true);

        (Shared {}, Local { uart })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            critical_section::with(|cs| embassy_mspm0::idle(cs));
        }
    }

    #[task(binds = UART0, priority = 2, local = [uart])]
    fn uart_isr(cx: uart_isr::Context) {
        let uart = cx.local.uart;

        if uart.is_pending(Event::Rx) || uart.is_pending(Event::RxTimeout) {
            uart.clear_pending(Event::Rx);
            uart.clear_pending(Event::RxTimeout);

            // Only as far as the transmitter has room. What is left stays in the receive FIFO, still
            // holding `RIS`, and comes back when the transmit source fires below.
            while !uart.is_tx_full() {
                let Some(byte) = uart.try_read() else { break };

                if let Ok(byte) = byte {
                    let _ = uart.try_write(byte);
                }
            }

            if uart.is_tx_full() {
                uart.enable_interrupt(Event::Rx, false);
                uart.enable_interrupt(Event::RxTimeout, false);
                uart.enable_interrupt(Event::Tx, true);
            }
        }

        if uart.is_pending(Event::Tx) {
            uart.clear_pending(Event::Tx);

            uart.enable_interrupt(Event::Tx, false);
            uart.enable_interrupt(Event::Rx, true);
            uart.enable_interrupt(Event::RxTimeout, true);
        }
    }
}
