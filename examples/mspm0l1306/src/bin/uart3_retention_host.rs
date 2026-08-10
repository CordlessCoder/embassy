//! Other side of the G3507 `uart3_retention` check.
//!
//! Sends a byte to wake the G3507 out of deep sleep, then waits for the reply its `UART3` sends after
//! waking. A reply means the PD1 instance came back configured without the driver touching it; silence
//! means it did not. Reports over defmt, since the G3507 cannot log while it deep-sleeps.
//!
//! Wiring, with both boards sharing a ground:
//!
//! | L1306        | G3507             |
//! |--------------|-------------------|
//! | `PA10` (TX)  | `PB7`  (UART1 RX) |
//! | `PA1`  (RX)  | `PB2`  (UART3 TX) |

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_mspm0::sysctl::clock;
use embassy_mspm0::uart::{self, Baud, BufferedUart, ClockSel, Config};
use embassy_mspm0::{bind_interrupts, peripherals};
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use panic_halt as _;

bind_interrupts!(
    struct Irqs {
        UART1 => uart::BufferedInterruptHandler<peripherals::UART1>;
    }
);

/// The reply is 6 bytes at 9600 baud, about 6 ms, and a wake costs microseconds. Kept well under the
/// time driver's own ~1 Hz wake so a pass cannot be explained by the G3507 having woken on its own.
const REPLY_TIMEOUT: Duration = Duration::from_millis(100);

/// The UART runs from MFCLK under the default clock tree, so the baud divider is solved here
/// rather than searched for on the device.
const BAUD: Baud = match Baud::solve(ClockSel::MfClk, clock::RESET_SETUP.clocks().mfclk, 9600) {
    Some(baud) => baud,
    None => core::panic!("this baud rate is not reachable from MFCLK"),
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Hello world!");

    let p = embassy_mspm0::init(Default::default());

    let config = Config::default().with_baud(BAUD);

    let mut tx_buf = [0u8; 32];
    let mut rx_buf = [0u8; 32];
    let mut uart = unwrap!(BufferedUart::new(
        p.UART1,
        p.PA10,
        p.PA1,
        Irqs,
        &mut tx_buf,
        &mut rx_buf,
        config
    ));

    let mut round: u8 = 0;

    loop {
        // Give the G3507 time to get back to sleep between rounds, so every round tests a fresh wake.
        Timer::after_secs(2).await;

        let probe = b'a' + (round % 26);
        round = round.wrapping_add(1);

        unwrap!(uart.write_all(&[probe]).await);

        match read_reply(&mut uart).await {
            Some(echo) if echo == probe => info!("ok: UART3 still configured, echoed {}", probe as char),
            Some(echo) => error!("echoed {} instead of {}", echo as char, probe as char),
            None => error!("no reply within {} ms", REPLY_TIMEOUT.as_millis()),
        }
    }
}

/// Wait for a `woke<byte>` marker and return the echoed byte, or `None` if none arrives in time.
///
/// Resynchronises rather than reading a fixed frame. The G3507 stops driving its TX pin whenever deep
/// sleep unpowers PD1, so the line glitches on every sleep and wake and the receiver picks up junk
/// frames around the real reply. Locking onto the marker skips them.
async fn read_reply(uart: &mut BufferedUart<'_>) -> Option<u8> {
    const MARKER: &[u8] = b"woke";

    let deadline = Timer::after(REPLY_TIMEOUT);
    let mut matched = 0;

    let read = async {
        loop {
            let mut byte = [0u8; 1];

            if uart.read_exact(&mut byte).await.is_err() {
                // A glitch that lands mid-frame shows up as a framing error. Keep listening.
                matched = 0;
                continue;
            }

            if matched == MARKER.len() {
                return byte[0];
            }

            matched = if byte[0] == MARKER[matched] {
                matched + 1
            } else if byte[0] == MARKER[0] {
                1
            } else {
                0
            };
        }
    };

    match select(read, deadline).await {
        Either::First(echo) => Some(echo),
        Either::Second(()) => None,
    }
}
