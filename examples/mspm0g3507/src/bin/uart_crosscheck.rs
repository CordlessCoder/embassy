//! Sends byte patterns to a UART on another vendor's HAL and checks what comes back.
//!
//! Other side of the U575 `usart_echo` example, which echoes every byte and nothing else. A loopback on
//! one board proves the divider agrees with itself; an echo from a different part with an independent
//! clock proves the bit period is what the standard says. That is what `uart::Baud`'s solved divider
//! needs, and it is not something a self-test can show.
//!
//! Wiring, with both boards sharing a ground. `PB6` and `PB7` are UART1, not the back-channel UART, so
//! nothing has to come off the isolation block:
//!
//! | MSPM0G3507   | U575               | Signal              |
//! |--------------|--------------------|---------------------|
//! | `PB6` (TX)   | `PA3` (RX), pin A0 | MSPM0 out, U575 in  |
//! | `PB7` (RX)   | `PA2` (TX), pin A1 | U575 out, MSPM0 in  |
//!
//! # What it sends
//!
//! Four fixed patterns and a rolling block, checked byte for byte:
//!
//! | Block | What it stresses |
//! |-------|------------------|
//! | `0x00` x8 | the longest run of dominant bits — worst case for a divider that drifts |
//! | `0xFF` x8 | idle-level bytes, so a framing error shows as a missing byte rather than a wrong one |
//! | `0x55`/`0xAA` | an edge every bit, the worst case for sampling in the wrong half of a bit |
//! | walking one | each bit position alone |
//! | 32 rolling bytes | every value 0-255 across eight passes, so a single stuck bit cannot hide |
//!
//! # What to check
//!
//! - **Mismatches, not errors.** A wrong bit period usually gives whole wrong bytes or nothing at all
//!   rather than an error, because a UART cannot tell a mis-sampled byte from a real one. The count is
//!   the measurement.
//! - **`BAUD` on both sides must match**, and it is a `const` in both files. The rates worth walking are
//!   9600, 115200, 921600 and 1000000: the last two are where a divider that rounds the wrong way stops
//!   being jitter and starts being framing errors.
//! - **The bit period on an analyser**, against `1 / BAUD`. `Baud::solve` works from the clock tree it is
//!   given, so this is the check that the tree it was given is the one running.
//! - A timeout means bytes went missing entirely: either the wiring, or a rate this end cannot reach from
//!   its clock — `Baud::solve` returning `None` is a build error, but a divider it rounds badly is not.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::sysctl::{LowPowerInstance, PowerDomain, clock};
use embassy_mspm0::uart::{Baud, BufferedUart, ClockSel, Config};
use embassy_mspm0::{bind_interrupts, peripherals, uart};
use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    UART1 => uart::BufferedInterruptHandler<peripherals::UART1>;
});

/// Must match the U575 side.
const BAUD_RATE: u32 = 115_200;

const CLOCKS: clock::Clocks = clock::RESET_SETUP.clocks();
const UART1_DOMAIN: PowerDomain = <peripherals::UART1 as LowPowerInstance>::SLEEP.power_domain;

/// Solved against the bus clock rather than MFCLK: at 32 MHz the divider has the resolution to hit the
/// higher rates, where 4 MHz does not.
const BAUD: Baud = match Baud::solve(
    ClockSel::BusClk,
    ClockSel::BusClk.frequency(&CLOCKS, UART1_DOMAIN),
    BAUD_RATE,
) {
    Some(baud) => baud,
    None => core::panic!("this baud rate is not reachable from the bus clock"),
};

/// Fixed patterns, worst cases first.
const PATTERNS: &[[u8; 8]] = &[
    [0x00; 8],
    [0xFF; 8],
    [0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA],
    [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80],
];

/// Bytes in the rolling block. Eight passes of 32 covers every value.
const ROLLING: usize = 32;

/// How long to wait for an echo. A 32 byte block at 9600 baud is 33 ms each way, so this covers the
/// slowest rate worth trying with room to spare.
const ECHO_TIMEOUT: Duration = Duration::from_millis(500);

/// How long the line has to stay silent before [`drain`] calls it quiet. Two byte times at 9600 baud is
/// 2 ms, so this is generous at every rate.
const QUIET: Duration = Duration::from_millis(20);

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut config = Config::default().with_baud(BAUD);
    config.clock_source = ClockSel::BusClk;

    let mut tx_buf = [0u8; 64];
    let mut rx_buf = [0u8; 64];
    let mut uart = unwrap!(BufferedUart::new(
        p.UART1,
        p.PB6,
        p.PB7,
        Irqs,
        &mut tx_buf,
        &mut rx_buf,
        config,
    ));

    drain(&mut uart).await;

    info!("checking echoes at {} baud", BAUD_RATE);

    let mut pass = 0u32;
    let mut mismatches = 0u32;
    let mut timeouts = 0u32;

    loop {
        for pattern in PATTERNS {
            let mut echo = [0u8; 8];
            match round_trip(&mut uart, pattern, &mut echo).await {
                Ok(()) if echo == *pattern => {}
                Ok(()) => {
                    mismatches += 1;
                    error!("sent {:?}, got back {:?}", pattern, echo);
                }
                Err(()) => timeouts += 1,
            }
        }

        // A different 32 bytes every pass, so eight passes cover the whole range.
        let base = (pass % 8) as u8 * ROLLING as u8;
        let mut sent = [0u8; ROLLING];
        for (offset, byte) in sent.iter_mut().enumerate() {
            *byte = base.wrapping_add(offset as u8);
        }

        let mut echo = [0u8; ROLLING];
        match round_trip(&mut uart, &sent, &mut echo).await {
            Ok(()) if echo == sent => {}
            Ok(()) => {
                mismatches += 1;
                let wrong = echo.iter().zip(sent.iter()).filter(|(a, b)| a != b).count();
                error!("rolling block from {}: {} of {} bytes wrong", base, wrong, ROLLING);
            }
            Err(()) => timeouts += 1,
        }

        pass += 1;
        info!(
            "pass {}: {} mismatched blocks, {} timeouts so far",
            pass, mismatches, timeouts
        );
    }
}

/// Discard whatever is already in flight, so block one starts from a quiet line.
///
/// `PB6` is not driven until the line above configures it, and it floats low until then. The other end
/// reads that release as a start bit, eight zero bits and a stop bit — a legitimate `0x00` frame, which
/// it echoes. One unasked-for byte offsets every block after it by one, which looks like corruption
/// rather than what it is.
async fn drain(uart: &mut BufferedUart<'_>) {
    let mut scratch = [0u8; 32];
    let mut dropped = 0usize;

    while let Ok(Ok(n)) = with_timeout(QUIET, uart.read(&mut scratch)).await {
        dropped += n;
    }

    if dropped != 0 {
        info!("discarded {} bytes left over from startup", dropped);
    }
}

/// Send `out`, read back as many bytes as were sent, or give up after [`ECHO_TIMEOUT`].
async fn round_trip(uart: &mut BufferedUart<'_>, out: &[u8], back: &mut [u8]) -> Result<(), ()> {
    if let Err(e) = uart.write_all(out).await {
        error!("write failed: {:?}", e);
        return Err(());
    }

    match with_timeout(ECHO_TIMEOUT, uart.read_exact(back)).await {
        Ok(Ok(())) => Ok(()),
        // `ReadExactError` has no `defmt::Format`, so report the two cases by hand.
        Ok(Err(embedded_io_async::ReadExactError::UnexpectedEof)) => {
            error!("the echo ended early");
            Err(())
        }
        Ok(Err(embedded_io_async::ReadExactError::Other(e))) => {
            error!("read failed: {:?}", e);
            Err(())
        }
        Err(_) => {
            error!("no echo within {} ms", ECHO_TIMEOUT.as_millis());
            Err(())
        }
    }
}
