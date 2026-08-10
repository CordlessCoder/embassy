//! Measures what the buffered UART's receiver can actually sustain, against a sender that does not slow
//! down for it.
//!
//! Other side of the U575 `usart_flood` example. This is the measurement `uart_overrun` cannot make and
//! `uart_crosscheck` makes only indirectly: **the pace is set entirely at the other end**. A loopback is
//! throttled by the receiver's own starved CPU and an echo by whatever this end manages to send, so
//! neither can overrun anything. Here the line runs at `BAUD_RATE` whatever happens here, and every byte
//! not collected is a real loss.
//!
//! # What it answers
//!
//! **Answered:** the per-byte interrupt cost is what bounds the receive rate, and the FIFO trigger level
//! is the lever. With the FIFOs off the handler runs once per byte at 17.7 µs against a 21.4 µs byte period
//! and 2.9% of a 460800 stream is lost with no transmit load at all; at half-full it holds to 921600.
//! A continuous transmitter costs almost nothing on top, so the ceiling was never about being
//! bidirectional. `TESTING.md` C10 has the table.
//!
//! It also settles what [`Config::fifo`]'s threshold is worth where it matters. `uart_overrun` could not
//! tell, because loopback is clean either way once the transmitter yields, and it understated the answer
//! by more than an order of magnitude — 9% against about 2x.
//!
//! # Wiring
//!
//! Rig E, both directions, sharing a ground:
//!
//! | MSPM0G3507     | U575               | Signal                     |
//! |----------------|--------------------|----------------------------|
//! | `PB7` (RX)     | `PA2` (TX), pin A1 | the stream under test      |
//! | `PB6` (TX)     | `PA3` (RX), pin A0 | only used by [`DUPLEX`]    |
//! | J1.20 or J3.22 | any Arduino GND    | ground                     |
//!
//! # The two shapes
//!
//! [`DUPLEX`] picks which question is being asked, and both need the same wiring:
//!
//! - **`false`** — receive only. Isolates the cost of receiving. `PB6` is still configured, so it idles
//!   high rather than floating; an undriven pin looks like a start bit to the far end.
//! - **`true`** — receive while transmitting continuously. This is `uart_crosscheck`'s load, except that
//!   the far end never slows down, so the two together can be measured against the same receiver in
//!   isolation. The U575 end never reads what arrives; its own overruns are not the measurement.
//!
//! Running both is what separates "RX costs too much" from "RX and TX together cost too much".
//!
//! # Analyser
//!
//! The same eight leads as C9, with **one moved**: D1 comes off `PB13`, which this binary does not drive,
//! and goes on the receive net instead — that is the signal under test here and C9 had nothing on it. Put
//! it on MSPM0 J2.14 or the U575's A1, whichever is less crowded; it is one net.
//!
//! | Channel | Pin | Header | Carries |
//! |---|---|---|---|
//! | **D0** | `PB6` | J2.13 | this end transmitting — flat unless [`DUPLEX`] |
//! | **D1** | `PB7` | J2.14 / U575 A1 | **the incoming stream**: its bit period is the far end's real rate |
//! | **D2** | `PB0` | J2.12 | `Marker::UartReadPoll` — one edge per `try_read` |
//! | **D3** | `PB1` | J4.39 | one edge per report line, for correlating a capture with the log |
//! | **D4** | `PB2` | J1.9 | `Marker::UartHandler` — the handler, bracketed |
//! | **D5** | `PB3` | J1.10 | `Marker::UartRxMask` — where it masks its own RX interrupt |
//! | **D6** | `PB20` | J4.36 | `Marker::UartWritePoll` — one edge per `write_inner` poll |
//! | **D7** | `PB4` | J4.40 | `Marker::UartTxWake` — one edge per transmit-waker wake |
//!
//! D1 against D4 is the measurement that matters: bytes arriving against handler entries. If the handler
//! stops keeping up, the two diverge and the gap is what is being lost.
//!
//! # Running it
//!
//! `BAUD_RATE` is a `const` at both ends and they must match, so a sweep means reflashing both. Flash the
//! U575 **detached** and stream RTT from this end only — probe-rs leaves the STM32 halted when it exits,
//! which would stop the flood mid-measurement:
//!
//! ```text
//! cd examples/stm32u575 && cargo build --release --bin usart_flood
//! probe-rs download --chip STM32U575ZITxQ --probe <u575> target/.../usart_flood
//! probe-rs reset    --chip STM32U575ZITxQ --probe <u575>
//! cd ../mspm0g3507-uart && cargo run --release --bin uart_rx_ceiling
//! ```
//!
//! # Reading the output
//!
//! One line a second. `lost` is derived from the sequence and is the measurement; `reported dropped` is
//! the driver's own count of bytes it knows it lost, which should agree with `lost` once the driver is
//! honest about it — the two disagreeing by orders of magnitude is what an under-reporting flag looks
//! like. It is what the
//! driver reported. A rate is clean when both stay zero across several lines — the first line always
//! shows a partial block and a gap, since this end starts mid-stream.
//!
//! `rate` is the delivered byte rate. Against the nominal `BAUD_RATE / 10` it says how much of the line
//! is arriving: a receiver that is merely losing bytes still reports a rate near nominal, while one that
//! has stopped reports near zero.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_mspm0::gpio::{Level, Output, Port};
use embassy_mspm0::probe::{self, Marker};
use embassy_mspm0::sysctl::{LowPowerInstance, PowerDomain, clock};
use embassy_mspm0::uart::{Baud, BufferedUart, BufferedUartRx, BufferedUartTx, ClockSel, Config, Error, FifoThreshold};
use embassy_mspm0::{bind_interrupts, peripherals, uart};
use embassy_time::{Duration, Instant, with_timeout};
use embedded_io_async::{Read, Write};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    UART1 => uart::BufferedInterruptHandler<peripherals::UART1>;
});

const CLOCKS: clock::Clocks = clock::RESET_SETUP.clocks();
const UART1_DOMAIN: PowerDomain = <peripherals::UART1 as LowPowerInstance>::SLEEP.power_domain;

/// Must match the U575 side.
const BAUD_RATE: u32 = 460_800;

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

/// Whether to enable the hardware FIFO. Worth running both ways: this is the rig where it should matter,
/// and the one where `uart_overrun` could not tell.
const FIFO: bool = true;

/// Transmit continuously as well as receiving. See the module doc.
const DUPLEX: bool = false;

/// Bytes per transmitted block when [`DUPLEX`] is set.
const TX_BLOCK: usize = 32;

/// How often to report.
///
/// The report line is not free: it costs handler time, and about a third of the residual loss at 921600
/// is this. Measured — at ten seconds the same rate loses ~155 bytes a second against ~230 at one. One
/// second is kept because it makes a sweep quick and matches the recorded tables; lengthen it when a
/// figure needs to be tight, and subtract nothing, because the rest of the loss is real.
const REPORT: Duration = Duration::from_secs(1);

/// Read chunk. Smaller than the ring so a full ring cannot mask the RX interrupt and manufacture a loss
/// that says nothing about the driver's rate.
const CHUNK: usize = 64;

/// In `.bss`, zeroed by the startup code rather than by a run-time clear.
static mut TX_BUF: [u8; 64] = [0; 64];
static mut RX_BUF: [u8; 256] = [0; 256];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // Held for the program's life: dropping one would return its pin to the reset state while the HAL is
    // still writing to it. Same pins as C9, so no lead moves for these.
    let _read_poll = Output::new(p.PB0, Level::Low);
    let _handler = Output::new(p.PB2, Level::Low);
    let _rx_mask = Output::new(p.PB3, Level::Low);
    let _write_poll = Output::new(p.PB20, Level::Low);
    let _tx_wake = Output::new(p.PB4, Level::Low);
    let mut report_tick = Output::new(p.PB1, Level::Low);

    probe::arm(Marker::UartReadPoll, Port::PortB, 0);
    probe::arm(Marker::UartHandler, Port::PortB, 2);
    probe::arm(Marker::UartRxMask, Port::PortB, 3);
    probe::arm(Marker::UartWritePoll, Port::PortB, 20);
    probe::arm(Marker::UartTxWake, Port::PortB, 4);

    let mut config = Config::default().with_baud(BAUD);
    config.clock_source = ClockSel::BusClk;
    config.fifo = if FIFO { Some(FifoThreshold::Half) } else { None };

    // Both pins are claimed even when only receiving, so `PB6` idles high instead of floating. An
    // undriven pin reads as a start bit at the far end, which `TESTING.md` C3 lost a run to.
    // SAFETY: single-threaded, and each is borrowed exactly once, here.
    let (tx_buf, rx_buf) = unsafe { (&mut *(&raw mut TX_BUF), &mut *(&raw mut RX_BUF)) };
    let mut uart = unwrap!(BufferedUart::new(p.UART1, p.PB6, p.PB7, Irqs, tx_buf, rx_buf, config,));

    info!(
        "receiving at {} baud, fifo {}, duplex {}, uart domain {:?}",
        BAUD_RATE, FIFO, DUPLEX, UART1_DOMAIN
    );

    let (mut tx, mut rx) = uart.split_ref();

    if DUPLEX {
        join(flood(&mut tx), measure(&mut rx, &mut report_tick)).await.1
    } else {
        measure(&mut rx, &mut report_tick).await
    }
}

/// Transmit continuously, so the receive measurement runs under the load of a transmitter as well.
///
/// The far end never reads this; it exists to occupy the same interrupt the receiver needs.
async fn flood(tx: &mut BufferedUartTx<'_>) -> ! {
    let mut next = 0u8;

    loop {
        let mut block = [0u8; TX_BLOCK];
        for byte in block.iter_mut() {
            *byte = next;
            next = next.wrapping_add(1);
        }

        if let Err(e) = tx.write_all(&block).await {
            error!("write failed: {:?}", e);
        }
    }
}

/// Read forever, checking the counter, and report once per [`REPORT`].
async fn measure(rx: &mut BufferedUartRx<'_>, tick: &mut Output<'_>) -> ! {
    let mut buf = [0u8; CHUNK];
    let mut expected: Option<u8> = None;

    let mut received = 0u32;
    let mut lost = 0u32;
    let mut gaps = 0u32;
    let mut dropped = 0u32;
    let mut faults = 0u32;
    let mut other = 0u32;
    let mut since = Instant::now();

    loop {
        match with_timeout(REPORT, rx.read(&mut buf)).await {
            Ok(Ok(n)) => {
                for &byte in &buf[..n] {
                    match expected {
                        // Starting mid-stream, so the first byte defines the phase rather than a gap.
                        None => {}
                        Some(want) if byte == want => {}
                        Some(want) => {
                            gaps += 1;
                            lost += byte.wrapping_sub(want) as u32;
                        }
                    }
                    expected = Some(byte.wrapping_add(1));
                }
                received += n as u32;
            }
            // Says only that it happened, and not reachable at all under a sustained overrun: the error
            // surfaces solely on a read that finds the buffer empty. `take_dropped` below is the figure.
            Ok(Err(Error::Overrun)) => {}
            Ok(Err(e)) => {
                other += 1;
                error!("read failed: {:?}", e);
            }
            // Nothing at all for a whole reporting period. Either the far end is not running or the
            // receiver has stopped, which the counters below tell apart.
            Err(_) => warn!("no data for {} ms", REPORT.as_millis()),
        }

        let elapsed = since.elapsed();
        if elapsed >= REPORT {
            tick.toggle();

            // Once per line, not once per read: it takes a critical section, and calling it on the read
            // path costs about 2.5% of the delivered rate at 1 Mbaud — the example measuring itself.
            dropped += rx.take_dropped() as u32;

            // Same reasoning as above: a critical section apiece, so both are read once per line rather
            // than on the read path. Noise, framing, parity and break — the faults that cost their own
            // byte and nothing further, where `dropped` counts bytes lost to an overrun.
            faults += rx.take_faults() as u32;

            let rate = (received as u64 * 1000 / elapsed.as_millis().max(1)) as u32;
            info!(
                "{} B/s ({} nominal): {} received, {} lost in {} gaps, {} reported dropped, {} line faults, {} other{}",
                rate,
                BAUD_RATE / 10,
                received,
                lost,
                gaps,
                dropped,
                faults,
                other,
                if lost == 0 && dropped == 0 && faults == 0 && other == 0 {
                    " -- clean"
                } else {
                    ""
                },
            );

            received = 0;
            lost = 0;
            gaps = 0;
            dropped = 0;
            faults = 0;
            other = 0;
            since = Instant::now();
        }
    }
}
