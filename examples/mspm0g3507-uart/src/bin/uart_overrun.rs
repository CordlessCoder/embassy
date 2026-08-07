//! Takes the buffered UART's interrupt path apart under a saturated transmitter.
//!
//! Written to chase `uart_crosscheck`'s ceiling just above 460800 baud, and **it cannot answer that** —
//! see "What this cannot measure" below; `uart_rx_ceiling` is the test that did. What it found instead is
//! a bug worth more than the ceiling was:
//!
//! **A `write_all` loop never yielded, so anything joined with it was never polled.**
//! [`BufferedUartTx`]'s write never returned `Pending`, because the handler starves thread mode enough
//! that the transmit ring has always drained by the time the task next runs. `send`'s first poll
//! therefore ran a whole 3 s case synchronously, and the receiver joined with it saw 128 bytes of
//! 100,000 — one RX ring, drained at the end.
//!
//! **Fixed in the driver**, which now yields once per `write` call. This example needs nothing of its
//! own for it; take the yield out of `write_inner` and the bug reproduces exactly, with
//! `128 of 100000` at every rate as its signature.
//!
//! The interrupt cost that causes the starvation is real: **17.7 µs an entry, about one entry per byte**,
//! 70% of the case inside the handler. Raising [`Config::fifo`]'s threshold off one entry looks worth only
//! 9% *here* — but that is this rig lying, not a result. Against a sender that does not wait it is worth
//! about 2x, and it is what sets the receive ceiling. `uart_rx_ceiling` has that measurement.
//!
//! # What this cannot measure
//!
//! **A receive ceiling.** Transmitter and receiver are one peripheral, one baud generator and one starved
//! CPU, so the sender can never outrun the receiver: if the interrupt falls behind, transmission falls
//! behind with it. Clean to 1 Mbaud here says nothing about keeping up with an independent talker, which
//! is what `uart_crosscheck` measures. `uart_rx_ceiling` is the rig that can: the U575 transmits into a
//! silent MSPM0 receiver, which also separates "RX costs too much" from "RX and TX together cost too
//! much". **Never quote a number from this file as a ceiling.**
//!
//! # No wiring, and eight optional leads
//!
//! This runs on one board with nothing attached. `CTL0.LBE` connects the transmitter to the receiver
//! inside the peripheral (SLAU846 24.2.3.18): the RX pin is ignored and the TX pin still toggles, so
//! `PB6` remains capturable. Both directions run at the baud rate through the same peripheral and the
//! same interrupt, which is the load under test — a wire and a second part would add an independent
//! clock, and that is `uart_crosscheck`'s question, not this one.
//!
//! The markers are what the software counters cannot show: an idle line reads the same whether the
//! transmitter stopped or the receiver did.
//!
//! | Channel | Pin | Header | Carries |
//! |---|---|---|---|
//! | **D0** | `PB6` | J2.13 | the stream itself — bit period, and whether it ever stops |
//! | **D1** | `PB13` | J4.35 | high for the length of one case, low between cases |
//! | **D2** | `PB0` | J2.12 | `Marker::UartReadPoll` — one edge per `try_read` |
//! | **D3** | `PB1` | J4.39 | one edge per poll of [`hand_join`], so per task poll |
//! | **D4** | `PB2` | J1.9 | `Marker::UartHandler` — the driver's interrupt handler, whole |
//! | **D5** | `PB3` | J1.10 | `Marker::UartRxMask` — where it masks its own RX interrupt |
//! | **D6** | `PB20` | J4.36 | `Marker::UartWritePoll` — one edge per `write_inner` poll |
//! | **D7** | `PB4` | J4.40 | `Marker::UartTxWake` — one edge per transmit-waker wake |
//!
//! D2, D4 and D5 come from the HAL's `_probe` feature, since the handler is `no_mangle` there and cannot
//! be bracketed from an example. **D4's rate is the throughput measurement.** A receiver that has gone
//! quiet while the stream keeps arriving is either a handler that stopped firing or one that fires
//! continuously and moves nothing, and those two are indistinguishable from the stream alone. D5 says
//! whether the driver masked itself once and stayed masked, or is masking on every entry.
//!
//! **D3, D2 and D6 are the accounting**, and they are what found the bug. Healthy, they run about 1:1:2 —
//! one join poll per `write_inner` poll, two `try_read` calls per join poll. With the yield removed they
//! read **2 : 3125 : 7** for a whole case, which is the starvation: the join is entered twice because
//! `send`'s first poll never returns.
//!
//! The join is written out in this file as [`hand_join`] rather than taken from `embassy_futures` so that
//! D3 exists at all. `embassy_futures::join` behaves identically and is not at fault — it polls every
//! unfinished child on every poll, `&=` and no short-circuit.
//!
//! D7 bounds the whole thing from below: the transmit waker is the driver's only way of getting a blocked
//! task run again, so the task cannot have been polled more often than D7 fires.
//!
//! D2 used to bracket `write_all`, which established that the transmitter applies real backpressure —
//! 3125 calls, held 91% of the case. That question is settled, so the channel was reused. D3 used to
//! carry read errors, which `stats.overruns` reports anyway.
//!
//! `Marker::ExecutorPoll` would be the direct measurement and is deliberately not used: it lives in
//! `embassy_mspm0::executor`, and this example runs on the plain `embassy_executor` one.
//!
//! **Cases are attributed by counting, not by a marker**: D1's pulses run in [`RATES`] order, each
//! rate twice, FIFO off then on. That is the same convention `wake_latency` uses for sleep levels.
//!
//! FIFO "off" here is [`Config::fifo`] set to `None`; "on" is `Some(FifoThreshold::Half)`, which is also
//! the default.
//!
//! # Reading the output
//!
//! One row per rate and FIFO setting. `lost` is derived from the sequence, so it is the measurement;
//! `overruns` is what the driver reported. The two should agree in shape, and a rate with neither is
//! clean.
//!
//! The handler logs `warn!("Overrun error")` from inside itself, which `DEFMT_LOG=info` compiles in and
//! which would make each overrun cause the next. Measured, it never fires — `MIS.OVRERR` stays clear
//! through a whole case — so it is not in these numbers. To rule it out again after a driver change,
//! silence the HAL without silencing this example:
//!
//! ```text
//! DEFMT_LOG=info,embassy_mspm0=error cargo run --release --bin uart_overrun
//! ```

#![no_std]
#![no_main]

use core::cell::Cell;
use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::Poll;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output, Port};
use embassy_mspm0::probe::{self, Marker};
use embassy_mspm0::sysctl::{LowPowerInstance, PowerDomain, clock};
use embassy_mspm0::uart::{Baud, BufferedUart, BufferedUartRx, BufferedUartTx, ClockSel, Config, Error, FifoThreshold};
use embassy_mspm0::{bind_interrupts, peripherals, uart};
use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    UART1 => uart::BufferedInterruptHandler<peripherals::UART1>;
});

const CLOCKS: clock::Clocks = clock::RESET_SETUP.clocks();
const UART1_DOMAIN: PowerDomain = <peripherals::UART1 as LowPowerInstance>::SLEEP.power_domain;

/// Rates to walk. The first is `uart_crosscheck`'s clean ceiling, so a FIFO-off run that fails there
/// means something other than this changed.
const RATES: &[u32] = &[460_800, 500_000, 576_000, 691_200, 921_600, 1_000_000];

/// Bytes per case. Enough that a rate failing once in a few thousand blocks still shows: at 460800
/// this is 2.2 s, and `uart_crosscheck` saw its first overrun at 500000 after about 200 kB.
const BYTES: u32 = 100_000;

/// Written and read in this size. Smaller than the RX ring on purpose — see [`main`].
const BLOCK: usize = 32;

/// How long the receiver waits before calling the stream finished. Generous at every rate here: a
/// block at 460800 is 0.7 ms.
const QUIET: Duration = Duration::from_millis(200);

#[derive(Default)]
struct Stats {
    received: u32,
    /// Sequence discontinuities, however many bytes each swallowed.
    gaps: u32,
    /// Bytes the sequence says went missing. Undercounts a gap of 256 or more, which cannot be told
    /// from no gap at all, so trust it alongside `gaps` rather than on its own.
    lost: u32,
    overruns: u32,
    other: u32,
    /// Quiet periods of [`QUIET`] with the transmitter still running. Non-zero means the receiver
    /// stopped for a reason other than the stream ending.
    stalls: u32,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut uart = p.UART1;
    let mut tx_pin = p.PB6;
    let mut rx_pin = p.PB7;

    let mut case_marker = Output::new(p.PB13, Level::Low);
    let mut join_marker = Output::new(p.PB1, Level::Low);

    // Held for the program's life: dropping one would return its pin to the reset state while the HAL is
    // still writing to it.
    let _read_poll = Output::new(p.PB0, Level::Low);
    let _handler = Output::new(p.PB2, Level::Low);
    let _rx_mask = Output::new(p.PB3, Level::Low);
    let _write_poll = Output::new(p.PB20, Level::Low);
    let _tx_wake = Output::new(p.PB4, Level::Low);

    probe::arm(Marker::UartReadPoll, Port::PortB, 0);
    probe::arm(Marker::UartHandler, Port::PortB, 2);
    probe::arm(Marker::UartRxMask, Port::PortB, 3);
    probe::arm(Marker::UartWritePoll, Port::PortB, 20);
    probe::arm(Marker::UartTxWake, Port::PortB, 4);

    // The RX ring holds four blocks. Anything smaller and a full ring would mask the RX interrupt
    // itself, which produces overruns that say nothing about the FIFO.
    let mut tx_buf = [0u8; 64];
    let mut rx_buf = [0u8; BLOCK * 4];

    info!("{} bytes per case, blocks of {}", BYTES, BLOCK);

    for &rate in RATES {
        let Some(baud) = Baud::solve(ClockSel::BusClk.frequency(&CLOCKS, UART1_DOMAIN), rate) else {
            warn!("{} baud is not reachable from the bus clock, skipping", rate);
            continue;
        };

        for fifo in [false, true] {
            let mut config = Config::default().with_baud(baud);
            config.clock_source = ClockSel::BusClk;
            config.loop_back_enable = true;
            config.fifo = if fifo { Some(FifoThreshold::Half) } else { None };

            let stats = {
                let mut uart = unwrap!(BufferedUart::new(
                    uart.reborrow(),
                    tx_pin.reborrow(),
                    rx_pin.reborrow(),
                    Irqs,
                    &mut tx_buf,
                    &mut rx_buf,
                    config,
                ));

                let (mut tx, mut rx) = uart.split_ref();
                let done = Cell::new(false);

                case_marker.set_high();
                let stats = hand_join(send(&mut tx, &done), receive(&mut rx, &done), &mut join_marker).await;
                case_marker.set_low();

                stats
            };

            info!(
                "{} baud, fifo {}: {} of {} bytes back, {} lost in {} gaps, {} overruns, {} stalls, {} other errors{}",
                rate,
                fifo,
                stats.received,
                BYTES,
                stats.lost,
                stats.gaps,
                stats.overruns,
                stats.stalls,
                stats.other,
                if stats.received == BYTES && stats.overruns == 0 && stats.other == 0 {
                    " -- clean"
                } else {
                    ""
                },
            );
        }
    }

    info!("done");
    loop {
        embassy_time::Timer::after_secs(1).await;
    }
}

/// `join`, written out, with a marker on the poll and one on each child's return.
///
/// `embassy_futures::join` should behave identically — it polls every unfinished child on every poll,
/// with `&=` rather than `&&` so there is no short-circuit. It is written out here because the measured
/// poll counts of the two children disagree by a factor of 450, which that cannot produce, and a hand
/// version is the only way to see the polls rather than infer them.
///
/// `marker` toggles once per poll of this future, so its edge count is the number of task polls the two
/// children were offered.
async fn hand_join<A: Future<Output = ()>, B: Future<Output = Stats>>(a: A, b: B, marker: &mut Output<'_>) -> Stats {
    let mut a = pin!(a);
    let mut b = pin!(b);
    let mut done_a = false;
    let mut out_b: Option<Stats> = None;

    poll_fn(move |cx| {
        marker.toggle();

        if !done_a && a.as_mut().poll(cx).is_ready() {
            done_a = true;
        }

        if out_b.is_none()
            && let Poll::Ready(stats) = b.as_mut().poll(cx)
        {
            out_b = Some(stats);
        }

        match (done_a, out_b.take()) {
            (true, Some(stats)) => Poll::Ready(stats),
            (_, other) => {
                out_b = other;
                Poll::Pending
            }
        }
    })
    .await
}

/// Push [`BYTES`] of a rolling counter, as fast as the ring will take them.
///
/// Blocking on a full TX ring is what keeps the transmitter saturated, so the receiver is under load
/// for the whole case rather than between blocks.
async fn send(tx: &mut BufferedUartTx<'_>, done: &Cell<bool>) {
    let mut next = 0u8;
    let mut sent = 0u32;

    while sent < BYTES {
        let mut block = [0u8; BLOCK];
        for byte in block.iter_mut() {
            *byte = next;
            next = next.wrapping_add(1);
        }

        let result = tx.write_all(&block).await;

        if let Err(e) = result {
            error!("write failed: {:?}", e);
            break;
        }
        sent += BLOCK as u32;
    }

    done.set(true);
}

/// Read until the transmitter is finished and the line has gone quiet, checking the counter.
///
/// A quiet period while the transmitter is still running is counted and read through rather than
/// treated as the end. Giving up on the first one hides the difference between a receiver that drops
/// a byte and one that stops dead, and the first version of this example did exactly that.
///
/// Resynchronising on every byte is the point: [`send`] never stops to be re-framed, so a lost byte
/// costs one gap rather than making every later block read as corrupt. `uart_crosscheck` reports the
/// same loss thousands of times for want of this.
async fn receive(rx: &mut BufferedUartRx<'_>, done: &Cell<bool>) -> Stats {
    let mut stats = Stats::default();
    let mut expected = 0u8;
    let mut buf = [0u8; BLOCK];

    loop {
        match with_timeout(QUIET, rx.read(&mut buf)).await {
            Ok(Ok(n)) => {
                for &byte in &buf[..n] {
                    if byte != expected {
                        stats.gaps += 1;
                        stats.lost += byte.wrapping_sub(expected) as u32;
                        expected = byte;
                    }
                    expected = expected.wrapping_add(1);
                }
                stats.received += n as u32;
            }
            // The driver holds an error back until the buffered bytes ahead of it have been read, so
            // this arrives in sequence rather than at the moment of the overrun.
            Ok(Err(Error::Overrun)) => stats.overruns += 1,
            Ok(Err(e)) => {
                stats.other += 1;
                error!("read failed: {:?}", e);
            }
            Err(_) if done.get() => return stats,
            Err(_) => stats.stalls += 1,
        }
    }
}
