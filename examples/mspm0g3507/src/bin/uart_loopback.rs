//! A blocking write and read against the UART's own internal loopback.
//!
//! **Nothing is wired.** `Config::loop_back_enable` ties the transmitter to the receiver inside the
//! peripheral, so this needs no peer and no pins — which is the point: it exercises the transmit path
//! on any board with a UART.
//!
//! # What it is for
//!
//! `blocking_write` used to wait for the transmit FIFO to go *empty* before handing over each byte, so
//! one byte was in flight at a time whatever depth `Config::fifo` asked for. It waits only while the
//! FIFO is full now, which is what the buffered driver has always done. This checks the bytes still
//! arrive, and arrive in order, when the FIFO is filled to its depth rather than one slot at a time.
//!
//! Both FIFO settings are covered, because the two status bits change meaning with `CTL0.FEN`: with the
//! FIFOs off they describe the single holding register instead, and the same loop has to be right for
//! both.
//!
//! A run reports per case and ends with a total. There is nothing to read on an analyser — the loopback
//! never reaches a pin.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::uart::{Config, FifoThreshold, Uart};
use embassy_time::Timer;
use panic_halt as _;

/// Long enough to fill a four-deep FIFO several times over.
const BURST: usize = 16;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut fail = 0u32;
    let mut sent = [0u8; BURST];
    for (i, byte) in sent.iter_mut().enumerate() {
        // Not a plain ramp: a value that changes in both nibbles catches a byte delivered twice as
        // readily as one delivered out of order.
        *byte = (i as u8).wrapping_mul(37).wrapping_add(11);
    }

    // One instance, reconfigured between cases: taking two would need two sets of pins.
    let mut config = Config::default();
    config.loop_back_enable = true;
    let mut uart = unwrap!(Uart::new_blocking(p.UART0, p.PA11, p.PA10, config));

    for fifo in [Some(FifoThreshold::Half), Some(FifoThreshold::Full), None] {
        let mut config = Config::default();
        config.loop_back_enable = true;
        config.fifo = fifo;

        if let Err(e) = uart.set_config(&config) {
            fail += 1;
            error!("{}: could not configure: {}", Debug2Format(&fifo), e);
            continue;
        }

        // The receiver is exactly as deep as the transmitter, and a loopback fills both at once, so a
        // burst longer than that depth overruns whatever the transmitter does. With the FIFOs off the
        // depth is the single holding register, which is why this is not a constant.
        let depth = if fifo.is_some() { 4 } else { 1 };

        let mut got = [0u8; BURST];
        let mut ok = true;

        for chunk in 0..BURST / depth {
            let at = chunk * depth;

            if let Err(e) = uart.begin_blocking_write().write(&sent[at..at + depth]) {
                fail += 1;
                error!("{}: write: {}", Debug2Format(&fifo), e);
                ok = false;
                break;
            }

            if let Err(e) = uart.blocking_read(&mut got[at..at + depth]) {
                fail += 1;
                error!("{}: read: {}", Debug2Format(&fifo), e);
                ok = false;
                break;
            }
        }

        if !ok {
            continue;
        }

        match got.iter().zip(sent.iter()).position(|(a, b)| a != b) {
            None => info!("{}: {} bytes, in order", Debug2Format(&fifo), BURST),
            Some(i) => {
                fail += 1;
                error!(
                    "{}: byte {} came back {}, sent {}",
                    Debug2Format(&fifo),
                    i,
                    got[i],
                    sent[i]
                );
            }
        }
    }

    if fail == 0 {
        info!("uart_loopback: ok");
    } else {
        error!("uart_loopback: {} failures", fail);
    }

    loop {
        Timer::after_millis(1000).await;
    }
}
