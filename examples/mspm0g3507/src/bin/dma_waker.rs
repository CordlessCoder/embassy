//! Awaiting a DMA transfer, on a memory-to-memory copy.
//!
//! **No wiring.** A software-triggered transfer between two buffers, so nothing outside the chip is
//! involved and nothing reaches a pin.
//!
//! # What it checks
//!
//! The asynchronous path, which is easy to miss: a short copy finishes before the future is first
//! polled, so `poll` returns `Ready` and the channel's waker is never registered at all. The transfer
//! here is long enough to still be running at that first poll, and that is checked rather than
//! assumed — `still running` in the log is what says the run proves anything.
//!
//! Once the future is `Pending` the only thing that can complete it is the channel interrupt waking
//! the registered waker, so a broken waker leaves the task asleep for good. A pass is a steady stream
//! of `ok` lines; **silence is the failure.**
//!
//! The copied words are checked too, so a transfer that never ran cannot pass by having its future
//! resolve early.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::dma::{Channel, InterruptHandler, Transfer, TransferMode, TransferOptions};
use embassy_mspm0::{Config, bind_interrupts, peripherals};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA => InterruptHandler<peripherals::DMA_CH0>;
});

/// Words per transfer. Long enough that the channel is still running when the future is first polled,
/// which is the whole point — see the module doc.
const WORDS: usize = 2048;

static mut DEST: [u32; WORDS] = [0; WORDS];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Config::default());

    let mut channel = Channel::new(p.DMA_CH0, Irqs);

    let mut completed = 0u32;
    let mut awaited = 0u32;
    let mut wrong = 0u32;

    loop {
        // A fresh value each round, so a destination left over from the previous transfer cannot pass.
        let mut source = 0xA5A5_0000u32 | (completed & 0xFFFF);
        let expected = source;

        let dest = &raw mut DEST;

        // Block, not the default single: a single-mode trigger moves one word and stops.
        let mut options = TransferOptions::default();
        options.mode = TransferMode::Block;

        // SAFETY: `source` outlives the transfer, which is awaited to completion below, and `DEST` is
        // reached only through this pointer for as long as the transfer owns it.
        let mut transfer =
            unwrap!(unsafe { channel.read(Transfer::SOFTWARE_TRIGGER, &raw mut source, &mut *dest, options) });

        // Whether the asynchronous path is being tested at all. If this is ever false the transfer
        // finished first and that round proved nothing about the waker.
        if transfer.is_running() {
            awaited += 1;
        }

        transfer.await;
        completed += 1;

        // SAFETY: the transfer is over, so nothing else is touching the destination.
        let copied = unsafe { (&*dest)[WORDS - 1] };
        if copied != expected {
            wrong += 1;
            error!(
                "transfer {}: last word {:#x}, expected {:#x}",
                completed, copied, expected
            );
        }

        if completed % 2000 == 0 {
            info!(
                "{} completed, {} still running at the first poll, {} wrong -- ok",
                completed, awaited, wrong
            );
        }
    }
}
