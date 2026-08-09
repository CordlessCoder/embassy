//! A hardware-triggered DMA transfer, started and then cancelled. Nothing wired.
//!
//! Two defects lived on this path because nothing in the tree ever configured a channel with anything
//! but the software trigger, and neither needs the trigger to actually fire:
//!
//! 1. **`start` asserted `DMAREQ` whatever the trigger source.** `DMAREQ` is the software trigger
//!    request (SLAU846 table 5-29, `DMATSEL` 0), so a hardware-triggered channel ran one transfer
//!    immediately instead of waiting for its event. In block mode that "completes" with whatever the
//!    source held at the time.
//! 2. **`request_pause` did not clear `DMAEN`**, which is both what stops a transfer (SLAU846 5.2.6)
//!    and what `is_running` reports. So `Drop` spun on `while is_running() {}` forever.
//!
//! The trigger chosen below is a real source that nothing in this binary publishes to, so it never
//! fires. That is the point: a channel configured to wait, which then must neither run nor wedge.
//!
//! # The pass
//!
//! ```text
//! phase 1: destination untouched after start   (defect 1)
//! phase 2: drop returned                       (defect 2)
//! phase 3: software trigger still completes    (regression)
//! done
//! ```
//!
//! **A hang is the failure.** Phase 2 prints before it drops, so a run that stops after
//! `phase 2: dropping` is the second defect still present rather than a board that never started.

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

/// ADC0 Publisher 2, table 8-3 of the device datasheet. Any non-zero select would do — this binary
/// never starts an ADC, so the source stays silent and the channel waits forever.
const HARDWARE_TRIGGER: u8 = 23;

const WORDS: usize = 64;

static mut DEST: [u32; WORDS] = [0; WORDS];

/// Rounds of the cancel test. The hang was deterministic, but a cancel that leaves the channel in a bad
/// state might only show on the round after.
const ROUNDS: u32 = 200;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Config::default());

    let mut channel = Channel::new(p.DMA_CH0, Irqs);

    let mut ran_early = 0u32;
    let mut dropped = 0u32;

    for round in 0..ROUNDS {
        let mut source = 0x5A5A_0000u32 | round;

        let dest = &raw mut DEST;

        // SAFETY: the destination is reached only through this pointer while the transfer owns it, and
        // the transfer is dropped below, before `source` goes out of scope.
        unsafe {
            (&mut *dest)[0] = 0;
            (&mut *dest)[WORDS - 1] = 0;
        }

        let mut options = TransferOptions::default();
        options.mode = TransferMode::Block;

        // SAFETY: as above.
        let transfer = unwrap!(unsafe { channel.read(HARDWARE_TRIGGER, &raw mut source, &mut *dest, options) });

        // Phase 1. The trigger has not fired and cannot have, so nothing may have moved.
        //
        // SAFETY: reading what the DMA would have written. If the transfer did run this races it, which
        // is exactly the defect being tested for and is reported rather than relied on.
        let moved = unsafe { (&*dest)[0] != 0 || (&*dest)[WORDS - 1] != 0 };
        if moved {
            ran_early += 1;
            if ran_early == 1 {
                error!(
                    "round {}: destination written with no trigger -- start asserted DMAREQ",
                    round
                );
            }
        }

        if round == 0 {
            info!("phase 2: dropping a transfer that never triggered");
        }

        // Phase 2. With `DMAEN` left set this never returns.
        drop(transfer);
        dropped += 1;
    }

    if ran_early == 0 {
        info!("phase 1: ok -- {} starts, destination untouched every time", ROUNDS);
    } else {
        error!("phase 1: FAIL -- {} of {} ran without a trigger", ran_early, ROUNDS);
    }

    info!("phase 2: ok -- {} cancels returned", dropped);

    // Phase 3. The software path still has to work; the fix conditions `DMAREQ` on the trigger source
    // and getting that backwards would break exactly this.
    let mut source = 0xC3C3_1234u32;
    let dest = &raw mut DEST;

    // SAFETY: awaited to completion before `source` goes out of scope.
    let transfer = unwrap!(unsafe {
        let mut options = TransferOptions::default();
        options.mode = TransferMode::Block;
        channel.read(Transfer::SOFTWARE_TRIGGER, &raw mut source, &mut *dest, options)
    });

    transfer.await;

    // SAFETY: the transfer is over.
    let copied = unsafe { (&*dest)[WORDS - 1] };
    if copied == 0xC3C3_1234 {
        info!("phase 3: ok -- software trigger still completes");
    } else {
        error!("phase 3: FAIL -- last word {:#x}, expected 0xC3C31234", copied);
    }

    info!("done");

    loop {
        embassy_time::Timer::after_secs(60).await;
    }
}
