//! Does a deep sleep after an awaited ADC conversion still wake?
//!
//! An older bring-up on an L1305 found that an async read followed by a deep sleep read correctly and
//! then **never woke**. The busy-poll that removed it was deliberately taken out again when the ADC
//! was parked on its interrupt, so the condition is reachable once more and nothing has tested it.
//!
//! Walks four sleep depths, shallowest first, so a failure names the shallowest depth that breaks.
//!
//! **Run it detached.** `probe-rs` holds the device out of deep sleep while RTT is attached, which
//! turns this into a measurement of nothing. It logs six lines at startup and then goes silent for
//! exactly that reason — `defmt-rtt` blocks here when nobody is draining it, and a stream would hang
//! the binary in the logger and look like the fault being hunted.
//!
//! # Reading it, with no analyser and no leads to move
//!
//! Progress goes into six words of RAM, read over SWD after the run:
//!
//! ```bash
//! arm-none-eabi-nm target/thumbv6m-none-eabi/release/adc_sleep | grep -i PROGRESS
//! probe-rs read b32 <address of ADC_SLEEP_PROGRESS> 6
//! ```
//!
//! **They live in `.uninit`, and that is the point.** Connecting a probe to a sleeping MSPM0 makes
//! probe-rs run its PWR-AP recovery, which resets the device — so the act of reading destroys an
//! ordinary `.bss` counter and the reader sees a fresh run rather than the one being measured. These
//! survive a reset, and the boot counter says how many times it happened.
//!
//! In order: magic, boots, deepest phase reached, furthest round in that phase, stage, done.
//!
//! - **`done` is `0xD0E`** if every phase ever completed. That is the pass, whatever the boot count.
//! - **`stage`** is 0 while a conversion is in flight and 1 while asleep, so a run that never gets
//!   past a phase says which half it stopped in. Stage 1 is the fault this exists for; stage 0 is a
//!   waker problem and wants `adc_waker` instead.
//!
//! **It drives no pins**, so it is safe on a board whose wiring is unknown. `PA15` is sampled and an
//! ADC input is high-impedance, so nothing is driven there either — a floating pin converts perfectly
//! well, and what is being checked is that the read completes and the sleep returns, not what it
//! reads.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::{self as adc, Adc, AdcChannel, Conversion};
use embassy_mspm0::sysctl::{SleepLevel, WakeGuard};
use embassy_mspm0::{bind_interrupts, peripherals};
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    ADC0 => adc::InterruptHandler<peripherals::ADC0>;
});

/// Depths to walk, shallowest first. The guard names the shallowest level to *block*, so the mode
/// actually entered is the one below it; `None` blocks nothing and reaches the bottom.
const PHASES: [(Option<SleepLevel>, &str); 4] = [
    (Some(SleepLevel::Stop0), "WFI only"),
    (Some(SleepLevel::Stop1), "STOP0"),
    (Some(SleepLevel::Standby0), "STOP2"),
    (None, "STANDBY1"),
];

/// Sleeps per phase. The old finding hung on the first one, but a race wants repetition.
const ROUNDS: u32 = 200;

/// Long enough to clear `Config::min_sleep` by a wide margin — that gate is a few ticks.
const SLEEP_MS: u64 = 20;

/// High-water marks, in a section the startup code does not clear.
///
/// Kept out of `.bss` because reading them is what resets the device: probe-rs recovers the debug port
/// on a sleeping part by resetting it, which would zero anything ordinary before it could be read.
/// Written with `Relaxed` ordering because nothing on the device reads them — the only reader is a
/// probe attaching afterwards, which sees memory rather than the program's view of it.
#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit")]
static ADC_SLEEP_PROGRESS: [AtomicU32; 6] = [const { AtomicU32::new(0) }; 6];

/// Index into [`ADC_SLEEP_PROGRESS`].
const MAGIC: usize = 0;
const BOOTS: usize = 1;
const PHASE: usize = 2;
const ROUND: usize = 3;
const STAGE: usize = 4;
const DONE: usize = 5;

/// Says the array holds this program's marks rather than whatever was in SRAM at power-up.
const MAGIC_VALUE: u32 = 0xADC5_1EEB;

/// [`STAGE`] values: which half of a round is in progress.
const STAGE_CONVERTING: u32 = 0;
const STAGE_SLEEPING: u32 = 1;

fn record(index: usize, value: u32) {
    ADC_SLEEP_PROGRESS[index].store(value, Ordering::Relaxed);
}

/// Keep the furthest point reached across every run since power-up.
fn record_max(index: usize, value: u32) {
    if ADC_SLEEP_PROGRESS[index].load(Ordering::Relaxed) < value {
        record(index, value);
    }
}

/// Zero the marks on a cold boot, and count the resets on a warm one.
fn start_run() {
    if ADC_SLEEP_PROGRESS[MAGIC].load(Ordering::Relaxed) == MAGIC_VALUE {
        record(BOOTS, ADC_SLEEP_PROGRESS[BOOTS].load(Ordering::Relaxed) + 1);
        return;
    }

    for word in &ADC_SLEEP_PROGRESS {
        word.store(0, Ordering::Relaxed);
    }
    record(MAGIC, MAGIC_VALUE);
    record(BOOTS, 1);
}

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) {
    let mut p = embassy_mspm0::init(Default::default());

    start_run();

    let mut adc = Adc::new_async(p.ADC0, Irqs, Default::default());

    // Every line this binary will print. After this it is silent, so that a detached run cannot block
    // in the logger.
    info!("adc_sleep: {} phases, {} rounds each", PHASES.len(), ROUNDS);
    for (_, name) in PHASES {
        info!("  phase: {}", name);
    }
    info!("detach before believing any of it");

    for (phase, (block, _)) in PHASES.iter().enumerate() {
        record_max(PHASE, phase as u32);
        // Held for the whole phase. Dropping it at the end of the iteration is what deepens the next.
        let _guard = block.map(WakeGuard::new);

        for round in 0..ROUNDS {
            record_max(ROUND, round);

            // A hang here is the waker, not the sleep.
            record(STAGE, STAGE_CONVERTING);
            let mut channel = p.PA15.reborrow_adc();
            let _ = adc.irq_read(&mut channel, Conversion::default()).await;

            // And this is the half under suspicion.
            record(STAGE, STAGE_SLEEPING);
            Timer::after_millis(SLEEP_MS).await;
        }
    }

    record(DONE, 0x0D0E);
    // Awake from here on, so a probe connecting to read the result does not have to recover the
    // debug port and reset the very thing it came to read.

    loop {
        cortex_m::asm::nop();
    }
}
