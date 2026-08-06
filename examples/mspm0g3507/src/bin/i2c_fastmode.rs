//! Runs the controller at 400 kHz against a known-good target, and at 100 kHz first for comparison.
//!
//! Everything else on this bench is at 100 kHz. This is the first test above it, and it is two tests in
//! one: the bus rate, and the branch that rate reaches. `Config::resolve` picks the functional clock from
//! the requested speed — MFCLK at or below 200 kHz, `BusClk` above it — and nothing has ever taken the
//! second path. `BusClk` is also the configuration whose 32 MHz functional clock hid `I2C_ERR_13` for
//! months, so the settling window `I2c::settle_after_start` waits out is at its shortest here.
//!
//! **This must not use `Timing::solve`.** A pre-solved timing carries its own clock source and
//! short-circuits `resolve`, which is exactly the code under test. `i2c_crosscheck` does solve its timing,
//! deliberately, and so tests the other thing.
//!
//! # Wiring
//!
//! Rig D, with one difference that decides whether any of this works:
//!
//! | U575  | MSPM0G3507 (`I2C1`) | Signal |
//! |-------|---------------------|--------|
//! | `PB8` | `PB2`               | SCL    |
//! | `PB9` | `PB3`               | SDA    |
//!
//! **Pull-ups of 1.5 kΩ, not the 4.7 kΩ standard mode runs on.** Measured on this rig with an analog
//! capture, 4.7 kΩ gives a 10-90% rise of about 850 ns: inside standard mode's 1000 ns limit, three times
//! outside fast mode's 300 ns. That implies 82 pF of bus capacitance, which needs 1.6 kΩ or lower to meet
//! fast mode. Run `i2c_target_fast` on the U575, not `i2c_target`.
//!
//! # What it reports
//!
//! Each phase runs the same battery — a write, a read, a write-read, then a burst of write-reads — and
//! counts failures by kind. The kinds are the point: with the wire too slow for the rate, an edge missed
//! at the target reads as `NackAddress` or `Arbitration`, not as wrong data. A phase that fails only at
//! 400 kHz, with 100 kHz clean in the same run and on the same wiring, is the bus and not the driver.
//!
//! A pass is both phases reporting `0 failures`. The rate itself is not observable from here — the device
//! cannot time its own SCL — so confirm it on the analyser, where fast mode's SCL low period has to be at
//! least 1.3 µs against standard mode's 4.7 µs.
//!
//! A heartbeat ticks throughout, which is not decoration. The first run of this test stopped after the
//! 400 kHz configuration was accepted and printed nothing more, and the heartbeat is what said the device
//! was still running and the executor still scheduling — so the transfer had completed and nothing had
//! woken the task, rather than the chip having wedged. That was `set_config` leaving the interrupt
//! disabled, since fixed. Any test that can hang is worth pairing with a sign of life.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::i2c::{BusSpeed, Config, Error, I2c, InterruptHandler};
use embassy_mspm0::{bind_interrupts, peripherals};
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C1 => InterruptHandler<peripherals::I2C1>;
});

/// The address the U575 target answers on.
const TARGET_ADDR: u8 = 0x48;

/// What a plain read returns.
const EXPECT_READ: [u8; 2] = [8, 8];

/// What a write-read returns.
const EXPECT_WRITE_READ: [u8; 2] = [9, 9];

/// The U575 answers a write by offering a read as well, which has to time out before it listens again.
const AFTER_WRITE_MS: u64 = 100;

/// Write-reads per battery after the three single transactions. Enough that a marginal edge has to show.
const BURST: u32 = 200;

/// Failures of one battery, split by kind because the kind is what says whether the wire or the driver is
/// at fault.
#[derive(Default)]
struct Failures {
    nack: u32,
    arbitration: u32,
    bus: u32,
    timeout: u32,
    other: u32,
    wrong_data: u32,
}

impl Failures {
    fn count(&mut self, err: Error) {
        match err {
            Error::NackAddress | Error::NackData => self.nack += 1,
            Error::Arbitration => self.arbitration += 1,
            Error::Bus | Error::BusStuck => self.bus += 1,
            Error::Timeout => self.timeout += 1,
            _ => self.other += 1,
        }
    }

    fn total(&self) -> u32 {
        self.nack + self.arbitration + self.bus + self.timeout + self.other + self.wrong_data
    }

    fn report(&self, phase: &str) {
        if self.total() == 0 {
            info!("{}: 0 failures", phase);
            return;
        }

        error!(
            "{}: {} failures — {} nack, {} arbitration, {} bus, {} timeout, {} other, {} wrong data",
            phase,
            self.total(),
            self.nack,
            self.arbitration,
            self.bus,
            self.timeout,
            self.other,
            self.wrong_data
        );
    }
}

/// Prints while the bus work runs, so a stalled transfer can be told from a stalled device.
///
/// A future that is never woken leaves this ticking; an interrupt that re-enters forever stops it.
#[embassy_executor::task]
async fn heartbeat() {
    let mut n = 0u32;
    loop {
        Timer::after_millis(500).await;
        n += 1;
        info!("heartbeat {}", n);
    }
}

/// A write, a read, a write-read, then `BURST` more write-reads.
async fn battery(phase: &str, i2c: &mut I2c<'static, embassy_mspm0::mode::Async>) -> Failures {
    let mut f = Failures::default();

    if let Err(e) = i2c.async_write(TARGET_ADDR, &[0xAA, 0x55]).await {
        error!("{}: write failed: {:?}", phase, e);
        f.count(e);
    }
    Timer::after_millis(AFTER_WRITE_MS).await;

    let mut read = [0u8; 2];
    match i2c.async_read(TARGET_ADDR, &mut read).await {
        Ok(()) if read == EXPECT_READ => {}
        Ok(()) => {
            error!("{}: read gave {:?}", phase, read);
            f.wrong_data += 1;
        }
        Err(e) => {
            error!("{}: read failed: {:?}", phase, e);
            f.count(e);
        }
    }

    for i in 0..=BURST {
        let mut buf = [0u8; 2];
        match i2c.async_write_read(TARGET_ADDR, &[0x01], &mut buf).await {
            Ok(()) if buf == EXPECT_WRITE_READ => {}
            Ok(()) => {
                // Only the first few, or a bus that has come apart fills the log and changes the timing.
                if f.wrong_data < 5 {
                    error!("{}: write-read {} gave {:?}", phase, i, buf);
                }
                f.wrong_data += 1;
            }
            Err(e) => {
                if f.total() < 5 {
                    error!("{}: write-read {} failed: {:?}", phase, i, e);
                }
                f.count(e);
            }
        }
    }

    f
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    spawner.spawn(heartbeat().unwrap());

    // Standard mode first, on the same instance and the same wiring, so the comparison has nothing else
    // in it. `Config::default` is 100 kHz off MFCLK.
    let mut config = Config::default();
    let mut i2c = unwrap!(I2c::new_async(p.I2C1, p.PB2, p.PB3, Irqs, config));

    info!(
        "target {:#x}, 100 kHz first, then 400 kHz on the same instance",
        TARGET_ADDR
    );

    let standard = battery("100 kHz", &mut i2c).await;
    standard.report("100 kHz");

    Timer::after_millis(AFTER_WRITE_MS).await;

    // The line under test: above 200 kHz `resolve` moves the functional clock to `BusClk`, and
    // `set_config` is what re-solves it on a live driver.
    config.bus_speed = BusSpeed::FastMode;
    match i2c.set_config(config) {
        Ok(()) => info!("400 kHz accepted; the functional clock should now be BusClk"),
        Err(e) => {
            error!("400 kHz was refused: {:?}", e);
            loop {
                Timer::after_secs(60).await;
            }
        }
    }

    let fast = battery("400 kHz", &mut i2c).await;
    fast.report("400 kHz");

    if standard.total() == 0 && fast.total() == 0 {
        info!("fastmode: ok");
    } else if standard.total() == 0 {
        warn!("fastmode: 400 kHz failed where 100 kHz did not — suspect the pull-ups before the driver");
    }

    loop {
        Timer::after_secs(60).await;
    }
}
