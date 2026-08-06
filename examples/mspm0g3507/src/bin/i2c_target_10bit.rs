//! Answers a 10-bit address, which the plain `i2c_target` example cannot reach.
//!
//! Other side of the U575 `i2c_controller_10bit` example. What is under test is `TOAR.TMODE`, the
//! target's own addressing mode. The **second** own address is not here: `OAR2` is only compared while
//! the target is in 7-bit mode, so the driver refuses the pair — see `i2c_target`, which has it.
//!
//! Wiring, with both boards sharing a ground and **one pull-up per line** — neither board fits them, so
//! 4.7 kΩ to 3V3 on each of SCL and SDA:
//!
//! | MSPM0G3507 (`I2C1`) | U575  | Signal |
//! |---------------------|-------|--------|
//! | `PB2`               | `PB8` | SCL    |
//! | `PB3`               | `PB9` | SDA    |
//!
//! # What to check
//!
//! Every command is logged with the address `matched_address()` reports, which for a 10-bit target has
//! to come back as the full `0x148` rather than the low byte alone.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::i2c::{Address, ClockDiv, ClockSel, Config, Timing};
use embassy_mspm0::i2c_target::{Command, Config as TargetConfig, I2cTarget, ReadStatus};
use embassy_mspm0::peripherals::I2C1;
use embassy_mspm0::sysctl::clock;
use embassy_mspm0::{bind_interrupts, i2c};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C1 => i2c::InterruptHandler<I2C1>;
});

/// The 10-bit address, in `TOAR`.
const TARGET_10BIT: Address = Address::TenBit(0x148);

/// The default 100 kHz bus off MFCLK, solved here rather than divided for on the device.
const TIMING: Timing = match Timing::solve(&clock::RESET_SETUP.clocks(), ClockSel::MfClk, ClockDiv::DivBy1, 100_000) {
    Some(timing) => timing,
    None => core::panic!("100 kHz is not reachable from MFCLK"),
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let config = Config::default().with_timing(TIMING);

    let mut target_config = TargetConfig::default();
    target_config.target_addr = TARGET_10BIT;

    let mut i2c = I2cTarget::new(p.I2C1, p.PB2, p.PB3, Irqs, config, target_config).unwrap();

    info!("answering {}", TARGET_10BIT);

    let mut read = [0u8; 8];
    let data = [8u8; 2];
    let data_wr = [9u8; 2];

    loop {
        let command = i2c.listen(&mut read).await;

        // Read before answering: the peripheral re-evaluates the match on every address comparison, so
        // responding first would report whatever the next command matched.
        let on = i2c.matched_address();

        match command {
            Ok(Command::GeneralCall(n)) => info!("{}: general call of {} bytes", on, n),
            Ok(Command::Read) => {
                info!("{}: read", on);
                match i2c.respond_to_read(&data).await.unwrap() {
                    ReadStatus::Done => debug!("finished reading"),
                    ReadStatus::NeedMoreBytes => {
                        debug!("read needs more bytes - will reset");
                        i2c.reset().unwrap();
                    }
                    ReadStatus::LeftoverBytes(_) => {
                        debug!("leftover bytes");
                        i2c.flush_tx_fifo();
                    }
                }
            }
            Ok(Command::Write(n)) => info!("{}: write of {:?}", on, read[..n]),
            Ok(Command::WriteRead(n)) => {
                info!("{}: write-read of {:?}", on, read[..n]);
                i2c.respond_and_fill(&data_wr, 0xFE).await.unwrap();
            }
            Err(e) => warn!("{}: {}", on, e),
        }
    }
}
