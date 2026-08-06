//! Example of using async I2C target
//!
//! This uses the virtual COM port provided on the LP-MSPM0G3507 board.

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

/// The default 100 kHz bus off MFCLK, solved here rather than divided for on the device.
///
/// A target does not drive SCL, so the solved `TPR` is never programmed — what this is really for is
/// naming the clock source and divider up front.
const TIMING: Timing = match Timing::solve(&clock::RESET_SETUP.clocks(), ClockSel::MfClk, ClockDiv::DivBy1, 100_000) {
    Some(timing) => timing,
    None => core::panic!("100 kHz is not reachable from MFCLK"),
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let instance = p.I2C1;
    let scl = p.PB2;
    let sda = p.PB3;

    let config = Config::default().with_timing(TIMING);
    let mut target_config = TargetConfig::default();
    target_config.target_addr = Address::SevenBit(0x48);
    target_config.general_call = true;
    let mut i2c = I2cTarget::new(instance, scl, sda, Irqs, config, target_config).unwrap();

    let mut read = [0u8; 8];
    let data = [8u8; 2];
    let data_wr = [9u8; 2];

    loop {
        match i2c.listen(&mut read).await {
            Ok(Command::GeneralCall(_)) => info!("General call received"),
            Ok(Command::Read) => {
                info!("Read command received");
                match i2c.respond_to_read(&data).await.unwrap() {
                    ReadStatus::Done => info!("Finished reading"),
                    ReadStatus::NeedMoreBytes => {
                        info!("Read needs more bytes - will reset");
                        i2c.reset().unwrap();
                    }
                    ReadStatus::LeftoverBytes(_) => {
                        info!("Leftover bytes received");
                        i2c.flush_tx_fifo();
                    }
                }
            }
            Ok(Command::Write(_)) => info!("Write command received"),
            Ok(Command::WriteRead(_)) => {
                info!("Write-Read command received");
                i2c.respond_and_fill(&data_wr, 0xFE).await.unwrap();
            }
            Err(e) => info!("Got error {}", e),
        }
    }
}
