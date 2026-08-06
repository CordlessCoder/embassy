//! Answers an MSPM0 I2C controller on a 10-bit address and a 7-bit one at the same time.
//!
//! Other side of the MSPM0 `i2c_10bit` example. Two addresses are live at once so that one controller
//! binary can switch addressing mode between transfers and be answered either way: `OA1` holds the
//! 10-bit address and `OA2` the 7-bit one. The answers are the same `[8, 8]` / `[9, 9]` the other target
//! examples in this crate give, so only the addressing is new.
//!
//! Wiring, with both boards sharing a ground and **one pull-up per line** — neither board fits them, so
//! 4.7 kΩ to 3V3 on each of SCL and SDA:
//!
//! | U575  | MSPM0G3507 (`I2C1`) | Signal |
//! |-------|---------------------|--------|
//! | `PB8` | `PB2`               | SCL    |
//! | `PB9` | `PB3`               | SDA    |
//!
//! # What it reports
//!
//! Every command with the address it arrived on, so the log says which of the two matched. A controller
//! that drops the two high bits of a 10-bit address would be answered on neither and show up here as
//! silence.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{AddrMask, Address, I2c, OA2, OwnAddresses, SlaveAddrConfig, SlaveCommand, SlaveCommandKind};
use embassy_stm32::mode::Async;
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, bind_interrupts, dma, i2c, peripherals};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});

/// The 10-bit address, on `OA1`.
///
/// The low eight bits are deliberately the 7-bit address below, so that a controller which sent only
/// the second address byte would still be answered here. Telling the two apart is what the controller's
/// probe of `0x048` is for.
const TARGET_ADDR_10BIT: u16 = 0x148;

/// The 7-bit address, on `OA2`, and the one the other examples use.
const TARGET_ADDR_7BIT: u8 = 0x48;

/// Answer to a plain read.
const READ_ANSWER: [u8; 2] = [8, 8];

/// Answer to a write-read.
const WRITE_READ_ANSWER: [u8; 2] = [9, 9];

/// 100 kHz, which is what the MSPM0 examples solve their timing for.
const FREQUENCY: Hertz = Hertz::khz(100);

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = Config::default();

    // 16 MHz HSI multiplied to 160 MHz, matching the other examples in this crate.
    config.rcc.hsi = true;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul10,
        divp: None,
        divq: None,
        divr: Some(PllDiv::Div1),
    });
    config.rcc.sys = Sysclk::Pll1R;
    config.rcc.voltage_range = VoltageScale::Range1;

    let p = embassy_stm32::init(config);

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = FREQUENCY;

    let addr_config = SlaveAddrConfig {
        addr: OwnAddresses::Both {
            oa1: Address::TenBit(TARGET_ADDR_10BIT),
            oa2: OA2 {
                addr: TARGET_ADDR_7BIT,
                mask: AddrMask::NOMASK,
            },
        },
        // The other target examples enable this, so match them.
        general_call: true,
    };

    let mut i2c: I2c<'_, Async, i2c::MultiMaster> =
        I2c::new(p.I2C1, p.PB8, p.PB9, p.GPDMA1_CH0, p.GPDMA1_CH1, Irqs, i2c_config)
            .into_slave_multimaster(addr_config);

    info!(
        "listening on 10-bit {:#x} and 7-bit {:#x}",
        TARGET_ADDR_10BIT, TARGET_ADDR_7BIT
    );

    let mut buf = [0u8; 64];

    loop {
        match i2c.listen().await {
            Ok(SlaveCommand { kind, address }) => match kind {
                SlaveCommandKind::Read => match i2c.respond_to_read(&READ_ANSWER).await {
                    Ok(status) => info!("read from {}: answered {:?}, {}", address, READ_ANSWER, status),
                    Err(e) => error!("responding to read failed: {}", e),
                },
                SlaveCommandKind::Write => match i2c.respond_to_write(&mut buf).await {
                    Ok(len) => {
                        info!("write from {}: {:?}", address, buf[..len]);

                        // A write-read arrives as a write followed by a read on the same transaction.
                        // Answering unconditionally is how the STM32 driver distinguishes them: a plain
                        // write times out here, which is not an error.
                        match i2c.respond_to_read(&WRITE_READ_ANSWER).await {
                            Ok(status) => debug!("...and the read half: {}", status),
                            Err(i2c::Error::Timeout) => debug!("...write only, no read followed"),
                            Err(e) => error!("responding to the read half failed: {}", e),
                        }
                    }
                    Err(e) => error!("receiving a write failed: {}", e),
                },
            },
            Err(e) => error!("listen failed: {}", e),
        }
    }
}
