//! An I2C target for transfers longer than a controller's FIFO.
//!
//! Wiring: `PB8` SCL and `PB9` SDA to the controller's pair, shared ground, 4.7 kΩ from each line to
//! 3V3.
//!
//! `i2c_target` answers everything in two bytes, which is fine for the fault tests and useless for a
//! controller that is being asked to move hundreds. This one carries a ramp instead, in both
//! directions, so the controller can check every byte rather than the first two:
//!
//! - **A write** must arrive as `0, 1, 2, …` truncated to whatever length was sent. The length is
//!   reported and any byte out of place is named, so a controller that drops or repeats part of a long
//!   write is caught here rather than showing up as a mismatch two transactions later.
//! - **A read** is answered with the same ramp, as many bytes as the controller takes.
//!
//! The ramp restarts at every transaction, so a controller reading 8 bytes and one reading 200 both
//! expect the same prefix and neither has to know what came before.
//!
//! Nothing here is specific to the peer: it is the controller under test that has the interesting
//! behaviour, and this end only has to be boring and long enough.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{Address, I2c, OwnAddresses, SlaveAddrConfig, SlaveCommand, SlaveCommandKind};
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

/// The address this answers on, matching every other target example here.
const TARGET_ADDR: u8 = 0x48;

/// Longest transfer either direction, which sets both buffers.
///
/// The controller's own ceiling is far higher — 4095 on an MSPM0, from its burst-length field — so this
/// is what the *test* covers rather than what either side can do.
const MAX_LEN: usize = 256;

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
        addr: OwnAddresses::OA1(Address::SevenBit(TARGET_ADDR)),
        general_call: true,
    };

    let mut i2c: I2c<'_, Async, i2c::MultiMaster> =
        I2c::new(p.I2C1, p.PB8, p.PB9, p.GPDMA1_CH0, p.GPDMA1_CH1, Irqs, i2c_config)
            .into_slave_multimaster(addr_config);

    // The ramp, sent as the answer to every read and expected as the payload of every write.
    let mut ramp = [0u8; MAX_LEN];
    for (i, byte) in ramp.iter_mut().enumerate() {
        *byte = i as u8;
    }

    let mut buf = [0u8; MAX_LEN];

    info!("listening on {:#x}, ramp of {} bytes", TARGET_ADDR, MAX_LEN);

    loop {
        match i2c.listen().await {
            Ok(SlaveCommand { kind, address }) => match kind {
                SlaveCommandKind::Read => match i2c.respond_to_read(&ramp).await {
                    Ok(status) => info!("read from {}: {}", address, status),
                    Err(e) => error!("responding to read failed: {}", e),
                },
                SlaveCommandKind::Write => match i2c.respond_to_write(&mut buf).await {
                    Ok(len) => {
                        check_ramp(&buf[..len]);

                        // A write-read arrives as a write then a read on one transaction, so offer the
                        // answer unconditionally. A plain write times out here, which is not an error —
                        // `i2c_target` documents the same shape and why it must not be gated.
                        match i2c.respond_to_read(&ramp).await {
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

/// Report whether `got` is the leading run of the ramp, and where it first is not.
///
/// Naming the first bad index and its neighbours is what separates a dropped byte from a repeated one,
/// which are the two ways a refill loop goes wrong and look identical in a length alone.
fn check_ramp(got: &[u8]) {
    match got.iter().enumerate().find(|(i, byte)| **byte != *i as u8) {
        None => info!("write of {}: ramp ok", got.len()),
        Some((i, byte)) => {
            error!("write of {}: byte {} is {}, expected {}", got.len(), i, byte, i as u8);
            let from = i.saturating_sub(2);
            let to = (i + 3).min(got.len());
            error!("  around it: {:?}", got[from..to]);
        }
    }
}
