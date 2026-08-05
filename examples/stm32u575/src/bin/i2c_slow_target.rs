//! `i2c_target` that stalls before answering, so the controller sees a multi-millisecond clock stretch.
//!
//! The STM32 target holds SCL low from the moment it matches an address until the application answers,
//! so sleeping between `listen()` and the response is all it takes — the driver already does the
//! stretching, this just gives it a reason to.
//!
//! It exists for the lockup half of [#6633](https://github.com/embassy-rs/embassy/pull/6633): the report
//! is that a write taking longer than 2.5 ms wedges the MSPM0 controller, after which nothing gets past
//! the unbounded `while !idle()` spin. `i2c_faults` against the ordinary `i2c_target` never produced a
//! transfer that long — transfers are capped at `fifo_size`, about 0.8 ms at 100 kHz, and a NACK aborts
//! fast — so the condition went untested. This produces it on demand.
//!
//! Wiring, with both boards sharing a ground and 4.7 kΩ from each of SCL and SDA to 3V3:
//!
//! | U575  | MSPM0G3507 (`I2C1`) | MSPM0L1306 (`I2C0`) | Signal |
//! |-------|---------------------|---------------------|--------|
//! | `PB8` | `PB2`               | `PA1`               | SCL    |
//! | `PB9` | `PB3`               | `PA0`               | SDA    |
//!
//! On the L1306 take the LED1 jumper off: `PA0` is LED1 as well as SDA.
//!
//! Only every [`STRETCH_EVERY`]th transaction is stalled, so ordinary ones surround each stretched one
//! and the controller's recovery is visible rather than inferred. Raise [`STRETCH_MS`] to find the
//! threshold; the log names the length before each stall, so the last line before a hang says how long
//! the transfer that wedged it was.

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
use embassy_time::Timer;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    GPDMA1_CHANNEL0 => dma::InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => dma::InterruptHandler<peripherals::GPDMA1_CH1>;
});

/// Same address the other target examples answer on.
const TARGET_ADDR: u8 = 0x48;

/// Answer to a plain read.
const READ_ANSWER: [u8; 2] = [8, 8];

/// Answer to a write-read.
const WRITE_READ_ANSWER: [u8; 2] = [9, 9];

/// 100 kHz, which is what the MSPM0 examples solve their timing for.
const FREQUENCY: Hertz = Hertz::khz(100);

/// How long to hold SCL before answering. Comfortably past the 2.5 ms the report names.
const STRETCH_MS: u64 = 5;

/// Stall one transaction in this many, so unstretched ones bracket each stretched one.
const STRETCH_EVERY: u32 = 4;

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

    info!(
        "listening on {:#x}, stalling {} ms every {} transactions",
        TARGET_ADDR, STRETCH_MS, STRETCH_EVERY
    );

    let mut buf = [0u8; 64];
    let mut seen: u32 = 0;

    loop {
        match i2c.listen().await {
            Ok(SlaveCommand { kind, address }) => {
                seen = seen.wrapping_add(1);

                // SCL is already being held low here — the address matched and nothing has answered yet.
                let stretching = seen % STRETCH_EVERY == 0;
                if stretching {
                    info!("transaction {}: stretching {} ms", seen, STRETCH_MS);
                    Timer::after_millis(STRETCH_MS).await;
                }

                match kind {
                    SlaveCommandKind::Read => match i2c.respond_to_read(&READ_ANSWER).await {
                        Ok(status) => debug!("read from {}: {}", address, status),
                        Err(e) => error!("responding to read failed: {}", e),
                    },
                    SlaveCommandKind::Write => match i2c.respond_to_write(&mut buf).await {
                        Ok(len) => {
                            debug!("write from {}: {:?}", address, buf[..len]);

                            // Stall the read half too, so a write-read is stretched twice over.
                            if stretching {
                                Timer::after_millis(STRETCH_MS).await;
                            }

                            match i2c.respond_to_read(&WRITE_READ_ANSWER).await {
                                Ok(status) => debug!("...and the read half: {}", status),
                                Err(i2c::Error::Timeout) => debug!("...write only, no read followed"),
                                Err(e) => error!("responding to the read half failed: {}", e),
                            }
                        }
                        Err(e) => error!("receiving a write failed: {}", e),
                    },
                }

                if stretching {
                    info!("transaction {}: released", seen);
                }
            }
            Err(e) => error!("listen failed: {}", e),
        }
    }
}
