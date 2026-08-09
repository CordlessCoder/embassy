//! Link-time proof that the I2C and ADC async drivers need no time source.
//!
//! Not a demonstration and not meant to be run: it addresses a device that is probably not on the bus
//! and converts a pin nothing drives. Its job is to fail the build if either driver ever grows a
//! dependency on `embassy-time`, which nothing else in this crate would catch — the other binaries
//! only exercise GPIO.
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0])]
mod app {
    use embassy_mspm0::adc::{self, Adc};
    use embassy_mspm0::i2c::{self, I2c};
    use embassy_mspm0::peripherals::{ADC0, PA25};
    use embassy_mspm0::{Config, Peri, bind_interrupts, mode};

    bind_interrupts!(struct Irqs {
        I2C0 => i2c::InterruptHandler<embassy_mspm0::peripherals::I2C0>;
        ADC0 => adc::InterruptHandler<ADC0>;
    });

    #[shared]
    struct Shared {}
    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let p = embassy_mspm0::init(Config::default());

        let i2c = I2c::new_async(p.I2C0, p.PA1, p.PA0, Irqs, i2c::Config::default()).unwrap();
        let adc = Adc::<'_, _, mode::Async>::new_async(p.ADC0, Irqs, adc::Config::default());

        work::spawn(i2c, adc, p.PA25).map_err(|_| ()).unwrap();
        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn work(
        _cx: work::Context,
        mut i2c: I2c<'static, mode::Async>,
        mut adc: Adc<'static, ADC0, mode::Async>,
        mut pin: Peri<'static, PA25>,
    ) {
        let mut buf = [0u8; 4];
        loop {
            let _ = i2c.async_write_read(0x48u8, &[0x00], &mut buf).await;
            let _ = adc.irq_read(&mut pin, adc::Conversion::default()).await;
        }
    }
}
