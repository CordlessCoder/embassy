//! Link-time proof that the buffered UART needs no time source.
//!
//! The companion to `drivers`, split out because `BufferedUart` wants `PA1` on this part and `I2C0`
//! wants it for SDA. Not meant to be run.
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0])]
mod app {
    use embassy_mspm0::uart::{self, BufferedUart};
    use embassy_mspm0::{Config, bind_interrupts};
    use static_cell::StaticCell;

    bind_interrupts!(struct Irqs {
        UART1 => uart::BufferedInterruptHandler<embassy_mspm0::peripherals::UART1>;
    });

    static TX: StaticCell<[u8; 16]> = StaticCell::new();
    static RX: StaticCell<[u8; 16]> = StaticCell::new();

    #[shared]
    struct Shared {}
    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let p = embassy_mspm0::init(Config::default());
        let uart = BufferedUart::new(
            p.UART1,
            p.PA10,
            p.PA1,
            Irqs,
            TX.init([0; 16]),
            RX.init([0; 16]),
            uart::Config::default(),
        )
        .unwrap();
        work::spawn(uart).map_err(|_| ()).unwrap();
        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn work(_cx: work::Context, mut uart: BufferedUart<'static>) {
        use embedded_io_async::{Read, Write};
        let mut buf = [0u8; 4];
        loop {
            let _ = uart.read(&mut buf).await;
            let _ = uart.write(&buf).await;
        }
    }
}
