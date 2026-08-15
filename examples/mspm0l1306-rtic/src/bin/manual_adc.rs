//! Conversions started and collected entirely from an RTIC hardware task.
//!
//! [`adc::low_level::Adc`] powers the instance up and applies a `Config`, then stops. There is no
//! waker, no future and no interrupt handler from the HAL — the task below is the handler, and it
//! starts the next conversion before it returns, so the loop sustains itself off the interrupt.
//!
//! # What it demonstrates
//!
//! **Arming one source and nothing else.** `arm_only` sets the whole mask in one write. A sequence
//! wants exactly that: its earlier results set their flags quietly and only the last one wakes
//! anything.
//!
//! **Guarding the sleep around a conversion.** A conversion runs on ADCCLK, and a deep sleep that
//! stops that clock stops the conversion with nothing reporting it. The guard is taken before
//! `start` and released once the result is read, which is the whole window rather than the moment of
//! starting. Because this restarts immediately, a conversion is in flight almost always and the guard
//! is almost always held — so the idle below reaches the shallowest level the guard permits and no
//! deeper. That is the guard working, not the example failing.
//!
//! **Clearing before reading.** The result register's flag is what the handler dispatches on, and
//! leaving it set would vector straight back in. Reading the result clears it too; doing both makes
//! the handler read the same whichever order the silicon settles it in.
//!
//! # Wiring
//!
//! None. PA15 is brought out on the LaunchPad's header and floats if nothing is attached, which is
//! fine — LED1 on PA0 lights while the reading is above the first one taken, so touching the pin or
//! grounding it changes the state.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0])]
mod app {
    use embassy_mspm0::adc::Conversion;
    use embassy_mspm0::adc::low_level::{Adc, Event};
    use embassy_mspm0::gpio::{Level, Output};
    use embassy_mspm0::peripherals::{ADC0, PA15};
    use embassy_mspm0::sysctl::WakeGuard;
    use embassy_mspm0::{Config, Peri};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        adc: Adc<'static, ADC0>,
        pin: Peri<'static, PA15>,
        led: Output<'static>,
        /// The first result, which every later one is compared against.
        reference: Option<u16>,
        /// Held while a conversion is in flight, and only then.
        guard: Option<WakeGuard>,
    }

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let p = embassy_mspm0::init(Config::default());

        let mut adc = Adc::new(p.ADC0, Default::default());
        let mut pin = p.PA15;
        let led = Output::new(p.PA0, Level::Low);

        // Only the first result reaches the CPU. Nothing else is armed, so nothing else can.
        adc.arm_only(Event::Result(0));

        // RTIC unmasked the line for us when it bound the task below; `low_level` never touches it.
        adc.set_conversion(&mut pin, Conversion::default());
        let guard = adc.conversion_floor().map(WakeGuard::new);
        adc.start();

        (
            Shared {},
            Local {
                adc,
                pin,
                led,
                reference: None,
                guard,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            critical_section::with(|cs| embassy_mspm0::idle(cs));
        }
    }

    #[task(binds = ADC0, priority = 2, local = [adc, pin, led, reference, guard])]
    fn on_adc(cx: on_adc::Context) {
        let adc = cx.local.adc;

        if !adc.is_pending(Event::Result(0)) {
            return;
        }

        adc.clear_pending(Event::Result(0));
        let code = adc.result(0);

        // The conversion is over, so the sleep it was holding open can close — briefly, since the
        // next one starts below.
        *cx.local.guard = None;

        let reference = *cx.local.reference.get_or_insert(code);
        cx.local
            .led
            .set_level(if code > reference { Level::High } else { Level::Low });

        adc.set_conversion(&mut *cx.local.pin, Conversion::default());
        *cx.local.guard = adc.conversion_floor().map(WakeGuard::new);
        adc.start();
    }
}
