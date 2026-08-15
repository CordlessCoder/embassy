//! `button_nolog`'s application with the edge serviced by hand, which is the floor below it.
//!
//! Nothing here is async. The pin is a `Blocking` one, RTIC owns `GROUP1`, and the handler reads and
//! clears the pin itself — so the binary links no waiter list, no demultiplexer and no future. What
//! that is worth against `button_nolog` is the point of building both.
//!
//! `Config::interrupts` is [`InterruptPolicy::External`] because RTIC's `pre_init` has already
//! prioritised and unmasked `GROUP1` by the time `init` runs.
//!
//! Press `S2` (`PA14`); `LED1` (`PA0`) toggles. There is nothing else to see, by design.
//!
//! Two things have to be cleared, not one: the pin's own status, and the group's latched record of
//! `GPIOA` having fired. Leave the second and the group's line stays asserted, so the handler returns
//! and the NVIC vectors straight back in.
#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0])]
mod app {
    use embassy_mspm0::gpio::{Edge, Input, Level, Output, Pull};
    use embassy_mspm0::mode::Blocking;
    use embassy_mspm0::{Config, InterruptPolicy, interrupt_group};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        button: Input<'static, Blocking>,
        led: Output<'static>,
    }

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let mut config = Config::default();
        config.interrupts = InterruptPolicy::External;

        let p = embassy_mspm0::init(config);

        let mut led = Output::new(p.PA0, Level::Low);
        // LED1 is active low.
        led.set_high();

        let mut button = Input::new(p.PA14, Pull::Up);
        button.enable_interrupt(Edge::Falling);

        (Shared {}, Local { button, led })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    /// Nothing else is on this group, so there is nothing to forward to — but the group still has to
    /// be told, or it holds its line asserted.
    #[task(binds = GROUP1, priority = 1, local = [button, led])]
    fn on_edge(cx: on_edge::Context) {
        if cx.local.button.take_pending() {
            cx.local.led.toggle();
        }

        interrupt_group::ack::group1();
    }
}
