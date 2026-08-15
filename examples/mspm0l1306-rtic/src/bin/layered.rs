//! An RTIC hardware task alongside async tasks driving the HAL's own peripherals.
//!
//! The point of the layered pattern is that nothing is given up in either direction. `watch` waits on
//! a GPIO edge through `Input<Async>` — the HAL's own API, not hand-written register writes — while
//! `report` runs on `embassy_time`, and `on_pend` is a real RTIC hardware task that preempts both.
//!
//! **RTIC never sees the GPIO interrupt.** `bind_group_interrupts!` puts the HAL's handler in the
//! vector table exactly as it would in a plain embassy application, and RTIC claims only the
//! interrupts it names. That is what makes the two coexist without either forwarding to the other.
//!
//! # Priorities
//!
//! `on_pend` at 3 preempts the two software tasks at 1 and 2. It is the one place a number is worth
//! choosing deliberately: an interrupt RTIC does not manage — every HAL driver's — keeps NVIC
//! priority 0, the highest, so it preempts *every* RTIC task unless it is given a priority of its own.
//! `init` gives the GPIO group one; the README's table says which `Priority` is which RTIC level.
//!
//! `UART0` is bound because it is an NVIC line nothing else in this binary uses, and it is pended
//! from software: the point here is the priority relationship, not the peripheral.
//!
//! Wiring: none. `S2` is the LaunchPad's own button on `PA14`, `LED1` its own on `PA0`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = embassy_mspm0, peripherals = false, dispatchers = [SPI0, I2C1])]
mod app {
    use cortex_m::peripheral::NVIC;
    use defmt::info;
    use embassy_mspm0::gpio::{self, Input, Level, Output, Pull};
    use embassy_mspm0::interrupt::Priority;
    use embassy_mspm0::mode::Async;
    use embassy_mspm0::{Config, InterruptPolicy, bind_group_interrupts, interrupt};
    use embassy_time::Timer;

    // The HAL's own binding. RTIC is not involved, and does not need to be.
    bind_group_interrupts!(struct Irqs {
        GPIOA => gpio::InterruptHandler;
    });

    #[shared]
    struct Shared {
        presses: u32,
    }

    #[local]
    struct Local {
        ticks: u32,
    }

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        info!("Hello world!");

        // The edge reaches the CPU on GROUP1's NVIC line. Left at the reset priority it is the
        // highest in the system and preempts `on_pend` as well as the tasks, so give it the level of
        // the task it wakes. Asking `init` for it rather than setting it afterwards is what keeps an
        // edge from being taken at the reset priority in between. On a chip where GPIO shares a group
        // this moves every source on that group, not just this pin.
        let mut config = Config::default();
        config.interrupts = InterruptPolicy::Prioritise(Priority::P2);

        let p = embassy_mspm0::init(config);

        let mut led = Output::new(p.PA0, Level::Low);
        // LED1 is active low.
        led.set_high();

        let button = Input::new_async(p.PA14, Pull::Up, Irqs);

        watch::spawn(button, led).map_err(|_| ()).unwrap();
        report::spawn().map_err(|_| ()).unwrap();

        (Shared { presses: 0 }, Local { ticks: 0 })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    /// Waits on the HAL's own edge API from an RTIC task.
    #[task(priority = 2, shared = [presses])]
    async fn watch(mut cx: watch::Context, mut button: Input<'static, Async>, mut led: Output<'static>) {
        loop {
            button.wait_for_falling_edge().await;
            led.toggle();
            cx.shared.presses.lock(|presses| *presses += 1);
        }
    }

    #[task(priority = 1, shared = [presses])]
    async fn report(mut cx: report::Context) {
        loop {
            Timer::after_secs(2).await;

            let presses = cx.shared.presses.lock(|presses| *presses);
            info!("presses: {}", presses);

            NVIC::pend(interrupt::UART0);
        }
    }

    /// An RTIC hardware task, above both software tasks and below every HAL driver's interrupt.
    #[task(binds = UART0, priority = 3, local = [ticks])]
    fn on_pend(cx: on_pend::Context) {
        *cx.local.ticks += 1;
        info!("hardware task: {}", *cx.local.ticks);
    }
}
