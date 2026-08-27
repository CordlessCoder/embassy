//! The ADC's window comparator, against a constant input.
//!
//! **No wiring at all.** `TESTING.md` C44.
//!
//! A window comparator needs a signal that crosses its thresholds, and the obvious way to get one is
//! a swept input — which needs a DAC this HAL does not have yet, or an external source. Neither is
//! necessary: **hold the input still and move the thresholds instead.** The relationship being
//! tested is between a result and a pair of registers, and which side moves does not matter.
//!
//! The supply monitor is the input because it is a known constant on every device in the portfolio:
//! it presents VDD/3, so against a VDD reference it reads full scale over three whatever the supply
//! is — 1365 of 4095 at 12 bits. That is a plausible-looking number rather than a measurement, which
//! is why the driver names it `ratiometric_code` rather than leaving callers to find it surprising.
//!
//! Four placements per pass, and each names the flag it should raise:
//!
//! - **band well below the reading** — `WindowHigh`, and neither of the others.
//! - **band bracketing it** — `WindowInRange` alone.
//! - **band well above** — `WindowLow` alone.
//! - **no window at all** — none of the three, which is what catches a comparator left armed.
//!
//! The thresholds move through [`Adc::set_window`] rather than by rebuilding the driver, so this also
//! covers the run-time path rather than only the one `Config` takes at construction.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::low_level::{Adc, Event};
use embassy_mspm0::adc::{Config, Conversion, PowerDown, SupplyMonitor, Window};
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let resolution = Config::new().resolution;
    let mut adc0 = p.ADC0;
    let mut channel = SupplyMonitor;

    info!(
        "Twakeup: max={:?} typ={:?} -> sizing against {} ns",
        Config::WAKEUP_MAX_NS,
        Config::WAKEUP_TYP_NS,
        Config::wakeup_ns()
    );

    // What the supply monitor should read at this resolution, from the divider rather than from a
    // measurement. The driver documents a margin because the divider's own spec is +/-1.5%.
    let expected = SupplyMonitor::ratiometric_code(resolution);
    info!("expected supply-monitor code: {}", expected);

    loop {
        // Both power modes. Under `Auto` the front end wakes before every sample window, and a window
        // too short for it returns a plausible number rather than an error -- so the check is that the
        // supply monitor still reads what the divider says it should.
        for power_down in [PowerDown::Manual, PowerDown::Auto] {
            let mut config = Config::new();
            config.power_down = power_down;

            let mut adc = Adc::new(adc0.reborrow(), config);

            for (name, window, want_high, want_in, want_low) in [
                ("below", Some(Window::new(8, 16)), true, false, false),
                (
                    "bracketing",
                    Some(Window::new(expected - 200, expected + 200)),
                    false,
                    true,
                    false,
                ),
                (
                    "above",
                    Some(Window::new(expected + 800, expected + 1000)),
                    false,
                    false,
                    true,
                ),
                ("none", None, false, false, false),
            ] {
                adc.set_window(window);

                let mut conversion = Conversion::new();
                conversion.window = window.is_some();

                adc.set_conversion(&mut channel, conversion);

                adc.clear_pending(Event::WindowHigh);
                adc.clear_pending(Event::WindowLow);
                adc.clear_pending(Event::WindowInRange);

                adc.start();
                while adc.is_converting() {}

                let code = adc.result(0);
                let high = adc.is_pending(Event::WindowHigh);
                let in_range = adc.is_pending(Event::WindowInRange);
                let low = adc.is_pending(Event::WindowLow);

                let pass = high == want_high && in_range == want_in && low == want_low;

                let sane = code.abs_diff(expected as u16) < 64;

                info!(
                    "{=str}/{}: code={} high={} in={} low={} -> {}",
                    match power_down {
                        PowerDown::Manual => "manual",
                        PowerDown::Auto => "auto",
                    },
                    name,
                    code,
                    high,
                    in_range,
                    low,
                    if pass && sane { "PASS" } else { "FAIL" }
                );
            }
        }

        info!("--- pass complete ---");
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
    }
}
