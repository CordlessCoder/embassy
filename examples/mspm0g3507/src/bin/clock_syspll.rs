//! Run MCLK at 80 MHz from the SYSPLL, and put the result somewhere it can be measured.
//!
//! This is the only example that programs a non-default clock tree, so it is what exercises
//! `sysctl::clock::apply` — the SYSPLL bring-up, the flash wait-state ordering and the `settle()`
//! timeouts. Everything else runs on the reset tree, where `apply` has nothing to do.
//!
//! # What to check
//! - The reported rates match the constants below. That only proves `resolve()`, which the `const`
//!   assertions already cover, but it confirms `init` published the tree the program asked for.
//! - **ULPCLK on PA22, divided by 12.** 40 MHz / 12 = 3.333 MHz. This is the real check: it comes
//!   from the hardware rather than from arithmetic, so it says whether the PLL actually locked and
//!   whether MCLK switched. If the tree failed to apply, MCLK stays on the 32 MHz SYSOSC and this
//!   pin reads 32 / 12 = 2.667 MHz instead.
//! - **MCLK counted against LFCLK**, logged once per 100 ms window. `TIMA0` is in PD1, so its bus clock is MCLK
//!   rather than ULPCLK: the pin above measures the 40 MHz side of the divider and this measures the
//!   80 MHz side. Both oscillators involved are RC, so expect a couple of percent of disagreement; a
//!   tree that failed to apply is out by 2.5x, MCLK having stayed on the 32 MHz SYSOSC.
//! - The LED toggles once per window. The time driver runs from LFCLK, which this tree leaves alone, so
//!   an LED that keeps time while the pin reads 3.333 MHz means MCLK moved and timekeeping did not.
//!
//! Running at 80 MHz needs two wait states in flash, which `apply` programs before the switch. A
//! part that reaches this point and then executes normally has that ordering right; getting it
//! wrong faults rather than reading slowly.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::peripherals;
use embassy_mspm0::sysctl::clock::{self, MclkSource, SysPllConfig, SysPllRef, SysPllTap, UlpclkDiv};
use embassy_mspm0::sysctl::{ClkOut, ClkOutDiv, ClkOutSource, LowPowerInstance};
use embassy_mspm0::tim::{ClockSel, low_level};
use embassy_time::{Instant, TICK_HZ, Timer};
use panic_probe as _;

/// TI's own 80 MHz recipe, from the G-series TRM.
///
/// SYSOSC at 32 MHz references the loop, `PDIV = 2` divides it to a 16 MHz loop input, `QDIV = 5`
/// multiplies that to an 80 MHz VCO. `CLK2X` doubles the VCO and divides by 2, landing back on
/// 80 MHz for MCLK; `CLK1` divides by 2 for a 40 MHz CANCLK. ULPCLK has to be halved because its
/// ceiling is 40 MHz where MCLK's is 80.
const CLOCK: clock::Config = clock::Config::new()
    .with_syspll(SysPllConfig {
        reference: SysPllRef::Sysosc,
        pdiv: 2,
        qdiv: 5,
        clk0_div: None,
        clk1_div: Some(2),
        clk2x_div: Some(2),
        mclk_tap: SysPllTap::Clk2x,
    })
    .with_mclk(MclkSource::Hsclk)
    .with_ulpclk_div(UlpclkDiv::Div2);

/// Resolving in a `const` is what makes an out-of-range tree a build error rather than a panic at
/// `init`. These spell out what the recipe above is expected to produce.
const CLOCKS: clock::Clocks = match CLOCK.resolve() {
    Ok(clocks) => clocks,
    Err(_) => core::panic!("the 80 MHz tree does not resolve on this chip"),
};

const _: () = core::assert!(CLOCKS.mclk == 80_000_000);
const _: () = core::assert!(CLOCKS.ulpclk == 40_000_000);
const _: () = core::assert!(CLOCKS.syspll_clk1 == 40_000_000);
const _: () = core::assert!(CLOCKS.flash_wait == 2);

/// What the CLK_OUT pin should read, in Hz. 40 MHz ULPCLK over the divider below.
const EXPECTED_CLK_OUT_HZ: u32 = CLOCKS.ulpclk / 12;

/// What the counter below should measure. `TIMA0` is in PD1, so its bus clock is MCLK.
const EXPECTED_MCLK_HZ: u32 = CLOCKS.bus_clock(<peripherals::TIMA0 as LowPowerInstance>::SLEEP.power_domain);

/// Window each measurement counts over. 100 ms rather than longer because 80 MHz needs every bit of
/// the prescaler's range to keep 16 bits from wrapping.
const WINDOW_MS: u64 = 100;

/// Keeps the 16-bit counter from wrapping inside the window while still counting as fast as it can:
/// the measurement can be no finer than one prescaled tick.
///
/// Sized against the part's MCLK ceiling rather than [`EXPECTED_MCLK_HZ`], because a wrapped counter
/// aliases modulo 65536 and the alias of a too-fast clock reads as a plausible slow one. They are the
/// same number here — this tree asks for the ceiling — but the expected rate is the wrong thing to
/// size against.
///
/// 60000 rather than 65536 leaves room for the software either side of the window, which is counted too.
const PRESCALER: u16 = {
    let mut prescaler = 1u32;

    while (embassy_mspm0::sysctl::MAX_MCLK_HZ as u64) * WINDOW_MS / 1000 / prescaler as u64 > 60_000 {
        prescaler *= 2;
    }

    core::assert!(prescaler <= 256, "no prescaler fits this window; shorten WINDOW_MS");

    prescaler as u16
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_mspm0::Config::default();
    config.clock = CLOCK.build();

    let p = embassy_mspm0::init(config);

    // Read back what `init` actually published, rather than trusting the constants.
    let clocks = embassy_mspm0::sysctl::clocks();

    info!(
        "mclk {} Hz, ulpclk {} Hz, sysosc {} Hz, lfclk {} Hz, {} flash wait states",
        clocks.mclk, clocks.ulpclk, clocks.sysosc, clocks.lfclk, clocks.flash_wait
    );
    info!(
        "syspll: clk0 {} Hz, clk1 {} Hz, clk2x {} Hz",
        clocks.syspll_clk0, clocks.syspll_clk1, clocks.syspll_clk2x
    );

    // `clocks()` is a runtime read of what was programmed; the constants are what was asked for.
    // They can only differ if `apply` took a different path from `resolve`.
    if clocks.mclk != CLOCKS.mclk || clocks.ulpclk != CLOCKS.ulpclk {
        error!(
            "programmed tree does not match the resolved one: expected mclk {} ulpclk {}",
            CLOCKS.mclk, CLOCKS.ulpclk
        );
    }

    // SYSRST must be initiated for the output to actually change. probe-rs currently does not do
    // this, so flash and then power cycle or press reset to see the pin move.
    info!("PA22 should read {} Hz", EXPECTED_CLK_OUT_HZ);
    let _clk_out = ClkOut::new(p.CLK_OUT, p.PA22, ClkOutSource::UlpClk(ClkOutDiv::Div12));

    let mut led = Output::new(p.PA0, Level::High);

    // `TIMA0` is in PD1, so its bus clock is MCLK itself rather than ULPCLK: this counts the 80 MHz
    // side of the divider where the pin above shows the 40 MHz side. Together they cover the whole
    // chain, and neither goes through `resolve()`'s arithmetic.
    let counter = low_level::Timer::new(
        p.TIMA0,
        low_level::Config {
            clock: ClockSel::BusClk,
            prescaler: PRESCALER,
            // The counter is read across windows, so enabling must not restart it.
            counter_on_enable: low_level::CounterOnEnable::Preserve,
            // A halted core must not stop the count, or a breakpoint would read as a slow clock.
            free_run_in_debug: true,
            ..Default::default()
        },
    );
    counter.start();

    // A counter reloading from its maximum wraps modulo `max + 1`.
    let wrap: u32 = counter.max_count().into();

    loop {
        led.toggle();

        let before: u32 = counter.counter().into();
        let start = Instant::now();

        Timer::after_millis(WINDOW_MS).await;

        // LFCLK is the reference: this tree does not touch it, and it is an RC oscillator, so a couple
        // of percent of disagreement is oscillator tolerance rather than a wrong tree. A tree that
        // failed to apply is out by the ratio of the two rates — 2.5x here, MCLK having stayed on the
        // 32 MHz SYSOSC.
        let elapsed = start.elapsed().as_ticks();
        let after: u32 = counter.counter().into();

        let counted = after.wrapping_sub(before) & wrap;
        let measured = counted as u64 * PRESCALER as u64 * TICK_HZ / elapsed;

        info!(
            "mclk measured against lfclk: {} Hz, expected {} Hz",
            measured, EXPECTED_MCLK_HZ
        );
    }
}
