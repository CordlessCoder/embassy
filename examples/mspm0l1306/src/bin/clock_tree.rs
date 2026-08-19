//! Measures what `clock::apply` actually programmed, against LFCLK.
//!
//! The L-series has no SYSPLL, so what there is to get wrong here is the rest of the tree: the
//! `SYSOSCCFG.FREQ` operating point, `MCLKCFG.MDIV`, whether MFCLK survives the choice, and the flash
//! wait states that have to move before MCLK does. `resolve()` is checked at compile time by the
//! assertions below, but nothing has ever confirmed that the registers agree with it — that is what
//! this measures.
//!
//! # What it does
//!
//! `TIMG2` free-runs from the bus clock while `embassy-time` counts the same window on LFCLK, which no
//! tree here touches. Bus ticks over LFCLK ticks is the bus frequency, measured rather than computed,
//! and the log compares it against what `init` published. `TIMG1` drives 1 kHz of PWM on `PA26` for the
//! same reason from the outside.
//!
//! | Pin    | Signal                                                                |
//! |--------|-----------------------------------------------------------------------|
//! | `PA26` | 1 kHz PWM, 50% duty — RGB LED D1's red channel via J12, or BoosterPack J4.38 |
//! | `PA0`  | LED1, toggled once per measurement as a liveness check                 |
//!
//! # What to check
//!
//! - **The measured bus clock matches the reported one.** Both oscillators are internal RC, so a few
//!   percent of disagreement is LFOSC and SYSOSC tolerance, not a bug. A wrong tree is out by a
//!   *factor* — 4, 8 or 32 — because every mistake available here is a divider or an operating point.
//! - **`PA26` reads 1 kHz.** The HAL solves the PWM divider from the tree it thinks it programmed, so
//!   this pin is only right if the tree it published is the one the hardware is running. It is the
//!   check that does not go through `embassy-time` or the counter.
//! - **The program keeps running at all.** Flash needs its wait states raised before MCLK goes up and
//!   lowered only after it comes down; getting that order wrong faults rather than running slowly.
//!
//! # Other trees to try
//!
//! `clock::apply` only runs inside `init`, so the tree is fixed per binary: to walk these, edit `TREE`
//! and reflash.
//!
//! ```ignore
//! // The control: the reset tree, 32 MHz SYSOSC straight to MCLK with MFCLK on. `apply` has nothing
//! // to do, and the measurement should land on 32 MHz.
//! const TREE: clock::Config = clock::Config::new();
//!
//! // SYSOSC's 4 MHz operating point, no divider. MFCLK is left on, which is only legal because it is
//! // held at 4 MHz by SYSCTL independently of SYSOSC.
//! const TREE: clock::Config = clock::Config::new().with_sysosc(Sysosc::Mhz4);
//!
//! // 32.768 kHz MCLK with SYSOSC off — the deepest RUN policy the part has. RTT stays readable and a
//! // plain re-flash still takes the device back, both of which were expected not to work here.
//! // `PA26` reads 1024 Hz rather than 1000: a 32 kHz counter has 32 ticks to divide.
//! const TREE: clock::Config = clock::Config::new()
//!     .with_mfclk(false)
//!     .with_mclk(MclkSource::Lfclk)
//!     .with_sysosc(Sysosc::Disabled);
//! ```

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output, Pull};
use embassy_mspm0::peripherals;
use embassy_mspm0::sysctl::LowPowerInstance;
use embassy_mspm0::sysctl::clock::{self, MclkSource, Sysosc};
use embassy_mspm0::tim::simple_pwm::{Config as PwmConfig, PwmPin, SimplePwm};
use embassy_mspm0::tim::{Channel, ClockSel, low_level};
use embassy_time::{Instant, TICK_HZ, Timer};
use panic_halt as _;

/// 1 MHz MCLK: SYSOSC at its 4 MHz operating point, divided by four.
///
/// The most it is possible to get wrong in one tree on this part — the operating point moves, `MDIV`
/// is only legal because of it, MFCLK has to go because it is mutually exclusive with the divider,
/// and the flash wait states come back down to zero.
const TREE: clock::Config = clock::Config::new()
    .with_mfclk(false)
    .with_sysosc(Sysosc::Mhz4)
    .with_mclk(MclkSource::Sysosc { divider: 4 });

/// Resolving in a `const` is what makes an out-of-range tree a build error rather than a panic in
/// `init`, and spells out what the recipe above is expected to produce.
const CLOCKS: clock::Clocks = match TREE.resolve() {
    Ok(clocks) => clocks,
    Err(_) => core::panic!("this tree does not resolve on this chip"),
};

const _: () = core::assert!(CLOCKS.mclk == 1_000_000);
const _: () = core::assert!(CLOCKS.mfclk == 0);
const _: () = core::assert!(CLOCKS.flash_wait == 0);

/// The instance doing the counting: `TIMG0` is the time driver's, `TIMG1` drives the PWM.
type Counter = peripherals::TIMG2;

/// Which bus clock the counter sees depends on the domain the instance sits in, so take it from the
/// instance rather than assuming ULPCLK.
const COUNTER_HZ: u32 = CLOCKS.bus_clock(<Counter as LowPowerInstance>::SLEEP.power_domain);

/// Window each measurement counts over.
const WINDOW_MS: u64 = 200;

/// Sized against the part's MCLK ceiling, **not** [`COUNTER_HZ`].
///
/// A wrapped 16-bit counter aliases modulo 65536, and the alias of a clock that came out too fast is
/// a plausible-looking slow reading — so sizing this off the expected rate makes the measurement lie
/// in exactly the case it exists to catch. No legal tree can outrun the ceiling, so nothing can wrap.
const PRESCALER: u16 = prescaler_for(embassy_mspm0::sysctl::MAX_MCLK_HZ);

/// Smallest power of two that fits the window's tick count in 16 bits.
///
/// 60000 rather than 65536 leaves room for the software either side of the window, which is counted
/// too.
const fn prescaler_for(hz: u32) -> u16 {
    let mut prescaler = 1u32;

    while (hz as u64) * WINDOW_MS / 1000 / prescaler as u64 > 60_000 {
        prescaler *= 2;
    }

    core::assert!(prescaler <= 256, "no prescaler fits this window; shorten WINDOW_MS");

    prescaler as u16
}

/// PWM output frequency. 1 kHz is reachable from every tree in the doc above, including the 32.768 kHz
/// one where the counter only has 32 ticks to divide.
const PWM_HZ: u32 = 1_000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = embassy_mspm0::Config::default();
    config.clock = TREE.build();
    let p = embassy_mspm0::init(config);

    let published = embassy_mspm0::sysctl::clocks();
    info!(
        "published: mclk {} ulpclk {} sysosc {} mfclk {} lfclk {}, {} flash wait states",
        published.mclk, published.ulpclk, published.sysosc, published.mfclk, published.lfclk, published.flash_wait,
    );
    info!(
        "counting the bus clock at {} Hz / {} over {} ms",
        COUNTER_HZ, PRESCALER, WINDOW_MS
    );

    let mut led = Output::new(p.PA0, Level::Low);

    let mut pwm = unwrap!(SimplePwm::new_2ch(
        p.TIMG1,
        Some(PwmPin::new(p.PA26, Pull::None)),
        None,
        PwmConfig {
            frequency: PWM_HZ,
            ..Default::default()
        },
    ));
    pwm.channel(Channel::Ch0).set_duty_percent(50);
    pwm.channel(Channel::Ch0).enable();
    pwm.start();

    let mut counter = low_level::Timer::new(
        p.TIMG2,
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

    // What the HAL believes, from the registers and the published tree. The measurement below is the
    // same number arrived at through the hardware, so the two disagreeing means the tree is not what
    // was programmed.
    info!(
        "counter ticks at {} Hz, PWM period is {} ticks",
        counter.tick_frequency(),
        pwm.max_duty(),
    );

    // A 16-bit counter reloading from its maximum wraps modulo `max + 1`.
    let wrap: u32 = counter.max_count().into();

    loop {
        led.toggle();

        let before: u32 = counter.counter().into();
        let start = Instant::now();

        Timer::after_millis(WINDOW_MS).await;

        let elapsed = start.elapsed().as_ticks();
        let after: u32 = counter.counter().into();

        let counted = after.wrapping_sub(before) & wrap;
        let measured = counted as u64 * PRESCALER as u64 * TICK_HZ / elapsed;

        // Signed, because a clock that came out slow is as interesting as one that came out fast.
        let deviation_permille = (measured * 1000 / COUNTER_HZ as u64) as i64 - 1000;

        info!(
            "{} prescaled ticks in {} LFCLK ticks: {} Hz measured against {} Hz published, {} permille off",
            counted, elapsed, measured, COUNTER_HZ, deviation_permille,
        );
    }
}
