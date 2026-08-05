//! Isolates what makes `UART3`'s TX line glitch across sleep, by sleep level and duration.
//!
//! Writes a marker on `UART3`/`PB2`, sleeps, repeats — walking every sleep level from plain `WFI` down
//! to STANDBY1, and three durations at each. Nothing is ever reconfigured, so any difference between
//! cases is the sleep itself.
//!
//! Two channels to probe:
//!
//! | Pin            | Signal                                                        |
//! |----------------|---------------------------------------------------------------|
//! | `PB2`          | UART3 TX — the marker bursts and whatever happens between      |
//! | `PB3` + `PB13` | high while asleep, low while awake — brackets the sleep window |
//!
//! The sleep window is what makes this readable: glitches on `PB2` can be attributed to sleep entry, the
//! sleep itself, or wake, by where they land against it. It is driven on two pins because which of them
//! a given board breaks out varies — probe whichever is available. Both emit ten fast pulses at startup,
//! so a flat line means the pin is not reaching the probe rather than the run being broken.
//!
//! `PA0` drives the on-board LED once per pass as a liveness check, but is not broken out.
//!
//! The marker is `0x55` bytes, an alternating bit pattern that is a clean square wave on an analyser
//! even when framing cannot be decoded. **The count identifies the case** — one byte for the first
//! level, six for the last — so cases stay distinguishable without decoding anything.
//!
//! `WFI` is the control: it never unpowers PD1, so if `PB2` is clean there and dirty everywhere else,
//! losing PD1 is the cause.
//!
//! It was not the cause. Every level transmits cleanly once the marker is flushed before sleeping — the
//! apparent glitching was frames truncated by sleep entry, and finding that turned up two HAL bugs:
//! `blocking_flush` waited on an inverted condition, and `UART_ERR_08` was scoped to three of the seven
//! affected families, leaving `STAT.BUSY` stuck high on this one. Kept as a regression check.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::peripherals;
use embassy_mspm0::sysctl::{LowPowerInstance, PowerDomain, SleepLevel, WakeGuard, clock};
use embassy_mspm0::uart::{Baud, ClockSel, Config, UartTx};
use embassy_time::Timer;
use panic_halt as _;

/// Each case guards one level deeper than the sleep it wants, since a guard blocks its own level and
/// everything below it. The first blocks all deep sleep, leaving a plain `WFI`; the last blocks nothing.
const CASES: &[(Option<SleepLevel>, &str)] = &[
    (Some(SleepLevel::Stop0), "wfi"),
    (Some(SleepLevel::Stop1), "stop0"),
    (Some(SleepLevel::Stop2), "stop1"),
    (Some(SleepLevel::Standby0), "stop2"),
    (Some(SleepLevel::Standby1), "standby0"),
    (None, "standby1"),
];

const DURATIONS_MS: &[u64] = &[20, 100, 500];

const MARKER: [u8; CASES.len()] = [0x55; CASES.len()];

/// UART3 is on the bus clock, whose rate depends on which power domain the instance sits in, so the
/// domain is taken from the instance rather than assumed.
const CLOCKS: clock::Clocks = clock::RESET_SETUP.clocks();
const UART3_DOMAIN: PowerDomain = <peripherals::UART3 as LowPowerInstance>::SLEEP.power_domain;
const BAUD: Baud = match Baud::solve(ClockSel::BusClk.frequency(&CLOCKS, UART3_DOMAIN), 9600) {
    Some(baud) => baud,
    None => core::panic!("9600 baud is not reachable from the bus clock"),
};

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    info!("re-flash window, starting in 5s");
    Timer::after_secs(5).await;

    let mut config = Config::default().with_baud(BAUD);
    config.clock_source = ClockSel::BusClk;
    let mut uart = unwrap!(UartTx::new_blocking(p.UART3, p.PB2, config));

    let mut asleep = Output::new(p.PB3, Level::Low);
    let mut asleep_alt = Output::new(p.PB13, Level::Low);
    let mut led = Output::new(p.PA0, Level::High);
    led.set_inversion(true);

    // Proves the sleep-window pins reach the probe before any of the measurements depend on them.
    for _ in 0..10 {
        asleep.set_high();
        asleep_alt.set_high();
        Timer::after_millis(50).await;
        asleep.set_low();
        asleep_alt.set_low();
        Timer::after_millis(50).await;
    }

    loop {
        led.toggle();

        for (case, (guard, name)) in CASES.iter().enumerate() {
            for duration in DURATIONS_MS {
                info!("{}: {} ms, {} marker bytes", name, duration, case + 1);
                unwrap!(uart.blocking_write(&MARKER[..case + 1]));
                // `blocking_write` only queues; sleeping before the shift register drains cuts the
                // marker mid-byte and is itself a source of edges on the line.
                unwrap!(uart.blocking_flush());

                let _guard = guard.map(WakeGuard::new);

                asleep.set_high();
                asleep_alt.set_high();
                Timer::after_millis(*duration).await;
                asleep.set_low();
                asleep_alt.set_low();
            }
        }
    }
}
