//! Checks that a PD1 UART is still configured after deep sleep.
//!
//! `UART3` is in PD1, which SYSCTL disables on entry to STOP and STANDBY. The TRM says it is re-enabled
//! on exit with its configuration intact, which is why `SleepInfo::floor_to_keep_configured` lets an
//! instance like this sleep. This is the check: configure `UART3` once, deep sleep, then write to it on
//! the other side without touching its configuration again.
//!
//! `UART3` cannot do the waking — a PD1 instance is unpowered in deep sleep and cannot see the start
//! bit, which the HAL rejects as `ConfigError::NoDeepSleepWake`. `UART1` (PD0) is the wake source.
//!
//! Run `uart3_retention_host` on an LP-MSPM0L1306 as the other side, wired:
//!
//! | L1306        | G3507             |            |
//! |--------------|-------------------|------------|
//! | `PA10` (TX)  | `PB7`  (UART1 RX) | wakes it   |
//! | `PA1`  (RX)  | `PB2`  (UART3 TX) | the answer |
//! | `GND`        | `GND`             |            |
//!
//! The L1306 is the observer rather than defmt: an attached RTT session holds the device out of deep
//! sleep, so the thing under test does not happen while something is watching it that way.
//!
//! The time driver wakes the device about once a second on its own, so a sleep is not proof on its own
//! that the UART did the waking. The host's reply timeout is what separates the two.
//!
//! Retention is confirmed two ways: the register comparison below, and a logic analyser on `PB2`
//! showing the post-wake bytes going out correctly at 9600.
//!
//! The `blocking_flush` after the reply is load-bearing. `blocking_write` returns once the last byte is
//! queued, so without it the loop reaches the next await and deep sleep cuts the frame mid-byte. That
//! looks like the TX line glitching during sleep and is easy to mistake for the power domain dropping
//! the pin, which is what `uart3_sleep_glitch` was written to rule out.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Level, Output};
use embassy_mspm0::sysctl::{LowPowerInstance, PowerDomain, clock};
use embassy_mspm0::uart::{self, Baud, BufferedUartRx, ClockSel, Config, UartTx};
use embassy_mspm0::{bind_interrupts, pac, peripherals};
use embassy_time::Timer;
use embedded_io_async::Read;
use panic_halt as _;

bind_interrupts!(
    struct Irqs {
        UART1 => uart::BufferedInterruptHandler<peripherals::UART1>;
    }
);

const BAUD: u32 = 9600;

/// Both instances run from the bus clock, whose rate depends on the power domain each sits in, so
/// the dividers are solved per instance rather than shared.
const CLOCKS: clock::Clocks = clock::RESET_SETUP.clocks();

const UART3_DOMAIN: PowerDomain = <peripherals::UART3 as LowPowerInstance>::SLEEP.power_domain;
const ANSWER_BAUD: Baud = match Baud::solve(
    ClockSel::BusClk,
    ClockSel::BusClk.frequency(&CLOCKS, UART3_DOMAIN),
    BAUD,
) {
    Some(baud) => baud,
    None => core::panic!("this baud rate is not reachable from UART3's bus clock"),
};

const UART1_DOMAIN: PowerDomain = <peripherals::UART1 as LowPowerInstance>::SLEEP.power_domain;
const WAKE_BAUD: Baud = match Baud::solve(
    ClockSel::BusClk,
    ClockSel::BusClk.frequency(&CLOCKS, UART1_DOMAIN),
    BAUD,
) {
    Some(baud) => baud,
    None => core::panic!("this baud rate is not reachable from UART1's bus clock"),
};

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // The instance under test. Configured here and never again.
    //
    // On the bus clock rather than the default MFCLK, which is off in STANDBY and has to restart. That
    // is not on its own enough to make the post-wake write come out at the right rate — see the note at
    // the top about what the register check cannot see.
    let mut answer_config = Config::default().with_baud(ANSWER_BAUD);
    answer_config.clock_source = ClockSel::BusClk;
    let mut answer = unwrap!(UartTx::new_blocking(p.UART3, p.PB2, answer_config));

    // The wake source has to be a PD0 instance on the bus clock: the asynchronous clock request only
    // speeds the MCLK/ULPCLK tree back up, so an LFCLK-sourced receiver cannot frame at all.
    let mut wake_config = Config::default().with_baud(WAKE_BAUD);
    wake_config.clock_source = ClockSel::BusClk;
    wake_config.low_power_rx_wake = true;

    let mut rx_buf = [0u8; 32];
    let mut wake = unwrap!(BufferedUartRx::new(p.UART1, p.PB7, Irqs, &mut rx_buf, wake_config));

    // Toggles on every wake, so the wake path can be confirmed by eye even with no probe attached and
    // nothing coming back over UART3.
    let mut led = Output::new(p.PA0, Level::High);
    led.set_inversion(true);

    unwrap!(answer.blocking_write(b"boot\n"));
    unwrap!(answer.blocking_flush());
    info!("armed, going to sleep between bytes");

    let expected = uart3_registers();
    let mut buf = [0u8; 1];

    loop {
        // The executor deep-sleeps here until the L1306 sends a byte.
        unwrap!(wake.read(&mut buf).await);
        led.toggle();

        // Nothing has reconfigured UART3 since boot, so any difference here is deep sleep's doing.
        if uart3_registers() != expected {
            error!("UART3 did not come back configured");

            loop {
                led.toggle();
                Timer::after_millis(100).await;
            }
        }

        // Only observable with the return line connected; the register check above stands without it.
        unwrap!(answer.blocking_write(b"woke"));
        unwrap!(answer.blocking_write(&buf));
        unwrap!(answer.blocking_write(b"\n"));
        // `blocking_write` returns once the last byte is queued, so without this the loop sleeps again
        // while the shift register is still going and the reply is truncated mid-byte.
        unwrap!(answer.blocking_flush());
    }
}

/// The UART3 state that has to survive deep sleep: whether it is enabled, the framing, the baud
/// divisors, and the clock source.
fn uart3_registers() -> [u32; 5] {
    let r = pac::UART3;

    [
        r.ctl0().read().0,
        r.lcrh().read().0,
        r.ibrd().read().0,
        r.fbrd().read().0,
        r.clksel().read().0,
    ]
}
