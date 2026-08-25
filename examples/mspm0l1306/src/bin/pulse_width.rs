//! Pulse-width capture measuring a PWM waveform the same board generates.
//!
//! **Wire BoosterPack 38 (PA26) to BoosterPack 36 (PA10).** TIMG1 drives PA26, TIMG4 measures the
//! high time on PA10, and each width should match what the PWM was asked for. One pin feeds both of
//! TIMG4's channels, so nothing is wired to its second one.
//!
//! The counter ticks at 1 MHz, so a width in ticks is a width in microseconds. The widths sweep from
//! a few microseconds to nearly the whole period: a narrow pulse and a wide one in the same period
//! are the corner a duty sweep clustered around the middle never reaches.
//!
//! Nothing here is symmetric. A 50% duty is its own complement and would pass with the two edges
//! swapped.
//!
//! The widest pulses leave only a microsecond or two before the next one starts, which is less than
//! the interrupt handler takes to read both capture registers. Those read as `counter_range - gap`
//! and are reported as a straddle — the point of including them is that the failure has a signature
//! a caller can reject rather than a plausible width.
//!
//! The glitch filter is off so the gap can be made short enough to reach that. `Filter::Ticks3` at
//! this tick rate rejects a gap below 3 us outright, and then no trailing edge arrives at all.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::gpio::Pull;
use embassy_mspm0::peripherals::TIMG4;
use embassy_mspm0::tim::Channel;
use embassy_mspm0::tim::input_capture::Filter;
use embassy_mspm0::tim::pulse_width::{Config as WidthConfig, InterruptHandler, PulseLevel, PulseWidth};
use embassy_mspm0::tim::simple_pwm::{Config as PwmConfig, PwmPin, SimplePwm};
use panic_probe as _;

/// 250 us, so a width in microseconds is a fraction of it with nothing to round.
const FREQUENCY: u32 = 4_000;
const PERIOD_US: u32 = 1_000_000 / FREQUENCY;

/// High times to ask for, in microseconds.
const WIDTHS_US: [u32; 8] = [5, 20, 40, 125, 210, 246, 247, 249];

bind_interrupts!(struct Irqs {
    TIMG4 => InterruptHandler<TIMG4>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut pwm = unwrap!(SimplePwm::new_2ch(
        p.TIMG1,
        Some(PwmPin::new(p.PA26, Pull::None)),
        None,
        PwmConfig::new().with_frequency(FREQUENCY),
    ));

    pwm.channel(Channel::Ch0).enable();
    pwm.start();

    let mut width = PulseWidth::new(
        p.TIMG4,
        p.PA10,
        Pull::Down,
        Irqs,
        // 32 MHz / 8 / 4, so one tick is one microsecond.
        WidthConfig::new()
            .with_divider(8)
            .with_prescaler(4)
            .with_level(PulseLevel::High)
            .with_filter(Filter::None),
    );

    let max_duty = pwm.channel(Channel::Ch0).max_duty();

    info!(
        "measuring at {} Hz over a {} us period, PWM resolution {} ticks",
        width.timer().tick_frequency(),
        PERIOD_US,
        max_duty
    );

    // Measured with nothing logged between the widths, so the interrupt-to-read latency is the
    // executor's and not defmt's.
    let mut measured = [0u32; WIDTHS_US.len()];
    let mut expected = [0u32; WIDTHS_US.len()];

    loop {
        for (index, requested) in WIDTHS_US.into_iter().enumerate() {
            pwm.channel(Channel::Ch0).set_duty_fraction(requested, PERIOD_US);

            // What the PWM can actually emit, which is what the capture should agree with.
            expected[index] = pwm.channel(Channel::Ch0).duty() * PERIOD_US / max_duty;

            // `set_duty` writes the live compare register, so the period the write lands in comes
            // out the wrong width and so can the one after it. Neither is the one to judge.
            width.wait_for_width().await;
            width.wait_for_width().await;

            measured[index] = u32::from(width.wait_for_width().await);
        }

        for (index, requested) in WIDTHS_US.into_iter().enumerate() {
            // A width can never exceed the period, so anything above it is the next pulse's leading
            // edge landing before this one could be read.
            if measured[index] > PERIOD_US {
                info!(
                    "asked {} us: straddled, {} us gap left too little to read the pair",
                    requested,
                    PERIOD_US - requested
                );
            } else {
                info!(
                    "asked {} us: measured {} us, expected {}, {} off",
                    requested,
                    measured[index],
                    expected[index],
                    measured[index] as i32 - expected[index] as i32
                );
            }
        }
    }
}
