//! Two-channel PWM crossfading the LP-MSPM0G3507's RGB LED between red and green.
//!
//! RGB red (PB26) and green (PB27) are TIMG6 CCP0 and CCP1.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::Pull;
use embassy_mspm0::tim::Channel;
use embassy_mspm0::tim::simple_pwm::{Config, PwmPin, SimplePwm};
use embassy_time::Timer;
use panic_halt as _;

const FREQUENCY: u32 = 1_000;
const STEPS: u32 = 100;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let red = PwmPin::new(p.PB26, Pull::None);
    let green = PwmPin::new(p.PB27, Pull::None);

    let config = Config {
        frequency: FREQUENCY,
        ..Default::default()
    };

    let mut pwm = unwrap!(SimplePwm::new_2ch(p.TIMG6, Some(red), Some(green), config));

    let max = pwm.max_duty();
    info!(
        "TIMG6 PWM at {} Hz: {} ticks per period from a {} Hz counter",
        FREQUENCY,
        max,
        pwm.timer().tick_frequency()
    );

    pwm.start();

    loop {
        for step in 0..=STEPS {
            pwm.channel(Channel::Ch0).set_duty_fraction(step, STEPS);
            pwm.channel(Channel::Ch1).set_duty_fraction(STEPS - step, STEPS);
            Timer::after_millis(20).await;
        }

        for step in 0..=STEPS {
            pwm.channel(Channel::Ch0).set_duty_fraction(STEPS - step, STEPS);
            pwm.channel(Channel::Ch1).set_duty_fraction(step, STEPS);
            Timer::after_millis(20).await;
        }
    }
}
