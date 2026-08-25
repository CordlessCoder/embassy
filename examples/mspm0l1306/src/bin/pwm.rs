//! Two-channel PWM crossfading the LP-MSPM0L1306's RGB LED between red and blue.
//!
//! LED2 red (PA26) and blue (PA27) are TIMG1 CCP0 and CCP1. Fit jumpers J12 and J13, or scope
//! BoosterPack 38 and 37.

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

    let red = PwmPin::new(p.PA26, Pull::None);
    let blue = PwmPin::new(p.PA27, Pull::None);

    let config = Config::new().with_frequency(FREQUENCY);

    let mut pwm = unwrap!(SimplePwm::new_2ch(p.TIMG1, Some(red), Some(blue), config));

    let max = pwm.max_duty();
    info!(
        "TIMG1 PWM at {} Hz: {} ticks per period from a {} Hz counter",
        FREQUENCY,
        max,
        pwm.timer().tick_frequency()
    );

    // Both channels start held, so this is what puts them on the pins.
    pwm.channel(Channel::Ch0).enable();
    pwm.channel(Channel::Ch1).enable();

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
