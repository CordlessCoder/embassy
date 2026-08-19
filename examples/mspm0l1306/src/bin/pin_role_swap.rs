//! One pin alternating between a GPIO output and a timer output at run time.
//!
//! A board that wires two functions to the same pin — a sounder driven by PWM and a pulse the
//! firmware drives directly, say — needs the pin to change role while the program runs, not once at
//! startup.
//!
//! [`SimplePwm`](embassy_mspm0::tim::simple_pwm::SimplePwm) cannot do this, and the borrow checker
//! says so: it takes the pin for its whole lifetime, which is what stops one pin reaching two
//! peripherals by accident. So the timer is driven at the low level, where a channel is configured
//! without a pin, and the pin stays in one [`Flex`] that decides which function reaches the pad.
//!
//! The timer runs throughout and nothing about it is reconfigured on a swap. Only the pad's function
//! number changes.
//!
//! PA26 is LED2's red channel on the LP-MSPM0L1306, and also TIMG1 CCP0. Fit jumper J12 to see it.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Flex, PfType, Pull};
use embassy_mspm0::peripherals::TIMG1;
use embassy_mspm0::tim::low_level::{Config as TimerConfig, Timer};
use embassy_mspm0::tim::{Ch0, Channel, CountingMode, TimerPin};
use embassy_time::Timer as Delay;
use panic_halt as _;

const FREQUENCY: u32 = 1_000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut p = embassy_mspm0::init(Default::default());

    // The function number for this pin as TIMG1 CCP0, taken from the pin trait rather than written
    // out: the pin already knows which function reaches which peripheral.
    let ccp0_pf = TimerPin::<TIMG1, Ch0>::pf_num(&*p.PA26);

    let mut timer = Timer::new(p.TIMG1.reborrow(), TimerConfig::default());
    unwrap!(timer.set_frequency(FREQUENCY));
    timer.setup_pwm_channel(Channel::Ch0, CountingMode::EdgeAlignedUp);

    // Duty rather than a bare compare write, and the output released: 0% and 100% live in a
    // forced-output override that `set_compare` cannot lift, and the channel starts at 0% with its
    // output held. Writing the compare alone leaves the pin dead with every other register correct.
    timer.set_pwm_duty(Channel::Ch0, timer.pwm_max_duty() / 2);
    timer.set_output_enabled(Channel::Ch0, true);
    timer.start();

    let mut pin = Flex::new(p.PA26.reborrow());

    loop {
        info!("timer drives the pin");
        pin.set_as_af(ccp0_pf, PfType::output(Pull::None, false));
        Delay::after_millis(1000).await;

        info!("the application drives the pin");
        pin.set_as_output();
        for _ in 0..5 {
            pin.set_high();
            Delay::after_millis(100).await;
            pin.set_low();
            Delay::after_millis(100).await;
        }
    }
}
