//! Input capture measuring a PWM waveform the same board generates.
//!
//! **Wire BoosterPack 38 (PA26) to BoosterPack 36 (PA10).** TIMG1 drives PA26, TIMG4 captures PA10 on
//! each rising edge, and the logged period should match the expected tick count. TIMG4 is 16-bit, so
//! its clock is divided to 4 MHz to keep one period inside the 65536-tick range.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::gpio::Pull;
use embassy_mspm0::peripherals::TIMG4;
use embassy_mspm0::tim::Channel;
use embassy_mspm0::tim::input_capture::{
    CaptureEdge, CapturePin, Config as CaptureConfig, Filter, InputCapture, InterruptHandler,
};
use embassy_mspm0::tim::simple_pwm::{Config as PwmConfig, PwmPin, SimplePwm};
use panic_probe as _;

const FREQUENCY: u32 = 1_000;

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
        PwmConfig {
            frequency: FREQUENCY,
            ..Default::default()
        },
    ));

    pwm.channel(Channel::Ch0).set_duty_percent(50);
    pwm.channel(Channel::Ch0).enable();
    pwm.start();

    let mut capture = InputCapture::new_2ch(
        p.TIMG4,
        Some(CapturePin::new(p.PA10, Pull::Down, CaptureEdge::Rising, Filter::Ticks3)),
        None,
        Irqs,
        CaptureConfig {
            divider: 8,
            ..Default::default()
        },
    );

    let expected = capture.timer().tick_frequency() / FREQUENCY;
    info!(
        "capturing at {} Hz, expecting {} ticks per {} Hz period",
        capture.timer().tick_frequency(),
        expected,
        FREQUENCY
    );

    // Yields u16, so the subtraction below wraps the way the 16-bit counter does.
    let mut channel = capture.channel(Channel::Ch0);
    let mut previous: Option<u16> = None;

    loop {
        let now = channel.wait_for_capture().await;

        if let Some(previous) = previous {
            let ticks = u32::from(now.wrapping_sub(previous));
            let error = ticks as i32 - expected as i32;

            info!("period: {} ticks, {} off expected", ticks, error);
        }

        previous = Some(now);
    }
}
