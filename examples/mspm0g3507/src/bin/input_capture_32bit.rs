//! 32-bit input capture, on TIMG12 — the only 32-bit timer on this part.
//!
//! **Wire BoosterPack 37 (PA31) to BoosterPack 35 (PB13).** TIMG7 drives 100 Hz on PA31 and TIMG12
//! captures PB13 undivided. The logged period is around 320000 ticks, which does not fit in 16 bits:
//! a 16-bit counter would wrap five times between edges.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::bind_interrupts;
use embassy_mspm0::gpio::Pull;
use embassy_mspm0::peripherals::TIMG12;
use embassy_mspm0::tim::Channel;
use embassy_mspm0::tim::input_capture::{
    CaptureEdge, CapturePin, Config as CaptureConfig, Filter, InputCapture, InterruptHandler,
};
use embassy_mspm0::tim::simple_pwm::{Config as PwmConfig, PwmPin, SimplePwm};
use panic_probe as _;

const FREQUENCY: u32 = 100;

bind_interrupts!(struct Irqs {
    TIMG12 => InterruptHandler<TIMG12>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // TIMG7 is 16-bit, so 100 Hz needs the clock divided before the period fits its counter.
    let mut pwm = unwrap!(SimplePwm::new_2ch(
        p.TIMG7,
        None,
        Some(PwmPin::new(p.PA31, Pull::None)),
        PwmConfig {
            divider: 8,
            frequency: FREQUENCY,
            ..Default::default()
        },
    ));

    let max = pwm.max_duty();
    pwm.channel(Channel::Ch1).set_duty_percent(50);
    pwm.channel(Channel::Ch1).enable();
    pwm.start();

    info!("TIMG7 driving {} Hz on PA31, {} ticks per period", FREQUENCY, max);

    let mut capture = InputCapture::new_2ch(
        p.TIMG12,
        Some(CapturePin::new(p.PB13, Pull::Down, CaptureEdge::Rising, Filter::Ticks3)),
        None,
        Irqs,
        CaptureConfig::default(),
    );

    let expected = capture.timer().tick_frequency() / FREQUENCY;
    info!(
        "TIMG12 capturing at {} Hz, expecting {} ticks per period",
        capture.timer().tick_frequency(),
        expected
    );

    // Yields u32; against a 16-bit timer `Option<u32>` below would not compile, which is the point.
    let mut channel = capture.channel(Channel::Ch0);
    let mut previous: Option<u32> = None;

    loop {
        let now = channel.wait_for_capture().await;

        if let Some(previous) = previous {
            let ticks = now.wrapping_sub(previous);
            let error = ticks as i32 - expected as i32;

            info!("period: {} ticks, {} off expected", ticks, error);
        }

        previous = Some(now);
    }
}
