//! Measures the OPA through its internal ADC channels, using only internal sources. No wiring.
//!
//! The L-family twin of the G3507 `opa` example: this family routes both amplifiers to ADC0, on
//! channels 12 (OPA0) and 13 (OPA1), so the two reads below cover both routes. Predictions as
//! there: buffered ground near zero, buffered VREF at `1.4 / 3.3` of full scale, the x2 PGA double
//! that, and a live gain step to x4 clipping near full scale.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::{Adc, Conversion};
use embassy_mspm0::opa::{Gain, NonInvertingInput, Opa};
use embassy_mspm0::vref::{self, Voltage, Vref};
use embassy_time::Timer;
use panic_halt as _;

/// Nominal VDDA on the LaunchPad, in millivolts. The ADC converts against it.
const VDDA_MV: u32 = 3300;

/// The reference voltage buffered and amplified below, in millivolts.
const VREF_MV: u32 = 1400;

/// Half-width of every acceptance band, in counts. Covers VREF and VDDA tolerance plus the
/// amplifier's un-chopped offset.
const BAND: u16 = 150;

fn check(name: &str, reading: u16, expected: u16) -> bool {
    let ok = reading.abs_diff(expected) <= BAND;
    if ok {
        info!("PASS: {} read {} (expected {} +/- {})", name, reading, expected, BAND);
    } else {
        error!("FAIL: {} read {} (expected {} +/- {})", name, reading, expected, BAND);
    }
    ok
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut adc = Adc::new_blocking(p.ADC0, Default::default());
    let max_count = adc.resolution().max_count() as u16;

    let mut vref_config = vref::Config::default();
    vref_config.voltage = Voltage::Volts1_4;
    let vref = Vref::new(p.VREF, vref_config);

    let expected_vref = (VREF_MV * max_count as u32 / VDDA_MV) as u16;
    let mut failed = 0u32;

    let mut opa0 = Opa::new(p.OPA0, Default::default());
    let mut opa1 = Opa::new(p.OPA1, Default::default());

    // Buffered ground: the offset alone, gained x1. Channel 12 is OPA0's route on this family.
    {
        let mut out = opa0.buffer_int(NonInvertingInput::ground());
        let r = adc.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA0 buffer(ground) on ADC0", r, 0) as u32;
    }

    // Buffered VREF through OPA1, the channel 13 route.
    {
        let mut out = opa1.buffer_int(NonInvertingInput::vref());
        let r = adc.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA1 buffer(vref) on ADC0", r, expected_vref) as u32;
    }

    // x2 PGA, then a live gain step to x4, which asks for 5.6 V and must clip near the rail.
    {
        let mut out = opa0.pga_int(NonInvertingInput::vref(), Gain::X2);
        let r = adc.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA0 pga x2(vref) on ADC0", r, expected_vref * 2) as u32;

        out.set_gain(Gain::X4);
        // tSETTLE is single-digit microseconds; a handful of cycles at any MCLK covers it.
        cortex_m::asm::delay(512);
        let r = adc.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA0 pga x4(vref) clipped", r, max_count) as u32;
    }

    drop(vref);

    if failed == 0 {
        info!("all OPA checks passed");
    } else {
        error!("{} OPA checks failed", failed);
    }

    loop {
        Timer::after_secs(60).await;
    }
}
