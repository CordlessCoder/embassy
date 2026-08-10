//! Measures the OPA through its internal ADC channels, using only internal sources. No wiring.
//!
//! # What it proves
//!
//! Both of this family's fixed OPA-to-ADC routes (OPA0 to ADC0 channel 13, OPA1 to ADC1 channel 13),
//! the buffer and non-inverting PGA topologies, and a live gain change. Every input is internal —
//! analog ground or the 1.4 V reference — so each reading has a predicted value:
//!
//! - buffered ground reads near zero
//! - buffered VREF reads `1.4 / 3.3` of full scale
//! - the x2 PGA doubles that
//! - stepping the gain to x4 while running asks for 5.6 V, so the output clips near full scale
//!
//! The last stage drives `PA16` (`OPA1_OUT`) with the buffered reference, so a meter on that pin
//! should read 1.4 V during the run; the ADC checks it regardless. It also reads a buffer of `PB19`
//! (`OPA1_IN0+`, J3.23), which is informational only — the pin floats unless something is wired to it.
//!
//! **`PA16` only reaches J3.29 with `J15` shorted (1)-(2)**, which selects it over `PA18`. The
//! amplifier drives the pin either way, so every check here passes with the jumper off; it is the
//! meter reading that silently measures nothing.
//!
//! # Why there is no L-series twin
//!
//! Every input this uses is one the L series does not have: its `PSEL` carries neither ground
//! (position 8) nor a reachable internal reference (position 5 is the `VREF+` pin, which the L-series
//! reference does not drive). What it does have — the COMP's 8-bit DAC, and the other amplifier's
//! ladder top — needs a COMP driver and cascade support that do not exist yet. So an L-series check
//! wants those first rather than a weaker version of this.
//!
//! # What it does not prove
//!
//! Absolute accuracy: the bands are wide enough to absorb VREF and VDDA tolerance, so a few percent
//! of gain error would pass. Tightening that needs a known external voltage and the `OPAx_OUT` pin
//! measured directly.

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
/// amplifier's un-chopped offset; see the module docs for what that gives away.
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

    let mut adc0 = Adc::new_blocking(p.ADC0, Default::default());
    let mut adc1 = Adc::new_blocking(p.ADC1, Default::default());
    let max_count = adc0.resolution().max_count() as u16;

    let mut vref_config = vref::Config::default();
    vref_config.voltage = Voltage::Volts1_4;
    let vref = Vref::new(p.VREF, vref_config);

    let expected_vref = (VREF_MV * max_count as u32 / VDDA_MV) as u16;
    let mut failed = 0u32;

    let mut opa0 = Opa::new(p.OPA0, Default::default());
    let mut opa1 = Opa::new(p.OPA1, Default::default());

    // Buffered ground: the offset alone, gained x1.
    {
        let mut out = opa0.buffer_int(NonInvertingInput::ground());
        let r = adc0.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA0 buffer(ground) on ADC0", r, 0) as u32;
    }

    // Buffered VREF, through both of the family's OPA-to-ADC routes.
    {
        let mut out = opa0.buffer_int(NonInvertingInput::vref());
        let r = adc0.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA0 buffer(vref) on ADC0", r, expected_vref) as u32;
    }
    {
        let mut out = opa1.buffer_int(NonInvertingInput::vref());
        let r = adc1.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA1 buffer(vref) on ADC1", r, expected_vref) as u32;
    }

    // x2 PGA, then a live gain step to x4, which asks for 5.6 V and must clip near the rail.
    {
        let mut out = opa0.pga_int(NonInvertingInput::vref(), Gain::X2);
        let r = adc0.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA0 pga x2(vref) on ADC0", r, expected_vref * 2) as u32;

        out.set_gain(Gain::X4);
        // tSETTLE is single-digit microseconds; a handful of cycles at any MCLK covers it.
        cortex_m::asm::delay(512);
        let r = adc0.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA0 pga x4(vref) clipped", r, max_count) as u32;
    }

    // The reference again, but out to the OPA1_OUT pin as well as to the ADC.
    {
        let mut out = opa1.buffer_ext(NonInvertingInput::vref(), p.PA16);
        let r = adc1.blocking_read(&mut out, Conversion::default());
        failed += !check("OPA1 buffer(vref) driving PA16", r, expected_vref) as u32;
    }

    // A pin input, floating unless wired: no expectation, only the mux path.
    {
        let mut out = opa1.buffer_int(p.PB19);
        let r = adc1.blocking_read(&mut out, Conversion::default());
        info!("OPA1 buffer(PB19, floating) read {} (informational)", r);
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
