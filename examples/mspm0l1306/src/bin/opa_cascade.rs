//! Two amplifiers chained, re-ordered and un-chained between measurements.
//!
//! The topology a two-stage analog front end uses: one amplifier takes a sensor pin, the second
//! amplifies the first's gain-ladder top, and neither output reaches a pin — the signal never leaves
//! the die. Two x2 stages give x4.
//!
//! What this is really demonstrating is that the arrangement is not fixed at construction. An
//! application alternating between two sensors reconfigures per measurement: chain one way, chain the
//! other, or run both amplifiers independently, without giving up either peripheral.
//!
//! Wiring: a voltage on `PA25` (`OPA0_IN0+`) and another on `PA18` (`OPA1_IN0+`). Both must sit
//! inside the amplifier's input range at the gain in use — x4 of anything above a quarter of the
//! supply clips.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::{Adc, Conversion};
use embassy_mspm0::opa::{Config, Gain, Opa, OpaPair, Stage};
use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_mspm0::init(Default::default());

    let mut adc = Adc::new_blocking(p.ADC0, Default::default());

    // Both amplifiers, constructed the ordinary way. The pair borrows nothing and owns both.
    let a = Opa::new(p.OPA0, Config::default());
    let b = Opa::new(p.OPA1, Config::default());
    let mut front = OpaPair::new(a, b);

    // Reborrowed rather than consumed, so each pin can drive a different arrangement below. The
    // input a pin becomes is `Copy` too, so an application that builds one per sensor up front can
    // apply either without holding the pin at all.
    let mut sensor_a = p.PA25;
    let mut sensor_b = p.PA18;

    info!("opa_cascade: start");

    // Chained: OPA0 takes its pin at x2, OPA1 amplifies OPA0's ladder top at x2. Output is OPA1's.
    {
        let mut out = front.chain_a_into_b(sensor_a.reborrow(), Stage::Pga(Gain::X2), Stage::Pga(Gain::X2));
        let counts = adc.blocking_read(&mut out, Conversion::default());
        info!("chained a->b, x4 total: {} counts", counts);
    }

    // The other way round, which the same hardware supports: OPA1 takes its pin, OPA0 amplifies it.
    {
        let mut out = front.chain_b_into_a(sensor_b.reborrow(), Stage::Pga(Gain::X2), Stage::Pga(Gain::X2));
        let counts = adc.blocking_read(&mut out, Conversion::default());
        info!("chained b->a, x4 total: {} counts", counts);
    }

    // Un-chained: both amplifiers on their own sensor, sampled one after the other.
    {
        let (mut first, mut second) = front.independent(
            sensor_a.reborrow(),
            Stage::Pga(Gain::X2),
            sensor_b.reborrow(),
            Stage::Buffer,
        );

        let a_counts = adc.blocking_read(&mut first, Conversion::default());
        let b_counts = adc.blocking_read(&mut second, Conversion::default());
        info!("independent: a x2 {} counts, b buffered {} counts", a_counts, b_counts);
    }

    // The upstream stage of a chain outlives its output handle, so release it explicitly before a
    // long idle. Dropping the pair does the same.
    front.disable();

    info!("opa_cascade: done");
}
