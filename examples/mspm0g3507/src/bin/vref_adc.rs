//! Converts one input against `VDDA` and then against the internal reference, and checks the ratio.
//!
//! No wiring, no analyser. **J9 must connect the TMP6131 thermistor to `PB24`**, the same jumper
//! `adc.rs` needs — any steady voltage below the reference would do as well.
//!
//! # What it proves
//!
//! The same input converted against two references gives counts in inverse proportion to those
//! references, so `count_vref / count_vdda` should be `VDDA / VREF` — about `3.3 / 2.5 = 1.32` on this
//! board. That number is only right if the reference is actually up at the moment of conversion, which
//! is what [`Vref::new`] is for: it does not return until the reference has settled.
//!
//! A reference that has not settled sits **below** its final value, and a low reference makes the
//! count **high**, so the failure has a direction — a ratio well above 1.32 is the shape to look for.
//!
//! # What it does not prove
//!
//! That the wait is *necessary*. Showing that needs the conversion issued before the reference is up,
//! which this driver deliberately makes unreachable; it would take register-level control and a
//! `unstable-pac` bench binary. What this checks is that the driver's contract holds — the first
//! conversion after `Vref::new` is as good as the hundredth.
//!
//! # Reading it
//!
//! `first` against `steady` is the settling check: they should agree. If `first` is high and later
//! readings fall towards `steady`, the startup wait is too short for this part.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::{self, Adc, AdcChannel, Conversion, Vrsel};
use embassy_mspm0::vref::{self, Vref};
use embassy_mspm0::{Config, bind_interrupts, peripherals};
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    ADC0 => adc::InterruptHandler<peripherals::ADC0>;
});

/// Readings taken after the first, to establish what settled looks like.
const STEADY_READINGS: u32 = 64;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut p = embassy_mspm0::init(Config::default());

    let mut adc = Adc::new_async(p.ADC0, Irqs, Default::default());
    let mut pin = p.PB24.reborrow_adc();
    let max_count = adc.resolution().max_count();

    info!(
        "VREF startup is {} ns, waited by {}",
        vref::STARTUP_NS,
        if vref::STARTUP_IS_TIMED {
            "counting cycles (VREF_ERR_01)"
        } else {
            "polling CTL1.READY"
        }
    );

    let against_vdda = adc.blocking_read(&mut pin, Conversion::default());

    // The reference is off until here, and off again the moment this is dropped.
    let vref = Vref::new(p.VREF, vref::Config::default());

    let internal = Conversion {
        vrsel: Vrsel::IntrefVssa,
        ..Default::default()
    };

    // The first conversion after the constructor returns is the one under test.
    let first = adc.blocking_read(&mut pin, internal);

    let mut total = 0u32;
    for _ in 0..STEADY_READINGS {
        total += adc.blocking_read(&mut pin, internal) as u32;
    }
    let steady = (total / STEADY_READINGS) as u16;

    info!(
        "counts: vdda {}, vref first {}, vref steady {} (full scale {})",
        against_vdda, first, steady, max_count
    );

    if against_vdda == 0 {
        error!("input reads zero against VDDA -- is J9 fitted?");
    } else if steady as u32 >= max_count {
        warn!("input clips against the reference; it is above VREF, so the ratio says nothing");
    } else {
        // In thousandths, because there is no FPU here and a runtime divide would pull in the
        // software divider this branch spent three commits removing.
        let ratio_milli = (steady as u32 * 1000) / against_vdda as u32;
        info!("vref/vdda count ratio {}/1000, expected about 1320", ratio_milli);

        // `first` and `steady` differing by more than a couple of counts means the reference was
        // still moving when the first conversion was taken.
        let drift = (first as i32 - steady as i32).abs();
        if drift <= 2 {
            info!("PASS: first reading within {} counts of steady", drift);
        } else {
            error!(
                "FAIL: first reading {} counts from steady ({} vs {}) -- startup wait too short",
                drift, first, steady
            );
        }
    }

    drop(vref);
    info!("reference powered down");

    loop {
        Timer::after_secs(60).await;
    }
}
