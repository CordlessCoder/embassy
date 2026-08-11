//! Reads the on-die temperature sensor. Needs no wiring and no external parts.
//!
//! The sensor is an internal ADC channel, so the whole of using it is selecting that channel. What
//! makes the reading absolute is a factory calibration code stored per unit, which
//! `adc::temp_calibration_code()` reads.
//!
//! Two settings here are load-bearing and both come from the datasheet's Temperature Sensor
//! section, not from the defaults:
//!
//! - **The sample window.** `tSET,TS` is 12.5 us on this part, and it is a minimum rather than a
//!   typical. The driver's default window is 6.25 us, which is ample for a pin and is not enough
//!   here -- a short window reads the sampling capacitor before it has charged and reports a
//!   temperature that is wrong by an amount nothing flags.
//!
//! - **The reference.** A conversion taken against a different reference from the factory
//!   calibration has to be rescaled before its code can be compared with the stored one, so `vrsel`
//!   is written out below rather than left to `Conversion::default()`. This part's datasheet gives
//!   the trim conditions as `VRSEL=0h (VDD = 3.3V)`, which is what the selection here matches.
//!
//!   **That has not been confirmed on this part, and the equivalent claim was wrong on an L1306** --
//!   whose datasheet says both things in two sections, and whose trim measured as the 1.4 V internal
//!   reference rather than VDD. The check takes one line and no wiring: work out what voltage
//!   `temp_calibration_code()` stands for under each candidate reference, and discard the one that
//!   is not a plausible sensor output. Getting it wrong is worth hundreds of degrees, not a few.
//!
//! Absolute accuracy is limited by the trim itself: the factory temperature is 30 C typical with a
//! 27 to 33 C spread, and the slope has its own tolerance. Expect a few degrees. Warming the
//! package with a fingertip is enough to see the reading move, which is the cheapest check that it
//! is measuring anything at all.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::{self, Adc, Config as AdcConfig, Conversion, TempSensor, Vrsel, temp_calibration_code};
use embassy_mspm0::{Config, bind_interrupts, peripherals};
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    ADC0 => adc::InterruptHandler<peripherals::ADC0>;
});

/// The supply this board runs at, which is what the conversion measures against.
///
/// The one figure the driver cannot supply, and on a part whose trim was taken against the supply
/// it scales the answer. A board running off something other than 3.3 V has to say so here.
const SUPPLY_MV: u32 = 3300;

/// Sample window, in ADC sample clock cycles.
///
/// `TempSensor::RECOMMENDED_SAMPLE_NS` is the figure to clear and the driver holds SAMPCLK at
/// 8 MHz, so a cycle is 125 ns and this part's requirement is 100 cycles. TI's own configuration
/// for this measurement asks for 50 us, and this follows it: the conversion is not on a hot path
/// and the margin costs nothing worth counting.
const SAMPLE_CYCLES: u16 = 400;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Config::default());

    let mut adc_config = AdcConfig::default();
    adc_config.sample_period_0 = SAMPLE_CYCLES.try_into().unwrap();
    let mut adc = Adc::new_async(p.ADC0, Irqs, adc_config);

    // The same sensor is on channel 12 of ADC1, so this reads it from either.
    let mut sensor = TempSensor;

    let mut conversion = Conversion::default();
    // Matches the reference the factory calibration was taken against.
    conversion.vrsel = Vrsel::VddaVssa;

    let resolution = adc.resolution();

    info!(
        "calibration code {} against {} mV, slope {} uV/C, trim {} C",
        temp_calibration_code(),
        TempSensor::CALIBRATION_REFERENCE_MV,
        TempSensor::SLOPE_UV_PER_C,
        TempSensor::TRIM_CELSIUS,
    );

    loop {
        let code = adc.irq_read(&mut sensor, conversion).await;
        let milli_c = TempSensor::celsius_millidegrees(code, resolution, SUPPLY_MV);

        // Reported in thousandths rather than with a decimal point, because placing one costs a
        // division by a thousand and this core has no instruction for it -- 476 bytes of
        // `compiler_builtins`, on a binary of about 4 kB, for the sake of the dot.
        info!("{} mC (code {})", milli_c, code);

        Timer::after_millis(500).await;
    }
}
