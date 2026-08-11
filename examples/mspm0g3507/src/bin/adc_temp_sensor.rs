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

/// The temperature the factory calibration was taken at.
///
/// `TSTRIM` from the datasheet: 30 C typical, and specified only as 27 to 33 C, which is the floor
/// on how accurate any of this can be.
const TRIM_DEG_C: i32 = 30;

/// Degrees Celsius per ADC count, with 16 fractional bits.
///
/// One count is 3300 mV / 4096 = 0.8057 mV, and the sensor's slope `TSc` is -1.8 mV/C, so a count
/// is 0.44759 C. The sign is applied where it is used: the sensor's output falls as the die warms,
/// so a code below the calibration code means a temperature above the trim temperature.
///
/// The whole calculation stays in this fixed-point form, which is what keeps it to multiplies and
/// shifts. ARMv6-M has no divide instruction and no `UMULL` for a reciprocal, so a division by a
/// constant that is not a power of two links a routine from `compiler_builtins` -- measured at
/// 476 bytes here, on a binary of about 4 kB, just to split the result for printing.
///
/// `round(0.4475911 * 65536)`. The rounding costs 0.003 C at the extremes of this sensor's range,
/// which is three orders of magnitude inside its accuracy.
const DEG_C_PER_COUNT_Q16: i32 = 29_333;

/// Sample window, in ADC sample clock cycles.
///
/// The driver holds SAMPCLK at 8 MHz, so a cycle is 125 ns and the datasheet's 12.5 us minimum is
/// 100 cycles. TI's own configuration for this measurement asks for 50 us, and this follows it:
/// the conversion is not on a hot path and the margin costs nothing worth counting.
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

    let calibration = temp_calibration_code();
    info!("calibration code: {} (the sensor's reading at 30 C)", calibration);

    loop {
        let code = adc.irq_read(&mut sensor, conversion).await;

        let delta = code as i32 - calibration as i32;
        let temp_q16 = (TRIM_DEG_C << 16) - delta * DEG_C_PER_COUNT_Q16;

        // Split for printing without dividing: the whole part is the top 16 bits, and scaling the
        // bottom 16 by 1000 puts three decimal places in the same top-16 position. Taking the
        // magnitude first keeps the fractional part meaningful below zero, where an arithmetic
        // shift would otherwise round the whole part away from the fraction.
        let sign = if temp_q16 < 0 { "-" } else { "" };
        let magnitude = temp_q16.unsigned_abs();
        let whole = magnitude >> 16;
        let thousandths = ((magnitude & 0xFFFF) * 1000) >> 16;

        info!(
            "{}{}.{:03} C (code {}, {} from calibration)",
            sign, whole, thousandths, code, delta
        );

        Timer::after_millis(500).await;
    }
}
