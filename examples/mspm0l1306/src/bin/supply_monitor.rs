//! Measure `VDD` through the ADC's supply monitor, and show what the same channel reads against the
//! wrong reference.
//!
//! No wiring. The supply monitor is a divider inside the chip on a fixed ADC channel.
//!
//! Two conversions of the same channel each pass, back to back. The first selects the internal
//! reference and reports the board's supply in millivolts. The second selects the supply as the
//! reference, which is what a caller gets by leaving [`Conversion`] alone, and prints the code next
//! to [`SupplyMonitor::ratiometric_code`] -- they match, and they go on matching whatever the board
//! is running at.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::{Adc, Conversion, SupplyMonitor, Vrsel};
use embassy_mspm0::vref::Vref;
use embassy_mspm0::{Config, vref};
use embassy_time::Timer;
use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Config::default());

    // 2.5 V, the default. A third of any supply this board runs at is inside it.
    let _vref = Vref::new(p.VREF, vref::Config::default());

    let mut adc = Adc::new_blocking(p.ADC0, Default::default());
    let resolution = adc.resolution();

    // The channel is the type. Nothing here names a channel number, which is per device: the same
    // monitor is channel 15 on this part and 31 on the larger ones.
    let mut monitor = SupplyMonitor;

    loop {
        let against_vref = adc.blocking_read(
            &mut monitor,
            Conversion {
                vrsel: Vrsel::IntrefVssa,
                ..Default::default()
            },
        );

        let against_supply = adc.blocking_read(&mut monitor, Conversion::default());

        info!(
            "VDD = {} mV (code {} against the 2.5 V reference)",
            SupplyMonitor::millivolts(against_vref, resolution, 2500),
            against_vref,
        );
        info!(
            "against the supply itself: code {}, which is {} at every supply",
            against_supply,
            SupplyMonitor::ratiometric_code(resolution),
        );

        Timer::after_millis(1000).await;
    }
}
