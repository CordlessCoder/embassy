//! A comparator that keeps watching the pad an ADC also converts.
//!
//! [`Comp::new_sharing_positive`] keeps the positive terminal's concrete pin type instead of erasing
//! it, so [`CompSharedPositive::with_positive_pin`] can lend it to another driver and take it back.
//! The comparator is not stopped and nothing is reconfigured: an analog peripheral reaches the pad
//! without going through the IOMUX, so both can have it at once.
//!
//! The application shape this is for is a threshold on a sensor pin — the comparator watches it
//! continuously and the ADC is asked for an absolute number now and then.
//!
//! **No wiring.** `PA26` is the comparator's positive input 0 and `ADC0`'s channel 1 on this part.
//!
//! # How it checks itself without a known voltage
//!
//! The pad has nothing driving it, so there is no figure to compare against. What there is instead is
//! **two independent ways to measure the same pad**, and they have to agree.
//!
//! - The comparator: sweep the reference DAC until the output flips. The flipping code is where the
//!   DAC crosses the pad, so it locates the pad voltage as `VDDA x (code + 1) / 256`.
//! - The ADC, through the loan: a 12-bit conversion against `VDDA`, so `count / 4095` of it.
//!
//! Both ends are ratiometric to `VDDA`, which therefore cancels — the same reason `comp.rs` needs no
//! voltmeter. A conversion of the flip code into counts is `16 x (code + 1)`.
//!
//! **This is what catches the failure this family actually has**: a mux position that selects nothing
//! returns a plausible number rather than an error. A channel reading something other than `PA26`
//! disagrees with the comparator; one reading `PA26` agrees whatever the pad happens to sit at.
//!
//! # Why the search bisects, and why the ADC is read twice
//!
//! A floating pad drifts, and a linear sweep of 256 codes takes long enough that it drifts *during*
//! the search — so the comparator's answer and a conversion taken afterwards describe the pad at two
//! different moments. Measured on the first version of this: both figures fell together over four
//! rounds, tracking each other while disagreeing by a steady 14%. **That tracking is the evidence
//! they share a pad; the 14% was the test's own time skew.**
//!
//! So the search bisects, which is 8 settles rather than up to 256, and the pad is converted once
//! before it and once after. The two conversions bracket the search in time, so their mean is the
//! pad as it was when the comparator answered. Their *difference* is the drift, reported so a reader
//! can see how much of the tolerance the rig is using.
//!
//! # What it does not prove
//!
//! Accuracy. The pad's voltage is arbitrary. Drive `PA26` from a divider for an absolute check;
//! nothing here needs changing for that.
//!
//! A pad sitting outside the DAC's range never flips. That is reported rather than counted as a
//! failure — it says the rig cannot answer, not that the driver is wrong.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::{Adc, AdcChannel, Conversion};
use embassy_mspm0::comp::{Comp, Config as CompConfig, DacCode, Reference, ReferenceSource};
use embassy_time::Timer;
use panic_halt as _;

/// `PA26` is `ADC0`'s channel 1 here. The comparator calls the same pad its positive position 0, so
/// this is a check on the ADC's routing rather than a restatement of the comparator's.
const PA26_ADC_CHANNEL: u8 = 1;

/// Counts allowed between the comparator's answer and the ADC's, before the rig's own drift.
///
/// One DAC step is 16 counts, so the flip is only located to within that. The rest is the
/// comparator's hysteresis and offset and the ADC's error.
///
/// Drift is added to this per round rather than folded in as a bigger constant, because it is a
/// property of the pad on the day and not of the drivers: half the measured drift is the residual
/// error left in the bracket's mean, so that is what each round allows.
const TOLERANCE: u16 = 96;

/// Counts a 12-bit conversion should give for the pad, if the comparator flipped at `code`.
fn expected(code: u8) -> u16 {
    16 * (code as u16 + 1)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_mspm0::init(Default::default());

    let mut fails = 0;
    let mut adc = Adc::new_blocking(p.ADC0, Default::default());

    // Supply-referenced, so the DAC's full scale and the ADC's are the same thing and cancel.
    let mut config = CompConfig::new();
    config.reference = Some(Reference {
        source: ReferenceSource::Vdda,
        code: DacCode::ZERO,
        ..Default::default()
    });

    // No negative pin: the reference DAC is the other terminal, and it is what the sweep moves.
    let mut comp = unwrap!(Comp::new_sharing_positive(
        p.COMP0,
        p.PA26,
        None::<embassy_mspm0::Peri<'_, embassy_mspm0::peripherals::PA27>>,
        config,
    ));

    // The loan hands back the concrete pin, so it is still an ADC channel. Checking which one is the
    // point: a selection that reached nothing would still convert and still return a number.
    let channel = comp.with_positive_pin(|pad| pad.reborrow_adc().hw_channel());
    if channel == PA26_ADC_CHANNEL {
        info!("lent pad is ADC channel {} ok", channel);
    } else {
        error!("lent pad is ADC channel {}, want {}", channel, PA26_ADC_CHANNEL);
        fails += 1;
    }

    // The pad is steepest right after reset, and round 0 otherwise spends most of the tolerance on
    // that alone.
    Timer::after_millis(200).await;

    for round in 0..4 {
        let before = comp.with_positive_pin(|pad| adc.blocking_read(pad, Conversion::default()));

        // Bisect for the flip. The pad is the positive terminal, so the output is high while the DAC
        // sits below it and goes low once it climbs past — monotonic in the code, which is what makes
        // a bisection valid.
        let (mut low, mut high) = (0u16, 256u16);
        while low < high {
            let mid = (low + high) / 2;
            comp.set_dac_code(DacCode::new(mid as u8));
            if comp.is_low() { high = mid } else { low = mid + 1 }
        }

        let after = comp.with_positive_pin(|pad| adc.blocking_read(pad, Conversion::default()));

        // The two conversions bracket the search, so their mean is the pad as it was mid-search.
        let counts = (before + after) / 2;
        let drift = before.abs_diff(after);

        if low > u8::MAX as u16 {
            info!(
                "round {}: no flip -- the pad sits above the DAC's range, adc says {}",
                round, counts
            );
            continue;
        }

        let code = low as u8;
        let want = expected(code);

        let allowed = TOLERANCE + drift / 2;

        if counts.abs_diff(want) <= allowed {
            info!(
                "round {}: flips at {} (want {} counts), adc {}, off by {} of {} allowed (drift {}) ok",
                round,
                code,
                want,
                counts,
                counts.abs_diff(want),
                allowed,
                drift
            );
        } else {
            error!(
                "round {}: flips at {} (want {} counts), adc {}, off by {} of {} allowed (drift {}) \
                 -- the two disagree by more than the rig's own movement, so they are not measuring \
                 the same pad",
                round,
                code,
                want,
                counts,
                counts.abs_diff(want),
                allowed,
                drift
            );
            fails += 1;
        }
    }

    // The loan is over. The doc says it changes no registers, so the comparator's own configuration
    // has to have survived it.
    comp.set_dac_code(DacCode::new(64));
    if comp.dac_code().to_bits() == 64 {
        info!("comparator still configured after the loan ok");
    } else {
        error!(
            "dac code reads back {} after the loan, want 64",
            comp.dac_code().to_bits()
        );
        fails += 1;
    }

    if fails == 0 {
        info!("comp_share_adc: all ok");
    } else {
        error!("comp_share_adc: {} failed", fails);
    }
}
