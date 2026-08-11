//! The comparator's 8-bit reference DAC, read back through the amplifier and the ADC.
//!
//! This is the path the COMP driver exists for on this family. The reference never leaves the die
//! here, so the comparator's DAC is the only internal source the amplifier has — every other
//! `NonInvertingInput` is a pin. It is also what the two-stage amplifier chain is built on, so a
//! fault here looks like a fault in the chain.
//!
//! No wiring. The check needs no voltmeter either: with the DAC referenced to `VDDA` and the ADC
//! converting against `VDDA`/`VSSA`, both ends scale with the supply and it cancels. The DAC puts out
//! `VDDA x (code + 1) / 256` and a 12-bit conversion of it should read `16 x (code + 1)` counts,
//! whatever the board is actually running at.
//!
//! # What this does not cover
//!
//! **The comparator's own inputs.** Both terminals reach pins and nothing on the die can drive one,
//! so a real threshold check needs an external stimulus and a jumper — worth doing at the bench,
//! where the board is in front of you, rather than from a pinout table.
//!
//! The TRM says an amplifier output can reach a comparator terminal internally, which would make a
//! wire-free check possible. The driver does not offer those mux positions: only the pin ones are
//! generated, and the per-device channel map is in a datasheet section that does not contain it.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::adc::{Adc, Conversion};
use embassy_mspm0::comp::{Comp, Config as CompConfig, ConfigError, DacCode, Reference, ReferenceSource};
use embassy_mspm0::opa::{Config as OpaConfig, NonInvertingInput, Opa};
use panic_halt as _;

/// Codes to sweep, spread across the range and kept off both rails.
///
/// Full scale is left out on purpose: there the DAC output is `VDDA` itself, and the amplifier
/// cannot drive its own supply rail, so a reading below prediction would be the amplifier behaving
/// correctly. It is checked for being the largest reading instead.
const CODES: [u8; 4] = [15, 63, 127, 191];

/// Counts allowed between prediction and reading.
///
/// Generous next to what the amplifier alone managed — the existing OPA measurements land within
/// five counts — because this stacks the DAC's own error, the amplifier's offset and the ADC's on
/// top of each other. Still far tighter than any of the ways this path fails: a dead DAC reads near
/// zero, an unselected mux position floats, and a stuck code does not move.
const TOLERANCE: u16 = 64;

/// Counts a 12-bit conversion should give for `code`, both ends being ratiometric to `VDDA`.
fn expected(code: u8) -> u16 {
    16 * (code as u16 + 1)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_mspm0::init(Default::default());

    let mut fails = 0;

    // The device metadata says whether this comparator implements the internal-reference positions.
    // This part does not, so asking for one has to be refused rather than selecting no reference at
    // all and producing a threshold of nothing. Compiled on every part, meaningful on this one.
    match Comp::new_reference_only(
        p.COMP0.reborrow(),
        Reference {
            source: ReferenceSource::Internal,
            ..Default::default()
        },
        CompConfig::default(),
    ) {
        Err(ConfigError::NoInternalReference) => info!("internal reference refused: ok"),
        Err(error) => {
            error!("internal reference: {}, which is not the refusal expected", error);
            fails += 1;
        }
        Ok(_) => {
            error!("internal reference: accepted, on a device that has none");
            fails += 1;
        }
    }

    // The transfer function, checked against itself before anything is measured against it. A
    // half-supply request has to come back as the code whose output is half of full scale.
    let half = DacCode::from_millivolts(1650, 3300);
    if half.to_bits() == 127 {
        info!("dac code for half scale: {} ok", half.to_bits());
    } else {
        error!("dac code for half scale: {}, want 127", half.to_bits());
        fails += 1;
    }

    let mut adc = Adc::new_blocking(p.ADC0, Default::default());

    {
        // Supply-referenced, so the DAC's full scale is whatever the board runs at and the ADC
        // measures against the same thing. The comparator is enabled with neither terminal driven;
        // its output is not read here and is not meaningful.
        let mut comp = unwrap!(Comp::new_reference_only(
            p.COMP0.reborrow(),
            Reference {
                source: ReferenceSource::Vdda,
                code: DacCode::ZERO,
                ..Default::default()
            },
            CompConfig::default(),
        ));

        let mut opa = Opa::new(p.OPA0.reborrow(), OpaConfig::default());
        let mut out = opa.buffer_int(NonInvertingInput::dac8());

        let mut previous = 0;

        for (index, code) in CODES.iter().enumerate() {
            comp.set_dac_code(DacCode::new(*code));

            let counts = adc.blocking_read(&mut out, Conversion::default());
            let want = expected(*code);

            if counts.abs_diff(want) <= TOLERANCE {
                info!("code {}: {} counts, want {} ok", *code, counts, want);
            } else {
                error!("code {}: {} counts, want {} +/- {}", *code, counts, want, TOLERANCE);
                fails += 1;
            }

            // A stuck DAC can still land inside the tolerance at one code. Rising readings across
            // the sweep are what says the code reached the hardware.
            if index > 0 && counts <= previous {
                error!("code {}: {} counts did not rise above {}", *code, counts, previous);
                fails += 1;
            }

            previous = counts;
        }

        // Full scale asks the DAC for the supply itself. The amplifier cannot reach its own rail, so
        // this is checked for going the right way rather than against a prediction.
        comp.set_dac_code(DacCode::FULL_SCALE);
        let full = adc.blocking_read(&mut out, Conversion::default());

        if full > previous {
            info!("full scale: {} counts, above {} ok", full, previous);
        } else {
            error!("full scale: {} counts, not above {}", full, previous);
            fails += 1;
        }
    }

    // The comparator was dropped at the end of that block, which powers it down. Enabling it again
    // is where a stale interrupt flag or a skipped settling wait would show, and it is the ordinary
    // case for an application that only wants the reference occasionally.
    {
        let comp = unwrap!(Comp::new_reference_only(
            p.COMP0.reborrow(),
            Reference {
                source: ReferenceSource::Vdda,
                code: DacCode::new(127),
                ..Default::default()
            },
            CompConfig::default(),
        ));

        let mut opa = Opa::new(p.OPA0.reborrow(), OpaConfig::default());
        let mut out = opa.buffer_int(NonInvertingInput::dac8());

        let counts = adc.blocking_read(&mut out, Conversion::default());
        let want = expected(127);

        if counts.abs_diff(want) <= TOLERANCE {
            info!("after a power cycle: {} counts, want {} ok", counts, want);
        } else {
            error!(
                "after a power cycle: {} counts, want {} +/- {}",
                counts, want, TOLERANCE
            );
            fails += 1;
        }

        // Read once more without touching the code, to separate "the DAC settled" from "the reading
        // happened to be taken while it was still moving".
        let again = adc.blocking_read(&mut out, Conversion::default());
        if again.abs_diff(counts) <= TOLERANCE / 4 {
            info!("repeat reading: {} counts ok", again);
        } else {
            error!(
                "repeat reading: {} counts against {}, so the output is still moving",
                again, counts
            );
            fails += 1;
        }

        // The code the constructor was given, read back out of the register rather than out of the
        // struct — the driver keeps no copy, so this is the hardware answering.
        if comp.dac_code() == DacCode::new(127) {
            info!("dac code reads back: ok");
        } else {
            error!("dac code reads back as {}, want 127", comp.dac_code().to_bits());
            fails += 1;
        }
    }

    if fails == 0 {
        info!("comp: all ok");
    } else {
        error!("comp: {} failed", fails);
    }
}
