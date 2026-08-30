//! Digital-to-analog converter (DAC12).
//!
//! A 12-bit voltage-output DAC. Write a code, get a voltage — the output is a fraction of the
//! reference, so [`Config::reference`] decides what a code is worth and nothing here converts to
//! volts on the caller's behalf.
//!
//! ```rust,ignore
//! let mut dac = Dac::new(p.DAC0, Config::new());
//!
//! dac.set_code(Dac::<DAC0>::MAX_CODE_12BIT / 2);   // half of the reference
//! ```
//!
//! # The output is a mux, not just an enable
//!
//! [`Config::output`] drives `CTL1.OPS`, and setting it connects the DAC to the OPA, the ADC, the
//! comparator **and** the `DAC_OUT` pin together — they share one node. With it set the pin cannot be
//! an input for any of them, and a DAC driving a pin something else already drives will fight it
//! rather than report anything. Leave it off and the DAC produces no output at all, which is the
//! reset state.
//!
//! # Reading the DAC back through the ADC
//!
//! `DAC_OUT` reaches the ADC as an internal channel, so a device with both can check the DAC against
//! itself with nothing wired. **The two have separate reference selections and the ratio is 1:1 only
//! if they agree.** A DAC on the supply read by an ADC on the 1.4 V internal reference saturates
//! above roughly a third of full scale, which reads like a broken driver rather than a mismatch.
//!
//! Putting both on the supply is the arrangement with nothing to get wrong: the ratio is one by
//! construction and there is no reference to wait for. Sharing the *internal* reference instead means
//! selecting [`PositiveReference::External`] here, which only works where the VREF module is buffered
//! out to the `VREF+` pin — true on the G families, false on the L-series parts, where the same
//! selection gets a floating pin.
//!
//! # Configure first, then enable
//!
//! §20.2.1 is explicit that changing a control register while the DAC is running "can cause
//! unpredictable results". [`Dac::new`] writes everything before it sets `CTL0.ENABLE` and then waits
//! for the module-ready flag, and there is deliberately no way to change the configuration of a
//! running driver — build a new one instead.

#![macro_use]

use embassy_hal_internal::{Peri, PeripheralType};
use mspm0_metapac::dac::vals;

use crate::pac::dac::Dac as Regs;

/// Conversion resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Resolution {
    /// Eight bits, so the largest useful code is 255.
    Bits8,

    /// Twelve bits, so the largest useful code is 4095.
    #[default]
    Bits12,
}

impl Resolution {
    /// The largest code this resolution expresses.
    ///
    /// A code above it is a caller error rather than a saturating write -- the hardware takes the low
    /// bits and the output is a wholly different voltage, so [`Dac::set_code`] refuses it.
    pub const fn max_code(self) -> u16 {
        match self {
            Self::Bits8 => 255,
            Self::Bits12 => 4095,
        }
    }

    const fn to_res(self) -> vals::Res {
        match self {
            Self::Bits8 => vals::Res::_8bits,
            Self::Bits12 => vals::Res::_12bits,
        }
    }
}

// The relationship a caller reaches for when converting between the two resolutions, and the one this
// crate has already got wrong once: §20.2.1 divides by **256 and 4096**, not by 255 and 4095, so full
// scale is one code short of the reference at either width and an eight-bit code is worth exactly
// sixteen twelve-bit ones. A test that divided by 255 put a systematic 16 counts into its error and it
// read as the DAC's. Nothing else re-measures this, so it is pinned here.
const _: () = {
    core::assert!(Resolution::Bits8.max_code() as u32 + 1 == 256);
    core::assert!(Resolution::Bits12.max_code() as u32 + 1 == 4096);
    core::assert!((Resolution::Bits12.max_code() as u32 + 1) / (Resolution::Bits8.max_code() as u32 + 1) == 16);
};

/// What a code is measured against at the top of the range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PositiveReference {
    /// The analog supply.
    ///
    /// Needs no external component and nothing to settle, and it is what makes a loopback against the
    /// ADC a ratio of one.
    #[default]
    Supply,

    /// The `VREF+` pin.
    ///
    /// **This is a pin, not the internal reference module.** On a device that buffers the internal
    /// reference out to that pin the two are the same node and this selects it; on one that does not,
    /// this selects a pin nothing is driving. Which of those a device is, is a per-device fact.
    External,
}

/// What a code is measured against at the bottom of the range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NegativeReference {
    /// Analog ground.
    #[default]
    Ground,

    /// The `VREF-` pin.
    External,
}

/// How a code is encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataFormat {
    /// Zero is the bottom of the range and the maximum code is the top.
    #[default]
    Binary,

    /// The code is signed, so zero sits at mid-scale.
    ///
    /// The same voltages either way -- this changes which number names them, which is worth having
    /// where the samples are already signed.
    TwosComplement,
}

/// What the output amplifier does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputBuffer {
    /// Buffered, which is what drives a load.
    #[default]
    Enabled,

    /// Off, with a pulldown on the output.
    ///
    /// The TRM calls this "ground"; the register description calls it a pulldown, which is the more
    /// precise of the two and the one this follows.
    DisabledPulldown,

    /// Off, with the output left floating.
    DisabledHighZ,
}

/// DAC configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// How many bits a code carries.
    pub resolution: Resolution,

    /// The top of the output range.
    pub positive_reference: PositiveReference,

    /// The bottom of the output range.
    pub negative_reference: NegativeReference,

    /// How a code is encoded.
    pub format: DataFormat,

    /// What the output amplifier does.
    pub output_buffer: OutputBuffer,

    /// Whether the DAC drives the shared output node at all.
    ///
    /// Off by default, which is the reset state and means the DAC converts but reaches nothing. See
    /// the module documentation -- this is a mux and it takes the `DAC_OUT` pin with it.
    pub output: bool,
}

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than dependent
    /// on this being inlined, which at `opt-level = "z"` has already failed once on a struct this
    /// size.
    pub const fn new() -> Self {
        Self {
            resolution: Resolution::Bits12,
            positive_reference: PositiveReference::Supply,
            negative_reference: NegativeReference::Ground,
            format: DataFormat::Binary,
            output_buffer: OutputBuffer::Enabled,
            output: false,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// DAC driver.
pub struct Dac<'d, T: Instance> {
    _peri: Peri<'d, T>,
    resolution: Resolution,
}

impl<'d, T: Instance> Dac<'d, T> {
    /// Claim the DAC and bring it up.
    ///
    /// Everything is written before the enable, and this returns once the module reports ready.
    pub fn new(peri: Peri<'d, T>, config: Config) -> Self {
        let r = T::regs();

        r.gprcm(0).rstctl().write(|w| {
            w.set_resetassert(true);
            w.set_resetstkyclr(true);
            w.set_key(vals::ResetKey::Key);
        });

        r.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });

        // The registers behind `PWREN` stay isolated for a few ULPCLK cycles and a write that lands in
        // that window is dropped. `tim::low_level::enable` carries the account.
        cortex_m::asm::delay(16);

        r.ctl1().write(|w| {
            w.set_refsp(match config.positive_reference {
                PositiveReference::Supply => vals::Refsp::Vdda,
                PositiveReference::External => vals::Refsp::Verefp,
            });
            w.set_refsn(match config.negative_reference {
                NegativeReference::Ground => vals::Refsn::Vssa,
                NegativeReference::External => vals::Refsn::Verefn,
            });
            w.set_ampen(matches!(config.output_buffer, OutputBuffer::Enabled));
            // **The field reads backwards.** `AMPHIZ` clear is high impedance and set is the
            // pulldown, so the name names the value it is *not*. Only read when the amplifier is off.
            w.set_amphiz(match config.output_buffer {
                OutputBuffer::DisabledPulldown => vals::Amphiz::Pulldown,
                OutputBuffer::Enabled | OutputBuffer::DisabledHighZ => vals::Amphiz::Hiz,
            });
            w.set_ops(if config.output {
                vals::Ops::Out0
            } else {
                vals::Ops::Noc0
            });
        });

        // The enable is a write of its own, after everything it depends on. §20.2.1 says a control
        // register changed while the DAC runs gives unpredictable results, so the order is the
        // contract rather than a preference.
        r.ctl0().write(|w| {
            w.set_res(config.resolution.to_res());
            w.set_dfm(match config.format {
                DataFormat::Binary => vals::Dfm::Binary,
                DataFormat::TwosComplement => vals::Dfm::TwosComp,
            });
            w.set_enable(true);
        });

        // Raised once when the core and the output buffer have settled. A spin rather than a wait:
        // there is no clock to size a delay against here, and the flag is the only thing that knows.
        while !r.cpu_int(0).ris().read().modrdyifg() {}

        Self {
            _peri: peri,
            resolution: config.resolution,
        }
    }

    /// Set the output code.
    ///
    /// Panics on a code the resolution cannot express: the hardware would take the low bits and
    /// output a different voltage, with nothing to say it had.
    pub fn set_code(&mut self, code: u16) {
        assert!(
            code <= self.resolution.max_code(),
            "code is larger than the resolution expresses"
        );

        T::regs().data0().write(|w| w.set_data_value(code));
    }

    /// The largest code this driver's resolution expresses.
    pub fn max_code(&self) -> u16 {
        self.resolution.max_code()
    }
}

impl<T: Instance> Drop for Dac<'_, T> {
    fn drop(&mut self) {
        let r = T::regs();

        r.ctl0().modify(|w| w.set_enable(false));

        r.gprcm(0).pwren().write(|w| {
            w.set_enable(false);
            w.set_key(vals::PwrenKey::Key);
        });
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> Regs;
}

/// DAC instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + crate::sysctl::LowPowerInstance + 'static {}

macro_rules! impl_dac_instance {
    ($inst:ident) => {
        impl crate::dac::SealedInstance for crate::peripherals::$inst {
            fn regs() -> crate::pac::dac::Dac {
                crate::pac::$inst
            }
        }

        impl crate::dac::Instance for crate::peripherals::$inst {}
    };
}
