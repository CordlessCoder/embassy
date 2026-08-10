//! Operational amplifier (OPA)
//!
//! A zero-drift, chopper-stabilized amplifier with a programmable gain ladder. This driver exposes
//! the topologies that need no external components (SLAU846 §21.2.7, SLAU847 §17.2.7):
//!
//! - **Buffer** (unity gain follower): [`Opa::buffer_ext`] / [`Opa::buffer_int`]
//! - **Non-inverting PGA** (x2..x32): [`Opa::pga_ext`] / [`Opa::pga_int`]
//! - **Non-inverting PGA about a reference**: [`Opa::pga_biased_ext`] / [`Opa::pga_biased_int`],
//!   which drive the bottom of the gain ladder from a [`LadderBottom`] source instead of grounding it
//!
//! `_ext` variants drive the `OPAx_OUT` pin; `_int` variants keep the output off the pins and only
//! route it to the ADC. Either can be sampled by passing a mutable reference to the returned handle
//! to [`Adc::blocking_read`](crate::adc::Adc::blocking_read) or
//! [`Adc::irq_read`](crate::adc::Adc::irq_read) — the output is a fixed internal ADC channel, so no
//! pin or wiring is involved.
//!
//! The non-inverting input can come from an `OPAx_INy+` pin or from an internal source — see
//! [`NonInvertingInput`].
//!
//! # The OPA depends on SYSOSC, and nothing requests it
//!
//! The amplifier's support circuits are clocked from SYSOSC and there is no hardware provision to
//! request it (SLAU846 §21.2.5): a tree that powers SYSOSC down gives an amplifier that is out of
//! spec with nothing reporting it. [`Opa::new`] therefore asserts that SYSOSC is running — and at
//! its 32 MHz base when [`Config::rail_to_rail_input`] is set, which is all that mode supports.
//!
//! While an output handle exists the driver holds a sleep guard: STOP1 is the deepest mode an
//! enabled OPA is supported in, and rail-to-rail input rules out every deep-sleep mode, STOP gearing
//! SYSOSC down to 4 MHz.
//!
//! # Accuracy
//!
//! Without chopping the input offset (a few millivolts, multiplied by the gain) appears at the
//! output. [`Chopping::Standard`] removes it but modulates ripple at the chop frequency onto the
//! output, which the TRM expects an external RC filter to remove, sized per gain (SLAU846
//! table 21-4). The hardware's third mode, ADC-assisted chopping, is not exposed: it only works
//! during an ADC hardware-averaging conversion, which [`crate::adc`] does not yet offer.
//!
//! An enabled amplifier settles within the datasheet's `tEN`, and a gain change within `tSETTLE` —
//! single-digit microseconds each. A sample taken sooner reads the output mid-slew, so it is
//! unreliable rather than wrong by any fixed amount. Below a 1.8 V supply the settling is further
//! delayed by `PMCU_ERR_10`; the workaround (VBOOST always on) is board-level and not applied here.

#![macro_use]

use core::marker::PhantomData;

use embassy_hal_internal::PeripheralType;

use crate::Peri;
use crate::pac::opa::{regs, vals};
use crate::sysctl::MaybeWakeGuard;

/// Gain-bandwidth selection (CFGBASE.GBW).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GainBandwidth {
    /// Low gain bandwidth, lower current. 1 MHz class; see the device datasheet.
    Low,
    /// High gain bandwidth, higher current. 6 MHz class; see the device datasheet.
    High,
}

/// Chopping mode (CFG.CHOP).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Chopping {
    /// No chopping. The raw input offset voltage, multiplied by the gain, appears at the output.
    Disabled,
    /// Standard chopping. Removes the input offset but modulates ripple at the chop frequency onto
    /// the output; the TRM sizes an external RC filter per gain to remove it.
    Standard,
}

/// Configuration common to all OPA topologies.
#[non_exhaustive]
#[derive(Clone, Copy)]
pub struct Config {
    /// Gain-bandwidth selection. Defaults to [`GainBandwidth::High`].
    pub gain_bandwidth: GainBandwidth,
    /// Rail-to-rail input. Defaults to `true`.
    ///
    /// Needs SYSOSC at its 32 MHz base (SLAU846 table 21-3), which also forbids every deep-sleep
    /// mode while the amplifier is enabled. Disable it when the input stays away from the rails and
    /// lower power or a deeper sleep matters.
    pub rail_to_rail_input: bool,
    /// Chopping mode. Defaults to [`Chopping::Disabled`].
    pub chopping: Chopping,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            gain_bandwidth: GainBandwidth::High,
            rail_to_rail_input: true,
            chopping: Chopping::Disabled,
        }
    }
}

/// Gain for the non-inverting PGA topology.
///
/// The discriminants are the CFG.GAIN encoding. The ladder starts at x2: GAIN=0x0 is not valid for
/// this topology (SLAU846 table 21-6), and unity gain is the buffer topology instead, which does not
/// go through the ladder at all. Keeping 0x0 unrepresentable is also what makes [`set_gain`] safe to
/// call while the amplifier runs — the TRM only forbids a live change to or from 0x0.
///
/// [`set_gain`]: OpaOutput::set_gain
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Gain {
    X2 = 1,
    X4 = 2,
    X8 = 3,
    X16 = 4,
    X32 = 5,
}

/// A source for the non-inverting (+) input (CFG.PSEL).
///
/// Created from an `OPAx_INy+` pin (which is configured for analog mode and consumed — use
/// [`Peri::reborrow`] to keep it), or from one of the internal-source constructors.
pub struct NonInvertingInput<'d, T: Instance> {
    channel: vals::Psel,
    _phantom: PhantomData<(&'d (), T)>,
}

impl<'d, T: Instance> NonInvertingInput<'d, T> {
    const fn internal(channel: vals::Psel) -> Self {
        Self {
            channel,
            _phantom: PhantomData,
        }
    }

    /// The DAC12 output, routed internally.
    ///
    /// This is the same channel as the `OPAx_IN2+` pad shared with `DAC_OUT`: with the DAC disabled,
    /// an external voltage on that pad drives it.
    #[cfg(dac)]
    pub const fn dac12() -> Self {
        Self::internal(vals::Psel::Dac12out)
    }

    /// The 8-bit reference DAC of the paired COMP peripheral.
    #[cfg(comp)]
    pub const fn dac8() -> Self {
        Self::internal(vals::Psel::Dac8out)
    }

    /// The internal voltage reference.
    ///
    /// Only meaningful while a [`Vref`](crate::vref::Vref) is alive; with the reference off this
    /// channel floats.
    #[cfg(vref)]
    pub const fn vref() -> Self {
        Self::internal(vals::Psel::Vref)
    }

    /// Analog ground.
    pub const fn ground() -> Self {
        Self::internal(vals::Psel::Vss)
    }
}

impl<'d, T: Instance, P: NonInvertingPin<T>> From<Peri<'d, P>> for NonInvertingInput<'d, T> {
    fn from(pin: Peri<'d, P>) -> Self {
        SealedNonInvertingPin::setup(&*pin);
        Self {
            channel: vals::Psel::from_bits(SealedNonInvertingPin::channel(&*pin)),
            _phantom: PhantomData,
        }
    }
}

/// A source for the bottom of the gain ladder (CFG.MSEL).
///
/// In the non-inverting PGA the ladder bottom is the point the gain pivots about, not merely a
/// return path:
///
/// ```text
/// Vout = gain * Vin + (1 - gain) * Vladder
/// ```
///
/// Grounding it gives the plain `Vout = gain * Vin`, which also means the input's own DC is
/// multiplied by the gain: at x32 an input sitting 50 mV away from where it needs to be moves the
/// output by 1.6 V. Driving the ladder bottom from the DAC12 instead makes the DC operating point of
/// the output a free variable, settable independently of the gain, which is what lets a high gain be
/// used on a signal whose DC is not already placed for it.
///
/// Only the sources that are unambiguous for this topology are exposed. The remaining CFG.MSEL
/// values are an external `OPAx_IN1-` pin, for which this driver has no pin trait yet, and the
/// previous instance's ladder top for cascading, which has no meaning on the first instance.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum LadderBottom {
    /// Analog ground, giving `Vout = gain * Vin`.
    #[default]
    Ground,
    /// The DAC12 output, giving `Vout = gain * Vin + (1 - gain) * Vdac`.
    ///
    /// The DAC must be enabled and settled before the amplifier is: it is the reference the whole
    /// transfer function is written against, so bringing it up afterwards means the first output the
    /// ADC sees is referred to whatever the DAC pin happened to be sitting at.
    #[cfg(dac)]
    Dac12,
}

/// OPA driver.
///
/// Power to the peripheral is enabled on construction and removed on drop. Use the topology methods
/// to configure and enable the amplifier.
pub struct Opa<'d, T: Instance> {
    _peri: Peri<'d, T>,
    chop: vals::Chop,
    rri: bool,
}

/// An enabled OPA whose output drives the `OPAx_OUT` pin.
///
/// Can also be sampled by passing a mutable reference to this handle to the ADC. The amplifier is
/// disabled when this is dropped.
pub struct OpaOutput<'a, T: Instance> {
    _guard: MaybeWakeGuard,
    _phantom: PhantomData<&'a mut T>,
}

/// An enabled OPA whose output is only routed internally (to the ADC).
///
/// Sample it by passing a mutable reference to this handle to the ADC. The amplifier is disabled
/// when this is dropped.
pub struct OpaInternalOutput<'a, T: Instance> {
    _guard: MaybeWakeGuard,
    _phantom: PhantomData<&'a mut T>,
}

impl<'d, T: Instance> Opa<'d, T> {
    /// Create a new OPA driver.
    ///
    /// Resets and powers up the peripheral and applies `config`. The amplifier itself stays disabled
    /// until a topology method is called.
    ///
    /// # Panics
    ///
    /// If the clock tree left SYSOSC powered down, or geared below its 32 MHz base while
    /// [`Config::rail_to_rail_input`] is set — see the module docs.
    pub fn new(peri: Peri<'d, T>, config: Config) -> Self {
        let sysosc = crate::sysctl::clocks().sysosc;
        assert!(sysosc != 0, "the OPA needs SYSOSC running (SLAU846 §21.2.5)");
        if config.rail_to_rail_input {
            assert!(
                sysosc == 32_000_000,
                "rail-to-rail input needs SYSOSC at its 32 MHz base (SLAU846 table 21-3)"
            );
        }

        let r = T::regs();

        r.gprcm().rstctl().write(|w| {
            w.set_key(vals::ResetKey::Key);
            w.set_resetassert(true);
            w.set_resetstkyclr(true);
        });
        r.gprcm().pwren().write(|w| {
            w.set_key(vals::PwrenKey::Key);
            w.set_enable(true);
        });
        // A few bus cycles are required after the power switch before touching peripheral registers.
        cortex_m::asm::delay(16);

        r.cfgbase().write(|w| {
            w.set_gbw(match config.gain_bandwidth {
                GainBandwidth::Low => vals::Gbw::Lowgain,
                GainBandwidth::High => vals::Gbw::Highgain,
            });
            w.set_rri(config.rail_to_rail_input);
        });

        let chop = match config.chopping {
            Chopping::Disabled => vals::Chop::Off,
            Chopping::Standard => vals::Chop::On,
        };

        Self {
            _peri: peri,
            chop,
            rri: config.rail_to_rail_input,
        }
    }

    /// Configure and switch the amplifier on, returning the sleep guard that covers it.
    fn enable(&self, mut cfg: regs::Cfg) -> MaybeWakeGuard {
        // SLAU846 and SLAU847 table 2-2: an enabled OPA is supported in RUN0, SLEEP0, STOP0 and STOP1, its
        // support circuits wanting SYSOSC's 4 MHz output — which is `floor_for_operation` at 4 MHz.
        // Rail-to-rail input additionally wants the 32 MHz base, which STOP gears away, so it
        // forbids deep sleep entirely.
        let sysosc_hz = if self.rri { 32_000_000 } else { 4_000_000 };
        let guard = MaybeWakeGuard::new(<T as crate::sysctl::LowPowerInstance>::SLEEP.floor_for_operation(sysosc_hz));

        let r = T::regs();
        cfg.set_chop(self.chop);
        r.cfg().write_value(cfg);
        r.ctl().write(|w| w.set_enable(true));
        // Bounded by the hardware: RDY follows within the datasheet's enable time, single-digit
        // microseconds.
        while !r.stat().read().rdy() {}

        guard
    }

    /// Unity-gain buffer of `input`, driving the output pin.
    pub fn buffer_ext<'a>(
        &'a mut self,
        input: impl Into<NonInvertingInput<'a, T>>,
        output: Peri<'a, impl OutputPin<T>>,
    ) -> OpaOutput<'a, T> {
        SealedOutputPin::setup(&*output);
        let mut cfg = Self::buffer_cfg(input.into());
        cfg.set_outpin(true);
        OpaOutput {
            _guard: self.enable(cfg),
            _phantom: PhantomData,
        }
    }

    /// Unity-gain buffer of `input`, output routed only to the ADC.
    pub fn buffer_int<'a>(&'a mut self, input: impl Into<NonInvertingInput<'a, T>>) -> OpaInternalOutput<'a, T> {
        OpaInternalOutput {
            _guard: self.enable(Self::buffer_cfg(input.into())),
            _phantom: PhantomData,
        }
    }

    fn buffer_cfg(input: NonInvertingInput<'_, T>) -> regs::Cfg {
        // Feedback from the ladder top; the ladder bottom is left open so no current flows and the
        // ladder acts as a plain wire.
        let mut cfg = regs::Cfg(0);
        cfg.set_psel(input.channel);
        cfg.set_nsel(vals::Nsel::Oanrtop);
        cfg
    }

    /// Non-inverting PGA: output is `gain * input`, driving the output pin.
    ///
    /// The gain ladder is grounded. Use [`Opa::pga_biased_ext`] to pivot the gain about a reference
    /// instead.
    pub fn pga_ext<'a>(
        &'a mut self,
        input: impl Into<NonInvertingInput<'a, T>>,
        output: Peri<'a, impl OutputPin<T>>,
        gain: Gain,
    ) -> OpaOutput<'a, T> {
        self.pga_biased_ext(input, output, gain, LadderBottom::Ground)
    }

    /// Non-inverting PGA: output is `gain * input`, routed only to the ADC.
    ///
    /// The gain ladder is grounded. Use [`Opa::pga_biased_int`] to pivot the gain about a reference
    /// instead.
    pub fn pga_int<'a>(
        &'a mut self,
        input: impl Into<NonInvertingInput<'a, T>>,
        gain: Gain,
    ) -> OpaInternalOutput<'a, T> {
        self.pga_biased_int(input, gain, LadderBottom::Ground)
    }

    /// Non-inverting PGA about `ladder`, driving the output pin.
    ///
    /// Output is `gain * input + (1 - gain) * ladder`; see [`LadderBottom`].
    pub fn pga_biased_ext<'a>(
        &'a mut self,
        input: impl Into<NonInvertingInput<'a, T>>,
        output: Peri<'a, impl OutputPin<T>>,
        gain: Gain,
        ladder: LadderBottom,
    ) -> OpaOutput<'a, T> {
        SealedOutputPin::setup(&*output);
        let mut cfg = Self::pga_cfg(input.into(), gain, ladder);
        cfg.set_outpin(true);
        OpaOutput {
            _guard: self.enable(cfg),
            _phantom: PhantomData,
        }
    }

    /// Non-inverting PGA about `ladder`, routed only to the ADC.
    ///
    /// Output is `gain * input + (1 - gain) * ladder`; see [`LadderBottom`].
    pub fn pga_biased_int<'a>(
        &'a mut self,
        input: impl Into<NonInvertingInput<'a, T>>,
        gain: Gain,
        ladder: LadderBottom,
    ) -> OpaInternalOutput<'a, T> {
        OpaInternalOutput {
            _guard: self.enable(Self::pga_cfg(input.into(), gain, ladder)),
            _phantom: PhantomData,
        }
    }

    fn pga_cfg(input: NonInvertingInput<'_, T>, gain: Gain, ladder: LadderBottom) -> regs::Cfg {
        // Feedback from the tap, with the ladder bottom held at `ladder`.
        let mut cfg = regs::Cfg(0);
        cfg.set_psel(input.channel);
        cfg.set_nsel(vals::Nsel::Oanrtap);
        cfg.set_msel(match ladder {
            LadderBottom::Ground => vals::Msel::Vss,
            #[cfg(dac)]
            LadderBottom::Dac12 => vals::Msel::Dac12out,
        });
        cfg.set_gain(gain as u8);
        cfg
    }
}

impl<'d, T: Instance> Drop for Opa<'d, T> {
    fn drop(&mut self) {
        let r = T::regs();
        r.ctl().write(|w| w.set_enable(false));
        r.gprcm().pwren().write(|w| {
            w.set_key(vals::PwrenKey::Key);
            w.set_enable(false);
        });
    }
}

impl<'a, T: Instance> OpaOutput<'a, T> {
    /// Change the PGA gain while the amplifier is running.
    ///
    /// Only meaningful for outputs created by [`Opa::pga_ext`] or [`Opa::pga_biased_ext`]; useful
    /// for auto-ranging. The output settles within the datasheet's `tSETTLE`, single-digit
    /// microseconds; a sample taken sooner is unreliable.
    pub fn set_gain(&mut self, gain: Gain) {
        T::regs().cfg().modify(|w| w.set_gain(gain as u8));
    }
}

impl<'a, T: Instance> OpaInternalOutput<'a, T> {
    /// Change the PGA gain while the amplifier is running.
    ///
    /// Only meaningful for outputs created by [`Opa::pga_int`] or [`Opa::pga_biased_int`]; useful
    /// for auto-ranging. The output settles within the datasheet's `tSETTLE`, single-digit
    /// microseconds; a sample taken sooner is unreliable.
    pub fn set_gain(&mut self, gain: Gain) {
        T::regs().cfg().modify(|w| w.set_gain(gain as u8));
    }
}

impl<'a, T: Instance> Drop for OpaOutput<'a, T> {
    fn drop(&mut self) {
        T::regs().ctl().write(|w| w.set_enable(false));
    }
}

impl<'a, T: Instance> Drop for OpaInternalOutput<'a, T> {
    fn drop(&mut self) {
        T::regs().ctl().write(|w| w.set_enable(false));
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> crate::pac::opa::Opa;
}

/// OPA instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + crate::sysctl::LowPowerInstance + 'static {}

pub(crate) trait SealedNonInvertingPin<T> {
    fn setup(&self);
    fn channel(&self) -> u8;
}

pub(crate) trait SealedOutputPin<T> {
    fn setup(&self);
}

/// A pin that can be used as the OPA non-inverting (+) input.
#[allow(private_bounds)]
pub trait NonInvertingPin<T: Instance>: PeripheralType + SealedNonInvertingPin<T> + Sized {}

/// The `OPAx_OUT` pin.
#[allow(private_bounds)]
pub trait OutputPin<T: Instance>: PeripheralType + SealedOutputPin<T> + Sized {}

macro_rules! impl_opa_instance {
    ($inst:ident) => {
        // No guard is held while the amplifier is merely configured, which is only sound while the
        // configuration registers survive deep sleep. True of PD0 on every device shipped so far;
        // checked so a device that moves the OPA fails to build instead of losing its configuration.
        const _: () = {
            use crate::sysctl::LowPowerInstance;

            core::assert!(
                <crate::peripherals::$inst as LowPowerInstance>::SLEEP
                    .floor_to_keep_configured()
                    .is_none(),
                "Opa needs a WakeGuard from construction on this device: its configuration does not survive deep sleep"
            );
        };

        impl crate::opa::SealedInstance for crate::peripherals::$inst {
            fn regs() -> crate::pac::opa::Opa {
                crate::pac::$inst
            }
        }
        impl crate::opa::Instance for crate::peripherals::$inst {}
    };
}

macro_rules! impl_opa_non_inverting_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::opa::NonInvertingPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
        impl crate::opa::SealedNonInvertingPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn setup(&self) {
                crate::gpio::SealedPin::set_as_analog(self);
            }

            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

macro_rules! impl_opa_output_pin {
    ($inst:ident, $pin:ident) => {
        impl crate::opa::OutputPin<crate::peripherals::$inst> for crate::peripherals::$pin {}
        impl crate::opa::SealedOutputPin<crate::peripherals::$inst> for crate::peripherals::$pin {
            fn setup(&self) {
                crate::gpio::SealedPin::set_as_analog(self);
            }
        }
    };
}

macro_rules! impl_opa_adc_channel {
    ($inst:ident, $adc:ident, $ch:expr) => {
        impl<'a> crate::adc::AdcChannel<crate::peripherals::$adc>
            for crate::opa::OpaOutput<'a, crate::peripherals::$inst>
        {
        }
        impl<'a> crate::adc::SealedAdcChannel<crate::peripherals::$adc>
            for crate::opa::OpaOutput<'a, crate::peripherals::$inst>
        {
            fn channel(&self) -> u8 {
                $ch
            }
        }

        impl<'a> crate::adc::AdcChannel<crate::peripherals::$adc>
            for crate::opa::OpaInternalOutput<'a, crate::peripherals::$inst>
        {
        }
        impl<'a> crate::adc::SealedAdcChannel<crate::peripherals::$adc>
            for crate::opa::OpaInternalOutput<'a, crate::peripherals::$inst>
        {
            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}
