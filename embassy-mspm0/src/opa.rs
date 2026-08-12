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
//! [`NonInvertingInput`]. **Which sources a device has differs per family**: the input mux positions
//! are not the same everywhere, and an absent one connects the input to nothing rather than failing,
//! so the amplifier reads a floating node. [`NonInvertingInput::ground`] refuses to compile where its
//! position is absent; [`NonInvertingInput::vref`] is the `VREF+` pin and says what reaches it.
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
//! table 21-4). [`Chopping::AdcAveraging`] needs no filter, because the ADC flips the chop state
//! between conversions and averages the pair — so it only works while the ADC is averaging this
//! output, and nothing here can check that it is.
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
    /// Chopping the ADC cancels for you, leaving no ripple and needing no filter.
    ///
    /// The ADC toggles the chop state at the end of each conversion and averages the pair away, so
    /// **it only works while the ADC is averaging this output**: set [`Config::averaging`] and ask
    /// for it with [`Conversion::average`]. Nothing here can check that — the amplifier cannot see
    /// how the ADC is configured — and with averaging off the output is chopped and never
    /// unchopped.
    ///
    /// The averaged count must be even, which every [`Averaging`] setting is.
    ///
    /// [`Config::averaging`]: crate::adc::Config::averaging
    /// [`Conversion::average`]: crate::adc::Conversion::average
    /// [`Averaging`]: crate::adc::Averaging
    AdcAveraging,
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

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            gain_bandwidth: GainBandwidth::High,
            rail_to_rail_input: true,
            chopping: Chopping::Disabled,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
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
///
/// Copyable, so one built from a pin can be applied again and again. An application switching
/// between two sensors builds both once and picks between them per measurement, rather than
/// re-consuming a pin it no longer has.
#[derive(Clone, Copy)]
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
        const { core::assert!(T::HAS_DAC12, "this device's OPA has no DAC12 input position") };
        Self::internal(vals::Psel::Dac12out)
    }

    /// The 8-bit reference DAC of the paired COMP peripheral.
    ///
    /// Which comparator that is differs by family — on the G series `OPAn` takes `COMPn`'s, on the L
    /// series both amplifiers take `COMP0`'s — and the DAC has to be running for this to carry
    /// anything. Build a [`Comp`](crate::comp::Comp) with a
    /// [`Reference`](crate::comp::Reference) and keep it alive across the measurement;
    /// `Comp::new_reference_only` exists for exactly that.
    ///
    /// Nothing here checks that a comparator is live: the position is a mux selection, so an
    /// unpowered DAC reads as a floating node rather than as an error.
    #[cfg(comp)]
    pub const fn dac8() -> Self {
        const { core::assert!(T::HAS_DAC8, "this device's OPA has no DAC8 input position") };
        Self::internal(vals::Psel::Dac8out)
    }

    /// The `VREF+` pin node.
    ///
    /// **Not the internal reference, though it is where the internal reference appears on some
    /// devices.** The G families buffer their reference out to this pin, so a live
    /// [`Vref`](crate::vref::Vref) is all this needs there. The C, H and L families do not: their
    /// reference feeds the ADC and comparator internally and the pin is an input only, so this
    /// carries whatever is applied to `VREF+` externally and nothing at all when that is nothing.
    ///
    /// An undriven pin does not read zero — it floats, and drifts toward a rail over seconds while
    /// looking like a plausible measurement on the way.
    #[cfg(vref)]
    pub const fn vref() -> Self {
        const { core::assert!(T::HAS_VREF_PLUS, "this device's OPA has no VREF+ input position") };
        Self::internal(vals::Psel::Vref)
    }

    /// The paired amplifier's gain-ladder top.
    ///
    /// Private: on its own this proves nothing about the amplifier it reads. [`OpaPair`] owns both
    /// and enables them together, which is what makes the source live.
    #[allow(dead_code)]
    const fn cascade() -> Self {
        Self::internal(vals::Psel::Oanm1rtop)
    }

    /// Analog ground.
    ///
    /// **Not every device has this position.** Where it is absent the mux connects the input to
    /// nothing, and the amplifier reads a floating node that often sits near zero and looks right —
    /// so this refuses to compile there rather than letting the reading be believed.
    pub const fn ground() -> Self {
        const { core::assert!(T::HAS_GROUND, "this device's OPA has no ground input position") };
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
///
/// # The instance parameter duplicates every method body
///
/// `T` puts a copy of each method in the binary for every instance it is used with, and a chip has
/// several of these. Nothing shows it in a symbol listing: the bodies inline into the caller.
///
/// **Erasing it is not automatically the fix.** Monomorphising folds the register addresses to
/// immediates, so a shared body has to carry them as arguments instead — measured on the timer, that
/// lost at every instance count a part reaches. [`simple_pwm::SimplePwm`](crate::tim::simple_pwm::SimplePwm)
/// carries the figures and what did pay.
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

/// The upstream stage's output within a standing [`Cascade`].
///
/// Both stages of a chain amplify at once, and each reaches its own ADC channel, so the first
/// stage's smaller gain can be sampled without taking the chain down. Reading both inside one
/// stimulus gives a clipped high-gain sample its low-gain companion from the same event, rather
/// than from a retry or a gain change.
///
/// Sample it by passing a mutable reference to the ADC, the same as the other two handles.
///
/// Unlike them it disables nothing when dropped. The stage it names belongs to the
/// [`OpaPair`], which switches it off at the next configuration call, [`OpaPair::disable`], or
/// its own drop.
pub struct OpaTap<'a, T: Instance> {
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
        let sysosc = crate::sysctl::with_clocks(|clocks| clocks.sysosc);
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
            Chopping::AdcAveraging => vals::Chop::Avgon,
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

    /// The register configuration for one [`Stage`], reusing the topology builders above.
    fn stage_cfg(input: NonInvertingInput<'_, T>, stage: Stage) -> regs::Cfg {
        match stage {
            Stage::Buffer => Self::buffer_cfg(input),
            Stage::Pga(gain) => Self::pga_cfg(input, gain, LadderBottom::Ground),
            Stage::PgaBiased(gain, ladder) => Self::pga_cfg(input, gain, ladder),
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

/// One amplifier's topology within an [`OpaPair`].
///
/// The same three shapes [`Opa`]'s own methods offer, named so a pair can be handed two of them.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Stage {
    /// Unity-gain buffer.
    Buffer,

    /// Non-inverting PGA with the ladder grounded.
    Pga(Gain),

    /// Non-inverting PGA pivoted about a reference; output is `gain * input + (1 - gain) * ladder`.
    PgaBiased(Gain, LadderBottom),
}

/// Proof that `Self`'s gain-ladder top reaches `T`'s non-inverting input mux.
///
/// Generated per instance from the device metadata, which names the source and its direction. Usually
/// mutual on a two-amplifier chip, so either can feed the other; a device offering only one direction
/// permits only that chain.
#[allow(private_bounds)]
pub trait CascadeInto<T: Instance>: Instance {}

/// Two amplifiers, chained or independent, reconfigurable while they run.
///
/// The pair owns both drivers so a topology can be torn down and rebuilt between measurements without
/// surrendering anything — which is what an application alternating between two sensors needs. Each
/// configuration method takes `&mut self` and hands back short-lived output handles, so the borrow
/// checker allows exactly one topology at a time and reconfiguring is just calling another method.
///
/// The two chain methods hand back a [`Cascade`], which reads either stage.
///
/// [`OpaPair::release`] gives the two [`Opa`] drivers back.
///
/// # Power
///
/// Each configuration method switches both amplifiers off before applying the new one. Dropping an
/// output handle disables the stage it names — but in a chain the *upstream* stage stays on until the
/// next configuration call, [`OpaPair::disable`], or the pair being dropped. That is what lets
/// [`OpaTap`] be read without owning anything. Call [`OpaPair::disable`] if a gap between
/// measurements is long enough to care about.
pub struct OpaPair<'d, A: Instance, B: Instance> {
    a: Opa<'d, A>,
    b: Opa<'d, B>,
    upstream: Option<MaybeWakeGuard>,
}

impl<'d, A: Instance, B: Instance> OpaPair<'d, A, B> {
    /// Take two amplifiers so they can be chained.
    ///
    /// Both are already reset, powered and checked against the clock tree by [`Opa::new`]; this adds
    /// no configuration of its own and leaves both switched off.
    pub fn new(a: Opa<'d, A>, b: Opa<'d, B>) -> Self {
        Self { a, b, upstream: None }
    }

    /// Give the two amplifiers back, both switched off.
    pub fn release(mut self) -> (Opa<'d, A>, Opa<'d, B>) {
        self.disable();

        // Neither `Opa` may be dropped here -- that would cut power to both -- and `Self` has no
        // `Drop`, so moving the fields out is a plain destructure.
        let Self { a, b, .. } = self;

        (a, b)
    }

    /// Switch both amplifiers off.
    ///
    /// Only worth calling to release the upstream stage of a chain, which outlives its output handle.
    pub fn disable(&mut self) {
        A::regs().ctl().write(|w| w.set_enable(false));
        B::regs().ctl().write(|w| w.set_enable(false));
        self.upstream = None;
    }

    /// Chain `A` into `B`: `A` amplifies `input`, and `B` amplifies `A`'s ladder top.
    ///
    /// Both stages are readable — see [`Cascade`].
    pub fn chain_a_into_b<'x>(
        &'x mut self,
        input: impl Into<NonInvertingInput<'x, A>>,
        first: Stage,
        second: Stage,
    ) -> Cascade<'x, A, B>
    where
        A: CascadeInto<B>,
    {
        self.disable();

        self.upstream = Some(self.a.enable(Opa::<A>::stage_cfg(input.into(), first)));

        Cascade {
            upstream: OpaTap { _phantom: PhantomData },
            output: OpaInternalOutput {
                _guard: self.b.enable(Opa::<B>::stage_cfg(NonInvertingInput::cascade(), second)),
                _phantom: PhantomData,
            },
        }
    }

    /// Chain `B` into `A`: `B` amplifies `input`, and `A` amplifies `B`'s ladder top.
    ///
    /// Both stages are readable — see [`Cascade`].
    pub fn chain_b_into_a<'x>(
        &'x mut self,
        input: impl Into<NonInvertingInput<'x, B>>,
        first: Stage,
        second: Stage,
    ) -> Cascade<'x, B, A>
    where
        B: CascadeInto<A>,
    {
        self.disable();

        self.upstream = Some(self.b.enable(Opa::<B>::stage_cfg(input.into(), first)));

        Cascade {
            upstream: OpaTap { _phantom: PhantomData },
            output: OpaInternalOutput {
                _guard: self.a.enable(Opa::<A>::stage_cfg(NonInvertingInput::cascade(), second)),
                _phantom: PhantomData,
            },
        }
    }

    /// Run both amplifiers unchained, each on its own input.
    ///
    /// Both handles are ADC channels, and dropping each disables its own amplifier.
    pub fn independent<'x>(
        &'x mut self,
        a_input: impl Into<NonInvertingInput<'x, A>>,
        a_stage: Stage,
        b_input: impl Into<NonInvertingInput<'x, B>>,
        b_stage: Stage,
    ) -> (OpaInternalOutput<'x, A>, OpaInternalOutput<'x, B>) {
        self.disable();

        (
            OpaInternalOutput {
                _guard: self.a.enable(Opa::<A>::stage_cfg(a_input.into(), a_stage)),
                _phantom: PhantomData,
            },
            OpaInternalOutput {
                _guard: self.b.enable(Opa::<B>::stage_cfg(b_input.into(), b_stage)),
                _phantom: PhantomData,
            },
        )
    }
}

/// A standing chain, and both of its outputs.
///
/// `Up` amplifies the input and `Down` amplifies `Up`'s ladder top, so the two readings differ by
/// `Down`'s gain. Which amplifier is which follows the chain direction rather than the instance
/// name, and each carries its own ADC channel, so a caller reads a stage by naming it here rather
/// than by knowing what it is routed to.
///
/// The chain stands for as long as this does. Dropping it disables the downstream stage; the
/// upstream one belongs to the [`OpaPair`] and outlives it, as [`OpaPair`]'s own docs describe.
pub struct Cascade<'a, Up: Instance, Down: Instance> {
    upstream: OpaTap<'a, Up>,
    output: OpaInternalOutput<'a, Down>,
}

impl<'a, Up: Instance, Down: Instance> Cascade<'a, Up, Down> {
    /// The chain's output, `Down`'s amplification of `Up`.
    pub fn output(&mut self) -> &mut OpaInternalOutput<'a, Down> {
        &mut self.output
    }

    /// The first stage's output, live at the same time and at the lower gain.
    pub fn upstream(&mut self) -> &mut OpaTap<'a, Up> {
        &mut self.upstream
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

    /// Whether this instance's `CFG.PSEL` has the ground position.
    ///
    /// It is absent on the L series, where selecting it connects the input to nothing — so the
    /// amplifier reads a floating node rather than ground, and reports a plausible near-zero. Per
    /// instance because the mux maps are.
    const HAS_GROUND: bool;

    /// Whether this instance's mux has the DAC12 position.
    ///
    /// Each of these is gated to match the constructor that reads it: a device without the peripheral
    /// has no constructor to guard, and an ungated constant is dead code there.
    #[cfg(dac)]
    const HAS_DAC12: bool;

    /// Whether this instance's mux has the paired comparator's 8-bit DAC.
    #[cfg(comp)]
    const HAS_DAC8: bool;

    /// Whether this instance's mux has the `VREF+` pin node.
    ///
    /// Presence of the position, not of a voltage on it: the pin can be present and undriven, which
    /// is what [`NonInvertingInput::vref`]'s own docs are about.
    #[cfg(vref)]
    const HAS_VREF_PLUS: bool;
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

/// `$source`'s ladder top reaches `$sink`'s non-inverting input mux.
macro_rules! impl_opa_cascade {
    ($source:ident, $sink:ident) => {
        impl crate::opa::CascadeInto<crate::peripherals::$sink> for crate::peripherals::$source {}
    };
}

macro_rules! impl_opa_instance {
    ($inst:ident, $has_ground:expr, $has_dac12:expr, $has_dac8:expr, $has_vref_plus:expr) => {
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

            const HAS_GROUND: bool = $has_ground;
            #[cfg(dac)]
            const HAS_DAC12: bool = $has_dac12;
            #[cfg(comp)]
            const HAS_DAC8: bool = $has_dac8;
            #[cfg(vref)]
            const HAS_VREF_PLUS: bool = $has_vref_plus;
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

        impl<'a> crate::adc::AdcChannel<crate::peripherals::$adc>
            for crate::opa::OpaTap<'a, crate::peripherals::$inst>
        {
        }
        impl<'a> crate::adc::SealedAdcChannel<crate::peripherals::$adc>
            for crate::opa::OpaTap<'a, crate::peripherals::$inst>
        {
            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}
