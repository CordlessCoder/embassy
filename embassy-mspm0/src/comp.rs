//! Analog comparator (COMP).
//!
//! Two things share this block: a comparator, and an 8-bit reference DAC that feeds it. The DAC is
//! worth having on its own — it is wired to the amplifier's input mux as well as to the comparator,
//! so on families where the OPA has no other internal source it is the only one it has. See
//! [`Comp::new_reference_only`].
//!
//! The comparator compares its positive and negative terminals and reports which is higher. Each
//! terminal takes either a pin or the reference, and [`Comp::wait_for_edge`] parks on a transition —
//! which, the block sitting in PD0 and staying usable to STANDBY1, is an analog wake at microamps.
//!
//! # What this driver does not expose
//!
//! **The input mux positions above the pins.** `IPSEL`/`IMSEL` also reach internal analog sources,
//! and which positions exist differs per family. An absent position selects nothing rather than
//! failing, so the comparator reads a floating node and reports a plausible answer — the same trap
//! that cost a day on the OPA. Only the positions the device metadata names as pins are generated.
//!
//! **Window comparator mode, blanking, and the input SHORT switch.** All reachable through the
//! registers; none has a device-independent story yet.
//!
//! # The reference generator
//!
//! [`Reference`] turns the generator on and picks what feeds the DAC. Three of the six sources
//! exist on every comparator; the other three reach a dedicated internal reference and exist only
//! where the metadata says so, [`Comp::new`] refusing them elsewhere rather than letting the
//! comparator run against a threshold that was never applied.
//!
//! **`VrefModule` is an internal route, not the `VREF+` pin.** SLAU847 figure 16-5 labels it "From
//! VREF Module", so it reaches the comparator on devices that never buffer their reference out to a
//! pin. A live `vref::Vref` is what it needs, not an external voltage.
//!
//! The DAC's output is `reference x (code + 1) / 256`, so [`DacCode::ZERO`] is one LSB above ground
//! rather than at it. TI's own `opa_dac8_output_buffer` example computes `code = mV * 255 / ref`,
//! which is a different function and up to two counts out at the top of scale — the datasheet's
//! `Vdac-code` row and the `CTL3` field description agree against it.
//!
//! # Errata this driver acts on
//!
//! # Two waits with nothing to wait on
//!
//! Neither the comparator's enable time nor the reference DAC's settling has a status bit behind it,
//! so both are blocking delays taken from the device's own datasheet figures. That makes
//! [`Comp::new`] and [`Comp::set_dac_code`] slower than the register writes they perform, and it is
//! why a threshold is trustworthy the moment either returns.
//!
//! - **`COMP_ERR_05`** — enabling the comparator raises both edge interrupts, so the first
//!   [`Comp::wait_for_edge`] would return without an edge. The flags are cleared after enabling.
//! - **`COMP_ERR_03`** — hysteresis is unstable with the inputs exchanged. The pair is rejected.
//! - **`COMP_ERR_01`** — a comparator whose negative terminal is on channel 0 toggles on its own in
//!   STANDBY0. The sleep floor is raised to keep the device out of it.
//!
//! **`COMP_ERR_02` needs no code**: it applies to hysteresis built by switching `DACCODE0`/`DACCODE1`
//! from the comparator's own output, and this driver does not offer that — `Config::hysteresis` is
//! the `CTL1.HYST` ladder, which is TI's own workaround.

#![macro_use]

use core::future::{Future, poll_fn};
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use mspm0_metapac::comp::{Comp as Regs, vals};

use crate::gpio::{AnyPin, MaybeAnyPin, SealedPin};
use crate::mode::{Async, Blocking, Mode as DriverMode};
use crate::sync::irq_waker::IrqWaker;
use crate::sysctl::{LowPowerInstance, MaybeWakeGuard, SleepLevel};

/// Speed and current of the comparator itself.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// Faster response, higher current.
    ///
    /// **Cannot run on LFCLK.** SYSCTL raises a clock error if a comparator is enabled in this mode
    /// while the bus clock is LFCLK, so [`Comp::new`] rejects the combination.
    #[default]
    Fast,

    /// Lower current, slower response. Runs on any bus clock.
    UltraLowPower,
}

/// Built-in hysteresis, in millivolts of separation between the two switching thresholds.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hysteresis {
    /// No hysteresis: one threshold, and a slow input crosses it repeatedly on noise.
    #[default]
    None,

    /// About 10 mV.
    Mv10,

    /// About 20 mV.
    Mv20,

    /// About 30 mV.
    Mv30,
}

impl Hysteresis {
    const fn to_vals(self) -> vals::Hyst {
        match self {
            Hysteresis::None => vals::Hyst::NoHys,
            Hysteresis::Mv10 => vals::Hyst::LowHys,
            Hysteresis::Mv20 => vals::Hyst::MedHys,
            Hysteresis::Mv30 => vals::Hyst::HighHys,
        }
    }
}

/// Output glitch filter delay.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FilterDelay {
    /// The shortest delay the block offers.
    Short,
    /// Twice [`FilterDelay::Short`].
    Medium,
    /// Four times [`FilterDelay::Short`].
    Long,
    /// Eight times [`FilterDelay::Short`].
    Longest,
}

impl FilterDelay {
    const fn to_vals(self) -> vals::Fltdly {
        match self {
            FilterDelay::Short => vals::Fltdly::Dly0,
            FilterDelay::Medium => vals::Fltdly::Dly1,
            FilterDelay::Long => vals::Fltdly::Dly2,
            FilterDelay::Longest => vals::Fltdly::Dly3,
        }
    }
}

/// Which way round the output is reported.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputPolarity {
    /// Output is high while the positive terminal is above the negative one.
    #[default]
    NonInverted,

    /// Output is inverted, and reads high while the comparator is disabled.
    Inverted,
}

/// What feeds the reference generator.
///
/// A bare name runs the DAC and takes its output as the reference; a `Direct` name bypasses the DAC
/// and uses the source itself.
///
/// **The three internal-reference sources do not exist everywhere.** Where a device lacks them they
/// select no reference at all rather than failing, so [`Comp::new`] rejects them with
/// [`ConfigError::NoInternalReference`] on those parts. The device metadata is what decides;
/// the register-block version cannot, one of the two blocks spanning families that answer
/// differently.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReferenceSource {
    /// The analog supply feeds the DAC, and the DAC's output is the reference.
    ///
    /// Needs no VREF module, and tracks the supply — so the threshold it produces is a fraction of
    /// `VDDA` rather than a voltage.
    Vdda,

    /// The VREF module feeds the DAC, and the DAC's output is the reference.
    ///
    /// Reaches the comparator internally, so it does not depend on the reference being buffered out
    /// to the `VREF+` pin — which most families do not do. Needs a live
    /// `vref::Vref`.
    VrefModule,

    /// The VREF module is the reference directly, with the DAC switched off.
    ///
    /// **Ignored while [`Reference::sampled`] is set**, where the input is connected to the DAC
    /// whatever this says.
    VrefModuleDirect,

    /// The analog supply is the reference directly, with the DAC switched off.
    ///
    /// Not on every device — see the type's own docs.
    VddaDirect,

    /// A dedicated internal reference feeds the DAC, and the DAC's output is the reference.
    ///
    /// Needs **no VREF module at all**, which is what makes it worth having: it reaches a real
    /// voltage in modes where VREF is unavailable, and it is the only source that does.
    ///
    /// Not on every device — see the type's own docs.
    Internal,

    /// The dedicated internal reference directly, with the DAC switched off.
    ///
    /// Not on every device — see the type's own docs.
    InternalDirect,
}

impl ReferenceSource {
    const fn to_vals(self) -> vals::Refsrc {
        match self {
            ReferenceSource::Vdda => vals::Refsrc::VddaDac,
            ReferenceSource::VrefModule => vals::Refsrc::VrefDac,
            ReferenceSource::VrefModuleDirect => vals::Refsrc::Vref,
            ReferenceSource::VddaDirect => vals::Refsrc::Vdda,
            ReferenceSource::Internal => vals::Refsrc::IntvrefDac,
            ReferenceSource::InternalDirect => vals::Refsrc::Intvref,
        }
    }

    /// Whether this is one of the positions only some devices implement.
    const fn needs_internal_reference(self) -> bool {
        matches!(
            self,
            ReferenceSource::VddaDirect | ReferenceSource::Internal | ReferenceSource::InternalDirect
        )
    }
}

/// Which terminal the reference is applied to.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReferenceTerminal {
    /// The reference is the threshold a rising input crosses from below.
    #[default]
    Negative,

    /// The reference is the threshold a falling input crosses from above.
    Positive,
}

/// An 8-bit DAC code.
///
/// The output is `reference x (code + 1) / 256`, so this never selects zero volts and full scale is
/// the reference itself.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DacCode(u8);

impl DacCode {
    /// The lowest code, one LSB above ground.
    pub const ZERO: Self = Self(0);

    /// The highest code, at which the DAC output is the reference.
    pub const FULL_SCALE: Self = Self(u8::MAX);

    /// Wrap a raw code.
    pub const fn new(code: u8) -> Self {
        Self(code)
    }

    /// The code whose output is nearest `millivolts` given a reference of `reference_mv`.
    ///
    /// Saturates at [`Self::FULL_SCALE`] rather than wrapping. Rounds to nearest, so the error is at
    /// most half an LSB where the request is in range.
    pub const fn from_millivolts(millivolts: u32, reference_mv: u32) -> Self {
        if reference_mv == 0 {
            return Self::ZERO;
        }

        // Inverting `mv = ref * (n + 1) / 256` gives `n = mv * 256 / ref - 1`, rounded to nearest by
        // adding half a step before the division. All of it fits `u32`: `mv * 512` needs more only
        // above 8388 V. Saturating rather than `u64`, which would be a 64-bit divide by a run-time
        // divisor and over a kilobyte of `compiler_builtins` in any caller that does not fold — and
        // an over-range request saturates to `FULL_SCALE`, which is what the doc above promises.
        let scaled = millivolts
            .saturating_mul(512)
            .saturating_add(reference_mv)
            .saturating_div(reference_mv.saturating_mul(2));

        match scaled {
            0 => Self::ZERO,
            n if n > 256 => Self::FULL_SCALE,
            n => Self((n - 1) as u8),
        }
    }

    /// The output this code produces, in millivolts, given a reference of `reference_mv`.
    pub const fn to_millivolts(self, reference_mv: u32) -> u32 {
        (reference_mv * (self.0 as u32 + 1)) / 256
    }

    /// The raw code.
    pub const fn to_bits(self) -> u8 {
        self.0
    }
}

/// The reference generator's configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Reference {
    /// What feeds the generator.
    pub source: ReferenceSource,

    /// Which comparator terminal it drives.
    ///
    /// Ignored where that terminal also has a pin: the channel selection takes precedence.
    pub terminal: ReferenceTerminal,

    /// The DAC code, where [`Self::source`] runs the DAC.
    pub code: DacCode,

    /// Run the bandgap, the buffer and the DAC in the low-power sampled mode.
    ///
    /// Lower current for relaxed accuracy. **The DAC cannot be bypassed while this is set** — the
    /// comparator input connects to it whatever [`Self::source`] says.
    pub sampled: bool,
}

impl Default for Reference {
    fn default() -> Self {
        Self {
            source: ReferenceSource::Vdda,
            terminal: ReferenceTerminal::default(),
            code: DacCode::ZERO,
            sampled: false,
        }
    }
}

/// Comparator configuration.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Speed and current of the comparator.
    pub speed: Speed,

    /// Separation between the rising and falling thresholds.
    pub hysteresis: Hysteresis,

    /// Which way round the output is reported.
    pub output_polarity: OutputPolarity,

    /// Glitch filter on the output, or [`None`] to report every transition.
    pub filter: Option<FilterDelay>,

    /// Swap the two terminals, which also inverts the output.
    ///
    /// Comparing a signal against itself both ways round is how the input offset is measured.
    /// **Rejected together with hysteresis** where `COMP_ERR_03` applies.
    pub exchange_inputs: bool,

    /// The reference generator, or [`None`] to leave it off.
    ///
    /// Leaving it off is not merely a default: an enabled generator holds VREF on and draws current
    /// whether or not anything reads it.
    pub reference: Option<Reference>,
}

/// Why a [`Comp`] could not be configured.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// [`Speed::Fast`] was asked for while the bus clock is LFCLK, which SYSCTL faults.
    FastModeOnLfclk,

    /// Hysteresis was combined with [`Config::exchange_inputs`], which `COMP_ERR_03` makes unstable.
    HysteresisWithExchangedInputs,

    /// A [`ReferenceSource`] this device does not implement, which would select no reference at all.
    NoInternalReference,
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> InterruptHandler<T> {
    /// Mask rather than clear: the flag is what tells the waiting future the edge happened, and the
    /// handler has no way to hand a value over.
    ///
    /// Public only so the generated instance impls can reach it; there is no reason to call it.
    #[doc(hidden)]
    pub fn handle() {
        T::regs().cpu_int(0).imask().write(|_| {});
        T::state().waker.wake();
    }
}

/// Peripheral state.
pub(crate) struct State {
    waker: IrqWaker,
}

impl State {
    pub(crate) const fn new() -> Self {
        Self { waker: IrqWaker::new() }
    }
}

/// CPU cycles covering `ns` at `mclk`, rounded up, and zero for zero.
///
/// Derived from MCLK because it is a busy-wait on the CPU, and MCLK is the CPU clock in RUN. Split
/// out so the arithmetic is checked rather than buried in a delay call — a factor-of-1000 slip here
/// is the one error no build would catch, and the same helper in `vref.rs` is where that was learnt.
///
/// Whole microseconds times cycles per microsecond. The exact form needs 64 bits — 80 MHz times
/// 10 us overflows a `u32` before the division brings it back — and a 64-bit divide on this core is
/// over a kilobyte of `compiler_builtins`. Splitting MCLK into whole megahertz and the remainder
/// keeps every divisor constant and the arithmetic in 32 bits, at the cost of rounding the request
/// up to a whole microsecond. Saturating, because a wrapped product would round the wait down.
///
/// **The remainder is what makes this usable below 1 MHz.** Rounding MCLK up to whole megahertz
/// instead costs nothing at 32 MHz and waits 28 times too long on an LFCLK-sourced MCLK, which is
/// the tree a low-power application picks.
const fn wait_cycles(mclk: u32, ns: u32) -> u32 {
    if ns == 0 {
        return 0;
    }

    let us = ns.div_ceil(1_000);
    let cycles = us
        .saturating_mul(mclk / 1_000_000)
        .saturating_add(us.saturating_mul(mclk % 1_000_000).div_ceil(1_000_000));

    if cycles == 0 { 1 } else { cycles }
}

// The unit conversion in `wait_cycles` is the one error here no build would catch: too large and the
// constructor is merely slow, too small and it hands back a comparator that is not ready, which reads
// as an inaccurate threshold rather than as a fault.
const _: () = {
    // At 1 GHz a cycle is a nanosecond, so the conversion is the identity and any unit slip shows.
    core::assert!(wait_cycles(1_000_000_000, 10_000) == 10_000);
    // Rounds up rather than truncating, and never waits zero for a real figure.
    core::assert!(wait_cycles(1, 1) == 1);
    // A clock this actually runs at, against the arithmetic done the other way round.
    core::assert!(wait_cycles(32_000_000, 10_000) == 320);
    // An absent datasheet row waits nothing at all, rather than one cycle.
    core::assert!(wait_cycles(32_000_000, 0) == 0);
    // Never short of the exact answer, at the figures the metapac carries and the rates this runs
    // at. Covering the wait is the property the rounding has to preserve; equality is not.
    core::assert!(wait_cycles(80_000_000, 1_500) >= 120);
    core::assert!(wait_cycles(80_000_000, 10_000) >= 800);
    core::assert!(wait_cycles(4_000_000, 1_500) >= 6);
    core::assert!(wait_cycles(32_768, 10_000) >= 1);
    // And not wildly over on a sub-megahertz MCLK, which rounding the rate up to whole megahertz
    // was: 10 us at 32768 Hz is one cycle, and that form asked for ten.
    core::assert!(wait_cycles(32_768, 10_000) == 1);
    core::assert!(wait_cycles(500_000, 10_000) == 5);
};

/// Proof that this instance's interrupt is bound to its [`InterruptHandler`].
///
/// Which binding satisfies it is fixed per chip: the comparator is a source on an interrupt group on
/// most, wanting [`bind_group_interrupts!`](crate::bind_group_interrupts), and the owner of an NVIC
/// line on the rest, wanting [`bind_interrupts!`](crate::bind_interrupts). A binding written for the
/// wrong one names a type that does not exist rather than silently linking nothing.
///
/// # Safety
///
/// Implementing this without installing the handler lets a wait park on an interrupt that reaches
/// nothing. Use the macros.
pub unsafe trait CompInterrupt<T: Instance> {}

/// Which of the two `DACCODE` registers this driver programs.
///
/// The pair exists so the comparator's own output can switch between them for asymmetric
/// hysteresis, which `COMP_ERR_02` breaks — so only one is ever used and `DACSW` is left pointing
/// at it.
const DACCODE: usize = 0;

/// Which edge of the comparator output to wait for.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Edge {
    /// The output going high.
    Rising,
    /// The output going low.
    Falling,
    /// Either transition.
    Any,
}

/// Analog comparator driver.
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
pub struct Comp<'d, T: Instance, M: DriverMode> {
    positive: MaybeAnyPin<'d>,
    negative: MaybeAnyPin<'d>,
    output: MaybeAnyPin<'d>,
    _guard: MaybeWakeGuard,
    _phantom: PhantomData<(T, M)>,
}

impl<'d, T: Instance> Comp<'d, T, Blocking> {
    /// Configure the comparator, without an interrupt behind it.
    ///
    /// Either terminal may be left without a pin, in which case [`Config::reference`] is what drives
    /// it. Giving neither a pin nor a reference compares two undriven nodes.
    pub fn new(
        _peri: Peri<'d, T>,
        positive: Option<Peri<'d, impl PositivePin<T>>>,
        negative: Option<Peri<'d, impl NegativePin<T>>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::build(erase_positive(positive), erase_negative(negative), config)
    }
}

impl<'d, T: Instance> Comp<'d, T, Blocking> {
    /// Turn on the reference generator and its DAC, with no comparator inputs.
    ///
    /// The comparator is enabled, because nothing published says the DAC runs without it — TI's own
    /// `opa_dac8_output_buffer` enables it with no input channels and this does the same. Its output
    /// is not meaningful here and neither terminal is driven.
    ///
    /// What this is for is the DAC, which reaches the amplifier's input mux as well as the
    /// comparator's terminals. Pair it with
    /// `opa::NonInvertingInput::dac8`.
    ///
    /// Blocks for the comparator's enable time, so the DAC's output is at the code when this
    /// returns. Nothing reports readiness — there is no status bit — so the datasheet figure is
    /// waited out, and which of the two applies is decided by [`Config::speed`] rather than by the
    /// caller. It is 5 to 10 us across the families, the newer comparators being the faster.
    ///
    /// **Stated rather than guaranteed.** The datasheet cell spans its MIN, TYP and MAX columns, the
    /// same shape as the voltage reference's startup figure, so a board at a temperature or supply
    /// extreme could want longer and nothing here would report it.
    pub fn new_reference_only(_peri: Peri<'d, T>, reference: Reference, config: Config) -> Result<Self, ConfigError> {
        Self::build(
            None,
            None,
            Config {
                reference: Some(reference),
                ..config
            },
        )
    }
}

impl<'d, T: Instance> Comp<'d, T, Async> {
    /// Configure the comparator with its interrupt bound, so its output can be awaited.
    pub fn new_async(
        _peri: Peri<'d, T>,
        positive: Option<Peri<'d, impl PositivePin<T>>>,
        negative: Option<Peri<'d, impl NegativePin<T>>>,
        _irq: impl CompInterrupt<T> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::build(erase_positive(positive), erase_negative(negative), config)
    }

    /// Wait for the comparator's output to make a transition.
    ///
    /// A transition that already happened is not remembered: this returns on the next one after it
    /// is called.
    pub fn wait_for_edge(&mut self, edge: Edge) -> impl Future<Output = ()> {
        let r = T::regs();

        // Cleared before the mask goes on, so an edge from this point is reported rather than one
        // that arrived while nothing was waiting.
        r.cpu_int(0).iclr().write(|w| {
            w.set_compifg(true);
            w.set_compinvifg(true);
        });

        let mut armed = false;

        poll_fn(move |cx| {
            T::state().waker.register(cx.waker());

            let pending = r.cpu_int(0).ris().read();
            let seen = match edge {
                Edge::Rising => pending.compifg(),
                Edge::Falling => pending.compinvifg(),
                Edge::Any => pending.compifg() || pending.compinvifg(),
            };

            if armed && seen {
                r.cpu_int(0).iclr().write(|w| {
                    w.set_compifg(true);
                    w.set_compinvifg(true);
                });

                return Poll::Ready(());
            }

            armed = true;
            r.cpu_int(0).imask().write(|w| {
                w.set_compifg(matches!(edge, Edge::Rising | Edge::Any));
                w.set_compinvifg(matches!(edge, Edge::Falling | Edge::Any));
            });

            Poll::Pending
        })
    }
}

impl<'d, T: Instance, M: DriverMode> Comp<'d, T, M> {
    fn build(
        positive: Option<(Peri<'d, AnyPin>, u8)>,
        negative: Option<(Peri<'d, AnyPin>, u8)>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        // SYSCTL faults a fast comparator on LFCLK rather than merely running it slowly, so this is a
        // refusal and not a rounding (SLAU847 16.2.1, Clock Control).
        if matches!(config.speed, Speed::Fast) {
            let bus_lfclk = crate::sysctl::with_clocks(|clocks| clocks.ulpclk == clocks.lfclk);
            if bus_lfclk {
                return Err(ConfigError::FastModeOnLfclk);
            }
        }

        if config.exchange_inputs && !matches!(config.hysteresis, Hysteresis::None) && T::HYSTERESIS_BREAKS_ON_EXCHANGE
        {
            return Err(ConfigError::HysteresisWithExchangedInputs);
        }

        // An absent position selects no reference rather than faulting, so this is refused here
        // instead of leaving the comparator to report against a threshold that was never applied.
        if let Some(reference) = config.reference {
            if reference.source.needs_internal_reference() && !T::HAS_INTERNAL_REFERENCE {
                return Err(ConfigError::NoInternalReference);
            }
        }

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

        let positive_channel = positive.as_ref().map(|(_, channel)| *channel);
        let negative_channel = negative.as_ref().map(|(_, channel)| *channel);

        r.ctl0().write(|w| {
            if let Some(channel) = positive_channel {
                w.set_ipsel(channel);
                w.set_ipen(true);
            }
            if let Some(channel) = negative_channel {
                w.set_imsel(channel);
                w.set_imen(true);
            }
        });

        if let Some(reference) = config.reference {
            r.ctl3().write(|w| w.set_daccode(DACCODE, reference.code.to_bits()));

            r.ctl2().write(|w| {
                w.set_refsrc(reference.source.to_vals());
                w.set_refsel(match reference.terminal {
                    ReferenceTerminal::Negative => vals::Refsel::Negative,
                    ReferenceTerminal::Positive => vals::Refsel::Positive,
                });
                w.set_refmode(if reference.sampled {
                    vals::Refmode::Sampled
                } else {
                    vals::Refmode::Static
                });
                // The code comes from one `DACCODE` register under software control. The
                // alternative is the comparator's own output switching between two of them, which is
                // what `COMP_ERR_02` breaks, so it is not offered.
                w.set_dacctl(vals::Dacctl::DacswSel);
                w.set_dacsw(vals::Dacsw::Daccode0Sel);
            });
        }

        r.ctl1().write(|w| {
            w.set_mode(match config.speed {
                Speed::Fast => vals::Mode::Fast,
                Speed::UltraLowPower => vals::Mode::Ulp,
            });
            w.set_hyst(config.hysteresis.to_vals());
            w.set_outpol(match config.output_polarity {
                OutputPolarity::NonInverted => vals::Outpol::NonInv,
                OutputPolarity::Inverted => vals::Outpol::Inv,
            });
            w.set_exch(config.exchange_inputs);

            if let Some(delay) = config.filter {
                w.set_flten(true);
                w.set_fltdly(delay.to_vals());
            }
        });

        r.ctl1().modify(|w| w.set_enable(true));

        // Nothing reports when the comparator is ready -- there is no status bit -- so the datasheet
        // figure is waited out instead. Which of the two applies is decided by the mode this driver
        // just programmed, so the caller does not have to know.
        let enable_ns = match config.speed {
            Speed::Fast => T::ENABLE_FAST_NS,
            Speed::UltraLowPower => T::ENABLE_ULP_NS,
        };
        cortex_m::asm::delay(crate::sysctl::with_clocks(|clocks| wait_cycles(clocks.mclk, enable_ns)));

        // `COMP_ERR_05`: enabling raises both edge flags, so without this the first wait returns
        // immediately on an edge that never happened. Harmless where the erratum does not apply --
        // nothing has been armed yet, so there is nothing to lose.
        r.cpu_int(0).iclr().write(|w| {
            w.set_compifg(true);
            w.set_compinvifg(true);
            w.set_outrdyifg(true);
        });

        Ok(Self {
            positive: MaybeAnyPin::new(positive.map(|(pin, _)| pin)),
            negative: MaybeAnyPin::new(negative.map(|(pin, _)| pin)),
            output: MaybeAnyPin::none(),
            _guard: MaybeWakeGuard::new(Self::sleep_floor(negative_channel)),
            _phantom: PhantomData,
        })
    }

    /// Shallowest sleep level this configuration has to block.
    fn sleep_floor(negative_channel: Option<u8>) -> Option<SleepLevel> {
        let usable = T::SLEEP.floor_to_stay_usable();

        // `COMP_ERR_01`: with the negative terminal on channel 0, the output toggles on its own in
        // STANDBY0 whatever the inputs are. TI's workaround is to move the input, which this driver
        // cannot do behind the caller's back -- the channel is the pin they passed. So the device is
        // kept out of the mode instead.
        //
        // Judged on the configuration rather than applied always: a comparator on channel 1, or one
        // driven only by the reference, is unaffected and should still reach STANDBY0.
        let erratum = if T::TOGGLES_IN_STANDBY0_ON_CHANNEL_0 && negative_channel == Some(0) {
            Some(SleepLevel::Standby0)
        } else {
            None
        };

        SleepLevel::stricter(usable, erratum)
    }

    /// Whether the comparator's output is currently high.
    ///
    /// Reads through [`Config::output_polarity`] and any filter, so it answers the same question the
    /// interrupt does.
    pub fn is_high(&self) -> bool {
        T::regs().stat().read().out()
    }

    /// Whether the comparator's output is currently low.
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Change the reference DAC's code.
    ///
    /// Blocks for the DAC's settling time, so the threshold is at the new code when this returns —
    /// about 1.5 us, a full-scale step to within one LSB. Writes the register either way; the code
    /// simply drives nothing where the configured source bypasses the DAC.
    ///
    /// The figure is the internal path, which is what the comparator and an amplifier sampling the
    /// DAC both see. Driving it out to a pin is several times slower, and this driver does not.
    pub fn set_dac_code(&mut self, code: DacCode) {
        T::regs().ctl3().write(|w| w.set_daccode(DACCODE, code.to_bits()));
        cortex_m::asm::delay(crate::sysctl::with_clocks(|clocks| {
            wait_cycles(clocks.mclk, T::DAC_SETTLE_NS)
        }));
    }

    /// The code the reference DAC is programmed with.
    pub fn dac_code(&self) -> DacCode {
        DacCode::new(T::regs().ctl3().read().daccode(DACCODE))
    }

    /// Drive the comparator's output onto a pin.
    pub fn set_output_pin(&mut self, pin: Peri<'d, impl OutputPin<T>>) {
        SealedOutputPin::setup(&*pin);
        self.output = MaybeAnyPin::new(Some(pin.into()));
    }
}

impl<T: Instance, M: DriverMode> Drop for Comp<'_, T, M> {
    fn drop(&mut self) {
        let r = T::regs();

        r.cpu_int(0).imask().write(|_| {});
        r.ctl1().modify(|w| w.set_enable(false));

        // Not just tidiness: a reference generator left on holds VREF up and draws current with
        // nothing reading it.
        r.ctl2().modify(|w| w.set_refsrc(vals::Refsrc::Off));

        r.gprcm(0).pwren().write(|w| {
            w.set_enable(false);
            w.set_key(vals::PwrenKey::Key);
        });

        for pin in [&self.positive, &self.negative, &self.output]
            .into_iter()
            .filter_map(MaybeAnyPin::pin)
        {
            pin.set_as_disconnected();
        }
    }
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + LowPowerInstance {}

pub(crate) trait SealedInstance {
    /// Whether `COMP_ERR_03` applies: hysteresis is unstable with the inputs exchanged.
    const HYSTERESIS_BREAKS_ON_EXCHANGE: bool;

    /// Whether `COMP_ERR_01` applies: the output toggles in STANDBY0 with `IMSEL` at 0.
    const TOGGLES_IN_STANDBY0_ON_CHANNEL_0: bool;

    /// Whether `CTL2.REFSRC` positions 5, 6 and 7 select a source on this instance.
    const HAS_INTERNAL_REFERENCE: bool;

    /// Enable time in [`Speed::Fast`], nanoseconds, or 0 where the datasheet has no such row.
    const ENABLE_FAST_NS: u32;

    /// Enable time in [`Speed::UltraLowPower`], nanoseconds, or 0 where there is no such row.
    const ENABLE_ULP_NS: u32;

    /// Reference DAC settling after a code change, nanoseconds, or 0 where there is no such row.
    const DAC_SETTLE_NS: u32;

    fn regs() -> Regs;
    fn state() -> &'static State;
}

/// Configure a positive-terminal pin and take its channel, dropping the pin's own type.
fn erase_positive<'d, T: Instance>(pin: Option<Peri<'d, impl PositivePin<T>>>) -> Option<(Peri<'d, AnyPin>, u8)> {
    let pin = pin?;
    SealedPositivePin::<T>::setup(&*pin);
    let channel = SealedPositivePin::<T>::channel(&*pin);

    Some((pin.into(), channel))
}

/// Configure a negative-terminal pin and take its channel, dropping the pin's own type.
fn erase_negative<'d, T: Instance>(pin: Option<Peri<'d, impl NegativePin<T>>>) -> Option<(Peri<'d, AnyPin>, u8)> {
    let pin = pin?;
    SealedNegativePin::<T>::setup(&*pin);
    let channel = SealedNegativePin::<T>::channel(&*pin);

    Some((pin.into(), channel))
}

pub(crate) trait SealedPositivePin<T: Instance> {
    fn channel(&self) -> u8;
    fn setup(&self);
}

pub(crate) trait SealedNegativePin<T: Instance> {
    fn channel(&self) -> u8;
    fn setup(&self);
}

pub(crate) trait SealedOutputPin<T: Instance> {
    fn setup(&self);
}

/// A pin that can drive the comparator's positive terminal.
#[allow(private_bounds)]
pub trait PositivePin<T: Instance>: SealedPositivePin<T> + crate::gpio::Pin {}

/// A pin that can drive the comparator's negative terminal.
#[allow(private_bounds)]
pub trait NegativePin<T: Instance>: SealedNegativePin<T> + crate::gpio::Pin {}

/// A pin the comparator's output can be driven onto.
#[allow(private_bounds)]
pub trait OutputPin<T: Instance>: SealedOutputPin<T> + crate::gpio::Pin {}

/// The half of an instance impl that does not depend on how its interrupt is dispatched.
#[allow(unused_macros)]
macro_rules! impl_comp_instance_common {
    ($instance:ident, $int_vref:expr, $enable_fast:expr, $enable_ulp:expr, $dac_settle:expr) => {
        impl crate::comp::SealedInstance for crate::peripherals::$instance {
            const HYSTERESIS_BREAKS_ON_EXCHANGE: bool = cfg!(comp_err_03);
            const TOGGLES_IN_STANDBY0_ON_CHANNEL_0: bool = cfg!(comp_err_01);
            const HAS_INTERNAL_REFERENCE: bool = $int_vref;
            const ENABLE_FAST_NS: u32 = $enable_fast;
            const ENABLE_ULP_NS: u32 = $enable_ulp;
            const DAC_SETTLE_NS: u32 = $dac_settle;

            #[inline]
            fn regs() -> mspm0_metapac::comp::Comp {
                crate::pac::$instance
            }

            fn state() -> &'static crate::comp::State {
                static STATE: crate::comp::State = crate::comp::State::new();
                &STATE
            }
        }
    };
}

#[allow(unused_macros)]
macro_rules! impl_comp_instance {
    ($instance:ident, $int_vref:expr, $enable_fast:expr, $enable_ulp:expr, $dac_settle:expr) => {
        impl_comp_instance_common!($instance, $int_vref, $enable_fast, $enable_ulp, $dac_settle);

        impl crate::comp::Instance for crate::peripherals::$instance {}

        #[cfg(feature = "rt")]
        impl crate::interrupt_group::Handler<crate::interrupt_group::$instance>
            for crate::comp::InterruptHandler<crate::peripherals::$instance>
        {
            unsafe fn on_interrupt() {
                Self::handle();
            }
        }

        #[cfg(feature = "rt")]
        unsafe impl<T> crate::comp::CompInterrupt<crate::peripherals::$instance> for T where
            T: crate::interrupt_group::Binding<
                    crate::interrupt_group::$instance,
                    crate::comp::InterruptHandler<crate::peripherals::$instance>,
                >
        {
        }
    };
}

/// The same, for a chip where the comparator owns an NVIC line instead of sitting on a group.
#[allow(unused_macros)]
macro_rules! impl_comp_instance_nvic {
    ($instance:ident, $int_vref:expr, $enable_fast:expr, $enable_ulp:expr, $dac_settle:expr, $line:ident) => {
        impl_comp_instance_common!($instance, $int_vref, $enable_fast, $enable_ulp, $dac_settle);

        impl crate::comp::Instance for crate::peripherals::$instance {}

        #[cfg(feature = "rt")]
        impl crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::$line>
            for crate::comp::InterruptHandler<crate::peripherals::$instance>
        {
            unsafe fn on_interrupt() {
                Self::handle();
            }
        }

        #[cfg(feature = "rt")]
        unsafe impl<T> crate::comp::CompInterrupt<crate::peripherals::$instance> for T where
            T: crate::interrupt::typelevel::Binding<
                    crate::interrupt::typelevel::$line,
                    crate::comp::InterruptHandler<crate::peripherals::$instance>,
                >
        {
        }
    };
}

macro_rules! impl_comp_positive_pin {
    ($instance:ident, $pin:ident, $channel:expr) => {
        impl crate::comp::SealedPositivePin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn channel(&self) -> u8 {
                $channel
            }

            fn setup(&self) {
                use crate::gpio::SealedPin;
                self.set_as_analog();
            }
        }

        impl crate::comp::PositivePin<crate::peripherals::$instance> for crate::peripherals::$pin {}
    };
}

macro_rules! impl_comp_negative_pin {
    ($instance:ident, $pin:ident, $channel:expr) => {
        impl crate::comp::SealedNegativePin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn channel(&self) -> u8 {
                $channel
            }

            fn setup(&self) {
                use crate::gpio::SealedPin;
                self.set_as_analog();
            }
        }

        impl crate::comp::NegativePin<crate::peripherals::$instance> for crate::peripherals::$pin {}
    };
}

macro_rules! impl_comp_output_pin {
    ($instance:ident, $pin:ident, $pf:expr) => {
        impl crate::comp::SealedOutputPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn setup(&self) {
                use crate::gpio::{PfType, Pull, SealedPin};
                self.set_as_pf($pf, PfType::output(Pull::None, false));
            }
        }

        impl crate::comp::OutputPin<crate::peripherals::$instance> for crate::peripherals::$pin {}
    };
}

// The DAC's transfer function is the one number here that nothing else would catch: TI's own example
// uses `code = mV * 255 / ref`, which is a different curve that agrees at neither end.
const _: () = {
    // Code 0 is one LSB above ground, not at it.
    core::assert!(DacCode::ZERO.to_millivolts(2560) == 10);
    // Full scale is the reference itself.
    core::assert!(DacCode::FULL_SCALE.to_millivolts(2560) == 2560);
    // Round trip at a point neither end anchors.
    core::assert!(DacCode::from_millivolts(1280, 2560).to_bits() == 127);
    core::assert!(DacCode::new(127).to_millivolts(2560) == 1280);
    // Out of range saturates rather than wrapping.
    core::assert!(DacCode::from_millivolts(9999, 2560).to_bits() == 255);
    core::assert!(DacCode::from_millivolts(0, 2560).to_bits() == 0);
};
