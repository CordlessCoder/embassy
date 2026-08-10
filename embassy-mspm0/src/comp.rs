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
//! [`Reference`] turns the generator on and picks what feeds the DAC. `CTL2.REFSRC` has seven
//! defined values, of which this driver exposes the four that exist on every comparator:
//! [`ReferenceSource::Vdda`] and [`ReferenceSource::VrefModule`] run the DAC, and
//! [`ReferenceSource::VrefModuleDirect`] bypasses it. The upper three — which include a dedicated
//! internal reference that needs no VREF module — exist on some families and select nothing on the
//! rest, so they are left out until the metadata can say which.
//!
//! **`VrefModule` is an internal route, not the `VREF+` pin.** SLAU847 figure 16-5 labels it "From
//! VREF Module", so it reaches the comparator on devices that never buffer their reference out to a
//! pin. A live [`Vref`](crate::vref::Vref) is what it needs, not an external voltage.
//!
//! The DAC's output is `reference x (code + 1) / 256`, so [`DacCode::ZERO`] is one LSB above ground
//! rather than at it. TI's own `opa_dac8_output_buffer` example computes `code = mV * 255 / ref`,
//! which is a different function and up to two counts out at the top of scale — the datasheet's
//! `Vdac-code` row and the `CTL3` field description agree against it.
//!
//! # Errata this driver acts on
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

use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt_group::Binding;
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
/// Only the sources every comparator has. The three that exist on some families and select nothing
/// on the rest are deliberately absent — see the module docs.
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
    /// [`Vref`](crate::vref::Vref).
    VrefModule,

    /// The VREF module is the reference directly, with the DAC switched off.
    ///
    /// **Ignored while [`Reference::sampled`] is set**, where the input is connected to the DAC
    /// whatever this says.
    VrefModuleDirect,
}

impl ReferenceSource {
    const fn to_vals(self) -> vals::Refsrc {
        match self {
            ReferenceSource::Vdda => vals::Refsrc::VddaDac,
            ReferenceSource::VrefModule => vals::Refsrc::VrefDac,
            ReferenceSource::VrefModuleDirect => vals::Refsrc::Vref,
        }
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
        // adding half a step before the division. Done in `u64` because `mv * 512` overflows `u32`
        // above about 8.4 V — which no supply here reaches, but the guard costs nothing at compile
        // time and the alternative is a silent wrap if it ever does.
        let scaled = (millivolts as u64 * 512 + reference_mv as u64) / (reference_mv as u64 * 2);

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
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt_group::Handler<T::GroupSource> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();

        // Mask rather than clear: the flag is what tells the waiting future the edge happened, and
        // the handler has no way to hand a value over.
        r.cpu_int(0).imask().write(|_| {});
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
pub struct Comp<'d, T: Instance, M: DriverMode> {
    positive: Option<Peri<'d, AnyPin>>,
    negative: Option<Peri<'d, AnyPin>>,
    output: Option<Peri<'d, AnyPin>>,
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
    /// [`NonInvertingInput::dac8`](crate::opa::NonInvertingInput::dac8).
    ///
    /// The comparator needs its enable time — 10 us on the parts that publish it — before the DAC's
    /// output is at the code, and [`Comp::set_dac_code`] costs a further `tdac_settle` after that.
    /// Neither is waited for here, there being nothing to wait on: both are datasheet figures with no
    /// status bit behind them.
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
        _irq: impl Binding<T::GroupSource, InterruptHandler<T>> + 'd,
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

        // `COMP_ERR_05`: enabling raises both edge flags, so without this the first wait returns
        // immediately on an edge that never happened. Harmless where the erratum does not apply --
        // nothing has been armed yet, so there is nothing to lose.
        r.cpu_int(0).iclr().write(|w| {
            w.set_compifg(true);
            w.set_compinvifg(true);
            w.set_outrdyifg(true);
        });

        Ok(Self {
            positive: positive.map(|(pin, _)| pin),
            negative: negative.map(|(pin, _)| pin),
            output: None,
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
    /// Does nothing where the configured source does not run the DAC. The output settles within the
    /// datasheet's `tdac_settle` — 1.5 us on the parts that publish it — so a comparison made sooner
    /// is against a threshold still on its way.
    pub fn set_dac_code(&mut self, code: DacCode) {
        T::regs().ctl3().write(|w| w.set_daccode(DACCODE, code.to_bits()));
    }

    /// The code the reference DAC is programmed with.
    pub fn dac_code(&self) -> DacCode {
        DacCode::new(T::regs().ctl3().read().daccode(DACCODE))
    }

    /// Drive the comparator's output onto a pin.
    pub fn set_output_pin(&mut self, pin: Peri<'d, impl OutputPin<T>>) {
        SealedOutputPin::setup(&*pin);
        self.output = Some(pin.into());
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

        for pin in [self.positive.as_ref(), self.negative.as_ref(), self.output.as_ref()]
            .into_iter()
            .flatten()
        {
            pin.set_as_disconnected();
        }
    }
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + LowPowerInstance {
    /// The interrupt-group source this instance dispatches through.
    type GroupSource: crate::interrupt_group::Source;
}

pub(crate) trait SealedInstance {
    /// Whether `COMP_ERR_03` applies: hysteresis is unstable with the inputs exchanged.
    const HYSTERESIS_BREAKS_ON_EXCHANGE: bool;

    /// Whether `COMP_ERR_01` applies: the output toggles in STANDBY0 with `IMSEL` at 0.
    const TOGGLES_IN_STANDBY0_ON_CHANNEL_0: bool;

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

macro_rules! impl_comp_instance {
    ($instance:ident) => {
        impl crate::comp::SealedInstance for crate::peripherals::$instance {
            const HYSTERESIS_BREAKS_ON_EXCHANGE: bool = cfg!(comp_err_03);
            const TOGGLES_IN_STANDBY0_ON_CHANNEL_0: bool = cfg!(comp_err_01);

            #[inline]
            fn regs() -> mspm0_metapac::comp::Comp {
                crate::pac::$instance
            }

            fn state() -> &'static crate::comp::State {
                static STATE: crate::comp::State = crate::comp::State::new();
                &STATE
            }
        }

        impl crate::comp::Instance for crate::peripherals::$instance {
            type GroupSource = crate::interrupt_group::$instance;
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
