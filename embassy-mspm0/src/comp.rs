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
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// Faster response, higher current.
    ///
    /// **Cannot run on LFCLK.** SYSCTL raises a clock error if a comparator is enabled in this mode
    /// while the bus clock is LFCLK, so [`Comp::new`] rejects the combination.
    Fast,

    /// Lower current, slower response. Runs on any bus clock.
    UltraLowPower,
}

impl Speed {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::Fast;
}

impl Default for Speed {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Built-in hysteresis, in millivolts of separation between the two switching thresholds.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hysteresis {
    /// No hysteresis: one threshold, and a slow input crosses it repeatedly on noise.
    None,

    /// About 10 mV.
    Mv10,

    /// About 20 mV.
    Mv20,

    /// About 30 mV.
    Mv30,
}

impl Hysteresis {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::None;

    const fn to_vals(self) -> vals::Hyst {
        match self {
            Hysteresis::None => vals::Hyst::NoHys,
            Hysteresis::Mv10 => vals::Hyst::LowHys,
            Hysteresis::Mv20 => vals::Hyst::MedHys,
            Hysteresis::Mv30 => vals::Hyst::HighHys,
        }
    }
}

impl Default for Hysteresis {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
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
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputPolarity {
    /// Output is high while the positive terminal is above the negative one.
    NonInverted,

    /// Output is inverted, and reads high while the comparator is disabled.
    Inverted,
}

impl OutputPolarity {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::NonInverted;
}

impl Default for OutputPolarity {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
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
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReferenceTerminal {
    /// The reference is the threshold a rising input crosses from below.
    Negative,

    /// The reference is the threshold a falling input crosses from above.
    Positive,
}

impl ReferenceTerminal {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::Negative;
}

impl Default for ReferenceTerminal {
    fn default() -> Self {
        Self::DEFAULT
    }
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

impl Reference {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            source: ReferenceSource::Vdda,
            terminal: ReferenceTerminal::DEFAULT,
            code: DacCode::ZERO,
            sampled: false,
        }
    }
}

impl Default for Reference {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Cycle counts for the two waits [`Comp`] takes, worked out ahead of time.
///
/// Built by [`Settling::solve`], which is `const`, so a caller whose clock tree is fixed can put the
/// whole computation in the compiler.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SettlingCycles {
    /// Comparator enable time, in MCLK cycles.
    enable: u32,

    /// Reference DAC settling after a code change, in MCLK cycles.
    dac: u32,
}

/// How the settling waits are worked out.
///
/// Both waits are blocking delays taken from the device's datasheet figures, because neither the
/// comparator's enable nor the DAC's settling has a status bit behind it. Turning a figure in
/// nanoseconds into a cycle count needs the clock rate, and where that rate comes from is what this
/// chooses.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Settling {
    /// Read MCLK from the live clock tree when the driver is built.
    ///
    /// Correct whatever the application does with clocks, and it carries the arithmetic into the
    /// binary — about 150 bytes, since dividing on this core is a library call.
    FromClockTree,

    /// Use counts worked out by [`Settling::solve`].
    ///
    /// **What folds is the arithmetic, not the wait.** The delay still happens, because the
    /// comparator still has to settle; what goes is the code that works out how long it should be.
    ///
    /// Two applications measured 188 and 212 bytes for pre-solving. Which you get depends on how many
    /// places build a comparator, since each keeps a call site whatever the counts come from. Choosing
    /// [`FromClockTree`](Self::FromClockTree) instead costs tens of bytes over having no choice at all
    /// — 20 on an isolated example and 52 on a whole firmware, and the figure belongs to the binary
    /// rather than to this enum.
    Solved(SettlingCycles),
}

impl Settling {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::FromClockTree;
}

impl Default for Settling {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl Settling {
    /// Work both waits out from a clock tree known at compile time.
    ///
    /// `clocks` comes from a [`ClockSetup`](crate::sysctl::clock::ClockSetup) in a `const` —
    /// `clock::RESET_SETUP.clocks()` for an application that leaves the tree alone. `speed` has to
    /// match [`Config::speed`], because the two enable figures differ and this picks between them.
    ///
    /// ```ignore
    /// const SETTLING: Settling = Settling::solve::<COMP0>(&clock::RESET_SETUP.clocks(), Speed::Fast);
    /// ```
    ///
    /// Nothing checks that `speed` agrees with the configuration: a mismatch waits the other mode's
    /// time, which is wrong in one direction and merely slow in the other.
    pub const fn solve<T: Instance>(clocks: &crate::sysctl::Clocks, speed: Speed) -> Self {
        let enable_ns = match speed {
            Speed::Fast => T::ENABLE_FAST_NS,
            Speed::UltraLowPower => T::ENABLE_ULP_NS,
        };

        Self::Solved(SettlingCycles {
            enable: wait_cycles(clocks.mclk, enable_ns),
            dac: wait_cycles(clocks.mclk, T::DAC_SETTLE_NS),
        })
    }

    /// Resolve to cycle counts, reading the live tree only where the caller did not pre-solve.
    ///
    /// The match is what makes pre-solving pay: on a `const` configuration the other arm is dead and
    /// takes `wait_cycles` and its helpers with it.
    fn resolve<T: Instance>(self, speed: Speed) -> SettlingCycles {
        match self {
            Settling::Solved(cycles) => cycles,
            Settling::FromClockTree => {
                let enable_ns = match speed {
                    Speed::Fast => T::ENABLE_FAST_NS,
                    Speed::UltraLowPower => T::ENABLE_ULP_NS,
                };

                crate::sysctl::with_clocks(|clocks| SettlingCycles {
                    enable: wait_cycles(clocks.mclk, enable_ns),
                    dac: wait_cycles(clocks.mclk, T::DAC_SETTLE_NS),
                })
            }
        }
    }
}

/// Comparator configuration.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
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

    /// Where the two settling waits get their cycle counts.
    ///
    /// Defaults to reading the live clock tree, which is right whatever the application does with
    /// clocks. [`Settling::solve`] is the alternative where the tree is fixed.
    pub settling: Settling,

    /// The reference generator, or [`None`] to leave it off.
    ///
    /// Leaving it off is not merely a default: an enabled generator holds VREF on and draws current
    /// whether or not anything reads it.
    pub reference: Option<Reference>,
}

impl Config {
    /// The default configuration, usable in a `const`.
    ///
    /// [`Default`] delegates here. This type is `#[non_exhaustive]`, so a caller outside the crate
    /// cannot write the struct literal, and `Default::default` is not `const` — without this there
    /// is no way to build a comparator's configuration in a `const` at all. That matters beyond
    /// tidiness: a constant cannot stop folding, where a `default()` call can once the struct grows
    /// past an inlining threshold, which has already cost this crate a clock tree's worth of
    /// constant propagation once.
    ///
    /// Pair it with [`Settling::solve`] to keep the settling arithmetic out of the binary too.
    pub const fn new() -> Self {
        Self {
            speed: Speed::DEFAULT,
            hysteresis: Hysteresis::DEFAULT,
            output_polarity: OutputPolarity::DEFAULT,
            filter: None,
            exchange_inputs: false,
            settling: Settling::DEFAULT,
            reference: None,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
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

    // Both quotients are bounded by their bit counts below, and both are checked. That is what lets
    // every multiply here be a plain one: `saturating_mul` is a *widening* multiply on this core and
    // links `__aeabi_lmul`, so the defensive version costs more than the case it defends against.
    let us = div_ceil_no_builtin(ns, 1_000, US_BITS);
    let (whole_mhz, rem_hz) = divmod_no_builtin(mclk, 1_000_000, MHZ_BITS);

    // A request or a clock past what the loops can represent. Waiting far too long is safe here and
    // waiting too little is not, so it saturates rather than wrapping into a short wait.
    if us > MAX_US || whole_mhz > MAX_MHZ {
        return u32::MAX;
    }

    // `us * whole_mhz` is at most 255 * 1023, and `us * rem_hz` at most 255 * 999_999. Both are
    // comfortably inside `u32`, which the assertions below pin down.
    let cycles = us * whole_mhz + div_ceil_no_builtin(us * rem_hz, 1_000_000, 9);

    if cycles == 0 { 1 } else { cycles }
}

/// Quotient bits allowed for nanoseconds-to-microseconds, and the largest quotient that leaves.
///
/// 255 us is two orders of magnitude past the longest settling figure any device publishes.
const US_BITS: u32 = 8;
const MAX_US: u32 = (1 << US_BITS) - 1;

/// The same for MCLK in whole megahertz. 1023 MHz is an order of magnitude past the fastest part.
const MHZ_BITS: u32 = 10;
const MAX_MHZ: u32 = (1 << MHZ_BITS) - 1;

/// `n / d` and `n % d`, by hand, because a division here links the software divider.
///
/// The divisors above are constants and it makes no difference: ARMv6-M has no widening multiply, so
/// the compiler cannot turn a constant divisor into a reciprocal multiply and reaches for
/// `__aeabi_uidiv` instead. That is 252 bytes of `compiler_builtins` in every binary that builds a
/// comparator, for arithmetic whose quotient never exceeds ten bits. `i2c`'s
/// `solve_clock_low_timeout` does the same thing for the same reason.
///
/// `bits` bounds the quotient and the caller proves it; `d << (bits - 1)` must not overflow.
const fn divmod_no_builtin(n: u32, d: u32, bits: u32) -> (u32, u32) {
    let mut rem = n;
    let mut quot = 0;
    let mut bit = bits;

    while bit > 0 {
        bit -= 1;
        let sub = d << bit;

        if rem >= sub {
            rem -= sub;
            quot |= 1 << bit;
        }
    }

    (quot, rem)
}

/// `n.div_ceil(d)`, on the same terms as [`divmod_no_builtin`].
const fn div_ceil_no_builtin(n: u32, d: u32, bits: u32) -> u32 {
    let (quot, rem) = divmod_no_builtin(n, d, bits);

    if rem == 0 { quot } else { quot + 1 }
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
    /// Resolved once here rather than per [`Comp::set_dac_code`], which is called from an interrupt
    /// in the applications this exists for.
    dac_settle_cycles: u32,
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
        Self::build(
            erase_positive(positive).map(|(pin, ch)| (Some(pin), ch)),
            erase_negative(negative),
            config,
        )
    }

    /// Configure the comparator, keeping the positive pad's own type so it can be lent back.
    ///
    /// The comparator behaves exactly as [`new`](Self::new) builds it. What differs is that the
    /// positive terminal's pin is kept as itself rather than erased, so
    /// [`CompSharedPositive::with_positive_pin`] can hand it to another driver -- an ADC channel
    /// being the case this exists for. Read that type's documentation before using it: the loan
    /// changes no registers, and it does not stop the comparator acting on what the other driver
    /// does to the pad.
    ///
    /// The positive terminal is required here, there being nothing to share otherwise.
    pub fn new_sharing_positive<P: PositivePin<T> + crate::gpio::Pin>(
        _peri: Peri<'d, T>,
        positive: Peri<'d, P>,
        negative: Option<Peri<'d, impl NegativePin<T>>>,
        config: Config,
    ) -> Result<CompSharedPositive<'d, T, Blocking, P>, ConfigError> {
        SealedPositivePin::<T>::setup(&*positive);
        let channel = SealedPositivePin::<T>::channel(&*positive);

        let comp = Self::build(Some((None, channel)), erase_negative(negative), config)?;

        Ok(CompSharedPositive { comp, pad: positive })
    }
}

impl<'d, T: Instance> Comp<'d, T, Blocking> {
    /// Arm the comparator's interrupt on `edge`, to be serviced by a handler the application owns.
    ///
    /// This is the escape hatch from [`wait_for_edge`](Comp::wait_for_edge). A wait is scheduled by
    /// the executor, so an edge is not acted on until the running task yields; where the response has
    /// to happen in interrupt context regardless of what else is runnable, the application supplies
    /// the handler and drives the comparator through these four methods.
    ///
    /// # The handler goes on the group, not on a vector of its own
    ///
    /// On most chips the comparator does not own an NVIC line — it is one source on an interrupt
    /// group, so there is no `COMP0` entry to define. Bind a handler to the source instead, and the
    /// group's demultiplexer calls it:
    ///
    /// ```rust,ignore
    /// struct RailHandler;
    ///
    /// impl interrupt_group::Handler<interrupt_group::COMP0> for RailHandler {
    ///     unsafe fn on_interrupt() {
    ///         // ... clear_interrupt, then set_dac_code, then arm the other edge
    ///     }
    /// }
    ///
    /// bind_group_interrupts!(struct Irqs {
    ///     COMP0 => RailHandler;
    ///     GPIOA => gpio::InterruptHandler;
    /// });
    /// ```
    ///
    /// Binding `GPIOA` alongside is what keeps the pin waits working: the two share the group, and
    /// the demultiplexer reaches only the sources that are bound. [`init`](crate::init) enables the
    /// group's NVIC line, so nothing else has to be unmasked.
    ///
    /// # Arming an edge does not clear the other one
    ///
    /// `RIS` is sticky. Alternating edges — which is what a threshold pair does — wants
    /// [`clear_interrupt`](Self::clear_interrupt) first, or a flag left over from before re-enters
    /// the handler the moment the other edge is armed. Clearing is left to the caller rather than
    /// folded in here, because doing it inside the arm would discard an edge that genuinely arrived
    /// while the handler was running.
    ///
    /// # Which flag means which edge
    ///
    /// `CTL1.IES` selects it, and this driver never writes that field: the register is written whole
    /// at configuration, so `IES` is zero and the rising edge is `COMPIFG`. That is what
    /// [`pending_edge`](Self::pending_edge) reports against.
    pub fn enable_edge_interrupt(&mut self, edge: Edge) {
        T::regs().cpu_int(0).imask().write(|w| {
            w.set_compifg(matches!(edge, Edge::Rising | Edge::Any));
            w.set_compinvifg(matches!(edge, Edge::Falling | Edge::Any));
        });
    }

    /// Stop the comparator's interrupt reaching the CPU, leaving the flags as they are.
    pub fn disable_edge_interrupt(&mut self) {
        T::regs().cpu_int(0).imask().write(|_| {});
    }

    /// Which edge the comparator has seen since the flags were last cleared, armed or not.
    ///
    /// Reads the raw flags rather than the masked ones, so it answers for a caller that polls without
    /// arming anything. [`Edge::Any`] means both are set, which a transition faster than the handler
    /// can produce.
    pub fn pending_edge(&self) -> Option<Edge> {
        let pending = T::regs().cpu_int(0).ris().read();

        match (pending.compifg(), pending.compinvifg()) {
            (true, true) => Some(Edge::Any),
            (true, false) => Some(Edge::Rising),
            (false, true) => Some(Edge::Falling),
            (false, false) => None,
        }
    }

    /// Clear both edge flags.
    pub fn clear_interrupt(&mut self) {
        T::regs().cpu_int(0).iclr().write(|w| {
            w.set_compifg(true);
            w.set_compinvifg(true);
        });
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
        Self::build(
            erase_positive(positive).map(|(pin, ch)| (Some(pin), ch)),
            erase_negative(negative),
            config,
        )
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

/// A [`Comp`] that kept the concrete type of its positive-terminal pad, so it can lend it back.
///
/// The comparator's positive terminal is often the only thing that wants a pad, and sometimes it is
/// not: the same pin can be an ADC channel, and an application may need to convert it between
/// comparisons. [`Comp`] erases its pins to a port and bit, which is enough to disconnect them on
/// drop and not enough to hand one to a driver that wants a named pin, so this keeps `P` instead.
///
/// # Nothing is reconfigured when the pad is lent out
///
/// **The hardware does not need arbitrating, and this type does not attempt any.** An analog
/// peripheral reaches a pad through its own input selection, and "analog peripherals have no
/// knowledge of, or interaction with, the IOMUX" (SLAU846 8.1). The comparator and the ADC each
/// select the pad from their own side, so both can be connected at once, and the pad's IOMUX state is
/// the same high-impedance default for either — `set_as_analog` writes the same value from both
/// drivers.
///
/// So [`with_positive_pin`](Self::with_positive_pin) changes no registers. The comparator keeps
/// running and keeps its terminal selected throughout.
///
/// # What it does buy, and what it does not
///
/// It buys one thing: the pad reaches a second driver only inside
/// [`with_positive_pin`](Self::with_positive_pin), which takes `&mut self`, so nothing can hold it
/// alongside the comparator or across a comparison.
///
/// **Two things it does not buy**, both the application's to handle:
///
/// - **It does not stop the pad being driven.** Inside the loan the pad is the whole pin, so a caller
///   can make an output of it -- and an output driver fighting an analog peripheral on one pad is the
///   one combination SLAU846 8.1 calls invalid. Reading it is what this is for.
/// - **It does not make the comparison meaningful during the loan.** Switching a divider onto the pad
///   changes what the comparator is comparing, and the comparator acts on it: an edge interrupt can
///   fire from the measurement rather than from the signal. Nothing here can know whether that
///   matters, so an application that cares has to quiet the comparator around the conversion.
pub struct CompSharedPositive<'d, T: Instance, M: DriverMode, P: crate::gpio::Pin> {
    comp: Comp<'d, T, M>,
    pad: Peri<'d, P>,
}

impl<'d, T: Instance, M: DriverMode, P: crate::gpio::Pin> CompSharedPositive<'d, T, M, P> {
    /// Run `f` with the positive terminal's pad, then take it back.
    ///
    /// The pad is the concrete pin, so it satisfies whatever a driver asks of it -- an ADC channel
    /// being the case this exists for. Nothing is reconfigured on the way in or out.
    #[inline]
    pub fn with_positive_pin<R>(&mut self, f: impl FnOnce(&mut Peri<'d, P>) -> R) -> R {
        f(&mut self.pad)
    }
}

impl<'d, T: Instance, M: DriverMode, P: crate::gpio::Pin> core::ops::Deref for CompSharedPositive<'d, T, M, P> {
    type Target = Comp<'d, T, M>;

    fn deref(&self) -> &Self::Target {
        &self.comp
    }
}

impl<'d, T: Instance, M: DriverMode, P: crate::gpio::Pin> core::ops::DerefMut for CompSharedPositive<'d, T, M, P> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.comp
    }
}

impl<'d, T: Instance, M: DriverMode, P: crate::gpio::Pin> Drop for CompSharedPositive<'d, T, M, P> {
    fn drop(&mut self) {
        // `Comp`'s own `Drop` disconnects the pins it owns, and this pad is not one of them.
        SealedPin::set_as_disconnected(&*self.pad);
    }
}

impl<'d, T: Instance, M: DriverMode> Comp<'d, T, M> {
    /// Build the driver, given each terminal's channel and, where the driver is to own it, its pin.
    ///
    /// The positive terminal's pin is optional *separately from its channel* so that
    /// [`CompSharedPositive`] can program the channel while keeping the pin itself, in the concrete
    /// type an ADC channel needs. `Drop` only disconnects the pins this owns.
    fn build(
        positive: Option<(Option<Peri<'d, AnyPin>>, u8)>,
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
        if let Some(reference) = config.reference
            && reference.source.needs_internal_reference()
            && !T::HAS_INTERNAL_REFERENCE
        {
            return Err(ConfigError::NoInternalReference);
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

        // The registers behind `PWREN` stay isolated for a few ULPCLK cycles and a write that lands in
        // that window is dropped. `tim::low_level::enable` carries the account.
        cortex_m::asm::delay(16);

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
        let settling = config.settling.resolve::<T>(config.speed);

        cortex_m::asm::delay(settling.enable);

        // `COMP_ERR_05`: enabling raises both edge flags, so without this the first wait returns
        // immediately on an edge that never happened. Harmless where the erratum does not apply --
        // nothing has been armed yet, so there is nothing to lose.
        r.cpu_int(0).iclr().write(|w| {
            w.set_compifg(true);
            w.set_compinvifg(true);
            w.set_outrdyifg(true);
        });

        Ok(Self {
            positive: MaybeAnyPin::new(positive.and_then(|(pin, _)| pin)),
            negative: MaybeAnyPin::new(negative.map(|(pin, _)| pin)),
            output: MaybeAnyPin::none(),
            dac_settle_cycles: settling.dac,
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
        cortex_m::asm::delay(self.dac_settle_cycles);
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
/// A comparator instance.
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
