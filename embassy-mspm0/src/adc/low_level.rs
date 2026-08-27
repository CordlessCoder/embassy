//! Register-level ADC, for an application that services the interrupt itself.
//!
//! [`Adc`] here powers the instance up, programs a [`Config`] and stops. It starts no conversion,
//! waits for nothing and installs no interrupt handler — what it adds over the raw registers is the
//! clock selection, the sample-time comparators, and the sleep floor a conversion has to be guarded
//! against.
//!
//! [`Adc<Blocking>`](super::Adc) and [`Adc<Async>`](super::Adc) are both built on this and hold one.
//! Reach for this one when neither fits: an RTIC hardware task, another executor, or a conversion
//! driven from somewhere that cannot await.
//!
//! # What the caller takes on
//!
//! **The interrupt.** Nothing here routes a vector or unmasks the NVIC line. [`Adc::interrupt`] names
//! it, [`Adc::enable_interrupt`] arms sources within the peripheral, and [`Adc::arm_only`] arms one
//! and masks the rest — which is what a sequence wants, since only its last result should wake
//! anything.
//!
//! **Starting, and knowing when it finished.** [`Adc::start`] converts and returns immediately;
//! [`Adc::is_converting`] is the same flag the interrupt reflects, so a caller with nothing better to
//! do can spin on it instead of arming anything.
//!
//! **Not sleeping through it.** A conversion runs on ADCCLK, and a deep sleep that stops that clock
//! stops the conversion. [`Adc::conversion_floor`] is the shallowest level that keeps it running; hold
//! a [`WakeGuard`](crate::sysctl::WakeGuard) at it from before [`start`](Adc::start) until the result
//! is read.
//!
//! ```ignore
//! #[task(binds = ADC0, local = [adc])]
//! fn on_adc(cx: on_adc::Context) {
//!     let adc = cx.local.adc;
//!
//!     if adc.is_pending(Event::Result(0)) {
//!         adc.clear_pending(Event::Result(0));
//!         let code = adc.result(0);
//!         // ...
//!     }
//! }
//! ```
//!
//! # The instance parameter is kept on purpose
//!
//! `T` stays, so each method is compiled per instance and the register addresses fold to immediates.
//! [`Adc`](super::Adc) says why erasing it is not automatically the fix, and
//! [`uart::low_level`](crate::uart::low_level) is the driver that does erase it — and pays for it with
//! an attribute on every method, since a public method that is *not* generic escapes the instance's
//! static. Nothing here needs that, and adding an erased type to this module would.

use core::hint::unreachable_unchecked;
use core::marker::PhantomData;
use core::num::NonZeroU16;

use super::{
    ADC_MEMCTL, Averaging, BorrowedAdcChannel, BorrowedChannel, Config, Conversion, Instance, Resolution, SampleClock,
    ConversionMode, PowerDown, SampleClockSel, SampleTimeComparator, Vrsel, Window,
};
use crate::Peri;
use crate::interrupt::Interrupt;
use crate::pac::adc::{Adc as Regs, regs, vals};
use crate::sysctl::SleepLevel;

/// A condition the instance can raise its interrupt on.
///
/// One variant per source this driver's configuration can reach. `DMADONE` is not among them —
/// `configure` clears `CTL2.DMAEN` and nothing exposes it — and neither is the window comparator's
/// `INIFG`, which is a mask bit rather than a source.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Event {
    /// A conversion result landed in `MEMRES[n]`.
    ///
    /// Reading the result clears it, so a handler that takes the value need not also
    /// [`clear_pending`](Adc::clear_pending) — but clearing costs one write and makes the handler
    /// read the same either way.
    ///
    /// Panics if `n` is at or above the instance's `MEMCTL` count.
    Result(u8),

    /// A result was overwritten before anything read it.
    Overflow,

    /// A result register was read before a conversion had filled it.
    Underflow,

    /// A sequence did not finish inside its timeout.
    SequenceTimeout,

    /// A result was above the window comparator's high threshold.
    WindowHigh,

    /// A result was below the window comparator's low threshold.
    WindowLow,

    /// A result was inside the window comparator's band, ends included.
    ///
    /// The complement of [`Self::WindowHigh`] and [`Self::WindowLow`] taken together, and the one a
    /// caller waiting for a signal to *return* to range arms.
    WindowInRange,

    /// Any `MEMRES` result at all.
    ///
    /// A sequence arms only its last result and lets the earlier ones latch quietly, so a handler
    /// asking "did the conversion finish" wants the group rather than one index.
    /// [`take_active`](Adc::take_active) with this is what the asynchronous driver's own handler does.
    AnyResult,
}

impl Event {
    pub(crate) fn mask(self) -> regs::CpuInt {
        let mut mask = regs::CpuInt(0);

        match self {
            Event::Result(n) => {
                assert!(n < ADC_MEMCTL, "Event::Result is above this instance's MEMRES count");
                mask.set_memresifg(n as usize, true);
            }
            Event::Overflow => mask.set_ovifg(true),
            Event::Underflow => mask.set_uvifg(true),
            Event::SequenceTimeout => mask.set_tovifg(true),
            Event::WindowHigh => mask.set_highifg(true),
            Event::WindowLow => mask.set_lowifg(true),
            Event::WindowInRange => mask.set_inifg(true),
            Event::AnyResult => return RESULT_SOURCES,
        }

        mask
    }
}

/// A powered, configured ADC that converts nothing until told to.
///
/// See the [module documentation](self) for what the caller takes on.
pub struct Adc<'d, T: Instance> {
    #[allow(unused)]
    adc: Peri<'d, T>,
    /// Shallowest sleep to block while a conversion runs, or [`None`] if none needs blocking.
    ///
    /// The answer rather than the rate it was worked out from. Both inputs are known when the driver
    /// is built and the arithmetic is `const`, so this stores the answer.
    #[allow(unused)]
    sleep_floor: Option<SleepLevel>,
    _phantom: PhantomData<T>,
}

impl<T: Instance> Drop for Adc<'_, T> {
    fn drop(&mut self) {
        let r = T::info().regs;

        r.cpu_int(0).imask().write_value(regs::CpuInt(0));
        r.ctl0().modify(|w| w.set_enc(false));

        // `PWRDN` is `Manual`, so the analog block stays biased until the instance loses power. Every
        // other driver in the crate powers its instance down here.
        r.gprcm(0).pwren().write(|w| {
            w.set_enable(false);
            w.set_key(vals::PwrenKey::Key);
        });
    }
}

impl<'d, T: Instance> Adc<'d, T> {
    /// Power up an instance and apply `config`, leaving it converting nothing.
    pub fn new(peri: Peri<'d, T>, config: Config) -> Self {
        Self {
            adc: peri,
            sleep_floor: configure::<T>(config),
            _phantom: PhantomData,
        }
    }

    /// Registers of this instance, for what this driver does not wrap.
    #[inline]
    pub fn regs(&self) -> Regs {
        T::info().regs
    }

    /// The interrupt line this instance raises.
    ///
    /// Nothing here enables it. See the [module documentation](self).
    #[inline]
    pub fn interrupt(&self) -> Interrupt {
        T::info().interrupt
    }

    /// Shallowest sleep level that keeps a conversion running, if one needs blocking at all.
    ///
    /// Asks about ADCCLK rather than MCLK. The two used to be interchangeable, since the sample clock
    /// was always SYSOSC and MCLK always ran from it, but a configured tree can put HFCLK far above an
    /// LFCLK-sourced MCLK — where the MCLK answer would allow a sleep deep enough to stop the clock
    /// the conversion is running on.
    pub fn conversion_floor(&self) -> Option<SleepLevel> {
        self.sleep_floor
    }

    /// Resolution conversions are taken at.
    pub fn resolution(&self) -> Resolution {
        from_res(self.regs().ctl2().read().res())
    }

    /// Set the resolution for conversions from here on.
    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.regs().ctl2().modify(|w| w.set_res(to_res(resolution)));
    }

    /// Set one comparator's sample period, in ADC sample clock cycles.
    ///
    /// Panics if `period` is above [`Config::MAX_SAMPLE_PERIOD`].
    pub fn set_sample_period(&mut self, comparator: SampleTimeComparator, period: NonZeroU16) {
        assert!(period <= Config::MAX_SAMPLE_PERIOD);

        self.regs().scomp(comparator.index()).write(|w| w.set_val(period.get()));
    }

    /// One comparator's sample period, in ADC sample clock cycles.
    pub fn sample_period(&self, comparator: SampleTimeComparator) -> u16 {
        self.regs().scomp(comparator.index()).read().val()
    }

    /// Move the window comparator's thresholds, or turn it off, without rebuilding the driver.
    ///
    /// [`Config::window`] sets these at construction. This is what a caller adjusting a band while
    /// running needs — tracking a signal, or widening after a threshold has been crossed — since the
    /// thresholds are two registers and nothing about them is tied to the rest of the configuration.
    ///
    /// Checked against the resolution in force now, not the one the [`Config`] was built with, which
    /// matters because [`Self::set_resolution`] can have moved it since.
    pub fn set_window(&mut self, window: Option<Window>) {
        write_window::<T>(window, self.resolution());
    }

    /// Aim the next conversion at one channel, and set the window to it alone.
    ///
    /// The result lands in `MEMRES[0]`.
    pub fn set_conversion<'a>(&mut self, channel: impl BorrowedChannel<'a, T>, conversion: Conversion) {
        setup_one::<T>(channel.reborrow_adc().hw_channel(), conversion);
    }

    /// Aim the next conversion at a sequence of channels, in order.
    ///
    /// Result `n` lands in `MEMRES[n]`. Panics on an empty sequence, or one longer than
    /// [`MAX_SEQUENCE_LEN`](super::MAX_SEQUENCE_LEN).
    pub fn set_sequence<'a>(
        &mut self,
        sequence: impl ExactSizeIterator<Item = (BorrowedAdcChannel<'a, T>, Conversion)>,
    ) {
        assert!(sequence.len() != 0, "Read sequence cannot be empty");
        assert!(
            sequence.len() <= super::MAX_SEQUENCE_LEN,
            "Read sequence cannot be more than {} in length",
            super::MAX_SEQUENCE_LEN
        );

        setup_sequence::<T>(sequence.map(|(ch, conv)| (ch.hw_channel(), conv)));
    }

    /// Start converting what [`set_conversion`](Self::set_conversion) or
    /// [`set_sequence`](Self::set_sequence) last aimed at, and return.
    ///
    /// Starting while [`is_converting`](Self::is_converting) is true is a caller error: the running
    /// conversion is using the `MEMCTL` entries this would be starting against.
    pub fn start(&mut self) {
        start::<T>();
    }

    /// Whether a conversion is in flight.
    ///
    /// The same flag the completion interrupt reflects, so this is what to spin on with nothing else
    /// to do, and what to check on entering a handler.
    pub fn is_converting(&self) -> bool {
        is_converting::<T>()
    }

    /// The result in `MEMRES[index]`.
    ///
    /// Reading clears that result's [`Event::Result`] flag.
    pub fn result(&self, index: usize) -> u16 {
        result::<T>(index)
    }

    /// Let `event` reach the CPU, or stop it.
    ///
    /// A read-modify-write on `IMASK`, which an interrupt handler writes as well. Use
    /// [`arm_only`](Self::arm_only) where the whole mask is being set.
    pub fn enable_interrupt(&mut self, event: Event, enable: bool) {
        enable_interrupt::<T>(event, enable);
    }

    /// Let `event` reach the CPU and mask every other source, in one write.
    ///
    /// What a sequence wants: only its last result should raise the line, and the earlier ones each
    /// set a flag as they land.
    pub fn arm_only(&mut self, event: Event) {
        arm_only::<T>(event);
    }

    /// Whether `event` is latched, whether or not it is unmasked.
    ///
    /// Reads `RIS`, so it answers about the source and not about what raised the line. Where only
    /// some of the sources are armed — a sequence arms its last result alone — that is the wrong
    /// question, and [`take_active`](Self::take_active) is the right one.
    pub fn is_pending(&self, event: Event) -> bool {
        is_pending::<T>(event)
    }

    /// Whether `event` is latched **and** unmasked, clearing the part of it that is.
    ///
    /// What raised the line, which is what a handler dispatches on: one `MIS` read and at most one
    /// `ICLR` write, whatever the event covers. `Event::AnyResult` makes it the whole result group,
    /// which is the shape a sequence's handler wants.
    ///
    /// Leaves a latched-but-masked source alone, so a result the caller has not armed keeps its flag
    /// for whoever does ask.
    pub fn take_active(&mut self, event: Event) -> bool {
        take_active::<T>(event)
    }

    /// Drop `event`'s latched flag.
    pub fn clear_pending(&mut self, event: Event) {
        clear_pending::<T>(event);
    }
}

// ==== The register work the mode drivers share ====
//
// Everything below takes a bare register block rather than a `&self`, for the reason
// `uart::low_level` states at the same place: the async driver's futures capture the register block
// by value instead of borrowing their driver, so a primitive offered only as a method is one they
// have to open-code. A method above is a one-line wrapper over one of these, never the other way
// round.
//
// **Every one of them is generic over the instance rather than taking a register block, and that is
// the opposite of what `uart::low_level` does.** The mode driver here keeps its `T`, so
// monomorphising folds each register address to an immediate; passing the block as an argument
// instead costs it. Measured on a binary holding one blocking and one asynchronous ADC: 48 bytes,
// which is what the `Regs`-taking shape cost before this was corrected. `tim::low_level::Timer`
// documents the same trade from the other direction.
//
// `uart::low_level` takes the block because its driver has already erased the instance and has no `T`
// to monomorphise over. The shapes differ because the drivers do.

/// Program the peripheral, and return the shallowest sleep a conversion on it can tolerate.
///
/// Resolved here rather than kept as a rate: `floor_for_operation` is `const` and both its inputs are
/// known by the end of this function, so the driver stores the answer.
/// Program the window comparator's thresholds, or clear them.
///
/// The codes are raw and the hardware does not rescale them when the resolution changes, so they are
/// checked against whichever resolution is in force at the moment they are written. Clearing writes a
/// zero high threshold, which is what tells [`set_conversion`] no window is configured — and is why
/// [`Window::new`] rejects one.
fn write_window<T: Instance>(window: Option<Window>, resolution: Resolution) {
    let r = T::info().regs;

    let Some(window) = window else {
        r.wchigh().write(|w| w.set_data(0));
        r.wclow().write(|w| w.set_data(0));
        return;
    };

    let full_scale = resolution.max_count();

    assert!(
        window.high as u32 <= full_scale,
        "the window's high threshold is above the resolution's full scale"
    );
    assert!(
        window.low as u32 <= full_scale,
        "the window's low threshold is above the resolution's full scale"
    );

    r.wclow().write(|w| w.set_data(window.low));
    r.wchigh().write(|w| w.set_data(window.high));
}

pub(crate) fn configure<T: Instance>(config: Config) -> Option<SleepLevel> {
    assert!(config.sample_period_0 <= Config::MAX_SAMPLE_PERIOD);
    assert!(config.sample_period_1 <= Config::MAX_SAMPLE_PERIOD);

    let r = T::info().regs;
    let (source, adcclk_hz, sclkdiv, frange) = adc_clock_regs(config.sample_clk);

    // Under `Auto` the front end wakes before *every* sample window rather than once at enable, so a
    // window shorter than the wake-up samples an ADC that is not ready yet. The reading is a real
    // number rather than an error, which is what makes it worth refusing here.
    //
    // Checked against the shorter of the two comparators, because a sequence is free to use either
    // and the driver cannot know which conversions will. `wakeup_ns` prefers the datasheet's worst
    // case and falls back to its typical, so on the L1 families this bound is a typical -- see
    // `Config::WAKEUP_MAX_NS`.
    if matches!(config.power_down, PowerDown::Auto) {
        // The period counts SAMPCLK cycles, and SAMPCLK is ADCCLK *after* `SCLKDIV` -- whose
        // encoding is the shift, so the divisor is `1 << bits`. Using ADCCLK here instead would make
        // the window look four to eight times shorter than it is and reject configurations that are
        // fine.
        let sampclk_hz = (adcclk_hz >> sclkdiv.to_bits()) as u64;
        let shortest = config.sample_period_0.get().min(config.sample_period_1.get()) as u64;
        let window_ns = shortest * 1_000_000_000 / sampclk_hz;

        assert!(
            window_ns >= Config::wakeup_ns() as u64,
            "PowerDown::Auto needs a sample window at least Config::wakeup_ns() long"
        );
    }

    r.gprcm(0).rstctl().write(|w| {
        w.set_resetstkyclr(true);
        w.set_resetassert(true);
        w.set_key(vals::ResetKey::Key);
    });

    r.gprcm(0).pwren().modify(|reg| {
        reg.set_enable(true);
        reg.set_key(vals::PwrenKey::Key);
    });

    // Wait for power up
    cortex_m::asm::delay(16);

    r.gprcm(0).clkcfg().write(|w| {
        w.set_key(vals::ClkcfgKey::Key);
        w.set_sampclk(sampclk(source));
    });

    r.ctl0().write(|w| {
        w.set_enc(false);
        w.set_pwrdn(match config.power_down {
            PowerDown::Manual => vals::Pwrdn::Manual,
            PowerDown::Auto => vals::Pwrdn::Auto,
        });
        w.set_sclkdiv(sclkdiv);
    });

    r.clkfreq().write(|w| {
        w.set_frange(frange);
    });

    r.ctl1().write(|w| {
        w.set_trigsrc(vals::Trigsrc::Software);
        // Configured, not converting; a read starts it.
        w.set_sc(false);
        w.set_conseq(match config.conversion_mode {
            ConversionMode::Single => vals::Conseq::Single,
            ConversionMode::Sequence => vals::Conseq::Sequence,
            ConversionMode::RepeatSingle => vals::Conseq::Repeatsingle,
            ConversionMode::RepeatSequence => vals::Conseq::Repeatsequence,
        });
        w.set_sampmode(vals::Sampmode::Auto);

        // One rate for the peripheral; `Conversion::average` picks which conversions use it.
        let (avgn, avgd) = match config.averaging {
            Some(averaging) => averaging_regs(averaging),
            None => (vals::Avgn::Disable, 0),
        };
        w.set_avgn(avgn);
        w.set_avgd(avgd);
    });

    r.ctl2().write(|w| {
        // Binary unsigned
        w.set_df(false);
        w.set_res(to_res(config.resolution));
        w.set_rstsampcapen(false);
        w.set_dmaen(false);
        w.set_fifoen(false);
        w.set_sampcnt(0);
        w.set_startadd(0);
        w.set_endadd(0);
    });

    write_window::<T>(config.window, config.resolution);

    r.scomp(SampleTimeComparator::Scomp0.index()).write(|w| {
        w.set_val(config.sample_period_0.get());
    });

    r.scomp(SampleTimeComparator::Scomp1.index()).write(|w| {
        w.set_val(config.sample_period_1.get());
    });

    <T as crate::sysctl::LowPowerInstance>::SLEEP.floor_for_operation(adcclk_hz)
}

/// Program one `MEMCTL` entry.
pub(crate) fn write_memctl<T: Instance>(i: usize, ch: u8, conversion: Conversion) {
    let r = T::info().regs;

    // Read back rather than kept on the driver: the rate lives in `CTL1` from `Config`, and a copy
    // here could disagree with what is actually programmed.
    assert!(
        !conversion.average || r.ctl1().read().avgn() != vals::Avgn::Disable,
        "Conversion::average needs Config::averaging set"
    );
    // Same reasoning: read back rather than keep a copy. `Window::new` rejects a zero high
    // threshold, so a zero here means `Config::window` was `None` and nothing programmed one.
    assert!(
        !conversion.window || r.wchigh().read().data() != 0,
        "Conversion::window needs Config::window set"
    );


    r.memctl(i).write(|w| {
        w.set_chansel(ch);
        w.set_vrsel(vrsel(conversion.vrsel));
        w.set_stime(convert_stime(conversion.stime));
        w.set_avgen(conversion.average);
        w.set_bcsen(false);
        w.set_trig(vals::Trig::AutoNext);
        w.set_wincomp(conversion.window);
    });
}

/// Set the conversion window to `MEMCTL[0..=last]`.
pub(crate) fn set_window<T: Instance>(last: usize) {
    T::info().regs.ctl2().modify(|w| {
        w.set_startadd(0);
        w.set_endadd(last as u8);
    });
}

/// A sequence of one, without building an iterator for it.
///
/// Single-channel reads are the common case. Going through [`setup_sequence`] left a one-element loop
/// whose iterator stopped being inlined once a second instance gave it a second call site.
pub(crate) fn setup_one<T: Instance>(ch: u8, conversion: Conversion) {
    write_memctl::<T>(0, ch, conversion);
    set_window::<T>(0);
}

pub(crate) fn setup_sequence<T: Instance>(sequence: impl ExactSizeIterator<Item = (u8, Conversion)>) {
    let len = sequence.len();

    for (i, (ch, conversion)) in sequence.enumerate() {
        write_memctl::<T>(i, ch, conversion);
    }

    set_window::<T>(len - 1);
}

/// Convert what the `MEMCTL` window currently names.
pub(crate) fn start<T: Instance>() {
    let r = T::info().regs;

    r.ctl0().modify(|w| w.set_enc(true));
    r.ctl1().modify(|w| w.set_sc(true));
}

/// Whether a conversion is in flight.
pub(crate) fn is_converting<T: Instance>() -> bool {
    T::info().regs.ctl0().read().enc()
}

/// The result in `MEMRES[index]`, clearing that result's flag.
pub(crate) fn result<T: Instance>(index: usize) -> u16 {
    T::info().regs.memres(index).read().data()
}

/// Every `MEMRESx` result flag this instance has, built from the setters so it cannot drift.
///
/// The handler dispatches on "any result landed" and clears the lot, which no single [`Event`] names.
/// Built over the instance's own `MEMCTL` count rather than the register's full width, so it never
/// writes a flag the device does not have.
pub(crate) const RESULT_SOURCES: regs::CpuInt = {
    let mut w = regs::CpuInt(0);
    let mut i = 0;

    while i < ADC_MEMCTL as usize {
        w.set_memresifg(i, true);
        i += 1;
    }

    w
};

// What the handler's dispatch used to assume by hardcoding `0xFFFF_FF00`: the result flags sit above
// every shared source, so testing them as a group cannot pick up an overflow or a window comparison.
const _: () = core::assert!(RESULT_SOURCES.0 & 0xFF == 0);

/// Every source that is both latched and unmasked, in one read.
pub(crate) fn masked_status<T: Instance>() -> regs::CpuInt {
    T::info().regs.cpu_int(0).mis().read()
}

/// Drop the given sources' latched flags.
pub(crate) fn clear<T: Instance>(sources: regs::CpuInt) {
    T::info().regs.cpu_int(0).iclr().write_value(sources);
}

pub(crate) fn enable_interrupt<T: Instance>(event: Event, enable: bool) {
    let mask = event.mask().0;

    T::info().regs.cpu_int(0).imask().modify(|w| {
        w.0 = if enable { w.0 | mask } else { w.0 & !mask };
    });
}

/// Unmask `event` and mask everything else, in one write.
pub(crate) fn arm_only<T: Instance>(event: Event) {
    T::info().regs.cpu_int(0).imask().write_value(event.mask());
}

/// Clear whichever of `event`'s sources are both latched and unmasked, reporting whether any were.
pub(crate) fn take_active<T: Instance>(event: Event) -> bool {
    let active = masked_status::<T>().0 & event.mask().0;

    if active != 0 {
        clear::<T>(regs::CpuInt(active));
    }

    active != 0
}

pub(crate) fn is_pending<T: Instance>(event: Event) -> bool {
    T::info().regs.cpu_int(0).ris().read().0 & event.mask().0 != 0
}

pub(crate) fn clear_pending<T: Instance>(event: Event) {
    T::info().regs.cpu_int(0).iclr().write_value(event.mask());
}

// ==== Encoding a configuration into the register's fields ====
//
// The same seam `uart::low_level` draws: what a caller configures in is vocabulary and stays with
// `Config`, and turning that value into a register field is register work and lives here. Every one
// of these was defined beside the configuration types and called from nowhere but this file.

/// The `GPRCM.CLKCFG.SAMPCLK` encoding.
pub(crate) const fn sampclk(source: SampleClock) -> vals::Sampclk {
    match source {
        SampleClock::Ulpclk => vals::Sampclk::Ulpclk,
        SampleClock::Sysosc => vals::Sampclk::Sysosc,
        #[cfg(mspm0_hfxt)]
        SampleClock::Hfclk => vals::Sampclk::Hfclk,
    }
}

/// The `MEMCTL.VRSEL` encoding.
///
/// A match rather than a cast: the discriminants used to be written out and read back with `as u8`,
/// which made the `repr` load-bearing from a register write several hundred lines away.
///
/// The two negative-reference selections exist only where `MEMCTL.VRSEL` is five wide, which is the
/// same condition `adc_neg_vref` is generated from — so every variant that compiles is one this
/// device has, and there is nothing left to check at run time.
pub(crate) const fn vrsel(selection: Vrsel) -> vals::Vrsel {
    match selection {
        Vrsel::VddaVssa => vals::Vrsel::VddaVssa,
        Vrsel::ExtrefVrefm => vals::Vrsel::ExtrefVrefm,
        Vrsel::IntrefVssa => vals::Vrsel::IntrefVssa,
        #[cfg(adc_neg_vref)]
        Vrsel::VddaVrefm => vals::Vrsel::VddaVrefm,
        #[cfg(adc_neg_vref)]
        Vrsel::IntrefVrefm => vals::Vrsel::IntrefVrefm,
    }
}

/// The accumulate count and the matching right shift (SLAU846 table 18-1).
pub(crate) const fn averaging_regs(averaging: Averaging) -> (vals::Avgn, u8) {
    match averaging {
        Averaging::X2 => (vals::Avgn::Avg2, 1),
        Averaging::X4 => (vals::Avgn::Avg4, 2),
        Averaging::X8 => (vals::Avgn::Avg8, 3),
        Averaging::X16 => (vals::Avgn::Avg16, 4),
        Averaging::X32 => (vals::Avgn::Avg32, 5),
        Averaging::X64 => (vals::Avgn::Avg64, 6),
        Averaging::X128 => (vals::Avgn::Avg128, 7),
    }
}

const fn to_res(resolution: Resolution) -> vals::Res {
    match resolution {
        Resolution::Bits12 => vals::Res::Bit12,
        Resolution::Bits10 => vals::Res::Bit10,
        Resolution::Bits8 => vals::Res::Bit8,
    }
}

const fn from_res(res: vals::Res) -> Resolution {
    match res {
        vals::Res::Bit12 => Resolution::Bits12,
        vals::Res::Bit10 => Resolution::Bits10,
        vals::Res::Bit8 => Resolution::Bits8,
        // SAFETY: The HAL will never program an invalid value.
        vals::Res::_RESERVED_3 => unsafe { unreachable_unchecked() },
    }
}

const fn convert_stime(stime: SampleTimeComparator) -> vals::Stime {
    match stime {
        SampleTimeComparator::Scomp0 => vals::Stime::SelScomp0,
        SampleTimeComparator::Scomp1 => vals::Stime::SelScomp1,
    }
}

/// What ADCCLK runs at with `source` selected as the sample clock.
///
/// # Panics
/// If the source is outside `fADCCLK`. Selecting [`SampleClock::Hfclk`] without configuring HFCLK
/// reads as stopped and lands here, as does [`SampleClock::Ulpclk`] under an LFCLK-sourced MCLK.
fn adc_clock_hz(source: SampleClock) -> u32 {
    let hz = crate::sysctl::with_clocks(|clocks| match source {
        SampleClock::Ulpclk => clocks.ulpclk,

        // Switching SYSOSC off does not take the ADC with it: the TRM (G-series 18.2.5) has the ADC
        // request SYSOSC back at its base frequency for the duration of a conversion, so that is
        // the rate the registers have to be programmed for.
        SampleClock::Sysosc => match clocks.sysosc {
            0 => crate::sysctl::clock::SYSOSC_BASE_HZ,
            hz => hz,
        },

        #[cfg(mspm0_hfxt)]
        SampleClock::Hfclk => clocks.hfclk,
    });

    assert!(
        (ADC_CLK_MIN_HZ..=ADC_CLK_MAX_HZ).contains(&hz),
        "the selected ADC sample clock is stopped or outside this device's fADCCLK range"
    );

    hz
}

/// The source, the rate, and the `CTL0.SCLKDIV`/`CLKFREQ.FRANGE` pair `sel` implies.
///
/// One function rather than separate calls in `setup`: a second instance then shares one body,
/// instead of outlining the clock helpers and duplicating the divider ladder at each call site.
///
/// Nothing here runs for a [`SampleClockSel::Solved`], which is the point of solving.
///
/// `inline(always)` on the match and not on the body it calls. Left to itself LLVM outlines the
/// whole of this at two instances, and the shared copy keeps the solving arm — the tree read, both
/// ladders and the range check — alive for callers that solved at compile time and reach none of it.
/// Measured on `dup_adc2s`: 1940 bytes with one shared body against 1596 with the match folded.
#[inline(always)]
fn adc_clock_regs(sel: SampleClockSel) -> (SampleClock, u32, vals::Sclkdiv, vals::Frange) {
    match sel {
        SampleClockSel::Solved(solved) => (solved.source, solved.adcclk_hz, solved.sclkdiv, solved.frange),
        SampleClockSel::Source(source) => solve_clock_regs(source),
    }
}

/// Work the rate, divider and band out from the clock tree.
///
/// Deliberately out of line from [`adc_clock_regs`]: a second instance that also solves at run time
/// shares this one body instead of duplicating the divider ladder at its call site.
fn solve_clock_regs(source: SampleClock) -> (SampleClock, u32, vals::Sclkdiv, vals::Frange) {
    let adcclk = adc_clock_hz(source);
    (source, adcclk, sample_clock_div(adcclk), clock_range(adcclk))
}

/// `fADCCLK`, the range this device's datasheet specifies for the selected sample clock.
///
/// Per device, and narrower than [`FRANGE_MIN_HZ`]..[`FRANGE_MAX_HZ`]: MSPM0C1104 is 12-24 MHz where
/// most parts are 4-32 or 4-48, and it does not follow the family or the SYSCTL version — MSPM0G3507
/// and MSPM0G5187 share both and are 4-48 and 4-32 respectively.
pub(crate) const ADC_CLK_MIN_HZ: u32 = crate::_generated::ADC_CLK_MIN_HZ;
pub(crate) const ADC_CLK_MAX_HZ: u32 = crate::_generated::ADC_CLK_MAX_HZ;

/// Span of ADCCLK the `CLKFREQ.FRANGE` bands cover, from band 0's floor to band 7's ceiling.
///
/// A property of the register field, the same on every device. What the device actually supports is
/// [`ADC_CLK_MIN_HZ`]..[`ADC_CLK_MAX_HZ`], which is always inside this.
const FRANGE_MIN_HZ: u32 = 1_000_000;
const FRANGE_MAX_HZ: u32 = 48_000_000;

/// Rate this driver aims to run SAMPCLK at by default.
///
/// `SCOMPx` counts the sample window in SAMPCLK cycles, so holding SAMPCLK steady is what keeps
/// [`Config::sample_period_0`](crate::adc::Config::sample_period_0) a fixed duration across clock
/// trees. Nothing published bounds SAMPCLK itself; `fADCCLK` bounds ADCCLK *before* `SCLKDIV`, and
/// the datasheets' `tSample` is a minimum sample **window** rather than a minimum clock period.
///
/// **`tSample` is per device and this rate does not clear it on every part.** It is 62.5 ns on the
/// G-series and **156 ns on the L-series**, both at `RS` = 50 Ω and `Cpext` = 10 pF, and a caller's
/// own source impedance moves it further. One cycle of this clock is 125 ns, so a sample period of
/// 1 is a window under the L-series minimum. The window is
/// `sample_period / SAMPCLK`, so a period of 2 clears it; that is the caller's to get right, and
/// [`SolvedSampleClock::solve_at`](crate::adc::SolvedSampleClock::solve_at) says so.
pub(crate) const TARGET_SAMPCLK_HZ: u32 = 8_000_000;

/// Largest `SCLKDIV` this picks. Beyond it the divider ladder stops being powers of two.
const MAX_SCLKDIV_INDEX: u8 = vals::Sclkdiv::DivBy8.to_bits();

/// Smallest `CTL0.SCLKDIV` that brings `adcclk_hz` down to [`TARGET_SAMPCLK_HZ`] or below.
pub(crate) const fn sample_clock_div(adcclk_hz: u32) -> vals::Sclkdiv {
    sample_clock_div_to(adcclk_hz, TARGET_SAMPCLK_HZ)
}

/// Smallest `CTL0.SCLKDIV` that brings `adcclk_hz` down to `target_hz` or below.
///
/// A shift rather than a chain of comparisons: the first four `SCLKDIV` encodings are the powers of
/// two in order, so the encoding is the shift that reaches the target.
pub(crate) const fn sample_clock_div_to(adcclk_hz: u32, target_hz: u32) -> vals::Sclkdiv {
    let mut i = 0;

    while i < MAX_SCLKDIV_INDEX && adcclk_hz > target_hz << i {
        i += 1;
    }

    vals::Sclkdiv::from_bits(i)
}

/// The `CLKFREQ.FRANGE` band `adcclk_hz` falls in.
///
/// Describes ADCCLK itself, *before* `SCLKDIV`. A band that does not match the real input gives
/// "unintended results" (SLAU846 table 18-2). Bands are open at the bottom and closed at the top, so
/// a rate on a boundary belongs to the lower one.
pub(crate) const fn clock_range(adcclk_hz: u32) -> vals::Frange {
    let mut i = 0;

    while i < FRANGE_CEILINGS.len() - 1 && adcclk_hz > FRANGE_CEILINGS[i] as u32 * FRANGE_STEP_HZ {
        i += 1;
    }

    vals::Frange::from_bits(i as u8)
}

/// Unit the `FRANGE` band ceilings are all multiples of.
const FRANGE_STEP_HZ: u32 = 4_000_000;

/// Each `FRANGE` band's ceiling in [`FRANGE_STEP_HZ`] units, in band order.
///
/// A table rather than a chain of comparisons: eight 32-bit rates put eight literals in the constant
/// pool, and every ceiling divides by 4 MHz into a byte.
const FRANGE_CEILINGS: [u8; 8] = [1, 2, 4, 5, 6, 8, 10, 12];

const _: () = {
    use crate::sysctl::clock::SYSOSC_BASE_HZ;

    // Band edges, against table 18-2.
    core::assert!(matches!(clock_range(4_000_000), vals::Frange::Range1to4));
    core::assert!(matches!(clock_range(4_000_001), vals::Frange::Range4to8));
    core::assert!(matches!(clock_range(24_000_000), vals::Frange::Range20to24));
    core::assert!(matches!(clock_range(24_000_001), vals::Frange::Range24to32));
    core::assert!(matches!(clock_range(32_000_000), vals::Frange::Range24to32));
    core::assert!(matches!(clock_range(48_000_000), vals::Frange::Range40to48));

    // The reset tree keeps the divider the hardcoded value used to give, on either base frequency.
    // The band is where it differs: on a 24 MHz part the old `Range24to32` named a band SYSOSC
    // never reached.
    core::assert!(SYSOSC_BASE_HZ == 32_000_000 || SYSOSC_BASE_HZ == 24_000_000);
    core::assert!(matches!(sample_clock_div(SYSOSC_BASE_HZ), vals::Sclkdiv::DivBy4));

    // The 4 MHz SYSOSC operating point sits at the bottom of the `fADCCLK` range, where no division
    // is left to do.
    core::assert!(matches!(sample_clock_div(4_000_000), vals::Sclkdiv::DivBy1));

    // The window `adc_clock_hz` accepts is exactly the one `clock_range` can name: nothing below it
    // has a band, and `sample_clock_div` assumes nothing above it can occur.
    core::assert!(matches!(clock_range(FRANGE_MIN_HZ), vals::Frange::Range1to4));
    core::assert!(matches!(clock_range(FRANGE_MAX_HZ), vals::Frange::Range40to48));
    core::assert!(matches!(sample_clock_div(FRANGE_MAX_HZ), vals::Sclkdiv::DivBy8));

    // The device's own range has to sit inside what `FRANGE` can name, or a legal ADCCLK would have
    // no band to describe it.
    core::assert!(ADC_CLK_MIN_HZ >= FRANGE_MIN_HZ && ADC_CLK_MAX_HZ <= FRANGE_MAX_HZ);

    // SYSOSC at its base frequency is the reset sample clock, so it must be in range on every part
    // or the ADC is unusable before any tree is configured. The other sources depend on the tree.
    core::assert!(SYSOSC_BASE_HZ >= ADC_CLK_MIN_HZ && SYSOSC_BASE_HZ <= ADC_CLK_MAX_HZ);
};
