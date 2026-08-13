//! Analog to Digital Converter (ADC)

#![macro_use]

use core::future::poll_fn;
use core::hint::unreachable_unchecked;
use core::marker::PhantomData;
use core::num::NonZeroU16;
use core::task::Poll;

use embassy_hal_internal::PeripheralType;

use crate::interrupt::{Interrupt, InterruptExt};
use crate::mode::{Async, Blocking, Mode};
use crate::pac::adc::{Adc as Regs, regs, vals};
use crate::sync::irq_waker::IrqWaker;
use crate::sysctl::WakeGuard;
use crate::{Peri, interrupt};

/// Maximum length allowed for [`Adc::irq_read_sequence`].
pub const MAX_SEQUENCE_LEN: usize = ADC_MEMCTL as usize;

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::info().regs;
        let state = T::state();

        let mis = r.cpu_int(0).mis().read().0;

        // Check if any MEMRES bits were set. irq reads will enable the IRQ for the last channel in use.
        if mis >> 8 != 0 {
            // Clear the MEMRES interrupt bits.
            r.cpu_int(0).iclr().write_value(regs::CpuInt(mis & 0xFFFF_FF00));
            state.waker.wake();
        }
    }
}

/// Sample clock source for ADC, which becomes ADCCLK.
///
/// Constructing an [`Adc`] panics if the chosen source is stopped, or outside this device's
/// `fADCCLK` range — which is per device rather than per family: 4-48 MHz on a G3507, 4-32 MHz on an
/// L1306, 12-24 MHz on a C1104.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleClock {
    /// ULPCLK, the PD0 bus clock, which follows MCLK.
    ///
    /// Useful for a deterministic start of sampling, and the only source that lets several ADC
    /// instances sample simultaneously, since it is the same clock the trigger is timed against.
    Ulpclk,

    /// SYSOSC, at whatever operating point the clock tree left it.
    ///
    /// Unlike the others this cannot fail to be present: an ADC trigger with SYSOSC switched off
    /// makes the ADC request it back at its base frequency for the conversion.
    Sysosc,

    /// HFCLK, which has to be configured through [`sysctl::clock::Config::with_hfclk`].
    ///
    /// The lowest-jitter option, for when the sampling instant has to be accurate.
    ///
    /// [`sysctl::clock::Config::with_hfclk`]: crate::sysctl::clock::Config::with_hfclk
    #[cfg(mspm0_hfxt)]
    Hfclk,
}

impl SampleClock {
    const fn to_sampclk(self) -> vals::Sampclk {
        match self {
            Self::Ulpclk => vals::Sampclk::Ulpclk,
            Self::Sysosc => vals::Sampclk::Sysosc,
            #[cfg(mspm0_hfxt)]
            Self::Hfclk => vals::Sampclk::Hfclk,
        }
    }
}

/// How the sample clock reaches the hardware.
///
/// One value rather than a source and an optional override, because only one of the two would ever be
/// read — the same shape, and for the same reason, as [`uart::BaudRate`](crate::uart::BaudRate).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleClockSel {
    /// Read the clock tree on the device and work the divider and band out from it.
    ///
    /// **Not free.** Reading the tree defeats constant folding, so the divider ladder, the band
    /// ladder and the `fADCCLK` range check all stay in the binary: measured at `opt-level = "z"`
    /// with fat LTO, an `Adc` built this way costs **164 bytes of flash more** than one handed a
    /// solved clock, on a driver whose whole cost is 300.
    Source(SampleClock),

    /// Apply a divider and band solved ahead of time, skipping both ladders.
    ///
    /// Build one with [`SolvedSampleClock::solve`] in a `const` when the clock tree is fixed for the
    /// binary, which it is whenever nothing calls
    /// [`clock::Config`](crate::sysctl::clock::Config) at run time.
    Solved(SolvedSampleClock),
}

impl From<SampleClock> for SampleClockSel {
    fn from(source: SampleClock) -> Self {
        Self::Source(source)
    }
}

impl From<SolvedSampleClock> for SampleClockSel {
    fn from(solved: SolvedSampleClock) -> Self {
        Self::Solved(solved)
    }
}

/// A sample clock with its `CTL0.SCLKDIV` and `CLKFREQ.FRANGE` already worked out.
///
/// Built by [`solve`](Self::solve) in a `const`. The fields are private because they have to agree
/// with each other and with the rate they were solved for.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SolvedSampleClock {
    source: SampleClock,
    /// What ADCCLK runs at, kept for the sleep guard rather than for the registers.
    adcclk_hz: u32,
    sclkdiv: vals::Sclkdiv,
    frange: vals::Frange,
}

impl SolvedSampleClock {
    /// Solve the divider and band for `source` running at `adcclk_hz`, or [`None`] if that rate is
    /// outside this device's `fADCCLK`.
    ///
    /// `adcclk_hz` must be the rate `source` actually runs at. Take it from
    /// [`clock::Setup::clocks`](crate::sysctl::clock::Setup::clocks) on the tree the binary applies,
    /// which is a `const`.
    ///
    /// Usable in a `const`, which is the point: handing the result to
    /// [`Config::with_sample_clk`] keeps both ladders and the range check out of the binary.
    pub const fn solve(source: SampleClock, adcclk_hz: u32) -> Option<Self> {
        if adcclk_hz < ADC_CLK_MIN_HZ || adcclk_hz > ADC_CLK_MAX_HZ {
            return None;
        }

        Some(Self {
            source,
            adcclk_hz,
            sclkdiv: sample_clock_div(adcclk_hz),
            frange: clock_range(adcclk_hz),
        })
    }
}

/// Conversion resolution of the ADC results.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Resolution {
    /// 12-bits resolution
    Bits12,

    /// 10-bits resolution
    Bits10,

    /// 8-bits resolution
    Bits8,
}

impl Resolution {
    /// Number of bits of the resolution.
    #[inline]
    pub const fn bits(&self) -> u8 {
        match self {
            Resolution::Bits12 => 12,
            Resolution::Bits10 => 10,
            Resolution::Bits8 => 8,
        }
    }

    /// Get the maximum reading value for this resolution.
    ///
    /// This is `2**n - 1`.
    #[inline]
    pub const fn max_count(&self) -> u32 {
        (1 << self.bits()) - 1
    }
}

/// Hardware sample time comparator.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleTimeComparator {
    /// Use simple time comparator 0.
    Scomp0,

    /// Use sample time comparator 1.
    Scomp1,
}

impl SampleTimeComparator {
    /// Every comparator, in index order.
    pub const ALL: [SampleTimeComparator; 2] = [SampleTimeComparator::Scomp0, SampleTimeComparator::Scomp1];

    /// Index of this comparator in the `SCOMP` registers.
    pub const fn index(self) -> usize {
        match self {
            SampleTimeComparator::Scomp0 => 0,
            SampleTimeComparator::Scomp1 => 1,
        }
    }
}

/// Reference voltage (Vref) selection for the ADC channels.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Vrsel {
    /// VDDA reference
    VddaVssa,

    /// External reference from pin
    ExtrefVrefm,

    /// Internal reference
    ///
    /// The ADC requests the internal reference when a sample is triggered, and that request does not
    /// hold off the sample window. A conversion taken before the reference buffer has started returns
    /// an unreliable value rather than reporting anything.
    ///
    /// **Hold a [`Vref`](crate::vref::Vref) across any conversion that selects this.** Its constructor
    /// does not return until the reference has settled, and dropping it powers the reference down, so
    /// the borrow is what says the reference was up for the conversion. The 200 us that used to be
    /// documented here as a caller's delay is `Tstartup`, which is per device and spans 20x.
    IntrefVssa,

    /// VDDA and VREFM connected to VREF+ and VREF- of ADC
    #[cfg(adc_neg_vref)]
    VddaVrefm,

    /// INTREF and VREFM connected to VREF+ and VREF- of ADC
    ///
    /// Carries the same startup requirement as [`IntrefVssa`](Self::IntrefVssa).
    #[cfg(adc_neg_vref)]
    IntrefVrefm,
}

impl Vrsel {
    /// The `MEMCTL.VRSEL` encoding.
    ///
    /// A match rather than a cast: the discriminants used to be written out and read back with
    /// `as u8`, which made the `repr` load-bearing from a register write several hundred lines away.
    ///
    /// The two negative-reference selections exist only where `MEMCTL.VRSEL` is five wide, which is
    /// the same condition `adc_neg_vref` is generated from — so every variant that compiles is one
    /// this device has, and there is nothing left to check at run time.
    const fn to_vals(self) -> vals::Vrsel {
        match self {
            Self::VddaVssa => vals::Vrsel::VddaVssa,
            Self::ExtrefVrefm => vals::Vrsel::ExtrefVrefm,
            Self::IntrefVssa => vals::Vrsel::IntrefVssa,
            #[cfg(adc_neg_vref)]
            Self::VddaVrefm => vals::Vrsel::VddaVrefm,
            #[cfg(adc_neg_vref)]
            Self::IntrefVrefm => vals::Vrsel::IntrefVrefm,
        }
    }
}

/// How many conversions the hardware averages into one result.
///
/// Each setting divides by what it accumulated, so the result stays on the same scale as an
/// unaveraged one and only the noise changes. The accumulate-without-dividing mode the registers also
/// allow is not offered: it overflows `MEMRES`'s 16 bits at most resolutions and truncates silently.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Averaging {
    /// Average 2 conversions.
    X2,
    /// Average 4 conversions.
    X4,
    /// Average 8 conversions.
    X8,
    /// Average 16 conversions.
    X16,
    /// Average 32 conversions.
    X32,
    /// Average 64 conversions.
    X64,
    /// Average 128 conversions.
    X128,
}

impl Averaging {
    /// The accumulate count and the matching right shift (SLAU846 table 18-1).
    const fn to_regs(self) -> (vals::Avgn, u8) {
        match self {
            Self::X2 => (vals::Avgn::Avg2, 1),
            Self::X4 => (vals::Avgn::Avg4, 2),
            Self::X8 => (vals::Avgn::Avg8, 3),
            Self::X16 => (vals::Avgn::Avg16, 4),
            Self::X32 => (vals::Avgn::Avg32, 5),
            Self::X64 => (vals::Avgn::Avg64, 6),
            Self::X128 => (vals::Avgn::Avg128, 7),
        }
    }
}

/// Sample conversion parameters.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Conversion {
    // TODO: Bitpack a regs::Memctl for smaller size?
    /// Voltage reference selection.
    pub vrsel: Vrsel,

    /// Sample time period.
    pub stime: SampleTimeComparator,

    /// Average this conversion, at the rate [`Config::averaging`] sets.
    ///
    /// Per conversion because the hardware enables averaging per conversion but holds one rate for
    /// the whole peripheral, so a sequence can average some of its channels and not others. Setting
    /// this with no [`Config::averaging`] is a caller error and panics.
    pub average: bool,
    // TODO: BCS, TRIG, WINCOMP
}

impl Conversion {
    /// The default conversion, usable in a `const`.
    pub const fn new() -> Self {
        Self {
            vrsel: Vrsel::VddaVssa,
            stime: SampleTimeComparator::Scomp0,
            average: false,
        }
    }
}

impl Default for Conversion {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// ADC configuration.
#[derive(Copy, Clone)]
#[non_exhaustive]
pub struct Config {
    /// Resolution of the ADC conversion. The number of bits used to represent an ADC measurement.
    pub resolution: Resolution,

    /// Sample clock source, either to solve for on the device or already solved.
    pub sample_clk: SampleClockSel,

    /// Length of [`SampleTimeComparator::Scomp0`]'s sample period, in ADC sample clock cycles.
    ///
    /// The window has to charge the sampling capacitor through whatever the source impedance is, so
    /// how long it needs is a property of what is being measured rather than of the ADC. In 12-bit
    /// mode the datasheet asks for:
    ///
    /// | source | `tSample` |
    /// |---|---|
    /// | a pin, 50 ohm source | 156 ns |
    /// | through an OPA, gain x1 | 0.31 us |
    /// | through an OPA, gain x32 | 1.5 us |
    /// | through the general-purpose amplifier | 2.5 us |
    /// | the supply monitor | 3 us |
    /// | the temperature sensor, to settle | 10 us on the L-series, 12.5 us on the G-series |
    ///
    /// The driver holds SAMPCLK at or just under 8 MHz, so the default of fifty cycles is about
    /// 6.25 us. That covers every source in the table except the temperature sensor, which is the
    /// one that needs [`Config::sample_period_1`] or a raised default. A higher source impedance
    /// wants more as well: 50 ohms is lower than most sensors.
    ///
    /// Two comparators exist so a sequence can mix them, taking the short window for the pins and the
    /// long one for whatever needs it, rather than paying the longest for every conversion.
    //
    // Two fields rather than an array indexed by `SampleTimeComparator`: `Config` is taken by value
    // and an array field of it spills to the stack, which costs 48 bytes of flash in every binary
    // that builds an `Adc`.
    pub sample_period_0: NonZeroU16,

    /// Length of [`SampleTimeComparator::Scomp1`]'s sample period, in ADC sample clock cycles.
    pub sample_period_1: NonZeroU16,

    /// How many conversions to average, for conversions that ask for it.
    ///
    /// One rate for the whole peripheral: the hardware has a single accumulate count and cannot hold
    /// a different one per channel. Which conversions use it is [`Conversion::average`].
    ///
    /// A conversion takes this many times as long, and the driver holds its sleep guard for all of
    /// it.
    pub averaging: Option<Averaging>,
}

impl Config {
    /// Maximum number of sample clocks that may be performed when sampling.
    pub const MAX_SAMPLE_PERIOD: NonZeroU16 = NonZeroU16::new((1 << 9) - 1).unwrap();

    /// Take the sample clock from a [`SolvedSampleClock`], skipping both divider ladders.
    pub const fn with_sample_clk(mut self, solved: SolvedSampleClock) -> Self {
        self.sample_clk = SampleClockSel::Solved(solved);
        self
    }
}

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            resolution: Resolution::Bits12,
            sample_clk: SampleClockSel::Source(SampleClock::Sysosc),
            // Fifty sample clocks, which is 6.25 us at the 8 MHz SAMPCLK the divider aims for. The
            // datasheet's `tSample` for 12-bit mode is 156 ns at a 50 ohm source, so this is forty
            // times the minimum -- margin worth having, because that figure assumes a source
            // impedance almost nothing real has, and the window has to charge the sampling capacitor
            // through whatever the input actually is.
            //
            // It covers the internal sources too, bar one: 2.5 us through the general-purpose
            // amplifier and 3 us for the supply monitor both fit, and the temperature sensor's 10 to
            // 12.5 us does not. See [`Config::sample_period_0`].
            sample_period_0: NonZeroU16::new(50).unwrap(),
            sample_period_1: NonZeroU16::new(50).unwrap(),
            averaging: None,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Analog to Digital driver.
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
pub struct Adc<'d, T: Instance, M: Mode> {
    #[allow(unused)]
    adc: crate::Peri<'d, T>,
    /// What ADCCLK runs at, so the sleep guard knows how deep a conversion can afford to go.
    ///
    /// The rate rather than the source: resolving one to the other reads the clock tree, and doing
    /// that here would put the lookup back into a driver whose configuration already solved it.
    #[allow(unused)]
    adcclk_hz: u32,
    _mode: PhantomData<M>,
}

impl<'d, T: Instance> Adc<'d, T, Blocking> {
    /// Create a blocking ADC driver.
    pub fn new_blocking(peri: Peri<'d, T>, config: Config) -> Self {
        Adc {
            adc: peri,
            adcclk_hz: Self::setup(config),
            _mode: PhantomData,
        }
    }
}

impl<'d, T: Instance, M: Mode> Adc<'d, T, M> {
    /// Read an ADC pin.
    pub fn blocking_read<'a>(&mut self, channel: impl BorrowedChannel<'a, T>, conversion: Conversion) -> u16 {
        let r = T::info().regs;
        let channel = channel.reborrow_adc();

        // A sampling future dropped half way through leaves a conversion running.
        while r.ctl0().read().enc() {}

        Self::setup_one(channel.get_hw_channel(), conversion);

        r.ctl0().modify(|w| {
            w.set_enc(true);
        });

        r.ctl1().modify(|w| {
            w.set_sc(true);
        });

        // Wait for conversion
        while r.ctl0().read().enc() {}
        r.memres(0).read().data()
    }

    pub fn resolution(&self) -> Resolution {
        let r = T::info().regs;
        let ctl2 = r.ctl2().read();
        from_res(ctl2.res())
    }

    pub fn set_resolution(&mut self, resolution: Resolution) {
        let r = T::info().regs;

        r.ctl2().modify(|w| {
            w.set_res(to_res(resolution));
        });
    }

    /// Set one comparator's sample period, in ADC sample clock cycles.
    ///
    /// Panics if `period` is above [`Config::MAX_SAMPLE_PERIOD`].
    pub fn set_sample_period(&mut self, comparator: SampleTimeComparator, period: NonZeroU16) {
        assert!(period <= Config::MAX_SAMPLE_PERIOD);
        let r = T::info().regs;

        r.scomp(comparator.index()).write(|w| {
            w.set_val(period.get());
        });
    }

    /// One comparator's sample period, in ADC sample clock cycles.
    pub fn sample_period(&self, comparator: SampleTimeComparator) -> u16 {
        let r = T::info().regs;
        r.scomp(comparator.index()).read().val()
    }
}

impl<'d, T: Instance> Adc<'d, T, Async> {
    pub fn new_async(
        peri: Peri<'d, T>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let adcclk_hz = Self::setup(config);
        unsafe { T::info().interrupt.enable() };
        Self {
            adc: peri,
            adcclk_hz,
            _mode: PhantomData,
        }
    }

    /// Shallowest sleep level to block while a conversion is in flight.
    fn conversion_guard(&self) -> Option<WakeGuard> {
        // Asks about ADCCLK rather than MCLK. The two used to be interchangeable, since the sample
        // clock was always SYSOSC and MCLK always ran from it, but a configured tree can now put
        // HFCLK far above an LFCLK-sourced MCLK — where the MCLK answer would allow a sleep deep
        // enough to stop the clock the conversion is running on.
        <T as crate::sysctl::LowPowerInstance>::SLEEP
            .floor_for_operation(self.adcclk_hz)
            .map(WakeGuard::new)
    }

    /// Read an ADC pin asynchronously using the irq handler.
    pub async fn irq_read<'a>(&mut self, channel: impl BorrowedChannel<'a, T>, conversion: Conversion) -> u16 {
        let _guard = self.conversion_guard();
        let r = T::info().regs;
        let channel = channel.reborrow_adc();

        // Wait until ADC is not converting to start - an active conversion might've been cancelled.
        Self::wait_for_conversion().await;
        Self::setup_one(channel.get_hw_channel(), conversion);

        // Write is used to zero the other MEMRES interrupt bits.
        r.cpu_int(0).imask().write(|w| {
            w.set_memresifg(0, true);
        });

        r.ctl0().modify(|w| {
            w.set_enc(true);
        });

        r.ctl1().modify(|w| {
            w.set_sc(true);
        });

        Self::wait_for_conversion().await;
        r.memres(0).read().data()
    }

    /// Read one or multiple ADC regular channels using the irq handler.
    ///
    /// `sequence` iterator and `readings` must have the same length.
    pub async fn irq_read_sequence<'a>(
        &mut self,
        sequence: impl ExactSizeIterator<Item = (BorrowedAdcChannel<'a, T>, Conversion)>,
        readings: &mut [u16],
    ) {
        assert!(sequence.len() != 0, "Read sequence cannot be empty");
        assert!(
            sequence.len() == readings.len(),
            "Sequence length must be equal to readings length"
        );
        assert!(
            sequence.len() <= MAX_SEQUENCE_LEN,
            "Asynchronous read sequence cannot be more than {} in length",
            MAX_SEQUENCE_LEN
        );

        let _guard = self.conversion_guard();
        let sequence_len = sequence.len();
        let r = T::info().regs;

        Self::wait_for_conversion().await;
        Self::setup_sequence(sequence.map(|(ch, conv)| (ch.get_hw_channel(), conv)));

        // Only wake up when the last bit is set.
        //
        // Write is used to zero the other MEMRES interrupt bits.
        r.cpu_int(0).imask().write(|w| {
            w.set_memresifg(sequence_len - 1, true);
        });

        r.ctl0().modify(|w| {
            w.set_enc(true);
        });

        r.ctl1().modify(|w| {
            w.set_sc(true);
        });

        Self::wait_for_conversion().await;

        for (i, reading) in readings.iter_mut().enumerate() {
            *reading = r.memres(i).read().data();
        }
    }

    // TODO: DMA driven ADC
}

/// Peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: PeripheralType + SealedInstance + crate::sysctl::LowPowerInstance + 'static {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// A type-erased borrowed channel for the given ADC instance.
///
/// The borrowed channel cannot consume the channel source because it might need to run drop code.
pub struct BorrowedAdcChannel<'a, T> {
    pub(crate) channel: u8,
    pub(crate) _marker: PhantomData<&'a mut T>,
}

impl<T> BorrowedAdcChannel<'_, T> {
    pub fn get_hw_channel(&self) -> u8 {
        self.channel
    }
}

impl<T: Instance> AdcChannel<T> for BorrowedAdcChannel<'_, T> {}
impl<T: Instance> SealedAdcChannel<T> for BorrowedAdcChannel<'_, T> {
    fn channel(&self) -> u8 {
        self.channel
    }
}

#[allow(private_bounds)]
pub trait BorrowedChannel<'a, T>: SealedBorrowedChannel<'a, T> {}
impl<'a, T, C: SealedBorrowedChannel<'a, T>> BorrowedChannel<'a, T> for C {}

impl<'a, T, C: AdcChannel<T>> SealedBorrowedChannel<'a, T> for &'a mut C {
    #[inline]
    fn reborrow_adc(self) -> BorrowedAdcChannel<'a, T> {
        self.reborrow_adc()
    }
}

impl<'a, T> SealedBorrowedChannel<'a, T> for BorrowedAdcChannel<'a, T> {
    #[inline]
    fn reborrow_adc(self) -> BorrowedAdcChannel<'a, T> {
        self
    }
}

/// ADC channel.
#[allow(private_bounds)]
pub trait AdcChannel<T>: SealedAdcChannel<T> + Sized {
    #[allow(unused_mut)]
    fn reborrow_adc<'a>(&'a mut self) -> BorrowedAdcChannel<'a, T> {
        self.setup();

        BorrowedAdcChannel {
            channel: self.channel(),
            _marker: PhantomData,
        }
    }
}

/// The on-die temperature sensor, as an ADC channel.
///
/// The sensor needs no pin and nothing switches it on: it is wired to a fixed channel of every ADC
/// that reaches it, and selecting that channel is the whole of using it. On a part with two ADCs it
/// is the same sensor from either, on a different channel number, so this implements
/// [`AdcChannel`] once per ADC and either reads it.
///
/// # It needs a longer sample window than the default
///
/// [`Config::sample_period_0`]'s default is around 6.25 us, and the datasheet's `tSET,TS` is the
/// **minimum** sampling time for this channel -- 10 us on the L-series, 12.5 us on the G-series. A
/// shorter window samples the capacitor before it has charged and reads low by a margin nothing
/// reports. TI's own configuration for this measurement asks for 50 us.
///
/// # Turning a reading into a temperature
///
/// The driver hands back the ADC code and stops there, because the two constants needed are per
/// device and stated only in its datasheet: `TSc`, the sensor's slope in mV/degC, and `TSTRIM`, the
/// temperature the factory calibration was taken at. [`temp_calibration_code`] reads the third,
/// which is per unit. The relation is SLAU846 equation 17.
///
/// # Which reference the calibration used is per device, and the datasheets are not reliable on it
///
/// The stored code only means a voltage once you know the reference it was taken against, and a
/// conversion run against a different one has to be rescaled before the two codes can be compared.
/// **The answer differs between families and one datasheet gives both answers**: the L-series states
/// the 1.4 V internal reference in its specification table and VDD in its detailed description, and
/// the G-series states VDD. Measured on an L1306, the specification table is the correct one there.
///
/// The device will tell you, and this is worth doing before trusting any absolute reading. Take
/// [`temp_calibration_code`] and work out what voltage it stands for under each candidate reference:
/// only one of them is a temperature sensor output, and the wrong choice is wrong by hundreds of
/// degrees rather than by a few.
#[cfg(adc_temp_sensor)]
pub struct TempSensor;

#[cfg(adc_temp_sensor)]
impl TempSensor {
    /// `TSTRIM`, the temperature the factory calibration was taken at.
    ///
    /// 30 on every device so far. Its own tolerance is the floor on absolute accuracy: the
    /// datasheets specify the trim as having been taken somewhere in 27 to 33 degrees, and no
    /// arithmetic here recovers that.
    pub const TRIM_CELSIUS: i16 = crate::_generated::TEMP_SENSOR_TRIM_C;

    /// `TSc`, the sensor's slope. Always negative: the output falls as the die warms.
    pub const SLOPE_UV_PER_C: i32 = crate::_generated::TEMP_SENSOR_SLOPE_UV_PER_C;

    /// `tSET,TS`, how long the sensor takes to settle once selected.
    pub const SETTLING_NS: Option<u32> = crate::_generated::TEMP_SENSOR_SETTLING_NS;

    /// The sample window the factory measurement itself used.
    ///
    /// Not always [`SETTLING_NS`](Self::SETTLING_NS), and the difference is the quiet one. "Long
    /// enough for the sensor to settle" and "the window the stored code was taken with" are
    /// different conditions, and only the second makes a live reading directly comparable with the
    /// stored code. A window that settles but is shorter than the factory's reads plausibly and
    /// drifts.
    pub const CALIBRATION_SAMPLE_NS: Option<u32> = crate::_generated::TEMP_SENSOR_CALIBRATION_SAMPLE_NS;

    /// The wider of the two windows above, which satisfies both. What to set the sample period from.
    pub const RECOMMENDED_SAMPLE_NS: Option<u32> = crate::_generated::TEMP_SENSOR_RECOMMENDED_SAMPLE_NS;

    /// The reference [`temp_calibration_code`] was measured against.
    ///
    /// Three values across the portfolio -- the supply, the 1.4 V internal reference, or the 4.05 V
    /// one -- and it splits families that sit next to each other in the part numbering. Nominal for
    /// the supply case: a board running off something other than 3.3 V has a trim voltage to match,
    /// and only the application knows.
    pub const CALIBRATION_REFERENCE_MV: u32 = crate::_generated::TEMP_SENSOR_CALIBRATION_REFERENCE_MV;

    /// Convert a reading of this sensor to millidegrees Celsius.
    ///
    /// `reference_mv` is the reference **the conversion** ran against, which is not always the one
    /// the factory used and is why this cannot be a one-argument call. For
    /// [`Vrsel::VddaVssa`] it is the supply the board actually runs at; the driver has no way to
    /// know that, and on a device calibrated against the supply a wrong value scales the answer.
    ///
    /// The calculation is SLAU846 equation 17, in integer arithmetic throughout. Both codes are
    /// turned into voltages before being compared, which is what lets the two references differ.
    ///
    /// Within 0.04 degrees of exact arithmetic anywhere in -40 to 130 degrees, across every slope
    /// and calibration reference in the portfolio. That is two orders of magnitude inside what the
    /// trim's own 27-to-33-degree spread allows, so the fixed point is not what limits the answer.
    pub fn celsius_millidegrees(code: u16, resolution: Resolution, reference_mv: u32) -> i32 {
        let sample_uv = code_to_microvolts(code, resolution, reference_mv);
        let trim_uv = code_to_microvolts(
            temp_calibration_code(),
            Resolution::Bits12,
            Self::CALIBRATION_REFERENCE_MV,
        );

        // The reciprocal folds at compile time -- the slope is a constant, and dividing by it here
        // would link a division routine on a core that has neither a divide instruction nor a
        // widening multiply to build one from.
        const MILLIDEGREES_PER_UV_Q12: i32 =
            (1000i64 * 4096 / crate::_generated::TEMP_SENSOR_SLOPE_UV_PER_C as i64) as i32;

        // Clamped so the multiply below cannot overflow whatever reference it was handed. The bound
        // is several hundred degrees from the trim point, so it engages only on a reading that was
        // already meaningless -- and clamping keeps that monotone where wrapping would not.
        let delta_uv = (sample_uv as i32 - trim_uv as i32).clamp(-800_000, 800_000);
        let offset_mc = (delta_uv * MILLIDEGREES_PER_UV_Q12) >> 12;

        Self::TRIM_CELSIUS as i32 * 1000 + offset_mc
    }
}

/// The largest reference [`code_to_microvolts`] can be handed.
///
/// Twice the highest reference any MSPM0 has, and set by where the arithmetic below stops fitting in
/// 32 bits rather than by anything electrical.
#[cfg(adc_temp_sensor)]
const MAX_REFERENCE_MV: u32 = 8000;

/// An ADC code as microvolts, given the reference it was taken against.
///
/// Codes below 12 bits are shifted up rather than scaled, so one function serves the stored
/// calibration code and a conversion at any resolution.
///
/// Panics above [`MAX_REFERENCE_MV`]. Callers pass a constant here in the ordinary case, and the
/// check folds away with it.
#[cfg(adc_temp_sensor)]
fn code_to_microvolts(code: u16, resolution: Resolution, reference_mv: u32) -> u32 {
    assert!(reference_mv <= MAX_REFERENCE_MV);

    let code = (code as u32) << (12 - resolution.bits());

    // `code * reference_mv * 1000 >> 12`, with the constants reduced so the product stays inside 32
    // bits. Plain multiplies: a `saturating_mul` here is not free on this core, which detects the
    // overflow with a widening multiply and links a 64-bit routine to do it.
    (code * reference_mv * 125) >> 9
}

/// Read this unit's temperature sensor calibration code from `FACTORYREGION.TEMP_SENSE0`.
///
/// The code the factory measured from this device's own sensor at the trim temperature, as a 12-bit
/// ADC result. It is what makes a reading absolute rather than relative: the slope is a family
/// figure, the offset is per unit, and this is the offset.
///
/// See [`TempSensor`] for what else the conversion needs and which reference the value is against.
#[cfg(adc_temp_sensor)]
#[must_use]
pub fn temp_calibration_code() -> u16 {
    // Every one of TI's factory-constant accessors takes the read with the cache off and restores
    // CPUSS.CTL after, which is the only statement anywhere of how this region wants to be read.
    // `flash.rs` does the same for the geometry constants.
    let saved = crate::pac::CPUSS.ctl().read();
    let mut suspended = saved;
    suspended.set_prefetch(false);
    suspended.set_icache(false);
    suspended.set_liten(false);
    crate::pac::CPUSS.ctl().write_value(suspended);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // DATA is the whole word rather than a field within it, so there is nothing to mask off; what
    // it holds is a 12-bit conversion result.
    let code = crate::pac::FACTORYREGION.temp_sense0().read();

    crate::pac::CPUSS.ctl().write_value(saved);

    code as u16
}

// Impl details

const ADC_VRSEL: u8 = crate::_generated::ADC_VRSEL;
const ADC_MEMCTL: u8 = crate::_generated::ADC_MEMCTL;

// What lets `Vrsel::to_vals` be a total match with no run-time bounds check: every variant that
// compiles is one this device has. `adc_neg_vref` and this count come from the same metapac field, so
// they cannot disagree today — this is what would notice if a variant were ever added without a `cfg`,
// or if the field grew a third value.
const _: () = core::assert!(ADC_VRSEL == if cfg!(adc_neg_vref) { 5 } else { 3 });

impl<'d, T: Instance, M: Mode> Adc<'d, T, M> {
    /// Program the peripheral, and return what ADCCLK ended up running at.
    fn setup(config: Config) -> u32 {
        assert!(config.sample_period_0 <= Config::MAX_SAMPLE_PERIOD);
        assert!(config.sample_period_1 <= Config::MAX_SAMPLE_PERIOD);

        let r = T::info().regs;
        let (source, adcclk_hz, sclkdiv, frange) = adc_clock_regs(config.sample_clk);

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
            w.set_sampclk(source.to_sampclk());
        });

        r.ctl0().write(|w| {
            w.set_enc(false);
            // TODO: power down config
            w.set_pwrdn(vals::Pwrdn::Manual);
            w.set_sclkdiv(sclkdiv);
        });

        r.clkfreq().write(|w| {
            w.set_frange(frange);
        });

        r.ctl1().write(|w| {
            w.set_trigsrc(vals::Trigsrc::Software);
            // Configured, not converting; a read starts it.
            w.set_sc(false);
            w.set_conseq(vals::Conseq::Sequence);
            w.set_sampmode(vals::Sampmode::Auto);

            // One rate for the peripheral; `Conversion::average` picks which conversions use it.
            let (avgn, avgd) = match config.averaging {
                Some(averaging) => averaging.to_regs(),
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

        r.scomp(SampleTimeComparator::Scomp0.index()).write(|w| {
            w.set_val(config.sample_period_0.get());
        });

        r.scomp(SampleTimeComparator::Scomp1.index()).write(|w| {
            w.set_val(config.sample_period_1.get());
        });

        adcclk_hz
    }

    /// Program one `MEMCTL` entry.
    fn write_memctl(i: usize, ch: u8, conversion: Conversion) {
        let r = T::info().regs;

        // Read back rather than kept on the driver: the rate lives in `CTL1` from `Config`, and a
        // copy here could disagree with what is actually programmed.
        assert!(
            !conversion.average || r.ctl1().read().avgn() != vals::Avgn::Disable,
            "Conversion::average needs Config::averaging set"
        );

        r.memctl(i).write(|w| {
            w.set_chansel(ch);
            w.set_vrsel(conversion.vrsel.to_vals());
            w.set_stime(convert_stime(conversion.stime));
            w.set_avgen(conversion.average);
            w.set_bcsen(false);
            w.set_trig(vals::Trig::AutoNext);
            w.set_wincomp(false);
        });
    }

    /// Set the conversion window to `MEMCTL[0..=last]`.
    fn set_window(last: usize) {
        T::info().regs.ctl2().modify(|w| {
            w.set_startadd(0);
            w.set_endadd(last as u8);
        });
    }

    /// A sequence of one, without building an iterator for it.
    ///
    /// Single-channel reads are the common case. Going through `setup_sequence` left a one-element
    /// loop whose iterator stopped being inlined once a second instance gave it a second call site.
    fn setup_one(ch: u8, conversion: Conversion) {
        Self::write_memctl(0, ch, conversion);
        Self::set_window(0);
    }

    fn setup_sequence(sequence: impl ExactSizeIterator<Item = (u8, Conversion)>) {
        let len = sequence.len();

        for (i, (ch, conversion)) in sequence.enumerate() {
            Self::write_memctl(i, ch, conversion);
        }

        Self::set_window(len - 1);
    }

    /// Return `impl Future` to reduce async state machine size.
    ///
    /// Parks on the conversion interrupt rather than polling for it. The caller arms the interrupt for
    /// the last channel of the sequence before starting it, and the handler clears the flag and wakes
    /// this. Waiting before a sequence is started is the same wait: it only ever blocks on a
    /// conversion a cancelled read left running, whose interrupt is still armed.
    #[inline]
    fn wait_for_conversion() -> impl Future<Output = ()> {
        let r = T::info().regs;

        poll_fn(move |cx| {
            // Registered before the test, so a conversion that finishes in between still wakes this.
            T::state().waker.register(cx.waker());

            if r.ctl0().read().enc() {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
    }
}

/// Peripheral state.
pub(crate) struct State {
    /// Woken by [`InterruptHandler`], which is the only waker side: the handler is bound per instance,
    /// and the driver owns the instance for as long as it can wait on it.
    waker: IrqWaker,
}

impl State {
    pub const fn new() -> Self {
        Self { waker: IrqWaker::new() }
    }
}

/// Peripheral information.
pub(crate) struct Info {
    pub(crate) regs: Regs,
    pub(crate) interrupt: Interrupt,
}

/// Peripheral instance trait.
pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

pub(crate) trait SealedAdcChannel<T> {
    fn setup(&self) {}

    fn channel(&self) -> u8;
}

trait SealedBorrowedChannel<'a, T> {
    fn reborrow_adc(self) -> BorrowedAdcChannel<'a, T>;
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
const ADC_CLK_MIN_HZ: u32 = crate::_generated::ADC_CLK_MIN_HZ;
const ADC_CLK_MAX_HZ: u32 = crate::_generated::ADC_CLK_MAX_HZ;

/// Span of ADCCLK the `CLKFREQ.FRANGE` bands cover, from band 0's floor to band 7's ceiling.
///
/// A property of the register field, the same on every device. What the device actually supports is
/// [`ADC_CLK_MIN_HZ`]..[`ADC_CLK_MAX_HZ`], which is always inside this.
const FRANGE_MIN_HZ: u32 = 1_000_000;
const FRANGE_MAX_HZ: u32 = 48_000_000;

/// Rate this driver aims to run SAMPCLK at.
///
/// `SCOMPx` counts the sample window in SAMPCLK cycles, so holding SAMPCLK steady is what keeps
/// [`Config::sample_period_0`] a fixed duration across clock trees. Its 125 ns period leaves twice
/// the 62.5 ns minimum sampling time the datasheets specify.
const TARGET_SAMPCLK_HZ: u32 = 8_000_000;

/// Largest `SCLKDIV` this picks. Beyond it the divider ladder stops being powers of two.
const MAX_SCLKDIV_INDEX: u8 = vals::Sclkdiv::DivBy8.to_bits();

/// Smallest `CTL0.SCLKDIV` that brings `adcclk_hz` down to [`TARGET_SAMPCLK_HZ`] or below.
///
/// A shift rather than a chain of comparisons: the first four `SCLKDIV` encodings are the powers of
/// two in order, so the encoding is the shift that reaches the target.
const fn sample_clock_div(adcclk_hz: u32) -> vals::Sclkdiv {
    let mut i = 0;

    while i < MAX_SCLKDIV_INDEX && adcclk_hz > TARGET_SAMPCLK_HZ << i {
        i += 1;
    }

    vals::Sclkdiv::from_bits(i)
}

/// The `CLKFREQ.FRANGE` band `adcclk_hz` falls in.
///
/// Describes ADCCLK itself, *before* `SCLKDIV`. A band that does not match the real input gives
/// "unintended results" (SLAU846 table 18-2). Bands are open at the bottom and closed at the top, so
/// a rate on a boundary belongs to the lower one.
const fn clock_range(adcclk_hz: u32) -> vals::Frange {
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

macro_rules! impl_adc_instance {
    ($instance: ident) => {
        impl crate::adc::SealedInstance for crate::peripherals::$instance {
            fn info() -> &'static crate::adc::Info {
                use crate::adc::Info;
                use crate::interrupt::typelevel::Interrupt;

                static INFO: Info = Info {
                    regs: crate::pac::$instance,
                    interrupt: crate::interrupt::typelevel::$instance::IRQ,
                };
                &INFO
            }

            fn state() -> &'static crate::adc::State {
                use crate::adc::State;

                static STATE: State = State::new();
                &STATE
            }
        }

        impl crate::adc::Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

macro_rules! impl_adc_temp_sensor {
    ($inst: ident, $ch: expr) => {
        impl crate::adc::AdcChannel<peripherals::$inst> for crate::adc::TempSensor {}
        impl crate::adc::SealedAdcChannel<peripherals::$inst> for crate::adc::TempSensor {
            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

macro_rules! impl_adc_pin {
    ($inst: ident, $pin: ident, $ch: expr) => {
        impl crate::adc::AdcChannel<peripherals::$inst> for crate::Peri<'_, crate::peripherals::$pin> {}
        impl crate::adc::SealedAdcChannel<peripherals::$inst> for crate::Peri<'_, crate::peripherals::$pin> {
            fn setup(&self) {
                <crate::peripherals::$pin as crate::gpio::SealedPin>::set_as_analog(self);
            }

            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

/// `new_blocking` must be callable without naming the mode.
///
/// It used to sit in the mode-generic impl while returning a `Blocking` one, so nothing constrained the
/// parameter and every call needed a turbofish. Placed here rather than in a test because the failure it
/// catches is a change to an impl block's bounds.
#[allow(dead_code)]
fn _assert_new_blocking_infers<'d, T: Instance>(peri: Peri<'d, T>) -> Adc<'d, T, Blocking> {
    Adc::new_blocking(peri, Config::default())
}

#[cfg(test)]
mod averaging_tests {
    use super::*;

    #[test]
    fn shift_matches_count() {
        // SLAU846 table 18-1 pairs each count with the shift that divides by it, so an averaged
        // result stays on the same scale as an unaveraged one.
        for (averaging, count) in [
            (Averaging::X2, 2u32),
            (Averaging::X4, 4),
            (Averaging::X8, 8),
            (Averaging::X16, 16),
            (Averaging::X32, 32),
            (Averaging::X64, 64),
            (Averaging::X128, 128),
        ] {
            let (avgn, avgd) = averaging.to_regs();
            assert_eq!(1u32 << avgd, count, "{averaging:?} divides by the wrong amount");
            assert_eq!(avgn.to_bits(), avgd, "{averaging:?} count and shift disagree");
        }
    }
}
