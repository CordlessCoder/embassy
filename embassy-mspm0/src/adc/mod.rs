//! Analog to Digital Converter (ADC)

#![macro_use]

pub mod low_level;

use core::future::poll_fn;
use core::marker::PhantomData;
use core::num::NonZeroU16;
use core::task::Poll;

use embassy_hal_internal::PeripheralType;
use low_level::{ADC_CLK_MAX_HZ, ADC_CLK_MIN_HZ, TARGET_SAMPCLK_HZ, clock_range, sample_clock_div_to};

use crate::interrupt::{Interrupt, InterruptExt};
use crate::mode::{Async, Blocking, Mode};
use crate::pac::adc::{Adc as Regs, vals};
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
        // Only the last result of a sequence is armed, so this asks about the group and clears
        // whichever of it actually raised the line.
        if low_level::take_active::<T>(low_level::Event::AnyResult) {
            T::state().waker.wake();
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
    /// [`ClockSetup::clocks`](crate::sysctl::clock::ClockSetup::clocks) on the tree the binary applies,
    /// which is a `const`.
    ///
    /// Usable in a `const`, which is the point: handing the result to
    /// [`Config::with_sample_clk`] keeps both ladders and the range check out of the binary.
    pub const fn solve(source: SampleClock, adcclk_hz: u32) -> Option<Self> {
        if adcclk_hz < ADC_CLK_MIN_HZ || adcclk_hz > ADC_CLK_MAX_HZ {
            return None;
        }

        Self::solve_at(source, adcclk_hz, TARGET_SAMPCLK_HZ)
    }

    /// Solve for a SAMPCLK of `target_hz` rather than the driver's default.
    ///
    /// Picks the smallest `SCLKDIV` that brings `adcclk_hz` to `target_hz` or below, so the SAMPCLK
    /// this produces is at most `target_hz` and never above it. [`None`] if `adcclk_hz` is outside
    /// this device's `fADCCLK`.
    ///
    /// # When to reach for this
    ///
    /// Two things run off SAMPCLK and they want opposite ends of it. `SCOMPx` counts the sample
    /// window in SAMPCLK cycles, so a slower SAMPCLK buys a longer reachable window — ten bits of
    /// `SCOMPx` is [`Config::MAX_SAMPLE_PERIOD`] cycles, which is 128 µs at 8 MHz and 32 µs at 32.
    /// The successive-approximation stage is clocked from it too, and that part is fixed latency
    /// rather than settling, so a faster SAMPCLK shortens every conversion.
    ///
    /// Which matters depends on the source impedances driving the inputs, which the driver cannot
    /// know. A high-impedance divider wants the long window; a low-impedance source would rather
    /// have the conversions back. [`solve`](Self::solve) is the answer when nothing forces the
    /// question.
    ///
    /// # The window still has a minimum, and it is per device
    ///
    /// The datasheets specify `tSample` as a minimum sample *window*, not a minimum clock period,
    /// and it is per device — [`Config::SAMPLE_MIN_NS`] carries this one's. Your own source
    /// impedance moves it further, and the datasheet gives the equation. This function does not
    /// check it: `target_hz` and [`Config::sample_period_0`] together are what set the window, and
    /// only the caller knows what is driving the input.
    pub const fn solve_at(source: SampleClock, adcclk_hz: u32, target_hz: u32) -> Option<Self> {
        if adcclk_hz < ADC_CLK_MIN_HZ || adcclk_hz > ADC_CLK_MAX_HZ {
            return None;
        }

        Some(Self {
            source,
            adcclk_hz,
            sclkdiv: sample_clock_div_to(adcclk_hz, target_hz),
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
    /// | source | minimum window |
    /// |---|---|
    /// | a pin, 50 ohm source | [`Config::SAMPLE_MIN_NS`] — this device's own figure |
    /// | through an OPA | [`Config::pga_sample_min_ns`], which takes the gain |
    /// | through the general-purpose amplifier | 2.5 us |
    /// | the supply monitor | 3 us |
    /// | the temperature sensor | do not read it from here — [`TempSensor::RECOMMENDED_SAMPLE_NS`] carries this device's figure |
    ///
    /// **None of these follows the family**, which is why they are looked up rather than written
    /// down: the bare-pin minimum is 62.5 ns on most of the G-series and 156 on most of the
    /// L-series, but the G5187 and L2117 are 188 and sit inside those families. Reading a figure
    /// from a sibling part is how this driver had two of them wrong. The amplifier row is the one most likely to catch you: it scales
    /// with gain, and at the top of the range it is an order of magnitude above the bare-pin
    /// figure, so a sequence that reads an OPA output at high gain with the pin's window is short
    /// by roughly ten times. The driver cannot check them for you either: the metapac does not carry these
    /// figures yet, which is request R20.
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
    /// The shortest sample window this device supports, in nanoseconds.
    ///
    /// `tSample` at the datasheet's reference conditions — 12-bit, `RS` = 50 ohm, `Cpext` = 10 pF.
    /// A higher source impedance needs more, and the datasheet gives the equation to rescale it.
    ///
    /// Whole nanoseconds rounded **up**, so the G-series' 62.5 reads as 63. Rounding a lower bound
    /// down would offer a window the datasheet does not support.
    ///
    /// [`None`] means the metadata is missing the figure, never that the device has no minimum.
    /// Every datasheet states one. Treat it as a reason to stop rather than as "no constraint".
    pub const SAMPLE_MIN_NS: Option<u32> = crate::_generated::ADC_SAMPLE_MIN_NS;

    /// The shortest window for reading an OPA output at `gain`, in nanoseconds.
    ///
    /// An order of magnitude above [`SAMPLE_MIN_NS`](Self::SAMPLE_MIN_NS) at the top of the range,
    /// so a sequence that reads an amplifier with the bare pin's window is short by roughly ten
    /// times.
    ///
    /// [`None`] for a gain the datasheet does not publish, and **that is not interpolatable**. The
    /// L-series publishes only x1 and x32, and the two series' curves cross — L is slower at x1 and
    /// faster at x32 — so there is no shared shape to interpolate along. `None` on a chip with no
    /// amplifier at all, which is why an empty table is a real answer rather than a gap.
    ///
    /// Every published figure is measured with `CFGBASE.GBW` at its high setting, which is what
    /// [`opa::Config`](crate::opa::Config) defaults to. Selecting the low setting puts the
    /// amplifier outside all of them.
    pub const fn pga_sample_min_ns(gain: u8) -> Option<u32> {
        let table = crate::_generated::ADC_PGA_SAMPLE_NS;
        let mut i = 0;

        while i < table.len() {
            if table[i].0 == gain {
                return Some(table[i].1);
            }
            i += 1;
        }

        None
    }

    /// Maximum number of sample clocks that may be performed when sampling.
    ///
    /// `SCOMPx.VAL` is ten bits on every supported device, checked against TI's
    /// `ADC12_SCOMP0_VAL_MASK` and the metapac's own accessor.
    pub const MAX_SAMPLE_PERIOD: NonZeroU16 = NonZeroU16::new((1 << 10) - 1).unwrap();

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
    /// The registers, the clock selection and the sleep floor. Everything here adds a way to wait.
    inner: low_level::Adc<'d, T>,
    _mode: PhantomData<M>,
}

impl<'d, T: Instance> Adc<'d, T, Blocking> {
    /// Create a blocking ADC driver.
    pub fn new_blocking(peri: Peri<'d, T>, config: Config) -> Self {
        Adc {
            inner: low_level::Adc::new(peri, config),
            _mode: PhantomData,
        }
    }
}

impl<'d, T: Instance, M: Mode> Adc<'d, T, M> {
    /// Read an ADC pin.
    pub fn blocking_read<'a>(&mut self, channel: impl BorrowedChannel<'a, T>, conversion: Conversion) -> u16 {
        // A sampling future dropped half way through leaves a conversion running.
        while low_level::is_converting::<T>() {}

        low_level::setup_one::<T>(channel.reborrow_adc().get_hw_channel(), conversion);
        low_level::start::<T>();

        // Wait for conversion
        while low_level::is_converting::<T>() {}
        low_level::result::<T>(0)
    }

    pub fn resolution(&self) -> Resolution {
        self.inner.resolution()
    }

    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.inner.set_resolution(resolution);
    }

    /// Set one comparator's sample period, in ADC sample clock cycles.
    ///
    /// Panics if `period` is above [`Config::MAX_SAMPLE_PERIOD`].
    pub fn set_sample_period(&mut self, comparator: SampleTimeComparator, period: NonZeroU16) {
        self.inner.set_sample_period(comparator, period);
    }

    /// One comparator's sample period, in ADC sample clock cycles.
    pub fn sample_period(&self, comparator: SampleTimeComparator) -> u16 {
        self.inner.sample_period(comparator)
    }
}

impl<'d, T: Instance> Adc<'d, T, Async> {
    pub fn new_async(
        peri: Peri<'d, T>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let inner = low_level::Adc::new(peri, config);
        unsafe { T::info().interrupt.enable() };

        Self {
            inner,
            _mode: PhantomData,
        }
    }

    /// Shallowest sleep level to block while a conversion is in flight.
    fn conversion_guard(&self) -> Option<WakeGuard> {
        // Asks about ADCCLK rather than MCLK. The two used to be interchangeable, since the sample
        // clock was always SYSOSC and MCLK always ran from it, but a configured tree can now put
        // HFCLK far above an LFCLK-sourced MCLK — where the MCLK answer would allow a sleep deep
        // enough to stop the clock the conversion is running on.
        self.inner.conversion_floor().map(WakeGuard::new)
    }

    /// Read an ADC pin asynchronously using the irq handler.
    pub async fn irq_read<'a>(&mut self, channel: impl BorrowedChannel<'a, T>, conversion: Conversion) -> u16 {
        let _guard = self.conversion_guard();
        let channel = channel.reborrow_adc();

        // Wait until ADC is not converting to start - an active conversion might've been cancelled.
        Self::wait_for_conversion().await;
        low_level::setup_one::<T>(channel.get_hw_channel(), conversion);

        // Armed alone, so nothing else in the mask is left over to wake this.
        low_level::arm_only::<T>(low_level::Event::Result(0));
        low_level::start::<T>();

        Self::wait_for_conversion().await;
        low_level::result::<T>(0)
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

        Self::wait_for_conversion().await;
        low_level::setup_sequence::<T>(sequence.map(|(ch, conv)| (ch.get_hw_channel(), conv)));

        // Only the last result wakes this; the earlier ones set their flags as they land.
        low_level::arm_only::<T>(low_level::Event::Result(sequence_len as u8 - 1));
        low_level::start::<T>();

        Self::wait_for_conversion().await;

        for (i, reading) in readings.iter_mut().enumerate() {
            *reading = low_level::result::<T>(i);
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

impl<T> BorrowedAdcChannel<'static, T> {
    /// Name a channel by its hardware number, without holding the pin that reaches it.
    ///
    /// The typed channels are the ordinary way in, and they fold to a constant, so reach for this
    /// only where the number is not known until the program runs — a command that reads whichever
    /// channel it is asked for. One of these replaces a match over every pin, and with it a copy of
    /// the conversion per arm.
    ///
    /// Nothing is configured on the way through. Putting a pad into analog mode, and starting whatever
    /// a channel is wired behind, is the typed channel's `setup` — which this skips. So a channel that
    /// is only a pad must already be in analog mode, and one behind an amplifier or a reference cannot
    /// be reached this way at all.
    ///
    /// # Safety
    ///
    /// This is where the pin's ownership is bypassed, and that is the whole of the contract: nothing
    /// stops the pad this channel reaches from being held, and driven, by another driver at the same
    /// time. The caller keeps two things apart that the type system otherwise would — that the pad is
    /// not being driven as an output while it is converted, and that two readers do not disagree about
    /// what it is for.
    ///
    /// A number no channel answers to is *not* unsound. `MEMCTL.CHANSEL` is five bits and the write is
    /// masked, so an out-of-range channel selects an input the device does not have and the conversion
    /// reads an unreliable value. It is a wrong reading, not undefined behaviour.
    pub unsafe fn steal(channel: u8) -> Self {
        Self {
            channel,
            _marker: PhantomData,
        }
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
    /// Return `impl Future` to reduce async state machine size.
    ///
    /// Parks on the conversion interrupt rather than polling for it. The caller arms the interrupt for
    /// the last channel of the sequence before starting it, and the handler clears the flag and wakes
    /// this. Waiting before a sequence is started is the same wait: it only ever blocks on a
    /// conversion a cancelled read left running, whose interrupt is still armed.
    #[inline]
    fn wait_for_conversion() -> impl Future<Output = ()> {
        poll_fn(move |cx| {
            // Registered before the test, so a conversion that finishes in between still wakes this.
            T::state().waker.register(cx.waker());

            if low_level::is_converting::<T>() {
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
            let (avgn, avgd) = low_level::averaging_regs(averaging);
            assert_eq!(1u32 << avgd, count, "{averaging:?} divides by the wrong amount");
            assert_eq!(avgn.to_bits(), avgd, "{averaging:?} count and shift disagree");
        }
    }
}
