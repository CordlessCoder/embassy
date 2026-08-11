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
    VddaVssa = 0,

    /// External reference from pin
    ExtrefVrefm = 1,

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
    IntrefVssa = 2,

    /// VDDA and VREFM connected to VREF+ and VREF- of ADC
    #[cfg(adc_neg_vref)]
    VddaVrefm = 3,

    /// INTREF and VREFM connected to VREF+ and VREF- of ADC
    ///
    /// Carries the same startup requirement as [`IntrefVssa`](Self::IntrefVssa).
    #[cfg(adc_neg_vref)]
    IntrefVrefm = 4,
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

impl Default for Conversion {
    #[inline]
    fn default() -> Self {
        Self {
            vrsel: Vrsel::VddaVssa,
            stime: SampleTimeComparator::Scomp0,
            average: false,
        }
    }
}

/// ADC configuration.
#[derive(Copy, Clone)]
#[non_exhaustive]
pub struct Config {
    /// Resolution of the ADC conversion. The number of bits used to represent an ADC measurement.
    pub resolution: Resolution,

    /// Sample clock source.
    pub sample_clk: SampleClock,

    /// Length of [`SampleTimeComparator::Scomp0`]'s sample period, in ADC sample clock cycles.
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
}

impl Default for Config {
    fn default() -> Self {
        Self {
            resolution: Resolution::Bits12,
            sample_clk: SampleClock::Sysosc,
            // TODO: What should these be by default?
            sample_period_0: NonZeroU16::new(50).unwrap(),
            sample_period_1: NonZeroU16::new(50).unwrap(),
            averaging: None,
        }
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
    /// Kept so the sleep guard knows which clock a conversion depends on.
    #[allow(unused)]
    sample_clk: SampleClock,
    _mode: PhantomData<M>,
}

impl<'d, T: Instance> Adc<'d, T, Blocking> {
    /// Create a blocking ADC driver.
    pub fn new_blocking(peri: Peri<'d, T>, config: Config) -> Self {
        Self::setup(config);
        Adc {
            adc: peri,
            sample_clk: config.sample_clk,
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
        Self::setup(config);
        unsafe { T::info().interrupt.enable() };
        Self {
            adc: peri,
            sample_clk: config.sample_clk,
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
            .floor_for_operation(adc_clock_hz(self.sample_clk))
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

// Impl details

const ADC_VRSEL: u8 = crate::_generated::ADC_VRSEL;
const ADC_MEMCTL: u8 = crate::_generated::ADC_MEMCTL;

impl<'d, T: Instance, M: Mode> Adc<'d, T, M> {
    fn setup(config: Config) {
        assert!(config.sample_period_0 <= Config::MAX_SAMPLE_PERIOD);
        assert!(config.sample_period_1 <= Config::MAX_SAMPLE_PERIOD);

        let r = T::info().regs;

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
            w.set_sampclk(config.sample_clk.to_sampclk());
        });

        let (sclkdiv, frange) = adc_clock_regs(config.sample_clk);

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
    }

    /// Program one `MEMCTL` entry.
    fn write_memctl(i: usize, ch: u8, conversion: Conversion) {
        let r = T::info().regs;

        assert!(
            (conversion.vrsel as u8) < ADC_VRSEL,
            "Reference voltage selection out of bounds"
        );

        // Read back rather than kept on the driver: the rate lives in `CTL1` from `Config`, and a
        // copy here could disagree with what is actually programmed.
        assert!(
            !conversion.average || r.ctl1().read().avgn() != vals::Avgn::Disable,
            "Conversion::average needs Config::averaging set"
        );

        r.memctl(i).write(|w| {
            w.set_chansel(ch);
            // TODO: Conversion function to not be repr dependent
            w.set_vrsel(vals::Vrsel::from_bits(conversion.vrsel as u8));
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
        hz >= ADC_CLK_MIN_HZ && hz <= ADC_CLK_MAX_HZ,
        "the selected ADC sample clock is stopped or outside this device's fADCCLK range"
    );

    hz
}

/// The `CTL0.SCLKDIV` and `CLKFREQ.FRANGE` pair `source` implies.
///
/// One function rather than three calls in `setup`: a second instance then shares one body, instead
/// of outlining the two clock helpers and duplicating the divider ladder at each call site.
#[inline]
fn adc_clock_regs(source: SampleClock) -> (vals::Sclkdiv, vals::Frange) {
    let adcclk = adc_clock_hz(source);
    (sample_clock_div(adcclk), clock_range(adcclk))
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

/// Smallest `CTL0.SCLKDIV` that brings `adcclk_hz` down to [`TARGET_SAMPCLK_HZ`] or below.
const fn sample_clock_div(adcclk_hz: u32) -> vals::Sclkdiv {
    if adcclk_hz <= TARGET_SAMPCLK_HZ {
        vals::Sclkdiv::DivBy1
    } else if adcclk_hz <= 2 * TARGET_SAMPCLK_HZ {
        vals::Sclkdiv::DivBy2
    } else if adcclk_hz <= 4 * TARGET_SAMPCLK_HZ {
        vals::Sclkdiv::DivBy4
    } else {
        vals::Sclkdiv::DivBy8
    }
}

/// The `CLKFREQ.FRANGE` band `adcclk_hz` falls in.
///
/// Describes ADCCLK itself, *before* `SCLKDIV`. A band that does not match the real input gives
/// "unintended results" (SLAU846 table 18-2). Bands are open at the bottom and closed at the top, so
/// a rate on a boundary belongs to the lower one.
const fn clock_range(adcclk_hz: u32) -> vals::Frange {
    if adcclk_hz <= 4_000_000 {
        vals::Frange::Range1to4
    } else if adcclk_hz <= 8_000_000 {
        vals::Frange::Range4to8
    } else if adcclk_hz <= 16_000_000 {
        vals::Frange::Range8to16
    } else if adcclk_hz <= 20_000_000 {
        vals::Frange::Range16to20
    } else if adcclk_hz <= 24_000_000 {
        vals::Frange::Range20to24
    } else if adcclk_hz <= 32_000_000 {
        vals::Frange::Range24to32
    } else if adcclk_hz <= 40_000_000 {
        vals::Frange::Range32to40
    } else {
        vals::Frange::Range40to48
    }
}

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
