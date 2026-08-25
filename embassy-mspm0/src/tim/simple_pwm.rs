//! Pulse-width modulation.
//!
//! Edge-aligned in either direction, and center-aligned. Center-aligned moves both edges to keep the
//! pulse centred, so one period is `2 * load` ticks and the duty resolves half as finely.

use core::marker::PhantomData;

use crate::Peri;
use crate::gpio::{AnyPin, MaybeAnyPin, PfType, Pull, SealedPin};
use crate::pac::tim::Tim;
use crate::pac::tim::vals::{Act, Ccpiv, Ccpo, Coc, Swfrcact};
pub use crate::tim::low_level::ConfigError;
use crate::tim::low_level::{self, Config as TimerConfig, Timer};
use crate::tim::{
    Ch0, Ch1, Ch2, Ch3, Channel, CountingMode, General2ChannelInstance, General4ChannelInstance, Instance,
    TimerChannel, TimerPin,
};

/// Level the duty cycle drives the output to.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    /// Duty is the time the output spends high.
    #[default]
    ActiveHigh,

    /// Duty is the time the output spends low.
    ActiveLow,
}

/// PWM configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Counting direction and alignment, which set where the pulse sits in the period.
    ///
    /// [`CountingMode::EdgeAlignedDown`] is driverlib's plain `DL_TIMER_PWM_MODE_EDGE_ALIGN` and
    /// [`CountingMode::CenterAligned`] its `DL_TIMER_PWM_MODE_CENTER_ALIGN`, so ported C expects those.
    pub counting_mode: CountingMode,

    /// Clock source driving the counter.
    ///
    /// This decides what the output survives. The default stops in every deep-sleep mode, and a live
    /// driver holds the chip out of them — see [`SimplePwm`]'s own docs.
    pub clock: crate::tim::ClockSel,

    /// Divider applied to the clock source, 1 to 8.
    pub divider: u8,

    /// Further divider applied after [`Self::divider`], 1 to 256.
    ///
    /// Panics if set to anything but 1 on an instance without a prescaler.
    pub prescaler: u16,

    /// Output frequency in Hz.
    ///
    /// With the dividers this also fixes the duty resolution, which is [`SimplePwm::max_duty`].
    ///
    /// Ignored when [`Self::load`] is set, that having been solved for a frequency already.
    pub frequency: u32,

    /// A load value solved ahead of time, skipping the divisions [`Self::frequency`] needs.
    ///
    /// Build one with [`low_level::solve_load`] in a `const`, from the same dividers and counting mode
    /// as this config. Solving it there rather than here is worth ~600 bytes of flash, the core having
    /// no divide instruction.
    pub load: Option<u32>,

    /// Keep the waveform running while the debugger holds the core halted.
    pub free_run_in_debug: bool,
}

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            counting_mode: CountingMode::DEFAULT,
            clock: crate::tim::ClockSel::DEFAULT,
            divider: 1,
            prescaler: 1,
            frequency: 1_000,
            load: None,
            free_run_in_debug: false,
        }
    }

    /// Set [`counting_mode`](Self::counting_mode).
    #[must_use]
    pub const fn with_counting_mode(mut self, counting_mode: CountingMode) -> Self {
        self.counting_mode = counting_mode;
        self
    }

    /// Set [`clock`](Self::clock).
    #[must_use]
    pub const fn with_clock(mut self, clock: crate::tim::ClockSel) -> Self {
        self.clock = clock;
        self
    }

    /// Set [`divider`](Self::divider).
    #[must_use]
    pub const fn with_divider(mut self, divider: u8) -> Self {
        self.divider = divider;
        self
    }

    /// Set [`prescaler`](Self::prescaler).
    #[must_use]
    pub const fn with_prescaler(mut self, prescaler: u16) -> Self {
        self.prescaler = prescaler;
        self
    }

    /// Set [`frequency`](Self::frequency).
    #[must_use]
    pub const fn with_frequency(mut self, frequency: u32) -> Self {
        self.frequency = frequency;
        self
    }

    /// Set [`load`](Self::load).
    #[must_use]
    pub const fn with_load(mut self, load: Option<u32>) -> Self {
        self.load = load;
        self
    }

    /// Set [`free_run_in_debug`](Self::free_run_in_debug).
    #[must_use]
    pub const fn with_free_run_in_debug(mut self, free_run_in_debug: bool) -> Self {
        self.free_run_in_debug = free_run_in_debug;
        self
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// A pin driven as channel `C`'s PWM output.
pub struct PwmPin<'d, T: Instance, C: TimerChannel> {
    pin: Peri<'d, AnyPin>,
    _phantom: PhantomData<(T, C)>,
}

impl<'d, T: Instance, C: TimerChannel> PwmPin<'d, T, C> {
    /// Claim `pin` as this channel's output.
    ///
    /// Inversion belongs to the timer, not the pin; see [`SimplePwmChannel::set_polarity`].
    pub fn new(pin: Peri<'d, impl TimerPin<T, C>>, pull: Pull) -> Self {
        pin.set_as_pf(pin.pf_num(), PfType::output(pull, false));

        Self::from_erased(pin.into())
    }

    /// Disconnect the pin and give it back, for use as a GPIO or by another peripheral.
    pub fn release(self) -> Peri<'d, AnyPin> {
        let pin = self.erase();
        pin.set_as_disconnected();

        pin
    }

    /// Wrap a pin already claimed as channel `C`'s output.
    ///
    /// Private because the channel is only a type parameter here: the caller is what makes it true.
    fn from_erased(pin: Peri<'d, AnyPin>) -> Self {
        Self {
            pin,
            _phantom: PhantomData,
        }
    }

    /// Take the pin out, leaving it configured.
    fn erase(self) -> Peri<'d, AnyPin> {
        let this = core::mem::ManuallyDrop::new(self);

        // SAFETY: `this` is never dropped and the pin is not touched again, so it is moved out once.
        unsafe { core::ptr::read(&this.pin) }
    }
}

impl<T: Instance, C: TimerChannel> Drop for PwmPin<'_, T, C> {
    fn drop(&mut self) {
        self.pin.set_as_disconnected();
    }
}

/// The pins of a [`SimplePwm`], as [`SimplePwm::release`] gives them back.
pub struct PwmPins<'d, T: Instance> {
    /// Channel 0's pin.
    pub ch0: Option<PwmPin<'d, T, Ch0>>,

    /// Channel 1's pin.
    pub ch1: Option<PwmPin<'d, T, Ch1>>,

    /// Channel 2's pin.
    pub ch2: Option<PwmPin<'d, T, Ch2>>,

    /// Channel 3's pin.
    pub ch3: Option<PwmPin<'d, T, Ch3>>,
}

/// PWM driver.
/// Aligned to two bytes so that moving one does not call `memcpy`.
///
/// With `low-power` on, `Timer` carries a one-byte `MaybeWakeGuard` and this becomes **9 bytes at
/// alignment 1**, which is over whatever threshold LLVM inlines a copy at on this target: it emits two
/// bytes inline and calls `memcpy` for the remaining seven, twice, on the way out of `new_2ch` and into
/// the caller. That is **616 bytes** on `cmp_pwm` — a quarter of the binary — for a nine-byte move.
///
/// Alignment two makes it ten bytes, copied as five halfwords inline, and **costs nothing when
/// `low-power` is off**: the struct is eight bytes there either way. Measured both ways.
///
/// **Do not "improve" this to `align(4)`.** That was measured too and brings the `memcpy` back — twelve
/// bytes is over the threshold again. More alignment is worse here, which is not what anyone guesses.
///
/// # A live `SimplePwm` blocks deep sleep unless it is clocked to survive one
///
/// The counter stops in any mode its clock stops in, so the driver holds a [`WakeGuard`] for as long
/// as it exists, keeping the chip shallower than that. A `SimplePwm` built once and never dropped
/// holds that guard for the life of the program.
///
/// [`ClockSel::BusClk`] is the default and stops in every deep-sleep mode, so **the default
/// configuration pins the device out of all of them**. Two LEDs on a PWM are enough to do it, and
/// nothing reports it — the output looks right and the only symptom is the current.
///
/// [`ClockSel::LfClk`] on an instance that keeps counting in STANDBY1 takes no guard at all, and the
/// waveform runs through the sleep. Which instances those are is per device, and
/// `low_level::sleep_floor` is what answers it — it returns `None` for exactly that case.
///
/// [`WakeGuard`]: crate::sysctl::WakeGuard
/// [`ClockSel::BusClk`]: crate::tim::ClockSel::BusClk
/// [`ClockSel::LfClk`]: crate::tim::ClockSel::LfClk
///
/// # What the instance parameter costs, and why erasing it is not the obvious win
///
/// `T` puts a copy of every method body in the binary for each timer it is instantiated with. Measured
/// on a G-series part with one, two and three PWM timers driving the same application: **+352 bytes for
/// the second instance and +428 for the third**, and the marginal cost rises rather than staying flat.
/// Nothing shows it in a symbol listing — the bodies inline into the caller — so a total is the only
/// place it appears.
///
/// **Erasing the parameter was tried and made things worse.** Passing the instance's register block and
/// metadata to a shared `low_level::configure` instead of monomorphising it cost **+224 bytes at one
/// timer, +180 at two and +156 at three**, by reference; passing them by value rather than behind an
/// `&'static Info` recovered about a third of that and still lost at every instance count. The reason is
/// that a monomorphised body folds the register addresses to immediates, and a shared one has to carry
/// them as runtime arguments — so duplication buys constant-folding, and the trade only turns over at an
/// instance count no MSPM0 reaches.
///
/// What did pay is narrower: `setup_channel` is a free function over the register block rather than a
/// method, which is worth 16 bytes at two instances and 36 at three because it is called up to four
/// times per instance as well as once per timer. **Measure per function; the type parameter is not
/// itself the cost.**
///
/// Three more went the same way, on a G-series part driving one, two and three timers: **−52 bytes at
/// one instance, −248 at two and −392 at three.** A four-channel timer with all four pins costs 8 bytes
/// more. What each was worth is in the order they were measured:
///
/// - `set_frequency`, `set_load_value` and `tick_frequency` erased to free functions over the register
///   block, taking the counter width and the power domain as arguments. The clock read reaches
///   `critical_section::with` through a closure, and a closure over `T` is a fresh body per instance.
/// - The channel loop in `build` unrolled, and each channel's pin read *before* `pins` moves into the
///   struct. Once it is a field, `is_some()` is a load rather than a constant, and the four calls all
///   survive whatever the caller passed. This is the half that also pays at one instance.
/// - `SimplePwmChannel::set_duty` erased the same way, so a caller reaches it with three registers
///   rather than a handle it has to build on the stack first.
///
/// Two that were worth **exactly zero**, both measured: erasing `teardown`, and `#[inline(always)]` on
/// `Config::default`.
#[repr(align(2))]
pub struct SimplePwm<'d, T: Instance> {
    timer: Timer<'d, T>,
    pins: [MaybeAnyPin<'d>; 4],
}

impl<'d, T: General2ChannelInstance> SimplePwm<'d, T> {
    /// Configure a two-channel timer for PWM output, leaving every channel at 0% duty, output
    /// disabled, and stopped.
    ///
    /// [`SimplePwmChannel::enable`] is what starts a channel driving its pin, as it is on every other
    /// embassy HAL. Setting a duty alone does not.
    ///
    /// Channels without a pin are left alone.
    pub fn new_2ch(
        timer: Peri<'d, T>,
        ch0: Option<PwmPin<'d, T, Ch0>>,
        ch1: Option<PwmPin<'d, T, Ch1>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::build(
            timer,
            [
                MaybeAnyPin::new(ch0.map(PwmPin::erase)),
                MaybeAnyPin::new(ch1.map(PwmPin::erase)),
                MaybeAnyPin::none(),
                MaybeAnyPin::none(),
            ],
            config,
        )
    }
}

impl<'d, T: General4ChannelInstance> SimplePwm<'d, T> {
    /// Configure a four-channel timer for PWM output, leaving every channel at 0% duty, output
    /// disabled, and stopped.
    ///
    /// [`SimplePwmChannel::enable`] is what starts a channel driving its pin, as it is on every other
    /// embassy HAL. Setting a duty alone does not.
    ///
    /// Channels without a pin are left alone.
    pub fn new_4ch(
        timer: Peri<'d, T>,
        ch0: Option<PwmPin<'d, T, Ch0>>,
        ch1: Option<PwmPin<'d, T, Ch1>>,
        ch2: Option<PwmPin<'d, T, Ch2>>,
        ch3: Option<PwmPin<'d, T, Ch3>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::build(
            timer,
            [
                MaybeAnyPin::new(ch0.map(PwmPin::erase)),
                MaybeAnyPin::new(ch1.map(PwmPin::erase)),
                MaybeAnyPin::new(ch2.map(PwmPin::erase)),
                MaybeAnyPin::new(ch3.map(PwmPin::erase)),
            ],
            config,
        )
    }
}

impl<'d, T: Instance> SimplePwm<'d, T> {
    fn build(timer: Peri<'d, T>, pins: [MaybeAnyPin<'d>; 4], config: Config) -> Result<Self, ConfigError> {
        let timer = Timer::new(
            timer,
            TimerConfig {
                clock: config.clock,
                divider: config.divider,
                prescaler: config.prescaler,
                counting_mode: config.counting_mode,
                free_run_in_debug: config.free_run_in_debug,
                ..Default::default()
            },
        );

        // Which channels have a pin, read while `pins` is still a value the caller built. Once it is a
        // field of `this` it lives in memory, and `is_some()` on it becomes a load the compiler cannot
        // fold — the four calls below then all survive whatever the caller passed.
        let [ch0, ch1, ch2, ch3] = [
            pins[0].is_some(),
            pins[1].is_some(),
            pins[2].is_some(),
            pins[3].is_some(),
        ];

        // Built before the frequency is applied so a rejected one still unwinds through `Drop`,
        // releasing the pins and powering the instance back down.
        let mut this = Self { timer, pins };

        // Matched here rather than handed to a helper: an `Option` that crosses a call boundary stops
        // the unused arm folding away, and folding it is the whole point of solving ahead of time.
        match config.load {
            Some(load) => this.timer.set_load_value(load)?,
            None => this.set_frequency(config.frequency)?,
        }

        // Unrolled rather than a loop over `Channel::ALL`: at `opt-level = "z"` nothing unrolls it, and
        // a run-time channel index reaches `setup_channel` as an argument instead of a constant.
        let regs = this.timer.regs();
        if ch0 {
            setup_channel(regs, Channel::Ch0, config.counting_mode);
        }
        if ch1 {
            setup_channel(regs, Channel::Ch1, config.counting_mode);
        }
        if ch2 {
            setup_channel(regs, Channel::Ch2, config.counting_mode);
        }
        if ch3 {
            setup_channel(regs, Channel::Ch3, config.counting_mode);
        }

        Ok(this)
    }

    /// Let the counter run, driving every configured output.
    pub fn start(&mut self) {
        self.timer.start();
    }

    /// Stop the counter, freezing every output at whatever level it holds.
    pub fn stop(&mut self) {
        self.timer.stop();
    }

    /// Duty value that means 100%.
    ///
    /// Equal to the period in ticks when edge-aligned, and half of it when center-aligned.
    pub fn max_duty(&self) -> u32 {
        max_duty(self.timer.regs())
    }

    /// Borrow one channel to set its duty or enable its output.
    #[inline]
    pub fn channel(&mut self, channel: Channel) -> SimplePwmChannel<'_> {
        SimplePwmChannel {
            regs: self.timer.regs(),
            channel,
            _phantom: PhantomData,
        }
    }

    /// Set the output frequency in Hz.
    ///
    /// Duties are in ticks, so they keep their tick count; reapply them to keep the same ratio.
    pub fn set_frequency(&mut self, hz: u32) -> Result<(), ConfigError> {
        self.timer.set_frequency(hz)
    }

    /// The underlying counter, for reading.
    ///
    /// Mutating it needs [`timer_mut`](Self::timer_mut): this driver has programmed the instance for
    /// what it does, and a shared borrow is not the place to reprogram it from.
    pub fn timer(&self) -> &Timer<'d, T> {
        &self.timer
    }

    /// The underlying counter, for changing something this driver does not wrap.
    ///
    /// Whatever is changed here outlives the call. Reprogramming the counter, the compare values or
    /// the interrupt sources under a running driver is the caller's to get right.
    pub fn timer_mut(&mut self) -> &mut Timer<'d, T> {
        &mut self.timer
    }

    /// Stop the outputs and give the timer and pins back, ready to build another driver as they are.
    pub fn release(self) -> (Peri<'d, T>, PwmPins<'d, T>) {
        let mut this = core::mem::ManuallyDrop::new(self);

        let [ch0, ch1, ch2, ch3] = core::mem::replace(&mut this.pins, [const { MaybeAnyPin::none() }; 4]);

        // SAFETY: `this` is never dropped and the timer is not touched again, so it is moved out once.
        let timer = unsafe { core::ptr::read(&this.timer) };

        let pins = PwmPins {
            ch0: ch0.into_peri().map(PwmPin::from_erased),
            ch1: ch1.into_peri().map(PwmPin::from_erased),
            ch2: ch2.into_peri().map(PwmPin::from_erased),
            ch3: ch3.into_peri().map(PwmPin::from_erased),
        };

        (timer.release(), pins)
    }
}

impl<T: Instance> Drop for SimplePwm<'_, T> {
    fn drop(&mut self) {
        crate::tim::disconnect_pins(&self.pins);
    }
}

/// One channel of a [`SimplePwm`].
///
/// Duty is in ticks, clamped to the period, so it always fits the counter whatever its width.
pub struct SimplePwmChannel<'d> {
    regs: Tim,
    channel: Channel,
    _phantom: PhantomData<&'d mut ()>,
}

impl<'d> SimplePwmChannel<'d> {
    /// Duty value that means 100%.
    ///
    /// Equal to the period in ticks when edge-aligned, and half of it when center-aligned.
    pub fn max_duty(&self) -> u32 {
        max_duty(self.regs)
    }

    /// Duty of this channel, in ticks.
    pub fn duty(&self) -> u32 {
        duty(self.regs, self.channel)
    }

    /// Set the duty in ticks, saturating at [`Self::max_duty`].
    ///
    /// Takes effect immediately, so a change mid-period shortens or lengthens that one period.
    ///
    /// **The two extremes are the exception.** 0% and 100% are a forced-output action rather than a
    /// compare value, and SLAU846E 34.2.5.3 defers a forced action to the end of the period in
    /// flight. So a write of either asserts at the next boundary. For a PWM that is arguably the
    /// wanted behaviour; it matters to a caller writing 0% and expecting the pin low on the next
    /// instruction. The latency has not been measured — `TESTING.md` C34 read the levels in steady
    /// state, which cannot tell a deferred write from an immediate one.
    #[inline]
    pub fn set_duty(&mut self, ticks: u32) {
        set_duty(self.regs, self.channel, ticks);
    }

    /// Hold the output at its inactive level regardless of the duty cycle.
    ///
    /// This forces the signal low *before* inversion, so under [`Polarity::ActiveLow`] the pin goes
    /// high rather than low.
    pub fn disable(&mut self) {
        set_output_enabled(self.regs, self.channel, false);
    }

    /// Let the signal generator drive the output again.
    pub fn enable(&mut self) {
        set_output_enabled(self.regs, self.channel, true);
    }

    /// Whether the output is being driven rather than held low.
    pub fn is_enabled(&self) -> bool {
        is_output_enabled(self.regs, self.channel)
    }

    /// Which level the duty drives the output to.
    pub fn polarity(&self) -> Polarity {
        polarity(self.regs, self.channel)
    }

    /// Set which level the duty drives the output to.
    ///
    /// Inverts the pin immediately, including while the counter is stopped.
    pub fn set_polarity(&mut self, polarity: Polarity) {
        set_polarity(self.regs, self.channel, polarity);
    }

    /// Set the duty as a fraction of the period, clamped to 100%.
    pub fn set_duty_fraction(&mut self, numerator: u32, denominator: u32) {
        assert!(denominator > 0, "duty denominator must be non-zero");

        let numerator = numerator.min(denominator);
        let max = self.max_duty();

        self.set_duty(mul_div(max, numerator, denominator));
    }

    /// Set the duty as a percentage, clamped to 100.
    pub fn set_duty_percent(&mut self, percent: u8) {
        self.set_duty_fraction(u32::from(percent), 100);
    }
}

/// `a * b / d`, for `b <= d` so the quotient always fits.
///
/// The product needs more than 32 bits for a long period, but dividing it as a `u64` would link
/// `__aeabi_uldivmod`. Splitting `a` around `d` keeps the common case to one multiply and one
/// 32-bit divide.
const fn mul_div(a: u32, b: u32, d: u32) -> u32 {
    let whole = a / d;
    let rem = a % d;

    // `b <= d`, so `whole * b <= whole * d <= a` and cannot overflow.
    let scaled = whole * b;

    // `rem < d` and `b <= d`, so this only fails to fit for denominators above 2^16.
    let fraction = match rem.checked_mul(b) {
        Some(product) => product / d,
        None => mul_div_bitwise(rem, b, d),
    };

    scaled + fraction
}

/// `a * b / d` for `a < d`, without forming the product.
///
/// Walks the bits of `b` keeping a quotient and a remainder below `d`, so no intermediate needs
/// more than 32 bits. Only reached for denominators above 2^16, which a duty fraction realistically
/// never uses.
const fn mul_div_bitwise(a: u32, b: u32, d: u32) -> u32 {
    // Doubling the remainder must stay in range. Halving both sides preserves the ratio, and only
    // happens for denominators beyond 2^31, where the lost bit is far below the timer's resolution.
    let (a, b, d) = if d > u32::MAX / 2 {
        (a / 2, b / 2, d / 2)
    } else {
        (a, b, d)
    };

    if d == 0 {
        return 0;
    }

    let mut quotient = 0;
    let mut remainder = 0;

    let mut bit = 32;
    while bit > 0 {
        bit -= 1;

        // Double the running value. `remainder < d` holds here, so this stays in range.
        quotient *= 2;
        remainder *= 2;
        if remainder >= d {
            remainder -= d;
            quotient += 1;
        }

        if (b >> bit) & 1 == 1 {
            // `a < d` and `remainder < d`, so this stays in range given the halving above.
            remainder += a;
            if remainder >= d {
                remainder -= d;
                quotient += 1;
            }
        }
    }

    quotient
}

#[cfg(test)]
mod tests {
    use super::{mul_div, mul_div_bitwise};

    fn reference(a: u32, b: u32, d: u32) -> u32 {
        ((a as u64) * (b as u64) / (d as u64)) as u32
    }

    #[test]
    fn duty_fractions() {
        // Percentages against a period too long for `max * percent` to fit 32 bits.
        for max in [1u32, 999, 65_535, 65_536, 4_000_000_000, u32::MAX] {
            for percent in [0u32, 1, 33, 50, 99, 100] {
                core::assert_eq!(
                    mul_div(max, percent, 100),
                    reference(max, percent, 100),
                    "{max} {percent}%"
                );
            }
        }
    }

    #[test]
    fn large_denominators() {
        // Denominators above 2^16 take the bitwise path.
        for &(a, b, d) in &[
            (u32::MAX, 1u32, 100_000u32),
            (u32::MAX, 99_999, 100_000),
            (1_000_000, 500_000, 1_000_000),
            (u32::MAX, u32::MAX, u32::MAX),
            (12345, 6789, 70_000),
        ] {
            core::assert_eq!(mul_div(a, b, d), reference(a, b, d), "{a} * {b} / {d}");
        }
    }

    #[test]
    fn bitwise_matches_reference() {
        // `mul_div_bitwise` requires `a < d`; check it directly across a spread of inputs.
        for &d in &[3u32, 100, 65_537, 1_000_000, u32::MAX / 2] {
            for &b in &[0u32, 1, d / 3, d / 2, d] {
                for &a in &[0u32, 1, d / 7, d - 1] {
                    core::assert_eq!(mul_div_bitwise(a, b, d), reference(a, b, d), "{a} * {b} / {d}");
                }
            }
        }
    }

    #[test]
    fn full_scale_is_exact() {
        // 100% must land exactly on the period, never one tick short.
        for max in [1u32, 65_535, 4_000_000_000, u32::MAX] {
            core::assert_eq!(mul_div(max, 100, 100), max);
            core::assert_eq!(mul_div(max, 1, 1), max);
        }
    }
}

/// Program one channel's compare block for PWM output, following SLAU847F 28.2.5.2.1.
///
/// Takes the register block rather than `&mut SimplePwm<T>` so that one copy serves every timer
/// instance. See the note on [`SimplePwm`] about what a type parameter costs here.
pub(crate) fn setup_channel(r: Tim, channel: Channel, counting_mode: CountingMode) {
    let n = channel.index();

    r.counterregs(0).ccctl(n).modify(|w| w.set_coc(Coc::Compare));

    r.commonregs(0).ccpd().modify(|w| w.set_c0ccp(n, true));

    // The actions are fixed for the channel's lifetime: duty moves the compare value, and the two
    // extremes use the forced-output override. Starts at 0%.
    r.counterregs(0).ccact(n).write(|w| {
        match counting_mode {
            CountingMode::EdgeAlignedUp => {
                w.set_zact(Act::CcpHigh);
                w.set_cuact(Act::CcpLow);
            }
            CountingMode::EdgeAlignedDown => {
                w.set_lact(Act::CcpHigh);
                w.set_cdact(Act::CcpLow);
            }
            // Both edges come from the compare, one per direction, which is what centres the
            // pulse on the load endpoint rather than pinning it to the start of the period.
            CountingMode::CenterAligned => {
                w.set_cuact(Act::CcpHigh);
                w.set_cdact(Act::CcpLow);
            }
        }

        w.set_swfrcact(Swfrcact::CcpLow);
    });

    r.counterregs(0).octl(n).write(|w| {
        w.set_ccpo(Ccpo::Funcval);
        w.set_ccpiv(Ccpiv::Low);
        w.set_ccpoinv(false);
    });

    // Held rather than driving, which is `ODIS`'s stated purpose — the TRM has it there so software
    // can "hold the CCP output low during configuration or shutdown". `SimplePwmChannel::enable` and
    // `low_level::Timer::set_output_enabled` are what release it.
    //
    // Two mechanisms hold this pin low and they mean different things. `ODIS` is whether the channel
    // drives the pin at all; `SWFRCACT` above is a duty of exactly 0%, which no compare value can
    // express. Keeping them separate is what gives a dead pin a register that reads wrong, and
    // `is_output_enabled` a useful answer.
    r.commonregs(0).odis().modify(|w| w.set_c0ccp(n, true));
}

/// Drive `channel`'s output, or hold it at its inactive level.
///
/// The hold is applied before inversion, so under [`Polarity::ActiveLow`] the pin goes high.
pub(crate) fn set_output_enabled(regs: Tim, channel: Channel, enabled: bool) {
    regs.commonregs(0)
        .odis()
        .modify(|w| w.set_c0ccp(channel.index(), !enabled));
}

/// Whether `channel`'s output is being driven rather than held.
pub(crate) fn is_output_enabled(regs: Tim, channel: Channel) -> bool {
    !regs.commonregs(0).odis().read().c0ccp(channel.index())
}

/// Which level `channel`'s active phase drives the output to.
pub(crate) fn polarity(regs: Tim, channel: Channel) -> Polarity {
    if regs.counterregs(0).octl(channel.index()).read().ccpoinv() {
        Polarity::ActiveLow
    } else {
        Polarity::ActiveHigh
    }
}

/// Set which level `channel`'s active phase drives the output to.
pub(crate) fn set_polarity(regs: Tim, channel: Channel, polarity: Polarity) {
    regs.counterregs(0)
        .octl(channel.index())
        .modify(|w| w.set_ccpoinv(polarity == Polarity::ActiveLow));
}

/// Duty of `channel` in ticks.
///
/// Reads the forced-output override first, since that is where both extremes live rather than in the
/// compare value.
pub(crate) fn duty(regs: Tim, channel: Channel) -> u32 {
    let n = channel.index();

    match regs.counterregs(0).ccact(n).read().swfrcact() {
        Swfrcact::CcpLow => 0,
        Swfrcact::CcpHigh => max_duty(regs),
        _ => duty_from_compare(regs, regs.counterregs(0).cc(n).read()),
    }
}

/// Set the duty in ticks, saturating at the period.
///
/// Takes the register block and the channel rather than `&mut SimplePwmChannel`, so a caller reaches it
/// with three registers instead of a handle it has to put on the stack first.
pub(crate) fn set_duty(regs: Tim, channel: Channel, ticks: u32) {
    let period = max_duty(regs);
    let ticks = ticks.min(period);

    // Both extremes use the forced-output override, in every counting mode. Measured on silicon, and
    // the compare value fails differently in each:
    //
    // - Counting up, 0% is a compare of zero, which puts the zero event and the compare match on the
    //   same tick. That resolved cleanly six times in seven and left a narrow spike the seventh, so
    //   it is a race rather than a wrong answer — the worst kind, since it passes a casual test.
    // - Counting down and centred, the same collision lands at `LOAD` and leaves a spike every time.
    // - Counting down, 100% has no compare value at all: duty is `LOAD - CC`, so a full period would
    //   need a negative one, and the nearest reachable is one tick short.
    //
    // A compare above `LOAD` does give a clean 100% counting up, but `CC` is the counter's width, so
    // at the maximum period there is no value above it. One path that always works beats four that
    // each work sometimes.
    let force = match ticks {
        0 => Swfrcact::CcpLow,
        t if t >= period => Swfrcact::CcpHigh,
        _ => Swfrcact::Disabled,
    };

    // Compare first, so the value is in place before the override is lifted.
    if ticks > 0 && ticks < period {
        let compare = compare_for_duty(regs, ticks);

        regs.counterregs(0).cc(channel.index()).write_value(compare);
    }

    regs.counterregs(0)
        .ccact(channel.index())
        .modify(|w| w.set_swfrcact(force));
}

/// Duty value that means 100%, for the channel handles that have no instance to ask.
pub(crate) fn max_duty(regs: Tim) -> u32 {
    let load = regs.counterregs(0).load().read();

    match low_level::counting_mode(regs) {
        // One step moves both edges of a centred pulse, so the period resolves half as finely as it
        // is long.
        CountingMode::CenterAligned => load,
        _ => load.saturating_add(1),
    }
}

/// Compare value that produces a duty of `ticks`.
fn compare_for_duty(regs: Tim, ticks: u32) -> u32 {
    let load = regs.counterregs(0).load().read();

    match low_level::counting_mode(regs) {
        // Counting up, the zero event starts the pulse and the compare ends it, so the compare is
        // the pulse length directly.
        CountingMode::EdgeAlignedUp => ticks,
        // Otherwise the pulse runs from the compare to the load endpoint — once counting down, twice
        // when centred — so the compare is the far end of the pulse rather than its length.
        _ => load - ticks,
    }
}

/// Duty in ticks that `compare` produces, the inverse of [`compare_for_duty`].
fn duty_from_compare(regs: Tim, compare: u32) -> u32 {
    let load = regs.counterregs(0).load().read();

    match low_level::counting_mode(regs) {
        CountingMode::EdgeAlignedUp => compare,
        _ => load.saturating_sub(compare),
    }
}

impl embedded_hal::pwm::ErrorType for SimplePwmChannel<'_> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::pwm::SetDutyCycle for SimplePwmChannel<'_> {
    /// The period, or `u16::MAX` if it is larger, since `embedded-hal` fixes this at 16 bits.
    ///
    /// Scaled rather than rejected; narrowing would panic on exactly the periods 32-bit counters exist for.
    fn max_duty_cycle(&self) -> u16 {
        self.max_duty().min(u32::from(u16::MAX)) as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let scale = self.max_duty_cycle();
        self.set_duty_fraction(u32::from(duty), u32::from(scale));

        Ok(())
    }

    fn set_duty_cycle_fully_off(&mut self) -> Result<(), Self::Error> {
        self.set_duty(0);

        Ok(())
    }

    fn set_duty_cycle_fully_on(&mut self) -> Result<(), Self::Error> {
        self.set_duty(self.max_duty());

        Ok(())
    }

    fn set_duty_cycle_fraction(&mut self, numerator: u16, denominator: u16) -> Result<(), Self::Error> {
        self.set_duty_fraction(u32::from(numerator), u32::from(denominator));

        Ok(())
    }

    fn set_duty_cycle_percent(&mut self, percent: u8) -> Result<(), Self::Error> {
        self.set_duty_percent(percent);

        Ok(())
    }
}
