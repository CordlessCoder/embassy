//! Pulse-width modulation.
//!
//! Edge-aligned in either direction, and center-aligned. Center-aligned moves both edges to keep the
//! pulse centred, so one period is `2 * load` ticks and the duty resolves half as finely.

use core::marker::PhantomData;

use crate::Peri;
use crate::gpio::{AnyPin, PfType, Pull, SealedPin};
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
pub struct Config {
    /// Counting direction and alignment, which set where the pulse sits in the period.
    ///
    /// [`CountingMode::EdgeAlignedDown`] is driverlib's plain `DL_TIMER_PWM_MODE_EDGE_ALIGN` and
    /// [`CountingMode::CenterAligned`] its `DL_TIMER_PWM_MODE_CENTER_ALIGN`, so ported C expects those.
    pub counting_mode: CountingMode,

    /// Clock source driving the counter.
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
    pub frequency: u32,

    /// Keep the waveform running while the debugger holds the core halted.
    pub free_run_in_debug: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            counting_mode: CountingMode::default(),
            clock: crate::tim::ClockSel::default(),
            divider: 1,
            prescaler: 1,
            frequency: 1_000,
            free_run_in_debug: false,
        }
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
pub struct SimplePwm<'d, T: Instance> {
    timer: Timer<'d, T>,
    pins: [Option<Peri<'d, AnyPin>>; 4],
}

impl<'d, T: General2ChannelInstance> SimplePwm<'d, T> {
    /// Configure a two-channel timer for PWM output, leaving every channel at 0% duty and stopped.
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
            [ch0.map(PwmPin::erase), ch1.map(PwmPin::erase), None, None],
            config,
        )
    }
}

impl<'d, T: General4ChannelInstance> SimplePwm<'d, T> {
    /// Configure a four-channel timer for PWM output, leaving every channel at 0% duty and stopped.
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
                ch0.map(PwmPin::erase),
                ch1.map(PwmPin::erase),
                ch2.map(PwmPin::erase),
                ch3.map(PwmPin::erase),
            ],
            config,
        )
    }
}

impl<'d, T: Instance> SimplePwm<'d, T> {
    fn build(timer: Peri<'d, T>, pins: [Option<Peri<'d, AnyPin>>; 4], config: Config) -> Result<Self, ConfigError> {
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

        // Built before the frequency is applied so a rejected one still unwinds through `Drop`,
        // releasing the pins and powering the instance back down.
        let mut this = Self { timer, pins };

        this.set_frequency(config.frequency)?;

        for channel in Channel::ALL {
            if this.pins[channel.index()].is_some() {
                this.setup_channel(channel, config.counting_mode);
            }
        }

        Ok(this)
    }

    /// Program one channel's compare block for PWM output, following SLAU847F 28.2.5.2.1.
    fn setup_channel(&mut self, channel: Channel, counting_mode: CountingMode) {
        let r = self.timer.regs();
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

        // SLAU847F 28.2.5.2.1 step 8 says write 1 here; 28.3.32 and driverlib agree 1 is "forced low".
        r.commonregs(0).odis().modify(|w| w.set_c0ccp(n, false));
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

    /// The underlying counter.
    pub fn timer(&self) -> &Timer<'d, T> {
        &self.timer
    }

    /// Stop the outputs and give the timer and pins back, ready to build another driver as they are.
    pub fn release(self) -> (Peri<'d, T>, PwmPins<'d, T>) {
        let mut this = core::mem::ManuallyDrop::new(self);

        let [ch0, ch1, ch2, ch3] = core::mem::replace(&mut this.pins, [const { None }; 4]);

        // SAFETY: `this` is never dropped and the timer is not touched again, so it is moved out once.
        let timer = unsafe { core::ptr::read(&this.timer) };

        let pins = PwmPins {
            ch0: ch0.map(PwmPin::from_erased),
            ch1: ch1.map(PwmPin::from_erased),
            ch2: ch2.map(PwmPin::from_erased),
            ch3: ch3.map(PwmPin::from_erased),
        };

        (timer.release(), pins)
    }
}

impl<T: Instance> Drop for SimplePwm<'_, T> {
    fn drop(&mut self) {
        for pin in self.pins.iter().flatten() {
            pin.set_as_disconnected();
        }
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
        let n = self.channel.index();

        match self.regs.counterregs(0).ccact(n).read().swfrcact() {
            Swfrcact::CcpLow => 0,
            Swfrcact::CcpHigh => self.max_duty(),
            _ => duty_from_compare(self.regs, self.regs.counterregs(0).cc(n).read()),
        }
    }

    /// Set the duty in ticks, saturating at [`Self::max_duty`].
    ///
    /// Takes effect immediately, so a change mid-period shortens or lengthens that one period.
    pub fn set_duty(&mut self, ticks: u32) {
        let period = self.max_duty();
        let ticks = ticks.min(period);

        // Neither extreme is reachable through the compare value, so both use the forced-output
        // override. Merely disabling the event that starts the pulse does not work: with SWFRCACT
        // clear the signal generator still drives its own compare-based waveform.
        let force = match ticks {
            0 => Swfrcact::CcpLow,
            t if t >= period => Swfrcact::CcpHigh,
            _ => Swfrcact::Disabled,
        };

        // Compare first, so the value is in place before the override is lifted.
        if ticks > 0 && ticks < period {
            let compare = compare_for_duty(self.regs, ticks);

            self.regs.counterregs(0).cc(self.channel.index()).write_value(compare);
        }

        self.regs
            .counterregs(0)
            .ccact(self.channel.index())
            .modify(|w| w.set_swfrcact(force));
    }

    /// Hold the output at its inactive level regardless of the duty cycle.
    ///
    /// This forces the signal low *before* inversion, so under [`Polarity::ActiveLow`] the pin goes
    /// high rather than low.
    pub fn disable(&mut self) {
        self.regs
            .commonregs(0)
            .odis()
            .modify(|w| w.set_c0ccp(self.channel.index(), true));
    }

    /// Let the signal generator drive the output again.
    pub fn enable(&mut self) {
        self.regs
            .commonregs(0)
            .odis()
            .modify(|w| w.set_c0ccp(self.channel.index(), false));
    }

    /// Whether the output is being driven rather than held low.
    pub fn is_enabled(&self) -> bool {
        !self.regs.commonregs(0).odis().read().c0ccp(self.channel.index())
    }

    /// Which level the duty drives the output to.
    pub fn polarity(&self) -> Polarity {
        if self.regs.counterregs(0).octl(self.channel.index()).read().ccpoinv() {
            Polarity::ActiveLow
        } else {
            Polarity::ActiveHigh
        }
    }

    /// Set which level the duty drives the output to.
    ///
    /// Inverts the pin immediately, including while the counter is stopped.
    pub fn set_polarity(&mut self, polarity: Polarity) {
        self.regs
            .counterregs(0)
            .octl(self.channel.index())
            .modify(|w| w.set_ccpoinv(polarity == Polarity::ActiveLow));
    }

    /// Set the duty as a fraction of the period, clamped to 100%.
    pub fn set_duty_fraction(&mut self, numerator: u32, denominator: u32) {
        assert!(denominator > 0, "duty denominator must be non-zero");

        let numerator = numerator.min(denominator);
        let max = self.max_duty();

        // Widened so a 32-bit period times the numerator cannot overflow.
        let duty = u64::from(max) * u64::from(numerator) / u64::from(denominator);

        self.set_duty(duty as u32);
    }

    /// Set the duty as a percentage, clamped to 100.
    pub fn set_duty_percent(&mut self, percent: u8) {
        self.set_duty_fraction(u32::from(percent), 100);
    }
}

/// Duty value that means 100%, for the channel handles that have no instance to ask.
fn max_duty(regs: Tim) -> u32 {
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
