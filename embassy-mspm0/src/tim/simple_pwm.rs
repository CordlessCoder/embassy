//! Pulse-width modulation.
//!
//! Edge-aligned only. Center-aligned needs a different period-to-compare relationship and is not
//! implemented; reach for [`Timer::regs`] to set it up by hand.

use core::marker::PhantomData;

use crate::Peri;
use crate::gpio::{AnyPin, PfType, Pull, SealedPin};
use crate::pac::tim::Tim;
use crate::pac::tim::vals::{Act, Ccpiv, Ccpo, Coc, Swfrcact};
use crate::tim::low_level::{self, Config as TimerConfig, Timer};
use crate::tim::{
    Ch0, Ch1, Ch2, Ch3, Channel, CountingDirection, General2ChannelInstance, General4ChannelInstance, Instance,
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
    /// Counting direction, which sets where the active edge sits in the period.
    ///
    /// [`CountingDirection::Down`] is what driverlib calls plain `DL_TIMER_PWM_MODE_EDGE_ALIGN`, so
    /// ported C expects that one.
    pub direction: CountingDirection,

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
            direction: CountingDirection::default(),
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

        Self {
            pin: pin.into(),
            _phantom: PhantomData,
        }
    }

    fn erase(self) -> Peri<'d, AnyPin> {
        self.pin
    }
}

/// Edge-aligned PWM driver.
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
    ) -> Self {
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
    ) -> Self {
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
    fn build(timer: Peri<'d, T>, pins: [Option<Peri<'d, AnyPin>>; 4], config: Config) -> Self {
        let timer = Timer::new(
            timer,
            TimerConfig {
                clock: config.clock,
                divider: config.divider,
                prescaler: config.prescaler,
                counting_mode: config.direction.counting_mode(),
                free_run_in_debug: config.free_run_in_debug,
                ..Default::default()
            },
        );

        timer.set_frequency(config.frequency);

        let mut this = Self { timer, pins };

        for channel in Channel::ALL {
            if this.pins[channel.index()].is_some() {
                this.setup_channel(channel, config.direction);
            }
        }

        this
    }

    /// Program one channel's compare block for edge-aligned output, following SLAU847F 28.2.5.2.1.
    fn setup_channel(&mut self, channel: Channel, direction: CountingDirection) {
        let r = self.timer.regs();
        let n = channel.index();

        r.counterregs(0).ccctl(n).modify(|w| w.set_coc(Coc::Compare));

        r.commonregs(0).ccpd().modify(|w| w.set_c0ccp(n, true));

        // The actions are fixed for the channel's lifetime: duty moves the compare value, and the two
        // extremes use the forced-output override. Starts at 0%.
        r.counterregs(0).ccact(n).write(|w| {
            match direction {
                CountingDirection::Up => {
                    w.set_zact(Act::CcpHigh);
                    w.set_cuact(Act::CcpLow);
                }
                CountingDirection::Down => {
                    w.set_lact(Act::CcpHigh);
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

    /// Ticks in one output period, the duty value that means 100%.
    pub fn max_duty(&self) -> u32 {
        self.timer.period_ticks()
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
    pub fn set_frequency(&mut self, hz: u32) {
        self.timer.set_frequency(hz);
    }

    /// The underlying counter.
    pub fn timer(&self) -> &Timer<'d, T> {
        &self.timer
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
    /// Ticks in one output period, the duty value that means 100%.
    pub fn max_duty(&self) -> u32 {
        low_level::period_ticks(self.regs)
    }

    /// Duty of this channel, in ticks.
    pub fn duty(&self) -> u32 {
        let n = self.channel.index();

        match self.regs.counterregs(0).ccact(n).read().swfrcact() {
            Swfrcact::CcpLow => 0,
            Swfrcact::CcpHigh => self.max_duty(),
            _ => {
                let compare = self.regs.counterregs(0).cc(n).read();

                match low_level::counting_direction(self.regs) {
                    CountingDirection::Up => compare,
                    CountingDirection::Down => self.max_duty() - 1 - compare,
                }
            }
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
            // Counting down the output is high from the load value to the compare, so the compare is
            // the far end of the pulse rather than its length.
            let compare = match low_level::counting_direction(self.regs) {
                CountingDirection::Up => ticks,
                CountingDirection::Down => period - 1 - ticks,
            };

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
