//! Low-level timer access.

use crate::Peri;
use crate::pac::tim::vals::{Cm, Cvae, CxC, PwrenKey, Repeat, ResetKey};
use crate::pac::tim::{Tim, regs};
use crate::sysctl::{SleepLevel, WakeGuard};
use crate::tim::{Channel, ClockSel, CountingDirection, CountingMode, Instance, Word};

/// What the counter does when it is enabled.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CounterOnEnable {
    /// Restart from the beginning of the period.
    #[default]
    Reset,

    /// Carry on from the current counter value, so `stop` then `start` resumes.
    Preserve,
}

/// Timer configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Clock source driving the counter.
    pub clock: ClockSel,

    /// Divider applied to the clock source, 1 to 8.
    pub divider: u8,

    /// Further divider applied after [`Self::divider`], 1 to 256.
    ///
    /// Panics if set to anything but 1 on an instance without a prescaler.
    pub prescaler: u16,

    /// Counting direction and alignment.
    pub counting_mode: CountingMode,

    /// Whether enabling the counter restarts it or resumes it.
    pub counter_on_enable: CounterOnEnable,

    /// Keep counting while the debugger holds the core halted.
    pub free_run_in_debug: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock: ClockSel::default(),
            divider: 1,
            prescaler: 1,
            counting_mode: CountingMode::default(),
            counter_on_enable: CounterOnEnable::default(),
            free_run_in_debug: false,
        }
    }
}

/// A counter event that can raise an interrupt.
///
/// Not exhaustive: the fault and QEI sources are reachable through [`Timer::regs`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// Counter reached zero.
    Zero,

    /// Counter reloaded from the load register.
    Load,

    /// Channel captured the counter, or matched its compare value, while counting up.
    ///
    /// One flag serves both; which it means follows the channel's `COC` mode.
    CaptureOrCompareUp(Channel),

    /// Channel captured the counter, or matched its compare value, while counting down.
    CaptureOrCompareDown(Channel),

    /// Repeat counter reached zero.
    RepeatCount,

    /// Counting direction reversed, in center-aligned mode.
    DirectionChange,
}

impl Event {
    /// This event's bit in `IMASK`, `RIS`, `MIS` and `ICLR`, which all share a layout.
    pub(crate) const fn mask(self) -> regs::CpuInt {
        let mut mask = regs::CpuInt(0);

        match self {
            Event::Zero => mask.set_z(true),
            Event::Load => mask.set_l(true),
            Event::RepeatCount => mask.set_repc(true),
            Event::DirectionChange => mask.set_dc(true),
            Event::CaptureOrCompareUp(channel) => match channel {
                Channel::Ch0 => mask.set_ccu0(true),
                Channel::Ch1 => mask.set_ccu1(true),
                Channel::Ch2 => mask.set_ccu2(true),
                Channel::Ch3 => mask.set_ccu3(true),
            },
            Event::CaptureOrCompareDown(channel) => match channel {
                Channel::Ch0 => mask.set_ccd0(true),
                Channel::Ch1 => mask.set_ccd1(true),
                Channel::Ch2 => mask.set_ccd2(true),
                Channel::Ch3 => mask.set_ccd3(true),
            },
        }

        mask
    }
}

/// Low-level timer driver.
pub struct Timer<'d, T: Instance> {
    _timer: Peri<'d, T>,
    /// Held for the driver's lifetime, not per operation: a counter that stops has lost time.
    _wake_guard: Option<WakeGuard>,
}

impl<'d, T: Instance> Timer<'d, T> {
    /// Power up an instance and apply `config`, leaving the counter stopped at 0.
    pub fn new(timer: Peri<'d, T>, config: Config) -> Self {
        configure::<T>(&config);

        Self {
            _timer: timer,
            _wake_guard: wake_guard::<T>(config.clock),
        }
    }

    /// Registers of this instance, for what this driver does not wrap.
    #[inline]
    pub fn regs(&self) -> Tim {
        T::info().regs
    }

    /// Let the counter advance.
    #[inline]
    pub fn start(&self) {
        T::info().regs.counterregs(0).ctrctl().modify(|w| w.set_en(true));
    }

    /// Halt the counter, keeping its value.
    #[inline]
    pub fn stop(&self) {
        T::info().regs.counterregs(0).ctrctl().modify(|w| w.set_en(false));
    }

    /// Current counter value.
    #[inline]
    pub fn counter(&self) -> T::Word {
        T::Word::from_reg(T::info().regs.counterregs(0).ctr().read())
    }

    /// Set the counter.
    #[inline]
    pub fn set_counter(&self, count: T::Word) {
        T::info().regs.counterregs(0).ctr().write_value(count.into());
    }

    /// Value the counter reloads from, one less than the period in ticks.
    #[inline]
    pub fn load(&self) -> T::Word {
        T::Word::from_reg(T::info().regs.counterregs(0).load().read())
    }

    /// Set the reload value, one less than the wanted period in ticks.
    #[inline]
    pub fn set_load(&self, load: T::Word) {
        T::info().regs.counterregs(0).load().write_value(load.into());
    }

    /// Capture/compare value of `channel`.
    #[inline]
    pub fn compare(&self, channel: Channel) -> T::Word {
        T::Word::from_reg(T::info().regs.counterregs(0).cc(channel.index()).read())
    }

    /// Set the capture/compare value of `channel`.
    #[inline]
    pub fn set_compare(&self, channel: Channel, value: T::Word) {
        T::info()
            .regs
            .counterregs(0)
            .cc(channel.index())
            .write_value(value.into());
    }

    /// Largest value this counter reaches.
    #[inline]
    pub fn max_count(&self) -> T::Word {
        T::Word::MAX
    }

    /// How many capture/compare channels this instance brings out to pins.
    ///
    /// Higher [`Channel`] values still address a real register, but one that reaches no pin.
    #[inline]
    pub fn channels(&self) -> u8 {
        T::info().channels
    }

    /// Clock source the counter runs from.
    pub fn clock_source(&self) -> ClockSel {
        let clksel = T::info().regs.clksel().read();

        if clksel.lfclk_sel() {
            ClockSel::LfClk
        } else if clksel.mfclk_sel() {
            ClockSel::MfClk
        } else {
            ClockSel::BusClk
        }
    }

    /// Rate the counter advances at, in Hz.
    pub fn tick_frequency(&self) -> u32 {
        let r = T::info().regs;
        let divider = r.clkdiv().read().ratio() as u32 + 1;
        let prescaler = r.commonregs(0).cps().read().pcnt() as u32 + 1;

        self.clock_source().frequency(T::SLEEP.power_domain) / divider / prescaler
    }

    /// Ticks in one counting period.
    pub fn period_ticks(&self) -> u32 {
        period_ticks(T::info().regs)
    }

    /// Set the period so the counter wraps at `hz`.
    ///
    /// Panics outside [`Timer::tick_frequency`] down to that divided by the counter's full range.
    pub fn set_frequency(&self, hz: u32) {
        assert!(hz > 0, "timer frequency must be non-zero");

        let ticks = self.tick_frequency() / hz;
        let load = ticks.checked_sub(1).expect("timer frequency is above the tick rate");

        assert!(
            load <= T::Word::MAX.into(),
            "timer frequency is below what the counter can reach"
        );

        self.set_load(T::Word::from_reg(load));
    }

    /// Enable or disable the interrupt for `event`.
    pub fn enable_interrupt(&self, event: Event, enable: bool) {
        enable_interrupt(T::info().regs, event, enable);
    }

    /// Whether `event` has fired, whether or not its interrupt is enabled.
    pub fn is_pending(&self, event: Event) -> bool {
        is_pending(T::info().regs, event)
    }

    /// Clear `event`'s pending flag.
    pub fn clear_pending(&self, event: Event) {
        clear_pending(T::info().regs, event);
    }
}

/// Power up an instance and apply `config`, leaving the counter stopped.
///
/// Separate from [`Timer::new`] so the time driver can share the sequence without owning a `Peri` — its
/// `static` needs a `const` initialiser, so it cannot hold a `Timer`.
pub(crate) fn configure<T: Instance>(config: &Config) {
    assert!((1..=8).contains(&config.divider), "timer divider must be 1 to 8");
    assert!(
        (1..=256).contains(&config.prescaler),
        "timer prescaler must be 1 to 256"
    );
    assert!(
        config.prescaler == 1 || T::info().prescaler,
        "this timer instance has no prescaler"
    );

    let r = T::info().regs;

    r.gprcm(0).rstctl().write(|w| {
        w.set_resetassert(true);
        w.set_resetstkyclr(true);
        w.set_key(ResetKey::Key);
    });

    r.gprcm(0).pwren().write(|w| {
        w.set_enable(true);
        w.set_key(PwrenKey::Key);
    });

    // SLAU847D 23.2.1 "TIMCLK Configuration": source, then dividers, then enable the clock.
    r.clksel().write(|w| match config.clock {
        ClockSel::LfClk => w.set_lfclk_sel(true),
        ClockSel::MfClk => w.set_mfclk_sel(true),
        ClockSel::BusClk => w.set_busclk_sel(true),
    });

    r.clkdiv().write(|w| w.set_ratio(config.divider - 1));

    r.commonregs(0)
        .cps()
        .write(|w| w.set_pcnt((config.prescaler - 1) as u8));

    r.pdbgctl().write(|w| w.set_free(config.free_run_in_debug));

    r.commonregs(0).cclkctl().write(|w| w.set_clken(true));

    let count_mode = match config.counting_mode {
        CountingMode::EdgeAlignedUp => Cm::Up,
        CountingMode::EdgeAlignedDown => Cm::Down,
        CountingMode::CenterAligned => Cm::UpDown,
    };

    let after_enable = match (config.counter_on_enable, config.counting_mode) {
        (CounterOnEnable::Preserve, _) => Cvae::Nochange,
        (CounterOnEnable::Reset, CountingMode::EdgeAlignedDown) => Cvae::Ldval,
        (CounterOnEnable::Reset, _) => Cvae::Zeroval,
    };

    r.counterregs(0).ctrctl().write(|w| {
        w.set_en(false);
        w.set_repeat(Repeat::Repeat1);
        w.set_cm(count_mode);
        w.set_cvae(after_enable);

        // These reset to 0x07, a reserved value that stops some instances counting at all.
        w.set_czc(CxC::Cctl0);
        w.set_cac(CxC::Cctl0);
        w.set_clc(CxC::Cctl0);
    });

    r.counterregs(0).load().write_value(T::Word::MAX.into());
}

/// Shallowest sleep level to block so an instance clocked from `clock` keeps counting, if any.
///
/// `None` means the counter survives every mode the chip has, so nothing needs blocking. Kept `const` so
/// the cost of a clock choice is answerable at compile time; only taking the guard has to be at runtime.
pub(crate) const fn sleep_floor<T: Instance>(clock: ClockSel) -> Option<SleepLevel> {
    let clock_hz = clock.frequency(T::SLEEP.power_domain);

    // The domain-level answer cannot know which instances stay clocked in STANDBY1, and reports
    // STANDBY1 for any LFCLK peripheral in PD0.
    if matches!(T::SLEEP.clocked_in_standby1, Some(true)) && clock_hz <= crate::sysctl::LFCLK_HZ {
        return None;
    }

    T::SLEEP.floor_for_operation(clock_hz)
}

/// Take a guard holding [`sleep_floor`], if that clock choice costs anything at all.
pub(crate) fn wake_guard<T: Instance>(clock: ClockSel) -> Option<WakeGuard> {
    sleep_floor::<T>(clock).map(WakeGuard::new)
}

// The channel handles have the instance erased, so they reach these with a bare register block.

pub(crate) fn enable_interrupt(regs: Tim, event: Event, enable: bool) {
    let mask = event.mask().0;

    regs.cpu_int(0).imask().modify(|w| {
        w.0 = if enable { w.0 | mask } else { w.0 & !mask };
    });
}

pub(crate) fn is_pending(regs: Tim, event: Event) -> bool {
    regs.cpu_int(0).ris().read().0 & event.mask().0 != 0
}

pub(crate) fn clear_pending(regs: Tim, event: Event) {
    regs.cpu_int(0).iclr().write_value(event.mask());
}

/// Which way the counter is running, for the channel handles that have no instance to ask.
pub(crate) fn counting_direction(regs: Tim) -> CountingDirection {
    match regs.counterregs(0).ctrctl().read().cm() {
        Cm::Down => CountingDirection::Down,
        _ => CountingDirection::Up,
    }
}

/// Ticks in one counting period.
///
/// Saturates: a 32-bit counter loaded to its maximum has a period of 2^32, which does not fit.
pub(crate) fn period_ticks(regs: Tim) -> u32 {
    regs.counterregs(0).load().read().saturating_add(1)
}

impl<T: Instance> Drop for Timer<'_, T> {
    fn drop(&mut self) {
        let r = T::info().regs;

        r.counterregs(0).ctrctl().modify(|w| w.set_en(false));
        r.cpu_int(0).imask().write_value(regs::CpuInt(0));

        r.gprcm(0).pwren().write(|w| {
            w.set_enable(false);
            w.set_key(PwrenKey::Key);
        });
    }
}
