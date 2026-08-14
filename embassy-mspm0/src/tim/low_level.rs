//! Low-level timer access.

use core::mem::ManuallyDrop;

use crate::Peri;
use crate::pac::tim::vals::{Cm, Cvae, CxC, PwrenKey, Repeat, ResetKey};
use crate::pac::tim::{Tim, regs};
use crate::sysctl::MaybeWakeGuard;
#[cfg(any(feature = "low-power", feature = "_time-driver"))]
use crate::sysctl::SleepLevel;
use crate::tim::{Channel, ClockSel, CountingMode, Instance, Word};

/// Why a frequency cannot be programmed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// The frequency was zero.
    Zero,

    /// One period would be shorter than a tick. Lower the divider or the prescaler.
    TooHigh,

    /// One period would need more ticks than the counter holds. Raise the divider or the prescaler.
    TooLow,
}

/// What the counter does when it is enabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CounterOnEnable {
    /// Restart from the beginning of the period.
    Reset,

    /// Carry on from the current counter value, so `stop` then `start` resumes.
    Preserve,
}

impl CounterOnEnable {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::Reset;
}

impl Default for CounterOnEnable {
    fn default() -> Self {
        Self::DEFAULT
    }
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

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            clock: ClockSel::DEFAULT,
            divider: 1,
            prescaler: 1,
            counting_mode: CountingMode::DEFAULT,
            counter_on_enable: CounterOnEnable::DEFAULT,
            free_run_in_debug: false,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
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
    pub(crate) const fn mask(self) -> regs::Int {
        let mut mask = regs::Int(0);

        match self {
            Event::Zero => mask.set_z(true),
            Event::Load => mask.set_l(true),
            Event::RepeatCount => mask.set_repc(true),
            Event::DirectionChange => mask.set_dc(true),
            // The four per-channel bits are contiguous, so the channel indexes one rather than
            // selecting between four arms — which the compiler cannot see through the generated
            // setters. Free where the event is a constant, and a table instead of a branch chain
            // where the channel is not.
            Event::CaptureOrCompareUp(channel) => return regs::Int(1 << (CCU0_BIT + channel.index())),
            Event::CaptureOrCompareDown(channel) => return regs::Int(1 << (CCD0_BIT + channel.index())),
        }

        mask
    }
}

/// Bit of `CCU0` in `IMASK` and the registers sharing its layout. `CCU1`..`CCU3` follow it.
const CCU0_BIT: usize = 8;

/// Bit of `CCD0` in the same registers, with `CCD1`..`CCD3` following.
const CCD0_BIT: usize = 4;

// [`Event::mask`] indexes those two runs by channel instead of asking the generated setters, so the
// runs have to be contiguous and in channel order. Checked one bit at a time against the setters
// themselves: a metapac that moved or reordered them fails to build rather than quietly masking the
// wrong channel's interrupt.
const _: () = {
    let mut channel = 0;

    while channel < Channel::ALL.len() {
        let mut up = regs::Int(0);
        let mut down = regs::Int(0);

        up.set_ccu(channel, true);
        down.set_ccd(channel, true);

        core::assert!(up.0 == 1 << (CCU0_BIT + channel));
        core::assert!(down.0 == 1 << (CCD0_BIT + channel));

        channel += 1;
    }
};

/// Low-level timer driver.
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
pub struct Timer<'d, T: Instance> {
    _timer: Peri<'d, T>,

    /// Held for the driver's lifetime, and about the *configuration* rather than the count: a
    /// peripheral whose registers do not survive a mode has to be kept out of it, or it comes back
    /// set up as something else. `None` for every PD0 instance, whose registers are never lost.
    _config_guard: MaybeWakeGuard,

    /// Held only between [`start`](Self::start) and [`stop`](Self::stop), and about the *count*: a
    /// counter that stops has lost time, and a stopped one has no time to lose.
    running_guard: MaybeWakeGuard,

    /// The floor [`start`](Self::start) takes, worked out once.
    ///
    /// Recomputing it per start would read the live clock tree under a critical section, which is
    /// more than the whole rest of `start` costs.
    ///
    /// Behind the cfg because of what computing it *calls*, not because of the byte it occupies:
    /// `sleep_floor` reaches the clock tree through `critical_section::with`, which does not inline,
    /// so a build with no deep sleep to guard against pays 60 bytes for a figure it never reads.
    #[cfg(feature = "low-power")]
    operation_floor: Option<SleepLevel>,
}

impl<'d, T: Instance> Timer<'d, T> {
    /// Power up an instance and apply `config`, leaving the counter stopped at 0.
    pub fn new(timer: Peri<'d, T>, config: Config) -> Self {
        configure::<T>(&config);

        Self {
            _timer: timer,
            _config_guard: MaybeWakeGuard::new(T::SLEEP.floor_to_keep_configured()),
            running_guard: MaybeWakeGuard::none(),
            #[cfg(feature = "low-power")]
            operation_floor: sleep_floor::<T>(config.clock),
        }
    }

    /// Apply a new [`Config`] to an instance that is already up, and stop the counter.
    ///
    /// For an instance that changes role while the program runs — a PWM source that becomes a one-shot
    /// countdown, say. [`new`](Self::new) cannot do it: it re-runs reset and power-up, which restarts
    /// the peripheral rather than re-aiming it.
    ///
    /// The counter is stopped first and left stopped, so [`start`](Self::start) is what resumes it.
    /// Stopping is not ceremony — `config` can reverse the counting direction, and a counter that
    /// changes direction mid-count is left at a value the new mode never meant to produce.
    /// [`Config::counter_on_enable`] then decides what the next `start` does with it.
    ///
    /// The reload value survives, since nothing in `Config` names it. Set it with
    /// [`set_load`](Self::set_load) if the new role wants a different period.
    ///
    /// This is also what keeps the sleep guard honest. The floor a running counter holds comes from
    /// its clock, so a role change that swaps the clock source re-derives it here — writing `CLKSEL`
    /// through [`regs`](Self::regs) instead leaves the old floor in place, and the counter is then
    /// guarded against the wrong sleep modes with nothing reporting it.
    pub fn reconfigure(&mut self, config: &Config) {
        self.stop();

        apply_config::<T>(config);

        #[cfg(feature = "low-power")]
        {
            self.operation_floor = sleep_floor::<T>(config.clock);
        }
    }

    /// Configure `channel` to drive a PWM output, without claiming a pin for it.
    ///
    /// [`SimplePwm`](super::simple_pwm::SimplePwm) is the ordinary way to get PWM, and it takes the
    /// pin — which is what stops one pin reaching two peripherals. Where a pin has to change role
    /// while the program runs, the channel is set up here and the pin is muxed separately with
    /// [`Flex::set_as_af`](crate::gpio::Flex::set_as_af).
    ///
    /// Duty is then the channel's compare value, [`set_compare`](Self::set_compare), against
    /// [`load`](Self::load).
    pub fn setup_pwm_channel(&mut self, channel: Channel, counting_mode: CountingMode) {
        super::simple_pwm::setup_channel(self.regs(), channel, counting_mode);
    }

    /// Registers of this instance, for what this driver does not wrap.
    #[inline]
    pub fn regs(&self) -> Tim {
        T::info().regs
    }

    /// Power the instance down and give the peripheral back, so another driver can claim it.
    pub fn release(self) -> Peri<'d, T> {
        let mut this = ManuallyDrop::new(self);

        teardown::<T>();

        // SAFETY: `this` is never dropped and no field is touched again, so each is moved out or
        // dropped exactly once.
        unsafe {
            core::ptr::drop_in_place(&mut this._config_guard);
            core::ptr::drop_in_place(&mut this.running_guard);
            core::ptr::read(&this._timer)
        }
    }

    /// Let the counter advance.
    ///
    /// Takes the sleep guard that keeps the counter counting, and holds it until
    /// [`stop`](Self::stop). The guard comes first: between taking it and setting `EN` the counter
    /// is not yet running, where the other order would leave it running unguarded.
    ///
    /// Assigning rather than releasing first matters on a `start` called while already running. The
    /// new guard is taken before the old one is dropped, so the block never reaches zero in between.
    #[inline]
    pub fn start(&mut self) {
        // A block rather than an attribute on the assignment, because an attribute on a bare
        // expression is still unstable.
        #[cfg(feature = "low-power")]
        {
            self.running_guard = MaybeWakeGuard::new(self.operation_floor);
        }

        T::info().regs.counterregs(0).ctrctl().modify(|w| w.set_en(true));
    }

    /// Halt the counter, keeping its value.
    ///
    /// Releases the guard [`start`](Self::start) took, so a stopped timer stops holding the chip out
    /// of the sleep modes its clock would not survive. Stopping comes first, for the same reason the
    /// guard comes first in `start`.
    #[inline]
    pub fn stop(&mut self) {
        T::info().regs.counterregs(0).ctrctl().modify(|w| w.set_en(false));

        self.running_guard.release();
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
        clock_source(T::info().regs)
    }

    /// Rate the counter advances at, in Hz.
    pub fn tick_frequency(&self) -> u32 {
        tick_frequency(T::info().regs, T::SLEEP.power_domain)
    }

    /// Ticks in one counting period.
    pub fn period_ticks(&self) -> u32 {
        period_ticks(T::info().regs)
    }

    /// Set the period so the counter completes one period at `hz`.
    ///
    /// Errors outside [`Timer::tick_frequency`] down to that divided by the counter's full range.
    pub fn set_frequency(&self, hz: u32) -> Result<(), ConfigError> {
        set_frequency(T::info().regs, T::SLEEP.power_domain, T::Word::MAX.into(), hz)
    }

    /// Program a load value, rejecting one the counter cannot hold.
    ///
    /// Takes what [`solve_load`] works out ahead of time, so a frequency known up front reaches the
    /// register without the device dividing for it.
    pub fn set_load_value(&self, load: u32) -> Result<(), ConfigError> {
        set_load_value(T::info().regs, T::Word::MAX.into(), load)
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

    // SLAU846 §2.2.6 and the same note in all four TRMs: after setting `PWREN.ENABLE`, wait at least
    // **4 ULPCLK cycles** before accessing the rest of the peripheral's registers, while the bus
    // isolation signals update. `MCLKCFG.UDIV` can halve ULPCLK, so 8 MCLK cycles is the worst case
    // across the portfolio.
    //
    // Without it the `CLKSEL` write below is dropped and the timer sits powered and enabled with no
    // source selected — a counter that never advances. It reached the time driver on any tree where
    // `clock::apply` has work to do; the reset tree happened to be slow enough to survive.
    //
    // Polling is not an alternative. The note covers "the rest of" the registers, so `PWREN` itself
    // stays readable and reads back true while writes behind it are still being lost.
    //
    // The count is generous on purpose: `asm::delay` runs iterations rather than cycles, about five
    // each here, so this is roughly 80 MCLK cycles against a requirement of 8. Do not trim it toward
    // the TRM's figure — the units are not the same.
    cortex_m::asm::delay(16);

    apply_config::<T>(config);

    r.counterregs(0).load().write_value(T::Word::MAX.into());
}

/// Program `config` into an instance that is already powered up and out of reset.
///
/// Everything [`configure`] sets that survives being set again, which is all of it bar the reload
/// value. Split out so [`Timer::reconfigure`] can change a running instance's role without the reset
/// and power-up sequence, which would be a restart rather than a mode change.
fn apply_config<T: Instance>(config: &Config) {
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
}

/// Shallowest sleep level to block so an instance clocked from `clock` keeps counting, if any.
///
/// `None` means the counter survives every mode the chip has, so nothing needs blocking.
///
/// Depends on the configured tree, so this reads the live clocks rather than answering at compile
/// time as it did while the tree was fixed.
#[cfg(any(feature = "low-power", feature = "_time-driver"))]
pub(crate) fn sleep_floor<T: Instance>(clock: ClockSel) -> Option<SleepLevel> {
    let clock_hz = crate::sysctl::with_clocks(|clocks| clock.frequency(clocks, T::SLEEP.power_domain));

    // The domain-level answer cannot know which instances stay clocked in STANDBY1, and reports
    // STANDBY1 for any LFCLK peripheral in PD0.
    if matches!(T::SLEEP.clocked_in_standby1, Some(true)) && clock_hz <= crate::sysctl::LFCLK_HZ {
        return None;
    }

    T::SLEEP.floor_for_operation(clock_hz)
}

/// Every channel's up-direction capture/compare flag.
///
/// The only bits the capture and compare handlers acknowledge, so an event the caller enabled through
/// [`Timer`] is left alone.
pub(crate) const CC_UP_BITS: u32 = Event::CaptureOrCompareUp(Channel::Ch0).mask().0
    | Event::CaptureOrCompareUp(Channel::Ch1).mask().0
    | Event::CaptureOrCompareUp(Channel::Ch2).mask().0
    | Event::CaptureOrCompareUp(Channel::Ch3).mask().0;

/// Every channel's down-direction capture/compare flag.
pub(crate) const CC_DOWN_BITS: u32 = Event::CaptureOrCompareDown(Channel::Ch0).mask().0
    | Event::CaptureOrCompareDown(Channel::Ch1).mask().0
    | Event::CaptureOrCompareDown(Channel::Ch2).mask().0
    | Event::CaptureOrCompareDown(Channel::Ch3).mask().0;

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

/// Set the period so the counter completes one period at `hz`.
///
/// `word_max` is the instance's counter width, the one fact this needs that the register block does
/// not carry.
fn set_frequency(regs: Tim, domain: crate::sysctl::PowerDomain, word_max: u32, hz: u32) -> Result<(), ConfigError> {
    let mode = counting_mode(regs);

    set_load_value(
        regs,
        word_max,
        load_for_frequency(tick_frequency(regs, domain), mode, hz)?,
    )
}

/// Program a load value, rejecting one the counter cannot hold.
fn set_load_value(regs: Tim, word_max: u32, load: u32) -> Result<(), ConfigError> {
    if load > word_max {
        return Err(ConfigError::TooLow);
    }

    regs.counterregs(0).load().write_value(load);

    Ok(())
}

/// Clock source the counter runs from.
pub(crate) fn clock_source(regs: Tim) -> ClockSel {
    let clksel = regs.clksel().read();

    if clksel.lfclk_sel() {
        ClockSel::LfClk
    } else if clksel.mfclk_sel() {
        ClockSel::MfClk
    } else {
        ClockSel::BusClk
    }
}

/// Rate the counter advances at, in Hz.
pub(crate) fn tick_frequency(regs: Tim, domain: crate::sysctl::PowerDomain) -> u32 {
    let divider = regs.clkdiv().read().ratio() as u32 + 1;
    let prescaler = regs.commonregs(0).cps().read().pcnt() as u32 + 1;

    let source_hz = crate::sysctl::with_clocks(|clocks| clock_source(regs).frequency(clocks, domain));

    source_hz / divider / prescaler
}

/// Counting direction and alignment, for the channel handles that have no instance to ask.
pub(crate) fn counting_mode(regs: Tim) -> CountingMode {
    match regs.counterregs(0).ctrctl().read().cm() {
        Cm::Down => CountingMode::EdgeAlignedDown,
        Cm::UpDown => CountingMode::CenterAligned,
        _ => CountingMode::EdgeAlignedUp,
    }
}

/// Ticks in one counting period.
///
/// Saturates: a 32-bit counter loaded to its maximum has a period of 2^32, which does not fit.
pub(crate) fn period_ticks(regs: Tim) -> u32 {
    let load = regs.counterregs(0).load().read();

    match counting_mode(regs) {
        // Up then down passes every value twice except the two endpoints, which it passes once.
        CountingMode::CenterAligned => load.saturating_mul(2),
        _ => load.saturating_add(1),
    }
}

/// Solve the load value for `hz` ahead of time, so the device never divides for it.
///
/// The counter's rate is three divisions away from the clock tree — the source divider, the prescaler,
/// and the period itself — and this core has no divide instruction, so leaving them to run time links a
/// ~400 byte software divider. Worse, [`Timer::tick_frequency`] recovers the two dividers by *reading
/// them back out of their registers*, which no amount of constant propagation can see through; solving
/// here is the only way they fold.
///
/// Takes the tree rather than reading it, so this stays a `const fn`, exactly as
/// [`ClockSel::frequency`] does: pass
/// [`clock::ClockSetup::clocks`](crate::sysctl::clock::ClockSetup::clocks) for the tree
/// [`crate::init`] is being given. A load solved against a tree the device does not end up running puts
/// the period out by the same ratio.
///
/// The arguments are the like-named fields of [`simple_pwm::Config`](crate::tim::simple_pwm::Config),
/// and the answer goes in its `load`. [`None`] means the frequency is unreachable on this instance —
/// what [`Timer::set_frequency`] would have reported as a [`ConfigError`] at run time, but at compile
/// time instead.
///
/// ```ignore
/// use embassy_mspm0::peripherals::TIMG1;
/// use embassy_mspm0::sysctl::clock;
/// use embassy_mspm0::tim::low_level::solve_load;
/// use embassy_mspm0::tim::simple_pwm::Config;
/// use embassy_mspm0::tim::{ClockSel, CountingMode};
///
/// const LOAD: u32 = match solve_load::<TIMG1>(
///     &clock::RESET_SETUP.clocks(),
///     ClockSel::BusClk,
///     1,
///     1,
///     CountingMode::EdgeAlignedUp,
///     1_000,
/// ) {
///     Some(load) => load,
///     None => core::panic!("1 kHz is not reachable with these dividers"),
/// };
///
/// let config = Config {
///     load: Some(LOAD),
///     ..Default::default()
/// };
/// ```
pub const fn solve_load<T: Instance>(
    clocks: &crate::sysctl::Clocks,
    clock: ClockSel,
    divider: u8,
    prescaler: u16,
    counting_mode: CountingMode,
    hz: u32,
) -> Option<u32> {
    if hz == 0 || divider == 0 || prescaler == 0 {
        return None;
    }

    let tick_hz = clock.frequency(clocks, T::SLEEP.power_domain) / divider as u32 / prescaler as u32;

    // Same arithmetic as `load_for_frequency`, which is what the run-time path still uses.
    let load = match counting_mode {
        CountingMode::CenterAligned => tick_hz / hz.saturating_mul(2),
        _ => match (tick_hz / hz).checked_sub(1) {
            Some(load) => load,
            None => return None,
        },
    };

    if load == 0 {
        return None;
    }

    // The counter's width is a compile-time fact, so a load it cannot hold is caught here rather than
    // by `set_load_value` on the device.
    if <T::Word as Word>::BITS < 32 && load > (1 << <T::Word as Word>::BITS) - 1 {
        return None;
    }

    Some(load)
}

/// Load value that makes a counter ticking at `tick_hz` complete one period at `hz`.
fn load_for_frequency(tick_hz: u32, mode: CountingMode, hz: u32) -> Result<u32, ConfigError> {
    if hz == 0 {
        return Err(ConfigError::Zero);
    }

    let load = match mode {
        // One center-aligned period is the range twice over, so it needs half the load an
        // edge-aligned period does. The counter also idles a tick at each endpoint rather than
        // wrapping, which is why this is `2 * load` and not `2 * (load + 1)`.
        CountingMode::CenterAligned => tick_hz / hz.saturating_mul(2),
        _ => (tick_hz / hz).checked_sub(1).ok_or(ConfigError::TooHigh)?,
    };

    // A zero load leaves no room for a duty value between the extremes.
    if load == 0 {
        return Err(ConfigError::TooHigh);
    }

    Ok(load)
}

/// Stop the counter, mask its interrupts and power the instance down.
fn teardown<T: Instance>() {
    let r = T::info().regs;

    r.counterregs(0).ctrctl().modify(|w| w.set_en(false));
    r.cpu_int(0).imask().write_value(regs::Int(0));

    r.gprcm(0).pwren().write(|w| {
        w.set_enable(false);
        w.set_key(PwrenKey::Key);
    });
}

impl<T: Instance> Drop for Timer<'_, T> {
    fn drop(&mut self) {
        teardown::<T>();
    }
}
