//! Timer (TIM) drivers.

#![macro_use]

pub mod compare;
pub mod input_capture;
pub mod low_level;
// The timekeeping core, for whichever front-end is compiled. Nothing else needs it.
// `pub` only so the RTIC monotonic's generated backends can name the counter in a trait
// signature. Nothing in it is callable from outside this crate.
#[cfg(any(feature = "_time-driver", feature = "rtic-monotonic"))]
#[doc(hidden)]
pub mod period;
pub mod simple_pwm;

use embassy_hal_internal::PeripheralType;
use mspm0_metapac::tim::Tim;

use crate::gpio::Pin;
use crate::interrupt;
use crate::sync::irq_waker::IrqWaker;
use crate::sysctl::{LowPowerInstance, PowerDomain};

/// A timer instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + LowPowerInstance {
    /// Interrupt this instance raises.
    type Interrupt: interrupt::typelevel::Interrupt;

    /// Counter value type: `u16` on a 16-bit timer, `u32` on a 32-bit one.
    type Word: Word;
}

/// A timer instance with 2 compare and capture channels.
pub trait General2ChannelInstance: Instance {}

/// A timer instance with 4 compare and capture channels.
pub trait General4ChannelInstance: General2ChannelInstance {}

/// A timer instance with a 32-bit counter.
pub trait General32BitInstance: Instance {}

/// An advanced timer instance with complementary channel outputs and fault detection.
pub trait AdvancedInstance: Instance {}

/// Counter value type of a timer instance.
///
/// The registers are 32 bits wide whatever the counter, so this is what stops an out-of-range compare
/// being written and then never matching.
#[allow(private_bounds)]
pub trait Word: SealedWord + Copy + Ord + Into<u32> + 'static {
    /// Counter width in bits.
    const BITS: u32;

    /// Largest value the counter reaches.
    const MAX: Self;

    /// Narrow a counter register read to the counter width.
    fn from_reg(reg: u32) -> Self;
}

impl Word for u16 {
    const BITS: u32 = 16;
    const MAX: Self = u16::MAX;

    #[inline]
    fn from_reg(reg: u32) -> Self {
        reg as u16
    }
}

impl Word for u32 {
    const BITS: u32 = 32;
    const MAX: Self = u32::MAX;

    #[inline]
    fn from_reg(reg: u32) -> Self {
        reg
    }
}

/// A timer pin.
#[allow(private_bounds)]
pub trait TimerPin<T: Instance, Channel: TimerChannel>: Pin + PeripheralType {
    /// Get the PF number needed to use this pin as a timer pin for the specified [`Channel`].
    fn pf_num(&self) -> u8;
}

/// Capture/compare channel of a timer.
///
/// Only the four that reach a pin; advanced instances have two more, via [`low_level::Timer::regs`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Channel {
    /// Channel 0.
    Ch0,

    /// Channel 1.
    Ch1,

    /// Channel 2.
    Ch2,

    /// Channel 3.
    Ch3,
}

impl Channel {
    /// Every channel, in index order.
    pub const ALL: [Channel; 4] = [Channel::Ch0, Channel::Ch1, Channel::Ch2, Channel::Ch3];

    /// Index of this channel in the per-channel register arrays.
    pub const fn index(self) -> usize {
        match self {
            Channel::Ch0 => 0,
            Channel::Ch1 => 1,
            Channel::Ch2 => 2,
            Channel::Ch3 => 3,
        }
    }
}

/// A timer channel
#[allow(private_bounds)]
pub trait TimerChannel: SealedChannel {
    /// Channel this marker selects; complementary markers share their channel's.
    const CHANNEL: Channel;
}

/// Marker type for channel 0.
pub enum Ch0 {}
impl TimerChannel for Ch0 {
    const CHANNEL: Channel = Channel::Ch0;
}

/// Marker type for channel 1.
pub enum Ch1 {}
impl TimerChannel for Ch1 {
    const CHANNEL: Channel = Channel::Ch1;
}

/// Marker type for channel 2.
pub enum Ch2 {}
impl TimerChannel for Ch2 {
    const CHANNEL: Channel = Channel::Ch2;
}

/// Marker type for channel 3.
pub enum Ch3 {}
impl TimerChannel for Ch3 {
    const CHANNEL: Channel = Channel::Ch3;
}

/// Marker type for channel 0 complementary output.
pub enum CompCh0 {}
impl TimerChannel for CompCh0 {
    const CHANNEL: Channel = Channel::Ch0;
}

/// Marker type for channel 1 complementary output.
pub enum CompCh1 {}
impl TimerChannel for CompCh1 {
    const CHANNEL: Channel = Channel::Ch1;
}

/// Marker type for channel 2 complementary output.
pub enum CompCh2 {}
impl TimerChannel for CompCh2 {
    const CHANNEL: Channel = Channel::Ch2;
}

/// Marker type for channel 3 complementary output.
pub enum CompCh3 {}
impl TimerChannel for CompCh3 {
    const CHANNEL: Channel = Channel::Ch3;
}

/// Clock source for the timer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSel {
    /// 32.768 kHz, the only source that keeps counting in STANDBY.
    LfClk,

    /// 4 MHz, stops below STOP1.
    MfClk,

    /// The power domain's bus clock: MCLK in PD1, ULPCLK in PD0. Stops in any deep-sleep mode.
    BusClk,
}

impl ClockSel {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::BusClk;
}

impl Default for ClockSel {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ClockSel {
    /// Frequency of this source, in Hz, for a timer in `domain`.
    ///
    /// Takes the tree rather than reading it, so this stays a `const fn`. Get one from
    /// [`crate::sysctl::clocks`].
    pub const fn frequency(self, clocks: &crate::sysctl::Clocks, domain: PowerDomain) -> u32 {
        match self {
            ClockSel::LfClk => clocks.lfclk,
            // MFCLK is held at 4 MHz by SYSCTL whatever SYSOSC is doing, and reads as 0 when it was
            // never enabled, in which case the timer would not be counting at all.
            ClockSel::MfClk => clocks.mfclk,
            ClockSel::BusClk => clocks.bus_clock(domain),
        }
    }
}

/// Which way the counter runs, and where a period starts.
///
/// # Porting a driverlib configuration
///
/// **`DL_TIMER_PWM_MODE_EDGE_ALIGN` is down-counting.** The unqualified name is
/// `GPTIMER_CTRCTL_CM_DOWN`, and `DL_TIMER_PWM_MODE_EDGE_ALIGN_UP` is the up-counting one, so
/// translating by name gives the opposite mode. They also program opposite output actions —
/// `LACT`/`CDACT` against `ZACT`/`CUACT` — which puts the high time at `LOAD - CC` for one and `CC`
/// for the other.
///
/// A port that carries over driverlib's compare arithmetic as well as its mode inverts twice. The
/// duty setters here take a duty in ticks and do the per-mode conversion themselves, so
/// [`SimplePwmChannel::set_duty`](simple_pwm::SimplePwmChannel::set_duty) and
/// [`low_level::Timer::set_pwm_duty`] are the ones to aim a port at, not [`low_level::Timer::set_compare`].
///
/// **The mistake survives testing at 50%**, which is its own complement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CountingMode {
    /// The timer counts up to the reload value and then resets back at 0.
    ///
    /// driverlib's `DL_TIMER_PWM_MODE_EDGE_ALIGN_UP`.
    EdgeAlignedUp,

    /// The timer counts down to 0 and then resets back to the load value.
    ///
    /// driverlib's `DL_TIMER_PWM_MODE_EDGE_ALIGN`, whose name does not say so.
    EdgeAlignedDown,

    /// The timer counts up to the load value and then counts back to 0.
    ///
    /// driverlib's `DL_TIMER_PWM_MODE_CENTER_ALIGN`.
    CenterAligned,
}

impl CountingMode {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::EdgeAlignedUp;
}

impl Default for CountingMode {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Which way an edge-aligned counter runs.
///
/// The drivers that cannot express [`CountingMode::CenterAligned`] take this instead, so the mode they
/// reject is not representable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CountingDirection {
    /// Count up from zero.
    Up,

    /// Count down from the load value.
    Down,
}

impl CountingDirection {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::Up;
}

impl Default for CountingDirection {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CountingDirection {
    /// The counting mode this direction selects.
    pub const fn counting_mode(self) -> CountingMode {
        match self {
            CountingDirection::Up => CountingMode::EdgeAlignedUp,
            CountingDirection::Down => CountingMode::EdgeAlignedDown,
        }
    }
}

impl CountingMode {
    /// Which way this mode runs the counter.
    ///
    /// [`Self::CenterAligned`] runs both ways and answers [`CountingDirection::Up`]; the drivers that
    /// ask cannot select it.
    pub const fn direction(self) -> CountingDirection {
        match self {
            CountingMode::EdgeAlignedDown => CountingDirection::Down,
            _ => CountingDirection::Up,
        }
    }
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    /// One waker per channel this instance has.
    ///
    /// A slice rather than `&'static State<N>`, because a trait method cannot return a type whose size
    /// varies per implementor without `generic_const_exprs`. Monomorphised per instance the pointer and
    /// the length are both constants, so it costs nothing to hand out.
    fn cc_wakers() -> &'static [IrqWaker];
}

/// Peripheral state, sized by how many capture/compare channels the instance has.
///
/// `N` comes from the metapac's `ccp_channels` by way of `build.rs`, the same figure that decides
/// whether an instance implements [`General4ChannelInstance`], so an instance cannot carry slots for
/// channels it does not have.
pub(crate) struct State<const N: usize> {
    /// Woken by a capture or compare event on the channel of the same index.
    pub(crate) cc: [IrqWaker; N],
}

impl<const N: usize> State<N> {
    pub(crate) const fn new() -> Self {
        Self {
            cc: [const { IrqWaker::new() }; N],
        }
    }
}

trait SealedWord {}
impl SealedWord for u16 {}
impl SealedWord for u32 {}

trait SealedChannel {}
impl SealedChannel for Ch0 {}
impl SealedChannel for Ch1 {}
impl SealedChannel for Ch2 {}
impl SealedChannel for Ch3 {}
impl SealedChannel for CompCh0 {}
impl SealedChannel for CompCh1 {}
impl SealedChannel for CompCh2 {}
impl SealedChannel for CompCh3 {}

/// Release every pin a driver holds, on the way out.
///
/// A free function over the erased pins rather than three identical loops: a `Drop` impl is generic
/// over the instance even when its body is not, so written inline this is one copy per driver per
/// timer a binary builds. The destructor path is where most of that duplication was — more than a
/// third of it on an application driving three timers.
pub(crate) fn disconnect_pins(pins: &[crate::gpio::MaybeAnyPin<'_>; 4]) {
    for pin in pins.iter().filter_map(crate::gpio::MaybeAnyPin::pin) {
        crate::gpio::SealedPin::set_as_disconnected(&pin);
    }
}

pub(crate) struct Info {
    pub(crate) regs: Tim,
    /// Whether this instance has the 8-bit prescaler in `CPS`.
    pub(crate) prescaler: bool,
    /// Capture/compare channels brought out to pins.
    pub(crate) channels: u8,
}

macro_rules! impl_tim_instance {
    (
        $instance: ident,
        prescaler: $prescaler: expr,
        word: $word: ty,
        channels: $channels: expr
    ) => {
        impl crate::tim::SealedInstance for crate::peripherals::$instance {
            #[inline]
            fn info() -> &'static crate::tim::Info {
                const INFO: crate::tim::Info = crate::tim::Info {
                    regs: crate::pac::$instance,
                    prescaler: $prescaler,
                    channels: $channels,
                };

                &INFO
            }

            #[inline]
            fn cc_wakers() -> &'static [crate::sync::irq_waker::IrqWaker] {
                static STATE: crate::tim::State<{ $channels as usize }> = crate::tim::State::new();

                &STATE.cc
            }
        }

        impl crate::tim::Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
            type Word = $word;
        }
    };
}

#[allow(unused)]
macro_rules! impl_tim_instance_general_32bit {
    ($instance: ident) => {
        impl crate::tim::General32BitInstance for crate::peripherals::$instance {}
    };
}

#[allow(unused)]
macro_rules! impl_tim_instance_general_2ch {
    ($instance: ident) => {
        impl crate::tim::General2ChannelInstance for crate::peripherals::$instance {}
    };
}

#[allow(unused)]
macro_rules! impl_tim_instance_general_4ch {
    ($instance: ident) => {
        impl crate::tim::General4ChannelInstance for crate::peripherals::$instance {}
    };
}

#[allow(unused)]
macro_rules! impl_tim_instance_advanced {
    ($instance: ident) => {
        impl crate::tim::AdvancedInstance for crate::peripherals::$instance {}
    };
}

macro_rules! impl_tim_pin {
    (
        $instance: ident,
        $pin: ident,
        $pf: expr,
        $channel: ident
    ) => {
        impl crate::tim::TimerPin<crate::peripherals::$instance, crate::tim::$channel> for crate::peripherals::$pin {
            #[inline]
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}
