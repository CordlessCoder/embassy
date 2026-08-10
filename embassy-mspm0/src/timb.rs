//! Basic timer (TIMB) driver.
//!
//! A basic timer is an array of independent 16-bit up-counters sharing one interrupt. Each counts
//! from zero to its own load value, raises an overflow event, and restarts. There is no
//! capture/compare block, no prescaler and no clock divider — [`tim`](crate::tim) is the driver for
//! any of that.
//!
//! What it has instead is a selector per counter for the clock, the start, the stop and the reset,
//! all sharing one encoding: another counter's overflow, a device event, or the instance's event
//! subscriber port.
//!
//! # Chaining, which is the only way to a long period
//!
//! With no prescaler, one counter spans 65536 bus clocks and nothing divides that down. Clocking a
//! counter from a lower one's overflow does: a chain overflows every `∏(load[j] + 1)` bus clocks.
//!
//! ```rust,ignore
//! let timb = BasicTimer::new(p.TIMB0);
//!
//! // 32000 * 1000 bus clocks, a second at 32 MHz.
//! timb.counter(0).set_load(31_999);
//! timb.counter(1).set_clock_source(ClockSource::Overflow(0));
//! timb.counter(1).set_load(999);
//!
//! timb.enable_interrupt(1, Event::Overflow, true);
//! timb.counter(0).start();
//! timb.counter(1).start();
//! ```
//!
//! Only a *lower-indexed* counter can be chained from. A pair reads as one 32-bit count only when
//! the lower counter's load is `0xFFFF`; with any other load the concatenated bits are not a number.
//! Reading a running chain is not atomic either — the low counter can wrap between the two reads —
//! and the hardware offers nothing to help, so read the high counter, the low one, then the high one
//! again and retry while the two disagree.
//!
//! # Sleep
//!
//! The counters are clocked by the bus clock, which for these PD0 instances is ULPCLK. On a device
//! whose datasheet keeps them clocked in STANDBY1 they keep counting there — **at whatever rate
//! ULPCLK runs at in that mode**, which is not the rate a period was programmed against in RUN. The
//! driver holds a [`WakeGuard`] only where the instance would otherwise stop.

#![macro_use]

use core::marker::PhantomData;
use core::mem::ManuallyDrop;

use embassy_hal_internal::{Peri, PeripheralType};
use mspm0_metapac::timb::Tim;
use mspm0_metapac::timb::vals::{PwrenKey, ResetKey};

use crate::interrupt;
use crate::sysctl::{LowPowerInstance, SleepLevel, WakeGuard};

/// What advances a counter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// The bus clock, free-running. The reset value.
    BusClock,

    /// Another counter's overflow, which is what builds a chain.
    ///
    /// Must name a lower-indexed counter than the one being configured.
    Overflow(u8),

    /// One of the device's event lines, 0 to 6.
    ///
    /// Which event each is comes from the device datasheet, and the L-series datasheets do not carry
    /// the table at all.
    Event(u8),

    /// The instance's generic event subscriber port.
    Subscriber,
}

/// What starts, stops or resets a counter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Trigger {
    /// Nothing; the counter is driven by software alone. The reset value.
    None,

    /// Another counter's overflow. Must name a lower-indexed counter.
    Overflow(u8),

    /// One of the device's event lines, 0 to 6.
    Event(u8),

    /// The instance's generic event subscriber port.
    Subscriber,
}

/// The four selectors share one encoding; only the meaning of zero differs.
const fn select_bits(overflow_or_event: Select) -> u8 {
    match overflow_or_event {
        Select::Zero => 0,
        Select::Overflow(counter) => {
            core::assert!(counter < 7, "only counters 0 to 6 can be selected as a source");
            counter + 1
        }
        Select::Event(event) => {
            core::assert!(event < 7, "only events 0 to 6 exist");
            event + 8
        }
        Select::Subscriber => 15,
    }
}

enum Select {
    Zero,
    Overflow(u8),
    Event(u8),
    Subscriber,
}

impl ClockSource {
    const fn bits(self) -> u8 {
        select_bits(match self {
            ClockSource::BusClock => Select::Zero,
            ClockSource::Overflow(counter) => Select::Overflow(counter),
            ClockSource::Event(event) => Select::Event(event),
            ClockSource::Subscriber => Select::Subscriber,
        })
    }
}

impl Trigger {
    const fn bits(self) -> u8 {
        select_bits(match self {
            Trigger::None => Select::Zero,
            Trigger::Overflow(counter) => Select::Overflow(counter),
            Trigger::Event(event) => Select::Event(event),
            Trigger::Subscriber => Select::Subscriber,
        })
    }
}

/// Something a counter reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// The counter reached its load value.
    Overflow,

    /// The counter started.
    Started,

    /// The counter stopped.
    Stopped,
}

/// A basic timer instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + LowPowerInstance {
    /// Interrupt this instance raises, for every counter and every event.
    type Interrupt: interrupt::typelevel::Interrupt;

    /// Independent counters this instance implements.
    ///
    /// Four on the G-series parts and two on the L-series ones. The register block addresses the
    /// eight the TRM documents, so this is what says which of those exist.
    const COUNTERS: u8;
}

pub(crate) trait SealedInstance {
    fn regs() -> Tim;
}

/// Basic timer driver.
///
/// Owns the whole instance, since its counters share an interrupt and can be chained to each other.
pub struct BasicTimer<'d, T: Instance> {
    _timer: Peri<'d, T>,
    /// Held for the driver's lifetime rather than per operation: a counter that stops has lost time.
    _wake_guard: Option<WakeGuard>,
}

impl<'d, T: Instance> BasicTimer<'d, T> {
    /// Power up an instance, leaving every counter stopped at zero and every interrupt masked.
    pub fn new(timer: Peri<'d, T>) -> Self {
        let r = T::regs();

        r.gprcm(0).rstctl().write(|w| {
            w.set_resetassert(true);
            w.set_resetstkyclr(true);
            w.set_key(ResetKey::Key);
        });

        r.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(PwrenKey::Key);
        });

        Self {
            _timer: timer,
            _wake_guard: sleep_floor::<T>().map(WakeGuard::new),
        }
    }

    /// One of the instance's counters.
    ///
    /// Panics if `index` is not below [`Instance::COUNTERS`].
    #[inline]
    pub fn counter(&self, index: u8) -> Counter<'_, T> {
        assert!(index < T::COUNTERS, "this instance does not have that many counters");

        Counter {
            index,
            _timer: PhantomData,
        }
    }

    /// Registers of this instance, for what this driver does not wrap.
    #[inline]
    pub fn regs(&self) -> Tim {
        T::regs()
    }

    /// Report `event` on `counter` through the instance's interrupt, or stop reporting it.
    #[inline]
    pub fn enable_interrupt(&self, counter: u8, event: Event, enable: bool) {
        assert!(counter < T::COUNTERS, "this instance does not have that many counters");

        // Each event has its own field array indexed by counter; the three interleave in the register.
        let counter = counter as usize;
        T::regs().cpu_int(0).imask().modify(|w| match event {
            Event::Overflow => w.set_cntovf(counter, enable),
            Event::Started => w.set_cntstrt(counter, enable),
            Event::Stopped => w.set_cntstop(counter, enable),
        });
    }

    /// Whether `event` has fired on `counter` since it was last cleared.
    #[inline]
    pub fn is_pending(&self, counter: u8, event: Event) -> bool {
        let counter = counter as usize;
        let ris = T::regs().cpu_int(0).ris().read();
        match event {
            Event::Overflow => ris.cntovf(counter),
            Event::Started => ris.cntstrt(counter),
            Event::Stopped => ris.cntstop(counter),
        }
    }

    /// Acknowledge `event` on `counter`.
    #[inline]
    pub fn clear_pending(&self, counter: u8, event: Event) {
        let counter = counter as usize;
        T::regs().cpu_int(0).iclr().write(|w| match event {
            Event::Overflow => w.set_cntovf(counter, true),
            Event::Started => w.set_cntstrt(counter, true),
            Event::Stopped => w.set_cntstop(counter, true),
        });
    }

    /// Power the instance down and give the peripheral back, so another driver can claim it.
    pub fn release(self) -> Peri<'d, T> {
        let mut this = ManuallyDrop::new(self);

        teardown::<T>();

        // SAFETY: `this` is never dropped and neither field is touched again, so each is moved out
        // exactly once.
        unsafe {
            core::ptr::drop_in_place(&mut this._wake_guard);
            core::ptr::read(&this._timer)
        }
    }
}

impl<T: Instance> Drop for BasicTimer<'_, T> {
    fn drop(&mut self) {
        teardown::<T>();
    }
}

/// One counter of a [`BasicTimer`].
pub struct Counter<'a, T: Instance> {
    index: u8,
    _timer: PhantomData<&'a T>,
}

impl<T: Instance> Counter<'_, T> {
    /// Let the counter advance.
    #[inline]
    pub fn start(&self) {
        // A read-modify-write, because hardware start and stop events write `EN` too and a blind
        // write would take a selector with them.
        self.ctl0().modify(|w| w.set_en(true));
    }

    /// Halt the counter, keeping its value.
    #[inline]
    pub fn stop(&self) {
        self.ctl0().modify(|w| w.set_en(false));
    }

    /// Whether the counter is advancing.
    #[inline]
    pub fn is_running(&self) -> bool {
        self.ctl0().read().en()
    }

    /// Current count.
    #[inline]
    pub fn count(&self) -> u16 {
        self.regs().cnt().read().value()
    }

    /// Set the current count.
    #[inline]
    pub fn set_count(&self, count: u16) {
        self.regs().cnt().write(|w| w.set_value(count));
    }

    /// Value the counter overflows at.
    #[inline]
    pub fn load(&self) -> u16 {
        self.regs().ld().read().val()
    }

    /// Set the value the counter overflows at, so it counts `load + 1` ticks per period.
    #[inline]
    pub fn set_load(&self, load: u16) {
        self.regs().ld().write(|w| w.set_val(load));
    }

    /// Set what advances the counter.
    #[inline]
    pub fn set_clock_source(&self, source: ClockSource) {
        self.assert_lower(match source {
            ClockSource::Overflow(counter) => Some(counter),
            _ => None,
        });

        self.ctl0().modify(|w| w.set_clksel(source.bits()));
    }

    /// Set what starts the counter, in addition to [`start`](Self::start).
    #[inline]
    pub fn set_start_trigger(&self, trigger: Trigger) {
        self.assert_lower(match trigger {
            Trigger::Overflow(counter) => Some(counter),
            _ => None,
        });

        self.ctl0().modify(|w| w.set_startsel(trigger.bits()));
    }

    /// Set what stops the counter.
    #[inline]
    pub fn set_stop_trigger(&self, trigger: Trigger) {
        self.assert_lower(match trigger {
            Trigger::Overflow(counter) => Some(counter),
            _ => None,
        });

        self.ctl0().modify(|w| w.set_stopsel(trigger.bits()));
    }

    /// Set what returns the counter to zero.
    #[inline]
    pub fn set_reset_trigger(&self, trigger: Trigger) {
        self.assert_lower(match trigger {
            Trigger::Overflow(counter) => Some(counter),
            _ => None,
        });

        self.ctl0().modify(|w| w.set_resetsel(trigger.bits()));
    }

    #[inline]
    fn assert_lower(&self, source: Option<u8>) {
        if let Some(source) = source {
            assert!(
                source < self.index,
                "a counter can only be driven by a lower-indexed one"
            );
        }
    }

    #[inline]
    fn regs(&self) -> mspm0_metapac::timb::Ctrregs {
        T::regs().ctrregs(self.index as usize)
    }

    #[inline]
    fn ctl0(&self) -> crate::pac::common::Reg<mspm0_metapac::timb::regs::Ctl0, crate::pac::common::RW> {
        self.regs().ctl0()
    }
}

/// Stop every counter and power the instance down.
fn teardown<T: Instance>() {
    let r = T::regs();

    for counter in 0..T::COUNTERS as usize {
        r.ctrregs(counter).ctl0().modify(|w| w.set_en(false));
    }

    r.cpu_int(0).imask().write(|_| {});

    r.gprcm(0).pwren().write(|w| {
        w.set_enable(false);
        w.set_key(PwrenKey::Key);
    });
}

/// Shallowest sleep level to block while a counter runs, or `None` if none of them stop it.
///
/// The counters take the bus clock and nothing else, so unlike [`tim`](crate::tim) there is no clock
/// choice to weigh — only whether this instance is one the device keeps clocked in STANDBY1.
fn sleep_floor<T: Instance>() -> Option<SleepLevel> {
    if matches!(T::SLEEP.clocked_in_standby1, Some(true)) {
        return None;
    }

    let bus_hz = crate::sysctl::with_clocks(|clocks| clocks.bus_clock(T::SLEEP.power_domain));

    T::SLEEP.floor_for_operation(bus_hz)
}

macro_rules! impl_timb_instance {
    ($instance: ident, counters: $counters: expr) => {
        impl crate::timb::SealedInstance for crate::peripherals::$instance {
            #[inline]
            fn regs() -> mspm0_metapac::timb::Tim {
                crate::pac::$instance
            }
        }

        impl crate::timb::Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;

            const COUNTERS: u8 = $counters;
        }
    };
}
