use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
#[cfg(feature = "rt")]
use mspm0_metapac::interrupt;
use mspm0_metapac::tim::Tim;

use crate::interrupt::typelevel::Interrupt;
use crate::tim::low_level::{self, CounterOnEnable};
use crate::tim::{Channel, ClockSel, CountingMode, General2ChannelInstance, SealedInstance, Word};
use crate::{peripherals, tim};

#[cfg(time_driver_timg0)]
type T = peripherals::TIMG0;
#[cfg(time_driver_timg1)]
type T = peripherals::TIMG1;
#[cfg(time_driver_timg2)]
type T = peripherals::TIMG2;
#[cfg(time_driver_timg3)]
type T = peripherals::TIMG3;
#[cfg(time_driver_timg4)]
type T = peripherals::TIMG4;
#[cfg(time_driver_timg5)]
type T = peripherals::TIMG5;
#[cfg(time_driver_timg6)]
type T = peripherals::TIMG6;
#[cfg(time_driver_timg7)]
type T = peripherals::TIMG7;
#[cfg(time_driver_timg8)]
type T = peripherals::TIMG8;
#[cfg(time_driver_timg9)]
type T = peripherals::TIMG9;
#[cfg(time_driver_timg10)]
type T = peripherals::TIMG10;
#[cfg(time_driver_timg11)]
type T = peripherals::TIMG11;
#[cfg(time_driver_timg12)]
type T = peripherals::TIMG12;
#[cfg(time_driver_timg13)]
type T = peripherals::TIMG13;
#[cfg(time_driver_timg14)]
type T = peripherals::TIMG14;
#[cfg(time_driver_tima0)]
type T = peripherals::TIMA0;
#[cfg(time_driver_tima1)]
type T = peripherals::TIMA1;

/// Counter value type of the selected timer: `u16`, or `u32` on TIMG12/TIMG13.
type W = <T as tim::Instance>::Word;

/// Ticks one `period` increment covers, as a shift.
const HALF_BITS: u32 = W::BITS - 1;

/// How far ahead an alarm has to be before arming is deferred to [`TimxDriver::next_period`].
///
/// One and a half `period`s, so the compare value is never ambiguous.
const ARM_AHEAD: u64 = 3 << (W::BITS - 2);

// The scheme needs two capture/compare channels: one for the half-period tick, one for the alarm.
const _: fn() = || {
    fn has_two_channels<C: General2ChannelInstance>() {}
    has_two_channels::<T>();
};

fn regs() -> Tim {
    T::info().regs
}

// Clock timekeeping works with something we call "periods", which are time intervals of half the
// counter's range — 2^15 ticks on a 16-bit timer, 2^31 on a 32-bit one. One "overflow cycle" is
// 2 periods.
//
// A `period` count is maintained in parallel to the Timer hardware `counter`, like this:
// - `period` and `counter` start at 0
// - `period` is incremented on overflow (at counter value 0)
// - `period` is incremented "midway" between overflows (at half the counter's range)
//
// When `period` is even the counter is in the lower half of its range, when odd the upper half. This
// allows for now() to return the correct value even if it races an overflow, which is why both events
// are load-bearing for `now()` and neither may be masked.
//
// `period` is a 32-bit integer, so it overflows after 2^32 half-periods: 136 years at 2^15 ticks, and
// far beyond any plausible uptime at 2^31.
//
// Generic over the counter type rather than a bit count, so the width cannot be given a value the
// hardware does not have. `Word::BITS` is an associated const, not a method, so this stays a `const fn`
// and the assertions below run at compile time.
const fn calc_now<C: Word>(period: u32, counter: u32) -> u64 {
    let half_bits = C::BITS - 1;

    ((period as u64) << half_bits) + ((counter ^ ((period & 1) << half_bits)) as u64)
}

// `calc_now` is the one piece of arithmetic here that has to be exactly right, and it is pure, so pin it
// down at compile time. It cannot be a unit test: `cortex-m` does not build for the host, so this crate
// has no host target to run tests on.
const _: () = {
    // 16-bit, half-period 0x8000. The sequence below must be strictly increasing across two parity
    // flips, which is the property `now()` depends on.
    core::assert!(calc_now::<u16>(0, 0x0000) == 0x0_0000);
    core::assert!(calc_now::<u16>(0, 0x7FFF) == 0x0_7FFF);
    core::assert!(calc_now::<u16>(1, 0x8000) == 0x0_8000);
    core::assert!(calc_now::<u16>(1, 0xFFFF) == 0x0_FFFF);
    core::assert!(calc_now::<u16>(2, 0x0000) == 0x1_0000);

    // 32-bit, half-period 0x8000_0000.
    core::assert!(calc_now::<u32>(0, 0x0000_0000) == 0x0_0000_0000);
    core::assert!(calc_now::<u32>(0, 0x7FFF_FFFF) == 0x0_7FFF_FFFF);
    core::assert!(calc_now::<u32>(1, 0x8000_0000) == 0x0_8000_0000);
    core::assert!(calc_now::<u32>(1, 0xFFFF_FFFF) == 0x0_FFFF_FFFF);
    core::assert!(calc_now::<u32>(2, 0x0000_0000) == 0x1_0000_0000);

    // The arming threshold is one and a half periods, whatever the width.
    core::assert!(3u64 << (16 - 2) == 0xC000);
    core::assert!(3u64 << (32 - 2) == 0xC000_0000);
};

/// TODO: Configurable tick rate
/// TODO: Compensate for per part variance. This can supposedly be done with the FCC system.
struct TimxDriver {
    /// Number of half-counter-range periods elapsed since boot.
    period: AtomicU32,
    /// Timestamp at which to fire the alarm, **stored inverted** — see [`TimxDriver::alarm_at`].
    alarm: Mutex<Cell<u64>>,
    queue: Mutex<RefCell<Queue>>,
}

impl TimxDriver {
    fn init(&'static self, _cs: CriticalSection) {
        // TODO: Configurable tick rate
        //
        // Shared with the user-facing timer drivers, so the power/reset/clock sequence and the CZC/CAC/CLC
        // reserved-reset-value trap live in one place. `LOAD` comes out of this as the counter's full
        // range, which is what the period scheme wants.
        low_level::configure::<T>(&low_level::Config {
            // LFCLK at 32.768 kHz, the only source available all the way down to STANDBY, and no
            // division needed to reach the tick rate.
            clock: ClockSel::LfClk,
            divider: 1,
            prescaler: 1,
            counting_mode: CountingMode::EdgeAlignedUp,
            // Zeroes the counter in the timer's own clock domain. Writing it from here instead does not
            // cross into that domain before the first `now()`, which then reads the written value once
            // and the counter's real zero afterwards — time going backwards by a tick.
            counter_on_enable: CounterOnEnable::Reset,
            free_run_in_debug: true,
        });

        // STANDBY1 unclocks all of PD0 apart from a handful of timers named per chip, and PD1 goes down in
        // every deep-sleep mode. A timer that stops in a mode cannot keep time through it, so forbid that
        // mode instead of stopping the clock: the guard is deliberately never dropped, because the time
        // driver never goes away.
        //
        // The cost is not uniform, which is why this is a silent trade rather than an error. A PD0 timer on
        // LFCLK that is not in the STANDBY1 list loses only that one mode and keeps STANDBY0. A PD1 timer
        // blocks all deep sleep, which defeats the point of a low-power build — `build.rs` warns about that
        // case at compile time, and `time-driver-any` avoids it where it can.
        //
        // `None` here means the timer survives everything and nothing is blocked, which is the case
        // `time-driver-any` selects for.
        if let Some(guard) = low_level::wake_guard::<T>(ClockSel::LfClk) {
            core::mem::forget(guard);
        }

        let regs = regs();

        // Half of the counter's range, the other point where `period` increments.
        regs.counterregs(0).cc(Channel::Ch0.index()).write_value(1 << HALF_BITS);

        regs.counterregs(0).ctrctl().modify(|w| {
            w.set_en(true);
        });

        // Enabling latches the events `period` counts. Unmasking without clearing them first delivers
        // both immediately and advances the clock by a period apiece.
        regs.cpu_int(0).iclr().write(|w| {
            w.set_z(true);
            w.set_ccu0(true);
            w.set_ccu1(true);
        });

        regs.cpu_int(0).imask().modify(|w| {
            w.set_z(true);
            w.set_ccu0(true);
        });

        <T as tim::Instance>::Interrupt::IRQ.unpend();
        unsafe { <T as tim::Instance>::Interrupt::IRQ.enable() };
    }

    fn next_period(&self, cs: CriticalSection) {
        let r = regs();

        // We only modify the period from the timer interrupt, so we know this can't race.
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);
        let t = (period as u64) << HALF_BITS;

        r.cpu_int(0).imask().modify(move |w| {
            if self.alarm_at(cs) < t + ARM_AHEAD {
                // just enable it. `set_alarm` has already set the correct CC1 val.
                w.set_ccu1(true);
            }
        });
    }

    fn on_interrupt(&self) {
        let r = regs();

        critical_section::with(|cs| {
            let mis = r.cpu_int(0).mis().read();

            // Clear the flags we are about to handle before handling them. MIS and ICLR have the
            // same layout, and writing back only the bits we read leaves any event latched during
            // the handler pending, so it is picked up on re-entry rather than lost.
            r.cpu_int(0).iclr().write_value(mis);

            // Overflow
            if mis.z() {
                self.next_period(cs);
            }

            // Half overflow
            if mis.ccu0() {
                self.next_period(cs);
            }

            if mis.ccu1() {
                self.trigger_alarm(cs);
            }
        });
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());

        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }

    /// The timestamp the alarm is armed for, `u64::MAX` when it is disarmed.
    ///
    /// Held inverted in the cell so that disarmed is zero and the whole of [`DRIVER`] is
    /// zero-initialised. One non-zero field would put all of it in `.data`, which is flash-resident on
    /// this target, so it would cost its own size in flash on top of the RAM it already takes.
    fn alarm_at(&self, cs: CriticalSection) -> u64 {
        !self.alarm.borrow(cs).get()
    }

    /// Arm for `timestamp`, or disarm with `u64::MAX`. See [`TimxDriver::alarm_at`].
    fn set_alarm_at(&self, cs: CriticalSection, timestamp: u64) {
        self.alarm.borrow(cs).set(!timestamp);
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let r = regs();

        self.set_alarm_at(cs, timestamp);

        let t = self.now();

        if timestamp <= t {
            // If alarm timestamp has passed the alarm will not fire.
            // Disarm the alarm and return `false` to indicate that.
            r.cpu_int(0).imask().modify(|w| w.set_ccu1(false));

            self.set_alarm_at(cs, u64::MAX);

            return false;
        }

        // Write the CC1 value regardless of whether we're going to enable it now or not.
        // This way, when we enable it later, the right value is already set.
        //
        // Narrowed to the counter's width, so the compare is a value the counter actually reaches.
        r.counterregs(0)
            .cc(Channel::Ch1.index())
            .write_value(W::from_reg(timestamp as u32).into());

        // Enable it if it'll happen soon. Otherwise, `next_period` will enable it.
        let diff = timestamp - t;
        r.cpu_int(0).imask().modify(|w| w.set_ccu1(diff < ARM_AHEAD));

        // Reevaluate if the alarm timestamp is still in the future
        let t = self.now();
        if timestamp <= t {
            // If alarm timestamp has passed since we set it, we have a race condition and
            // the alarm may or may not have fired.
            // Disarm the alarm and return `false` to indicate that.
            // It is the caller's responsibility to handle this ambiguity.
            r.cpu_int(0).imask().modify(|w| w.set_ccu1(false));

            self.set_alarm_at(cs, u64::MAX);

            return false;
        }

        // We're confident the alarm will ring in the future.
        true
    }
}

impl Driver for TimxDriver {
    fn now(&self) -> u64 {
        let regs = regs();

        // On MSPM0 this sequence reread and comparison must be done or else time may
        // appear to go backwards.
        loop {
            let period = self.period.load(Ordering::Relaxed);
            // Ensure the compiler does not read the counter before the period.
            compiler_fence(Ordering::Acquire);

            let counter = W::from_reg(regs.counterregs(0).ctr().read()).into();

            // Ensure the compiler does not read the period again before the counter.
            compiler_fence(Ordering::Acquire);
            let period2 = self.period.load(Ordering::Relaxed);

            if period != period2 {
                continue;
            }

            return calc_now::<W>(period, counter);
        }
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());

                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimxDriver = TimxDriver {
    period: AtomicU32::new(0),
    // Disarmed, which inverted is zero, which is what keeps `DRIVER` out of `.data`.
    alarm: Mutex::new(Cell::new(!u64::MAX)),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs);
}

/// Whether this driver leaves the core asleep for at least `ticks`.
///
/// Both the queued alarm and the `period` tick count. Nobody asks for the tick, but `now()` depends on
/// it, so its interrupt is never masked and it cuts short any sleep entered just before one.
///
/// Asked as a predicate rather than as a distance: the caller only ever compares the answer against
/// its minimum, and neither the saturating subtraction nor the minimum of the two wakes has to be
/// evaluated to decide that.
///
/// **A `ticks` of zero answers `false` while an alarm is overdue, and that is deliberate.** Asking the
/// distance instead would have called an overdue alarm zero ticks away and let the sleep through, which
/// is the one case where the two phrasings disagree. Declining is what is wanted: an alarm already due is
/// work the core should be doing rather than sleeping on. Nothing reaches it by default —
/// [`Config::min_sleep`](crate::Config::min_sleep) is non-zero — so it is not worth an early return, but
/// it is not an off-by-one either.
#[cfg(feature = "low-power")]
pub(crate) fn wake_at_least(cs: CriticalSection, ticks: u32) -> bool {
    let now = DRIVER.now();

    // The low `HALF_BITS` of `now` are the position within the current period, so what is left of it
    // is the distance to the next tick, whether that comes from the overflow or the half-range compare.
    let period_end = (1 << HALF_BITS) - (now & ((1 << HALF_BITS) - 1));
    let ticks = ticks as u64;

    // A disarmed alarm reads `u64::MAX`, which is further off than any `ticks`.
    period_end >= ticks && DRIVER.alarm_at(cs) >= now.saturating_add(ticks)
}

#[cfg(all(time_driver_timg0, feature = "rt"))]
#[interrupt]
fn TIMG0() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg1, feature = "rt"))]
#[interrupt]
fn TIMG1() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg2, feature = "rt"))]
#[interrupt]
fn TIMG2() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg3, feature = "rt"))]
#[interrupt]
fn TIMG3() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg4, feature = "rt"))]
#[interrupt]
fn TIMG4() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg5, feature = "rt"))]
#[interrupt]
fn TIMG5() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg6, feature = "rt"))]
#[interrupt]
fn TIMG6() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg7, feature = "rt"))]
#[interrupt]
fn TIMG7() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg8, feature = "rt"))]
#[interrupt]
fn TIMG8() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg9, feature = "rt"))]
#[interrupt]
fn TIMG9() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg10, feature = "rt"))]
#[interrupt]
fn TIMG10() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg11, feature = "rt"))]
#[interrupt]
fn TIMG11() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg12, feature = "rt"))]
#[interrupt]
fn TIMG12() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg13, feature = "rt"))]
#[interrupt]
fn TIMG13() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_timg14, feature = "rt"))]
#[interrupt]
fn TIMG14() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_tima0, feature = "rt"))]
#[interrupt]
fn TIMA0() {
    DRIVER.on_interrupt();
}

#[cfg(all(time_driver_tima1, feature = "rt"))]
#[interrupt]
fn TIMA1() {
    DRIVER.on_interrupt();
}
