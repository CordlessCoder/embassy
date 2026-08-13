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
use crate::sysctl::MaybeWakeGuard;
use crate::tim::low_level::{self, CounterOnEnable};
use crate::tim::{Channel, ClockSel, CountingMode, General2ChannelInstance, SealedInstance, Word};
use crate::{peripherals, tim};

/// Emit the selected instance's type alias and its interrupt handler.
///
/// Exactly one `time_driver_*` cfg is ever enabled, so exactly one arm of each expands.
/// `build.rs::TIME_DRIVER_TIMERS` is the same list and declares every cfg named here.
macro_rules! time_driver_instances {
    ($($peri:ident => $cfg:ident),* $(,)?) => {
        $(
            #[cfg($cfg)]
            type T = peripherals::$peri;

            #[cfg(all($cfg, feature = "rt"))]
            #[interrupt]
            fn $peri() {
                DRIVER.on_interrupt();
            }
        )*
    };
}

time_driver_instances!(
    TIMG0 => time_driver_timg0,
    TIMG1 => time_driver_timg1,
    TIMG2 => time_driver_timg2,
    TIMG3 => time_driver_timg3,
    TIMG4 => time_driver_timg4,
    TIMG5 => time_driver_timg5,
    TIMG6 => time_driver_timg6,
    TIMG7 => time_driver_timg7,
    TIMG8 => time_driver_timg8,
    TIMG9 => time_driver_timg9,
    TIMG10 => time_driver_timg10,
    TIMG11 => time_driver_timg11,
    TIMG12 => time_driver_timg12,
    TIMG13 => time_driver_timg13,
    TIMG14 => time_driver_timg14,
    TIMA0 => time_driver_tima0,
    TIMA1 => time_driver_tima1,
);

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

    // The arming threshold is one and a half periods, whatever the width. Clippy folds both sides of
    // these two and reports them as the same expression, which is what an assertion pinning an
    // arithmetic identity looks like from the outside.
    #[allow(clippy::eq_op, clippy::assertions_on_constants)]
    {
        core::assert!(3u64 << (16 - 2) == 0xC000);
        core::assert!(3u64 << (32 - 2) == 0xC000_0000);
    }
};

/// `embassy-time`'s clock, kept by a timer counting half its range at a time.
///
/// The counter is narrower than the timestamps callers get, so `period` supplies the high bits and
/// its parity says which half the counter is in. Everything that reads the two together does so
/// under a critical section, since a wrap between the two reads would place the timestamp a whole
/// half-range out.
///
/// The tick rate is fixed at build time by the `time-driver-*` features, and nothing corrects for
/// the oscillator's per-part error, so a long interval drifts by whatever the source is specified
/// at.
struct TimxDriver {
    /// Number of half-counter-range periods elapsed since boot.
    period: AtomicU32,
    /// Timestamp at which to fire the alarm, **stored inverted** — see [`TimxDriver::alarm_at`].
    alarm: Mutex<Cell<u64>>,
    queue: Mutex<RefCell<Queue>>,
}

impl TimxDriver {
    fn init(&'static self, _cs: CriticalSection) {
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
        //
        // The lint below is reading the wrong build. Without `low-power` a `MaybeWakeGuard` is an empty
        // struct with no `Drop`, so forgetting one is correctly a no-op; with the feature it holds a real
        // guard and the forget is the point. Clippy sees only the first.
        #[allow(clippy::forget_non_drop)]
        core::mem::forget(MaybeWakeGuard::new(low_level::sleep_floor::<T>(ClockSel::LfClk)));

        let regs = regs();

        // Half of the counter's range, the other point where `period` increments.
        regs.counterregs(0).cc(Channel::Ch0.index()).write_value(1 << HALF_BITS);

        regs.counterregs(0).ctrctl().modify(|w| {
            w.set_en(true);
        });

        // Enabling latches the events `period` counts. Unmasking without clearing them first delivers
        // both immediately and advances the clock by a period apiece.
        //
        // The clear does not close the window entirely. `CTRCTL.EN` takes a functional clock cycle to
        // take effect — 30.5 us at LFCLK undivided, per `TIMER_ERR_04` — while this write follows it a
        // few core cycles later, so the counter can start and latch an event after the clear has already
        // happened. That costs one spurious handler entry near the first tick. It is harmless because
        // `next_period` reconciles rather than counting entries; it was not harmless before that.
        regs.cpu_int(0).iclr().write(|w| {
            w.set_z(true);
            w.set_ccu(0, true);
            w.set_ccu(1, true);
        });

        regs.cpu_int(0).imask().modify(|w| {
            w.set_z(true);
            w.set_ccu(0, true);
        });

        <T as tim::Instance>::Interrupt::IRQ.unpend();
        unsafe { <T as tim::Instance>::Interrupt::IRQ.enable() };
    }

    fn next_period(&self, cs: CriticalSection) {
        let r = regs();

        // Advance only when the counter disagrees with what `period` claims.
        //
        // `calc_now` reads the period's parity as saying which half of its range the counter is in, so
        // the two have to stay in step. Advancing on every entry does not keep them there: the timer is
        // clocked from LFCLK, so clearing the interrupt needs an LFCLK edge to take effect while the
        // handler returns in a microsecond. The core re-enters on the same event, and blind counting
        // takes the parity with it — permanently, since nothing ever puts it back. Measured: an extra
        // advance ~47 us after a real one, and from then on `now()` sits half a counter range out.
        //
        // Reconciling instead makes a repeated entry a no-op. The repeat is still worth removing, but it
        // stops being a correctness problem and becomes a wasted wake.
        let counter: u32 = W::from_reg(r.counterregs(0).ctr().read()).into();
        let period = self.period.load(Ordering::Relaxed);

        // We only modify the period from the timer interrupt, so we know this can't race.
        if (period & 1) == (counter >> HALF_BITS) {
            return;
        }

        let period = period + 1;
        self.period.store(period, Ordering::Relaxed);

        #[cfg(feature = "_probe")]
        crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverPeriod));

        let t = (period as u64) << HALF_BITS;

        let arming = self.alarm_at(cs) < t + ARM_AHEAD;

        #[cfg(feature = "_probe")]
        if arming {
            crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverArm));
        }

        if arming {
            // Just unmask it: `set_alarm` has already written CC1.
            r.cpu_int(0).imask().modify(|w| w.set_ccu(1, true));
        }
    }

    fn on_interrupt(&self) {
        let r = regs();

        #[cfg(feature = "_probe")]
        crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverIrqEntry));

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
            if mis.ccu(0) {
                self.next_period(cs);
            }

            if mis.ccu(1) {
                #[cfg(feature = "_probe")]
                crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverAlarm));

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
            r.cpu_int(0).imask().modify(|w| w.set_ccu(1, false));

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

        #[cfg(feature = "_probe")]
        if diff < ARM_AHEAD {
            crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverSetAlarm));
        }

        r.cpu_int(0).imask().modify(|w| w.set_ccu(1, diff < ARM_AHEAD));

        // Reevaluate if the alarm timestamp is still in the future
        let t = self.now();
        if timestamp <= t {
            // If alarm timestamp has passed since we set it, we have a race condition and
            // the alarm may or may not have fired.
            // Disarm the alarm and return `false` to indicate that.
            // It is the caller's responsibility to handle this ambiguity.
            r.cpu_int(0).imask().modify(|w| w.set_ccu(1, false));

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

            // `calc_now` assumes the period's parity says which half of its range the counter is in.
            // Nothing enforces that across the clock domain boundary, and breaking it either way
            // overstates the answer by half the range.
            #[cfg(feature = "_probe")]
            if (period & 1) != (counter >> HALF_BITS) {
                crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverSkew));
            }

            return calc_now::<W>(period, counter);
        }
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            // Bound to a local so the queue borrow is released before `trigger_alarm` takes its own.
            let rearm = self.queue.borrow(cs).borrow_mut().schedule_wake(at, waker);

            if rearm {
                self.trigger_alarm(cs);
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
