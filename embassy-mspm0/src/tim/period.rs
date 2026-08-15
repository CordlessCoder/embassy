//! A timer counting half its range at a time, so a narrow counter carries a 64-bit timestamp.
//!
//! Split out of the `embassy-time` driver rather than written for it, because a second front-end wants
//! the same thing: the period extension, the deferred compare and the reconciliation below took a
//! measured artefact to get right, and a second copy would not have any of it.
//!
//! # The scheme
//!
//! A `period` count is maintained in parallel to the hardware counter:
//! - `period` and `counter` start at 0
//! - `period` is incremented on overflow (at counter value 0)
//! - `period` is incremented "midway" between overflows (at half the counter's range)
//!
//! When `period` is even the counter is in the lower half of its range, when odd the upper half. That
//! is what lets [`PeriodCounter::now`] return the right value even if it races an overflow, and it is
//! why **both events are load-bearing and neither may be masked**.
//!
//! `period` is a 32-bit integer, so it overflows after 2^32 half-periods: 136 years at 2^15 ticks, and
//! far beyond any plausible uptime at 2^31.

use core::cell::Cell;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};

use critical_section::{CriticalSection, Mutex};
use mspm0_metapac::tim::Tim;

use crate::sysctl::MaybeWakeGuard;
use crate::tim::low_level::{self, CounterOnEnable};
use crate::tim::{Channel, ClockSel, CountingMode, General2ChannelInstance, Instance, Word};

/// Counter value type of `T`: `u16`, or `u32` on TIMG12/TIMG13.
type W<T> = <T as Instance>::Word;

/// Which of the counter's events fired, as `MIS` lays them out.
///
/// The register's own type rather than a struct of `bool`s: the caller tests two or three bits of it
/// and a struct costs twelve bytes in the handler to arrive at the same tests. `Z` is the counter
/// reaching zero, `CCU(0)` its half-range compare, and `CCU(1)` the one the owner arms.
pub(crate) type Events = mspm0_metapac::tim::regs::Int;

/// Timekeeping state for one timer instance.
///
/// Everything here is the counter and its compare. What is done with a compare that fires — a queue, a
/// waker, an alarm — belongs to whoever owns one of these.
pub struct PeriodCounter<T: General2ChannelInstance> {
    /// Number of half-counter-range periods elapsed since boot.
    period: AtomicU32,
    /// Timestamp at which to fire the compare, **stored inverted** — see
    /// [`PeriodCounter::compare_at`].
    compare: Mutex<Cell<u64>>,
    _t: PhantomData<T>,
}

/// Timestamp from a `period` count and a counter reading.
///
/// Generic over the counter type rather than a bit count, so the width cannot be given a value the
/// hardware does not have. `Word::BITS` is an associated const, not a method, so this stays a `const fn`
/// and the assertions below run at compile time.
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

impl<T: General2ChannelInstance> PeriodCounter<T> {
    /// Ticks one `period` increment covers, as a shift.
    const HALF_BITS: u32 = W::<T>::BITS - 1;

    /// How far ahead a compare has to be before arming is deferred to
    /// [`PeriodCounter::next_period`].
    ///
    /// One and a half `period`s, so the compare value is never ambiguous.
    const ARM_AHEAD: u64 = 3 << (W::<T>::BITS - 2);

    pub(crate) const fn new() -> Self {
        Self {
            period: AtomicU32::new(0),
            // Disarmed, which inverted is zero, so an owner of one of these stays out of `.data`.
            compare: Mutex::new(Cell::new(!u64::MAX)),
            _t: PhantomData,
        }
    }

    fn regs(&self) -> Tim {
        T::info().regs
    }

    /// Power up the timer, clock it from `clock`, and start it counting.
    ///
    /// Returns the sleep level this timer's clock forces the device to stay above, which the caller
    /// holds for as long as it wants to keep time. `None` means the counter survives every mode.
    ///
    /// The NVIC line is left alone: whoever owns the vector unmasks it, and may want to give it a
    /// priority first.
    pub(crate) fn start(&'static self, _cs: CriticalSection, clock: ClockSel) -> MaybeWakeGuard {
        // Shared with the user-facing timer drivers, so the power/reset/clock sequence and the CZC/CAC/CLC
        // reserved-reset-value trap live in one place. `LOAD` comes out of this as the counter's full
        // range, which is what the period scheme wants.
        low_level::configure::<T>(&low_level::Config {
            clock,
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
        // every deep-sleep mode. A timer that stops in a mode cannot keep time through it, so the caller
        // forbids that mode instead of stopping the clock.
        //
        // The cost is not uniform, which is why this is a silent trade rather than an error. A PD0 timer on
        // LFCLK that is not in the STANDBY1 list loses only that one mode and keeps STANDBY0. A PD1 timer
        // blocks all deep sleep, which defeats the point of a low-power build.
        let guard = MaybeWakeGuard::new(low_level::sleep_floor::<T>(clock));

        let regs = self.regs();

        // Half of the counter's range, the other point where `period` increments.
        regs.counterregs(0)
            .cc(Channel::Ch0.index())
            .write_value(1 << Self::HALF_BITS);

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

        guard
    }

    /// The timestamp now.
    pub(crate) fn now(&self) -> u64 {
        let regs = self.regs();

        // On MSPM0 this sequence reread and comparison must be done or else time may
        // appear to go backwards.
        loop {
            let period = self.period.load(Ordering::Relaxed);
            // Ensure the compiler does not read the counter before the period.
            compiler_fence(Ordering::Acquire);

            let counter = W::<T>::from_reg(regs.counterregs(0).ctr().read()).into();

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
            if (period & 1) != (counter >> Self::HALF_BITS) {
                crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverSkew));
            }

            return calc_now::<W<T>>(period, counter);
        }
    }

    /// Which events fired, clearing exactly those.
    ///
    /// Clearing before handling leaves anything latched during the handler pending, so it is picked up
    /// on re-entry rather than lost.
    pub(crate) fn take_events(&self) -> Events {
        let r = self.regs();
        let mis = r.cpu_int(0).mis().read();

        // MIS and ICLR have the same layout, so writing back what was read clears exactly the bits
        // this reports.
        r.cpu_int(0).iclr().write_value(mis);

        mis
    }

    /// Advance `period` if the counter says it should, and arm a deferred compare that is now near.
    pub(crate) fn next_period(&self, cs: CriticalSection) {
        let r = self.regs();

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
        let counter: u32 = W::<T>::from_reg(r.counterregs(0).ctr().read()).into();
        let period = self.period.load(Ordering::Relaxed);

        // We only modify the period from the timer interrupt, so we know this can't race.
        if (period & 1) == (counter >> Self::HALF_BITS) {
            return;
        }

        let period = period + 1;
        self.period.store(period, Ordering::Relaxed);

        #[cfg(feature = "_probe")]
        crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverPeriod));

        let t = (period as u64) << Self::HALF_BITS;

        let arming = self.compare_at(cs) < t + Self::ARM_AHEAD;

        #[cfg(feature = "_probe")]
        if arming {
            crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverArm));
        }

        if arming {
            // Just unmask it: `set_compare` has already written CC1.
            r.cpu_int(0).imask().modify(|w| w.set_ccu(1, true));
        }
    }

    /// The timestamp the compare is armed for, `u64::MAX` when it is disarmed.
    ///
    /// Held inverted in the cell so that disarmed is zero and the whole of an owning static is
    /// zero-initialised. One non-zero field would put all of it in `.data`, which is flash-resident on
    /// this target, so it would cost its own size in flash on top of the RAM it already takes.
    pub(crate) fn compare_at(&self, cs: CriticalSection) -> u64 {
        !self.compare.borrow(cs).get()
    }

    /// Record the armed timestamp. `u64::MAX` is disarmed. See [`PeriodCounter::compare_at`].
    fn set_compare_at(&self, cs: CriticalSection, timestamp: u64) {
        self.compare.borrow(cs).set(!timestamp);
    }

    /// Arm the compare for `timestamp`, answering `false` if it has already passed.
    pub(crate) fn set_compare(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let r = self.regs();

        self.set_compare_at(cs, timestamp);

        let t = self.now();

        if timestamp <= t {
            // If the timestamp has passed the compare will not fire.
            // Disarm it and return `false` to indicate that.
            r.cpu_int(0).imask().modify(|w| w.set_ccu(1, false));

            self.set_compare_at(cs, u64::MAX);

            return false;
        }

        // Write the CC1 value regardless of whether we're going to enable it now or not.
        // This way, when we enable it later, the right value is already set.
        //
        // Narrowed to the counter's width, so the compare is a value the counter actually reaches.
        r.counterregs(0)
            .cc(Channel::Ch1.index())
            .write_value(W::<T>::from_reg(timestamp as u32).into());

        // Enable it if it'll happen soon. Otherwise, `next_period` will enable it.
        let diff = timestamp - t;

        #[cfg(feature = "_probe")]
        if diff < Self::ARM_AHEAD {
            crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverSetAlarm));
        }

        r.cpu_int(0).imask().modify(|w| w.set_ccu(1, diff < Self::ARM_AHEAD));

        // Reevaluate if the timestamp is still in the future
        let t = self.now();
        if timestamp <= t {
            // If it has passed since we set it, we have a race condition and the compare may or may
            // not have fired. Disarm it and return `false` to indicate that.
            // It is the caller's responsibility to handle this ambiguity.
            r.cpu_int(0).imask().modify(|w| w.set_ccu(1, false));

            self.set_compare_at(cs, u64::MAX);

            return false;
        }

        // We're confident it will fire in the future.
        true
    }

    /// Whether this counter leaves the core asleep for at least `ticks`.
    ///
    /// Both the armed compare and the `period` tick count. Nobody asks for the tick, but `now()` depends
    /// on it, so its interrupt is never masked and it cuts short any sleep entered just before one.
    ///
    /// Asked as a predicate rather than as a distance: the caller only ever compares the answer against
    /// its minimum, and neither the saturating subtraction nor the minimum of the two wakes has to be
    /// evaluated to decide that.
    ///
    /// **A `ticks` of zero answers `false` while a compare is overdue, and that is deliberate.** Asking
    /// the distance instead would have called an overdue compare zero ticks away and let the sleep
    /// through, which is the one case where the two phrasings disagree. Declining is what is wanted: work
    /// already due is work the core should be doing rather than sleeping on.
    #[cfg(all(feature = "low-power", feature = "_time-driver"))]
    pub(crate) fn wake_at_least(&self, cs: CriticalSection, ticks: u32) -> bool {
        let now = self.now();

        // The low `HALF_BITS` of `now` are the position within the current period, so what is left of it
        // is the distance to the next tick, whether that comes from the overflow or the half-range compare.
        let period_end = (1 << Self::HALF_BITS) - (now & ((1 << Self::HALF_BITS) - 1));
        let ticks = ticks as u64;

        // A disarmed compare reads `u64::MAX`, which is further off than any `ticks`.
        period_end >= ticks && self.compare_at(cs) >= now.saturating_add(ticks)
    }
}
