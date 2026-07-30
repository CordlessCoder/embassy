use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use mspm0_metapac::interrupt;
use mspm0_metapac::tim::Tim;

use crate::interrupt::typelevel::Interrupt;
use crate::tim::low_level::{self, CounterOnEnable};
use crate::tim::{ClockSel, CountingMode, General2ChannelInstance, SealedInstance};
use crate::{peripherals, tim};

#[cfg(any(time_driver_timg12, time_driver_timg13))]
compile_error!("TIMG12 and TIMG13 are not supported by the time driver yet");

// Currently TIMG12 and TIMG13 are excluded because those are 32-bit timers.
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
#[cfg(time_driver_timg14)]
type T = peripherals::TIMG14;
#[cfg(time_driver_tima0)]
type T = peripherals::TIMA0;
#[cfg(time_driver_tima1)]
type T = peripherals::TIMA1;

// The scheme needs two capture/compare channels: one for the half-period tick, one for the alarm.
const _: fn() = || {
    fn has_two_channels<C: General2ChannelInstance>() {}
    has_two_channels::<T>();
};

fn regs() -> Tim {
    T::info().regs
}

// Clock timekeeping works with something we call "periods", which are time intervals
// of 2^15 ticks. The Clock counter value is 16 bits, so one "overflow cycle" is 2 periods.
//
// A `period` count is maintained in parallel to the Timer hardware `counter`, like this:
// - `period` and `counter` start at 0
// - `period` is incremented on overflow (at counter value 0)
// - `period` is incremented "midway" between overflows (at counter value 0x8000)
//
// When `period` is even, counter is in 0..0x7FFF. When odd, counter is in 0x8000..0xFFFF
// This allows for now() to return the correct value even if it races an overflow.
//
// `period` is a 32bit integer, so It overflows on 2^32 * 2^15 / 32768 seconds of uptime, which is 136 years.
fn calc_now(period: u32, counter: u16) -> u64 {
    ((period as u64) << 15) + ((counter as u32 ^ ((period & 1) << 15)) as u64)
}

/// TODO: Configurable tick rate
/// TODO: Compensate for per part variance. This can supposedly be done with the FCC system.
/// TODO: Allow using 32-bit timers (TIMG12 and TIMG13).
struct TimxDriver {
    /// Number of 2^15 periods elapsed since boot.
    period: AtomicU32,
    /// Timestamp at which to fire alarm. u64::MAX if no alarm is scheduled.
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
            // The counter is preloaded below; enabling must not reset it.
            counter_on_enable: CounterOnEnable::Preserve,
            free_run_in_debug: true,
        });

        // STANDBY1 unclocks all of PD0 apart from a handful of timers named per chip, and PD1 goes down in
        // every deep-sleep mode. A timer that stops in a mode cannot keep time through it, so forbid that
        // mode instead of stopping the clock: the guard is deliberately never dropped, because the time
        // driver never goes away.
        //
        // The cost is not uniform, which is why this is a silent trade rather than an error. A PD0 timer on
        // LFCLK that merely is not in the STANDBY1 list loses only that one mode and keeps STANDBY0. A PD1
        // timer blocks all deep sleep, which is self-defeating in a low-power build — `build.rs` warns
        // about that case at compile time, and `time-driver-any` avoids it where it can.
        //
        // `None` here means the timer survives everything and nothing is blocked, which is the case
        // `time-driver-any` selects for.
        if let Some(guard) = low_level::wake_guard::<T>(ClockSel::LfClk) {
            core::mem::forget(guard);
        }

        let regs = regs();

        // Middle
        regs.counterregs(0).cc(0).write_value(0x8000 as u32);
        // Start with the counter at 1 to avoid immediately incrementing period.
        regs.counterregs(0).ctr().write_value(1);

        regs.cpu_int(0).imask().modify(|w| {
            w.set_z(true);
            w.set_ccu0(true);
        });

        // Allow the counter to start counting.
        regs.counterregs(0).ctrctl().modify(|w| {
            w.set_en(true);
        });

        <T as tim::Instance>::Interrupt::IRQ.unpend();
        unsafe { <T as tim::Instance>::Interrupt::IRQ.enable() };
    }

    fn next_period(&self, cs: CriticalSection) {
        let r = regs();

        // We only modify the period from the timer interrupt, so we know this can't race.
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);
        let t = (period as u64) << 15;

        r.cpu_int(0).imask().modify(move |w| {
            let alarm = self.alarm.borrow(cs);
            let at = alarm.get();

            if at < t + 0xC000 {
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

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let r = regs();

        self.alarm.borrow(cs).set(timestamp);

        let t = self.now();

        if timestamp <= t {
            // If alarm timestamp has passed the alarm will not fire.
            // Disarm the alarm and return `false` to indicate that.
            r.cpu_int(0).imask().modify(|w| w.set_ccu1(false));

            self.alarm.borrow(cs).set(u64::MAX);

            return false;
        }

        // Write the CC1 value regardless of whether we're going to enable it now or not.
        // This way, when we enable it later, the right value is already set.
        //
        // Cast to u16 and then u32 to clamp to 16-bit timer limits.
        r.counterregs(0).cc(1).write_value(timestamp as u16 as u32);

        // Enable it if it'll happen soon. Otherwise, `next_period` will enable it.
        let diff = timestamp - t;
        r.cpu_int(0).imask().modify(|w| w.set_ccu1(diff < 0xC000));

        // Reevaluate if the alarm timestamp is still in the future
        let t = self.now();
        if timestamp <= t {
            // If alarm timestamp has passed since we set it, we have a race condition and
            // the alarm may or may not have fired.
            // Disarm the alarm and return `false` to indicate that.
            // It is the caller's responsibility to handle this ambiguity.
            r.cpu_int(0).imask().modify(|w| w.set_ccu1(false));

            self.alarm.borrow(cs).set(u64::MAX);

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

            let counter = regs.counterregs(0).ctr().read() as u16;

            // Ensure the compiler does not read the period again before the counter.
            compiler_fence(Ordering::Acquire);
            let period2 = self.period.load(Ordering::Relaxed);

            if period != period2 {
                continue;
            }

            return calc_now(period, counter);
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
    alarm: Mutex::new(Cell::new(u64::MAX)),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs);
}

#[cfg(time_driver_timg0)]
#[interrupt]
fn TIMG0() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg1)]
#[interrupt]
fn TIMG1() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg2)]
#[interrupt]
fn TIMG2() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg3)]
#[interrupt]
fn TIMG3() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg4)]
#[interrupt]
fn TIMG4() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg5)]
#[interrupt]
fn TIMG5() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg6)]
#[interrupt]
fn TIMG6() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg7)]
#[interrupt]
fn TIMG7() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg8)]
#[interrupt]
fn TIMG8() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg9)]
#[interrupt]
fn TIMG9() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg10)]
#[interrupt]
fn TIMG10() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_timg11)]
#[interrupt]
fn TIMG11() {
    DRIVER.on_interrupt();
}

// TODO: TIMG12 and TIMG13

#[cfg(time_driver_timg14)]
#[interrupt]
fn TIMG14() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_tima0)]
#[interrupt]
fn TIMA0() {
    DRIVER.on_interrupt();
}

#[cfg(time_driver_tima1)]
#[interrupt]
fn TIMA1() {
    DRIVER.on_interrupt();
}
