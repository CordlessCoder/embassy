use core::cell::RefCell;
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
#[cfg(feature = "rt")]
use mspm0_metapac::interrupt;

use crate::interrupt::typelevel::Interrupt;
use crate::tim::ClockSel;
use crate::tim::period::PeriodCounter;
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

/// `embassy-time`'s clock, kept by a [`PeriodCounter`] and a queue of wakers.
///
/// The tick rate is fixed at build time by the `time-driver-*` features, and nothing corrects for
/// the oscillator's per-part error, so a long interval drifts by whatever the source is specified
/// at.
struct TimxDriver {
    counter: PeriodCounter<T>,
    queue: Mutex<RefCell<Queue>>,
}

impl TimxDriver {
    fn init(&'static self, cs: CriticalSection) {
        // LFCLK at 32.768 kHz, the only source available all the way down to STANDBY, and no division
        // needed to reach the tick rate.
        //
        // The guard is deliberately never dropped, because the time driver never goes away. `build.rs`
        // warns at compile time where the selected timer blocks all deep sleep, and `time-driver-any`
        // avoids that where it can.
        //
        // The lint below is reading the wrong build. Without `low-power` a `MaybeWakeGuard` is an empty
        // struct with no `Drop`, so forgetting one is correctly a no-op; with the feature it holds a real
        // guard and the forget is the point. Clippy sees only the first.
        #[allow(clippy::forget_non_drop)]
        core::mem::forget(self.counter.start(cs, ClockSel::LfClk));

        <T as tim::Instance>::Interrupt::IRQ.unpend();
        unsafe { <T as tim::Instance>::Interrupt::IRQ.enable() };
    }

    fn on_interrupt(&self) {
        #[cfg(feature = "_probe")]
        crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverIrqEntry));

        critical_section::with(|cs| {
            let mis = self.counter.take_events();

            // Overflow
            if mis.z() {
                self.counter.next_period(cs);
            }

            // Half overflow
            if mis.ccu(0) {
                self.counter.next_period(cs);
            }

            if mis.ccu(1) {
                #[cfg(feature = "_probe")]
                crate::probe::count(crate::probe::target(crate::probe::Marker::TimeDriverAlarm));

                self.trigger_alarm(cs);
            }
        });
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.counter.now());

        while !self.counter.set_compare(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.counter.now());
        }
    }
}

impl Driver for TimxDriver {
    fn now(&self) -> u64 {
        self.counter.now()
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
    counter: PeriodCounter::new(),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs);
}

/// Whether this driver leaves the core asleep for at least `ticks`.
///
/// See [`PeriodCounter::wake_at_least`], which this forwards to.
#[cfg(feature = "low-power")]
pub(crate) fn wake_at_least(cs: CriticalSection, ticks: u32) -> bool {
    DRIVER.counter.wake_at_least(cs, ticks)
}
