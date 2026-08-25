//! An [RTIC](https://rtic.rs) monotonic, on the timer core that backs the `embassy-time` driver.
//!
//! # Why this one rather than SysTick
//!
//! `rtic-monotonics`' SysTick backend is clocked from the core, so it stops dead in every mode that
//! unclocks it — which is every deep-sleep mode this part has. This one runs its timer from LFCLK at
//! 32.768 kHz and holds the same sleep guard the time driver does, so **a deadline survives STOP and
//! STANDBY**, and an application can sleep between them.
//!
//! **That is a property of the timer, not of the monotonic.** It holds only for an instance clocked in
//! STANDBY1; on any other, the guard is what stops the device reaching the modes that sentence
//! promises, and on a PD1 instance it blocks every deep-sleep level. The macro refuses one under
//! `low-power`, and `allow-rtic-monotonic-sleep-floor` is the way to say you meant it.
//!
//! # A vector someone else owns
//!
//! `start` plants this timer's `#[no_mangle]` handler, which collides with an RTIC
//! `#[task(binds = TIMGx)]` on the same timer. Write `unsafe` before the name and no handler is
//! emitted; call [`on_interrupt`](crate::rtic_monotonic) from the task instead.
//!
//! ```rust,ignore
//! embassy_mspm0::rtic_monotonic!(unsafe Mono, TIMG0);
//!
//! #[task(binds = TIMG0, priority = 2)]
//! fn timer(_: timer::Context) {
//!     unsafe { Mono::on_interrupt() }
//! }
//! ```
//!
//! # Use
//!
//! ```rust,ignore
//! embassy_mspm0::rtic_monotonic!(Mono, TIMG0);
//!
//! #[init]
//! fn init(_: init::Context) -> (Shared, Local) {
//!     let p = embassy_mspm0::init(Config::default());
//!
//!     Mono::start(p.TIMG0);
//!     // ...
//! }
//!
//! #[task]
//! async fn blink(_: blink::Context) {
//!     loop {
//!         Mono::delay(500.millis()).await;
//!     }
//! }
//! ```
//!
//! The timer is an argument rather than a cargo feature, so `Peripherals` decides what is available:
//! a `time-driver-*` build has already had its timer removed from it, and naming that one here does
//! not compile. Everything else does, so the two can coexist on separate timers.
//!
//! # Sleeping between deadlines
//!
//! [`idle`](crate::idle) reaches the deepest mode the held
//! [`WakeGuard`](crate::sysctl::WakeGuard)s allow, and the compare wakes the core from it. What a
//! monotonic build does *not* get is `Config::min_sleep`, the gate that keeps the chip in RUN when the
//! next wake is too close for a deep sleep to pay for itself: that reads the `embassy-time` driver's
//! queue, and a monotonic has its own. So a deadline a few ticks away still enters a deep mode and
//! comes straight back out, which costs a little more energy than staying awake would have.
//!
//! This is the same behaviour a build with no time driver has always had, rather than a regression,
//! and closing it means giving [`low_power`](crate::low_power) a way to ask a backend it cannot name.
//!
//! # The interrupt is left at its reset priority
//!
//! Which is 0, above every RTIC task. That is what a timekeeper wants, and it is sound here because
//! the handler takes no `#[shared]` resource. To place it in RTIC's scheme instead, bind the timer to
//! a hardware task with [`bind_interrupts!`](crate::bind_interrupts)'s `unsafe struct` arm and call
//! `Mono::on_interrupt` from it — RTIC's `pre_init` then sets the priority before `#[init]` runs.

use embassy_hal_internal::interrupt::InterruptExt;
use rtic_time::timer_queue::TimerQueueBackend;

use crate::interrupt::typelevel::Interrupt;
use crate::tim::period::PeriodCounter;
use crate::tim::{ClockSel, General2ChannelInstance, Instance};

/// The tick rate every monotonic here runs at, which is LFCLK undivided.
pub const TICK_HZ: u32 = 32_768;

/// Start `counter`'s timer and unmask its interrupt.
///
/// Held by the caller for the life of the program: the guard the counter returns is what stops the
/// device entering a mode the timer would not survive.
pub fn start<B: MonotonicBackend>() {
    critical_section::with(|cs| {
        // The lint is reading the wrong build. Without `low-power` a `MaybeWakeGuard` is an empty
        // struct with no `Drop`, so forgetting one is correctly a no-op; with the feature it holds a
        // real guard and the forget is the point.
        #[allow(clippy::forget_non_drop)]
        core::mem::forget(B::counter().start(cs, ClockSel::LfClk));

        <B::Timer as Instance>::Interrupt::IRQ.unpend();
        unsafe { <B::Timer as Instance>::Interrupt::IRQ.enable() };
    });
}

/// Arm the compare for `instant`.
///
/// The queue re-checks `now()` against it and calls again if it has already passed, so a compare that
/// cannot be armed only has to leave the timer alone.
pub fn set_compare<B: MonotonicBackend>(instant: u64) {
    critical_section::with(|cs| {
        B::counter().set_compare(cs, instant);
    });
}

/// Service the timer, then let the queue wake whatever is due.
///
/// The counter's own events come first: `now()` depends on them, and a queue given a timestamp taken
/// before the period was reconciled would be comparing against one half a counter range out. That is
/// what the section covers — the period and the compare are shared with tasks that update them under
/// one, so a task at a higher priority must not see either half-written. It is dropped before the
/// queue walk, which takes its own where it needs one and is far too long to hold interrupts off for.
///
/// **The queue is called whether or not the compare fired.** It pends this interrupt in software when
/// a new deadline becomes the head of the queue, which is how the first `delay` of a run gets its
/// compare armed; answering only to the hardware event would leave that deadline sitting in the queue
/// forever. The cost is a queue walk on each period tick as well, which is once a counter half-range.
///
/// # Safety
///
/// Call only from this timer's own interrupt handler.
pub unsafe fn on_interrupt<B: MonotonicBackend>() {
    let counter = B::counter();

    critical_section::with(|cs| {
        let mis = counter.take_events();

        // Overflow
        if mis.z() {
            counter.next_period(cs);
        }

        // Half overflow
        if mis.ccu(0) {
            counter.next_period(cs);
        }
    });

    // SAFETY: the caller's contract is this handler, which is what the queue asks for.
    unsafe { B::timer_queue().on_monotonic_interrupt() };
}

/// Whether the macro's sleep-floor check applies.
///
/// A `cfg!` inside `rtic_monotonic!` would be evaluated in the crate that calls it, where neither
/// feature exists, so the check would pass on every timer. It has to be answered here.
pub const CHECK_SLEEP_FLOOR: bool = cfg!(feature = "low-power") && !cfg!(feature = "allow-rtic-monotonic-sleep-floor");

/// Ties a generated backend to the timer it counts.
///
/// Implemented by the per-timer backends in [`crate::rtic_backend`]; nothing else should.
pub trait MonotonicBackend: TimerQueueBackend<Ticks = u64> {
    /// The timer this backend counts.
    type Timer: General2ChannelInstance;

    /// Its counter.
    fn counter() -> &'static PeriodCounter<Self::Timer>;
}

/// Define an RTIC monotonic on `$timer`.
///
/// See the [module docs](crate::rtic_monotonic) for what it is for and how it behaves across sleep.
#[macro_export]
macro_rules! rtic_monotonic {
    (unsafe $name:ident, $timer:ident) => {
        $crate::rtic_monotonic!(@common $name, $timer);

        impl $name {
            /// Start the monotonic, without planting the timer's interrupt handler.
            ///
            /// Call once, after `embassy_mspm0::init`. Whoever owns the vector table has to route
            /// this timer's interrupt to [`on_interrupt`](Self::on_interrupt).
            pub fn start(_timer: $crate::Peri<'static, $crate::peripherals::$timer>) {
                $crate::rtic_monotonic!(@init $name, $timer);
            }
        }
    };

    ($name:ident, $timer:ident) => {
        $crate::rtic_monotonic!(@common $name, $timer);

        impl $name {
            /// Start the monotonic. Call once, after `embassy_mspm0::init`, which programs the clock
            /// tree this reads.
            pub fn start(_timer: $crate::Peri<'static, $crate::peripherals::$timer>) {
                #[allow(non_snake_case)]
                #[unsafe(no_mangle)]
                unsafe extern "C" fn $timer() {
                    unsafe { $name::on_interrupt() }
                }

                $crate::rtic_monotonic!(@init $name, $timer);
            }
        }
    };

    (@init $name:ident, $timer:ident) => {
        $crate::rtic_time::timer_queue::TimerQueue::initialize(
            <$crate::rtic_backend::$timer as $crate::rtic_time::timer_queue::TimerQueueBackend>::timer_queue(),
            $crate::rtic_backend::$timer,
        );

        $crate::rtic_monotonic::start::<$crate::rtic_backend::$timer>();
    };

    (@common $name:ident, $timer:ident) => {
        /// An RTIC monotonic, ticking at 32.768 kHz and surviving deep sleep.
        pub struct $name;

        // The guard this holds for the life of the program comes from the timer, and on an instance
        // that is not clocked in STANDBY1 it is what blocks the sleep the module doc promises. A PD1
        // instance blocks every level. The choice is a call-site constant, so it is answerable here.
        const _: () = ::core::assert!(
            !$crate::rtic_monotonic::CHECK_SLEEP_FLOOR
                || ::core::matches!(
                    <$crate::peripherals::$timer as $crate::sysctl::LowPowerInstance>::SLEEP.clocked_in_standby1,
                    Some(true)
                ),
            "this timer is not clocked in STANDBY1, so the monotonic holds the device out of deep \
             sleep for the life of the program. Pick one that is, or enable the \
             `allow-rtic-monotonic-sleep-floor` feature."
        );

        impl $name {
            /// Service the timer.
            ///
            /// Called by the handler `start` plants. Public so that an application binding the timer
            /// to a hardware task of its own can call it instead.
            ///
            /// # Safety
            ///
            /// Call only from this timer's own interrupt handler.
            pub unsafe fn on_interrupt() {
                unsafe { $crate::rtic_monotonic::on_interrupt::<$crate::rtic_backend::$timer>() }
            }
        }

        impl $crate::rtic_time::monotonic::TimerQueueBasedMonotonic for $name {
            type Backend = $crate::rtic_backend::$timer;
            type Instant = $crate::fugit::Instant<u64, 1, { $crate::rtic_monotonic::TICK_HZ }>;
            type Duration = $crate::fugit::Duration<u64, 1, { $crate::rtic_monotonic::TICK_HZ }>;
        }

        $crate::rtic_time::impl_embedded_hal_delay_fugit!($name);
        $crate::rtic_time::impl_embedded_hal_async_delay_fugit!($name);
    };
}
