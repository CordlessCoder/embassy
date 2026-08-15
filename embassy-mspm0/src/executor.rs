//! MSPM0-specific `embassy-executor` platform.
//!
//! This module provides an `embassy-executor` platform specific for MSPM0 chips, which idles into a
//! sleep rather than spinning.
//! Read the `embassy-executor` README for information about what "executor platforms" are and how they work.
//!
//! # What the idle does, and why it is not `embassy-executor`'s own
//!
//! With `low-power` it enters the deepest sleep the [`WakeGuard`](crate::sysctl::WakeGuard)s allow.
//! Without it there are no guards to consult and no mode to pick, so the idle is a plain `WFI` —
//! **but still one with the prefetcher suspended across it**, which is the reason to prefer this
//! executor over `embassy-executor`'s own even when deep sleep is not wanted.
//!
//! `CPU_ERR_03` applies to every family this crate supports and is written against low-power modes
//! rather than the deep ones, so a shallow idle is in scope. `embassy-executor`'s Cortex-M executor
//! idles on `WFE` and takes no such guard. Measured on one application, the difference between the
//! two idles is **8 bytes of flash**; deep sleep on top of that is another 140.
//!
//! To use it:
//! - Enable the `executor-thread` and/or `executor-interrupt` feature on this crate.
//! - Add `low-power` as well if the idle should reach a deep-sleep mode. It is no longer implied.
//! - **Do not** enable features `platform-cortex-m`, `executor-thread` or `executor-interrupt` in the `embassy-executor` crate.
//! - Tell the `main` macro to use this executor like this:
//!
//! ```rust,no_run
//! #[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
//! async fn main(spawner: Spawner) {
//!     let p = embassy_mspm0::init(Config::default());
//!     // ...
//! }
//! ```

#[unsafe(export_name = "__pender")]
#[cfg(any(feature = "executor-thread", feature = "executor-interrupt"))]
fn __pender(context: *mut ()) {
    // `context` is either `THREAD_PENDER`, or an interrupt number passed to `InterruptExecutor::start`.
    let context = context as usize;

    #[cfg(feature = "executor-thread")]
    // Try to optimize away the branch when only thread mode is enabled.
    if !cfg!(feature = "executor-interrupt") || context == thread::THREAD_PENDER {
        thread::SIGNAL_WORK_THREAD_MODE.store(true, core::sync::atomic::Ordering::Relaxed);
        return;
    }

    #[cfg(feature = "executor-interrupt")]
    {
        use cortex_m::peripheral::NVIC;

        // MSPM0 is Cortex-M0+, which has no STIR, and implements 32 interrupts — so ISPR is a
        // single word and the index is a constant. `NVIC::pend` derives it from the number
        // instead, leaving a bounds check the optimiser cannot fold.
        //
        // SAFETY: `context` was an `InterruptNumber` when passed to `InterruptExecutor::start`, so
        // it names a line this core implements and the mask below does not change it.
        unsafe { (*NVIC::PTR).ispr[0].write(1 << (context & 31)) };
    }
}

#[cfg(feature = "executor-thread")]
pub use thread::*;
#[cfg(feature = "executor-thread")]
mod thread {
    use core::marker::PhantomData;
    use core::sync::atomic::{AtomicBool, Ordering};

    use embassy_executor::{Spawner, raw};

    pub(super) const THREAD_PENDER: usize = usize::MAX;

    /// Set by the pender to signal pending work; checked before sleeping since `WFI` ignores `SEV`.
    pub(crate) static SIGNAL_WORK_THREAD_MODE: AtomicBool = AtomicBool::new(false);

    /// Thread-mode executor that sleeps on idle.
    ///
    /// It runs on thread mode, at the lowest priority level, and sleeps when it has no more work to
    /// do. With `low-power`, how deep that sleep goes is decided by the
    /// [`WakeGuard`](crate::sysctl::WakeGuard)s the drivers hold and, with a time driver, by
    /// `Config::min_sleep`; with nothing to block it the chip reaches its deepest allowed level rather
    /// than plain `WFI`. Without `low-power` it is a plain `WFI`, with the prefetcher suspended across
    /// it either way — see this module's own docs for why that matters.
    ///
    /// The sleep is entered with interrupts masked, so a task woken between the poll and the sleep
    /// would otherwise be missed. `WFI` has no event register for a `SEV` to latch into, so the
    /// pender instead sets a flag that the executor checks inside the same critical section.
    pub struct Executor {
        inner: raw::Executor,
        not_send: PhantomData<*mut ()>,
    }

    impl Executor {
        /// Create a new Executor.
        pub fn new() -> Self {
            Self {
                inner: raw::Executor::new(THREAD_PENDER as *mut ()),
                not_send: PhantomData,
            }
        }

        /// Run the executor.
        ///
        /// The `init` closure is called with a [`Spawner`] that spawns tasks on
        /// this executor. Use it to spawn the initial task(s). After `init` returns,
        /// the executor starts running the tasks.
        ///
        /// To spawn more tasks later, you may keep copies of the [`Spawner`] (it is `Copy`),
        /// for example by passing it as an argument to the initial tasks.
        ///
        /// This function requires `&'static mut self`. This means you have to store the
        /// Executor instance in a place where it'll live forever and grants you mutable
        /// access. There's a few ways to do this:
        ///
        /// - a [StaticCell](https://docs.rs/static_cell/latest/static_cell/) (safe)
        /// - a `static mut` (unsafe)
        /// - a local variable in a function you know never returns (like `fn main() -> !`), upgrading its lifetime with `transmute`. (unsafe)
        ///
        /// This function never returns.
        pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
            init(self.inner.spawner());

            loop {
                unsafe {
                    // A woken task's own pin moves inside this bracket, which is what separates "getting
                    // back to the executor" from "the executor running the task".
                    #[cfg(feature = "_probe")]
                    let poll_marker = crate::probe::target(crate::probe::Marker::ExecutorPoll);
                    #[cfg(feature = "_probe")]
                    crate::probe::set(poll_marker);

                    self.inner.poll();

                    #[cfg(feature = "_probe")]
                    crate::probe::clear(poll_marker);

                    critical_section::with(|cs| {
                        // `Relaxed` is enough on both sides. Every MSPM0 is a single core, so the only
                        // thing that races the executor is an interrupt on the same core, and this check
                        // runs with them masked — the section is what orders it against the pender, and
                        // its acquire is the compiler barrier that stops the load being hoisted out.
                        // Anything stronger only buys `dmb`s against observers that do not exist.
                        if SIGNAL_WORK_THREAD_MODE.load(Ordering::Relaxed) {
                            SIGNAL_WORK_THREAD_MODE.store(false, Ordering::Relaxed);
                        } else {
                            #[cfg(feature = "low-power")]
                            crate::low_power::sleep(cs);

                            // Without `low-power` there are no sleep guards to consult and no mode to
                            // pick, so the idle is a plain `WFI` -- but still a guarded one, because
                            // `CPU_ERR_03` covers a shallow sleep as well.
                            #[cfg(not(feature = "low-power"))]
                            {
                                let _ = cs;
                                crate::prefetch::guarded_wfi();
                            }
                        }
                    });
                }
            }
        }
    }

    impl Default for Executor {
        fn default() -> Self {
            Self::new()
        }
    }
}

#[cfg(feature = "executor-interrupt")]
pub use interrupt::*;
#[cfg(feature = "executor-interrupt")]
mod interrupt {
    use core::cell::{Cell, UnsafeCell};
    use core::mem::MaybeUninit;

    use cortex_m::interrupt::InterruptNumber;
    use cortex_m::peripheral::NVIC;
    use critical_section::Mutex;
    use embassy_executor::raw;

    /// Interrupt-mode executor.
    ///
    /// This executor runs tasks in interrupt mode. The interrupt handler is set up
    /// to poll tasks, and when a task is woken the interrupt is pended from software.
    ///
    /// This allows running async tasks at a priority higher than thread mode. One
    /// use case is to leave thread mode free for non-async tasks. Another use case is
    /// to run multiple executors: one in thread mode for low priority tasks and another in
    /// interrupt mode for higher priority tasks. Higher priority tasks will preempt lower
    /// priority ones.
    ///
    /// It is even possible to run multiple interrupt mode executors at different priorities,
    /// by assigning different priorities to the interrupts.
    ///
    /// To use it, you have to pick an interrupt that won't be used by the hardware.
    /// MSPM0 has no dedicated software interrupt, so use the interrupt of a peripheral the
    /// application leaves unused.
    ///
    /// It is somewhat more complex to use, it's recommended to use the thread-mode
    /// `Executor` instead, if it works for your use case.
    pub struct InterruptExecutor {
        started: Mutex<Cell<bool>>,
        executor: UnsafeCell<MaybeUninit<raw::Executor>>,
    }

    unsafe impl Send for InterruptExecutor {}
    unsafe impl Sync for InterruptExecutor {}

    impl InterruptExecutor {
        /// Create a new, not started `InterruptExecutor`.
        #[inline]
        pub const fn new() -> Self {
            Self {
                started: Mutex::new(Cell::new(false)),
                executor: UnsafeCell::new(MaybeUninit::uninit()),
            }
        }

        /// Executor interrupt callback.
        ///
        /// # Safety
        ///
        /// - You MUST call this from the interrupt handler, and from nowhere else.
        /// - You must not call this before calling `start()`.
        pub unsafe fn on_interrupt(&'static self) {
            let executor = unsafe { (&*self.executor.get()).assume_init_ref() };
            executor.poll();
        }

        /// Start the executor.
        ///
        /// This initializes the executor, enables the interrupt, and returns.
        /// The executor keeps running in the background through the interrupt.
        ///
        /// This returns a [`SendSpawner`] you can use to spawn tasks on it. A [`SendSpawner`]
        /// is returned instead of a [`Spawner`](embassy_executor::Spawner) because the executor effectively runs in a
        /// different "thread" (the interrupt), so spawning tasks on it is effectively
        /// sending them.
        ///
        /// To obtain a [`Spawner`](embassy_executor::Spawner) for this executor, use [`Spawner::for_current_executor()`](embassy_executor::Spawner::for_current_executor()) from
        /// a task running in it.
        ///
        /// # Interrupt requirements
        ///
        /// You must write the interrupt handler yourself, and make it call [`on_interrupt()`](Self::on_interrupt).
        ///
        /// This method already enables (unmasks) the interrupt, you must NOT do it yourself.
        ///
        /// You must set the interrupt priority before calling this method. You MUST NOT
        /// do it after.
        ///
        /// [`SendSpawner`]: embassy_executor::SendSpawner
        pub fn start(&'static self, irq: impl InterruptNumber) -> embassy_executor::SendSpawner {
            if critical_section::with(|cs| self.started.borrow(cs).replace(true)) {
                panic!("InterruptExecutor::start() called multiple times on the same executor.");
            }

            unsafe {
                (&mut *self.executor.get())
                    .as_mut_ptr()
                    .write(raw::Executor::new(irq.number() as *mut ()))
            }

            let executor = unsafe { (&*self.executor.get()).assume_init_ref() };

            unsafe { NVIC::unmask(irq) }

            executor.spawner().make_send()
        }

        /// Get a SendSpawner for this executor
        ///
        /// This returns a [`SendSpawner`](embassy_executor::SendSpawner) you can use to spawn tasks on this
        /// executor.
        ///
        /// This MUST only be called on an executor that has already been started.
        /// The function will panic otherwise.
        pub fn spawner(&'static self) -> embassy_executor::SendSpawner {
            if !critical_section::with(|cs| self.started.borrow(cs).get()) {
                panic!("InterruptExecutor::spawner() called on uninitialized executor.");
            }
            let executor = unsafe { (&*self.executor.get()).assume_init_ref() };
            executor.spawner().make_send()
        }
    }

    impl Default for InterruptExecutor {
        fn default() -> Self {
            Self::new()
        }
    }
}
