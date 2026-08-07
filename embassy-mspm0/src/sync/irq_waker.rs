//! A single waiter, woken from an interrupt handler without taking a lock.
//!
//! One slot per thing that can be waited on — a buffered UART's receive side, its transmit side. Where
//! [`linked_waiter`](super::linked_waiter) exists because a source has several waits outstanding and has
//! to tell them apart, this exists for the opposite case: a driver whose waiting future is reached
//! through `&mut self`, so there is at most one, and a list would be a walk over a single node.
//!
//! What it buys over `embassy_sync`'s `AtomicWaker` is the wake path, and only that. On a core without
//! CAS that resolves to the critical-section variant, whose `wake` masks interrupts to take the waker
//! out, wake it by reference and put it back. The taking is the only reason it needs a section; reading
//! in place needs none. Everything else — one waker, kept registered until it is replaced, woken on
//! every call — is deliberately identical, because the drivers depend on it: they test for work and then
//! register without testing again, and a waker that stays registered is what covers the gap between.
//!
//! # Who may touch one, and when
//!
//! The rules are [`linked_waiter`](super::linked_waiter)'s, with the list-shaped ones dropped:
//!
//! 1. **Tasks write only with interrupts off.** [`Self::register`] does every write inside one critical
//!    section, so a handler sees the state wholly before or wholly after it, never part-way.
//! 2. **The waker side only reads.** [`Self::wake`] reads the waker and wakes it by reference; it never
//!    writes one. This is what lets a task's fast-path comparison in [`Self::register`] run outside a
//!    section: the only writer is that same task.
//! 3. **One waker side.** Exactly one context — in practice one interrupt handler — may call
//!    [`Self::wake`], and it must not be able to preempt itself. Not expressible in the type system: the
//!    handler holds no token, and taking one there would only mask interrupts that are already masked.
//!
//! # Single core only
//!
//! [`Self::wake`] reads a non-atomic `Waker` with no token and no lock. That is sound because it runs in
//! an interrupt handler on the same core as the task that writes it, so a task inside a critical section
//! is a task that is not running. On a second core it would be running, and the read could overlap the
//! write — a data race whatever the `critical-section` implementation does. Every MSPM0 is a single-core
//! Cortex-M0+, which is the assumption this is built on rather than one it can check.
//!
//! # What it does not change
//!
//! A registered waker is woken on every call, whether or not anyone is waiting on it just then. That is
//! the behaviour it replaces, and it is load-bearing rather than incidental: it is what makes the window
//! between a driver testing for work and registering harmless. Waking only once per registration would
//! close that cover, and a byte arriving in the window would leave the task asleep with data already in
//! the buffer — invisible to a throughput test and a hang at the end of a message.

use core::cell::Cell;
use core::task::Waker;

/// Somewhere for one task to wait, and for one interrupt handler to wake it.
///
/// One `Option<Waker>` wide, the same as what it replaces: there is no state beyond the waker, because
/// the critical section it drops was never a lock with a representation — it is the section itself.
pub(crate) struct IrqWaker {
    /// Whom to wake.
    ///
    /// A `Waker` is two words, so the waker side could read it half-written if a task ever wrote it with
    /// interrupts on. It does not — see rule 1.
    waker: Cell<Option<Waker>>,
}

/// SAFETY: every write happens with interrupts off, so no sharer can observe a partial one, and the only
/// reader that runs without a token is an interrupt handler on the same core. See the module docs.
unsafe impl Sync for IrqWaker {}

impl IrqWaker {
    pub(crate) const fn new() -> Self {
        Self { waker: Cell::new(None) }
    }

    /// Arm from a poll, where interrupts are on.
    ///
    /// The waker a future is polled with almost never changes, and the waker side leaves it in place, so
    /// the common path clones nothing.
    pub(crate) fn register(&self, waker: &Waker) {
        // SAFETY: a task is the only writer and holds `&mut` the driver, so nothing can be changing this
        // under the read. The handler only reads it too.
        let stored = unsafe { &*self.waker.as_ptr() };
        let parked = match stored {
            Some(stored) if stored.will_wake(waker) => None,
            // Cloned before the section: a `Waker`'s clone is someone else's code and has no business
            // running with interrupts off.
            _ => Some(waker.clone()),
        };

        // The displaced waker is bound out here because its drop is someone else's code as well.
        let _displaced = critical_section::with(|_cs| parked.and_then(|parked| self.waker.replace(Some(parked))));
    }

    /// Wake whoever is registered, from the interrupt, with no lock.
    ///
    /// Reading the waker rather than taking it is the whole difference: a take is a write, and a write
    /// from here is what would need a section.
    pub(crate) fn wake(&self) {
        // SAFETY: written only by a task, and only with interrupts off, so this cannot land on a
        // half-written `Waker`.
        if let Some(waker) = unsafe { &*self.waker.as_ptr() } {
            waker.wake_by_ref();
        }
    }
}
