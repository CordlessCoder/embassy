//! An intrusive list of tasks waiting to be woken, whose nodes live in the futures that wait.
//!
//! One list per source of wakes — a GPIO port, a peripheral instance — holding a [`Waiter`] per
//! outstanding wait. The node carries whatever the waker side needs to tell one wait from another, so a
//! handler with several waits pending finds the right one by walking the list rather than by indexing a
//! table sized for every possible waiter.
//!
//! Costs one pointer per list and one node per wait, and the node is a local of the waiting future, so a
//! driver pays for waits that exist rather than for waits it could have.
//!
//! # Who may touch a list, and when
//!
//! No lock, which is what makes a wake cheap. Soundness comes from the access rules instead. Two of the
//! four are checked for you; the other two are what the `unsafe` on [`WaiterList::link`] and
//! [`WaiterList::find`] is about:
//!
//! 1. **Tasks mutate only with interrupts off.** Every method that edits a list or a waker takes a
//!    `CriticalSection`, so this one cannot be got wrong — a caller with no token cannot call them.
//! 2. **The waker side only reads.** It clears [`Waiter::outstanding`], which is atomic for that reason,
//!    and reads the waker; it never edits a link. Enforced by [`WaiterList::find`] handing out `&Waiter`,
//!    which reaches nothing that writes a link.
//! 3. **One waker side per list.** Exactly one context — in practice one interrupt handler — may call
//!    [`WaiterList::find`], and it must not be able to preempt itself. Two lists may be walked at once.
//!    Not expressible in the type system: the handler holds no token, and taking one there would only
//!    mask interrupts that are already masked.
//! 4. **A node is unlinked before it is dropped**, and does not move while linked. The future that owns
//!    the node is what guarantees this, by unlinking in its `Drop`. This is what makes every *other*
//!    node in a list safe to dereference, so it is the promise [`WaiterList::link`] asks for.
//!
//! # Single core only
//!
//! The waker side holds nothing at all: [`WaiterList::find`] walks a list, and [`Waiter::complete`]
//! reads a node's non-atomic waker, with no token and no lock. What makes that sound is that they run in
//! an interrupt handler on the same core as every task that edits the list, so a task inside a critical
//! section is a task that is not running.
//!
//! On a second core it *would* be running, and the token it holds would say nothing about this core's
//! handler — a walk and an edit could overlap, which is a data race on the `Cell` fields whatever the
//! `critical-section` implementation does. Every MSPM0 is a single-core Cortex-M0+, which is the
//! assumption this module is built on rather than one it can check.

use core::cell::Cell;
use core::ptr::{self, NonNull};
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
use core::task::Waker;

use critical_section::CriticalSection;

/// One wait, linked into the list of whatever will wake it.
///
/// `T` is what the waker side matches on — a pin number and the edges it accepts, a channel, a request
/// number. It is read by the waker side with no lock, so it is written once at construction and never
/// again.
///
/// Only [`Waiter::outstanding`] is atomic, because it is the one field both sides write.
pub(crate) struct Waiter<T> {
    /// The next waiter on this list, or `None` at the end.
    next: Cell<Option<NonNull<Waiter<T>>>>,

    /// Cleared by the waker side when the thing happens. This, not anything the hardware latches, is
    /// what completion means: a status bit says what arrived, not who was waiting for it.
    outstanding: AtomicBool,

    /// Whom to wake.
    ///
    /// A `Waker` is two words, so the waker side could read it half-written if a task ever wrote it with
    /// interrupts on. It does not: both writers take a critical section.
    waker: Cell<Option<Waker>>,

    /// What this wait is for, in whatever terms the waker side matches on.
    pub(crate) state: T,
}

/// SAFETY: the pointers a `Waiter` holds only mean anything while it is linked, and linking happens after
/// the node has been pinned — so any move of one has already happened by then, and a node that can still
/// be moved is one nothing else can reach.
unsafe impl<T: Send> Send for Waiter<T> {}

/// SAFETY: as above, plus every mutation is made with interrupts off, so no sharer can see a partial one.
unsafe impl<T: Sync> Sync for Waiter<T> {}

impl<T> Waiter<T> {
    /// A waiter that is outstanding from the moment it exists, with nobody to wake yet.
    pub(crate) const fn new(state: T) -> Self {
        Self {
            next: Cell::new(None),
            outstanding: AtomicBool::new(true),
            waker: Cell::new(None),
            state,
        }
    }

    /// Whether the wait is still outstanding, which is the completion test a future polls.
    pub(crate) fn is_outstanding(&self) -> bool {
        self.outstanding.load(Ordering::Relaxed)
    }

    /// Store `waker`, handing back whatever it displaced.
    ///
    /// The old value is returned rather than dropped here because a `Waker`'s drop is someone else's
    /// code, and it has no business running with interrupts off. Clone the waker before the section for
    /// the same reason.
    #[must_use = "the displaced waker has to be dropped outside the critical section"]
    pub(crate) fn store_waker(&self, waker: Waker, _cs: CriticalSection<'_>) -> Option<Waker> {
        self.waker.replace(Some(waker))
    }

    /// Store `waker` from a poll, where interrupts are on.
    ///
    /// The waker a future is polled with almost never changes, so the common path is a comparison and no
    /// write at all — which is the only reason this can afford to be on the completing poll's path.
    pub(crate) fn register(&self, waker: &Waker) {
        // SAFETY: this task is the only writer, so nothing can be changing it under this read.
        if let Some(stored) = unsafe { &*self.waker.as_ptr() }
            && stored.will_wake(waker)
        {
            return;
        }

        // Cloned before the section: a `Waker`'s clone is someone else's code and has no business
        // running with interrupts off. Its drop has none either, so the displaced one is bound rather
        // than discarded and goes out of scope out here.
        let parked = waker.clone();

        let _displaced = critical_section::with(|cs| self.store_waker(parked, cs));
    }

    /// Mark the wait done and wake whoever is on it.
    ///
    /// Called from the waker side with no lock. Reading the waker rather than taking it is what makes
    /// that sound — the task is then the only writer of the field.
    pub(crate) fn complete(&self) {
        self.outstanding.store(false, Ordering::Relaxed);

        // SAFETY: written only by the owning task, and only with interrupts off.
        if let Some(waker) = unsafe { &*self.waker.as_ptr() } {
            waker.wake_by_ref();
        }
    }
}

/// The waiters on one source of wakes, newest first.
///
/// One pointer, not one slot per possible waiter: the nodes live in the futures waiting on them, so this
/// costs RAM per list and stack per wait.
///
/// Only load and store are used, both of which a core without a compare-and-swap does natively, so the
/// list needs no lock of its own — see the module docs for who is allowed to touch it when.
pub(crate) struct WaiterList<T> {
    head: AtomicPtr<Waiter<T>>,
}

impl<T> WaiterList<T> {
    /// An empty list, `const` so it can be a `static`.
    pub(crate) const fn new() -> Self {
        Self {
            head: AtomicPtr::new(ptr::null_mut()),
        }
    }

    /// Push `waiter` onto the list.
    ///
    /// # Safety
    ///
    /// The node must stay where it is and be [`WaiterList::unlink`]ed before it is dropped. Every other
    /// method here dereferences the nodes it walks past, so this promise is what makes them safe.
    pub(crate) unsafe fn link(&self, waiter: &Waiter<T>, _cs: CriticalSection<'_>) {
        waiter.next.set(NonNull::new(self.head.load(Ordering::Relaxed)));
        // Relaxed, not Release: the token says interrupts are masked, so the waker side cannot observe
        // the list between the two stores, and leaving the critical section is itself a barrier. See
        // the module docs — the same single-core argument the non-atomic `Waker` read rests on.
        self.head.store(ptr::from_ref(waiter).cast_mut(), Ordering::Relaxed);
    }

    /// Take `waiter` back off the list, wherever in it the node happens to be.
    ///
    /// A search rather than a doubly-linked list: the length is the number of waits outstanding at once,
    /// which is one in the cases measured, and a `prev` pointer would cost every node a word and every
    /// link an extra store to save nothing at that length.
    ///
    /// Safe, unlike its opposite: the token rules out a concurrent editor, and every node it walks past
    /// is live because [`WaiterList::link`]'s caller promised to unlink before dropping.
    pub(crate) fn unlink(&self, waiter: &Waiter<T>, _cs: CriticalSection<'_>) {
        let me = NonNull::from(waiter);

        if self.head.load(Ordering::Relaxed) == me.as_ptr() {
            self.head.store(
                waiter.next.get().map_or(ptr::null_mut(), NonNull::as_ptr),
                Ordering::Relaxed,
            );

            return;
        }

        let mut node = NonNull::new(self.head.load(Ordering::Relaxed));

        while let Some(current) = node {
            // SAFETY: every node in the list is live until its owner unlinks it, which needs the token
            // this call already holds.
            let current = unsafe { current.as_ref() };

            if current.next.get() == Some(me) {
                current.next.set(waiter.next.get());

                return;
            }

            node = current.next.get();
        }
    }

    /// The first waiter whose state `wanted` accepts, if any.
    ///
    /// # Safety
    ///
    /// Only the list's one waker side may call this, so that no task can be editing the list.
    pub(crate) unsafe fn find(&self, wanted: impl Fn(&T) -> bool) -> Option<&Waiter<T>> {
        // Relaxed for the reason `link`'s store is: one core, and every edit ran to completion with
        // interrupts masked before this handler could start.
        let mut node = NonNull::new(self.head.load(Ordering::Relaxed));

        while let Some(current) = node {
            // SAFETY: nodes leave the list before they are dropped, and the caller cannot be interrupting
            // a task that is editing it.
            let current = unsafe { current.as_ref() };

            if wanted(&current.state) {
                return Some(current);
            }

            node = current.next.get();
        }

        None
    }
}
