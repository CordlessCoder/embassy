#![macro_use]

#[cfg(feature = "rt")]
use core::cell::Cell;
use core::convert::Infallible;
#[cfg(feature = "rt")]
use core::future::{Future, poll_fn};
#[cfg(feature = "rt")]
use core::ptr::{self, NonNull};
#[cfg(feature = "rt")]
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
#[cfg(feature = "rt")]
use core::task::{Poll, Waker};

#[cfg(all(feature = "rt", feature = "gpio-embassy-tasks-only"))]
use embassy_executor::raw::{TaskRef, task_from_waker, wake_task};
use embassy_hal_internal::{Peri, PeripheralType, impl_peripheral};

use crate::pac::gpio::vals::*;
use crate::pac::gpio::{self};
#[cfg(all(feature = "rt", any(gpioa_interrupt, gpiob_interrupt)))]
use crate::pac::interrupt;
use crate::pac::{self};

/// Represents a digital input or output level.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    /// Logical low.
    Low,
    /// Logical high.
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

/// Represents a pull setting for an input.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum Pull {
    /// No pull.
    None,
    /// Internal pull-up resistor.
    Up,
    /// Internal pull-down resistor.
    Down,
}

/// A GPIO bank with up to 32 pins.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum Port {
    /// Port A.
    PortA = 0,

    /// Port B.
    #[cfg(gpio_pb)]
    PortB = 1,

    /// Port C.
    #[cfg(gpio_pc)]
    PortC = 2,
}

/// GPIO flexible pin.
///
/// This pin can either be a disconnected, input, or output pin, or both. The level register bit will remain
/// set while not in output mode, so the pin's level will be 'remembered' when it is not in output
/// mode.
pub struct Flex<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> Flex<'d> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains disconnected. The initial output level is unspecified, but can be changed
    /// before the pin is put into output mode.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        // Pin will be in disconnected state.
        Self { pin: pin.into() }
    }

    /// Set the pin's pull.
    #[inline]
    pub fn set_pull(&mut self, pull: Pull) {
        let pincm = pac::IOMUX.pincm(self.pin.pin_cm() as usize);

        pincm.modify(|w| {
            w.set_pipd(matches!(pull, Pull::Down));
            w.set_pipu(matches!(pull, Pull::Up));
        });
    }

    /// Put the pin into input mode.
    ///
    /// The pull setting is left unchanged.
    #[inline]
    pub fn set_as_input(&mut self) {
        let pincm = pac::IOMUX.pincm(self.pin.pin_cm() as usize);

        pincm.modify(|w| {
            w.set_pf(GPIO_PF);
            w.set_hiz1(false);
            w.set_pc(true);
            w.set_inena(true);
        });

        self.pin.block().doeclr31_0().write(|w| {
            w.set_dio(self.pin.bit_index(), true);
        });
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn set_as_output(&mut self) {
        let pincm = pac::IOMUX.pincm(self.pin.pin_cm() as usize);

        pincm.modify(|w| {
            w.set_pf(GPIO_PF);
            w.set_hiz1(false);
            w.set_pc(true);
            w.set_inena(false);
        });

        self.pin.block().doeset31_0().write(|w| {
            w.set_dio(self.pin.bit_index(), true);
        });
    }

    /// Put the pin into input + open-drain output mode.
    ///
    /// The hardware will drive the line low if you set it to low, and will leave it floating if you set
    /// it to high, in which case you can read the input to figure out whether another device
    /// is driving the line low.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    ///
    /// The internal weak pull-up and pull-down resistors will be disabled.
    #[inline]
    pub fn set_as_input_output(&mut self) {
        let pincm = pac::IOMUX.pincm(self.pin.pin_cm() as usize);

        pincm.modify(|w| {
            w.set_pf(GPIO_PF);
            w.set_hiz1(true);
            w.set_pc(true);
            w.set_inena(true);
        });

        // Enable output driver (DOE) - required for open-drain to drive low
        self.pin.block().doeset31_0().write(|w| {
            w.set_dio(self.pin.bit_index(), true);
        });

        self.set_pull(Pull::None);
    }

    /// Set the pin as "disconnected", ie doing nothing and consuming the lowest
    /// amount of power possible.
    ///
    /// Drivers should disconnect their pins when dropped. This also disables the internal weak
    /// pull-up and pull-down resistors.
    #[inline]
    pub fn set_as_disconnected(&mut self) {
        let pincm = pac::IOMUX.pincm(self.pin.pin_cm() as usize);

        pincm.modify(|w| {
            w.set_pf(DISCONNECT_PF);
            w.set_hiz1(false);
            w.set_pc(false);
            w.set_inena(false);
        });

        self.set_pull(Pull::None);
        self.set_inversion(false);
    }

    /// Configure the logic inversion of this pin.
    ///
    /// Logic inversion applies to both the input and output path of this pin.
    #[inline]
    pub fn set_inversion(&mut self, invert: bool) {
        let pincm = pac::IOMUX.pincm(self.pin.pin_cm() as usize);

        pincm.modify(|w| {
            w.set_inv(invert);
        });
    }

    // TODO: drive strength, hysteresis, wakeup enable, wakeup compare

    /// Put the pin into the PF mode, unchecked.
    ///
    /// This puts the pin into the PF mode, with the request number. This is completely unchecked,
    /// it can attach the pin to literally any peripheral, so use with care. In addition the pin
    /// peripheral is connected in the iomux.
    ///
    /// The peripheral attached to the pin depends on the part in use. Consult the datasheet
    /// or technical reference manual for additional details.
    #[inline]
    pub fn set_pf_unchecked(&mut self, pf: u8) {
        // Per SLAU893 and SLAU846B, PF is only 6 bits
        assert_eq!(pf & 0xC0, 0, "PF is out of range");

        let pincm = pac::IOMUX.pincm(self.pin.pin_cm() as usize);

        pincm.modify(|w| {
            w.set_pf(pf);
            // If the PF is manually set, connect the pin
            w.set_pc(true);
        });
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.block().din31_0().read().dio(self.pin.bit_index())
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Returns current pin level
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.block().doutset31_0().write(|w| {
            w.set_dio(self.pin.bit_index() as usize, true);
        });
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.block().doutclr31_0().write(|w| {
            w.set_dio(self.pin.bit_index(), true);
        });
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.block().douttgl31_0().write(|w| {
            w.set_dio(self.pin.bit_index(), true);
        })
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// Get the current pin output level.
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    /// Is the output level high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.block().dout31_0().read().dio(self.pin.bit_index())
    }

    /// Is the output level low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_high(&mut self) {
        if self.is_high() {
            return;
        }

        self.wait_for_rising_edge().await
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_low(&mut self) {
        if self.is_low() {
            return;
        }

        self.wait_for_falling_edge().await
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[cfg(feature = "rt")]
    #[inline]
    pub fn wait_for_rising_edge(&mut self) -> impl Future<Output = ()> {
        self.wait_inner(Edge::Rising)
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[cfg(feature = "rt")]
    #[inline]
    pub fn wait_for_falling_edge(&mut self) -> impl Future<Output = ()> {
        self.wait_inner(Edge::Falling)
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[cfg(feature = "rt")]
    #[inline]
    pub fn wait_for_any_edge(&mut self) -> impl Future<Output = ()> {
        self.wait_inner(Edge::Any)
    }

    #[cfg(feature = "rt")]
    async fn wait_inner(&mut self, edge: Edge) {
        // Not armed here: `park` arms from inside its first poll, where the waker exists, so that the
        // registration and the unmask are one critical section and no edge can land between them.
        let arm = EdgeArm::new(self.pin.block(), self.pin.pin_port(), edge);

        park(&arm).await;
    }
}

/// Park until the interrupt reports the edge, arming the pin on the first poll.
///
/// The interrupt clears `outstanding`, so that is the completion test rather than the status bit. Nothing
/// is checked before arming because there is nothing to check: the edge that completes this wait is by
/// definition one that arrives after the pin is unmasked.
///
/// The arm is taken by value and moved into the closure rather than borrowed across the await: a borrow
/// would need `EdgeArm` to be `Sync`, which is a much larger claim than it needs to make.
#[cfg(feature = "rt")]
async fn park(arm: &EdgeArm) {
    let mut armed = false;

    poll_fn(|cx| {
        if !armed {
            arm.arm(cx.waker());
            armed = true;

            return Poll::Pending;
        }

        if !arm.waiter.outstanding.load(Ordering::Relaxed) {
            return Poll::Ready(());
        }

        // A spurious poll, or one from a different waker than the wait was armed with. Re-register and
        // look again, in that order, so an edge landing in between is not lost.
        arm.waiter.register(cx.waker());

        if !arm.waiter.outstanding.load(Ordering::Relaxed) {
            return Poll::Ready(());
        }

        Poll::Pending
    })
    .await;
}

/// Whether `GPIO_ERR_01` applies, which forces both directions to be detected.
///
/// Its case 2 loses every STANDBY1 wake after the first unless the pin detects both edges, so where it
/// applies the direction is filtered in software instead.
#[cfg(feature = "rt")]
const DETECT_BOTH_EDGES: bool = cfg!(gpio_err_01);

/// Which edge a task is waiting for.
#[derive(Clone, Copy)]
#[cfg(feature = "rt")]
enum Edge {
    Rising,
    Falling,
    Any,
}

#[cfg(feature = "rt")]
impl Edge {
    fn polarity(self) -> Polarity {
        match self {
            Edge::Rising => Polarity::Rise,
            Edge::Falling => Polarity::Fall,
            Edge::Any => Polarity::RiseFall,
        }
    }

    /// Whether an edge that left the pin reading `level` is one this wait asked for.
    fn accepts(self, level: bool) -> bool {
        match self {
            Edge::Rising => level,
            Edge::Falling => !level,
            Edge::Any => true,
        }
    }
}

/// Holds a pin's edge detection armed for one wait, and disarms it however the wait ends.
///
/// The waiter is a field rather than a separate allocation so that one `Drop` both disarms the pin and
/// unlinks the node, and so that the node's address is the address of a local in the future that owns it.
#[cfg(feature = "rt")]
struct EdgeArm {
    block: gpio::Gpio,
    bit: usize,
    port: usize,
    waiter: Waiter,
}

#[cfg(feature = "rt")]
impl EdgeArm {
    /// Describe a wait without touching the hardware. [`EdgeArm::arm`] is what starts it.
    ///
    /// Nothing here may be observable, because this value is returned by move: until it has come to rest
    /// in the frame that owns it, publishing its address would publish an address about to go stale.
    fn new(block: gpio::Gpio, pin_port: u8, edge: Edge) -> Self {
        Self {
            block,
            bit: usize::from(pin_port % 32),
            port: usize::from(pin_port / 32),
            waiter: Waiter::new(pin_port % 32, edge),
        }
    }

    /// Start the wait: select the edge, publish the waiter, and let the interrupt through.
    ///
    /// One critical section for the lot. Every write is either a read-modify-write of a register shared
    /// with the other pins, or a store the interrupt must not see half of, so each needed one anyway;
    /// nothing here waits, so holding interrupts off across all of them costs no more than the shortest.
    ///
    /// The order inside matters in one place: the unmask is last, so the waiter is reachable and the
    /// waker is stored before any edge can be reported.
    fn arm(&self, waker: &Waker) {
        // Before the section, not inside it: with `gpio-embassy-tasks-only` this is what rejects a waker
        // that is not an embassy task's, and it does it by panicking.
        let parked = Waiter::parked(waker);

        let polarity = if DETECT_BOTH_EDGES {
            Polarity::RiseFall
        } else {
            self.waiter.edge.polarity()
        };

        critical_section::with(|_cs| {
            if self.bit >= 16 {
                self.block.polarity31_16().modify(|w| {
                    w.set_dio(self.bit - 16, polarity);
                });
            } else {
                self.block.polarity15_0().modify(|w| {
                    w.set_dio(self.bit, polarity);
                });
            };

            // Drop edges from before the wait, after the polarity write so that selecting the event
            // cannot leave a status bit behind.
            self.block.cpu_int().iclr().write(|w| {
                w.set_dio(self.bit, true);
            });

            // Nothing was here to displace: the node is fresh, and this is its only arming.
            let _ = self.waiter.store_parked(parked);

            // SAFETY: interrupts are off, so no reader of the list can run; and `self` is borrowed for
            // the whole wait, which `EdgeArm::drop` ends by unlinking.
            unsafe { self.waiter.link(self.port) };

            // Without fast wake the input synchronizer is unclocked in STOP and STANDBY, which loses the
            // edge rather than delaying it.
            self.block.fastwake().modify(|w| w.set_din(self.bit, true));

            self.block.cpu_int().imask().modify(|w| {
                w.set_dio(self.bit, true);
            });
        });
    }
}

/// The edge waits must stay usable from a task that has to be `Send`, which the interior mutability in
/// [`Waiter`] would otherwise take away. Placed here rather than in a test because the failure it catches
/// is a change to a private field's type.
#[cfg(feature = "rt")]
fn _assert_edge_waits_are_send(pin: &mut Flex<'static>) {
    fn is_send<T: Send>(_: &T) {}

    is_send(&pin.wait_for_any_edge());
}

#[cfg(feature = "rt")]
impl Drop for EdgeArm {
    fn drop(&mut self) {
        critical_section::with(|_cs| {
            self.block.fastwake().modify(|w| w.set_din(self.bit, false));
            self.block.cpu_int().imask().modify(|w| w.set_dio(self.bit, false));

            // An edge that arrived while masked left this set with nobody to consume it.
            self.block.cpu_int().iclr().write(|w| w.set_dio(self.bit, true));

            // SAFETY: interrupts are off, and the pin is masked, so nothing can be walking the list or
            // about to read this node. Unlinking last is what makes the node safe to drop.
            unsafe { self.waiter.unlink(self.port) };
        });
    }
}

/// What a [`Waiter`] stores to wake with: the waker itself, or the task it belongs to.
#[cfg(all(feature = "rt", not(feature = "gpio-embassy-tasks-only")))]
type Parked = Waker;

#[cfg(all(feature = "rt", feature = "gpio-embassy-tasks-only"))]
type Parked = TaskRef;

/// One task waiting on one pin, linked into its port's list of waiters.
///
/// **The interrupt reads this with no lock held**, which is sound because of who writes it: the owning
/// task, always with interrupts off, and the port's own interrupt handler, which cannot preempt itself.
/// A second port's handler can preempt this one, but it walks a different list.
///
/// Only [`Waiter::outstanding`] is atomic, because it is the one field both sides write.
#[cfg(feature = "rt")]
struct Waiter {
    /// The next waiter on this port, or `None` at the end.
    next: Cell<Option<NonNull<Waiter>>>,

    /// Which pin within the port. The 5 bits the interrupt and the task have to agree on.
    bit: u8,

    /// Which edges this wait accepts. Read by the interrupt under [`DETECT_BOTH_EDGES`], where the
    /// polarity is set to both and the direction is filtered here instead.
    edge: Edge,

    /// Cleared by the interrupt when the edge arrives. This, not the status bit, is completion — the
    /// status bit says nothing about direction and is set by edges the task did not ask for.
    outstanding: AtomicBool,

    /// Whom to wake.
    ///
    /// A `Waker` is two words, so the interrupt could read it half-written if the task ever wrote it with
    /// interrupts on. It does not: [`Waiter::store_waker`] is called from inside a critical section, and
    /// [`Waiter::register`] takes one before it writes.
    #[cfg(not(feature = "gpio-embassy-tasks-only"))]
    waker: Cell<Option<Waker>>,

    /// Whom to wake, as one word, which needs no critical section to store.
    #[cfg(feature = "gpio-embassy-tasks-only")]
    task: AtomicPtr<()>,
}

/// SAFETY: the pointers a `Waiter` holds only mean anything while it is linked, and linking happens after
/// the node has been pinned — so any move of one has already happened by then, and a node that can still
/// be moved is one nothing else can reach.
#[cfg(feature = "rt")]
unsafe impl Send for Waiter {}

/// SAFETY: as above, plus every mutation is made with interrupts off, so no sharer can see a partial one.
#[cfg(feature = "rt")]
unsafe impl Sync for Waiter {}

#[cfg(feature = "rt")]
impl Waiter {
    fn new(bit: u8, edge: Edge) -> Self {
        Self {
            next: Cell::new(None),
            bit,
            edge,
            outstanding: AtomicBool::new(true),
            #[cfg(not(feature = "gpio-embassy-tasks-only"))]
            waker: Cell::new(None),
            #[cfg(feature = "gpio-embassy-tasks-only")]
            task: AtomicPtr::new(ptr::null_mut()),
        }
    }

    /// Push this waiter onto its port's list.
    ///
    /// # Safety
    ///
    /// Interrupts must be off, and the node must stay where it is and be [`Waiter::unlink`]ed before it
    /// is dropped.
    unsafe fn link(&self, port: usize) {
        let head = &WAITERS[port];

        self.next.set(NonNull::new(head.load(Ordering::Relaxed)));
        head.store(ptr::from_ref(self).cast_mut(), Ordering::Release);
    }

    /// Take this waiter back off its port's list, wherever in it the node happens to be.
    ///
    /// A search rather than a doubly-linked list: the length is the number of pins being awaited at once,
    /// which is one in every case measured, and a `prev` pointer would cost every node four bytes and
    /// every link an extra store to save nothing at that length.
    ///
    /// # Safety
    ///
    /// Interrupts must be off.
    unsafe fn unlink(&self, port: usize) {
        let me = NonNull::from(self);
        let head = &WAITERS[port];

        if head.load(Ordering::Relaxed) == me.as_ptr() {
            head.store(
                self.next.get().map_or(ptr::null_mut(), NonNull::as_ptr),
                Ordering::Release,
            );

            return;
        }

        let mut node = NonNull::new(head.load(Ordering::Relaxed));

        while let Some(current) = node {
            // SAFETY: every node in the list is live until its owner unlinks it, which needs interrupts
            // off, and they are off here.
            let current = unsafe { current.as_ref() };

            if current.next.get() == Some(me) {
                current.next.set(self.next.get());

                return;
            }

            node = current.next.get();
        }
    }

    /// Work out what to store for `waker`, before any critical section is taken.
    ///
    /// # Panics
    ///
    /// With `gpio-embassy-tasks-only`, if the waker is not an embassy task's — which is what that feature
    /// asserts about every edge wait in the build. Kept out of the critical section on purpose: a panic
    /// with interrupts off takes the log with it.
    fn parked(waker: &Waker) -> Parked {
        #[cfg(not(feature = "gpio-embassy-tasks-only"))]
        return waker.clone();

        #[cfg(feature = "gpio-embassy-tasks-only")]
        return task_from_waker(waker);
    }

    /// Store what [`Waiter::parked`] worked out, handing back whatever it displaced.
    ///
    /// Every caller is inside a critical section, which is why the old value is returned rather than
    /// dropped here: a `Waker`'s drop is someone else's code, and it has no business running with
    /// interrupts off.
    #[must_use = "the displaced waker has to be dropped outside the critical section"]
    fn store_parked(&self, parked: Parked) -> Option<Parked> {
        #[cfg(not(feature = "gpio-embassy-tasks-only"))]
        return self.waker.replace(Some(parked));

        #[cfg(feature = "gpio-embassy-tasks-only")]
        {
            self.task.store(parked.as_raw().as_ptr(), Ordering::Release);

            None
        }
    }

    /// Store `waker` from a poll, where interrupts are on.
    ///
    /// The waker a future is polled with almost never changes, so the common path is a comparison and no
    /// write at all — which is the only reason this can afford to be on the completing poll's path.
    fn register(&self, waker: &Waker) {
        #[cfg(not(feature = "gpio-embassy-tasks-only"))]
        {
            // SAFETY: this task is the only writer, so nothing can be changing it under this read.
            if let Some(stored) = unsafe { &*self.waker.as_ptr() }
                && stored.will_wake(waker)
            {
                return;
            }
        }

        let parked = Self::parked(waker);

        // Bound rather than discarded: this is what drops the waker it displaced, and it has to happen
        // out here rather than inside the section.
        let _displaced = critical_section::with(|_cs| self.store_parked(parked));
    }

    /// Report the edge: mark the wait done and wake whoever is on it.
    ///
    /// Called from the interrupt, with no lock. Reading the waker rather than taking it is what makes
    /// that sound — the task is then the only writer of the field.
    fn report(&self) {
        self.outstanding.store(false, Ordering::Relaxed);

        #[cfg(not(feature = "gpio-embassy-tasks-only"))]
        // SAFETY: written only by the owning task, and only with interrupts off.
        if let Some(waker) = unsafe { &*self.waker.as_ptr() } {
            waker.wake_by_ref();
        }

        #[cfg(feature = "gpio-embassy-tasks-only")]
        if let Some(task) = NonNull::new(self.task.load(Ordering::Acquire)) {
            // SAFETY: written by `store_waker` from `as_raw`, and an embassy task lives for the rest of
            // the program.
            unsafe { wake_task(TaskRef::from_raw(task)) };
        }
    }
}

/// The waiter parked on `bit` of `port`, if any.
///
/// # Safety
///
/// Only the port's own interrupt handler may call this, so that no task can be editing the list.
#[cfg(feature = "rt")]
unsafe fn waiter_for(port: usize, bit: usize) -> Option<&'static Waiter> {
    let mut node = NonNull::new(WAITERS[port].load(Ordering::Acquire));

    while let Some(current) = node {
        // SAFETY: nodes leave the list before they are dropped, and the caller cannot be interrupting a
        // task that is editing it.
        let current = unsafe { current.as_ref() };

        if usize::from(current.bit) == bit {
            return Some(current);
        }

        node = current.next.get();
    }

    None
}

#[cfg(feature = "rt")]
const PORT_COUNT: usize = if cfg!(gpio_pc) {
    3
} else if cfg!(gpio_pb) {
    2
} else {
    1
};

/// The waiters on each port, newest first.
///
/// One pointer per port, not per pin: the nodes live in the futures waiting on them, so this costs RAM
/// per port and stack per wait rather than RAM per pin.
///
/// Only load and store are used, both of which this core does without a compare-and-swap to emulate, so
/// the list needs no lock of its own — see [`Waiter`] for who is allowed to touch it when.
#[cfg(feature = "rt")]
static WAITERS: [AtomicPtr<Waiter>; PORT_COUNT] = [const { AtomicPtr::new(ptr::null_mut()) }; PORT_COUNT];

impl<'d> Drop for Flex<'d> {
    #[inline]
    fn drop(&mut self) {
        self.set_as_disconnected();
    }
}

/// GPIO input driver.
pub struct Input<'d> {
    pin: Flex<'d>,
}

impl<'d> Input<'d> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull] configuration.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_input();
        pin.set_pull(pull);
        Self { pin }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    /// Configure the logic inversion of this pin.
    ///
    /// Logic inversion applies to the input path of this pin.
    #[inline]
    pub fn set_inversion(&mut self, invert: bool) {
        self.pin.set_inversion(invert)
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await
    }
}

/// GPIO output driver.
///
/// Note that pins will **return to their floating state** when `Output` is dropped.
/// If pins should retain their state indefinitely, either keep ownership of the
/// `Output`, or pass it to [`core::mem::forget`].
pub struct Output<'d> {
    pin: Flex<'d>,
}

impl<'d> Output<'d> {
    /// Create GPIO output driver for a [Pin] with the provided [Level] configuration.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_output();
        pin.set_level(initial_output);
        Self { pin }
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }

    /// Configure the logic inversion of this pin.
    ///
    /// Logic inversion applies to the input path of this pin.
    #[inline]
    pub fn set_inversion(&mut self, invert: bool) {
        self.pin.set_inversion(invert)
    }
}

/// GPIO output open-drain driver.
///
/// Note that pins will **return to their floating state** when `OutputOpenDrain` is dropped.
/// If pins should retain their state indefinitely, either keep ownership of the
/// `OutputOpenDrain`, or pass it to [`core::mem::forget`].
pub struct OutputOpenDrain<'d> {
    pin: Flex<'d>,
}

impl<'d> OutputOpenDrain<'d> {
    /// Create a new GPIO open drain output driver for a [Pin] with the provided [Level].
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_level(initial_output);
        pin.set_as_input_output();
        Self { pin }
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        !self.pin.is_low()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Get the current pin input level.
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level);
    }

    /// Get whether the output level is set to high.
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Get whether the output level is set to low.
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// Get the current output level.
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }

    /// Configure the logic inversion of this pin.
    ///
    /// Logic inversion applies to the input path of this pin.
    #[inline]
    pub fn set_inversion(&mut self, invert: bool) {
        self.pin.set_inversion(invert)
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[cfg(feature = "rt")]
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await
    }
}

/// Type-erased GPIO pin
pub struct AnyPin {
    pub(crate) pin_port: u8,
}

impl AnyPin {
    /// Create an [AnyPin] for a specific pin.
    ///
    /// # Safety
    /// - `pin_port` should not in use by another driver.
    #[inline]
    pub unsafe fn steal(pin_port: u8) -> Peri<'static, Self> {
        Peri::new_unchecked(Self { pin_port })
    }
}

impl_peripheral!(AnyPin);

impl Pin for AnyPin {}
impl SealedPin for AnyPin {
    #[inline]
    fn pin_port(&self) -> u8 {
        self.pin_port
    }
}

/// Interface for a Pin that can be configured by an [Input] or [Output] driver, or converted to an [AnyPin].
#[allow(private_bounds)]
pub trait Pin: PeripheralType + Into<AnyPin> + SealedPin + Sized + 'static {
    /// The index of this pin in PINCM (pin control management) registers.
    #[inline]
    fn pin_cm(&self) -> u8 {
        self._pin_cm()
    }
}

impl<'d> embedded_hal::digital::ErrorType for Flex<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::InputPin for Flex<'d> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal::digital::OutputPin for Flex<'d> {
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }
}

impl<'d> embedded_hal::digital::StatefulOutputPin for Flex<'d> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

#[cfg(feature = "rt")]
impl<'d> embedded_hal_async::digital::Wait for Flex<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

impl<'d> embedded_hal::digital::ErrorType for Input<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::InputPin for Input<'d> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

#[cfg(feature = "rt")]
impl<'d> embedded_hal_async::digital::Wait for Input<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

impl<'d> embedded_hal::digital::ErrorType for Output<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::OutputPin for Output<'d> {
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }
}

impl<'d> embedded_hal::digital::StatefulOutputPin for Output<'d> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal::digital::ErrorType for OutputOpenDrain<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::InputPin for OutputOpenDrain<'d> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal::digital::OutputPin for OutputOpenDrain<'d> {
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }
}

impl<'d> embedded_hal::digital::StatefulOutputPin for OutputOpenDrain<'d> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

#[cfg(feature = "rt")]
impl<'d> embedded_hal_async::digital::Wait for OutputOpenDrain<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

#[cfg_attr(mspm0g518x, allow(dead_code))]
#[derive(Copy, Clone)]
pub struct PfType {
    pull: Pull,
    input: bool,
    invert: bool,
}

impl PfType {
    pub const fn input(pull: Pull, invert: bool) -> Self {
        Self {
            pull,
            input: true,
            invert,
        }
    }

    pub const fn output(pull: Pull, invert: bool) -> Self {
        Self {
            pull,
            input: false,
            invert,
        }
    }
}

/// The level on a pin that wakes the device from SHUTDOWN.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WakeLevel {
    /// Wake when the pin is driven low.
    Low,

    /// Wake when the pin is driven high.
    High,
}

/// Arms a wake-capable pin to bring the device out of SHUTDOWN.
///
/// SHUTDOWN powers down `VCORE`, so the GPIO peripheral is off and none of the ordinary pin APIs
/// survive it; the wake controller watches the pin instead, and waking is a reset rather than a
/// resume. Hold one of these across `low_power::shutdown` and identify
/// the cause on the next boot with
/// [`ResetCause::BorWakeFromShutdown`](crate::ResetCause::BorWakeFromShutdown).
///
/// Only pins with wakeup logic can do this, which [`WakeCapablePin`] enforces at compile time. For
/// waking from STOP or STANDBY use the ordinary edge-wait methods on [`Flex`], which work on any pin.
pub struct ShutdownWake<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> ShutdownWake<'d> {
    /// Wake the device from SHUTDOWN when `pin` reaches `level`.
    ///
    /// Takes the pin so that nothing else reconfigures it while armed. Pull it to the opposite level
    /// first if it would otherwise float, since the comparison is against the pin, not an edge.
    pub fn new(pin: Peri<'d, impl WakeCapablePin>, level: WakeLevel) -> Self {
        let this = Self { pin: pin.into() };

        pac::IOMUX.pincm(this.pin.pin_cm() as usize).modify(|w| {
            w.set_wcomp(matches!(level, WakeLevel::High));
            w.set_wuen(true);
        });

        this
    }
}

impl Drop for ShutdownWake<'_> {
    fn drop(&mut self) {
        pac::IOMUX.pincm(self.pin.pin_cm() as usize).modify(|w| {
            w.set_wuen(false);
        });
    }
}

/// The pin function to disconnect peripherals from the pin.
///
/// This is also the pin function used to connect to analog peripherals, such as an ADC.
const DISCONNECT_PF: u8 = 0;

/// The pin function for the GPIO peripheral.
///
/// This is fixed to `1` for every part.
pub(crate) const GPIO_PF: u8 = 1;

/// A pin with wakeup logic, able to bring the device out of SHUTDOWN.
///
/// Distinct from waking the device at all: `FASTWAKE`, which [`Flex::wait_for_any_edge`] and friends
/// use, works on any GPIO pin but only down to STANDBY. SHUTDOWN powers the GPIO logic off entirely,
/// and only these pins keep a path to the wake controller. See [`ShutdownWake`].
///
/// Which pins qualify comes from the chip metadata, and `mspm0c110x` has none at all. Where the vendor
/// data omits the information entirely, every pin is accepted rather than none.
pub trait WakeCapablePin: Pin {}

// mspm0c110x has no wake-capable pin.
#[allow(unused_macros)]
macro_rules! impl_wake_capable_pin {
    ($name: ident) => {
        impl crate::gpio::WakeCapablePin for crate::peripherals::$name {}
    };
}

macro_rules! impl_pin {
    ($name: ident, $port: expr, $pin_num: expr) => {
        impl crate::gpio::Pin for crate::peripherals::$name {}
        impl crate::gpio::SealedPin for crate::peripherals::$name {
            #[inline]
            fn pin_port(&self) -> u8 {
                ($port as u8) * 32 + $pin_num
            }
        }

        impl From<crate::peripherals::$name> for crate::gpio::AnyPin {
            fn from(val: crate::peripherals::$name) -> Self {
                Self {
                    pin_port: crate::gpio::SealedPin::pin_port(&val),
                }
            }
        }
    };
}

pub(crate) trait SealedPin {
    fn pin_port(&self) -> u8;

    fn _pin_cm(&self) -> u8 {
        // Some parts like the MSPM0L222x have pincm mappings all over the place.
        crate::gpio_pincm(self.pin_port())
    }

    fn bit_index(&self) -> usize {
        (self.pin_port() % 32) as usize
    }

    #[inline]
    fn set_as_analog(&self) {
        let pincm = pac::IOMUX.pincm(self._pin_cm() as usize);

        pincm.modify(|w| {
            w.set_pf(DISCONNECT_PF);
            w.set_pipu(false);
            w.set_pipd(false);
        });
    }

    #[cfg_attr(mspm0g518x, allow(dead_code))]
    fn update_pf(&self, ty: PfType) {
        let pincm = pac::IOMUX.pincm(self._pin_cm() as usize);
        let pf = pincm.read().pf();

        set_pf(self._pin_cm() as usize, pf, ty);
    }

    #[cfg_attr(mspm0g518x, allow(dead_code))]
    fn set_as_pf(&self, pf: u8, ty: PfType) {
        set_pf(self._pin_cm() as usize, pf, ty)
    }

    /// Set the pin as "disconnected", ie doing nothing and consuming the lowest
    /// amount of power possible.
    ///
    /// This is currently the same as [`Self::set_as_analog()`] but is semantically different
    /// really. Drivers should `set_as_disconnected()` pins when dropped.
    ///
    /// Note that this also disables the internal weak pull-up and pull-down resistors.
    #[inline]
    #[cfg_attr(mspm0g518x, allow(dead_code))]
    fn set_as_disconnected(&self) {
        self.set_as_analog();
    }

    #[inline]
    fn block(&self) -> gpio::Gpio {
        match self.pin_port() / 32 {
            0 => pac::GPIOA,
            #[cfg(gpio_pb)]
            1 => pac::GPIOB,
            #[cfg(gpio_pc)]
            2 => pac::GPIOC,
            _ => unreachable!(),
        }
    }
}

#[inline(never)]
fn set_pf(pincm: usize, pf: u8, ty: PfType) {
    pac::IOMUX.pincm(pincm).modify(|w| {
        w.set_pf(pf);
        w.set_pc(true);
        w.set_pipu(ty.pull == Pull::Up);
        w.set_pipd(ty.pull == Pull::Down);
        w.set_inena(ty.input);
        w.set_inv(ty.invert);
    });
}

pub(crate) fn init(gpio: gpio::Gpio) {
    gpio.gprcm().rstctl().write(|w| {
        w.set_resetstkyclr(true);
        w.set_resetassert(true);
        w.set_key(ResetKey::Key);
    });

    gpio.gprcm().pwren().write(|w| {
        w.set_enable(true);
        w.set_key(PwrenKey::Key);
    });

    gpio.evt_mode().modify(|w| {
        // The CPU will clear it's own interrupts
        w.set_cpu_cfg(EvtCfg::Software);
    });
}

/// Classify the edges that have arrived and answer the requests they satisfy.
#[cfg(feature = "rt")]
fn irq_handler(gpio: gpio::Gpio, port: Port) {
    use crate::BitIter;

    // Opened as the handler's first action, so the rising edge timestamps the silicon wake plus the
    // interrupt entry and group dispatch, with none of this handler in it. Both markers are resolved
    // here so that neither bracket pays for a lookup.
    #[cfg(feature = "_probe")]
    let (handler_marker, waker_marker) = {
        use crate::probe::{Marker, target};

        let markers = (target(Marker::Handler), target(Marker::Waker));
        crate::probe::set(markers.0);
        markers
    };

    // Only pins with the interrupt unmasked, which is only pins with a wait armed.
    let pending = gpio.cpu_int().mis().read().0;

    // One snapshot for all of them: the status bit carries no direction, so the level a pin settled
    // at is the only thing an edge can be classified by.
    let level = gpio.din31_0().read();

    for bit in BitIter(pending).map(|bit| bit as usize) {
        gpio.cpu_int().iclr().write(|w| {
            w.set_dio(bit, true);
        });

        // SAFETY: this is the port's own interrupt handler, which is the only caller allowed.
        let waiter = unsafe { waiter_for(port as usize, bit) };

        // An edge the other way leaves the wait standing, so it continues without the task ever being
        // woken. Skipped where `POLARITY` did the filtering, since the level can have moved on since the
        // edge and would only be a chance to classify it wrongly.
        if let Some(waiter) = waiter
            && DETECT_BOTH_EDGES
            && !waiter.edge.accepts(level.dio(bit))
        {
            continue;
        }

        #[cfg(feature = "_probe")]
        crate::probe::set(waker_marker);

        // A pin with no waiter is one whose wait was dropped between the edge and here. Nothing to
        // report, and the mask below is what stops it arriving again.
        if let Some(waiter) = waiter {
            waiter.report();
        }

        #[cfg(feature = "_probe")]
        crate::probe::clear(waker_marker);

        // Nothing left to report until the task arms the next wait, so keep this pin out of here —
        // and out of the wake path, in case the device sleeps first.
        gpio.cpu_int().imask().modify(|w| {
            w.set_dio(bit, false);
        });
    }

    // Falling edge closes the handler, so what remains before the task's own pin moves is the return to
    // whichever executor is waiting and one poll of it.
    #[cfg(feature = "_probe")]
    crate::probe::clear(handler_marker);
}

#[cfg(all(gpioa_interrupt, gpioa_group))]
compile_error!("gpioa_interrupt and gpioa_group are mutually exclusive cfgs");
#[cfg(all(gpiob_interrupt, gpiob_group))]
compile_error!("gpiob_interrupt and gpiob_group are mutually exclusive cfgs");

// C110x and L110x have a dedicated interrupts just for GPIOA.
//
// These chips do not have a GROUP1 interrupt.
#[cfg(all(feature = "rt", gpioa_interrupt))]
#[interrupt]
fn GPIOA() {
    irq_handler(pac::GPIOA, Port::PortA);
}

#[cfg(all(feature = "rt", gpiob_interrupt))]
#[interrupt]
fn GPIOB() {
    irq_handler(pac::GPIOB, Port::PortB);
}

// These symbols are weakly defined as DefaultHandler and are called by the interrupt group implementation.
//
// Defining these as no_mangle is required so that the linker will pick these over the default handler.

#[cfg(all(feature = "rt", gpioa_group))]
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
fn GPIOA() {
    irq_handler(pac::GPIOA, Port::PortA);
}

#[cfg(all(feature = "rt", gpiob_group))]
#[unsafe(no_mangle)]
#[allow(non_snake_case)]
fn GPIOB() {
    irq_handler(pac::GPIOB, Port::PortB);
}

#[cfg(all(feature = "rt", gpioc_group))]
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
fn GPIOC() {
    irq_handler(pac::GPIOC, Port::PortC);
}
