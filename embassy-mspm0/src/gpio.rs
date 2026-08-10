#![macro_use]

use core::convert::Infallible;
#[cfg(feature = "rt")]
use core::future::{Future, poll_fn};
use core::marker::PhantomData;
#[cfg(feature = "rt")]
use core::task::{Poll, Waker};

use embassy_hal_internal::{Peri, PeripheralType, impl_peripheral};

#[cfg(feature = "rt")]
use crate::mode::Async;
use crate::mode::{Blocking, Mode};
use crate::pac::gpio::vals::*;
use crate::pac::gpio::{self};
use crate::pac::{self};
#[cfg(feature = "rt")]
use crate::sync::linked_waiter::{Waiter, WaiterList};

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
///
/// [`Flex::new`] gives a pin whose level can be read and driven. Waiting for an edge needs an
/// interrupt handler behind it, so it lives on [`Flex<Async>`] and [`Flex::new_async`], which asks
/// for the binding that installs one.
pub struct Flex<'d, M: Mode = Blocking> {
    pin: Peri<'d, AnyPin>,
    _mode: PhantomData<M>,
}

impl<'d> Flex<'d, Blocking> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains disconnected. The initial output level is unspecified, but can be changed
    /// before the pin is put into output mode.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        // Pin will be in disconnected state.
        Self {
            pin: pin.into(),
            _mode: PhantomData,
        }
    }
}

#[cfg(feature = "rt")]
impl<'d> Flex<'d, Async> {
    /// Wrap the pin in a `Flex` that can wait for an edge.
    ///
    /// The pin remains disconnected. The initial output level is unspecified, but can be changed
    /// before the pin is put into output mode.
    #[inline]
    pub fn new_async(pin: Peri<'d, impl Pin>, _irqs: impl PortInterrupts + 'd) -> Self {
        Self {
            pin: pin.into(),
            _mode: PhantomData,
        }
    }
}

impl<'d, M: Mode> Flex<'d, M> {
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
}

#[cfg(feature = "rt")]
impl<'d> Flex<'d, Async> {
    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        if self.is_high() {
            return;
        }

        // Not `wait_for_rising_edge`: this promises a level, so the wait has to survive the pin going
        // high between the test above and the edge detector being armed. `park` re-tests it there.
        self.wait_inner(Edge::Rising, Some(true)).await
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        if self.is_low() {
            return;
        }

        self.wait_inner(Edge::Falling, Some(false)).await
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub fn wait_for_rising_edge(&mut self) -> impl Future<Output = ()> {
        self.wait_inner(Edge::Rising, None)
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub fn wait_for_falling_edge(&mut self) -> impl Future<Output = ()> {
        self.wait_inner(Edge::Falling, None)
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub fn wait_for_any_edge(&mut self) -> impl Future<Output = ()> {
        self.wait_inner(Edge::Any, None)
    }

    async fn wait_inner(&mut self, edge: Edge, settled_high: Option<bool>) {
        // Not armed here: `park` arms from inside its first poll, where the waker exists, so that the
        // registration and the unmask are one critical section and no edge can land between them.
        let arm = EdgeArm::new(self.pin.block(), self.pin.pin_port(), edge, settled_high);

        park(&arm).await;
    }
}

/// Park until the interrupt reports the edge, arming the pin on the first poll.
///
/// The interrupt clears `outstanding`, so that is the completion test rather than the status bit.
///
/// `settled_high` is what separates a level wait from an edge wait. An edge wait passes [`None`]: the
/// edge that completes it is by definition one that arrives after the pin is unmasked, so there is
/// nothing to check before arming. A level wait passes the level it is waiting for, because it promises
/// something else — that it returns once the pin *is* at that level, however it got there.
///
/// **Arming clears the edge status**, so a level that arrives between the caller's own test and the arm
/// below takes its edge with it and the wait would otherwise block for a second one that may never come.
/// Re-testing the level immediately after arming closes that window: either the edge is still to come
/// and the wait proceeds, or the level is already there and the wait is over.
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

            if let Some(high) = arm.settled_high {
                if arm.is_high() == high {
                    return Poll::Ready(());
                }
            }

            return Poll::Pending;
        }

        if !arm.waiter.is_outstanding() {
            return Poll::Ready(());
        }

        // A spurious poll, or one from a different waker than the wait was armed with. Re-register and
        // look again, in that order, so an edge landing in between is not lost.
        arm.waiter.register(cx.waker());

        if !arm.waiter.is_outstanding() {
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
    /// Bytes rather than indices: this lives in the task frame for as long as the wait does, and on
    /// this core a byte load zero-extends for free, so the widening at each use costs four bytes of
    /// flash against eight of RAM per waiting task.
    bit: u8,
    port: u8,
    /// The level a level wait is settling for, or [`None`] for an edge wait.
    ///
    /// Here rather than in [`park`]'s future because `bit` and `port` leave padding before `waiter`,
    /// so this rides along free; in the future it grew the task frame for every edge wait too.
    settled_high: Option<bool>,
    waiter: Waiter<EdgeWait>,
}

#[cfg(feature = "rt")]
impl EdgeArm {
    /// Whether the pin reads high right now.
    ///
    /// [`park`] uses this to close the window between a caller testing the level and the edge detector
    /// being armed.
    #[inline]
    fn is_high(&self) -> bool {
        self.block.din31_0().read().dio(self.bit as usize)
    }

    /// Describe a wait without touching the hardware. [`EdgeArm::arm`] is what starts it.
    ///
    /// Nothing here may be observable, because this value is returned by move: until it has come to rest
    /// in the frame that owns it, publishing its address would publish an address about to go stale.
    ///
    /// **Both of this type's `unsafe` assumptions are established here**, and neither field is written
    /// again: the `% 32` is what lets [`EdgeArm::bit`] promise a bit in range, and the `/ 32` of a pin
    /// this chip has is what lets [`EdgeArm::waiters`] index the port array without a check.
    fn new(block: gpio::Gpio, pin_port: u8, edge: Edge, settled_high: Option<bool>) -> Self {
        Self {
            block,
            bit: pin_port % 32,
            port: pin_port / 32,
            settled_high,
            waiter: Waiter::new(EdgeWait {
                bit: pin_port % 32,
                edge,
            }),
        }
    }

    /// The pin's bit within its port, with its range restated.
    ///
    /// `bit` is below 32 by construction, but it is reloaded from the task frame across the await, so
    /// that is lost by the time the register writes need it and every `set_dio`/`set_din` below carries
    /// the metapac's bounds assert into the critical section. Restating it folds those asserts and
    /// their panic site away; masking instead would do the same but pays an `and` at every use.
    ///
    /// # Safety
    ///
    /// `bit` is `pin_port % 32`, set once by [`EdgeArm::new`] and never written again.
    ///
    /// Always inlined: the hint only reaches the register writes if it lands in the same body as they
    /// do, and out of line it would buy a call instead of removing a branch.
    #[inline(always)]
    fn bit(&self) -> usize {
        unsafe { core::hint::assert_unchecked(self.bit < 32) };
        usize::from(self.bit)
    }

    /// The list this pin's port waits on.
    ///
    /// `port` comes from a run-time pin number, so nothing proves it indexes the array, and both
    /// callers below would otherwise carry a bounds check and a panic site. Masking the index instead
    /// is worse than the check it removes: `PORT_COUNT` is 3 on the widest parts, and at `opt-level`
    /// `"z"` a modulo by 3 is lowered to `__aeabi_uidivmod`, which drags in 400 bytes of divider.
    ///
    /// # Safety
    ///
    /// `port` is `pin_port / 32` for a pin this chip has, and `PORT_COUNT` counts this chip's ports;
    /// both come from the same generated pin metadata, so the index is in range.
    fn waiters(&self) -> &'static WaiterList<EdgeWait> {
        unsafe { WAITERS.get_unchecked(usize::from(self.port)) }
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
        // Before the section: a `Waker`'s clone is someone else's code, and it has no business running
        // with interrupts off.
        let parked = waker.clone();

        let polarity = if DETECT_BOTH_EDGES {
            Polarity::RiseFall
        } else {
            self.waiter.state.edge.polarity()
        };

        critical_section::with(|cs| {
            // Both halves hold sixteen two-bit fields in a `u32`, and the metapac gives them separate
            // types, so writing through those would emit this read-modify-write twice. Choosing the
            // register first and editing the field by hand emits it once.
            let polarity_reg = if self.bit >= 16 {
                self.block.polarity31_16().as_ptr() as *mut u32
            } else {
                self.block.polarity15_0().as_ptr() as *mut u32
            };
            let shift = (self.bit() % 16) * 2;

            // SAFETY: the pointer is one of this block's own polarity registers, and the critical
            // section this runs in keeps the read-modify-write whole against the port's interrupt.
            unsafe {
                let polarity_bits = polarity_reg.read_volatile() & !(0b11 << shift);
                polarity_reg.write_volatile(polarity_bits | ((polarity as u32) << shift));
            }

            // Drop edges from before the wait, after the polarity write so that selecting the event
            // cannot leave a status bit behind.
            self.block.cpu_int().iclr().write(|w| {
                w.set_dio(self.bit(), true);
            });

            // Nothing was here to displace: the node is fresh, and this is its only arming.
            let _ = self.waiter.store_waker(parked, cs);

            // SAFETY: interrupts are off, so no reader of the list can run; and `self` is borrowed for
            // the whole wait, which `EdgeArm::drop` ends by unlinking.
            unsafe { self.waiters().link(&self.waiter, cs) };

            // Without fast wake the input synchronizer is unclocked in STOP and STANDBY, which loses the
            // edge rather than delaying it.
            self.block.fastwake().modify(|w| w.set_din(self.bit(), true));

            self.block.cpu_int().imask().modify(|w| {
                w.set_dio(self.bit(), true);
            });
        });
    }
}

/// The edge waits must stay usable from a task that has to be `Send`, which the interior mutability in
/// [`Waiter`] would otherwise take away. Placed here rather than in a test because the failure it catches
/// is a change to a private field's type.
#[cfg(feature = "rt")]
fn _assert_edge_waits_are_send(pin: &mut Flex<'static, Async>) {
    fn is_send<T: Send>(_: &T) {}

    is_send(&pin.wait_for_any_edge());
}

#[cfg(feature = "rt")]
impl Drop for EdgeArm {
    fn drop(&mut self) {
        critical_section::with(|cs| {
            self.block.fastwake().modify(|w| w.set_din(self.bit(), false));
            self.block.cpu_int().imask().modify(|w| w.set_dio(self.bit(), false));

            // An edge that arrived while masked left this set with nobody to consume it.
            self.block.cpu_int().iclr().write(|w| w.set_dio(self.bit(), true));

            // The pin is masked, so nothing can be walking the list or about to read this node.
            // Unlinking last is what makes the node safe to drop.
            self.waiters().unlink(&self.waiter, cs);
        });
    }
}

/// What the port's interrupt matches a wake against: which pin, and which edges that wait accepts.
#[cfg(feature = "rt")]
struct EdgeWait {
    /// Which pin within the port. The 5 bits the interrupt and the task have to agree on.
    bit: u8,

    /// Which edges this wait accepts. Read by the interrupt under [`DETECT_BOTH_EDGES`], where the
    /// polarity is set to both and the direction is filtered here instead.
    edge: Edge,
}

#[cfg(feature = "rt")]
const PORT_COUNT: usize = if cfg!(gpio_pc) {
    3
} else if cfg!(gpio_pb) {
    2
} else {
    1
};

/// The waiters on each port. One list per port because one interrupt handler per port is what walks
/// them, which is the rule `linked_waiter` is sound under.
#[cfg(feature = "rt")]
static WAITERS: [WaiterList<EdgeWait>; PORT_COUNT] = [const { WaiterList::new() }; PORT_COUNT];

impl<'d, M: Mode> Drop for Flex<'d, M> {
    #[inline]
    fn drop(&mut self) {
        self.set_as_disconnected();
    }
}

/// GPIO input driver.
///
/// [`Input::new`] gives a pin whose level can be read. Waiting for an edge needs an interrupt
/// handler behind it, so it lives on [`Input<Async>`] and [`Input::new_async`], which asks for the
/// binding that installs one.
pub struct Input<'d, M: Mode = Blocking> {
    pin: Flex<'d, M>,
}

impl<'d> Input<'d, Blocking> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull] configuration.
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, pull: Pull) -> Self {
        Self {
            pin: Self::configure(Flex::new(pin), pull),
        }
    }
}

#[cfg(feature = "rt")]
impl<'d> Input<'d, Async> {
    /// Create a GPIO input driver that can wait for an edge.
    #[inline]
    pub fn new_async(pin: Peri<'d, impl Pin>, pull: Pull, irqs: impl PortInterrupts + 'd) -> Self {
        Self {
            pin: Self::configure(Flex::new_async(pin, irqs), pull),
        }
    }
}

impl<'d, M: Mode> Input<'d, M> {
    #[inline]
    fn configure(mut pin: Flex<'d, M>, pull: Pull) -> Flex<'d, M> {
        pin.set_as_input();
        pin.set_pull(pull);
        pin
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
}

#[cfg(feature = "rt")]
impl<'d> Input<'d, Async> {
    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
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
        pin.set_level(initial_output);
        pin.set_as_output();
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
///
/// [`OutputOpenDrain::new`] gives a pin whose level can be read and driven. Waiting for an edge
/// needs an interrupt handler behind it, so it lives on [`OutputOpenDrain<Async>`] and
/// [`OutputOpenDrain::new_async`], which asks for the binding that installs one.
pub struct OutputOpenDrain<'d, M: Mode = Blocking> {
    pin: Flex<'d, M>,
}

impl<'d> OutputOpenDrain<'d, Blocking> {
    /// Create a new GPIO open drain output driver for a [Pin] with the provided [Level].
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, initial_output: Level) -> Self {
        Self {
            pin: Self::configure(Flex::new(pin), initial_output),
        }
    }
}

#[cfg(feature = "rt")]
impl<'d> OutputOpenDrain<'d, Async> {
    /// Create a new GPIO open drain output driver that can wait for an edge.
    #[inline]
    pub fn new_async(pin: Peri<'d, impl Pin>, initial_output: Level, irqs: impl PortInterrupts + 'd) -> Self {
        Self {
            pin: Self::configure(Flex::new_async(pin, irqs), initial_output),
        }
    }
}

impl<'d, M: Mode> OutputOpenDrain<'d, M> {
    #[inline]
    fn configure(mut pin: Flex<'d, M>, initial_output: Level) -> Flex<'d, M> {
        pin.set_level(initial_output);
        pin.set_as_input_output();
        pin
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
}

#[cfg(feature = "rt")]
impl<'d> OutputOpenDrain<'d, Async> {
    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
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
    /// - `pin_port` must name a pin this chip has. The edge waits index their port's waiter list
    ///   without a bounds check, on the strength of this.
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

impl<'d, M: Mode> embedded_hal::digital::ErrorType for Flex<'d, M> {
    type Error = Infallible;
}

impl<'d, M: Mode> embedded_hal::digital::InputPin for Flex<'d, M> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d, M: Mode> embedded_hal::digital::OutputPin for Flex<'d, M> {
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }
}

impl<'d, M: Mode> embedded_hal::digital::StatefulOutputPin for Flex<'d, M> {
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
impl<'d> embedded_hal_async::digital::Wait for Flex<'d, Async> {
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

impl<'d, M: Mode> embedded_hal::digital::ErrorType for Input<'d, M> {
    type Error = Infallible;
}

impl<'d, M: Mode> embedded_hal::digital::InputPin for Input<'d, M> {
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
impl<'d> embedded_hal_async::digital::Wait for Input<'d, Async> {
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

impl<'d, M: Mode> embedded_hal::digital::ErrorType for OutputOpenDrain<'d, M> {
    type Error = Infallible;
}

impl<'d, M: Mode> embedded_hal::digital::InputPin for OutputOpenDrain<'d, M> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d, M: Mode> embedded_hal::digital::OutputPin for OutputOpenDrain<'d, M> {
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_low())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.set_high())
    }
}

impl<'d, M: Mode> embedded_hal::digital::StatefulOutputPin for OutputOpenDrain<'d, M> {
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
impl<'d> embedded_hal_async::digital::Wait for OutputOpenDrain<'d, Async> {
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

#[cfg_attr(unicomm, allow(dead_code))]
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
        // The whole register rather than a read-modify-write of three fields. `PC` is what connects the
        // pad to the peripheral at all, and leaving it set left a dropped driver still driving the pin;
        // `INENA`, `INV` and `HIZ1` have no business surviving either. TI's own routine is the same
        // single store of zero, and a store is smaller than the read-modify-write it replaces.
        pac::IOMUX
            .pincm(self._pin_cm() as usize)
            .write_value(pac::iomux::regs::Pincm(0));
    }

    #[cfg_attr(unicomm, allow(dead_code))]
    fn update_pf(&self, ty: PfType) {
        let pincm = pac::IOMUX.pincm(self._pin_cm() as usize);
        let pf = pincm.read().pf();

        set_pf(self._pin_cm() as usize, pf, ty);
    }

    #[cfg_attr(unicomm, allow(dead_code))]
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
    #[cfg_attr(unicomm, allow(dead_code))]
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
    // Opened as the handler's first action, so the rising edge timestamps the silicon wake plus the
    // interrupt entry and group dispatch, with none of this handler in it. Both markers are resolved
    // here so that neither bracket pays for a lookup.
    #[cfg(feature = "_probe")]
    let (handler_marker, waker_marker) = {
        use crate::probe::{Marker, target};

        let markers = (target(Marker::GpioHandler), target(Marker::GpioWaker));
        crate::probe::set(markers.0);
        markers
    };

    // One snapshot for all of them: the status bit carries no direction, so the level a pin settled
    // at is the only thing an edge can be classified by. Taken before the loop, so every pin in this
    // handler is classified against the same instant.
    //
    // A volatile read survives dead-code elimination, so where nothing classifies edges it is skipped
    // rather than read and ignored.
    let level = if DETECT_BOTH_EDGES {
        gpio.din31_0().read()
    } else {
        gpio::regs::Dio(0)
    };

    // `IIDX` answers "which pin, and clear it" in one read, which is what the hardware is for: SLAU846
    // 9.3.10 has it return the lowest set *enabled* status bit, clear that bit in `RIS` and `MIS`, and
    // present the next one, reading zero when none are left. That is this loop's condition, its index
    // and its `ICLR` write all at once, and it costs no bit scan — `u32::trailing_zeros` has no
    // instruction behind it on ARMv6-M and lowers to a multiply and a 32-byte table.
    loop {
        // Taken as bits rather than through the generated enum: the index is what is wanted, the enum
        // has a variant per pin, and zero-means-none is the register's own definition.
        let stat = gpio.cpu_int().iidx().read().stat().to_bits();

        if stat == 0 {
            break;
        }

        // Indices are one-based, zero having been spent on "nothing pending".
        //
        // Masked because `stat` is an 8-bit field the compiler cannot bound, so the classification below
        // and the `imask` write would each carry the metapac's bounds assert and a panic site into the
        // handler. `IIDX.STAT` encodes 0x00-0x20 (SLAU846 9.3.10), so the mask never changes a value the
        // hardware can produce.
        //
        // `assert_unchecked` is the cheaper-looking option and is **measurably worse here**: stated over
        // `stat`, over `bit`, or over both, one of the two bounds checks survives and the binary comes
        // out 12 bytes larger than this. The `and` costs two bytes and pays for itself in what it folds.
        let bit = (stat as usize - 1) & 31;

        // SAFETY: this is the port's own interrupt handler, which is the only walker allowed.
        let waiter = unsafe { WAITERS[port as usize].find(|wait| usize::from(wait.bit) == bit) };

        // An edge the other way leaves the wait standing, so it continues without the task ever being
        // woken. Skipped where `POLARITY` did the filtering, since the level can have moved on since the
        // edge and would only be a chance to classify it wrongly.
        if let Some(waiter) = waiter
            && DETECT_BOTH_EDGES
            && !waiter.state.edge.accepts(level.dio(bit))
        {
            continue;
        }

        #[cfg(feature = "_probe")]
        crate::probe::set(waker_marker);

        // A pin with no waiter is one whose wait was dropped between the edge and here. Nothing to
        // report, and the mask below is what stops it arriving again.
        if let Some(waiter) = waiter {
            waiter.complete();
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

/// Interrupt handler for a GPIO port.
///
/// One type serves every port: which port a call is for follows from the interrupt it was bound to.
///
/// Bind it with [`bind_group_interrupts!`](crate::bind_group_interrupts) on the chips where the
/// ports share an interrupt group, and with [`bind_interrupts!`](crate::bind_interrupts) on the
/// ones where a port owns an NVIC line — which is which is fixed per chip, and a binding written
/// for the wrong one will not name a type that exists.
pub struct InterruptHandler {
    _private: (),
}

/// Proof that every GPIO port on the chip is bound to [`InterruptHandler`].
///
/// Required by [`Flex::new_async`] and the other `new_async` constructors, and is what keeps the
/// handler out of a binary that never waits on an edge.
///
/// All of them, rather than the pin's own, because a pin's port is not in its type — it is a
/// run-time read. A wait armed on a port whose handler was never installed would never complete.
///
/// # Safety
///
/// The blanket implementation over the interrupt bindings is the only one, so this cannot be
/// implemented by hand.
pub unsafe trait PortInterrupts {}

#[cfg(all(gpioa_interrupt, gpioa_group))]
compile_error!("gpioa_interrupt and gpioa_group are mutually exclusive cfgs");
#[cfg(all(gpiob_interrupt, gpiob_group))]
compile_error!("gpiob_interrupt and gpiob_group are mutually exclusive cfgs");

// C110x and L110x have a dedicated interrupt just for GPIOA, and no GROUP1 at all. Everywhere else a
// port is one source of an interrupt group, which is a different trait to implement even though the
// symbol the binding defines has the same name.
#[cfg(all(feature = "rt", gpioa_interrupt))]
impl crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::GPIOA> for InterruptHandler {
    unsafe fn on_interrupt() {
        irq_handler(pac::GPIOA, Port::PortA);
    }
}

#[cfg(all(feature = "rt", gpiob_interrupt))]
impl crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::GPIOB> for InterruptHandler {
    unsafe fn on_interrupt() {
        irq_handler(pac::GPIOB, Port::PortB);
    }
}

#[cfg(all(feature = "rt", gpioa_group))]
impl crate::interrupt_group::Handler<crate::interrupt_group::GPIOA> for InterruptHandler {
    unsafe fn on_interrupt() {
        irq_handler(pac::GPIOA, Port::PortA);
    }
}

#[cfg(all(feature = "rt", gpiob_group))]
impl crate::interrupt_group::Handler<crate::interrupt_group::GPIOB> for InterruptHandler {
    unsafe fn on_interrupt() {
        irq_handler(pac::GPIOB, Port::PortB);
    }
}

#[cfg(all(feature = "rt", gpioc_group))]
impl crate::interrupt_group::Handler<crate::interrupt_group::GPIOC> for InterruptHandler {
    unsafe fn on_interrupt() {
        irq_handler(pac::GPIOC, Port::PortC);
    }
}
