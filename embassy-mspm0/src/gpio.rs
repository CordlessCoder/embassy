//! General-purpose I/O.
//!
//! [`Input`], [`Output`], [`OutputOpenDrain`] and [`Flex`], which is all three at once. Everything
//! except `Output` takes a mode parameter defaulting to [`Blocking`]; `Input<'d>` still names the
//! blocking one, so code that only reads and writes pins needs no change.
//!
//! # Waiting on an edge
//!
//! The waits live on the `Async` flavour, built with `new_async` and an interrupt binding. **Every
//! port on the chip has to be bound, not just the pin's own** — a pin's port is a run-time value, not
//! part of its type, so nothing else can rule out a wait armed on a port with no handler installed.
//!
//! Which macro binds it is fixed per chip: a port is either a source on an interrupt group, wanting
//! [`bind_group_interrupts!`](crate::bind_group_interrupts), or the owner of an NVIC line, wanting
//! [`bind_interrupts!`](crate::bind_interrupts). There is one handler type and it implements both
//! traits under the matching cfg, so a binding written for the wrong one names a type that does not
//! exist rather than silently linking nothing.

#![macro_use]

use core::convert::Infallible;
#[cfg(feature = "rt")]
use core::future::Future;
use core::marker::PhantomData;
#[cfg(feature = "rt")]
use core::marker::PhantomPinned;
#[cfg(feature = "rt")]
use core::task::{Context, Poll, Waker};

use critical_section::CriticalSection;
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
    ///
    /// **Not every pin has one.** The 5 V tolerant open-drain structure implements a pulldown and no
    /// pullup, so `PIPU` on one of those pins is accepted, reads back, and does nothing — the pin
    /// floats. That structure is `PA0` and `PA1` on every family except the H321x, which has no
    /// open-drain pins at all.
    ///
    /// A `debug_assert!` catches it, so a debug build panics and a release build does not. The
    /// choice is deliberate: a pin that cannot be pulled up is visible from the datasheet and is
    /// normally settled when the schematic is drawn, so the cost of finding out belongs at
    /// development time rather than in the field.
    ///
    /// An external pullup is the fix, and is what those pins expect — being 5 V tolerant, they are
    /// meant for a bus that supplies its own.
    Up,

    /// Internal pull-down resistor. Every structure has one.
    Down,
}

/// Whether `pincm`'s IO structure implements a pullup.
///
/// The table it reads exists only for the `debug_assert!` below, so both vanish in a release build.
#[inline]
fn has_pullup(pincm: u8) -> bool {
    !crate::_generated::PINS_WITHOUT_PULLUP.contains(&pincm)
}

/// Whether `pincm`'s IO structure implements `PINCM.DRV`.
#[inline]
fn has_drive_strength(pincm: u8) -> bool {
    crate::_generated::PINS_WITH_DRIVE_STRENGTH.contains(&pincm)
}

/// Whether `pincm`'s IO structure implements `PINCM.HYSTEN`.
#[inline]
fn has_hysteresis(pincm: u8) -> bool {
    crate::_generated::PINS_WITH_HYSTERESIS.contains(&pincm)
}

/// How hard an output drives.
///
/// **Only the high-drive and high-speed structures have this**, which is a handful of pins per
/// device — SLAU846 8.2.6 says outright that "drive strength control is not available for standard
/// drive and open drain IO types". Asking for it elsewhere is accepted and does nothing, so
/// [`Flex::set_drive_strength`] catches that in a debug build.
///
/// Independent of the peripheral function selected on the pin, and changeable at any time.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DriveStrength {
    /// The reset default, and all a standard-drive pin can do.
    Low,

    /// The high-drive output, which the datasheet's Digital IO section specifies per device — 20 mA
    /// on the parts that describe it that way.
    High,
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
/// interrupt handler behind it, so it lives on [`Flex<Async>`] and `Flex::new_async`, which asks
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

    /// Latch this pin's edges and let them reach the CPU.
    ///
    /// For an application that writes its own handler. Selects the edges, drops any latched before the
    /// call, arms `FASTWAKE` so the input synchroniser stays clocked in STOP and STANDBY, and unmasks
    /// the pin. **The port's NVIC line is not touched**: that belongs to whoever owns the vector, and
    /// [`Config::interrupts`](crate::Config::interrupts) says who.
    ///
    /// [`Flex<Async>`](Flex) is the other way to use these edges, and the two do not mix on one pin.
    ///
    /// # Servicing it alongside the async waits
    ///
    /// A handler that also dispatches to this crate's own edge waits must deal with its own pins
    /// **first**, and clear each one before forwarding. `IIDX` reports the lowest set *enabled* status
    /// bit and clears it as it is read, so a pin still pending when the demultiplexer runs is taken for
    /// one of its own: the edge is consumed and the pin is left masked for good.
    ///
    /// ```rust,ignore
    /// #[task(binds = GROUP1, local = [button])]
    /// fn on_group1(cx: on_group1::Context) {
    ///     if cx.local.button.take_active() {
    ///         // ...
    ///     }
    ///
    ///     unsafe { Irqs::GROUP1() }
    /// }
    /// ```
    ///
    /// # Both directions, on some devices
    ///
    /// Where `GPIO_ERR_01` applies both edges are latched whatever `edge` asks for, because its case 2
    /// otherwise loses every STANDBY1 wake after the first. Classify with [`Flex::get_level`] in the
    /// handler rather than trusting the selection.
    pub fn enable_interrupt(&mut self, edge: Edge) {
        let block = self.pin.block();
        let bit = self.pin.bit_index();

        let polarity = if DETECT_BOTH_EDGES {
            Polarity::RiseFall
        } else {
            edge.polarity()
        };

        // One section for the lot: every write below is either a read-modify-write of a register
        // shared with the port's other pins, or one the handler must not see half of.
        critical_section::with(|cs| {
            set_polarity(cs, block, bit, polarity);

            // After the polarity write, so selecting the event cannot leave a status bit behind.
            block.cpu_int().iclr().write(|w| w.set_dio(bit, true));

            block.fastwake().modify(|w| w.set_din(bit, true));

            // Last, so nothing is delivered before the pin is fully set up.
            block.cpu_int().imask().modify(|w| w.set_dio(bit, true));
        });
    }

    /// Stop this pin's edges reaching the CPU, and drop any already latched.
    ///
    /// Leaves the selected edges and `FASTWAKE` alone, so a later [`Flex::enable_interrupt`] with the
    /// same edge is just the unmask.
    pub fn disable_interrupt(&mut self) {
        let block = self.pin.block();
        let bit = self.pin.bit_index();

        critical_section::with(|_cs| {
            block.cpu_int().imask().modify(|w| w.set_dio(bit, false));
            block.cpu_int().iclr().write(|w| w.set_dio(bit, true));
        });
    }

    /// Whether an edge is latched for this pin, whether or not it is unmasked.
    ///
    /// Reads `RIS`, so it neither clears the pin nor disturbs the port's `IIDX` ordering.
    #[inline]
    pub fn is_pending(&self) -> bool {
        self.pin.is_pending()
    }

    /// Drop a latched edge without acting on it.
    #[inline]
    pub fn clear_pending(&mut self) {
        self.pin.clear_pending();
    }

    /// Whether an edge is latched, clearing it.
    ///
    /// What a handler wants: reading and clearing separately drops an edge that arrives between the
    /// two, where this reports it on the next entry.
    #[inline]
    pub fn take_active(&mut self) -> bool {
        self.pin.take_active()
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
    ///
    /// Panics in a debug build if `pull` is [`Pull::Up`] on a pin that has no pullup — see that
    /// variant for which pins those are and why this is not a release-build check.
    #[inline]
    pub fn set_pull(&mut self, pull: Pull) {
        debug_assert!(
            pull != Pull::Up || has_pullup(self.pin.pin_cm()),
            "this pin's IO structure has no pullup, so Pull::Up would do nothing"
        );

        let pincm = pac::IOMUX.pincm(self.pin.pin_cm() as usize);

        pincm.modify(|w| {
            w.set_pipd(matches!(pull, Pull::Down));
            w.set_pipu(matches!(pull, Pull::Up));
        });
    }

    /// Set how hard the pin drives when it is an output.
    ///
    /// Panics in a debug build on a pin whose IO structure has no drive-strength control, which is
    /// every one but high-drive and high-speed — see [`DriveStrength`].
    #[inline]
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        debug_assert!(
            has_drive_strength(self.pin.pin_cm()),
            "this pin's IO structure has no drive strength control, so setting it would do nothing"
        );

        pac::IOMUX
            .pincm(self.pin.pin_cm() as usize)
            .modify(|w| w.set_drv(matches!(strength, DriveStrength::High)));
    }

    /// Turn input hysteresis on or off.
    ///
    /// Panics in a debug build on a pin whose IO structure has no hysteresis control. Only the 5 V
    /// tolerant open-drain structure has it, which is two pins on most devices and none on some.
    ///
    /// It is on by default there, and turning it off is what a fast edge from a clean driver wants;
    /// leave it on for anything slow or noisy.
    #[inline]
    pub fn set_hysteresis(&mut self, enable: bool) {
        debug_assert!(
            has_hysteresis(self.pin.pin_cm()),
            "this pin's IO structure has no hysteresis control, so setting it would do nothing"
        );

        pac::IOMUX
            .pincm(self.pin.pin_cm() as usize)
            .modify(|w| w.set_hysten(enable));
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

    /// Hand the pin to a peripheral function, and stop driving it from here.
    ///
    /// The counterpart to [`set_as_output`](Self::set_as_output) and friends, for a pin that has to
    /// change role while the program runs — a pin wired to both a timer output and something the
    /// application drives directly, say. Taking it back is [`set_as_output`](Self::set_as_output) or
    /// [`set_as_input`](Self::set_as_input); this driver keeps ownership either way, so the pin
    /// cannot be handed to two peripherals at once.
    ///
    /// `pf` is the function number for the peripheral and signal wanted, which the peripheral's own
    /// pin trait knows: [`TimerPin::pf_num`](crate::tim::TimerPin::pf_num) and the equivalents on the
    /// other drivers' pin traits report it, so it does not have to be written out. A number that does
    /// not name a function on this pin selects nothing and the pin goes quiet, which is the usual
    /// silent failure here.
    #[inline]
    pub fn set_as_af(&mut self, pf: u8, ty: PfType) {
        self.pin.set_as_pf(pf, ty);
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

    // Four `PINCM` fields are still unexposed, and none of them is the ordinary pin setting it looks
    // like. SLAU846 table 8-1 gives the features per IO structure, and they do not overlap:
    //
    // - `DRV`, drive strength, exists only on the high-drive and high-speed types. The TRM is explicit
    //   that "drive strength control is not available for standard drive and open drain IO types".
    // - `HYSTEN`, hysteresis, exists only on the 5 V tolerant open-drain type, and on nothing else.
    // - `WUEN`/`WCOMP`, the wake logic, is on the two "with wake" variants, high-drive and open drain.
    //   `build.rs` already generates `impl_wake_capable_pin!` for this one, from the metadata's
    //   `io_wakeup`, so it is the only one of the four whose capability is answerable today.
    //
    // Writing any of them on a pin whose structure lacks it is accepted and does nothing, which is the
    // failure this crate keeps meeting: a setting that silently is not applied reads as a working
    // configuration. So the first three want the pin's IO structure in the device metadata, which the
    // pinned revision does not carry -- `Pin` has `pin`, `pincm` and `wakeup` and no type. Raised as a
    // metapac request rather than guessed at from a pin-name list.
    //
    // The same table says the open-drain type has no pullup at all, which `Pull` does not know either.
    // Worth checking against a device that has such pins before treating it as a defect.

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
            w.set_dio(self.pin.bit_index(), true);
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
    ///
    #[inline]
    pub async fn wait_for_high(&mut self) {
        if self.is_high() {
            return;
        }

        // Not `wait_for_rising_edge`: this promises a level, so the wait has to survive the pin going
        // high between the test above and the edge detector being armed. `Park` re-tests it there.
        //
        // Testing it in `Park` instead, which would make this a plain `fn`, was measured and is not
        // worth it: the test cannot fold away for the edge waits sharing that future, so it costs
        // 40 bytes in every binary that only waits on edges to save 44 in one that waits on a level.
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

    /// Wait for the pin to undergo a transition, choosing which one at run time.
    ///
    /// The three fixed-edge waits each return a distinct opaque future, so a caller selecting
    /// between them has to wrap the choice in an `async` block — a state machine around a future
    /// that needs none. Selecting the [`Edge`] instead keeps one future type.
    #[inline]
    pub fn wait_for_edge(&mut self, edge: Edge) -> impl Future<Output = ()> {
        self.wait_inner(edge, None)
    }

    fn wait_inner(&mut self, edge: Edge, settled_high: Option<bool>) -> Park {
        // Described here, armed from inside the first poll, where the waker exists — so that the
        // registration and the unmask are one critical section and no edge can land between them.
        Park {
            arm: EdgeArm::new(self.pin.block(), self.pin.pin_port(), edge, settled_high),
            _pin: PhantomPinned,
        }
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
/// **Not `async fn` and not a `poll_fn`.** [`EdgeArm::arm`] publishes the address of a field of `arm`
/// into the port's waiter list, so this future may not move between its first poll and its drop.
/// Written as a generator that guarantee came free, at the price of a discriminant, a resume switch and
/// a nested future; written as a `poll_fn` owning the arm it would be lost, `PollFn` being `Unpin`
/// whenever its closure is. [`PhantomPinned`] is what states it instead.
#[cfg(feature = "rt")]
struct Park {
    arm: EdgeArm,
    _pin: PhantomPinned,
}

#[cfg(feature = "rt")]
impl Future for Park {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        // SAFETY: nothing below moves out of the future or hands out a `&mut` that could, so the
        // address `arm` publishes stays the address it keeps until `EdgeArm::drop` withdraws it.
        let arm = unsafe { &mut self.get_unchecked_mut().arm };

        if !arm.armed {
            arm.arm(cx.waker());

            if let Some(high) = arm.settled_high
                && arm.is_high() == high
            {
                return Poll::Ready(());
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
    }
}

/// Adapts an infallible wait to the `Result` `embedded_hal_async` asks for.
///
/// An `async` block would do the same and cost a state machine per future — a discriminant, a resume
/// switch, and the inner future inside it. This is laid out as the future it holds and its `poll` is
/// the inner `poll` plus a tag.
#[cfg(feature = "rt")]
struct AlwaysOk<F>(F);

#[cfg(feature = "rt")]
impl<F: Future<Output = ()>> Future for AlwaysOk<F> {
    type Output = Result<(), core::convert::Infallible>;

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Self::Output> {
        // SAFETY: a structural projection to the only field. `AlwaysOk` is never unpinned, moved
        // out of, or given a `Drop`, so the inner future stays pinned for as long as this is.
        let inner = unsafe { self.map_unchecked_mut(|this| &mut this.0) };

        inner.poll(cx).map(Ok)
    }
}

/// Whether `GPIO_ERR_01` applies, which forces both directions to be detected.
///
/// Its case 2 loses every STANDBY1 wake after the first unless the pin detects both edges, so where it
/// applies the direction is filtered in software instead.
const DETECT_BOTH_EDGES: bool = cfg!(gpio_err_01);

/// Which edge to detect.
///
/// Pass this to [`Flex::wait_for_edge`] where the edge is chosen at run time, or to
/// [`Flex::enable_interrupt`] where the handler is the application's own. The three `wait_for_*_edge`
/// methods are the same wait with the edge fixed, and cost a caller who knows it at compile time
/// nothing extra.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Edge {
    /// A transition from low to high.
    Rising,
    /// A transition from high to low.
    Falling,
    /// Either transition.
    Any,
}

impl Edge {
    fn polarity(self) -> Polarity {
        match self {
            Edge::Rising => Polarity::Rise,
            Edge::Falling => Polarity::Fall,
            Edge::Any => Polarity::RiseFall,
        }
    }

    /// Whether an edge that left the pin reading `level` is one this wait asked for.
    #[cfg(feature = "rt")]
    fn accepts(self, level: bool) -> bool {
        match self {
            Edge::Rising => level,
            Edge::Falling => !level,
            Edge::Any => true,
        }
    }
}

/// Select which edges a pin latches, leaving everything else about it alone.
///
/// Both halves hold sixteen two-bit fields in a `u32`, and the metapac gives them separate types, so
/// writing through those would emit this read-modify-write twice. Choosing the register first and
/// editing the field by hand emits it once.
///
/// The section is a parameter because this is a read-modify-write of a register the port's other pins
/// share, so it is a requirement rather than a convention.
fn set_polarity(_cs: CriticalSection, block: gpio::Gpio, bit: usize, polarity: Polarity) {
    let polarity_reg = if bit >= 16 {
        block.polarity31_16().as_ptr() as *mut u32
    } else {
        block.polarity15_0().as_ptr() as *mut u32
    };
    let shift = (bit % 16) * 2;

    // SAFETY: the pointer is one of this block's own polarity registers, and the caller's critical
    // section keeps the read-modify-write whole against the port's interrupt.
    unsafe {
        let polarity_bits = polarity_reg.read_volatile() & !(0b11 << shift);
        polarity_reg.write_volatile(polarity_bits | ((polarity as u32) << shift));
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
    /// Here rather than in [`Park`] because `bit` and `port` leave padding before `waiter`, so this
    /// rides along free; in the future it grew the task frame for every edge wait too.
    settled_high: Option<bool>,
    /// Whether [`EdgeArm::arm`] has run, which is what [`Park`] tests to tell its first poll from the
    /// rest and what [`EdgeArm::drop`] tests to tell whether there is anything to undo.
    ///
    /// A `Park` is built before it is polled and may be dropped without ever being, so the drop cannot
    /// assume the pin was ever unmasked. Same padding as `settled_high`.
    armed: bool,
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
            armed: false,
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
    fn arm(&mut self, waker: &Waker) {
        // Before the section: a `Waker`'s clone is someone else's code, and it has no business running
        // with interrupts off.
        let parked = waker.clone();
        self.armed = true;

        let polarity = if DETECT_BOTH_EDGES {
            Polarity::RiseFall
        } else {
            self.waiter.state.edge.polarity()
        };

        critical_section::with(|cs| {
            set_polarity(cs, self.block, self.bit(), polarity);

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
        // A wait built and dropped without ever being polled has nothing to undo, and clearing the
        // pin's status here would throw away an edge the pin is not even unmasked for.
        if !self.armed {
            return;
        }

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
/// handler behind it, so it lives on [`Input<Async>`] and `Input::new_async`, which asks for the
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

    /// Latch this pin's edges and let them reach the CPU. See [`Flex::enable_interrupt`], which this
    /// forwards to — including what it says about servicing a pin alongside the async waits.
    #[inline]
    pub fn enable_interrupt(&mut self, edge: Edge) {
        self.pin.enable_interrupt(edge);
    }

    /// Stop this pin's edges reaching the CPU. See [`Flex::disable_interrupt`].
    #[inline]
    pub fn disable_interrupt(&mut self) {
        self.pin.disable_interrupt();
    }

    /// Whether an edge is latched for this pin. See [`Flex::is_pending`].
    #[inline]
    pub fn is_pending(&self) -> bool {
        self.pin.is_pending()
    }

    /// Drop a latched edge without acting on it. See [`Flex::clear_pending`].
    #[inline]
    pub fn clear_pending(&mut self) {
        self.pin.clear_pending();
    }

    /// Whether an edge is latched, clearing it. See [`Flex::take_active`].
    #[inline]
    pub fn take_active(&mut self) -> bool {
        self.pin.take_active()
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
    pub fn wait_for_high(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_high()
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub fn wait_for_low(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_low()
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub fn wait_for_rising_edge(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_rising_edge()
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub fn wait_for_falling_edge(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_falling_edge()
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub fn wait_for_any_edge(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_any_edge()
    }

    /// Wait for the pin to undergo a transition, choosing which one at run time.
    #[inline]
    pub fn wait_for_edge(&mut self, edge: Edge) -> impl Future<Output = ()> {
        self.pin.wait_for_edge(edge)
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

impl Output<'_> {
    /// Set how hard the pin drives. See [`Flex::set_drive_strength`].
    #[inline]
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive_strength(strength);
    }
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
    /// Logic inversion applies to the output path of this pin.
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
/// `OutputOpenDrain::new_async`, which asks for the binding that installs one.
pub struct OutputOpenDrain<'d, M: Mode = Blocking> {
    pin: Flex<'d, M>,
}

impl<M: Mode> OutputOpenDrain<'_, M> {
    /// Turn input hysteresis on or off. See [`Flex::set_hysteresis`].
    ///
    /// This is the structure that has it, so the check behind that method never fires here.
    #[inline]
    pub fn set_hysteresis(&mut self, enable: bool) {
        self.pin.set_hysteresis(enable);
    }
}

impl<'d> OutputOpenDrain<'d, Blocking> {
    /// Create a new GPIO open drain output driver for a [Pin] with the provided [Level].
    #[inline]
    pub fn new(pin: Peri<'d, impl Pin>, initial_output: Level) -> Self {
        Self {
            pin: Self::configure(Flex::new(pin), initial_output),
        }
    }

    /// Latch this pin's edges and let them reach the CPU. See [`Flex::enable_interrupt`], which this
    /// forwards to — including what it says about servicing a pin alongside the async waits.
    #[inline]
    pub fn enable_interrupt(&mut self, edge: Edge) {
        self.pin.enable_interrupt(edge);
    }

    /// Stop this pin's edges reaching the CPU. See [`Flex::disable_interrupt`].
    #[inline]
    pub fn disable_interrupt(&mut self) {
        self.pin.disable_interrupt();
    }

    /// Whether an edge is latched for this pin. See [`Flex::is_pending`].
    #[inline]
    pub fn is_pending(&self) -> bool {
        self.pin.is_pending()
    }

    /// Drop a latched edge without acting on it. See [`Flex::clear_pending`].
    #[inline]
    pub fn clear_pending(&mut self) {
        self.pin.clear_pending();
    }

    /// Whether an edge is latched, clearing it. See [`Flex::take_active`].
    #[inline]
    pub fn take_active(&mut self) -> bool {
        self.pin.take_active()
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
    /// One control serves both directions, so this inverts what the pin drives and what it reads back.
    #[inline]
    pub fn set_inversion(&mut self, invert: bool) {
        self.pin.set_inversion(invert)
    }
}

#[cfg(feature = "rt")]
impl<'d> OutputOpenDrain<'d, Async> {
    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub fn wait_for_high(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_high()
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub fn wait_for_low(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_low()
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub fn wait_for_rising_edge(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_rising_edge()
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub fn wait_for_falling_edge(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_falling_edge()
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub fn wait_for_any_edge(&mut self) -> impl Future<Output = ()> {
        self.pin.wait_for_any_edge()
    }

    /// Wait for the pin to undergo a transition, choosing which one at run time.
    #[inline]
    pub fn wait_for_edge(&mut self, edge: Edge) -> impl Future<Output = ()> {
        self.pin.wait_for_edge(edge)
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
    /// - `pin_port` should not be in use by another driver, with one exception: a handle taken only
    ///   to reach [`is_pending`](Self::is_pending), [`clear_pending`](Self::clear_pending) or
    ///   [`take_active`](Self::take_active) may name a pin a driver already holds. Those three
    ///   touch `RIS`, which is read-only, and `ICLR`, which is write-one-to-clear — neither is a
    ///   read-modify-write, so neither can disturb the port's other pins or a configuration write
    ///   the owning driver is making. Nothing else here is safe to alias, and in particular a second
    ///   handle must not be turned into a driver: [`Flex`]'s `Drop` disconnects the pin.
    /// - `pin_port` must name a pin this chip has. Two things rest on it: the edge waits index their
    ///   port's waiter list without a bounds check, and `gpio_pincm` tells the optimiser its `match`
    ///   is exhaustive over the real pins. A value outside them is undefined behaviour, not a panic.
    #[inline]
    pub unsafe fn steal(pin_port: u8) -> Peri<'static, Self> {
        Peri::new_unchecked(Self { pin_port })
    }

    /// Whether an edge is latched for this pin, whether or not it is unmasked.
    ///
    /// Reads `RIS`, so it neither clears the pin nor disturbs the port's `IIDX` ordering.
    ///
    /// # For a handler that owns no driver
    ///
    /// These three are on the pin rather than on [`Flex`] so that an interrupt handler can reach
    /// them. A handler bound to a group source through
    /// [`bind_group_interrupts!`](crate::bind_group_interrupts)'s `unsafe struct` arm has no driver
    /// to call — the application holds it — and acknowledging the *group* does not clear the
    /// *source*: the port's `RIS` bit survives it and the line asserts again immediately. Take an
    /// [`AnyPin::steal`] handle for the pin and clear it here.
    #[inline]
    pub fn is_pending(&self) -> bool {
        self.block().cpu_int().ris().read().dio(self.bit_index())
    }

    /// Drop a latched edge without acting on it.
    ///
    /// See [`AnyPin::is_pending`] for reaching this from a handler.
    #[inline]
    pub fn clear_pending(&self) {
        self.block()
            .cpu_int()
            .iclr()
            .write(|w| w.set_dio(self.bit_index(), true));
    }

    /// Whether an edge is latched, clearing it.
    ///
    /// What a handler wants: reading and clearing separately drops an edge that arrives between the
    /// two, where this reports it on the next entry.
    ///
    /// See [`AnyPin::is_pending`] for reaching this from a handler, and
    /// [`Flex::enable_interrupt`] for the order to service it in when this crate's own edge waits
    /// share the port.
    #[inline]
    pub fn take_active(&self) -> bool {
        let block = self.block();
        let bit = self.bit_index();

        critical_section::with(|_cs| {
            let pending = block.cpu_int().ris().read().dio(bit);

            if pending {
                block.cpu_int().iclr().write(|w| w.set_dio(bit, true));
            }

            pending
        })
    }
}

/// An optional [`AnyPin`], in one byte.
///
/// `Option<Peri<'d, AnyPin>>` costs **two** bytes to carry one byte of payload: every `u8` is a valid
/// `pin_port` as far as the compiler knows, so `Option` has no niche to use and adds a discriminant.
/// A driver holding four of them — the three timer drivers and the buffered UART each do — pays four
/// bytes for nothing. This spends the one `pin_port` value no chip can produce instead.
///
/// **Why a sentinel rather than a niche on `AnyPin` itself.** Biasing `pin_port` by one so that zero
/// becomes free was tried first and is a trap: `pin_cm` feeds `pin_port` into `gpio_pincm`, a generated
/// `match` over every pin, which LLVM emits as a **240-byte lookup table** in `.rodata`. The subtract
/// the bias adds costs it the range knowledge that table depends on, and the match expands into a
/// comparison chain — measured at **+384 bytes of `.text` to save 240 of `.rodata`**. A sentinel keeps
/// `pin_port` exactly what it was, so the table survives.
pub(crate) struct MaybeAnyPin<'d> {
    /// `port * 32 + bit`, or [`MaybeAnyPin::NONE`].
    pin_port: u8,
    /// Owns the pin for `'d`, as the `Peri` this replaces did.
    _lifetime: PhantomData<&'d mut AnyPin>,
}

impl<'d> MaybeAnyPin<'d> {
    /// No pin. `port * 32 + bit` reaches 95 on the widest part, so this is unreachable by construction.
    const NONE: u8 = u8::MAX;

    /// Take ownership of `pin`, if there is one.
    #[inline]
    pub(crate) fn new(pin: Option<Peri<'d, AnyPin>>) -> Self {
        Self {
            pin_port: match pin {
                Some(pin) => pin.pin_port(),
                None => Self::NONE,
            },
            _lifetime: PhantomData,
        }
    }

    /// No pin.
    #[inline]
    pub(crate) const fn none() -> Self {
        Self {
            pin_port: Self::NONE,
            _lifetime: PhantomData,
        }
    }

    /// Give the pin back, so a driver can hand it to its caller.
    #[inline]
    pub(crate) fn into_peri(self) -> Option<Peri<'d, AnyPin>> {
        let pin_port = self.pin_port;

        // SAFETY: `self` owned this pin for `'d` and is consumed here, so the token is moved rather
        // than duplicated.
        self.is_some()
            .then(|| unsafe { Peri::new_unchecked(AnyPin { pin_port }) })
    }

    /// Borrow the pin for a shorter lifetime, as `Peri::reborrow` does.
    ///
    /// Only the DMA and buffered-UART drivers split a driver in two, and a UNICOMM device has
    /// neither, so on those there is no caller.
    #[inline]
    #[cfg_attr(unicomm, allow(dead_code))]
    pub(crate) fn reborrow(&mut self) -> MaybeAnyPin<'_> {
        MaybeAnyPin {
            pin_port: self.pin_port,
            _lifetime: PhantomData,
        }
    }

    /// Whether a pin was given.
    #[inline]
    pub(crate) fn is_some(&self) -> bool {
        self.pin_port != Self::NONE
    }

    /// The pin, for the register-level methods on [`SealedPin`].
    ///
    /// Handed back by value rather than by reference because [`AnyPin`] is one byte and every method
    /// on it only reads that byte. The copy does not duplicate ownership in any way that matters:
    /// `self` still owns the pin for `'d`, and the result cannot outlive the borrow.
    #[inline]
    pub(crate) fn pin(&self) -> Option<AnyPin> {
        self.is_some().then_some(AnyPin {
            pin_port: self.pin_port,
        })
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
        self.set_low();
        Ok(())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
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
    fn wait_for_high(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_high())
    }

    fn wait_for_low(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_low())
    }

    fn wait_for_rising_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_rising_edge())
    }

    fn wait_for_falling_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_falling_edge())
    }

    fn wait_for_any_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_any_edge())
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
    fn wait_for_high(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_high())
    }

    fn wait_for_low(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_low())
    }

    fn wait_for_rising_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_rising_edge())
    }

    fn wait_for_falling_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_falling_edge())
    }

    fn wait_for_any_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_any_edge())
    }
}

impl<'d> embedded_hal::digital::ErrorType for Output<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::OutputPin for Output<'d> {
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
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
        self.set_low();
        Ok(())
    }

    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
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
    fn wait_for_high(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_high())
    }

    fn wait_for_low(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_low())
    }

    fn wait_for_rising_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_rising_edge())
    }

    fn wait_for_falling_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_falling_edge())
    }

    fn wait_for_any_edge(&mut self) -> impl Future<Output = Result<(), Self::Error>> {
        AlwaysOk(self.wait_for_any_edge())
    }
}

/// How the pad is configured while a peripheral function drives it.
///
/// The function number decides which peripheral reaches the pin; this decides the direction, the
/// pull and whether the signal is inverted on the way through.
#[cfg_attr(unicomm, allow(dead_code))]
#[derive(Copy, Clone)]
pub struct PfType {
    pull: Pull,
    input: bool,
    invert: bool,
}

impl PfType {
    /// The peripheral reads the pin.
    pub const fn input(pull: Pull, invert: bool) -> Self {
        Self {
            pull,
            input: true,
            invert,
        }
    }

    /// The peripheral drives the pin.
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
/// Distinct from waking the device at all: `FASTWAKE`, which `Flex::wait_for_any_edge` and friends
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
        impl crate::peripherals::$name {
            /// What [`AnyPin::steal`](crate::gpio::AnyPin::steal) names this pin by.
            ///
            /// A handler that has to reach a pin it does not own needs the number, and the pin's own
            /// driver is usually the only thing holding a handle. Writing `PA9::PIN_PORT` rather
            /// than `9` puts the pin type in the caller's source, so moving the function to another
            /// pin, or building for a package that does not bring this one out, is a compile error
            /// instead of a handler that quietly services the wrong pin.
            pub const PIN_PORT: u8 = ($port as u8) * 32 + $pin_num;
        }

        impl crate::gpio::Pin for crate::peripherals::$name {}
        impl crate::gpio::SealedPin for crate::peripherals::$name {
            #[inline]
            fn pin_port(&self) -> u8 {
                Self::PIN_PORT
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
    debug_assert!(
        ty.pull != Pull::Up || has_pullup(pincm as u8),
        "this pin's IO structure has no pullup, so Pull::Up would do nothing"
    );

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

    // The registers behind `PWREN` stay isolated for a few ULPCLK cycles and a write that lands in
    // that window is dropped. `tim::low_level::enable` carries the account.
    cortex_m::asm::delay(16);

    // `EVT_MODE.INT0_CFG` is not writable. All four TRMs type it `R` with a reset of `1h`, software
    // mode, which is what the CPU interrupt line needs and what it already holds — G TRM table 9-31,
    // L TRM the same register. Driverlib never writes it either. The read-modify-write this replaces
    // set the field to the value it reads back regardless.
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

    // The level a pin settled at is the only thing an edge can be classified by, the status bit carrying
    // no direction.
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
    // present the next one, reading zero when none are left. So it is the index and the `ICLR` write at
    // once, and it costs no bit scan — `u32::trailing_zeros` has no instruction behind it on ARMv6-M and
    // lowers to a multiply and a 32-byte table.
    //
    // **One pin per entry.** Clearing `MIS` is what deasserts the line, so while any pin is still
    // pending the NVIC re-enters this handler rather than a loop here going round again. Which is
    // cheaper depends on how many pins are pending at once, and one is the case that matters: with a
    // loop, every wake pays a second `IIDX` read to be told there is nothing left.
    'dispatch: {
        // Taken as bits rather than through the generated enum: the index is what is wanted, the enum
        // has a variant per pin, and zero-means-none is the register's own definition.
        let stat = gpio.cpu_int().iidx().read().stat().to_bits();

        if stat == 0 {
            break 'dispatch;
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
            break 'dispatch;
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
/// Required by `Flex::new_async` and the other `new_async` constructors, and is what keeps the
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
