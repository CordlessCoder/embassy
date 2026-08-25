//! Marker pins for taking a driver's interrupt path apart on an analyser.
//!
//! Bench instrumentation behind the `_probe` feature, not part of the API and not covered by semver.
//! It lives in the HAL because the timestamps have to come from inside it: the interrupt vectors are
//! `no_mangle` here, so an application cannot install its own handler and bracket it from outside.
//!
//! Each [`Marker`] is a stretch of code that a pin is driven high across and low out of. Arm the ones
//! you want, leave the rest unarmed, and read the segments off the capture. An unarmed marker costs the
//! path one relaxed load and a branch.
//!
//! One mechanism, several subsystems: [`arm`] and the slot table are shared, and the variants are
//! grouped by which driver drives them. Any two markers can be armed at once as long as they are on
//! different pins — nothing here checks that.
//!
//! See `examples/mspm0g3507-lowpower/src/bin/wake_latency_probe.rs` for the GPIO markers and
//! `examples/mspm0g3507-uart/src/bin/uart_overrun.rs` for the UART ones.

use portable_atomic::{AtomicU8, Ordering};

use crate::gpio::Port;
use crate::pac;

/// Set in a marker's byte once [`arm`] has been called for it.
const ARMED: u8 = 1 << 7;

/// Bits a pin index occupies. A port has at most 32 pins.
const PIN_BITS: u8 = 5;

/// Mask for the pin index.
const PIN_MASK: u8 = (1 << PIN_BITS) - 1;

/// A stretch of driver code, bracketed by one pin.
///
/// [`GpioHandler`](Marker::GpioHandler) contains [`GpioWaker`](Marker::GpioWaker), and
/// [`ExecutorPoll`](Marker::ExecutorPoll) comes after both, so those three can be armed at once and read
/// as a nesting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Marker {
    /// The GPIO interrupt handler, from its first instruction to after the wakers have run.
    GpioHandler = 0,

    /// Only the waiter-list wake inside that handler — the one part of it that is not a register access.
    GpioWaker = 1,

    /// One poll of the thread-mode executor. Rises before the woken task runs and falls after, so a task
    /// that drives its own pin puts that edge inside this bracket.
    ExecutorPoll = 2,

    /// The buffered UART interrupt handler, whole. Its rate is the measurement: a handler that stops
    /// firing and one that fires continuously are the two ways a stalled receiver looks from outside.
    UartHandler = 3,

    /// A pulse where that handler masks its own RX interrupt, on a full ring buffer or a receive error.
    ///
    /// Pairs with [`UartHandler`](Marker::UartHandler) to say whether the driver ever turns itself back on.
    UartRxMask = 4,

    /// High across one `try_read`, the receive side's only path out of the ring buffer. Says whether a
    /// receiver that is delivering nothing is being polled and finding nothing, or is not being polled.
    UartReadPoll = 5,

    /// One poll of the transmit side, the counterpart to [`UartReadPoll`](Marker::UartReadPoll). Two
    /// futures joined onto one task are polled together, so a large difference between these two counts
    /// is the whole finding.
    UartWritePoll = 6,

    /// A pulse where the handler wakes the transmit waker, which is the driver's only way of getting the
    /// task run again once it has blocked. Bounds how often the task can have been polled at all.
    UartTxWake = 7,

    /// The DMA interrupt handler, whole. Where its edge falls against
    /// [`DmaTrigger`](Marker::DmaTrigger) is the measurement: an interrupt that arrives before the
    /// transfer it belongs to has finished is one nobody is waiting on yet.
    DmaHandler = 8,

    /// A toggle where a channel is triggered, at the end of its configuration. Marks the start of a
    /// transfer, which is otherwise invisible — a memory-to-memory transfer reaches no pin.
    DmaTrigger = 9,

    /// One poll of a transfer future, which is where its waker is registered. A completion interrupt
    /// landing before the first of these is a completion the waiter cannot have been woken by.
    DmaPoll = 10,

    /// An edge per time-driver bookkeeping tick, where `period` advances. These should be evenly spaced
    /// at half the counter's range forever; a gap is the driver losing track of time.
    TimeDriverPeriod = 11,

    /// An edge where the deferred arm actually unmasks the alarm compare. Its distance back to the
    /// preceding [`TimeDriverPeriod`](Marker::TimeDriverPeriod) edge says whether the arm happened at
    /// the boundary it should have or a period late.
    TimeDriverArm = 12,

    /// An edge where the alarm compare fires in the handler. Against
    /// [`TimeDriverArm`](Marker::TimeDriverArm) this separates "armed late" from "armed on time and the
    /// match went missing".
    TimeDriverAlarm = 13,

    /// An edge where `set_alarm` arms the compare itself, rather than deferring to
    /// [`TimeDriverArm`](Marker::TimeDriverArm). A deadline re-scheduled from inside the handler takes
    /// this path, so an arm that appears here and not there is one the deferred path never saw.
    TimeDriverSetAlarm = 14,

    /// An edge each time `now()` returns a value built from a `period` whose parity disagrees with the
    /// counter's top bit. That pairing is what `calc_now` XORs on, and either way of breaking it
    /// overstates the answer by exactly half the counter's range.
    TimeDriverSkew = 15,

    /// An edge per `on_interrupt` entry, whatever caused it. Against
    /// [`TimeDriverPeriod`](Marker::TimeDriverPeriod) this says whether two `period` advances came from
    /// one entry servicing both the zero and half-overflow flags, or from two entries.
    TimeDriverIrqEntry = 16,
}

/// One byte per marker: bit 7 is [`ARMED`], bits 6:5 the port, bits 4:0 the pin. Zero is unarmed, which
/// is why the armed bit is needed at all — port A pin 0 is otherwise an all-zero entry.
///
/// A byte rather than a word because a pin index needs 5 bits and a port 2, and because the table is
/// read on every instrumented path: a byte index needs no scaling, so the load is one `ldrb` rather than
/// a shift and an `ldr`. That matters more than the RAM — `_probe` distorting what it measures is a
/// mistake this branch has already made once.
///
/// Exported under a fixed name so a debugger can arm a marker, or read which are armed, without the
/// binary having to call [`arm`] itself.
#[unsafe(export_name = "embassy_mspm0_probe_markers")]
static MARKERS: [AtomicU8; 17] = [const { AtomicU8::new(0) }; 17];

/// Drive `pin` on `port` across `marker`.
///
/// The pin must already be configured as an output, and stay one for as long as the marker is armed —
/// this records where to write and nothing else, so that the instrumented path costs a store rather than
/// a pin setup. Nothing checks the pin is not in use elsewhere.
pub fn arm(marker: Marker, port: Port, pin: u8) {
    debug_assert!(pin <= PIN_MASK, "pin index does not fit the marker table");

    MARKERS[marker as usize].store(ARMED | (port as u8) << PIN_BITS | (pin & PIN_MASK), Ordering::Relaxed);
}

/// Where a marker's pin is, resolved once so both edges of a bracket cannot straddle an [`arm`] call.
#[inline]
pub(crate) fn target(marker: Marker) -> Option<(pac::gpio::Gpio, usize)> {
    let probe = MARKERS[marker as usize].load(Ordering::Relaxed);
    if probe & ARMED == 0 {
        return None;
    }

    let block = match probe >> PIN_BITS & 0x3 {
        0 => pac::GPIOA,
        #[cfg(gpio_pb)]
        1 => pac::GPIOB,
        #[cfg(gpio_pc)]
        2 => pac::GPIOC,
        _ => return None,
    };

    Some((block, (probe & PIN_MASK) as usize))
}

/// Open a bracket.
#[inline]
pub(crate) fn set(target: Option<(pac::gpio::Gpio, usize)>) {
    if let Some((block, pin)) = target {
        block.doutset31_0().write(|w| w.set_dio(pin, true));
    }
}

/// Close a bracket.
#[inline]
pub(crate) fn clear(target: Option<(pac::gpio::Gpio, usize)>) {
    if let Some((block, pin)) = target {
        block.doutclr31_0().write(|w| w.set_dio(pin, true));
    }
}

/// Count an event, for a point that has no width worth measuring or too many exits to bracket.
///
/// A toggle rather than a pulse, and one store rather than two. A pulse from adjacent stores can be
/// narrower than the sample period and is then missed entirely, silently undercounting; an edge cannot
/// be, because the level it leaves behind persists. Count edges, not pulses — either direction is one
/// event.
#[inline]
pub(crate) fn count(target: Option<(pac::gpio::Gpio, usize)>) {
    if let Some((block, pin)) = target {
        block.douttgl31_0().write(|w| w.set_dio(pin, true));
    }
}
