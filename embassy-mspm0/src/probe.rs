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

use portable_atomic::{AtomicU32, Ordering};

use crate::gpio::Port;
use crate::pac;

/// Set in a marker's word once [`arm`] has been called for it.
const ARMED: u32 = 1 << 31;

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
}

/// One word per marker: bit 31 is [`ARMED`], bits 15:8 the port, bits 7:0 the pin. Zero is unarmed,
/// which is why the armed bit is needed at all — port A pin 0 is otherwise an all-zero word.
static MARKERS: [AtomicU32; 11] = [const { AtomicU32::new(0) }; 11];

/// Drive `pin` on `port` across `marker`.
///
/// The pin must already be configured as an output, and stay one for as long as the marker is armed —
/// this records where to write and nothing else, so that the instrumented path costs a store rather than
/// a pin setup. Nothing checks the pin is not in use elsewhere.
pub fn arm(marker: Marker, port: Port, pin: u8) {
    MARKERS[marker as usize].store(ARMED | (port as u32) << 8 | pin as u32, Ordering::Relaxed);
}

/// Where a marker's pin is, resolved once so both edges of a bracket cannot straddle an [`arm`] call.
#[inline]
pub(crate) fn target(marker: Marker) -> Option<(pac::gpio::Gpio, usize)> {
    let probe = MARKERS[marker as usize].load(Ordering::Relaxed);
    if probe & ARMED == 0 {
        return None;
    }

    let block = match (probe >> 8) & 0xff {
        0 => pac::GPIOA,
        #[cfg(gpio_pb)]
        1 => pac::GPIOB,
        #[cfg(gpio_pc)]
        2 => pac::GPIOC,
        _ => return None,
    };

    Some((block, (probe & 0xff) as usize))
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
