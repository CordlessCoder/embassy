//! Marker pins for taking a wake apart on an analyser.
//!
//! Bench instrumentation behind the `_probe` feature, not part of the API and not covered by semver.
//! It lives in the HAL because the timestamps have to come from inside it: `GROUP1` and the GPIO group
//! vectors are `no_mangle` here, so an application cannot install its own.
//!
//! Each [`Marker`] is a point in the wake path that a pin is driven high across and low out of. Arm the
//! ones you want, leave the rest unarmed, and read the segments off the capture. An unarmed marker costs
//! the path one relaxed load and a branch.
//!
//! See `examples/mspm0g3507-lowpower/src/bin/wake_latency_probe.rs`.

use portable_atomic::{AtomicU32, Ordering};

use crate::gpio::Port;
use crate::pac;

/// Set in a marker's word once [`arm`] has been called for it.
const ARMED: u32 = 1 << 31;

/// A stage of the wake path, bracketed by one pin.
///
/// [`Handler`](Marker::Handler) contains [`Waker`](Marker::Waker), and [`Poll`](Marker::Poll) comes after
/// both, so all three can be armed at once and read as a nesting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Marker {
    /// The GPIO interrupt handler, from its first instruction to after the wakers have run.
    Handler = 0,

    /// Only the wait-map wake inside that handler — the one part of it that is not a register access.
    Waker = 1,

    /// One poll of the thread-mode executor. Rises before the woken task runs and falls after, so a task
    /// that drives its own pin puts that edge inside this bracket.
    Poll = 2,
}

/// One word per marker: `ARMED | port << 8 | pin`, or zero for unarmed.
static MARKERS: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];

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
