//! Debug subsystem (DEBUGSS).
//!
//! Two things live here that have nothing to do with each other beyond sharing a peripheral: knowing
//! whether a debug probe is attached, and a message channel to one. Only the first is implemented.
//!
//! # Why a HAL cares that a probe is attached
//!
//! **Sleep depth cannot be observed from outside, because attaching a probe holds the part awake.**
//! So a figure measured with a debugger connected is a figure for a device that never reached the
//! mode it was asked for, and nothing in the sleep path can tell. This is the register that can.
//!
//! Two different questions, and the hardware answers them separately.
//!
//! **[`ProbeWatch::debug_access_enabled`] is "now".** `SPECIAL_AUTH.AHBAPEN` is a level, and it resets
//! to zero, so a one there means a debugger has since been given access to memory. Measured true with
//! a probe attached; **the false case is unverified**, because reading it with no probe attached needs
//! a channel a detached probe does not leave behind.
//!
//! **The latched flags are "since", and they are edges.** They report a probe *arriving or leaving*
//! during the window -- **not one attached throughout it**, which raises no edge and leaves them
//! clear. Measured: they stay clear while a probe is continuously attached. So they catch the case
//! nobody expected and miss the case everybody knows about, which is the right way round but is not
//! the same as "the window was clean".
//!
//! ```rust,ignore
//! let mut watch = ProbeWatch::new(p.DEBUGSS);
//!
//! watch.clear();
//! // ... the measurement ...
//! if watch.attached_since_cleared() {
//!     warn!("a probe attached during this window; the sleep figures are not trustworthy");
//! }
//! ```
//!
//! # There is nothing to bring up
//!
//! DEBUGSS has no `PWREN`, no `RSTCTL` and no clock select -- unusually for this portfolio, and the
//! reason this driver has no configuration and cannot fail to start. The block is there whether or
//! not anything is using it.

#![macro_use]

use embassy_hal_internal::Peri;

use crate::pac;
use crate::peripherals::DEBUGSS;

/// What the debug subsystem last reported about a probe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProbeEvent {
    /// A probe attached, and the debug subsystem powered up because of it.
    Attached,

    /// A probe disconnected, and the debug subsystem powered down.
    Detached,
}

/// Watches for a debug probe attaching or detaching.
pub struct ProbeWatch<'d> {
    _peri: Peri<'d, DEBUGSS>,
}

impl<'d> ProbeWatch<'d> {
    /// Claim the debug subsystem.
    ///
    /// Nothing is powered on or reset -- the block has neither -- and the latched flags are left
    /// alone, so an attach that happened before this call is still reported. Call [`clear`](Self::clear)
    /// to start from a known state.
    pub fn new(peri: Peri<'d, DEBUGSS>) -> Self {
        Self { _peri: peri }
    }

    /// Whether a probe has attached since [`clear`](Self::clear) was last called.
    ///
    /// **This latches an edge, which is the point and also the limit.** A probe that attached and
    /// detached again inside the window still reads true afterwards, so a connection gone by the time
    /// anyone looked is still reported. A probe attached before the window and still attached after
    /// raises no edge and reads false -- ask
    /// [`debug_access_enabled`](Self::debug_access_enabled) instead.
    pub fn attached_since_cleared(&self) -> bool {
        Self::regs().cpu_int(0).ris().read().pwrupifg()
    }

    /// Whether a probe has detached since [`clear`](Self::clear) was last called.
    ///
    /// Separate from [`attached_since_cleared`](Self::attached_since_cleared) rather than derived from
    /// it: both can be set, and that means a probe came and went rather than that the second event
    /// undid the first.
    pub fn detached_since_cleared(&self) -> bool {
        Self::regs().cpu_int(0).ris().read().pwrdwnifg()
    }

    /// Forget both, so the next reading describes a window starting now.
    pub fn clear(&mut self) {
        Self::regs().cpu_int(0).iclr().write(|w| {
            w.set_pwrupifg(true);
            w.set_pwrdwnifg(true);
        });
    }

    /// Whether the core's debug access is enabled right now.
    ///
    /// `SPECIAL_AUTH.AHBAPEN` -- whether a debugger can reach memory through the AHB-AP. Unlike the
    /// latched flags this is a level, so it answers "now" rather than "since".
    pub fn debug_access_enabled(&self) -> bool {
        Self::regs().special_auth().read().ahbapen()
    }

    /// Whether a probe arrived or left during the window.
    ///
    /// **Not "the window was clean".** A probe attached for the whole window raises no edge and reads
    /// false here; use [`debug_access_enabled`](Self::debug_access_enabled) for that. What this
    /// catches is the connection nobody expected, which is the one that silently invalidates a
    /// measurement.
    pub fn disturbed(&self) -> bool {
        let ris = Self::regs().cpu_int(0).ris().read();

        ris.pwrupifg() || ris.pwrdwnifg()
    }

    #[inline]
    fn regs() -> pac::debugss::Debugss {
        pac::DEBUGSS
    }
}
