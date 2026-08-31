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
//! **[`Debugss::debug_access_enabled`] is "now".** `SPECIAL_AUTH.AHBAPEN` is a level, and it resets
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
//! let mut watch = Debugss::new(p.DEBUGSS);
//!
//! watch.clear();
//! // ... the measurement ...
//! if watch.attached_since_cleared() {
//!     warn!("a probe attached during this window; the sleep figures are not trustworthy");
//! }
//! ```
//!
//! # The mailbox
//!
//! A 32-bit word each way between this CPU and an attached debug probe, over the SWD pair and no
//! other pins. One word deep in each direction with real flow control, so it is a channel for
//! results rather than for logging -- a value that must not be truncated, where a byte stream would
//! be the wrong shape.
//!
//! **The register names are the probe's, and this API's are yours.** `TXD` is what the probe
//! transmits, so the CPU *reads* it and cannot write it; `RXD` is what the probe receives, so the CPU
//! writes it. A driver that took `TX` to mean "out of this CPU" would have both directions inverted,
//! and the failure is a mailbox that looks dead rather than one that errors. [`try_receive`](Debugss::try_receive)
//! reads and [`try_send`](Debugss::try_send) writes, from the caller's point of view, and the register each
//! touches is the opposite one to the name's.
//!
//! Backpressure is visible in both directions and neither has a queue. A word written stays pending
//! until the probe collects it; a word from the probe stays pending until this reads it, and
//! **reading is the only thing that clears it** -- there is no acknowledge register.
//!
//! # What the far end is
//!
//! **The probe reaches these buffers through a dedicated access port, not through memory.** SLAU847
//! table 35-7 puts the mailbox on **SEC-AP, AP index 2**: `TXDATA` at AP address `0x00`, `TXCTL`
//! `0x04`, `RXDATA` `0x08`, `RXCTL` `0x0C`, and the AP `IDR` at `0x0FC`. So a host drives it by
//! selecting that AP and reading and writing its registers -- **an ordinary AHB-AP memory write to
//! this peripheral's address does not reach it**, even though the same buffers are memory-mapped
//! from the CPU's side. The two ends see one pair of buffers through two different windows.
//!
//! The flow control is the probe's mirror of what this driver sees. `TXCTL.TRANSMIT` is set when the
//! probe writes `TXDATA` and clears only when this CPU reads it or a POR happens; `RXCTL.RECEIVE` is
//! set when this CPU writes `RXDATA` and clears only when the probe reads it. Neither side can clear
//! the other's flag by any route but reading the data.
//!
//! The flag fields are asymmetric and write-once-per-side: `TXCTL`'s upper 31 bits are
//! `TRANSMIT_FLAGS`, writable only by the probe, and `RXCTL`'s bits 1 through 7 are `RECEIVE_FLAGS`,
//! writable only by this CPU. Each side reads the other's and cannot modify them.
//!
//! **The TRM contradicts itself on `RXIFG`, and the measurement settles it.** Table 35-6 says
//! `RXIFG` "is also set on a write by the target device", which would make it fire on *our own*
//! send; table 35-9 says it "indicates that the data in `RX_DATA` buffer in the DSSM was read",
//! which would make it fire when the probe collects it. **Table 35-9 is right.** So the probe taking
//! a word is observable, and a future that waits for space has a signal to park on.
//!
//! **Clear `ICLR` before reading `RIS`, or this measurement gives the opposite answer.** The
//! `CPU_INT.RIS` bits latch and nothing clears them on their own, so on a part that has been running
//! a while `RIS` reads `0xF` — every source, meaning only "this fired at some point since
//! power-up". Asking whether `RXIFG` was set before a probe read then answers a different question
//! and answers it consistently, which is what makes it convincing and wrong. Cleared first, `RIS` is
//! `0x1` after our own write (`TXIFG` alone) and `0x3` after the probe reads. The first attempt at
//! this concluded table 35-6.
//!
//! **The channel is verified end to end**, on an L1306 with a host on SEC-AP: a word written to
//! `TXDATA` sets `TRANSMIT`, a CPU read clears it, and the CPU's reply comes back through `RXDATA`.
//! The returning word was a value the firmware computed rather than one resident in the buffer,
//! which is what separates a channel from an echo. The AP `IDR` reads `0x002E0000`. Measured by the
//! consuming project, not here.
//!
//! # There is nothing to bring up
//!
//! DEBUGSS has no `PWREN`, no `RSTCTL` and no clock select -- unusually for this portfolio, and the
//! reason this driver has no configuration and cannot fail to start. The block is there whether or
//! not anything is using it.

#![macro_use]

use core::future::Future;
use core::task::Poll;

use embassy_hal_internal::Peri;
use mspm0_metapac::debugss::vals;

use crate::pac;
use crate::peripherals::DEBUGSS;
use crate::sync::irq_waker::IrqWaker;

/// What the debug subsystem last reported about a probe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProbeEvent {
    /// A probe attached, and the debug subsystem powered up because of it.
    Attached,

    /// A probe disconnected, and the debug subsystem powered down.
    Detached,
}

/// Interrupt handler.
pub struct InterruptHandler;

impl InterruptHandler {
    /// Mask rather than clear.
    ///
    /// The latched flag is both what wakes the waiter and what tells it *which* edge arrived, and the
    /// handler has no way to hand a value over. Clearing here would wake a future that then finds
    /// nothing to report. Masking deasserts the line without touching `RIS`, which also leaves the
    /// polled reads on this type working.
    ///
    /// Public only so the generated instance impl can reach it; there is no reason to call it.
    #[doc(hidden)]
    pub fn handle() {
        pac::DEBUGSS.cpu_int(0).imask().write(|_| {});
        STATE.waker.wake();
    }
}

static STATE: State = State::new();

struct State {
    waker: IrqWaker,
}

impl State {
    const fn new() -> Self {
        Self { waker: IrqWaker::new() }
    }
}

/// Proof that the debug subsystem's interrupt is bound to its [`InterruptHandler`].
///
/// **Which macro produces it differs per device.** DEBUGSS is a source on an interrupt group on most
/// chips, wanting [`bind_group_interrupts!`](crate::bind_group_interrupts), and the owner of an NVIC
/// line on eight of them, wanting [`bind_interrupts!`](crate::bind_interrupts). A binding written for
/// the wrong one names a type that does not exist rather than silently linking nothing.
///
/// # Safety
///
/// Implementing this without installing the handler lets a wait park on an interrupt that reaches
/// nothing. Use the macros.
pub unsafe trait DebugssInterrupt {}

/// The debug subsystem.
///
/// One driver for the whole peripheral rather than one per capability, and the reason is the
/// interrupt block: `TXIFG`, `RXIFG`, `PWRUPIFG` and `PWRDWNIFG` are bits of a single `CPU_INT`
/// register set. Two drivers would each read-modify-write `IMASK` and each read `IIDX`, which is two
/// writers of one register and the shape that has already cost this crate a defect elsewhere.
pub struct Debugss<'d> {
    _peri: Peri<'d, DEBUGSS>,
}

impl<'d> Debugss<'d> {
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

    /// Claim the debug subsystem with its interrupt bound, so a change can be awaited.
    pub fn new_async(peri: Peri<'d, DEBUGSS>, _irq: impl DebugssInterrupt + 'd) -> Self {
        Self { _peri: peri }
    }

    /// Wait for the next attach or detach.
    ///
    /// **Returns at once if one is already latched**, so an edge that arrived before this was called
    /// is reported rather than waited past. Taking it clears that flag and leaves the other alone, so
    /// a probe that came and went is reported as two events by two calls rather than one.
    ///
    /// Attach is reported first when both are pending. That is a choice, not a hardware ordering:
    /// the flags carry no sequence, so a probe that arrived and left between two calls is
    /// indistinguishable from one that left and arrived.
    ///
    /// Needs [`new_async`](Self::new_async) -- without the handler this parks forever.
    pub fn wait_for_change(&mut self) -> impl Future<Output = ProbeEvent> + '_ {
        core::future::poll_fn(move |cx| {
            let r = Self::regs();

            STATE.waker.register(cx.waker());

            let ris = r.cpu_int(0).ris().read();

            if ris.pwrupifg() {
                r.cpu_int(0).iclr().write(|w| w.set_pwrupifg(true));
                return Poll::Ready(ProbeEvent::Attached);
            }

            if ris.pwrdwnifg() {
                r.cpu_int(0).iclr().write(|w| w.set_pwrdwnifg(true));
                return Poll::Ready(ProbeEvent::Detached);
            }

            // Armed only while something is waiting. The handler masks when it fires, so this is what
            // re-arms it, and a driver nobody is awaiting costs no interrupt entries.
            r.cpu_int(0).imask().modify(|w| {
                w.set_pwrupifg(true);
                w.set_pwrdwnifg(true);
            });

            Poll::Pending
        })
    }

    /// Take a word the probe has sent, if there is one.
    ///
    /// Reads `TXD`, which the probe writes. **The read is what clears the pending flag** -- there is
    /// no acknowledge path, so a caller that peeks at the status without reading the word leaves the
    /// channel blocked.
    pub fn try_receive(&mut self) -> Option<u32> {
        let r = Self::regs();

        if r.txctl().read().transmit() != vals::Transmit::Full {
            return None;
        }

        Some(r.txd().read())
    }

    /// The flags the probe set alongside its last word.
    ///
    /// 31 bits, and **the outbound side has only 7** -- see [`send_flags`](Self::send_flags). The
    /// asymmetry is the hardware's.
    pub fn received_flags(&self) -> u32 {
        Self::regs().txctl().read().transmit_flags()
    }

    /// Give the probe a word, if the last one has been collected.
    ///
    /// Writes `RXD`, which the probe reads. Returns the word back when the channel is still full:
    /// there is no queue, and overwriting would drop a word the probe has not seen with nothing to
    /// say it happened.
    pub fn try_send(&mut self, word: u32) -> Result<(), u32> {
        let r = Self::regs();

        if r.rxctl().read().receive() == vals::Receive::Full {
            return Err(word);
        }

        r.rxd().write_value(word);

        Ok(())
    }

    /// Whether a word given to the probe is still waiting to be collected.
    ///
    /// The backpressure signal, and the only one there is.
    pub fn send_pending(&self) -> bool {
        Self::regs().rxctl().read().receive() == vals::Receive::Full
    }

    /// Set the flags the probe reads alongside the next word.
    ///
    /// **Seven bits**, against 31 in the other direction. Values above are a caller error rather than
    /// a truncation, because a flag silently dropped is indistinguishable from one never set.
    pub fn send_flags(&mut self, flags: u8) {
        assert!(flags < 0x80, "only seven flag bits are sent to the probe");

        Self::regs().rxctl().modify(|w| w.set_receive_flags(flags));
    }

    #[inline]
    fn regs() -> pac::debugss::Debugss {
        pac::DEBUGSS
    }
}

#[allow(unused_macros)]
macro_rules! impl_debugss_grouped {
    () => {
        #[cfg(feature = "rt")]
        impl crate::interrupt_group::Handler<crate::interrupt_group::DEBUGSS> for crate::debugss::InterruptHandler {
            unsafe fn on_interrupt() {
                Self::handle();
            }
        }

        #[cfg(feature = "rt")]
        unsafe impl<T> crate::debugss::DebugssInterrupt for T where
            T: crate::interrupt_group::Binding<crate::interrupt_group::DEBUGSS, crate::debugss::InterruptHandler>
        {
        }
    };
}

/// The same, for a chip where the debug subsystem owns an NVIC line instead of sitting on a group.
#[allow(unused_macros)]
macro_rules! impl_debugss_nvic {
    ($line:ident) => {
        #[cfg(feature = "rt")]
        impl crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::$line>
            for crate::debugss::InterruptHandler
        {
            unsafe fn on_interrupt() {
                Self::handle();
            }
        }

        #[cfg(feature = "rt")]
        unsafe impl<T> crate::debugss::DebugssInterrupt for T where
            T: crate::interrupt::typelevel::Binding<
                    crate::interrupt::typelevel::$line,
                    crate::debugss::InterruptHandler,
                >
        {
        }
    };
}
