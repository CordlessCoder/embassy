//! Register-level I2C controller, for an application that services the interrupt itself.
//!
//! [`I2c`] here powers the instance up, claims the pins and programs a [`Config`], and then stops. It
//! runs no transfer and waits for nothing — what it adds over the raw registers is the clock
//! solution, the pin setup, the fault decoding and the recovery sequences, none of which are obvious
//! from the register map and several of which were established on a logic analyser.
//!
//! [`I2c<Blocking>`](super::I2c) and [`I2c<Async>`](super::I2c) are both built on this and hold one.
//!
//! # What the caller takes on
//!
//! **The interrupt.** Nothing here routes a vector or unmasks the NVIC line. [`I2c::interrupt`] names
//! it, [`I2c::enable_interrupt`] arms sources within the peripheral, and [`I2c::arm_only`] sets the
//! whole mask in one write, which is what a burst wants.
//!
//! **The burst.** [`I2c::start_write`] and [`I2c::start_read`] arm one and return; the FIFO is then
//! the caller's to fill or drain with [`I2c::fill_tx`] and [`I2c::drain_rx`]. A transfer is one burst,
//! since `CCTR.CBLEN` is twelve bits — 4095 bytes — and the FIFO is fed through it rather than being
//! the unit a transfer moves in.
//!
//! **Recovering.** [`I2c::reset_peripheral`] is the only thing that reliably ends a burst nobody
//! serviced: a STOP-only command is illegal until the transaction finishes, and no interrupt is
//! raised when one does. [`I2c::bus_is_stuck`] then says whether the target is still holding SDA, and
//! [`I2c::recover_stuck_bus`] clocks it off.
//!
//! # Two things that are not the caller's to get right
//!
//! **`I2C_ERR_13`.** A controller must let the address phase settle before `CSR` means anything, and
//! [`I2c::settle_after_start`] is that wait, sized from the configured bus speed. **Call it yourself
//! after `start_write` or `start_read`** — they do not, and polling `CSR` any sooner reads it before
//! the controller has raised `BUSY`, so the wait falls straight through and a NACK goes unnoticed.
//! The mode drivers call it on the line after every start; this is one of the two things the module
//! heading says are yours rather than theirs.
//!
//! **The clock solution.** [`Timing::solve`](super::Timing) is `const`, so a fixed bus speed costs no
//! run-time arithmetic — and the divider bands, the headroom check and the clock-low timeout are all
//! decided there rather than here.
//!
//! # Every method is `#[inline]`, for the reason `uart::low_level` gives
//!
//! These types carry no instance parameter, so a public method that is not generic is compiled
//! whether or not anything calls it, and it takes a `&self` holding the instance's `&'static State`.
//! That makes the static escape and keeps dead stores alive. See that module for the measurement.

use core::sync::atomic::Ordering;

use super::{
    Address, ClockSel, Config, ConfigError, Error, GroupCursor, IDLE_HALF_PERIODS, Info, Instance, Resolved, SclPin,
    SdaPin, State,
};
use crate::gpio::{MaybeAnyPin, PfType, Pull, SealedPin};
use crate::interrupt::InterruptExt;
use crate::pac::i2c::{self, vals};
use crate::sysctl::SleepLevel;
use crate::{Peri, pac};

/// A condition the controller can raise its interrupt on.
///
/// One variant per source this driver's configuration can reach, all of them controller-side —
/// `i2c_target` is a separate driver with its own. `TIMEOUTB` is left out deliberately: it watches
/// SCL *high* on a per-clock timescale, nothing here enables its counter, and TI's own
/// `setClockTimeout` is a stub. The DMA and PEC sources are out because neither is configured.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Event {
    /// A transmit burst sent its last byte.
    TransmitDone,

    /// A receive burst took its last byte.
    ReceiveDone,

    /// The transmit FIFO fell to its trigger level, so there is room to queue more.
    TransmitTrigger,

    /// The receive FIFO reached its trigger level, so there is something to take.
    ReceiveTrigger,

    /// The transmit FIFO went empty.
    TransmitEmpty,

    /// The receive FIFO filled.
    ReceiveFull,

    /// A target did not acknowledge. [`nack_kind`](I2c::nack_kind) says which phase.
    Nack,

    /// A START went out.
    Start,

    /// A STOP went out, which is what releases the bus.
    Stop,

    /// Arbitration was lost to another controller.
    ArbitrationLost,

    /// A target held SCL low for longer than `Config::clock_low_timeout_us` allows.
    ClockLowTimeout,

    /// Anything that ends a burst without completing it: [`Nack`](Event::Nack),
    /// [`ArbitrationLost`](Event::ArbitrationLost) or [`ClockLowTimeout`](Event::ClockLowTimeout).
    ///
    /// The three a handler tests before anything else, since they are the same whichever direction
    /// the burst was running.
    AnyFault,
}

impl Event {
    #[inline]
    pub(crate) const fn mask(self) -> i2c::regs::CpuInt {
        let mut mask = i2c::regs::CpuInt(0);

        match self {
            Event::TransmitDone => mask.set_ctxdone(true),
            Event::ReceiveDone => mask.set_crxdone(true),
            Event::TransmitTrigger => mask.set_ctxfifotrg(true),
            Event::ReceiveTrigger => mask.set_crxfifotrg(true),
            Event::TransmitEmpty => mask.set_ctxempty(true),
            Event::ReceiveFull => mask.set_crxfifofull(true),
            Event::Nack => mask.set_cnack(true),
            Event::Start => mask.set_cstart(true),
            Event::Stop => mask.set_cstop(true),
            Event::ArbitrationLost => mask.set_carblost(true),
            Event::ClockLowTimeout => mask.set_timeouta(true),
            Event::AnyFault => return fault_sources(),
        }

        mask
    }
}

/// The three sources that end a burst without completing it.
pub(crate) const fn fault_sources() -> i2c::regs::CpuInt {
    let mut sources = i2c::regs::CpuInt(0);
    sources.set_cnack(true);
    sources.set_carblost(true);
    sources.set_timeouta(true);
    sources
}

// The sets the mode drivers arm, each a `const fn` folding to one immediate.
//
// **The public [`Event`] API cannot serve these, and that is measured rather than assumed.** Its
// `enable_interrupts` ORs a mask out of an array at run time, which `opt-level = "z"` does not unroll
// — 188 bytes on an asynchronous binary as a slice, 368 with `#[inline(always)]` forcing the match to
// expand per element. A caller arming a burst once per transfer will not notice; a driver that does it
// on every transfer does. Same shape as `uart::low_level`'s `rx_sources` and friends.

/// A transmit burst: the faults, the completion, and — only where the FIFO could not take the whole
/// transfer — the trigger that tops it up.
///
/// `top_up` is a parameter rather than a second constant so it stays one conditional bit inside one
/// write. Two constants selected by a branch measured 12 bytes more.
pub(crate) const fn write_sources(top_up: bool) -> i2c::regs::CpuInt {
    let mut sources = fault_sources();
    sources.set_ctxdone(true);
    sources.set_ctxfifotrg(top_up);
    sources
}

/// A receive burst: the faults, the completion, and the trigger that delivers each FIFO's worth.
pub(crate) const fn read_sources() -> i2c::regs::CpuInt {
    let mut sources = fault_sources();
    sources.set_crxdone(true);
    sources.set_crxfifotrg(true);
    sources
}

/// Waiting for the bus: the STOP that releases it, and the timeout that is the way out.
pub(crate) const fn bus_free_sources() -> i2c::regs::CpuInt {
    let mut sources = i2c::regs::CpuInt(0);
    sources.set_cstop(true);
    sources.set_timeouta(true);
    sources
}

/// The highest-priority latched source as the register reports it, clearing it.
///
/// [`I2c::take_next`] is the same read mapped onto [`Event`]. The mode drivers take this one and
/// match it directly; see that method for what the mapping costs.
#[inline]
pub(crate) fn next_status(regs: crate::pac::i2c::I2c) -> vals::CpuIntIidxStat {
    regs.cpu_int(0).iidx().read().stat()
}

/// Let the given sources reach the CPU.
#[inline]
pub(crate) fn unmask(regs: crate::pac::i2c::I2c, sources: i2c::regs::CpuInt) {
    regs.cpu_int(0).imask().modify(|w| w.0 |= sources.0);
}

/// Stop the given sources reaching the CPU.
#[inline]
pub(crate) fn mask(regs: crate::pac::i2c::I2c, sources: i2c::regs::CpuInt) {
    regs.cpu_int(0).imask().modify(|w| w.0 &= !sources.0);
}

/// Drop the given sources' latched flags.
#[inline]
pub(crate) fn clear(regs: crate::pac::i2c::I2c, sources: i2c::regs::CpuInt) {
    regs.cpu_int(0).iclr().write_value(sources);
}

/// A powered, configured I2C controller that is running nothing.
///
/// See the [module documentation](self) for what the caller takes on.
pub struct I2c<'d> {
    pub(crate) info: &'static Info,
    pub(crate) state: &'static State,
    pub(crate) scl: MaybeAnyPin<'d>,
    pub(crate) sda: MaybeAnyPin<'d>,
    pub(crate) wake_floor: Option<SleepLevel>,
    /// What the peripheral is configured to, kept so [`reset_peripheral`](Self::reset_peripheral) can
    /// restore it.
    pub(crate) resolved: Resolved,
}

/// The most bytes one burst can move: `CCTR.CBLEN` is twelve bits wide.
///
/// A length past this truncates rather than saturating, and a `CBLEN` of zero with START and STOP is
/// an address-only transaction — a transfer that reports success without moving the caller's bytes.
pub const MAX_BURST_LEN: usize = 0xFFF;

impl<'d> I2c<'d> {
    /// Power up an instance, claim the pins and apply `config`, leaving the bus idle.
    ///
    /// Nothing is armed and no vector is routed — see the [module documentation](self).
    #[inline]
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let resolved = config.resolve()?;

        Self::new_inner(peri, scl, sda, config, resolved)
    }

    #[inline]
    pub(crate) fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
        resolved: Resolved,
    ) -> Result<Self, ConfigError> {
        power_up(T::info().regs);

        let scl_inner = new_pin!(scl, config.scl_pf());
        let sda_inner = new_pin!(sda, config.sda_pf());

        if let Some(ref scl) = scl_inner {
            let pincm = pac::IOMUX.pincm(scl._pin_cm() as usize);
            pincm.modify(|w| {
                w.set_hiz1(true);
            });
        }

        if let Some(ref sda) = sda_inner {
            let pincm = pac::IOMUX.pincm(sda._pin_cm() as usize);
            pincm.modify(|w| {
                w.set_hiz1(true);
            });
        }

        let mut this = Self {
            info: T::info(),
            state: T::state(),
            scl: MaybeAnyPin::new(scl_inner),
            sda: MaybeAnyPin::new(sda_inner),
            wake_floor: None,
            resolved,
        };
        this.init()?;

        Ok(this)
    }
    /// Whether the controller has finished everything it was given.
    #[inline]
    pub fn is_idle(&self) -> bool {
        self.info.regs.controller(0).csr().read().idle()
    }

    /// Whether a burst is still running.
    #[inline]
    pub fn is_busy(&self) -> bool {
        self.info.regs.controller(0).csr().read().busy()
    }

    /// How many bytes are waiting in the receive FIFO.
    #[inline]
    pub fn rx_fifo_count(&self) -> u8 {
        self.info.regs.controller(0).cfifosr().read().rxfifocnt()
    }

    /// The highest-priority latched source, clearing it.
    ///
    /// `IIDX` reports one source and clears that one, so this is the dispatch a handler runs **once**
    /// on entry — not in a loop. Re-entering costs six to eight cycles where a loop costs a branch and
    /// the stack traffic around it, which is why every driver here reads it once and returns.
    ///
    /// `None` means nothing was pending, or that what was pending is a source [`Event`] does not
    /// model — the target-side and DMA ones, which this driver never arms.
    ///
    /// The mode drivers do not use this; they match the raw status, because mapping it to an `Event`
    /// and matching that costs a second match the optimiser does not collapse — 60 bytes on an
    /// asynchronous binary. The capability is the same, and this is the form to write against.
    #[inline]
    pub fn take_next(&mut self) -> Option<Event> {
        use vals::CpuIntIidxStat as Stat;

        Some(match self.info.regs.cpu_int(0).iidx().read().stat() {
            Stat::Ctxdonefg => Event::TransmitDone,
            Stat::Crxdonefg => Event::ReceiveDone,
            Stat::Ctxfifotrg => Event::TransmitTrigger,
            Stat::Crxfifotrg => Event::ReceiveTrigger,
            Stat::CtxEmpty => Event::TransmitEmpty,
            Stat::Crxfifofull => Event::ReceiveFull,
            Stat::Cnackfg => Event::Nack,
            Stat::Cstartfg => Event::Start,
            Stat::Cstopfg => Event::Stop,
            Stat::Carblostfg => Event::ArbitrationLost,
            Stat::Timeouta => Event::ClockLowTimeout,
            _ => return None,
        })
    }

    /// Registers of this instance, for what this driver does not wrap.
    #[inline]
    pub fn regs(&self) -> crate::pac::i2c::I2c {
        self.info.regs
    }

    /// The interrupt line this instance raises.
    ///
    /// Nothing here enables it.
    #[inline]
    pub fn interrupt(&self) -> crate::interrupt::Interrupt {
        self.info.interrupt
    }

    /// How many entries each FIFO holds on this instance.
    #[inline]
    pub fn fifo_size(&self) -> usize {
        self.info.fifo_size
    }

    /// Shallowest sleep level that keeps a transfer running, if one needs blocking at all.
    #[inline]
    pub fn transfer_floor(&self) -> Option<SleepLevel> {
        self.wake_floor
    }

    /// Let `event` reach the CPU, or stop it.
    ///
    /// A read-modify-write on `IMASK`, which an interrupt handler writes as well. Use
    /// [`arm_only`](Self::arm_only) where the whole mask is being set, which is what arming a burst
    /// does.
    #[inline]
    pub fn enable_interrupt(&mut self, event: Event, enable: bool) {
        let mask = event.mask().0;

        self.info.regs.cpu_int(0).imask().modify(|w| {
            w.0 = if enable { w.0 | mask } else { w.0 & !mask };
        });
    }

    /// Let all of `events` reach the CPU, leaving the rest of the mask as it is.
    ///
    /// One read-modify-write for the whole set, which is what arming a burst wants: three faults plus
    /// whichever completion it waits on.
    ///
    /// Takes an array rather than a slice so the length is a constant and the mask folds to one. A
    /// slice measured 188 bytes on an asynchronous binary, because the loop stays a loop.
    #[inline]
    pub fn enable_interrupts<const N: usize>(&mut self, events: [Event; N], enable: bool) {
        let mut mask = i2c::regs::CpuInt(0);

        for event in events {
            mask.0 |= event.mask().0;
        }

        self.info.regs.cpu_int(0).imask().modify(|w| {
            w.0 = if enable { w.0 | mask.0 } else { w.0 & !mask.0 };
        });
    }

    /// Let exactly `events` reach the CPU and nothing else, in one write.
    ///
    /// What a burst wants: the three faults plus whichever completion it is waiting for, with
    /// everything the last burst armed gone.
    #[inline]
    pub fn arm_only<const N: usize>(&mut self, events: [Event; N]) {
        let mut mask = i2c::regs::CpuInt(0);

        for event in events {
            mask.0 |= event.mask().0;
        }

        self.info.regs.cpu_int(0).imask().write_value(mask);
    }

    /// Mask every source, leaving nothing armed.
    ///
    /// What a cancelled transfer does first: an armed source with no future left to consume it fires
    /// into a handler that only wakes, and re-enters until something masks it.
    #[inline]
    pub fn disarm(&mut self) {
        disarm(self.info.regs);
    }

    /// Whether `event` is latched, whether or not it is unmasked.
    ///
    /// Reads `RIS`, so it answers about the source and not about what raised the line;
    /// [`take_active`](Self::take_active) is the other question.
    #[inline]
    pub fn is_pending(&self, event: Event) -> bool {
        self.info.regs.cpu_int(0).ris().read().0 & event.mask().0 != 0
    }

    /// Drop `event`'s latched flag.
    #[inline]
    pub fn clear_pending(&mut self, event: Event) {
        self.info.regs.cpu_int(0).iclr().write_value(event.mask());
    }

    /// Whether `event` is latched **and** unmasked, clearing the part of it that is.
    ///
    /// What raised the line, in one `MIS` read and at most one `ICLR` write.
    /// [`Event::AnyFault`] makes it the three burst-ending faults at once.
    #[inline]
    pub fn take_active(&mut self, event: Event) -> bool {
        let active = self.info.regs.cpu_int(0).mis().read().0 & event.mask().0;

        if active != 0 {
            self.info.regs.cpu_int(0).iclr().write_value(i2c::regs::CpuInt(active));
        }

        active != 0
    }

    /// Whether another controller or an unfinished transaction still holds the bus.
    #[inline]
    pub fn bus_is_busy(&self) -> bool {
        self.info.regs.controller(0).csr().read().busbsy()
    }

    /// Record that a transfer was abandoned part-way, so the next one resets before it starts.
    ///
    /// Cannot be derived from the registers: `CCTR` reads the same after a completed burst as after
    /// an abandoned one. [`take_abandoned`](Self::take_abandoned) is the other half.
    #[inline]
    pub fn mark_abandoned(&mut self) {
        mark_abandoned(self.state);
    }

    /// Whether a transfer was abandoned since this was last asked, clearing the record.
    #[inline]
    pub fn take_abandoned(&mut self) -> bool {
        let abandoned = self.state.abandoned.load(Ordering::Relaxed);

        if abandoned {
            self.state.abandoned.store(false, Ordering::Relaxed);
        }

        abandoned
    }
    /// Reconfigure the driver
    #[inline]
    pub fn set_config(&mut self, config: Config) -> Result<(), ConfigError> {
        let resolved = config.resolve()?;

        // Kept so a later [`I2c::reset_peripheral`] restores this config rather than the one the driver was
        // built with.
        self.resolved = resolved;

        // Off across the reprogramming and back on afterwards, but only if it was on to begin with: this
        // method is shared by both modes, `new_async` is what enables the line, and leaving it disabled
        // strands every later async transfer — the transfer completes on the wire and nothing wakes the
        // task waiting on it.
        let was_enabled = self.info.interrupt.is_enabled();
        self.info.interrupt.disable();

        if let Some(sda) = self.sda.pin() {
            sda.update_pf(config.sda_pf());
        }

        if let Some(scl) = self.scl.pin() {
            scl.update_pf(config.scl_pf());
        }

        let configured = self.init();

        if was_enabled {
            self.info.interrupt.unpend();
            // SAFETY: re-arming a line this driver owns and had enabled a moment ago.
            unsafe { self.info.interrupt.enable() };
        }

        configured
    }

    #[inline(never)]
    pub(crate) fn init(&mut self) -> Result<(), ConfigError> {
        let resolved = self.resolved;

        self.info.regs.clksel().write(|w| match resolved.clock_source {
            ClockSel::BusClk => {
                w.set_mfclk_sel(false);
                w.set_busclk_sel(true);
            }
            ClockSel::MfClk => {
                w.set_mfclk_sel(true);
                w.set_busclk_sel(false);
            }
        });
        self.info
            .regs
            .clkdiv()
            .write(|w| w.set_ratio(resolved.clock_div.into()));

        self.info.regs.gfctl().modify(|w| {
            w.set_agfen(false);
            w.set_agfsel(vals::Agfsel::Aglit50);
            w.set_chain(true);
        });

        // Reset controller transfer, follow TI example
        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_burstrun(false);
            w.set_start(false);
            w.set_stop(false);
            w.set_ack(false);
            w.set_cackoen(false);
            w.set_rd_on_txempty(false);
            w.set_cblen(0);
        });

        self.wake_floor = resolved.wake_floor(&self.info.sleep);

        self.info.regs.controller(0).ctpr().write(|w| w.set_tpr(resolved.tpr));

        // SLAU846: the low timeout is to be configured at initialisation and not while active. Counter A
        // is the SCL-low one; B, which watches SCL high, is left alone.
        self.info.regs.timeout_ctl().modify(|w| {
            w.set_tcntaen(resolved.clock_low_timeout.is_some());
            w.set_tcntla(resolved.clock_low_timeout.unwrap_or_default());
        });

        self.info.regs.controller(0).cfifoctl().write(|w| {
            w.set_txtrig(vals::CfifoctlTxtrig::Empty);
            w.set_rxtrig(vals::CfifoctlRxtrig::Level1);
        });

        self.info.regs.controller(0).ccr().modify(|w| {
            w.set_clkstretch(true);
            w.set_active(true);
        });

        Ok(())
    }

    /// Wait for the controller to report itself idle, for a few SCL half-periods and no longer.
    ///
    /// Bounded rather than spun on, because `CSR` is not trustworthy in this window: after a timeout it
    /// reads `IDLE` and `BUSBSY` at once, permanently, which SLAU846 says cannot happen. A poll that can
    /// exit early on a wrong answer is a poll that can also never exit at all, and the second is worse.
    ///
    /// The answer is returned for callers that have something better to do with it than flush anyway.
    #[inline]
    pub fn wait_for_idle(&self) -> bool {
        let ctrl = self.info.regs.controller(0);
        let half_period = self.resolved.half_period_cycles as u32;

        for _ in 0..IDLE_HALF_PERIODS {
            if ctrl.csr().read().idle() {
                return true;
            }

            cortex_m::asm::delay(half_period);
        }

        ctrl.csr().read().idle()
    }

    /// Discard whatever an abandoned transfer left queued, driverlib's `DL_I2C_flushController*FIFO`.
    ///
    /// A cancelled write leaves its unsent bytes in the TX FIFO and a cancelled read leaves what it
    /// received in the RX FIFO. Left there, the next transfer transmits the previous one's byte and reads
    /// back the previous one's data — an error reported against a transfer that succeeded, one
    /// transaction later.
    ///
    /// SLAU846 §25.2.3.13 asks for three things around a flush and this does all of them: the controller
    /// must be idle, the FIFO interrupts must be masked first, and their flags must be dealt with after —
    /// emptying the TX FIFO raises exactly the events a finished transfer would, and left latched they
    /// would be answered by the next transfer.
    #[inline]
    pub fn flush_fifos(&mut self) {
        // Flushing under a live burst takes bytes out from under it, so idleness is worth asking for even
        // though the answer cannot be relied on.
        self.wait_for_idle();

        let ctrl = self.info.regs.controller(0);
        let int = self.info.regs.cpu_int(0);

        // Read back and restored one field at a time rather than saved and rewritten whole, so a change
        // to any other bit between here and the end of the flush survives it.
        let armed = int.imask().read();
        int.imask().modify(|w| {
            w.set_ctxfifotrg(false);
            w.set_crxfifotrg(false);
            w.set_ctxempty(false);
            w.set_crxfifofull(false);
        });

        ctrl.cfifoctl().modify(|w| {
            w.set_txflush(true);
            w.set_rxflush(true);
        });
        // Unbounded, unlike the idle poll above, and deliberately: this waits on the FIFO emptying itself
        // with the flush bits held, which is the peripheral's own doing and does not depend on the bus.
        while ctrl.cfifosr().read().txfifocnt() as usize != self.info.fifo_size
            || ctrl.cfifosr().read().rxfifocnt() != 0
        {}
        ctrl.cfifoctl().modify(|w| {
            w.set_txflush(false);
            w.set_rxflush(false);
        });

        int.iclr().write(|w| {
            w.set_ctxfifotrg(true);
            w.set_crxfifotrg(true);
            w.set_ctxempty(true);
            w.set_crxfifofull(true);
        });
        int.imask().modify(|w| {
            w.set_ctxfifotrg(armed.ctxfifotrg());
            w.set_crxfifotrg(armed.crxfifotrg());
            w.set_ctxempty(armed.ctxempty());
            w.set_crxfifofull(armed.crxfifofull());
        });
    }

    /// Reset the peripheral and put its configuration back.
    ///
    /// The escape hatch for a controller that cannot be talked round. It ends whatever burst was running
    /// at once, empties the FIFOs, releases SCL and SDA, and is the only thing that clears `BUSBSY` after a
    /// clock-low timeout — `IDLE` comes back set with `BUSBSY` still set, and SLAU846 gives the controller
    /// reset as the other way to clear it.
    ///
    /// Cheap: a few register writes and a 16-cycle settle, against the hundreds of milliseconds that
    /// waiting on a stuck bus costs.
    #[inline]
    pub fn reset_peripheral(&mut self) {
        self.info.regs.gprcm(0).rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(vals::ResetKey::Key);
        });
        self.info.regs.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });
        cortex_m::asm::delay(16);

        // Re-derives `wake_floor` too. Infallible: the config was resolved once already, and nothing about
        // the clock tree can have changed since.
        let _ = self.init();
    }

    /// Is the bus stuck with a target holding SDA low?
    ///
    /// SDA low while SCL sits idle high. Another controller mid-transaction also holds SDA low, but it would
    /// be clocking SCL, so the line is watched across a few half-periods to tell the two apart. A controller
    /// that stretches SCL low indefinitely is indistinguishable from a busy bus and reads as not stuck.
    ///
    /// This is what [`Error::BusStuck`] reports and what [`I2c::recover_stuck_bus`] acts on, so the two
    /// cannot disagree about whether there is anything to do.
    #[inline]
    pub fn bus_is_stuck(&self) -> bool {
        if self.info.regs.controller(0).cbmon().read().sda() {
            return false;
        }

        // Time is what separates a stuck target from a STOP still on the wire, which looks identical —
        // SDA low, SCL high — for up to a bit period after every NACK. Twenty half-periods is ten bit
        // times, and both questions are re-asked each pass, so the common case costs a bit period rather
        // than the whole window.
        let half = self.resolved.half_period_cycles as u32;
        for _ in 0..20 {
            cortex_m::asm::delay(half);

            let mon = self.info.regs.controller(0).cbmon().read();
            if mon.sda() {
                return false;
            }
            if !mon.scl() {
                return false;
            }
        }
        true
    }

    /// Clock a target off the bus when it is holding SDA low.
    ///
    /// Nine SCL pulses let a target that lost sync finish the byte it is stuck part-way through — eight
    /// bits and the ACK — after which a STOP leaves the bus idle.
    ///
    /// Does nothing when SDA is already high. `Err(Error::Bus)` means nine clocks did not free it, which is
    /// either a target holding SDA for good or a short to ground — neither recoverable from here.
    ///
    /// Only sound when nothing else is using the bus: it drives SCL without arbitration, so calling it while
    /// another controller is mid-transaction corrupts that transaction.
    #[inline]
    pub fn recover_stuck_bus(&mut self) -> Result<(), Error> {
        if !self.bus_is_stuck() {
            return Ok(());
        }
        let half = self.resolved.half_period_cycles as u32;

        let (Some(scl), Some(sda)) = (self.scl.pin(), self.sda.pin()) else {
            return Err(Error::Bus);
        };

        // The whole word rather than the function number: the pin is type-erased by the time it is
        // stored here, and `set_pf` rewrites the pulls and the inversion from the `PfType` it is given.
        // Synthesising one from `Pull::None` switches off an internal pull-up, and an open-drain line
        // whose pull-up is the internal one then has nothing to raise it — nine clocks with no rising
        // edge, reported as a target that will not let go.
        let scl_pincm = pac::IOMUX.pincm(scl._pin_cm() as usize).read();
        let sda_pincm = pac::IOMUX.pincm(sda._pin_cm() as usize).read();

        // `hiz1` is already set on both from `new_inner` and nothing here clears it, so a GPIO output is
        // open-drain: low is driven, high is released for the pull-up to take.
        for (pin, pincm) in [(scl, scl_pincm), (sda, sda_pincm)] {
            let pull = if pincm.pipu() {
                Pull::Up
            } else if pincm.pipd() {
                Pull::Down
            } else {
                Pull::None
            };

            pin.set_as_pf(crate::gpio::GPIO_PF, PfType::input(pull, pincm.inv()));
            pin.block().doutset31_0().write(|w| w.set_dio(pin.bit_index(), true));
            pin.block().doeset31_0().write(|w| w.set_dio(pin.bit_index(), true));
        }

        let sda_high = || sda.block().din31_0().read().dio(sda.bit_index());

        // All nine, without breaking at the first high sample: SDA goes high on any `1` bit of the byte the
        // target is still shifting out, so breaking there leaves it mid-byte and free to pull the line back
        // down before the STOP lands.
        for _ in 0..9 {
            scl.block().doutclr31_0().write(|w| w.set_dio(scl.bit_index(), true));
            cortex_m::asm::delay(half);
            scl.block().doutset31_0().write(|w| w.set_dio(scl.bit_index(), true));
            cortex_m::asm::delay(half);
        }
        let freed = sda_high();

        // STOP is SDA rising while SCL is high, so both have to be driven low first to set it up.
        scl.block().doutclr31_0().write(|w| w.set_dio(scl.bit_index(), true));
        sda.block().doutclr31_0().write(|w| w.set_dio(sda.bit_index(), true));
        cortex_m::asm::delay(half);
        scl.block().doutset31_0().write(|w| w.set_dio(scl.bit_index(), true));
        cortex_m::asm::delay(half);
        sda.block().doutset31_0().write(|w| w.set_dio(sda.bit_index(), true));
        cortex_m::asm::delay(half);

        pac::IOMUX.pincm(scl._pin_cm() as usize).write_value(scl_pincm);
        pac::IOMUX.pincm(sda._pin_cm() as usize).write_value(sda_pincm);

        // The controller watched none of that, so its idea of the bus is stale.
        self.reset_peripheral();

        if freed {
            debug!("i2c: bus recovery freed SDA");
            Ok(())
        } else {
            warn!("i2c: bus recovery clocked 9 times and SDA is still low");
            Err(Error::Bus)
        }
    }

    /// Put the peripheral back in a state the next transfer can use, after `err` ended this one.
    ///
    /// A timeout is the one failure a STOP cannot clear, so it takes the reset. Anything else only needs
    /// the bus released and the queued bytes dropped, which is what SLAU846 asks for: "if a timeout is
    /// detected before the end of a transfer, software should flush the FIFO before initializing the next
    /// transfer".
    #[inline]
    pub fn recover_after(&mut self, err: Error) {
        if err == Error::Timeout {
            self.reset_peripheral();
        } else {
            self.stop();
            self.flush_fifos();
        }
    }

    #[inline]
    /// Send a STOP on its own, ending a transfer whose last burst did not carry one.
    pub fn stop(&mut self) {
        // not the first transaction, delay 1000 cycles
        cortex_m::asm::delay(1000);

        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_cblen(0);
            w.set_stop(true);
            w.set_start(false);
        });
    }

    #[inline]
    /// Start a receiving burst of `length` bytes.
    ///
    /// `restart` sends a repeated START rather than a START, and `send_stop` ends the transfer.
    /// `Err` if `length` is past [`MAX_BURST_LEN`].
    pub fn start_read(
        &mut self,
        address: Address,
        length: usize,
        restart: bool,
        send_ack_nack: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        if length > MAX_BURST_LEN {
            return Err(Error::TransferLengthIsOverLimit);
        }

        if restart {
            // not the first transaction, delay 1000 cycles
            cortex_m::asm::delay(1000);
        }

        // START may be set even while the bus is busy or the peripheral is in target mode.
        self.info.regs.controller(0).csa().modify(|w| {
            w.set_taddr(address.addr());
            w.set_cmode(address.mode());
            w.set_dir(vals::Dir::Receive);
        });

        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_cblen(length as u16);
            w.set_burstrun(true);
            w.set_ack(send_ack_nack);
            w.set_start(true);
            w.set_stop(send_stop);
        });

        Ok(())
    }

    #[inline]
    /// Start a transmitting burst of `length` bytes, whose bytes come from the transmit FIFO.
    ///
    /// `Err` if `length` is past [`MAX_BURST_LEN`].
    pub fn start_write(&mut self, address: Address, length: usize, send_stop: bool) -> Result<(), Error> {
        if length > MAX_BURST_LEN {
            return Err(Error::TransferLengthIsOverLimit);
        }

        self.info.regs.controller(0).csa().modify(|w| {
            w.set_taddr(address.addr());
            w.set_cmode(address.mode());
            w.set_dir(vals::Dir::Transmit);
        });
        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_cblen(length as u16);
            w.set_burstrun(true);
            w.set_start(true);
            w.set_stop(send_stop);
        });

        Ok(())
    }

    /// Wait out `I2C_ERR_13` before reading `CSR` after starting a transfer.
    ///
    /// Polling `BUSY` any sooner reads it before the controller has raised it, so the wait falls straight
    /// through and the caller checks for errors against a transfer that has not happened yet. A NACK then
    /// goes unnoticed and the transfer is reported as a success.
    #[inline]
    pub fn settle_after_start(&self) {
        cortex_m::asm::delay(self.resolved.settle_cycles as u32);
    }

    /// Wait for whoever holds the bus to release it, giving up on the SCL-low timeout.
    ///
    /// A bus stuck on SDA is reported as [`Error::BusStuck`] rather than waited on, since no amount of
    /// waiting fixes it. Otherwise only a bus held *low* can time out, because counter A watches SCL low: a
    /// bus left marked busy with SCL high still waits forever, which is what counter B would be for.
    #[inline]
    pub fn blocking_wait_bus_free(&mut self) -> Result<(), Error> {
        if self.bus_is_stuck() {
            return Err(Error::BusStuck);
        }

        self.clear_timeout();
        while self.info.regs.controller(0).csr().read().busbsy() {
            if self.timed_out() {
                self.clear_timeout();
                self.reset_peripheral();
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }

    /// Has the SCL-low timeout fired? Always false unless [`Config::clock_low_timeout_us`] enabled it.
    #[inline]
    pub fn timed_out(&self) -> bool {
        self.info.regs.cpu_int(0).ris().read().timeouta()
    }

    /// Forget any timeout left over from an earlier transfer, so it is not blamed on the next one.
    #[inline]
    pub fn clear_timeout(&self) {
        self.info.regs.cpu_int(0).iclr().write(|w| w.set_timeouta(true));
    }

    /// Turn whatever the controller latched into an error for the caller.
    ///
    /// Ordered by how fundamental the failure is. A timeout means the bus never gave the transfer a
    /// chance, so it outranks a NACK that may just be the tail of it.
    #[inline]
    pub fn check_error(&self) -> Result<(), Error> {
        if self.timed_out() {
            self.clear_timeout();
            return Err(Error::Timeout);
        }

        let csr = self.info.regs.controller(0).csr().read();
        if csr.arblst() {
            return Err(Error::Arbitration);
        }
        if csr.err() {
            return Err(self.nack_kind());
        }
        Ok(())
    }

    /// Push what fits into the transmit FIFO, returning how many bytes went in.
    ///
    /// `TXFIFOCNT` counts the space left, not what is queued.
    #[inline]
    pub fn fill_tx(&self, bytes: &[u8]) -> usize {
        let ctrl = self.info.regs.controller(0);
        let mut sent = 0;

        while sent < bytes.len() && ctrl.cfifosr().read().txfifocnt() != 0 {
            ctrl.ctxdata().write(|w| w.set_value(bytes[sent]));
            sent += 1;
        }

        sent
    }

    /// Take what the receive FIFO holds, returning how many bytes came out.
    #[inline]
    pub fn drain_rx(&self, into: &mut [u8]) -> usize {
        let ctrl = self.info.regs.controller(0);
        let mut got = 0;

        while got < into.len() && ctrl.cfifosr().read().rxfifocnt() != 0 {
            into[got] = ctrl.crxdata().read().value();
            got += 1;
        }

        got
    }

    /// Push what fits into the transmit FIFO from a run of write operations, returning how many went in.
    ///
    /// The run moves as one burst, so the FIFO is fed from each operation's buffer in turn.
    #[inline]
    pub(crate) fn fill_tx_group(
        &self,
        ops: &[embedded_hal::i2c::Operation<'_>],
        end: usize,
        cur: &mut GroupCursor,
    ) -> usize {
        let ctrl = self.info.regs.controller(0);
        let mut sent = 0;

        while cur.op < end {
            let embedded_hal::i2c::Operation::Write(buf) = &ops[cur.op] else {
                break;
            };

            if cur.pos == buf.len() {
                cur.op += 1;
                cur.pos = 0;
                continue;
            }

            if ctrl.cfifosr().read().txfifocnt() == 0 {
                break;
            }

            ctrl.ctxdata().write(|w| w.set_value(buf[cur.pos]));
            cur.pos += 1;
            sent += 1;
        }

        sent
    }

    /// Take what the receive FIFO holds into a run of read operations, returning how many came out.
    #[inline]
    pub(crate) fn drain_rx_group(
        &self,
        ops: &mut [embedded_hal::i2c::Operation<'_>],
        end: usize,
        cur: &mut GroupCursor,
    ) -> usize {
        let ctrl = self.info.regs.controller(0);
        let mut got = 0;

        while cur.op < end {
            let embedded_hal::i2c::Operation::Read(buf) = &mut ops[cur.op] else {
                break;
            };

            if cur.pos == buf.len() {
                cur.op += 1;
                cur.pos = 0;
                continue;
            }

            if ctrl.cfifosr().read().rxfifocnt() == 0 {
                break;
            }

            buf[cur.pos] = ctrl.crxdata().read().value();
            cur.pos += 1;
            got += 1;
        }

        got
    }

    /// Which half of the transfer went unanswered.
    ///
    /// `ADRACK` and `DATACK` are the difference between nothing being at that address and the target being
    /// there but rejecting a byte. The async paths need this separately because they learn about a NACK from
    /// the interrupt rather than from [`I2c::check_error`], and would otherwise report the same failure less
    /// precisely than the blocking ones.
    #[inline]
    pub fn nack_kind(&self) -> Error {
        let csr = self.info.regs.controller(0).csr().read();
        match (csr.adrack(), csr.datack()) {
            (true, _) => Error::NackAddress,
            (false, true) => Error::NackData,
            (false, false) => Error::Nack,
        }
    }
}

// ==== Reached with a bare register block ====
//
// The cancellation guard runs from `OnDrop` and cannot hold a `&mut I2c`, so the two things it does
// are here as well as on the driver.

/// Take the instance out of reset and power it, then wait out the bus-isolation update.
///
/// A register write immediately after `PWREN.ENABLE` is dropped while that update runs, and `PWREN`
/// reads back true throughout — so the wait is the only thing standing between this and a
/// configuration that silently did not land.
#[inline]
pub(crate) fn power_up(regs: crate::pac::i2c::I2c) {
    regs.gprcm(0).rstctl().write(|w| {
        w.set_resetstkyclr(true);
        w.set_resetassert(true);
        w.set_key(vals::ResetKey::Key);
    });

    regs.gprcm(0).pwren().write(|w| {
        w.set_enable(true);
        w.set_key(vals::PwrenKey::Key);
    });

    // init delay, 16 cycles
    cortex_m::asm::delay(16);
}

/// Mask every source.
#[inline]
pub(crate) fn disarm(regs: crate::pac::i2c::I2c) {
    regs.cpu_int(0).imask().write_value(i2c::regs::CpuInt::default());
}

/// Record that a transfer was abandoned part-way.
#[inline]
pub(crate) fn mark_abandoned(state: &State) {
    state.abandoned.store(true, Ordering::Relaxed);
}
