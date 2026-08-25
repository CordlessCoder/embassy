//! Register-level UART, for an application that services the interrupt itself.
//!
//! [`Uart`] here powers the instance up, claims the pins and programs a [`Config`], and then stops.
//! It queues no bytes for you, waits for nothing, and installs no interrupt handler — what it adds
//! over the raw registers is the configuration, the pin ownership and the sleep guard that keeps the
//! instance's setup alive across a deep sleep.
//!
//! [`Uart<Blocking>`](super::Uart) and [`Uart<Async>`](super::Uart) are both built on this, and hold
//! one. Reach for this one when neither fits: an RTIC hardware task, another executor, or a driver
//! whose servicing does not decompose into calls that return.
//!
//! # What the caller takes on
//!
//! **The interrupt.** Nothing here routes a vector or unmasks the NVIC line — `new` leaves the line
//! exactly as it found it. [`Uart::interrupt`] names it so the application can enable it, and
//! [`Uart::enable_interrupt`] arms the sources within the peripheral.
//!
//! **The FIFO.** [`Uart::try_read`] and [`Uart::try_write`] move one byte and report whether there
//! was one to move. Draining a receive FIFO before returning from a handler is the caller's job, and
//! it is what stops the next byte overrunning — the FIFOs are four entries deep.
//!
//! **Clearing what fired.** [`Uart::clear_pending`] writes `ICLR`. `RIS` is sticky, so a source left
//! set re-raises the line as soon as the handler returns.
//!
//! ```ignore
//! #[task(binds = UART0, local = [uart, ring])]
//! fn on_uart(cx: on_uart::Context) {
//!     let uart = cx.local.uart;
//!
//!     if uart.is_pending(Event::Rx) || uart.is_pending(Event::RxTimeout) {
//!         uart.clear_pending(Event::Rx);
//!         uart.clear_pending(Event::RxTimeout);
//!
//!         while let Some(byte) = uart.try_read() {
//!             cx.local.ring.push(byte);
//!         }
//!     }
//! }
//! ```
//!
//! The clear comes before the drain, not after. A byte that arrives while the drain is running sets
//! `RIS` again, and clearing afterwards discards that flag with the byte still in the FIFO — the
//! handler then never runs again until something else raises the line. Clearing first costs at worst
//! one spurious entry, which finds nothing to move.
//!
//! # The instance is not erased by mistake
//!
//! These types carry no `T`. The register block, the interrupt number and the sleep information are
//! read once in `new` and kept behind a `&'static`, so a chip with four instances links one copy of
//! each method rather than four. [`low_level::Timer`](crate::tim::low_level::Timer) makes the other
//! choice, and its own documentation says what that costs.
//!
//! # Every method here is `#[inline]`, and that is not a style choice
//!
//! Erasing the instance costs something the generic drivers never paid, and the attributes are what
//! pay it back. A method that is public and *not* generic is compiled into this crate's rlib whether
//! or not any binary calls it, and it takes a `&self` holding the instance's `&'static State`. That
//! makes the static's address escape into a function the optimiser cannot see the end of, so the
//! clock [`configure`] stores there can never be proved dead — and the clock-tree read behind it,
//! and the 40-byte `CLOCKS` static behind that.
//!
//! Measured on a blocking-only transmit binary: 12 bytes of text and 40 bytes of `.data`, on a
//! program of 1092. `#[inline]` restores it exactly, by making these compile on demand in a caller's
//! unit rather than unconditionally in ours.
//!
//! The mode drivers were immune by accident. Every one of their methods is generic over
//! [`Mode`](crate::mode::Mode), so an uncalled one is never instantiated and never escapes anything.
//! Nothing about the erasure here is wrong — but a method added below without the attribute puts the
//! 52 bytes back in every binary that builds a UART, and no build fails.

use core::sync::atomic::{AtomicU32, Ordering};

use super::{
    Baud, BaudRate, BitOrder, ClockSel, Config, ConfigError, CtsPin, DataBits, Error, FifoThreshold, Instance, Parity,
    RtsPin, RxPin, StopBits, TxPin,
};
use crate::Peri;
use crate::gpio::{AnyPin, MaybeAnyPin, SealedPin};
use crate::interrupt::{Interrupt, InterruptExt};
use crate::pac::uart::regs::{self, CpuInt};
use crate::pac::uart::{Uart as Regs, vals};
use crate::sysctl::{MaybeWakeGuard, PowerDomain, SleepInfo, SleepLevel};

/// Bit times of silence after which the receiver reports a FIFO that has not reached its level.
///
/// Must exceed 1: `UART_ERR_11` starts the counter in the middle of the STOP bit, so 1 fires early. The
/// resulting timeout is `(RX_TIMEOUT_BITS - 0.5) / baud`, so this is a little under one character —
/// short enough that a trailing byte is not held up, long enough that a back-to-back stream never
/// reaches it.
const RX_TIMEOUT_BITS: u8 = 8;

/// A condition the instance can raise its interrupt on.
///
/// One variant per source this driver's configuration can actually reach. The register has more —
/// LIN capture, the address match, the DMA completions and the edge detectors — and every one of them
/// needs a mode [`Config`] cannot select, so arming it here would read back set and never fire.
///
/// `NERR`, the majority-voting disagreement, is left out for the same reason: [`configure`] clears
/// `CTL0.MAJVOTE` and nothing exposes it yet.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Event {
    /// The receive FIFO reached the level [`Config::fifo`] set.
    Rx,

    /// Reception stopped with the receive FIFO below its level, so [`Event::Rx`] will not arrive.
    ///
    /// Needs a FIFO level above one entry to be reachable at all — [`configure`] leaves `RXTOSEL` at
    /// zero without one, which is what disables the counter.
    RxTimeout,

    /// The transmit FIFO fell to the level [`Config::fifo`] set, so there is room to queue more.
    Tx,

    /// The last bit of the last queued byte left the shift register.
    EndOfTransmission,

    /// A byte arrived without the stop bit its frame promised.
    Framing,

    /// A byte arrived whose parity disagreed with [`Config::parity`].
    Parity,

    /// The line was held low for longer than a whole frame.
    Break,

    /// A byte arrived with the receive FIFO already full, and was lost.
    Overrun,

    /// The clear-to-send input changed level.
    Cts,

    /// Anything a receive waits on: the FIFO reaching its level, or the timeout that delivers a
    /// message shorter than it.
    ///
    /// A group, so one [`take_active`](Uart::take_active) answers "is there something to read" and
    /// clears whichever half of it fired.
    AnyReceive,

    /// Any line fault: [`Framing`](Event::Framing), [`Parity`](Event::Parity),
    /// [`Break`](Event::Break) or [`Overrun`](Event::Overrun).
    AnyError,
}

impl Event {
    pub(crate) const fn mask(self) -> CpuInt {
        let mut mask = CpuInt(0);

        match self {
            Event::Rx => mask.set_rxint(true),
            Event::RxTimeout => mask.set_rtout(true),
            Event::Tx => mask.set_txint(true),
            Event::EndOfTransmission => mask.set_eot(true),
            Event::Framing => mask.set_frmerr(true),
            Event::Parity => mask.set_parerr(true),
            Event::Break => mask.set_brkerr(true),
            Event::Overrun => mask.set_ovrerr(true),
            Event::Cts => mask.set_cts(true),
            Event::AnyReceive => return rx_sources(),
            Event::AnyError => return error_sources(),
        }

        mask
    }
}

/// Transmitting half of a register-level UART.
pub struct UartTx<'d> {
    pub(crate) info: &'static Info,
    pub(crate) state: &'static State,
    pub(crate) tx: MaybeAnyPin<'d>,
    pub(crate) cts: MaybeAnyPin<'d>,
    /// Held for as long as the driver exists; see [`SleepInfo::floor_to_keep_configured`].
    _retention_guard: MaybeWakeGuard,
}

/// Receiving half of a register-level UART.
pub struct UartRx<'d> {
    pub(crate) info: &'static Info,
    pub(crate) state: &'static State,
    pub(crate) rx: MaybeAnyPin<'d>,
    pub(crate) rts: MaybeAnyPin<'d>,
    /// Held for as long as the driver exists; see [`SleepInfo::floor_to_keep_configured`].
    _retention_guard: MaybeWakeGuard,
}

/// A register-level UART, powered up and configured, servicing nothing.
///
/// See the [module documentation](self) for what the caller takes on.
pub struct Uart<'d> {
    pub(crate) tx: UartTx<'d>,
    pub(crate) rx: UartRx<'d>,
}

impl<'d> Uart<'d> {
    /// Power up an instance and configure it for full duplex, with no flow control.
    #[inline]
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, config.tx_pf()),
            new_pin!(rx, config.rx_pf()),
            None,
            None,
            config,
        )
    }

    /// Power up an instance and configure it for full duplex, with hardware flow control.
    #[inline]
    pub fn new_with_rtscts<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        rts: Peri<'d, impl RtsPin<T>>,
        cts: Peri<'d, impl CtsPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, config.tx_pf()),
            new_pin!(rx, config.rx_pf()),
            new_pin!(rts, config.rts_pf()),
            new_pin!(cts, config.cts_pf()),
            config,
        )
    }

    #[inline(always)]
    pub(crate) fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        tx: Option<Peri<'d, AnyPin>>,
        rx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let this = Self {
            tx: UartTx::build::<T>(tx, cts),
            rx: UartRx::build::<T>(rx, rts),
        };
        this.enable_and_configure(&config)?;

        Ok(this)
    }

    #[inline(always)]
    pub(crate) fn enable_and_configure(&self, config: &Config) -> Result<(), ConfigError> {
        let info = self.rx.info;

        enable(info.regs);
        configure(
            info,
            self.rx.state,
            config,
            true,
            self.rx.rts.is_some(),
            true,
            self.tx.cts.is_some(),
        )
    }

    /// Split into the two halves, so each can be given away separately.
    #[inline]
    pub fn split(self) -> (UartTx<'d>, UartRx<'d>) {
        (self.tx, self.rx)
    }

    /// Borrow the two halves separately.
    #[inline]
    pub fn split_ref(&mut self) -> (&mut UartTx<'d>, &mut UartRx<'d>) {
        (&mut self.tx, &mut self.rx)
    }

    /// Registers of this instance, for what this driver does not wrap.
    #[inline]
    pub fn regs(&self) -> Regs {
        self.rx.info.regs
    }

    /// The interrupt line this instance raises.
    ///
    /// Nothing here enables it. See the [module documentation](self).
    #[inline]
    pub fn interrupt(&self) -> Interrupt {
        self.rx.info.interrupt
    }

    /// Take one byte from the receive FIFO, or `None` if it is empty.
    #[inline]
    pub fn try_read(&mut self) -> Option<Result<u8, Error>> {
        self.rx.try_read()
    }

    /// Put one byte in the transmit FIFO, reporting `false` if it was full.
    #[must_use = "the byte is dropped when the FIFO is full"]
    #[inline]
    pub fn try_write(&mut self, byte: u8) -> bool {
        self.tx.try_write(byte)
    }

    /// Whether the receive FIFO holds nothing.
    #[inline]
    pub fn is_rx_empty(&self) -> bool {
        self.rx.is_rx_empty()
    }

    /// Whether the transmit FIFO has no room.
    #[inline]
    pub fn is_tx_full(&self) -> bool {
        self.tx.is_tx_full()
    }

    /// Whether the transmitter still holds a byte. See [`UartTx::busy`].
    #[inline]
    pub fn busy(&self) -> bool {
        self.tx.busy()
    }

    /// Hold the transmit line low until [`clear_break`](Self::clear_break).
    ///
    /// Takes effect once the character being transmitted finishes. SLAU846 table 24-40 asks for the
    /// line to be held for at least two character periods before it is lifted, and that wait is the
    /// caller's: sizing a spin needs a CPU clock this driver does not own.
    #[inline]
    pub fn set_break(&mut self) {
        self.tx.set_break();
    }

    /// Release a break started with [`set_break`](Self::set_break).
    #[inline]
    pub fn clear_break(&mut self) {
        self.tx.clear_break();
    }

    /// Let `event` reach the CPU, or stop it.
    ///
    /// A read-modify-write on `IMASK`, which an interrupt handler writes as well.
    #[inline]
    pub fn enable_interrupt(&mut self, event: Event, enable: bool) {
        enable_interrupt(self.rx.info.regs, event, enable);
    }

    /// Whether `event` is latched, whether or not it is unmasked.
    #[inline]
    pub fn is_pending(&self, event: Event) -> bool {
        is_pending(self.rx.info.regs, event)
    }

    /// Drop `event`'s latched flag.
    #[inline]
    pub fn clear_pending(&mut self, event: Event) {
        clear_pending(self.rx.info.regs, event);
    }

    /// Whether `event` is latched **and** unmasked, clearing the part of it that is.
    ///
    /// What raised the line, which is what a handler dispatches on: one `MIS` read and at most one
    /// `ICLR` write, whatever the event covers. [`Event::AnyReceive`] makes it the whole receive
    /// group, so "is there something to read" is one call rather than two.
    ///
    /// Leaves a latched-but-masked source alone, so a source the caller has not armed keeps its flag
    /// for whoever does ask. [`is_pending`](Self::is_pending) reads `RIS` and ignores the mask.
    #[inline]
    pub fn take_active(&mut self, event: Event) -> bool {
        take_active(self.rx.info.regs, event)
    }

    /// Take one byte from the receive FIFO, keeping it even when the receiver faulted it.
    ///
    /// [`try_read`](Self::try_read) reports a fault as an `Err` and **discards the byte**, which is
    /// wrong for an overrun: the loss happened before this byte, and the byte itself is good. A
    /// caller draining into its own buffer wants this one.
    #[inline]
    pub fn try_read_flagged(&mut self) -> Option<(u8, Option<Error>)> {
        self.rx.try_read_flagged()
    }

    /// Shallowest sleep level that keeps a transmission running, if one is needed at all.
    ///
    /// See [`UartTx::transmit_floor`].
    #[inline]
    pub fn transmit_floor(&self) -> Option<SleepLevel> {
        self.tx.transmit_floor()
    }

    /// Apply a new [`Config`], keeping the pins.
    #[inline]
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.tx.update_pins(config);
        self.rx.update_pins(config);

        reconfigure(self.rx.info, self.rx.state, config)
    }

    /// Set the baud rate, leaving the rest of the configuration alone.
    #[inline]
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        set_baudrate(self.rx.info, self.rx.state.clock.load(Ordering::Relaxed), baudrate)
    }
}

impl<'d> UartTx<'d> {
    /// Power up an instance and configure it to transmit only.
    #[inline]
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(peri, new_pin!(tx, config.tx_pf()), None, config)
    }

    /// Power up an instance and configure it to transmit only, with a clear-to-send pin.
    #[inline]
    pub fn new_with_cts<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        cts: Peri<'d, impl CtsPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, config.tx_pf()),
            new_pin!(cts, config.cts_pf()),
            config,
        )
    }

    #[inline(always)]
    pub(crate) fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        tx: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let this = Self::build::<T>(tx, cts);
        this.enable_and_configure(&config)?;

        Ok(this)
    }

    /// The struct alone, with the peripheral untouched. [`Uart`] configures both halves at once.
    #[inline(always)]
    fn build<T: Instance>(tx: Option<Peri<'d, AnyPin>>, cts: Option<Peri<'d, AnyPin>>) -> Self {
        Self {
            info: T::info(),
            state: T::state(),
            tx: MaybeAnyPin::new(tx),
            cts: MaybeAnyPin::new(cts),
            _retention_guard: retention_guard(T::info()),
        }
    }

    #[inline(always)]
    fn enable_and_configure(&self, config: &Config) -> Result<(), ConfigError> {
        enable(self.info.regs);
        configure(self.info, self.state, config, false, false, true, self.cts.is_some())
    }

    /// Registers of this instance, for what this driver does not wrap.
    #[inline]
    pub fn regs(&self) -> Regs {
        self.info.regs
    }

    /// The interrupt line this instance raises.
    #[inline]
    pub fn interrupt(&self) -> Interrupt {
        self.info.interrupt
    }

    /// Put one byte in the transmit FIFO, reporting `false` if it was full.
    #[must_use = "the byte is dropped when the FIFO is full"]
    #[inline]
    pub fn try_write(&mut self, byte: u8) -> bool {
        let r = self.info.regs;

        if tx_full(r) {
            return false;
        }

        write_byte(r, byte);

        true
    }

    /// Whether the transmit FIFO has no room.
    #[inline]
    pub fn is_tx_full(&self) -> bool {
        tx_full(self.info.regs)
    }

    /// Whether the transmitter still holds a byte.
    ///
    /// Answers "the transmit FIFO still has something in it", which is one bit time short of the wire
    /// going idle — see the [module documentation](super) on waiting out that last bit.
    #[inline]
    pub fn busy(&self) -> bool {
        busy(self.info.regs)
    }

    /// Hold the transmit line low until [`clear_break`](Self::clear_break).
    ///
    /// Takes effect once the character being transmitted finishes. SLAU846 table 24-40 asks for the
    /// line to be held for at least two character periods before it is lifted, and that wait is the
    /// caller's: sizing a spin needs a CPU clock this driver does not own.
    #[inline]
    pub fn set_break(&mut self) {
        set_break(self.info.regs, true);
    }

    /// Release a break started with [`set_break`](Self::set_break).
    #[inline]
    pub fn clear_break(&mut self) {
        set_break(self.info.regs, false);
    }

    /// Shallowest sleep level that keeps a transmission running, if one is needed at all.
    ///
    /// Deep sleep entered while bytes are still going out cuts the frame mid-byte, and on an instance
    /// in PD1 the transmit pin then sits low until the next wake. Hold a
    /// [`WakeGuard`](crate::sysctl::WakeGuard) at this level from before the first byte is queued
    /// until [`busy`](Self::busy) reads false.
    ///
    /// `None` means no level blocks it, so nothing has to be held.
    #[inline]
    pub fn transmit_floor(&self) -> Option<SleepLevel> {
        self.info
            .sleep
            .floor_for_operation(self.state.clock.load(Ordering::Relaxed))
    }

    /// Let `event` reach the CPU, or stop it.
    ///
    /// A read-modify-write on `IMASK`, which an interrupt handler writes as well.
    #[inline]
    pub fn enable_interrupt(&mut self, event: Event, enable: bool) {
        enable_interrupt(self.info.regs, event, enable);
    }

    /// Whether `event` is latched, whether or not it is unmasked.
    #[inline]
    pub fn is_pending(&self, event: Event) -> bool {
        is_pending(self.info.regs, event)
    }

    /// Drop `event`'s latched flag.
    #[inline]
    pub fn clear_pending(&mut self, event: Event) {
        clear_pending(self.info.regs, event);
    }

    /// Whether `event` is latched **and** unmasked, clearing the part of it that is.
    ///
    /// What raised the line, which is what a handler dispatches on: one `MIS` read and at most one
    /// `ICLR` write, whatever the event covers. [`Event::AnyReceive`] makes it the whole receive
    /// group, so "is there something to read" is one call rather than two.
    ///
    /// Leaves a latched-but-masked source alone, so a source the caller has not armed keeps its flag
    /// for whoever does ask. [`is_pending`](Self::is_pending) reads `RIS` and ignores the mask.
    #[inline]
    pub fn take_active(&mut self, event: Event) -> bool {
        take_active(self.info.regs, event)
    }

    /// Apply a new [`Config`], keeping the pins.
    #[inline]
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.update_pins(config);

        reconfigure(self.info, self.state, config)
    }

    /// Set the baud rate, leaving the rest of the configuration alone.
    #[inline]
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        set_baudrate(self.info, self.state.clock.load(Ordering::Relaxed), baudrate)
    }

    fn update_pins(&self, config: &Config) {
        if let Some(tx) = self.tx.pin() {
            tx.update_pf(config.tx_pf());
        }

        if let Some(cts) = self.cts.pin() {
            cts.update_pf(config.cts_pf());
        }
    }
}

impl<'d> Drop for UartTx<'d> {
    fn drop(&mut self) {
        if let Some(pin) = self.tx.pin() {
            pin.set_as_disconnected();
        }
        if let Some(pin) = self.cts.pin() {
            pin.set_as_disconnected();
        }
    }
}

impl<'d> UartRx<'d> {
    /// Power up an instance and configure it to receive only.
    #[inline]
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(peri, new_pin!(rx, config.rx_pf()), None, config)
    }

    /// Power up an instance and configure it to receive only, with a request-to-send pin.
    #[inline]
    pub fn new_with_rts<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        rts: Peri<'d, impl RtsPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, config.rx_pf()),
            new_pin!(rts, config.rts_pf()),
            config,
        )
    }

    #[inline(always)]
    pub(crate) fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let this = Self::build::<T>(rx, rts);
        this.enable_and_configure(&config)?;

        Ok(this)
    }

    /// The struct alone, with the peripheral untouched. [`Uart`] configures both halves at once.
    #[inline(always)]
    fn build<T: Instance>(rx: Option<Peri<'d, AnyPin>>, rts: Option<Peri<'d, AnyPin>>) -> Self {
        Self {
            info: T::info(),
            state: T::state(),
            rx: MaybeAnyPin::new(rx),
            rts: MaybeAnyPin::new(rts),
            _retention_guard: retention_guard(T::info()),
        }
    }

    #[inline(always)]
    fn enable_and_configure(&self, config: &Config) -> Result<(), ConfigError> {
        enable(self.info.regs);
        configure(self.info, self.state, config, true, self.rts.is_some(), false, false)
    }

    /// Registers of this instance, for what this driver does not wrap.
    #[inline]
    pub fn regs(&self) -> Regs {
        self.info.regs
    }

    /// The interrupt line this instance raises.
    #[inline]
    pub fn interrupt(&self) -> Interrupt {
        self.info.interrupt
    }

    /// Take one byte from the receive FIFO, or `None` if it is empty.
    ///
    /// The error is the one the byte itself carries, so it names what was wrong with the byte being
    /// returned rather than reporting a condition of the receiver.
    #[inline]
    pub fn try_read(&mut self) -> Option<Result<u8, Error>> {
        let r = self.info.regs;

        if rx_empty(r) {
            return None;
        }

        Some(read_with_error(r))
    }

    /// Whether the receive FIFO holds nothing.
    #[inline]
    pub fn is_rx_empty(&self) -> bool {
        rx_empty(self.info.regs)
    }

    /// Let `event` reach the CPU, or stop it.
    ///
    /// A read-modify-write on `IMASK`, which an interrupt handler writes as well.
    #[inline]
    pub fn enable_interrupt(&mut self, event: Event, enable: bool) {
        enable_interrupt(self.info.regs, event, enable);
    }

    /// Whether `event` is latched, whether or not it is unmasked.
    #[inline]
    pub fn is_pending(&self, event: Event) -> bool {
        is_pending(self.info.regs, event)
    }

    /// Drop `event`'s latched flag.
    #[inline]
    pub fn clear_pending(&mut self, event: Event) {
        clear_pending(self.info.regs, event);
    }

    /// Whether `event` is latched **and** unmasked, clearing the part of it that is.
    ///
    /// What raised the line, which is what a handler dispatches on: one `MIS` read and at most one
    /// `ICLR` write, whatever the event covers. [`Event::AnyReceive`] makes it the whole receive
    /// group, so "is there something to read" is one call rather than two.
    ///
    /// Leaves a latched-but-masked source alone, so a source the caller has not armed keeps its flag
    /// for whoever does ask. [`is_pending`](Self::is_pending) reads `RIS` and ignores the mask.
    #[inline]
    pub fn take_active(&mut self, event: Event) -> bool {
        take_active(self.info.regs, event)
    }

    /// Take one byte from the receive FIFO, keeping it even when the receiver faulted it.
    ///
    /// [`try_read`](Self::try_read) reports a fault as an `Err` and **discards the byte**, which is
    /// wrong for an overrun: the loss happened before this byte, and the byte itself is good. A
    /// caller draining into its own buffer wants this one.
    #[inline]
    pub fn try_read_flagged(&mut self) -> Option<(u8, Option<Error>)> {
        let r = self.info.regs;

        if rx_empty(r) {
            return None;
        }

        let rx = read_raw(r);

        Some((rx.data(), fault_of(rx)))
    }

    /// Apply a new [`Config`], keeping the pins.
    #[inline]
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.update_pins(config);

        reconfigure(self.info, self.state, config)
    }

    /// Set the baud rate, leaving the rest of the configuration alone.
    #[inline]
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        set_baudrate(self.info, self.state.clock.load(Ordering::Relaxed), baudrate)
    }

    fn update_pins(&self, config: &Config) {
        if let Some(rx) = self.rx.pin() {
            rx.update_pf(config.rx_pf());
        }

        if let Some(rts) = self.rts.pin() {
            rts.update_pf(config.rts_pf());
        }
    }
}

impl<'d> Drop for UartRx<'d> {
    fn drop(&mut self) {
        if let Some(pin) = self.rx.pin() {
            pin.set_as_disconnected();
        }
        if let Some(pin) = self.rts.pin() {
            pin.set_as_disconnected();
        }
    }
}

// ==== The register work the mode drivers share ====
//
// Everything below is what the types above wrap, and every one of them takes a bare register block
// rather than a `&self`.
//
// **That is what makes them reachable from the mode drivers, and it is not a stylistic preference.**
// The async futures capture the register block by value instead of borrowing their driver, and
// `TxWrite` holds one next to a `PhantomData` for the same reason — a `Drop` type with a route to the
// whole driver stops the compiler proving nothing reads the clock `configure` stores, measured at 164
// bytes. Neither can hold a `&low_level::UartTx`, so a primitive offered only as a method is a
// primitive they have to open-code.
//
// So: a method above is a one-line wrapper over one of these, never the other way round. Adding a
// register access to a method body puts a second copy of it in this file.

pub(crate) fn enable_interrupt(regs: Regs, event: Event, enable: bool) {
    let mask = event.mask().0;

    regs.cpu_int(0).imask().modify(|w| {
        w.0 = if enable { w.0 | mask } else { w.0 & !mask };
    });
}

pub(crate) fn is_pending(regs: Regs, event: Event) -> bool {
    regs.cpu_int(0).ris().read().0 & event.mask().0 != 0
}

pub(crate) fn clear_pending(regs: Regs, event: Event) {
    regs.cpu_int(0).iclr().write_value(event.mask());
}

/// Clear the given sources so a later unmask reflects what happens from here on.
pub(crate) fn clear(r: Regs, sources: CpuInt) {
    r.cpu_int(0).iclr().write_value(sources);
}

/// Let the given sources reach the CPU.
pub(crate) fn unmask(r: Regs, sources: CpuInt) {
    r.cpu_int(0).imask().modify(|w| w.0 |= sources.0);
}

/// Stop the given sources reaching the CPU.
pub(crate) fn mask(r: Regs, sources: CpuInt) {
    r.cpu_int(0).imask().modify(|w| w.0 &= !sources.0);
}

/// Clear whichever of `event`'s sources are both latched and unmasked, reporting whether any were.
#[inline]
pub(crate) fn take_active(r: Regs, event: Event) -> bool {
    let active = masked_status(r).0 & event.mask().0;

    if active != 0 {
        clear(r, CpuInt(active));
    }

    active != 0
}

/// Every source that is both latched and unmasked, in one read.
///
/// What a handler dispatches on. [`is_pending`] answers per event and reads `RIS`, so it reports a
/// source that is latched but masked — right for a caller polling one it never armed, wrong for
/// deciding what raised the line.
pub(crate) fn masked_status(r: Regs) -> CpuInt {
    r.cpu_int(0).mis().read()
}

/// Whether the receive FIFO holds nothing.
pub(crate) fn rx_empty(r: Regs) -> bool {
    r.stat().read().rxfe()
}

/// Whether the transmit FIFO has no room.
///
/// Both this and [`rx_empty`] track `CTL0.FEN`, so they read correctly with the FIFOs off, where the
/// depth is one byte.
pub(crate) fn tx_full(r: Regs) -> bool {
    r.stat().read().txff()
}

/// Queue one byte, having already found room with [`tx_full`].
pub(crate) fn write_byte(r: Regs, byte: u8) {
    r.txdata().write(|w| w.set_data(byte));
}

/// The `RXDATA` overrun bit, built from its own setter so it cannot drift from the register.
///
/// The only fault bit anything needs by value: the buffered driver counts the bytes an overrun lost,
/// which is a different question from which fault to report. [`fault_of`] answers that one, through
/// the register's own accessors, and is the only other place the layout is written.
pub(crate) const RX_OVERRUN: u8 = {
    let mut w = regs::Rxdata(0);
    w.set_ovrerr(true);
    (w.0 >> 8) as u8
};

/// The `CPU_INT` sources for the four faults [`Event`] names.
///
/// `NERR` is not among them: majority voting is off and nothing configures it, so the source cannot
/// fire. [`buffered`](super::buffered) arms it anyway because it always has.
pub(crate) const fn error_sources() -> CpuInt {
    let mut sources = CpuInt(0);
    sources.set_frmerr(true);
    sources.set_parerr(true);
    sources.set_brkerr(true);
    sources.set_ovrerr(true);
    sources
}

/// The fault a received word carries, taking the lowest flag bit set.
///
/// A byte can arrive with several faults and `Error` names one, so the order is a decision: **least
/// significant bit first**, which is framing, parity, break, overrun, noise. It is the register's own
/// order and it is now the only one — the buffered driver used to run the ladder the other way, so
/// the two drivers named a doubly-faulted byte differently.
///
/// Takes the whole word rather than [`read_flagged`]'s mask: the byte and the flags come out of one
/// read either way, and testing the fields directly is what the accessors lower to. Going through
/// the shifted mask instead measured 8 bytes on a blocking receive.
#[inline]
pub(crate) const fn fault_of(rx: regs::Rxdata) -> Option<Error> {
    if rx.frmerr() {
        Some(Error::Framing)
    } else if rx.parerr() {
        Some(Error::Parity)
    } else if rx.brkerr() {
        Some(Error::Break)
    } else if rx.ovrerr() {
        Some(Error::Overrun)
    } else if rx.nerr() {
        Some(Error::Noise)
    } else {
        None
    }
}

/// The fault an accumulated flag mask reports, by the same rule as [`fault_of`].
///
/// Puts the flags back where the register had them so there is one ladder rather than two facing
/// each other. The caller here is off the receive path — it reports faults already counted — so the
/// shift costs nothing that matters.
#[inline]
pub(crate) const fn fault_of_flags(flags: u8) -> Option<Error> {
    fault_of(regs::Rxdata((flags as u32) << 8))
}

/// One byte, and the fault bits the receiver tagged it with as a raw mask.
///
/// [`read_with_error`] is the same read reported as a `Result`, which keeps only the first fault and
/// discards the byte. A driver that counts faults, or that wants the byte an overrun arrived with,
/// needs both halves.
#[inline]
pub(crate) fn read_flagged(r: Regs) -> (u8, u8) {
    let data = read_raw(r);

    (data.data(), (data.0 >> 8) as u8)
}

/// The receive word: one byte and the flags the receiver tagged it with, unseparated.
#[inline]
pub(crate) fn read_raw(r: Regs) -> regs::Rxdata {
    r.rxdata().read()
}

/// The receive FIFO level `threshold` selects, as the register encodes it for an instance in `domain`.
///
/// **A PD0 instance has only two levels**, one entry and full, encoded differently from every other
/// instance's; SLAU846 Table 24-44 says anything else falls back to the reset value. Rather than leave
/// that silent, everything from half up takes the full level.
///
/// Rounding *up* is measured, not a guess. On a G3507, whose `UART1` is PD0, half-mapped-to-full
/// receives a 921600 baud stream with 0.27% loss where half-mapped-to-one-entry loses 19%. The finer
/// levels a non-PD0 instance has are the untested path here.
const fn rx_level(threshold: FifoThreshold, domain: PowerDomain) -> vals::Iflssel {
    match (domain, threshold) {
        (PowerDomain::Pd0, FifoThreshold::AtLeastOne | FifoThreshold::Quarter) => vals::Iflssel::OneFourthUlp,
        (PowerDomain::Pd0, FifoThreshold::Half | FifoThreshold::ThreeQuarter | FifoThreshold::Full) => {
            vals::Iflssel::FullUlp
        }
        (_, FifoThreshold::AtLeastOne) => vals::Iflssel::AtLeastOne,
        (_, FifoThreshold::Quarter) => vals::Iflssel::OneFourth,
        (_, FifoThreshold::Half) => vals::Iflssel::Half,
        (_, FifoThreshold::ThreeQuarter) => vals::Iflssel::ThreeFourth,
        (_, FifoThreshold::Full) => vals::Iflssel::Full,
    }
}

/// The transmit FIFO level, which has no per-domain restriction.
const fn tx_level(threshold: FifoThreshold) -> vals::Iflssel {
    match threshold {
        FifoThreshold::AtLeastOne => vals::Iflssel::AtLeastOne,
        FifoThreshold::Quarter => vals::Iflssel::OneFourth,
        FifoThreshold::Half => vals::Iflssel::Half,
        FifoThreshold::ThreeQuarter => vals::Iflssel::ThreeFourth,
        FifoThreshold::Full => vals::Iflssel::Full,
    }
}

/// Program a solved divider into the peripheral.
///
/// [`Baud`] itself stays with [`Config`] as the vocabulary a caller configures in — it can be solved
/// at compile time, and `solve`'s arithmetic is the same whatever block runs it. Only this is the
/// register work, and both callers of it are in this file.
pub(crate) fn apply_baud(r: Regs, baud: &Baud) {
    r.clkdiv().write(|w| w.set_ratio(baud.div));
    r.ibrd().write(|w| w.set_divint(baud.ibrd));
    r.fbrd().write(|w| w.set_divfrac(baud.fbrd));
    r.ctl0().modify(|w| w.set_hse(baud.hse));
}

/// Hold the line low, or release it. `BRK` is a level and stays set until it is written back.
pub(crate) fn set_break(r: Regs, on: bool) {
    r.lcrh().modify(|w| w.set_brk(on));
}

/// The sources a receive waits on: the FIFO reaching its level, and the timeout that delivers one
/// that never will.
pub(crate) const fn rx_sources() -> CpuInt {
    let mut sources = CpuInt(0);
    sources.set_rxint(true);
    sources.set_rtout(true);
    sources
}

/// The source a transmit waits on: room in the FIFO.
pub(crate) const fn tx_sources() -> CpuInt {
    let mut sources = CpuInt(0);
    sources.set_txint(true);
    sources
}

/// The source a flush waits on: the last bit leaving the shift register.
pub(crate) const fn eot_sources() -> CpuInt {
    let mut sources = CpuInt(0);
    sources.set_eot(true);
    sources
}

/// Let the instance ask for its clock back when a receive starts in a sleep mode that stopped it.
///
/// Two masks can suppress it: the instance's own `CLKCFG.BLOCKASYNC`, and `SYSOSCCFG.BLOCKASYNCALL`
/// for every peripheral at once. Both reset to "not blocked" and nothing here ever sets them, which is
/// what keeps `UART_ERR_04` — a bit misread when ULPCLK drops from SYSOSC to LFOSC mid-receive with the
/// request disabled — out of reach. Anything gaining the ability to block them has to account for it.
fn arm_async_clock_request(info: &Info) {
    // `Some(false)` means the instance has no mask of its own and is gated only by `BLOCKASYNCALL`.
    // `None` means no SVD is published for the family, so leave the register alone rather than guess
    // at a bit that may not exist.
    if info.sleep.block_async == Some(true) {
        info.regs.gprcm(0).clkcfg().modify(|w| {
            w.set_key(vals::ClkcfgKey::Key);
            w.set_blockasync(false);
        });
    }

    crate::pac::SYSCTL.sysosccfg().modify(|w| w.set_blockasyncall(false));
}

/// Guard keeping the instance's configuration intact, held for the driver's lifetime.
pub(crate) fn retention_guard(info: &'static Info) -> MaybeWakeGuard {
    MaybeWakeGuard::new(info.sleep.floor_to_keep_configured())
}

pub(crate) struct Info {
    pub(crate) regs: Regs,
    pub(crate) interrupt: Interrupt,
    pub(crate) sleep: SleepInfo,
}

pub(crate) struct State {
    /// The clock rate of the UART in Hz.
    pub(crate) clock: AtomicU32,
}

impl State {
    pub const fn new() -> Self {
        Self {
            clock: AtomicU32::new(0),
        }
    }
}

pub(crate) fn enable(regs: Regs) {
    let gprcm = regs.gprcm(0);

    gprcm.rstctl().write(|w| {
        w.set_resetstkyclr(true);
        w.set_resetassert(true);
        w.set_key(vals::ResetKey::Key);
    });

    gprcm.pwren().write(|w| {
        w.set_enable(true);
        w.set_key(vals::PwrenKey::Key);
    });

    // The registers behind `PWREN` stay isolated for a few ULPCLK cycles and a write that lands in
    // that window is dropped. `tim::low_level::enable` carries the account.
    cortex_m::asm::delay(16);
}

#[inline(always)]
pub(crate) fn configure(
    info: &Info,
    state: &State,
    config: &Config,
    enable_rx: bool,
    enable_rts: bool,
    enable_tx: bool,
    enable_cts: bool,
) -> Result<(), ConfigError> {
    let r = info.regs;

    // Read out by value up front. Several of the register writes below are closures, and a closure
    // that borrows `config` puts its address into a callee — which is enough to stop every field
    // read folding, including the one deciding whether the baud search is reachable.
    let &Config {
        clock_source,
        baud,
        data_bits,
        stop_bits,
        parity,
        msb_order,
        loop_back_enable,
        fifo,
        low_power_rx_wake,
        ..
    } = config;

    if !enable_rx && !enable_tx {
        return Err(ConfigError::RxOrTxNotEnabled);
    }

    if low_power_rx_wake {
        if !info.sleep.power_domain.is_powered_in_deep_sleep() {
            return Err(ConfigError::NoDeepSleepWake);
        }

        arm_async_clock_request(info);
    }

    // SLAU846B says that clocks should be enabled before disabling the uart.
    r.clksel().write(|w| match clock_source {
        ClockSel::LfClk => {
            w.set_lfclk_sel(true);
            w.set_mfclk_sel(false);
            w.set_busclk_sel(false);
        }
        ClockSel::MfClk => {
            w.set_mfclk_sel(true);
            w.set_lfclk_sel(false);
            w.set_busclk_sel(false);
        }
        ClockSel::BusClk => {
            w.set_busclk_sel(true);
            w.set_lfclk_sel(false);
            w.set_mfclk_sel(false);
        }
    });

    // Read the tree once rather than per arm, and take the rates from it instead of assuming the
    // reset values: MFCLK in particular reads as absent when the clock configuration left it off.
    let domain = info.sleep.power_domain;
    let clock = crate::sysctl::with_clocks(|clocks| match clock_source {
        ClockSel::LfClk => clocks.lfclk,
        ClockSel::MfClk => clocks.mfclk,
        ClockSel::BusClk => clocks.bus_clock(domain),
    });

    state.clock.store(clock, Ordering::Relaxed);

    info.regs.ctl0().modify(|w| {
        w.set_lbe(loop_back_enable);
        // Errata UART_ERR_02, must set RXE to allow use of EOT.
        w.set_rxe(enable_rx | enable_tx);
        w.set_txe(enable_tx);
        // RXD_OUT_EN and TXD_OUT_EN?
        w.set_menc(false);
        w.set_mode(vals::Mode::Uart);
        w.set_rtsen(enable_rts);
        w.set_ctsen(enable_cts);
        // oversampling is set later
        w.set_fen(fifo.is_some());
        // Majority voting and glitch suppression are both off and neither is configurable yet.
        w.set_majvote(false);
        w.set_msbfirst(matches!(msb_order, BitOrder::MsbFirst));
    });

    // A FIFO is only worth having if the interrupt batches across it. At one entry the handler runs once
    // per byte and its fixed cost is never amortised, which is what bounds the receive rate rather than
    // any buffer size. Half-full halves the entries; the FIFOs are four deep.
    //
    // With the FIFOs off there is one byte of depth and no level to reach, so the choice only applies
    // when they are on.
    let (rx_level, tx_level) = if let Some(threshold) = fifo {
        (rx_level(threshold, info.sleep.power_domain), tx_level(threshold))
    } else {
        (vals::Iflssel::AtLeastOne, vals::Iflssel::AtLeastOne)
    };

    info.regs.ifls().modify(|w| {
        w.set_txiflsel(tx_level);
        w.set_rxiflsel(rx_level);
        // A receive level above one entry needs the timeout armed, or a partial FIFO waits for bytes
        // that never come and the last few of a message are never delivered. Zero, the reset value,
        // disables it entirely and is what makes `RTOUT` unable to fire.
        w.set_rxtosel(if fifo.is_some() { RX_TIMEOUT_BITS } else { 0 });
    });

    info.regs.lcrh().modify(|w| {
        let eps = if matches!(parity, Parity::ParityEven) {
            vals::Eps::Even
        } else {
            vals::Eps::Odd
        };

        let wlen = match data_bits {
            DataBits::DataBits5 => vals::Wlen::Databit5,
            DataBits::DataBits6 => vals::Wlen::Databit6,
            DataBits::DataBits7 => vals::Wlen::Databit7,
            DataBits::DataBits8 => vals::Wlen::Databit8,
        };

        // Used in LIN mode only
        w.set_brk(false);
        w.set_pen(parity != Parity::ParityNone);
        w.set_eps(eps);
        w.set_stp2(matches!(stop_bits, StopBits::Stop2));
        w.set_wlen(wlen);
        // appears to only be used in RS-485 mode.
        w.set_sps(false);
        // IDLE pattern?
        w.set_sendidle(false);
        // ignore extdir_setup and extdir_hold, only used in RS-485 mode.
    });

    // A pre-solved divider skips the search entirely, which is what keeps the software divider out
    // of the binary when the clock and baud rate are both compile-time constants.
    match baud {
        BaudRate::Solved(baud) => apply_baud(info.regs, &baud),
        BaudRate::Rate(rate) => set_baudrate_inner(info.regs, clock, rate)?,
    }

    r.ctl0().modify(|w| {
        w.set_enable(true);
    });

    Ok(())
}

pub(crate) fn reconfigure(info: &Info, state: &State, config: &Config) -> Result<(), ConfigError> {
    let was_enabled = info.interrupt.is_enabled();
    info.interrupt.disable();

    let r = info.regs;
    let ctl0 = r.ctl0().read();

    // SLAU846 24.2.3: a control register is only to be programmed with `ENABLE` clear, and `configure`
    // rewrites `CTL0`, `LCRH`, `IFLS` and `CLKSEL`. Drain first — disabling completes the character in
    // the shift register and abandons whatever is still in the FIFO.
    while busy(r) {}

    critical_section::with(|_cs| r.ctl0().modify(|w| w.set_enable(false)));

    // A rejected config must leave the instance as it was found. `configure` returns before it
    // re-enables, so an early `?` here would hand back an `Err` that reads recoverable over a
    // peripheral that is off with its interrupt masked.
    let configured = configure(info, state, config, ctl0.rxe(), ctl0.rtsen(), ctl0.txe(), ctl0.ctsen());

    if configured.is_err() {
        r.ctl0().write_value(ctl0);
    }

    // Only if it was. Unmasking a line the caller had left masked strands it in whatever the
    // application's `DefaultHandler` does, and `i2c::low_level::set_config` already conditions this.
    if was_enabled {
        info.interrupt.unpend();
        unsafe { info.interrupt.enable() };
    }

    configured
}

/// Set the baud rate and clock settings.
///
/// This should be done relatively late during configuration since some clock settings are invalid depending on mode.
pub(crate) fn set_baudrate(info: &Info, clock: u32, baudrate: u32) -> Result<(), ConfigError> {
    let r = info.regs;

    let was_enabled = info.interrupt.is_enabled();
    info.interrupt.disable();

    // Wait for end of transmission per suggestion in SLAU 845 section 18.3.28. It has to happen while
    // the transmitter still runs: disabling completes only the character already in the shift register
    // (SLAU846 table 24-41), so anything left in the FIFO stays there and a wait after the disable
    // never finishes.
    while busy(r) {}

    // Not for the divisor registers — SLAU846 24.2.3 says `IBRD` and `FBRD` can be written on a live
    // UART. It is for `CTL0.HSE`, the oversampling select, which `apply_baud` also writes and which is
    // a control register like any other.
    critical_section::with(|_cs| {
        r.ctl0().modify(|w| {
            w.set_enable(false);
        });
    });

    // Not `?`. An unreachable baud rate is an ordinary caller error -- above about 10.9 kbaud on an
    // LFCLK-sourced instance -- and returning here would leave the peripheral disabled and its line
    // masked for good.
    let programmed = set_baudrate_inner(r, clock, baudrate);

    critical_section::with(|_cs| {
        r.ctl0().modify(|w| {
            w.set_enable(true);
        });
    });

    if was_enabled {
        info.interrupt.unpend();
        unsafe { info.interrupt.enable() };
    }

    programmed
}

pub(crate) fn set_baudrate_inner(regs: Regs, clock: u32, baudrate: u32) -> Result<(), ConfigError> {
    // Read the source back rather than taking it from the config: this also runs from `set_baudrate`,
    // where the only record of what the instance is clocked from is the register.
    let clksel = regs.clksel().read();
    let source = if clksel.lfclk_sel() {
        ClockSel::LfClk
    } else if clksel.mfclk_sel() {
        ClockSel::MfClk
    } else {
        ClockSel::BusClk
    };

    let Some(baud) = Baud::solve(source, clock, baudrate) else {
        return Err(ConfigError::InvalidBaudRate);
    };

    apply_baud(regs, &baud);

    Ok(())
}

#[inline]
pub(crate) fn read_with_error(r: Regs) -> Result<u8, Error> {
    let rx = read_raw(r);

    match fault_of(rx) {
        Some(error) => Err(error),
        None => Ok(rx.data()),
    }
}

/// Whether the transmitter still holds a byte.
///
/// Answers "the transmit FIFO still has something in it", which is one bit time short of "the wire is
/// idle" — see below. Assumes `CTL0.ENABLE` is set.
pub(crate) fn busy(r: Regs) -> bool {
    // **`STAT.BUSY` is the wrong flag for a transmit drain, and not because of the erratum.** The TRM's
    // field description has it: `BUSY` is set when the transmit FIFO becomes nonempty "or if a receive
    // data is currently ongoing (after the start edge have been detected until a complete byte,
    // including all stop bits, has been received by the shift register)". It covers **both**
    // directions.
    //
    // A `UartTx` can exist with no receive pin at all. The unmuxed RX input reads low, which is a start
    // edge that never completes, so `BUSY` is set from configure onward and a drain polling it never
    // returns. Measured on an L1306 and a G3507: `STAT` `0x41` — `BUSY` set with `TXFE` set — on a
    // transmitter that had not yet sent a byte. Give the same driver a receive pin on an idle-high line
    // and `BUSY` behaves perfectly on both parts. Even then it would be wrong here, because a transmit
    // drain must not block until the *other* end stops sending.
    //
    // `UART_ERR_08` is a second, narrower reason: `BUSY` also sticks with the module disabled and data
    // in the TX FIFO. It applies to every family this crate builds for except G511x/G5187, whose
    // UNICOMM UART is a different module. It is not the reason this substitution exists, and gating the
    // substitution on `CTL0.ENABLE` would not recover anything.
    //
    // **What the substitution costs is one bit time, not one frame.** `TXFE` rises one bit before the
    // transmission completes; measured at 3276 to 3308 cycles against a 33 330-cycle frame at 9600, the
    // same on both parts and flat across one, four and eight bytes. So a caller that deep-sleeps the
    // instant this returns can still cut the final bit. Closing that needs a baud-derived wait, since
    // no register reports it.
    //
    // `STAT.IDLE` is not an alternative; it is a receive-side address tag for idle-line multiprocessor
    // mode.
    !r.stat().read().txfe()
}

// Always false: the driver never sets `DMAEN`, having no receive or transmit DMA path.
pub(crate) fn dma_enabled(_r: Regs) -> bool {
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flags(set: impl Fn(&mut regs::Rxdata)) -> u8 {
        let mut word = regs::Rxdata(0);
        set(&mut word);

        (word.0 >> 8) as u8
    }

    /// Which fault a multiply-faulted byte reports is a decision, so it is pinned here rather than
    /// left to whichever of the two ladders someone reads first — they used to disagree.
    #[test]
    fn a_fault_is_the_lowest_bit_set() {
        let framing = flags(|w| w.set_frmerr(true));
        let parity = flags(|w| w.set_parerr(true));
        let brk = flags(|w| w.set_brkerr(true));
        let overrun = flags(|w| w.set_ovrerr(true));
        let noise = flags(|w| w.set_nerr(true));

        // `fault_of` is written lowest-bit-first by hand, so the bits have to be in that order for
        // the ladder to mean what it says.
        core::assert!(
            framing < parity && parity < brk && brk < overrun && overrun < noise,
            "the fault bits are not in the order fault_of tests them"
        );

        core::assert_eq!(fault_of_flags(0), None);
        core::assert_eq!(fault_of_flags(framing), Some(Error::Framing));
        core::assert_eq!(fault_of_flags(noise), Some(Error::Noise));

        core::assert_eq!(fault_of_flags(framing | noise), Some(Error::Framing));
        core::assert_eq!(fault_of_flags(overrun | noise), Some(Error::Overrun));
        core::assert_eq!(fault_of_flags(parity | brk | overrun), Some(Error::Parity));
    }
}
