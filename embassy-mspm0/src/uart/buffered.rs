use core::future::{Future, poll_fn};
use core::marker::PhantomData;
use core::slice;
use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};
use core::task::Poll;

use embassy_embedded_hal::SetConfig;
use embassy_hal_internal::atomic_ring_buffer::RingBuffer;
use embassy_hal_internal::interrupt::InterruptExt;
use embedded_hal_nb::nb;

use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt::typelevel::Binding;
use crate::pac::uart::Uart as Regs;
use crate::sync::irq_waker::IrqWaker;
use crate::sysctl::{MaybeWakeGuard, SleepLevel};
use crate::uart::{Config, ConfigError, CtsPin, Error, Info, Instance, RtsPin, RxPin, State, TxPin};
use crate::{Peri, interrupt, pac};

/// Interrupt handler.
pub struct BufferedInterruptHandler<T: Instance> {
    _uart: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for BufferedInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt(T::info().regs, T::buffered_state())
    }
}

/// Bidirectional buffered UART which acts as a combination of [`BufferedUartTx`] and [`BufferedUartRx`].
pub struct BufferedUart<'d> {
    rx: BufferedUartRx<'d>,
    tx: BufferedUartTx<'d>,
}

impl SetConfig for BufferedUart<'_> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(config)
    }
}

impl<'d> BufferedUart<'d> {
    /// Create a new bidirectional buffered UART.
    ///
    /// # Where to put the buffers
    ///
    /// Give it buffers with a `'static` home — a `StaticCell`, or a `static mut` — rather than arrays
    /// declared in the calling task. An array declared in an `async fn` lives in that task's frame and
    /// is zeroed there every time the task starts, which links the software `memset`; the same buffers
    /// as statics are zeroed once by the startup code instead. Measured at **188 bytes of flash for two
    /// 32-byte buffers, with no change in RAM** — they were already in the task arena either way.
    pub fn new<T: Instance>(
        uart: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            uart,
            new_pin!(rx, config.rx_pf()),
            new_pin!(tx, config.tx_pf()),
            None,
            None,
            tx_buffer,
            rx_buffer,
            config,
        )
    }

    /// Create a new bidirectional buffered UART with request-to-send and clear-to-send pins
    pub fn new_with_rtscts<T: Instance>(
        uart: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        rts: Peri<'d, impl RtsPin<T>>,
        cts: Peri<'d, impl CtsPin<T>>,
        _irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            uart,
            new_pin!(rx, config.rx_pf()),
            new_pin!(tx, config.tx_pf()),
            new_pin!(rts, config.rts_pf()),
            new_pin!(cts, config.cts_pf()),
            tx_buffer,
            rx_buffer,
            config,
        )
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.tx.set_config(config)?;
        self.rx.set_config(config)
    }

    /// Set baudrate
    pub fn set_baudrate(&mut self, baudrate: u32) -> Result<(), ConfigError> {
        self.rx.set_baudrate(baudrate)
    }

    /// Write to UART TX buffer, blocking execution until done.
    ///
    /// Returns once the bytes are in the ring buffer, not once they have been transmitted. Call
    /// [`Self::blocking_flush`] before anything that can deep sleep.
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<usize, Error> {
        self.tx.blocking_write(buffer)
    }

    /// Flush UART TX buffer, blocking execution until done.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    /// Check if UART is busy.
    pub fn busy(&self) -> bool {
        self.tx.busy()
    }

    /// Read from UART RX buffer, blocking execution until done.
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        self.rx.blocking_read(buffer)
    }

    /// Bytes the receiver knows it dropped since this was last called. See
    /// [`BufferedUartRx::take_dropped`].
    pub fn take_dropped(&self) -> u16 {
        self.rx.take_dropped()
    }

    /// Line faults the receiver has seen since this was last called. See
    /// [`BufferedUartRx::take_faults`].
    pub fn take_faults(&self) -> u16 {
        self.rx.take_faults()
    }

    /// Send break character.
    pub fn send_break(&mut self) {
        self.tx.send_break()
    }

    /// Split into separate RX and TX handles.
    pub fn split(self) -> (BufferedUartTx<'d>, BufferedUartRx<'d>) {
        (self.tx, self.rx)
    }

    /// Split into separate RX and TX handles.
    pub fn split_ref(&mut self) -> (BufferedUartTx<'_>, BufferedUartRx<'_>) {
        (
            BufferedUartTx {
                info: self.tx.info,
                state: self.tx.state,
                tx: self.tx.tx.as_mut().map(Peri::reborrow),
                cts: self.tx.cts.as_mut().map(Peri::reborrow),
                reborrowed: true,
                _retention_guard: MaybeWakeGuard::none(),
            },
            BufferedUartRx {
                info: self.rx.info,
                state: self.rx.state,
                rx: self.rx.rx.as_mut().map(Peri::reborrow),
                rts: self.rx.rts.as_mut().map(Peri::reborrow),
                reborrowed: true,
                wake_guard: MaybeWakeGuard::none(),
                _retention_guard: MaybeWakeGuard::none(),
            },
        )
    }
}

/// Rx-only buffered UART.
///
/// Can be obtained from [`BufferedUart::split`], or can be constructed independently,
/// if you do not need the transmitting half of the driver.
pub struct BufferedUartRx<'d> {
    info: &'static Info,
    state: &'static BufferedState,
    rx: Option<Peri<'d, AnyPin>>,
    rts: Option<Peri<'d, AnyPin>>,
    reborrowed: bool,
    wake_guard: MaybeWakeGuard,
    /// Held for as long as the driver exists; see
    /// [`SleepInfo::floor_to_keep_configured`](crate::sysctl::SleepInfo::floor_to_keep_configured).
    _retention_guard: MaybeWakeGuard,
}

impl SetConfig for BufferedUartRx<'_> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(config)
    }
}

impl<'d> BufferedUartRx<'d> {
    /// Create a new rx-only buffered UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Rx. It saves 1 pin.
    pub fn new<T: Instance>(
        uart: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        rx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(uart, new_pin!(rx, config.rx_pf()), None, rx_buffer, config)
    }

    /// Create a new rx-only buffered UART with a request-to-send pin
    pub fn new_with_rts<T: Instance>(
        uart: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        rts: Peri<'d, impl RtsPin<T>>,
        _irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        rx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            uart,
            new_pin!(rx, config.rx_pf()),
            new_pin!(rts, config.rts_pf()),
            rx_buffer,
            config,
        )
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        if let Some(ref rx) = self.rx {
            rx.update_pf(config.rx_pf());
        }

        if let Some(ref rts) = self.rts {
            rts.update_pf(config.rts_pf());
        }

        super::reconfigure(&self.info, &self.state.state, config)?;

        if !self.reborrowed {
            self.wake_guard = self.rx_wake_guard(config.low_power_rx_wake);
        }
        Ok(())
    }

    /// Set baudrate
    pub fn set_baudrate(&mut self, baudrate: u32) -> Result<(), ConfigError> {
        super::set_baudrate(&self.info, self.state.state.clock.load(Ordering::Relaxed), baudrate)
    }

    /// Floor to hold while armed for receive-wake, or the plain operating floor otherwise.
    ///
    /// Capping at STANDBY0 is required, not a tuning choice. `UART_ERR_01` (every family this driver
    /// builds for) loses a frame that starts while the device is on its way back down to STANDBY1 after
    /// servicing an earlier one, and TI's workaround is "use STANDBY0 mode or higher low power mode when
    /// expecting repeated UART start conditions" — which is exactly this case. STANDBY0 is also the only
    /// depth fast enough to catch the first bits, since the fast clock request needs 241 us typical from
    /// STANDBY1.
    fn rx_wake_guard(&self, low_power_rx_wake: bool) -> MaybeWakeGuard {
        if low_power_rx_wake {
            MaybeWakeGuard::new(Some(SleepLevel::Standby1))
        } else {
            MaybeWakeGuard::new(
                self.info
                    .sleep
                    .floor_for_operation(self.state.state.clock.load(Ordering::Relaxed)),
            )
        }
    }

    /// Read from UART RX buffer, blocking execution until done.
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        self.blocking_read_inner(buffer)
    }
}

impl Drop for BufferedUartRx<'_> {
    fn drop(&mut self) {
        if !self.reborrowed {
            let state = self.state;

            // SAFETY: RX is being dropped (and is not reborrowed), so the ring buffer must be deinitialized
            // in order to meet the requirements of init.
            unsafe {
                state.rx_buf.deinit();
            }

            // TX is inactive if the buffer is not available. If this is true, then disable the
            // interrupt handler since we are running in RX only mode.
            if state.tx_buf.len() == 0 {
                self.info.interrupt.disable();
            } else {
                // Same as the transmit half above, and the receive sources are the worse of the two to
                // leave behind: nothing drains the FIFO once the buffer is gone, so a level that is
                // already met keeps the line asserted rather than raising one stray interrupt.
                self.info.regs.cpu_int(0).imask().modify(|w| {
                    w.set_rxint(false);
                    w.set_rtout(false);
                });
                self.info.regs.cpu_int(0).iclr().write(|w| w.set_rtout(true));
            }

            self.rx.as_ref().map(|x| x.set_as_disconnected());
            self.rts.as_ref().map(|x| x.set_as_disconnected());
        }
    }
}

/// Tx-only buffered UART.
///
/// Can be obtained from [`BufferedUart::split`], or can be constructed independently,
/// if you do not need the receiving half of the driver.
pub struct BufferedUartTx<'d> {
    info: &'static Info,
    state: &'static BufferedState,
    tx: Option<Peri<'d, AnyPin>>,
    cts: Option<Peri<'d, AnyPin>>,
    reborrowed: bool,
    /// Held for as long as the driver exists; see
    /// [`SleepInfo::floor_to_keep_configured`](crate::sysctl::SleepInfo::floor_to_keep_configured).
    _retention_guard: MaybeWakeGuard,
}

impl SetConfig for BufferedUartTx<'_> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(config)
    }
}

impl<'d> BufferedUartTx<'d> {
    /// Create a new tx-only buffered UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Tx. It saves 1 pin.
    pub fn new<T: Instance>(
        uart: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        _irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        tx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(uart, new_pin!(tx, config.tx_pf()), None, tx_buffer, config)
    }

    /// Create a new tx-only buffered UART with a clear-to-send pin
    pub fn new_with_rts<T: Instance>(
        uart: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        cts: Peri<'d, impl CtsPin<T>>,
        _irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
        tx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            uart,
            new_pin!(tx, config.tx_pf()),
            new_pin!(cts, config.cts_pf()),
            tx_buffer,
            config,
        )
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        if let Some(ref tx) = self.tx {
            tx.update_pf(config.tx_pf());
        }

        if let Some(ref cts) = self.cts {
            cts.update_pf(config.cts_pf());
        }

        super::reconfigure(self.info, &self.state.state, config)
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        super::set_baudrate(&self.info, self.state.state.clock.load(Ordering::Relaxed), baudrate)
    }

    /// Write to UART TX buffer, blocking execution until done.
    ///
    /// Returns once the bytes are in the ring buffer, not once they have been transmitted. Call
    /// [`Self::blocking_flush`] before anything that can deep sleep.
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<usize, Error> {
        self.blocking_write_inner(buffer)
    }

    /// Flush UART TX buffer, blocking execution until done.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        let state = self.state;

        // An empty ring only means the interrupt handed everything to the hardware. The FIFO and shift
        // register still have to drain, and deep sleep entered before they do cuts the frame mid-byte.
        while !state.tx_buf.is_empty() {}
        while super::busy(self.info.regs) {}

        Ok(())
    }

    /// Check if UART is busy.
    pub fn busy(&self) -> bool {
        super::busy(self.info.regs)
    }

    /// Send break character
    pub fn send_break(&mut self) {
        let r = self.info.regs;

        r.lcrh().modify(|w| {
            w.set_brk(true);
        });
    }
}

impl Drop for BufferedUartTx<'_> {
    fn drop(&mut self) {
        if !self.reborrowed {
            let state = self.state;

            // SAFETY: TX is being dropped (and is not reborrowed), so the ring buffer must be deinitialized
            // in order to meet the requirements of init.
            unsafe {
                state.tx_buf.deinit();
            }

            // RX is inactive if the buffer is not available. If this is true, then disable the
            // interrupt handler since we are running in TX only mode.
            if state.rx_buf.len() == 0 {
                self.info.interrupt.disable();
            } else {
                // The receiver keeps the line alive, so the transmit half's own source has to be turned
                // off with it. A completion left armed here raises an interrupt that finds a
                // deinitialised buffer and does nothing but cost a wake — and left pending, it fires
                // again the moment a new transmitter arms it.
                self.info.regs.cpu_int(0).imask().modify(|w| w.set_eot(false));
                self.info.regs.cpu_int(0).iclr().write(|w| w.set_eot(true));
            }

            self.tx.as_ref().map(|x| x.set_as_disconnected());
            self.cts.as_ref().map(|x| x.set_as_disconnected());
        }
    }
}

impl embedded_io_async::ErrorType for BufferedUart<'_> {
    type Error = Error;
}

impl embedded_io_async::ErrorType for BufferedUartRx<'_> {
    type Error = Error;
}

impl embedded_io_async::ErrorType for BufferedUartTx<'_> {
    type Error = Error;
}

impl embedded_io_async::Read for BufferedUart<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf).await
    }
}

impl embedded_io_async::Read for BufferedUartRx<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_inner(buf).await
    }
}

impl embedded_io_async::ReadReady for BufferedUart<'_> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.rx.read_ready()
    }
}

impl embedded_io_async::ReadReady for BufferedUartRx<'_> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.read_ready_inner()
    }
}

impl embedded_io_async::BufRead for BufferedUart<'_> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        self.rx.fill_buf().await
    }

    fn consume(&mut self, amt: usize) {
        self.rx.consume(amt);
    }
}

impl embedded_io_async::BufRead for BufferedUartRx<'_> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        self.fill_buf_inner().await
    }

    fn consume(&mut self, amt: usize) {
        self.consume_inner(amt);
    }
}

impl embedded_io_async::Write for BufferedUart<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write_inner(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush_inner().await
    }
}

impl embedded_io_async::Write for BufferedUartTx<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_inner(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_inner().await
    }
}

impl embedded_io::Read for BufferedUart<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf)
    }
}

impl embedded_io::Read for BufferedUartRx<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.blocking_read_inner(buf)
    }
}

impl embedded_io::Write for BufferedUart<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl embedded_io::Write for BufferedUartTx<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.blocking_write_inner(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match self {
            Error::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            Error::Noise => embedded_hal_nb::serial::ErrorKind::Noise,
            Error::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
            Error::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
            Error::Break => embedded_hal_nb::serial::ErrorKind::Other,
        }
    }
}

impl embedded_hal_nb::serial::ErrorType for BufferedUart<'_> {
    type Error = Error;
}

impl embedded_hal_nb::serial::ErrorType for BufferedUartRx<'_> {
    type Error = Error;
}

impl embedded_hal_nb::serial::ErrorType for BufferedUartTx<'_> {
    type Error = Error;
}

impl embedded_hal_nb::serial::Read for BufferedUart<'_> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}

impl embedded_hal_nb::serial::Read for BufferedUartRx<'_> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if self.info.regs.stat().read().rxfe() {
            return Err(nb::Error::WouldBlock);
        }

        super::read_with_error(self.info.regs).map_err(nb::Error::Other)
    }
}

impl embedded_hal_nb::serial::Write for BufferedUart<'_> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl embedded_hal_nb::serial::Write for BufferedUartTx<'_> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.blocking_write(&[word]).map(drop).map_err(nb::Error::Other)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.blocking_flush().map_err(nb::Error::Other)
    }
}

// Impl details

/// Buffered UART state.
pub(crate) struct BufferedState {
    /// non-buffered UART state. This is inline in order to avoid [`BufferedUartRx`]/Tx
    /// needing to carry around a 2nd static reference and waste another 4 bytes.
    state: State,
    rx_waker: IrqWaker,
    rx_buf: RingBuffer,
    tx_waker: IrqWaker,
    tx_buf: RingBuffer,
    rx_error: AtomicU8,
    /// Bytes the receiver is known to have dropped since the last report, saturating.
    ///
    /// `rx_error` is a set of flags, so on its own it cannot say whether one byte was lost or a hundred
    /// thousand — which is exactly how a sustained overrun once read as a handful of them.
    rx_dropped: AtomicU16,
    /// Line faults the receiver has seen since the last report, saturating.
    ///
    /// Noise, framing, parity and break, counted together. Overruns are `rx_dropped`, which says how
    /// many bytes went with them; these four cost the byte they arrived on and nothing more, so a count
    /// is the whole story.
    rx_faults: AtomicU16,
}

// these must match bits 8..12 in RXDATA, but shifted by 8 to the right
const RXE_NOISE: u8 = 16;
const RXE_OVERRUN: u8 = 8;
const RXE_BREAK: u8 = 4;
const RXE_PARITY: u8 = 2;
const RXE_FRAMING: u8 = 1;

impl BufferedState {
    pub const fn new() -> Self {
        Self {
            state: State::new(),
            rx_waker: IrqWaker::new(),
            rx_buf: RingBuffer::new(),
            tx_waker: IrqWaker::new(),
            tx_buf: RingBuffer::new(),
            rx_error: AtomicU8::new(0),
            rx_dropped: AtomicU16::new(0),
            rx_faults: AtomicU16::new(0),
        }
    }
}

impl<'d> BufferedUart<'d> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        tx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        let info = T::info();
        let state = T::buffered_state();

        let mut this = Self {
            tx: BufferedUartTx {
                info,
                state,
                tx,
                cts,
                reborrowed: false,
                _retention_guard: super::retention_guard(info),
            },
            rx: BufferedUartRx {
                info,
                state,
                rx,
                rts,
                reborrowed: false,
                wake_guard: MaybeWakeGuard::none(),
                _retention_guard: super::retention_guard(info),
            },
        };
        this.enable_and_configure(tx_buffer, rx_buffer, &config)?;
        this.rx.wake_guard = this.rx.rx_wake_guard(config.low_power_rx_wake);

        Ok(this)
    }

    fn enable_and_configure(
        &mut self,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        config: &Config,
    ) -> Result<(), ConfigError> {
        let info = self.rx.info;
        let state = self.rx.state;

        assert!(!tx_buffer.is_empty());
        assert!(!rx_buffer.is_empty());

        init_buffers(info, state, Some(tx_buffer), Some(rx_buffer));
        super::enable(info.regs);
        super::configure(
            info,
            &state.state,
            config,
            true,
            self.rx.rts.is_some(),
            true,
            self.tx.cts.is_some(),
        )?;

        info.regs.cpu_int(0).imask().modify(|w| {
            w.set_rxint(true);
            // Unmasked here rather than only after the first read: with a receive level above one entry,
            // a first message shorter than that level is delivered by the timeout alone.
            w.set_rtout(true);
            arm_errors(w);
        });

        info.interrupt.unpend();
        unsafe { info.interrupt.enable() };

        Ok(())
    }
}

impl<'d> BufferedUartRx<'d> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        rx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        let mut this = Self {
            info: T::info(),
            state: T::buffered_state(),
            rx,
            rts,
            reborrowed: false,
            wake_guard: MaybeWakeGuard::none(),
            _retention_guard: super::retention_guard(T::info()),
        };
        this.enable_and_configure(rx_buffer, &config)?;
        this.wake_guard = this.rx_wake_guard(config.low_power_rx_wake);

        Ok(this)
    }

    fn enable_and_configure(&mut self, rx_buffer: &'d mut [u8], config: &Config) -> Result<(), ConfigError> {
        let info = self.info;
        let state = self.state;

        init_buffers(info, state, None, Some(rx_buffer));
        super::enable(info.regs);
        super::configure(info, &self.state.state, config, true, self.rts.is_some(), false, false)?;

        info.regs.cpu_int(0).imask().modify(|w| {
            w.set_rxint(true);
            w.set_rtout(true);
            arm_errors(w);
        });

        info.interrupt.unpend();
        unsafe { info.interrupt.enable() };

        Ok(())
    }

    async fn read_inner(&self, buf: &mut [u8]) -> Result<usize, Error> {
        poll_fn(move |cx| {
            let state = self.state;

            if let Poll::Ready(r) = self.try_read(buf) {
                return Poll::Ready(r);
            }

            state.rx_waker.register(cx.waker());
            Poll::Pending
        })
        .await
    }

    fn blocking_read_inner(&self, buffer: &mut [u8]) -> Result<usize, Error> {
        loop {
            match self.try_read(buffer) {
                Poll::Ready(res) => return res,
                Poll::Pending => continue,
            }
        }
    }

    fn fill_buf_inner(&self) -> impl Future<Output = Result<&'_ [u8], Error>> {
        poll_fn(move |cx| {
            let mut rx_reader = unsafe { self.state.rx_buf.reader() };
            let (p, n) = rx_reader.pop_buf();
            let result = if n == 0 {
                match Self::get_rx_error(self.state) {
                    None => {
                        self.state.rx_waker.register(cx.waker());
                        return Poll::Pending;
                    }
                    Some(e) => Err(e),
                }
            } else {
                let buf = unsafe { slice::from_raw_parts(p, n) };
                Ok(buf)
            };

            Poll::Ready(result)
        })
    }

    fn consume_inner(&self, amt: usize) {
        let mut rx_reader = unsafe { self.state.rx_buf.reader() };
        rx_reader.pop_done(amt);

        // (Re-)Enable the interrupt to receive more data in case it was
        // disabled because the buffer was full or errors were detected.
        self.info.regs.cpu_int(0).imask().modify(|w| {
            w.set_rxint(true);
            w.set_rtout(true);
        });
    }

    /// we are ready to read if there is data in the buffer
    fn read_ready_inner(&self) -> Result<bool, Error> {
        Ok(!self.state.rx_buf.is_empty())
    }

    fn try_read(&self, buf: &mut [u8]) -> Poll<Result<usize, Error>> {
        // A pulse rather than a bracket: this has several exits, and the question is only whether it runs
        // at all.
        #[cfg(feature = "_probe")]
        crate::probe::count(crate::probe::target(crate::probe::Marker::UartReadPoll));

        let state = self.state;

        if buf.is_empty() {
            return Poll::Ready(Ok(0));
        }

        let mut rx_reader = unsafe { state.rx_buf.reader() };
        let n = rx_reader.pop(|data| {
            let n = data.len().min(buf.len());
            buf[..n].copy_from_slice(&data[..n]);
            n
        });

        let result = if n == 0 {
            match Self::get_rx_error(state) {
                None => return Poll::Pending,
                Some(e) => Err(e),
            }
        } else {
            Ok(n)
        };

        // (Re-)Enable the interrupt to receive more data in case it was
        // disabled because the buffer was full or errors were detected.
        self.info.regs.cpu_int(0).imask().modify(|w| {
            w.set_rxint(true);
            w.set_rtout(true);
        });

        Poll::Ready(result)
    }

    /// Bytes the receiver knows it dropped since this was last called, clearing the count.
    ///
    /// [`Error::Overrun`] says only that it happened, and reaches a caller solely when a read finds the
    /// buffer empty — which a receiver losing bytes because it cannot keep up never does. This is the
    /// figure, without that condition attached, and it is cleared independently of the error. Saturates
    /// rather than wrapping, so a large value means "at least this many".
    pub fn take_dropped(&self) -> u16 {
        critical_section::with(|_cs| {
            let dropped = self.state.rx_dropped.load(Ordering::Relaxed);
            self.state.rx_dropped.store(0, Ordering::Relaxed);

            dropped
        })
    }

    /// Line faults the receiver has seen since this was last called, clearing the count.
    ///
    /// Noise, framing, parity and break together — the faults that cost the byte they arrived on and
    /// nothing further. Bytes lost to an overrun are [`Self::take_dropped`], which counts bytes rather
    /// than events. Saturates rather than wrapping, so a large value means "at least this many".
    pub fn take_faults(&self) -> u16 {
        critical_section::with(|_cs| {
            let faults = self.state.rx_faults.load(Ordering::Relaxed);
            self.state.rx_faults.store(0, Ordering::Relaxed);

            faults
        })
    }

    fn get_rx_error(state: &BufferedState) -> Option<Error> {
        // Cortex-M0 has does not support atomic swap, so we must do two operations.
        // `rx_dropped` is deliberately left alone: it is read by `take_dropped` and reporting an error
        // here must not consume a count the caller has not seen.
        let errs = critical_section::with(|_cs| {
            let errs = state.rx_error.load(Ordering::Relaxed);
            state.rx_error.store(0, Ordering::Relaxed);

            errs
        });

        if errs & RXE_NOISE != 0 {
            Some(Error::Noise)
        } else if errs & RXE_OVERRUN != 0 {
            Some(Error::Overrun)
        } else if errs & RXE_BREAK != 0 {
            Some(Error::Break)
        } else if errs & RXE_PARITY != 0 {
            Some(Error::Parity)
        } else if errs & RXE_FRAMING != 0 {
            Some(Error::Framing)
        } else {
            None
        }
    }
}

impl<'d> BufferedUartTx<'d> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        tx: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        tx_buffer: &'d mut [u8],
        config: Config,
    ) -> Result<Self, ConfigError> {
        let mut this = Self {
            info: T::info(),
            state: T::buffered_state(),
            tx,
            cts,
            reborrowed: false,
            _retention_guard: super::retention_guard(T::info()),
        };

        this.enable_and_configure(tx_buffer, &config)?;

        Ok(this)
    }

    async fn write_inner(&self, buf: &[u8]) -> Result<usize, Error> {
        // Whether this call has already let the rest of the task run. Local to the call, so a transfer
        // that starts on an idle transmitter is not delayed by it.
        let mut yielded = false;

        poll_fn(move |cx| {
            #[cfg(feature = "_probe")]
            crate::probe::count(crate::probe::target(crate::probe::Marker::UartWritePoll));

            let state = self.state;

            if buf.is_empty() {
                return Poll::Ready(Ok(0));
            }

            // Sampled before the push, so it says whether the handler already has work queued.
            let was_empty = state.tx_buf.is_empty();

            // A write only pends when the buffer is full, and under load it may never be: the interrupt
            // can starve this task enough that the buffer has always drained by the time it runs again.
            // A `write_all` loop then never yields, and anything joined or selected with it is never
            // polled — a receiver sharing the task goes deaf for the whole transfer.
            //
            // So yield once per call, unconditionally. Both cheaper-looking conditions were measured and
            // both are wrong: at the instant the starved task runs, the buffer is empty *and* the
            // hardware reads idle, because the wire has run dry waiting for it. There is no state that
            // is true when this is needed. The cost is one extra poll per call, taken immediately —
            // against a byte time of 21 µs at 460800, it does not show.
            if !yielded {
                yielded = true;
                cx.waker().wake_by_ref();
                return Poll::Pending;
            }

            let mut tx_writer = unsafe { state.tx_buf.writer() };
            let n = tx_writer.push(|data| {
                let n = data.len().min(buf.len());
                data[..n].copy_from_slice(&buf[..n]);
                n
            });

            if n == 0 {
                state.tx_waker.register(cx.waker());
                return Poll::Pending;
            }

            // The TX interrupt only triggers when there was data in the FIFO and the number of bytes
            // drops below a threshold. When the buffer was empty there may be no interrupt on its way,
            // so pend one by hand to shovel this into the FIFO; when it was not, the handler that is
            // already coming will pick these up too. Pending unconditionally forces an interrupt per
            // write, which is a large share of them under a saturated transmitter, and starves thread
            // mode enough that this never has to return `Pending` at all — see `blocking_write_inner`,
            // which has always had this condition.
            if was_empty {
                self.info.interrupt.pend();
            }

            Poll::Ready(Ok(n))
        })
        .await
    }

    fn blocking_write_inner(&self, buffer: &[u8]) -> Result<usize, Error> {
        let state = self.state;

        loop {
            let empty = state.tx_buf.is_empty();

            // SAFETY: tx buf must be initialized if BufferedUartTx exists.
            let mut tx_writer = unsafe { state.tx_buf.writer() };
            let data = tx_writer.push_slice();

            if !data.is_empty() {
                let n = data.len().min(buffer.len());
                data[..n].copy_from_slice(&buffer[..n]);
                tx_writer.push_done(n);

                if empty {
                    self.info.interrupt.pend();
                }

                return Ok(n);
            }
        }
    }

    async fn flush_inner(&self) -> Result<(), Error> {
        let _guard = MaybeWakeGuard::new(
            self.info
                .sleep
                .floor_for_operation(self.state.state.clock.load(Ordering::Relaxed)),
        );

        poll_fn(move |cx| {
            let state = self.state;

            // The ring empties as soon as the interrupt moves the last byte into the hardware FIFO, so
            // the hardware has to be checked too. The end-of-transmission interrupt re-polls this once
            // it drains, which is why waiting here does not need to spin.
            if !state.tx_buf.is_empty() || super::busy(self.info.regs) {
                state.tx_waker.register(cx.waker());
                return Poll::Pending;
            }

            Poll::Ready(Ok(()))
        })
        .await
    }

    fn enable_and_configure(&mut self, tx_buffer: &'d mut [u8], config: &Config) -> Result<(), ConfigError> {
        let info = self.info;
        let state = self.state;

        init_buffers(info, state, Some(tx_buffer), None);
        super::enable(info.regs);
        super::configure(info, &state.state, config, false, false, true, self.cts.is_some())?;

        info.regs.cpu_int(0).imask().modify(|w| {
            w.set_rxint(true);
        });

        info.interrupt.unpend();
        unsafe { info.interrupt.enable() };

        Ok(())
    }
}

fn init_buffers<'d>(
    _info: &Info,
    state: &BufferedState,
    tx_buffer: Option<&'d mut [u8]>,
    rx_buffer: Option<&'d mut [u8]>,
) {
    if let Some(tx_buffer) = tx_buffer {
        let len = tx_buffer.len();
        unsafe { state.tx_buf.init(tx_buffer.as_mut_ptr(), len) };
    }

    if let Some(rx_buffer) = rx_buffer {
        let len = rx_buffer.len();
        unsafe { state.rx_buf.init(rx_buffer.as_mut_ptr(), len) };
    }
}

fn on_interrupt(r: Regs, state: &'static BufferedState) {
    // Opened before anything is read, so the bracket counts an entry that finds nothing to do the same
    // as one that moves a byte. Which of those is happening is the question the marker exists for.
    #[cfg(feature = "_probe")]
    let (handler_marker, mask_marker) = {
        use crate::probe::{Marker, target};

        let markers = (target(Marker::UartHandler), target(Marker::UartRxMask));
        crate::probe::set(markers.0);
        markers
    };

    let int = r.cpu_int(0).mis().read();

    // Per https://github.com/embassy-rs/embassy/pull/1458, both buffered and unbuffered handlers may be bound.
    if super::dma_enabled(r) {
        #[cfg(feature = "_probe")]
        crate::probe::clear(handler_marker);

        return;
    }

    // RX
    if state.rx_buf.is_available() {
        // SAFETY: RX must have been initialized if RXE is set.
        let mut rx_writer = unsafe { state.rx_buf.writer() };
        let rx_buf = rx_writer.push_slice();
        let mut n_read = 0;
        // Accumulated here and published once below. Both stores need a critical section on M0, and
        // taking one per faulty byte would cost the most exactly when the receiver can least afford it.
        let mut errs = 0u8;
        let mut dropped = 0u16;

        while n_read < rx_buf.len() {
            let stat = r.stat().read();

            if stat.rxfe() {
                break;
            }

            let data = r.rxdata().read();
            let flags = (data.0 >> 8) as u8;

            if flags != 0 {
                errs |= flags;

                // An overrun says bytes were lost before this one, so one counted per faulty byte is a
                // lower bound rather than the true figure.
                if flags & RXE_OVERRUN != 0 {
                    dropped = dropped.saturating_add(1);
                }

                // Only fill the buffer with valid characters. The current character is fine if the error
                // is an overrun, but adding it would report the overrun one character too late; drop it
                // and pretend we were a little slower at draining than we were, which is what the
                // blocking path reports too.
                //
                // Keep draining, though. The rest of the FIFO is good, and abandoning it leaves the
                // receiver further behind than it already is, which is how one overrun becomes a run.
                continue;
            }

            rx_buf[n_read] = data.data();
            n_read += 1;
        }

        if errs != 0 {
            // Cortex-M0 does not support atomic fetch_or, must do 2 operations.
            critical_section::with(|_cs| {
                state
                    .rx_error
                    .store(state.rx_error.load(Ordering::Relaxed) | errs, Ordering::Relaxed);

                if dropped != 0 {
                    let total = state.rx_dropped.load(Ordering::Relaxed).saturating_add(dropped);
                    state.rx_dropped.store(total, Ordering::Relaxed);
                }
            });
        }

        if n_read > 0 {
            rx_writer.push_done(n_read);
            state.rx_waker.wake();
        } else if errs != 0 {
            state.rx_waker.wake();
        }

        // Disable any further RX interrupts when the buffer becomes full, which is real backpressure:
        // there is nowhere to put what arrives until the caller reads.
        //
        // An error is not that. Masking on one stalls the receiver until the caller next reads, which
        // under a sustained overrun feeds itself: 16,000 maskings a second at 1 Mbaud, and the delivered
        // rate *falling* as more was offered — 64,000 B/s against 76,000 with an error left alone.
        if state.rx_buf.is_full() {
            #[cfg(feature = "_probe")]
            crate::probe::set(mask_marker);

            r.cpu_int(0).imask().modify(|w| {
                w.set_rxint(false);
                w.set_rtout(false);
            });

            #[cfg(feature = "_probe")]
            crate::probe::clear(mask_marker);
        }
    }

    if int.eot() {
        r.cpu_int(0).imask().modify(|w| {
            w.set_eot(false);
        });

        r.cpu_int(0).iclr().write(|w| {
            w.set_eot(true);
        });

        #[cfg(feature = "_probe")]
        crate::probe::count(crate::probe::target(crate::probe::Marker::UartTxWake));

        state.tx_waker.wake();
    }

    // TX
    //
    // `is_empty` before `reader`, because `pop_slice` is an out-of-line call that computes a contiguous
    // span and then finds nothing in it. A receive-only application would pay for that on every entry.
    if state.tx_buf.is_available() && !state.tx_buf.is_empty() {
        // SAFETY: TX must have been initialized if TXE is set.
        let mut tx_reader = unsafe { state.tx_buf.reader() };
        let buf = tx_reader.pop_slice();
        let mut n_written = 0;

        for tx_byte in buf.iter_mut() {
            let stat = r.stat().read();

            if stat.txff() {
                break;
            }

            r.txdata().write(|w| {
                w.set_data(*tx_byte);
            });
            n_written += 1;
        }

        if n_written > 0 {
            // EOT will wake.
            r.cpu_int(0).imask().modify(|w| {
                w.set_eot(true);
            });

            tx_reader.pop_done(n_written);
        }
    }

    // Clear TX and error interrupt flags
    // RX interrupt flags are cleared by writing to ICLR.
    let mis = r.cpu_int(0).mis().read();
    r.cpu_int(0).iclr().write(|w| {
        // The receive timeout is unmasked on every read and was never cleared here, so once it had
        // fired the flag stayed set and each unmask re-asserted the interrupt: a second entry per read
        // that finds the FIFO already drained and does nothing. Measured on isolated bytes, that was
        // 2.00 handler entries per byte at 26.7 us against 1.01 at 16.2 us.
        //
        // Safe to clear unconditionally because `mis` is the *masked* status: when the ring filled and
        // the timeout was masked above, this reads false and the pending delivery survives.
        w.set_rtout(mis.rtout());
        w.set_nerr(mis.nerr());
        w.set_frmerr(mis.frmerr());
        w.set_parerr(mis.parerr());
        w.set_brkerr(mis.brkerr());
        w.set_ovrerr(mis.ovrerr());
    });

    // Errors. Gated on the lot of them together: five separate bit tests run on every entry that has no
    // error to report, which is every entry on a healthy line.
    if mis.0 & ERROR_INTERRUPTS != 0 {
        count_errors(state, mis);
    }

    #[cfg(feature = "_probe")]
    crate::probe::clear(handler_marker);
}

/// Unmask the error interrupts.
///
/// Called where the receive interrupts are armed rather than where the buffers are set up: the
/// peripheral is reset between the two, and an unmask before it does not survive.
fn arm_errors(w: &mut pac::uart::regs::CpuInt) {
    w.set_nerr(true);
    w.set_frmerr(true);
    w.set_parerr(true);
    w.set_brkerr(true);
    w.set_ovrerr(true);
}

/// The error bits of `CPU_INT`, built from the setters so it cannot drift from the register.
const ERROR_INTERRUPTS: u32 = {
    let mut w = pac::uart::regs::CpuInt(0);
    w.set_nerr(true);
    w.set_frmerr(true);
    w.set_parerr(true);
    w.set_brkerr(true);
    w.set_ovrerr(true);
    w.0
};

/// Add this entry's line faults to the running count.
///
/// Counted rather than logged. These arrive at the line's fault rate, which on a noisy link at a high
/// baud rate has been measured in the tens of thousands per second — enough that logging one apiece
/// costs more throughput than the faults themselves, which is how a sustained overrun once hid.
///
/// The overrun bit is deliberately not counted here: `rx_dropped` already carries it, in bytes rather
/// than in events.
#[cold]
fn count_errors(state: &BufferedState, mis: pac::uart::regs::CpuInt) {
    let faults = u16::from(mis.nerr()) + u16::from(mis.frmerr()) + u16::from(mis.parerr()) + u16::from(mis.brkerr());

    if faults != 0 {
        let seen = state.rx_faults.load(Ordering::Relaxed);
        state.rx_faults.store(seen.saturating_add(faults), Ordering::Relaxed);
    }
}
