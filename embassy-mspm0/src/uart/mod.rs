//! Universal Asynchronous Receiver/Transmitter (UART) driver.
//!
//! # Deep sleep truncates an unflushed write
//!
//! A write that returns once the bytes are queued has not put them on the wire. Deep sleep entered
//! before the transmitter drains cuts the frame mid-byte, and on a PD1 instance the TX pin then sits
//! low until the next wake.
//!
//! Nothing reports this: the bytes were accepted, and the receiver on the other end sees a framing
//! error rather than a byte the sender can act on.
//!
//! Two paths close it for you. [`UartTx::begin_blocking_write`] hands out a [`TxWrite`] that waits when
//! dropped, so a caller who does nothing gets the safe behaviour. The asynchronous
//! [`write`](UartTx::write) waits before it resolves, which makes a following
//! [`flush`](UartTx::flush) redundant. The buffered writes do not, and still want a flush before
//! anything that can sleep.
//!
//! # The last bit is yours to wait out
//!
//! Every drain here waits for the transmit FIFO to empty, and that is **one bit time short of the wire
//! going idle**. So even a fully flushed transmitter can lose its final bit to a deep sleep entered
//! immediately afterwards.
//!
//! No register reports the difference. `STAT.BUSY` looks like the flag for it and is not: it is also
//! set while a byte is being *received*, so a transmitter with no receive pin leaves it set forever and
//! a drain polling it never returns. See `busy` in this module's source for the measurements.
//!
//! The gap was measured at one bit time on both an L-series and a G-series part, flat across transfer
//! lengths. That is about 104 us at 9600 baud and 9 us at 115200.
//!
//! **Closing it is the application's job**, because the cost of the wait depends on things this driver
//! does not own: the CPU clock a spin has to be sized against, and whether you would rather spin at all
//! than arm a timer. Wait one to two bit times at your configured baud after the flush, before you
//! sleep. Size a [`cortex_m::asm::delay`] generously — it counts loop iterations rather than cycles,
//! about five cycles each on this silicon, so a count derived as if it were cycles errs long.
//!
//! Most callers can ignore all of this. It matters when the receiving end cannot detect a truncated
//! frame, which is to say when the protocol carries no length and no checksum.
//!
//! # Pin order
//!
//! Every full-duplex constructor here takes its pins as `(tx, rx)` — [`Uart`] and [`BufferedUart`]
//! alike. [`Uart`] used to take them the other way round, which meant the two drivers in this module
//! disagreed and porting between them was a needless edit.
#![macro_use]

mod buffered;
pub mod low_level;

use core::future::{Future, poll_fn};
use core::marker::PhantomData;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

pub use buffered::*;
use embassy_embedded_hal::SetConfig;
use embassy_hal_internal::PeripheralType;
// The register work is [`low_level`]'s, and the two drivers here reach it directly rather than
// through its types: they hold the halves, so `self.inner.info` is the same `&'static` either way and
// a wrapper method would only be a second name for it.
pub(crate) use low_level::{
    Info, State, busy, clear, eot_sources, mask, masked_status, read_with_error, reconfigure, rx_empty, rx_sources,
    tx_full, tx_sources, unmask, write_byte,
};

use crate::Peri;
use crate::gpio::{AnyPin, PfType, Pull};
use crate::interrupt::InterruptExt;
use crate::interrupt::typelevel::{Binding, Interrupt as _};
use crate::mode::{Async, Blocking, Mode};
use crate::pac::uart::{Uart as Regs, vals};
use crate::sync::irq_waker::IrqWaker;
use crate::sysctl::{MaybeWakeGuard, PowerDomain, SleepLevel};

/// The clock source for the UART.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSel {
    /// Use the low frequency clock.
    ///
    /// The LFCLK runs at 32.768 kHz.
    LfClk,

    /// Use the middle frequency clock.
    ///
    /// MFCLK runs at 4 MHz.
    MfClk,

    /// Use the bus clock.
    ///
    /// Which clock that is depends on the power domain the instance is in: MCLK for PD1, ULPCLK for
    /// PD0. The two differ on G-series, where the ULPCLK ceiling is half MCLK's.
    BusClk,
}

impl ClockSel {
    /// Frequency of this source, in Hz, for an instance in `domain`.
    ///
    /// A `const fn`, so a [`Baud`] for a `BusClk`-sourced UART can be solved at compile time. Take
    /// `domain` from the instance rather than assuming it:
    ///
    /// ```ignore
    /// use embassy_mspm0::sysctl::{self, LowPowerInstance, clock};
    /// use embassy_mspm0::{peripherals, uart::ClockSel};
    ///
    /// const CLOCKS: clock::Clocks = clock::RESET_SETUP.clocks();
    /// const DOMAIN: sysctl::PowerDomain = <peripherals::UART1 as LowPowerInstance>::SLEEP.power_domain;
    /// const RATE: u32 = ClockSel::BusClk.frequency(&CLOCKS, DOMAIN);
    /// ```
    pub const fn frequency(self, clocks: &crate::sysctl::Clocks, domain: crate::sysctl::PowerDomain) -> u32 {
        match self {
            ClockSel::LfClk => clocks.lfclk,
            ClockSel::MfClk => clocks.mfclk,
            ClockSel::BusClk => clocks.bus_clock(domain),
        }
    }
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// The order of bits in byte.
pub enum BitOrder {
    /// The most significant bit is first.
    MsbFirst,

    /// The least significant bit is first.
    LsbFirst,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Number of data bits
pub enum DataBits {
    /// 5 Data Bits
    DataBits5,

    /// 6 Data Bits
    DataBits6,

    /// 7 Data Bits
    DataBits7,

    /// 8 Data Bits
    DataBits8,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Parity
pub enum Parity {
    /// No parity
    ParityNone,

    /// Even Parity
    ParityEven,

    /// Odd Parity
    ParityOdd,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Number of stop bits
pub enum StopBits {
    /// One stop bit
    Stop1,

    /// Two stop bits
    Stop2,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Why a [`Uart`] could not be built or reconfigured.
pub enum ConfigError {
    /// Rx or Tx not enabled
    RxOrTxNotEnabled,

    /// The baud rate could not be configured with the given clocks.
    InvalidBaudRate,

    /// [`Config::low_power_rx_wake`] was set on an instance that deep sleep powers down.
    ///
    /// SYSCTL disables PD1 peripherals on entry to STOP and STANDBY, so a PD1 UART cannot detect the
    /// start bit that would wake the chip. Use a PD0 instance.
    NoDeepSleepWake,
}

/// How full a FIFO must be before it raises an interrupt.
///
/// The FIFOs are four entries deep, and the level decides how much of the handler's fixed cost is
/// amortised: at [`AtLeastOne`](Self::AtLeastOne) it runs once per byte, at [`Half`](Self::Half) once
/// per two. A level above one entry needs the receive timeout, which the driver arms alongside it, or a
/// FIFO that never reaches the level is never delivered.
///
/// Receive and transmit read the same encoding from opposite ends — a receive level is how many entries
/// have arrived, a transmit level how many are free — so one setting configures both sensibly.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoThreshold {
    /// A single entry: interrupt as soon as one byte can move.
    ///
    /// Lowest latency and highest cost. This was the driver's only behaviour before the level was
    /// configurable, and it is what holds the reliable receive rate below 460800 baud.
    AtLeastOne,

    /// One of four.
    Quarter,

    /// Two of four. The default, and the fastest of these that measured clean to 921600.
    Half,

    /// Three of four.
    ThreeQuarter,

    /// All four. Most batching, and the longest a byte can wait for the timeout to deliver it.
    Full,
}

impl FifoThreshold {
    /// The receive level, as the register encodes it for an instance in `domain`.
    ///
    /// **A PD0 instance has only two levels**, one entry and full, encoded differently from every other
    /// instance's; SLAU846 Table 24-44 says anything else falls back to the reset value. Rather than
    /// leave that silent, everything from half up takes the full level.
    ///
    /// Rounding *up* is measured, not a guess. On a G3507, whose `UART1` is PD0, half-mapped-to-full
    /// receives a 921600 baud stream with 0.27% loss where half-mapped-to-one-entry loses 19%. The finer
    /// levels a non-PD0 instance has are the untested path here.
    const fn rx(self, domain: PowerDomain) -> vals::Iflssel {
        match (domain, self) {
            (PowerDomain::Pd0, Self::AtLeastOne | Self::Quarter) => vals::Iflssel::OneFourthUlp,
            (PowerDomain::Pd0, Self::Half | Self::ThreeQuarter | Self::Full) => vals::Iflssel::FullUlp,
            (_, Self::AtLeastOne) => vals::Iflssel::AtLeastOne,
            (_, Self::Quarter) => vals::Iflssel::OneFourth,
            (_, Self::Half) => vals::Iflssel::Half,
            (_, Self::ThreeQuarter) => vals::Iflssel::ThreeFourth,
            (_, Self::Full) => vals::Iflssel::Full,
        }
    }

    /// The transmit level, which has no per-domain restriction.
    const fn tx(self) -> vals::Iflssel {
        match self {
            Self::AtLeastOne => vals::Iflssel::AtLeastOne,
            Self::Quarter => vals::Iflssel::OneFourth,
            Self::Half => vals::Iflssel::Half,
            Self::ThreeQuarter => vals::Iflssel::ThreeFourth,
            Self::Full => vals::Iflssel::Full,
        }
    }
}

/// How the line rate reaches the hardware.
///
/// One value rather than a rate and an optional override, because only one of those two was ever
/// read: a pre-solved divider won and the rate beside it was ignored, with nothing to say so.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BaudRate {
    /// Solve for this rate on the device.
    ///
    /// **Not free.** The search divides, so it pulls in the 32-bit division and long-multiply
    /// helpers a core with no divider needs: measured at `opt-level = "z"` with fat LTO, the same
    /// program costs **664 bytes of flash and 40 of RAM more** than one handing a divider over.
    /// That is a fifth of a small binary, and none of it is reachable once the rate is known up
    /// front.
    Rate(u32),

    /// Apply a divider solved ahead of time, skipping the search.
    ///
    /// Build one with [`Baud::solve`] in a `const` when the clock and the rate are both known at
    /// compile time.
    Solved(Baud),
}

impl From<u32> for BaudRate {
    fn from(rate: u32) -> Self {
        Self::Rate(rate)
    }
}

impl From<Baud> for BaudRate {
    fn from(baud: Baud) -> Self {
        Self::Solved(baud)
    }
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// How a [`Uart`] drives its line.
///
/// `repr(C)` here is worth 200 bytes and is not decoration. Left to order these fields itself the
/// compiler groups the single-byte ones, and since most of them default to their zero discriminant that
/// leaves a six-byte run of zeroes for `Default` to write — which LLVM merges into one memset and then
/// lowers to a call, pulling a 192-byte helper into every binary that builds a UART. Declaration order
/// interleaves the fields that default to something else, so no run long enough to be worth a call forms.
///
/// Measured both ways: the helper goes, and no binary that builds no UART moves at all. **Reordering
/// these fields can bring it back silently**, so check for `__aeabi_memclr` if you do.
#[repr(C)]
pub struct Config {
    /// UART clock source.
    pub clock_source: ClockSel,

    /// The line rate, either as a number to solve for or as a divider already solved.
    pub baud: BaudRate,

    /// Number of data bits.
    pub data_bits: DataBits,

    /// Number of stop bits.
    pub stop_bits: StopBits,

    /// Parity type.
    pub parity: Parity,

    /// The order of bits in a transmitted/received byte.
    pub msb_order: BitOrder,

    /// If true: the `TX` is internally connected to `RX`.
    pub loop_back_enable: bool,

    // Manchester coding is an extended-UART feature, and the metadata defines one `uart` block
    // version for every instance, so nothing here can tell an extended instance from a main one.
    // /// If true: [manchester coding] is used.
    // ///
    // /// [manchester coding]: https://en.wikipedia.org/wiki/Manchester_code
    // pub manchester: bool,
    /// How full a FIFO must be before it raises an interrupt, or `None` to run without the FIFOs.
    ///
    /// One enable bit covers both directions, so this is one setting rather than two. Without the FIFOs
    /// each direction is a single byte deep and the handler runs once per byte, which is what bounds the
    /// receive rate: measured on a G3507, the receiver loses 2.9% of a 460800 baud stream at
    /// [`AtLeastOne`](FifoThreshold::AtLeastOne) and everything above it, against 0.03% at
    /// [`Half`](FifoThreshold::Half) up to 921600.
    pub fifo: Option<FifoThreshold>,

    /// If true: invert TX pin signal values (V<sub>DD</sub> = 0/mark, Gnd = 1/idle).
    pub invert_tx: bool,

    /// If true: invert RX pin signal values (V<sub>DD</sub> = 0/mark, Gnd = 1/idle).
    pub invert_rx: bool,

    /// If true: invert RTS pin signal values (V<sub>DD</sub> = 0/mark, Gnd = 1/idle).
    pub invert_rts: bool,

    /// If true: invert CTS pin signal values (V<sub>DD</sub> = 0/mark, Gnd = 1/idle).
    pub invert_cts: bool,

    /// Set the pull configuration for the TX pin.
    pub tx_pull: Pull,

    /// Set the pull configuration for the RX pin.
    pub rx_pull: Pull,

    /// Set the pull configuration for the RTS pin.
    pub rts_pull: Pull,

    /// Set the pull configuration for the CTS pin.
    pub cts_pull: Pull,

    /// Let the chip deep-sleep (down to STANDBY0) while an async [`BufferedUart`] receiver is
    /// listening, waking on an incoming RX start bit.
    ///
    /// Only PD0 instances can do this; anything else is a [`ConfigError::NoDeepSleepWake`].
    pub low_power_rx_wake: bool,
}

impl Config {
    /// Use a divider solved ahead of time, skipping the search on the device.
    ///
    /// Replaces whatever [`Self::baud`] held, rate or divider, since it is one or the other.
    pub const fn with_baud(mut self, baud: Baud) -> Self {
        self.baud = BaudRate::Solved(baud);
        self
    }
}

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            clock_source: ClockSel::MfClk,
            baud: BaudRate::Rate(115200),
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::Stop1,
            parity: Parity::ParityNone,
            // hardware default
            msb_order: BitOrder::LsbFirst,
            loop_back_enable: false,
            // manchester: false,
            // Measured rather than assumed: the level is what decides whether the handler amortises
            // across the FIFO, and half-full is clean where one entry is not.
            fifo: Some(FifoThreshold::Half),
            invert_tx: false,
            invert_rx: false,
            invert_rts: false,
            invert_cts: false,
            tx_pull: Pull::None,
            rx_pull: Pull::None,
            rts_pull: Pull::None,
            cts_pull: Pull::None,
            low_power_rx_wake: false,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Bidirectional UART Driver, which acts as a combination of [`UartTx`] and [`UartRx`].
///
/// ### Notes on [`embedded_io::Read`]
///
/// [`embedded_io::Read`] requires guarantees that the base [`UartRx`] cannot provide.
///
/// See [`UartRx`] for more details, and [`BufferedUart`] for an alternative that does provide them.
pub struct Uart<'d, M: ModeState> {
    tx: UartTx<'d, M>,
    rx: UartRx<'d, M>,
}

impl<'d, M: ModeState> SetConfig for Uart<'d, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(config)
    }
}

/// Serial error
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    Framing,

    Noise,

    /// The receiver dropped at least one byte.
    ///
    /// How many is not in here: a flag cannot say, and the buffered driver reports the figure through
    /// [`BufferedUartRx::take_dropped`] instead.
    Overrun,

    Parity,

    Break,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::Framing => "Framing Error",
            Self::Noise => "Noise Error",
            Self::Overrun => "RX Buffer Overrun",
            Self::Parity => "Parity Check Error",
            Self::Break => "Break Error",
        };

        write!(f, "{}", message)
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// Rx-only UART Driver.
///
/// Can be obtained from [`Uart::split`], or can be constructed independently,
/// if you do not need the transmitting half of the driver.
pub struct UartRx<'d, M: ModeState> {
    /// The registers, the pins and the configuration. Everything below adds a way to wait.
    inner: low_level::UartRx<'d>,
    /// Zero-sized unless this driver can wait; see [`ModeState`].
    wait: M::Wait,
    _phantom: PhantomData<M>,
}

impl<'d, M: ModeState> SetConfig for UartRx<'d, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(config)
    }
}

impl<'d> UartRx<'d, Blocking> {
    /// Create a new rx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Rx. It saves 1 pin.
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(peri, new_pin!(rx, config.rx_pf()), None, (), config)
    }

    /// Create a new rx-only UART with a request-to-send pin
    pub fn new_blocking_with_rts<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        rts: Peri<'d, impl RtsPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, config.rx_pf()),
            new_pin!(rts, config.rts_pf()),
            (),
            config,
        )
    }
}

impl<'d> UartRx<'d, Async> {
    /// Create a new rx-only UART that waits on the FIFO rather than a software buffer.
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let this = Self::new_inner(peri, new_pin!(rx, config.rx_pf()), None, T::async_state(), config)?;
        enable_interrupt::<T>();

        Ok(this)
    }

    /// Create a new rx-only UART with a request-to-send pin.
    pub fn new_with_rts<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        rts: Peri<'d, impl RtsPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let this = Self::new_inner(
            peri,
            new_pin!(rx, config.rx_pf()),
            new_pin!(rts, config.rts_pf()),
            T::async_state(),
            config,
        )?;
        enable_interrupt::<T>();

        Ok(this)
    }

    /// Fill `buffer`, waiting on the receive FIFO for as long as it takes.
    ///
    /// The FIFO is four entries deep and nothing here adds to it, so the wait tolerates the caller
    /// being away for four character times and no more. Past that the receiver overruns and the byte
    /// that caused it is reported as [`Error::Overrun`] — [`BufferedUart`] is what absorbs a longer
    /// absence.
    ///
    /// An error abandons the read with the bytes already in `buffer` written and no count of them.
    pub fn read<'a>(&'a mut self, buffer: &'a mut [u8]) -> impl Future<Output = Result<(), Error>> + 'a {
        let r = self.inner.info.regs;
        let state = self.wait;
        let mut read = 0;

        poll_fn(move |cx| {
            clear(r, rx_sources());

            while let Some(slot) = buffer.get_mut(read) {
                if rx_empty(r) {
                    break;
                }

                compiler_fence(Ordering::Acquire);
                match read_with_error(r) {
                    Ok(byte) => {
                        *slot = byte;
                        read += 1;
                    }
                    Err(err) => return Poll::Ready(Err(err)),
                }
            }

            if read == buffer.len() {
                return Poll::Ready(Ok(()));
            }

            state.rx_waker.register(cx.waker());
            unmask(r, rx_sources());

            Poll::Pending
        })
    }
}

impl<'d, M: ModeState> UartRx<'d, M> {
    /// Perform a blocking read into `buffer`
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let r = self.inner.info.regs;

        for b in buffer {
            // Wait if nothing has arrived yet.
            while rx_empty(r) {}

            // Prevent the compiler from reading from buffer too early
            compiler_fence(Ordering::Acquire);
            *b = read_with_error(r)?;
        }

        Ok(())
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.inner.set_config(config)
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        self.inner.set_baudrate(baudrate)
    }
}

/// Tx-only UART Driver.
///
/// Can be obtained from [`Uart::split`], or can be constructed independently,
/// if you do not need the receiving half of the driver.
pub struct UartTx<'d, M: ModeState> {
    /// The registers, the pins and the configuration. Everything below adds a way to wait.
    inner: low_level::UartTx<'d>,
    /// Zero-sized unless this driver can wait; see [`ModeState`].
    wait: M::Wait,
    /// Resolved once rather than per call. It derives from the bus clock, which nothing but a
    /// reconfigure changes, and recomputing it put the clock-tree lookup in every `write` and
    /// `flush`. Absent rather than `None` without `low-power`, so it costs no byte there.
    #[cfg(feature = "low-power")]
    sleep_floor: Option<SleepLevel>,
    _phantom: PhantomData<M>,
}

impl<'d, M: ModeState> SetConfig for UartTx<'d, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    // Unlike its two siblings this does not forward to the inherent `set_config`, so a pin's function
    // is left as it was. Preserved rather than fixed here: the restructure this file just went
    // through is gated on generating the same code, and a behaviour change would be invisible in that
    // gate.
    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        reconfigure(self.inner.info, self.inner.state, config)?;
        self.resolve_sleep_floor();

        Ok(())
    }
}

impl<'d> UartTx<'d, Blocking> {
    /// Create a new blocking tx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Tx. It saves 1 pin.
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(peri, new_pin!(tx, config.tx_pf()), None, (), config)
    }

    /// Create a new blocking tx-only UART with a clear-to-send pin
    pub fn new_blocking_with_cts<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        cts: Peri<'d, impl CtsPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, config.tx_pf()),
            new_pin!(cts, config.cts_pf()),
            (),
            config,
        )
    }
}

impl<'d> UartTx<'d, Async> {
    /// Create a new tx-only UART that waits on the FIFO rather than a software buffer.
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let this = Self::new_inner(peri, new_pin!(tx, config.tx_pf()), None, T::async_state(), config)?;
        enable_interrupt::<T>();

        Ok(this)
    }

    /// Create a new tx-only UART with a clear-to-send pin.
    pub fn new_with_cts<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        cts: Peri<'d, impl CtsPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let this = Self::new_inner(
            peri,
            new_pin!(tx, config.tx_pf()),
            new_pin!(cts, config.cts_pf()),
            T::async_state(),
            config,
        )?;
        enable_interrupt::<T>();

        Ok(this)
    }

    /// Send every byte of `buffer`, and wait for the last of them to leave the wire.
    ///
    /// Sent, not merely queued: this resolves once the transmitter is idle, so the sleep guard it
    /// holds covers the whole transmission and deep sleep entered afterwards cannot cut a frame. That
    /// makes a following [`flush`](Self::flush) redundant rather than merely cheap.
    ///
    /// The cost is on the wire, not the CPU. Waiting out the transmitter means the line goes idle
    /// between one write and the next, where returning early let the FIFO cover the gap. A caller
    /// sending a stream in small pieces gets fewer bytes per second for the same CPU. Send bigger
    /// pieces rather than reaching for a way to overlap them.
    ///
    /// One bit time is still outstanding when this resolves; see the module documentation.
    pub fn write<'a>(&'a mut self, buffer: &'a [u8]) -> impl Future<Output = Result<(), Error>> + 'a {
        let r = self.inner.info.regs;
        let state = self.wait;
        let mut written = 0;

        // Held for the whole future, not just the first poll. Unlike `BufferedUartTx::write_inner`,
        // which only fills a software ring, this queues straight into the hardware FIFO and then
        // returns `Pending` with those bytes already going out — so yielding here hands the executor
        // an idle path to sleep down while a frame is in flight. `begin_blocking_write` guards the
        // same window.
        let guard = MaybeWakeGuard::new(self.wake_floor());

        poll_fn(move |cx| {
            let _ = &guard;

            // `written` is the phase as well as the position, so draining costs no extra state in the
            // future: the buffer is exhausted only once, and every poll after that is a drain poll.
            if written < buffer.len() {
                clear(r, tx_sources());

                while let Some(&byte) = buffer.get(written) {
                    if tx_full(r) {
                        break;
                    }

                    compiler_fence(Ordering::Release);
                    write_byte(r, byte);
                    written += 1;
                }

                if written < buffer.len() {
                    state.tx_waker.register(cx.waker());
                    unmask(r, tx_sources());

                    return Poll::Pending;
                }
            }

            // Cleared before the test, so an end-of-transmission left over from an earlier write
            // cannot answer for this one.
            clear(r, eot_sources());

            if !busy(r) {
                return Poll::Ready(Ok(()));
            }

            state.tx_waker.register(cx.waker());
            unmask(r, eot_sources());

            Poll::Pending
        })
    }

    /// Wait for the transmitter to go idle, so nothing is left in the FIFO or the shift register.
    ///
    /// [`write`](Self::write) already waits for this before it resolves, so calling this after one is
    /// redundant. It is here for a transmitter that something else queued into — the blocking write
    /// paths, or a [`TxWrite`] given up with [`disarm`](TxWrite::disarm).
    pub fn flush(&mut self) -> impl Future<Output = Result<(), Error>> + '_ {
        let r = self.inner.info.regs;
        let state = self.wait;

        // The same guard `BufferedUartTx::flush_inner` takes, for the same reason: this is the call
        // that waits for the transmitter to drain, so it is the one that must keep the chip shallow
        // until it has.
        let guard = MaybeWakeGuard::new(self.wake_floor());

        poll_fn(move |cx| {
            let _ = &guard;

            clear(r, eot_sources());

            if !busy(r) {
                return Poll::Ready(Ok(()));
            }

            state.tx_waker.register(cx.waker());
            unmask(r, eot_sources());

            Poll::Pending
        })
    }
}

impl<'d, M: ModeState> UartTx<'d, M> {
    /// Open a transmission, returning the [`TxWrite`] that queues the bytes and sees them onto the wire.
    ///
    /// A write only queues: the last bytes are still in the FIFO when it returns, and on most families
    /// still in the shift register after that. Deep sleep entered in between cuts the transmission
    /// mid-byte with nothing reporting it, so the guard holds the chip shallow until it drains, and
    /// **dropping it waits**.
    ///
    /// One write is a single expression, and the wait happens at the semicolon:
    ///
    /// ```ignore
    /// tx.begin_blocking_write().write(b"hello\n")?;
    /// ```
    ///
    /// A run of them keeps the guard, so the wait is paid once at the end rather than after each:
    ///
    /// ```ignore
    /// let mut w = tx.begin_blocking_write();
    /// w.write(header)?;
    /// for chunk in body {
    ///     w.write(chunk)?;
    /// }
    /// ```
    ///
    /// [`TxWrite::disarm`] gives up the guard without waiting.
    pub fn begin_blocking_write(&mut self) -> TxWrite<'_, 'd, M> {
        // Taken before any byte goes out, so the level is held for the whole time any of them is in
        // flight rather than from whenever the last one was queued.
        TxWrite {
            regs: self.inner.info.regs,
            guard: MaybeWakeGuard::new(self.wake_floor()),
            tx: PhantomData,
        }
    }

    /// Shallowest level to block while a transmission is in flight.
    ///
    /// Asked only when there is a sleep to prevent. Reading the clock keeps `configure`'s store to it
    /// live, and with it the clock-tree lookup — 16 bytes of flash and a 40-byte static in a binary
    /// that cannot sleep at all.
    #[cfg(feature = "low-power")]
    fn wake_floor(&self) -> Option<SleepLevel> {
        self.sleep_floor
    }

    /// Re-resolve the floor. Anything that changes the instance's bus clock has to call this.
    fn resolve_sleep_floor(&mut self) {
        #[cfg(feature = "low-power")]
        {
            self.sleep_floor = self.inner.transmit_floor();
        }
    }

    #[cfg(not(feature = "low-power"))]
    fn wake_floor(&self) -> Option<SleepLevel> {
        None
    }

    /// Block until transmission completes.
    ///
    /// [`TxWrite`] does this when it is dropped, so this is for the case where the transmitter was fed
    /// some other way.
    ///
    /// Waits for the transmit FIFO to drain, one bit time short of the wire going idle. See the module
    /// documentation on waiting out that last bit.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        while self.inner.busy() {}
        Ok(())
    }

    /// Send break character
    pub fn send_break(&self) {
        self.inner.send_break();
    }

    /// Check if UART is busy.
    pub fn busy(&self) -> bool {
        self.inner.busy()
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.inner.set_config(config)?;
        self.resolve_sleep_floor();

        Ok(())
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        self.inner.set_baudrate(baudrate)
    }
}

/// An open transmission: bytes written through it may not have reached the wire yet.
///
/// Holds the chip shallow enough that the transmitter keeps running, and **waits for it to drain when
/// dropped** — so the hazard is closed by doing nothing. Write through it to queue more under the same
/// guard, [`Self::flush`] to wait now and see any error, or [`Self::disarm`] to abandon the bytes to
/// whatever the device does next.
///
/// The drain leaves one bit time outstanding; see the module documentation.
///
/// Not `#[must_use]`: dropping this immediately is the correct thing, not a mistake.
pub struct TxWrite<'a, 'd, M: ModeState> {
    regs: Regs,
    guard: MaybeWakeGuard,
    /// Borrows the driver without holding a reference to it.
    ///
    /// A `&'a mut UartTx` here would give this type's drop glue a route to the whole driver, and the
    /// compiler then cannot prove that nothing reads back the clock `configure` stores — which keeps
    /// the clock-tree lookup and its 40-byte static alive in binaries that never ask. Measured at 164
    /// bytes on a plain transmit binary. Only the registers are needed to finish a write.
    tx: PhantomData<&'a mut UartTx<'d, M>>,
}

impl<'a, 'd, M: ModeState> TxWrite<'a, 'd, M> {
    /// Queue more bytes, keeping the one guard.
    ///
    /// This is what makes a loop of writes cost one wait rather than one per iteration.
    pub fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let r = self.regs;

        for &b in buffer {
            // Wait only while there is nowhere to put the byte. Waiting for the FIFO to *empty* instead
            // spends the depth it was configured with: one byte would be in flight at a time whatever
            // `Config::fifo` asked for, and the call would return that much later with the rest still to
            // send. Both bits track `CTL0.FEN`, so this reads correctly with the FIFOs off too.
            while tx_full(r) {}

            // Prevent the compiler from writing to buffer too early
            compiler_fence(Ordering::Release);
            write_byte(r, b);
        }

        Ok(())
    }

    /// Wait for the transmitter to drain, then release the guard.
    ///
    /// The same wait dropping this performs, with the error visible.
    pub fn flush(self) -> Result<(), Error> {
        // Skips the drop below, which would otherwise wait a second time. Releasing the guard by hand
        // is the whole of what that drop would have left to do.
        let mut this = core::mem::ManuallyDrop::new(self);
        while busy(this.regs) {}
        this.guard.release();
        Ok(())
    }

    /// Release the guard without waiting, leaving the queued bytes to take their chances.
    ///
    /// Deep sleep entered after this truncates whatever has not reached the wire, which is what a
    /// blocking write did before it handed out a guard.
    pub fn disarm(self) {
        let mut this = core::mem::ManuallyDrop::new(self);
        this.guard.release();
    }
}

impl<'a, 'd, M: ModeState> Drop for TxWrite<'a, 'd, M> {
    fn drop(&mut self) {
        while busy(self.regs) {}
    }
}

impl<'d> Uart<'d, Blocking> {
    /// Create a new blocking bidirectional UART.
    pub fn new_blocking<T: Instance>(
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
            (),
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins
    pub fn new_blocking_with_rtscts<T: Instance>(
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
            (),
            config,
        )
    }
}

impl<'d> Uart<'d, Async> {
    /// Create a new bidirectional UART that waits on the FIFOs rather than a software buffer.
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, config.tx_pf()),
            new_pin!(rx, config.rx_pf()),
            None,
            None,
            T::async_state(),
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins.
    pub fn new_with_rtscts<T: Instance>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        rts: Peri<'d, impl RtsPin<T>>,
        cts: Peri<'d, impl CtsPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, config.tx_pf()),
            new_pin!(rx, config.rx_pf()),
            new_pin!(rts, config.rts_pf()),
            new_pin!(cts, config.cts_pf()),
            T::async_state(),
            config,
        )
    }

    /// Fill `buffer`; see [`UartRx::read`].
    pub fn read<'a>(&'a mut self, buffer: &'a mut [u8]) -> impl Future<Output = Result<(), Error>> + 'a {
        self.rx.read(buffer)
    }

    /// Send every byte of `buffer` and wait for the wire to go idle; see [`UartTx::write`].
    pub fn write<'a>(&'a mut self, buffer: &'a [u8]) -> impl Future<Output = Result<(), Error>> + 'a {
        self.tx.write(buffer)
    }

    /// Wait for the transmitter to go idle; see [`UartTx::flush`].
    pub fn flush(&mut self) -> impl Future<Output = Result<(), Error>> + '_ {
        self.tx.flush()
    }
}

impl<'d, M: ModeState> Uart<'d, M> {
    /// Open a transmission. See [`UartTx::begin_blocking_write`], which this defers to.
    pub fn begin_blocking_write(&mut self) -> TxWrite<'_, 'd, M> {
        self.tx.begin_blocking_write()
    }

    /// Block until transmission complete
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    /// Check if UART is busy.
    pub fn busy(&self) -> bool {
        self.tx.busy()
    }

    /// Perform a blocking read into `buffer`
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    /// Split the Uart into a transmitter and receiver, which is
    /// particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartTx<'d, M>, UartRx<'d, M>) {
        (self.tx, self.rx)
    }

    /// Split the Uart into a transmitter and receiver by mutable reference,
    /// which is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split_ref(&mut self) -> (&mut UartTx<'d, M>, &mut UartRx<'d, M>) {
        (&mut self.tx, &mut self.rx)
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.tx.set_config(config)?;
        self.rx.set_config(config)
    }

    /// Send break character
    pub fn send_break(&self) {
        self.tx.send_break();
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        self.tx.set_baudrate(baudrate)
    }
}

/// Peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// UART `TX` pin trait
pub trait TxPin<T: Instance>: crate::gpio::Pin {
    /// Get the PF number needed to use this pin as `TX`.
    fn pf_num(&self) -> u8;
}

/// UART `RX` pin trait
pub trait RxPin<T: Instance>: crate::gpio::Pin {
    /// Get the PF number needed to use this pin as `RX`.
    fn pf_num(&self) -> u8;
}

/// UART `CTS` pin trait
pub trait CtsPin<T: Instance>: crate::gpio::Pin {
    /// Get the PF number needed to use this pin as `CTS`.
    fn pf_num(&self) -> u8;
}

/// UART `RTS` pin trait
pub trait RtsPin<T: Instance>: crate::gpio::Pin {
    /// Get the PF number needed to use this pin as `RTS`.
    fn pf_num(&self) -> u8;
}

/// Let the instance's line reach the CPU.
///
/// Only the half-duplex async constructors need this: [`Uart::new_inner`] already enables the line for
/// every mode. Nothing is unmasked in `IMASK` here, so the line stays quiet until a wait arms it.
fn enable_interrupt<T: Instance>() {
    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };
}

// ==== IMPL types ====

/// What a driver has to carry in order to wait, which is nothing unless it can.
///
/// A `Blocking` driver's is `()`, so a binary that never awaits a UART carries no reference to the
/// wakers and never links the static holding them. That is the whole reason this is an associated type
/// rather than a field on [`State`]: the wakers are 16 bytes of `.bss` that only an async caller uses.
#[doc(hidden)]
pub trait ModeState: Mode {
    /// Where this mode's driver finds whom to wake, if it can wait at all.
    type Wait: Copy;
}

impl ModeState for Blocking {
    type Wait = ();
}

impl ModeState for Async {
    type Wait = &'static AsyncState;
}

/// Wakers for the unbuffered async driver.
///
/// Separate from [`BufferedState`] rather than shared with it: the two paths never run on the same
/// instance, and a caller that binds one has no use for the other's storage.
#[doc(hidden)]
pub struct AsyncState {
    /// Woken for anything the receiver waits on — a FIFO at its level, and the timeout that delivers
    /// one that never reaches it.
    rx_waker: IrqWaker,
    /// Woken for room in the transmit FIFO, and for the end of transmission a drain waits on.
    tx_waker: IrqWaker,
}

impl AsyncState {
    pub const fn new() -> Self {
        Self {
            rx_waker: IrqWaker::new(),
            tx_waker: IrqWaker::new(),
        }
    }
}

impl Default for AsyncState {
    fn default() -> Self {
        Self::new()
    }
}

/// Interrupt handler for the unbuffered async driver.
///
/// Deliberately not [`BufferedInterruptHandler`]: that one drains the FIFO into a ring, tracks error
/// flags and manages backpressure, and a caller who wanted none of that would still link all of it.
/// This one masks what fired and wakes, and the future does the rest.
pub struct InterruptHandler<T: Instance> {
    _uart: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::info().regs;
        let int = masked_status(r);

        // Mask rather than clear. Every source armed here is a FIFO level or a timeout, and `RIS` is
        // sticky: clearing it while the condition still holds re-raises the line the moment this
        // returns. The future clears and re-arms once it has drained, which is the only point at which
        // the condition is known to be gone.
        mask(r, int);

        let state = T::async_state();

        if int.rxint() || int.rtout() {
            state.rx_waker.wake();
        }

        if int.txint() || int.eot() {
            state.tx_waker.wake();
        }
    }
}

// Every wait below runs the same three steps in the same order, and the order is the whole of what
// makes them safe:
//
//   1. clear, before looking at the FIFO at all;
//   2. move what can be moved — drain the receiver, fill the transmitter;
//   3. register, then unmask, if there is more to do.
//
// `RIS` is sticky, so the clear has to come first. Clearing after step 2 discards the flag left by a
// byte that arrived while the drain was running, and the task then parks with that byte sitting in the
// FIFO and nothing left to raise the line — a stall that ends only when the next byte happens to
// arrive. Clearing first costs at worst one spurious wake, whose poll finds nothing to move, clears
// again and unmasks against a receiver that is genuinely idle.

impl<'d, M: ModeState> UartRx<'d, M> {
    fn new_inner<T: Instance>(
        peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        wait: M::Wait,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Ok(Self {
            inner: low_level::UartRx::new_inner(peri, rx, rts, config)?,
            wait,
            _phantom: PhantomData,
        })
    }
}

impl<'d, M: ModeState> UartTx<'d, M> {
    fn new_inner<T: Instance>(
        peri: Peri<'d, T>,
        tx: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        wait: M::Wait,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let mut this = Self {
            inner: low_level::UartTx::new_inner(peri, tx, cts, config)?,
            wait,
            #[cfg(feature = "low-power")]
            sleep_floor: None,
            _phantom: PhantomData,
        };
        this.resolve_sleep_floor();

        Ok(this)
    }
}

impl<'d, M: ModeState> Uart<'d, M> {
    fn new_inner<T: Instance>(
        peri: Peri<'d, T>,
        tx: Option<Peri<'d, AnyPin>>,
        rx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        wait: M::Wait,
        config: Config,
    ) -> Result<Self, ConfigError> {
        // One `configure` for both directions, so the halves are built together and split afterwards
        // rather than each configuring the instance for itself.
        let (tx, rx) = low_level::Uart::new_inner(peri, tx, rx, rts, cts, config)?.split();

        let mut this = Self {
            tx: UartTx {
                inner: tx,
                wait,
                #[cfg(feature = "low-power")]
                sleep_floor: None,
                _phantom: PhantomData,
            },
            rx: UartRx {
                inner: rx,
                wait,
                _phantom: PhantomData,
            },
        };
        this.tx.resolve_sleep_floor();

        // Only the full-duplex constructors enable the line for every mode; the half-duplex async ones
        // call `enable_interrupt` for themselves. [`low_level`] enables nothing, since its whole point
        // is that the vector belongs to the application.
        let info = this.rx.inner.info;
        info.interrupt.unpend();
        unsafe { info.interrupt.enable() };

        Ok(this)
    }
}

impl Config {
    fn tx_pf(&self) -> PfType {
        PfType::output(self.tx_pull, self.invert_tx)
    }

    fn rx_pf(&self) -> PfType {
        PfType::input(self.rx_pull, self.invert_rx)
    }

    fn rts_pf(&self) -> PfType {
        PfType::output(self.rts_pull, self.invert_rts)
    }

    fn cts_pf(&self) -> PfType {
        PfType::input(self.cts_pull, self.invert_cts)
    }
}

/// A solved baud-rate divider.
///
/// [`Baud::solve`] is a `const fn`, so a program whose clock and baud rate are known up front can
/// solve at compile time and keep the search out of the binary:
///
/// ```no_run
/// # #![no_std]
/// # #[panic_handler]
/// # fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
/// use embassy_mspm0::sysctl;
/// use embassy_mspm0::uart::{Baud, ClockSel, Config};
///
/// # fn main() {
/// // The clock tree is a constant, so its rates are too.
/// const CLOCK: u32 = sysctl::clock::RESET_SETUP.clocks().ulpclk;
/// const BAUD: Baud = match Baud::solve(ClockSel::MfClk, CLOCK, 9600) {
///     Some(baud) => baud,
///     None => core::panic!("9600 baud is not reachable from this clock"),
/// };
///
/// let config = Config::default().with_baud(BAUD);
/// # }
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
// Naturally 2, because every field is sub-word. At 2 the compiler is free to write `ibrd` and `fbrd`
// as one misaligned word, which ARMv6-M cannot do inline and which lowers to an out-of-line
// `__aeabi_uwrite4`. Two bytes of padding on a type that is usually a `const` buys that back.
#[repr(align(4))]
pub struct Baud {
    hse: vals::Hse,
    div: vals::Clkdiv,
    ibrd: u16,
    fbrd: u8,
}

impl Baud {
    // Quoting SLAU846 section 18.2.3.4:
    // "When IBRD = 0, FBRD is ignored and no data gets transferred by the UART."
    const MIN_IBRD: u16 = 1;

    // FBRD can be 0. FBRD is at most a 6-bit number.
    const MAX_FBRD: u8 = 2_u8.pow(6);

    const DIVS: [(u8, vals::Clkdiv); 8] = [
        (1, vals::Clkdiv::DivBy1),
        (2, vals::Clkdiv::DivBy2),
        (3, vals::Clkdiv::DivBy3),
        (4, vals::Clkdiv::DivBy4),
        (5, vals::Clkdiv::DivBy5),
        (6, vals::Clkdiv::DivBy6),
        (7, vals::Clkdiv::DivBy7),
        (8, vals::Clkdiv::DivBy8),
    ];

    // Quoting from SLAU 846 section 18.2.3.4:
    // "Select oversampling by 3 or 8 to achieve higher speed with UARTclk/8 or UARTclk/3. In this case
    //  the receiver tolerance to clock deviation is reduced."
    //
    // "Select oversampling by 16 to increase the tolerance of the receiver to clock deviations. The
    //  maximum speed is limited to UARTclk/16."
    //
    // Based on these requirements, prioritize higher oversampling first to increase tolerance to clock
    // deviation. If no valid BRD value can be found satisifying the highest sample rate, then reduce
    // sample rate until valid parameters are found.
    const OVS: [(u8, vals::Hse); 3] = [(16, vals::Hse::Ovs16), (8, vals::Hse::Ovs8), (3, vals::Hse::Ovs3)];

    /// Solve for `baudrate` on an instance clocked from `source`, or [`None`] if it cannot be reached.
    ///
    /// Usable in a `const`, which is the point: handing the result to [`Config::with_baud`] keeps the
    /// search — and the 32-bit division it needs — out of the binary entirely.
    ///
    /// `clock_hz` must be the rate `source` actually runs at, which [`ClockSel::frequency`] gives.
    ///
    /// # Oversampling
    ///
    /// 16x, 8x and 3x are tried in that order, because a higher oversampling tolerates more clock
    /// deviation at the receiver. Only the rates that fit nothing else fall to 3x, and **from LFCLK
    /// that is most of the useful ones**: 8x needs eight times the baud rate, so 32.768 kHz reaches
    /// only 4096 baud without 3x and 10922 with it, which is what puts 4800 and 9600 in range.
    ///
    /// `source` is needed because `UART_ERR_03` makes 3x unsafe from BUSCLK and MFCLK on the parts that
    /// carry it — TI's own workaround is to source LFCLK where 3x is required.
    ///
    /// The peripheral also forbids 3x under Manchester coding, in DALI mode and with IrDA. None of the
    /// three is reachable through this driver — [`Config`] does not offer them, `configure` writes them
    /// off, and the IrDA register is only ever read — so this answer is exact rather than optimistic.
    /// **Adding any of them to [`Config`] means revisiting this.**
    ///
    /// ```no_run
    /// # #![no_std]
    /// # #[panic_handler]
    /// # fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
    /// use embassy_mspm0::sysctl::clock;
    /// use embassy_mspm0::uart::{Baud, ClockSel};
    ///
    /// const CLOCKS: clock::Clocks = clock::RESET_SETUP.clocks();
    /// const BAUD: Baud = match Baud::solve(ClockSel::LfClk, CLOCKS.lfclk, 9600) {
    ///     Some(baud) => baud,
    ///     None => core::panic!("9600 is not reachable from LFCLK"),
    /// };
    /// # fn main() {}
    /// ```
    pub const fn solve(source: ClockSel, clock_hz: u32, baudrate: u32) -> Option<Self> {
        // `UART_ERR_03` — 3x from BUSCLK or MFCLK sets RXINT erroneously and can corrupt transmitted
        // data.
        let allow_x3 = !(cfg!(uart_err_03) && matches!(source, ClockSel::BusClk | ClockSel::MfClk));

        let mut o = 0;
        while o < Self::OVS.len() {
            let (oversampling, hse) = Self::OVS[o];
            o += 1;

            if matches!(hse, vals::Hse::Ovs3) && !allow_x3 {
                continue;
            }

            // Verify that the selected oversampling does not require a clock faster than what the
            // hardware is provided.
            let Some(min_clock) = baudrate.checked_mul(oversampling as u32) else {
                continue;
            };

            if min_clock > clock_hz {
                continue;
            }

            let mut d = 0;
            while d < Self::DIVS.len() {
                let (div, div_value) = Self::DIVS[d];
                d += 1;

                let Some((ibrd, fbrd)) = calculate_brd(clock_hz, div, baudrate, oversampling) else {
                    continue;
                };

                if ibrd < Self::MIN_IBRD || fbrd > Self::MAX_FBRD {
                    continue;
                }

                return Some(Self {
                    hse,
                    div: div_value,
                    ibrd,
                    fbrd,
                });
            }
        }

        None
    }

    /// Program this divider into the peripheral.
    fn apply(&self, regs: Regs) {
        regs.clkdiv().write(|w| w.set_ratio(self.div));
        regs.ibrd().write(|w| w.set_divint(self.ibrd));
        regs.fbrd().write(|w| w.set_divfrac(self.fbrd));
        regs.ctl0().modify(|w| w.set_hse(self.hse));
    }

    /// The integer and fractional parts of `BRD`, as programmed.
    pub const fn brd(&self) -> (u16, u8) {
        (self.ibrd, self.fbrd)
    }
}

/// `floor(num * 64 / den)`, or [`None`] if the quotient does not fit a `u32`.
///
/// Produces the 6 fractional bits one at a time rather than widening `num` by 64 up front, which
/// would overflow a `u32` above 67.1 MHz and in 64 bits would pull in `__aeabi_uldivmod`.
const fn scaled_div_q6(num: u32, den: u32) -> Option<u32> {
    // `r` is doubled each round, so anything above this would wrap before being reduced.
    if den == 0 || den > u32::MAX / 2 {
        return None;
    }

    let mut q = num / den;
    let mut r = num % den;

    let mut bit = 0;
    while bit < 6 {
        q = match q.checked_mul(2) {
            Some(q) => q,
            None => return None,
        };

        // `r < den` holds on entry, so this cannot overflow given the bound checked above.
        r *= 2;
        if r >= den {
            r -= den;
            q += 1;
        }

        bit += 1;
    }

    Some(q)
}

/// Calculate the integer and fractional parts of the `BRD` value.
///
/// Returns [`None`] if calculating this results in overflows.
///
/// Values returned are `(ibrd, fbrd)`
const fn calculate_brd(clock: u32, div: u8, baud: u32, oversampling: u8) -> Option<(u16, u8)> {
    // Calculate BRD according to SLAU 846 section 18.2.3.4.
    //
    // BRD is a 22-bit value with 16 integer bits and 6 fractional bits.
    //
    // uart_clock = clock / div
    // brd = ibrd.fbrd = uart_clock / (oversampling * baud)
    //
    // Both divisions truncate to the 6 fractional bits BRD can represent, and for positive integers
    // `floor(floor(a / b) / c) == floor(a / (b * c))`, so the two collapse into one division by
    // `div * oversampling * baud` without changing the result.
    let Some(den) = (div as u32).checked_mul(oversampling as u32) else {
        return None;
    };
    let Some(den) = den.checked_mul(baud) else {
        return None;
    };

    let Some(brd) = scaled_div_q6(clock, den) else {
        return None;
    };

    // BRD is a U16F6, so anything above this cannot be programmed.
    let ibrd = brd >> 6;
    if ibrd > u16::MAX as u32 {
        return None;
    }

    // The fractional part is already scaled by 64 by virtue of being the low 6 bits, which is what
    // `FBRD = INT(FRAC(BRD) * 64)` asks for.
    let fbrd = (brd & 0x3f) as u8;

    Some((ibrd as u16, fbrd))
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
    fn buffered_state() -> &'static BufferedState;
    fn async_state() -> &'static AsyncState;
}

macro_rules! impl_uart_instance {
    ($instance: ident) => {
        impl crate::uart::SealedInstance for crate::peripherals::$instance {
            fn info() -> &'static crate::uart::Info {
                use crate::interrupt::typelevel::Interrupt;
                use crate::uart::Info;

                const INFO: Info = Info {
                    regs: crate::pac::$instance,
                    interrupt: crate::interrupt::typelevel::$instance::IRQ,
                    sleep: <crate::peripherals::$instance as crate::sysctl::LowPowerInstance>::SLEEP,
                };
                &INFO
            }

            fn state() -> &'static crate::uart::State {
                use crate::uart::State;

                static STATE: State = State::new();
                &STATE
            }

            fn buffered_state() -> &'static crate::uart::BufferedState {
                use crate::uart::BufferedState;

                static STATE: BufferedState = BufferedState::new();
                &STATE
            }

            fn async_state() -> &'static crate::uart::AsyncState {
                use crate::uart::AsyncState;

                static STATE: AsyncState = AsyncState::new();
                &STATE
            }
        }

        impl crate::uart::Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

macro_rules! impl_uart_tx_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::uart::TxPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

macro_rules! impl_uart_rx_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::uart::RxPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

macro_rules! impl_uart_cts_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::uart::CtsPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

macro_rules! impl_uart_rts_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::uart::RtsPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use super::{Baud, ClockSel, calculate_brd, vals};

    /// What 3x oversampling is worth, at both ends of the range.
    ///
    /// It is not a corner: 3x moves the reachable band from `clock/8` up to `clock/3` at the top, and
    /// from LFCLK it is the difference between 4096 baud and 10922 — which is what puts 4800 and 9600
    /// in range at all.
    #[test]
    fn three_times_oversampling_widens_the_band() {
        const LFCLK: u32 = 32_768;
        const MFCLK: u32 = 4_000_000;

        // Below the 8x ceiling, nothing changes.
        for baud in [2400, 4096] {
            core::assert!(Baud::solve(ClockSel::LfClk, LFCLK, baud).is_some(), "{baud}");
        }

        // Between the two ceilings, 3x is the only thing that reaches.
        for baud in [4800, 9600, 10_922] {
            core::assert!(Baud::solve(ClockSel::LfClk, LFCLK, baud).is_some(), "{baud} wants 3x");
        }

        // Past 3x's own ceiling nothing helps.
        core::assert!(Baud::solve(ClockSel::LfClk, LFCLK, 19_200).is_none());
        core::assert!(Baud::solve(ClockSel::MfClk, MFCLK, MFCLK / 3 + 1).is_none());
    }

    /// `UART_ERR_03` bars 3x from BUSCLK and MFCLK, and only on the parts that carry it.
    ///
    /// LFCLK is never barred, which is what makes TI's workaround — source LFCLK where 3x is needed —
    /// something this solver can actually honour.
    #[test]
    fn errata_bars_three_times_only_where_it_applies() {
        const LFCLK: u32 = 32_768;

        // Reachable only at 3x, so it answers the question directly.
        let barred = Baud::solve(ClockSel::MfClk, LFCLK, 9600).is_none();
        core::assert_eq!(barred, cfg!(uart_err_03));

        core::assert!(Baud::solve(ClockSel::LfClk, LFCLK, 9600).is_some());
    }

    /// This is a smoke test based on the example in SLAU 846 section 18.2.3.4.
    #[test]
    fn datasheet() {
        let brd = calculate_brd(40_000_000, 1, 19200, 16);

        core::assert!(matches!(brd, Some((130, 13))));
    }

    /// What `calculate_brd` is defined to compute, done in 64 bits.
    ///
    /// `BRD = clock / (div * oversampling * baud)`, truncated to 6 fractional bits.
    fn reference(clock: u32, div: u8, baud: u32, oversampling: u8) -> Option<(u16, u8)> {
        let den = (div as u64) * (oversampling as u64) * (baud as u64);
        if den == 0 {
            return None;
        }

        let brd = (clock as u64) * 64 / den;
        let ibrd = brd >> 6;
        if ibrd > u16::MAX as u64 {
            return None;
        }

        Some((ibrd as u16, (brd & 0x3f) as u8))
    }

    /// The 32-bit implementation must agree with the 64-bit definition everywhere it is reachable.
    #[test]
    fn matches_reference() {
        // Every divider and oversampling the search loop in `set_baudrate_inner` can pick.
        const DIVS: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
        const OVS: [u8; 3] = [16, 8, 3];

        // Clocks the tree can produce, including the 80 MHz the G-series reaches through SYSPLL,
        // and the standard baud rates plus the extremes around them.
        const CLOCKS: [u32; 9] = [
            32_768, 4_000_000, 16_000_000, 24_000_000, 32_000_000, 40_000_000, 48_000_000, 64_000_000, 80_000_000,
        ];
        const BAUDS: [u32; 12] = [
            300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115_200, 230_400, 921_600, 4_000_000,
        ];

        for clock in CLOCKS {
            for div in DIVS {
                for ovs in OVS {
                    for baud in BAUDS {
                        core::assert_eq!(
                            calculate_brd(clock, div, baud, ovs),
                            reference(clock, div, baud, ovs),
                            "clock={clock} div={div} baud={baud} ovs={ovs}"
                        );
                    }
                }
            }
        }
    }

    /// The const solver must agree with what the runtime search would pick.
    #[test]
    fn const_baud_matches_search() {
        for clock in [32_768u32, 4_000_000, 32_000_000, 80_000_000] {
            for baud in [9600u32, 19200, 115_200] {
                // `solve` excludes 3x oversampling, so compare against the same restriction.
                core::assert_eq!(
                    Baud::solve(ClockSel::LfClk, clock, baud),
                    Baud::solve(ClockSel::MfClk, clock, baud),
                    "clock={clock} baud={baud}"
                );
            }
        }
    }

    /// The case this exists for: a constant clock and a constant baud rate, solved at build time.
    #[test]
    fn const_baud_is_const_evaluable() {
        const BAUD: Baud = match Baud::solve(ClockSel::MfClk, 4_000_000, 9600) {
            Some(baud) => baud,
            None => core::panic!("9600 baud must be reachable from MFCLK"),
        };

        // 4 MHz / (1 * 16 * 9600) = 26.041..., so IBRD 26 and FBRD 2 (0.0416 * 64 = 2.67 -> 2).
        core::assert_eq!(BAUD.brd(), (26, 2));
        core::assert_eq!(Some(BAUD), Baud::solve(ClockSel::MfClk, 4_000_000, 9600));
    }

    /// The previous implementation converted the clock into a `U26F6`, whose integer part tops out
    /// at 67_108_863, so it panicked for any clock above that. The G-series reaches 80 MHz.
    #[test]
    fn handles_clocks_above_26_bits() {
        // 80 MHz / (1 * 16 * 115200) = 43.402..., so IBRD 43 and FBRD 25 (0.402 * 64 = 25.7).
        core::assert_eq!(calculate_brd(80_000_000, 1, 115_200, 16), Some((43, 25)));
        core::assert_eq!(
            calculate_brd(80_000_000, 1, 115_200, 16),
            reference(80_000_000, 1, 115_200, 16)
        );
    }

    /// Denominators large enough to overflow must be rejected, not wrap.
    #[test]
    fn rejects_unrepresentable() {
        // Baud far above the clock leaves IBRD 0, which the caller rejects, but it must not panic.
        core::assert_eq!(calculate_brd(32_768, 8, 4_000_000, 16), Some((0, 0)));

        // A denominator beyond `u32::MAX / 2` is refused rather than wrapping.
        core::assert_eq!(calculate_brd(80_000_000, 8, u32::MAX, 16), None);

        // A clock this low with a slow baud still fits, and agrees with the definition.
        core::assert_eq!(calculate_brd(32_768, 1, 300, 3), reference(32_768, 1, 300, 3));
    }
}
