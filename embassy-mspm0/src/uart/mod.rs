//! Universal Asynchronous Receiver/Transmitter (UART) driver.
//!
//! # Deep sleep truncates an unflushed write
//!
//! Every write here returns once the bytes are queued, not once they are on the wire. Deep sleep
//! entered before the transmitter drains cuts the frame mid-byte, and on a PD1 instance the TX pin
//! then sits low until the next wake. Flush before awaiting anything that can sleep.
//!
//! Nothing reports this: the bytes were accepted, and the receiver on the other end sees a framing
//! error rather than a byte the sender can act on.
#![macro_use]

mod buffered;

use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering, compiler_fence};

pub use buffered::*;
use embassy_embedded_hal::SetConfig;
use embassy_hal_internal::PeripheralType;

use crate::Peri;
use crate::gpio::{AnyPin, PfType, Pull, SealedPin};
use crate::interrupt::{Interrupt, InterruptExt};
use crate::mode::{Blocking, Mode};
use crate::pac::uart::{Uart as Regs, vals};
use crate::sysctl::{PowerDomain, SleepInfo, WakeGuard};

/// Bit times of silence after which the receiver reports a FIFO that has not reached its level.
///
/// Must exceed 1: `UART_ERR_11` starts the counter in the middle of the STOP bit, so 1 fires early. The
/// resulting timeout is `(RX_TIMEOUT_BITS - 0.5) / baud`, so this is a little under one character —
/// short enough that a trailing byte is not held up, long enough that a back-to-back stream never
/// reaches it.
const RX_TIMEOUT_BITS: u8 = 8;

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
    /// use embassy_mspm0::sysctl::{LowPowerInstance, clock};
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
/// Config Error
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

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// Config
pub struct Config {
    /// UART clock source.
    pub clock_source: ClockSel,

    /// Baud rate.
    ///
    /// Ignored when [`Self::baud`] carries a pre-solved divider.
    pub baudrate: u32,

    /// A divider solved ahead of time, skipping the search on the device.
    ///
    /// Build one with [`Baud::solve`] in a `const` when the clock and baud rate are both known at
    /// compile time. Leave as [`None`] to solve at runtime from [`Self::baudrate`].
    pub baud: Option<Baud>,

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

    // TODO: Pending way to check if uart is extended
    // /// If true: [manchester coding] is used.
    // ///
    // /// [manchester coding]: https://en.wikipedia.org/wiki/Manchester_code
    // pub manchester: bool,

    // TODO: majority voting
    /// How full a FIFO must be before it raises an interrupt, or `None` to run without the FIFOs.
    ///
    /// One enable bit covers both directions, so this is one setting rather than two. Without the FIFOs
    /// each direction is a single byte deep and the handler runs once per byte, which is what bounds the
    /// receive rate: measured on a G3507, the receiver loses 2.9% of a 460800 baud stream at
    /// [`AtLeastOne`](FifoThreshold::AtLeastOne) and everything above it, against 0.03% at
    /// [`Half`](FifoThreshold::Half) up to 921600.
    pub fifo: Option<FifoThreshold>,

    // TODO: glitch suppression
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
    /// [`Self::baudrate`] is ignored once this is set.
    pub const fn with_baud(mut self, baud: Baud) -> Self {
        self.baud = Some(baud);
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_source: ClockSel::MfClk,
            baudrate: 115200,
            baud: None,
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

/// Bidirectional UART Driver, which acts as a combination of [`UartTx`] and [`UartRx`].
///
/// ### Notes on [`embedded_io::Read`]
///
/// [`embedded_io::Read`] requires guarantees that the base [`UartRx`] cannot provide.
///
/// See [`UartRx`] for more details, and [`BufferedUart`] for an alternative that does provide them.
pub struct Uart<'d, M: Mode> {
    tx: UartTx<'d, M>,
    rx: UartRx<'d, M>,
}

impl<'d, M: Mode> SetConfig for Uart<'d, M> {
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
pub struct UartRx<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    rx: Option<Peri<'d, AnyPin>>,
    rts: Option<Peri<'d, AnyPin>>,
    /// Held for as long as the driver exists; see [`SleepInfo::floor_to_keep_configured`].
    _retention_guard: Option<WakeGuard>,
    _phantom: PhantomData<M>,
}

impl<'d, M: Mode> SetConfig for UartRx<'d, M> {
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
        Self::new_inner(peri, new_pin!(rx, config.rx_pf()), None, config)
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
            config,
        )
    }
}

impl<'d, M: Mode> UartRx<'d, M> {
    /// Perform a blocking read into `buffer`
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let r = self.info.regs;

        for b in buffer {
            // Wait if nothing has arrived yet.
            while r.stat().read().rxfe() {}

            // Prevent the compiler from reading from buffer too early
            compiler_fence(Ordering::Acquire);
            *b = read_with_error(r)?;
        }

        Ok(())
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        if let Some(ref rx) = self.rx {
            rx.update_pf(config.rx_pf());
        }

        if let Some(ref rts) = self.rts {
            rts.update_pf(config.rts_pf());
        }

        reconfigure(self.info, self.state, config)
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        set_baudrate(&self.info, self.state.clock.load(Ordering::Relaxed), baudrate)
    }
}

impl<'d, M: Mode> Drop for UartRx<'d, M> {
    fn drop(&mut self) {
        self.rx.as_ref().map(|x| x.set_as_disconnected());
        self.rts.as_ref().map(|x| x.set_as_disconnected());
    }
}

/// Tx-only UART Driver.
///
/// Can be obtained from [`Uart::split`], or can be constructed independently,
/// if you do not need the receiving half of the driver.
pub struct UartTx<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    tx: Option<Peri<'d, AnyPin>>,
    cts: Option<Peri<'d, AnyPin>>,
    /// Held for as long as the driver exists; see [`SleepInfo::floor_to_keep_configured`].
    _retention_guard: Option<WakeGuard>,
    _phantom: PhantomData<M>,
}

impl<'d, M: Mode> SetConfig for UartTx<'d, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        reconfigure(self.info, self.state, config)
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
        Self::new_inner(peri, new_pin!(tx, config.tx_pf()), None, config)
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
            config,
        )
    }
}

impl<'d, M: Mode> UartTx<'d, M> {
    /// Perform a blocking UART write
    ///
    /// Returns once the last byte is queued, not once it has been transmitted. Call
    /// [`Self::blocking_flush`] before anything that can deep sleep.
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let r = self.info.regs;

        for &b in buffer {
            // Wait if there is no space
            while !r.stat().read().txfe() {}

            // Prevent the compiler from writing to buffer too early
            compiler_fence(Ordering::Release);
            r.txdata().write(|w| {
                w.set_data(b);
            });
        }

        Ok(())
    }

    /// Block until transmission completes.
    ///
    /// [`Self::blocking_write`] returns as soon as the last byte is queued, so deep sleep entered before
    /// this returns cuts the transmission mid-byte. On the families affected by `UART_ERR_08` this can
    /// only wait for the FIFO to drain, leaving the byte in the shift register still going.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        while busy(self.info.regs) {}
        Ok(())
    }

    /// Send break character
    pub fn send_break(&self) {
        let r = self.info.regs;

        r.lcrh().modify(|w| {
            w.set_brk(true);
        });
    }

    /// Check if UART is busy.
    pub fn busy(&self) -> bool {
        busy(self.info.regs)
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        if let Some(ref tx) = self.tx {
            tx.update_pf(config.tx_pf());
        }

        if let Some(ref cts) = self.cts {
            cts.update_pf(config.cts_pf());
        }

        reconfigure(self.info, self.state, config)
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        set_baudrate(&self.info, self.state.clock.load(Ordering::Relaxed), baudrate)
    }
}

impl<'d, M: Mode> Drop for UartTx<'d, M> {
    fn drop(&mut self) {
        self.tx.as_ref().map(|x| x.set_as_disconnected());
        self.cts.as_ref().map(|x| x.set_as_disconnected());
    }
}

impl<'d> Uart<'d, Blocking> {
    /// Create a new blocking bidirectional UART.
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, config.rx_pf()),
            new_pin!(tx, config.tx_pf()),
            None,
            None,
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins
    pub fn new_blocking_with_rtscts<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        rts: Peri<'d, impl RtsPin<T>>,
        cts: Peri<'d, impl CtsPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, config.rx_pf()),
            new_pin!(tx, config.tx_pf()),
            new_pin!(rts, config.rts_pf()),
            new_pin!(cts, config.cts_pf()),
            config,
        )
    }
}

impl<'d, M: Mode> Uart<'d, M> {
    /// Perform a blocking write
    ///
    /// Returns once the last byte is queued, not once it has been transmitted. Call
    /// [`Self::blocking_flush`] before anything that can deep sleep.
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
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
        set_baudrate(&self.tx.info, self.tx.state.clock.load(Ordering::Relaxed), baudrate)
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

/// Let this instance raise an asynchronous fast clock request.
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
pub(crate) fn retention_guard(info: &'static Info) -> Option<WakeGuard> {
    info.sleep.floor_to_keep_configured().map(WakeGuard::new)
}

// ==== IMPL types ====

pub(crate) struct Info {
    pub(crate) regs: Regs,
    pub(crate) interrupt: Interrupt,
    pub(crate) sleep: SleepInfo,
}

pub(crate) struct State {
    /// The clock rate of the UART in Hz.
    clock: AtomicU32,
}

impl State {
    pub const fn new() -> Self {
        Self {
            clock: AtomicU32::new(0),
        }
    }
}

impl<'d, M: Mode> UartRx<'d, M> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let mut this = Self {
            info: T::info(),
            state: T::state(),
            rx,
            rts,
            _retention_guard: retention_guard(T::info()),
            _phantom: PhantomData,
        };
        this.enable_and_configure(&config)?;

        Ok(this)
    }

    fn enable_and_configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        let info = self.info;

        enable(info.regs);
        configure(info, self.state, config, true, self.rts.is_some(), false, false)?;

        Ok(())
    }
}

impl<'d, M: Mode> UartTx<'d, M> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        tx: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let mut this = Self {
            info: T::info(),
            state: T::state(),
            tx,
            cts,
            _retention_guard: retention_guard(T::info()),
            _phantom: PhantomData,
        };
        this.enable_and_configure(&config)?;

        Ok(this)
    }

    fn enable_and_configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        let info = self.info;
        let state = self.state;

        enable(info.regs);
        configure(info, state, config, false, false, true, self.cts.is_some())?;

        Ok(())
    }
}

impl<'d, M: Mode> Uart<'d, M> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        tx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let info = T::info();
        let state = T::state();

        let mut this = Self {
            tx: UartTx {
                info,
                state,
                tx,
                cts,
                _retention_guard: retention_guard(info),
                _phantom: PhantomData,
            },
            rx: UartRx {
                info,
                state,
                rx,
                rts,
                _retention_guard: retention_guard(info),
                _phantom: PhantomData,
            },
        };
        this.enable_and_configure(&config)?;

        Ok(this)
    }

    fn enable_and_configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        let info = self.rx.info;
        let state = self.rx.state;

        enable(info.regs);
        configure(
            info,
            state,
            config,
            true,
            self.rx.rts.is_some(),
            true,
            self.tx.cts.is_some(),
        )?;

        info.interrupt.unpend();
        unsafe { info.interrupt.enable() };

        Ok(())
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
        PfType::input(self.rts_pull, self.invert_rts)
    }
}

fn enable(regs: Regs) {
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
}

fn configure(
    info: &Info,
    state: &State,
    config: &Config,
    enable_rx: bool,
    enable_rts: bool,
    enable_tx: bool,
    enable_cts: bool,
) -> Result<(), ConfigError> {
    let r = info.regs;

    if !enable_rx && !enable_tx {
        return Err(ConfigError::RxOrTxNotEnabled);
    }

    if config.low_power_rx_wake {
        if !info.sleep.power_domain.is_powered_in_deep_sleep() {
            return Err(ConfigError::NoDeepSleepWake);
        }

        arm_async_clock_request(info);
    }

    // SLAU846B says that clocks should be enabled before disabling the uart.
    r.clksel().write(|w| match config.clock_source {
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
    let clock = crate::sysctl::with_clocks(|clocks| match config.clock_source {
        ClockSel::LfClk => clocks.lfclk,
        ClockSel::MfClk => clocks.mfclk,
        ClockSel::BusClk => clocks.bus_clock(info.sleep.power_domain),
    });

    state.clock.store(clock, Ordering::Relaxed);

    info.regs.ctl0().modify(|w| {
        w.set_lbe(config.loop_back_enable);
        // Errata UART_ERR_02, must set RXE to allow use of EOT.
        w.set_rxe(enable_rx | enable_tx);
        w.set_txe(enable_tx);
        // RXD_OUT_EN and TXD_OUT_EN?
        w.set_menc(false);
        w.set_mode(vals::Mode::Uart);
        w.set_rtsen(enable_rts);
        w.set_ctsen(enable_cts);
        // oversampling is set later
        w.set_fen(config.fifo.is_some());
        // TODO: config
        w.set_majvote(false);
        w.set_msbfirst(matches!(config.msb_order, BitOrder::MsbFirst));
    });

    // A FIFO is only worth having if the interrupt batches across it. At one entry the handler runs once
    // per byte and its fixed cost is never amortised, which is what bounds the receive rate rather than
    // any buffer size. Half-full halves the entries; the FIFOs are four deep.
    //
    // With the FIFOs off there is one byte of depth and no level to reach, so the choice only applies
    // when they are on.
    let (rx_level, tx_level) = if let Some(threshold) = config.fifo {
        (threshold.rx(info.sleep.power_domain), threshold.tx())
    } else {
        (vals::Iflssel::AtLeastOne, vals::Iflssel::AtLeastOne)
    };

    info.regs.ifls().modify(|w| {
        w.set_txiflsel(tx_level);
        w.set_rxiflsel(rx_level);
        // A receive level above one entry needs the timeout armed, or a partial FIFO waits for bytes
        // that never come and the last few of a message are never delivered. Zero, the reset value,
        // disables it entirely and is what makes `RTOUT` unable to fire.
        w.set_rxtosel(if config.fifo.is_some() { RX_TIMEOUT_BITS } else { 0 });
    });

    info.regs.lcrh().modify(|w| {
        let eps = if matches!(config.parity, Parity::ParityEven) {
            vals::Eps::Even
        } else {
            vals::Eps::Odd
        };

        let wlen = match config.data_bits {
            DataBits::DataBits5 => vals::Wlen::Databit5,
            DataBits::DataBits6 => vals::Wlen::Databit6,
            DataBits::DataBits7 => vals::Wlen::Databit7,
            DataBits::DataBits8 => vals::Wlen::Databit8,
        };

        // Used in LIN mode only
        w.set_brk(false);
        w.set_pen(config.parity != Parity::ParityNone);
        w.set_eps(eps);
        w.set_stp2(matches!(config.stop_bits, StopBits::Stop2));
        w.set_wlen(wlen);
        // appears to only be used in RS-485 mode.
        w.set_sps(false);
        // IDLE pattern?
        w.set_sendidle(false);
        // ignore extdir_setup and extdir_hold, only used in RS-485 mode.
    });

    // A pre-solved divider skips the search entirely, which is what keeps the software divider out
    // of the binary when the clock and baud rate are both compile-time constants.
    match config.baud {
        Some(baud) => baud.apply(info.regs),
        None => set_baudrate_inner(info.regs, clock, config.baudrate)?,
    }

    r.ctl0().modify(|w| {
        w.set_enable(true);
    });

    Ok(())
}

fn reconfigure(info: &Info, state: &State, config: &Config) -> Result<(), ConfigError> {
    info.interrupt.disable();
    let r = info.regs;
    let ctl0 = r.ctl0().read();
    configure(info, state, config, ctl0.rxe(), ctl0.rtsen(), ctl0.txe(), ctl0.ctsen())?;

    info.interrupt.unpend();
    unsafe { info.interrupt.enable() };

    Ok(())
}

/// Set the baud rate and clock settings.
///
/// This should be done relatively late during configuration since some clock settings are invalid depending on mode.
fn set_baudrate(info: &Info, clock: u32, baudrate: u32) -> Result<(), ConfigError> {
    let r = info.regs;

    info.interrupt.disable();

    // Wait for end of transmission per suggestion in SLAU 845 section 18.3.28. It has to happen while
    // the transmitter still runs: disabling completes only the character already in the shift register
    // (SLAU846 table 24-41), so anything left in the FIFO stays there and a wait after the disable
    // never finishes.
    while busy(r) {}

    // Programming baud rate requires that the peripheral is disabled
    critical_section::with(|_cs| {
        r.ctl0().modify(|w| {
            w.set_enable(false);
        });
    });

    set_baudrate_inner(r, clock, baudrate)?;

    critical_section::with(|_cs| {
        r.ctl0().modify(|w| {
            w.set_enable(true);
        });
    });

    info.interrupt.unpend();
    unsafe { info.interrupt.enable() };

    Ok(())
}

fn set_baudrate_inner(regs: Regs, clock: u32, baudrate: u32) -> Result<(), ConfigError> {
    // 3x oversampling is not supported with manchester coding, DALI or IrDA.
    let allow_x3 = {
        let ctl0 = regs.ctl0().read();
        let irctl = regs.irctl().read();

        // `UART_ERR_03` — 3x oversampling sourced from BUSCLK or MFCLK sets RXINT erroneously and can
        // corrupt transmitted data. TI's workaround is to oversample higher, or to use LFCLK where 3x
        // is required, so drop 3x and let the search fall back.
        let errata_x3 = if cfg!(uart_err_03) {
            let clksel = regs.clksel().read();
            clksel.busclk_sel() || clksel.mfclk_sel()
        } else {
            false
        };

        !(ctl0.menc() || matches!(ctl0.mode(), vals::Mode::Dali) || irctl.iren() || errata_x3)
    };

    let Some(baud) = Baud::solve_inner(clock, baudrate, allow_x3) else {
        return Err(ConfigError::InvalidBaudRate);
    };

    baud.apply(regs);

    Ok(())
}

/// A solved baud-rate divider.
///
/// [`Baud::solve`] is a `const fn`, so a program whose clock and baud rate are known up front can
/// solve at compile time and keep the search out of the binary:
///
/// ```ignore
/// use embassy_mspm0::uart::{Baud, Config};
/// use embassy_mspm0::sysctl;
///
/// // The clock tree is a constant, so its rates are too.
/// const CLOCK: u32 = sysctl::clock::RESET_SETUP.clocks().ulpclk;
/// const BAUD: Baud = match Baud::solve(CLOCK, 9600) {
///     Some(baud) => baud,
///     None => panic!("9600 baud is not reachable from this clock"),
/// };
///
/// let config = Config::default().with_baud(BAUD);
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

    /// Solve for `baudrate` from a `clock_hz` source, or [`None`] if it cannot be reached.
    ///
    /// Usable in a `const`. Only 8x and 16x oversampling are considered, which covers every ordinary
    /// baud rate: whether 3x is legal depends on runtime state a compile-time solve cannot inspect.
    pub const fn solve(clock_hz: u32, baudrate: u32) -> Option<Self> {
        Self::solve_inner(clock_hz, baudrate, false)
    }

    /// [`Self::solve`], with 3x oversampling permitted when the caller has checked it is legal.
    const fn solve_inner(clock: u32, baudrate: u32, allow_x3: bool) -> Option<Self> {
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

            if min_clock > clock {
                continue;
            }

            let mut d = 0;
            while d < Self::DIVS.len() {
                let (div, div_value) = Self::DIVS[d];
                d += 1;

                let Some((ibrd, fbrd)) = calculate_brd(clock, div, baudrate, oversampling) else {
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

    /// The integer and fractional parts of `BRD`, as programmed.
    pub const fn brd(&self) -> (u16, u8) {
        (self.ibrd, self.fbrd)
    }

    /// Program this divider into the peripheral.
    fn apply(&self, regs: Regs) {
        regs.clkdiv().write(|w| w.set_ratio(self.div));
        regs.ibrd().write(|w| w.set_divint(self.ibrd));
        regs.fbrd().write(|w| w.set_divfrac(self.fbrd));
        regs.ctl0().modify(|w| w.set_hse(self.hse));
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

fn read_with_error(r: Regs) -> Result<u8, Error> {
    let rx = r.rxdata().read();

    if rx.frmerr() {
        return Err(Error::Framing);
    } else if rx.parerr() {
        return Err(Error::Parity);
    } else if rx.brkerr() {
        return Err(Error::Break);
    } else if rx.ovrerr() {
        return Err(Error::Overrun);
    } else if rx.nerr() {
        return Err(Error::Noise);
    }

    Ok(rx.data())
}

/// This function assumes CTL0.ENABLE is set (for errata cases).
fn busy(r: Regs) -> bool {
    // `UART_ERR_08` — `STAT.BUSY` stays high even with the module disabled and data in the TX FIFO, so
    // polling it never finishes. Applies to every family this crate builds for except G511x/G5187,
    // whose UNICOMM UART is a different module.
    if cfg!(uart_err_08) {
        let stat = r.stat().read();
        // "Poll TXFIFO status and the CTL0.ENABLE register bit to identify BUSY status."
        !stat.txfe()
    } else {
        r.stat().read().busy()
    }
}

// TODO: Implement when dma uart is implemented.
fn dma_enabled(_r: Regs) -> bool {
    false
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
    fn buffered_state() -> &'static BufferedState;
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
    use super::{Baud, calculate_brd};

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
                    Baud::solve(clock, baud),
                    Baud::solve_inner(clock, baud, false),
                    "clock={clock} baud={baud}"
                );
            }
        }
    }

    /// The case this exists for: a constant clock and a constant baud rate, solved at build time.
    #[test]
    fn const_baud_is_const_evaluable() {
        const BAUD: Baud = match Baud::solve(4_000_000, 9600) {
            Some(baud) => baud,
            None => core::panic!("9600 baud must be reachable from MFCLK"),
        };

        // 4 MHz / (1 * 16 * 9600) = 26.041..., so IBRD 26 and FBRD 2 (0.0416 * 64 = 2.67 -> 2).
        core::assert_eq!(BAUD.brd(), (26, 2));
        core::assert_eq!(Some(BAUD), Baud::solve_inner(4_000_000, 9600, false));
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
