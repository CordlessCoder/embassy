#![macro_use]

use core::future;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use embassy_embedded_hal::SetConfig;
use embassy_hal_internal::PeripheralType;
use embassy_hal_internal::drop::OnDrop;
use embassy_sync::waitqueue::AtomicWaker;
use mspm0_metapac::i2c;

use crate::Peri;
use crate::gpio::{AnyPin, PfType, Pull, SealedPin};
use crate::interrupt::typelevel::Binding;
use crate::interrupt::{Interrupt, InterruptExt};
use crate::mode::{Async, Blocking, Mode};
use crate::pac::i2c::{I2c as Regs, vals};
use crate::pac::{self};
use crate::sysctl::{SleepInfo, SleepLevel, WakeGuard};

/// The clock source for the I2C.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSel {
    /// Use the bus clock.
    ///
    /// Configurable clock.
    BusClk,

    /// Use the middle frequency clock.
    ///
    /// The MCLK runs at 4 MHz.
    MfClk,
}

impl ClockSel {
    /// Rate this source feeds the peripheral at, before the peripheral's own divider.
    ///
    /// Takes the tree rather than reading it, so this stays a `const fn` and [`Timing::solve`] can be
    /// evaluated at compile time. `BusClk` is ULPCLK rather than MCLK because every I2C instance is in
    /// PD0 — asserted per instance in `impl_i2c_instance!`, so this does not have to ask for the domain.
    pub const fn frequency(self, clocks: &crate::sysctl::Clocks) -> u32 {
        match self {
            // MFCLK is held at 4 MHz by SYSCTL whatever SYSOSC is doing, and reads as 0 when it was
            // never enabled, in which case the peripheral would not be clocked at all.
            Self::MfClk => clocks.mfclk,
            Self::BusClk => clocks.ulpclk,
        }
    }
}

/// The clock divider for the I2C.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockDiv {
    // "Do not divide clock source.
    DivBy1,
    // "Divide clock source by 2.
    DivBy2,
    // "Divide clock source by 3.
    DivBy3,
    // "Divide clock source by 4.
    DivBy4,
    // "Divide clock source by 5.
    DivBy5,
    // "Divide clock source by 6.
    DivBy6,
    // "Divide clock source by 7.
    DivBy7,
    // "Divide clock source by 8.
    DivBy8,
}

impl ClockDiv {
    pub(crate) fn into(self) -> vals::Ratio {
        match self {
            Self::DivBy1 => vals::Ratio::DivBy1,
            Self::DivBy2 => vals::Ratio::DivBy2,
            Self::DivBy3 => vals::Ratio::DivBy3,
            Self::DivBy4 => vals::Ratio::DivBy4,
            Self::DivBy5 => vals::Ratio::DivBy5,
            Self::DivBy6 => vals::Ratio::DivBy6,
            Self::DivBy7 => vals::Ratio::DivBy7,
            Self::DivBy8 => vals::Ratio::DivBy8,
        }
    }

    const fn divider(self) -> u32 {
        match self {
            Self::DivBy1 => 1,
            Self::DivBy2 => 2,
            Self::DivBy3 => 3,
            Self::DivBy4 => 4,
            Self::DivBy5 => 5,
            Self::DivBy6 => 6,
            Self::DivBy7 => 7,
            Self::DivBy8 => 8,
        }
    }
}

/// The I2C mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusSpeed {
    /// Standard mode.
    ///
    /// The Standard mode runs at 100 kHz.
    Standard,

    /// Fast mode.
    ///
    /// The fast mode runs at 400 kHz.
    FastMode,

    /// Fast mode plus.
    ///
    /// The fast mode plus runs at 1 MHz.
    FastModePlus,

    /// Custom mode.
    ///
    /// The custom mode frequency (in Hz) can be set manually.
    Custom(u32),
}

impl BusSpeed {
    fn hertz(self) -> u32 {
        match self {
            Self::Standard => 100_000,
            Self::FastMode => 400_000,
            Self::FastModePlus => 1_000_000,
            Self::Custom(s) => s,
        }
    }
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Config Error
pub enum ConfigError {
    /// Invalid clock rate.
    ///
    /// The clock rate could not be configured with the given conifguratoin.
    InvalidClockRate,

    /// Clock source not enabled.
    ///
    /// The clock soure is not enabled is SYSCTL.
    ClockSourceNotEnabled,

    /// Invalid target address.
    ///
    /// The target address is not 7-bit.
    InvalidTargetAddress,
}

/// A solved I2C timing, so the device never has to divide.
///
/// [`Timing::solve`] is a `const fn`, so a bus speed known up front costs no division on the device:
///
/// ```ignore
/// use embassy_mspm0::i2c::{ClockDiv, ClockSel, Config, Timing};
/// use embassy_mspm0::sysctl::clock;
///
/// const CLOCK: clock::ClockSetup = clock::Config::new().build();
/// const TIMING: Timing = match Timing::solve(&CLOCK.clocks(), ClockSel::MfClk, ClockDiv::DivBy1, 100_000) {
///     Some(timing) => timing,
///     None => panic!("100 kHz is not reachable from MFCLK"),
/// };
///
/// let config = Config::default().with_timing(TIMING);
/// ```
///
/// A period only means anything against the clock it was solved for, so the source is part of the
/// solution and [`Config::with_timing`] programs both.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timing {
    clock_source: ClockSel,
    clock_div: ClockDiv,
    tpr: u8,
    clock_hz: u32,
}

impl Timing {
    /// Solve the timer period for `bus_speed_hz` from `clock_source` on the tree `clocks` describes.
    ///
    /// Takes the tree rather than reading it, so this stays a `const fn`: pass
    /// [`clock::ClockSetup::clocks`](crate::sysctl::clock::ClockSetup::clocks) for the tree
    /// [`crate::init`] is being given, which is the one the peripheral will run on.
    ///
    /// Returns [`None`] if the resulting `TPR` is outside the 1..=127 the register holds, or if the
    /// source is not at least 20x the bus speed, which is the same headroom the runtime path checks.
    pub const fn solve(
        clocks: &crate::sysctl::Clocks,
        clock_source: ClockSel,
        clock_div: ClockDiv,
        bus_speed_hz: u32,
    ) -> Option<Self> {
        let i2c_clk = clock_source.frequency(clocks) / clock_div.divider();

        // Same 20x headroom [`Config::resolve`] requires at runtime.
        let Some(needed) = bus_speed_hz.checked_mul(20) else {
            return None;
        };
        if i2c_clk < needed {
            return None;
        }

        let Some(denominator) = bus_speed_hz.checked_mul(10) else {
            return None;
        };
        if denominator == 0 {
            return None;
        }

        let ticks = i2c_clk / denominator;
        if ticks == 0 || ticks > 128 {
            return None;
        }

        Some(Self {
            clock_source,
            clock_div,
            tpr: (ticks - 1) as u8,
            clock_hz: i2c_clk,
        })
    }

    /// The `TPR` value this programs.
    pub const fn tpr(&self) -> u8 {
        self.tpr
    }

    /// The rate the peripheral sees after its own divider.
    pub const fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    /// The clock this period was solved against.
    pub const fn clock_source(&self) -> ClockSel {
        self.clock_source
    }
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// Config
pub struct Config {
    /// I2C clock source.
    pub(crate) clock_source: ClockSel,

    /// I2C clock divider.
    pub clock_div: ClockDiv,

    /// A timing solved ahead of time, skipping the divisions on the device.
    ///
    /// Build one with [`Timing::solve`] in a `const`. When set, [`Self::bus_speed`],
    /// [`Self::clock_div`] and the clock source all come from it rather than being picked from the
    /// bus speed.
    pub timing: Option<Timing>,

    /// If true: invert SDA pin signal values (V<sub>DD</sub> = 0/mark, Gnd = 1/idle).
    pub invert_sda: bool,

    /// If true: invert SCL pin signal values (V<sub>DD</sub> = 0/mark, Gnd = 1/idle).
    pub invert_scl: bool,

    /// Set the pull configuration for the SDA pin.
    pub sda_pull: Pull,

    /// Set the pull configuration for the SCL pin.
    pub scl_pull: Pull,

    /// Set the pull configuration for the SCL pin.
    pub bus_speed: BusSpeed,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_source: ClockSel::MfClk,
            clock_div: ClockDiv::DivBy1,
            timing: None,
            invert_sda: false,
            invert_scl: false,
            sda_pull: Pull::None,
            scl_pull: Pull::None,
            bus_speed: BusSpeed::Standard,
        }
    }
}

impl Config {
    /// Use a timing solved ahead of time, skipping the divisions on the device.
    ///
    /// Sets the clock source as well: the timing carries the clock it was solved against, and a period
    /// programmed against a different one puts the bus at the wrong speed.
    pub const fn with_timing(mut self, timing: Timing) -> Self {
        self.clock_source = timing.clock_source;
        self.clock_div = timing.clock_div;
        self.timing = Some(timing);
        self
    }

    pub fn sda_pf(&self) -> PfType {
        PfType::input(self.sda_pull, self.invert_sda)
    }
    pub fn scl_pf(&self) -> PfType {
        PfType::input(self.scl_pull, self.invert_scl)
    }
    /// Derive everything the driver needs from this configuration, in one place.
    ///
    /// This is deliberately the *only* place the I2C setup path divides. Cortex-M0+ has no divide
    /// instruction, so each division site that survives optimization drags in a ~400 byte software
    /// divider; funnelling them here means a pre-solved [`Timing`] removes every one of them.
    pub(crate) fn resolve(&self) -> Result<Resolved, ConfigError> {
        let clocks = crate::sysctl::clocks();

        // A pre-solved timing already carries its clock source, the divider, the resulting rate and the
        // timer period, so nothing below needs computing.
        if let Some(timing) = self.timing {
            return Ok(Resolved {
                clock_source: timing.clock_source(),
                clock_div: timing.clock_div,
                clock_hz: timing.clock_hz(),
                source_hz: timing.clock_source().frequency(&clocks),
                tpr: timing.tpr(),
            });
        }

        let divider = self.clock_div.divider();
        let bus_speed = self.bus_speed.hertz();

        // Pick the source from the bus speed: at or below 200 kHz MFCLK suffices, above it the bus
        // clock is needed.
        let clock_source = if bus_speed / divider > 200_000 {
            // TODO: check if BUSCLK enabled
            ClockSel::BusClk
        } else {
            if !pac::SYSCTL.mclkcfg().read().usemftick() {
                return Err(ConfigError::ClockSourceNotEnabled);
            }

            ClockSel::MfClk
        };

        let source_hz = clock_source.frequency(&clocks);
        let clock_hz = source_hz / divider;

        // The source must be ~20x the bus speed.
        if clock_hz < (bus_speed / divider) * 20 {
            return Err(ConfigError::InvalidClockRate);
        }

        // Sets the timer period to bring the clock frequency to the selected I2C speed.
        // From the documentation: TPR = (I2C_CLK / (I2C_FREQ * (SCL_LP + SCL_HP))) - 1 where:
        // - I2C_FREQ is desired I2C frequency (= I2C_BASE_FREQ divided by I2C_DIV)
        // - TPR is the Timer Period register value (range of 1 to 127)
        // - SCL_LP is the SCL Low period (fixed at 6)
        // - SCL_HP is the SCL High period (fixed at 4)
        // - I2C_CLK is functional clock frequency
        let ticks = clock_hz / (bus_speed * 10);
        if ticks == 0 || ticks > 128 {
            return Err(ConfigError::InvalidClockRate);
        }

        Ok(Resolved {
            clock_source,
            clock_div: self.clock_div,
            clock_hz,
            source_hz,
            tpr: (ticks - 1) as u8,
        })
    }

    /// Check the config.
    ///
    /// Make sure that configuration is valid and enabled by the system, writing back the clock
    /// source that was chosen for the requested bus speed.
    pub fn check_config(&mut self) -> Result<(), ConfigError> {
        let resolved = self.resolve()?;
        self.clock_source = resolved.clock_source;

        Ok(())
    }
}

/// A [`Config`] with everything the driver needs derived from it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct Resolved {
    pub clock_source: ClockSel,
    pub clock_div: ClockDiv,

    /// Rate the peripheral sees after its own divider.
    pub clock_hz: u32,

    /// Rate of the source before the divider, which is what deep-sleep survival depends on.
    pub source_hz: u32,

    /// The timer period register value.
    pub tpr: u8,
}

impl Resolved {
    /// Shallowest sleep level to block so this instance keeps working.
    pub(crate) fn wake_floor(&self, sleep: &SleepInfo) -> Option<SleepLevel> {
        // Undivided on purpose: the question is whether the source still runs at the rate the
        // peripheral was configured for, not what it was divided down to.
        sleep.floor_for_operation(self.source_hz)
    }
}

/// Serial error
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,

    /// Arbitration lost
    Arbitration,

    /// ACK not received (either to the address or to a data byte)
    Nack,

    /// Timeout
    Timeout,

    /// CRC error
    Crc,

    /// Overrun error
    Overrun,

    /// Zero-length transfers are not allowed.
    ZeroLengthTransfer,

    /// Transfer length is over limit.
    TransferLengthIsOverLimit,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::Bus => "Bus Error",
            Self::Arbitration => "Arbitration Lost",
            Self::Nack => "ACK Not Received",
            Self::Timeout => "Request Timed Out",
            Self::Crc => "CRC Mismatch",
            Self::Overrun => "Buffer Overrun",
            Self::ZeroLengthTransfer => "Zero-Length Transfers are not allowed",
            Self::TransferLengthIsOverLimit => "Transfer length is over limit",
        };

        write!(f, "{}", message)
    }
}

impl core::error::Error for Error {}

/// I2C Driver.
pub struct I2c<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    scl: Option<Peri<'d, AnyPin>>,
    sda: Option<Peri<'d, AnyPin>>,
    wake_floor: Option<SleepLevel>,
    /// CPU cycles to let a freshly started transfer settle. See [`I2c::settle_after_start`].
    settle_cycles: u32,
    _phantom: PhantomData<M>,
}

impl<'d, M: Mode> SetConfig for I2c<'d, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(*config)
    }
}

impl<'d> I2c<'d, Blocking> {
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        mut config: Config,
    ) -> Result<Self, ConfigError> {
        if let Err(err) = config.check_config() {
            return Err(err);
        }

        Self::new_inner(peri, scl, sda, config)
    }
}

impl<'d> I2c<'d, Async> {
    pub fn new_async<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        mut config: Config,
    ) -> Result<Self, ConfigError> {
        if let Err(err) = config.check_config() {
            return Err(err);
        }

        let i2c = Self::new_inner(peri, scl, sda, config);

        T::info().interrupt.unpend();
        unsafe { T::info().interrupt.enable() };

        i2c
    }
}

impl<'d, M: Mode> I2c<'d, M> {
    /// Reconfigure the driver
    pub fn set_config(&mut self, config: Config) -> Result<(), ConfigError> {
        let resolved = config.resolve()?;

        self.info.interrupt.disable();

        if let Some(ref sda) = self.sda {
            sda.update_pf(config.sda_pf());
        }

        if let Some(ref scl) = self.scl {
            scl.update_pf(config.scl_pf());
        }

        self.init(&resolved)
    }

    fn init(&mut self, resolved: &Resolved) -> Result<(), ConfigError> {
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

        self.state.clock.store(resolved.clock_hz, Ordering::Relaxed);

        self.wake_floor = resolved.wake_floor(&self.info.sleep);

        // `I2C_ERR_13`: `CSR` is not valid for three functional clock cycles after a transfer is started,
        // so the status has to be left alone for that long. Rounded up, and at least one cycle so a
        // functional clock faster than the CPU still waits.
        //
        // Scaling with the clock is what makes this bite on MFCLK and not on the bus clock: at 4 MHz
        // against a 32 MHz CPU it is 24 cycles, where at 32 MHz it is 3 and the register read alone
        // covers it.
        let cpu_hz = crate::sysctl::clocks().mclk;
        self.settle_cycles = (3 * cpu_hz).div_ceil(resolved.clock_hz.max(1)).max(1);

        self.info.regs.controller(0).ctpr().write(|w| w.set_tpr(resolved.tpr));

        // Set Tx Fifo threshold, follow TI example
        self.info
            .regs
            .controller(0)
            .cfifoctl()
            .write(|w| w.set_txtrig(vals::CfifoctlTxtrig::Empty));
        // Set Rx Fifo threshold, follow TI example
        self.info
            .regs
            .controller(0)
            .cfifoctl()
            .write(|w| w.set_rxtrig(vals::CfifoctlRxtrig::Level1));
        // Enable controller clock stretching, follow TI example

        self.info.regs.controller(0).ccr().modify(|w| {
            w.set_clkstretch(true);
            w.set_active(true);
        });

        Ok(())
    }

    /// Discard whatever an abandoned transfer left queued, driverlib's `DL_I2C_flushController*FIFO`.
    ///
    /// A cancelled write leaves its unsent bytes in the TX FIFO and a cancelled read leaves what it
    /// received in the RX FIFO. Left there, the next transfer transmits the previous one's byte and reads
    /// back the previous one's data — an error reported against a transfer that succeeded, one
    /// transaction later.
    ///
    /// Only safe once the burst has ended; flushing under a running one takes bytes out from under it.
    /// Wait out `I2C_ERR_13` before reading `CSR` after starting a transfer.
    ///
    /// Polling `BUSY` any sooner reads it before the controller has raised it, so the wait falls straight
    /// through and the caller checks for errors against a transfer that has not happened yet. A NACK then
    /// goes unnoticed and the transfer is reported as a success.
    fn settle_after_start(&self) {
        cortex_m::asm::delay(self.settle_cycles);
    }

    /// Wait for whoever holds the bus to release it.
    ///
    /// Waits on the STOP that ends the transfer holding it, rather than spinning on `BUSBSY`: the bus can
    /// be held by another controller for as long as it likes, and an async caller must not block the
    /// executor for that.
    async fn wait_bus_free(&mut self) -> Result<(), Error> {
        if !self.info.regs.controller(0).csr().read().busbsy() {
            return Ok(());
        }

        // Dropping this future part-way has to leave `CSTOP` masked. Left armed it fires into a handler
        // that only wakes, with no future to consume it, and re-enters until something else masks it.
        let regs = self.info.regs;
        let _disarm = OnDrop::new(|| regs.cpu_int(0).imask().modify(|w| w.set_cstop(false)));

        future::poll_fn(|cx| {
            self.state.waker.register(cx.waker());

            self.info.regs.cpu_int(0).iclr().write(|w| w.set_cstop(true));
            self.info.regs.cpu_int(0).imask().modify(|w| w.set_cstop(true));

            // Checked after arming, so a STOP that arrives in between is caught here rather than waited
            // on forever.
            if self.info.regs.controller(0).csr().read().busbsy() {
                return Poll::Pending;
            }

            Poll::Ready(Ok(()))
        })
        .await
    }

    fn master_stop(&mut self) {
        // not the first transaction, delay 1000 cycles
        cortex_m::asm::delay(1000);

        // Stop transaction
        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_cblen(0);
            w.set_stop(true);
            w.set_start(false);
        });
    }

    fn master_continue(&mut self, length: usize, send_ack_nack: bool, send_stop: bool) -> Result<(), Error> {
        // delay between ongoing transactions, 1000 cycles
        cortex_m::asm::delay(1000);

        // Update transaction to length amount of bytes
        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_cblen(length as u16);
            w.set_start(false);
            w.set_ack(send_ack_nack);
            w.set_stop(send_stop);
        });

        Ok(())
    }

    fn master_read(
        &mut self,
        address: u8,
        length: usize,
        restart: bool,
        send_ack_nack: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        if restart {
            // not the first transaction, delay 1000 cycles
            cortex_m::asm::delay(1000);
        }

        // Set START and prepare to receive bytes into
        // `buffer`. The START bit can be set even if the bus
        // is BUSY or I2C is in slave mode.
        self.info.regs.controller(0).csa().modify(|w| {
            w.set_taddr(address as u16);
            w.set_cmode(vals::Mode::Mode7);
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

    fn master_write(&mut self, address: u8, length: usize, send_stop: bool) -> Result<(), Error> {
        // Start transfer of length amount of bytes
        self.info.regs.controller(0).csa().modify(|w| {
            w.set_taddr(address as u16);
            w.set_cmode(vals::Mode::Mode7);
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

    fn check_error(&self) -> Result<(), Error> {
        let csr = self.info.regs.controller(0).csr().read();
        if csr.err() {
            return Err(Error::Nack);
        } else if csr.arblst() {
            return Err(Error::Arbitration);
        }
        Ok(())
    }

    /// Flush both controller FIFOs.
    ///
    /// A flush is only legal while the controller is IDLE (TRM §25.2.3.12), so wait for
    /// that first.
    fn flush_fifos(&mut self) {
        let regs = self.info.regs.controller(0);

        while !regs.csr().read().idle() {}

        regs.cfifoctl().modify(|w| w.set_txflush(true));
        while (regs.cfifosr().read().txfifocnt() as usize) < self.info.fifo_size {}
        regs.cfifoctl().modify(|w| w.set_txflush(false));

        regs.cfifoctl().modify(|w| w.set_rxflush(true));
        while regs.cfifosr().read().rxfifocnt() != 0 {}
        regs.cfifoctl().modify(|w| w.set_rxflush(false));
    }
}

impl<'d> I2c<'d, Blocking> {
    fn master_blocking_continue(&mut self, length: usize, send_ack_nack: bool, send_stop: bool) -> Result<(), Error> {
        // Perform transaction
        self.master_continue(length, send_ack_nack, send_stop)?;

        self.settle_after_start();

        // Poll until the Controller process all bytes or NACK
        while self.info.regs.controller(0).csr().read().busy() {}

        Ok(())
    }

    fn master_blocking_read(
        &mut self,
        address: u8,
        length: usize,
        restart: bool,
        send_ack_nack: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        // unless restart, Wait for the controller to be idle,
        if !restart {
            while !self.info.regs.controller(0).csr().read().idle() {}
        }

        self.master_read(address, length, restart, send_ack_nack, send_stop)?;

        self.settle_after_start();

        // Poll until the Controller process all bytes or NACK
        while self.info.regs.controller(0).csr().read().busy() {}

        Ok(())
    }

    fn master_blocking_write(&mut self, address: u8, length: usize, send_stop: bool) -> Result<(), Error> {
        // Wait for the controller to be idle
        while !self.info.regs.controller(0).csr().read().idle() {}

        // Perform writing
        self.master_write(address, length, send_stop)?;

        self.settle_after_start();

        // Poll until the Controller writes all bytes or NACK
        while self.info.regs.controller(0).csr().read().busy() {}

        Ok(())
    }

    fn read_blocking_internal(
        &mut self,
        address: u8,
        read: &mut [u8],
        restart: bool,
        end_w_stop: bool,
    ) -> Result<(), Error> {
        if read.is_empty() {
            return Err(Error::ZeroLengthTransfer);
        }
        if read.len() > self.info.fifo_size {
            return Err(Error::TransferLengthIsOverLimit);
        }

        let read_len = read.len();
        let mut bytes_to_read = read_len;
        for (number, chunk) in read.chunks_mut(self.info.fifo_size).enumerate() {
            bytes_to_read -= chunk.len();
            // if the current transaction is the last & end_w_stop, send stop
            let send_stop = bytes_to_read == 0 && end_w_stop;
            // if there are still bytes to read, send ACK
            let send_ack_nack = bytes_to_read != 0;

            if number == 0 {
                self.master_blocking_read(
                    address,
                    chunk.len().min(self.info.fifo_size),
                    restart,
                    send_ack_nack,
                    send_stop,
                )?
            } else {
                self.master_blocking_continue(chunk.len(), send_ack_nack, send_stop)?;
            }

            // check errors
            if let Err(err) = self.check_error() {
                self.master_stop();
                self.flush_fifos();
                return Err(err);
            }

            for byte in chunk {
                *byte = self.info.regs.controller(0).crxdata().read().value();
            }
        }
        Ok(())
    }

    fn write_blocking_internal(&mut self, address: u8, write: &[u8], end_w_stop: bool) -> Result<(), Error> {
        if write.is_empty() {
            return Err(Error::ZeroLengthTransfer);
        }
        if write.len() > self.info.fifo_size {
            return Err(Error::TransferLengthIsOverLimit);
        }

        let mut bytes_to_send = write.len();
        for (number, chunk) in write.chunks(self.info.fifo_size).enumerate() {
            for byte in chunk {
                let ctrl0 = self.info.regs.controller(0).ctxdata();
                ctrl0.write(|w| w.set_value(*byte));
            }

            // if the current transaction is the last & end_w_stop, send stop
            bytes_to_send -= chunk.len();
            let send_stop = end_w_stop && bytes_to_send == 0;

            if number == 0 {
                self.master_blocking_write(address, chunk.len(), send_stop)?;
            } else {
                self.master_blocking_continue(chunk.len(), false, send_stop)?;
            }

            // check errors
            if let Err(err) = self.check_error() {
                self.master_stop();
                // A NACK leaves the bytes that were never sent queued (TRM §25.2.3.14);
                // flush them so they don't go out ahead of the next write.
                self.flush_fifos();
                return Err(err);
            }
        }
        Ok(())
    }

    // =========================
    //  Blocking public API

    /// Blocking read.
    pub fn blocking_read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Error> {
        // wait until bus is free
        while self.info.regs.controller(0).csr().read().busbsy() {}
        self.read_blocking_internal(address, read, false, true)
    }

    /// Blocking write.
    pub fn blocking_write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        // wait until bus is free
        while self.info.regs.controller(0).csr().read().busbsy() {}
        self.write_blocking_internal(address, write, true)
    }

    /// Blocking write, restart, read.
    pub fn blocking_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        // wait until bus is free
        while self.info.regs.controller(0).csr().read().busbsy() {}
        let err = self.write_blocking_internal(address, write, false);
        if err != Ok(()) {
            return err;
        }
        self.read_blocking_internal(address, read, true, true)
    }
}

impl<'d> I2c<'d, Async> {
    async fn write_async_internal(&mut self, addr: u8, write: &[u8], end_w_stop: bool) -> Result<(), Error> {
        let _guard = self.wake_floor.map(WakeGuard::new);

        let ctrl = self.info.regs.controller(0);

        let mut bytes_to_send = write.len();
        for (number, chunk) in write.chunks(self.info.fifo_size).enumerate() {
            self.info.regs.cpu_int(0).imask().modify(|w| {
                w.set_carblost(true);
                w.set_cnack(true);
                w.set_ctxdone(true);
            });

            for byte in chunk {
                ctrl.ctxdata().write(|w| w.set_value(*byte));
            }

            // if the current transaction is the last & end_w_stop, send stop
            bytes_to_send -= chunk.len();
            let send_stop = end_w_stop && bytes_to_send == 0;

            if number == 0 {
                self.master_write(addr, chunk.len(), send_stop)?;
            } else {
                self.master_continue(chunk.len(), false, send_stop)?;
            }

            let res: Result<(), Error> = future::poll_fn(|cx| {
                use crate::i2c::vals::CpuIntIidxStat;
                // Register prior to checking the condition
                self.state.waker.register(cx.waker());

                let result = match self.info.regs.cpu_int(0).iidx().read().stat() {
                    CpuIntIidxStat::NoIntr => Poll::Pending,
                    CpuIntIidxStat::Cnackfg => Poll::Ready(Err(Error::Nack)),
                    CpuIntIidxStat::Carblostfg => Poll::Ready(Err(Error::Arbitration)),
                    CpuIntIidxStat::Ctxdonefg => Poll::Ready(Ok(())),
                    _ => Poll::Pending,
                };

                if !result.is_pending() {
                    self.info
                        .regs
                        .cpu_int(0)
                        .imask()
                        .write_value(i2c::regs::CpuInt::default());
                }
                return result;
            })
            .await;

            if res.is_err() {
                self.master_stop();
                return res;
            }
        }
        Ok(())
    }

    async fn read_async_internal(
        &mut self,
        addr: u8,
        read: &mut [u8],
        restart: bool,
        end_w_stop: bool,
    ) -> Result<(), Error> {
        let _guard = self.wake_floor.map(WakeGuard::new);

        let read_len = read.len();

        let mut bytes_to_read = read_len;
        for (number, chunk) in read.chunks_mut(self.info.fifo_size).enumerate() {
            bytes_to_read -= chunk.len();
            // if the current transaction is the last & end_w_stop, send stop
            let send_stop = bytes_to_read == 0 && end_w_stop;
            // if there are still bytes to read, send ACK
            let send_ack_nack = bytes_to_read != 0;

            self.info.regs.cpu_int(0).imask().modify(|w| {
                w.set_carblost(true);
                w.set_cnack(true);
                w.set_crxdone(true);
            });

            if number == 0 {
                self.master_read(addr, chunk.len(), restart, send_ack_nack, send_stop)?
            } else {
                self.master_continue(chunk.len(), send_ack_nack, send_stop)?;
            }

            let res: Result<(), Error> = future::poll_fn(|cx| {
                use crate::i2c::vals::CpuIntIidxStat;
                // Register prior to checking the condition
                self.state.waker.register(cx.waker());

                let result = match self.info.regs.cpu_int(0).iidx().read().stat() {
                    CpuIntIidxStat::NoIntr => Poll::Pending,
                    CpuIntIidxStat::Cnackfg => Poll::Ready(Err(Error::Nack)),
                    CpuIntIidxStat::Carblostfg => Poll::Ready(Err(Error::Arbitration)),
                    CpuIntIidxStat::Crxdonefg => Poll::Ready(Ok(())),
                    _ => Poll::Pending,
                };

                if !result.is_pending() {
                    self.info
                        .regs
                        .cpu_int(0)
                        .imask()
                        .write_value(i2c::regs::CpuInt::default());
                }
                return result;
            })
            .await;

            if res.is_err() {
                self.master_stop();
                return res;
            }

            for byte in chunk {
                *byte = self.info.regs.controller(0).crxdata().read().value();
            }
        }
        Ok(())
    }

    // =========================
    //  Async public API

    pub async fn async_write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        // wait until bus is free
        self.wait_bus_free().await?;
        self.write_async_internal(address, write, true).await
    }

    pub async fn async_read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Error> {
        // wait until bus is free
        self.wait_bus_free().await?;
        self.read_async_internal(address, read, false, true).await
    }

    pub async fn async_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        // wait until bus is free
        self.wait_bus_free().await?;

        let err = self.write_async_internal(address, write, false).await;
        if err != Ok(()) {
            return err;
        }
        self.read_async_internal(address, read, true, true).await
    }
}

impl<'d> embedded_hal_02::blocking::i2c::Read for I2c<'d, Blocking> {
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, buffer)
    }
}

impl<'d> embedded_hal_02::blocking::i2c::Write for I2c<'d, Blocking> {
    type Error = Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, bytes)
    }
}

impl<'d> embedded_hal_02::blocking::i2c::WriteRead for I2c<'d, Blocking> {
    type Error = Error;

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, bytes, buffer)
    }
}

impl<'d> embedded_hal_02::blocking::i2c::Transactional for I2c<'d, Blocking> {
    type Error = Error;

    fn exec(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_02::blocking::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // wait until bus is free
        while self.info.regs.controller(0).csr().read().busbsy() {}
        for i in 0..operations.len() {
            match &mut operations[i] {
                embedded_hal_02::blocking::i2c::Operation::Read(buf) => {
                    self.read_blocking_internal(address, buf, false, false)?
                }
                embedded_hal_02::blocking::i2c::Operation::Write(buf) => {
                    self.write_blocking_internal(address, buf, false)?
                }
            }
        }
        self.master_stop();
        Ok(())
    }
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match *self {
            Self::Bus => embedded_hal::i2c::ErrorKind::Bus,
            Self::Arbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Self::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown),
            Self::Timeout => embedded_hal::i2c::ErrorKind::Other,
            Self::Crc => embedded_hal::i2c::ErrorKind::Other,
            Self::Overrun => embedded_hal::i2c::ErrorKind::Overrun,
            Self::ZeroLengthTransfer => embedded_hal::i2c::ErrorKind::Other,
            Self::TransferLengthIsOverLimit => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<'d, M: Mode> embedded_hal::i2c::ErrorType for I2c<'d, M> {
    type Error = Error;
}

impl<'d> embedded_hal::i2c::I2c for I2c<'d, Blocking> {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, write)
    }

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // wait until bus is free
        while self.info.regs.controller(0).csr().read().busbsy() {}
        for i in 0..operations.len() {
            match &mut operations[i] {
                embedded_hal::i2c::Operation::Read(buf) => self.read_blocking_internal(address, buf, false, false)?,
                embedded_hal::i2c::Operation::Write(buf) => self.write_blocking_internal(address, buf, false)?,
            }
        }
        self.master_stop();
        Ok(())
    }
}

impl<'d> embedded_hal_async::i2c::I2c for I2c<'d, Async> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.async_read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.async_write(address, write).await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.async_write_read(address, write, read).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // wait until bus is free
        self.wait_bus_free().await?;
        for i in 0..operations.len() {
            match &mut operations[i] {
                embedded_hal::i2c::Operation::Read(buf) => self.read_async_internal(address, buf, false, false).await?,
                embedded_hal::i2c::Operation::Write(buf) => self.write_async_internal(address, buf, false).await?,
            }
        }
        self.master_stop();
        Ok(())
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _i2c: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    // Mask interrupts and wake any task waiting for this interrupt
    unsafe fn on_interrupt() {
        T::state().waker.wake();
    }
}

/// Peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// I2C `SDA` pin trait
pub trait SdaPin<T: Instance>: crate::gpio::Pin {
    /// Get the PF number needed to use this pin as `SDA`.
    fn pf_num(&self) -> u8;
}

/// I2C `SCL` pin trait
pub trait SclPin<T: Instance>: crate::gpio::Pin {
    /// Get the PF number needed to use this pin as `SCL`.
    fn pf_num(&self) -> u8;
}

// ==== IMPL types ====

pub(crate) struct Info {
    pub(crate) regs: Regs,
    pub(crate) interrupt: Interrupt,
    pub fifo_size: usize,
    pub(crate) sleep: SleepInfo,
}

pub(crate) struct State {
    /// The clock rate of the I2C. This might be configured.
    pub(crate) clock: AtomicU32,
    pub(crate) waker: AtomicWaker,
}

impl<'d, M: Mode> I2c<'d, M> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        // Init power for I2C
        T::info().regs.gprcm(0).rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(vals::ResetKey::Key);
        });

        T::info().regs.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });

        // init delay, 16 cycles
        cortex_m::asm::delay(16);

        // Init GPIO
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
            scl: scl_inner,
            sda: sda_inner,
            wake_floor: None,
            settle_cycles: 0,
            _phantom: PhantomData,
        };
        this.init(&config.resolve()?)?;

        Ok(this)
    }
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

macro_rules! impl_i2c_instance {
    ($instance: ident, $fifo_size: expr) => {
        // `Config::source_hz` can assume `BusClk` is ULPCLK because of this
        const _: () = core::assert!(
            matches!(
                <crate::peripherals::$instance as crate::sysctl::LowPowerInstance>::SLEEP.power_domain,
                crate::sysctl::PowerDomain::Pd0
            ),
            "this I2C instance is in PD1, so its bus clock is MCLK rather than ULPCLK"
        );

        impl crate::i2c::SealedInstance for crate::peripherals::$instance {
            fn info() -> &'static crate::i2c::Info {
                use crate::i2c::Info;
                use crate::interrupt::typelevel::Interrupt;

                const INFO: Info = Info {
                    regs: crate::pac::$instance,
                    interrupt: crate::interrupt::typelevel::$instance::IRQ,
                    fifo_size: $fifo_size,
                    sleep: <crate::peripherals::$instance as crate::sysctl::LowPowerInstance>::SLEEP,
                };
                &INFO
            }

            fn state() -> &'static crate::i2c::State {
                use crate::i2c::State;
                use crate::interrupt::typelevel::Interrupt;

                static STATE: State = State {
                    clock: core::sync::atomic::AtomicU32::new(0),
                    waker: embassy_sync::waitqueue::AtomicWaker::new(),
                };
                &STATE
            }
        }

        impl crate::i2c::Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

macro_rules! impl_i2c_sda_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::i2c::SdaPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

macro_rules! impl_i2c_scl_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::i2c::SclPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use crate::i2c::{ClockDiv, ClockSel, Timing};
    use crate::sysctl::Clocks;

    /// A tree with both sources at explicit rates, so the expected periods below do not depend on what
    /// this chip's SYSOSC happens to boot at.
    const CLOCKS: Clocks = Clocks {
        ulpclk: 32_000_000,
        mfclk: 4_000_000,
        ..Clocks::RESET
    };

    const BUSCLK: ClockSel = ClockSel::BusClk;
    const MFCLK: ClockSel = ClockSel::MfClk;

    const STANDARD: u32 = 100_000;
    const FAST_MODE: u32 = 400_000;
    const FAST_MODE_PLUS: u32 = 1_000_000;

    /// These are based on TI's reference calculation.
    #[test]
    fn ti_timer_period() {
        // 32 MHz / (400 kHz * 10) - 1
        core::assert_eq!(
            Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, FAST_MODE).map(|t| t.tpr()),
            Some(7)
        );

        // 16 MHz / (400 kHz * 10) - 1
        core::assert_eq!(
            Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy2, FAST_MODE).map(|t| t.tpr()),
            Some(3)
        );

        // 16 MHz / (100 kHz * 10) - 1
        core::assert_eq!(
            Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy2, STANDARD).map(|t| t.tpr()),
            Some(15)
        );
    }

    /// The divided source rate travels with the timing, so the driver never recomputes it.
    #[test]
    fn timing_carries_divided_rate() {
        let timing = Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy2, FAST_MODE).unwrap();

        core::assert_eq!(timing.clock_hz(), CLOCKS.ulpclk / 2);
    }

    /// The source must be at least 20x the bus speed.
    #[test]
    fn rejects_insufficient_headroom() {
        // 32 MHz against 400 kHz and 1 MHz both clear 20x.
        core::assert!(Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, FAST_MODE).is_some());
        core::assert!(Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, FAST_MODE_PLUS).is_some());

        // 4 MHz does not: 400 kHz needs 8 MHz and 1 MHz needs 20 MHz.
        core::assert!(Timing::solve(&CLOCKS, MFCLK, ClockDiv::DivBy1, FAST_MODE).is_none());
        core::assert!(Timing::solve(&CLOCKS, MFCLK, ClockDiv::DivBy1, FAST_MODE_PLUS).is_none());

        // 100 kHz off MFCLK does clear it.
        core::assert!(Timing::solve(&CLOCKS, MFCLK, ClockDiv::DivBy1, STANDARD).is_some());
    }

    /// A period the register cannot hold is refused rather than wrapping.
    #[test]
    fn rejects_unrepresentable_period() {
        // A very slow bus off a fast clock overflows TPR's 7 bits.
        core::assert!(Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, 1_000).is_none());

        // A zero bus speed cannot be divided by.
        core::assert!(Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, 0).is_none());
    }
}
