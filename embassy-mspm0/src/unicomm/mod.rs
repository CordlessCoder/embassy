//! Unified communication module (UNICOMM).
//!
//! One peripheral that is configured into a UART, an SPI, an I2C controller or an I2C target. It
//! replaces the standalone `uart`, `i2c` and SPI peripherals on the devices that have it — those
//! modules do not exist on a UNICOMM chip — and those devices have nothing else, so on an MSPM0L2117
//! every serial bus is a UNICOMM instance.
//!
//! # An instance is not four peripherals
//!
//! **No instance implements all four modes**, and which it implements is fixed in silicon. On an
//! MSPM0L2117 `UC4` and `UC8` are UART or SPI, `UC6` is an I2C controller and nothing else, `UC7` an
//! I2C target, and `UC11` a UART. On an MSPM0G518x `UC0` and `UC1` do UART and both I2C roles but not
//! SPI, and `UC2` is SPI only.
//!
//! That is why each mode has a marker trait — [`UartInstance`], [`SpiInstance`],
//! [`I2cControllerInstance`], [`I2cTargetInstance`] — generated per instance from the device
//! metadata. Asking for a mode an instance does not have fails to compile rather than writing a mode
//! select the hardware ignores.
//!
//! # What this module is, so far
//!
//! The wrapper only: power, reset, and the mode select. **The four mode drivers do not exist yet**, so
//! nothing here yet drives a bus. [`Unicomm`] is what they will be built on, and is useful on its own
//! only to reach the registers through [`Unicomm::regs`].

#![macro_use]

use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};
use mspm0_metapac::unicomm::Unicomm as Regs;
use mspm0_metapac::unicomm::vals::{PwrenKey, ResetKey, Select};

use crate::interrupt;
use crate::sysctl::LowPowerInstance;

/// Which register map an instance is configured into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// A UART.
    Uart,

    /// An SPI controller or peripheral.
    Spi,

    /// An I2C controller.
    I2cController,

    /// An I2C target. TI's headers call this the peripheral mode.
    I2cTarget,
}

impl Mode {
    const fn select(self) -> Select {
        match self {
            Mode::Uart => Select::Uart,
            Mode::Spi => Select::Spi,
            Mode::I2cController => Select::I2cController,
            Mode::I2cTarget => Select::I2cTarget,
        }
    }
}

/// A UNICOMM instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + LowPowerInstance {
    /// Interrupt this instance raises, whichever mode it is in.
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// An instance that implements the UART register map.
#[allow(private_bounds)]
pub trait UartInstance: Instance + SealedUartInstance {}

/// An instance that implements the SPI register map.
#[allow(private_bounds)]
pub trait SpiInstance: Instance + SealedSpiInstance {}

/// An instance that implements the I2C controller register map.
#[allow(private_bounds)]
pub trait I2cControllerInstance: Instance + SealedI2cControllerInstance {}

/// An instance that implements the I2C target register map.
#[allow(private_bounds)]
pub trait I2cTargetInstance: Instance + SealedI2cTargetInstance {}

pub(crate) trait SealedInstance {
    fn regs() -> Regs;
}

pub(crate) trait SealedUartInstance {
    fn uart_regs() -> mspm0_metapac::unicommuart::Unicommuart;
}

pub(crate) trait SealedSpiInstance {
    fn spi_regs() -> mspm0_metapac::unicommspi::Unicommspi;
}

pub(crate) trait SealedI2cControllerInstance {
    fn i2c_controller_regs() -> mspm0_metapac::unicommi2cc::Unicommi2cc;
}

pub(crate) trait SealedI2cTargetInstance {
    fn i2c_target_regs() -> mspm0_metapac::unicommi2ct::Unicommi2ct;
}

/// The wrapper around a UNICOMM instance: power, reset, and which mode it is in.
///
/// A mode driver takes this rather than the peripheral, so the mode is selected once and cannot be
/// changed underneath it.
pub struct Unicomm<'d, T: Instance> {
    _instance: Peri<'d, T>,
    _mode: PhantomData<T>,
}

impl<'d, T: UartInstance> Unicomm<'d, T> {
    /// Power the instance up as a UART.
    ///
    /// Only compiles for an instance that implements the UART register map.
    pub fn new_uart(instance: Peri<'d, T>) -> Self {
        let this = Self::new(instance, Mode::Uart);

        // The mode's own interrupts, which the wrapper's reset does not reach.
        T::uart_regs().cpu_int(0).imask().write(|_| {});

        this
    }

    /// The active UART register map.
    #[cfg(feature = "unstable-pac")]
    #[inline]
    pub fn uart_regs(&self) -> mspm0_metapac::unicommuart::Unicommuart {
        T::uart_regs()
    }
}

impl<'d, T: SpiInstance> Unicomm<'d, T> {
    /// Power the instance up as an SPI.
    ///
    /// Only compiles for an instance that implements the SPI register map.
    pub fn new_spi(instance: Peri<'d, T>) -> Self {
        let this = Self::new(instance, Mode::Spi);

        // The mode's own interrupts, which the wrapper's reset does not reach.
        T::spi_regs().cpu_int(0).imask().write(|_| {});

        this
    }

    /// The active SPI register map.
    #[cfg(feature = "unstable-pac")]
    #[inline]
    pub fn spi_regs(&self) -> mspm0_metapac::unicommspi::Unicommspi {
        T::spi_regs()
    }
}

impl<'d, T: I2cControllerInstance> Unicomm<'d, T> {
    /// Power the instance up as an I2C controller.
    ///
    /// Only compiles for an instance that implements the I2C controller register map.
    pub fn new_i2c_controller(instance: Peri<'d, T>) -> Self {
        let this = Self::new(instance, Mode::I2cController);

        // The mode's own interrupts, which the wrapper's reset does not reach.
        T::i2c_controller_regs().cpu_int(0).imask().write(|_| {});

        this
    }

    /// The active I2C controller register map.
    #[cfg(feature = "unstable-pac")]
    #[inline]
    pub fn i2c_controller_regs(&self) -> mspm0_metapac::unicommi2cc::Unicommi2cc {
        T::i2c_controller_regs()
    }
}

impl<'d, T: I2cTargetInstance> Unicomm<'d, T> {
    /// Power the instance up as an I2C target.
    ///
    /// Only compiles for an instance that implements the I2C target register map.
    pub fn new_i2c_target(instance: Peri<'d, T>) -> Self {
        let this = Self::new(instance, Mode::I2cTarget);

        // The mode's own interrupts, which the wrapper's reset does not reach.
        T::i2c_target_regs().cpu_int(0).imask().write(|_| {});

        this
    }

    /// The active I2C target register map.
    #[cfg(feature = "unstable-pac")]
    #[inline]
    pub fn i2c_target_regs(&self) -> mspm0_metapac::unicommi2ct::Unicommi2ct {
        T::i2c_target_regs()
    }
}

impl<'d, T: Instance> Unicomm<'d, T> {
    fn new(instance: Peri<'d, T>, mode: Mode) -> Self {
        let r = T::regs();

        r.gprcm(0).rstctl().write(|w| {
            w.set_resetassert(true);
            w.set_resetstkyclr(true);
            w.set_key(ResetKey::Key);
        });

        r.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(PwrenKey::Key);
        });

        // An instance with one mode has nothing to select and ignores this register; writing it
        // anyway keeps the sequence the same for every instance.
        r.ipmode().write(|w| w.set_select(mode.select()));

        Self {
            _instance: instance,
            _mode: PhantomData,
        }
    }

    /// Wrapper registers of this instance, for what the mode drivers do not cover.
    #[inline]
    pub fn regs(&self) -> Regs {
        T::regs()
    }
}

impl<T: Instance> Drop for Unicomm<'_, T> {
    fn drop(&mut self) {
        T::regs().gprcm(0).pwren().write(|w| {
            w.set_enable(false);
            w.set_key(PwrenKey::Key);
        });
    }
}

macro_rules! impl_unicomm_instance {
    ($instance: ident) => {
        impl crate::unicomm::SealedInstance for crate::peripherals::$instance {
            #[inline]
            fn regs() -> mspm0_metapac::unicomm::Unicomm {
                crate::pac::$instance
            }
        }

        impl crate::unicomm::Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

macro_rules! impl_unicomm_uart {
    ($instance: ident, $regs: ident) => {
        impl crate::unicomm::SealedUartInstance for crate::peripherals::$instance {
            #[inline]
            fn uart_regs() -> mspm0_metapac::unicommuart::Unicommuart {
                crate::pac::$regs
            }
        }

        impl crate::unicomm::UartInstance for crate::peripherals::$instance {}
    };
}

macro_rules! impl_unicomm_spi {
    ($instance: ident, $regs: ident) => {
        impl crate::unicomm::SealedSpiInstance for crate::peripherals::$instance {
            #[inline]
            fn spi_regs() -> mspm0_metapac::unicommspi::Unicommspi {
                crate::pac::$regs
            }
        }

        impl crate::unicomm::SpiInstance for crate::peripherals::$instance {}
    };
}

macro_rules! impl_unicomm_i2c_controller {
    ($instance: ident, $regs: ident) => {
        impl crate::unicomm::SealedI2cControllerInstance for crate::peripherals::$instance {
            #[inline]
            fn i2c_controller_regs() -> mspm0_metapac::unicommi2cc::Unicommi2cc {
                crate::pac::$regs
            }
        }

        impl crate::unicomm::I2cControllerInstance for crate::peripherals::$instance {}
    };
}

macro_rules! impl_unicomm_i2c_target {
    ($instance: ident, $regs: ident) => {
        impl crate::unicomm::SealedI2cTargetInstance for crate::peripherals::$instance {
            #[inline]
            fn i2c_target_regs() -> mspm0_metapac::unicommi2ct::Unicommi2ct {
                crate::pac::$regs
            }
        }

        impl crate::unicomm::I2cTargetInstance for crate::peripherals::$instance {}
    };
}
