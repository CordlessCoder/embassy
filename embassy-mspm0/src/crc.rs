//! Cyclic redundancy check (CRC).
//!
//! A hardware checksum generator. Seed it, feed it the data, read the result — each feed is a single
//! store and the engine keeps up with the bus, so a checksum over a block costs about what copying it
//! would.
//!
//! ```rust,ignore
//! let mut crc = Crc::new(p.CRC, Config::default());
//!
//! crc.reset(0xFFFF);
//! crc.feed_bytes(b"123456789");
//!
//! assert_eq!(crc.result() as u16, 0x29B1);
//! ```
//!
//! # Feed width is the width of the store
//!
//! The input register takes a byte, a half-word or a word depending on how it is written, and the
//! engine consumes exactly that much. [`Crc::feed_u8`], [`Crc::feed_u16`] and [`Crc::feed_u32`] are
//! the three, and they are not interchangeable — one word is not four bytes unless
//! [`Config::input_endianness`] matches the order the bytes are in.
//!
//! [`Crc::feed_bytes`] takes the safe route and feeds a byte at a time. **Feeding words is roughly
//! four times fewer stores**, so a caller with word-aligned data and a matching endianness should use
//! [`Crc::feed_words`] instead — but nothing here can check that pairing, which is why it is not done
//! automatically.
//!
//! # Which named CRC you get
//!
//! [`Config::bit_reversed`] is what selects between the reflected variants and the unreflected ones,
//! and it reflects the output as well as the input — the register name says only "bit reverse", so
//! this is measured rather than read. The final inversion several standards apply is the caller's:
//! the engine stops at the raw remainder.
//!
//! Check values for `"123456789"`, seeded all-ones, confirmed on silicon:
//!
//! | polynomial | `bit_reversed` | raw result | inverted |
//! |---|---|---|---|
//! | `0x1021` | `false` | `0x29B1` CRC-16/CCITT-FALSE | |
//! | `0x1021` | `true` | `0x6F91` CRC-16/MCRF4XX | `0x906E` CRC-16/X-25 |
//! | `0x04C11DB7` | `false` | `0x0376E6E7` CRC-32/MPEG-2 | `0xFC891918` CRC-32/BZIP2 |
//! | `0x04C11DB7` | `true` | | `0xCBF43926` CRC-32, the zlib and Ethernet one |
//!
//! So **the common CRC-32 needs `bit_reversed`**, and leaving it at its default gives a different
//! named variant rather than a wrong answer.
//!
//! [`Config::output_byteswap`] swaps the result's bytes and nothing else, for wire formats that want
//! it the other way round. [`Config::input_endianness`] has no effect on [`Crc::feed_bytes`], which
//! stores one byte at a time — it decides the order within a [`Crc::feed_u16`] or [`Crc::feed_u32`].
//!
//! # What differs per device
//!
//! Three register blocks, and they differ in what they can compute rather than only in layout:
//!
//! - the 16-bit block computes CRC-16-CCITT and nothing else;
//! - `v1` adds CRC-32 ISO-3309, selected by [`Config::polynomial`];
//! - `p` adds a polynomial of your own, `Polynomial::Custom16` and `Polynomial::Custom32`.
//!
//! A checksum a device cannot compute is not offered — it fails to build rather than quietly
//! computing something else. **Which block a chip has does not follow its family**, so gate on the
//! block and never on the part. A block the driver has not been taught also fails to build, rather
//! than being treated as one of the ones it knows.
//!
//! # `CRC_ERR_01`
//!
//! On the families it applies to, **DMA cannot be triggered to reach this peripheral in a suspended
//! low-power mode**. It does not affect anything this driver does — every feed here is a CPU store —
//! but a design that planned to checksum a buffer by DMA while the core sleeps does not work, and
//! nothing reports it. There is no workaround, so there is no code for it.

#![macro_use]

use embassy_hal_internal::{Peri, PeripheralType};
use mspm0_metapac::crc::{Crc as Regs, vals};

use crate::sysctl::LowPowerInstance;

/// Which checksum the generator computes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polynomial {
    /// CRC-16-CCITT, polynomial `0x1021`.
    Crc16Ccitt,

    /// CRC-32 ISO-3309, polynomial `0x04C11DB7`.
    ///
    /// Not on every device — the 16-bit block computes CRC-16 and nothing else.
    #[cfg(not(crc_16))]
    Crc32,

    /// A 16-bit polynomial of your own.
    ///
    /// Only where the block has a polynomial register.
    #[cfg(crc_p)]
    Custom16(u16),

    /// A 32-bit polynomial of your own.
    ///
    /// Only where the block has a polynomial register.
    #[cfg(crc_p)]
    Custom32(u32),
}

impl Polynomial {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::Crc16Ccitt;

    /// The `POLYSIZE` this needs, where the block has that field.
    #[cfg(not(crc_16))]
    const fn size(self) -> vals::Polysize {
        match self {
            Polynomial::Crc16Ccitt => vals::Polysize::Crc16,
            Polynomial::Crc32 => vals::Polysize::Crc32,
            #[cfg(crc_p)]
            Polynomial::Custom16(_) => vals::Polysize::Crc16,
            #[cfg(crc_p)]
            Polynomial::Custom32(_) => vals::Polysize::Crc32,
        }
    }

    /// The value for the `POLY` register, where the block has one.
    ///
    /// Written for the named checksums too, rather than trusting the register's reset value to be the
    /// standard polynomial.
    #[cfg(crc_p)]
    const fn value(self) -> u32 {
        match self {
            Polynomial::Crc16Ccitt => 0x1021,
            Polynomial::Crc32 => 0x04C1_1DB7,
            Polynomial::Custom16(poly) => poly as u32,
            Polynomial::Custom32(poly) => poly,
        }
    }

    /// Whether the result is 16 bits wide rather than 32.
    pub const fn is_16_bit(self) -> bool {
        match self {
            Polynomial::Crc16Ccitt => true,
            #[cfg(not(crc_16))]
            Polynomial::Crc32 => false,
            #[cfg(crc_p)]
            Polynomial::Custom16(_) => true,
            #[cfg(crc_p)]
            Polynomial::Custom32(_) => false,
        }
    }
}

impl Default for Polynomial {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Which end of a multi-byte input the engine consumes first.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Endianness {
    /// The least significant byte is at the lowest address and goes in first.
    ///
    /// This is what makes one [`Crc::feed_u32`] equal to its four bytes fed in address order on this
    /// core, which is little-endian.
    Little,

    /// The least significant byte is at the highest address and goes in last.
    Big,
}

impl Endianness {
    /// The default, as a `const` so a configuration's `new` can reach it.
    pub const DEFAULT: Self = Self::Little;
}

impl Default for Endianness {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// CRC configuration.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Which checksum to compute.
    pub polynomial: Polynomial,

    /// Feed each input bit-reversed, which several standard CRCs specify.
    pub bit_reversed: bool,

    /// Which end of a multi-byte input goes in first.
    pub input_endianness: Endianness,

    /// Byte-swap the result as it is read.
    pub output_byteswap: bool,
}

impl Config {
    /// The default configuration, usable in a `const`.
    ///
    /// [`Default`] delegates here. This type is `#[non_exhaustive]`, so a caller outside the crate
    /// cannot write the struct literal, and `Default::default` is not `const` — without this there
    /// is no way to build a CRC configuration in a `const` at all.
    pub const fn new() -> Self {
        Self {
            polynomial: Polynomial::DEFAULT,
            bit_reversed: false,
            input_endianness: Endianness::DEFAULT,
            output_byteswap: false,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// CRC driver.
///
/// Powered on construction and off on drop. The generator holds no state between [`Crc::reset`]
/// calls beyond the running checksum.
///
/// # The instance parameter costs nothing here, today
///
/// `T` would duplicate every method body per instance, but every supported device has exactly one of
/// these. A part with two would start paying, and
/// [`simple_pwm::SimplePwm`](crate::tim::simple_pwm::SimplePwm) has the measurements and the reason
/// erasing the parameter is not automatically the fix.
pub struct Crc<'d, T: Instance> {
    _peri: Peri<'d, T>,
}

impl<'d, T: Instance> Crc<'d, T> {
    /// Reset the peripheral, power it up and apply `config`.
    ///
    /// The checksum is undefined until [`Crc::reset`] seeds it.
    pub fn new(peri: Peri<'d, T>, config: Config) -> Self {
        let r = T::regs();

        r.gprcm(0).rstctl().write(|w| {
            w.set_resetassert(true);
            w.set_resetstkyclr(true);
            w.set_key(vals::ResetKey::Key);
        });

        r.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });

        #[cfg(crc_p)]
        r.poly().write_value(config.polynomial.value());

        r.ctrl().write(|w| {
            #[cfg(not(crc_16))]
            w.set_polysize(config.polynomial.size());

            w.set_bitreverse(config.bit_reversed);
            w.set_input_endianness(match config.input_endianness {
                Endianness::Little => vals::InputEndianness::LittleEndian,
                Endianness::Big => vals::InputEndianness::BigEndian,
            });
            w.set_output_byteswap(config.output_byteswap);
        });

        Self { _peri: peri }
    }

    /// Start a new checksum from `seed`.
    ///
    /// A 16-bit checksum ignores the upper half. Which seed a given standard wants is the caller's to
    /// know — CRC-16-CCITT as usually specified starts from `0xFFFF`.
    pub fn reset(&mut self, seed: u32) {
        T::regs().seed().write_value(seed);
    }

    /// Feed one byte.
    pub fn feed_u8(&mut self, data: u8) {
        // SAFETY: the input register consumes exactly as much as the store writes, so a byte feed has
        // to be a byte-wide store to it. The address is the peripheral's own and is aligned for any
        // width; the register is write-only, so nothing is read back and no other access is racing.
        unsafe { (T::regs().in_().as_ptr() as *mut u8).write_volatile(data) }
    }

    /// Feed one half-word, in the configured endianness.
    pub fn feed_u16(&mut self, data: u16) {
        // SAFETY: as `feed_u8`, at half-word width.
        unsafe { (T::regs().in_().as_ptr() as *mut u16).write_volatile(data) }
    }

    /// Feed one word, in the configured endianness.
    pub fn feed_u32(&mut self, data: u32) {
        T::regs().in_().write_value(data);
    }

    /// Feed every byte of `data`, in order.
    ///
    /// One store per byte. [`Crc::feed_words`] is about four times fewer for data that is already
    /// words.
    pub fn feed_bytes(&mut self, data: &[u8]) {
        for byte in data {
            self.feed_u8(*byte);
        }
    }

    /// Feed every word of `data`, in order and in the configured endianness.
    pub fn feed_words(&mut self, data: &[u32]) {
        for word in data {
            self.feed_u32(*word);
        }
    }

    /// The checksum of everything fed since the last [`Crc::reset`].
    ///
    /// A 16-bit polynomial leaves the upper half zero. Reading does not disturb the running value, so
    /// this can be called part-way through and fed more afterwards.
    pub fn result(&self) -> u32 {
        T::regs().out().read()
    }
}

impl<T: Instance> Drop for Crc<'_, T> {
    fn drop(&mut self) {
        T::regs().gprcm(0).pwren().write(|w| {
            w.set_enable(false);
            w.set_key(vals::PwrenKey::Key);
        });
    }
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + LowPowerInstance {}

pub(crate) trait SealedInstance {
    fn regs() -> Regs;
}

macro_rules! impl_crc_instance {
    ($instance:ident) => {
        // The driver holds no `WakeGuard`: the generator keeps its configuration and its running
        // checksum through any sleep the metadata says it survives. That is a fact about this device
        // rather than about the block, so it is checked rather than assumed -- a part that moves the
        // CRC somewhere it loses its configuration fails to build instead of silently resuming a
        // checksum that was reset underneath it.
        const _: () = {
            use crate::sysctl::LowPowerInstance;

            core::assert!(
                <crate::peripherals::$instance as LowPowerInstance>::SLEEP
                    .floor_to_keep_configured()
                    .is_none(),
                "CRC needs a WakeGuard on this device: its configuration does not survive deep sleep"
            );
        };

        impl crate::crc::SealedInstance for crate::peripherals::$instance {
            #[inline]
            fn regs() -> mspm0_metapac::crc::Crc {
                crate::pac::$instance
            }
        }

        impl crate::crc::Instance for crate::peripherals::$instance {}
    };
}
