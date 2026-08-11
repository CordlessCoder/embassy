//! Checksum a known vector, so a wrong answer is visible without an instrument.
//!
//! `"123456789"` is the standard CRC check string, and every value below is that string's published
//! check value for a named variant. Which variant the peripheral computes depends on
//! [`Config::bit_reversed`]: without it the unreflected ones, with it the reflected ones, which are
//! most of the CRCs in use.
//!
//! No wiring.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::crc::{Config, Crc, Polynomial};
use panic_halt as _;

/// CRC-16/CCITT-FALSE: polynomial `0x1021`, seed `0xFFFF`, no reflection, no final inversion.
const CCITT_FALSE_CHECK: u16 = 0x29B1;

/// CRC-32, the zlib and Ethernet one, which is **reflected** — so it needs
/// [`Config::bit_reversed`]. The final inversion is the standard's and the hardware does not do it.
const CRC32_CHECK: u32 = 0xCBF4_3926;

/// CRC-32/BZIP2: the same polynomial, seed and inversion as above with the reflection left off, which
/// is what the default configuration computes.
const CRC32_BZIP2_CHECK: u32 = 0xFC89_1918;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_mspm0::init(Default::default());

    let mut fails = 0;

    {
        let mut config = Config::default();
        config.polynomial = Polynomial::Crc16Ccitt;

        let mut crc = Crc::new(p.CRC.reborrow(), config);

        crc.reset(0xFFFF);
        crc.feed_bytes(b"123456789");

        let got = crc.result() as u16;
        if got == CCITT_FALSE_CHECK {
            info!("crc16/ccitt-false: {=u16:#06x} ok", got);
        } else {
            error!(
                "crc16/ccitt-false: {=u16:#06x}, want {=u16:#06x}",
                got, CCITT_FALSE_CHECK
            );
            fails += 1;
        }

        // Fed as one word rather than four bytes. Same four bytes in the same order on a
        // little-endian core, so the running checksum should agree with the byte-at-a-time path.
        crc.reset(0xFFFF);
        crc.feed_u32(u32::from_le_bytes(*b"1234"));
        crc.feed_bytes(b"56789");

        let word_fed = crc.result() as u16;
        if word_fed == CCITT_FALSE_CHECK {
            info!("crc16/ccitt-false, word-fed head: {=u16:#06x} ok", word_fed);
        } else {
            error!(
                "crc16/ccitt-false, word-fed head: {=u16:#06x}, want {=u16:#06x} — a word feed is not its bytes",
                word_fed, CCITT_FALSE_CHECK
            );
            fails += 1;
        }
    }

    {
        // Reflected, which is what makes this the CRC-32 everything else means by the name. The first
        // version of this example left it off and asserted the reflected check value against the
        // unreflected computation, which is the fault the two cases below now separate.
        let mut config = Config::default();
        config.polynomial = Polynomial::Crc32;
        config.bit_reversed = true;

        let mut crc = Crc::new(p.CRC.reborrow(), config);

        crc.reset(0xFFFF_FFFF);
        crc.feed_bytes(b"123456789");

        // The hardware stops at the raw remainder; the standard's final inversion is the caller's.
        let got = !crc.result();
        if got == CRC32_CHECK {
            info!("crc32 (zlib): {=u32:#010x} ok", got);
        } else {
            error!("crc32 (zlib): {=u32:#010x}, want {=u32:#010x}", got, CRC32_CHECK);
            fails += 1;
        }
    }

    {
        // The same polynomial with the reflection left off, which is a named variant of its own and
        // is what the default configuration gives. Here to prove `bit_reversed` is what separates
        // them rather than something else having changed.
        let mut config = Config::default();
        config.polynomial = Polynomial::Crc32;

        let mut crc = Crc::new(p.CRC.reborrow(), config);

        crc.reset(0xFFFF_FFFF);
        crc.feed_bytes(b"123456789");

        let got = !crc.result();
        if got == CRC32_BZIP2_CHECK {
            info!("crc32/bzip2: {=u32:#010x} ok", got);
        } else {
            error!("crc32/bzip2: {=u32:#010x}, want {=u32:#010x}", got, CRC32_BZIP2_CHECK);
            fails += 1;
        }
    }

    if fails == 0 {
        info!("crc: all ok");
    } else {
        error!("crc: {} failed", fails);
    }
}
