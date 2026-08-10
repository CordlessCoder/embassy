//! Checksum a known vector, so a wrong answer is visible without an instrument.
//!
//! `"123456789"` is the standard CRC check string. CRC-16-CCITT seeded `0xFFFF` gives `0x29B1`, and
//! CRC-32 ISO-3309 seeded `0xFFFFFFFF` gives `0xCBF43F26` once inverted — this prints both and says
//! whether each matched, so the pass is on the wire rather than in the reader's head.
//!
//! No wiring.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::crc::{Config, Crc, Polynomial};
use panic_halt as _;

/// The check value every CRC-16-CCITT implementation agrees on for `"123456789"`.
const CCITT_CHECK: u16 = 0x29B1;

/// The same for CRC-32 ISO-3309, after the final inversion the standard applies and the hardware
/// does not.
const CRC32_CHECK: u32 = 0xCBF4_3F26;

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
        if got == CCITT_CHECK {
            info!("crc16-ccitt: {=u16:#06x} ok", got);
        } else {
            error!("crc16-ccitt: {=u16:#06x}, want {=u16:#06x}", got, CCITT_CHECK);
            fails += 1;
        }

        // Fed as one word rather than four bytes. Same four bytes in the same order on a
        // little-endian core, so the running checksum should agree with the byte-at-a-time path.
        crc.reset(0xFFFF);
        crc.feed_u32(u32::from_le_bytes(*b"1234"));
        crc.feed_bytes(b"56789");

        let word_fed = crc.result() as u16;
        if word_fed == CCITT_CHECK {
            info!("crc16-ccitt, word-fed head: {=u16:#06x} ok", word_fed);
        } else {
            error!(
                "crc16-ccitt, word-fed head: {=u16:#06x}, want {=u16:#06x} — a word feed is not its bytes",
                word_fed, CCITT_CHECK
            );
            fails += 1;
        }
    }

    {
        let mut config = Config::default();
        config.polynomial = Polynomial::Crc32;

        let mut crc = Crc::new(p.CRC.reborrow(), config);

        crc.reset(0xFFFF_FFFF);
        crc.feed_bytes(b"123456789");

        // The hardware stops at the raw remainder; the standard's check value includes a final
        // inversion, which is the caller's to apply.
        let got = !crc.result();
        if got == CRC32_CHECK {
            info!("crc32: {=u32:#010x} ok", got);
        } else {
            error!("crc32: {=u32:#010x}, want {=u32:#010x}", got, CRC32_CHECK);
            fails += 1;
        }
    }

    if fails == 0 {
        info!("crc: all ok");
    } else {
        error!("crc: {} failed", fails);
    }
}
