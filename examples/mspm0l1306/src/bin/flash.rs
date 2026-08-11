//! Erase, program and read back the last sector of flash.
//!
//! The last sector is used because nothing else is there — this binary is a few kilobytes and the
//! part has 64. Every step prints whether it passed, so a wrong answer is visible without an
//! instrument.
//!
//! Three of the checks are about things a plain memory read cannot tell you: that an erased word
//! reports blank and a programmed one does not, that a word reads back exactly the bytes it was
//! given, and that the driver refuses an offset or a length that is not a whole flash word.
//!
//! The last two phases reprogram a word that was not erased first. SLAU847 §6.3.3 makes them
//! different cases, and only the second is the dangerous one: re-writing *identical* data is masked
//! bit by bit and passes silently, while a write needing any bit to go 0 -> 1 cannot be done at all
//! and should be reported. Both results are printed rather than judged, because what the silicon
//! does decides whether the driver should enable `CMDCTL.DATAVEREN`.
//!
//! No wiring.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::flash::{self, Error, Flash};
use panic_halt as _;

/// The last sector, which holds nothing.
const SECTOR: u32 = flash::SIZE as u32 - flash::SECTOR_SIZE as u32;

/// A pattern with no byte repeated, so a byte-order fault shows up rather than cancelling out.
const PATTERN: [u8; 16] = [
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10,
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_mspm0::init(Default::default());

    let mut flash = Flash::new(p.FLASHCTL);
    let mut fails = 0;

    info!("flash: {} bytes, {} byte sectors", flash::SIZE, flash::SECTOR_SIZE);

    match flash.blocking_erase(SECTOR, SECTOR + flash::SECTOR_SIZE as u32) {
        Ok(()) => info!("erase: ok"),
        Err(error) => {
            error!("erase: {}", error);
            fails += 1;
        }
    }

    match flash.blocking_is_blank(SECTOR) {
        Ok(true) => info!("blank after erase: ok"),
        Ok(false) => {
            error!("blank after erase: reports programmed");
            fails += 1;
        }
        Err(error) => {
            error!("blank after erase: {}", error);
            fails += 1;
        }
    }

    match flash.blocking_write(SECTOR, &PATTERN) {
        Ok(()) => info!("write: ok"),
        Err(error) => {
            error!("write: {}", error);
            fails += 1;
        }
    }

    let mut read = [0u8; PATTERN.len()];
    match flash.blocking_read(SECTOR, &mut read) {
        Ok(()) if read == PATTERN => info!("read back: ok"),
        Ok(()) => {
            error!("read back: {=[u8]:#04x}, want {=[u8]:#04x}", read, PATTERN);
            fails += 1;
        }
        Err(error) => {
            error!("read back: {}", error);
            fails += 1;
        }
    }

    match flash.blocking_is_blank(SECTOR) {
        Ok(false) => info!("blank after write: ok"),
        Ok(true) => {
            error!("blank after write: still reports blank, so the write did not land");
            fails += 1;
        }
        Err(error) => {
            error!("blank after write: {}", error);
            fails += 1;
        }
    }

    // A word inside the erased sector that nothing has touched, to show the sector was erased as a
    // whole rather than only at its first word.
    match flash.blocking_is_blank(SECTOR + flash::SECTOR_SIZE as u32 - flash::WORD_SIZE as u32) {
        Ok(true) => info!("blank at sector end: ok"),
        Ok(false) => {
            error!("blank at sector end: reports programmed");
            fails += 1;
        }
        Err(error) => {
            error!("blank at sector end: {}", error);
            fails += 1;
        }
    }

    // The offset is a word past a boundary and the length is not a whole word; both are refused
    // before anything reaches the controller.
    for (what, result) in [
        ("unaligned offset", flash.blocking_write(SECTOR + 1, &PATTERN)),
        ("partial word", flash.blocking_write(SECTOR, &PATTERN[..4])),
        ("past the end", flash.blocking_write(flash::SIZE as u32, &PATTERN[..8])),
        ("unaligned erase", flash.blocking_erase(SECTOR + 8, SECTOR + 16)),
    ] {
        match result {
            Err(Error::NotAligned) | Err(Error::OutOfBounds) => info!("{}: refused, ok", what),
            Ok(()) => {
                error!("{}: accepted", what);
                fails += 1;
            }
            Err(error) => {
                error!("{}: {}, which is not the refusal expected", what, error);
                fails += 1;
            }
        }
    }

    // The benign case: the same word programmed again with the data it already holds. §6.3.3.1's bit
    // masking gives already-correct bits no pulses, so this is expected to pass — it still spends one
    // of the word line's writes before an erase is required.
    match flash.blocking_write(SECTOR, &PATTERN[..8]) {
        Ok(()) => info!("second write, identical data: accepted, as expected"),
        Err(error) => warn!("second write, identical data: {}", error),
    }

    // The dangerous case, and the one that decides `DATAVEREN`: every bit of this is a one, and the
    // word holds zeros, so it cannot be programmed without an erase. The controller should report it
    // rather than leave the word half written.
    match flash.blocking_write(SECTOR, &[0xFF; 8]) {
        Err(error) => info!("second write, needs 0 -> 1: refused with {}", error),
        Ok(()) => {
            warn!("second write, needs 0 -> 1: ACCEPTED — the word is now whatever the pulses managed");
            fails += 1;
        }
    }

    // What the word actually holds afterwards, which says whether anything was corrupted by the two
    // attempts above.
    let mut after = [0u8; 8];
    if flash.blocking_read(SECTOR, &mut after).is_ok() {
        info!("word after both attempts: {=[u8]:#04x}", after);
    }

    if fails == 0 {
        info!("flash: all ok");
    } else {
        error!("flash: {} failed", fails);
    }
}
