//! Flash memory controller (FLASHCTL).
//!
//! Erase and program the MAIN flash region — the same memory the application runs from — through the
//! [`embedded_storage`] traits, which is what `sequential-storage`, `embassy-boot` and the rest of
//! that ecosystem are written against. Read the erased-flash section below before choosing one.
//!
//! ```rust,ignore
//! let mut flash = Flash::new(p.FLASHCTL);
//!
//! flash.blocking_erase(0xF000, 0xF400)?;
//! flash.blocking_write_words(0xF000, &[0x1111_1111, 0x1111_1111])?;
//! ```
//!
//! An offset is an address: MAIN starts at zero, so offset `0xF000` is address `0xF000`. Nothing
//! here reserves a region — erasing offset 0 erases the vector table — so a caller keeps its data
//! away from its code, usually with a linker symbol.
//!
//! # Erased flash does not read as `0xFF`
//!
//! An erased word returns nondeterministic data (SLAU847 §6.3.6), so a checksum, a magic number or a
//! run of `0xFF` cannot tell "erased" from "written". [`Flash::blocking_is_blank`] asks the
//! controller instead, which is the only way to get the answer.
//!
//! On a device with ECC the ECC bits are erased along with the data, so reading an erased word can
//! also raise an uncorrectable ECC error, which several families escalate to a reset.
//!
//! This is worth knowing before picking a key-value store. A log-structured one finds its free space
//! by reading ahead and recognising the erased pattern, and that is the read this part does not
//! promise. TI's own EEPROM emulation sidesteps it by programming a header word onto every record
//! and never inferring anything from an unprogrammed one.
//!
//! # One program per word per erase, and the driver enforces it
//!
//! A flash word is eight bytes and can be programmed once between erases, whatever the sector's
//! remaining lifetime. That is why [`embedded_storage::nor_flash::MultiwriteNorFlash`] is not
//! implemented: it promises the opposite.
//!
//! **Left to itself the controller does not report a write it cannot perform.** Measured: writing
//! all-ones over a programmed word returns success and changes nothing, so the data is lost with no
//! error anywhere. `CMDCTL.DATAVEREN` is what turns that into [`Error::NotErased`], and this driver
//! sets it — TI's own code never does, and its absence is why the failure is silent.
//!
//! Re-writing a word with the *same* data it already holds still succeeds, which is correct: no bit
//! has to change, and SLAU847 §6.3.3.1's masking gives it no pulses. It still spends one of the word
//! line's writes before an erase is required.
//!
//! # There is nothing to await
//!
//! An erase or program takes the flash bank away from the CPU for the duration — tens of
//! microseconds for a word, tens of milliseconds for a sector — and the core cannot fetch
//! instructions from a bank the controller owns. So no other task can run, and the driver is
//! blocking. The [`embedded_storage_async`] impls are here because the ecosystem asks for them; they
//! complete without ever yielding.
//!
//! The command itself runs from RAM, which is what makes this safe rather than merely slow.
//!
//! On the families carrying `FLASH_ERR_06` a read taken during a command comes back wrong instead of
//! waiting, so there interrupts are masked as well — for one word at a time while programming, and
//! for a whole sector erase, which is the long one.
//!
//! **Masking does not cover DMA.** The same erratum names a DMA read of flash, and `PRIMASK` has no
//! say over a channel already running. Stop any channel whose source is flash — a transmit buffer in
//! `.rodata` is the usual one — before erasing or programming.
//!
//! # What this deliberately does not do
//!
//! NONMAIN is not reachable from here. It holds the boot configuration, and a bad write to it leaves
//! a device that no longer starts and cannot be recovered over SWD.
//!
//! Bank and mass erase, multi-word programming, sub-word programming, the DATA region and manual ECC
//! are all absent. Each wants a device this has never run on, or an API above `embedded-storage`.

#![macro_use]

use embassy_hal_internal::{Peri, PeripheralType};
use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};
use mspm0_metapac::factoryregion::vals::Mainnumbanks;
use mspm0_metapac::flashctl::{Flashctl as Regs, regs, vals};

use crate::_generated::{FLASH_HAS_ECC, FLASH_SECTOR_SIZE, FLASH_SIZE, FLASH_WEPROTA_BITS, FLASH_WEPROTB_BITS};
use crate::pac;

/// Size of the MAIN flash region, in bytes.
pub const SIZE: usize = FLASH_SIZE as usize;

/// Erase granularity, in bytes. An erase covers whole sectors and starts on one.
pub const SECTOR_SIZE: usize = FLASH_SECTOR_SIZE as usize;

/// Program granularity, in bytes. A write covers whole flash words and starts on one.
///
/// Eight on every device. The G518x's controller can program two words in one command, which is a
/// wider command rather than a wider word — its datasheet still states a 64-bit flash word.
pub const WORD_SIZE: usize = 8;

/// Sectors covered by one `CMDWEPROTB` bit.
const SECTORS_PER_WEPROTB_BIT: u32 = 8;

/// Sectors covered by `CMDWEPROTA`, which protects them one bit each.
const WEPROTA_SECTORS: u32 = FLASH_WEPROTA_BITS as u32;

/// What went wrong.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The range is not inside the MAIN region.
    OutOfBounds,

    /// An offset or a length that is not a whole number of sectors, or of flash words.
    NotAligned,

    /// The sector is write protected, and not by anything this driver did.
    ///
    /// Static protection, from the boot configuration, is the usual cause; the dynamic protection
    /// the driver clears per operation cannot produce this.
    Protected,

    /// The controller could not finish within its pulse-count limit, so the word or sector did not
    /// reach the state asked for. From [`Flash::blocking_is_blank`] it means the word is not blank,
    /// which is an answer rather than a fault.
    Verify,

    /// The address is outside any region the command may touch.
    IllegalAddress,

    /// A bank was left in a mode other than read, so no program or erase can run.
    Mode,

    /// A program tried to return a stored zero to one, which only an erase can do.
    ///
    /// Reported because the driver sets `CMDCTL.DATAVEREN`. Without it the controller accepts such a
    /// write, performs none of it and says nothing.
    NotErased,

    /// The controller reported a failure it does not break down further.
    Other,
}

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Error::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Error::NotAligned => NorFlashErrorKind::NotAligned,
            _ => NorFlashErrorKind::Other,
        }
    }
}

/// Flash controller driver.
///
/// # The instance parameter costs nothing here, today
///
/// `T` would duplicate every method body per instance, but every supported device has exactly one of
/// these. A part with two would start paying, and
/// [`simple_pwm::SimplePwm`](crate::tim::simple_pwm::SimplePwm) has the measurements and the reason
/// erasing the parameter is not automatically the fix.
pub struct Flash<'d, T: Instance> {
    _peri: Peri<'d, T>,
}

impl<'d, T: Instance> Flash<'d, T> {
    /// Take the flash controller.
    ///
    /// Nothing is written and no protection is changed until an erase or a program asks for it.
    pub fn new(peri: Peri<'d, T>) -> Self {
        Self { _peri: peri }
    }

    /// Read `bytes` from `offset`.
    ///
    /// A plain memory read, with the bounds check `embedded-storage` requires. Reading a location
    /// that has been erased and not programmed gives nondeterministic data — see the module docs.
    pub fn blocking_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        check_range(offset, bytes.len())?;

        // SAFETY: the range is inside MAIN, which is mapped for reads at its own address for the
        // life of the program. `copy_nonoverlapping` rather than a slice, because the destination is
        // a caller's buffer that cannot alias flash.
        unsafe {
            core::ptr::copy_nonoverlapping(offset as *const u8, bytes.as_mut_ptr(), bytes.len());
        }

        Ok(())
    }

    /// Erase every sector from `from` up to `to`.
    ///
    /// Both have to be sector boundaries. The sectors are erased one at a time, so a failure part
    /// way through leaves the earlier ones erased.
    pub fn blocking_erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        if to < from {
            return Err(Error::OutOfBounds);
        }

        check_range(from, (to - from) as usize)?;

        if !is_aligned(from, SECTOR_SIZE) || !is_aligned(to, SECTOR_SIZE) {
            return Err(Error::NotAligned);
        }

        for address in (from..to).step_by(SECTOR_SIZE) {
            self.command(address, |r| {
                r.cmdtype().write(|w| {
                    w.set_command(vals::Command::Erase);
                    w.set_size(vals::Size::Sector);
                });
            })?;
        }

        Ok(())
    }

    /// Program `bytes` at `offset`, rebuilding each flash word from eight of them.
    ///
    /// **Private, and reachable only through the [`embedded_storage`] impls**, which speak bytes and
    /// cannot be changed. A flash word is two 32-bit registers, and reassembling one from bytes whose
    /// alignment is not visible costs eight byte loads and a chain of shifts, spilled across the
    /// stack because this core has too few registers to hold it. There is no reason to reach for that
    /// by hand, so there is no public entry point to it —
    /// [`blocking_write_words`](Self::blocking_write_words) is the one to use.
    fn write_bytes(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        check_range(offset, bytes.len())?;

        if !is_aligned(offset, WORD_SIZE) || !bytes.len().is_multiple_of(WORD_SIZE) {
            return Err(Error::NotAligned);
        }

        for (index, word) in bytes.chunks_exact(WORD_SIZE).enumerate() {
            let low = u32::from_le_bytes(word[..4].try_into().unwrap());
            let high = u32::from_le_bytes(word[4..].try_into().unwrap());

            self.program_word(offset + (index * WORD_SIZE) as u32, low, high)?;
        }

        Ok(())
    }

    /// Program whole flash words at `offset`, taking them as words.
    ///
    /// A flash word is [`WORD_SIZE`] bytes, so `words` is consumed in pairs and its length has to be
    /// even. The offset has to be word-aligned, and every word has to have been erased since it was
    /// last programmed.
    ///
    /// **This is the only way to program flash directly.** The [`embedded_storage`] impls take bytes
    /// because their traits do, and pay to rebuild each word from eight of them.
    pub fn blocking_write_words(&mut self, offset: u32, words: &[u32]) -> Result<(), Error> {
        const HALVES: usize = WORD_SIZE / size_of::<u32>();

        check_range(offset, size_of_val(words))?;

        if !is_aligned(offset, WORD_SIZE) || !words.len().is_multiple_of(HALVES) {
            return Err(Error::NotAligned);
        }

        for (index, halves) in words.chunks_exact(HALVES).enumerate() {
            self.program_word(offset + (index * WORD_SIZE) as u32, halves[0], halves[1])?;
        }

        Ok(())
    }

    /// Program one flash word, given its two halves.
    fn program_word(&mut self, address: u32, low: u32, high: u32) -> Result<(), Error> {
        self.command(address, |r| {
            r.cmdtype().write(|w| {
                w.set_command(vals::Command::Program);
                w.set_size(vals::Size::Oneword);
            });

            // Every byte of the word, and the ECC byte alongside it where there is one. Leaving
            // the ECC byte out is how a sub-word program avoids spending the word's one
            // programming pass, and it makes reading the word an ECC error until the rest
            // arrives -- so a whole-word write programs it.
            r.cmdbyten().write(|w| {
                for byte in 0..WORD_SIZE {
                    w.set_data(byte, true);
                }
                w.set_ecc(FLASH_HAS_ECC);
            });

            // The low half at the lower address, which is what the controller expects
            // (SLAU847 §6.3.3.3) and what makes a word read back as the bytes it was given.
            r.cmddata(0).write_value(low);
            r.cmddata(1).write_value(high);
        })
    }

    /// Whether the flash word at `offset` is still in its erased state.
    ///
    /// The only way to ask. An erased word does not read as any particular value, so the answer
    /// cannot be had from [`Flash::blocking_read`].
    ///
    /// A word programmed with all ones answers `true` as well, having nothing to distinguish it.
    ///
    /// **So this does not answer "has anything been stored here".** A sparsely populated image is
    /// mostly `0xFF`, and every one of those words reports blank — a freshly written region comes
    /// back as partly erased. Program a marker onto whatever is stored and test for the marker, which
    /// is what TI's own EEPROM emulation does. What this is for is confirming an erase, and reading
    /// back a region before programming it.
    pub fn blocking_is_blank(&mut self, offset: u32) -> Result<bool, Error> {
        check_range(offset, WORD_SIZE)?;

        if !is_aligned(offset, WORD_SIZE) {
            return Err(Error::NotAligned);
        }

        match self.command(offset, |r| {
            r.cmdtype().write(|w| {
                w.set_command(vals::Command::Blankverify);
                w.set_size(vals::Size::Oneword);
            });
        }) {
            Ok(()) => Ok(true),
            Err(Error::Verify) => Ok(false),
            Err(error) => Err(error),
        }
    }

    /// Run one command at `address`, having `configure` select it and load its operands.
    ///
    /// The order is the controller's rather than a preference: clearing the status also restores
    /// full write protection, so the unprotect has to follow it, and both have to precede the
    /// execute.
    fn command(&mut self, address: u32, configure: impl FnOnce(Regs)) -> Result<(), Error> {
        let r = T::regs();

        // Only the two fields this command's correctness rests on, and read-modify-write rather than
        // a whole-register store.
        //
        // The TRM warns that the boot configuration routine may leave the command registers away
        // from their reset values, and these are the two that would matter: `ADDRXLATEOVR` decides
        // whether `CMDADDR` is a system address or a bank offset, so a stale one erases somewhere
        // else entirely, and `ECCGENOVR` decides whether the ECC byte is computed from the data or
        // taken from `CMDDATAECC`.
        //
        // Storing the whole register instead would mean claiming a reset value for the rest of it,
        // and `CMDCTL` is the register where that claim cannot be made. Four of its bits — the two
        // verify enables and the two mask disables — are described only by the L, C and H field
        // tables; the G table calls the same bits reserved, TI's own `hw_flashctl.h` and every SVD
        // omit them, and no TI code touches them. driverlib only ever read-modify-writes this
        // register, which is what leaves whatever those bits are worth undisturbed.
        r.cmdctl().modify(|w| {
            w.set_addrxlateovr(false);
            w.set_eccgenovr(false);

            // Reports a program that would need a stored zero to return to one, rather than
            // accepting it and doing nothing -- which is measurably what happens without it. Costs
            // no pulses and no word-line write, since the check is made before the operation runs,
            // and it does not refuse a legitimate first write: every other case in the flash example
            // still passes with it on.
            w.set_dataveren(true);
        });

        // The prefetcher would otherwise speculate into a bank the controller is about to take, and
        // the cache would hold lines the operation invalidates -- the TRM asks for a flush after a
        // program and offers no register for one, so turning the cache off across the operation and
        // restoring it after is the whole of what the device provides. It also covers the
        // FACTORYREGION read below, which TI's own accessors take with the cache off.
        let saved = pac::CPUSS.ctl().read();
        let mut suspended = saved;
        suspended.set_prefetch(false);
        suspended.set_icache(false);
        suspended.set_liten(false);
        pac::CPUSS.ctl().write_value(suspended);

        // CPU_ERR_02: the disable does not take effect while a flash access is pending, and what
        // completes one is a bus transaction to another slave rather than a barrier. Same sequence as
        // `prefetch::PrefetchSuspend::new`, which is the only other place that clears these bits.
        #[cfg(mspm0_shutdnstore)]
        let _ = pac::SYSCTL.shutdnstore(0).read();
        #[cfg(not(mspm0_shutdnstore))]
        let _ = pac::SYSCTL.clkstatus().read();

        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        clear_status(r);
        configure(r);
        r.cmdaddr().write_value(address);
        unprotect(r, address);

        let status = reserved(|| execute(r));

        pac::CPUSS.ctl().write_value(saved);

        if status.pass() {
            return Ok(());
        }

        Err(if status.failweprot() {
            Error::Protected
        } else if status.faililladdr() {
            Error::IllegalAddress
        } else if status.failmode() {
            Error::Mode
        } else if status.failinvdata() {
            Error::NotErased
        } else if status.failverify() {
            Error::Verify
        } else {
            Error::Other
        })
    }
}

/// Run `operation` with nothing else allowed to read the flash.
///
/// `FLASH_ERR_06`: on this device a read taken while the controller owns the bank returns incorrect
/// data rather than stalling for it, so an interrupt would fetch its vector and its handler out of
/// flash that is being written. Masking is the erratum's own workaround.
///
/// It does not cover DMA. A channel whose source is flash — a transmit buffer in `.rodata`, say —
/// reads the same wrong data, and `PRIMASK` has no say over it. Stop such a channel before erasing.
#[cfg(flash_err_06)]
fn reserved<R>(operation: impl FnOnce() -> R) -> R {
    critical_section::with(|_cs| operation())
}

/// Run `operation`, which needs nothing reserved on this device.
///
/// Reads to a bank under program or erase stall until it is released, so an interrupt taken here is
/// delayed and then runs correctly. Masking would buy nothing and would hold every other interrupt
/// off for the length of a sector erase.
#[cfg(not(flash_err_06))]
fn reserved<R>(operation: impl FnOnce() -> R) -> R {
    operation()
}

/// Start the command and wait for it to finish, from RAM.
///
/// The controller takes the flash bank for the duration, so this cannot run from the flash it is
/// operating on — TI's driverlib puts the same two instructions in RAM for the same reason, and its
/// EEPROM emulation calls no other variant. `.data` is loaded into RAM at startup and a function's
/// constant pool goes to the section the function is in, so nothing this touches is a flash fetch.
///
/// The wait is for `CMDDONE` and not for `CMDINPROGRESS` to clear: the in-progress bit takes a few
/// cycles to assert, and a fast core polling it first would read the command as finished before it
/// started.
#[inline(never)]
#[unsafe(link_section = ".data.ram_func")]
fn execute(r: Regs) -> regs::Statcmd {
    r.cmdexec().write(|w| w.set_val(true));

    loop {
        let status = r.statcmd().read();
        if status.done() {
            return status;
        }
    }
}

/// Clear the previous command's status, which also re-protects every sector.
///
/// This one waits on `CMDINPROGRESS` rather than on `CMDDONE`, unlike every other command here. It
/// touches status bits and never takes a bank, so there is no window in which the core could outrun
/// it and read the previous command's `CMDDONE` as this one's.
fn clear_status(r: Regs) {
    r.cmdtype().write(|w| w.set_command(vals::Command::Clearstatus));
    r.cmdexec().write(|w| w.set_val(true));

    while r.statcmd().read().inprogress() {}
}

/// Clear the dynamic write protection covering the sector `address` is in, and only that sector.
///
/// The registers reset to fully protected after every completed command, so this runs before each
/// one rather than once at startup.
///
/// Unprotecting one sector rather than all of MAIN is what makes a mistake here survivable: a wrong
/// sector number leaves the target protected, and the command comes back as [`Error::Protected`]
/// instead of erasing something the application is running from.
fn unprotect(r: Regs, address: u32) {
    let sector = address / FLASH_SECTOR_SIZE;

    // FLASHCTL describes its own geometry in GBLINFO0 and BANKINFO0, and those registers are not
    // what to read: nothing published states their value on any device, no TI code reads them, and
    // the IP instantiation parameters they come from overstate several devices -- an L122x is built
    // from a five-bank, 128-bit-word, three-protection-register configuration and is none of those
    // things. FACTORYREGION carries the device's own view, which is what driverlib's mask formulas
    // divide by.
    let sramflash = pac::FACTORYREGION.sramflash().read();
    let sectors = sramflash.mainflash_sz() as u32 * 1024 / FLASH_SECTOR_SIZE;

    let banks = sramflash.mainnumbanks();

    // Division by a runtime bank count would link a 32-bit divider into every binary that touches
    // flash, so the four possible counts are spelled out instead. Three of them fold to shifts.
    //
    // Three does not, and that is the whole reason this is written out: ARMv6-M has no `UMULL`, so
    // the optimiser cannot turn a divide by three into a reciprocal multiply and emits a call. It
    // is done by hand here. Exact for every sector count this register can describe -- `MAINFLASH_SZ`
    // is twelve bits of kilobytes, so the count cannot exceed `4095 * 1024 / SECTOR_SIZE`, and the
    // identity was checked against every value up to 65520. The product stays inside a `u32` until
    // 98303 and the identity itself holds until 131072.
    let sectors_per_bank = match banks {
        Mainnumbanks::Onebank => sectors,
        Mainnumbanks::Twobanks => sectors / 2,
        Mainnumbanks::Threebanks => (sectors * 0xAAAB) >> 17,
        Mainnumbanks::Fourbanks => sectors / 4,
    };

    // And the same for the remainder, which is at most three subtractions with a bank count of four.
    let mut sector_in_bank = sector;
    while sectors_per_bank > 0 && sector_in_bank >= sectors_per_bank {
        sector_in_bank -= sectors_per_bank;
    }

    if WEPROTA_SECTORS == 0 {
        // No CMDWEPROTA: CMDWEPROTB covers MAIN from its first sector, eight to a bit.
        let bit = (sector_in_bank / SECTORS_PER_WEPROTB_BIT) % 32;
        clear_weprotb(r, bit);
        return;
    }

    // CMDWEPROTA protects physical bank 0 a sector at a time. Where the banks are running swapped
    // the sector an address names is in the other half of the flash, so which register protects it
    // moves with it.
    let physical = physical_sector(sector, sectors);

    // Unreachable where `WEPROTA_SECTORS` is zero, because the guard above has already returned --
    // which is a device with no `CMDWEPROTA` at all, and clippy reads the comparison as always false
    // without following the early return that makes it moot.
    #[allow(clippy::absurd_extreme_comparisons)]
    if physical < WEPROTA_SECTORS {
        r.cmdweprota().modify(|w| w.0 &= !(1 << physical));
        return;
    }

    // Past CMDWEPROTA, CMDWEPROTB takes over eight sectors to a bit. On a single-bank part its first
    // bit sits above the sectors CMDWEPROTA already covers; on a multi-bank part it starts at each
    // bank's base, because CMDADDR is what says which bank.
    let bit = if matches!(banks, Mainnumbanks::Onebank) {
        (sector_in_bank - WEPROTA_SECTORS) / SECTORS_PER_WEPROTB_BIT
    } else {
        sector_in_bank / SECTORS_PER_WEPROTB_BIT
    };

    clear_weprotb(r, bit);
}

/// Clear one `CMDWEPROTB` bit.
///
/// A bit past the register's width would protect nothing and unprotect nothing, and the command
/// would come back as [`Error::Protected`] with no hint of where the arithmetic went wrong. The
/// assert names it in a debug build; the release build still fails safe.
fn clear_weprotb(r: Regs, bit: u32) {
    debug_assert!(
        bit < FLASH_WEPROTB_BITS as u32,
        "CMDWEPROTB has no such bit: the bank count or the sector number is wrong"
    );

    r.cmdweprotb().modify(|w| w.0 &= !(1 << bit));
}

/// Where a sector physically sits, which is where it sits logically unless the banks are swapped.
///
/// `CMDWEPROTA` protects physical bank 0, so a swap moves which of its sectors an address reaches.
#[cfg(mspm0_flash_bank_swap)]
fn physical_sector(sector: u32, sectors: u32) -> u32 {
    if !pac::SYSCTL.secstatus().read().flbankswp() {
        return sector;
    }

    let half = sectors / 2;

    if sector >= half { sector - half } else { sector + half }
}

/// The banks cannot be swapped on this device, so a sector is where it says it is.
#[cfg(not(mspm0_flash_bank_swap))]
fn physical_sector(sector: u32, _sectors: u32) -> u32 {
    sector
}

/// Whether `offset` starts on a boundary of `granularity`.
///
/// Both callers pass a constant, so the remainder folds to a mask. Passing a run-time granularity
/// would link a 32-bit divider on a core that has no divide instruction.
fn is_aligned(offset: u32, granularity: usize) -> bool {
    (offset as usize).is_multiple_of(granularity)
}

// The divide-by-three above is the one arm the optimiser cannot lower without a `UMULL` this core
// does not have, so it is a hand-written reciprocal and its correctness is worth proving rather than
// asserting. Every sector count reachable with the sector size this device is built for.
const _: () = {
    let mut sectors = 0u32;
    while sectors <= 4095 * 1024 / SECTOR_SIZE as u32 {
        core::assert!((sectors * 0xAAAB) >> 17 == sectors / 3);
        sectors += 1;
    }
};

/// Whether `len` bytes from `offset` are inside MAIN.
fn check_range(offset: u32, len: usize) -> Result<(), Error> {
    match (offset as usize).checked_add(len) {
        Some(end) if end <= SIZE => Ok(()),
        _ => Err(Error::OutOfBounds),
    }
}

impl<T: Instance> ErrorType for Flash<'_, T> {
    type Error = Error;
}

impl<T: Instance> ReadNorFlash for Flash<'_, T> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<T: Instance> NorFlash for Flash<'_, T> {
    const WRITE_SIZE: usize = WORD_SIZE;
    const ERASE_SIZE: usize = SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.blocking_erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(offset, bytes)
    }
}

impl<T: Instance> embedded_storage_async::nor_flash::ReadNorFlash for Flash<'_, T> {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<T: Instance> embedded_storage_async::nor_flash::NorFlash for Flash<'_, T> {
    const WRITE_SIZE: usize = WORD_SIZE;
    const ERASE_SIZE: usize = SECTOR_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.blocking_erase(from, to)
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(offset, bytes)
    }
}

#[allow(private_bounds)]
/// A flash controller instance.
pub trait Instance: SealedInstance + PeripheralType {}

pub(crate) trait SealedInstance {
    fn regs() -> Regs;
}

macro_rules! impl_flash_instance {
    ($instance:ident) => {
        impl crate::flash::SealedInstance for crate::peripherals::$instance {
            #[inline]
            fn regs() -> mspm0_metapac::flashctl::Flashctl {
                crate::pac::$instance
            }
        }

        impl crate::flash::Instance for crate::peripherals::$instance {}
    };
}
