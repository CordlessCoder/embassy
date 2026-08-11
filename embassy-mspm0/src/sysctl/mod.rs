//! System controller (SYSCTL) driver.

#![macro_use]

use core::cell::Cell;

use critical_section::{CriticalSection, Mutex};
use embassy_hal_internal::PeripheralType;

use crate::gpio::{AnyPin, PfType, Pin, Pull, SealedPin};
use crate::pac::sysctl::vals;
use crate::peripherals::CLK_OUT;
use crate::{Peri, pac};

mod clk_out_source;

pub mod clock;

pub use clk_out_source::ClkOutSource;
#[cfg(mspm0_ulpclk_div)]
pub use clock::UlpclkDiv;
pub use clock::{ClockError, Clocks, Config as ClockConfig, MclkSource, Sysosc};

/// Deep-sleep idle modes, ordered by increasing power saving.
///
/// Has no effect when the `low-power` feature is disabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SleepLevel {
    /// SYSOSC stays at full speed. Fastest wake, highest current.
    Stop0,

    /// SYSOSC limited to 4 MHz. Rounds up to [`SleepLevel::Stop2`] where the chip has no STOP1.
    Stop1,

    /// SYSOSC disabled; ULPCLK runs from LFCLK. Lowest STOP current.
    Stop2,

    /// Low-speed peripherals retained.
    Standby0,

    /// Only a few timers, named per chip, remain clocked. Lowest wake-capable current.
    Standby1,
}

impl SleepLevel {
    #[allow(unused)]
    pub(crate) const LEVELS: [SleepLevel; 5] = [
        SleepLevel::Stop0,
        SleepLevel::Stop1,
        SleepLevel::Stop2,
        SleepLevel::Standby0,
        SleepLevel::Standby1,
    ];

    /// The floor that leaves `mode` reachable, or `None` where nothing has to be blocked.
    ///
    /// Both ladders in [`SleepInfo`] are the same step: block the level *past* the deepest mode that
    /// still does whatever is being asked of the instance. Written as arithmetic over the two
    /// discriminants rather than as a match, because a match over eight modes lands out of line.
    #[inline(always)]
    const fn past(mode: Option<PowerMode>) -> Option<Self> {
        let Some(mode) = mode else { return None };

        // Nothing is past the two deepest, and RUN and SLEEP both step to `Stop0` rather than below it.
        if mode as u8 >= PowerMode::Standby1 as u8 {
            return None;
        }

        // RUN and SLEEP both step to `Stop0`; every deeper mode steps one rung down. `saturating_sub`
        // is 12 B smaller here than the equivalent `d - (d != 0)`, measured.
        Some(Self::LEVELS[(mode as u8).saturating_sub(1) as usize])
    }

    /// The more restrictive of two floors, where `None` restricts nothing.
    ///
    /// A floor names the shallowest blocked level, so blocking from a shallower level is stricter.
    pub const fn stricter(a: Option<Self>, b: Option<Self>) -> Option<Self> {
        match (a, b) {
            (Some(a), Some(b)) => Some(if (a as u8) <= (b as u8) { a } else { b }),
            (Some(only), None) | (None, Some(only)) => Some(only),
            (None, None) => None,
        }
    }
}

/// An operating mode, ordered from shallowest to deepest.
///
/// The sub-modes are the granularity the datasheets answer at, and the difference is load-bearing: a
/// windowed watchdog is usable in STANDBY0 and not in STANDBY1, which a single `Standby` could not say.
///
/// **RUN and SLEEP are deliberately not split.** RUN1/RUN2 and SLEEP1/SLEEP2 are clock-source policies
/// rather than depths — RUN2 runs the CPU with SYSOSC off and the *deeper* SLEEP0 turns it back on — so
/// a peripheral can be unusable in RUN2 and usable in SLEEP0, and no total order can express that.
///
/// Two things that look like gaps and are not: **`Stop2` never appears in the metadata**, because STOP2
/// disables SYSOSC so a peripheral either stops by STOP1 or runs from LFCLK and reaches STANDBY; and
/// **not every family has STOP1**, so `>= Stop1` is not a synonym for "deeper than STOP0".
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerMode {
    Run,
    Sleep,
    Stop0,
    Stop1,
    Stop2,
    Standby0,
    Standby1,

    /// Nothing but the `SHUTDNSTORE` bytes in SYSCTL survives this.
    Shutdown,
}

/// What deep sleep does to one peripheral instance, from the chip metadata.
///
/// Every field is a property of the instance on this chip, not of the peripheral kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SleepInfo {
    /// The domain this instance is in.
    pub power_domain: PowerDomain,

    /// Deepest mode through which the instance keeps its configuration registers, or `None` outside
    /// [`PowerDomain::Pd1`], where nothing disables it.
    pub retained_through: Option<PowerMode>,

    /// Deepest mode the datasheet says the instance may be *used* in.
    ///
    /// `None` where the datasheet table cannot answer, which is not the same as unusable.
    pub usable_through: Option<PowerMode>,

    /// Whether the instance has its own `CLKCFG.BLOCKASYNC` bit, or `None` where the family has no
    /// published SVD yet.
    ///
    /// `false` does not mean it cannot raise an asynchronous clock request, only that nothing but
    /// `SYSOSCCFG.BLOCKASYNCALL` masks it.
    pub block_async: Option<bool>,

    /// Whether this timer keeps being clocked in STANDBY1, making it able to wake the core from the
    /// deepest sleep. `None` for anything that is not a timer.
    pub clocked_in_standby1: Option<bool>,
}

impl SleepInfo {
    /// Shallowest level to block so the datasheet still supports operating this instance, or `None` to
    /// block nothing.
    ///
    /// An unknown [`Self::usable_through`] reads as no constraint, the datasheet tables being unable to
    /// resolve some instances.
    pub const fn floor_to_stay_usable(&self) -> Option<SleepLevel> {
        // An unknown `usable_through` and one reaching the bottom both come back `None`.
        SleepLevel::past(self.usable_through)
    }

    /// Shallowest level to block for the duration of an operation on this instance, clocked at
    /// `clock_hz`, or `None` to block nothing.
    ///
    /// The stricter of "does its clock survive" and "does the datasheet support using it there".
    pub const fn floor_for_operation(&self, clock_hz: u32) -> Option<SleepLevel> {
        SleepLevel::stricter(
            self.power_domain.floor_to_keep_running(clock_hz),
            self.floor_to_stay_usable(),
        )
    }

    /// Shallowest level to block so the instance is still set up on the other side, or `None` if deep
    /// sleep leaves its configuration intact.
    ///
    /// Being in PD1 is not on its own a reason to block: SYSCTL disables those peripherals on entry but
    /// re-enables them on exit, so only losing the configuration registers needs anything done about it.
    pub const fn floor_to_keep_configured(&self) -> Option<SleepLevel> {
        SleepLevel::past(self.retained_through)
    }
}

/// The power domain a peripheral instance belongs to.
///
/// Differs between chips of the same family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerDomain {
    /// Low-speed domain, clocked by ULPCLK. Powered in every mode but SHUTDOWN.
    Pd0,

    /// High-performance domain, clocked by MCLK. Powered only in RUN and SLEEP; SYSCTL forces its
    /// peripherals to a disabled state on deep-sleep entry.
    Pd1,

    /// Backup domain, powered from `VBAT` and clocked by LFCLK. Survives even SHUTDOWN.
    ///
    /// Only present on chips with an independent `VBAT` supply.
    Backup,
}

impl PowerDomain {
    /// Whether the domain stays powered through deep sleep.
    pub const fn is_powered_in_deep_sleep(self) -> bool {
        !matches!(self, Self::Pd1)
    }

    /// Shallowest level to block so an instance in this domain, clocked at `clock_hz`, keeps running,
    /// or `None` to block nothing.
    ///
    /// `clock_hz` is the undivided rate of the clock the peripheral depends on. Answers only for
    /// peripherals that must run *continuously* — work that is only started while asleep raises an
    /// asynchronous clock request instead, and must not block sleep on this.
    pub const fn floor_to_keep_running(self, clock_hz: u32) -> Option<SleepLevel> {
        // Per-mode clock ceilings, from the family TRMs' "DMA Operating Mode Support" and "Operating
        // Modes" sections. STANDBY0 clocks all PD0 peripherals from LFCLK; STANDBY1 does not.
        // Assumes RUN0, the only run mode the HAL configures: STOP0 reaches 4 MHz only from there.
        const STOP_HZ: u32 = 4_000_000;

        match self {
            // Disabled by SYSCTL on entry to any deep-sleep mode, whatever it is clocked at.
            Self::Pd1 => Some(SleepLevel::Stop0),

            // Powered from VBAT, so no mode reaches it.
            Self::Backup => None,

            Self::Pd0 => {
                if clock_hz > STOP_HZ {
                    // Needs MCLK
                    Some(SleepLevel::Stop0)
                } else if clock_hz > LFCLK_HZ {
                    // Requires MFCLK
                    Some(SleepLevel::Stop2)
                } else if clock_hz > 0 {
                    // LFCLK is enough, keep PD0 alive
                    Some(SleepLevel::Standby1)
                } else {
                    // No clock
                    None
                }
            }
        }
    }
}

// Boundary checks for `floor_to_keep_running`. `crate::fmt` cannot be used in const.
const _: () = {
    use PowerDomain::{Backup, Pd0, Pd1};

    core::assert!(matches!(Pd0.floor_to_keep_running(4_000_001), Some(SleepLevel::Stop0)));
    core::assert!(matches!(Pd0.floor_to_keep_running(4_000_000), Some(SleepLevel::Stop2)));
    core::assert!(matches!(Pd0.floor_to_keep_running(32_769), Some(SleepLevel::Stop2)));
    core::assert!(matches!(Pd0.floor_to_keep_running(32_768), Some(SleepLevel::Standby1)));
    core::assert!(matches!(Pd0.floor_to_keep_running(1), Some(SleepLevel::Standby1)));
    core::assert!(matches!(Pd0.floor_to_keep_running(0), None));

    // No clock rate makes PD1 survive, or stops the backup domain from surviving.
    core::assert!(matches!(Pd1.floor_to_keep_running(0), Some(SleepLevel::Stop0)));
    core::assert!(matches!(Pd1.floor_to_keep_running(32_000_000), Some(SleepLevel::Stop0)));
    core::assert!(matches!(Backup.floor_to_keep_running(32_000_000), None));

    core::assert!(Pd0.is_powered_in_deep_sleep());
    core::assert!(!Pd1.is_powered_in_deep_sleep());
    core::assert!(Backup.is_powered_in_deep_sleep());
};

// Boundary checks for `SleepLevel::stricter`, `floor_to_stay_usable` and `floor_for_operation`.
const _: () = {
    use SleepLevel::{Standby0, Standby1, Stop0, Stop1, Stop2};

    core::assert!(matches!(SleepLevel::stricter(None, None), None));
    core::assert!(matches!(SleepLevel::stricter(Some(Standby1), None), Some(Standby1)));
    core::assert!(matches!(SleepLevel::stricter(None, Some(Stop0)), Some(Stop0)));
    // Blocking from a shallower level is stricter, either way round.
    core::assert!(matches!(SleepLevel::stricter(Some(Standby0), Some(Stop2)), Some(Stop2)));
    core::assert!(matches!(SleepLevel::stricter(Some(Stop2), Some(Standby0)), Some(Stop2)));

    const fn usable(mode: Option<PowerMode>) -> SleepInfo {
        SleepInfo {
            power_domain: PowerDomain::Pd0,
            retained_through: None,
            usable_through: mode,
            block_async: None,
            clocked_in_standby1: None,
        }
    }

    // Every rung, because `SleepLevel::past` reads the step off the two discriminants rather than
    // matching each mode. Adding a `PowerMode` variant anywhere but the end breaks it here.
    core::assert!(matches!(usable(None).floor_to_stay_usable(), None));
    core::assert!(matches!(usable(Some(PowerMode::Shutdown)).floor_to_stay_usable(), None));
    core::assert!(matches!(usable(Some(PowerMode::Standby1)).floor_to_stay_usable(), None));
    core::assert!(matches!(
        usable(Some(PowerMode::Stop2)).floor_to_stay_usable(),
        Some(Standby0)
    ));
    core::assert!(matches!(
        usable(Some(PowerMode::Run)).floor_to_stay_usable(),
        Some(Stop0)
    ));

    // The one the sub-mode split exists for: usable in STANDBY0 blocks only the deeper STANDBY1, where
    // a single `Standby` value used to force this all the way down to blocking STANDBY0 as well.
    core::assert!(matches!(
        usable(Some(PowerMode::Standby0)).floor_to_stay_usable(),
        Some(Standby1)
    ));
    core::assert!(matches!(
        usable(Some(PowerMode::Stop1)).floor_to_stay_usable(),
        Some(Stop2)
    ));
    core::assert!(matches!(
        usable(Some(PowerMode::Stop0)).floor_to_stay_usable(),
        Some(Stop1)
    ));
    core::assert!(matches!(
        usable(Some(PowerMode::Sleep)).floor_to_stay_usable(),
        Some(Stop0)
    ));

    // An LFCLK-clocked instance the datasheet only supports to STOP1 takes the usability floor, not the
    // clock one; a fast-clocked instance usable to the bottom takes the clock floor instead.
    core::assert!(matches!(
        usable(Some(PowerMode::Stop1)).floor_for_operation(32_768),
        Some(Stop2)
    ));
    core::assert!(matches!(
        usable(Some(PowerMode::Standby1)).floor_for_operation(32_000_000),
        Some(Stop0)
    ));

    const fn retained(domain: PowerDomain, mode: Option<PowerMode>) -> SleepInfo {
        SleepInfo {
            power_domain: domain,
            retained_through: mode,
            usable_through: None,
            block_async: None,
            clocked_in_standby1: None,
        }
    }

    // Being in PD1 costs nothing by itself, only losing the configuration does.
    core::assert!(matches!(
        retained(PowerDomain::Pd1, Some(PowerMode::Standby1)).floor_to_keep_configured(),
        None
    ));
    core::assert!(matches!(
        retained(PowerDomain::Pd1, Some(PowerMode::Standby0)).floor_to_keep_configured(),
        Some(Standby1)
    ));
    core::assert!(matches!(
        retained(PowerDomain::Pd1, Some(PowerMode::Stop1)).floor_to_keep_configured(),
        Some(Stop2)
    ));
    core::assert!(matches!(
        retained(PowerDomain::Pd1, Some(PowerMode::Sleep)).floor_to_keep_configured(),
        Some(Stop0)
    ));
    core::assert!(matches!(
        retained(PowerDomain::Pd0, None).floor_to_keep_configured(),
        None
    ));
};

/// What deep sleep does to a peripheral instance.
///
/// Implemented for every peripheral singleton but GPIO pins, whose logic is in PD0 on every chip.
/// Type-erased drivers cannot name their instance and carry a [`SleepInfo`] in their `Info` instead.
pub trait LowPowerInstance: PeripheralType {
    /// How this instance behaves across deep sleep.
    const SLEEP: SleepInfo;
}

macro_rules! impl_low_power {
    ($instance:ident, $sleep:expr) => {
        impl crate::sysctl::LowPowerInstance for crate::peripherals::$instance {
            const SLEEP: crate::sysctl::SleepInfo = $sleep;
        }
    };
}

/// A token forbidding a deep-sleep mode, and anything deeper, while held.
///
/// Refcounted per level, and a no-op without the `low-power` feature, so drivers can hold one
/// unconditionally.
#[must_use]
pub struct WakeGuard {
    #[cfg(feature = "low-power")]
    level: SleepLevel,
    _unit: (),
}

impl WakeGuard {
    /// Forbid entering `level` or any deeper mode until dropped.
    ///
    /// [`SleepLevel::Stop0`] blocks all deep sleep, leaving only `WFI`.
    #[inline]
    pub fn new(level: SleepLevel) -> Self {
        #[cfg(not(feature = "low-power"))]
        let _ = level;
        #[cfg(feature = "low-power")]
        crate::low_power::block(level);

        Self {
            #[cfg(feature = "low-power")]
            level,
            _unit: (),
        }
    }
}

impl Drop for WakeGuard {
    #[inline]
    fn drop(&mut self) {
        #[cfg(feature = "low-power")]
        crate::low_power::unblock(self.level);
    }
}

/// A [`WakeGuard`] a driver may or may not be holding, costing nothing when the crate cannot sleep.
///
/// `Option<WakeGuard>` would be the obvious spelling and is **one byte** with `low-power` off:
/// `WakeGuard` is a zero-sized type there, so the option has no niche to put its discriminant in and
/// takes one of its own. Almost every driver holds one, several hold two, so that byte multiplies
/// across a chip with four kilobytes of RAM.
pub(crate) struct MaybeWakeGuard {
    #[cfg(feature = "low-power")]
    guard: Option<WakeGuard>,
}

impl MaybeWakeGuard {
    /// Hold a guard at `level`, or nothing if there is no level to hold.
    #[inline]
    pub(crate) fn new(level: Option<SleepLevel>) -> Self {
        #[cfg(not(feature = "low-power"))]
        let _ = level;

        Self {
            #[cfg(feature = "low-power")]
            guard: level.map(WakeGuard::new),
        }
    }

    /// Hold nothing.
    #[inline]
    pub(crate) const fn none() -> Self {
        Self {
            #[cfg(feature = "low-power")]
            guard: None,
        }
    }

    /// Drop whatever is held, without waiting for the owner to be dropped.
    #[inline]
    pub(crate) fn release(&mut self) {
        #[cfg(feature = "low-power")]
        drop(self.guard.take());
    }
}

/// The brown-out supervisor's threshold.
///
/// Needs the `bor-warning` feature: selecting a level is not free in the sleep path, so an
/// application that leaves the supervisor where it boots should not carry it.
///
/// `Bor0` is the reset threshold and is always the floor: a `BOR0-` violation resets the device
/// whatever this says. The other three sit *above* it and change what happens in the band between —
/// the supervisor raises an interrupt instead of resetting, which is an early warning that the
/// supply is sagging rather than a second reset level. SLAU847's fault table puts it plainly: a
/// `BOR0-` supply error generates a BOR, a `BOR1/2/3-` supply error generates a `BORLVL` interrupt.
///
/// The voltages are per device. The datasheet's supply-monitor table is keyed on these names.
///
/// # The warning is an NMI, and an unhandled one hangs
///
/// `BORLVL` arrives as a **non-maskable** interrupt in SYSCTL's NMI registers. Two consequences a
/// caller has to plan for:
///
/// - It is not held off by a critical section, so it can arrive in the middle of one.
/// - An application with no `NonMaskableInt` handler gets `cortex-m-rt`'s default, which is an
///   endless loop. **Arming a warning level without a handler turns a supply dip into a hang** —
///   and the device stays there until the supply falls far enough for `BOR0-` to reset it, or does
///   not.
///
/// Arm one only alongside a handler that does something useful with it: park the outputs, flush what
/// has to survive, and let the reset come.
///
/// **A warning level is one-shot.** When a `BOR1-`, `BOR2-` or `BOR3-` violation raises its
/// interrupt the supervisor drops itself back to `Bor0`, so that a further fall still resets the
/// device. Re-arming is another [`set_bor_threshold`] call, which is also what clears the violation.
///
/// The supervisor runs in RUN, SLEEP, STOP and STANDBY, and is disabled by SHUTDOWN.
///
/// # Errata
/// - `SYSCTL_ERR_11` (L110x/L13xx) — with the frequency correction loop enabled and the device in
///   RUN2 or SLEEP2, a warning level produces an unexpected BOR *reset* followed by the NMI. Both are
///   clock-tree choices the HAL fixes per binary rather than something a driver can guard, so it is
///   the caller's to avoid: do not combine a warning level with FCL in those modes.
///
#[cfg(feature = "bor-warning")]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BorThreshold {
    /// Reset on a `BOR0-` violation. The level every device boots at.
    #[default]
    Bor0,

    /// Interrupt on a `BOR1-` violation; reset still at `BOR0-`.
    Bor1,

    /// Interrupt on a `BOR2-` violation; reset still at `BOR0-`.
    Bor2,

    /// Interrupt on a `BOR3-` violation; reset still at `BOR0-`.
    Bor3,
}

#[cfg(feature = "bor-warning")]
impl BorThreshold {
    const fn to_bits(self) -> u8 {
        self as u8
    }

    const fn from_active(active: vals::Borcurthreshold) -> Self {
        match active {
            vals::Borcurthreshold::Borlevel1 => BorThreshold::Bor1,
            vals::Borcurthreshold::Borlevel2 => BorThreshold::Bor2,
            vals::Borcurthreshold::Borlevel3 => BorThreshold::Bor3,
            // `Bormin` and the reserved encodings. Reporting the reset level for one that should not
            // occur is the safe direction: it is what the hardware falls back to.
            _ => BorThreshold::Bor0,
        }
    }
}

/// The threshold the supervisor is enforcing now.
///
/// Not necessarily the one last asked for — a warning level disarms itself when it fires.
#[cfg(feature = "bor-warning")]
pub fn bor_threshold() -> BorThreshold {
    BorThreshold::from_active(pac::SYSCTL.sysstatus().read().borcurthreshold())
}

/// Ask the supervisor for `threshold`, and report whether it took.
///
/// The change is not immediate. SLAU847 §2.2.3.2 gives it about 15 us, **during which the supervisor
/// is blind to the supply**, so this waits that out and then reads back what became active. A
/// mismatch is returned rather than ignored: asking for a warning level while the supply is already
/// below it leaves the supervisor at [`BorThreshold::Bor0`], and a caller that assumed otherwise
/// would be waiting for a warning that cannot arrive.
///
/// This also clears any recorded violation, which is what re-arms a warning level that has fired.
///
/// # Errata
/// - `PMCU_ERR_03` (L110x/L13xx revisions C and D) — the warning levels do not work in STANDBY, and
///   a device passing one there does not reset properly. [`low_power::sleep`](crate::low_power::sleep)
///   handles it: it drops to [`BorThreshold::Bor0`] before entering STANDBY and restores the asked-for
///   level on wake, so a caller does not have to know.
#[cfg(feature = "bor-warning")]
pub fn set_bor_threshold(threshold: BorThreshold) -> Result<(), BorThreshold> {
    program_bor_threshold(threshold);
    bor_settle();

    match bor_threshold() {
        active if active == threshold => Ok(()),
        active => Err(active),
    }
}

/// Ask for `threshold` without waiting for it.
///
/// Split from the wait because the two directions do not need the same thing. Going *down* to the
/// reset level has to be complete before anything depends on it, so the sleep path waits. Coming back
/// *up* does not: until the change lands the supervisor is still at the reset level, which is the
/// safer of the two, and a wake path measured in tens of microseconds should not spend fifteen of
/// them re-arming a warning.
#[cfg(feature = "bor-warning")]
pub(crate) fn program_bor_threshold(threshold: BorThreshold) {
    let sysctl = pac::SYSCTL;

    sysctl.borthreshold().write(|w| w.set_level(threshold.to_bits()));
    sysctl.borclrcmd().write(|w| {
        w.set_key(vals::BorclrcmdKey::Key);
        w.set_go(true);
    });
}

/// Wait out a threshold change.
///
/// Nothing reports the transit — `BORCURTHRESHOLD` reads the old level until it reads the new one, so
/// polling it cannot tell "not yet" from "refused" and a poll on a refused change never ends.
#[cfg(feature = "bor-warning")]
pub(crate) fn bor_settle() {
    cortex_m::asm::delay(bor_change_cycles(clocks().mclk));
}

/// Cycles covering the threshold change at `mclk`.
#[cfg(feature = "bor-warning")]
const fn bor_change_cycles(mclk: u32) -> u32 {
    // 15 us, from SLAU847 §2.2.3.2, as `mclk / (1e9 / 15_000)`. Kept in `u32`: the obvious
    // `mclk * 15_000 / 1e9` needs 64 bits, which on this core is a call to `__aeabi_lmul` and another
    // to `__aeabi_uldivmod` -- measured at 132 B for this one function, and it would link the 64-bit
    // divider into low-power binaries that carry no other reason for it.
    mclk.div_ceil(66_667)
}

#[cfg(feature = "bor-warning")]
const _: () = {
    core::assert!(bor_change_cycles(32_000_000) == 480);
    core::assert!(bor_change_cycles(80_000_000) == 1_200);
    // Never zero, however slow the clock, or the change would be read back before it started.
    core::assert!(bor_change_cycles(32_768) == 1);
    core::assert!(bor_change_cycles(1) == 1);
};

/// Highest frequency MCLK may run at on this chip.
pub const MAX_MCLK_HZ: u32 = crate::_generated::MAX_MCLK_HZ;

/// Highest frequency ULPCLK may run at on this chip, in RUN and SLEEP.
pub const MAX_ULPCLK_HZ: u32 = crate::_generated::MAX_ULPCLK_HZ;

/// Frequency of LFCLK, the only clock that survives STANDBY.
pub const LFCLK_HZ: u32 = 32_768;

/// Frequency of MFCLK, the middle-frequency clock available down to STOP1.
pub const MFCLK_HZ: u32 = 4_000_000;

/// The clock tree currently programmed.
///
/// Written once by [`crate::init`] and read by every driver that needs a rate. It starts at
/// [`Clocks::RESET`], so a read before initialisation reports the tree the device actually boots
/// with rather than panicking.
static CLOCKS: Mutex<Cell<Clocks>> = Mutex::new(Cell::new(Clocks::RESET));

/// The rates the clock tree is running at.
///
/// Before [`crate::init`] this is the reset tree. Configure it through
/// [`Config::clock`](crate::Config::clock).
pub fn clocks() -> Clocks {
    with_clocks(|clocks| *clocks)
}

/// Read the tree in place, without the copy [`clocks`] hands back.
///
/// [`Clocks`] is forty bytes and the core has no wide load, so returning one by value is a `memcpy` call
/// — which links the software one, ~600 bytes, into a binary that may need it for nothing else. A driver
/// that only reads rates should take them through here.
#[inline]
pub(crate) fn with_clocks<R>(f: impl FnOnce(&Clocks) -> R) -> R {
    critical_section::with(|cs| {
        // SAFETY: every write goes through `set_clocks`, which needs the same token, so nothing can be
        // mutating the cell for as long as `cs` is held.
        f(unsafe { &*CLOCKS.borrow(cs).as_ptr() })
    })
}

/// Publish the tree [`crate::init`] just programmed.
#[inline(always)]
pub(crate) fn set_clocks(cs: CriticalSection, clocks: Clocks) {
    // The static already holds the reset tree, so a program that does not configure one has nothing
    // to publish. Folds away entirely when the tree is a constant, which is the common case.
    if clocks != Clocks::RESET {
        CLOCKS.borrow(cs).set(clocks);
    }
}

/// Rate an instance sees when it selects the bus clock, which depends on the domain it is in.
///
/// [`PowerDomain::Backup`] answers ULPCLK: its logic runs from LFCLK, but its registers are reached
/// over the PD0 bus.
///
/// This reads the live tree; where a driver already holds a [`Clocks`], prefer
/// [`Clocks::bus_clock`] to avoid a second critical section.
pub fn bus_clock_hz(domain: PowerDomain) -> u32 {
    clocks().bus_clock(domain)
}

/// Divider applied to the clock source of the CLK_OUT pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClkOutDiv {
    /// Divide by 2.
    Div2,

    /// Divide by 4.
    Div4,

    /// Divide by 6.
    Div6,

    /// Divide by 8.
    Div8,

    /// Divide by 10.
    Div10,

    /// Divide by 12.
    Div12,

    /// Divide by 14.
    Div14,

    /// Divide by 16.
    Div16,
}

/// CLK_OUT pin driver.
pub struct ClkOut<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> ClkOut<'d> {
    /// Create a new CLK_OUT instance.
    pub fn new(_peri: Peri<'d, CLK_OUT>, pin: Peri<'d, impl ClkOutPin>, source: ClkOutSource) -> Self {
        // The pin only ever drives the clock out, so there is nothing to configure on it: a pull
        // would fight the output driver and inversion would invert the clock.
        let pf = PfType::output(Pull::None, false);
        pin.set_as_pf(pin.pf_num(), pf);
        let pin: Peri<'d, AnyPin> = pin.into();

        let (en_div, div) = source.convert_div();
        let src = source.convert_src();
        pac::SYSCTL.genclkcfg().modify(|w| {
            w.set_exclksrc(src);
            w.set_exclkdivval(div);
            w.set_exclkdiven(en_div);
        });

        pac::SYSCTL.genclken().modify(|w| {
            w.set_exclken(true);
        });

        Self { pin }
    }
}

impl<'d> Drop for ClkOut<'d> {
    fn drop(&mut self) {
        pac::SYSCTL.genclken().modify(|w| {
            w.set_exclken(false);
        });

        self.pin.set_as_disconnected();
    }
}

/// ClkOut pin trait.
pub trait ClkOutPin: Pin {
    /// Get the PF number needed to use this pin aas ClkOut pin.
    fn pf_num(&self) -> u8;
}

macro_rules! impl_clk_out_pin {
    ($pin: ident, $pf: expr) => {
        impl crate::sysctl::ClkOutPin for $crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

/// (DIVEN, DIVVAL)
fn div_to_pac(div: Option<ClkOutDiv>) -> (bool, vals::Exclkdivval) {
    match div {
        Some(ClkOutDiv::Div2) => (true, vals::Exclkdivval::Div2),
        Some(ClkOutDiv::Div4) => (true, vals::Exclkdivval::Div4),
        Some(ClkOutDiv::Div6) => (true, vals::Exclkdivval::Div6),
        Some(ClkOutDiv::Div8) => (true, vals::Exclkdivval::Div8),
        Some(ClkOutDiv::Div10) => (true, vals::Exclkdivval::Div10),
        Some(ClkOutDiv::Div12) => (true, vals::Exclkdivval::Div12),
        Some(ClkOutDiv::Div14) => (true, vals::Exclkdivval::Div14),
        Some(ClkOutDiv::Div16) => (true, vals::Exclkdivval::Div16),
        // divider is ignored. set to default value
        None => (false, vals::Exclkdivval::Div2),
    }
}
