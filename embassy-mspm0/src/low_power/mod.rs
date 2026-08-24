//! Low-power (deep-sleep) support.
//!
//! Deep-sleep depth is gated by [`WakeGuard`](crate::sysctl::WakeGuard), which drivers hold to keep
//! the chip shallower than a given [`SleepLevel`]; the low-power executor then idles into the deepest
//! mode no guard blocks.
//!
//! With a time driver, duration gates it too: a wake scheduled sooner than `Config::min_sleep` leaves
//! the idle a plain `WFI`. Without one nothing schedules a wake, so only the guards decide and this
//! module needs no time source at all.
//!
//! # Wake source caveats
//! - `GPIO_ERR_01` (L110x/L13xx, G1x0x/G3x0x) — a wake edge can be missed. Only the STANDBY1 half is
//!   handled: if the pin is still asserted when the chip goes back to sleep no further edge is detected
//! - `GPIO_ERR_08` (G151x/G351x, H) — in low-power mode a GPIO can trigger a fast wake regardless of
//!   the `FASTWAKE` register and the pin configuration.
//! - `UART_ERR_01` (every family but G511x/G5187) — a start bit arriving while the chip is on its way
//!   back into STANDBY1 is not received.
//! - `PMCU_ERR_08` (L122x/L222x, G1x0x/G3x0x) — a wake arriving while the chip is still on its way
//!   into the mode adds ~3 µs of wake latency. Timing only, and there is no workaround.
//! - `RTC_ERR_01` (G1x0x/G3x0x, G151x/G351x, C1105/C1106) — `RTCRDY` and `RTC_PRESCALER1` do not wake
//!   from STANDBY1. Wake on `RTC_ALARM` or `RTC_PRESCALER0` instead.
use core::sync::atomic::Ordering;

use critical_section::CriticalSection;
#[cfg(feature = "_time-driver")]
use embassy_time::{Duration, TICK_HZ};
use pac::sysctl::vals::Dsleep;
use portable_atomic::AtomicU8;
#[cfg(feature = "_time-driver")]
use portable_atomic::AtomicU32;

use crate::pac;
use crate::prefetch::PrefetchSuspend;

mod inner;

pub use inner::{SleepMode, enter_sleep};

pub use crate::sysctl::SleepLevel;

/// `WakeGuard`s held at each [`SleepLevel`], one byte per level in `SleepLevel::LEVELS` order.
///
/// # Exported so a debugger can read it while the part runs
///
/// Sleep depth is the one thing an instrument cannot observe from outside, because attaching holds
/// the part awake -- so a current measurement taken with a probe on the pins says nothing about what
/// the firmware intended. These counts are the intent, and reading them is the difference between
/// "the board draws more than expected" and "something holds a guard at `Standby1`".
///
/// The name is fixed rather than mangled so that address is findable across builds. No `volatile` is
/// needed and no halt: these are already atomics, so the operations on them cannot be folded away
/// the way a plain `static`'s can, and a byte cannot tear, so all five read consistently from a
/// running core.
///
/// **Two copies of this crate in one image now fail to link rather than each keeping their own
/// counts.** That is the better failure -- the same duplication already produces a `__pender`
/// clash, and a silent second set of guard counts would be worse than a link error.
#[unsafe(export_name = "embassy_mspm0_sleep_blocks")]
static SLEEP_BLOCKS: [AtomicU8; 5] = [const { AtomicU8::new(0) }; 5];

/// Longest wake-up latency this device's datasheet publishes for a deep-sleep mode, in nanoseconds.
///
/// Typical rather than a ceiling: TI gives one unqualified figure per mode, in a cell spanning the
/// MIN, TYP and MAX columns. Modes the datasheet has no figure for do not count towards it.
pub const MAX_WAKE_NS: u32 = crate::_generated::MAX_WAKE_NS;

/// Default [`Config::min_sleep`](crate::Config::min_sleep): four times [`MAX_WAKE_NS`], rounded up to
/// a whole tick.
///
/// Entry costs roughly what wake does, and the published figures are typical rather than worst case,
/// so this is about the shortest sleep that can pay for itself — two or three ticks on every MSPM0.
#[cfg(feature = "_time-driver")]
pub const DEFAULT_MIN_SLEEP: Duration = Duration::from_ticks(ns_to_ticks(4 * MAX_WAKE_NS as u64));

/// Ticks covering `ns`, rounded up.
#[cfg(feature = "_time-driver")]
const fn ns_to_ticks(ns: u64) -> u64 {
    (ns * TICK_HZ).div_ceil(1_000_000_000)
}

/// [`Config::min_sleep`](crate::Config::min_sleep) in ticks, saturated to what fits.
///
/// A `u32` reaches 36 hours at 32.768 kHz, and reading it needs no critical section on a target
/// without 64-bit atomics.
///
/// Exported under a fixed name for the same reason as [`SLEEP_BLOCKS`]: with the guard counts it
/// answers why a part is not sleeping as deeply as expected, and this is the half that says the next
/// wake is simply too soon to be worth it.
#[cfg(feature = "_time-driver")]
#[unsafe(export_name = "embassy_mspm0_min_sleep_ticks")]
static MIN_SLEEP_TICKS: AtomicU32 = AtomicU32::new(DEFAULT_MIN_SLEEP.as_ticks() as u32);

/// Apply [`Config::min_sleep`](crate::Config::min_sleep).
#[cfg(feature = "_time-driver")]
pub(crate) fn set_min_sleep(min_sleep: Duration) {
    MIN_SLEEP_TICKS.store(min_sleep.as_ticks().min(u32::MAX as u64) as u32, Ordering::Relaxed);
}

/// Whether the next wake is far enough out for a deep-sleep mode to be worth entering.
#[cfg(feature = "_time-driver")]
fn min_sleep_met(cs: CriticalSection) -> bool {
    crate::time_driver::wake_at_least(cs, MIN_SLEEP_TICKS.load(Ordering::Relaxed))
}

/// Without a time driver nothing schedules a wake, so the sleep is unbounded and always long enough.
#[cfg(not(feature = "_time-driver"))]
fn min_sleep_met(_cs: CriticalSection) -> bool {
    true
}

/// Block sleep at `level` and every deeper mode. Paired with [`unblock`] by
/// [`WakeGuard`](crate::sysctl::WakeGuard).
#[inline]
pub(crate) fn block(level: SleepLevel) {
    apply(level, 1);
}

/// Remove a block previously added at `level`.
#[inline]
pub(crate) fn unblock(level: SleepLevel) {
    apply(level, -1);
}

/// Move [`SLEEP_BLOCKS`] at `level` by `delta`, which is `1` to take a guard and `-1` to release one.
///
/// # One address, both directions, deliberately
///
/// The counts say a floor is held; this says by whom. Out of line so there is a single instruction to
/// break on, and *shared* so that one breakpoint covers every acquisition and release in the image
/// rather than one per call site -- which matters when the core has four of them and a session wants
/// one for something else. AAPCS puts the level in `r0` and the delta in `r1`, so a halt here reads
/// both which level moved and which way straight out of the registers, and the call leaves a frame to
/// unwind through to the driver that took the guard. An inlined read-modify-write leaves neither.
///
/// Armed from reset this produces a ledger of every guard operation, and whatever has a `+1` with no
/// matching `-1` is what holds the floor. That costs a reset and a halt per operation, so it is the
/// escalation rather than the first thing to reach for: poll the counts, and come here once they say
/// something is stuck.
#[inline(never)]
fn apply(level: SleepLevel, delta: i8) {
    trace!("Sleep block at level {:?}: {}", level, delta);

    let blocks = &SLEEP_BLOCKS[level as usize];

    if delta > 0 {
        if blocks.fetch_add(1, Ordering::Relaxed) == (u8::MAX - 1) {
            panic!("Blocking at SleepLevel {:?} would overflow", level)
        }
    } else {
        blocks.fetch_sub(1, Ordering::Relaxed);
    }
}

/// Deepest mode currently permitted, or `None` if all deep sleep is blocked.
fn deepest_allowed() -> Option<SleepLevel> {
    for (i, blocks) in SLEEP_BLOCKS.iter().enumerate() {
        if blocks.load(Ordering::Relaxed) > 0 {
            return i.checked_sub(1).map(|j| SleepLevel::LEVELS[j]);
        }
    }
    Some(SleepLevel::Standby1)
}

/// Enter the deepest sleep permitted by the active [`WakeGuard`](crate::sysctl::WakeGuard)s, waiting
/// for an interrupt.
///
/// Called by the low-power executor on idle. With no guards held it enters the deepest mode the
/// chip supports; a held guard caps the depth, and a guard on [`SleepLevel::Stop0`] keeps it a
/// plain `WFI`.
///
/// With a time driver, a wake scheduled sooner than `Config::min_sleep` also leaves it a plain `WFI`.
///
/// Another scheduler can call it to get the same idle behaviour: under RTIC that is `#[idle]`, which
/// is the only place the safety condition below holds.
///
/// # Safety
/// Must be called from thread mode. `WFI` in a handler is only woken by an interrupt of *higher*
/// priority than the one running, so sleeping inside the lowest-priority handler never returns. An
/// RTIC software task runs in its dispatcher's handler and is therefore not a valid caller.
///
/// Deep sleep powers down PD1 (and, in STANDBY, most of PD0). The drivers hold their own
/// [`WakeGuard`](crate::sysctl::WakeGuard)s for work that has to survive it; anything driving a
/// peripheral through the raw `pac` is responsible for its own.
pub unsafe fn sleep(cs: CriticalSection) {
    trace!("Attempting to enter low-power sleep");

    // `CPU_ERR_03` is written against "low power modes" rather than the deep ones, so the guard is
    // taken before the `WFI` below as well as before a deep sleep. See its note on `PrefetchSuspend`:
    // an application idling through `embassy-executor`'s own executor does not get this.
    let _prefetch = PrefetchSuspend::new();

    match deepest_allowed().filter(|_| min_sleep_met(cs)) {
        None => {
            trace!("Low-power sleep blocked");
            cortex_m::asm::dsb();
            cortex_m::asm::wfi();
            cortex_m::asm::isb();
        }
        Some(level) => {
            trace!("Low-power sleep allowed, mode: {:?}", level);
            let _bor = BorSuspend::new(level);
            enter_sleep(cs, inner::level_to_mode(level));
        }
    }
}

/// Enter SHUTDOWN, the lowest-power state. Does not return.
///
/// SHUTDOWN powers down VCORE: all SRAM is lost except the `SHUTDNSTORE` bytes, and the only wake
/// sources are a wake-capable IO event, NRST, or SWD activity. The wake is a reset, which boot can
/// identify with [`ResetCause::BorWakeFromShutdown`](crate::ResetCause).
///
/// Arm an IO wake with [`ShutdownWake`](crate::gpio::ShutdownWake), which accepts only the pins that
/// have wakeup logic. The `FASTWAKE` path the edge-wait methods on
/// [`Flex`](crate::gpio::Flex) use does not reach this far — it stops at STANDBY — so a pin armed only
/// that way will not bring the device back.
///
/// # Errata
/// - `SYSCTL_ERR_05` (L110x/L13xx, G1x0x/G3x0x, G151x/G351x, C1103/C1104) — LFCLK is stuck after the
///   wake if `LFCLK_IN` is configured as an input or with a pull-up. Give that pin a pull-down, or
///   another function, before entering.
/// - `PMCU_ERR_11` (G151x/G351x) — waking with an NRST pulse shorter than 1 s reports the wrong reset
///   cause, so the `ResetCause` check above does not identify the wake. No workaround.
//
// From the TRM: SYSCTL "Operating Modes": set `PMODECFG.DSLEEP = SHUTDOWN`, arm `SLEEPDEEP`,
// then `WFI`. This is identical across every MSPM0 family.
pub fn shutdown(_cs: CriticalSection) -> ! {
    let sysctl = pac::SYSCTL;
    sysctl.pmodecfg().modify(|w| w.set_dsleep(Dsleep::Shutdown));

    let _prefetch = PrefetchSuspend::new();

    let mut scb = unsafe { cortex_m::Peripherals::steal() }.SCB;
    scb.set_sleepdeep();
    cortex_m::asm::dsb();

    // `WFI` completes without sleeping when a wake event is already pending, and a debug event counts,
    // so it can fall through with a probe attached. Ask again rather than assume it slept.
    loop {
        cortex_m::asm::wfi();
    }
}

/// Workaround for `PMCU_ERR_03` — the brown-out supervisor's warning levels do not work in STANDBY,
/// and a supply passing one there does not reset the device properly.
///
/// The errata sheet's own workaround: go back to the reset level before entering STANDBY, and put the
/// warning level back on the way out. Doing it here rather than leaving it to the caller is what makes
/// it reliable — the failure is a brown-out that does not reset, which nothing reports and no test
/// finds by accident.
///
/// Costs nothing unless it is doing something. A caller that never raised the threshold, which is
/// every application until one asks for a warning level, takes one register read and no delay. Where
/// it does act it spends the change time twice, about 30 us against a wake path measured in tens —
/// that is the price of the feature, paid only by the applications that want it.
#[cfg(mspm0_bor_sleep_guard)]
struct BorSuspend(Option<crate::sysctl::BorThreshold>);

#[cfg(mspm0_bor_sleep_guard)]
impl BorSuspend {
    fn new(level: SleepLevel) -> Self {
        // Only STANDBY. The advisory names it and no other mode, and the supervisor is documented as
        // running normally in STOP -- dropping the warning level there would give up the feature for
        // a reason that does not apply.
        if level < SleepLevel::Standby0 {
            return Self(None);
        }

        // What the application asked for, which the hardware keeps even after a warning has fired and
        // dropped the *active* level. Restoring the active level would re-arm nothing.
        let asked = pac::SYSCTL.borthreshold().read().level();

        if asked == crate::sysctl::BorThreshold::Bor0 as u8 {
            return Self(None);
        }

        crate::sysctl::program_bor_threshold(crate::sysctl::BorThreshold::Bor0);
        crate::sysctl::bor_settle();

        Self(Some(match asked {
            1 => crate::sysctl::BorThreshold::Bor1,
            2 => crate::sysctl::BorThreshold::Bor2,
            _ => crate::sysctl::BorThreshold::Bor3,
        }))
    }
}

#[cfg(mspm0_bor_sleep_guard)]
impl Drop for BorSuspend {
    fn drop(&mut self) {
        // Not waited out: see `program_bor_threshold`. The interim state is the reset level.
        if let Some(threshold) = self.0 {
            crate::sysctl::program_bor_threshold(threshold);
        }
    }
}

/// The brown-out supervisor needs nothing done around sleep here — the `bor-warning` feature is off,
/// so no warning level can have been selected, or `PMCU_ERR_03` does not apply to this device.
#[cfg(not(mspm0_bor_sleep_guard))]
struct BorSuspend;

#[cfg(not(mspm0_bor_sleep_guard))]
impl BorSuspend {
    #[inline(always)]
    fn new(_level: SleepLevel) -> Self {
        Self
    }
}
/// Arm ARM deep-sleep (`SLEEPDEEP`), wait for an interrupt, then clear it.
///
/// The mode-specific SYSCTL programming must already be done by the caller, and the prefetcher must
/// already be suspended by [`PrefetchSuspend`]. `WFI` wakes on a pending enabled interrupt even with
/// PRIMASK set; `SLEEPDEEP` is cleared on wake so a later plain executor idle does not deep-sleep.
pub(crate) unsafe fn arm_and_wait() {
    let mut scb = unsafe { cortex_m::Peripherals::steal() }.SCB;
    scb.set_sleepdeep();
    cortex_m::asm::dsb();
    cortex_m::asm::wfi();
    cortex_m::asm::isb();
    scb.clear_sleepdeep();
}
