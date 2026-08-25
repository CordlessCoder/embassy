//! Window Watchdog Timer (WWDT).
//!
//! A windowed watchdog: petting it too early is a violation as much as petting it too late, so a task
//! stuck in a tight loop is caught as well as one that has stopped.
//!
//! # It stops in STANDBY1, so it blocks it
//!
//! **The watchdog is usable down to STANDBY0 and stops in STANDBY1**, which unclocks PD0 — on 16 of the
//! 18 families; on MSPM0C1105/C1106 and MSPM0H321x it reaches STANDBY1 as well. That is the datasheet's
//! own per-mode answer, and the driver takes it from the chip metadata rather than assuming.
//!
//! A [`Watchdog`] built with `stop_in_sleep` left `false` — the default, meaning "keep counting while
//! the CPU is asleep" — **holds a sleep guard blocking whatever the metadata says it cannot count in**.
//! Deep sleep cannot then disarm it without saying so. On most parts that costs the deepest mode only:
//! such an application reaches STANDBY0 and not STANDBY1.
//!
//! Set `stop_in_sleep` to `true` and no guard is taken. That configuration asks the watchdog to pause
//! while the CPU sleeps, which is what the hardware does anyway, so the two agree and every level stays
//! reachable. **It also means nothing is watching during the sleep.**
//!
//! **To be watched in STANDBY1 too**, use the IWDT on a part that has one: a different peripheral (TRM
//! chapter 38, where this is chapter 39), usable through STANDBY1 on all three families that carry it.
//! This HAL does not drive it and the metapac generates no register block for it. Failing that, a timer
//! that survives the depth — `clocked_in_standby1` — can wake the device, though a timer supervises
//! nothing by itself.
//!
//! # Stopping it
//!
//! Dropping a [`Watchdog`] stops it and releases the guard. `WWDTCTL0` is write protected once the
//! watchdog is running and writing it is itself a violation, so the peripheral reset is what does this;
//! it is the only way out short of resetting the device.
//!
//! Keep the handle alive for as long as the watchdog should be watching — [`core::mem::forget`] it if
//! that is for ever.

#![allow(missing_docs)]
// 65 undocumented items, and documenting them properly is a pass of its own rather than a line each.
#![macro_use]

use core::marker::PhantomData;

use embassy_hal_internal::PeripheralType;

use crate::Peri;
use crate::pac::wwdt::{Wwdt as Regs, vals};
use crate::pac::{self};
use crate::sysctl::{LowPowerInstance, MaybeWakeGuard};

/// Possible watchdog timeout values.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Timeout {
    USec1950,
    USec3910,
    USec5860,
    USec7810,
    USec9770,
    USec11720,
    USec13670,
    USec15630,
    USec23440,
    USec31250,
    USec39060,
    USec46880,
    USec54690,
    USec62500,
    USec93750,
    USec125000,
    USec156250,
    USec187500,
    USec218750,
    MSec130,
    MSec250,
    MSec380,
    MSec500,
    MSec630,
    MSec750,
    MSec880,
    Sec1,
    Sec2,
    Sec3,
    Sec4,
    Sec5,
    Sec6,
    Sec7,
    Sec8,
    Sec16,
    Sec24,
    Sec32,
    Sec40,
    Sec48,
    Sec56,
    Sec64,
    Sec128,  // 2.13 min
    Sec192,  // 3.20 min
    Sec256,  // 4.27 min
    Sec320,  // 5.33 min
    Sec384,  // 6.40 min
    Sec448,  // 7.47 min
    Sec512,  // 8.53 min
    Sec1024, // 17.07 min
    Sec2048, // 34.13 min
    Sec3072, // 51.20 min
    Sec4096, // 68.27 min
    Sec5120, // 85.33 min
    Sec6144, // 102.40 min
    Sec7168, // 119.47 min
    Sec8192, // 136.53 min
}

impl Timeout {
    /// How long the watchdog runs before it expires.
    pub const fn period_micros(self) -> u64 {
        // The counter is clocked from LFCLK, divided by `clkdiv + 1`, and expires after 2^exp ticks.
        let exp = self.get_period_exponent() as u64;
        let divider = self.get_clkdiv() as u64 + 1;

        (1 << exp) * divider * 1_000_000 / crate::sysctl::LFCLK_HZ as u64
    }

    const fn get_period_exponent(self) -> u8 {
        match self.get_period() {
            vals::Per::En6 => 6,
            vals::Per::En8 => 8,
            vals::Per::En10 => 10,
            vals::Per::En12 => 12,
            vals::Per::En15 => 15,
            vals::Per::En18 => 18,
            vals::Per::En21 => 21,
            vals::Per::En25 => 25,
        }
    }

    const fn get_period(self) -> vals::Per {
        match self {
            //  period count is 2**25
            Self::Sec1024
            | Self::Sec2048
            | Self::Sec3072
            | Self::Sec4096
            | Self::Sec5120
            | Self::Sec6144
            | Self::Sec7168
            | Self::Sec8192 => vals::Per::En25,
            //  period count is 2**21
            Self::Sec64
            | Self::Sec128
            | Self::Sec192
            | Self::Sec256
            | Self::Sec320
            | Self::Sec384
            | Self::Sec448
            | Self::Sec512 => vals::Per::En21,
            //  period count is 2**18
            Self::Sec8 | Self::Sec16 | Self::Sec24 | Self::Sec32 | Self::Sec40 | Self::Sec48 | Self::Sec56 => {
                vals::Per::En18
            }
            //  period count is 2**15
            Self::Sec1 | Self::Sec2 | Self::Sec3 | Self::Sec4 | Self::Sec5 | Self::Sec6 | Self::Sec7 => vals::Per::En15,
            //  period count is 2**12
            Self::MSec130
            | Self::MSec250
            | Self::MSec380
            | Self::MSec500
            | Self::MSec630
            | Self::MSec750
            | Self::MSec880 => vals::Per::En12,
            //  period count is 2**10
            Self::USec31250
            | Self::USec62500
            | Self::USec93750
            | Self::USec125000
            | Self::USec156250
            | Self::USec187500
            | Self::USec218750 => vals::Per::En10,
            //  period count is 2**8
            Self::USec7810
            | Self::USec15630
            | Self::USec23440
            | Self::USec39060
            | Self::USec46880
            | Self::USec54690 => vals::Per::En8,
            //  period count is 2**6
            Self::USec1950 | Self::USec3910 | Self::USec5860 | Self::USec9770 | Self::USec11720 | Self::USec13670 => {
                vals::Per::En6
            }
        }
    }

    const fn get_clkdiv(self) -> u8 {
        match self {
            //  divide by 1
            Self::USec1950
            | Self::USec7810
            | Self::USec31250
            | Self::MSec130
            | Self::Sec1
            | Self::Sec8
            | Self::Sec64
            | Self::Sec1024 => 0u8,
            //  divide by 2
            Self::USec3910
            | Self::USec15630
            | Self::USec62500
            | Self::MSec250
            | Self::Sec2
            | Self::Sec16
            | Self::Sec128
            | Self::Sec2048 => 1u8,
            //  divide by 3
            Self::USec5860
            | Self::USec23440
            | Self::USec93750
            | Self::MSec380
            | Self::Sec3
            | Self::Sec24
            | Self::Sec192
            | Self::Sec3072 => 2u8,
            //  divide by 4
            Self::USec125000 | Self::MSec500 | Self::Sec4 | Self::Sec32 | Self::Sec256 | Self::Sec4096 => 3u8,
            //  divide by 5
            Self::USec9770
            | Self::USec39060
            | Self::USec156250
            | Self::MSec630
            | Self::Sec5
            | Self::Sec40
            | Self::Sec320
            | Self::Sec5120 => 4u8,
            //  divide by 6
            Self::USec11720
            | Self::USec46880
            | Self::USec187500
            | Self::MSec750
            | Self::Sec6
            | Self::Sec48
            | Self::Sec384
            | Self::Sec6144 => 5u8,
            //  divide by 7
            Self::USec13670
            | Self::USec54690
            | Self::USec218750
            | Self::MSec880
            | Self::Sec7
            | Self::Sec56
            | Self::Sec448
            | Self::Sec7168 => 6u8,
            //  divide by 8
            Self::Sec512 | Self::Sec8192 => 7u8,
        }
    }
}

/// Timeout percentage that is treated as "too early" and generates violation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClosedWindowPercentage {
    // window period is not used
    Zero,
    // 12.5% percents
    Twelve,
    // 18.75% percents
    Eighteen,
    // 25% percents
    TwentyFive,
    // 50% percents
    Fifty,
    // 75% percents
    SeventyFive,
    // 81.25% percents
    EightyOne,
    // 87.5% percents
    EightySeven,
}

impl ClosedWindowPercentage {
    /// The closed part of the period, in sixteenths. Every percentage the hardware offers is one.
    const fn sixteenths(self) -> u64 {
        match self {
            Self::Zero => 0,
            Self::Twelve => 2,
            Self::Eighteen => 3,
            Self::TwentyFive => 4,
            Self::Fifty => 8,
            Self::SeventyFive => 12,
            Self::EightyOne => 13,
            Self::EightySeven => 14,
        }
    }

    const fn get_native_size(self) -> vals::Window {
        match self {
            Self::Zero => vals::Window::Size0,
            Self::Twelve => vals::Window::Size12,
            Self::Eighteen => vals::Window::Size18,
            Self::TwentyFive => vals::Window::Size25,
            Self::Fifty => vals::Window::Size50,
            Self::SeventyFive => vals::Window::Size75,
            Self::EightyOne => vals::Window::Size81,
            Self::EightySeven => vals::Window::Size87,
        }
    }
}

// Boundary checks for `period_micros` and `pet_interval_micros`. `crate::fmt` cannot be used in const.
const _: () = {
    // Shortest and longest the hardware offers, either end of the divider and period ranges.
    core::assert!(Timeout::USec1950.period_micros() == 1_953);
    core::assert!(Timeout::Sec8192.period_micros() == 8_192_000_000);
    core::assert!(Timeout::Sec1.period_micros() == 1_000_000);

    const fn interval(closed: ClosedWindowPercentage) -> u64 {
        Config {
            timeout: Timeout::Sec1,
            closed_window: closed,
            stop_in_sleep: false,
        }
        .pet_interval_micros()
    }

    // With no closed window, pet halfway; otherwise halfway between the window and the timeout.
    core::assert!(interval(ClosedWindowPercentage::Zero) == 500_000);
    core::assert!(interval(ClosedWindowPercentage::TwentyFive) == 625_000);
    core::assert!(interval(ClosedWindowPercentage::EightySeven) == 937_500);
};

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// Watchdog Config
pub struct Config {
    /// Watchdog timeout
    pub timeout: Timeout,

    /// closed window percentage
    pub closed_window: ClosedWindowPercentage,

    /// Stop counting while the CPU is asleep, resuming from the same count on wake.
    ///
    /// **This field decides whether the watchdog blocks deep sleep**, because the hardware cannot
    /// deliver what `false` promises from every mode — see the module docs.
    ///
    /// - `false`, the default and the hardware's: keep counting through sleep. [`Watchdog`] then holds
    ///   a sleep guard for its whole life, blocking the modes this instance's metadata says it is not
    ///   usable in — STANDBY1 on most parts, nothing at all on the two families that reach it. Every
    ///   mode left reachable is one the watchdog counts through.
    /// - `true`: pause while asleep and resume from the same count. No guard, every sleep level
    ///   reachable, and **nothing supervises the device while it sleeps**.
    pub stop_in_sleep: bool,
}

impl Config {
    /// How long to wait after a pet before petting again, the middle of the open window.
    pub const fn pet_interval_micros(&self) -> u64 {
        let closed = self.closed_window.sixteenths();

        self.timeout.period_micros() * (16 + closed) / 32
    }
}

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            timeout: Timeout::Sec1,
            closed_window: ClosedWindowPercentage::Zero,
            // The hardware default, and the only one that still guards a sleeping device.
            stop_in_sleep: false,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

pub struct Watchdog<'d> {
    regs: &'static Regs,
    config: Config,
    _instance: PhantomData<&'d mut ()>,

    /// Blocks the sleep levels the watchdog is disabled in, unless the caller asked it to stop in
    /// sleep. Held for the driver's whole life, because a watchdog is only worth anything while it is
    /// counting. See the module docs.
    _sleep_guard: MaybeWakeGuard,
}

impl<'d> Watchdog<'d> {
    /// Watchdog initialization.
    pub fn new<T: Instance + LowPowerInstance>(_instance: Peri<'d, T>, config: Config) -> Self {
        T::regs().gprcm(0).rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(vals::ResetKey::Key);
        });

        T::regs().gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });

        // init delay, 16 cycles
        cortex_m::asm::delay(16);

        critical_section::with(|_| {
            // make sure watchdog triggers BOOTRST
            pac::SYSCTL.systemcfg().modify(|w| {
                if *T::regs() == pac::WWDT0 {
                    w.set_wwdtlp0rstdis(false);
                }

                #[cfg(wwdt1)]
                if *T::regs() == pac::WWDT1 {
                    w.set_wwdtlp1rstdis(false);
                }
            });
        });

        //init watchdog
        T::regs().ctl0().write(|w| {
            w.set_clkdiv(config.timeout.get_clkdiv());
            w.set_per(config.timeout.get_period());
            w.set_mode(vals::Mode::Window);
            w.set_window(0, config.closed_window.get_native_size());
            w.set_window(1, vals::Window::Size0);
            w.set_stism(if config.stop_in_sleep {
                vals::Stism::Stop
            } else {
                vals::Stism::Cont
            });
            w.set_key(vals::Wwdtctl0Key::Key);
        });

        T::regs().ctl1().write(|w| {
            w.set_winsel(vals::Winsel::Win0);
            w.set_key(vals::Wwdtctl1Key::Key);
        });

        // The datasheet's own answer for this instance rather than a constant here: `usable_through` is
        // `Standby0` on most parts, so the floor comes out at STANDBY1 and everything shallower stays
        // reachable. A watchdog asked to stop in sleep wants no floor at all — the hardware stopping it
        // is then the behaviour, not a surprise.
        let floor = if config.stop_in_sleep {
            None
        } else {
            <T as LowPowerInstance>::SLEEP.floor_to_stay_usable()
        };

        Self {
            _instance: PhantomData,
            regs: T::regs(),
            config,
            _sleep_guard: MaybeWakeGuard::new(floor),
        }
    }

    /// The configuration this watchdog was started with.
    pub fn config(&self) -> Config {
        self.config
    }

    /// Pet (reload, refresh) the watchdog.
    pub fn pet(&mut self) {
        self.regs.cntrst().write(|w| {
            w.set_restart(vals::WwdtcntrstRestart::Restart);
        });
    }

    /// How long [`Self::run`] waits between pets, the middle of the open window.
    #[cfg(feature = "time")]
    pub fn pet_interval(&self) -> embassy_time::Duration {
        embassy_time::Duration::from_micros(self.config.pet_interval_micros())
    }

    /// Pet the watchdog forever, at [`Self::pet_interval`].
    ///
    /// Meant to be spawned as its own task. Nothing else may pet the watchdog while this runs, since
    /// petting during the closed window is itself a fault.
    #[cfg(feature = "time")]
    pub async fn run(mut self) -> ! {
        let interval = self.pet_interval();

        loop {
            embassy_time::Timer::after(interval).await;
            self.pet();
        }
    }
}

/// Stops the watchdog and gives its instance back.
///
/// **A stopped watchdog is watching nothing**, so dropping the handle is a real decision and not
/// bookkeeping. It is the right default here for two reasons: it is what every other driver in this HAL
/// does with its peripheral, and without it dropping the handle would release the sleep guard while the
/// counter kept running — arming a reset for whenever the device next woke with nobody petting it.
///
/// **To make the watchdog un-stoppable, do not let the handle drop**: [`core::mem::forget`] it, or keep
/// it for the life of the program. A watchdog that a stray scope exit can switch off is not the thing a
/// safety case wants, and this API cannot tell the two uses apart.
impl Drop for Watchdog<'_> {
    fn drop(&mut self) {
        // The peripheral reset is the only way out. `WWDTCTL0` becomes write protected the moment the
        // watchdog is enabled, and writing it after that is itself a violation (SLAU846 §39.2.1), so the
        // configuration registers cannot turn it off — they can only reset the device.
        //
        // Measured: the reset alone stops a running watchdog, which then survives well past its timeout.
        self.regs.gprcm(0).rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(vals::ResetKey::Key);
        });

        // Not what stops it — the reset above did that. This leaves the instance as `init` found it, so
        // a later `new` starts from the same place the first one did.
        self.regs.gprcm(0).pwren().write(|w| {
            w.set_enable(false);
            w.set_key(vals::PwrenKey::Key);
        });
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> &'static Regs;
}

/// WWDT instance trait
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

macro_rules! impl_wwdt_instance {
    ($instance: ident) => {
        impl crate::wwdt::SealedInstance for crate::peripherals::$instance {
            fn regs() -> &'static crate::pac::wwdt::Wwdt {
                &crate::pac::$instance
            }
        }

        impl crate::wwdt::Instance for crate::peripherals::$instance {}
    };
}
