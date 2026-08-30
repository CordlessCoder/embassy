//! Clock tree configuration.
//!
//! [`Config`] describes what the oscillators and muxes should be set to, and [`Config::resolve`]
//! turns that into the [`Clocks`] every driver reads its rate from. Resolution is a `const fn` over
//! `u32`, so a configuration built from constants folds at compile time:
//!
//! ```ignore
//! use embassy_mspm0::sysctl::clock::{Clocks, Config, MclkSource};
//!
//! const CFG: Config = Config::new().with_mclk(MclkSource::Hsclk);
//! const CLOCKS: Clocks = match CFG.resolve() {
//!     Ok(clocks) => clocks,
//!     Err(_) => core::panic!("clock configuration is out of range"),
//! };
//! ```
//!
//! Nothing here touches a register — [`crate::init`] applies the resolved tree.
//!
//! # Device differences
//! What this device has is `METADATA.clock_tree`, turned into cfgs by the build script. Broadly:
//! SYSOSC and LFOSC only (L110x/L130x/L134x); those plus a high-speed input (C110x); plus crystals
//! (C1105/C1106, H321x, L122x/L222x); and the full tree including SYSPLL (G-series).
//!
//! The gates are per capability, not per family: `mspm0_hfclk` for the HSCLK path at all,
//! `mspm0_hfxt` for a crystal driver, `mspm0_hfclk_in` for a digital input, `mspm0_syspll`,
//! `mspm0_lfxt`, `mspm0_ulpclk_div`.

use super::{LFCLK_HZ, MAX_MCLK_HZ, MAX_ULPCLK_HZ, MFCLK_HZ};
use crate::pac;
use crate::pac::sysctl::vals;

/// How long to wait for a source to report itself good before giving up.
///
/// Generous against the slowest thing waited on: a crystal's startup is specified in milliseconds,
/// where the SYSPLL's is in microseconds. Only ever reached when an oscillator is not going to start
/// at all, typically because the board has no crystal fitted.
#[cfg(any(mspm0_hfxt, mspm0_lfxt, mspm0_syspll))]
const SETTLE_TIMEOUT_US: u32 = 100_000;

/// Cycles to wait between polls, so the budget is spent in coarse steps rather than on bus traffic.
#[cfg(any(mspm0_hfxt, mspm0_lfxt, mspm0_syspll))]
const SETTLE_STEP_CYCLES: u32 = 64;

/// Why a [`Config`] cannot be realised on this chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ClockError {
    /// MCLK would exceed [`MAX_MCLK_HZ`].
    MclkTooFast,

    /// ULPCLK would exceed [`MAX_ULPCLK_HZ`]. On parts where ULPCLK is capped below MCLK, a fast
    /// MCLK needs `UlpclkDiv::Div2`.
    UlpclkTooFast,

    /// MCLK was set to `MclkSource::Hsclk` but neither HFCLK nor SYSPLL was configured to feed it.
    NoHsclkSource,

    /// A source the rest of the configuration depends on was left disabled. MFCLK and the SYSPLL
    /// both require SYSOSC, and the SYSPLL requires it at its base frequency.
    SourceDisabled,

    /// `MCLKCFG.MDIV` was used together with something that forbids it: it only applies to a SYSOSC
    /// fixed at 4 MHz, and MFCLK requires `MDIV = 0`.
    DividerNotAllowed,

    /// The requested divider is not one the hardware provides.
    DividerOutOfRange,

    /// The HFXT frequency is outside the 4-48 MHz the crystal driver supports.
    HfxtOutOfRange,

    /// The SYSPLL reference is outside the 4-48 MHz `fSYSPLLREF` range.
    PllReferenceOutOfRange,

    /// The SYSPLL VCO would fall outside the 80-400 MHz `fVCO` range.
    PllVcoOutOfRange,

    /// A SYSPLL output would fall outside its permitted range, or the tap chosen to drive MCLK was
    /// not enabled.
    PllOutputOutOfRange,

    /// A crystal or the PLL never reported itself good. Only returned when applying a
    /// configuration, never by [`Config::resolve`].
    SourceDidNotSettle,
}

/// The rate SYSOSC comes up at, which is also its "base frequency".
///
/// 32 MHz on every part but MSPM0C1103/C1104, where it is 24. Deliberately not inferred from
/// [`MAX_MCLK_HZ`]: the two agree on that one part by coincidence, not by construction.
pub const SYSOSC_BASE_HZ: u32 = crate::_generated::SYSOSC_BASE_HZ;

/// `fHFXT` and `fHFIN`, the range HFCLK must stay within, whether a crystal or a digital input.
///
/// Per device, not per family: only the G datasheets reach 48 MHz, everything else with an HFCLK path
/// stops at 32. Absent on MSPM0C110x, which specifies no `fHFIN` at all — the input is still usable
/// there, with only this check skipped, since [`MAX_MCLK_HZ`] bounds what it can drive.
#[cfg(mspm0_hfclk_range)]
const HFCLK_MIN_HZ: u32 = crate::_generated::HFCLK_MIN_HZ;
#[cfg(mspm0_hfclk_range)]
const HFCLK_MAX_HZ: u32 = crate::_generated::HFCLK_MAX_HZ;

/// `fSYSPLLREF`, the range the loop input must stay within.
///
/// Deliberately not [`HFCLK_MIN_HZ`]/[`HFCLK_MAX_HZ`]: this one really is 4-48 MHz on every device
/// that has a SYSPLL, and the loop can also be referenced from SYSOSC rather than HFCLK.
#[cfg(mspm0_syspll)]
const PLL_REF_MIN_HZ: u32 = 4_000_000;
#[cfg(mspm0_syspll)]
const PLL_REF_MAX_HZ: u32 = 48_000_000;

/// `fVCO` range, from the G350x datasheet section 7.9.3.
#[cfg(mspm0_syspll)]
const VCO_MIN_HZ: u32 = 80_000_000;
#[cfg(mspm0_syspll)]
const VCO_MAX_HZ: u32 = 400_000_000;

/// `fSYSPLL` output ranges, likewise. CLK2X has the doubler in front of its divider.
#[cfg(mspm0_syspll)]
const PLL_CLK_MIN_HZ: u32 = 2_500_000;
#[cfg(mspm0_syspll)]
const PLL_CLK_MAX_HZ: u32 = 200_000_000;
#[cfg(mspm0_syspll)]
const PLL_CLK2X_MIN_HZ: u32 = 10_000_000;
#[cfg(mspm0_syspll)]
const PLL_CLK2X_MAX_HZ: u32 = 400_000_000;

/// What SYSOSC is running at.
///
/// The frequency field may only be changed while SYSOSC is what sources MCLK, which
/// [`crate::init`] guarantees by programming it before switching MCLK anywhere else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sysosc {
    /// The device's base frequency, [`SYSOSC_BASE_HZ`]. Required for HFXT and SYSPLL operation.
    Base,

    /// The fixed 4 MHz low-power operating point.
    Mhz4,

    /// Powered down. Only valid when nothing else needs it, which rules out MFCLK and the SYSPLL.
    Disabled,
    // The 16 MHz and 24 MHz user-trimmed operating points are deliberately absent: USER in
    // `SYSOSCCFG.FREQ` only means something once `SYSOSCTRIMUSER` holds a trim measured per device
    // (TRM 2.3.1.2), so no value the HAL could program makes the resulting frequency knowable.
}

impl Sysosc {
    /// The rate this operating point runs at, or `None` when SYSOSC is off.
    pub const fn frequency(self) -> Option<u32> {
        match self {
            Self::Base => Some(SYSOSC_BASE_HZ),
            Self::Mhz4 => Some(4_000_000),
            Self::Disabled => None,
        }
    }
}

/// What MCLK, and through it the CPU and the whole bus tree, runs from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MclkSource {
    /// SYSOSC, optionally divided.
    ///
    /// `divider` drives `MCLKCFG.MDIV` and may only be anything other than 1 when SYSOSC is fixed
    /// at 4 MHz, giving 250 kHz to 4 MHz. It is also incompatible with MFCLK.
    Sysosc {
        /// 1 to 16.
        divider: u8,
    },

    /// LFCLK, putting the device in the RUN1/RUN2 low-power policy at 32.768 kHz.
    Lfclk,

    /// The high-speed mux, which is SYSPLL or HFCLK. Only reachable where one of those exists.
    #[cfg(mspm0_hfclk)]
    Hsclk,
}

/// Divider from MCLK to ULPCLK, the PD0 bus clock.
///
/// Only applies when MCLK is sourced from HSCLK; otherwise ULPCLK follows MCLK exactly. Only exists
/// on families with `MCLKCFG.UDIV`, which are the ones whose ULPCLK ceiling is below their MCLK
/// ceiling; elsewhere ULPCLK always equals MCLK.
#[cfg(mspm0_ulpclk_div)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UlpclkDiv {
    /// ULPCLK equals MCLK.
    Div1,

    /// ULPCLK is half MCLK. Needed on parts whose ULPCLK ceiling is below their MCLK ceiling.
    Div2,
}

#[cfg(mspm0_ulpclk_div)]
impl UlpclkDiv {
    const fn divisor(self) -> u32 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
        }
    }
}

/// Where the 32.768 kHz LFCLK comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LfclkSource {
    /// The internal low-frequency oscillator, running by default out of reset.
    Lfosc,

    /// An external 32.768 kHz crystal on LFXIN/LFXOUT.
    ///
    /// Selecting it is irreversible until the next BOR.
    #[cfg(mspm0_lfxt)]
    Lfxt {
        /// Drive strength, 0 (lowest) to 3 (highest). The reset default is the highest.
        drive: u8,

        /// Set for crystals with a load capacitance below 3 pF.
        low_cap: bool,
    },

    /// A digital 32 kHz clock on LFCLK_IN. Mutually exclusive with the crystal.
    #[cfg(mspm0_lfclk_in)]
    External,
}

/// The high-frequency clock, either a crystal or a digital input.
#[cfg(mspm0_hfclk)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HfclkConfig {
    /// Whether HFCLK is driven by a crystal or fed in digitally.
    pub source: HfclkSource,

    /// The frequency present, 4 to 48 MHz.
    pub frequency: u32,

    /// How long the crystal needs to settle, in microseconds.
    ///
    /// Rounded up to the 64 µs the `HFXTTIME` field counts in. Ignored for a digital input.
    pub startup_us: u32,
}

/// Whether HFCLK is a crystal or an external digital clock.
///
/// These are separate hardware, and a device can have either without the other: MSPM0C110x accepts
/// a digital HFCLK but has no crystal driver, so only [`HfclkSource::External`] exists there.
#[cfg(mspm0_hfclk)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HfclkSource {
    /// A crystal across HFXIN/HFXOUT, driven by HFXT.
    #[cfg(mspm0_hfxt)]
    Crystal,

    /// A digital clock on HFCLK_IN.
    #[cfg(mspm0_hfclk_in)]
    External,
}

/// What the SYSPLL multiplies up from.
#[cfg(mspm0_syspll)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SysPllRef {
    /// SYSOSC at its base frequency.
    Sysosc,

    /// HFCLK, which must itself be configured.
    Hfclk,
}

/// Which SYSPLL output drives the HSCLK mux, and through it MCLK.
///
/// `SYSPLLCLK1` is deliberately absent: `SYSPLLCFG0.MCLK2XVCO` is a single bit selecting CLK0 or
/// CLK2X, so CLK1 cannot reach MCLK. It is still configurable as an output, where it feeds CANCLK.
#[cfg(mspm0_syspll)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SysPllTap {
    /// `SYSPLLCLK0`, the VCO divided by an even divider. Selected by `MCLK2XVCO = 0`.
    Clk0,

    /// `SYSPLLCLK2X`, the doubled VCO divided by an integer divider. Selected by `MCLK2XVCO = 1`,
    /// and the tap TI's own 80 MHz configuration uses for MCLK.
    Clk2x,
}

/// SYSPLL setup.
///
/// `fVCO = reference / pdiv * (qdiv + 1)`, and each output tap then divides that down. The
/// resolver checks every intermediate against the datasheet ranges, so an unbuildable combination
/// is a compile error when the configuration is a constant.
#[cfg(mspm0_syspll)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SysPllConfig {
    /// What the loop multiplies up from.
    pub reference: SysPllRef,

    /// Reference predivider: 1, 2, 4 or 8.
    pub pdiv: u8,

    /// Feedback divider, 2 to 127. The register holds this minus one.
    pub qdiv: u8,

    /// `SYSPLLCLK0` divider: an even number from 2 to 32, or `None` to leave the tap off.
    pub clk0_div: Option<u8>,

    /// `SYSPLLCLK1` divider: an even number from 2 to 32, or `None` to leave the tap off.
    pub clk1_div: Option<u8>,

    /// `SYSPLLCLK2X` divider: 1 to 16, or `None` to leave the tap off.
    pub clk2x_div: Option<u8>,

    /// Which tap reaches the HSCLK mux. It must be one that is enabled.
    pub mclk_tap: SysPllTap,
}

/// A clock tree to program.
///
/// [`Config::new`] is the reset tree: SYSOSC at its base frequency driving MCLK, LFCLK from the
/// internal oscillator, and MFCLK on. The `with_*` methods are `const fn`, so a whole configuration
/// can be built in a constant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// What SYSOSC runs at.
    pub sysosc: Sysosc,

    /// What MCLK runs from.
    pub mclk: MclkSource,

    /// Divider from MCLK to ULPCLK. Only has an effect on an HSCLK-sourced MCLK.
    #[cfg(mspm0_ulpclk_div)]
    pub ulpclk_div: UlpclkDiv,

    /// Where LFCLK comes from.
    pub lfclk: LfclkSource,

    /// Whether the 4 MHz MFCLK is enabled for peripheral use. Requires SYSOSC and `MDIV = 0`.
    pub mfclk: bool,

    /// The high-frequency clock, if any.
    #[cfg(mspm0_hfclk)]
    pub hfclk: Option<HfclkConfig>,

    /// The SYSPLL, if any.
    #[cfg(mspm0_syspll)]
    pub syspll: Option<SysPllConfig>,
}

impl Config {
    /// The tree the device comes out of reset with, plus MFCLK, which most drivers expect.
    pub const fn new() -> Self {
        Self {
            sysosc: Sysosc::Base,
            mclk: MclkSource::Sysosc { divider: 1 },
            #[cfg(mspm0_ulpclk_div)]
            ulpclk_div: UlpclkDiv::Div1,
            lfclk: LfclkSource::Lfosc,
            mfclk: true,
            #[cfg(mspm0_hfclk)]
            hfclk: None,
            #[cfg(mspm0_syspll)]
            syspll: None,
        }
    }

    /// Set what MCLK runs from.
    pub const fn with_mclk(mut self, mclk: MclkSource) -> Self {
        self.mclk = mclk;
        self
    }

    /// Set what SYSOSC runs at.
    pub const fn with_sysosc(mut self, sysosc: Sysosc) -> Self {
        self.sysosc = sysosc;
        self
    }

    /// Set the MCLK-to-ULPCLK divider.
    #[cfg(mspm0_ulpclk_div)]
    pub const fn with_ulpclk_div(mut self, div: UlpclkDiv) -> Self {
        self.ulpclk_div = div;
        self
    }

    /// Set where LFCLK comes from.
    pub const fn with_lfclk(mut self, lfclk: LfclkSource) -> Self {
        self.lfclk = lfclk;
        self
    }

    /// Enable or disable MFCLK.
    ///
    /// It is on out of reset and costs current whether or not anything selects it, so a low-power
    /// tree usually turns it off. Some choices require that: it cannot coexist with
    /// [`MclkSource::Sysosc`]'s divider.
    pub const fn with_mfclk(mut self, mfclk: bool) -> Self {
        self.mfclk = mfclk;
        self
    }

    /// Configure the high-frequency clock.
    #[cfg(mspm0_hfclk)]
    pub const fn with_hfclk(mut self, hfclk: HfclkConfig) -> Self {
        self.hfclk = Some(hfclk);
        self
    }

    /// Configure the SYSPLL.
    #[cfg(mspm0_syspll)]
    pub const fn with_syspll(mut self, syspll: SysPllConfig) -> Self {
        self.syspll = Some(syspll);
        self
    }

    /// Validate this configuration and pair it with the rates it produces.
    ///
    /// Prefer this over [`Self::resolve`]: a [`ClockSetup`] built in a `const` carries its answers
    /// with it, so an unbuildable tree is a compile error rather than a runtime panic.
    ///
    /// ```no_run
    /// # #![no_std]
    /// # #[panic_handler]
    /// # fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
    /// use embassy_mspm0::sysctl::clock::{ClockSetup, Config};
    ///
    /// const SETUP: ClockSetup = Config::new().build();
    /// # fn main() {}
    /// ```
    ///
    /// # Panics
    /// If the configuration cannot be realised; use [`Self::try_build`] to handle that at runtime.
    pub const fn build(self) -> ClockSetup {
        match self.try_build() {
            Ok(setup) => setup,
            Err(_) => core::panic!("this clock configuration cannot be realised on this chip"),
        }
    }

    /// [`Self::build`], reporting why the tree cannot be realised instead of panicking.
    pub const fn try_build(self) -> Result<ClockSetup, ClockError> {
        match self.resolve() {
            Ok(clocks) => Ok(ClockSetup { config: self, clocks }),
            Err(err) => Err(err),
        }
    }

    /// Work out every rate this configuration produces, or why it cannot be built.
    ///
    /// Pure arithmetic on `u32` with no register access, so this is usable in a `const` item.
    pub const fn resolve(&self) -> Result<Clocks, ClockError> {
        let sysosc = self.sysosc.frequency();

        // MFCLK is divided down from SYSOSC by SYSCTL, so it needs SYSOSC running, and the TRM
        // requires MDIV to be zero whenever it is enabled.
        if self.mfclk {
            if sysosc.is_none() {
                return Err(ClockError::SourceDisabled);
            }

            if let MclkSource::Sysosc { divider } = self.mclk
                && divider != 1
            {
                return Err(ClockError::DividerNotAllowed);
            }
        }

        let hfclk = match self.hfclk_frequency() {
            Ok(hfclk) => hfclk,
            Err(err) => return Err(err),
        };

        let pll = match self.pll_outputs(sysosc, hfclk) {
            Ok(pll) => pll,
            Err(err) => return Err(err),
        };

        let mclk = match self.mclk_frequency(sysosc, hfclk, &pll) {
            Ok(mclk) => mclk,
            Err(err) => return Err(err),
        };

        if mclk > MAX_MCLK_HZ {
            return Err(ClockError::MclkTooFast);
        }

        // UDIV only sits between HSCLK and ULPCLK; every other MCLK source drives both alike, and
        // families without the divider always run ULPCLK at MCLK.
        #[cfg(mspm0_ulpclk_div)]
        let ulpclk = if self.mclk_is_hsclk() {
            mclk / self.ulpclk_div.divisor()
        } else {
            mclk
        };
        #[cfg(not(mspm0_ulpclk_div))]
        let ulpclk = mclk;

        if ulpclk > MAX_ULPCLK_HZ {
            return Err(ClockError::UlpclkTooFast);
        }

        Ok(Clocks {
            mclk,
            ulpclk,
            lfclk: LFCLK_HZ,
            sysosc: match sysosc {
                Some(hz) => hz,
                None => 0,
            },
            mfclk: if self.mfclk { MFCLK_HZ } else { 0 },
            hfclk: match hfclk {
                Some(hz) => hz,
                None => 0,
            },
            syspll_clk0: pll.clk0,
            syspll_clk1: pll.clk1,
            syspll_clk2x: pll.clk2x,
            flash_wait: flash_wait_states(mclk),
        })
    }

    /// Whether MCLK is taken from the high-speed mux.
    #[cfg(mspm0_ulpclk_div)]
    const fn mclk_is_hsclk(&self) -> bool {
        #[cfg(mspm0_hfclk)]
        {
            matches!(self.mclk, MclkSource::Hsclk)
        }
        #[cfg(not(mspm0_hfclk))]
        {
            false
        }
    }

    /// The HFCLK rate, validated against this device's `fHFXT`/`fHFIN`.
    const fn hfclk_frequency(&self) -> Result<Option<u32>, ClockError> {
        #[cfg(mspm0_hfclk)]
        {
            match self.hfclk {
                None => Ok(None),
                Some(hfclk) => {
                    #[cfg(mspm0_hfclk_range)]
                    if hfclk.frequency < HFCLK_MIN_HZ || hfclk.frequency > HFCLK_MAX_HZ {
                        return Err(ClockError::HfxtOutOfRange);
                    }

                    Ok(Some(hfclk.frequency))
                }
            }
        }
        #[cfg(not(mspm0_hfclk))]
        {
            Ok(None)
        }
    }

    /// The three SYSPLL tap frequencies, validated against `fVCO` and the output ranges.
    const fn pll_outputs(
        &self,
        #[allow(unused)] sysosc: Option<u32>,
        #[allow(unused)] hfclk: Option<u32>,
    ) -> Result<PllOutputs, ClockError> {
        #[cfg(mspm0_syspll)]
        {
            let Some(pll) = self.syspll else {
                return Ok(PllOutputs::NONE);
            };

            // The TRM requires SYSOSC at its base frequency whenever the PLL runs, even when HFCLK
            // is what the loop references.
            if !matches!(self.sysosc, Sysosc::Base) {
                return Err(ClockError::SourceDisabled);
            }

            let reference = match pll.reference {
                SysPllRef::Sysosc => sysosc,
                SysPllRef::Hfclk => hfclk,
            };

            let Some(reference) = reference else {
                return Err(ClockError::SourceDisabled);
            };

            let pdiv = match pll.pdiv {
                1 | 2 | 4 | 8 => pll.pdiv as u32,
                _ => return Err(ClockError::DividerOutOfRange),
            };

            // QDIV holds the feedback divider minus one, and zero is explicitly invalid.
            if pll.qdiv < 2 || pll.qdiv > 127 {
                return Err(ClockError::DividerOutOfRange);
            }

            let loop_in = reference / pdiv;
            if loop_in < PLL_REF_MIN_HZ || loop_in > PLL_REF_MAX_HZ {
                return Err(ClockError::PllReferenceOutOfRange);
            }

            // `qdiv` is a field the caller fills in, and a runtime `try_build` reaches here with it
            // unchecked: a plausible reference and a large multiplier wrap rather than being rejected.
            let Some(vco) = loop_in.checked_mul(pll.qdiv as u32) else {
                return Err(ClockError::PllVcoOutOfRange);
            };
            if vco < VCO_MIN_HZ || vco > VCO_MAX_HZ {
                return Err(ClockError::PllVcoOutOfRange);
            }

            let clk0 = match even_tap(vco, pll.clk0_div) {
                Ok(clk) => clk,
                Err(err) => return Err(err),
            };
            let clk1 = match even_tap(vco, pll.clk1_div) {
                Ok(clk) => clk,
                Err(err) => return Err(err),
            };
            let clk2x = match doubled_tap(vco, pll.clk2x_div) {
                Ok(clk) => clk,
                Err(err) => return Err(err),
            };

            Ok(PllOutputs { clk0, clk1, clk2x })
        }
        #[cfg(not(mspm0_syspll))]
        {
            let _ = (sysosc, hfclk);
            Ok(PllOutputs::NONE)
        }
    }

    /// What MCLK ends up at.
    #[allow(unused_variables)]
    const fn mclk_frequency(
        &self,
        sysosc: Option<u32>,
        hfclk: Option<u32>,
        pll: &PllOutputs,
    ) -> Result<u32, ClockError> {
        match self.mclk {
            MclkSource::Sysosc { divider } => {
                let Some(sysosc) = sysosc else {
                    return Err(ClockError::SourceDisabled);
                };

                if divider == 0 || divider > 16 {
                    return Err(ClockError::DividerOutOfRange);
                }

                // MDIV is only legal against a SYSOSC pinned at 4 MHz.
                if divider != 1 && !matches!(self.sysosc, Sysosc::Mhz4) {
                    return Err(ClockError::DividerNotAllowed);
                }

                Ok(sysosc / divider as u32)
            }

            MclkSource::Lfclk => Ok(LFCLK_HZ),

            #[cfg(mspm0_hfclk)]
            MclkSource::Hsclk => {
                // The PLL wins the HSCLK mux when both are configured, matching HSCLKSEL's reset
                // value and TI's own 80 MHz recipe.
                #[cfg(mspm0_syspll)]
                if self.syspll.is_some() {
                    let tap = match unwrap_syspll_tap(&self.syspll) {
                        SysPllTap::Clk0 => pll.clk0,
                        SysPllTap::Clk2x => pll.clk2x,
                    };

                    // A tap reading 0 was left disabled, so it cannot drive MCLK.
                    return if tap == 0 {
                        Err(ClockError::PllOutputOutOfRange)
                    } else {
                        Ok(tap)
                    };
                }

                let _ = pll;

                match hfclk {
                    Some(hz) => Ok(hz),
                    None => Err(ClockError::NoHsclkSource),
                }
            }
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Which tap the PLL drives MCLK from, in a form the const resolver can reach.
#[cfg(mspm0_syspll)]
const fn unwrap_syspll_tap(syspll: &Option<SysPllConfig>) -> SysPllTap {
    match syspll {
        Some(pll) => pll.mclk_tap,
        // Only called behind an `is_some` check.
        None => SysPllTap::Clk2x,
    }
}

/// A `SYSPLLCLK0`/`SYSPLLCLK1` tap: the VCO over an even divider from 2 to 32.
#[cfg(mspm0_syspll)]
const fn even_tap(vco: u32, div: Option<u8>) -> Result<u32, ClockError> {
    let Some(div) = div else {
        return Ok(0);
    };

    if div < 2 || div > 32 || div % 2 != 0 {
        return Err(ClockError::DividerOutOfRange);
    }

    let hz = vco / div as u32;
    if hz < PLL_CLK_MIN_HZ || hz > PLL_CLK_MAX_HZ {
        return Err(ClockError::PllOutputOutOfRange);
    }

    Ok(hz)
}

/// The `SYSPLLCLK2X` tap: twice the VCO over a divider from 1 to 16.
#[cfg(mspm0_syspll)]
const fn doubled_tap(vco: u32, div: Option<u8>) -> Result<u32, ClockError> {
    let Some(div) = div else {
        return Ok(0);
    };

    if div < 1 || div > 16 {
        return Err(ClockError::DividerOutOfRange);
    }

    // The VCO tops out at 400 MHz, so doubling it stays inside a u32.
    let hz = (vco * 2) / div as u32;
    if hz < PLL_CLK2X_MIN_HZ || hz > PLL_CLK2X_MAX_HZ {
        return Err(ClockError::PllOutputOutOfRange);
    }

    Ok(hz)
}

/// The SYSPLL tap frequencies, or `None` per tap where it is disabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PllOutputs {
    clk0: u32,
    clk1: u32,
    clk2x: u32,
}

impl PllOutputs {
    const NONE: Self = Self {
        clk0: 0,
        clk1: 0,
        clk2x: 0,
    };
}

/// Flash wait states needed to run at `mclk`.
///
/// The bands are per device: three on the G-series (24/48/80 MHz), two on most of the L-series
/// (24/32), one on MSPM0C1103/C1104.
///
/// SYSCTL programs these itself for a SYSOSC- or LFCLK-sourced MCLK, but not for one taken from
/// HSCLK — the case a configured clock tree introduces.
const fn flash_wait_states(mclk: u32) -> u8 {
    let bands = crate::_generated::FLASH_WAIT_HZ;

    let mut wait = 0;
    while wait < bands.len() {
        if mclk <= bands[wait] {
            return wait as u8;
        }

        wait += 1;
    }

    // Past the deepest band is past `MAX_MCLK_HZ`, which `resolve` rejects before reaching here.
    (bands.len() - 1) as u8
}

/// The reset tree, already validated.
///
/// This is what [`crate::Config::default`] uses, so a program that does not configure clocks pays
/// nothing for the validation machinery.
pub const RESET_SETUP: ClockSetup = Config::new().build();

/// A [`Config`] that has been validated, together with the rates it produces.
///
/// Built by [`Config::build`]. Doing that in a `const` keeps every range check out of the binary,
/// leaving [`crate::init`] with only the register writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockSetup {
    pub(crate) config: Config,
    pub(crate) clocks: Clocks,
}

impl ClockSetup {
    /// The rates this setup produces.
    pub const fn clocks(&self) -> Clocks {
        self.clocks
    }

    /// The configuration it was built from.
    pub const fn config(&self) -> Config {
        self.config
    }
}

impl ClockSetup {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Config::new().build()
    }
}

impl Default for ClockSetup {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Program `setup` into SYSCTL and report the rates it produced.
///
/// Ordering follows SLAU846's SYSPLL bring-up (2.3.6): each source is up and good before anything
/// depends on it, flash wait states widen before MCLK speeds up, the MCLK mux switches once, and
/// SYSOSC powers down last because the sources configured before it may need it running.
///
/// Called once from [`crate::init`] while it holds the critical section.
#[inline(always)]
pub(crate) fn apply(setup: &ClockSetup) -> Result<Clocks, ClockError> {
    // Already validated by `Config::build`, so nothing here re-derives or re-checks anything.
    let config = &setup.config;
    let clocks = setup.clocks;
    let sysctl = pac::SYSCTL;

    // LFCLK first: it is the one clock nothing else can substitute for, and selecting a crystal is
    // irreversible until the next BOR. MCLK is still on the reset SYSOSC here.
    apply_lfclk(config, SYSOSC_BASE_HZ)?;

    // An asynchronous fast clock request forces SYSOSC to base and MCLK onto it, keeping MDIV. A tree
    // that is already there is unaffected, and one on HSCLK is ignored outright; anything slower gets
    // overridden.
    let boost_overrides_tree = match config.mclk {
        MclkSource::Sysosc { divider } => divider != 1 || !matches!(config.sysosc, Sysosc::Base),
        MclkSource::Lfclk => true,
        #[cfg(mspm0_hfclk)]
        MclkSource::Hsclk => false,
    };

    // SYSOSC's frequency field may only be written while SYSOSC is what drives MCLK, which is the
    // case here because MCLK has not been switched yet.
    sysctl.sysosccfg().modify(|w| {
        w.set_freq(match config.sysosc {
            Sysosc::Base | Sysosc::Disabled => vals::SysosccfgFreq::Sysoscbase,
            Sysosc::Mhz4 => vals::SysosccfgFreq::Sysosc4m,
        });

        // `FASTCPUEVENT` resets set, making every interrupt raise a request, so a slow tree would not
        // survive its first one. `BLOCKASYNCALL` stops these too but is the wrong knob: peripheral
        // requests are what wake a TIMGx from STANDBY1 and what keeps `UART_ERR_04` out of reach.
        w.set_fastcpuevent(!boost_overrides_tree);
    });

    // From here until MCLK is switched the core runs on SYSOSC at whatever was just selected, which
    // is what the settle timeouts are measured against.
    let cpu_hz = match config.sysosc {
        Sysosc::Mhz4 => 4_000_000,
        // Disabled still leaves SYSOSC at base until it is switched off at the very end.
        Sysosc::Base | Sysosc::Disabled => SYSOSC_BASE_HZ,
    };

    apply_hfclk(config, cpu_hz)?;
    apply_syspll(config, &clocks, cpu_hz)?;

    // Widen the flash wait states before MCLK speeds up. SYSCTL manages these itself for a SYSOSC-
    // or LFCLK-sourced MCLK, but not for one taken from HSCLK. The C-series has no such field, and
    // no HSCLK to need it.
    #[cfg(mspm0_flashwait)]
    let target_wait = match clocks.flash_wait {
        0 => vals::Flashwait::Wait0,
        1 => vals::Flashwait::Wait1,
        _ => vals::Flashwait::Wait2,
    };

    // Read once and reuse below: nothing but this function writes the field, and if the widen
    // below fires then the narrow after the switch cannot, so the stale value is never wrong.
    #[cfg(mspm0_flashwait)]
    let current_wait = sysctl.mclkcfg().read().flashwait().to_bits();

    #[cfg(mspm0_flashwait)]
    if clocks.flash_wait > current_wait {
        sysctl.mclkcfg().modify(|w| w.set_flashwait(target_wait));
    }

    // Switch MCLK, and with it ULPCLK, to its configured source.
    sysctl.mclkcfg().modify(|w| {
        match config.mclk {
            MclkSource::Sysosc { divider } => {
                #[cfg(mspm0_hfclk)]
                w.set_usehsclk(false);
                w.set_uselfclk(false);
                // MDIV holds the divider minus one, and is only legal against a 4 MHz SYSOSC.
                w.set_mdiv(divider - 1);
            }
            MclkSource::Lfclk => {
                #[cfg(mspm0_hfclk)]
                w.set_usehsclk(false);
                w.set_uselfclk(true);
                w.set_mdiv(0);
            }
            #[cfg(mspm0_hfclk)]
            MclkSource::Hsclk => {
                w.set_uselfclk(false);
                w.set_usehsclk(true);
                w.set_mdiv(0);
            }
        }

        #[cfg(mspm0_ulpclk_div)]
        w.set_udiv(match config.ulpclk_div {
            UlpclkDiv::Div1 => vals::Udiv::Nodivide,
            UlpclkDiv::Div2 => vals::Udiv::Divide2,
        });

        w.set_usemftick(config.mfclk);
    });

    // Now that MCLK has slowed down, the wait states can come back in.
    #[cfg(mspm0_flashwait)]
    if clocks.flash_wait < current_wait {
        sysctl.mclkcfg().modify(|w| w.set_flashwait(target_wait));
    }

    // MFPCLK feeds CLK_OUT and the DAC, and is what the existing drivers expect to be available.
    sysctl.genclken().modify(|w| w.set_mfpclken(config.mfclk));

    // Only safe once nothing above still depends on SYSOSC.
    if matches!(config.sysosc, Sysosc::Disabled) {
        sysctl.sysosccfg().modify(|w| w.set_disable(true));
    }

    Ok(clocks)
}

/// Select the LFCLK source, waiting for a crystal to settle.
#[allow(unused_variables, unused_mut)]
#[inline(always)]
fn apply_lfclk(config: &Config, cpu_hz: u32) -> Result<(), ClockError> {
    #[allow(unused)]
    let sysctl = pac::SYSCTL;

    match config.lfclk {
        // Running out of reset; nothing to do.
        LfclkSource::Lfosc => {}

        #[cfg(mspm0_lfxt)]
        LfclkSource::Lfxt { drive, low_cap } => {
            if drive > 3 {
                return Err(ClockError::DividerOutOfRange);
            }

            sysctl.lfclkcfg().modify(|w| {
                w.set_xt1drive(vals::Xt1drive::from_bits(drive));
                w.set_lowcap(low_cap);
            });

            // Write-only trigger registers, so these are whole writes rather than read-modify-write.
            sysctl.lfxtctl().write(|w| w.set_startlfxt(true));
            settle(cpu_hz, || pac::SYSCTL.clkstatus().read().lfxtgood())?;

            // Irreversible until the next BOR. Keep the oscillator started in the same write.
            sysctl.lfxtctl().write(|w| {
                w.set_startlfxt(true);
                w.set_setuselfxt(true);
            });
        }

        #[cfg(mspm0_lfclk_in)]
        LfclkSource::External => {
            sysctl.exlfctl().write(|w| w.set_setuseexlf(true));
        }
    }

    Ok(())
}

/// Start HFXT or accept an external HFCLK, waiting for it to be good.
#[inline(always)]
fn apply_hfclk(config: &Config, #[allow(unused)] cpu_hz: u32) -> Result<(), ClockError> {
    #[cfg(mspm0_hfclk)]
    {
        let sysctl = pac::SYSCTL;

        let Some(hfclk) = config.hfclk else {
            return Ok(());
        };

        match hfclk.source {
            #[cfg(mspm0_hfxt)]
            HfclkSource::Crystal => {
                sysctl.hfclkclkcfg().modify(|w| {
                    // Written as disjoint bands rather than a cascade of upper bounds: the arms then
                    // say the same thing the variants do, and none is shadowed by the one above it.
                    w.set_hfxtrsel(match hfclk.frequency {
                        ..=8_000_000 => vals::Hfxtrsel::Range4to8,
                        8_000_001..=16_000_000 => vals::Hfxtrsel::Range8to16,
                        16_000_001..=32_000_000 => vals::Hfxtrsel::Range16to32,
                        _ => vals::Hfxtrsel::Range32to48,
                    });

                    // HFXTTIME counts in 64 us units; round up so the wait is never short.
                    w.set_hfxttime(hfclk.startup_us.div_ceil(64).min(u8::MAX as u32) as u8);
                });

                sysctl.hsclken().modify(|w| w.set_hfxten(true));
            }

            #[cfg(mspm0_hfclk_in)]
            HfclkSource::External => {
                sysctl.hsclken().modify(|w| w.set_useexthfclk(true));
            }
        }

        // `CLKSTATUS.HFCLKGOOD` exists on exactly the blocks that have a crystal driver, and doubles
        // as the TRM's stuck-clock check for a digital input. Where it is absent there is nothing to
        // wait for: the only source is a digital clock, which needs no startup time.
        #[cfg(mspm0_hfxt)]
        settle(cpu_hz, || pac::SYSCTL.clkstatus().read().hfclkgood())?;

        Ok(())
    }
    #[cfg(not(mspm0_hfclk))]
    {
        let _ = (config, cpu_hz);
        Ok(())
    }
}

/// Bring the SYSPLL up and point the HSCLK mux at it.
#[inline(always)]
fn apply_syspll(config: &Config, #[allow(unused)] clocks: &Clocks, cpu_hz: u32) -> Result<(), ClockError> {
    #[cfg(mspm0_syspll)]
    {
        let sysctl = pac::SYSCTL;

        let Some(pll) = config.syspll else {
            // With no PLL configured the mux must take HFCLK instead.
            if config.hfclk.is_some() {
                sysctl.hsclkcfg().modify(|w| w.set_hsclksel(vals::Hsclksel::Hfclkclk));
            }

            return Ok(());
        };

        sysctl.syspllcfg0().modify(|w| {
            w.set_syspllref(match pll.reference {
                SysPllRef::Sysosc => vals::Syspllref::Sysosc,
                SysPllRef::Hfclk => vals::Syspllref::Hfclk,
            });

            w.set_enableclk0(pll.clk0_div.is_some());
            w.set_enableclk1(pll.clk1_div.is_some());
            w.set_enableclk2x(pll.clk2x_div.is_some());

            if let Some(div) = pll.clk0_div {
                w.set_rdivclk0(vals::Rdivclk0::from_bits(div / 2 - 1));
            }
            if let Some(div) = pll.clk1_div {
                w.set_rdivclk1(vals::Rdivclk1::from_bits(div / 2 - 1));
            }
            if let Some(div) = pll.clk2x_div {
                w.set_rdivclk2x(vals::Rdivclk2x::from_bits(div - 1));
            }

            w.set_mclk2xvco(matches!(pll.mclk_tap, SysPllTap::Clk2x));
        });

        sysctl.syspllcfg1().modify(|w| {
            w.set_pdiv(match pll.pdiv {
                1 => vals::Pdiv::Refdiv1,
                2 => vals::Pdiv::Refdiv2,
                4 => vals::Pdiv::Refdiv4,
                _ => vals::Pdiv::Refdiv8,
            });

            // QDIV holds the feedback divider minus one.
            w.set_qdiv(vals::Qdiv::from_bits(pll.qdiv - 1));
        });

        // The loop's analog tuning depends on its input frequency, and reset loads none of the four
        // bands' values. Without this the VCO locks off frequency and still reports SYSPLLGOOD.
        let reference = match pll.reference {
            SysPllRef::Sysosc => clocks.sysosc,
            SysPllRef::Hfclk => clocks.hfclk,
        };

        let factory = pac::FACTORYREGION;
        let (param0, param1) = match reference / pll.pdiv as u32 {
            32_000_000.. => (
                factory.pllstartup0_32_48mhz().read().0,
                factory.pllstartup1_32_48mhz().read().0,
            ),
            16_000_000.. => (
                factory.pllstartup0_16_32mhz().read().0,
                factory.pllstartup1_16_32mhz().read().0,
            ),
            8_000_000.. => (
                factory.pllstartup0_8_16mhz().read().0,
                factory.pllstartup1_8_16mhz().read().0,
            ),
            // `resolve` rejects a loop input below 4 MHz, so this is the 4-8 MHz band.
            _ => (
                factory.pllstartup0_4_8mhz().read().0,
                factory.pllstartup1_4_8mhz().read().0,
            ),
        };

        sysctl
            .syspllparam0()
            .write_value(pac::sysctl::regs::Syspllparam0(param0));
        sysctl
            .syspllparam1()
            .write_value(pac::sysctl::regs::Syspllparam1(param1));

        sysctl.hsclken().modify(|w| w.set_syspllen(true));
        settle(cpu_hz, || pac::SYSCTL.clkstatus().read().syspllgood())?;

        sysctl.hsclkcfg().modify(|w| w.set_hsclksel(vals::Hsclksel::Syspll));

        Ok(())
    }
    #[cfg(not(mspm0_syspll))]
    {
        // There is no PLL to bring up, but the mux still has to be pointed at HFCLK. `HSCLKCFG`
        // resets to zero, which is the SYSPLL position — a source these devices do not have — and
        // every TRM says "to change the HSCLK source to HFCLK, set the HSCLKSEL bit". Leaving it
        // alone leaves HSCLK unsourced, and SYSCTL then refuses to switch MCLK to it.
        //
        // `mspm0_hsclk_mux` is what says the field exists: `c110x` has an HFCLK path but no
        // `HSCLKCFG` at all, and there HSCLK really is HFCLK with nothing to select.
        #[cfg(all(mspm0_hfclk, mspm0_hsclk_mux))]
        if config.hfclk.is_some() {
            pac::SYSCTL
                .hsclkcfg()
                .modify(|w| w.set_hsclksel(vals::Hsclksel::Hfclkclk));
        }

        let _ = (config, cpu_hz);
        Ok(())
    }
}

/// Spin until `ready` reports the source has come up, giving up after [`SETTLE_TIMEOUT_US`].
///
/// Counted in CPU cycles against `cpu_hz` so the timeout is the same duration on every part.
///
/// It cannot use [`embassy_time`](crate::time_driver): this runs during [`crate::init`], before the
/// time driver starts, and that driver is itself clocked by the tree being configured here.
#[cfg(any(mspm0_hfxt, mspm0_lfxt, mspm0_syspll))]
fn settle(cpu_hz: u32, ready: impl Fn() -> bool) -> Result<(), ClockError> {
    // `cpu_hz / 1_000_000` first: the multiplication would otherwise overflow for any real clock.
    let mut remaining = (cpu_hz / 1_000_000).max(1) * SETTLE_TIMEOUT_US;

    loop {
        if ready() {
            return Ok(());
        }

        if remaining < SETTLE_STEP_CYCLES {
            return Err(ClockError::SourceDidNotSettle);
        }

        cortex_m::asm::delay(SETTLE_STEP_CYCLES);
        remaining -= SETTLE_STEP_CYCLES;
    }
}

/// Every rate the configured tree produces.
///
/// Read it with [`crate::sysctl::clocks`]. A rate whose source is switched off reads as **0** rather
/// than being an `Option`: it keeps the struct half the size, which matters because it is copied out
/// of a static on every read, and a peripheral told to use a stopped clock is not running anyway.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Clocks {
    /// The PD1 bus clock, and the CPU clock in RUN.
    pub mclk: u32,

    /// The PD0 bus clock.
    pub ulpclk: u32,

    /// The only clock that survives STANDBY.
    pub lfclk: u32,

    /// SYSOSC, or 0 when it is powered down.
    pub sysosc: u32,

    /// The 4 MHz peripheral clock, or 0 when disabled.
    pub mfclk: u32,

    /// HFCLK, or 0 when no high-frequency source is configured.
    pub hfclk: u32,

    /// `SYSPLLCLK0`, or 0 when the tap is off.
    pub syspll_clk0: u32,

    /// `SYSPLLCLK1`, or 0 when the tap is off. This is what CANCLK can take.
    pub syspll_clk1: u32,

    /// `SYSPLLCLK2X`, or 0 when the tap is off.
    pub syspll_clk2x: u32,

    /// Flash wait states this MCLK requires.
    pub flash_wait: u8,
}

impl Clocks {
    /// The tree the device runs on before [`crate::init`], which is the reset configuration.
    ///
    /// Reading a rate before initialisation gives this rather than panicking.
    pub const RESET: Self = match Config::new().resolve() {
        Ok(clocks) => clocks,
        // The reset configuration is within range on every supported part, which the const
        // assertions below check.
        Err(_) => core::panic!("the reset clock tree must always resolve"),
    };

    /// The rate an instance in `domain` sees when it selects the bus clock.
    ///
    /// [`PowerDomain::Backup`](super::PowerDomain::Backup) answers ULPCLK: its logic runs from
    /// LFCLK, but its registers are reached over the PD0 bus.
    pub const fn bus_clock(&self, domain: super::PowerDomain) -> u32 {
        match domain {
            super::PowerDomain::Pd1 => self.mclk,
            super::PowerDomain::Pd0 | super::PowerDomain::Backup => self.ulpclk,
        }
    }
}

// The reset tree must resolve on every part, and must land where the generated rates say.
//
// **This constrains the resolver, not the rates.** `SYSOSC_BASE_HZ` and its siblings come from device
// metadata, and `resolve` derives from the same constants, so a metapac bump moves both sides
// together and this cannot fire on one. What it does catch is `resolve` growing a step that shifts
// the reset tree away from the untouched-device rates -- which is what it is for, and is less than
// the phrase "the old fixed constants" implied when those were still written here.
const _: () = {
    let reset = Clocks::RESET;

    core::assert!(reset.mclk == SYSOSC_BASE_HZ);
    core::assert!(reset.ulpclk == SYSOSC_BASE_HZ || reset.ulpclk <= MAX_ULPCLK_HZ);
    core::assert!(reset.lfclk == LFCLK_HZ);
    core::assert!(reset.mfclk == MFCLK_HZ);

    // Wait-state bands. Where they fall is per device, so check the shape rather than the numbers:
    // every band's ceiling selects that band, one past it selects the next, and the deepest covers
    // MCLK's ceiling.
    let bands = crate::_generated::FLASH_WAIT_HZ;

    let mut wait = 0;
    while wait < bands.len() {
        core::assert!(flash_wait_states(bands[wait]) == wait as u8);

        if wait + 1 < bands.len() {
            core::assert!(flash_wait_states(bands[wait] + 1) == wait as u8 + 1);
        }

        wait += 1;
    }

    core::assert!(flash_wait_states(MAX_MCLK_HZ) == bands.len() as u8 - 1);

    // MFCLK cannot be combined with an MCLK divider.
    core::assert!(matches!(
        Config::new()
            .with_sysosc(Sysosc::Mhz4)
            .with_mclk(MclkSource::Sysosc { divider: 2 })
            .resolve(),
        Err(ClockError::DividerNotAllowed)
    ));

    // ...but is fine once MFCLK is off.
    let mut divided = Config::new()
        .with_sysosc(Sysosc::Mhz4)
        .with_mclk(MclkSource::Sysosc { divider: 2 });
    divided.mfclk = false;
    core::assert!(matches!(divided.resolve(), Ok(clocks) if clocks.mclk == 2_000_000));

    // A divider against a SYSOSC that is not pinned at 4 MHz is rejected.
    let mut base_divided = Config::new().with_mclk(MclkSource::Sysosc { divider: 2 });
    base_divided.mfclk = false;
    core::assert!(matches!(base_divided.resolve(), Err(ClockError::DividerNotAllowed)));

    // MFCLK needs SYSOSC.
    core::assert!(matches!(
        Config::new().with_sysosc(Sysosc::Disabled).resolve(),
        Err(ClockError::SourceDisabled)
    ));

    // LFCLK-sourced MCLK is the RUN1/RUN2 policy.
    core::assert!(matches!(
        Config::new().with_mclk(MclkSource::Lfclk).resolve(),
        Ok(clocks) if clocks.mclk == LFCLK_HZ && clocks.ulpclk == LFCLK_HZ
    ));
};

// HSCLK-specific checks, only meaningful where there is a high-speed source.
#[cfg(mspm0_hfclk)]
const _: () = {
    // Asking for HSCLK without configuring one is caught.
    core::assert!(matches!(
        Config::new().with_mclk(MclkSource::Hsclk).resolve(),
        Err(ClockError::NoHsclkSource)
    ));
};

// A crystal outside the device's input range is rejected. Only checkable where the datasheet gives
// one: 50 MHz is above `fHFXT` on every part that has a crystal, since the widest is 4-48 MHz.
#[cfg(all(mspm0_hfxt, mspm0_hfclk_range))]
const _: () = {
    core::assert!(matches!(
        Config::new()
            .with_hfclk(HfclkConfig {
                source: HfclkSource::Crystal,
                frequency: 50_000_000,
                startup_us: 1_000,
            })
            .resolve(),
        Err(ClockError::HfxtOutOfRange)
    ));
};

#[cfg(mspm0_syspll)]
const _: () = {
    // TI's own 80 MHz recipe from the G-series TRM: SYSOSC 32 MHz reference, PDIV /2 giving a
    // 16 MHz loop input, QDIV 5 giving an 80 MHz VCO, CLK1 /2 for 40 MHz CANCLK and CLK2X /2 for
    // 80 MHz MCLK. ULPCLK has to be halved to stay inside its 40 MHz ceiling.
    let cfg = Config::new()
        .with_syspll(SysPllConfig {
            reference: SysPllRef::Sysosc,
            pdiv: 2,
            qdiv: 5,
            clk0_div: None,
            clk1_div: Some(2),
            clk2x_div: Some(2),
            mclk_tap: SysPllTap::Clk2x,
        })
        .with_mclk(MclkSource::Hsclk)
        .with_ulpclk_div(UlpclkDiv::Div2);

    core::assert!(matches!(
        cfg.resolve(),
        Ok(clocks)
            if clocks.mclk == 80_000_000
            && clocks.ulpclk == 40_000_000
            && clocks.syspll_clk1 == 40_000_000
            && clocks.flash_wait == 2
    ));

    // Leaving ULPCLK undivided at 80 MHz exceeds its ceiling.
    core::assert!(matches!(
        cfg.with_ulpclk_div(UlpclkDiv::Div1).resolve(),
        Err(ClockError::UlpclkTooFast)
    ));

    // A feedback divider that puts the VCO under 80 MHz is rejected.
    core::assert!(matches!(
        Config::new()
            .with_syspll(SysPllConfig {
                reference: SysPllRef::Sysosc,
                pdiv: 2,
                qdiv: 4,
                clk0_div: None,
                clk1_div: None,
                clk2x_div: Some(2),
                mclk_tap: SysPllTap::Clk2x,
            })
            .with_mclk(MclkSource::Hsclk)
            .resolve(),
        Err(ClockError::PllVcoOutOfRange)
    ));

    // Driving MCLK from a tap that was left off is caught.
    core::assert!(matches!(
        Config::new()
            .with_syspll(SysPllConfig {
                reference: SysPllRef::Sysosc,
                pdiv: 2,
                qdiv: 5,
                clk0_div: None,
                clk1_div: None,
                clk2x_div: Some(2),
                mclk_tap: SysPllTap::Clk0,
            })
            .with_mclk(MclkSource::Hsclk)
            .resolve(),
        Err(ClockError::PllOutputOutOfRange)
    ));

    // The PLL requires SYSOSC at base frequency even when HFCLK is the reference.
    core::assert!(matches!(
        Config::new()
            .with_sysosc(Sysosc::Mhz4)
            .with_syspll(SysPllConfig {
                reference: SysPllRef::Sysosc,
                pdiv: 1,
                qdiv: 5,
                clk0_div: None,
                clk1_div: None,
                clk2x_div: Some(2),
                mclk_tap: SysPllTap::Clk2x,
            })
            .resolve(),
        Err(ClockError::SourceDisabled)
    ));
};
