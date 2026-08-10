//! `GENCLKCFG.EXCLKSRC`, the one part of SYSCTL whose shape differs between register-block versions.

use mspm0_metapac::sysctl::vals;

use crate::sysctl::{ClkOutDiv, div_to_pac};

/// Source and configuration for CLK_OUT pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClkOutSource {
    /// Use SYSOSC as the source.
    ///
    /// The divider is optional for this clock source.
    Sysosc(Option<ClkOutDiv>),

    /// Use ULPCLK as the source.
    ///
    /// The divider is required for this clock source.
    UlpClk(ClkOutDiv),

    /// Use LFCLK as the source.
    ///
    /// The divider is optional for this clock source.
    LfClk(Option<ClkOutDiv>),

    /// Use MFPCLK as the source.
    ///
    /// The divider is required for this clock source.
    MfpClk(ClkOutDiv),

    /// Use HFCLK as the source.
    ///
    /// The divider is optional for this clock source.
    #[cfg(mspm0_clkout_hfclk)]
    Hfclk(Option<ClkOutDiv>),

    /// Use SYSPLLCLK1 as the source.
    ///
    /// The divider is optional for this clock source.
    #[cfg(mspm0_clkout_syspllclk1)]
    SysPllClk1(Option<ClkOutDiv>),

    /// Use USBFLL as the source.
    ///
    /// The divider is required for this clock source.
    #[cfg(usbfs)]
    UsbFll(ClkOutDiv),
}

impl ClkOutSource {
    pub(super) fn convert_div(self) -> (bool, vals::Exclkdivval) {
        match self {
            ClkOutSource::Sysosc(div) => div_to_pac(div),
            ClkOutSource::UlpClk(div) => div_to_pac(Some(div)),
            ClkOutSource::LfClk(div) => div_to_pac(div),
            ClkOutSource::MfpClk(div) => div_to_pac(Some(div)),
            #[cfg(mspm0_clkout_hfclk)]
            ClkOutSource::Hfclk(div) => div_to_pac(div),
            #[cfg(mspm0_clkout_syspllclk1)]
            ClkOutSource::SysPllClk1(div) => div_to_pac(div),
            #[cfg(usbfs)]
            ClkOutSource::UsbFll(div) => div_to_pac(Some(div)),
        }
    }

    pub(super) fn convert_src(self) -> vals::Exclksrc {
        match self {
            ClkOutSource::Sysosc(_) => vals::Exclksrc::Sysosc,
            ClkOutSource::UlpClk(_) => vals::Exclksrc::Ulpclk,
            ClkOutSource::LfClk(_) => vals::Exclksrc::Lfclk,
            // The C-series SVDs name position 3 `MFCLK`; it is MFPCLK on every block, so the cfg
            // picks the spelling rather than the meaning.
            #[cfg(mspm0_exclksrc_mfclk_name)]
            ClkOutSource::MfpClk(_) => vals::Exclksrc::Mfclk,
            #[cfg(not(mspm0_exclksrc_mfclk_name))]
            ClkOutSource::MfpClk(_) => vals::Exclksrc::Mfpclk,
            #[cfg(mspm0_clkout_hfclk)]
            ClkOutSource::Hfclk(_) => vals::Exclksrc::Hfclk,
            #[cfg(mspm0_clkout_syspllclk1)]
            ClkOutSource::SysPllClk1(_) => vals::Exclksrc::Syspllout1,
            // Position 6 is the USB FLL, which the SVDs leave reserved, so it is named as reserved
            // here rather than by what it selects.
            #[cfg(usbfs)]
            ClkOutSource::UsbFll(_) => vals::Exclksrc::_RESERVED_6,
        }
    }
}
