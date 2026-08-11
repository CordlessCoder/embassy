//! Suspending the instruction prefetcher across an idle.
//!
//! Not gated on `low-power`: the erratum this works around applies to a plain `WFI` as much as to a
//! deep sleep, so [`crate::executor`] needs it whether or not the deep-sleep machinery is compiled in.

use crate::pac;

/// Workaround for CPU_ERR_02, CPU_ERR_03, PMCU_ERR_13 - the prefetcher has at least one errata in
/// sleep for every currently supported MCU.
///
/// Ungated because `CPU_ERR_03` covers every family this crate supports, `PMCU_ERR_13` is narrower
/// (STOP2 and STANDBY0), and `CPU_ERR_02` is not a sleep erratum at all — it says a prefetch disable
/// does not take effect while a flash access is pending, which is why the register read below is
/// here.
///
/// # This only covers the sleeps this module performs
///
/// `CPU_ERR_03` is written against "low power modes", not against the deep ones, and MSPM0 counts
/// plain SLEEP among them — so a bare `WFI` or `WFE` is in scope. It also names the wake this matters
/// most for: "a HW Event wake is another example of a process that will wake the device, but not
/// flush the prefetcher."
///
/// **`embassy-executor`'s own Cortex-M executor idles on `WFE` and takes no such guard**, so an
/// application using it rather than [`crate::executor::Executor`] runs the erratum's exposed case on
/// every idle. Whether that is reachable in practice is not established here: the corruption needs
/// the prefetched zeros to survive the wake, and an interrupt handler running from flash is likely to
/// overwrite them, which is why nothing has been seen. That is an argument about likelihood and not a
/// guarantee, and it has not been tested on silicon either way.
pub(crate) struct PrefetchSuspend(pac::cpuss::regs::Ctl);

impl PrefetchSuspend {
    pub(crate) fn new() -> Self {
        let saved = pac::CPUSS.ctl().read();
        let mut disabled = saved;
        disabled.set_prefetch(false);
        pac::CPUSS.ctl().write_value(disabled);

        // CPU_ERR_02 means the prefetcher will not be disabled until pending flash access is finished.
        // Reading any SYSCTL register after disabling prefetch will complete the pending flash access.
        #[cfg(mspm0_shutdnstore)]
        let _ = pac::SYSCTL.shutdnstore(0).read();
        #[cfg(not(mspm0_shutdnstore))]
        let _ = pac::SYSCTL.clkstatus().read();

        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        Self(saved)
    }
}

impl Drop for PrefetchSuspend {
    fn drop(&mut self) {
        pac::CPUSS.ctl().write_value(self.0);
    }
}
