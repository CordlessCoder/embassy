#![no_std]
#![allow(unsafe_op_in_unsafe_fn)]
// Doc feature labels can be tested locally by running RUSTDOCFLAGS="--cfg=docsrs" cargo +nightly doc
#![cfg_attr(docsrs, feature(doc_auto_cfg, doc_cfg_hide), doc(cfg_hide(doc, docsrs)))]
#![cfg_attr(
    docsrs,
    doc = "<div style='padding:30px;background:#810;color:#fff;text-align:center;'><p>You might want to <a href='https://docs.embassy.dev/embassy-mspm0'>browse the `embassy-mspm0` documentation on the Embassy website</a> instead.</p><p>The documentation here on `docs.rs` is built for a single chip only, while on the Embassy website you can pick your exact chip from the top menu. Available peripherals and their APIs change depending on the chip.</p></div>\n\n"
)]
#![doc = include_str!("../README.md")]

// These mods MUST go first, so that the others see the macros.
pub(crate) mod fmt;
mod macros;

pub mod adc;
pub mod dma;
#[cfg(feature = "_executor")]
pub mod executor;
pub mod gpio;
// TODO: I2C unicomm
#[cfg(not(unicomm))]
pub mod i2c;
#[cfg(not(unicomm))]
pub mod i2c_target;
#[cfg(feature = "low-power")]
pub mod low_power;
#[cfg(mathacl)]
pub mod mathacl;
#[cfg(feature = "_probe")]
pub mod probe;
pub(crate) mod sync;
pub mod sysctl;
pub mod tim;
#[cfg(timb)]
pub mod timb;
#[cfg(trng)]
pub mod trng;
#[cfg(unicomm)]
pub mod unicomm;
// TODO: UART unicomm
#[cfg(not(unicomm))]
pub mod uart;
pub mod wwdt;

/// Operating modes for peripherals.
pub mod mode {
    trait SealedMode {}

    /// Operating mode for a peripheral.
    #[allow(private_bounds)]
    pub trait Mode: SealedMode {}

    /// Blocking mode.
    pub struct Blocking;
    impl SealedMode for Blocking {}
    impl Mode for Blocking {}

    /// Async mode.
    pub struct Async;
    impl SealedMode for Async {}
    impl Mode for Async {}
}

#[cfg(all(feature = "_time-driver", not(feature = "rt")))]
compile_error!(
    "a `time-driver-*` feature needs `rt`. The time driver installs its own interrupt handler, and \
     without `rt` there is no vector table to install it into, so it could never keep time. Enable \
     `rt`, or drop the time driver and use the HAL's blocking APIs."
);

#[cfg(feature = "_time-driver")]
mod time_driver;

pub(crate) mod _generated {
    #![allow(dead_code)]
    #![allow(unused_imports)]
    #![allow(non_snake_case)]
    #![allow(missing_docs)]

    include!(concat!(env!("OUT_DIR"), "/_generated.rs"));
}

// Reexports
pub(crate) use _generated::gpio_pincm;
pub use _generated::{Peripherals, peripherals};
pub use embassy_hal_internal::Peri;
#[cfg(feature = "unstable-pac")]
pub use mspm0_metapac as pac;
#[cfg(not(feature = "unstable-pac"))]
pub(crate) use mspm0_metapac as pac;
/// How many priority bits the NVIC implements.
///
/// Re-exported at the path [RTIC](https://rtic.rs)'s `#[app(device = embassy_mspm0)]` looks for.
/// Without it an application has to write a shim module of its own to hold it.
#[cfg(feature = "rt")]
pub use pac::NVIC_PRIO_BITS;

/// The interrupt groups' demultiplexers, called by the vector-table symbols
/// [`bind_group_interrupts!`] emits. Public only so that macro can name them from the user's crate.
#[cfg(feature = "rt")]
#[doc(hidden)]
pub use crate::_generated::group_demux as _group_demux;
pub use crate::_generated::interrupt;
/// The interrupt enum, at the path RTIC's `#[app(device = embassy_mspm0)]` expects it.
///
/// Needed only by a hardware task: `#[task(binds = ...)]` names the enum from the crate root, while
/// software tasks and dispatchers do not. It is the same type as
/// [`interrupt::Interrupt`].
#[cfg(feature = "rt")]
pub use crate::interrupt::Interrupt;

/// Interrupt sources dispatched by an interrupt group rather than by an NVIC line of their own.
///
/// Several peripherals share one NVIC line, and the group's handler reads `INT_GROUPn.IIDX` to
/// decide which of them fired and calls that source's symbol. The symbol is weakly defined as
/// `DefaultHandler`, so a source nothing handles costs its caller a branch and nothing else — which
/// is why a driver that needs a handler asks to be given one, the same way an NVIC-line driver does.
///
/// [`interrupt::typelevel`] cannot describe these: it is keyed on NVIC numbers, and a group source
/// has none. Bind them with [`bind_group_interrupts!`] instead.
pub mod interrupt_group {
    // Empty on the 19 chips that group nothing and give every source an NVIC line of its own — the
    // C1105, C1106 and H3216 families — where a glob over an empty module is an unused import, which
    // `-D warnings` turns into a build failure.
    #[allow(unused_imports)]
    pub use crate::_generated::group_source::*;

    /// A source an interrupt group dispatches.
    pub trait Source {}

    /// A handler for one group source.
    pub trait Handler<S: Source> {
        /// Called by the group's handler when this source fired.
        ///
        /// # Safety
        ///
        /// Called from an interrupt, and only by the generated group handler.
        unsafe fn on_interrupt();
    }

    /// Proof that `H` is bound to `S`, produced by [`bind_group_interrupts!`](crate::bind_group_interrupts).
    ///
    /// # Safety
    ///
    /// Implementing this without defining the source's symbol lets a driver wait on an interrupt
    /// that reaches no handler. Use the macro.
    pub unsafe trait Binding<S: Source, H: Handler<S>> {}
}

/// Macro to bind handlers to sources dispatched by an interrupt group.
///
/// The counterpart to [`bind_interrupts!`] for peripherals that share an NVIC line through an
/// interrupt group — see [`interrupt_group`]. It defines the source's symbol, which is what makes
/// the group's handler reach the driver, and implements
/// [`Binding`](crate::interrupt_group::Binding) so a driver can require one.
///
/// ```rust,ignore
/// use embassy_mspm0::{bind_group_interrupts, trng};
///
/// bind_group_interrupts!(struct Irqs {
///     TRNG => trng::InterruptHandler;
/// });
/// ```
///
/// # At most once per binary
///
/// This also emits the groups' vector-table entries, which is what keeps a demultiplexer out of a
/// binary that binds nothing. They can only be defined once, so bind every source a binary needs in a
/// single invocation — a second one fails to link, naming the duplicated group. Sources sharing a
/// group is the normal case rather than the exception: on most chips `GPIOA`, `GPIOB`, `TRNG` and the
/// comparators are all on the same one.
#[macro_export]
macro_rules! bind_group_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_source:meta)])?
            $source:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        // The vector-table entries for the groups these sources land on, which nothing else emits: a
        // group nothing binds links no demultiplexer at all. Passed the source names so each group can
        // decide for itself — so **this macro may appear at most once in a binary**, and a second
        // invocation is a duplicate-symbol error naming the group.
        $crate::__mspm0_group_vectors!($($source)*);

        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        $(
            // Deliberately not `extern "C"`: the generated group handler declares these
            // `extern "Rust"`, and the definition has to agree with it.
            #[allow(non_snake_case)]
            #[unsafe(no_mangle)]
            $(#[cfg($cond_source)])?
            fn $source() {
                unsafe {
                    $(
                        $(#[cfg($cond_handler)])?
                        <$handler as $crate::interrupt_group::Handler<
                            $crate::interrupt_group::$source,
                        >>::on_interrupt();
                    )*
                }
            }

            $(#[cfg($cond_source)])?
            $crate::bind_group_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt_group::Binding<
                        $crate::interrupt_group::$source,
                        $handler,
                    > for $name {}
                )*
            );
        )*
    };
    (@inner $($t:tt)*) => {
        $($t)*
    }
}

/// Macro to bind interrupts to handlers.
///
/// This defines the right interrupt handlers, and creates a unit struct (like `struct Irqs;`)
/// and implements the right [`Binding`](crate::interrupt::typelevel::Binding)s for it. You can pass
/// this struct to drivers to prove at compile-time that the right interrupts have been bound.
///
/// Example of how to bind one interrupt:
///
/// ```rust,ignore
/// use embassy_nrf::{bind_interrupts, spim, peripherals};
///
/// bind_interrupts!(
///     /// Binds the SPIM3 interrupt.
///     struct Irqs {
///         SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
///     }
/// );
/// ```
///
/// Example of how to bind multiple interrupts in a single macro invocation:
///
/// ```rust,ignore
/// use embassy_nrf::{bind_interrupts, spim, twim, peripherals};
///
/// bind_interrupts!(struct Irqs {
///     SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
///     TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
/// });
/// ```

// developer note: this macro can't be in `embassy-hal-internal` due to the use of `$crate`.
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[unsafe(no_mangle)]
            $(#[cfg($cond_irq)])?
            unsafe extern "C" fn $irq() {
                unsafe {
                    $(
                        $(#[cfg($cond_handler)])?
                        <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();

                    )*
                }
            }

            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
                )*
            );
        )*
    };
    (@inner $($t:tt)*) => {
        $($t)*
    }
}

/// `embassy-mspm0` global configuration.
#[non_exhaustive]
#[derive(Clone, Copy)]
pub struct Config {
    /// The clock tree to program.
    ///
    /// Defaults to the reset tree: SYSOSC at its base frequency driving MCLK, LFCLK from the
    /// internal oscillator, and MFCLK enabled. Build another with [`sysctl::clock::Config`], which
    /// resolves in a `const`, so an invalid tree fails to compile:
    ///
    /// ```ignore
    /// use embassy_mspm0::sysctl::clock::{self, MclkSource, SysPllConfig, SysPllRef, SysPllTap, UlpclkDiv};
    ///
    /// // 80 MHz MCLK from the SYSPLL, with ULPCLK halved to stay inside its 40 MHz ceiling.
    /// const CLOCK: clock::Config = clock::Config::new()
    ///     .with_syspll(SysPllConfig {
    ///         reference: SysPllRef::Sysosc,
    ///         pdiv: 2,
    ///         qdiv: 5,
    ///         clk0_div: None,
    ///         clk1_div: Some(2),
    ///         clk2x_div: Some(2),
    ///         mclk_tap: SysPllTap::Clk2x,
    ///     })
    ///     .with_mclk(MclkSource::Hsclk)
    ///     .with_ulpclk_div(UlpclkDiv::Div2);
    /// const _: () = assert!(CLOCK.resolve().is_ok());
    /// ```
    pub clock: sysctl::clock::ClockSetup,

    /// The size of DMA block transfer burst.
    ///
    /// Bounds how many transfers one channel makes before the controller re-evaluates priority, so a
    /// smaller burst lets a higher-priority channel in sooner and arbitrates more often. The default
    /// runs a whole block uninterrupted.
    pub dma_burst_size: dma::BurstSize,

    /// Whether the DMA channels are used in a fixed priority or a round robin fashion.
    ///
    /// If [`false`], the DMA priorities are fixed.
    ///
    /// If [`true`], after a channel finishes a transfer it becomes the lowest priority.
    pub dma_round_robin: bool,

    /// Shortest time to the next wake for which [`low_power::sleep`] enters a deep-sleep mode.
    ///
    /// Below it the chip stays in RUN for a plain `WFI`, since entering and leaving a mode costs about
    /// twice its wake-up latency. Only the time driver's wake counts, so this has no effect without
    /// one, and a value longer than the driver's bookkeeping tick — one second with a 16-bit timer,
    /// 18 hours with a 32-bit one — stops the chip deep-sleeping at all.
    #[cfg(feature = "low-power")]
    pub min_sleep: embassy_time::Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock: sysctl::clock::RESET_SETUP,
            dma_burst_size: dma::BurstSize::Complete,
            dma_round_robin: false,
            #[cfg(feature = "low-power")]
            min_sleep: low_power::DEFAULT_MIN_SLEEP,
        }
    }
}

// Called exactly once — `Peripherals::take_with_cs` panics otherwise — so inlining costs no
// duplication and lets a constant `Config` fold the clock programming down to its live branches.
#[inline(always)]
pub fn init(config: Config) -> Peripherals {
    critical_section::with(|cs| {
        let peripherals = Peripherals::take_with_cs(cs);

        // Program the clock tree before anything reads a rate from it. Every driver constructed
        // later asks `sysctl::clocks()` what it is running at.
        //
        // Matched rather than unwrapped: the tree was already validated by `Config::build`, so the
        // only failure left is an oscillator that never started, and formatting the error would pull
        // the whole `defmt` value-formatting path into every binary for a case that cannot be
        // recovered from anyway.
        let clocks = match sysctl::clock::apply(&config.clock) {
            Ok(clocks) => clocks,
            Err(_) => core::panic!("a configured clock source never started"),
        };
        sysctl::set_clocks(cs, clocks);

        // TODO: Errata PMCU_ERR_03 states that BOR thresholds other than 0 don't work in STANDBY.
        // It is only listed for L110x/L13xx, so we can expose this option on other MCUs.
        pac::SYSCTL.borthreshold().modify(|w| {
            w.set_level(0);
        });

        gpio::init(pac::GPIOA);
        #[cfg(gpio_pb)]
        gpio::init(pac::GPIOB);
        #[cfg(gpio_pc)]
        gpio::init(pac::GPIOC);

        // Without `rt` there is no handler behind these, so the first edge would reach `DefaultHandler`.
        #[cfg(feature = "rt")]
        _generated::enable_group_interrupts(cs);

        // Where GPIOA has an NVIC line of its own rather than sharing an interrupt group,
        // `enable_group_interrupts` does not reach it.
        #[cfg(all(gpioa_interrupt, feature = "rt"))]
        unsafe {
            use crate::_generated::interrupt::typelevel::Interrupt;
            crate::interrupt::typelevel::GPIOA::enable();
        }

        // SAFETY: Peripherals::take_with_cs will only be run once or panic.
        unsafe { dma::init(cs, config.dma_burst_size, config.dma_round_robin) };

        #[cfg(feature = "low-power")]
        low_power::set_min_sleep(config.min_sleep);

        #[cfg(feature = "_time-driver")]
        time_driver::init(cs);

        peripherals
    })
}

pub(crate) mod sealed {
    #[allow(dead_code)]
    pub trait Sealed {}
}

struct BitIter(u32);

impl Iterator for BitIter {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        match self.0.trailing_zeros() {
            32 => None,
            b => {
                self.0 &= !(1 << b);
                Some(b)
            }
        }
    }
}

/// Reset cause values from SYSCTL.RSTCAUSE register.
/// Based on MSPM0 L-series Technical Reference Manual Table 2-9 and
/// MSPM0 G-series Technical Reference Manual Table 2-12.
///
/// Three of these exist on only some devices and are gated accordingly. Which, is derived from the
/// SYSCTL peripheral version rather than from a list of chip families, so it follows the PAC.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetCause {
    /// No reset since last read
    NoReset,
    /// VDD < POR- violation, PMU trim parity fault, or SHUTDNSTOREx parity fault
    PorHwFailure,
    /// NRST pin reset (>1s)
    PorExternalNrst,
    /// Software-triggered POR
    PorSwTriggered,
    /// VDD < BOR- violation
    BorSupplyFailure,
    /// Wake from SHUTDOWN
    BorWakeFromShutdown,
    /// Non-PMU trim parity fault
    #[cfg(rstcause_nonpmuparity)]
    BootrstNonPmuParityFault,
    /// Fatal clock fault
    BootrstClockFault,
    /// Software-triggered BOOTRST
    BootrstSwTriggered,
    /// NRST pin reset (<1s)
    BootrstExternalNrst,
    /// WWDT0 violation
    BootrstWwdt0Violation,
    /// WWDT1 violation (G-series only)
    #[cfg(rstcause_wwdt1)]
    SysrstWwdt1Violation,
    /// BSL exit (if present)
    SysrstBslExit,
    /// BSL entry (if present)
    SysrstBslEntry,
    /// Uncorrectable flash ECC error (if present)
    #[cfg(rstcause_flashecc)]
    SysrstFlashEccError,
    /// CPU lockup violation
    SysrstCpuLockupViolation,
    /// Debug-triggered SYSRST
    SysrstDebugTriggered,
    /// Software-triggered SYSRST
    SysrstSwTriggered,
    /// Debug-triggered CPURST
    CpurstDebugTriggered,
    /// Software-triggered CPURST
    CpurstSwTriggered,
}

/// Read the reset cause from the SYSCTL.RSTCAUSE register.
///
/// This function reads the reset cause register which indicates why the last
/// system reset occurred. The register is automatically cleared after being read,
/// so this should be called only once per application startup.
///
/// If the reset cause is not recognized, an `Err` containing the raw value is returned.
#[must_use = "Reading reset cause will clear it"]
pub fn read_reset_cause() -> Result<ResetCause, u8> {
    let cause_raw = pac::SYSCTL.rstcause().read().id();

    use ResetCause::*;
    use pac::sysctl::vals::Id;

    // Three causes exist on only some devices, where the PAC generates `_RESERVED_n` instead of the
    // variant, so naming one in an arm is a compile error on the wrong chip. The cfgs guarding them
    // come from the SYSCTL peripheral version, which is what selects that enum in the first place.
    match cause_raw {
        Id::Norst => Ok(NoReset),
        Id::Porhwfail => Ok(PorHwFailure),
        Id::Porexnrst => Ok(PorExternalNrst),
        Id::Porsw => Ok(PorSwTriggered),
        Id::Borsupply => Ok(BorSupplyFailure),
        Id::Borwakeshutdn => Ok(BorWakeFromShutdown),
        #[cfg(rstcause_nonpmuparity)]
        Id::Bootnonpmuparity => Ok(BootrstNonPmuParityFault),
        Id::Bootclkfail => Ok(BootrstClockFault),
        Id::Bootsw => Ok(BootrstSwTriggered),
        Id::Bootexnrst => Ok(BootrstExternalNrst),
        Id::Bootwwdt0 => Ok(BootrstWwdt0Violation),
        Id::Sysbslexit => Ok(SysrstBslExit),
        Id::Sysbslentry => Ok(SysrstBslEntry),
        #[cfg(rstcause_wwdt1)]
        Id::Syswwdt1 => Ok(SysrstWwdt1Violation),
        #[cfg(rstcause_flashecc)]
        Id::Sysflashecc => Ok(SysrstFlashEccError),
        Id::Syscpulock => Ok(SysrstCpuLockupViolation),
        Id::Sysdbg => Ok(SysrstDebugTriggered),
        Id::Syssw => Ok(SysrstSwTriggered),
        Id::Cpudbg => Ok(CpurstDebugTriggered),
        Id::Cpusw => Ok(CpurstSwTriggered),
        other => Err(other as u8),
    }
}
