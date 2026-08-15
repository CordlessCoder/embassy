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
pub mod flash;
pub mod gpio;
// A UNICOMM chip reaches I2C through that block instead, and its mode driver is not written yet.
#[cfg(comp)]
pub mod comp;
#[cfg(crc)]
pub mod crc;
#[cfg(not(unicomm))]
pub mod i2c;
#[cfg(not(unicomm))]
pub mod i2c_target;
#[cfg(feature = "low-power")]
pub mod low_power;
#[cfg(mathacl)]
pub mod mathacl;
#[cfg(opa)]
pub mod opa;
mod prefetch;
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
// A UNICOMM chip reaches UART through that block instead, and its mode driver is not written yet.
#[cfg(not(unicomm))]
pub mod uart;
#[cfg(vref)]
pub mod vref;
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

#[cfg(all(feature = "bor-warning", not(mspm0_bor_warning_levels)))]
compile_error!(
    "`bor-warning` is on for a device whose brown-out supervisor has only the reset level. Its \
     datasheet publishes no VBOR1-VBOR3, so the upper `BORTHRESHOLD.LEVEL` encodings arm nothing. \
     Drop the feature: BOR0 still resets the device, which is all this part offers."
);

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
    /// Clearing a group's latched source without dispatching it.
    ///
    /// One function per group, for a handler that services the group's sources itself and so has
    /// nothing to hand them to. A handler that *does* dispatch does not need these:
    /// [`bind_group_interrupts!`](crate::bind_group_interrupts)'s `unsafe struct` arm gives it an
    /// entry point that acknowledges the group on the way past.
    ///
    /// **A group's sources are latched, and one read clears one of them.** Leave a source latched and
    /// the group's line stays asserted, so the handler returns and the NVIC vectors straight back in.
    pub use crate::_generated::group_ack as ack;
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
///
/// # `unsafe struct`, for a vector table someone else owns
///
/// Write `unsafe struct` instead of `struct` and no vector-table entry is emitted. Everything else is
/// the same, and each group the bound sources land on gets a function on the struct for whoever does
/// own the vector to call:
///
/// ```rust,ignore
/// bind_group_interrupts!(unsafe struct Irqs {
///     GPIOA => gpio::InterruptHandler;
/// });
///
/// #[task(binds = GROUP1, priority = 2)]
/// fn on_group1(_: on_group1::Context) {
///     unsafe { Irqs::GROUP1() }
/// }
/// ```
///
/// The once-per-binary rule relaxes with it, since there is no vector to duplicate: several
/// invocations link, and a binary may own one group this way while the HAL owns another the safe way.
/// Bind each *source* once.
///
/// **What the `unsafe` covers is the calling, not the wiring.** A binding whose group is never
/// dispatched leaves a driver waiting forever, which is a hang and not unsoundness. The obligation is
/// that the generated function runs only from that group's own handler, and from one place: the
/// drivers' waker lists are written by exactly one context that cannot preempt itself.
#[macro_export]
macro_rules! bind_group_interrupts {
    // Ahead of the safe arm so the `unsafe` is matched rather than backtracked into.
    ($(#[$attr:meta])* $vis:vis unsafe struct $name:ident {
        $(
            $(#[cfg($cond_source:meta)])?
            $source:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        // A way into each group's demultiplexer, where the safe arm emits that group's vector.
        $crate::__mspm0_group_entries!($vis $name; $($source)*);

        $(
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
/// # `unsafe struct`, for a vector table someone else owns
///
/// Write `unsafe struct` instead of `struct` and no vector-table entry is emitted. The
/// [`Binding`](crate::interrupt::typelevel::Binding)s a driver asks for are the same; in place of the
/// entry, each interrupt gets a function on the struct for whoever does own the vector to call:
///
/// ```rust,ignore
/// bind_interrupts!(unsafe struct Irqs {
///     UART0 => uart::InterruptHandler<peripherals::UART0>;
/// });
///
/// #[task(binds = UART0, priority = 2)]
/// fn on_uart(_: on_uart::Context) {
///     unsafe { Irqs::UART0() }
/// }
/// ```
///
/// See [`bind_group_interrupts!`] for what the `unsafe` covers, and for the peripherals that share an
/// NVIC line through an interrupt group.
// developer note: this macro can't be in `embassy-hal-internal` due to the use of `$crate`.
#[macro_export]
macro_rules! bind_interrupts {
    // Ahead of the safe arm so the `unsafe` is matched rather than backtracked into.
    ($(#[$attr:meta])* $vis:vis unsafe struct $name:ident {
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

        impl $name {
            $(
                #[doc = concat!("Run every handler bound to `", stringify!($irq), "` here.")]
                #[doc = ""]
                #[doc = "# Safety"]
                #[doc = ""]
                #[doc = concat!(
                    "Call this from `", stringify!($irq), "`'s own interrupt handler and from nowhere \
                     else. A driver's waiter list is written by exactly one context that cannot \
                     preempt itself, and a second caller breaks that.",
                )]
                #[allow(non_snake_case)]
                #[inline(always)]
                $(#[cfg($cond_irq)])?
                $vis unsafe fn $irq() {
                    unsafe {
                        $(
                            $(#[cfg($cond_handler)])?
                            <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();
                        )*
                    }
                }
            )*
        }

        $(
            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
                )*
            );
        )*
    };

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
    /// When the analog charge pump runs.
    ///
    /// Defaults to [`sysctl::Vboost::OnDemand`], which is what the device does on its own. Raise it where the
    /// startup delay it adds to a comparator or an amplifier matters more than its current — see
    /// [`sysctl::Vboost`] for what that delay is and for the erratum that asks for
    /// [`sysctl::Vboost::OnAlways`] below 1.8 V.
    pub vboost: sysctl::Vboost,

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
    /// twice its wake-up latency. A value longer than the driver's bookkeeping tick — one second with a
    /// 16-bit timer, 18 hours with a 32-bit one — stops the chip deep-sleeping at all.
    ///
    /// Only the time driver's wake counts, so this exists only alongside one. Without a time driver
    /// nothing schedules a wake and the guards alone decide the depth.
    #[cfg(all(feature = "low-power", feature = "_time-driver"))]
    pub min_sleep: embassy_time::Duration,

    /// Which interrupt lines [`init`] enables, and at what priority.
    ///
    /// Covers the interrupt groups and the dedicated GPIO line — the ones no driver owns, so nothing
    /// else would enable them. A driver with an NVIC line of its own enables it when constructed and
    /// is not affected.
    pub interrupts: InterruptPolicy,
}

/// Which interrupt lines [`init`] enables on the application's behalf.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum InterruptPolicy {
    /// Enable every interrupt group, and the dedicated GPIO line where the chip has one, leaving the
    /// priority where reset left it.
    ///
    /// Reset leaves it at 0, the highest the NVIC has, so these preempt everything. That is what an
    /// application with no other scheduler wants and is what this crate has always done.
    #[default]
    Enable,

    /// Enable the same lines at `priority`, set inside the critical section that unmasks them.
    ///
    /// Setting it afterwards instead leaves a window in which an edge is taken at the reset priority.
    Prioritise(interrupt::Priority),

    /// Enable nothing, because something else owns the NVIC.
    ///
    /// Under [RTIC](https://rtic.rs) this is what `#[task(binds = GROUP1)]` means: its `pre_init` sets
    /// the priority and unmasks before `#[init]` runs, and enabling the line again here would only put
    /// back what was deliberately left alone.
    ///
    /// **Nothing checks that something else did the work.** A binary that selects this and then binds
    /// no handler for a group leaves the line masked, so an edge is latched, never delivered, and the
    /// driver waiting on it waits forever.
    External,
}

impl Config {
    /// The reset configuration, usable in a `const`.
    // A hundred bytes of `Clocks` and clock configuration is past what LLVM will inline at
    // `opt-level = "z"`, so without this the tree arrives at `init` through memory and stops being a
    // constant. Everything downstream then stays a run-time decision: the rates land in a static, and
    // with them which `WakeGuard` a driver takes and which sleep modes `enter_sleep` has to be able
    // to program. Worth up to 364 bytes of flash and 52 of RAM.
    ///
    /// A `const` is what makes that certain. The attribute below asks the optimiser for the same
    /// thing and has been enough so far, but it is a heuristic over a struct whose size is exactly
    /// what defeats it; `const CONFIG: Config = Config::new()` cannot be defeated.
    pub const fn new() -> Self {
        Self {
            vboost: sysctl::Vboost::OnDemand,
            clock: sysctl::clock::RESET_SETUP,
            dma_burst_size: dma::BurstSize::Complete,
            dma_round_robin: false,
            #[cfg(all(feature = "low-power", feature = "_time-driver"))]
            min_sleep: low_power::DEFAULT_MIN_SLEEP,
            interrupts: InterruptPolicy::Enable,
        }
    }
}

impl Default for Config {
    #[inline(always)]
    fn default() -> Self {
        Self::new()
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
        // Before the clock tree: starting HFXT raises the pump, and a policy applied afterwards would
        // have let that first start pay the pump's startup time for nothing.
        sysctl::set_vboost(config.vboost);

        let clocks = match sysctl::clock::apply(&config.clock) {
            Ok(clocks) => clocks,
            Err(_) => core::panic!("a configured clock source never started"),
        };
        sysctl::set_clocks(cs, clocks);

        // The brown-out supervisor is left where the device booted it, which is the reset level on
        // every MSPM0. Raising it is `sysctl::set_bor_threshold`, and a level that only takes effect
        // on the `BORCLRCMD` write cannot be set here anyway -- the write this used to make selected
        // the level the part was already enforcing and never activated it, so it did nothing at all.

        gpio::init(pac::GPIOA);
        #[cfg(gpio_pb)]
        gpio::init(pac::GPIOB);
        #[cfg(gpio_pc)]
        gpio::init(pac::GPIOC);

        // Without `rt` there is no handler behind these, so the first edge would reach `DefaultHandler`.
        #[cfg(feature = "rt")]
        match config.interrupts {
            InterruptPolicy::Enable => enable_hal_interrupts(cs, None),
            InterruptPolicy::Prioritise(priority) => enable_hal_interrupts(cs, Some(priority)),
            InterruptPolicy::External => {}
        }

        dma::init(cs, config.dma_burst_size, config.dma_round_robin);

        #[cfg(all(feature = "low-power", feature = "_time-driver"))]
        low_power::set_min_sleep(config.min_sleep);

        #[cfg(feature = "_time-driver")]
        time_driver::init(cs);

        peripherals
    })
}

/// Enable the interrupt lines no driver owns: the groups, and the dedicated GPIO line where the chip
/// gives one.
///
/// `priority` is applied inside the caller's section, ahead of each unmask, so no edge is taken at the
/// reset priority on the way past.
#[cfg(feature = "rt")]
fn enable_hal_interrupts(cs: critical_section::CriticalSection, priority: Option<interrupt::Priority>) {
    _generated::enable_group_interrupts(cs, priority);

    // Where GPIOA has an NVIC line of its own rather than sharing an interrupt group,
    // `enable_group_interrupts` does not reach it.
    #[cfg(gpioa_interrupt)]
    {
        use crate::_generated::interrupt::typelevel::Interrupt;

        if let Some(priority) = priority {
            crate::interrupt::typelevel::GPIOA::set_priority_with_cs(cs, priority);
        }

        unsafe { crate::interrupt::typelevel::GPIOA::enable() };
    }
}

/// Sleep until an interrupt, once.
///
/// The idle policy for a scheduler that has none of its own. With `low-power` this enters the deepest
/// mode the held [`WakeGuard`](sysctl::WakeGuard)s and the next scheduled wake permit; without it, a
/// plain `WFI`. Either way the instruction prefetcher is suspended across it, which `CPU_ERR_03` asks
/// for and a bare `WFI` does not do.
///
/// The section comes from the caller so that whatever decides there is nothing to do can decide it in
/// the same one — a race there is a wake missed and slept through. Under [RTIC](https://rtic.rs) there
/// is nothing to check and `#[idle]` is the whole of it:
///
/// ```rust,ignore
/// #[idle]
/// fn idle(_: idle::Context) -> ! {
///     loop {
///         critical_section::with(embassy_mspm0::idle);
///     }
/// }
/// ```
///
/// [`low_power::sleep`] is the same sleep without the guard rail below, for a caller that has already
/// established it.
///
/// # Panics
///
/// In debug builds, when called from a handler. `WFI` there is woken only by an interrupt of *higher*
/// priority than the one running, so the lowest-priority handler never returns — and an RTIC software
/// task runs inside its dispatcher's handler, which is how this gets reached by accident. Release
/// builds do not check.
pub fn idle(cs: critical_section::CriticalSection) {
    debug_assert!(
        matches!(
            cortex_m::peripheral::SCB::vect_active(),
            cortex_m::peripheral::scb::VectActive::ThreadMode
        ),
        "embassy_mspm0::idle sleeps, so it must run in thread mode rather than in a handler"
    );

    // SAFETY: thread mode, which the assertion above catches in a debug build and the doc comment
    // carries in a release one.
    #[cfg(feature = "low-power")]
    unsafe {
        low_power::sleep(cs)
    };

    #[cfg(not(feature = "low-power"))]
    {
        let _ = cs;
        prefetch::guarded_wfi();
    }
}

pub(crate) mod sealed {
    #[allow(dead_code)]
    pub trait Sealed {}
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

/// Reset the device, and do not come back.
///
/// The counterpart to [`read_reset_cause`], which is how the code that runs afterwards finds out this
/// is why. A [`sysctl::ResetLevel::Cpu`] or [`sysctl::ResetLevel::Boot`] reset reports `PorSwTriggered`.
///
/// SRAM is not cleared by any of these, but nothing may be assumed about it either: `.data` and
/// `.bss` are reinitialised on the way back up, exactly as after a power cycle.
///
/// The two levels that enter and leave the bootstrap loader are deliberately absent. They are a
/// firmware-update mechanism rather than a reset, and one of them leaves the device running something
/// other than this application.
pub fn reset(level: sysctl::ResetLevel) -> ! {
    sysctl::reset_device(level)
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
