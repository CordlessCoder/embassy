use std::cmp::Ordering;
use std::fmt::Write;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::{env, fs};

use common::CfgSet;
use mspm0_metapac::metadata::{ALL_CHIPS, METADATA, MemoryKind, Peripheral, PowerDomain, PowerMode};
use proc_macro2::{Ident, Literal, Span, TokenStream};
use quote::{format_ident, quote};

#[path = "./build_common.rs"]
mod common;

fn main() {
    let mut cfgs = common::CfgSet::new();
    common::set_target_cfgs(&mut cfgs);

    check_nvic_priority_bits();
    check_sram_retention();
    generate_code(&mut cfgs);
    select_gpio_features(&mut cfgs);
    interrupt_group_linker_magic();
}

fn generate_code(cfgs: &mut CfgSet) {
    // Unconditional, even though `interrupt_group.x` only does anything where there is a vector table
    // to patch: every example's build script passes `-Tinterrupt_group.x`, so a search path that
    // appeared only with `rt` turned a missing feature into a linker script that cannot be found.
    println!(
        "cargo:rustc-link-search={}",
        PathBuf::from(env::var_os("OUT_DIR").unwrap()).display(),
    );

    cfgs.declare_all(&["gpio_pb", "gpio_pc", "int_group1", "unicomm"]);

    let chip_name = match env::vars()
        .map(|(a, _)| a)
        .filter(|x| x.starts_with("CARGO_FEATURE_MSPM0") || x.starts_with("CARGO_FEATURE_MSPS"))
        .get_one()
    {
        Ok(x) => x,
        Err(GetOneError::None) => panic!("No mspm0xx/mspsxx Cargo feature enabled"),
        Err(GetOneError::Multiple) => panic!("Multiple mspm0xx/mspsxx Cargo features enabled"),
    }
    .strip_prefix("CARGO_FEATURE_")
    .unwrap()
    .to_ascii_lowercase()
    .replace('_', "-");

    eprintln!("chip: {chip_name}");

    cfgs.enable_all(&get_chip_cfgs(&chip_name));
    for chip in ALL_CHIPS {
        cfgs.declare_all(&get_chip_cfgs(&chip));
    }

    peripheral_kind_cfgs(cfgs);
    peripheral_name_cfgs(cfgs);
    errata_cfgs(cfgs);
    sysctl_version_cfgs(cfgs);
    let clock_tree = clock_tree_cfgs(cfgs);

    let mut singletons = get_singletons(cfgs);

    time_driver(&mut singletons, cfgs);
    pin_features(&mut singletons);

    let mut g = TokenStream::new();

    g.extend(generate_singletons(&singletons));
    g.extend(generate_pincm_mapping());
    g.extend(generate_pin());
    g.extend(generate_timers());
    g.extend(generate_interrupts());
    g.extend(generate_peripheral_instances());
    g.extend(generate_low_power(&singletons));
    g.extend(generate_pin_trait_impls());
    g.extend(generate_groups());
    g.extend(generate_gpio_port_interrupts());
    g.extend(generate_dma_channel_count());
    g.extend(generate_adc_constants(cfgs));
    g.extend(generate_trng_constants());
    g.extend(generate_clock_ceilings());
    g.extend(clock_tree);

    let out_dir = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let out_file = out_dir.join("_generated.rs").to_string_lossy().to_string();
    fs::write(&out_file, g.to_string()).unwrap();
    rustfmt(&out_file);
}

/// Peripheral kinds a module is gated on, so it can say `#[cfg(trng)]` rather than naming the
/// families that happen to have the peripheral.
///
/// A family list goes stale — `trng` was gated on six families while the metadata reported seven.
/// Add a kind here when a module starts gating on it; an undeclared cfg warns, so an omission shows
/// up at once.
const PERIPHERAL_KIND_CFGS: &[&str] = &["mathacl", "trng", "usbfs"];

/// Enable a cfg for each kind in [`PERIPHERAL_KIND_CFGS`] this chip actually has.
fn peripheral_kind_cfgs(cfgs: &mut CfgSet) {
    cfgs.declare_all(PERIPHERAL_KIND_CFGS);

    for kind in PERIPHERAL_KIND_CFGS {
        if METADATA.peripherals.iter().any(|peripheral| peripheral.kind == *kind) {
            cfgs.enable(kind);
        }
    }
}

/// Peripheral *instances* a driver gates on, where the kind is not specific enough.
///
/// `wwdt` is on every chip, but only some have a second instance, and the driver has to name
/// `pac::WWDT1` to reach it.
const PERIPHERAL_NAME_CFGS: &[&str] = &["WWDT1"];

/// Enable a cfg, lowercased, for each instance in [`PERIPHERAL_NAME_CFGS`] this chip has.
fn peripheral_name_cfgs(cfgs: &mut CfgSet) {
    for name in PERIPHERAL_NAME_CFGS {
        let cfg = name.to_lowercase();

        cfgs.declare(&cfg);
        if METADATA.peripherals.iter().any(|peripheral| peripheral.name == *name) {
            cfgs.enable(cfg);
        }
    }
}

/// Errata a driver has a workaround for, by TI's identifier.
///
/// Add one here when a driver starts gating on it. The cfg is emitted from the device's own errata
/// sheet, so unlike the family lists these replace it cannot miss a part — and a new device gets its
/// workarounds without an edit.
const ERRATA_CFGS: &[&str] = &["GPIO_ERR_01", "UART_ERR_03", "UART_ERR_08"];

/// Enable a cfg, lowercased, for each erratum in [`ERRATA_CFGS`] that applies to this chip.
fn errata_cfgs(cfgs: &mut CfgSet) {
    for erratum in ERRATA_CFGS {
        let cfg = erratum.to_lowercase();

        cfgs.declare(&cfg);
        if METADATA.has_erratum(erratum) {
            cfgs.enable(cfg);
        }
    }
}

/// What a SYSCTL register block provides that the device metadata does not.
///
/// Which clock sources exist is [`METADATA.clock_tree`](clock_tree_cfgs) instead, since two families
/// can share a block and still differ. What is left here are facts about the block itself: a step the
/// C-series TRM adds to STOP0 entry, and which `RSTCAUSE.ID` variants its enum defines.
struct SysctlCaps {
    /// Whether entering STOP0 must also clear `MCLKCFG.USELFCLK`.
    ///
    /// Not field presence — `USELFCLK` exists everywhere — but a step the C-series TRM adds to the
    /// entry sequence.
    stop0_clears_lfclk: bool,

    /// Whether SYSCTL has the `SHUTDNSTORE` array, the only bytes that survive SHUTDOWN.
    ///
    /// A 4-element array at `0x1400` on every block but `h321x`.
    shutdnstore: bool,

    /// `RSTCAUSE.ID` causes that only some blocks define: non-PMU trim parity fault, WWDT1
    /// violation, and uncorrectable flash ECC error.
    rstcause_nonpmuparity: bool,
    rstcause_wwdt1: bool,
    rstcause_flashecc: bool,

    /// Whether SYSCTL has `HSCLKCFG.HSCLKSEL`, the mux between the SYSPLL and HFCLK.
    ///
    /// Absent on `c110x` and `l110x_l130x_l134x`. Where it is absent and the device still has an
    /// HFCLK path, HSCLK is HFCLK with nothing to select. Where it is present it must be programmed
    /// even without a SYSPLL: it resets to the SYSPLL position, which is a source those devices do
    /// not have.
    hsclk_mux: bool,
}

impl SysctlCaps {
    const NONE: Self = Self {
        stop0_clears_lfclk: false,
        shutdnstore: false,
        hsclk_mux: false,
        rstcause_nonpmuparity: false,
        rstcause_wwdt1: false,
        rstcause_flashecc: false,
    };
}

/// Every SYSCTL version the crate knows, each of which gets a `sysctl_<version>` cfg.
///
/// `sysctl/mod.rs` picks its per-version file with these, and there is one file per entry.
const SYSCTL_VERSIONS: &[&str] = &[
    "c110x",
    "c1105_c1106",
    "g350x_g310x_g150x_g110x",
    "g351x_g151x",
    "h321x",
    "l110x_l130x_l134x",
    "l122x_l222x",
];

/// Emit a cfg for the parts of SYSCTL that only the register block can answer.
///
/// Keyed on the SYSCTL peripheral *version*, which is what selects the register block, so the table
/// cannot drift from the enum variants it describes and a new device reusing an existing SYSCTL needs
/// no edit. An unrecognised version is an error, since a new register block has to be looked at.
fn sysctl_version_cfgs(cfgs: &mut CfgSet) {
    let version = METADATA
        .peripherals
        .iter()
        .find(|peripheral| peripheral.kind == "sysctl")
        .and_then(|peripheral| peripheral.version)
        .expect("chip has no SYSCTL peripheral version");

    let caps = match version {
        "c110x" => SysctlCaps {
            stop0_clears_lfclk: true,
            shutdnstore: true,
            ..SysctlCaps::NONE
        },

        "c1105_c1106" => SysctlCaps {
            stop0_clears_lfclk: true,
            shutdnstore: true,
            hsclk_mux: true,
            ..SysctlCaps::NONE
        },

        "l110x_l130x_l134x" => SysctlCaps {
            shutdnstore: true,
            rstcause_nonpmuparity: true,
            rstcause_flashecc: true,
            ..SysctlCaps::NONE
        },

        "l122x_l222x" => SysctlCaps {
            shutdnstore: true,
            hsclk_mux: true,
            rstcause_nonpmuparity: true,
            rstcause_flashecc: true,
            ..SysctlCaps::NONE
        },

        "h321x" => SysctlCaps {
            hsclk_mux: true,
            rstcause_nonpmuparity: true,
            rstcause_flashecc: true,
            ..SysctlCaps::NONE
        },

        "g350x_g310x_g150x_g110x" => SysctlCaps {
            shutdnstore: true,
            hsclk_mux: true,
            rstcause_wwdt1: true,
            rstcause_flashecc: true,
            ..SysctlCaps::NONE
        },

        "g351x_g151x" => SysctlCaps {
            shutdnstore: true,
            hsclk_mux: true,
            rstcause_wwdt1: true,
            ..SysctlCaps::NONE
        },

        other => panic!(
            "unknown SYSCTL version {other:?}: work out which RSTCAUSE.ID causes it defines, and \
             whether it has SHUTDNSTORE and whether its TRM adds the USELFCLK step to STOP0 \
             entry, and add it to `sysctl_version_cfgs`, to `SYSCTL_VERSIONS`, and as a file in \
             `src/sysctl/`"
        ),
    };

    for known in SYSCTL_VERSIONS {
        cfgs.declare(&format!("sysctl_{known}"));
    }
    cfgs.enable(&format!("sysctl_{version}"));

    for (cfg, present) in [
        ("mspm0_stop0_clears_lfclk", caps.stop0_clears_lfclk),
        ("mspm0_shutdnstore", caps.shutdnstore),
        ("mspm0_hsclk_mux", caps.hsclk_mux),
        ("rstcause_nonpmuparity", caps.rstcause_nonpmuparity),
        ("rstcause_wwdt1", caps.rstcause_wwdt1),
        ("rstcause_flashecc", caps.rstcause_flashecc),
    ] {
        cfgs.declare(cfg);
        if present {
            cfgs.enable(cfg);
        }
    }
}

/// Emit a cfg for each clock source and divider the device has, and the HFCLK input range.
///
/// Curated per device rather than derived from the SYSCTL block, because the two disagree: mspm0l112x
/// and mspm0l211x share a block whose `SYSOSCCFG.USE4MHZSTOP` exists but have no STOP1, and
/// mspm0c1105_c1106 has a crystal driver its `c110x` sibling does not.
///
/// `mspm0_hfxt` and `mspm0_hfclk_in` are separate hardware — a crystal driver and a digital clock
/// input — and mspm0c110x has the input without the driver. `mspm0_hfclk` is the umbrella that gates
/// the HSCLK path itself.
fn clock_tree_cfgs(cfgs: &mut CfgSet) -> TokenStream {
    let tree = METADATA.clock_tree;

    for (cfg, present) in [
        ("mspm0_hfclk", tree.hfxt || tree.hfclk_in),
        ("mspm0_hfxt", tree.hfxt),
        ("mspm0_hfclk_in", tree.hfclk_in),
        ("mspm0_hfclk_range", tree.hfclk_hz.is_some()),
        ("mspm0_lfxt", tree.lfxt),
        ("mspm0_lfclk_in", tree.lfclk_in),
        ("mspm0_syspll", tree.syspll),
        ("mspm0_ulpclk_div", tree.ulpclk_div),
        ("mspm0_stop1", tree.stop1),
        // More than one band means `MCLKCFG.FLASHWAIT` exists and software has to program it. A
        // single band means the device's MCLK ceiling is inside the zero-wait-state range.
        ("mspm0_flashwait", METADATA.flash_wait_hz.len() > 1),
    ] {
        cfgs.declare(cfg);
        if present {
            cfgs.enable(cfg);
        }
    }

    // `None` on mspm0c110x, which offers HFCLK_IN but whose datasheet specifies no `fHFIN`. The
    // input stays usable there; only the range check is skipped, and the MCLK ceiling still bounds it.
    let Some(range) = tree.hfclk_hz else {
        return quote! {};
    };

    let (min, max) = (range.min_hz, range.max_hz);

    quote! {
        pub const HFCLK_MIN_HZ: u32 = #min;
        pub const HFCLK_MAX_HZ: u32 = #max;
    }
}

fn get_chip_cfgs(chip_name: &str) -> Vec<String> {
    let mut cfgs = Vec::new();

    // GPIO on C110x is special as it does not belong to an interrupt group.
    if chip_name.starts_with("mspm0c1103") || chip_name.starts_with("mspm0c1104") || chip_name.starts_with("msps003f") {
        cfgs.push("mspm0c110x".to_string());
    }

    if chip_name.starts_with("mspm0c1105") || chip_name.starts_with("mspm0c1106") {
        cfgs.push("mspm0c1105_c1106".to_string());
    }

    // Family ranges (temporary until int groups are generated)
    //
    // TODO: Remove this once int group stuff is generated.
    if chip_name.starts_with("mspm0g110") {
        cfgs.push("mspm0g110x".to_string());
    }

    if chip_name.starts_with("mspm0g150") {
        cfgs.push("mspm0g150x".to_string());
    }

    if chip_name.starts_with("mspm0g151") {
        cfgs.push("mspm0g151x".to_string());
    }

    if chip_name.starts_with("mspm0g310") {
        cfgs.push("mspm0g310x".to_string());
    }

    if chip_name.starts_with("mspm0g350") {
        cfgs.push("mspm0g350x".to_string());
    }

    if chip_name.starts_with("mspm0g351") {
        cfgs.push("mspm0g351x".to_string());
    }

    if chip_name.starts_with("mspm0g518") {
        cfgs.push("mspm0g518x".to_string());
    }

    if chip_name.starts_with("mspm0h321") {
        cfgs.push("mspm0h321x".to_string());
    }

    if chip_name.starts_with("mspm0l110") {
        cfgs.push("mspm0l110x".to_string());
    }

    if chip_name.starts_with("mspm0l122") {
        cfgs.push("mspm0l122x".to_string());
    }

    if chip_name.starts_with("mspm0l130") {
        cfgs.push("mspm0l130x".to_string());
    }

    if chip_name.starts_with("mspm0l134") {
        cfgs.push("mspm0l134x".to_string());
    }

    if chip_name.starts_with("mspm0l222") {
        cfgs.push("mspm0l222x".to_string());
    }

    cfgs
}

/// Interrupt groups use a weakly linked symbols and #[linkage = "extern_weak"] is nightly we need to
/// do some linker magic to create weak linkage.
fn interrupt_group_linker_magic() {
    let mut file = String::new();

    for group in METADATA.interrupt_groups {
        for interrupt in group.interrupts.iter() {
            let name = interrupt.name;

            writeln!(&mut file, "PROVIDE({name} = DefaultHandler);").unwrap();
        }
    }

    let out_dir = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let out_file = out_dir.join("interrupt_group.x");
    fs::write(&out_file, file).unwrap();
}

fn generate_groups() -> TokenStream {
    let group_vectors = METADATA.interrupt_groups.iter().map(|group| {
        let vectors = group.interrupts.iter().map(|interrupt| {
            let fn_name = Ident::new(interrupt.name, Span::call_site());

            quote! {
                pub(crate) fn #fn_name();
            }
        });

        quote! { #(#vectors)* }
    });

    let groups = METADATA.interrupt_groups.iter().map(|group| {
        let demux_name = Ident::new(&group.name.to_lowercase(), Span::call_site());
        let group_enum = Ident::new(&format!("Group{}", &group.name[5..]), Span::call_site());
        let group_number = Literal::u32_unsuffixed(group.number);

        let matches = group.interrupts.iter().map(|interrupt| {
            let variant = Ident::new(&interrupt.name, Span::call_site());

            quote! {
                #group_enum::#variant => unsafe { super::group_vectors::#variant() },
            }
        });

        // The body, as an ordinary function. The vector-table symbol that calls it is emitted by
        // `bind_group_interrupts!` instead, so a binary that binds no source on any group links neither
        // — 60 bytes per group, which every binary used to carry whether or not it had a handler behind
        // it. Nothing is lost when it is absent: an unbound source already resolved to `DefaultHandler`.
        quote! {
            pub fn #demux_name() {
                use crate::pac::#group_enum;

                let group = crate::pac::CPUSS.int_group(#group_number);
                let stat = group.iidx().read().stat();

                // check for spurious interrupts
                if stat == crate::pac::cpuss::vals::Iidx::NoIntr {
                    return;
                }

                // MUST subtract by 1 because NoIntr offsets IIDX values.
                let iidx = stat.to_bits() - 1;

                let Ok(group) = #group_enum::try_from(iidx as u8) else {
                    return;
                };

                match group {
                    #(#matches)*
                }
            }
        }
    });

    // One marker per source a group dispatches, so a binding can name the source it handles. A source
    // has no NVIC line of its own, so `interrupt::typelevel` has nothing to name it with.
    let sources = METADATA
        .interrupt_groups
        .iter()
        .flat_map(|group| group.interrupts.iter())
        .map(|interrupt| {
            let name = Ident::new(interrupt.name, Span::call_site());
            let doc = format!("The `{}` source, dispatched by its interrupt group.", interrupt.name);

            quote! {
                #[doc = #doc]
                #[allow(non_camel_case_types)]
                pub struct #name;

                impl crate::interrupt_group::Source for #name {}
            }
        });

    // The vector-table symbols, emitted into the user's crate by `bind_group_interrupts!` rather than
    // here. Each is a call away from its body, so a binary that never invokes the macro links no demux at
    // all, and one that does pays what it did before.
    //
    // Gated at build time rather than with a `cfg`, because a `cfg` inside the macro would be read
    // against the *user's* features, not this crate's.
    let has_rt = env::var_os("CARGO_FEATURE_RT").is_some();

    // One scanner per group, emitting that group's entry the first time it sees one of its own sources
    // in the bound list and nothing at all if it sees none. A group with several sources bound — the
    // ordinary case, since `GPIOA` and `GPIOB` share one — must still emit exactly once, and stopping at
    // the first match is what `macro_rules!` can express where counting is not.
    let scanners = METADATA.interrupt_groups.iter().filter(|_| has_rt).map(|group| {
        let scanner = format_ident!("__mspm0_vectors_{}", group.name.to_lowercase());
        let symbol = Ident::new(group.name, Span::call_site());
        let demux_name = Ident::new(&group.name.to_lowercase(), Span::call_site());
        let doc = format!("Emit `{}`'s vector-table entry if anything binds a source on it.", group.name);

        let hits = group.interrupts.iter().map(|interrupt| {
            let source = Ident::new(interrupt.name, Span::call_site());

            quote! {
                (#source $($rest:tt)*) => {
                    #[allow(non_snake_case)]
                    #[unsafe(no_mangle)]
                    unsafe extern "C" fn #symbol() {
                        $crate::_group_demux::#demux_name();
                    }
                };
            }
        });

        quote! {
            #[doc = #doc]
            #[doc(hidden)]
            #[macro_export]
            macro_rules! #scanner {
                #(#hits)*
                // Not one of this group's: drop it and keep looking.
                ($other:tt $($rest:tt)*) => { $crate::#scanner!($($rest)*); };
                () => {};
            }
        }
    });

    let scanner_calls = METADATA.interrupt_groups.iter().filter(|_| has_rt).map(|group| {
        let scanner = format_ident!("__mspm0_vectors_{}", group.name.to_lowercase());

        quote! { $crate::#scanner!($($source)*); }
    });

    quote! {
        /// One demultiplexer per interrupt group, called by the vector-table symbol
        /// `bind_group_interrupts!` emits for it.
        #[cfg(feature = "rt")]
        pub mod group_demux {
            #(#groups)*
        }

        #[cfg(feature = "rt")]
        mod group_vectors {
            unsafe extern "Rust" {
                #(#group_vectors)*
            }
        }

        pub mod group_source {
            #(#sources)*
        }

        #(#scanners)*

        /// The group handlers' vector-table entries, for the groups the bound sources actually land on.
        ///
        /// Expands to nothing on a chip that groups nothing, and without `rt`.
        #[doc(hidden)]
        #[macro_export]
        macro_rules! __mspm0_group_vectors {
            ($($source:ident)*) => {
                #(#scanner_calls)*
            };
        }
    }
}

/// One binding per GPIO port, naming whichever kind of interrupt dispatches that port: a group
/// source on most chips, an NVIC line of its own on the ones with a single port.
///
/// Every port is required rather than the pin's own, because a pin's port is not in its type — only
/// [`SealedPin::pin_port`] knows it, and that is a run-time read.
fn generate_gpio_port_interrupts() -> TokenStream {
    let bounds: Vec<_> = METADATA
        .peripherals
        .iter()
        .filter(|p| p.kind == "gpio")
        .flat_map(|p| p.interrupts.iter().map(move |interrupt| (p, interrupt)))
        .map(|(peripheral, interrupt)| {
            if interrupt.group_iidx.is_some() {
                // `interrupt.name` is the group's NVIC line, shared with the other sources on it.
                // The source is named after the port.
                let name = Ident::new(peripheral.name, Span::call_site());

                quote! {
                    crate::interrupt_group::Binding<crate::interrupt_group::#name, crate::gpio::InterruptHandler>
                }
            } else {
                let name = Ident::new(interrupt.name, Span::call_site());

                quote! {
                    crate::interrupt::typelevel::Binding<crate::interrupt::typelevel::#name, crate::gpio::InterruptHandler>
                }
            }
        })
        .collect();

    quote! {
        #[cfg(feature = "rt")]
        unsafe impl<T> crate::gpio::PortInterrupts for T where T: #(#bounds)+* {}
    }
}

fn generate_dma_channel_count() -> TokenStream {
    let count = METADATA.dma_channels.len();

    quote! { pub const DMA_CHANNELS: usize = #count; }
}

/// The `clock_range_hz` every instance of `kind` shares, or `None` where the datasheet gives none.
///
/// Per device rather than per family: two parts can share `max_mclk_hz` and a SYSCTL version and
/// still have different `fADCCLK` ranges, and the minimum is not always 4 MHz.
fn peripheral_clock_range(kind: &str) -> Option<(u32, u32)> {
    let mut instances = METADATA
        .peripherals
        .iter()
        .filter(|peripheral| peripheral.kind == kind)
        .map(|peripheral| (peripheral.name, peripheral.clock_range_hz));

    let (first_name, first) = instances.next()?;

    for (name, range) in instances {
        assert_eq!(
            range, first,
            "{name} and {first_name} give different input clock ranges, so the {kind} driver can no \
             longer hold one as a crate-wide constant"
        );
    }

    let range = first?;

    Some((range.min_hz, range.max_hz))
}

/// Emit the TRNG's `TRNGCLKF` input range, on the chips that have one.
fn generate_trng_constants() -> TokenStream {
    let Some((min, max)) = peripheral_clock_range("trng") else {
        return quote! {};
    };

    quote! {
        pub const TRNG_CLK_MIN_HZ: u32 = #min;
        pub const TRNG_CLK_MAX_HZ: u32 = #max;
    }
}

/// Emit the ADC facts that the single `adc_v1` register block does not describe.
///
/// The metadata states these per ADC instance, but no device has two ADCs that disagree, so they
/// stay crate-wide constants and `MAX_SEQUENCE_LEN` stays a `pub const` a caller can size an array
/// with. A device that breaks the assumption fails the build here rather than silently taking
/// whichever instance came first.
fn generate_adc_constants(cfgs: &mut CfgSet) -> TokenStream {
    cfgs.declare("adc_neg_vref");

    let mut instances = METADATA
        .peripherals
        .iter()
        .filter_map(|peripheral| peripheral.adc.map(|adc| (peripheral.name, adc)));

    let (first_name, first) = instances.next().expect("chip has no ADC instance");

    for (name, adc) in instances {
        assert_eq!(
            adc, first,
            "{name} and {first_name} disagree about MEMCTL or VRSEL, so the ADC driver can no \
             longer hold them as crate-wide constants"
        );
    }

    match first.vrsel {
        3 => (),
        5 => cfgs.enable("adc_neg_vref"),
        vrsel => panic!("Unsupported ADC VRSEL value: {vrsel}"),
    }

    let vrsel = first.vrsel;
    let memctl = first.memctl;
    let (min, max) = peripheral_clock_range("adc").expect("chip's ADC has no fADCCLK range");

    quote! {
        pub const ADC_VRSEL: u8 = #vrsel;
        pub const ADC_MEMCTL: u8 = #memctl;

        /// `fADCCLK`, the range the clock selected by `CLKCFG.SAMPCLK` must stay within.
        pub const ADC_CLK_MIN_HZ: u32 = #min;
        pub const ADC_CLK_MAX_HZ: u32 = #max;
    }
}

/// Emit the RUN/SLEEP clock ceilings.
///
/// These are ceilings, not the rate the chip boots at: G-series starts on a 32 MHz SYSOSC but can
/// reach 80 MHz through the PLL. They bound what a clock configuration may ask for, and give PD0
/// peripherals their real rate, which is lower than MCLK on G-series.
fn generate_clock_ceilings() -> TokenStream {
    let max_mclk = METADATA.max_mclk_hz;
    let max_ulpclk = METADATA.max_ulpclk_hz;
    let sysosc_base = METADATA.sysosc_base_hz;
    let flash_wait = METADATA.flash_wait_hz;

    quote! {
        pub const MAX_MCLK_HZ: u32 = #max_mclk;
        pub const MAX_ULPCLK_HZ: u32 = #max_ulpclk;
        pub const SYSOSC_BASE_HZ: u32 = #sysosc_base;

        /// MCLK ceiling for each `MCLKCFG.FLASHWAIT` setting, starting at zero wait states.
        pub const FLASH_WAIT_HZ: &[u32] = &[#(#flash_wait),*];
    }
}

/// Check that the RAM the linker places `.data`/`.bss` in survives deep sleep.
fn check_sram_retention() {
    let Some(ram) = METADATA
        .memory
        .iter()
        .find(|region| region.kind == MemoryKind::Ram && region.name == "RAM")
    else {
        panic!("{} has no RAM region named RAM", METADATA.name);
    };

    assert!(
        ram.retained_through >= PowerMode::Standby,
        "{}'s RAM is only retained through {:?}, so deep sleep would lose .data and .bss",
        METADATA.name,
        ram.retained_through,
    );
}

/// Check the NVIC priority width the HAL was compiled for against the chip.
///
/// The width is a Cargo feature (`embassy-hal-internal/prio-bits-2`), so it cannot be selected from
/// metadata.
fn check_nvic_priority_bits() {
    const HAL_PRIO_BITS: u8 = 2;

    let bits = METADATA.nvic_priority_bits;
    assert_eq!(
        bits, HAL_PRIO_BITS,
        "{} has {bits} NVIC priority bits, but embassy-mspm0 depends on \
         embassy-hal-internal/prio-bits-{HAL_PRIO_BITS}",
        METADATA.name,
    );
}

#[derive(Debug, Clone)]
struct Singleton {
    name: String,

    /// `#[cfg]` guard which enables this singleton instance to be obtained.
    cfg: Option<TokenStream>,
}

impl PartialEq for Singleton {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
    }
}

impl Eq for Singleton {}

impl PartialOrd for Singleton {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Singleton {
    fn cmp(&self, other: &Self) -> Ordering {
        self.name.cmp(&other.name)
    }
}

fn get_singletons(cfgs: &mut common::CfgSet) -> Vec<Singleton> {
    let mut singletons = Vec::<Singleton>::new();

    for peripheral in METADATA.peripherals {
        // Some peripherals do not generate a singleton, but generate a singleton for each pin.
        let skip_peripheral_singleton = match peripheral.kind {
            "gpio" => {
                // Also enable ports that are present.
                match peripheral.name {
                    "GPIOB" => cfgs.enable("gpio_pb"),
                    "GPIOC" => cfgs.enable("gpio_pc"),
                    _ => (),
                }

                true
            }

            // Each channel gets a singleton, handled separately.
            "dma" => true,

            // These peripherals do not exist as singletons, and have no signals but are managed
            // by the HAL.
            "iomux" | "cpuss" => true,

            // Unicomm instances get their own singletons, but we need to enable a cfg for unicomm drivers.
            "unicomm" => {
                cfgs.enable("unicomm");
                false
            }

            // TODO: Remove after TIMB is fixed
            "tim" if peripheral.name.starts_with("TIMB") => true,

            _ => false,
        };

        if !skip_peripheral_singleton {
            singletons.push(Singleton {
                name: peripheral.name.to_string(),
                cfg: None,
            });
        }

        // Generate each GPIO pin singleton
        if peripheral.name.starts_with("GPIO") {
            for pin in peripheral.pins {
                let singleton = make_valid_identifier(&pin.signal);
                singletons.push(singleton);
            }
        }
    }

    // TODO: Generate this more generally for other signals (e.g. FCC_IN, HFCLKIN, HFXIN, HFXOUT, etc)
    // Generate CLK_OUT manually for SYSCTL.
    singletons.push(Singleton {
        name: String::from("CLK_OUT"),
        cfg: None,
    });

    // DMA channels get their own singletons
    for dma_channel in METADATA.dma_channels.iter() {
        singletons.push(Singleton {
            name: format!("DMA_CH{}", dma_channel.number),
            cfg: None,
        });
    }

    singletons.sort_by(|a, b| a.name.cmp(&b.name));
    singletons
}

fn make_valid_identifier(s: &str) -> Singleton {
    let name = s.replace('+', "_P").replace("-", "_N");

    Singleton { name, cfg: None }
}

fn generate_pincm_mapping() -> TokenStream {
    let pincms = METADATA.pins.iter().map(|mapping| {
        let port_letter = mapping.pin.strip_prefix("P").unwrap();
        let port_base = (port_letter.chars().next().unwrap() as u8 - b'A') * 32;
        // This assumes all ports are single letter length.
        // This is fine unless TI releases a part with 833+ GPIO pins.
        let pin_number = mapping.pin[2..].parse::<u8>().unwrap();

        let num = port_base + pin_number;

        // But subtract 1 since pincm indices start from 0, not 1.
        let pincm = Literal::u8_unsuffixed(mapping.pincm - 1);
        quote! {
            #num => #pincm
        }
    });

    quote! {
        #[doc = "Get the mapping from GPIO pin port to IOMUX PINCM index. This is required since the mapping from IO to PINCM index is not consistent across parts."]
        pub(crate) fn gpio_pincm(pin_port: u8) -> u8 {
            match pin_port {
                #(#pincms),*,
                _ => unreachable!(),
            }
        }
    }
}

fn generate_pin() -> TokenStream {
    let pin_impls = METADATA.pins.iter().map(|pin| {
        let name = Ident::new(&pin.pin, Span::call_site());
        let port_letter = pin.pin.strip_prefix("P").unwrap();
        let port_letter = port_letter.chars().next().unwrap();
        let pin_number = Literal::u8_unsuffixed(pin.pin[2..].parse::<u8>().unwrap());

        let port = Ident::new(&format!("Port{}", port_letter), Span::call_site());

        // TODO: Feature gate pins that can be used as NRST

        // `None` is a gap in the vendor data, not a pin that cannot wake; arm it rather than making the
        // SHUTDOWN-wake API uncompilable on the families whose sysconfig omits `io_wakeup`.
        let wake_capable = pin.wakeup.unwrap_or(true).then(|| {
            quote! { impl_wake_capable_pin!(#name); }
        });

        quote! {
            impl_pin!(#name, crate::gpio::Port::#port, #pin_number);
            #wake_capable
        }
    });

    quote! {
        #(#pin_impls)*
    }
}

/// Whether a timer instance keeps being clocked in STANDBY1, and so can wake the core from it.
fn clocked_in_standby1(name: &str) -> bool {
    METADATA
        .peripherals
        .iter()
        .find(|peripheral| peripheral.name == name)
        .and_then(|peripheral| peripheral.clocked_in_standby1)
        .unwrap_or(false)
}

fn time_driver(singletons: &mut Vec<Singleton>, cfgs: &mut CfgSet) {
    let low_power = env::var_os("CARGO_FEATURE_LOW_POWER").is_some();

    // Every one of these is declared on every chip, not just the ones that have the timer:
    // `time_driver/tim.rs` names all of them unconditionally, and an undeclared cfg warns.
    for timer in TIME_DRIVER_TIMERS {
        cfgs.declare(&format!("time_driver_{}", timer.to_lowercase()));
    }

    let time_driver = match env::vars()
        .map(|(a, _)| a)
        .filter(|x| x.starts_with("CARGO_FEATURE_TIME_DRIVER_"))
        .get_one()
    {
        Ok(x) => Some(
            x.strip_prefix("CARGO_FEATURE_TIME_DRIVER_")
                .unwrap()
                .to_ascii_lowercase(),
        ),
        Err(GetOneError::None) => None,
        Err(GetOneError::Multiple) => panic!("Multiple time-driver-xxx Cargo features enabled"),
    };

    // Verify the selected timer is available
    let selected_timer = match time_driver.as_ref().map(|x| x.as_ref()) {
        None => "",
        Some("any") => {
            // Order of timer candidates:
            // 1. Basic timers
            // 2. 16-bit, 2 channel
            // 3. 16-bit, 2 channel with shadow registers
            // 4. 16-bit, 4 channel
            // 5. 16-bit with QEI
            // 6. Advanced timers
            //
            // 32-bit timers are deliberately absent: TIMG12/TIMG13 are usually the only 32-bit timers on
            // a part, and taking one here removes it from `Peripherals` entirely. Select by name to trade
            // the 32-bit counter's much longer period for losing it as a capture or compare timer.
            const CANDIDATES: &[&str] = &[
                // basic timers. No PWM pins
                // "TIMB0", // 16-bit, 2 channel
                "TIMG0", "TIMG1", "TIMG2", "TIMG3", // 16-bit, 2 channel with shadow registers
                "TIMG4", "TIMG5", "TIMG6", "TIMG7",  // 16-bit, 4 channel
                "TIMG14", // 16-bit with QEI
                "TIMG8", "TIMG9", "TIMG10", "TIMG11", // Advanced timers
                "TIMA0", "TIMA1",
            ];

            let available = |tim: &&&str| singletons.iter().any(|s| s.name == **tim);

            // A low-power build has to wake from STANDBY1, so prefer a timer that is still clocked
            // there. Every family has at least one that this list can select, but fall back rather
            // than fail: the const assertion in the time driver is what actually enforces it.
            CANDIDATES
                .iter()
                .filter(|tim| !low_power || clocked_in_standby1(tim))
                .find(available)
                .or_else(|| CANDIDATES.iter().find(available))
                .expect("Could not find any timer")
        }
        // The 32-bit timers are reachable here but not through `any`: they are scarce — often the
        // only 32-bit timer on the part — so `any` leaves them for capture and compare. Naming one
        // trades that for the much longer period a 32-bit counter gives.
        Some(name) => TIME_DRIVER_TIMERS
            .iter()
            .find(|timer| timer.eq_ignore_ascii_case(name))
            .copied()
            .unwrap_or_else(|| panic!("unknown time_driver {name:?}")),
    };

    // Using a timer that doens't work in STANDBY locks the application out of deep-sleep the timer
    // won't survive. The power consumption increase is easy to miss, so warn about it.
    let allow_sleep_floor = env::var_os("CARGO_FEATURE_ALLOW_TIME_DRIVER_SLEEP_FLOOR").is_some();

    if low_power && !allow_sleep_floor && !selected_timer.is_empty() && !clocked_in_standby1(selected_timer) {
        let usable = TIME_DRIVER_TIMERS
            .iter()
            .filter(|tim| clocked_in_standby1(tim) && singletons.iter().any(|s| &s.name == **tim))
            .map(|tim| format!("time-driver-{}", tim.to_lowercase()))
            .collect::<Vec<_>>()
            .join(", ");

        println!(
            "cargo:warning={selected_timer} on {chip} is not active in STANDBY1, so the time driver will \
             prevent deep-sleep that would lose the timer. For full sleep depth use one of: {usable} \
             (or `time-driver-any`, which selects a one available in STANDBY1). To keep this \
             timer and silence this warning, enable the `allow-time-driver-sleep-floor` feature.",
            chip = METADATA.name,
        );
    }

    if !selected_timer.is_empty() {
        cfgs.enable(format!("time_driver_{}", selected_timer.to_lowercase()));
    }

    let pin_suffixes = ["_CCP", "_FAULT", "_IDX"];

    // Apply cfgs to each timer and it's pins
    for singleton in singletons.iter_mut() {
        if singleton.name.starts_with("TIM") {
            // Remove suffixes for pin singletons.
            let name = pin_suffixes
                .into_iter()
                .filter_map(|suffix| singleton.name.strip_suffix(suffix))
                .next()
                .unwrap_or(&singleton.name);

            let feature = format!("time-driver-{}", name.to_lowercase());

            if singleton.name.contains(selected_timer) {
                singleton.cfg = Some(quote! { #[cfg(not(any(feature = "time-driver-any", feature = #feature)))] });
            } else {
                singleton.cfg = Some(quote! { #[cfg(not(feature = #feature))] });
            }
        }
    }
}

fn pin_features(singletons: &mut Vec<Singleton>) {
    let sysctl = METADATA
        .peripherals
        .iter()
        .find(|p| p.name == "SYSCTL")
        .expect("no SYSCTL peripheral");

    // Some packages make NRST share a physical pin with a GPIO.
    if let Some(pin) = sysctl.pins.iter().find(|p| p.signal == "NRST" && p.pin != "NRST") {
        let pin = singletons
            .iter_mut()
            .find(|s| s.name == pin.pin)
            .expect("Could not find NRST pin to cfg gate");

        pin.cfg = Some(quote! { #[cfg(feature = "nrst-pin-as-gpio")] });
    }

    let debugss = METADATA
        .peripherals
        .iter()
        .find(|p| p.name == "DEBUGSS")
        .expect("Could not find DEBUGSS peripheral");

    for pin in debugss.pins.iter() {
        let pin = singletons
            .iter_mut()
            .find(|s| s.name == pin.pin)
            .expect("Could not find SWD pin to cfg gate");

        pin.cfg = Some(quote! { #[cfg(feature = "swd-pins-as-gpio")] });
    }
}

fn generate_singletons(singletons: &[Singleton]) -> TokenStream {
    let singletons_peripherals_struct = singletons
        .iter()
        .map(|s| {
            let cfg = s.cfg.clone().unwrap_or_default();

            let ident = format_ident!("{}", s.name);

            quote! {
                #cfg
                #ident
            }
        })
        .collect::<Vec<_>>();

    let singletons_peripherals_def = singletons
        .iter()
        .map(|s| {
            let ident = format_ident!("{}", s.name);

            quote! {
                #ident
            }
        })
        .collect::<Vec<_>>();

    quote! {
        embassy_hal_internal::peripherals_definition!(#(#singletons_peripherals_def),*);
        embassy_hal_internal::peripherals_struct!(#(#singletons_peripherals_struct),*);
    }
}

fn generate_timers() -> TokenStream {
    // Generate timers
    let timer_impls = METADATA
        .peripherals
        .iter()
        .filter_map(|peripheral| peripheral.timer.map(|timer| (peripheral, timer)))
        // The basic timers are a bare counter with no capture/compare block, which is what
        // `ccp_channels == 0` says. `tim` has no driver for one, and their registers are laid out
        // differently from the `tim_v1` block the metapac maps them to, so they get no impls at all.
        .filter(|(_, timer)| timer.ccp_channels > 0)
        .flat_map(|(peripheral, timer)| {
            let name = Ident::new(&peripheral.name, Span::call_site());

            let word = match timer.bits {
                16 => quote! { u16 },
                32 => quote! { u32 },
                bits => panic!("{} has a {bits}-bit counter, which has no `tim::Word`", peripheral.name),
            };

            let mut impls = Vec::new();
            let prescaler = timer.prescaler;
            let channels = timer.ccp_channels;

            impls.push(quote! {
                impl_tim_instance!(
                    #name,
                    prescaler: #prescaler,
                    word: #word,
                    channels: #channels
                );
            });

            if timer.bits == 32 {
                impls.push(quote! {
                    impl_tim_instance_general_32bit!(#name);
                });
            }

            if timer.ccp_channels >= 2 {
                impls.push(quote! {
                    impl_tim_instance_general_2ch!(#name);
                });
            }

            if timer.ccp_channels >= 4 {
                impls.push(quote! {
                    impl_tim_instance_general_4ch!(#name);
                });
            }

            // Deadband insertion and a fault handler are what the advanced-timer driver programs,
            // and only the `TIMA` instances have them.
            if timer.deadband && timer.fault_handler {
                impls.push(quote! {
                    impl_tim_instance_advanced!(#name);
                });
            }

            impls
        });

    quote! {
        #(#timer_impls)*
    }
}

fn generate_interrupts() -> TokenStream {
    // Generate interrupt module
    let interrupts: Vec<Ident> = METADATA
        .interrupts
        .iter()
        .map(|interrupt| Ident::new(interrupt.name, Span::call_site()))
        .collect();

    let group_interrupt_enables = METADATA
        .interrupts
        .iter()
        .filter(|interrupt| interrupt.name.contains("GROUP"))
        .map(|interrupt| {
            let name = Ident::new(interrupt.name, Span::call_site());

            quote! {
                crate::interrupt::typelevel::#name::enable();
            }
        });

    // Generate interrupt enables for groups
    quote! {
        embassy_hal_internal::interrupt_mod! {
            #(#interrupts),*
        }

        pub fn enable_group_interrupts(_cs: critical_section::CriticalSection) {
            use crate::interrupt::typelevel::Interrupt;

            // This is empty for C1105/6
            #[allow(unused_unsafe)]
            unsafe {
                #(#group_interrupt_enables)*
            }
        }
    }
}

fn power_domain_ident(domain: &PowerDomain) -> Ident {
    format_ident!(
        "{}",
        match domain {
            PowerDomain::Pd0 => "Pd0",
            PowerDomain::Pd1 => "Pd1",
            PowerDomain::Backup => "Backup",
        }
    )
}

fn power_mode_tokens(mode: Option<PowerMode>) -> TokenStream {
    match mode {
        None => quote! { None },
        Some(mode) => {
            let variant = format_ident!(
                "{}",
                match mode {
                    PowerMode::Run => "Run",
                    PowerMode::Sleep => "Sleep",
                    PowerMode::Stop => "Stop",
                    PowerMode::Standby => "Standby",
                    PowerMode::Shutdown => "Shutdown",
                }
            );
            quote! { Some(crate::sysctl::PowerMode::#variant) }
        }
    }
}

fn optional_bool_tokens(value: Option<bool>) -> TokenStream {
    match value {
        None => quote! { None },
        Some(value) => quote! { Some(#value) },
    }
}

/// Implement `LowPowerInstance` for every peripheral singleton, and emit the wake-up latency.
fn generate_low_power(singletons: &[Singleton]) -> TokenStream {
    let max_wake_ns = max_wake_ns();

    let impls = singletons.iter().filter_map(|singleton| {
        let name = singleton.name.as_str();

        // A pin's domain says nothing useful: GPIO logic is in PD0 on every chip.
        if METADATA.pins.iter().any(|pin| pin.pin == name) {
            return None;
        }

        // Singletons without a metadata entry of their own inherit from the peripheral they belong to.
        let owner = match name {
            "CLK_OUT" => "SYSCTL",
            _ if name.starts_with("DMA_CH") => "DMA",
            _ => name,
        };

        let peripheral = METADATA
            .peripherals
            .iter()
            .find(|peripheral| peripheral.name == owner)
            .unwrap_or_else(|| panic!("no metadata for singleton {name} (looked for peripheral {owner})"));

        let peri = format_ident!("{}", name);
        let sleep = sleep_info_tokens(peripheral);

        Some(quote! { impl_low_power!(#peri, #sleep); })
    });

    quote! {
        #(#impls)*

        pub const MAX_WAKE_NS: u32 = #max_wake_ns;
    }
}

/// Longest wake-up latency the datasheet publishes for a deep-sleep mode, in nanoseconds.
///
/// One number rather than the per-mode table: the spread between modes is under a tick (7.2 to 20 µs
/// against 30.5 µs a tick), so a per-mode threshold could not tell them apart. It also covers the
/// modes the datasheet omits a figure for, such as STOP0 on L110x/L13xx.
fn max_wake_ns() -> u32 {
    let wake = METADATA.wake_ns;

    [wake.stop0, wake.stop1, wake.stop2, wake.standby0, wake.standby1]
        .into_iter()
        .flatten()
        .max()
        .unwrap_or_else(|| panic!("{} publishes no deep-sleep wake-up latency", METADATA.name))
}

fn sleep_info_tokens(peripheral: &Peripheral) -> TokenStream {
    let domain = power_domain_ident(&peripheral.power_domain);
    let retained_through = power_mode_tokens(peripheral.retained_through);
    let usable_through = power_mode_tokens(peripheral.usable_through);
    let block_async = optional_bool_tokens(peripheral.block_async);
    let clocked_in_standby1 = optional_bool_tokens(peripheral.clocked_in_standby1);

    quote! {
        crate::sysctl::SleepInfo {
            power_domain: crate::sysctl::PowerDomain::#domain,
            retained_through: #retained_through,
            usable_through: #usable_through,
            block_async: #block_async,
            clocked_in_standby1: #clocked_in_standby1,
        }
    }
}

fn generate_peripheral_instances() -> TokenStream {
    let mut impls = Vec::<TokenStream>::new();

    for peripheral in METADATA.peripherals {
        let peri = format_ident!("{}", peripheral.name);
        let fifo_size = peripheral.sys_fentries;

        let tokens = match peripheral.kind {
            "uart" => Some(quote! { impl_uart_instance!(#peri); }),
            "i2c" => Some(quote! { impl_i2c_instance!(#peri, #fifo_size); }),
            "wwdt" => Some(quote! { impl_wwdt_instance!(#peri); }),
            "adc" => Some(quote! { impl_adc_instance!(#peri); }),
            "mathacl" => Some(quote! { impl_mathacl_instance!(#peri); }),
            _ => None,
        };

        if let Some(tokens) = tokens {
            impls.push(tokens);
        }
    }

    // DMA channels
    for dma_channel in METADATA.dma_channels.iter() {
        let peri = format_ident!("DMA_CH{}", dma_channel.number);
        let num = dma_channel.number;

        if dma_channel.full {
            impls.push(quote! { impl_full_dma_channel!(#peri, #num); });
        } else {
            impls.push(quote! { impl_dma_channel!(#peri, #num); });
        }
    }

    quote! {
        #(#impls)*
    }
}

fn generate_pin_trait_impls() -> TokenStream {
    let mut impls = Vec::<TokenStream>::new();

    for peripheral in METADATA.peripherals {
        for pin in peripheral.pins {
            let key = (peripheral.kind, pin.signal);

            let pin_name = format_ident!("{}", pin.pin);
            let peri = format_ident!("{}", peripheral.name);
            let pf = pin.pf;

            let tokens = match key {
                ("adc", s) => {
                    let signal = s.parse::<u8>().unwrap();
                    Some(quote! { impl_adc_pin!(#peri, #pin_name, #signal); })
                }
                ("i2c", "SDA") => Some(quote! { impl_i2c_sda_pin!(#peri, #pin_name, #pf); }),
                ("i2c", "SCL") => Some(quote! { impl_i2c_scl_pin!(#peri, #pin_name, #pf); }),
                ("sysctl", "CLK_OUT") => Some(quote! { impl_clk_out_pin!(#pin_name, #pf); }),
                ("tim", "CCP0") => Some(quote! { impl_tim_pin!(#peri, #pin_name, #pf, Ch0); }),
                ("tim", "CCP1") => Some(quote! { impl_tim_pin!(#peri, #pin_name, #pf, Ch1); }),
                ("tim", "CCP2") => Some(quote! { impl_tim_pin!(#peri, #pin_name, #pf, Ch2); }),
                ("tim", "CCP3") => Some(quote! { impl_tim_pin!(#peri, #pin_name, #pf, Ch3); }),
                ("tim", "CCP0_CMPL") => Some(quote! { impl_tim_pin!(#peri, #pin_name, #pf, CompCh0); }),
                ("tim", "CCP1_CMPL") => Some(quote! { impl_tim_pin!(#peri, #pin_name, #pf, CompCh1); }),
                ("tim", "CCP2_CMPL") => Some(quote! { impl_tim_pin!(#peri, #pin_name, #pf, CompCh2); }),
                ("tim", "CCP3_CMPL") => Some(quote! { impl_tim_pin!(#peri, #pin_name, #pf, CompCh3); }),
                ("uart", "TX") => Some(quote! { impl_uart_tx_pin!(#peri, #pin_name, #pf); }),
                ("uart", "RX") => Some(quote! { impl_uart_rx_pin!(#peri, #pin_name, #pf); }),
                ("uart", "CTS") => Some(quote! { impl_uart_cts_pin!(#peri, #pin_name, #pf); }),
                ("uart", "RTS") => Some(quote! { impl_uart_rts_pin!(#peri, #pin_name, #pf); }),

                _ => None,
            };

            if let Some(tokens) = tokens {
                impls.push(tokens);
            }
        }
    }

    quote! {
        #(#impls)*
    }
}

fn select_gpio_features(cfgs: &mut CfgSet) {
    cfgs.declare_all(&[
        "gpioa_interrupt",
        "gpioa_group",
        "gpiob_interrupt",
        "gpiob_group",
        "gpioc_group",
    ]);

    // A GPIO port either owns an NVIC line or shares an interrupt group, and `gpio.rs` needs a
    // different handler for each. A peripheral can raise more than one interrupt, so classify every
    // one it has rather than assuming a single line — a port that somehow had both kinds would need
    // both handlers, and `gpio.rs` rejects that pair with a `compile_error!`.
    for (peripheral, interrupt) in METADATA
        .peripherals
        .iter()
        .filter(|p| p.kind == "gpio")
        .flat_map(|p| p.interrupts.iter().map(move |interrupt| (p, interrupt)))
    {
        let grouped = interrupt.group_iidx.is_some();

        match (peripheral.name, grouped) {
            ("GPIOA", false) => cfgs.enable("gpioa_interrupt"),
            ("GPIOA", true) => cfgs.enable("gpioa_group"),
            ("GPIOB", false) => cfgs.enable("gpiob_interrupt"),
            ("GPIOB", true) => cfgs.enable("gpiob_group"),
            ("GPIOC", true) => cfgs.enable("gpioc_group"),
            // No chip has a GPIOC on its own NVIC line, and `gpio.rs` has no handler for one, so it
            // would silently take no interrupts. Fail instead of losing them.
            (name, grouped) => panic!(
                "{name} has {} interrupt, which the GPIO driver has no handler for",
                if grouped { "a group" } else { "its own NVIC" }
            ),
        }
    }
}

/// rustfmt a given path.
/// Failures are logged to stderr and ignored.
fn rustfmt(path: impl AsRef<Path>) {
    let path = path.as_ref();
    match Command::new("rustfmt").args([path]).output() {
        Err(e) => {
            eprintln!("failed to exec rustfmt {:?}: {:?}", path, e);
        }
        Ok(out) => {
            if !out.status.success() {
                eprintln!("rustfmt {:?} failed:", path);
                eprintln!("=== STDOUT:");
                std::io::stderr().write_all(&out.stdout).unwrap();
                eprintln!("=== STDERR:");
                std::io::stderr().write_all(&out.stderr).unwrap();
            }
        }
    }
}

/// Timers a `time-driver-*` Cargo feature can name.
///
/// This is the portfolio-wide list, not the chip's: every entry is declared as a cfg on every chip
/// because `time_driver/tim.rs` refers to all of them, and asking for one the chip does not have is
/// caught by the singleton lookup instead.
///
/// What each timer *is* comes from `Peripheral::timer`, per instance and per device. Only the set of
/// selectable names lives here.
///
/// **No TIMB, deliberately.** A basic timer has no capture/compare, so an alarm would mean writing
/// `LD` on the same counter the clock is read from; and SLAU847 §29.1.2 clocks every counter from the
/// bus clock, whose rate changes with the power mode, where the driver wants LFCLK so that STANDBY
/// does not stop it. Nothing loses by it: every device with a TIMB also has a TIMA and a TIMG.
const TIME_DRIVER_TIMERS: &[&str] = &[
    "TIMG0", "TIMG1", "TIMG2", "TIMG3", "TIMG4", "TIMG5", "TIMG6", "TIMG7", "TIMG8", "TIMG9", "TIMG10", "TIMG11",
    "TIMG12", "TIMG13", "TIMG14", "TIMA0", "TIMA1",
];

enum GetOneError {
    None,
    Multiple,
}

trait IteratorExt: Iterator {
    fn get_one(self) -> Result<Self::Item, GetOneError>;
}

impl<T: Iterator> IteratorExt for T {
    fn get_one(mut self) -> Result<Self::Item, GetOneError> {
        match self.next() {
            None => Err(GetOneError::None),
            Some(res) => match self.next() {
                Some(_) => Err(GetOneError::Multiple),
                None => Ok(res),
            },
        }
    }
}
