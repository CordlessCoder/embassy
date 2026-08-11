# Embassy MSPM0 HAL

The embassy-mspm0 HAL aims to provide a safe, idiomatic hardware abstraction layer for all MSPM0 and MSPS003 chips.

* [Documentation](https://docs.embassy.dev/embassy-mspm0/) (**Important:** use docs.embassy.dev rather than docs.rs to see the specific docs for the chip you’re using!)
* [Source](https://github.com/embassy-rs/embassy/tree/main/embassy-mspm0)
* [Examples](https://github.com/embassy-rs/embassy/tree/main/examples)

## Embedded-hal

The `embassy-mspm0` HAL implements the traits from [embedded-hal](https://crates.io/crates/embedded-hal) (1.0) and [embedded-hal-async](https://crates.io/crates/embedded-hal-async), as well as [embedded-io](https://crates.io/crates/embedded-io) and [embedded-io-async](https://crates.io/crates/embedded-io-async).

## A note on feature flag names

Feature flag names for chips do not include temperature rating or distribution format.

Usually chapter 10 of your device's datasheet will explain the device nomenclature and how to decode it. Feature names in embassy-mspm0 only use the following from device nomenclature:
- MCU platform
- Product family
- Device subfamily
- Flash memory
- Package type

This means for a part such as `MSPM0G3507SPMR`, the feature name is `mspm0g3507pm`. This also means that `MSPM0G3507QPMRQ1` uses the feature `mspm0g3507pm`, since the Q1 parts are just qualified variants of the base G3507 with a PM (QFP-64) package.

## Interoperability

This crate can run on any executor.

## Idling on MSPM0 needs the prefetcher suspended

**`CPU_ERR_03` applies to every chip this crate supports.** The instruction prefetcher can fetch all
zeros when the device enters a low-power mode with a prefetch pending, and the advisory is written
against low-power modes rather than only the deep ones — plain `SLEEP`, which is what a bare `WFI` or
`WFE` enters, is in scope. It names the case that matters most: "a HW Event wake is another example of
a process that will wake the device, but not flush the prefetcher."

The workaround is to disable the prefetcher across the sleep, and **`embassy-executor`'s own Cortex-M
executor does not do it** — it idles on `WFE` with the prefetcher running. Nothing in this crate can
detect that: Cargo tells a build script nothing about a dependency's features, so there is no way to
warn you at build time.

So if the idle matters, use this crate's executor:

```toml
embassy-mspm0 = { version = "0.1.0", features = ["executor-thread", ...] }
```

```rust,ignore
#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
```

It suspends the prefetcher across every idle. Measured on one application that is **eight bytes** of
flash over the unguarded one. Add `low-power` on top and the idle also reaches a deep-sleep mode; leave
it off and it is a plain, guarded `WFI`. Under RTIC, call `low_power::sleep` from `#[idle]` for the
same thing.

**Whether the erratum bites in practice is not established here.** The corruption needs the prefetched
zeros to survive the wake, and an interrupt handler running from flash is likely to overwrite them,
which is the likeliest reason nobody has reported it. That is an argument about probability, not a
guarantee, and it has not been tested on silicon either way.
