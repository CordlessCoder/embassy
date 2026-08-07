# Changelog for embassy-mspm0

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->
## Unreleased - ReleaseDate

- feat: Add I2C Controller (blocking & async) + examples for mspm0l1306, mspm0g3507 (tested MCUs) (#4435)
- fix gpio interrupt not being set for mspm0l110x
- feat: Add window watchdog implementation based on WWDT0, WWDT1 peripherals (#4574)
- feat: Add MSPM0C1105/C1106 support
- feat: Add adc implementation (#4646)
- fix: gpio OutputOpenDrain config (#4735)
- fix: add MSPM0C1106 to build test matrix
- feat: add MSPM0H3216 support
- feat: Add i2c target implementation (#4605)
- fix: group irq handlers must check for NO_INTR (#4785)
- feat: Add read_reset_cause function
- feat: Add module Mathacl & example for mspm0g3507 (#4897)
- feat: Add MSPM0G5187 support
- feat: add CPU accelerated division function (#4966)
- feat: Add trng implementation (#5172)
- fix: feature guard pins used for NRST and SWD (#5257)
- feat: Move from GPIO waker arrays to maitake-sync wait map
- fix: Flush the I2C controller FIFOs on the NACK/error paths, to prevent stale data
- fix: Only block deep sleep for PD1 drivers that actually lose their configuration, not for all of them
- fix: Hold a sleep guard across a software-triggered DMA transfer, which deep sleep would otherwise cut
- fix: mspm0/sysctl: only block deep sleep for PD1 drivers that actually lose their configuration, not for all of them
- fix: mspm0/dma: hold a sleep guard across a software-triggered transfer, which deep sleep would otherwise cut
- fix: mspm0/uart: `UartTx::blocking_flush` waited on an inverted condition and returned while the transmitter was still busy
- fix: mspm0/uart: `BufferedUartTx`'s blocking and async flush returned once the software buffer drained, before the hardware had sent it
- fix: mspm0/uart: apply the `UART_ERR_08` workaround on every affected family, not just three of the seven
- fix: mspm0/uart: avoid 3x oversampling on BUSCLK/MFCLK for L122x/L222x, per `UART_ERR_03`
- feat: mspm0/wwdt: add `Config::stop_in_sleep`, so the watchdog no longer has to count through deep sleep
- feat: mspm0/wwdt: add `Watchdog::run`, an async pet loop to spawn as a task, behind the new `time` feature
- fix: mspm0/wwdt: remove `Timeout::USec32250`, a misnamed duplicate of `USec31250`
- fix: mspm0: forward the `defmt` feature to `mspm0-metapac`, so PAC value types implement `defmt::Format`
- feat: mspm0: move to `rand_core` 0.10, whose `TryRng` replaces the `TryRngCore` the TRNG implemented
- fix: mspm0/trng: enable the driver on G310x parts, which have a TRNG but were left out of the family list gating the module
- fix: mspm0/wwdt: clear `WWDTLP1RSTDIS` on G5187, whose WWDT1 could not trigger a BOOTRST
- fix: mspm0/uart: apply the `UART_ERR_03` and `UART_ERR_08` workarounds from the device's errata sheet rather than a hand-written family list
- fix: mspm0: derive the SYSCTL capability cfgs from the SYSCTL peripheral version instead of chip family lists. An unrecognised version is now a build error rather than a silently reduced feature set
- fix: mspm0/gpio: enable the GPIOA interrupt on C1105/C1106 and H3216, where port A has its own NVIC line and took no interrupts at all
- feat: mspm0/sysctl: add a configurable clock tree via `Config::clock`, reported by `sysctl::clocks()`. Resolution is a `const fn`, so an out-of-range tree fails to compile
- feat: mspm0/sysctl: support HFXT and the SYSPLL, letting G-series parts reach 80 MHz MCLK. Flash wait states are programmed alongside the MCLK switch
- feat: mspm0/sysctl: derive the flash wait-state bands and the SYSOSC base frequency per device
- feat: mspm0/sysctl: skip publishing the resolved clock tree when it is the reset tree, which the static already holds
- **breaking** mspm0/sysctl: `MCLK_HZ` and `ULPCLK_HZ` are replaced by `sysctl::clocks()`; `bus_clock_hz` is no longer `const`
- **breaking** mspm0/tim: `ClockSel::frequency` takes the resolved `Clocks` as its first argument
- feat: mspm0/uart: add `Baud`, a baud-rate divider solved by a `const fn` and taken by `Config::with_baud`
- fix: mspm0/uart: compute the baud rate divider in 32-bit arithmetic, which also fixes a panic for clocks above 67.1 MHz
- feat: mspm0/i2c: add `Timing`, a timer period solved by a `const fn` and taken by `Config::with_timing`
- fix: mspm0/i2c: program the I2C target's clock source from the resolved configuration, which for a bus speed above 200 kHz differed from `Config::clock_source`
- fix: mspm0/tim: compute the PWM duty fraction without a 64-bit division
- fix: mspm0/adc: derive `FRANGE` and `SCLKDIV` from the configured sample clock rate instead of hardcoding the 32 MHz boot tree. The hardcoded `FRANGE` was already wrong on C-series parts, whose SYSOSC base is 24 MHz
- feat: mspm0/adc: add `SampleClock::Ulpclk` and `SampleClock::Hfclk`, and honour `Config::sample_clk`, which was ignored in favour of SYSOSC
- fix: mspm0/adc: check the sample clock against the device's `fADCCLK` instead of the span `FRANGE` can encode
- fix: mspm0/trng: take the divider bands from the device's `TRNGCLKF` range instead of the 9.5-20 MHz the TRM quotes
- fix: mspm0/time-driver: `Instant::now` stepped back a tick shortly after `init`, from a counter preload that had not crossed into the timer's clock domain
- feat: mspm0/low-power: support deep sleep on every chip family, rather than failing to compile on the ones without a hand-written entry sequence
- feat: mspm0/low-power: add `Config::min_sleep`, below which deep sleep is skipped for a plain `WFI`; defaults to four times the device's published wake-up latency
- feat: mspm0: add `unsafe-atomics-single-core`, emulating atomic read-modify-writes inline instead of through `critical-section`. Pair with `default-features = false`
- fix: mspm0/i2c: `set_config` left the interrupt disabled, so every async transfer after it completed on the bus and never woke the task
- fix: mspm0/i2c: apply the `I2C_ERR_13` settling delay before polling `CSR`, without which a controller transfer was checked before it started and a NACK came back as success
- fix: mspm0/i2c: the async entry guards wait on `CSTOP` instead of spinning on `BUSBSY`
- fix: mspm0/i2c: dropping an async transfer no longer wedges the peripheral; the next one resets the controller, which is the only thing that frees the bus afterwards
- fix: mspm0/i2c: `set_config` did not record the new configuration, so a later internal reset restored the previous one
- fix: mspm0/i2c: flush the FIFOs after a failed transfer, so its unsent byte is not transmitted by the next one
- feat: mspm0/i2c: add `Config::clock_low_timeout_us`, failing a transfer with `Error::Timeout` when a target holds SCL low; off by default
- feat: mspm0/i2c: report `Error::NackAddress` and `Error::NackData` where the controller says which went unanswered, from both the blocking and the async paths
- feat: mspm0/i2c: report `Error::BusStuck` when a target is holding SDA low, and add `I2c::recover_stuck_bus` to clock it off the bus plus `I2c::bus_is_stuck` to ask
- feat: mspm0/gpio: replace the `maitake-sync` wait map with a per-port list of waiters, which halves the cost of a GPIO wake and removes the dependency
- fix: mspm0/i2c: honour SLAU846's conditions for a FIFO flush — wait for the controller to go idle, and mask the FIFO interrupts across it
- **breaking** mspm0/i2c: addresses are now `Address`, taken as `impl Into<Address>`; a `u8` is 7-bit and a `u16` 10-bit, so an untyped integer literal needs a `u8` suffix
- feat: mspm0/i2c: support 10-bit addressing in the controller, including `embedded_hal::i2c::I2c<TenBitAddress>` for the blocking and async drivers
- **breaking** mspm0/i2c-target: `Config::target_addr` is an `Address`, and may be 10-bit
- feat: mspm0/i2c-target: add `Config::second_addr`, a second address to answer on with a mask covering a range of them, plus `I2cTarget::matched_address` to ask which one a command arrived on. Pairing it with a 10-bit primary address is rejected, `OAR2` being compared only in 7-bit mode
- **breaking** mspm0/trng: `Trng::new`, `new_fast` and `new_secure` take an interrupt binding, made with the new `bind_group_interrupts!`; without one the handler is no longer linked into every binary
- **breaking** mspm0/gpio: `Flex`, `Input` and `OutputOpenDrain` take a mode, and the edge waits move to the `Async` one, built with `new_async` and an interrupt binding. Without one the port handlers are no longer linked into every binary
- fix: mspm0: build on the C1105, C1106 and H3216 families, whose empty interrupt-group source list made the re-export an unused import
- fix: mspm0/i2c: solve the SCL-low timeout and the bus-recovery delays without 32- or 64-bit division, so a pre-solved `Timing` no longer links the software dividers — 1.7 kB off a blocking I2C binary
- feat: mspm0/tim: add `low_level::solve_load` and `simple_pwm::Config::load`, a `const` period that spares the device the divisions `Config::frequency` needs
- fix: mspm0/low-power: narrow the wake-distance check to 32 bits, which is all the time driver ever returns
- fix: mspm0/executor: drop the redundant memory barriers from the low-power idle path
