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
