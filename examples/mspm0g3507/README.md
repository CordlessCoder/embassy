# Examples for MSPM0M3507

Run individual examples with
```
cargo run --bin <module-name>
```
for example
```
cargo run --bin blinky
```

## Checklist before running examples
A large number of the examples are written for the [LP-MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) board.

You might need to adjust `.cargo/config.toml`, `Cargo.toml` and possibly update pin numbers or peripherals to match the specific MCU or board you are using.

* [ ] Update .cargo/config.toml with the correct probe-rs command to use your specific MCU. For example for G3507 it should be `probe-rs run --chip MSPM0G3507`. (use `probe-rs chip list` to find your chip)
* [ ] Update Cargo.toml to have the correct `embassy-mspm0` feature. For the LP-MSPM0G3507 it should be `mspm0g3507pm`. Look in the `Cargo.toml` file of the `embassy-mspm0` project to find the correct feature flag for your chip.
* [ ] If your board has a special clock or power configuration, make sure that it is set up appropriately.
* [ ] If your board has different pin mapping, update any pin numbers or peripherals in the given example code to match your schematic

If you are unsure, please drop by the Embassy Matrix chat for support, and let us know:

* Which example you are trying to run
* Which chip and board you are using

Embassy Chat: https://matrix.to/#/#embassy-rs:matrix.org

## Examples that pair with another board

| Example | Other side |
|---|---|
| `i2c_crosscheck` | `i2c_target` on a NUCLEO-U575ZI-Q, in `examples/stm32u575` |
| `uart_crosscheck` | `usart_echo` on a NUCLEO-U575ZI-Q |
| `i2c_target` | `i2c_controller` on a NUCLEO-U575ZI-Q |

`clock_syspll` runs MCLK at 80 MHz from the SYSPLL and measures it against LFCLK; it needs no second
board, but `CLK_OUT` only moves after a reset or power cycle, since `probe-rs` does not issue SYSRST.
