# STM32U575ZI examples — companion for the MSPM0 work

These sit on the other end of a wire from an MSPM0 and are the side already trusted, so a disagreement
is evidence about the MSPM0 driver rather than about one HAL agreeing with itself.

`examples/stm32u5` covers the same family properly; it is pinned to `stm32u5g9zj`, and a chip feature is
crate-wide, which is the whole reason this crate is separate.

```bash
cargo run --release --bin i2c_controller
```

| Example | Pairs with | Tests |
|---|---|---|
| `i2c_controller` | MSPM0 `i2c_target` | our I2C **target**: command decoding, `respond_and_fill`, general call, the masked second address, recovery from an over-long read |
| `i2c_target` | MSPM0 `i2c_crosscheck` | our I2C **controller**: solved `Timing`, the restart in a write-read |
| `i2c_slow_target` | MSPM0 `i2c_faults` | the same, with a multi-millisecond clock stretch on one transaction in four — the condition [#6633](https://github.com/embassy-rs/embassy/pull/6633) reports as wedging the controller |
| `i2c_controller_10bit` | MSPM0 `i2c_target_10bit` | our I2C **target** on a 10-bit own address |
| `i2c_target_10bit` | MSPM0 `i2c_10bit` | our I2C **controller** in 10-bit mode, with a 7-bit address live at the same time so one binary can switch between them |
| `usart_echo` | MSPM0 `uart_crosscheck` | our UART: the solved baud divider, against a clock that is not ours |

Board: NUCLEO-U575ZI-Q. Every example runs the same 160 MHz PLL off HSI so the clock setup is never the
variable, and none of them touch an LED — liveness is in the log.

Pins were chosen to be on the Arduino headers and to avoid the LEDs and port G, which needs `VDDIO2`
powered before it will drive anything:

| Signal | Pin | Header |
|---|---|---|
| I2C1 SCL | `PB8` | D15 |
| I2C1 SDA | `PB9` | D14 |
| USART2 TX | `PA2` | A1 |
| USART2 RX | `PA3` | A0 |

USART2's alternate mapping. Its `PD5`/`PD6` pins reach only the ST morpho headers, which ship unsoldered,
so they are unusable out of the box.

**Do not substitute the pins marked RX and TX on CN9.** Those are D0/D1, which are `PG8`/`PG7`: LPUART1,
connected to the ST-LINK virtual COM port, and on the port that needs `VDDIO2` before it drives at all.
Any of the three gives silence or garbage that reads as a baud-rate fault.

**I2C needs pull-ups.** Neither board fits them: 4.7 kΩ from each of SCL and SDA to 3V3.
