# Examples for MSPM0L1306

Run individual examples with
```
cargo run --bin <module-name>
```
for example
```
cargo run --bin blinky
```

## Checklist before running examples
A large number of the examples are written for the [LP-MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) board.

You might need to adjust `.cargo/config.toml`, `Cargo.toml` and possibly update pin numbers or peripherals to match the specific MCU or board you are using.

* [ ] Update .cargo/config.toml with the correct probe-rs command to use your specific MCU. For example for L1306 it should be `probe-rs run --chip MSPM0L1306`. (use `probe-rs chip list` to find your chip)
* [ ] Update Cargo.toml to have the correct `embassy-mspm0` feature. For the LP-MSPM0L1306 it should be `mspm0l1306rhb`. Look in the `Cargo.toml` file of the `embassy-mspm0` project to find the correct feature flag for your chip.
* [ ] If your board has a special clock or power configuration, make sure that it is set up appropriately.
* [ ] If your board has different pin mapping, update any pin numbers or peripherals in the given example code to match your schematic

If you are unsure, please drop by the Embassy Matrix chat for support, and let us know:

* Which example you are trying to run
* Which chip and board you are using

Embassy Chat: https://matrix.to/#/#embassy-rs:matrix.org

## Examples that pair with another board

| Example | Other side |
|---|---|
| `wake_latency_host` | `wake_latency` on an LP-MSPM0G3507 |
| `uart3_retention_host` | `uart3_retention` on an LP-MSPM0G3507 |
| `i2c_target` | `i2c_controller` on a NUCLEO-U575ZI-Q, in `examples/stm32u575` |

`clock_tree` measures the programmed clock tree against LFCLK and needs no second board.

`i2c_rejects` checks the addresses and configurations the I2C driver refuses, all of which are settled
before the peripheral touches the bus, so it needs no wiring and no target.

`supply_monitor` reads VDD through the ADC's internal divider and needs no wiring either. It converts
the same channel against the internal reference and against the supply, one after the other, to show
what the second one is worth: a third of full scale at every supply, whatever the board is running at.

## Measurement examples

Two binaries measure the HAL rather than demonstrate it, and they are behind a `bench` feature because
they link **no logger at all**:

```
DEFMT_LOG=off cargo build --release --features bench --bin wake_edge
```

Keeping the logger out is most of what makes them measurements: `defmt-rtt`'s buffer is a kilobyte of
RAM, its encoder runs inside a critical section, and a probe attached to drain it holds the device out
of the idle being measured. Flash them with `probe-rs download` and start them with `probe-rs reset`,
so nothing stays attached while they run.

**Two separate things keep it out, and only one of them removes the kilobyte.** Not importing
`defmt-rtt` is what drops the transport — the ring buffer, the control block and the encoder.
`DEFMT_LOG=off` only compiles out the call sites, and it is needed here because the HAL's own logging
is enabled in this crate and would otherwise want a global logger that these binaries do not provide.
Setting the environment variable without dropping the import gets you a binary that logs nothing and
still spends the kilobyte.

Check the binary rather than the build command, because the failure is silent in the direction that
looks fine:

```
llvm-nm --defined-only --print-size target/thumbv6m-none-eabi/release/wake_edge | grep BUFFER
```

Nothing on these two. Every other binary here answers with a 0x400-byte `defmt_rtt::BUFFER`.

| Example | What it measures | Needs |
|---|---|---|
| `wake_edge` | edge-to-response through the async GPIO path, from an idle executor | analyser on two pins |
| `clock_witness` | the core rate, as a control for anything timed | analyser on one pin |

### `wake_edge`

The executor idles, a press on **S1** wakes it, and a pin toggles as the first thing after the wait
returns. The interval is the whole software path: interrupt entry, the port handler, the waker, the
executor hand-off and the poll that completes.

| channel | pin | header | carries |
|---|---|---|---|
| D0 | `PA18` — S1 | J2.26 | the stimulus. Idles low; a press drives it to 3V3 |
| D1 | `PA16` | J2.24 | the response. Toggles once per edge |

Ground the analyser to the board. Nothing else is wired and no second board is involved.

**S1 is active high where S2 is active low** — J11 gives `PA18` an external pulldown and the switch
connects it to 3V3. Copying `button.rs`'s `Pull::Up` and falling edge onto this pin gives a wait that
never completes *and* no edge on the analyser, because the internal pull-up holds the pin at the level
the press drives it to. The capture comes back clean and empty, which looks exactly like a dead lead.

Take the first response after a quiet period. Switch bounce produces further edges within a few
milliseconds and only the first is a wake from idle.

### `clock_witness`

Toggles the same pin with a fixed cycle delay between edges, so the half-period is a measure of the
core rate and nothing else. It exists because a latency difference between two builds is meaningless
until you know they ran at the same speed: **run it before believing any timing comparison.** One
source built against two versions of the HAL, half-periods agreeing to 0.1 µs in 5 ms, is what says a
difference in `wake_edge` is software rather than clocks.

### What these read on the reference board

Measured 2026-08-11 on an LP-MSPM0L1306, analyser at 50 MS/s, stable toolchain, the release profile in
this crate's `Cargo.toml`, and the default clock configuration.

| | median | min | max | σ | n | flash | RAM |
|---|---:|---:|---:|---:|---:|---:|---:|
| `wake_edge` | **13.90 µs** | 13.90 | 13.94 | 11 ns | 28 | 2140 | 104 |

Every stimulus edge produced exactly one response and no release edge produced one. The 40 ns spread is
two sample periods, so the measurement sits at the analyser's resolution rather than the firmware's.

For scale, the same application written against `embassy-mspm0` as it stood at `08b2f06d0` measured
**42.06 µs, 3264 bytes of flash and 176 bytes of RAM** on the same board and toolchain, with
`clock_witness` confirming both ran the core at the same rate. That is a dated comparison against one
commit, not a standing claim; re-running both arms is the point of these examples existing.
