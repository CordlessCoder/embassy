# MSPM0G3507 low-power examples

Examples that idle in deep sleep, so they use `embassy-mspm0`'s own executor rather than the one from
`embassy-executor`. That is why they are a separate crate from `examples/mspm0g3507`: the two executors
both define `__pender`, so a single crate cannot enable `embassy-mspm0/executor-thread` alongside
`embassy-executor/platform-cortex-m`.

```bash
cargo run --release --bin min_sleep_gate
```

| Example | What it measures | Needs |
|---|---|---|
| `min_sleep_gate` | that `Config::min_sleep` keeps short sleeps out of a deep mode, by timing what each sleep costs | one board |
| `wake_latency` | how long each sleep level takes to answer a pin edge, timed by the L1306 | `wake_latency_host` on an L1306 |
| `wake_latency_irq` | the same, with the answering task on an interrupt-mode executor — an A/B against the above | `wake_latency_host` on an L1306 |
| `wake_latency_probe` | splits a wake into silicon, GPIO handler and executor, via marker pins the HAL drives | the above, plus an analyser on `PB13`, `PB0`, `PB1` |
| `wake_cycles` | the same split in CPU cycles, raising the interrupt from software instead of from a pin | one board, nothing wired |
| `uart3_retention` | that a PD1 `UART3` comes back configured after deep sleep | `uart3_retention_host` on an L1306 |
| `uart3_sleep_glitch` | that sleep entry does not corrupt a UART frame, at every level | analyser on `PB2` |

**The debug probe drops as soon as the device deep-sleeps**, and a device already running one of these
cannot be re-flashed normally — recover it with a mass erase in UniFlash. Each example waits a few
seconds before its first sleep to leave a window for `probe-rs` to take the device back.

This crate enables `unsafe-atomics-single-core`, which takes a GPIO wake from 35.6 µs to 31.3 µs on this
part — time spent out of sleep, so it is current. `wake_latency_probe` is what measures it, and its module
doc says how the segments divide up.
