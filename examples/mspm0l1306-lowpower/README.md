# MSPM0L1306 low-power examples

Examples that idle in deep sleep, so they use `embassy-mspm0`'s own executor rather than the one from
`embassy-executor`. That is why they are a separate crate from `examples/mspm0l1306`: the two executors
both define `__pender`, so a single crate cannot enable `embassy-mspm0/executor-thread` alongside
`embassy-executor/platform-cortex-m`.

```bash
cargo run --release --bin min_sleep_gate
```

| Example | What it measures | Needs |
|---|---|---|
| `min_sleep_gate` | that `Config::min_sleep` keeps short sleeps out of a deep mode, by timing what each sleep costs | one board |

**The debug probe drops as soon as the device deep-sleeps**, and a device already running one of these
cannot be re-flashed normally — recover it with a mass erase in UniFlash. Each example waits a few
seconds before its first sleep to leave a window for `probe-rs` to take the device back.

The L-series counterpart to `examples/mspm0g3507-lowpower`. Running the same measurement on both is the
point: the two parts are different SYSCTL families with different published wake-up latencies, and this
one has no STOP0 figure at all.

This crate enables `unsafe-atomics-single-core` and `gpio-embassy-tasks-only`, which shorten every GPIO wake
and so the time spent out of sleep. Measured on a G3507, not here — see `wake_latency` in
`examples/mspm0g3507-lowpower`.
