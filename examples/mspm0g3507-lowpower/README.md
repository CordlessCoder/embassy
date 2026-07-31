# MSPM0G3507 low-power examples

Examples that idle in deep sleep, so they use `embassy-mspm0`'s own executor rather than the one from
`embassy-executor`. That is why they are a separate crate from `examples/mspm0g3507`: the two executors
both define `__pender`, so a single crate cannot enable `embassy-mspm0/executor-thread` alongside
`embassy-executor/platform-cortex-m`.

```bash
cargo run --release --bin uart3_retention
```

**The debug probe drops as soon as the device deep-sleeps**, and a device already running one of these
cannot be re-flashed normally — recover it with a mass erase in UniFlash. Each example waits a few
seconds before its first sleep to leave a window for `probe-rs` to take the device back.
