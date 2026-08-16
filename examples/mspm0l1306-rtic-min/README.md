# `embassy-mspm0` drivers with no embassy runtime

Run individual examples with
```
cargo run --bin <module-name>
```
for example
```
cargo run --bin button
```

Written for the [LP-MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) board. Nothing needs wiring:
`LED1` is on `PA0` and `S2` on `PA14`, both on the board itself.

## What this crate is for

`examples/mspm0l1306-rtic` shows RTIC layered on an ordinary embassy application, keeping
`embassy_time` and the whole async driver set. This crate is the other end of that scale: the async
drivers with **no embassy runtime crate in the build at all**.

There is no `embassy-executor`, no `embassy-time`, no `embassy-time-driver` and no
`embassy-time-queue-utils`. What remains under `embassy-mspm0` is `embassy-hal-internal`,
`embassy-sync`, `embassy-futures` and `embassy-embedded-hal` — no-std support crates, not a runtime.
`cargo tree` is the check, and it is worth re-running after any dependency change.

Nothing about this needs a HAL feature that the layered pattern lacks. An async driver parks on its
interrupt through the HAL's own waiter list and is polled back by whatever scheduler is present; RTIC
supplies one, so no time source is involved.

| Example | What it shows |
|---|---|
| `button` | an async GPIO edge wait, with logging |
| `button_nolog` | the same with no logger: the size floor |
| `lp_nolog` | deep sleep from `#[idle]` with no time driver |
| `drivers` | link-time proof that I2C and ADC async need no time source |
| `uart_min` | the same for the buffered UART |

`drivers` and `uart_min` are not meant to be run. They exist so that a driver growing a dependency on
`embassy-time` fails the build rather than being noticed later.

## Sizes

Built at `opt-level = "z"`, `lto = "fat"`, `codegen-units = 1`, `panic = "abort"` — the profile in this
crate's `Cargo.toml`. Figures taken any other way are not comparable; no LTO alone inflates a binary
this small by around 3x.

| binary | flash | RAM |
|---|---:|---:|
| `button_nolog` | 1696 | 12 |
| `lp_nolog` | 1792 | 12 |
| `button` | 6736 | 1104 |

The first row is a complete application: vector table, clock setup, GPIO driver, async edge wait and
RTIC's dispatcher. **Deep sleep costs 96 B on top of it and no RAM at all.**

The third row is the one to read carefully. It is the same application as the first with logging added,
and the logger costs about three times what everything else together does — most of that RAM being
`defmt-rtt`'s buffer. Quote a size for this HAL from a binary that links a logger and you are mostly
quoting the logger.

## Two traps

- **`default-features = false` needs an `atomics-*` feature re-listed.** Without one, RTIC fails to
  build on `AtomicBool::compare_exchange` wanting `HasCompareExchange`, and the error names
  `portable-atomic` rather than anything you wrote. This crate uses `unsafe-atomics-single-core`, which
  is sound here because the part is single-core.
- **`low_power::sleep` may only be called from `#[idle]`.** It has to run in thread mode; an RTIC
  software task runs in its dispatcher's interrupt handler, where a `WFI` at the lowest priority is
  never woken. The same call from a `#[task]` compiles and hangs.
- **A software task's future does not show up in a section-based RAM count.** RTIC allocates the task
  executor as a local in its generated `main` and publishes the pointer, so the future sits on a stack
  frame held for the life of the program instead of in `.bss`. Comparing `.bss` against an executor
  that puts its tasks in a static therefore reports a saving that is really a move, and a linker
  stack-floor check does not see it either. Read the frame size off `main`'s prologue and add it. Every
  binary here uses hardware tasks only, so none of them pays this.
