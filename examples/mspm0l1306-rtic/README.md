# RTIC examples for MSPM0L1306

Run individual examples with
```
cargo run --bin <module-name>
```
for example
```
cargo run --bin blinky
```

Written for the [LP-MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) board. Neither example needs
wiring: `LED1` is on `PA0` and `S2` on `PA14`, both on the board itself.

| Example | What it shows |
|---|---|
| `blinky` | the smallest RTIC application that uses `embassy_time` and an `embassy-mspm0` driver |
| `layered` | an RTIC hardware task preempting async tasks that wait on the HAL's own GPIO driver |
| `lp_idle` | `#[idle]` entering the HAL's deep sleep instead of spinning in `wfi` |

## RTIC on top of `embassy-mspm0`, rather than instead of it

`rt` stays **on** and the HAL keeps its own vector table. RTIC claims only the interrupts it is told to
— its dispatchers, and any `#[task(binds = ...)]` — and everything else reaches the HAL exactly as it
would in an ordinary embassy application. So an RTIC application gets the whole async driver set,
`embassy_time::Timer` and the GPIO edge waits without any of them being reimplemented, and keeps RTIC's
preemptive priorities and shared-resource locks for the work that wants them.

`#[app(device = embassy_mspm0)]` works directly: the crate root carries the `NVIC_PRIO_BITS` and
`Interrupt` that RTIC looks for.

Four things to know before writing one:

- **`embassy-time-queue-utils` is a direct dependency**, with a `generic-queue-*` feature. Nothing else
  backs `embassy_time`'s waker queue once the embassy executor is not the thing running the tasks.
- **The dispatchers must be interrupts nothing else uses.** `SPI0` and `I2C1` are free on this part as
  long as the application drives neither peripheral, and `time-driver-any` never selects them.
- **`-Tinterrupt_group.x` is required**, alongside `-Tlink.x`, as in any `embassy-mspm0` example's
  `build.rs`. Without it a binding fails at *link* time naming a peripheral the application never
  mentioned — the group demultiplexer resolves every source on the group, and that script is what
  provides the unbound ones. The error gives no hint that a linker script is missing.
- **A HAL interrupt outranks every RTIC task.** RTIC assigns priorities only to the interrupts it
  manages; the rest keep NVIC priority 0, the highest. So a driver's interrupt preempts a hardware task
  at any priority, which is usually what you want and is worth knowing either way.

## Sleeping, rather than spinning in `wfi`

RTIC has no idle policy of its own, and the bare `wfi` an `#[idle]` usually holds only reaches the
shallowest mode. `low_power::sleep` is the call the low-power executor makes on idle, and an RTIC
application can make it directly for the same sleep depths:

```rust
#[idle]
fn idle(_: idle::Context) -> ! {
    loop {
        critical_section::with(|cs| unsafe { embassy_mspm0::low_power::sleep(cs) });
    }
}
```

It needs the `low-power` feature on `embassy-mspm0`, and `critical-section` as a direct dependency
because the HAL does not re-export it. `low-power` pulls in no executor, so this is the whole cost.

**`#[idle]` is the only place it may be called.** `sleep` must run in thread mode — a `WFI` in a
handler is woken only by something of higher priority than the handler, so at the lowest priority it
never returns. RTIC's software tasks run in their dispatcher's handler, so the same call from a
`#[task]` compiles and hangs. `lp_idle` is the worked example.

## What this costs

`critical_section::with` — which the HAL uses throughout — masks **all** interrupts, because Cortex-M0+
has no `BASEPRI`. It is coarser than RTIC's own lock, so a HAL critical section briefly delays a
higher-priority RTIC task that shares nothing with it. The delay is bounded by the longest critical
section in the drivers in use. RTIC hits the same wall at its own masking ceiling on this core, so there
is nothing the HAL could offer that would be finer.
