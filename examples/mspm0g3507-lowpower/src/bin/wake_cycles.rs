//! Times the GPIO wake path in CPU cycles, with no wire, no analyser and no sleep.
//!
//! `GPIO.CPU_INT.ISET` sets a pin's interrupt exactly as an edge would — "allows interrupts to be set by
//! software (useful in diagnostics and safety checks)", SLAU846 9.3.14 — so everything from interrupt
//! entry to the waiting task resuming can be triggered from code. SysTick times it at one cycle.
//!
//! This measures the software tail, which is the part worth optimising and the part that is hard to see:
//! `TESTING.md` B3c found everything after the handler constant to within 20 ns across all six sleep
//! levels, so running in RUN costs nothing. What it cannot see is the silicon wake, the one segment that
//! does depend on sleep depth — for that, use `wake_latency` and the two-board rig.
//!
//! # Cases
//!
//! | case | what the window holds |
//! |---|---|
//! | `entry` | interrupt entry, `irq_handler`, and the wake call. The task is not resumed. |
//! | `poll` | the future's completing poll, called inline, with no executor in between |
//! | `dispatch` | one whole wake taken from another task, so the executor's hand-off is included |
//!
//! `entry` and `poll` are stamped by the task that is waiting, which polls its own future by hand — so no
//! executor runs between them. `dispatch` is the same wake driven from a second task, so
//! `dispatch − (entry + poll)` is the executor's share, the largest item in the analyser's breakdown.
//!
//! Note what `poll` contains: an async fn drops its locals when it completes, so the edge detection is
//! disarmed inside that final poll rather than later. Arming the *next* wait is what falls outside every
//! window here.
//!
//! # What to check
//!
//! - **`min`, not `mean`.** The time driver's interrupt lands inside a window now and then; the minimum
//!   is the path with nothing else in it. `dispatch` is bimodal by about one pass of the executor loop,
//!   depending on where the injecting task sits in the run queue, so compare minima across builds.
//! - **The interrupt has to be taken inside the window.** `inject` ends with `dsb`/`isb` for that reason —
//!   without them the store is still in flight, the handler lands in the next window, and `entry` reads a
//!   flat 12 cycles.
//! - **`dispatch` moving when the executor does not.** The hand-off figure is the only one that contains
//!   no HAL code, so it should not move when only `embassy-mspm0` changes. It is a check on the
//!   measurement, not a result.
//!
//! Rebuild against a different `embassy-mspm0` feature set to compare wake mechanisms — the choice is
//! crate-wide, so that means editing this crate's `Cargo.toml` and reflashing, one build per variant.

#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::pin::{Pin, pin};
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use cortex_m::peripheral::SYST;
use cortex_m::peripheral::syst::SystClkSource;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{self, Input, Pull};
use embassy_mspm0::mode::Async;
use embassy_mspm0::{bind_group_interrupts, pac};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use panic_halt as _;

/// Which pin the wait is armed on. Nothing is connected to it — the interrupt is raised by writing
/// `ISET` — but it is held high by its pull-up, because `GPIO_ERR_01` makes the handler classify an
/// edge by the level the pin settled at, and a rising edge has to find it high.
const WAKE_BIT: usize = 7;

/// Wakes per case. Enough for the minimum to be the uncontended path rather than luck.
const WAKES: u32 = 64;

/// Set by the injecting task immediately before it raises the interrupt.
static STAMP: AtomicU32 = AtomicU32::new(0);

/// Tells the injector that the waiting task is about to arm.
static ARMED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// SysTick counts down and wraps at its reload value.
fn elapsed(from: u32, to: u32) -> u32 {
    from.wrapping_sub(to) & 0x00ff_ffff
}

/// Raise `WAKE_BIT`'s interrupt as an edge on the pin would, and do not return until it has been taken.
///
/// Without the barriers the store is still in flight when the next instruction retires, so the handler
/// runs a few cycles into whatever is timed next — which reads as a 12-cycle interrupt entry and a
/// suspiciously expensive poll.
fn inject() {
    pac::GPIOB.cpu_int().iset().write(|w| w.set_dio(WAKE_BIT, true));

    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Smallest, mean and largest of a run, as cycles with the cost of reading SysTick taken out.
struct Spread {
    min: u32,
    max: u32,
    sum: u32,
    n: u32,
    overhead: u32,
}

impl Spread {
    fn new(overhead: u32) -> Self {
        Self {
            min: u32::MAX,
            max: 0,
            sum: 0,
            n: 0,
            overhead,
        }
    }

    fn add(&mut self, cycles: u32) {
        let cycles = cycles.saturating_sub(self.overhead);

        self.min = self.min.min(cycles);
        self.max = self.max.max(cycles);
        self.sum += cycles;
        self.n += 1;
    }

    fn report(&self, name: &str, mclk: u32) {
        let ns = |cycles: u32| (cycles as u64 * 1_000_000_000 / mclk as u64) as u32;

        info!(
            "{}: min {} cycles ({} ns), mean {}, max {}",
            name,
            self.min,
            ns(self.min),
            self.sum / self.n,
            self.max,
        );
    }
}

#[embassy_executor::task]
async fn injector() {
    loop {
        ARMED.wait().await;

        // Stamped before the write, so the window opens on the last instruction before the interrupt.
        STAMP.store(SYST::get_current(), Ordering::Relaxed);
        inject();
    }
}

// Every port has to be bound, because which one a pin belongs to is not known until run time.
bind_group_interrupts!(struct Irqs {
    GPIOA => gpio::InterruptHandler;
    GPIOB => gpio::InterruptHandler;
});

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());
    let mclk = embassy_mspm0::sysctl::clocks().mclk;

    // SAFETY: nothing else in this build uses SysTick — the time driver is a timer peripheral.
    let mut syst = unsafe { cortex_m::Peripherals::steal() }.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(0x00ff_ffff);
    syst.clear_current();
    syst.enable_counter();

    let mut input = Input::new_async(p.PB7, Pull::Up, Irqs);

    if input.is_low() {
        error!("PB7 is being driven low; nothing should be connected to it. Halting.");
        loop {}
    }

    // Two reads back to back, so every figure below is the path and not the instrument.
    let overhead = {
        let mut least = u32::MAX;

        for _ in 0..WAKES {
            let a = SYST::get_current();
            let b = SYST::get_current();

            least = least.min(elapsed(a, b));
        }

        least
    };

    info!(
        "MCLK {} Hz, reading SysTick costs {} cycles, taken out below",
        mclk, overhead
    );

    let mut entry = Spread::new(overhead);
    let mut poll = Spread::new(overhead);

    for _ in 0..WAKES {
        let mut wait = pin!(input.wait_for_rising_edge());

        // One poll to arm the pin, which must not complete: there has been no edge yet.
        if poll_once(wait.as_mut()).await.is_ready() {
            error!("the wait completed before the interrupt was raised");
            continue;
        }

        let armed = SYST::get_current();
        inject();
        let handled = SYST::get_current();

        // The task was woken while it was running, so this polls the future where it stands.
        wait.await;
        let polled = SYST::get_current();

        entry.add(elapsed(armed, handled));
        poll.add(elapsed(handled, polled));
    }

    entry.report("entry", mclk);
    poll.report("poll ", mclk);

    spawner.spawn(injector().unwrap());

    let mut dispatch = Spread::new(overhead);

    for _ in 0..WAKES {
        // The injector cannot run until this task suspends, so signalling before arming is in order.
        ARMED.signal(());
        input.wait_for_rising_edge().await;

        dispatch.add(elapsed(STAMP.load(Ordering::Relaxed), SYST::get_current()));
    }

    dispatch.report("dispatch", mclk);

    let executor = dispatch.min.saturating_sub(entry.min + poll.min);
    info!("of which the executor hand-off is {} cycles", executor);

    two_waiters(input, Input::new_async(p.PB2, Pull::Up, Irqs)).await;

    loop {
        embassy_time::Timer::after_secs(60).await;
    }
}

/// Wake one of two pins that are being awaited at once, and check that only that one wakes.
///
/// Everything above waits on one pin at a time, which is the case a per-pin array answers without
/// looking. This is the case it does not: with a list, a second waiter is something both the interrupt's
/// search and the drop's removal have to get past, and neither is reached at all by a single waiter.
///
/// `second` is armed after `first` but dropped after it, so `first` is behind it in the list either way.
/// That ordering is the point — declaring it first and arming it second is what puts it there.
async fn two_waiters(mut first: Input<'static, Async>, mut second: Input<'static, Async>) {
    let mut behind = pin!(second.wait_for_any_edge());

    {
        let mut wait = pin!(first.wait_for_rising_edge());

        if poll_once(wait.as_mut()).await.is_ready() || poll_once(behind.as_mut()).await.is_ready() {
            error!("two waiters: a wait completed before anything was injected");
            return;
        }

        inject();

        if poll_once(wait.as_mut()).await.is_pending() {
            error!("two waiters: the injected pin did not wake — the interrupt did not find its waiter");
            return;
        }

        if poll_once(behind.as_mut()).await.is_ready() {
            error!("two waiters: the other pin woke as well");
            return;
        }
    }

    // `wait` has been dropped from behind `behind`, so if removal cannot get past a node that is not the
    // one being removed, the list is corrupt now and this last wait never completes.
    let mut proof = pin!(first.wait_for_rising_edge());

    if poll_once(proof.as_mut()).await.is_ready() {
        error!("two waiters: the re-armed wait completed before anything was injected");
        return;
    }

    inject();
    proof.await;

    info!("two waiters: ok");
}

/// Poll `future` exactly once, whatever it answers.
async fn poll_once(mut future: Pin<&mut impl Future<Output = ()>>) -> Poll<()> {
    poll_fn(|cx| Poll::Ready(future.as_mut().poll(cx))).await
}
