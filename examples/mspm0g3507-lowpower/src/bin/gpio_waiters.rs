//! Checks the GPIO waiter list's three paths that a single wait never reaches, without wiring anything.
//!
//! `GPIO.CPU_INT.ISET` raises a pin's interrupt exactly as an edge would — SLAU846 9.3.14 offers it "to be
//! set by software (useful in diagnostics and safety checks)" — so every case here is driven from code and
//! is reproducible run to run. Nothing is timed: `wake_cycles` on the G3507 is the instrument, this is the
//! correctness half.
//!
//! # Pins
//!
//! `PB17` (J1.18) and `PA17` (J3.28) — **the same bit on each port**, which is what makes `across ports`
//! mean anything: the two waits differ only by port, so a list that is not per-port matches them to each
//! other. Plus `PB0` (J2.12) for a second waiter within one port.
//!
//! All three are free: SLAU873's jumper table puts no load on them, unlike `PA18` (S1 and its external
//! pulldown), `PA22` and `PA27` (light sensor), or `PB24` (thermistor). Rig C drives only `PB7` and `PB2`
//! on this board, so **this runs with rig C wired and needs no rewiring**.
//!
//! `PB17` is driven by this example through an open-drain output with its pull-up, so its level can be
//! moved; the other two are only ever inputs.
//!
//! This crate builds with `_probe`, but a marker does nothing until `probe::arm` is called and this binary
//! never calls it.
//!
//! # Cases
//!
//! | case | what it proves |
//! |---|---|
//! | `two waiters` | the interrupt's search and the drop's removal both get past a node that is not theirs |
//! | `wrong direction` | an edge the other way leaves the wait standing, and the wait still completes later |
//! | `dropped in flight` | a wait dropped with its interrupt already raised corrupts nothing |
//! | `across ports` | each port keeps its own list: one port's edges never reach the other's waiter |
//!
//! `across ports` is the case this board can run and the L1306 cannot, having one port. Three ports needs
//! an L2228, and there is not one on the bench.
//!
//! The last two are what `GPIO_ERR_01` and cancellation respectively make reachable. This device has the
//! erratum, so `DETECT_BOTH_EDGES` is on and the handler classifies an edge by the level the pin settled
//! at — which is what `wrong direction` exercises. On a part without it the polarity register does the
//! filtering and that case cannot happen; it would pass trivially.
//!
//! # What a pass looks like
//!
//! Four `ok` lines and nothing else. Every failure prints what it expected and stops that case — a hang
//! is also a failure, and means a wake that never arrived.

#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::pin::{Pin, pin};
use core::task::Poll;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::gpio::{Flex, Input, Pull};
use embassy_mspm0::mode::Async;
use embassy_mspm0::pac::gpio;
use embassy_mspm0::{bind_group_interrupts, pac};
use embassy_time::{Duration, with_timeout};
use panic_halt as _;

/// The pin this example drives, so that a wait can be given an edge of either direction. Port B.
///
/// Bit 17 on purpose: `across` is bit 17 of port A, so the two differ only by port. A list that was not
/// per-port, or one indexed by the wrong thing, would match them to each other.
const DRIVEN_BIT: usize = 17;

/// The pin that is only ever waited on, to put a second node in port B's list.
const OTHER_BIT: usize = 0;

/// The same bit on the *other* port, to check that a list belongs to its port and not to the driver.
const ACROSS_BIT: usize = 17;

/// Raise `bit`'s interrupt as an edge on the pin would, and do not return until it has been taken.
///
/// The barriers matter: without them the store is still in flight when the next instruction retires, and
/// the handler runs somewhere after the check that was meant to observe it.
fn inject(block: gpio::Gpio, bit: usize) {
    block.cpu_int().iset().write(|w| w.set_dio(bit, true));

    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Drive `bit` low, or release it to its pull-up, while a wait holds the `Flex` that configured it.
///
/// Through the PAC because the wait borrows the pin for as long as it is armed, and moving the level
/// under an armed wait is the whole point of the case that uses this. Open drain, so "high" is a release
/// and the pull-up needs a moment; the read back is what says it got there.
fn drive(bit: usize, high: bool) -> bool {
    if high {
        pac::GPIOB.doutset31_0().write(|w| w.set_dio(bit, true));
    } else {
        pac::GPIOB.doutclr31_0().write(|w| w.set_dio(bit, true));
    }

    // Long enough for a pull-up through whatever is clipped to the pin, and irrelevant to what is timed:
    // nothing here is.
    cortex_m::asm::delay(32_000);

    pac::GPIOB.din31_0().read().dio(bit) == high
}

/// Await `future`, reporting rather than hanging if the wake never arrives.
///
/// Every terminal await here is one the list is supposed to complete, so a hang is a failure — and a
/// failure that says which case it was is worth the timer.
async fn expect_wake(case: &str, future: impl Future<Output = ()>) -> bool {
    if with_timeout(Duration::from_millis(100), future).await.is_err() {
        error!("{}: the wake never arrived", case);
        return false;
    }

    true
}

/// Poll `future` exactly once, whatever it answers.
async fn poll_once(mut future: Pin<&mut impl Future<Output = ()>>) -> Poll<()> {
    poll_fn(|cx| Poll::Ready(future.as_mut().poll(cx))).await
}

/// Wake one of two pins that are being awaited at once, and check that only that one wakes.
///
/// `behind` is armed first and dropped last, so the other wait is always further down the list than it —
/// which is what makes the interrupt's search and the drop's removal walk past a node that is not theirs.
async fn two_waiters(driven: &mut Flex<'static, Async>, other: &mut Input<'static, Async>) {
    let mut behind = pin!(other.wait_for_any_edge());

    {
        let mut wait = pin!(driven.wait_for_rising_edge());

        if poll_once(wait.as_mut()).await.is_ready() || poll_once(behind.as_mut()).await.is_ready() {
            error!("two waiters: a wait completed before anything was injected");
            return;
        }

        inject(pac::GPIOB, DRIVEN_BIT);

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
    let mut proof = pin!(driven.wait_for_rising_edge());

    if poll_once(proof.as_mut()).await.is_ready() {
        error!("two waiters: the re-armed wait completed before anything was injected");
        return;
    }

    inject(pac::GPIOB, DRIVEN_BIT);

    if !expect_wake("two waiters", proof).await {
        return;
    }

    info!("two waiters: ok");
}

/// Give a wait an edge going the other way, and check it is left standing rather than woken or lost.
///
/// Under `GPIO_ERR_01` both directions are detected in hardware and the handler filters by the level the
/// pin settled at, so a falling wait on a pin sitting high is an interrupt the handler must decline. The
/// wait then has to still be armed: the second half drives the pin low and injects again, and that one
/// has to complete.
async fn wrong_direction(driven: &mut Flex<'static, Async>) {
    if !drive(DRIVEN_BIT, true) {
        error!("wrong direction: the pin did not come up with its pull-up; is something driving it?");
        return;
    }

    let mut wait = pin!(driven.wait_for_falling_edge());

    if poll_once(wait.as_mut()).await.is_ready() {
        error!("wrong direction: the wait completed before anything was injected");
        return;
    }

    // The pin is high, so this is a rising edge arriving at a wait for a falling one.
    inject(pac::GPIOB, DRIVEN_BIT);

    if poll_once(wait.as_mut()).await.is_ready() {
        error!("wrong direction: an edge the other way woke the wait");
        return;
    }

    // Now make it true: the level the handler reads is what classifies the edge.
    if !drive(DRIVEN_BIT, false) {
        error!("wrong direction: the pin would not go low");
        return;
    }

    inject(pac::GPIOB, DRIVEN_BIT);

    if poll_once(wait.as_mut()).await.is_pending() {
        error!("wrong direction: the wait was still armed but the edge it asked for did not wake it");
        return;
    }

    // Back to idle-high before anything else injects: a rising edge on a pin left low is one the handler
    // is right to refuse, and every later case asks for one.
    if !drive(DRIVEN_BIT, true) {
        error!("wrong direction: the pin would not return high");
        return;
    }

    info!("wrong direction: ok");
}

/// Drop a wait with its interrupt already raised, and check the list survives it.
///
/// The interrupt is raised inside a critical section and the wait is dropped before that section ends, so
/// the handler cannot run until the node is already unlinked — the ordering a cancellation races for and
/// almost never loses. `other` stays armed across it, so a list that lost its head takes that wait with it.
async fn dropped_in_flight(driven: &mut Flex<'static, Async>, other: &mut Input<'static, Async>) {
    let mut bystander = pin!(other.wait_for_any_edge());

    if poll_once(bystander.as_mut()).await.is_ready() {
        error!("dropped in flight: the bystander completed before anything was injected");
        return;
    }

    let restore = {
        let mut doomed = pin!(driven.wait_for_rising_edge());

        if poll_once(doomed.as_mut()).await.is_ready() {
            error!("dropped in flight: the wait completed before anything was injected");
            return;
        }

        // SAFETY: released below, on every path out of this block.
        let restore = unsafe { critical_section::acquire() };

        inject(pac::GPIOB, DRIVEN_BIT);

        restore
        // `doomed` is dropped here, still inside the section, with its interrupt raised.
    };

    // SAFETY: acquired immediately above, and nothing has released it since.
    unsafe { critical_section::release(restore) };

    if poll_once(bystander.as_mut()).await.is_ready() {
        error!("dropped in flight: the dropped wait's edge woke the other pin");
        return;
    }

    // A fresh wait on the pin whose node was pulled out from under the handler.
    let mut after = pin!(driven.wait_for_rising_edge());

    if poll_once(after.as_mut()).await.is_ready() {
        error!("dropped in flight: the re-armed wait completed before anything was injected");
        return;
    }

    inject(pac::GPIOB, DRIVEN_BIT);

    if !expect_wake("dropped in flight", after).await {
        return;
    }

    // And the bystander, which was in the list the whole time, still wakes.
    inject(pac::GPIOB, OTHER_BIT);

    if !expect_wake("dropped in flight (bystander)", bystander).await {
        return;
    }

    info!("dropped in flight: ok");
}

/// Check that a port's list is its own: an edge on one port must not reach a waiter on the other.
///
/// `WAITERS` is an array indexed by port and each port has its own handler, so a waiter on port A has to
/// sit out everything port B does — and then still wake when its own port is injected. With one list for
/// the whole chip, or an index computed from the wrong thing, the first half of this passes and the
/// second half wakes the wrong task.
async fn across_ports(driven: &mut Flex<'static, Async>, across: &mut Input<'static, Async>) {
    let mut far = pin!(across.wait_for_any_edge());

    if poll_once(far.as_mut()).await.is_ready() {
        error!("across ports: the port A wait completed before anything was injected");
        return;
    }

    {
        let mut near = pin!(driven.wait_for_rising_edge());

        if poll_once(near.as_mut()).await.is_ready() {
            error!("across ports: the port B wait completed before anything was injected");
            return;
        }

        inject(pac::GPIOB, DRIVEN_BIT);

        if poll_once(near.as_mut()).await.is_pending() {
            error!("across ports: the injected port B pin did not wake");
            return;
        }

        if poll_once(far.as_mut()).await.is_ready() {
            error!("across ports: a port B edge woke the port A waiter");
            return;
        }
    }

    inject(pac::GPIOA, ACROSS_BIT);

    if !expect_wake("across ports", far).await {
        return;
    }

    info!("across ports: ok");
}

// Every port has to be bound, because which one a pin belongs to is not known until run time.
bind_group_interrupts!(struct Irqs {
    GPIOA => embassy_mspm0::gpio::InterruptHandler;
    GPIOB => embassy_mspm0::gpio::InterruptHandler;
});

#[embassy_executor::main(executor = "embassy_mspm0::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    // Open drain with the pull-up, so this example can put the pin low and let it back up without
    // fighting anything that might be on the far end of an analyser lead.
    let mut driven = Flex::new_async(p.PB17, Irqs);
    driven.set_as_input_output();
    driven.set_pull(Pull::Up);
    driven.set_high();

    let mut other = Input::new_async(p.PB0, Pull::Up, Irqs);
    let mut across = Input::new_async(p.PA17, Pull::Up, Irqs);

    if driven.is_low() || other.is_low() || across.is_low() {
        error!("PB17/PB0/PA17 are not idling high; something is driving them. Halting.");
        loop {}
    }

    info!("gpio_waiters: PB17 driven, PB0 and PA17 inputs, all idle high");

    two_waiters(&mut driven, &mut other).await;
    wrong_direction(&mut driven).await;
    dropped_in_flight(&mut driven, &mut other).await;
    across_ports(&mut driven, &mut across).await;

    info!("gpio_waiters: done");

    loop {
        embassy_time::Timer::after_secs(60).await;
    }
}
