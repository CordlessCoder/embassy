//! Driving the comparator from an interrupt handler the application owns.
//!
//! [`Comp::wait_for_edge`] is scheduled by the executor, so an edge waits for the running task to
//! yield. Where the response has to happen whatever else is runnable — a rail that must not sag, a
//! pulse that must not run long — the application takes the handler and the driver keeps only
//! configuration and the DAC.
//!
//! The pattern here alternates thresholds: the low code arms the rising edge, the high code arms the
//! falling edge, and each edge reprograms the other. That is the shape of a hysteretic regulator.
//!
//! On this chip the comparator has no NVIC line of its own. `COMP0` is one source on `GROUP1`, which
//! it shares with `GPIOA`, so the handler is bound to the *source* and the group's demultiplexer
//! calls it. Binding `GPIOA` alongside is what leaves the pin waits working.
//!
//! # Wiring
//!
//! | Signal | Pin |
//! |---|---|
//! | comparator input | `PA26`, driven between 0 V and `VDDA` |
//!
//! With nothing attached the input floats and the edge count is meaningless — the point of the
//! example is the structure, not the reading.

#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::comp::{Comp, Config as CompConfig, DacCode, Edge, Reference, ReferenceSource};
use embassy_mspm0::mode::Blocking;
use embassy_mspm0::peripherals::{COMP0, PA27};
use embassy_mspm0::{Peri, bind_group_interrupts, gpio, interrupt_group};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::Timer;
use panic_halt as _;

/// The code the rising edge is armed against, and the one the falling edge is armed against.
const LOW_CODE: u8 = 0x40;
const HIGH_CODE: u8 = 0xc0;

/// The driver, reachable from both the handler and the main task.
static COMP: CriticalSectionMutex<RefCell<Option<Comp<'static, COMP0, Blocking>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Edges the handler has serviced, so the main task can show it is running.
static EDGES: CriticalSectionMutex<RefCell<u32>> = CriticalSectionMutex::new(RefCell::new(0));

struct RailHandler;

impl interrupt_group::Handler<interrupt_group::COMP0> for RailHandler {
    unsafe fn on_interrupt() {
        COMP.lock(|comp| {
            let mut comp = comp.borrow_mut();
            let Some(comp) = comp.as_mut() else {
                return;
            };

            let Some(edge) = comp.pending_edge() else {
                return;
            };

            // Clear before arming the other edge. The flags are sticky, so one left over re-enters
            // this handler as soon as the arm below lands.
            comp.clear_pending();

            match edge {
                Edge::Rising | Edge::Any => {
                    comp.set_dac_code(DacCode::new(HIGH_CODE));
                    comp.enable_edge_interrupt(Edge::Falling);
                }
                Edge::Falling => {
                    comp.set_dac_code(DacCode::new(LOW_CODE));
                    comp.enable_edge_interrupt(Edge::Rising);
                }
            }
        });

        EDGES.lock(|edges| *edges.borrow_mut() += 1);
    }
}

bind_group_interrupts!(struct Irqs {
    COMP0 => RailHandler;
    GPIOA => gpio::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());

    let mut config = CompConfig::default();
    config.reference = Some(Reference {
        source: ReferenceSource::Vdda,
        ..Default::default()
    });

    // The negative terminal is left to the reference DAC. `None` needs naming because several pins
    // could have filled it.
    let negative: Option<Peri<'_, PA27>> = None;

    let mut comp = unwrap!(Comp::new(p.COMP0, Some(p.PA26), negative, config));

    comp.set_dac_code(DacCode::new(LOW_CODE));
    comp.clear_pending();
    comp.enable_edge_interrupt(Edge::Rising);

    COMP.lock(|slot| slot.replace(Some(comp)));

    info!("comparator armed; the handler owns the edges from here");

    loop {
        Timer::after_millis(1000).await;
        info!("edges serviced: {}", EDGES.lock(|edges| *edges.borrow()));
    }
}
