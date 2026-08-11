//! Edge-to-response wake latency on one board, measured on the analyser.
//!
//! Scratch. Not for committing.
//!
//! The executor idles, a falling edge on the stimulus pin wakes it, and the response pin toggles as
//! the first thing after the wait returns. The analyser sees both, and the interval between them is
//! the whole software path: interrupt entry, the GPIO handler, the waker, the executor hand-off and
//! the poll that completes.
//!
//! # Wiring
//!
//! | channel | pin | what it carries |
//! |---|---|---|
//! | D0 | `PA18` — S1, J2.26 | the stimulus. Idles low through its pulldown; a press drives it to 3V3 |
//! | D1 | `PA16` — J2.24 | the response. Toggles once per accepted edge |
//!
//! **S1 is active high and S2 is active low.** The board's own guide is what says so — J11 gives S1
//! an external pulldown and the switch connects it to 3V3, where S2 pulls to ground. Copying the
//! `button.rs` example's `Pull::Up` and falling edge onto this pin produces a wait that never
//! completes and, worse, no edge on the analyser either: the internal pull-up holds the pin at the
//! same level the press drives it to, so the capture is clean and empty and looks like a dead lead.
//!
//! Ground the analyser to the board. Nothing else is wired, and no second board is involved.
//!
//! **S1 is on `PA18` and `PA18` is broken out to J2.26**, which is what makes this measurable with
//! one board: the analyser watches the same node the port sees, rather than a copy of it.
//! `PA16` is a marker pin nothing else on this board drives.
//!
//! # Reading it
//!
//! Trigger on D0 falling. The first D1 edge after it is the response; everything after is switch
//! bounce re-arming the wait, which is why the response toggles rather than pulsing — a toggle per
//! edge keeps bounce visible instead of hiding it.
//!
//! **Take the first response after a quiet period.** A bounce burst produces several edges within a
//! few milliseconds and only the first is a wake from idle; the rest arrive with the core already
//! awake and measure something else.
//!
//! # What is deliberately not in this binary
//!
//! **The stock `embassy-executor` thread executor**, whose idle is `WFE`. Not the HAL's low-power
//! executor: that one picks a sleep mode, and both the mode choice and the errata work around it
//! carries would land inside the interval being measured. The point here is the plain path.
//!
//! **No `low-power` and no `unstable-pac`**, which is why this lives in the ordinary example crate
//! rather than the low-power one.
//!
//! **No `_probe`.** Marker instrumentation has twice distorted the thing it was measuring on this
//! branch, once by a third of this very segment.
//!
//! **No `defmt`.** RTT's buffer is a kilobyte of RAM, its encoder runs inside a critical section, and
//! a probe attached to stream it holds the device out of idle. Build with `DEFMT_LOG=off`, which the
//! crate's other binaries do not need but this one does — it links no logger at all.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mspm0::gpio::{self, Input, Level, Output, Pull};
use embassy_mspm0::{Config, bind_group_interrupts};
use panic_halt as _;

bind_group_interrupts!(struct Irqs {
    GPIOA => gpio::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Config::default());

    let mut response = Output::new(p.PA16, Level::Low);
    let mut stimulus = Input::new_async(p.PA18, Pull::Down, Irqs);

    loop {
        stimulus.wait_for_rising_edge().await;
        response.toggle();
    }
}
