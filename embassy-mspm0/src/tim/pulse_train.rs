//! Pulse-train output.
//!
//! Emits a sequence of pulses of differing widths back to back on one channel, then leaves the
//! output at a chosen level. Each element sets both the period and the high time, so a train is not
//! a PWM whose duty is being changed — every pulse can differ from the last in both.
//!
//! The point of the driver is that no edge depends on software arriving on time. Each element is
//! written into the instance's shadow registers a whole period before it takes effect, and the
//! hardware transfers it at the boundary. The handler is what refills them, so the deadline is one
//! period rather than the gap an executor wake would leave.
//!
//! Only instances with both shadow registers can do this, which the bound on
//! [`PulseTrain::new`] enforces — [`ShadowLoadInstance`] and [`ShadowCompareInstance`] come from the
//! device metadata, and they are not the same set.

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::Ordering;
use core::task::{Context, Poll};

use portable_atomic::{AtomicBool, AtomicPtr, AtomicU8, AtomicUsize};

use crate::gpio::{AnyPin, Level, PfType, Pull, SealedPin};
use crate::interrupt::typelevel::Interrupt as _;
use crate::pac::tim::vals::{Act, Swfrcact};
use crate::tim::low_level::{self, Config as TimerConfig, Event, Timer};
pub use crate::tim::simple_pwm::Polarity;
use crate::tim::{
    Channel, ClockSel, CompareUpdate, CountingMode, Instance, ShadowCompareInstance, ShadowLoadInstance, TimerChannel,
    TimerPin, Word, simple_pwm,
};
use crate::{Peri, interrupt};

/// One element of a train: a high time and the low time that follows it.
///
/// Both are in counter ticks and both have to be at least one. A zero would be a duty of 0% or 100%,
/// which no compare value expresses — those are a forced-output override, and an override is not
/// shadowed, so applying one mid-train would reintroduce exactly the race this driver removes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pulse {
    /// Ticks the output is at the active level.
    pub high: u32,

    /// Ticks it is at the idle level afterwards.
    pub low: u32,
}

impl Pulse {
    /// Ticks the whole element occupies.
    ///
    /// Saturates. A wrapped sum is a small number, and a small number is exactly what the bound in
    /// [`emit`](PulseTrain::emit) is looking for, so wrapping would turn an unrepresentable element
    /// into an accepted one of the wrong width.
    pub const fn period(&self) -> u32 {
        self.high.saturating_add(self.low)
    }
}

/// What the handler needs to keep between elements.
///
/// Written only with the instance's interrupt masked, or from the handler itself, so plain loads and
/// stores are enough and none of it is a read-modify-write.
pub struct TrainState {
    /// The elements being emitted, borrowed by the [`ActiveTrain`] running them.
    pulses: AtomicPtr<Pulse>,

    /// How many of them there are.
    len: AtomicUsize,

    /// Zero events seen, which is how many elements have finished.
    ///
    /// Also names the next element to write into the shadow registers: the enable-time zero event
    /// is the first one seen and writes element one, so the count and the index move together.
    seen: AtomicUsize,

    /// The channel driving the pin.
    channel: AtomicU8,

    /// Set once the last element has finished.
    done: AtomicBool,

    /// Whether the output rests high between trains.
    idle_high: AtomicBool,

    /// Whether the train stops at the final element's compare match rather than its boundary.
    at_final_compare: AtomicBool,
}

impl TrainState {
    /// The compare action that drives the pin to the configured idle level.
    fn idle_act(&self) -> Act {
        act(self.idle_level())
    }

    /// The configured idle level, as the handler sees it.
    fn idle_level(&self) -> Level {
        if self.idle_high.load(Ordering::Relaxed) {
            Level::High
        } else {
            Level::Low
        }
    }

    /// The override that holds the pin at the configured idle level.
    fn idle_force(&self) -> Swfrcact {
        force(self.idle_level())
    }
}

impl TrainState {
    pub(crate) const fn new() -> Self {
        Self {
            pulses: AtomicPtr::new(core::ptr::null_mut()),
            len: AtomicUsize::new(0),
            seen: AtomicUsize::new(0),
            channel: AtomicU8::new(0),
            done: AtomicBool::new(false),
            idle_high: AtomicBool::new(false),
            at_final_compare: AtomicBool::new(false),
        }
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: core::marker::PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::info().regs;

        let state = T::train_state();
        let channel = Channel::ALL[state.channel.load(Ordering::Relaxed) as usize];
        let active = low_level::active(r);

        // The compare arm exists only for `End::AtFinalCompare`, and only for the last element: the
        // event is armed one boundary ahead and masked again here. Taken first because both events
        // can be latched at once on a one-element train, and this is the one that ends it.
        if active.contains(Event::CaptureOrCompareUp(channel)) {
            low_level::clear_pending(r, Event::CaptureOrCompareUp(channel));
            low_level::enable_interrupt(r, Event::CaptureOrCompareUp(channel), false);
            low_level::enable_interrupt(r, Event::Zero, false);

            // The compare has already driven the pin to the resting level in hardware, because the
            // final element's compare action was rewritten to it. Stopping here is what drops the
            // trailing phase; the override then holds the pin once the counter is dead.
            r.counterregs(0).ctrctl().modify(|w| w.set_en(false));

            r.counterregs(0)
                .ccact(channel.index())
                .modify(|w| w.set_swfrcact(state.idle_force()));

            state.done.store(true, Ordering::Release);
            T::cc_wakers()[channel.index()].wake();

            return;
        }

        // Other events the caller enabled through `Timer` are not ours to acknowledge.
        if !active.contains(Event::Zero) {
            return;
        }

        low_level::clear_pending(r, Event::Zero);

        let len = state.len.load(Ordering::Relaxed);
        let seen = state.seen.load(Ordering::Relaxed) + 1;

        state.seen.store(seen, Ordering::Relaxed);

        // The counter is zeroed when it is enabled, and that is a zero event like any other: it
        // transfers the shadow and raises this interrupt before the first element has run. So the
        // first one seen ends nothing, and every one after it ends an element.
        let finished = seen - 1;

        // One zero event per element, so this is the end of the last one. The shadow still holds the
        // element that just went live and would repeat it at the next boundary, which is why the
        // counter stops here rather than being left to run dry.
        if finished >= len {
            low_level::enable_interrupt(r, Event::Zero, false);

            // Park the pin here rather than leaving it for the task. The zero event that ends the
            // last element has already driven the output to its active level, so anything that waits
            // for an executor to run leaves that as a stub pulse on the wire.
            r.counterregs(0)
                .ccact(channel.index())
                .modify(|w| w.set_swfrcact(state.idle_force()));

            r.counterregs(0).ctrctl().modify(|w| w.set_en(false));

            state.done.store(true, Ordering::Release);
            T::cc_wakers()[channel.index()].wake();

            return;
        }

        // The zero event that ends the last element drives the pin to its active level in hardware,
        // and no software can beat it there — parking the pin from here still leaves a stub the
        // length of this handler's own latency. So the action register, which is shadowed like the
        // compare register, is given a closing action instead. It transfers the cycle after this
        // event, which puts it in force at exactly the boundary that ends the train.
        if finished == len - 1 {
            if state.at_final_compare.load(Ordering::Relaxed) {
                // This zero started the last element, so its compare has not fired yet and the write
                // has to reach the live register rather than the shadow — there is no later boundary
                // to transfer one. `CCACTUPD` goes back to immediate for exactly this write.
                r.counterregs(0)
                    .ccctl(channel.index())
                    .modify(|w| w.set_ccactupd(CompareUpdate::Immediately.into()));

                r.counterregs(0)
                    .ccact(channel.index())
                    .modify(|w| w.set_cuact(state.idle_act()));

                low_level::clear_pending(r, Event::CaptureOrCompareUp(channel));
                low_level::enable_interrupt(r, Event::CaptureOrCompareUp(channel), true);
            } else {
                // Only the zero action. `CUACT` has to keep driving the pin low, because the final
                // element still has its own falling edge to make — and this write has been measured
                // taking effect *during* that element rather than at the boundary after it, so a
                // closing `CUACT` eats that edge and the last pulse merges into the resting level.
                //
                // Invisible whenever the resting level is low, since the closing action is then the
                // value `CUACT` already held. It is what C39 could not see and C40 does.
                r.counterregs(0)
                    .ccact(channel.index())
                    .modify(|w| w.set_zact(state.idle_act()));
            }
        }

        if seen < len {
            // SAFETY: the running train borrows the slice, and every path that ends that borrow
            // masks this event first — the completion arm above, and `halt` under
            // `ActiveTrain`'s `Drop`. `seen < len` was checked above.
            let pulse = unsafe { &*state.pulses.load(Ordering::Relaxed).add(seen) };

            write_element(r, channel, pulse);
        }
    }
}

/// The compare action that drives a pin to `level`.
const fn act(level: Level) -> Act {
    match level {
        Level::Low => Act::CcpLow,
        Level::High => Act::CcpHigh,
    }
}

/// The forced-output override that holds a pin at `level`.
const fn force(level: Level) -> Swfrcact {
    match level {
        Level::Low => Swfrcact::CcpLow,
        Level::High => Swfrcact::CcpHigh,
    }
}

/// Put one element into the load and compare registers.
///
/// Whether that reaches the hardware now or at the next zero event is the shadow configuration's
/// business, not this function's — which is why the same call serves the first element, written
/// before the counter starts, and every later one.
fn write_element(r: crate::pac::tim::Tim, channel: Channel, pulse: &Pulse) {
    // The counter spans 0..=LOAD, so a period of `n` ticks is a load value of `n - 1`.
    r.counterregs(0).load().write_value(pulse.period() - 1);
    r.counterregs(0).cc(channel.index()).write_value(pulse.high);
}

/// Pulse-train configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Clock source driving the counter, which sets what one tick is worth.
    pub clock: ClockSel,

    /// Divider applied to the clock source, 1 to 8.
    pub divider: u8,

    /// Further divider applied after [`Self::divider`], 1 to 256.
    pub prescaler: u16,

    /// Keep counting while the debugger holds the core halted.
    pub free_run_in_debug: bool,

    /// What the pin does between trains.
    pub idle: Idle,

    /// Where the train stops.
    pub end: End,

    /// Which level a pulse's `high` time drives the pin to.
    ///
    /// [`Polarity::ActiveLow`] inverts the whole waveform, so a train begins with a low period and
    /// ends with a high one. That is how a sequence starting on a falling edge is expressed — there
    /// is no per-element polarity, and there does not need to be.
    pub polarity: Polarity,
}

/// Where a train stops.
///
/// Every element is one counter period, so a train of `n` elements emits `n` active phases and `n`
/// trailing ones. This is what makes an odd sequence reachable.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum End {
    /// Run every element in full. The train ends at the last one's period boundary.
    #[default]
    Complete,

    /// Stop at the last element's compare match, so its trailing time is never emitted.
    ///
    /// A train of `n` elements then has `n` active phases and `n - 1` trailing ones — the odd
    /// sequence a whole number of periods cannot express. Combined with
    /// [`Polarity::ActiveLow`](Polarity) it drops a trailing *high* instead.
    ///
    /// The counter stops the moment the compare fires rather than running out the rest of the
    /// period, so the train is shorter as well as shaped differently.
    AtFinalCompare,
}

/// What a [`PulseTrain`] does with its pin between trains.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Idle {
    /// Drive it low.
    #[default]
    Low,

    /// Drive it high.
    High,

    /// Stop driving it, so something else on the line can.
    ///
    /// The pin is disconnected between trains and muxed back to the timer when one starts. **It is
    /// still driven briefly at the end of a train**: the closing boundary acts in hardware, and the
    /// release cannot happen until the handler runs, so the resting level appears on the pin for the
    /// handler's own latency first.
    HighImpedance,
}

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            clock: ClockSel::DEFAULT,
            divider: 1,
            prescaler: 1,
            free_run_in_debug: false,
            idle: Idle::Low,
            end: End::Complete,
            polarity: Polarity::ActiveHigh,
        }
    }

    /// Set [`clock`](Self::clock).
    #[must_use]
    pub const fn with_clock(mut self, clock: ClockSel) -> Self {
        self.clock = clock;
        self
    }

    /// Set [`divider`](Self::divider).
    #[must_use]
    pub const fn with_divider(mut self, divider: u8) -> Self {
        self.divider = divider;
        self
    }

    /// Set [`prescaler`](Self::prescaler).
    #[must_use]
    pub const fn with_prescaler(mut self, prescaler: u16) -> Self {
        self.prescaler = prescaler;
        self
    }

    /// Set [`free_run_in_debug`](Self::free_run_in_debug).
    #[must_use]
    pub const fn with_free_run_in_debug(mut self, free_run_in_debug: bool) -> Self {
        self.free_run_in_debug = free_run_in_debug;
        self
    }

    /// Set [`idle`](Self::idle).
    #[must_use]
    pub const fn with_idle(mut self, idle: Idle) -> Self {
        self.idle = idle;
        self
    }

    /// Set [`end`](Self::end).
    #[must_use]
    pub const fn with_end(mut self, end: End) -> Self {
        self.end = end;
        self
    }

    /// Set [`polarity`](Self::polarity).
    #[must_use]
    pub const fn with_polarity(mut self, polarity: Polarity) -> Self {
        self.polarity = polarity;
        self
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Pulse-train driver.
///
/// Holds the whole instance: the counter's period is the element being emitted, so no other channel
/// of it can be doing anything else.
///
/// # Widths on MFCLK, with `low-power`
///
/// A train is only as accurate as the clock counting it, and on [`ClockSel::MfClk`] that clock
/// changes underneath a sleeping part. [`SleepLevel::Stop1`](crate::sysctl::SleepLevel::Stop1)
/// limits SYSOSC to 4 MHz, and MFCLK is taken straight from SYSOSC at that point rather than through
/// the divider that otherwise holds it at 4 — so the counter ticks on SYSOSC's 4 MHz trim, which the
/// datasheets band separately from its base frequency and considerably wider.
///
/// **A train on MFCLK reaches STOP1 and a train on the bus clock does not**, because a PD0 timer at
/// 4 MHz or less only has to block STOP2 to keep running, where one at 32 MHz blocks every STOP
/// level. Emitting from a sleep is the point of that, and the width error is its price: measured at
/// 0.73% short on one part, inside the trim's published band and therefore not a fault to be fixed
/// in silicon or here.
///
/// Callers who need the width rather than the sleep can hold a
/// [`WakeGuard`](crate::sysctl::WakeGuard) at `Stop1` for as long as the train runs, which restores
/// the widths to the accuracy the bus clock gives. Without `low-power` nothing sleeps and none of
/// this applies.
pub struct PulseTrain<'d, T: Instance> {
    timer: Timer<'d, T>,
    pin: Peri<'d, AnyPin>,
    channel: Channel,
    /// The resting level as the **signal generator** sees it, which is the caller's asked-for pin
    /// level already complemented for [`Polarity::ActiveLow`].
    ///
    /// `CCPIV`, the compare actions and the forced output all sit upstream of `CCPOINV` — the TRM
    /// calls `CCPIV` "the logical value put on the signal generator state" — so every one of them is
    /// written pre-inversion and the complement has to happen once, here.
    idle: Level,
    /// Whether the pin is disconnected between trains rather than driven.
    release: bool,
    /// Whether a train stops at its final element's compare match.
    at_final_compare: bool,
    /// What to mux the pin back to when a train starts, for [`Idle::HighImpedance`].
    pf: u8,
    pull: Pull,
    /// `divider * prescaler`, which is what one tick costs in source clocks. Only the cancel repair's
    /// spin reads it, scaled by 8192. [`Timer::new`] has already asserted the divider at 8 or less
    /// and the prescaler at 256 or less, so the scaled product tops out at 2^24 — the spin's bound
    /// leans on those assertions, not on the field types, and relaxing either moves it. A checked
    /// multiply would be a widening one, which links `__aeabi_lmul`.
    tick_divisor: u32,
}

impl<'d, T: ShadowLoadInstance + ShadowCompareInstance> PulseTrain<'d, T> {
    /// Claim `pin` and its instance to emit trains on.
    ///
    /// The elements themselves are handed to [`emit`](Self::emit), which is where the length of a
    /// train is decided.
    pub fn new<C: TimerChannel>(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, C>>,
        pull: Pull,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        // The pin is **not** muxed yet. Connecting it here would put it on a timer output that has
        // not been powered up, let alone told what level to rest at, and the pin then shows whatever
        // the reset state produces until the configuration below catches up — measured at 17 us on
        // this part. That is once per driver, which is once per frame for a caller whose pin is
        // shared and whose driver lives for one transaction, and it lands in the window where the
        // far end is looking for the start of a response.
        let pf = pin.pf_num();

        let mut timer = Timer::new(
            timer,
            TimerConfig {
                clock: config.clock,
                divider: config.divider,
                prescaler: config.prescaler,
                counting_mode: CountingMode::EdgeAlignedUp,
                free_run_in_debug: config.free_run_in_debug,
                ..Default::default()
            },
        );

        timer.setup_pwm_channel(C::CHANNEL, CountingMode::EdgeAlignedUp);
        timer.set_polarity(C::CHANNEL, config.polarity);

        // Inverting the whole waveform is what makes a train that starts low and ends high, so there
        // is no per-element polarity.
        //
        // **`CCPIV` is not routed through `CCPOINV` and the compare actions are.** The TRM calls
        // `CCPIV` the value put on the signal generator state, which reads as though the inverter is
        // downstream of it; on silicon the resting level comes out as written while the train comes
        // out inverted. So the level the caller asked for goes to `CCPIV` unchanged, and only the
        // actions and the forced output are complemented.
        let invert = matches!(config.polarity, Polarity::ActiveLow);
        let pin_idle = match config.idle {
            // Wherever the last pulse left the pin, so releasing adds no edge of its own.
            Idle::HighImpedance | Idle::Low => Level::Low,
            Idle::High => Level::High,
        };
        let idle = match (pin_idle, invert) {
            (Level::Low, true) => Level::High,
            (Level::High, true) => Level::Low,
            (level, false) => level,
        };

        // `CCPIV` is where the pin sits when the signal generator is not driving it, which is every
        // moment the counter is stopped. The forced-output override cannot do this job: it is an
        // action the counter evaluates, so it holds nothing once the counter halts.
        timer.set_idle_level(C::CHANNEL, pin_idle);

        let mut this = Self {
            timer,
            pin: pin.into(),
            channel: C::CHANNEL,
            idle,
            release: matches!(config.idle, Idle::HighImpedance),
            at_final_compare: matches!(config.end, End::AtFinalCompare),
            pf,
            pull,
            tick_divisor: config.divider as u32 * config.prescaler as u32,
        };

        // Before the mux, so the microseconds it takes are spent on a pin nothing is connected to.
        // Skipping it costs the first train a notch instead.
        this.park_latch();
        this.park();

        // Now. The output is already sitting at the resting level, so the mux is a no-op on the wire
        // rather than an edge. `park` has already disconnected it where the caller asked for that.
        if !this.release {
            this.pin.set_as_pf(pf, PfType::output(pull, false));
        }

        unsafe { T::Interrupt::enable() };

        this
    }

    /// Start emitting `pulses`, and hand back the train that is running them.
    ///
    /// The counter is going by the time this returns, so a caller can leave and await the returned
    /// [`ActiveTrain`] later. Awaiting it resolves when the last element has finished. Dropping it,
    /// or calling [`stop`](ActiveTrain::stop), halts the train and parks the output where an
    /// untouched one would be rather than part way through an element.
    ///
    /// The handler reads `pulses` as the train runs, which is why it is borrowed until the returned
    /// value goes away rather than for the length of this call.
    ///
    /// Every element needs a non-zero high and low time and a period the counter can reach. Both are
    /// checked here rather than emitting a waveform that is quietly not the one asked for.
    ///
    /// # Safety
    ///
    /// The interrupt handler reads `pulses` through a raw pointer for as long as the train runs, and
    /// **that outlives the borrow if the returned [`ActiveTrain`] is never dropped**. Awaiting it,
    /// calling [`stop`](ActiveTrain::stop) or dropping it all end the train; leaking it with
    /// [`mem::forget`](core::mem::forget()) leaves the handler reading memory the borrow checker
    /// considers free again.
    ///
    /// A destructor is not something the language promises to run, so this cannot be checked here.
    /// [`emit_static`](Self::emit_static) is the safe form: a buffer that outlives the program cannot
    /// be reclaimed underneath the handler, so leaking the train is a leak rather than a use after
    /// free.
    pub unsafe fn emit<'a>(&'a mut self, pulses: &'a [Pulse]) -> ActiveTrain<'a, 'd, T> {
        assert!(!pulses.is_empty(), "a train needs at least one pulse");

        // The load value, not the period: a 32-bit counter would make `MAX + 1` overflow, and the
        // bound only holds because a period of at least two is asserted on the line above.
        let max_load = <T::Word as Word>::MAX.into();

        for pulse in pulses {
            assert!(pulse.high > 0 && pulse.low > 0, "a pulse needs a high and a low time");
            // Not `period()`: on a 32-bit counter the saturated value is itself a legal load, so
            // the sum has to be rejected for overflowing rather than clamped into range.
            assert!(
                pulse
                    .high
                    .checked_add(pulse.low)
                    .is_some_and(|period| period - 1 <= max_load),
                "a pulse is longer than the counter can count"
            );
        }

        self.arm(pulses);

        ActiveTrain { train: self }
    }

    /// Start emitting a train the handler may read for as long as it likes.
    ///
    /// The safe form of [`emit`](Self::emit), and identical in every other way. A `'static` buffer is
    /// never reclaimed, so leaking the returned [`ActiveTrain`] leaks the timer with it rather than
    /// leaving the handler reading freed memory -- a leak the borrow checker already permits, not a
    /// use after free.
    ///
    /// A waveform table is usually a `static`, which makes this the ordinary way in and
    /// [`emit`](Self::emit) the one for a buffer built at run time.
    pub fn emit_static<'a>(&'a mut self, pulses: &'static [Pulse]) -> ActiveTrain<'a, 'd, T> {
        // SAFETY: `pulses` outlives the program, so nothing the handler reads can be reclaimed while
        // it is still reading, however the returned train is disposed of.
        unsafe { self.emit(pulses) }
    }

    /// The underlying counter, for reading.
    ///
    /// Mutating it needs [`timer_mut`](Self::timer_mut): this driver has programmed the instance for
    /// what it does, and a shared borrow is not the place to reprogram it from.
    pub fn timer(&self) -> &Timer<'d, T> {
        &self.timer
    }

    /// The underlying counter, for changing something this driver does not wrap.
    ///
    /// Whatever is changed here outlives the call. Reprogramming the counter, the compare values or
    /// the interrupt sources under a running driver is the caller's to get right.
    pub fn timer_mut(&mut self) -> &mut Timer<'d, T> {
        &mut self.timer
    }

    /// Stop emitting and give the timer and the pin back, with the pin disconnected.
    pub fn release(self) -> (Peri<'d, T>, Peri<'d, AnyPin>) {
        let mut this = core::mem::ManuallyDrop::new(self);

        this.halt();

        // SAFETY: `this` is never dropped and neither field is touched again, so each moves out once.
        let timer = unsafe { core::ptr::read(&this.timer) };
        let pin = unsafe { core::ptr::read(&this.pin) };

        pin.set_as_disconnected();

        (timer.release(), pin)
    }

    /// Program the first element, queue the second, and start the counter.
    ///
    /// The order matters and is not interchangeable. The first element is written **before**
    /// shadowing is turned on, so it reaches the registers rather than the shadows and is live the
    /// moment the counter starts. Shadowing then goes on before the second element is written, which
    /// is what the TRM asks for: a value written first and shadowed afterwards leaves the shadow
    /// holding its reset value, to be transferred at the next event.
    fn arm(&mut self, pulses: &[Pulse]) {
        let state = T::train_state();
        let r = self.timer.regs();
        let channel = self.channel;

        // Back on the timer before anything drives it. A released pin is a GPIO input until here.
        if self.release {
            self.pin.set_as_pf(self.pf, PfType::output(self.pull, false));
        }

        low_level::enable_interrupt(r, Event::Zero, false);
        low_level::enable_interrupt(r, Event::CaptureOrCompareUp(channel), false);

        // Shadowing off first. It is left on by the previous train, so without this the write below
        // lands in the shadow instead of the registers and the first element arrives a boundary late
        // — on every train but the first, which is the one that happens to start with it off.
        self.timer.set_shadow_load(false);
        self.timer.set_compare_update(channel, CompareUpdate::Immediately);

        write_element(r, channel, &pulses[0]);

        // The driving action, and the override released, into the register itself — the update is
        // still immediate here. Both have to be live before the counter starts: a write made after
        // the update setting below goes to the shadow, and then element zero runs under whatever
        // action the last train left behind, with the pin still held.
        r.counterregs(0).ccact(channel.index()).write(|w| {
            w.set_zact(Act::CcpHigh);
            w.set_cuact(Act::CcpLow);
            w.set_swfrcact(Swfrcact::Disabled);
        });

        self.timer.set_compare_update(channel, CompareUpdate::AtZero);
        self.timer.set_action_update(channel, CompareUpdate::AtZero);
        self.timer.set_shadow_load(true);

        // The same action again, into the shadow this time, so the boundary that starts the counter
        // transfers what is already live rather than the shadow's reset value.
        r.counterregs(0).ccact(channel.index()).write(|w| {
            w.set_zact(Act::CcpHigh);
            w.set_cuact(Act::CcpLow);
            w.set_swfrcact(Swfrcact::Disabled);
        });

        // The same element again, into the shadow this time. Starting the counter transfers it
        // straight back over the live copy, so what would otherwise be the shadow's reset value
        // landing on the first period is a write of the values already there.
        write_element(r, channel, &pulses[0]);

        // The second element is left for the handler, which the enable-time zero event calls before
        // the first element has finished. Writing it here instead would race that transfer.

        // Read through, never written — `AtomicPtr` is the only pointer atomic there is.
        state.pulses.store(pulses.as_ptr().cast_mut(), Ordering::Relaxed);
        state.len.store(pulses.len(), Ordering::Relaxed);
        state.seen.store(0, Ordering::Relaxed);
        state.channel.store(channel.index() as u8, Ordering::Relaxed);
        state
            .idle_high
            .store(matches!(self.idle, Level::High), Ordering::Relaxed);
        state.at_final_compare.store(self.at_final_compare, Ordering::Relaxed);
        state.done.store(false, Ordering::Release);

        low_level::clear_pending(r, Event::Zero);
        low_level::enable_interrupt(r, Event::Zero, true);

        self.timer.set_output_enabled(channel, true);
        self.timer.start();
    }

    /// Stop the counter, mask the event and park the output, whatever state a train was in.
    fn halt(&mut self) {
        let r = self.timer.regs();
        let channel = self.channel;

        low_level::enable_interrupt(r, Event::Zero, false);
        low_level::enable_interrupt(r, Event::CaptureOrCompareUp(channel), false);

        // Let a shared line go before anything else happens to it. The repair below runs the
        // counter, and a pin still muxed to the timer turns that into a driven level on a line the
        // caller asked to only borrow. The latch the repair fixes is inside the timer, so the pin
        // has no part in it; `arm` is what muxes it back.
        if self.release {
            self.pin.set_as_disconnected();
        }

        self.timer.stop();

        // A completed train stops on a boundary, with the counter already at zero. A cancelled one
        // stops wherever the element had got to, and zeroing that at the next enable costs the first
        // element a couple of ticks — visible only on the train *after* a cancelled one, which is
        // not where anybody would look.
        self.timer.set_counter(<T::Word as Word>::from_reg(0));

        // The output generator latches the last level an action drove, and the latch is not a
        // register. The software force cannot reach it while the counter is stopped either: a
        // forced action is itself deferred to a period boundary (SLAU846E 34.2.5.3), and a stopped
        // counter never has one. A cancelled train leaves the cancelled element's active level in
        // the latch, and at the next enable the generator drives the pin with it from `EN` until
        // the enable-time zero event lands — a stretch of the first element's high time. A
        // completed train's closing boundary action parks the latch, so only a cancellation needs
        // the repair: run the counter for one throwaway zero event with every action set to idle
        // and the output held by `ODIS`, which is the one thing that does drive the latch.
        if T::train_state().done.load(Ordering::Acquire) {
            // The closing action is in the register, transferred by the boundary that ended the
            // train, and the update setting is still shadowed. Put it back to immediate so the idle
            // actions land in the register rather than in a shadow no boundary will ever transfer:
            // without this the *next* train's first element comes out long, and only that one,
            // which is a nasty thing to go looking for.
            self.timer.set_action_update(channel, CompareUpdate::Immediately);

            r.counterregs(0).ccact(channel.index()).modify(|w| {
                w.set_zact(self.idle_act());
                w.set_cuact(self.idle_act());
            });
        } else {
            self.park_latch();
        }

        T::train_state().len.store(0, Ordering::Relaxed);
        T::train_state().pulses.store(core::ptr::null_mut(), Ordering::Relaxed);

        self.park();
    }

    /// Drive the output generator's level latch to the idle level, with the pin held quiet.
    ///
    /// **The latch is not a register**, so nothing writes it and nothing reads it back. The only
    /// thing that moves it is a compare action the counter evaluates, and a stopped counter never
    /// reaches one — so parking it means running the counter for a single throwaway zero event
    /// with every action set to idle.
    ///
    /// Two callers need that and for the same reason. A cancelled train leaves the cancelled
    /// element's active level in the latch. A driver that has just been built has never run the
    /// counter at all, so the latch sits at its reset level, which is low. Either way the next
    /// enable drives the pin from `EN` until the enable-time zero event lands, and on the wire
    /// that is a notch against the resting level — 2 us, and invisible whenever the resting level
    /// is itself low, which is why it went unnoticed until a train was asked to rest high.
    fn park_latch(&mut self) {
        let r = self.timer.regs();
        let channel = self.channel;

        // Both copies of the action register have to say idle: the zero event evaluates the
        // live one and then transfers the shadow over it. Shadow first — the update mode has
        // to be set before the register it routes, or the write lands in the wrong copy.
        self.timer.set_action_update(channel, CompareUpdate::AtZero);

        r.counterregs(0).ccact(channel.index()).write(|w| {
            w.set_zact(self.idle_act());
            w.set_cuact(self.idle_act());
        });

        self.timer.set_action_update(channel, CompareUpdate::Immediately);

        r.counterregs(0).ccact(channel.index()).write(|w| {
            w.set_zact(self.idle_act());
            w.set_cuact(self.idle_act());
        });

        self.timer.set_output_enabled(channel, false);

        // `ODIS` does not let the pin go: it holds the signal low *before* the conditional
        // inversion (SLAU846E 34.3.32, and its L-series sibling), so on a generator-domain
        // high idle the repair window would drive the pin at the opposite of its resting
        // level. Inverting the output for the window turns that held low into the resting
        // level. Flipping `CCPOINV` while the counter is stopped moves nothing on the pin,
        // which is showing `CCPIV` — not routed through the inverter.
        let invert_for_repair = matches!(self.idle, Level::High);

        if invert_for_repair {
            self.flip_polarity();
        }

        low_level::clear_pending(r, Event::Zero);

        // A raw enable rather than `start`: the stop above released the running guard, and
        // this window is over before the executor could reach a sleep.
        r.counterregs(0).ctrctl().modify(|w| w.set_en(true));

        // The enable-time zero event is what evaluates the idle actions. Bounded, so a part
        // that never raises it cannot hang a `Drop`; the flag arrives within a few ticks — but a
        // tick is `source / divider / prescaler`, so a fixed iteration count expires early on a
        // slow tree and leaves the latch unrepaired. Scaling by the two dividers is a multiply
        // where deriving it from the tick rate would link a software divider.
        for _ in 0..8192 * self.tick_divisor {
            if low_level::is_pending(r, Event::Zero) {
                break;
            }
        }

        // A little margin for the action itself — each read is a volatile register access.
        for _ in 0..16 {
            let _ = low_level::is_pending(r, Event::Zero);
        }

        r.counterregs(0).ctrctl().modify(|w| w.set_en(false));

        low_level::clear_pending(r, Event::Zero);
        self.timer.set_counter(<T::Word as Word>::from_reg(0));

        if invert_for_repair {
            self.flip_polarity();
        }

        self.timer.set_output_enabled(channel, true);
    }

    /// The compare action that drives the pin to the configured idle level.
    fn idle_act(&self) -> Act {
        act(self.idle)
    }

    /// Swap the channel's output inversion, for the cancel repair's window.
    fn flip_polarity(&mut self) {
        let flipped = match self.timer.polarity(self.channel) {
            Polarity::ActiveHigh => Polarity::ActiveLow,
            Polarity::ActiveLow => Polarity::ActiveHigh,
        };

        self.timer.set_polarity(self.channel, flipped);
    }

    /// Leave the forced-output override at the idle level, and put the channel back on the pin.
    ///
    /// **The override is not what holds the pin between trains.** A forced action is evaluated at a
    /// period boundary and a stopped counter never reaches one, so this write never asserts. What
    /// holds a stopped channel is `OCTL.CCPIV`, set once in [`new`](Self::new).
    ///
    /// Writing it anyway is state hygiene: it stops a force left over from a cancelled train being
    /// evaluated by the first boundary of the next one, which is why `arm` clears it before starting.
    fn park(&mut self) {
        self.timer.set_forced_output(self.channel, Some(self.idle));
        self.timer.set_output_enabled(self.channel, true);

        // `ODIS` holds the output at a level; it does not stop driving it. Letting go of the line
        // means taking the pin off the timer altogether.
        if self.release {
            self.pin.set_as_disconnected();
        }
    }
}

/// A train that is running.
///
/// Awaiting it resolves when the last element has finished. Dropping it, including by losing a
/// `select!`, stops the train and parks the output, so the pin never rests part way through an
/// element. The elements stay borrowed for as long as this value lives, because the handler is
/// reading them.
///
/// **The destructor is what ends that read.** The handler reaches the elements through a raw pointer
/// held in a `static`, so leaking this with [`mem::forget`](core::mem::forget) ends the borrow without
/// stopping the train, and the handler goes on dereferencing storage the caller is then free to reuse.
/// That is the same bargain every DMA driver in this crate takes.
#[must_use = "dropping this stops the train at once; await it, or hold it while the train runs"]
pub struct ActiveTrain<'a, 'd, T: ShadowLoadInstance + ShadowCompareInstance> {
    train: &'a mut PulseTrain<'d, T>,
}

impl<T: ShadowLoadInstance + ShadowCompareInstance> ActiveTrain<'_, '_, T> {
    /// Whether the last element has finished.
    pub fn is_done(&self) -> bool {
        T::train_state().done.load(Ordering::Acquire)
    }

    /// Stop the train and park the output.
    ///
    /// What dropping this does, named so that a caller cancelling on purpose says so.
    pub fn stop(self) {}
}

impl<T: ShadowLoadInstance + ShadowCompareInstance> Future for ActiveTrain<'_, '_, T> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        T::cc_wakers()[self.train.channel.index()].register(cx.waker());

        if T::train_state().done.load(Ordering::Acquire) {
            return Poll::Ready(());
        }

        Poll::Pending
    }
}

impl<T: ShadowLoadInstance + ShadowCompareInstance> Drop for ActiveTrain<'_, '_, T> {
    fn drop(&mut self) {
        self.train.halt();
    }
}

impl<T: Instance> Drop for PulseTrain<'_, T> {
    fn drop(&mut self) {
        let r = self.timer.regs();

        low_level::enable_interrupt(r, Event::Zero, false);
        low_level::enable_interrupt(r, Event::CaptureOrCompareUp(self.channel), false);
        self.timer.stop();

        T::train_state().len.store(0, Ordering::Relaxed);
        T::train_state().pulses.store(core::ptr::null_mut(), Ordering::Relaxed);

        self.pin.set_as_disconnected();
    }
}

// Keeps the import honest: the channel setup this driver relies on is `simple_pwm`'s, so the two
// have to agree about which action fires at zero and which at the compare match.
const _: () = {
    #[allow(unused)]
    fn setup_is_shared(r: crate::pac::tim::Tim, channel: Channel) {
        simple_pwm::setup_channel(r, channel, CountingMode::EdgeAlignedUp);
    }
};
