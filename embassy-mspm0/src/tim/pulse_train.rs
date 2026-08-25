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
    pub const fn period(&self) -> u32 {
        self.high + self.low
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

    /// The next element to write into the shadow registers.
    next: AtomicUsize,

    /// Zero events seen, which is how many elements have finished.
    seen: AtomicUsize,

    /// The channel driving the pin.
    channel: AtomicU8,

    /// Set once the last element has finished.
    done: AtomicBool,

    /// Whether the output rests high between trains.
    idle_high: AtomicBool,
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
            next: AtomicUsize::new(0),
            seen: AtomicUsize::new(0),
            channel: AtomicU8::new(0),
            done: AtomicBool::new(false),
            idle_high: AtomicBool::new(false),
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

        // Other events the caller enabled through `Timer` are not ours to acknowledge.
        if !low_level::active(r).contains(Event::Zero) {
            return;
        }

        low_level::clear_pending(r, Event::Zero);

        let state = T::train_state();
        let channel = Channel::ALL[state.channel.load(Ordering::Relaxed) as usize];
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
            r.counterregs(0).ccact(channel.index()).modify(|w| {
                w.set_zact(state.idle_act());
                w.set_cuact(state.idle_act());
            });
        }

        let next = state.next.load(Ordering::Relaxed);

        if next < len {
            // SAFETY: the running train borrows the slice, and every path that ends that borrow
            // masks this event first — the completion arm above, and `halt` under
            // `ActiveTrain`'s `Drop`. `next < len` was checked above.
            let pulse = unsafe { &*state.pulses.load(Ordering::Relaxed).add(next) };

            write_element(r, channel, pulse);
            state.next.store(next + 1, Ordering::Relaxed);
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

    /// Level the output rests at between trains, and the complement of a pulse's active level.
    pub idle: Level,
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
            idle: Level::Low,
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
    pub const fn with_idle(mut self, idle: Level) -> Self {
        self.idle = idle;
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
pub struct PulseTrain<'d, T: Instance> {
    timer: Timer<'d, T>,
    pin: Peri<'d, AnyPin>,
    channel: Channel,
    idle: Level,
    /// `divider * prescaler`, which is what one tick costs in source clocks. Only the cancel repair's
    /// spin reads it, and a plain multiply there cannot overflow: `divider` is asserted at 8 or less
    /// and a `u16` prescaler leaves the product inside `u32`. A checked multiply would be a widening
    /// one, which links `__aeabi_lmul`.
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
        pin.set_as_pf(pin.pf_num(), PfType::output(pull, false));

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

        // `CCPIV` is where the pin sits when the signal generator is not driving it, which is every
        // moment the counter is stopped. The forced-output override cannot do this job: it is an
        // action the counter evaluates, so it holds nothing once the counter halts.
        timer.set_idle_level(C::CHANNEL, config.idle);

        let mut this = Self {
            timer,
            pin: pin.into(),
            channel: C::CHANNEL,
            idle: config.idle,
            tick_divisor: config.divider as u32 * config.prescaler as u32,
        };

        this.park();

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
    pub fn emit<'a>(&'a mut self, pulses: &'a [Pulse]) -> ActiveTrain<'a, 'd, T> {
        assert!(!pulses.is_empty(), "a train needs at least one pulse");

        // The load value, not the period: a 32-bit counter would make `MAX + 1` overflow, and the
        // bound only holds because a period of at least two is asserted on the line above.
        let max_load = <T::Word as Word>::MAX.into();

        for pulse in pulses {
            assert!(pulse.high > 0 && pulse.low > 0, "a pulse needs a high and a low time");
            assert!(
                pulse.period() - 1 <= max_load,
                "a pulse is longer than the counter can count"
            );
        }

        self.arm(pulses);

        ActiveTrain { train: self }
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

        low_level::enable_interrupt(r, Event::Zero, false);

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
        let next = 1;

        // Read through, never written — `AtomicPtr` is the only pointer atomic there is.
        state.pulses.store(pulses.as_ptr().cast_mut(), Ordering::Relaxed);
        state.len.store(pulses.len(), Ordering::Relaxed);
        state.next.store(next, Ordering::Relaxed);
        state.seen.store(0, Ordering::Relaxed);
        state.channel.store(channel.index() as u8, Ordering::Relaxed);
        state
            .idle_high
            .store(matches!(self.idle, Level::High), Ordering::Relaxed);
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
        self.timer.stop();

        // A completed train stops on a boundary, with the counter already at zero. A cancelled one
        // stops wherever the element had got to, and zeroing that at the next enable costs the first
        // element a couple of ticks — visible only on the train *after* a cancelled one, which is
        // not where anybody would look.
        self.timer.set_counter(<T::Word as Word>::from_reg(0));

        // A completed train leaves the closing action in the register, transferred by the boundary
        // that ended it. A cancelled one leaves it stuck in the shadow, because the counter stopped
        // before any boundary could transfer it — so the update has to go back to immediate before
        // the same action is written here. Without this the *next* train's first element comes out
        // long, and only that one, which is a nasty thing to go looking for.
        self.timer.set_action_update(channel, CompareUpdate::Immediately);

        r.counterregs(0).ccact(channel.index()).modify(|w| {
            w.set_zact(self.idle_act());
            w.set_cuact(self.idle_act());
        });

        // The output generator latches the last level an action drove, and the latch is not a
        // register. The software force cannot reach it while the counter is stopped either: a
        // forced action is itself deferred to a period boundary (SLAU846E 34.2.5.3), and a stopped
        // counter never has one. A cancelled train leaves the cancelled element's active level in
        // the latch, and at the next enable the generator drives the pin with it from `EN` until
        // the enable-time zero event lands — a stretch of the first element's high time. A
        // completed train's closing boundary action parks the latch, so only a cancellation needs
        // the repair: run the counter for one throwaway zero event with every action set to idle
        // and the output held by `ODIS`, which is the one thing that does drive the latch.
        if !T::train_state().done.load(Ordering::Acquire) {
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

            self.timer.set_output_enabled(channel, true);
        }

        T::train_state().len.store(0, Ordering::Relaxed);
        T::train_state().pulses.store(core::ptr::null_mut(), Ordering::Relaxed);

        self.park();
    }

    /// The compare action that drives the pin to the configured idle level.
    fn idle_act(&self) -> Act {
        act(self.idle)
    }

    /// Hold the output at the idle level through the forced-output override.
    ///
    /// The override is what expresses a level no compare value can, and it is the only thing holding
    /// the pin between trains — the counter is stopped, so the compare actions never fire.
    fn park(&mut self) {
        self.timer.set_forced_output(self.channel, Some(self.idle));
        self.timer.set_output_enabled(self.channel, true);
    }
}

/// A train that is running.
///
/// Awaiting it resolves when the last element has finished. Dropping it, including by losing a
/// `select!`, stops the train and parks the output, so the pin never rests part way through an
/// element. The elements stay borrowed for as long as this value lives, because the handler is
/// reading them.
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
