//! Pulse-width capture.
//!
//! One pin reaches both channels of a pair, so one channel captures the pulse's leading edge and the
//! other its trailing edge. The difference between the two capture registers is the width. That is
//! one interrupt per pulse, and neither timestamp has to be carried in software between them.
//!
//! The counter runs free. A pulse that outlasts one turn of the counter reads as a short one, so
//! pick the clock and the dividers to fit the longest pulse expected.
//!
//! This is the arrangement driverlib reaches for too, in `DL_Timer_initCaptureCombinedMode`. The
//! mode the TRM names for measuring a pulse instead starts the counter from the input's own edge,
//! and `TIMER_ERR_01` makes that capture the start value rather than the counter on more than half
//! the portfolio. Nothing here sets a zero or load condition, so the erratum does not reach it.
//!
//! # Cancelling a wait
//!
//! Dropping [`PulseWidth::wait_for_width`]'s future leaves both channels armed and the interrupt
//! unmasked, so the next pulse enters the handler, which stores its width and finds no waiter. That
//! width is then what the following wait returns, which is a stale measurement rather than a fresh
//! one — the next wait resolves on the pulse *after* the one it was called for.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::Ordering;
use core::task::Poll;

use portable_atomic::{AtomicBool, AtomicU32};

use crate::gpio::{AnyPin, PfType, Pull, SealedPin};
use crate::interrupt::typelevel::Interrupt as _;
use crate::tim::input_capture::{self, CaptureEdge, CaptureInput, Filter};
use crate::tim::low_level::{self, Config as TimerConfig, Event, Events, Timer};
use crate::tim::{Channel, ClockSel, CountingMode, General2ChannelInstance, Instance, TimerChannel, TimerPin, Word};
use crate::{Peri, interrupt};

/// Where the interrupt handler leaves the pulse it measured.
///
/// The handler reads both capture registers itself, because the leading one is overwritten by the
/// next pulse and an executor takes longer to arrive than a short gap allows.
pub struct Snapshot {
    /// Width in counter ticks of the pulse [`Self::fresh`] describes.
    width: AtomicU32,

    /// Whether [`Self::width`] holds a pulse nothing has taken yet.
    fresh: AtomicBool,
}

impl Snapshot {
    pub(crate) const fn new() -> Self {
        Self {
            width: AtomicU32::new(0),
            fresh: AtomicBool::new(false),
        }
    }

    /// Whether a pulse is waiting.
    fn is_fresh(&self) -> bool {
        self.fresh.load(Ordering::Acquire)
    }

    /// Take the waiting pulse, if there is one.
    ///
    /// Plain loads and stores rather than a swap, which on this core is a critical section. The
    /// handler masks its own interrupt after storing and only [`PulseWidth::wait_for_width`]
    /// unmasks it, so nothing writes here while this reads.
    fn take(&self) -> Option<u32> {
        self.is_fresh().then(|| {
            let width = self.width.load(Ordering::Relaxed);
            self.fresh.store(false, Ordering::Relaxed);

            width
        })
    }

    /// Discard whatever is waiting.
    fn clear(&self) {
        self.fresh.store(false, Ordering::Relaxed);
    }

    /// Record a pulse. The handler is the only caller.
    fn store(&self, width: u32) {
        self.width.store(width, Ordering::Relaxed);
        self.fresh.store(true, Ordering::Release);
    }
}

/// Interrupt handler.
///
/// Unlike [`input_capture`]'s, this one reads the capture registers rather than leaving them for the
/// waiting task: the leading register survives only until the next pulse begins, and on a signal
/// whose pulses nearly fill their period that is shorter than an executor wake.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::info().regs;

        // Other events the caller enabled through `Timer` are not ours to acknowledge.
        let fired = low_level::active(r).intersection(Events::ANY_CAPTURE_OR_COMPARE_UP);

        // The instance's own channels, not all four: a two-channel timer has neither the slots nor
        // the events for the other two.
        for (index, waker) in T::cc_wakers().iter().enumerate() {
            let channel = Channel::ALL[index];
            let closed = Event::CaptureOrCompareUp(channel);

            if !fired.contains(closed) {
                continue;
            }

            let leading = paired(channel);

            // Leading edge first. Only that register moves when the next pulse begins, so reading it
            // before the trailing one leaves no window where the pair straddles two pulses.
            let start = low_level::compare(r, leading);
            let end = low_level::compare(r, channel);

            low_level::clear_pending(r, closed);

            // Nothing opened this pulse: the driver started part way through one, and the leading
            // register still holds its reset value. Leave the interrupt armed for the next.
            if !low_level::is_pending(r, Event::CaptureOrCompareUp(leading)) {
                continue;
            }

            // `from_reg` narrows to the counter width, which is what makes the subtraction modular:
            // a pulse spanning the counter's wrap gives the right width.
            T::pulse_snapshot().store(T::Word::from_reg(end.wrapping_sub(start)).into());

            // Masked until the next `wait_for_width`, so an unread instance costs no handler entries
            // and nothing writes the snapshot while it is being read.
            low_level::enable_interrupt(r, closed, false);

            waker.wake();
        }
    }
}

/// Which part of the input a [`PulseWidth`] measures.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PulseLevel {
    /// Time the input spends high, rising edge to falling edge.
    #[default]
    High,

    /// Time the input spends low, falling edge to rising edge.
    Low,
}

impl PulseLevel {
    /// Edge that opens the measured pulse.
    const fn leading(self) -> CaptureEdge {
        match self {
            Self::High => CaptureEdge::Rising,
            Self::Low => CaptureEdge::Falling,
        }
    }

    /// Edge that closes it.
    const fn trailing(self) -> CaptureEdge {
        match self {
            Self::High => CaptureEdge::Falling,
            Self::Low => CaptureEdge::Rising,
        }
    }
}

/// Pulse-width capture configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Clock source driving the counter, which sets what one tick of a width is worth.
    pub clock: ClockSel,

    /// Divider applied to the clock source, 1 to 8.
    pub divider: u8,

    /// Further divider applied after [`Self::divider`], 1 to 256.
    pub prescaler: u16,

    /// Keep counting while the debugger holds the core halted.
    pub free_run_in_debug: bool,

    /// Whether a pulse is a high time or a low time.
    pub level: PulseLevel,

    /// Glitch filter, applied to both channels so they see the same signal.
    pub filter: Filter,
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
            level: PulseLevel::High,
            filter: Filter::None,
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

    /// Set [`level`](Self::level).
    #[must_use]
    pub const fn with_level(mut self, level: PulseLevel) -> Self {
        self.level = level;
        self
    }

    /// Set [`filter`](Self::filter).
    #[must_use]
    pub const fn with_filter(mut self, filter: Filter) -> Self {
        self.filter = filter;
        self
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Pulse-width capture driver.
///
/// Takes one pin and the whole instance: both channels of the pin's pair are spoken for, and the
/// counter belongs to the measurement.
pub struct PulseWidth<'d, T: Instance> {
    timer: Timer<'d, T>,
    pin: Peri<'d, AnyPin>,
    channel: Channel,
}

impl<'d, T: General2ChannelInstance> PulseWidth<'d, T> {
    /// Claim `pin` and its instance to measure the pulses arriving on it.
    ///
    /// The pin can be either channel of a pair — its partner is cross-connected to it, so a pin that
    /// reaches only one channel of the instance is enough.
    pub fn new<C: TimerChannel>(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, C>>,
        pull: Pull,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        pin.set_as_pf(pin.pf_num(), PfType::input(pull, false));

        let timer = Timer::new(
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

        let mut this = Self {
            timer,
            pin: pin.into(),
            channel: C::CHANNEL,
        };

        let regs = this.timer.regs();

        // The pin's own channel takes the trailing edge, so the capture that completes a measurement
        // is the one raising the interrupt. Its setup is also what puts the pin in input mode, which
        // the cross-connected partner then reads, so it goes first.
        input_capture::setup_channel(
            regs,
            this.channel,
            CaptureInput::OwnPin,
            config.level.trailing(),
            config.filter,
        );

        input_capture::setup_channel(
            regs,
            paired(this.channel),
            CaptureInput::PairedPin,
            config.level.leading(),
            config.filter,
        );

        // A previous driver on this instance may have left one behind.
        T::pulse_snapshot().clear();

        unsafe { T::Interrupt::enable() };

        this.timer.start();

        this
    }

    /// Wait for the next complete pulse and return its width in counter ticks.
    ///
    /// Convert with [`Timer::tick_frequency`](Timer::tick_frequency), through
    /// [`timer`](Self::timer).
    ///
    /// A trailing edge whose leading edge came before the driver did is skipped rather than
    /// reported, so the first width is a width and not the time since the counter started.
    ///
    /// # The gap to the next pulse is a deadline
    ///
    /// Both captures have to be read before the next leading edge overwrites the first one. The
    /// slack is the *gap* between the pulse ending and the next one starting, not the period, and
    /// what has to fit inside it is the interrupt handler — which is why the handler does the
    /// reading and this only collects the result. **Measured at 3 us on an LP-MSPM0L1306**, against
    /// 17 us for the same driver reading the registers from here.
    ///
    /// Missing the deadline has a signature rather than being noise: the leading register holds the
    /// *next* pulse's start, so the subtraction returns `counter_range - gap` — a value a few ticks
    /// below [`Word::MAX`](crate::tim::Word::MAX). A caller measuring pulses far shorter than the
    /// counter's range can reject it with one comparison, which is a bound this driver does not
    /// have.
    pub fn wait_for_width(&mut self) -> impl Future<Output = T::Word> {
        let event = Event::CaptureOrCompareUp(self.channel);
        let regs = self.timer.regs();
        let waker = &T::cc_wakers()[self.channel.index()];

        poll_fn(move |cx| {
            waker.register(cx.waker());

            if let Some(width) = T::pulse_snapshot().take() {
                return Poll::Ready(T::Word::from_reg(width));
            }

            low_level::enable_interrupt(regs, event, true);

            Poll::Pending
        })
    }

    /// Whether a completed pulse is waiting to be read.
    pub fn is_captured(&self) -> bool {
        T::pulse_snapshot().is_fresh()
    }

    /// Discard a completed pulse that is already waiting.
    pub fn clear(&mut self) {
        T::pulse_snapshot().clear();
        low_level::clear_pending(self.timer.regs(), Event::CaptureOrCompareUp(self.channel));
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

    /// Stop measuring and give the timer and the pin back, with the pin disconnected.
    pub fn release(self) -> (Peri<'d, T>, Peri<'d, AnyPin>) {
        let this = core::mem::ManuallyDrop::new(self);

        // SAFETY: `this` is never dropped and neither field is touched again, so each moves out once.
        let timer = unsafe { core::ptr::read(&this.timer) };
        let pin = unsafe { core::ptr::read(&this.pin) };

        pin.set_as_disconnected();

        (timer.release(), pin)
    }
}

impl<T: Instance> Drop for PulseWidth<'_, T> {
    fn drop(&mut self) {
        self.pin.set_as_disconnected();
    }
}

/// The other channel of `channel`'s input pair.
///
/// The 2/3 half comes from driverlib's `DL_Timer_getInChanPairConfig`, not from the TRM: the
/// `IFCTL_23` field table is the `IFCTL_01` one verbatim, down to naming CCP0 and CCP1, so it says
/// nothing about the channels its own register serves.
const fn paired(channel: Channel) -> Channel {
    match channel {
        Channel::Ch0 => Channel::Ch1,
        Channel::Ch1 => Channel::Ch0,
        Channel::Ch2 => Channel::Ch3,
        Channel::Ch3 => Channel::Ch2,
    }
}
