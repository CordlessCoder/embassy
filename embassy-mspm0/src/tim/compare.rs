//! Output compare.
//!
//! Raises an event when the counter reaches a value, and optionally acts on a pin at the same moment.
//! Edge-aligned only: in center-aligned mode a value matches twice per period, once each way, which
//! this API has no way to express.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use crate::gpio::{AnyPin, MaybeAnyPin, PfType, Pull};
use crate::interrupt::typelevel::Interrupt as _;
use crate::pac::tim::Tim;
use crate::pac::tim::vals::{Act, Ccpiv, Ccpo, Coc};
use crate::sync::irq_waker::IrqWaker;
use crate::tim::low_level::{self, Config as TimerConfig, Event, Events, Timer};
use crate::tim::{
    Ch0, Ch1, Ch2, Ch3, Channel, ClockSel, CountingDirection, General2ChannelInstance, General4ChannelInstance,
    Instance, TimerChannel, TimerPin, Word,
};
use crate::{Peri, interrupt};

/// What the pin does when the counter reaches the compare value.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CompareAction {
    /// Leave the pin alone; the match only raises an event.
    #[default]
    None,

    /// Drive the pin high.
    SetHigh,

    /// Drive the pin low.
    SetLow,

    /// Invert the pin, giving one output period per two counter periods.
    Toggle,
}

impl CompareAction {
    const fn to_act(self) -> Act {
        match self {
            CompareAction::None => Act::Disabled,
            CompareAction::SetHigh => Act::CcpHigh,
            CompareAction::SetLow => Act::CcpLow,
            CompareAction::Toggle => Act::CcpToggle,
        }
    }
}

/// Output compare configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Counting direction, which selects the compare-up or compare-down event.
    pub direction: CountingDirection,

    /// Clock source driving the counter, which sets what one compare tick is worth.
    pub clock: ClockSel,

    /// Divider applied to the clock source, 1 to 8.
    pub divider: u8,

    /// Further divider applied after [`Self::divider`], 1 to 256.
    ///
    /// Panics if set to anything but 1 on an instance without a prescaler.
    pub prescaler: u16,

    /// Keep counting while the debugger holds the core halted.
    pub free_run_in_debug: bool,
}

impl Config {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            direction: CountingDirection::DEFAULT,
            clock: ClockSel::DEFAULT,
            divider: 1,
            prescaler: 1,
            free_run_in_debug: false,
        }
    }
}

impl Default for Config {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::info().regs;
        let wakers = T::cc_wakers();

        // Both directions, since the counting mode is a runtime choice. Only the enabled one can be
        // set in `MIS`, and other events the caller enabled through `Timer` are not ours to acknowledge.
        let fired = low_level::active(r)
            .intersection(Events::ANY_CAPTURE_OR_COMPARE_UP.union(Events::ANY_CAPTURE_OR_COMPARE_DOWN));

        // Mask rather than clear: the flag is what tells the waiting future the match happened.
        low_level::enable_interrupts(r, fired, false);

        // The instance's own channels, not all four: a two-channel timer has neither the slots nor the
        // events for the other two.
        for (index, waker) in wakers.iter().enumerate() {
            let channel = Channel::ALL[index];
            let mask =
                Events::of(Event::CaptureOrCompareUp(channel)).union(Events::of(Event::CaptureOrCompareDown(channel)));

            if !fired.intersection(mask).is_empty() {
                waker.wake();
            }
        }
    }
}

/// A pin driven by channel `C`'s compare match.
pub struct ComparePin<'d, T: Instance, C: TimerChannel> {
    pin: Peri<'d, AnyPin>,
    action: CompareAction,
    _phantom: PhantomData<(T, C)>,
}

impl<'d, T: Instance, C: TimerChannel> ComparePin<'d, T, C> {
    /// Claim `pin` as this channel's compare output.
    pub fn new(pin: Peri<'d, impl TimerPin<T, C>>, pull: Pull, action: CompareAction) -> Self {
        pin.set_as_pf(pin.pf_num(), PfType::output(pull, false));

        Self {
            pin: pin.into(),
            action,
            _phantom: PhantomData,
        }
    }

    fn erase(self) -> (Peri<'d, AnyPin>, CompareAction) {
        (self.pin, self.action)
    }
}

/// Output compare driver.
///
/// Channels without a pin still raise events, which is the usual way to use this as a timed wake.
pub struct Compare<'d, T: Instance> {
    timer: Timer<'d, T>,
    pins: [MaybeAnyPin<'d>; 4],
}

/// One channel's pin and action, taken before the pin types are erased.
type Channels<'d> = [Option<(Peri<'d, AnyPin>, CompareAction)>; 4];

impl<'d, T: General2ChannelInstance> Compare<'d, T> {
    /// Configure a two-channel timer for compare, leaving the counter stopped.
    ///
    /// Set a compare value before [`Self::start`], or the first match waits for a full wrap.
    pub fn new_2ch(
        timer: Peri<'d, T>,
        ch0: Option<ComparePin<'d, T, Ch0>>,
        ch1: Option<ComparePin<'d, T, Ch1>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        Self::build(
            timer,
            [ch0.map(ComparePin::erase), ch1.map(ComparePin::erase), None, None],
            config,
        )
    }
}

impl<'d, T: General4ChannelInstance> Compare<'d, T> {
    /// Configure a four-channel timer for compare, leaving the counter stopped.
    ///
    /// Set a compare value before [`Self::start`], or the first match waits for a full wrap.
    pub fn new_4ch(
        timer: Peri<'d, T>,
        ch0: Option<ComparePin<'d, T, Ch0>>,
        ch1: Option<ComparePin<'d, T, Ch1>>,
        ch2: Option<ComparePin<'d, T, Ch2>>,
        ch3: Option<ComparePin<'d, T, Ch3>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        Self::build(
            timer,
            [
                ch0.map(ComparePin::erase),
                ch1.map(ComparePin::erase),
                ch2.map(ComparePin::erase),
                ch3.map(ComparePin::erase),
            ],
            config,
        )
    }
}

impl<'d, T: Instance> Compare<'d, T> {
    fn build(timer: Peri<'d, T>, channels: Channels<'d>, config: Config) -> Self {
        let timer = Timer::new(
            timer,
            TimerConfig {
                clock: config.clock,
                divider: config.divider,
                prescaler: config.prescaler,
                counting_mode: config.direction.counting_mode(),
                free_run_in_debug: config.free_run_in_debug,
                ..Default::default()
            },
        );

        let actions = channels.each_ref().map(|c| c.as_ref().map(|(_, action)| *action));

        let mut this = Self {
            timer,
            pins: channels.map(|c| MaybeAnyPin::new(c.map(|(pin, _)| pin))),
        };

        for channel in Channel::ALL {
            this.setup_channel(channel, actions[channel.index()], config.direction);
        }

        unsafe { T::Interrupt::enable() };

        this
    }

    /// Put one channel in compare mode, wiring its pin only if it was given one.
    fn setup_channel(&mut self, channel: Channel, action: Option<CompareAction>, direction: CountingDirection) {
        setup_channel(self.timer.regs(), channel, action, direction);
    }

    /// Let the counter run.
    pub fn start(&mut self) {
        self.timer.start();
    }

    /// Halt the counter, keeping its value.
    pub fn stop(&mut self) {
        self.timer.stop();
    }

    /// Borrow one channel to set its compare value or await its match.
    pub fn channel(&mut self, channel: Channel) -> CompareChannel<'_, <T as Instance>::Word> {
        CompareChannel {
            regs: self.timer.regs(),
            waker: &T::cc_wakers()[channel.index()],
            channel,
            _phantom: PhantomData,
        }
    }

    /// The underlying counter.
    pub fn timer(&self) -> &Timer<'d, T> {
        &self.timer
    }
}

impl<T: Instance> Drop for Compare<'_, T> {
    fn drop(&mut self) {
        crate::tim::disconnect_pins(&self.pins);
    }
}

/// One channel of a [`Compare`].
///
/// `W` is kept so a compare value out of range for the counter cannot be written, which would
/// otherwise be a match that never happens.
pub struct CompareChannel<'d, W: Word> {
    regs: Tim,
    waker: &'static IrqWaker,
    channel: Channel,
    _phantom: PhantomData<(&'d mut (), W)>,
}

/// Which event a match raises in `mode`.
/// Put one channel in compare mode, wiring its pin only if it was given one.
///
/// Takes the register block rather than `&mut Compare<T>` so that one copy serves every timer
/// instance, the same way [`simple_pwm`](super::simple_pwm)'s does.
pub(crate) fn setup_channel(r: Tim, channel: Channel, action: Option<CompareAction>, direction: CountingDirection) {
    let n = channel.index();

    r.counterregs(0).ccctl(n).modify(|w| w.set_coc(Coc::Compare));

    if let Some(action) = action {
        r.commonregs(0).ccpd().modify(|w| w.set_c0ccp(n, true));

        r.counterregs(0).ccact(n).write(|w| match direction {
            CountingDirection::Up => w.set_cuact(action.to_act()),
            CountingDirection::Down => w.set_cdact(action.to_act()),
        });

        r.counterregs(0).octl(n).write(|w| {
            w.set_ccpo(Ccpo::Funcval);
            w.set_ccpiv(Ccpiv::Low);
            w.set_ccpoinv(false);
        });

        r.commonregs(0).odis().modify(|w| w.set_c0ccp(n, false));
    }

    // Discard any match from before this driver existed.
    low_level::clear_pending(r, event(channel, direction));
}

/// Change what a compare match does to `channel`'s pin.
///
/// The direction comes from the configured counting mode, so the field the counter is not using is
/// left alone rather than programmed to no effect.
pub(crate) fn set_action(regs: Tim, channel: Channel, action: CompareAction) {
    let direction = low_level::counting_mode(regs).direction();

    regs.counterregs(0).ccact(channel.index()).modify(|w| match direction {
        CountingDirection::Up => w.set_cuact(action.to_act()),
        CountingDirection::Down => w.set_cdact(action.to_act()),
    });
}

const fn event(channel: Channel, direction: CountingDirection) -> Event {
    match direction {
        CountingDirection::Up => Event::CaptureOrCompareUp(channel),
        CountingDirection::Down => Event::CaptureOrCompareDown(channel),
    }
}

impl<W: Word> CompareChannel<'_, W> {
    /// Event a match on this channel raises.
    fn event(&self) -> Event {
        event(self.channel, low_level::counting_mode(self.regs).direction())
    }

    /// Counter value this channel matches on.
    pub fn compare(&self) -> W {
        W::from_reg(low_level::compare(self.regs, self.channel))
    }

    /// Set the counter value to match on.
    ///
    /// Takes effect immediately, so a value the counter has already passed does not match until the
    /// next wrap.
    pub fn set_compare(&mut self, value: W) {
        low_level::set_compare(self.regs, self.channel, value.into());
    }

    /// Wait for the counter to reach the compare value.
    ///
    /// Cancelling leaves an already-fired match pending, so the next call returns immediately.
    pub fn wait_for_compare(&mut self) -> impl Future<Output = ()> {
        let event = self.event();

        poll_fn(move |cx| {
            self.waker.register(cx.waker());

            if low_level::is_pending(self.regs, event) {
                low_level::clear_pending(self.regs, event);

                return Poll::Ready(());
            }

            low_level::enable_interrupt(self.regs, event, true);

            Poll::Pending
        })
    }

    /// Set the compare value and wait for the counter to reach it.
    ///
    /// The value is written when this is called rather than when the future is first polled, so a
    /// future built and dropped without awaiting still leaves it programmed.
    pub fn wait_until(&mut self, value: W) -> impl Future<Output = ()> {
        self.set_compare(value);
        self.clear();
        self.wait_for_compare()
    }

    /// Whether a match is waiting to be acknowledged.
    pub fn is_pending(&self) -> bool {
        low_level::is_pending(self.regs, self.event())
    }

    /// Discard a match that has already fired.
    pub fn clear(&mut self) {
        low_level::clear_pending(self.regs, self.event());
    }

    /// Set what the pin does on a match, for a channel that was given one.
    pub fn set_action(&mut self, action: CompareAction) {
        set_action(self.regs, self.channel, action);
    }
}
