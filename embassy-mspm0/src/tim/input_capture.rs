//! Input capture.
//!
//! The counter runs free, so a captured value is a tick timestamp; subtract two to get an interval.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use crate::gpio::{AnyPin, PfType, Pull, SealedPin};
use crate::interrupt::typelevel::Interrupt as _;
use crate::pac::tim::Tim;
use crate::pac::tim::vals::{Ccond, Coc, Cpv, Fp, Isel};
use crate::tim::low_level::{self, Config as TimerConfig, Event, Timer};
use crate::tim::{
    Ch0, Ch1, Ch2, Ch3, Channel, ClockSel, CountingMode, General2ChannelInstance, General4ChannelInstance, Instance,
    State, TimerChannel, TimerPin, Word,
};
use crate::{Peri, interrupt};

/// Which edge on the input captures the counter.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CaptureEdge {
    /// Capture on a rising edge.
    #[default]
    Rising,

    /// Capture on a falling edge.
    Falling,

    /// Capture on both edges.
    Both,
}

/// Glitch filter on a capture input.
///
/// The voting variant, which tolerates one opposite sample instead, is reachable via [`Timer::regs`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Filter {
    /// No filtering: every edge the synchroniser sees captures.
    #[default]
    None,

    /// Require the level to hold for 3 ticks.
    Ticks3,

    /// Require the level to hold for 5 ticks.
    Ticks5,

    /// Require the level to hold for 8 ticks.
    Ticks8,
}

/// Input capture configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Clock source driving the counter, which sets what one captured tick is worth.
    pub clock: ClockSel,

    /// Divider applied to the clock source, 1 to 8.
    pub divider: u8,

    /// Further divider applied after [`Self::divider`], 1 to 256.
    pub prescaler: u16,

    /// Keep counting while the debugger holds the core halted.
    pub free_run_in_debug: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock: ClockSel::default(),
            divider: 1,
            prescaler: 1,
            free_run_in_debug: false,
        }
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::info().regs;
        let state = T::state();

        // Other events the caller enabled through `Timer` are not ours to acknowledge.
        let fired = r.cpu_int(0).mis().read().0 & low_level::CC_UP_BITS;

        // Mask rather than clear: the flag has to survive until the future reads `CC`.
        r.cpu_int(0).imask().modify(|w| w.0 &= !fired);

        for channel in Channel::ALL {
            if fired & Event::CaptureOrCompareUp(channel).mask().0 != 0 {
                state.cc[channel.index()].wake();
            }
        }
    }
}

/// A pin captured on channel `C`.
pub struct CapturePin<'d, T: Instance, C: TimerChannel> {
    pin: Peri<'d, AnyPin>,
    edge: CaptureEdge,
    filter: Filter,
    _phantom: PhantomData<(T, C)>,
}

impl<'d, T: Instance, C: TimerChannel> CapturePin<'d, T, C> {
    /// Claim `pin` as this channel's capture input.
    pub fn new(pin: Peri<'d, impl TimerPin<T, C>>, pull: Pull, edge: CaptureEdge, filter: Filter) -> Self {
        pin.set_as_pf(pin.pf_num(), PfType::input(pull, false));

        Self {
            pin: pin.into(),
            edge,
            filter,
            _phantom: PhantomData,
        }
    }

    fn erase(self) -> (Peri<'d, AnyPin>, CaptureEdge, Filter) {
        (self.pin, self.edge, self.filter)
    }
}

/// Input capture driver.
pub struct InputCapture<'d, T: Instance> {
    timer: Timer<'d, T>,
    pins: [Option<Peri<'d, AnyPin>>; 4],
}

/// One channel's settings, taken before the pin types are erased.
type Channels<'d> = [Option<(Peri<'d, AnyPin>, CaptureEdge, Filter)>; 4];

impl<'d, T: General2ChannelInstance> InputCapture<'d, T> {
    /// Configure a two-channel timer to capture, and start the counter.
    pub fn new_2ch(
        timer: Peri<'d, T>,
        ch0: Option<CapturePin<'d, T, Ch0>>,
        ch1: Option<CapturePin<'d, T, Ch1>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        Self::build(
            timer,
            [ch0.map(CapturePin::erase), ch1.map(CapturePin::erase), None, None],
            config,
        )
    }
}

impl<'d, T: General4ChannelInstance> InputCapture<'d, T> {
    /// Configure a four-channel timer to capture, and start the counter.
    pub fn new_4ch(
        timer: Peri<'d, T>,
        ch0: Option<CapturePin<'d, T, Ch0>>,
        ch1: Option<CapturePin<'d, T, Ch1>>,
        ch2: Option<CapturePin<'d, T, Ch2>>,
        ch3: Option<CapturePin<'d, T, Ch3>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        Self::build(
            timer,
            [
                ch0.map(CapturePin::erase),
                ch1.map(CapturePin::erase),
                ch2.map(CapturePin::erase),
                ch3.map(CapturePin::erase),
            ],
            config,
        )
    }
}

impl<'d, T: Instance> InputCapture<'d, T> {
    fn build(timer: Peri<'d, T>, channels: Channels<'d>, config: Config) -> Self {
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

        let settings = channels
            .each_ref()
            .map(|c| c.as_ref().map(|(_, edge, filter)| (*edge, *filter)));

        let mut this = Self {
            timer,
            pins: channels.map(|c| c.map(|(pin, _, _)| pin)),
        };

        for channel in Channel::ALL {
            if let Some((edge, filter)) = settings[channel.index()] {
                this.setup_channel(channel, edge, filter);
            }
        }

        unsafe { T::Interrupt::enable() };

        this.timer.start();

        this
    }

    /// Program one channel's compare block for capture, following SLAU847F 28.2.3.1.2.1.
    fn setup_channel(&mut self, channel: Channel, edge: CaptureEdge, filter: Filter) {
        let r = self.timer.regs();
        let n = channel.index();

        r.counterregs(0).ccctl(n).modify(|w| {
            w.set_coc(Coc::Capture);
            w.set_ccond(match edge {
                CaptureEdge::Rising => Ccond::CcTrigRise,
                CaptureEdge::Falling => Ccond::CcTrigFall,
                CaptureEdge::Both => Ccond::CcTrigEdge,
            });
        });

        r.commonregs(0).ccpd().modify(|w| w.set_c0ccp(n, false));

        r.counterregs(0).ifctl(n).write(|w| {
            w.set_isel(Isel::CcpxInput);
            w.set_inv(false);
            w.set_cpv(Cpv::Consecutive);
            w.set_fe(filter != Filter::None);

            w.set_fp(match filter {
                Filter::None | Filter::Ticks3 => Fp::_3,
                Filter::Ticks5 => Fp::_5,
                Filter::Ticks8 => Fp::_8,
            });
        });

        // Discard any edge from before this driver existed.
        self.timer.clear_pending(Event::CaptureOrCompareUp(channel));
    }

    /// Borrow one channel to await its captures.
    pub fn channel(&mut self, channel: Channel) -> CaptureChannel<'_, <T as Instance>::Word> {
        CaptureChannel {
            regs: self.timer.regs(),
            state: T::state(),
            channel,
            _phantom: PhantomData,
        }
    }

    /// The underlying counter.
    pub fn timer(&self) -> &Timer<'d, T> {
        &self.timer
    }
}

/// One channel of an [`InputCapture`].
///
/// `W` is kept so subtracting two captures wraps at the counter's width, which is the only width that
/// gives the right interval.
pub struct CaptureChannel<'d, W: Word> {
    regs: Tim,
    state: &'static State,
    channel: Channel,
    _phantom: PhantomData<(&'d mut (), W)>,
}

impl<W: Word> CaptureChannel<'_, W> {
    /// Wait for the next capture and return the counter value it recorded.
    ///
    /// One capture register per channel, so a further edge before this returns overwrites the value.
    pub async fn wait_for_capture(&mut self) -> W {
        let event = Event::CaptureOrCompareUp(self.channel);

        poll_fn(|cx| {
            self.state.cc[self.channel.index()].register(cx.waker());

            if low_level::is_pending(self.regs, event) {
                let value = W::from_reg(self.regs.counterregs(0).cc(self.channel.index()).read());
                low_level::clear_pending(self.regs, event);

                return Poll::Ready(value);
            }

            low_level::enable_interrupt(self.regs, event, true);

            Poll::Pending
        })
        .await
    }

    /// Whether a capture is waiting to be read.
    pub fn is_captured(&self) -> bool {
        low_level::is_pending(self.regs, Event::CaptureOrCompareUp(self.channel))
    }

    /// Discard any capture that is already waiting.
    pub fn clear(&mut self) {
        low_level::clear_pending(self.regs, Event::CaptureOrCompareUp(self.channel));
    }
}

impl<T: Instance> Drop for InputCapture<'_, T> {
    fn drop(&mut self) {
        for pin in self.pins.iter().flatten() {
            pin.set_as_disconnected();
        }
    }
}
