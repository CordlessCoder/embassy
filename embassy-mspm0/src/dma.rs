//! Direct Memory Access (DMA)

#![macro_use]

use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::{Context, Poll};
use core::{fmt, mem};

use critical_section::CriticalSection;
use embassy_hal_internal::PeripheralType;
use embassy_hal_internal::interrupt::InterruptExt;
use mspm0_metapac::common::{RW, Reg};
use mspm0_metapac::dma::regs;
use mspm0_metapac::dma::vals::{self, Autoen, Em, Incr, Preirq, Wdth};

use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::sync::irq_waker::IrqWaker;
use crate::sysctl::{MaybeWakeGuard, SleepLevel};
use crate::{Peri, interrupt, pac};

/// DMA interrupt handler.
pub struct InterruptHandler<T: ChannelInstance> {
    _marker: PhantomData<T>,
}

impl<T: ChannelInstance> Handler<<T as ChannelInstance>::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_irq(pac::DMA);
    }
}

/// The burst size of a DMA transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BurstSize {
    /// The whole block transfer is completed in one transfer without interruption.
    Complete,

    /// The burst size is 8, after 9 transfers the block transfer is interrupted and the priority
    /// is reevaluated.
    _8,

    /// The burst size is 16, after 17 transfers the block transfer is interrupted and the priority
    /// is reevaluated.
    _16,

    /// The burst size is 32, after 33 transfers the block transfer is interrupted and the priority
    /// is reevaluated.
    _32,
}

/// Basic DMA channel driver.
pub struct Channel<'d> {
    id: u8,
    sw_wake_floor: Option<SleepLevel>,
    _marker: PhantomData<&'d ()>,
}

impl<'d> Channel<'d> {
    /// Create a new basic DMA channel driver.
    pub fn new<T: ChannelInstance>(
        _ch: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        Self {
            id: T::ID,
            sw_wake_floor: <T as crate::sysctl::LowPowerInstance>::SLEEP
                .floor_for_operation(crate::sysctl::with_clocks(|clocks| clocks.mclk)),
            _marker: PhantomData,
        }
    }

    /// Reborrow the channel, allowing it to be used in multiple places.
    pub fn reborrow(&mut self) -> Channel<'_> {
        Channel {
            id: self.id,
            sw_wake_floor: self.sw_wake_floor,
            _marker: PhantomData,
        }
    }

    /// Floor to hold for a transfer that nothing else keeps the DMA clocked through.
    ///
    /// A hardware trigger reaches the event manager, which suspends STOP or STANDBY for as long as the
    /// transfer needs (TRM, "Suspended Low-Power Mode Operation"). A software request never reaches it,
    /// so deep sleep would cut the transfer until something unrelated woke the device.
    fn transfer_guard(&self, trigger_source: u8) -> MaybeWakeGuard {
        if trigger_source != Transfer::SOFTWARE_TRIGGER {
            return MaybeWakeGuard::none();
        }

        MaybeWakeGuard::new(self.sw_wake_floor)
    }

    /// Create a new read DMA transfer.
    ///
    /// # Safety
    ///
    /// `src` must be valid for reads of `dst.len()` words for as long as the transfer runs, and must
    /// not be written by anything else meanwhile. The hardware writes `dst` behind the compiler's
    /// back, so the returned [`Transfer`] must be awaited, `blocking_wait`ed or dropped before `dst`
    /// is read; leaking it with [`mem::forget`](core::mem::forget()) leaves the DMA writing into memory
    /// the borrow checker considers free again.
    pub unsafe fn read<'a, SW: Word, DW: Word>(
        &'a mut self,
        trigger_source: u8,
        src: *mut SW,
        dst: &'a mut [DW],
        options: TransferOptions,
    ) -> Result<Transfer<'a>, Error> {
        self.read_raw(trigger_source, src, dst, options)
    }

    /// Create a new read DMA transfer, using raw pointers.
    ///
    /// # Safety
    ///
    /// As [`read`](Self::read), and additionally `dst` must be valid for writes for its whole length
    /// for as long as the transfer runs. Nothing here ties that to a lifetime — the caller keeps the
    /// destination alive.
    pub unsafe fn read_raw<'a, SW: Word, DW: Word>(
        &'a mut self,
        trigger_source: u8,
        src: *mut SW,
        dst: *mut [DW],
        options: TransferOptions,
    ) -> Result<Transfer<'a>, Error> {
        verify_transfer::<DW>(dst)?;

        let wake_guard = self.transfer_guard(trigger_source);
        let transfer = Transfer {
            channel: self.reborrow(),
            wake_guard,
        };
        transfer.channel.configure(
            trigger_source,
            src.cast(),
            SW::width(),
            dst.cast(),
            DW::width(),
            dst.len() as u16,
            false,
            true,
            options,
        );
        transfer.channel.start();

        Ok(transfer)
    }

    /// Create a new write DMA transfer.
    ///
    /// # Safety
    ///
    /// `dst` must be valid for writes of `src.len()` words for as long as the transfer runs, and must
    /// not be read or written by anything else meanwhile. The returned [`Transfer`] must be awaited,
    /// `blocking_wait`ed or dropped before `src` is reused; leaking it with
    /// [`mem::forget`](core::mem::forget()) leaves the DMA reading memory the borrow checker considers
    /// free again.
    pub unsafe fn write<'a, SW: Word, DW: Word>(
        &'a mut self,
        trigger_source: u8,
        src: &'a [SW],
        dst: *mut DW,
        options: TransferOptions,
    ) -> Result<Transfer<'a>, Error> {
        self.write_raw(trigger_source, src, dst, options)
    }

    /// Create a new write DMA transfer, using raw pointers.
    ///
    /// # Safety
    ///
    /// As [`write`](Self::write), and additionally `src` must be valid for reads for its whole length
    /// for as long as the transfer runs. Nothing here ties that to a lifetime — the caller keeps the
    /// source alive.
    pub unsafe fn write_raw<'a, SW: Word, DW: Word>(
        &'a mut self,
        trigger_source: u8,
        src: *const [SW],
        dst: *mut DW,
        options: TransferOptions,
    ) -> Result<Transfer<'a>, Error> {
        verify_transfer::<SW>(src)?;

        let wake_guard = self.transfer_guard(trigger_source);
        let transfer = Transfer {
            channel: self.reborrow(),
            wake_guard,
        };
        transfer.channel.configure(
            trigger_source,
            src.cast(),
            SW::width(),
            dst.cast(),
            DW::width(),
            src.len() as u16,
            true,
            false,
            options,
        );
        transfer.channel.start();

        Ok(transfer)
    }
}

/// Full DMA channel driver.
///
/// This is an extended [`Channel`] driver and can be [reborrowed](Self::reborrow) to be used as a basic
/// channel driver.
pub struct FullChannel<'d>(Channel<'d>);

impl<'d> FullChannel<'d> {
    /// Create a new full DMA channel driver.
    pub fn new<T: FullChannelInstance>(
        ch: Peri<'d, T>,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        Self(Channel::new(ch, irq))
    }

    /// Reborrow the full channel, allowing it to be used in multiple places.
    pub fn reborrow_full(&mut self) -> FullChannel<'_> {
        FullChannel(self.0.reborrow())
    }

    /// Reborrow the channel as a basic [`Channel`], allowing it to be used in multiple places.
    pub fn reborrow(&mut self) -> Channel<'_> {
        self.0.reborrow()
    }
}

/// DMA channel instance.
#[allow(private_bounds)]
pub trait ChannelInstance: SealedChannel + PeripheralType + crate::sysctl::LowPowerInstance {
    /// Interrupt type for this DMA channel.
    type Interrupt: Interrupt;
}

/// Full DMA channel instance.
#[allow(private_bounds)]
pub trait FullChannelInstance: ChannelInstance {}

#[allow(private_bounds)]
pub trait Word: SealedWord + 'static {
    /// Size in bytes for the width.
    fn size() -> isize;
}

impl SealedWord for u8 {
    fn width() -> vals::Wdth {
        vals::Wdth::Byte
    }
}
impl Word for u8 {
    fn size() -> isize {
        1
    }
}

impl SealedWord for u16 {
    fn width() -> vals::Wdth {
        vals::Wdth::Half
    }
}
impl Word for u16 {
    fn size() -> isize {
        2
    }
}

impl SealedWord for u32 {
    fn width() -> vals::Wdth {
        vals::Wdth::Word
    }
}
impl Word for u32 {
    fn size() -> isize {
        4
    }
}

impl SealedWord for u64 {
    fn width() -> vals::Wdth {
        vals::Wdth::Long
    }
}
impl Word for u64 {
    fn size() -> isize {
        8
    }
}

// TODO: u128 (LONGLONG) support. G350x does support it, but other parts do not such as C110x. More metadata is
// needed to properly enable this.
// impl SealedWord for u128 {
//     fn width() -> vals::Wdth {
//         vals::Wdth::LONGLONG
//     }
// }
// impl Word for u128 {
//     fn size() -> isize {
//         16
//     }
// }

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The DMA transfer is too large.
    ///
    /// The hardware limits the DMA to 16384 transfers per channel at a time. This means that transferring
    /// 16384 `u8` and 16384 `u64` are equivalent, since the DMA must copy 16384 values.
    TooManyTransfers,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::TooManyTransfers => write!(f, "too many transfers"),
        }
    }
}

impl core::error::Error for Error {}

/// DMA transfer mode for basic channels.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransferMode {
    /// Each DMA trigger will transfer a single value.
    Single,

    /// Each DMA trigger will transfer the complete block with one trigger.
    Block,
}

/// DMA transfer options.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    /// DMA transfer mode.
    pub mode: TransferMode,
    // TODO: Read and write stride.
}

impl Default for TransferOptions {
    fn default() -> Self {
        Self {
            mode: TransferMode::Single,
        }
    }
}

/// DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: Channel<'a>,
    wake_guard: MaybeWakeGuard,
}

impl<'a> Transfer<'a> {
    /// Software trigger source.
    ///
    /// Using this trigger source means that a transfer will start immediately rather than waiting for
    /// a hardware event. This can be useful if you want to do a DMA accelerated memcpy.
    pub const SOFTWARE_TRIGGER: u8 = 0;

    /// Request the transfer to resume.
    pub fn resume(&mut self) {
        self.channel.resume();
    }

    /// Request the transfer to pause, keeping the existing configuration for this channel.
    /// To restart the transfer, call [`resume`](Self::resume).
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_pause(&mut self) {
        self.channel.request_pause();
    }

    /// Return whether this transfer is still running.
    ///
    /// If this returns [`false`], it can be because either the transfer finished, or
    /// it was paused with [`request_pause`](Self::request_pause).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Blocking wait until the transfer finishes.
    pub fn blocking_wait(mut self) {
        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);

        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);

        // Prevent drop from being called since we ran to completion (drop will try to pause). The wake
        // guard still has to be released, or it would block deep sleep for the rest of the program.
        self.wake_guard.release();
        mem::forget(self);
    }
}

impl<'a> Unpin for Transfer<'a> {}
impl<'a> Future for Transfer<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        #[cfg(feature = "_probe")]
        let probe = crate::probe::target(crate::probe::Marker::DmaPoll);
        #[cfg(feature = "_probe")]
        crate::probe::set(probe);

        let state: &ChannelState = &STATE[self.channel.id as usize];

        state.waker.register(cx.waker());

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);

        let running = self.channel.is_running();

        #[cfg(feature = "_probe")]
        crate::probe::clear(probe);

        if running { Poll::Pending } else { Poll::Ready(()) }
    }
}

impl<'a> Drop for Transfer<'a> {
    fn drop(&mut self) {
        self.channel.request_pause();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);
    }
}

// impl details

fn verify_transfer<W: Word>(ptr: *const [W]) -> Result<(), Error> {
    if ptr.len() > (u16::MAX as usize) {
        return Err(Error::TooManyTransfers);
    }

    // TODO: Stride checks

    Ok(())
}

fn convert_burst_size(value: BurstSize) -> vals::Burstsz {
    match value {
        BurstSize::Complete => vals::Burstsz::Infiniti,
        BurstSize::_8 => vals::Burstsz::Burst8,
        BurstSize::_16 => vals::Burstsz::Burst16,
        BurstSize::_32 => vals::Burstsz::Burst32,
    }
}

fn convert_mode(mode: TransferMode) -> vals::Tm {
    match mode {
        TransferMode::Single => vals::Tm::Single,
        TransferMode::Block => vals::Tm::Block,
    }
}

const CHANNEL_COUNT: usize = crate::_generated::DMA_CHANNELS;
static STATE: [ChannelState; CHANNEL_COUNT] = [const { ChannelState::new() }; CHANNEL_COUNT];

struct ChannelState {
    /// Woken by [`on_irq`], which is the only waker side: every channel's interrupt is the one `DMA`
    /// line, so the handler cannot preempt itself.
    waker: IrqWaker,
}

impl ChannelState {
    const fn new() -> Self {
        Self { waker: IrqWaker::new() }
    }
}

/// SAFETY: Must only be called once.
///
/// Changing the burst size mid transfer may have some odd behavior.
pub(crate) unsafe fn init(_cs: CriticalSection, burst_size: BurstSize, round_robin: bool) {
    // Reset leaves fixed priority and an uninterrupted block transfer, which is what `Config`
    // defaults to, so a program that leaves it there has nothing to program. Folds away entirely
    // when the config is a constant.
    if !matches!(burst_size, BurstSize::Complete) || round_robin {
        pac::DMA.prio().modify(|prio| {
            prio.set_burstsz(convert_burst_size(burst_size));
            prio.set_roundrobin(round_robin);
        });
    }

    pac::DMA.int_event(0).imask().modify(|w| {
        w.set_dataerr(true);
        w.set_addrerr(true);
    });

    interrupt::DMA.enable();
}

pub(crate) trait SealedWord {
    fn width() -> vals::Wdth;
}

pub(crate) trait SealedChannel {
    const ID: u8;
}

impl<'d> Channel<'d> {
    #[inline]
    fn tctl(&self) -> Reg<regs::Tctl, RW> {
        pac::DMA.trig(self.id as usize).tctl()
    }

    #[inline]
    fn ctl(&self) -> Reg<regs::Ctl, RW> {
        pac::DMA.chan(self.id as usize).ctl()
    }

    #[inline]
    fn sa(&self) -> Reg<u32, RW> {
        pac::DMA.chan(self.id as usize).sa()
    }

    #[inline]
    fn da(&self) -> Reg<u32, RW> {
        pac::DMA.chan(self.id as usize).da()
    }

    #[inline]
    fn sz(&self) -> Reg<regs::Sz, RW> {
        pac::DMA.chan(self.id as usize).sz()
    }

    #[inline]
    fn mask_interrupt(&self, enable: bool) {
        // Enabling interrupts is an RMW operation.
        critical_section::with(|_cs| {
            pac::DMA.int_event(0).imask().modify(|w| {
                w.set_ch(self.id as usize, enable);
            });
        })
    }

    /// # Safety
    ///
    /// - `src` must be valid for the lifetime of the transfer.
    /// - `dst` must be valid for the lifetime of the transfer.
    unsafe fn configure(
        &self,
        trigger_sel: u8,
        src: *const u32,
        src_wdth: Wdth,
        dst: *const u32,
        dst_wdth: Wdth,
        transfer_count: u16,
        increment_src: bool,
        increment_dst: bool,
        options: TransferOptions,
    ) {
        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);

        // SLAU 5.2.5:
        // "The DMATSEL bits should be modified only when the DMACTLx.DMAEN bit is
        //  0; otherwise, unpredictable DMA triggers can occur."
        //
        // This has to be a write of its own: the channel has to be *already* disabled for the rest of
        // DMACTL to take, so a store that clears DMAEN alongside them leaves them at their old values.
        // DMATM is the one that shows it — the channel keeps whatever mode it was last configured for.
        self.ctl().modify(|w| {
            w.set_en(false);
            w.set_req(false);
        });

        self.ctl().modify(|w| {
            // Not every part supports auto enable, so force its value to 0.
            w.set_autoen(Autoen::None);
            w.set_preirq(Preirq::PreirqDisable);
            w.set_srcwdth(src_wdth);
            w.set_dstwdth(dst_wdth);
            w.set_srcincr(if increment_src {
                Incr::Increment
            } else {
                Incr::Unchanged
            });
            w.set_dstincr(if increment_dst {
                Incr::Increment
            } else {
                Incr::Unchanged
            });

            w.set_em(Em::Normal);
            // Single and block will clear the enable bit when the transfers finish.
            w.set_tm(convert_mode(options.mode));
        });

        self.tctl().write(|w| {
            w.set_tsel(trigger_sel);
            // Basic channels do not implement cross triggering.
            w.set_tint(vals::Tint::External);
        });

        self.sz().write(|w| {
            w.set_size(transfer_count);
        });

        self.sa().write_value(src as u32);
        self.da().write_value(dst as u32);

        // Left disabled. Enabling is what starts a software-triggered transfer, so it belongs in
        // `start`, after the interrupt has been armed.
    }

    /// Arm the completion interrupt, then trigger the transfer.
    ///
    /// The order is the point. Enabling the channel before arming runs a transfer whose completion is
    /// reported to nobody, and the handler masks the channel off on its way out, so the transfer the
    /// caller goes on to await has nothing left to interrupt with.
    fn start(&self) {
        // A stale flag does the same thing from the other direction: it latches whether or not the
        // channel is unmasked, so one left over from an earlier transfer fires the moment the channel
        // is armed and is again masked off before this transfer has been triggered.
        pac::DMA.int_event(0).iclr().write(|w| {
            w.set_ch(self.id as usize, true);
        });

        self.mask_interrupt(true);

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);

        #[cfg(feature = "_probe")]
        crate::probe::count(crate::probe::target(crate::probe::Marker::DmaTrigger));

        // Enable and request together, with the interrupt already armed above.
        self.ctl().modify(|w| {
            w.set_en(true);
            w.set_req(true);
        });
    }

    fn resume(&self) {
        self.mask_interrupt(true);

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);

        // SLAU846 5.2.6: a halted block transfer continues once `DMAEN` is set again *and* a trigger is
        // resent — "a trigger is necessary for halted transfer to resume". `DMAREQ` supplies that for a
        // software-triggered channel and is ignored by one waiting on a hardware source, which is why it
        // is asserted unconditionally.
        self.ctl().modify(|w| {
            w.set_en(true);
            w.set_req(true);
        });
    }

    fn request_pause(&self) {
        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);

        // Stop the transfer.
        //
        // SLAU846 5.2.6:
        // "A DMA block transfer in progress can be stopped by clearing the DMAEN bit"
        //
        // `DMAEN` is what stops it, and it is also what `is_running` reports, so leaving it set means a
        // cancelled transfer never reads as stopped and `Drop` spins on it forever. `DMAREQ` goes too,
        // dropping a software request the hardware has not taken up yet.
        self.ctl().modify(|w| {
            w.set_en(false);
            w.set_req(false);
        });
    }

    fn is_running(&self) -> bool {
        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::SeqCst);

        // DMAEN alone. The hardware clears it when a single or block transfer runs out of DMASZ, which
        // is what "finished" means here; DMAREQ is the trigger and the hardware clears that as soon as
        // it takes the request up, so testing it as well reports a transfer that is still writing as
        // done — and the waiter that believes it walks away while the DMA still holds the buffer.
        self.ctl().read().en()
    }
}

// Some future parts will have no DMA.
#[allow(unused_macros)]
macro_rules! impl_dma_channel {
    ($instance: ident, $num: expr) => {
        impl crate::dma::SealedChannel for crate::peripherals::$instance {
            const ID: u8 = $num;
        }

        impl crate::dma::ChannelInstance for crate::peripherals::$instance {
            // One line for every channel: no supported chip has a second DMA, checked across all of
            // them. A part that grows one needs the instance picked per channel here.
            type Interrupt = crate::interrupt::typelevel::DMA;
        }
    };
}

// C1104 has no full DMA channels.
#[allow(unused_macros)]
macro_rules! impl_full_dma_channel {
    ($instance: ident, $num: expr) => {
        impl_dma_channel!($instance, $num);

        impl crate::dma::FullChannelInstance for crate::peripherals::$instance {}
    };
}

/// Lowest `IIDX` index that names a channel; zero is "nothing pending".
const IIDX_CH0: u8 = pac::dma::vals::Stat::Ch0.to_bits();

/// The two error indices, which sort above every channel and every `PRE-IRQ`.
const IIDX_ADDRERR: u8 = pac::dma::vals::Stat::Addrerr.to_bits();
const IIDX_DATAERR: u8 = pac::dma::vals::Stat::Dataerr.to_bits();

fn on_irq(dma: pac::dma::Dma) {
    #[cfg(feature = "_probe")]
    let probe = crate::probe::target(crate::probe::Marker::DmaHandler);
    #[cfg(feature = "_probe")]
    crate::probe::set(probe);

    let events = dma.int_event(0);

    // `IIDX` names the highest-priority pending event and clears it, so it replaces the `MIS` read, the
    // bit scan over it — ARMv6-M has no `clz`, so that was a shift loop — and the `ICLR` write, and it
    // hands back the channel number rather than a bit position to search for.
    //
    // **One event per entry.** Clearing `MIS` is what deasserts the line, so the NVIC re-enters this
    // handler while any event remains. That is cheaper here than looping: the entry it adds is a
    // shorter one than the iteration it replaces.
    'dispatch: {
        let stat = events.iidx().read().stat().to_bits();

        // An error sorts above every channel, so a completion pending at the same instant is reported
        // first and the error arrives on the next entry. It is not lost, only later.
        match stat {
            0 => break 'dispatch,
            IIDX_DATAERR => panic!("DMA data error"),
            IIDX_ADDRERR => panic!("DMA address error"),
            _ => {}
        }

        // `PRE-IRQ` indices sit above the channels and below the errors. Every channel disables it in
        // `CTL`, so nothing unmasks one and `IIDX` cannot report it — but it costs a bounds check to
        // say so rather than indexing past the end.
        let channel = (stat - IIDX_CH0) as usize;

        let Some(state) = STATE.get(channel) else {
            break 'dispatch;
        };

        state.waker.wake();

        // Nothing more to report until the next transfer arms it again. Masking is not clearing — the
        // flag would stay latched in `RIS` and the next unmask would raise the interrupt at once — but
        // reading `IIDX` above already cleared it.
        events.imask().modify(|w| {
            w.set_ch(channel, false);
        });
    }

    #[cfg(feature = "_probe")]
    crate::probe::clear(probe);
}
