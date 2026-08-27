//! Direct Memory Access (DMA)
//!
//! # Cancelling a transfer
//!
//! **Dropping a [`Transfer`] waits.** It requests a pause and then spins until the channel has
//! actually stopped, because the hardware writes the destination behind the compiler's back and the
//! borrow ends when the drop returns. So a `select!` that loses a DMA race blocks for the rest of the
//! burst rather than returning at once, and a cancelled transfer is not free.

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
        arm_error_events();

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
    /// `src` must be valid for reads of as many words as this moves, and must not be written by
    /// anything else meanwhile. The hardware writes `dst` behind the compiler's back, so the returned
    /// [`Transfer`] must be awaited, `blocking_wait`ed or dropped before `dst` is read; leaking it
    /// with [`mem::forget`](core::mem::forget()) leaves the DMA writing into memory the borrow checker
    /// considers free again.
    ///
    /// `dst` bounds the memory written, not the number of words: under
    /// `TransferOptions::dst_stride` this moves `dst.len() / stride` words spread across the whole
    /// of `dst`, so a strided read wants a destination `stride` times longer than the data.
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
        assert!(
            options.mode.terminates(),
            "a repeating TransferMode never finishes; use FullChannel's repeating constructors"
        );

        unsafe { self.start_read(trigger_source, src, dst, options) }
    }

    /// The read path without the mode gate, so the repeating constructors can reach it.
    unsafe fn start_read<'a, SW: Word, DW: Word>(
        &'a mut self,
        trigger_source: u8,
        src: *mut SW,
        dst: *mut [DW],
        options: TransferOptions,
    ) -> Result<Transfer<'a>, Error> {
        // Only the destination advances on a read, so only its stride shortens the count.
        #[cfg(dma_stride)]
        let count = strided_count(dst.len(), options.dst_stride.step());
        #[cfg(not(dma_stride))]
        let count = dst.len();

        verify_transfer(count)?;

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
            count as u16,
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
    /// `dst` must be valid for writes of as many words as this moves, and must not be read or written
    /// by anything else meanwhile. The returned [`Transfer`] must be awaited, `blocking_wait`ed or
    /// dropped before `src` is reused; leaking it with [`mem::forget`](core::mem::forget()) leaves the
    /// DMA reading memory the borrow checker considers free again.
    ///
    /// `src` bounds the memory read, not the number of words: under
    /// `TransferOptions::src_stride` this moves `src.len() / stride` words taken from across the
    /// whole of `src`.
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
        assert!(
            options.mode.terminates(),
            "a repeating TransferMode never finishes; use FullChannel's repeating constructors"
        );

        unsafe { self.start_write(trigger_source, src, dst, options) }
    }

    /// The write path without the mode gate, so the repeating constructors can reach it.
    unsafe fn start_write<'a, SW: Word, DW: Word>(
        &'a mut self,
        trigger_source: u8,
        src: *const [SW],
        dst: *mut DW,
        options: TransferOptions,
    ) -> Result<Transfer<'a>, Error> {
        // Only the source advances on a write, so only its stride shortens the count.
        #[cfg(dma_stride)]
        let count = strided_count(src.len(), options.src_stride.step());
        #[cfg(not(dma_stride))]
        let count = src.len();

        verify_transfer(count)?;

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
            count as u16,
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

    /// Start a repeating read, which runs until it is stopped.
    ///
    /// [`TransferOptions::mode`] has to be one of the repeating modes; the terminating ones belong
    /// on [`Channel::read`], which returns a future instead.
    ///
    /// # Safety
    ///
    /// As [`Channel::read`], and for longer: the hardware reloads and writes `dst` again every time
    /// the count runs out, so the borrow lasts until the returned handle is dropped rather than
    /// until one pass finishes.
    pub unsafe fn read_repeating<'a, SW: Word, DW: Word>(
        &'a mut self,
        trigger_source: u8,
        src: *mut SW,
        dst: &'a mut [DW],
        options: TransferOptions,
    ) -> Result<RepeatingTransfer<'a>, Error> {
        assert!(
            !options.mode.terminates(),
            "a repeating constructor needs a repeating TransferMode"
        );

        // The gate in `read_raw` is on the terminating side, so this cannot reach it.
        unsafe { self.0.start_read(trigger_source, src, dst as *mut [DW], options) }.map(RepeatingTransfer)
    }

    /// Start a repeating write, which runs until it is stopped.
    ///
    /// [`TransferOptions::mode`] has to be one of the repeating modes; the terminating ones belong
    /// on [`Channel::write`], which returns a future instead.
    ///
    /// # Safety
    ///
    /// As [`Channel::write`], and for longer: the hardware reloads and reads `src` again every time
    /// the count runs out, so the borrow lasts until the returned handle is dropped rather than
    /// until one pass finishes.
    pub unsafe fn write_repeating<'a, SW: Word, DW: Word>(
        &'a mut self,
        trigger_source: u8,
        src: &'a [SW],
        dst: *mut DW,
        options: TransferOptions,
    ) -> Result<RepeatingTransfer<'a>, Error> {
        assert!(
            !options.mode.terminates(),
            "a repeating constructor needs a repeating TransferMode"
        );

        unsafe { self.0.start_write(trigger_source, src as *const [SW], dst, options) }.map(RepeatingTransfer)
    }
}

/// A DMA transfer that reloads and runs again instead of finishing.
///
/// There is no future here on purpose. The repeating modes leave `DMAEN` set and restore the
/// address and count registers, so nothing ever reports completion -- the transfer ends when this
/// handle is dropped or [`stop`](Self::stop) is called.
#[must_use = "the transfer stops when this is dropped"]
pub struct RepeatingTransfer<'a>(Transfer<'a>);

impl<'a> RepeatingTransfer<'a> {
    /// Whether the channel is still running.
    ///
    /// False here means it was stopped or paused; it never means the work is done.
    pub fn is_running(&mut self) -> bool {
        self.0.is_running()
    }

    /// Stop the transfer and wait for the channel to come to rest.
    ///
    /// The same wait [`Drop`] does. Naming it is what lets a caller stop without dropping the
    /// borrow.
    pub fn stop(&mut self) {
        self.0.request_pause();
        while self.0.is_running() {}

        compiler_fence(Ordering::SeqCst);
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
/// A width a transfer can move, which is what sizes each step of the address arithmetic.
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

/// Available on the seven families whose DMA is the variant that implements the 128-bit width.
///
/// TI builds the DMA as two IPs and only the newer one carries `LONGLONG`, which it gives to its basic
/// and full-feature channels alike — so it is a property of the device, not of the channel.
///
/// Gated rather than attempted. `DMASRCWDTH` and `DMADSTWDTH` are three bits wide on every device, so
/// the encoding lands in the register on a part that does not implement it, and nothing published says
/// what the transfer then moves.
///
/// A note here used to say the G350x had this and the C110x did not. **The G350x does not**: its
/// datasheet dashes the row, its feature list stops at 64 bits, its header defines no
/// `DMA_SYS_MMR_LLONG` and its SVD does not enumerate the encoding. The claim most likely came from
/// SLAU846 §5.2.3, which describes all five widths because that chapter documents both IP variants at
/// once, flagging the value as "not present in all devices" in a line that is easy to read past.
#[cfg(dma_long_long)]
impl SealedWord for u128 {
    fn width() -> vals::Wdth {
        vals::Wdth::Longlong
    }
}

#[cfg(dma_long_long)]
impl Word for u128 {
    fn size() -> isize {
        16
    }
}

/// What went wrong with a transfer.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The DMA transfer is too large.
    ///
    /// `SZ.SIZE` is sixteen bits, so a channel moves at most 65535 elements in one transfer. The
    /// width does not enter into it: 65535 `u8` and 65535 `u64` are both 65535 values to move.
    TooManyTransfers,

    /// The transfer would move nothing.
    ///
    /// An empty buffer, or a strided transfer whose buffer is shorter than one stride.
    NoTransfers,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::TooManyTransfers => write!(f, "too many transfers"),
            Error::NoTransfers => write!(f, "no transfers"),
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

    /// As [`Single`](Self::Single), but the transfer reloads and runs again instead of stopping.
    ///
    /// Full-feature channels only, and it never finishes -- see [`terminates`](Self::terminates).
    RepeatSingle,

    /// As [`Block`](Self::Block), but the transfer reloads and runs again instead of stopping.
    ///
    /// Full-feature channels only, and it never finishes -- see [`terminates`](Self::terminates).
    RepeatBlock,
}

impl TransferMode {
    /// Whether the hardware stops on its own once the count is exhausted.
    ///
    /// The repeating modes reload `SA`, `DA` and `SZ` and leave `DMAEN` set, so a [`Transfer`] over
    /// one is a future that never resolves. [`read`](Channel::read) and [`write`](Channel::write)
    /// refuse them for that reason, and the repeating constructors on [`FullChannel`] refuse the
    /// terminating ones.
    pub const fn terminates(self) -> bool {
        matches!(self, Self::Single | Self::Block)
    }
}

/// DMA transfer options.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    /// DMA transfer mode.
    pub mode: TransferMode,

    /// How far the source address moves between elements.
    #[cfg(dma_stride)]
    pub src_stride: Stride,

    /// How far the destination address moves between elements.
    #[cfg(dma_stride)]
    pub dst_stride: Stride,
}

impl TransferOptions {
    /// The reset-ish default, usable in a `const`.
    ///
    /// [`Default`] delegates here. A `const` is what makes the folding certain rather than
    /// dependent on this being inlined, which at `opt-level = "z"` has already failed once on a
    /// struct this size.
    pub const fn new() -> Self {
        Self {
            mode: TransferMode::Single,
            #[cfg(dma_stride)]
            src_stride: Stride::One,
            #[cfg(dma_stride)]
            dst_stride: Stride::One,
        }
    }
}

/// How far an address advances between elements, in elements.
///
/// The step is in units of the transfer width, so [`Two`](Self::Two) on a `u32` transfer moves eight
/// bytes. Reading every third sample out of an interleaved buffer is what this is for.
///
/// Only the newer DMA implements this; the older one has no encoding for it, which is why this type
/// does not exist on those devices rather than being accepted and ignored. Whether the address
/// advances at all is separate — a transfer that does not increment ignores this.
#[cfg(dma_stride)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Stride {
    /// One element, the ordinary contiguous case.
    One,
    /// Every second element.
    Two,
    /// Every third element.
    Three,
    /// Every fourth element.
    Four,
    /// Every fifth element.
    Five,
    /// Every sixth element.
    Six,
    /// Every seventh element.
    Seven,
    /// Every eighth element.
    Eight,
    /// Every ninth element.
    Nine,
}

#[cfg(dma_stride)]
impl Stride {
    /// The `DMASRCINCR`/`DMADSTINCR` encoding for an incrementing transfer with this step.
    const fn to_incr(self) -> Incr {
        match self {
            Self::One => Incr::Increment,
            Self::Two => Incr::Stride2,
            Self::Three => Incr::Stride3,
            Self::Four => Incr::Stride4,
            Self::Five => Incr::Stride5,
            Self::Six => Incr::Stride6,
            Self::Seven => Incr::Stride7,
            Self::Eight => Incr::Stride8,
            Self::Nine => Incr::Stride9,
        }
    }

    /// How many elements of span one transferred element costs.
    pub(crate) const fn step(self) -> usize {
        match self {
            Self::One => 1,
            Self::Two => 2,
            Self::Three => 3,
            Self::Four => 4,
            Self::Five => 5,
            Self::Six => 6,
            Self::Seven => 7,
            Self::Eight => 8,
            Self::Nine => 9,
        }
    }
}

impl Default for TransferOptions {
    #[inline]
    fn default() -> Self {
        Self::new()
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
    ///
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

/// Elements moved through a buffer of `len` when each one advances the address by `step`.
///
/// The hardware counts *transfers*, not addresses, so a strided transfer of `n` elements covers
/// `n * step` of the buffer. Deriving the count from the span is what keeps the slice the caller
/// passed a bound on what the DMA touches; taking the count from `len` directly would write
/// `step - 1` elements past the end of every one of them.
#[cfg(dma_stride)]
const fn strided_count(len: usize, step: usize) -> usize {
    len / step
}

fn verify_transfer(count: usize) -> Result<(), Error> {
    // `SZ.SIZE` counts down and clears `EN` when it reaches zero, so a transfer programmed at zero has
    // nothing to decrement and the TRM says no transfers occur. A strided transfer is the way in that
    // is not obviously a caller mistake: the count is the buffer's span over the stride, so asking for
    // every sixth word of a five-word buffer yields zero.
    if count == 0 {
        return Err(Error::NoTransfers);
    }

    if count > (u16::MAX as usize) {
        return Err(Error::TooManyTransfers);
    }

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
        TransferMode::RepeatSingle => vals::Tm::Rptsngl,
        TransferMode::RepeatBlock => vals::Tm::Rptblck,
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

/// Program the channel arbitration policy.
///
/// Nothing here arms an interrupt: the error events and the NVIC line are [`Channel::new`]'s business,
/// because that is where a binding proves a handler exists. `init` runs in every binary, including the
/// ones that never build a channel.
///
/// Changing the burst size mid transfer may have some odd behavior, so this expects to run once, before
/// any channel exists.
pub(crate) fn init(_cs: CriticalSection, burst_size: BurstSize, round_robin: bool) {
    // Reset leaves fixed priority and an uninterrupted block transfer, which is what `Config`
    // defaults to, so a program that leaves it there has nothing to program. Folds away entirely
    // when the config is a constant.
    if !matches!(burst_size, BurstSize::Complete) || round_robin {
        pac::DMA.prio().modify(|prio| {
            prio.set_burstsz(convert_burst_size(burst_size));
            prio.set_roundrobin(round_robin);
        });
    }
}

/// Arm the transfer error events and let the NVIC line through.
///
/// Called from [`Channel::new`], which takes a [`Binding`](interrupt::typelevel::Binding) and so cannot
/// be reached without a handler behind the line. Doing it in `crate::init` instead unmasked the
/// interrupt in every binary, with `DefaultHandler` behind it wherever nothing bound one — an
/// unreachable state, since no transfer can raise an error before a channel exists, but the one place
/// this HAL armed a source it could not prove was handled.
///
/// Idempotent: a second channel repeats both writes, which is why the first is a read-modify-write
/// under a critical section rather than a whole-register store. [`on_irq`] edits the same register.
fn arm_error_events() {
    critical_section::with(|_cs| {
        pac::DMA.int_event(0).imask().modify(|w| {
            w.set_dataerr(true);
            w.set_addrerr(true);
        });
    });

    // SAFETY: the caller holds a binding for this interrupt, so a handler is linked.
    unsafe { interrupt::DMA.enable() };
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
    // One argument per descriptor field. Bundling them into a struct would only move the same values
    // behind a by-value parameter that has measured worse on this core.
    #[allow(clippy::too_many_arguments)]
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
                #[cfg(dma_stride)]
                {
                    options.src_stride.to_incr()
                }
                #[cfg(not(dma_stride))]
                {
                    Incr::Increment
                }
            } else {
                Incr::Unchanged
            });
            w.set_dstincr(if increment_dst {
                #[cfg(dma_stride)]
                {
                    options.dst_stride.to_incr()
                }
                #[cfg(not(dma_stride))]
                {
                    Incr::Increment
                }
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
