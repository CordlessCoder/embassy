//! Inter-Integrated Circuit (I2C).
//!
//! [`I2c`] is the controller and [`I2cTarget`](crate::i2c_target::I2cTarget) the target. Both come in
//! a blocking and an [`Async`] flavour, chosen by which constructor is used; the asynchronous one
//! needs an interrupt binding and parks the core rather than spinning, which is what lets the device
//! sleep between bytes.
//!
//! A transfer is **one burst**, up to 4095 bytes, with the FIFO fed through it rather than being the
//! unit that moves. The controller stretches SCL while the transmit FIFO is empty or the receive FIFO
//! full, so a late refill costs bus time rather than bytes.
//!
//! # Cancelling an asynchronous transfer
//!
//! Dropping the future is safe and needs nothing from the caller. It cannot finish the transfer on
//! the way out — a STOP on its own is illegal until the transaction ends — so it masks the interrupts
//! and marks the peripheral dirty, and **the next transfer pays**: it resets the controller first,
//! which is the only thing that clears `BUSBSY`. A target still holding SDA down survives that and is
//! reported as [`Error::BusStuck`], which [`I2c::recover_stuck_bus`] will try to clear.
//!
//! # A bus that cannot clock
//!
//! A target holding **SDA** low is caught before anything waits on it, and comes back as
//! [`Error::BusStuck`] from either path.
//!
//! A bus held low on **SCL** is the other failure, and it is the one nothing detects: a controller
//! stretching legitimately holds SCL low too, so the two are indistinguishable from a register. Nothing
//! bounds a transfer against it except [`Config::clock_low_timeout_us`], which is `None` by default —
//! so **a transfer onto such a bus never returns**, blocking or asynchronous.
//!
//! That default is deliberate. Only the caller knows how long its slowest target may legitimately hold
//! SCL, and a timeout picked here would fail those buses instead. Set one on any bus whose targets are
//! not trusted to keep clocking.

#![macro_use]

use core::future::{self, Future};
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use embassy_embedded_hal::SetConfig;
use embassy_hal_internal::PeripheralType;
use embassy_hal_internal::drop::OnDrop;
use mspm0_metapac::i2c;

use crate::Peri;
use crate::gpio::{AnyPin, PfType, Pull, SealedPin};
use crate::interrupt::typelevel::Binding;
use crate::interrupt::{Interrupt, InterruptExt};
use crate::mode::{Async, Blocking, Mode};
use crate::pac::i2c::{I2c as Regs, vals};
use crate::pac::{self};
use crate::sync::irq_waker::IrqWaker;
use crate::sysctl::{SleepInfo, SleepLevel, WakeGuard};

/// The clock source for the I2C.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSel {
    /// Use the bus clock.
    ///
    /// Configurable clock.
    BusClk,

    /// Use the middle frequency clock.
    ///
    /// MFCLK runs at 4 MHz.
    MfClk,
}

impl ClockSel {
    /// Rate this source feeds the peripheral at, before the peripheral's own divider.
    ///
    /// Takes the tree rather than reading it, so this stays a `const fn` and [`Timing::solve`] can be
    /// evaluated at compile time. `BusClk` is ULPCLK rather than MCLK because every I2C instance is in
    /// PD0 — asserted per instance in `impl_i2c_instance!`, so this does not have to ask for the domain.
    pub const fn frequency(self, clocks: &crate::sysctl::Clocks) -> u32 {
        match self {
            // MFCLK is held at 4 MHz by SYSCTL whatever SYSOSC is doing, and reads as 0 when it was
            // never enabled, in which case the peripheral would not be clocked at all.
            Self::MfClk => clocks.mfclk,
            Self::BusClk => clocks.ulpclk,
        }
    }
}

/// The clock divider for the I2C.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockDiv {
    DivBy1,
    DivBy2,
    DivBy3,
    DivBy4,
    DivBy5,
    DivBy6,
    DivBy7,
    DivBy8,
}

impl ClockDiv {
    pub(crate) fn into(self) -> vals::Ratio {
        match self {
            Self::DivBy1 => vals::Ratio::DivBy1,
            Self::DivBy2 => vals::Ratio::DivBy2,
            Self::DivBy3 => vals::Ratio::DivBy3,
            Self::DivBy4 => vals::Ratio::DivBy4,
            Self::DivBy5 => vals::Ratio::DivBy5,
            Self::DivBy6 => vals::Ratio::DivBy6,
            Self::DivBy7 => vals::Ratio::DivBy7,
            Self::DivBy8 => vals::Ratio::DivBy8,
        }
    }

    const fn divider(self) -> u32 {
        match self {
            Self::DivBy1 => 1,
            Self::DivBy2 => 2,
            Self::DivBy3 => 3,
            Self::DivBy4 => 4,
            Self::DivBy5 => 5,
            Self::DivBy6 => 6,
            Self::DivBy7 => 7,
            Self::DivBy8 => 8,
        }
    }
}

/// The I2C mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusSpeed {
    /// Standard mode.
    ///
    /// The Standard mode runs at 100 kHz.
    Standard,

    /// Fast mode.
    ///
    /// The fast mode runs at 400 kHz.
    FastMode,

    /// Fast mode plus.
    ///
    /// The fast mode plus runs at 1 MHz.
    FastModePlus,

    /// Custom mode.
    ///
    /// The custom mode frequency (in Hz) can be set manually.
    Custom(u32),
}

impl BusSpeed {
    fn hertz(self) -> u32 {
        match self {
            Self::Standard => 100_000,
            Self::FastMode => 400_000,
            Self::FastModePlus => 1_000_000,
            Self::Custom(s) => s,
        }
    }
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Why an [`I2c`] could not be built or reconfigured.
pub enum ConfigError {
    /// Invalid clock rate.
    ///
    /// The clock rate could not be configured with the given configuration.
    InvalidClockRate,

    /// Clock source not enabled.
    ///
    /// The clock source is not enabled in SYSCTL.
    ClockSourceNotEnabled,

    /// Invalid target address.
    ///
    /// The address does not fit the addressing mode it was given in.
    InvalidTargetAddress,

    /// A second target address was asked for alongside a 10-bit primary address.
    ///
    /// `OAR2` is only compared while the target is in 7-bit mode, so the second address would never
    /// match. Measured, not just implied by SLAU846's "OAR2 supports only 7-bit addressing mode": with a
    /// 10-bit own address the target answers that and nothing else.
    SecondAddressWith10Bit,

    /// [`Config::clock_low_timeout_us`] is outside what the counter can represent.
    InvalidClockLowTimeout,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// How an [`I2c`] drives the bus.
pub struct Config {
    /// I2C clock source.
    pub(crate) clock_source: ClockSel,

    /// I2C clock divider.
    pub clock_div: ClockDiv,

    /// A timing solved ahead of time, skipping the divisions on the device.
    ///
    /// Build one with [`Timing::solve`] in a `const`. When set, [`Self::bus_speed`],
    /// [`Self::clock_div`] and the clock source all come from it rather than being picked from the
    /// bus speed.
    pub timing: Option<Timing>,

    /// If true: invert SDA pin signal values (V<sub>DD</sub> = 0/mark, Gnd = 1/idle).
    pub invert_sda: bool,

    /// If true: invert SCL pin signal values (V<sub>DD</sub> = 0/mark, Gnd = 1/idle).
    pub invert_scl: bool,

    /// Set the pull configuration for the SDA pin.
    pub sda_pull: Pull,

    /// Set the pull configuration for the SCL pin.
    pub scl_pull: Pull,

    /// Speed of the I2C bus.
    pub bus_speed: BusSpeed,

    /// Fail a transfer once SCL has been held low this many microseconds, or `None` to wait forever.
    ///
    /// A transfer that hits it fails with [`Error::Timeout`]. The peripheral does the counting, so it
    /// bounds a blocking transfer too. `None` by default, because a bus whose targets legitimately
    /// stretch for longer would start failing — pick it from the slowest target, not from the bus speed.
    ///
    /// **`None` means a transfer onto a bus that cannot clock never returns**, on either path. Nothing
    /// else can bound one: a target holding SCL down is indistinguishable from one stretching.
    ///
    /// Representable only in steps of `8320 / clock_hz` seconds, 2 to 255 of them: **520 µs to 66 ms
    /// from a 32 MHz functional clock, 4.2 ms to 530 ms from MFCLK**. Outside that range is a
    /// [`ConfigError::InvalidClockLowTimeout`]; inside it, the value rounds up.
    pub clock_low_timeout_us: Option<u32>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_source: ClockSel::MfClk,
            clock_div: ClockDiv::DivBy1,
            timing: None,
            invert_sda: false,
            invert_scl: false,
            sda_pull: Pull::None,
            scl_pull: Pull::None,
            bus_speed: BusSpeed::Standard,
            clock_low_timeout_us: None,
        }
    }
}

/// A solved I2C timing, so the device never has to divide.
///
/// [`Timing::solve`] is a `const fn`, so a bus speed known up front costs no division on the device:
///
/// ```no_run
/// # #![no_std]
/// # #[panic_handler]
/// # fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
/// use embassy_mspm0::i2c::{ClockDiv, ClockSel, Config, Timing};
/// use embassy_mspm0::sysctl::clock;
///
/// # fn main() {
/// const CLOCK: clock::ClockSetup = clock::Config::new().build();
/// const TIMING: Timing = match Timing::solve(&CLOCK.clocks(), ClockSel::MfClk, ClockDiv::DivBy1, 100_000) {
///     Some(timing) => timing,
///     None => core::panic!("100 kHz is not reachable from MFCLK"),
/// };
///
/// let config = Config::default().with_timing(TIMING);
/// # }
/// ```
///
/// A period only means anything against the clock it was solved for, so the source is part of the
/// solution and [`Config::with_timing`] programs both.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timing {
    clock_source: ClockSel,
    clock_div: ClockDiv,
    tpr: u8,
    clock_hz: u32,
    half_period_cycles: u16,
    settle_cycles: u16,
}

impl Timing {
    /// Solve the timer period for `bus_speed_hz` from `clock_source` on the tree `clocks` describes.
    ///
    /// Takes the tree rather than reading it, so this stays a `const fn`: pass
    /// [`clock::ClockSetup::clocks`](crate::sysctl::clock::ClockSetup::clocks) for the tree
    /// [`crate::init`] is being given, which is the one the peripheral will run on. A timing solved
    /// against a tree the device does not end up running puts the bus at the wrong speed, and the delays
    /// the driver paces bus recovery with — which come from MCLK, not from the I2C's own source — out by
    /// the same ratio.
    ///
    /// Returns [`None`] if the resulting `TPR` is outside the 1..=127 the register holds, or if the
    /// source is not at least 20x the bus speed, which is the same headroom the runtime path checks.
    pub const fn solve(
        clocks: &crate::sysctl::Clocks,
        clock_source: ClockSel,
        clock_div: ClockDiv,
        bus_speed_hz: u32,
    ) -> Option<Self> {
        let i2c_clk = clock_source.frequency(clocks) / clock_div.divider();

        // Same 20x headroom [`Config::resolve`] requires at runtime.
        let Some(needed) = bus_speed_hz.checked_mul(20) else {
            return None;
        };
        if i2c_clk < needed {
            return None;
        }

        let Some(denominator) = bus_speed_hz.checked_mul(10) else {
            return None;
        };
        if denominator == 0 {
            return None;
        }

        let ticks = i2c_clk / denominator;
        if ticks == 0 || ticks > 128 {
            return None;
        }

        let tpr = (ticks - 1) as u8;

        Some(Self {
            clock_source,
            clock_div,
            tpr,
            clock_hz: i2c_clk,
            // Solved here rather than on the device for the same reason `tpr` is: these are the only
            // other divisions the setup path does, and leaving one of them behind links the software
            // divider anyway.
            half_period_cycles: half_period_cycles(clocks.mclk, i2c_clk, tpr),
            settle_cycles: settle_cycles(clocks.mclk, i2c_clk),
        })
    }

    /// The `TPR` value this programs.
    pub const fn tpr(&self) -> u8 {
        self.tpr
    }

    /// The rate the peripheral sees after its own divider.
    pub const fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    /// The clock this period was solved against.
    pub const fn clock_source(&self) -> ClockSel {
        self.clock_source
    }
}

impl Config {
    /// Use a timing solved ahead of time, skipping the divisions on the device.
    ///
    /// Sets the clock source as well: the timing carries the clock it was solved against, and a period
    /// programmed against a different one puts the bus at the wrong speed.
    pub const fn with_timing(mut self, timing: Timing) -> Self {
        self.clock_source = timing.clock_source;
        self.clock_div = timing.clock_div;
        self.timing = Some(timing);
        self
    }

    pub fn sda_pf(&self) -> PfType {
        PfType::input(self.sda_pull, self.invert_sda)
    }
    pub fn scl_pf(&self) -> PfType {
        PfType::input(self.scl_pull, self.invert_scl)
    }
    /// Derive everything the driver needs from this configuration, in one place.
    ///
    /// This is deliberately the *only* place the I2C setup path divides. Cortex-M0+ has no divide
    /// instruction, so each division site that survives optimization drags in a ~400 byte software
    /// divider; funnelling them here means a pre-solved [`Timing`] removes every one of them.
    pub(crate) fn resolve(&self) -> Result<Resolved, ConfigError> {
        crate::sysctl::with_clocks(|clocks| self.resolve_on(clocks))
    }

    /// [`Self::resolve`] against a tree already in hand.
    fn resolve_on(&self, clocks: &crate::sysctl::Clocks) -> Result<Resolved, ConfigError> {
        // A pre-solved timing already carries its clock source, the divider, the resulting rate and the
        // timer period, so nothing below needs computing.
        if let Some(timing) = self.timing {
            return Ok(Resolved {
                clock_source: timing.clock_source(),
                clock_div: timing.clock_div,
                clock_hz: timing.clock_hz(),
                source_hz: timing.clock_source().frequency(clocks),
                tpr: timing.tpr(),
                clock_low_timeout: match self.clock_low_timeout_us {
                    Some(us) => Some(solve_clock_low_timeout(us, timing.clock_hz())?),
                    None => None,
                },
                half_period_cycles: timing.half_period_cycles,
                settle_cycles: timing.settle_cycles,
            });
        }

        let divider = self.clock_div.divider();
        let bus_speed = self.bus_speed.hertz();

        // Pick the source from the bus speed: at or below 200 kHz MFCLK suffices, above it the bus
        // clock is needed.
        let clock_source = if bus_speed / divider > 200_000 {
            // TODO: check if BUSCLK enabled
            ClockSel::BusClk
        } else {
            if !pac::SYSCTL.mclkcfg().read().usemftick() {
                return Err(ConfigError::ClockSourceNotEnabled);
            }

            ClockSel::MfClk
        };

        let source_hz = clock_source.frequency(clocks);
        let clock_hz = source_hz / divider;

        // The source must be ~20x the bus speed.
        if clock_hz < (bus_speed / divider) * 20 {
            return Err(ConfigError::InvalidClockRate);
        }

        // Sets the timer period to bring the clock frequency to the selected I2C speed.
        // From the documentation: TPR = (I2C_CLK / (I2C_FREQ * (SCL_LP + SCL_HP))) - 1 where:
        // - I2C_FREQ is desired I2C frequency (= I2C_BASE_FREQ divided by I2C_DIV)
        // - TPR is the Timer Period register value (range of 1 to 127)
        // - SCL_LP is the SCL Low period (fixed at 6)
        // - SCL_HP is the SCL High period (fixed at 4)
        // - I2C_CLK is functional clock frequency
        let ticks = clock_hz / (bus_speed * 10);
        if ticks == 0 || ticks > 128 {
            return Err(ConfigError::InvalidClockRate);
        }

        let tpr = (ticks - 1) as u8;

        Ok(Resolved {
            clock_source,
            clock_div: self.clock_div,
            clock_hz,
            source_hz,
            tpr,
            clock_low_timeout: match self.clock_low_timeout_us {
                Some(us) => Some(solve_clock_low_timeout(us, clock_hz)?),
                None => None,
            },
            half_period_cycles: half_period_cycles(clocks.mclk, clock_hz, tpr),
            settle_cycles: settle_cycles(clocks.mclk, clock_hz),
        })
    }
}

/// Solve [`Config::clock_low_timeout_us`] into a `TIMEOUT_CTL.TCNTLA` load value.
///
/// One step of `TCNTLA` is 8320 functional clocks: a count is 520 clocks and `TCNTLA` holds the upper 8
/// bits of a 12-bit counter, so each unit is 16 counts. The 520 comes from the register description, not
/// from SLAU846's body, which gives `(1 + TPR) x 12` instead — measured on a G3507 at 4 MHz, `TCNTLA` of
/// 2, 8 and 32 timed out after 4302, 17120 and 66497 µs, or 134 µs per count against the register's 130 and
/// the body's 12.
///
/// Rounded up, so the timeout is never shorter than what was asked for.
///
/// The divisor is split into a power of two and an odd factor so that no 64-bit division is ever
/// emitted: `ceil(n / (a * b)) == ceil(ceil(n / a) / b)`, and once the product has been shifted down by
/// `a` a result inside the register's range is small enough that the second step is 32-bit. A 64-bit
/// divide would link the software divider, ~900 bytes, into every binary that constructs an [`I2c`]
/// whether or not it asks for a timeout.
fn solve_clock_low_timeout(timeout_us: u32, clock_hz: u32) -> Result<u8, ConfigError> {
    /// Functional clocks in one step of `TCNTLA`, times the microseconds in a second: `8320 * 1e6`,
    /// factored as `SHIFT`'s power of two times `ODD`.
    const SHIFT: u32 = 13;
    const ODD: u32 = 1_015_625;

    const _: () = core::assert!((1u64 << SHIFT) * ODD as u64 == 8320 * 1_000_000);

    let scaled = (timeout_us as u64 * clock_hz as u64).div_ceil(1 << SHIFT);

    // Reject out of range before narrowing, which is also what makes the narrowing sound: anything the
    // register can hold leaves `scaled` well inside `u32`.
    if scaled > 255 * ODD as u64 {
        return Err(ConfigError::InvalidClockLowTimeout);
    }

    // Divide by the odd factor by hand. It is a constant, but the core has no widening multiply, so the
    // compiler reaches for the general software divider — ~400 bytes, and on a pre-solved [`Timing`] this
    // is the only division left in the driver. The quotient fits in eight bits, so eight
    // compare-subtract steps settle it.
    let mut scaled = scaled as u32;
    let mut steps: u32 = 0;

    for bit in (0..8u32).rev() {
        let sub = ODD << bit;

        if scaled >= sub {
            scaled -= sub;
            steps |= 1 << bit;
        }
    }

    // Anything left over falls inside the next step, and the timeout is never to be shorter than asked.
    if scaled != 0 {
        steps += 1;
    }

    // Below 2 the counter does not run at all, which SLAU846 states outright.
    if steps < 2 {
        return Err(ConfigError::InvalidClockLowTimeout);
    }
    Ok(steps as u8)
}

/// CPU cycles in half an SCL period, which is what the bus-recovery delays are paced by.
///
/// `const` so [`Timing::solve`] can settle it at compile time; `Ord::max` is not, hence the long way
/// round on the two guards.
const fn half_period_cycles(mclk: u32, clock_hz: u32, tpr: u8) -> u16 {
    // `TPR` is solved as `clock_hz / (bus_speed * 10) - 1`, so this runs it backwards.
    let bus_speed = clock_hz / (10 * (tpr as u32 + 1));
    let bus_speed = if bus_speed == 0 { 1 } else { bus_speed };

    let cycles = mclk / (2 * bus_speed);
    // The count feeds `asm::delay`, and a `u16` covers every bus speed a device actually runs at — at
    // 10 kHz off an 80 MHz MCLK it is 4000. Saturating needs a sub-kHz bus off an MCLK above 51 MHz,
    // which only the `ticks <= 128` bound keeps reachable at all, and it shortens the delay rather
    // than losing it.
    if cycles == 0 {
        1
    } else if cycles > u16::MAX as u32 {
        u16::MAX
    } else {
        cycles as u16
    }
}

/// CPU cycles a freshly started transfer needs before `CSR` is valid. See [`I2c::settle_after_start`].
///
/// `I2C_ERR_13` makes it three functional clock cycles. Rounded up, and at least one cycle so a functional
/// clock faster than the CPU still waits.
///
/// Scaling with the clock is what makes this bite on MFCLK and not on the bus clock: at 4 MHz against a
/// 32 MHz CPU it is 24 cycles, where at 32 MHz it is 3 and the register read alone covers it.
const fn settle_cycles(mclk: u32, clock_hz: u32) -> u16 {
    let clock_hz = if clock_hz == 0 { 1 } else { clock_hz };

    let cycles = (3 * mclk).div_ceil(clock_hz);
    // The count feeds `asm::delay`, and a `u16` cannot be reached: the slowest functional clock a
    // driver can be given is MFCLK divided by eight, which against the fastest MCLK is 480 cycles.
    // The clamp is there for the guarded `clock_hz` above, not for any real pair.
    if cycles == 0 {
        1
    } else if cycles > u16::MAX as u32 {
        u16::MAX
    } else {
        cycles as u16
    }
}

/// SCL half-periods to give the controller to go idle before a FIFO flush.
///
/// A stop condition and the bus turnaround after it are why it is not idle the instant `master_stop`
/// returns. Four half-periods is twice what that needs, and short enough that an error path which hits the
/// bound is still an error path rather than a hang.
const IDLE_HALF_PERIODS: u32 = 4;

/// Most bytes one burst can carry, from the width of `CCTR.CBLEN`.
///
/// Nothing to do with the FIFO, which is a buffer the transfer is fed through rather than the unit it
/// moves in.
const MAX_TRANSFER_LEN: usize = 0xFFF;

/// Where a run of same-direction operations has got to.
///
/// `embedded-hal` merges consecutive operations of one type into a single stretch of bus traffic, so a
/// run is one burst fed from several buffers rather than one burst per buffer.
struct GroupCursor {
    /// Operation being moved, indexing the transaction's own slice.
    op: usize,
    /// Bytes of that operation already moved.
    pos: usize,
}

/// A run of consecutive operations that move in one direction.
struct Group {
    /// First operation of the run.
    start: usize,
    /// One past the last operation that carries any bytes.
    end: usize,
    /// Whether the run writes; it reads otherwise.
    write: bool,
    /// Bytes the whole run moves, which is the burst length.
    total: usize,
}

/// Bytes an operation carries.
fn op_len(op: &embedded_hal::i2c::Operation<'_>) -> usize {
    match op {
        embedded_hal::i2c::Operation::Read(buf) => buf.len(),
        embedded_hal::i2c::Operation::Write(buf) => buf.len(),
    }
}

/// Whether an operation writes.
fn op_is_write(op: &embedded_hal::i2c::Operation<'_>) -> bool {
    matches!(op, embedded_hal::i2c::Operation::Write(_))
}

/// The next run of same-direction operations at or after `from`, or [`None`] once there are none left.
///
/// **Operations carrying no bytes are skipped rather than addressed.** Nothing is gained by putting an
/// address on the bus to move nothing, the transfer paths refuse a zero-length buffer anyway, and an
/// empty operation between two writes must not break the run they form.
fn next_group(ops: &[embedded_hal::i2c::Operation<'_>], from: usize) -> Option<Group> {
    let mut start = from;
    while start < ops.len() && op_len(&ops[start]) == 0 {
        start += 1;
    }

    if start >= ops.len() {
        return None;
    }

    let write = op_is_write(&ops[start]);
    let mut total = op_len(&ops[start]);
    let mut end = start + 1;

    let mut at = start + 1;
    while at < ops.len() {
        if op_len(&ops[at]) == 0 {
            at += 1;
            continue;
        }

        if op_is_write(&ops[at]) != write {
            break;
        }

        total += op_len(&ops[at]);
        at += 1;
        end = at;
    }

    Some(Group {
        start,
        end,
        write,
        total,
    })
}

/// A [`Config`] with everything the driver needs derived from it.
///
/// The two cycle counts are here rather than recomputed where they are used because deriving either one
/// costs a division, and both are wanted on the transfer path.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct Resolved {
    pub clock_source: ClockSel,
    pub clock_div: ClockDiv,

    /// Rate the peripheral sees after its own divider.
    pub clock_hz: u32,

    /// Rate of the source before the divider, which is what deep-sleep survival depends on.
    pub source_hz: u32,

    /// The timer period register value.
    pub tpr: u8,

    /// Load value for the SCL-low timeout counter, or `None` to leave the counter off.
    pub clock_low_timeout: Option<u8>,

    /// CPU cycles in half an SCL period, the unit the bus-recovery delays are counted in.
    pub half_period_cycles: u16,

    /// CPU cycles to let a freshly started transfer settle. See [`I2c::settle_after_start`].
    pub settle_cycles: u16,
}

impl Resolved {
    /// Shallowest sleep level to block so this instance keeps working.
    pub(crate) fn wake_floor(&self, sleep: &SleepInfo) -> Option<SleepLevel> {
        // Undivided on purpose: the question is whether the source still runs at the rate the
        // peripheral was configured for, not what it was divided down to.
        sleep.floor_for_operation(self.source_hz)
    }
}

/// What an I2C transfer can fail with.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,

    /// Arbitration lost
    Arbitration,

    /// The bus is stuck with a target holding SDA low
    ///
    /// Nothing will complete until the line is released. Call [`I2c::recover_stuck_bus`] and retry.
    BusStuck,

    /// ACK not received, and the controller did not say to what
    Nack,

    /// The address was not acknowledged: nothing is answering on it
    NackAddress,

    /// A data byte was not acknowledged: the target is there but rejected the byte
    NackData,

    /// Timeout
    Timeout,

    /// Zero-length transfers are not allowed.
    ZeroLengthTransfer,

    /// Transfer length is over limit.
    ///
    /// A transfer is one burst, and `CCTR.CBLEN` counts it in 12 bits, so 4095 bytes is the most any
    /// one of them can carry.
    TransferLengthIsOverLimit,

    /// The address does not fit the addressing mode it was given in.
    InvalidAddress,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::Bus => "Bus Error",
            Self::BusStuck => "Bus Stuck, SDA Held Low",
            Self::NackAddress => "Address Not Acknowledged",
            Self::NackData => "Data Not Acknowledged",
            Self::Arbitration => "Arbitration Lost",
            Self::Nack => "ACK Not Received",
            Self::Timeout => "Request Timed Out",
            Self::ZeroLengthTransfer => "Zero-Length Transfers are not allowed",
            Self::TransferLengthIsOverLimit => "Transfer length is over limit",
            Self::InvalidAddress => "Address too large for its addressing mode",
        };

        write!(f, "{}", message)
    }
}

impl core::error::Error for Error {}

/// An I2C address, 7- or 10-bit.
///
/// A 10-bit address goes on the wire as a `11110xx` header byte followed by a second byte holding the
/// low eight bits. The peripheral sequences that itself, so the driver only selects the mode.
///
/// Which mode a bare integer means is decided by its width, following `embedded-hal`: a [`u8`] is a
/// 7-bit address and a [`u16`] a 10-bit one.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Address {
    /// A 7-bit address.
    SevenBit(u8),

    /// A 10-bit address, in `0..=0x3ff`.
    TenBit(u16),
}

impl From<u8> for Address {
    fn from(value: u8) -> Self {
        Address::SevenBit(value)
    }
}

impl From<u16> for Address {
    /// # Panics
    ///
    /// If the address does not fit in ten bits.
    fn from(value: u16) -> Self {
        assert!(value < 0x400, "Ten bit address must be less than 0x400");
        Address::TenBit(value)
    }
}

impl Address {
    /// The address itself, in either mode.
    pub fn addr(self) -> u16 {
        match self {
            Address::SevenBit(addr) => addr as u16,
            Address::TenBit(addr) => addr,
        }
    }

    /// Whether the address fits the mode it was given in.
    ///
    /// Both variants are wider than the address they carry, and both are constructible directly rather
    /// than through the `From` impls that check. Left unchecked the peripheral truncates to its field
    /// width, and answers to or addresses a different device.
    pub(crate) fn fits(self) -> bool {
        match self {
            Address::SevenBit(addr) => addr < 0x80,
            Address::TenBit(addr) => addr < 0x400,
        }
    }

    /// The address, if it [fits](Self::fits) the mode it was given in.
    pub(crate) fn checked(address: impl Into<Address>) -> Result<Address, Error> {
        let address = address.into();

        if address.fits() {
            Ok(address)
        } else {
            Err(Error::InvalidAddress)
        }
    }

    pub(crate) fn mode(self) -> vals::Mode {
        match self {
            Address::SevenBit(_) => vals::Mode::Mode7,
            Address::TenBit(_) => vals::Mode::Mode10,
        }
    }
}

/// I2C Driver.
pub struct I2c<'d, M: Mode> {
    info: &'static Info,
    state: &'static State,
    scl: Option<Peri<'d, AnyPin>>,
    sda: Option<Peri<'d, AnyPin>>,
    wake_floor: Option<SleepLevel>,
    /// What the peripheral is configured to, kept so [`I2c::reset_peripheral`] can restore it.
    resolved: Resolved,
    _phantom: PhantomData<M>,
}

impl<'d, M: Mode> SetConfig for I2c<'d, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(*config)
    }
}

impl<'d> I2c<'d, Blocking> {
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let resolved = config.resolve()?;

        Self::new_inner(peri, scl, sda, config, resolved)
    }
}

impl<'d> I2c<'d, Async> {
    pub fn new_async<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let resolved = config.resolve()?;

        let i2c = Self::new_inner(peri, scl, sda, config, resolved);

        T::info().interrupt.unpend();
        unsafe { T::info().interrupt.enable() };

        i2c
    }
}

impl<'d, M: Mode> I2c<'d, M> {
    /// Reconfigure the driver
    pub fn set_config(&mut self, config: Config) -> Result<(), ConfigError> {
        let resolved = config.resolve()?;

        // Kept so a later [`I2c::reset_peripheral`] restores this config rather than the one the driver was
        // built with.
        self.resolved = resolved;

        // Off across the reprogramming and back on afterwards, but only if it was on to begin with: this
        // method is shared by both modes, `new_async` is what enables the line, and leaving it disabled
        // strands every later async transfer — the transfer completes on the wire and nothing wakes the
        // task waiting on it.
        let was_enabled = self.info.interrupt.is_enabled();
        self.info.interrupt.disable();

        if let Some(ref sda) = self.sda {
            sda.update_pf(config.sda_pf());
        }

        if let Some(ref scl) = self.scl {
            scl.update_pf(config.scl_pf());
        }

        let configured = self.init();

        if was_enabled {
            self.info.interrupt.unpend();
            // SAFETY: re-arming a line this driver owns and had enabled a moment ago.
            unsafe { self.info.interrupt.enable() };
        }

        configured
    }

    fn init(&mut self) -> Result<(), ConfigError> {
        let resolved = self.resolved;

        self.info.regs.clksel().write(|w| match resolved.clock_source {
            ClockSel::BusClk => {
                w.set_mfclk_sel(false);
                w.set_busclk_sel(true);
            }
            ClockSel::MfClk => {
                w.set_mfclk_sel(true);
                w.set_busclk_sel(false);
            }
        });
        self.info
            .regs
            .clkdiv()
            .write(|w| w.set_ratio(resolved.clock_div.into()));

        self.info.regs.gfctl().modify(|w| {
            w.set_agfen(false);
            w.set_agfsel(vals::Agfsel::Aglit50);
            w.set_chain(true);
        });

        // Reset controller transfer, follow TI example
        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_burstrun(false);
            w.set_start(false);
            w.set_stop(false);
            w.set_ack(false);
            w.set_cackoen(false);
            w.set_rd_on_txempty(false);
            w.set_cblen(0);
        });

        self.wake_floor = resolved.wake_floor(&self.info.sleep);

        self.info.regs.controller(0).ctpr().write(|w| w.set_tpr(resolved.tpr));

        // SLAU846: the low timeout is to be configured at initialisation and not while active. Counter A
        // is the SCL-low one; B, which watches SCL high, is left alone.
        self.info.regs.timeout_ctl().modify(|w| {
            w.set_tcntaen(resolved.clock_low_timeout.is_some());
            w.set_tcntla(resolved.clock_low_timeout.unwrap_or_default());
        });

        self.info.regs.controller(0).cfifoctl().write(|w| {
            w.set_txtrig(vals::CfifoctlTxtrig::Empty);
            w.set_rxtrig(vals::CfifoctlRxtrig::Level1);
        });

        self.info.regs.controller(0).ccr().modify(|w| {
            w.set_clkstretch(true);
            w.set_active(true);
        });

        Ok(())
    }

    /// Wait for the controller to report itself idle, for a few SCL half-periods and no longer.
    ///
    /// Bounded rather than spun on, because `CSR` is not trustworthy in this window: after a timeout it
    /// reads `IDLE` and `BUSBSY` at once, permanently, which SLAU846 says cannot happen. A poll that can
    /// exit early on a wrong answer is a poll that can also never exit at all, and the second is worse.
    ///
    /// The answer is returned for callers that have something better to do with it than flush anyway.
    fn wait_for_idle(&self) -> bool {
        let ctrl = self.info.regs.controller(0);
        let half_period = self.resolved.half_period_cycles as u32;

        for _ in 0..IDLE_HALF_PERIODS {
            if ctrl.csr().read().idle() {
                return true;
            }

            cortex_m::asm::delay(half_period);
        }

        ctrl.csr().read().idle()
    }

    /// Discard whatever an abandoned transfer left queued, driverlib's `DL_I2C_flushController*FIFO`.
    ///
    /// A cancelled write leaves its unsent bytes in the TX FIFO and a cancelled read leaves what it
    /// received in the RX FIFO. Left there, the next transfer transmits the previous one's byte and reads
    /// back the previous one's data — an error reported against a transfer that succeeded, one
    /// transaction later.
    ///
    /// SLAU846 §25.2.3.13 asks for three things around a flush and this does all of them: the controller
    /// must be idle, the FIFO interrupts must be masked first, and their flags must be dealt with after —
    /// emptying the TX FIFO raises exactly the events a finished transfer would, and left latched they
    /// would be answered by the next transfer.
    fn flush_fifos(&mut self) {
        // Flushing under a live burst takes bytes out from under it, so idleness is worth asking for even
        // though the answer cannot be relied on.
        self.wait_for_idle();

        let ctrl = self.info.regs.controller(0);
        let int = self.info.regs.cpu_int(0);

        // Read back and restored one field at a time rather than saved and rewritten whole, so a change
        // to any other bit between here and the end of the flush survives it.
        let armed = int.imask().read();
        int.imask().modify(|w| {
            w.set_ctxfifotrg(false);
            w.set_crxfifotrg(false);
            w.set_ctxempty(false);
            w.set_crxfifofull(false);
        });

        ctrl.cfifoctl().modify(|w| {
            w.set_txflush(true);
            w.set_rxflush(true);
        });
        // Unbounded, unlike the idle poll above, and deliberately: this waits on the FIFO emptying itself
        // with the flush bits held, which is the peripheral's own doing and does not depend on the bus.
        while ctrl.cfifosr().read().txfifocnt() as usize != self.info.fifo_size
            || ctrl.cfifosr().read().rxfifocnt() != 0
        {}
        ctrl.cfifoctl().modify(|w| {
            w.set_txflush(false);
            w.set_rxflush(false);
        });

        int.iclr().write(|w| {
            w.set_ctxfifotrg(true);
            w.set_crxfifotrg(true);
            w.set_ctxempty(true);
            w.set_crxfifofull(true);
        });
        int.imask().modify(|w| {
            w.set_ctxfifotrg(armed.ctxfifotrg());
            w.set_crxfifotrg(armed.crxfifotrg());
            w.set_ctxempty(armed.ctxempty());
            w.set_crxfifofull(armed.crxfifofull());
        });
    }

    /// Reset the peripheral and put its configuration back.
    ///
    /// The escape hatch for a controller that cannot be talked round. It ends whatever burst was running
    /// at once, empties the FIFOs, releases SCL and SDA, and is the only thing that clears `BUSBSY` after a
    /// clock-low timeout — `IDLE` comes back set with `BUSBSY` still set, and SLAU846 gives the controller
    /// reset as the other way to clear it.
    ///
    /// Cheap: a few register writes and a 16-cycle settle, against the hundreds of milliseconds that
    /// waiting on a stuck bus costs.
    fn reset_peripheral(&mut self) {
        self.info.regs.gprcm(0).rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(vals::ResetKey::Key);
        });
        self.info.regs.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });
        cortex_m::asm::delay(16);

        // Re-derives `wake_floor` too. Infallible: the config was resolved once already, and nothing about
        // the clock tree can have changed since.
        let _ = self.init();
    }

    /// Is the bus stuck with a target holding SDA low?
    ///
    /// SDA low while SCL sits idle high. Another controller mid-transaction also holds SDA low, but it would
    /// be clocking SCL, so the line is watched across a few half-periods to tell the two apart. A controller
    /// that stretches SCL low indefinitely is indistinguishable from a busy bus and reads as not stuck.
    ///
    /// This is what [`Error::BusStuck`] reports and what [`I2c::recover_stuck_bus`] acts on, so the two
    /// cannot disagree about whether there is anything to do.
    pub fn bus_is_stuck(&self) -> bool {
        if self.info.regs.controller(0).cbmon().read().sda() {
            return false;
        }

        // Time is what separates a stuck target from a STOP still on the wire, which looks identical —
        // SDA low, SCL high — for up to a bit period after every NACK. Twenty half-periods is ten bit
        // times, and both questions are re-asked each pass, so the common case costs a bit period rather
        // than the whole window.
        let half = self.resolved.half_period_cycles as u32;
        for _ in 0..20 {
            cortex_m::asm::delay(half);

            let mon = self.info.regs.controller(0).cbmon().read();
            if mon.sda() {
                return false;
            }
            if !mon.scl() {
                return false;
            }
        }
        true
    }

    /// Clock a target off the bus when it is holding SDA low.
    ///
    /// Nine SCL pulses let a target that lost sync finish the byte it is stuck part-way through — eight
    /// bits and the ACK — after which a STOP leaves the bus idle.
    ///
    /// Does nothing when SDA is already high. `Err(Error::Bus)` means nine clocks did not free it, which is
    /// either a target holding SDA for good or a short to ground — neither recoverable from here.
    ///
    /// Only sound when nothing else is using the bus: it drives SCL without arbitration, so calling it while
    /// another controller is mid-transaction corrupts that transaction.
    pub fn recover_stuck_bus(&mut self) -> Result<(), Error> {
        if !self.bus_is_stuck() {
            return Ok(());
        }
        let half = self.resolved.half_period_cycles as u32;

        let (Some(scl), Some(sda)) = (self.scl.as_ref(), self.sda.as_ref()) else {
            return Err(Error::Bus);
        };

        // Read back rather than remembered: the pin is type-erased by the time it is stored here, so its
        // peripheral function number is not otherwise recoverable.
        let scl_pf = pac::IOMUX.pincm(scl._pin_cm() as usize).read().pf();
        let sda_pf = pac::IOMUX.pincm(sda._pin_cm() as usize).read().pf();

        // `hiz1` is already set on both from `new_inner` and nothing here clears it, so a GPIO output is
        // open-drain: low is driven, high is released for the pull-up to take.
        let released = PfType::input(Pull::None, false);
        for pin in [scl, sda] {
            pin.set_as_pf(crate::gpio::GPIO_PF, released);
            pin.block().doutset31_0().write(|w| w.set_dio(pin.bit_index(), true));
            pin.block().doeset31_0().write(|w| w.set_dio(pin.bit_index(), true));
        }

        let sda_high = || sda.block().din31_0().read().dio(sda.bit_index());

        // All nine, without breaking at the first high sample: SDA goes high on any `1` bit of the byte the
        // target is still shifting out, so breaking there leaves it mid-byte and free to pull the line back
        // down before the STOP lands.
        for _ in 0..9 {
            scl.block().doutclr31_0().write(|w| w.set_dio(scl.bit_index(), true));
            cortex_m::asm::delay(half);
            scl.block().doutset31_0().write(|w| w.set_dio(scl.bit_index(), true));
            cortex_m::asm::delay(half);
        }
        let freed = sda_high();

        // STOP is SDA rising while SCL is high, so both have to be driven low first to set it up.
        scl.block().doutclr31_0().write(|w| w.set_dio(scl.bit_index(), true));
        sda.block().doutclr31_0().write(|w| w.set_dio(sda.bit_index(), true));
        cortex_m::asm::delay(half);
        scl.block().doutset31_0().write(|w| w.set_dio(scl.bit_index(), true));
        cortex_m::asm::delay(half);
        sda.block().doutset31_0().write(|w| w.set_dio(sda.bit_index(), true));
        cortex_m::asm::delay(half);

        scl.set_as_pf(scl_pf, released);
        sda.set_as_pf(sda_pf, released);

        // The controller watched none of that, so its idea of the bus is stale.
        self.reset_peripheral();

        if freed {
            debug!("i2c: bus recovery freed SDA");
            Ok(())
        } else {
            warn!("i2c: bus recovery clocked 9 times and SDA is still low");
            Err(Error::Bus)
        }
    }

    /// Put the peripheral back in a state the next transfer can use, after `err` ended this one.
    ///
    /// A timeout is the one failure a STOP cannot clear, so it takes the reset. Anything else only needs
    /// the bus released and the queued bytes dropped, which is what SLAU846 asks for: "if a timeout is
    /// detected before the end of a transfer, software should flush the FIFO before initializing the next
    /// transfer".
    fn recover_after(&mut self, err: Error) {
        if err == Error::Timeout {
            self.reset_peripheral();
        } else {
            self.master_stop();
            self.flush_fifos();
        }
    }

    fn master_stop(&mut self) {
        // not the first transaction, delay 1000 cycles
        cortex_m::asm::delay(1000);

        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_cblen(0);
            w.set_stop(true);
            w.set_start(false);
        });
    }

    fn master_read(&mut self, address: Address, length: usize, restart: bool, send_ack_nack: bool, send_stop: bool) {
        if restart {
            // not the first transaction, delay 1000 cycles
            cortex_m::asm::delay(1000);
        }

        // START may be set even while the bus is busy or the peripheral is in target mode.
        self.info.regs.controller(0).csa().modify(|w| {
            w.set_taddr(address.addr());
            w.set_cmode(address.mode());
            w.set_dir(vals::Dir::Receive);
        });

        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_cblen(length as u16);
            w.set_burstrun(true);
            w.set_ack(send_ack_nack);
            w.set_start(true);
            w.set_stop(send_stop);
        });
    }

    fn master_write(&mut self, address: Address, length: usize, send_stop: bool) {
        self.info.regs.controller(0).csa().modify(|w| {
            w.set_taddr(address.addr());
            w.set_cmode(address.mode());
            w.set_dir(vals::Dir::Transmit);
        });
        self.info.regs.controller(0).cctr().modify(|w| {
            w.set_cblen(length as u16);
            w.set_burstrun(true);
            w.set_start(true);
            w.set_stop(send_stop);
        });
    }

    /// Wait out `I2C_ERR_13` before reading `CSR` after starting a transfer.
    ///
    /// Polling `BUSY` any sooner reads it before the controller has raised it, so the wait falls straight
    /// through and the caller checks for errors against a transfer that has not happened yet. A NACK then
    /// goes unnoticed and the transfer is reported as a success.
    fn settle_after_start(&self) {
        cortex_m::asm::delay(self.resolved.settle_cycles as u32);
    }

    /// Wait for whoever holds the bus to release it, giving up on the SCL-low timeout.
    ///
    /// A bus stuck on SDA is reported as [`Error::BusStuck`] rather than waited on, since no amount of
    /// waiting fixes it. Otherwise only a bus held *low* can time out, because counter A watches SCL low: a
    /// bus left marked busy with SCL high still waits forever, which is what counter B would be for.
    fn blocking_wait_bus_free(&mut self) -> Result<(), Error> {
        if self.bus_is_stuck() {
            return Err(Error::BusStuck);
        }

        self.clear_timeout();
        while self.info.regs.controller(0).csr().read().busbsy() {
            if self.timed_out() {
                self.clear_timeout();
                self.reset_peripheral();
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }

    /// Has the SCL-low timeout fired? Always false unless [`Config::clock_low_timeout_us`] enabled it.
    fn timed_out(&self) -> bool {
        self.info.regs.cpu_int(0).ris().read().timeouta()
    }

    /// Forget any timeout left over from an earlier transfer, so it is not blamed on the next one.
    fn clear_timeout(&self) {
        self.info.regs.cpu_int(0).iclr().write(|w| w.set_timeouta(true));
    }

    /// Turn whatever the controller latched into an error for the caller.
    ///
    /// Ordered by how fundamental the failure is. A timeout means the bus never gave the transfer a
    /// chance, so it outranks a NACK that may just be the tail of it.
    fn check_error(&self) -> Result<(), Error> {
        if self.timed_out() {
            self.clear_timeout();
            return Err(Error::Timeout);
        }

        let csr = self.info.regs.controller(0).csr().read();
        if csr.arblst() {
            return Err(Error::Arbitration);
        }
        if csr.err() {
            return Err(self.nack_kind());
        }
        Ok(())
    }

    /// Push what fits into the transmit FIFO, returning how many bytes went in.
    ///
    /// `TXFIFOCNT` counts the space left, not what is queued.
    fn fill_tx(&self, bytes: &[u8]) -> usize {
        let ctrl = self.info.regs.controller(0);
        let mut sent = 0;

        while sent < bytes.len() && ctrl.cfifosr().read().txfifocnt() != 0 {
            ctrl.ctxdata().write(|w| w.set_value(bytes[sent]));
            sent += 1;
        }

        sent
    }

    /// Take what the receive FIFO holds, returning how many bytes came out.
    fn drain_rx(&self, into: &mut [u8]) -> usize {
        let ctrl = self.info.regs.controller(0);
        let mut got = 0;

        while got < into.len() && ctrl.cfifosr().read().rxfifocnt() != 0 {
            into[got] = ctrl.crxdata().read().value();
            got += 1;
        }

        got
    }

    /// Push what fits into the transmit FIFO from a run of write operations, returning how many went in.
    ///
    /// The run moves as one burst, so the FIFO is fed from each operation's buffer in turn.
    fn fill_tx_group(&self, ops: &[embedded_hal::i2c::Operation<'_>], end: usize, cur: &mut GroupCursor) -> usize {
        let ctrl = self.info.regs.controller(0);
        let mut sent = 0;

        while cur.op < end {
            let embedded_hal::i2c::Operation::Write(buf) = &ops[cur.op] else {
                break;
            };

            if cur.pos == buf.len() {
                cur.op += 1;
                cur.pos = 0;
                continue;
            }

            if ctrl.cfifosr().read().txfifocnt() == 0 {
                break;
            }

            ctrl.ctxdata().write(|w| w.set_value(buf[cur.pos]));
            cur.pos += 1;
            sent += 1;
        }

        sent
    }

    /// Take what the receive FIFO holds into a run of read operations, returning how many came out.
    fn drain_rx_group(&self, ops: &mut [embedded_hal::i2c::Operation<'_>], end: usize, cur: &mut GroupCursor) -> usize {
        let ctrl = self.info.regs.controller(0);
        let mut got = 0;

        while cur.op < end {
            let embedded_hal::i2c::Operation::Read(buf) = &mut ops[cur.op] else {
                break;
            };

            if cur.pos == buf.len() {
                cur.op += 1;
                cur.pos = 0;
                continue;
            }

            if ctrl.cfifosr().read().rxfifocnt() == 0 {
                break;
            }

            buf[cur.pos] = ctrl.crxdata().read().value();
            cur.pos += 1;
            got += 1;
        }

        got
    }

    /// Which half of the transfer went unanswered.
    ///
    /// `ADRACK` and `DATACK` are the difference between nothing being at that address and the target being
    /// there but rejecting a byte. The async paths need this separately because they learn about a NACK from
    /// the interrupt rather than from [`I2c::check_error`], and would otherwise report the same failure less
    /// precisely than the blocking ones.
    fn nack_kind(&self) -> Error {
        let csr = self.info.regs.controller(0).csr().read();
        match (csr.adrack(), csr.datack()) {
            (true, _) => Error::NackAddress,
            (false, true) => Error::NackData,
            (false, false) => Error::Nack,
        }
    }
}

impl<'d> I2c<'d, Blocking> {
    /// Arm a receive burst for `length` bytes and return once the address phase has settled.
    ///
    /// The caller drains the FIFO as the bytes arrive; this does not wait for the burst to finish.
    fn master_blocking_read(
        &mut self,
        address: Address,
        length: usize,
        restart: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        // unless restart, Wait for the controller to be idle,
        if !restart {
            while !self.info.regs.controller(0).csr().read().idle() && !self.timed_out() {}
        }

        // The burst covers the whole transfer, so its last byte is the transfer's last byte and must be
        // NACKed to release the target.
        self.master_read(address, length, restart, false, send_stop);

        self.settle_after_start();

        Ok(())
    }

    /// Arm a transmit burst for `length` bytes and return once the address phase has settled.
    ///
    /// The caller keeps the FIFO fed; this does not wait for the burst to finish.
    ///
    /// `restart` says this continues a transaction rather than opening one. Waiting for idle then would
    /// wait for something that cannot happen: no STOP has been sent, so the controller is still busy by
    /// design, and with no clock-low timeout configured the wait has nothing to end it.
    fn master_blocking_write(
        &mut self,
        address: Address,
        length: usize,
        restart: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        if !restart {
            while !self.info.regs.controller(0).csr().read().idle() && !self.timed_out() {}
        }

        self.master_write(address, length, send_stop);

        self.settle_after_start();

        Ok(())
    }

    fn read_blocking_internal(
        &mut self,
        address: Address,
        read: &mut [u8],
        restart: bool,
        end_w_stop: bool,
    ) -> Result<(), Error> {
        self.clear_timeout();
        if read.is_empty() {
            return Err(Error::ZeroLengthTransfer);
        }
        if read.len() > MAX_TRANSFER_LEN {
            return Err(Error::TransferLengthIsOverLimit);
        }

        self.master_blocking_read(address, read.len(), restart, end_w_stop)?;

        // One burst for the whole transfer, drained as it arrives. The controller stretches SCL while
        // the FIFO is full (SLAU846 25.2.3.8), so falling behind costs bus time rather than bytes.
        let mut got = 0;
        while got < read.len() {
            if let Err(err) = self.check_error() {
                self.recover_after(err);
                return Err(err);
            }

            got += self.drain_rx(&mut read[got..]);

            // Nothing left to come and nothing left to take: the burst ended early without setting a
            // status bit to say why.
            if got < read.len()
                && !self.info.regs.controller(0).csr().read().busy()
                && self.info.regs.controller(0).cfifosr().read().rxfifocnt() == 0
            {
                self.recover_after(Error::Bus);
                return Err(Error::Bus);
            }
        }

        Ok(())
    }

    fn write_blocking_internal(&mut self, address: Address, write: &[u8], end_w_stop: bool) -> Result<(), Error> {
        self.clear_timeout();
        if write.is_empty() {
            return Err(Error::ZeroLengthTransfer);
        }
        if write.len() > MAX_TRANSFER_LEN {
            return Err(Error::TransferLengthIsOverLimit);
        }

        // Prime the FIFO before arming, the order TI's own examples use, then keep it fed. The
        // controller stretches SCL while the FIFO is empty (SLAU846 25.2.3.8), so falling behind costs
        // bus time rather than bytes.
        let mut sent = self.fill_tx(write);

        self.master_blocking_write(address, write.len(), false, end_w_stop)?;

        while sent < write.len() {
            if let Err(err) = self.check_error() {
                self.recover_after(err);
                return Err(err);
            }

            // The burst stopped with bytes still to hand over, and no status bit says why.
            if !self.info.regs.controller(0).csr().read().busy() {
                self.recover_after(Error::Bus);
                return Err(Error::Bus);
            }

            sent += self.fill_tx(&write[sent..]);
        }

        // The last bytes are queued but not yet on the wire.
        while self.info.regs.controller(0).csr().read().busy() && !self.timed_out() {}

        if let Err(err) = self.check_error() {
            self.recover_after(err);
            return Err(err);
        }

        Ok(())
    }

    // =========================
    //  Blocking public API

    /// Blocking read.
    ///
    /// `read` may hold between one and 4095 bytes.
    pub fn blocking_read(&mut self, address: impl Into<Address>, read: &mut [u8]) -> Result<(), Error> {
        let address = Address::checked(address)?;
        self.blocking_wait_bus_free()?;
        self.read_blocking_internal(address, read, false, true)
    }

    /// Blocking write.
    ///
    /// `write` may hold between one and 4095 bytes.
    pub fn blocking_write(&mut self, address: impl Into<Address>, write: &[u8]) -> Result<(), Error> {
        let address = Address::checked(address)?;
        self.blocking_wait_bus_free()?;
        self.write_blocking_internal(address, write, true)
    }

    /// Blocking write, restart, read.
    ///
    /// Each buffer may hold between one and 4095 bytes.    ///
    /// **A 10-bit address is re-sent before the read.** The hardware puts the addressing header on the
    /// bus a second time after the repeated START, so a target sees this as indistinguishable from a
    /// plain 10-bit read. Measured, and TI documents neither.
    pub fn blocking_write_read(
        &mut self,
        address: impl Into<Address>,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Error> {
        let address = Address::checked(address)?;
        self.blocking_wait_bus_free()?;
        let err = self.write_blocking_internal(address, write, false);
        if err != Ok(()) {
            return err;
        }
        self.read_blocking_internal(address, read, true, true)
    }
}

impl<'d> I2c<'d, Async> {
    /// Run an armed burst to completion, disarming every interrupt once it ends.
    ///
    /// The three faults a controller reports are the same whichever direction the burst runs, so only
    /// the FIFO-trigger and burst-done statuses reach `step`, which returns `Pending` for anything it
    /// does not recognise.
    fn run_burst(
        &mut self,
        mut step: impl FnMut(&Self, vals::CpuIntIidxStat) -> Poll<Result<(), Error>>,
    ) -> impl Future<Output = Result<(), Error>> {
        future::poll_fn(move |cx| {
            // Register prior to checking the condition
            self.state.waker.register(cx.waker());

            let result = match self.info.regs.cpu_int(0).iidx().read().stat() {
                vals::CpuIntIidxStat::Cnackfg => Poll::Ready(Err(self.nack_kind())),
                vals::CpuIntIidxStat::Carblostfg => Poll::Ready(Err(Error::Arbitration)),
                vals::CpuIntIidxStat::Timeouta => Poll::Ready(Err(Error::Timeout)),
                other => step(self, other),
            };

            if !result.is_pending() {
                self.info
                    .regs
                    .cpu_int(0)
                    .imask()
                    .write_value(i2c::regs::CpuInt::default());
            }

            result
        })
    }

    async fn write_async_internal(&mut self, addr: Address, write: &[u8], end_w_stop: bool) -> Result<(), Error> {
        self.clear_timeout();
        if write.is_empty() {
            return Err(Error::ZeroLengthTransfer);
        }
        if write.len() > MAX_TRANSFER_LEN {
            return Err(Error::TransferLengthIsOverLimit);
        }

        let _guard = self.wake_floor.map(WakeGuard::new);
        let abort = Self::abort_on_drop(self.info.regs, self.state);

        // Prime the FIFO before arming, then let the trigger interrupt top it up. The controller
        // stretches SCL while the FIFO is empty (SLAU846 25.2.3.8), so a late refill costs bus time
        // rather than bytes.
        let mut sent = self.fill_tx(write);

        self.info.regs.cpu_int(0).imask().modify(|w| {
            w.set_carblost(true);
            w.set_cnack(true);
            w.set_timeouta(true);
            w.set_ctxdone(true);
            // Nothing to top up when the whole transfer already fits.
            w.set_ctxfifotrg(sent < write.len());
        });

        self.master_write(addr, write.len(), end_w_stop);

        let res = self
            .run_burst(|this, stat| match stat {
                vals::CpuIntIidxStat::Ctxfifotrg => {
                    sent += this.fill_tx(&write[sent..]);

                    // Reading `IIDX` cleared this one, so the next wake comes from the FIFO draining
                    // again or from the burst finishing. Stop asking once there is nothing left to add.
                    if sent == write.len() {
                        this.info.regs.cpu_int(0).imask().modify(|w| w.set_ctxfifotrg(false));
                    }

                    Poll::Pending
                }
                vals::CpuIntIidxStat::Ctxdonefg => Poll::Ready(Ok(())),
                _ => Poll::Pending,
            })
            .await;

        if let Err(err) = res {
            // The guard's cleanup done eagerly, so it must not run a second time.
            self.recover_after(err);
            abort.defuse();
            return Err(err);
        }

        abort.defuse();
        Ok(())
    }

    async fn read_async_internal(
        &mut self,
        addr: Address,
        read: &mut [u8],
        restart: bool,
        end_w_stop: bool,
    ) -> Result<(), Error> {
        self.clear_timeout();
        if read.is_empty() {
            return Err(Error::ZeroLengthTransfer);
        }
        if read.len() > MAX_TRANSFER_LEN {
            return Err(Error::TransferLengthIsOverLimit);
        }

        let _guard = self.wake_floor.map(WakeGuard::new);
        let abort = Self::abort_on_drop(self.info.regs, self.state);

        self.info.regs.cpu_int(0).imask().modify(|w| {
            w.set_carblost(true);
            w.set_cnack(true);
            w.set_timeouta(true);
            w.set_crxdone(true);
            w.set_crxfifotrg(true);
        });

        // One burst for the whole transfer, so its last byte is the transfer's last byte and is NACKed
        // to release the target. The FIFO is drained as it fills; the controller stretches SCL while it
        // is full (SLAU846 25.2.3.8), so a late drain costs bus time rather than bytes.
        self.master_read(addr, read.len(), restart, false, end_w_stop);

        let mut got = 0;
        let res = self
            .run_burst(|this, stat| match stat {
                vals::CpuIntIidxStat::Crxfifotrg => {
                    got += this.drain_rx(&mut read[got..]);
                    Poll::Pending
                }
                // The burst is over, so what is still in the FIFO is its tail: those bytes are ours
                // whether or not the trigger level is reached again.
                vals::CpuIntIidxStat::Crxdonefg => {
                    got += this.drain_rx(&mut read[got..]);
                    Poll::Ready(Ok(()))
                }
                _ => Poll::Pending,
            })
            .await;

        if let Err(err) = res {
            // The guard's cleanup done eagerly, so it must not run a second time.
            self.recover_after(err);
            abort.defuse();
            return Err(err);
        }

        if got < read.len() {
            // The burst ended without delivering everything and no status bit says why.
            self.recover_after(Error::Bus);
            abort.defuse();
            return Err(Error::Bus);
        }

        abort.defuse();
        Ok(())
    }

    /// Leave the peripheral safe to reuse if a transfer future is dropped part-way.
    ///
    /// Deliberately does not release the bus. A STOP-only command is only legal "after previous
    /// transaction success finished" (SLAU846 table 25-10) and a cancelled transaction has not, so one
    /// issued here is never executed; waiting for the burst to end instead is unbounded, which a drop
    /// handler may not do. [`I2c::recover_bus`] finishes the job at the front of the next transfer.
    ///
    /// [`OnDrop::defuse`] it when the transfer finished on its own.
    fn abort_on_drop(regs: Regs, state: &'static State) -> OnDrop<impl FnOnce()> {
        OnDrop::new(move || {
            // Masking matters on its own: an armed interrupt with nothing left to consume it fires into a
            // handler that only wakes, and re-enters until something masks it. The flags it latched need no
            // clearing here, because `recover_bus` resets the peripheral before the next transfer.
            regs.cpu_int(0).imask().write_value(i2c::regs::CpuInt::default());

            state.abandoned.store(true, Ordering::Relaxed);
        })
    }

    /// Make the peripheral fit for a new transfer after a dropped one, then wait for the bus.
    ///
    /// A dropped transfer leaves a burst running that nobody is servicing, and nothing short of a reset
    /// ends it in bounded time: a STOP-only command is illegal until the transaction finishes (SLAU846
    /// table 25-10), and no interrupt is raised when it does, so waiting for it means polling.
    async fn recover_bus(&mut self) -> Result<(), Error> {
        // Load and clear rather than swap: `thumbv6m` has no CAS, and both sides of this flag run in task
        // context on a `&mut self`, never against an interrupt.
        if self.state.abandoned.load(Ordering::Relaxed) {
            self.state.abandoned.store(false, Ordering::Relaxed);

            // A reset does every part of the cleanup at once and is the only thing that reliably ends the
            // abandoned burst: nothing is raised when one finishes, so anything else means polling.
            self.reset_peripheral();

            // The reset released our end of the bus. If SDA is still down, the target is holding it, and
            // only clocking it out will help — which is the caller's call to make, not ours.
            if self.bus_is_stuck() {
                return Err(Error::BusStuck);
            }
        }

        self.wait_bus_free().await
    }

    /// Wait for the bus to go free, without spinning on `BUSBSY`.
    ///
    /// A STOP is what releases the bus, so `CSTOP` is the wake-up, and the clock-low timeout is the way out
    /// if it never comes. Nothing here is fast — a busy bus means another transfer is in flight, which is
    /// milliseconds — so a spin would hold the executor for a very long time by its standards.
    ///
    /// The flag is cleared before the mask goes on: a STOP that lands between the two would otherwise sit
    /// pending against a handler that only wakes, and re-enter forever without anyone consuming it.
    async fn wait_bus_free(&mut self) -> Result<(), Error> {
        if !self.info.regs.controller(0).csr().read().busbsy() {
            return Ok(());
        }
        if self.bus_is_stuck() {
            return Err(Error::BusStuck);
        }
        self.clear_timeout();

        // Dropping this future part-way has to leave `CSTOP` masked. Left armed it fires into a handler
        // that only wakes, with no future to consume it, and re-enters until something else masks it.
        let regs = self.info.regs;
        let _disarm = OnDrop::new(|| {
            regs.cpu_int(0).imask().modify(|w| {
                w.set_cstop(false);
                w.set_timeouta(false);
            })
        });

        let waited = future::poll_fn(|cx| {
            self.state.waker.register(cx.waker());

            self.info.regs.cpu_int(0).iclr().write(|w| w.set_cstop(true));
            self.info.regs.cpu_int(0).imask().modify(|w| {
                w.set_cstop(true);
                w.set_timeouta(true);
            });

            if self.timed_out() {
                self.clear_timeout();
                return Poll::Ready(Err(Error::Timeout));
            }

            // Checked after arming, so a STOP that arrives in between is caught here rather than waited
            // on forever.
            if self.info.regs.controller(0).csr().read().busbsy() {
                return Poll::Pending;
            }

            Poll::Ready(Ok(()))
        })
        .await;

        // A timeout sticks: `BUSBSY` stays set with `IDLE` set too, so without this the next call waits on
        // a bus that will never be reported free and cannot time out again either, SCL now being high.
        if waited.is_err() {
            self.reset_peripheral();
        }
        waited
    }

    // =========================
    //  Async public API

    /// Write `write` to `address`, ending with a STOP.
    ///
    /// See the module docs on cancelling one of these.
    pub async fn async_write(&mut self, address: impl Into<Address>, write: &[u8]) -> Result<(), Error> {
        let address = Address::checked(address)?;
        self.recover_bus().await?;
        self.write_async_internal(address, write, true).await
    }

    /// Read `read.len()` bytes from `address`, ending with a STOP.
    ///
    /// See the module docs on cancelling one of these.
    pub async fn async_read(&mut self, address: impl Into<Address>, read: &mut [u8]) -> Result<(), Error> {
        let address = Address::checked(address)?;
        self.recover_bus().await?;
        self.read_async_internal(address, read, false, true).await
    }

    /// Write, restart, read.
    ///
    /// Each buffer may hold between one and 4095 bytes.    ///
    /// **A 10-bit address is re-sent before the read.** The hardware puts the addressing header on the
    /// bus a second time after the repeated START, so a target sees this as indistinguishable from a
    /// plain 10-bit read. Measured, and TI documents neither.
    pub async fn async_write_read(
        &mut self,
        address: impl Into<Address>,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Error> {
        let address = Address::checked(address)?;
        self.recover_bus().await?;

        let err = self.write_async_internal(address, write, false).await;
        if err != Ok(()) {
            return err;
        }
        self.read_async_internal(address, read, true, true).await
    }
}

impl<'d> embedded_hal_02::blocking::i2c::Read for I2c<'d, Blocking> {
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, buffer)
    }
}

impl<'d> embedded_hal_02::blocking::i2c::Write for I2c<'d, Blocking> {
    type Error = Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, bytes)
    }
}

impl<'d> embedded_hal_02::blocking::i2c::WriteRead for I2c<'d, Blocking> {
    type Error = Error;

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, bytes, buffer)
    }
}

impl<'d> embedded_hal_02::blocking::i2c::Transactional for I2c<'d, Blocking> {
    type Error = Error;

    fn exec(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_02::blocking::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let address = Address::checked(address)?;
        self.blocking_wait_bus_free()?;
        for i in 0..operations.len() {
            match &mut operations[i] {
                embedded_hal_02::blocking::i2c::Operation::Read(buf) => {
                    self.read_blocking_internal(address, buf, false, false)?
                }
                embedded_hal_02::blocking::i2c::Operation::Write(buf) => {
                    self.write_blocking_internal(address, buf, false)?
                }
            }
        }
        self.master_stop();
        Ok(())
    }
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match *self {
            Self::Bus => embedded_hal::i2c::ErrorKind::Bus,
            Self::BusStuck => embedded_hal::i2c::ErrorKind::Bus,
            Self::Arbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Self::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown),
            Self::NackAddress => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address)
            }
            Self::NackData => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Data),
            Self::Timeout => embedded_hal::i2c::ErrorKind::Other,
            Self::ZeroLengthTransfer => embedded_hal::i2c::ErrorKind::Other,
            Self::TransferLengthIsOverLimit => embedded_hal::i2c::ErrorKind::Other,
            Self::InvalidAddress => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<'d, M: Mode> embedded_hal::i2c::ErrorType for I2c<'d, M> {
    type Error = Error;
}

impl<'d> I2c<'d, Blocking> {
    /// Body of [`embedded_hal::i2c::I2c::transaction`], shared by the impl per addressing mode.
    /// Run a transaction the way `embedded-hal` defines one.
    ///
    /// Consecutive operations of the same type merge into one stretch of bus traffic — one address
    /// phase, then every byte of the run — and a change of direction is a repeated START with the
    /// address again. The last run ends with the STOP, so nothing issues one separately.
    fn eh_transaction(
        &mut self,
        address: Address,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Error> {
        self.blocking_wait_bus_free()?;

        let mut from = 0;
        let mut opened = false;

        while let Some(group) = next_group(operations, from) {
            // A run merges into one burst, and the burst length register bounds it. Splitting a longer
            // run across bursts would put the FIFO back in charge of what a transfer is.
            if group.total > MAX_TRANSFER_LEN {
                return Err(Error::TransferLengthIsOverLimit);
            }

            let last = next_group(operations, group.end).is_none();

            let result = if group.write {
                self.write_group_blocking(address, operations, &group, opened, last)
            } else {
                self.read_group_blocking(address, operations, &group, opened, last)
            };

            if let Err(err) = result {
                self.recover_after(err);
                return Err(err);
            }

            opened = true;
            from = group.end;
        }

        Ok(())
    }

    /// One run of writes, as a single burst.
    fn write_group_blocking(
        &mut self,
        address: Address,
        ops: &mut [embedded_hal::i2c::Operation<'_>],
        group: &Group,
        restart: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        self.clear_timeout();

        let mut cur = GroupCursor {
            op: group.start,
            pos: 0,
        };
        let mut sent = self.fill_tx_group(ops, group.end, &mut cur);

        self.master_blocking_write(address, group.total, restart, send_stop)?;

        while sent < group.total {
            self.check_error()?;

            if !self.info.regs.controller(0).csr().read().busy() {
                return Err(Error::Bus);
            }

            sent += self.fill_tx_group(ops, group.end, &mut cur);
        }

        while self.info.regs.controller(0).csr().read().busy() && !self.timed_out() {}

        self.check_error()
    }

    /// One run of reads, as a single burst.
    fn read_group_blocking(
        &mut self,
        address: Address,
        ops: &mut [embedded_hal::i2c::Operation<'_>],
        group: &Group,
        restart: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        self.clear_timeout();

        self.master_blocking_read(address, group.total, restart, send_stop)?;

        let mut cur = GroupCursor {
            op: group.start,
            pos: 0,
        };
        let mut got = 0;

        while got < group.total {
            self.check_error()?;

            got += self.drain_rx_group(ops, group.end, &mut cur);

            if got < group.total
                && !self.info.regs.controller(0).csr().read().busy()
                && self.info.regs.controller(0).cfifosr().read().rxfifocnt() == 0
            {
                return Err(Error::Bus);
            }
        }

        Ok(())
    }
}

impl<'d> embedded_hal::i2c::I2c for I2c<'d, Blocking> {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, write)
    }

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.eh_transaction(Address::checked(address)?, operations)
    }
}

impl<'d> embedded_hal::i2c::I2c<embedded_hal::i2c::TenBitAddress> for I2c<'d, Blocking> {
    fn read(&mut self, address: u16, read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(Address::TenBit(address), read)
    }

    fn write(&mut self, address: u16, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(Address::TenBit(address), write)
    }

    fn write_read(&mut self, address: u16, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(Address::TenBit(address), write, read)
    }

    fn transaction(
        &mut self,
        address: u16,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.eh_transaction(Address::checked(Address::TenBit(address))?, operations)
    }
}

impl<'d> I2c<'d, Async> {
    /// Body of [`embedded_hal_async::i2c::I2c::transaction`], shared by the impl per addressing mode.
    /// Run a transaction the way `embedded-hal` defines one. See the blocking twin for the shape.
    async fn eh_transaction(
        &mut self,
        address: Address,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Error> {
        self.recover_bus().await?;

        let mut from = 0;
        let mut opened = false;

        while let Some(group) = next_group(operations, from) {
            if group.total > MAX_TRANSFER_LEN {
                return Err(Error::TransferLengthIsOverLimit);
            }

            let last = next_group(operations, group.end).is_none();

            let result = if group.write {
                self.write_group_async(address, operations, &group, last).await
            } else {
                self.read_group_async(address, operations, &group, opened, last).await
            };

            if let Err(err) = result {
                self.recover_after(err);
                return Err(err);
            }

            opened = true;
            from = group.end;
        }

        Ok(())
    }

    /// One run of writes, as a single burst fed from the FIFO trigger interrupt.
    async fn write_group_async(
        &mut self,
        addr: Address,
        ops: &mut [embedded_hal::i2c::Operation<'_>],
        group: &Group,
        send_stop: bool,
    ) -> Result<(), Error> {
        self.clear_timeout();

        let _guard = self.wake_floor.map(WakeGuard::new);
        let abort = Self::abort_on_drop(self.info.regs, self.state);

        let mut cur = GroupCursor {
            op: group.start,
            pos: 0,
        };
        let mut sent = self.fill_tx_group(ops, group.end, &mut cur);

        self.info.regs.cpu_int(0).imask().modify(|w| {
            w.set_carblost(true);
            w.set_cnack(true);
            w.set_timeouta(true);
            w.set_ctxdone(true);
            w.set_ctxfifotrg(sent < group.total);
        });

        self.master_write(addr, group.total, send_stop);

        let res = self
            .run_burst(|this, stat| match stat {
                vals::CpuIntIidxStat::Ctxfifotrg => {
                    sent += this.fill_tx_group(ops, group.end, &mut cur);

                    if sent == group.total {
                        this.info.regs.cpu_int(0).imask().modify(|w| w.set_ctxfifotrg(false));
                    }

                    Poll::Pending
                }
                vals::CpuIntIidxStat::Ctxdonefg => Poll::Ready(Ok(())),
                _ => Poll::Pending,
            })
            .await;

        abort.defuse();
        res
    }

    /// One run of reads, as a single burst drained from the FIFO trigger interrupt.
    async fn read_group_async(
        &mut self,
        addr: Address,
        ops: &mut [embedded_hal::i2c::Operation<'_>],
        group: &Group,
        restart: bool,
        send_stop: bool,
    ) -> Result<(), Error> {
        self.clear_timeout();

        let _guard = self.wake_floor.map(WakeGuard::new);
        let abort = Self::abort_on_drop(self.info.regs, self.state);

        self.info.regs.cpu_int(0).imask().modify(|w| {
            w.set_carblost(true);
            w.set_cnack(true);
            w.set_timeouta(true);
            w.set_crxdone(true);
            w.set_crxfifotrg(true);
        });

        self.master_read(addr, group.total, restart, false, send_stop);

        let mut cur = GroupCursor {
            op: group.start,
            pos: 0,
        };
        let mut got = 0;

        let res = self
            .run_burst(|this, stat| match stat {
                vals::CpuIntIidxStat::Crxfifotrg => {
                    got += this.drain_rx_group(ops, group.end, &mut cur);
                    Poll::Pending
                }
                vals::CpuIntIidxStat::Crxdonefg => {
                    got += this.drain_rx_group(ops, group.end, &mut cur);
                    Poll::Ready(Ok(()))
                }
                _ => Poll::Pending,
            })
            .await;

        abort.defuse();
        res?;

        if got < group.total { Err(Error::Bus) } else { Ok(()) }
    }
}

impl<'d> embedded_hal_async::i2c::I2c for I2c<'d, Async> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.async_read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.async_write(address, write).await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.async_write_read(address, write, read).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.eh_transaction(Address::checked(address)?, operations).await
    }
}

impl<'d> embedded_hal_async::i2c::I2c<embedded_hal::i2c::TenBitAddress> for I2c<'d, Async> {
    async fn read(&mut self, address: u16, read: &mut [u8]) -> Result<(), Self::Error> {
        self.async_read(Address::TenBit(address), read).await
    }

    async fn write(&mut self, address: u16, write: &[u8]) -> Result<(), Self::Error> {
        self.async_write(Address::TenBit(address), write).await
    }

    async fn write_read(&mut self, address: u16, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.async_write_read(Address::TenBit(address), write, read).await
    }

    async fn transaction(
        &mut self,
        address: u16,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.eh_transaction(Address::checked(Address::TenBit(address))?, operations)
            .await
    }
}

impl<'d, M: Mode> Drop for I2c<'d, M> {
    fn drop(&mut self) {
        // Only the pins. A controller has no legal way to stand down mid-burst — a STOP-only command is
        // refused until the transaction finishes (SLAU846 table 25-10) and nothing reports when that is —
        // so releasing the pads is what takes this instance off the bus. Whatever the peripheral is still
        // doing reaches nothing, and the next `I2c::new` on this instance resets it before configuring.
        self.scl.as_ref().map(|x| x.set_as_disconnected());
        self.sda.as_ref().map(|x| x.set_as_disconnected());
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _i2c: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    /// Wake the waiting transfer, leaving every interrupt unmasked.
    ///
    /// The completing poll is what disarms them — it writes `IMASK` clear once its status is no longer
    /// `Pending` — so the handler has nothing to do but wake. That is sound because the controller
    /// pulses its events rather than holding them: a status the poll has not consumed does not re-raise
    /// the line, so returning without masking cannot re-enter.
    unsafe fn on_interrupt() {
        T::state().waker.wake();
    }
}

/// Peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// I2C `SDA` pin trait
pub trait SdaPin<T: Instance>: crate::gpio::Pin {
    /// Get the PF number needed to use this pin as `SDA`.
    fn pf_num(&self) -> u8;
}

/// I2C `SCL` pin trait
pub trait SclPin<T: Instance>: crate::gpio::Pin {
    /// Get the PF number needed to use this pin as `SCL`.
    fn pf_num(&self) -> u8;
}

// ==== IMPL types ====

pub(crate) struct Info {
    pub(crate) regs: Regs,
    pub(crate) interrupt: Interrupt,
    pub fifo_size: usize,
    pub(crate) sleep: SleepInfo,
}

pub(crate) struct State {
    /// Woken by [`InterruptHandler`], which is the only waker side: the handler is bound per
    /// instance, and the driver owns the instance for as long as it can wait on it.
    pub(crate) waker: IrqWaker,
    /// A transfer future was dropped part-way, so the controller is still running a burst nobody is
    /// servicing. Set by [`I2c::abort_on_drop`] and cleared by [`I2c::recover_bus`].
    pub(crate) abandoned: AtomicBool,
}

impl<'d, M: Mode> I2c<'d, M> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
        resolved: Resolved,
    ) -> Result<Self, ConfigError> {
        T::info().regs.gprcm(0).rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(vals::ResetKey::Key);
        });

        T::info().regs.gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });

        // init delay, 16 cycles
        cortex_m::asm::delay(16);

        let scl_inner = new_pin!(scl, config.scl_pf());
        let sda_inner = new_pin!(sda, config.sda_pf());

        if let Some(ref scl) = scl_inner {
            let pincm = pac::IOMUX.pincm(scl._pin_cm() as usize);
            pincm.modify(|w| {
                w.set_hiz1(true);
            });
        }

        if let Some(ref sda) = sda_inner {
            let pincm = pac::IOMUX.pincm(sda._pin_cm() as usize);
            pincm.modify(|w| {
                w.set_hiz1(true);
            });
        }

        let mut this = Self {
            info: T::info(),
            state: T::state(),
            scl: scl_inner,
            sda: sda_inner,
            wake_floor: None,
            resolved,
            _phantom: PhantomData,
        };
        this.init()?;

        Ok(this)
    }
}

pub(crate) trait SealedInstance {
    fn info() -> &'static Info;
    fn state() -> &'static State;
}

macro_rules! impl_i2c_instance {
    ($instance: ident, $fifo_size: expr) => {
        // `Config::source_hz` can assume `BusClk` is ULPCLK because of this
        const _: () = core::assert!(
            matches!(
                <crate::peripherals::$instance as crate::sysctl::LowPowerInstance>::SLEEP.power_domain,
                crate::sysctl::PowerDomain::Pd0
            ),
            "this I2C instance is in PD1, so its bus clock is MCLK rather than ULPCLK"
        );

        impl crate::i2c::SealedInstance for crate::peripherals::$instance {
            fn info() -> &'static crate::i2c::Info {
                use crate::i2c::Info;
                use crate::interrupt::typelevel::Interrupt;

                const INFO: Info = Info {
                    regs: crate::pac::$instance,
                    interrupt: crate::interrupt::typelevel::$instance::IRQ,
                    fifo_size: $fifo_size,
                    sleep: <crate::peripherals::$instance as crate::sysctl::LowPowerInstance>::SLEEP,
                };
                &INFO
            }

            fn state() -> &'static crate::i2c::State {
                use crate::i2c::State;
                use crate::interrupt::typelevel::Interrupt;

                static STATE: State = State {
                    waker: crate::sync::irq_waker::IrqWaker::new(),
                    abandoned: core::sync::atomic::AtomicBool::new(false),
                };
                &STATE
            }
        }

        impl crate::i2c::Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

macro_rules! impl_i2c_sda_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::i2c::SdaPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

macro_rules! impl_i2c_scl_pin {
    ($instance: ident, $pin: ident, $pf: expr) => {
        impl crate::i2c::SclPin<crate::peripherals::$instance> for crate::peripherals::$pin {
            fn pf_num(&self) -> u8 {
                $pf
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use crate::i2c::{ClockDiv, ClockSel, Timing};
    use crate::sysctl::Clocks;

    /// A tree with both sources at explicit rates, so the expected periods below do not depend on what
    /// this chip's SYSOSC happens to boot at.
    const CLOCKS: Clocks = Clocks {
        ulpclk: 32_000_000,
        mfclk: 4_000_000,
        ..Clocks::RESET
    };

    const BUSCLK: ClockSel = ClockSel::BusClk;
    const MFCLK: ClockSel = ClockSel::MfClk;

    const STANDARD: u32 = 100_000;
    const FAST_MODE: u32 = 400_000;
    const FAST_MODE_PLUS: u32 = 1_000_000;

    /// These are based on TI's reference calculation.
    #[test]
    fn ti_timer_period() {
        // 32 MHz / (400 kHz * 10) - 1
        core::assert_eq!(
            Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, FAST_MODE).map(|t| t.tpr()),
            Some(7)
        );

        // 16 MHz / (400 kHz * 10) - 1
        core::assert_eq!(
            Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy2, FAST_MODE).map(|t| t.tpr()),
            Some(3)
        );

        // 16 MHz / (100 kHz * 10) - 1
        core::assert_eq!(
            Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy2, STANDARD).map(|t| t.tpr()),
            Some(15)
        );
    }

    /// The divided source rate travels with the timing, so the driver never recomputes it.
    #[test]
    fn timing_carries_divided_rate() {
        let timing = Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy2, FAST_MODE).unwrap();

        core::assert_eq!(timing.clock_hz(), CLOCKS.ulpclk / 2);
    }

    /// The source must be at least 20x the bus speed.
    #[test]
    fn rejects_insufficient_headroom() {
        // 32 MHz against 400 kHz and 1 MHz both clear 20x.
        core::assert!(Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, FAST_MODE).is_some());
        core::assert!(Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, FAST_MODE_PLUS).is_some());

        // 4 MHz does not: 400 kHz needs 8 MHz and 1 MHz needs 20 MHz.
        core::assert!(Timing::solve(&CLOCKS, MFCLK, ClockDiv::DivBy1, FAST_MODE).is_none());
        core::assert!(Timing::solve(&CLOCKS, MFCLK, ClockDiv::DivBy1, FAST_MODE_PLUS).is_none());

        // 100 kHz off MFCLK does clear it.
        core::assert!(Timing::solve(&CLOCKS, MFCLK, ClockDiv::DivBy1, STANDARD).is_some());
    }

    /// A period the register cannot hold is refused rather than wrapping.
    #[test]
    fn rejects_unrepresentable_period() {
        // A very slow bus off a fast clock overflows TPR's 7 bits.
        core::assert!(Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, 1_000).is_none());

        // A zero bus speed cannot be divided by.
        core::assert!(Timing::solve(&CLOCKS, BUSCLK, ClockDiv::DivBy1, 0).is_none());
    }
}
