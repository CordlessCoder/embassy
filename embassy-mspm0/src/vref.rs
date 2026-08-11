//! Internal voltage reference (VREF).
//!
//! Drives the reference the ADC, the comparators and the DAC can select instead of `VDDA`. A
//! [`Vref`] exists only while the reference is powered, and constructing one does not return until
//! the output has settled — so a peripheral configured to use it after that point is reading a
//! reference that is up.
//!
//! # `STAT.READY` is not how this knows
//!
//! It cannot be, on 93 of the 216 supported devices. `VREF_ERR_01` says the bit works the first time
//! VREF is enabled after a reset and **never clears again** once VREF has been disabled, so on every
//! later enable it reads "ready" immediately and means nothing. TI's own workaround is to wait the
//! datasheet startup time instead, and that is what [`Vref::new`] does wherever the erratum applies.
//!
//! Devices without the erratum wait on the bit, which is both faster and exact.

#![macro_use]

use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};
use mspm0_metapac::vref::Vref as Regs;
use mspm0_metapac::vref::vals::{Bufconfig, PwrenKey, ResetKey};

use crate::sysctl::LowPowerInstance;

/// How long the reference takes to settle after being enabled, in nanoseconds.
///
/// Per device, from the datasheet's `Tstartup` row: 200 us on an MSPM0G3507 against 10 us on an
/// MSPM0C1104, a 20x spread that follows neither the family nor the register block.
///
/// **Typical rather than a guaranteed ceiling.** The datasheet cell spans its MIN, TYP and MAX
/// columns, so the figure cannot be read as the worst case — and `VREF_ERR_01`'s workaround asks for
/// the *maximum*. It has measured sufficient on the one part checked on silicon; a board at a
/// temperature or capacitance extreme could want more, and there is nothing here that would report
/// it.
///
/// **Where a datasheet gives the row under several conditions this is the slowest**, which is why the
/// large figures look out of line: the G5187 states 20 us bare and 200 us with a 1 uF capacitor on
/// `VREF+`, and this is the latter. So the number describes a loaded reference, which is what a board
/// following TI's reference design has.
pub const STARTUP_NS: u32 = crate::_generated::VREF_STARTUP_NS;

/// The reference buffer this driver drives.
///
/// `CTL0.ENABLE` addresses three and `CTL1.READY` reports on three, but **nothing published says how
/// many a given device implements** — not the datasheets, and not sysconfig. TI's own `dl_vref.h` only
/// ever touches buffer 0, and buffer 0 is what feeds the ADC on the part this was measured on.
///
/// The other two are not spare copies: `hw_vref.h` calls buffer 1 `COMP_VREF_ENABLE` and buffer 2
/// `ADC_VREF_ENABLE`, so they are dedicated paths on whatever devices carry them. Reaching them wants a
/// device that is known to have them, which is why this is a constant and not a parameter.
const BUFFER: usize = 0;

/// Whether [`Vref::new`] has to wait [`STARTUP_NS`] rather than ask the hardware.
///
/// `true` on the 93 of 216 devices carrying `VREF_ERR_01`, where `CTL1.READY` is stuck set from an
/// earlier enable and cannot be trusted. `false` elsewhere, where the bit is polled and construction
/// takes as long as the reference actually needs and no longer.
pub const STARTUP_IS_TIMED: bool = cfg!(vref_err_01);

/// Output voltage of the reference buffer.
///
/// Chosen once, when the driver is built. **Changing it while VREF is running is deliberately not
/// offered**: on the G-series, `VREF_ERR_02` makes the 2.5 V to 1.4 V direction slew so slowly that
/// TI's workaround is to disable VREF, drive the `VREF+` pin low as a GPIO for 100 us against an
/// external 1 uF capacitor, and re-enable. That needs a pin this driver was not given and a capacitor
/// it cannot know about, so it is the caller's to perform — by dropping the [`Vref`] and building a
/// new one around it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Voltage {
    /// 1.4 V.
    Volts1_4,

    /// 2.5 V. Needs `VDDA` above it with headroom; see the device datasheet.
    Volts2_5,
}

impl Voltage {
    const fn bufconfig(self) -> Bufconfig {
        match self {
            Voltage::Volts1_4 => Bufconfig::Output1p4v,
            Voltage::Volts2_5 => Bufconfig::Output2p5v,
        }
    }
}

/// VREF configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Output voltage of the reference buffer.
    pub voltage: Voltage,

    /// Clock the reference's regulation runs from.
    ///
    /// **The reference does nothing without one.** Left unselected it never regulates: `CTL1.READY`
    /// stays clear for ever and every consumer reads a level that was never established.
    ///
    /// This decides what the reference survives, so it is the caller's to pick. [`ClockSel::BusClk`]
    /// is the default and is what TI's own initialisation uses, but the bus clock stops in every
    /// deep-sleep mode, so a reference that has to keep regulating across one wants
    /// [`ClockSel::LfClk`].
    pub clock: ClockSel,
}

/// The clock source for the reference.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSel {
    /// The bus clock. Stops in every deep-sleep mode.
    BusClk,

    /// MFCLK, 4 MHz. Off in STANDBY and below.
    MfClk,

    /// LFCLK, 32 kHz. Runs in every mode, so the reference keeps regulating across a deep sleep.
    LfClk,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // The reset value, and the one an ADC measuring against a 3.3 V rail usually wants.
            voltage: Voltage::Volts2_5,
            // What driverlib's own initialisation picks. A caller that needs the reference through a
            // deep sleep has to say so.
            clock: ClockSel::BusClk,
        }
    }
}

/// The internal voltage reference, powered and settled.
///
/// Dropping this powers the reference down, so it has to outlive whatever selected it — an [`Adc`]
/// converting against `Vrsel::IntrefVssa` with no live `Vref` is measuring against a reference that
/// is off.
///
/// [`Adc`]: crate::adc::Adc
///
/// # The instance parameter costs nothing here, today
///
/// `T` would duplicate every method body per instance, but every supported device has exactly one of
/// these. A part with two would start paying, and
/// [`simple_pwm::SimplePwm`](crate::tim::simple_pwm::SimplePwm) has the measurements and the reason
/// erasing the parameter is not automatically the fix.
pub struct Vref<'d, T: Instance> {
    _instance: Peri<'d, T>,
    _phantom: PhantomData<&'d mut ()>,
}

impl<'d, T: Instance> Vref<'d, T> {
    /// Power the reference up and wait for it to settle.
    ///
    /// Blocks for as long as the reference takes to start — see [`STARTUP_NS`], up to 200 us today.
    /// When it returns, a peripheral may select the reference and trust what it reads.
    pub fn new(instance: Peri<'d, T>, config: Config) -> Self {
        let r = T::regs();

        r.gprcm().rstctl().write(|w| {
            w.set_resetassert(true);
            w.set_resetstkyclr(true);
            w.set_key(ResetKey::Key);
        });

        r.gprcm().pwren().write(|w| {
            w.set_enable(true);
            w.set_key(PwrenKey::Key);
        });

        // Without a clock the reference never regulates: `CTL1.READY` stays clear for ever and
        // nothing that selects the reference reads anything. Measured — selecting a source flips
        // `READY` at once, on two parts. Driverlib's own init picks a source before enabling, which
        // is what this mirrors.
        //
        // A source that is not running is the same failure with a different cause, so it is refused
        // here rather than left to show up as a reference that reads nothing.
        let running = crate::sysctl::with_clocks(|clocks| match config.clock {
            ClockSel::BusClk => true,
            ClockSel::MfClk => clocks.mfclk != 0,
            ClockSel::LfClk => clocks.lfclk != 0,
        });
        assert!(running, "the clock source VREF was given is not running");

        // Undivided. The reference regulates from this clock rather than timing anything with it, so
        // there is nothing for a divider to buy, and driverlib leaves it at one too.
        r.clkdiv().write(|w| w.set_ratio(0));
        r.clksel().write(|w| match config.clock {
            ClockSel::BusClk => w.set_busclk_sel(true),
            ClockSel::MfClk => w.set_mfclk_sel(true),
            ClockSel::LfClk => w.set_lfclk_sel(true),
        });

        // The voltage goes in before the buffer is enabled, so the reference ramps once to the level
        // that was asked for rather than ramping to the reset one and then changing — which is the
        // transition `VREF_ERR_02` makes slow.
        r.ctl0().write(|w| {
            w.set_bufconfig(config.voltage.bufconfig());
            w.set_enable(BUFFER, true);
        });

        Self::wait_until_settled();

        // No `WakeGuard`. VREF is in PD0 on every supported device and deep sleep does not power PD0
        // down, so its configuration survives without anything held — checked per instance in
        // `impl_vref_instance!` rather than assumed here.
        Self {
            _instance: instance,
            _phantom: PhantomData,
        }
    }

    /// Wait out the reference's startup, by whichever means this device allows.
    #[cfg(vref_err_01)]
    fn wait_until_settled() {
        // `STAT.READY` is stuck set from a previous enable on this device, so it cannot be asked. The
        // wait is derived from MCLK because it is a CPU-cycle delay, and MCLK is the CPU clock in RUN.
        let cycles = startup_cycles(crate::sysctl::clocks().mclk);
        cortex_m::asm::delay(cycles);
    }

    /// Wait out the reference's startup, by whichever means this device allows.
    #[cfg(not(vref_err_01))]
    fn wait_until_settled() {
        // No `VREF_ERR_01` here, so the bit means what it says and is both faster and exact. Bounded
        // by the hardware: the reference either comes up or the device has no usable reference at all.
        while !T::regs().ctl1().read().ready(BUFFER) {}
    }
}

impl<'d, T: Instance> Drop for Vref<'d, T> {
    fn drop(&mut self) {
        let r = T::regs();

        r.ctl0().modify(|w| w.set_enable(BUFFER, false));

        r.gprcm().pwren().write(|w| {
            w.set_enable(false);
            w.set_key(PwrenKey::Key);
        });
    }
}

/// CPU cycles that cover [`STARTUP_NS`] at `mclk`, rounded up and at least one.
///
/// Only the `VREF_ERR_01` path waits by counting; everywhere else the hardware is asked.
///
/// Split out so the arithmetic is checked rather than inlined into a delay call: at 80 MHz the 200 us
/// placeholder is 16,000 cycles, which is well inside a `u32` but not inside a `u16`.
#[cfg(vref_err_01)]
const fn startup_cycles(mclk: u32) -> u32 {
    let mclk = if mclk == 0 { 1 } else { mclk };

    // `mclk / 1_000_000 * STARTUP_NS / 1000` regrouped to keep it exact without overflowing: MCLK is
    // at most 80 MHz, so `mclk / 1000` is at most 80,000 and the product at most 16 billion — which is
    // why it is done in `u64`.
    let cycles = (mclk as u64 * STARTUP_NS as u64).div_ceil(1_000_000_000);

    if cycles == 0 { 1 } else { cycles as u32 }
}

// A factor-of-1000 slip in `startup_cycles` is the one error here that no test would catch: too large
// and the reference is merely slow to hand over, too small and it is handed over unsettled, which reads
// as an inaccurate conversion rather than as a fault.
//
// Written against `STARTUP_NS` rather than against its value, because that value is per device now — an
// earlier version pinned 6400 cycles at 32 MHz, which was right for the 200 us part it was written on
// and failed to build on the 15 us one.
#[cfg(vref_err_01)]
const _: () = {
    // At 1 GHz a cycle is a nanosecond, so the conversion is the identity and any unit slip shows here.
    core::assert!(startup_cycles(1_000_000_000) == STARTUP_NS);
    // Rounds up rather than truncating, and never waits zero cycles.
    core::assert!(startup_cycles(1) == 1);
    // A clock this actually runs at, against the arithmetic done the other way round.
    core::assert!(startup_cycles(32_000_000) == (STARTUP_NS as u64 * 32 / 1000) as u32);
};

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + LowPowerInstance {}

pub(crate) trait SealedInstance {
    fn regs() -> Regs;
}

macro_rules! impl_vref_instance {
    ($instance: ident) => {
        // `Vref` holds no `WakeGuard` because on every device shipped so far VREF sits in PD0, which
        // deep sleep leaves powered. That is a fact about the metadata rather than about the register
        // block, so it is checked here: a device that moves VREF into PD1 fails to build instead of
        // silently losing its reference configuration across a sleep.
        const _: () = {
            use crate::sysctl::LowPowerInstance;

            core::assert!(
                <crate::peripherals::$instance as LowPowerInstance>::SLEEP
                    .floor_to_keep_configured()
                    .is_none(),
                "VREF needs a WakeGuard on this device: its configuration does not survive deep sleep"
            );
        };

        impl crate::vref::SealedInstance for crate::peripherals::$instance {
            #[inline]
            fn regs() -> mspm0_metapac::vref::Vref {
                crate::pac::$instance
            }
        }

        impl crate::vref::Instance for crate::peripherals::$instance {}
    };
}
