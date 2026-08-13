//! MATHACL, the math accelerator.
//!
//! Division, trigonometry and coordinate conversion in hardware, so a core with no divider and no
//! floating-point unit does not have to link one. Every entry point takes and returns [`IQType`],
//! fixed point in the format the registers use; the `f32` wrappers are a convenience that costs the
//! caller a soft-float library.

#![macro_use]

use core::f32::consts::PI;
use core::marker::PhantomData;

use embassy_hal_internal::PeripheralType;
use micromath::F32Ext;

use crate::Peri;
use crate::pac::mathacl::{Mathacl as Regs, vals};
use crate::sysctl::MaybeWakeGuard;

/// How close a float has to be for the unit tests to call it equal.
///
/// It used to double as `div_iq`'s divide-by-zero guard, rejecting any divisor inside it. That test is
/// on the encoding now: a divisor of 1e-6 is not a division by zero, and testing it as a float was
/// what pulled the software floating-point routines into every caller of `div_iq`.
#[cfg(test)]
const ERROR_TOLERANCE: f32 = 0.00001;

/// How many bits of the result the accelerator iterates for, one per cycle.
///
/// The count is the cost: [`Precision::High`] takes 31 cycles and [`Precision::Low`] one.
pub enum Precision {
    High = 31,
    Medium = 15,
    Low = 1,
}

/// `-0.5` as the accelerator's registers hold it: **two's complement**, 31 fractional bits and no
/// integer bits, so a half is the top bit below the sign.
///
/// The one input `MATHACL_ERR_02` gets wrong, which is -90 degrees expressed per unit of pi.
#[cfg(mathacl_err_02)]
const NEGATIVE_HALF: u32 = 0xC000_0000;

/// `-1.0` in the same format, which two's complement holds exactly where a sign-and-magnitude layout
/// could not.
///
/// Worth being careful with: reading this as sign-and-magnitude gives `0xFFFF_FFFF`, which decodes to
/// **one least-significant bit** rather than to minus one, and the difference does not show up until
/// the answer is printed.
#[cfg(mathacl_err_02)]
const NEGATIVE_ONE: u32 = 0x8000_0000;

/// Reads of `STATUS.BUSY` before an operation is declared wedged.
///
/// An operation takes at most `NUMITER` cycles — 31 at [`Precision::High`] — and a poll is a register
/// read, so the loop resolves in tens of iterations. This is orders of magnitude above that, which is
/// the point: it bounds a spin that would otherwise be unbounded without ever being reached in
/// ordinary use.
const MAX_POLLS: u32 = 4096;

/// Error type for Mathacl operations.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// A division produced a quotient wider than the 32 bits the result register holds.
    ///
    /// The value in `RES1` is saturated or truncated rather than correct, so it is not returned.
    /// Reachable from `div_iq` whenever the quotient exceeds the dividend's own fixed-point format,
    /// which needs no extreme inputs at all, and from `div_i32` at `i32::MIN / -1`.
    Overflow,
    /// The accelerator did not finish an operation within `MAX_POLLS` reads of `STATUS.BUSY`.
    ///
    /// Not reachable by a caller doing anything wrong: an operation takes at most `NUMITER` cycles,
    /// so this means the peripheral is wedged.
    Timeout,
    /// The angle was outside the accepted range.
    ValueInWrongRange,
    /// Refused before the accelerator saw it, which reports sooner than `STATUS.ERR` and keeps
    /// `MATHACL_ERR_01`'s reset-to-recover out of reach.
    DivideByZero,
    /// The two operands of a division are in different fixed-point formats.
    FaultIQTypeFormat,
    /// A value could not be put into the fixed-point format asked for.
    IQTypeError(IQTypeError),
}

pub struct Mathacl<'d> {
    regs: &'static Regs,
    /// Held for as long as the driver exists; see
    /// [`SleepInfo::floor_to_keep_configured`](crate::sysctl::SleepInfo::floor_to_keep_configured).
    ///
    /// MATHACL keeps its configuration no deeper than SLEEP, so deep sleep would discard it.
    _retention_guard: MaybeWakeGuard,
    _phantom: PhantomData<&'d mut ()>,
}

impl<'d> Mathacl<'d> {
    /// Mathacl initialization.
    pub fn new<T: Instance>(_instance: Peri<'d, T>) -> Self {
        T::regs().gprcm(0).rstctl().write(|w| {
            w.set_resetstkyclr(true);
            w.set_resetassert(true);
            w.set_key(vals::ResetKey::Key);
        });

        T::regs().gprcm(0).pwren().write(|w| {
            w.set_enable(true);
            w.set_key(vals::PwrenKey::Key);
        });

        // Init delay from the M0 examples by TI in CCStudio (16 cycles)
        cortex_m::asm::delay(16);

        Self {
            regs: T::regs(),
            _retention_guard: MaybeWakeGuard::new(
                <T as crate::sysctl::LowPowerInstance>::SLEEP.floor_to_keep_configured(),
            ),
            _phantom: PhantomData,
        }
    }

    /// Wait for the current operation to finish, or give up.
    ///
    /// Every result read goes through this. The alternative — trusting the bus to stall on a result
    /// register read — holds for `RES1` and not for `RES2`, so it is not a distinction worth building
    /// on when the poll costs tens of cycles.
    fn wait_for_result(&mut self) -> Result<(), Error> {
        for _ in 0..MAX_POLLS {
            let status = self.regs.status().read();

            if !status.busy() {
                // Free: the status word is already loaded for the `BUSY` test above, so noticing an
                // overflow costs a bit test rather than a register read.
                //
                // `STATUS.ERR` is not checked with it, and that is deliberate. It reports a division
                // by zero, which the guards on the public divides refuse before the accelerator ever
                // sees one — catching it there tells the caller sooner and keeps `MATHACL_ERR_01`,
                // where a set `ERR` latches until the peripheral is reset, out of reach entirely.
                if status.ovf() {
                    self.clear_overflow();
                    return Err(Error::Overflow);
                }

                return Ok(());
            }
        }

        Err(Error::Timeout)
    }

    /// Clear a latched overflow, which stays set until written otherwise and would then be read as
    /// belonging to the next operation.
    fn clear_overflow(&mut self) {
        self.regs.statusclr().write(|w| w.set_clr_ovf(true));
    }

    /// Sine of an angle given as a fraction of pi, without touching floating point.
    ///
    /// `per_unit` is the angle divided by pi, so `-0.5` is -90 degrees. That is the form the
    /// accelerator takes, so this is the shortest path to it: no normalising division, and nothing
    /// that would link the software floating-point routines a core with no FPU needs for [`Self::sin`].
    ///
    /// The argument must carry no integer bits and be signed, which is what its range of just under
    /// -1 to 1 needs; anything else is [`Error::FaultIQTypeFormat`]. The result comes back in the same
    /// format.
    pub fn sin_per_unit(&mut self, per_unit: IQType, precision: Precision) -> Result<IQType, Error> {
        self.sincos_per_unit(per_unit, precision, true)
    }

    /// Cosine of an angle given as a fraction of pi, without touching floating point.
    ///
    /// See [`Self::sin_per_unit`].
    pub fn cos_per_unit(&mut self, per_unit: IQType, precision: Precision) -> Result<IQType, Error> {
        self.sincos_per_unit(per_unit, precision, false)
    }

    /// The fixed-point SINCOS, which every other trigonometric entry point goes through.
    fn sincos_per_unit(&mut self, per_unit: IQType, precision: Precision, sin: bool) -> Result<IQType, Error> {
        if per_unit.i_bits != 0 || !per_unit.signed {
            return Err(Error::FaultIQTypeFormat);
        }

        let operand = per_unit.to_reg();

        // `MATHACL_ERR_02`: the accelerator answers `SIN(-90)` with `+1`. TI offers no workaround but
        // correcting it in software, and the exact answer is known, so it is returned rather than the
        // hardware's negated — which would be the same number by a longer route.
        //
        // Exactly one input is affected, which is what makes a point fix right: measured either side,
        // -89.8 and -88.2 degrees both come back correctly signed. Compared against the encoded
        // operand rather than a decoded value, so the test is on the number the accelerator is handed.
        #[cfg(mathacl_err_02)]
        if sin && operand == NEGATIVE_HALF {
            return IQType::from_reg(NEGATIVE_ONE, 0, true).map_err(Error::IQTypeError);
        }

        self.regs.ctl().write(|w| {
            w.set_func(vals::Func::Sincos);
            w.set_numiter(precision as u8);
        });

        self.regs.op1().write_value(operand);

        // SLAU846 §10.3 calls this poll optional, on the grounds that reading a result before the
        // operation finishes stalls the bus until it completes. **That is not true of `RES2`**, which
        // is where SINCOS puts the sine: without this, `sin` returns the previous call's answer, while
        // `cos` off `RES1` is correct. Measured, and it is why the poll is back.
        self.wait_for_result()?;

        let result = match sin {
            true => self.regs.res2().read(),
            false => self.regs.res1().read(),
        };

        IQType::from_reg(result, 0, true).map_err(Error::IQTypeError)
    }

    /// Internal helper SINCOS function.
    fn sincos(&mut self, rad: f32, precision: Precision, sin: bool) -> Result<f32, Error> {
        if !(-PI..=PI).contains(&rad) {
            return Err(Error::ValueInWrongRange);
        }

        let native = self
            .div_iq(IQType::from_f32(rad, 15, true)?, IQType::from_f32(PI, 15, true)?)?
            .to_f32();

        // The hardware takes the angle per unit of pi, in a format with no integer bit, so a magnitude
        // of exactly one has nowhere to go — `from_f32` rejects it and this used to unwrap that into a
        // panic on `sin(PI)`, an input the range check above accepts. Both results are exact, so answer
        // them here rather than finding a way to hand the accelerator a number it cannot hold.
        if native <= -1.0 || native >= 1.0 {
            return Ok(if sin { 0.0 } else { -1.0 });
        }

        Ok(self
            .sincos_per_unit(IQType::from_f32(native, 0, true)?, precision, sin)?
            .to_f32())
    }

    /// Sine of `rad`, which must be in `[-PI, PI]`.
    ///
    /// Takes and returns `f32`, so it links a soft-float library on a core with no FPU;
    /// [`Mathacl::sin_per_unit`] is the same operation in the units the hardware wants.
    pub fn sin(&mut self, rad: f32, precision: Precision) -> Result<f32, Error> {
        self.sincos(rad, precision, true)
    }

    /// Cosine of `rad`, which must be in `[-PI, PI]`. See [`Mathacl::sin`] on the `f32` cost.
    pub fn cos(&mut self, rad: f32, precision: Precision) -> Result<f32, Error> {
        self.sincos(rad, precision, false)
    }

    /// Signed division, reporting a zero divisor rather than dividing by it.
    pub fn div_i32(&mut self, dividend: i32, divisor: i32) -> Result<i32, Error> {
        if divisor == 0 {
            return Err(Error::DivideByZero);
        } else if dividend == 0 {
            return Ok(0);
        }
        let signed = true;

        self.regs.ctl().write(|w| {
            w.set_func(vals::Func::Div);
            w.set_optype(signed);
        });

        self.regs.op2().write_value(divisor as u32);

        self.regs.op1().write_value(dividend as u32);

        self.wait_for_result()?;
        Ok(self.regs.res1().read() as i32)
    }

    /// Unsigned division, reporting a zero divisor rather than dividing by it.
    pub fn div_u32(&mut self, dividend: u32, divisor: u32) -> Result<u32, Error> {
        if divisor == 0 {
            return Err(Error::DivideByZero);
        } else if dividend == 0 {
            return Ok(0);
        }
        let signed = false;

        self.regs.ctl().write(|w| {
            w.set_func(vals::Func::Div);
            w.set_optype(signed);
        });

        self.regs.op2().write_value(divisor);

        self.regs.op1().write_value(dividend);

        self.wait_for_result()?;
        Ok(self.regs.res1().read())
    }

    /// Divide function (DIV) computes with a known dividend and divisor.
    pub fn div_iq(&mut self, dividend: IQType, divisor: IQType) -> Result<IQType, Error> {
        // Tested on the encoding rather than on `to_f32`, so a fixed-point caller never reaches the
        // software floating-point routines through this. It is also exact where the old test was not:
        // a divisor of 1e-6 is not zero and no longer reports as a division by zero — it divides, and
        // says `Overflow` if the quotient will not fit.
        //
        // Catching the true zero here is what keeps `MATHACL_ERR_01` unreachable, where a `STATUS.ERR`
        // the accelerator sets stays set until the peripheral is reset.
        if divisor.i_data == 0 && divisor.f_data == 0 {
            return Err(Error::DivideByZero);
        }

        // check if both numbers have the same number of bits
        if dividend.f_bits != divisor.f_bits {
            return Err(Error::FaultIQTypeFormat);
        }

        // dividen and divisor must have the same signedness
        if dividend.signed ^ divisor.signed {
            return Err(Error::FaultIQTypeFormat);
        }

        self.regs.ctl().write(|w| {
            w.set_func(vals::Func::Div);
            w.set_optype(dividend.signed);
            w.set_qval(dividend.f_bits.into());
        });

        self.regs.op2().write_value(divisor.to_reg());

        self.regs.op1().write_value(dividend.to_reg());

        self.wait_for_result()?;

        IQType::from_reg(self.regs.res1().read(), dividend.i_bits, dividend.signed).map_err(Error::IQTypeError)
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> &'static Regs;
}

/// Mathacl instance trait
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + crate::sysctl::LowPowerInstance {}

macro_rules! impl_mathacl_instance {
    ($instance: ident) => {
        impl crate::mathacl::SealedInstance for crate::peripherals::$instance {
            fn regs() -> &'static crate::pac::mathacl::Mathacl {
                &crate::pac::$instance
            }
        }

        impl crate::mathacl::Instance for crate::peripherals::$instance {}
    };
}

/// Why a value could not be held in the fixed-point format asked for.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IQTypeError {
    /// A negative value was asked for in an unsigned format.
    FaultySignParameter,
    /// The integer part needs more bits than the format leaves for it.
    IntPartIsTrimmed,
}

impl From<IQTypeError> for Error {
    fn from(e: IQTypeError) -> Self {
        Error::IQTypeError(e)
    }
}

/// A 32-bit fixed-point number, with the integer and fractional widths chosen per value.
///
/// This is the accelerator's own format, so a caller that stays in it links no soft-float at all.
/// The register layout is **two's complement**, which [`IQType::from_reg`] is the authority on — a
/// hand-written constant read as sign-and-magnitude decodes to one LSB rather than to the value it
/// looks like.
#[derive(Debug, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IQType {
    i_bits: u8,
    f_bits: u8,
    negative: bool,
    signed: bool,
    i_data: u32,
    f_data: u32,
}

impl IQType {
    /// Decode a result register, `i_bits` of which are the integer part.
    ///
    /// A `signed` value spends one bit on the sign and holds the rest in two's complement.
    pub fn from_reg(data: u32, i_bits: u8, signed: bool) -> Result<Self, IQTypeError> {
        let negative = signed && ((1u32 << 31) & data != 0);

        let total_bits = if signed { 31 } else { 32 };

        let f_bits = total_bits - i_bits;

        let max_mask = if signed { 0x7FFFFFFF } else { 0xFFFFFFFF };
        let (i_mask, f_mask) = if i_bits == 0 {
            (0, max_mask)
        } else if i_bits == total_bits {
            (max_mask, 0)
        } else {
            ((1u32 << i_bits) - 1, (1u32 << f_bits) - 1)
        };

        let mut i_data = if i_bits == 0 {
            0
        } else if i_bits == total_bits {
            data & i_mask
        } else {
            (data >> f_bits) & i_mask
        };
        let mut f_data = data & f_mask;

        // if negative, do 2’s complement
        if negative {
            i_data = !i_data & i_mask;
            f_data = (!f_data & f_mask) + 1;
        }

        Ok(Self {
            i_bits,
            f_bits,
            negative,
            signed,
            i_data,
            f_data,
        })
    }

    /// Convert from `f32`, which is what pulls a soft-float library in. Prefer [`IQType::from_reg`]
    /// where the value is already fixed point.
    pub fn from_f32(data: f32, i_bits: u8, signed: bool) -> Result<Self, IQTypeError> {
        let negative = data < 0.0;

        if !signed && negative {
            return Err(IQTypeError::FaultySignParameter);
        }

        let abs = if data < 0.0 { -data } else { data };

        let total_bits = if signed { 31 } else { 32 };

        let f_bits: u8 = total_bits - i_bits;

        let abs_floor = abs.floor();
        let i_data = abs_floor as u32;
        let f_data = ((abs - abs_floor) * (1u32 << f_bits) as f32).round() as u32;

        if i_bits == 0 && i_data > 0 {
            return Err(IQTypeError::IntPartIsTrimmed);
        }

        Ok(Self {
            i_bits,
            f_bits,
            negative,
            signed,
            i_data,
            f_data,
        })
    }

    /// Convert to `f32`. See [`IQType::from_f32`] on what that costs.
    pub fn to_f32(&self) -> f32 {
        let mut value = (self.i_data as f32) + (self.f_data as f32) / (1u32 << self.f_bits) as f32;
        if self.negative {
            value = -value;
        }
        value
    }

    /// Encode for an operand register, in the two's-complement layout [`IQType::from_reg`] describes.
    pub fn to_reg(&self) -> u32 {
        // `f_data` can be one past its field, carrying into the integer part: `from_reg` two's
        // complements the fraction on its own, and `from_f32` rounds it up. Add the two rather than
        // masking and OR-ing them, which drops the carry and encodes a whole number one too small.
        let mut res = if self.i_bits == 0 {
            0
        } else {
            self.i_data << self.f_bits
        };
        res = res.wrapping_add(self.f_data);

        // if negative, do 2’s complement
        if self.negative {
            res = res.wrapping_neg();
        }
        res
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mathacl_iqtype_errors() {
        // integer part trimmed
        let mut test_float = 1.0;
        core::assert_eq!(
            IQType::from_f32(test_float, 0, true),
            Err(IQTypeError::IntPartIsTrimmed)
        );
        // negative value for unsigned type
        test_float = -1.0;
        core::assert_eq!(
            IQType::from_f32(test_float, 1, false),
            Err(IQTypeError::FaultySignParameter)
        );
    }

    #[test]
    fn mathacl_iqtype_f32_to_f32() {
        core::assert_eq!(IQType::from_f32(0.0, 15, true).unwrap().to_f32(), 0.0);
        core::assert_eq!(IQType::from_f32(0.0, 16, false).unwrap().to_f32(), 0.0);

        core::assert_eq!(IQType::from_f32(1.5, 16, false).unwrap().to_f32(), 1.5);
        core::assert_eq!(IQType::from_f32(1.5, 15, true).unwrap().to_f32(), 1.5);
        core::assert_eq!(IQType::from_f32(-1.5, 15, true).unwrap().to_f32(), -1.5);
    }

    #[test]
    fn mathacl_iqtype_reg_to_reg() {
        core::assert_eq!(IQType::from_reg(0x0, 15, true).unwrap().to_reg(), 0x0);
        core::assert_eq!(IQType::from_reg(0x0, 16, false).unwrap().to_reg(), 0x0);

        core::assert_eq!(IQType::from_reg(0x00018000, 15, true).unwrap().to_reg(), 0x00018000);
        core::assert_eq!(IQType::from_reg(0x00018000, 16, false).unwrap().to_reg(), 0x00018000);
        core::assert_eq!(IQType::from_reg(0xFFFE5556, 15, true).unwrap().to_reg(), 0xFFFE5556);
    }

    #[test]
    fn mathacl_iqtype_f32_to_register() {
        let mut test_float = 0.0;
        core::assert_eq!(IQType::from_f32(test_float, 15, true).unwrap().to_reg(), 0x0);
        core::assert_eq!(IQType::from_f32(test_float, 16, false).unwrap().to_reg(), 0x0);

        test_float = 1.5;
        core::assert_eq!(IQType::from_f32(test_float, 15, true).unwrap().to_reg(), 0x00018000);
        core::assert_eq!(IQType::from_f32(test_float, 16, false).unwrap().to_reg(), 0x00018000);

        test_float = -1.5;
        core::assert_eq!(IQType::from_f32(test_float, 15, true).unwrap().to_reg(), 0xFFFE8000);

        test_float = 1.666657;
        core::assert_eq!(IQType::from_f32(test_float, 15, true).unwrap().to_reg(), 0x0001AAAA);
        core::assert_eq!(IQType::from_f32(test_float, 16, false).unwrap().to_reg(), 0x0001AAAA);

        test_float = -1.666657;
        core::assert_eq!(IQType::from_f32(test_float, 15, true).unwrap().to_reg(), 0xFFFE5556);
    }

    #[test]
    fn mathacl_iqtype_register_to_signed_f32() {
        let mut test_u32: u32 = 0x7FFFFFFF;

        let mut result = IQType::from_reg(test_u32, 0, true).unwrap().to_f32();
        core::assert!(result < 1.0 + ERROR_TOLERANCE && result > 1.0 - ERROR_TOLERANCE);

        test_u32 = 0x0;
        result = IQType::from_reg(test_u32, 0, true).unwrap().to_f32();
        core::assert!(result < 0.0 + ERROR_TOLERANCE && result > 0.0 - ERROR_TOLERANCE);

        test_u32 = 0x0001AAAA;
        result = IQType::from_reg(test_u32, 15, true).unwrap().to_f32();
        core::assert!(result < 1.666657 + ERROR_TOLERANCE && result > 1.666657 - ERROR_TOLERANCE);

        test_u32 = 0xFFFE5556;
        result = IQType::from_reg(test_u32, 15, true).unwrap().to_f32();
        core::assert!(result < -1.666657 + ERROR_TOLERANCE && result > -1.666657 - ERROR_TOLERANCE);
    }
}
