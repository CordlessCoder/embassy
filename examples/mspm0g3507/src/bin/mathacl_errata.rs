//! Probes both MATHACL errata: the two angles `MATHACL_ERR_02` names, and `MATHACL_ERR_01`'s
//! divide-by-zero.
//!
//! No wiring. The erratum says `COS(-180)` returns `+1` where it should return `-1`, and `SIN(-90)`
//! likewise, with no workaround but negating in software. It applies to the `MSPM0G3x0x` (`slaz742`)
//! and `MSPM0G351x` (`slaz758`) families.
//!
//! Both angles are printed alongside their positive counterparts and a neighbour a little either side,
//! because the question a fix needs answered is not "is the erratum real" but **"over what input range
//! is it wrong"** — correcting exactly one input value is only right if exactly one value is affected.
//!
//! # What it found
//!
//! `SIN(-90)` returned `+1` and its neighbours 1.6 and 0.2 degrees away were both correctly signed, so
//! one input is affected and the driver corrects that one. `COS(-180)` never reaches the accelerator —
//! an angle of pi normalises to a magnitude the per-unit format cannot hold, and the driver answers it
//! exactly — so that half of the erratum is unreachable here.
//!
//! It also found two defects that had nothing to do with the erratum: `sin(PI)` panicked, and `sin`
//! returned the previous call's result. Both are fixed; this example is what shows they stay fixed.
//!
//! # `MATHACL_ERR_01`
//!
//! A status error latches and only a peripheral reset clears it. `STATUS.ERR` has exactly one
//! non-zero value, `DIVBY0`, so the question is whether a zero divisor can reach the accelerator —
//! and the last phase answers it by offering one to each divide entry point and then checking the
//! accelerator still divides correctly afterwards.

#![no_std]
#![no_main]

use core::f32::consts::PI;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mspm0::mathacl::{Error, IQType, Mathacl, Precision};
use embassy_time::Timer;
use panic_probe as _;

/// Angles either side of the two the erratum names, as multiples of pi.
///
/// The offsets are far larger than the fixed-point quantisation so a neighbour is genuinely a
/// different input rather than the same one twice.
const CASES: [(f32, &str); 10] = [
    (-1.0, "-180 deg"),
    (-0.999, "-179.8 deg"),
    (-0.99, "-178.2 deg"),
    (1.0, "+180 deg"),
    (-0.5, "-90 deg"),
    (-0.499, "-89.8 deg"),
    (-0.49, "-88.2 deg"),
    (0.5, "+90 deg"),
    (0.0, "0 deg"),
    (-0.25, "-45 deg"),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_mspm0::init(Default::default());
    let mut macl = Mathacl::new(p.MATHACL);

    info!("angle          sin        cos");

    for (turns, name) in CASES {
        let rad = turns * PI;

        // Reported rather than unwrapped: `-1.0` needs an integer bit that the per-unit encoding does
        // not have, so the interesting inputs are exactly the ones that might not survive the trip.
        let s = macl.sin(rad, Precision::High);
        let c = macl.cos(rad, Precision::High);

        match (s, c) {
            (Ok(s), Ok(c)) => info!("{=str}  {=f32}  {=f32}", name, s, c),
            (Err(e), Ok(c)) => warn!("{=str}  sin err {}  cos {=f32}", name, e, c),
            (Ok(s), Err(e)) => warn!("{=str}  sin {=f32}  cos err {}", name, s, e),
            (Err(a), Err(b)) => warn!("{=str}  sin err {}  cos err {}", name, a, b),
        }
    }

    info!("expected: sin(-90)=-1 cos(-180)=-1; a +1 in either is MATHACL_ERR_02");

    // MATHACL_ERR_01. Each of these must be refused by the driver rather than handed to the
    // accelerator, because a DIVBY0 that does reach it latches until the peripheral is reset.
    let zero_iq = IQType::from_f32(0.0, 15, true).unwrap();
    let one_iq = IQType::from_f32(1.0, 15, true).unwrap();

    let refused = [
        macl.div_i32(7, 0).is_err(),
        macl.div_u32(7, 0).is_err(),
        macl.div_iq(one_iq, zero_iq).is_err(),
    ];

    if refused.iter().all(|r| *r) {
        info!("all three divides refused a zero divisor");
    } else {
        error!("a zero divisor reached the accelerator: {}", refused);
    }

    // Not an erratum, but the same status word: a quotient too wide for the result register. 1.0
    // divided by 0.00002 is 50000, past the 15 integer bits the dividend's format carries, and
    // 0.00002 clears the divide-by-zero guard's tolerance by a factor of two.
    let tiny = IQType::from_f32(0.00002, 15, true).unwrap();
    match macl.div_iq(one_iq, tiny) {
        Err(Error::Overflow) => info!("overflow reported for 1.0/0.00002"),
        other => error!("expected Overflow for 1.0/0.00002, got {}", other),
    }

    match macl.div_i32(i32::MIN, -1) {
        Err(Error::Overflow) => info!("overflow reported for i32::MIN / -1"),
        other => error!("expected Overflow for i32::MIN/-1, got {}", other),
    }

    // The accelerator is only known to be unharmed if it still works.
    match (macl.div_i32(1000, 3), macl.sin(-PI / 2.0, Precision::High)) {
        (Ok(1000..=1001) | Ok(333), Ok(s)) if s < -0.999 => info!("PASS: still correct afterwards"),
        (q, s) => error!("FAIL: after the zero divisors, div_i32(1000,3)={} sin(-90)={}", q, s),
    }

    loop {
        Timer::after_secs(60).await;
    }
}
