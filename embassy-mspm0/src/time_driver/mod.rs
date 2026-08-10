//! The `embassy-time-driver` implementation.
//!
//! One driver, backed by a general-purpose timer's counter and one of its capture/compare channels.
//! A basic timer cannot stand in for it — see `build.rs`'s `TIME_DRIVER_TIMERS` — and no device needs
//! it to, every part with a TIMB having a TIMA and a TIMG as well.

mod tim;
pub(crate) use tim::*;
