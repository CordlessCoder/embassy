//! Synchronisation primitives the drivers share.
//!
//! Internal: these are shaped by what this HAL's interrupt handlers need, not by what a general async
//! program does. `embassy-sync` is where to look for the latter.

pub(crate) mod irq_waker;
pub(crate) mod linked_waiter;
