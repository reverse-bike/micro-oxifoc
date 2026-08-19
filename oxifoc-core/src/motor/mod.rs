//! Reusable synchronous motor-control driver and optional policy layers.

#[cfg(feature = "algorithms")]
pub mod derating;
#[cfg(feature = "algorithms")]
pub mod failsafe;
pub mod foc_driver;

#[cfg(feature = "algorithms")]
pub use failsafe::{FailsafeConfig, FailsafeController, FailsafePolicy};
pub use foc_driver::FocDriver;
