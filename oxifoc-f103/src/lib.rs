#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "firmware", feature(optimize_attribute))]

//! Synchronous STM32F103 firmware built around the fixed-point OxiFOC core.
//!
//! The current-control ISR uses no floating-point operations. Physical units
//! cross the CAN boundary as explicitly scaled integers; conversion and
//! calibration may use floating point on the host, never in the 16 kHz loop.

pub mod config;
pub mod control;
pub mod protocol;
pub mod safety;
pub mod sensors;

#[cfg(feature = "board")]
pub mod hardware;
#[cfg(feature = "board")]
pub mod transport;
