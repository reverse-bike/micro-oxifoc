#![cfg_attr(not(test), no_std)]

//! Synchronous STM32F103 firmware built around the fixed-point OxiFOC core.
//!
//! The current-control ISR uses no floating-point operations. Physical units
//! cross the CAN boundary as explicitly scaled integers; conversion and
//! calibration may use floating point on the host, never in the 16 kHz loop.

pub mod config;
pub mod control;
pub mod protocol;
pub mod sensors;

#[cfg(feature = "firmware")]
pub mod hardware;
#[cfg(feature = "firmware")]
pub mod safety;
#[cfg(feature = "firmware")]
pub mod transport;
