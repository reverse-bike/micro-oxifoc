#![cfg_attr(not(test), no_std)]

//! Synchronous calibration application built on the fixed-point OxiFOC path.

pub mod calibration;
pub mod config;
pub mod protocol;

#[cfg(feature = "firmware")]
pub mod app;
#[cfg(feature = "firmware")]
pub mod control;
#[cfg(feature = "firmware")]
pub mod transport;
