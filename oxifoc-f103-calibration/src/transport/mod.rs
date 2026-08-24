//! CAN transport for the calibration application.

pub mod can;

pub use can::{initialize, take_received_frame};
