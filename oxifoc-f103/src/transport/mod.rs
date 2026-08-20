//! Synchronous transports used by the recovered controller.

pub mod can;

pub use can::initialize;
#[cfg(feature = "firmware")]
pub use can::service;
#[cfg(feature = "calibration-image")]
pub use can::take_received_frame;
#[cfg(feature = "firmware")]
pub use can::take_reset_request;
