//! Synchronous transports used by the recovered controller.

pub mod can;

#[cfg(feature = "firmware")]
mod ride;
#[cfg(feature = "firmware")]
pub use ride::{RideCan, RuntimeTelemetry, receive_interrupt};
