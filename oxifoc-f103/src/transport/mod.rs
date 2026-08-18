//! Synchronous transports used by the recovered controller.

pub mod can;

pub use can::{initialize, service, take_reset_request};
