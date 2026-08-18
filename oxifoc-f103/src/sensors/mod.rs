//! Local analog, digital, Hall-adjacent, and wheel sensing.

#[cfg(feature = "firmware")]
pub mod analog;
pub mod environment;
pub mod inputs;
pub mod throttle;
pub mod wheel;
#[cfg(feature = "firmware")]
pub mod wheel_capture;

#[cfg(feature = "firmware")]
pub use analog::{InputMonitor, Snapshot, bus_voltage_mv, latest};
