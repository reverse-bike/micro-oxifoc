//! Local analog, digital, Hall-adjacent, and wheel sensing.

#[cfg(feature = "board")]
pub mod analog;
pub mod environment;
pub mod inputs;
pub mod throttle;
pub mod wheel;
#[cfg(feature = "board")]
pub mod wheel_capture;

#[cfg(feature = "board")]
pub use analog::{InputMonitor, Snapshot, bus_voltage_mv, latest};
