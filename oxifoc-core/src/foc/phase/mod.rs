//! Electrical-phase source management for synchronous FOC control.
//!
//! Sensors and estimators implement PhaseProvider. PhaseManager owns the Hall
//! sensor and optional back-EMF observer, validates source selection, and
//! presents one phase stream to the current loop.

mod manager;
mod observer;
mod provider;
mod source;

pub use manager::{ObserverDiagnostics, PhaseManager};
pub use observer::BackEmfObserver;
pub use provider::{PhaseEstimate, PhaseInput, PhaseProvider};
pub use source::{PhaseSource, PhaseSourceError};
