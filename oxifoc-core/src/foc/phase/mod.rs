//! Electrical-phase source management for FOC control.
//!
//! Sensors and estimators implement [`PhaseProvider`]. [`PhaseManager`] owns
//! those providers, validates source selection, and presents one phase stream
//! to the synchronous current loop. The F103 build installs Hall and the
//! fixed-point back-EMF observer as an active hybrid while retaining the
//! source identities needed for later encoder, HFI, manual, and open-loop
//! experiments.

mod manager;
mod observer;
mod provider;
mod source;
#[cfg(feature = "algorithms")]
mod startup;

pub use manager::{ObserverDiagnostics, PhaseManager};
pub use observer::BackEmfObserver;
pub use provider::{PhaseEstimate, PhaseInput, PhaseProvider};
pub use source::{PhaseSource, PhaseSourceError};
#[cfg(feature = "algorithms")]
pub use startup::{SensorlessStartup, StartupOutput, StartupPhase};
