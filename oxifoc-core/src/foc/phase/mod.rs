//! Electrical-phase source management for FOC control.
//!
//! Sensors and estimators implement [`PhaseProvider`]. [`PhaseManager`] owns
//! those providers, validates source selection, and presents one phase stream
//! to the synchronous current loop. The F103 build installs only its Hall
//! sensor today while retaining the source identities needed for later
//! encoder, observer, HFI, hybrid, manual, and open-loop experiments.

mod manager;
#[cfg(feature = "algorithms")]
mod observer;
mod provider;
mod source;
#[cfg(feature = "algorithms")]
mod startup;

pub use manager::PhaseManager;
#[cfg(feature = "algorithms")]
pub use observer::{
    BackEmfObserver, DEFAULT_CENTERING_GAIN, DEFAULT_LAMBDA_GAIN, Observer, ObserverInput,
};
#[cfg(feature = "hfi")]
pub use observer::{HFI_DEFAULT_AMPLITUDE_RATIO, HFI_DEFAULT_FREQ_HZ, HfiObserver};
pub use provider::{PhaseEstimate, PhaseInput, PhaseProvider};
pub use source::{PhaseSource, PhaseSourceError};
#[cfg(feature = "algorithms")]
pub use startup::{SensorlessStartup, StartupOutput, StartupPhase};
