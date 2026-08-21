//! Synchronous, platform-neutral fixed-point field-oriented control.
//!
//! The crate contains the control path shared by the STM32F103 ride and
//! calibration images: coordinate transforms, PI current control, SVPWM,
//! Hall estimation, and the back-EMF phase observer. Hardware access, safety
//! policy, calibration sequencing, and CAN remain in the device crates.

#![cfg_attr(not(test), no_std)]
#![warn(
    clippy::unwrap_used,
    clippy::panic,
    clippy::todo,
    clippy::unimplemented,
    clippy::get_unwrap,
    clippy::unwrap_in_result,
    clippy::panic_in_result_fn
)]

pub mod motor;

pub mod foc {
    pub mod control_types;
    pub mod controller;
    pub mod hall_sensor;
    pub mod numeric;
    pub mod offset_tracker;
    pub mod phase;
    pub mod pi_controller;
    pub mod ramp;
    pub mod svpwm;
    pub mod transforms;
    pub mod trig;

    pub use control_types::{AlphaBeta, Dq, PwmDuty};
    pub use controller::{
        DecouplingModel, FixedDecoupling, FixedFocController, FocController, NoDecoupling,
    };
    pub use numeric::{Fixed, Scalar};
    pub use pi_controller::PIController;
}
