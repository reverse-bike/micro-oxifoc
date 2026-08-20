//! Shared synchronous calibration control and failure types.

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum Routine {
    #[default]
    None = 0,
    Resistance = 1,
    Inductance = 2,
    FluxLinkage = 3,
    Hall = 4,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum Failure {
    #[default]
    None = 0,
    Stopped = 1,
    LocalInterlock = 2,
    CurrentSample = 3,
    PhaseOvercurrent = 4,
    HardwareFault = 5,
    PwmOutput = 6,
    ControlTiming = 7,
    CurrentDidNotSettle = 8,
    InvalidSlope = 9,
    BusVoltage = 10,
    MissingPrerequisite = 11,
    PulseResponse = 12,
    InductanceRange = 13,
    MotorNotResponding = 14,
    FluxRange = 15,
    HallStates = 16,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Actuation {
    #[default]
    Off,
    Current {
        angle: u32,
        direct_counts: i16,
        quadrature_counts: i16,
    },
    DirectVoltage {
        angle: u32,
        direct_tick_bits: i32,
    },
}
