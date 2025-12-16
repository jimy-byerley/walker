use bilge::prelude::*;
use packbytes::{FromBytes, ToBytes};
use uartcat::registers::{Register, SlaveRegister};
use uartcat::{pack_enum, pack_bilge};

/// current state of actuator and control loop
pub mod current {
    use super::*;
    
    pub const STATUS: SlaveRegister<Status> = Register::new(0x500);
    pub const ERROR: SlaveRegister<ControlError> = Register::new(0x501);
    pub const POSITION: SlaveRegister<f32> = Register::new(0x504);
    pub const VELOCITY: SlaveRegister<f32> = Register::new(0x508);
    pub const FORCE: SlaveRegister<f32> = Register::new(0x50c);
    pub const CURRENTS: SlaveRegister<Phases> = Register::new(0x510);
    pub const VOLTAGES: SlaveRegister<Phases> = Register::new(0x516);
}
/// target and settings for control loop
pub mod target {
    use super::*;
    
    pub const MODE: SlaveRegister<Mode> = Register::new(0x540);
    /// affine force reaction
    pub const FORCE: SlaveRegister<f32> = Register::new(0x544);
    pub const FORCE_CONSTANT: SlaveRegister<f32> = Register::new(0x560);
    pub const FORCE_POSITION: SlaveRegister<f32> = Register::new(0x564);
    pub const FORCE_VELOCITY: SlaveRegister<f32> = Register::new(0x568);
    /// min/max force allowed on the motor
    pub const LIMIT_FORCE: SlaveRegister<Range> = Register::new(0x56c);
    /// min/max velocity, force going against will be clamped
    pub const LIMIT_VELOCITY: SlaveRegister<Range> = Register::new(0x574);
    /// min/max position, force going against will be clamped
    pub const LIMIT_POSITION: SlaveRegister<Range> = Register::new(0x57c);
}
/// characteristics of actuator and driver
pub mod typical {
    use super::*;
    
    pub const RATED_FORCE: SlaveRegister<f32> = Register::new(0x5ae);
    pub const RATED_CURRENT: SlaveRegister<f32> = Register::new(0x5ae);
    pub const MAX_VOLTAGE: SlaveRegister<f32> = Register::new(0x5a2);
    pub const MAX_CURRENT: SlaveRegister<f32> = Register::new(0x5a6);
    pub const MAX_FORCE: SlaveRegister<f32> = Register::new(0x5aa);
}
pub const END: usize = 0x5d0;

pub const CURRENT_UNIT: f32 = 1. / (1u16<<12) as f32;
pub const VOLTAGE_UNIT: f32 = 1. / (1u16<<12) as f32;


#[bitsize(8)]
#[derive(Copy, Clone, FromBits, PartialEq, Default, Debug)]
pub enum Mode {
    #[default]
    #[fallback]
    Off = 0,
    Control = 1,
    CalibrateImpedance = 2,
    CalibrateFocConstant = 3,
    CalibrateFocVibrations = 4,
}
pack_enum!(Mode);

#[bitsize(8)]
#[derive(Copy, Clone, FromBits, PartialEq, Default, DebugBits)]
pub struct Status {
    pub fault: bool,
    pub powered: bool,
    pub calibrated: bool,
    _reserved: u5,
}
pack_bilge!(Status);

#[derive(Copy, Clone, FromBytes, ToBytes, Debug)]
pub struct Range {
    pub start: f32,
    pub stop: f32,
}

#[derive(Copy, Clone, FromBytes, ToBytes, Debug)]
pub struct Phases {
    pub phases: [u16; 3],
}
impl From<[u16; 3]> for Phases {
    fn from(phases: [u16; 3]) -> Self {Self {phases}}
}

#[bitsize(8)]
#[derive(FromBits, Copy, Clone, Default, Debug, PartialEq)]
pub enum ControlError {
    #[default]
    None = 0,
    #[fallback]
    Unknown = 255,
    
    OverCurrent = 1,
    OverVoltage = 2,
    
    PositionSensorNotDetected = 10,
    PositionMagnetNotDetected = 11,
    
    CalibrationFailed = 20,
}
pack_enum!(ControlError);
