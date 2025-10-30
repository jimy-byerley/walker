use bilge::prelude::*;
use packbytes::{FromBytes, ToBytes};
use crate::artcat::Register;
use crate::pack_bilge;


pub mod current {
    use super::*;
    
    pub const status: Register<Status> = Register::new(0x300);
    pub const error: Register<ControlError> = Register::new(0x301);
    pub const position: Register<u32> = Register::new(304);
    pub const velocity: Register<u32> = Register::new(308);
    pub const force: Register<u32> = Register::new(312);
    pub const currents: Register<[u16; 3]> = Register::new(316);
    pub const voltages: Register<[u16; 3]> = Register::new(322);
}
pub mod target {
    use super::*;
    
    pub const control: Register<Control> = Register::new(0x400);
    /// affine force reaction
    pub const force: Register<[f32; 3]> = Register::new(0x401);
    pub const force_constant: Register<f32> = Register::new(0x401);
    pub const force_position: Register<f32> = Register::new(0x405);
    pub const force_velocity: Register<f32> = Register::new(0x409);
    /// min/max force allowed on the motor
    pub const limit_force: Register<Range> = todo!();
    /// min/max velocity, force going against will be clamped
    pub const limit_velocity: Register<Range> = todo!();
    /// min/max position, force going against will be clamped
    pub const limit_position: Register<Range> = todo!();
}


#[bitsize(8)]
#[derive(Copy, Clone, FromBits, PartialEq, Default)]
pub struct Control {
    reset: bool,
    power: bool,
    calibrate: bool,
    _reserved: u5,
}
pack_bilge!(Control);

#[bitsize(8)]
#[derive(Copy, Clone, FromBits, PartialEq, Default)]
pub struct Status {
    fault: bool,
    powered: bool,
    calibrated: bool,
    _reserved: u6,
}
pack_bilge!(Status);

#[derive(Copy, Clone, FromBytes, ToBytes)]
pub struct Range {
    start: f32,
    stop: f32,
}

#[repr(u8)]
#[derive(Copy, Clone, Default)]
pub enum ControlError {
    #[default]
    None = 0,
    
    OverCurrent = 0,
    OverVoltage = 1,
    
    PositionSensorNotDetected = 10,
    PositionMagnetNotDetected = 11,
}
