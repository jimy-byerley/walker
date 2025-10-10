
use std::{
    rc::Rc,
    cell::RefCell,
};
use esp_idf_hal::{
    peripheral::Peripheral,
    i2c::I2cDriver,
    adc::{self, AdcContDriver, AdcContConfig},
    ledc::{self, LedcDriver, LedcTimerDriver},
    gpio::{self, PinDriver, AnyOutputPin, Output},
};
use anyhow::Result;
use embedded_hal::digital::{OutputPin};
use nalgebra::base::{Vector2, Matrix2, SMatrix};

use crate::as5600::{self, AS5600};


type Float = f32;
const PHASES: usize = 3;

pub struct Foc<'d> {
    enable: PinDriver<'d, AnyOutputPin, Output>,
    power: [LedcDriver<'d>; PHASES],
    current: [AdcContDriver<'d>; PHASES-1],
    position_sensor: AS5600<I2cDriver<'d>>,
    control: TorqueControl,
    transform: SpaceVectorTransform<PHASES>,
    position_offset: Float,
}
pub struct State {
    position: Float,
    force: Float,
    voltages: [Float; PHASES],
    currents: [Float; PHASES],
}
impl<'d> Foc<'d> {
    pub fn new(
        bus: Rc<RefCell<I2cDriver>>,
        
        power_enable: impl Peripheral<P = gpio::OutputPin> + 'd,
        power_timer: impl Peripheral<P = ledc::LedcTimer> + 'd,
        power_channels: [impl Peripheral<P = ledc::LedcChannel> + 'd; PHASES],
        power_pins: [impl Peripheral<P = gpio::OutputPin> + 'd; PHASES],
        
        current_adc: impl Peripheral + 'd,
        current_pins: [impl Peripheral + 'd; PHASES-1],
        ) -> Result<Self>
    {
        let position_sensor = crate::i2c::Slave::new(bus, as5600::ADDRESS);
     
        let enable = PinDriver::output(power_enable)?;
        enable.set_low();
        
        let config = ledc::config::TimerConfig::default()
            .frequency(25.kHz().into())
            .resolution(ledc::Resolution::Bits8);
        let timer = LedcTimerDriver::new(power_timer, &config)?;
        let power = [
            LedcDriver::new(power_channels[0], &timer, power_pins[0])?,
            LedcDriver::new(power_channels[1], &timer, power_pins[1])?,
            LedcDriver::new(power_channels[2], &timer, power_pins[2])?,
            ];
        for pwm in power {
            pwm.set_duty(0);
        }
        
        let config = AdcContConfig::default()
            .sample_freq(2.kHz().into())
            .frame_measurements(100)
            .frame_counts(10);
        let current = [
            AdcContDriver::new(current_adc, &config, adc::Attenuated::db11(current_pins[0]))?,
            AdcContDriver::new(current_adc, &config, adc::Attenuated::db11(current_pins[1]))?,
            ];  
            
        let control = todo!();
        let transform = todo!();
        
        Ok(Self {
            power,
            current,
            position_sensor,
            control,
            transform,
            })
    }
    
    /*
    pub fn disable(&mut self) {
        self.enable.set_low();
        self.control.disable();
    }
    pub async fn step(&mut self, enable: bool, target_torque: Float) -> State {
        self.enable.set_high();
        let (current_position, current_currents) = (
            self.position_sensor.position(),
            self.current_sensors.iter()
                .map(|sensor|  sensor.current())
                .collect::<[_; PHASES]>().join(),
            ).join().await;
        self.transform.set_position(current_position + position_offset);
        let current_field = self.transform.phases_to_field(current_currents);
        
        let (current_torque, target_voltage) = self.control.step(current_field, target_torque);
        
        let target_voltages = self.transform.field_to_phases(target_voltage);
        let target_modulations = clamp_voltage(target_voltages / self.power_voltage, -1, 1);
        self.power.iter().zip(&target_modulations)
            .for_each(|pwm, modulation|  pwm.set_duty_with_hpoint(
                (modulation * Float::from(modulation.get_max_duty())) .try_into().unwrap()
                ));
        State {
            position: current_position,
            currents: current_currents, 
            force: current_torque,
            voltage: target_voltage,
            }
    }
    */
}

struct SpaceVectorTransform<const N: usize> {
    // transformation matrices
    rotor_to_stator: Matrix2<Float>,
    phases_to_stator: SMatrix<Float, 2, N>,
    stator_to_phases: SMatrix<Float, N, 2>,
}
// impl SpaceVectorTransform {
//     fn new() -> Self {
//         let phases = Matrix::<Float, 2, N>::from_array_storage((0 .. N).map(|i| {
//             let angle = (Float::PI * Float::from(i) / Float::from(N));
//             Vector::new(angle.cos(), angle.sin())
//             }).collect());
//         Self {
//             rotor_to_stator: Matrix::full(Float::NAN),
//             phases_to_stator: phases,
//             stator_to_phases: phases.transpose() * (phases * phases.transpose()),
//         }
//     }
//     fn set_position(&mut self, position: Float) {
//         self.rotor_to_stator = Matrix2::from(Rotation2::new(position));
//     }
//     fn phases_to_rotor(&self, phases: [Float; N]) -> Vector2<Float> {
//         self.rotor_to_stator.transpose() * self.phases_to_stator * phases
//     }
//     fn rotor_to_phases(&self, field: Vector2) -> [Float; N] {
//         self.stator_to_phases * self.rotor_to_stator * field
//     }
// }
// 
// /// if voltage out of bounds, center it to reduce saturations, then saturate
// fn clamp_voltage(voltages: [Float; PHASES], saturation: (Float, Float)) -> [Float; PHASES] {
//     
//     let range = (voltages.min(), voltages.max());
//     let target_currents = if range.0 < saturation.0 || range.1 > saturation.1 {
//         let center = (range.1 + range.0) * 0.5;
//         voltages.iter()
//             .map(|voltage|  (voltage - center).clamp(saturation.0, saturation.1))
//             .collect::<[Float; PHASES]>();
//     }
//     else {voltages}
// }

struct TorqueControl {
    rated_torque: Float,
    rated_current: Float,
    phase_inductance: Float,
    phase_resistance: Float,
    power_voltage: Float,
    period: Float,
    gains: CorrectorGain,
    integral: Vector2<Float>,
}
struct CorrectorGain {
    proportional: Float,
    integral: Float,
}
// impl TorqueControl {
//     pub fn new(period: Float, phase_inductance: Float, phase_resistance: Float, power_voltage: Float, gains: CorrectorGain) -> Self {
//         Self {
//             phase_inductance,
//             phase_resistance,
//             power_voltage,
//             period,
//             gains: CorrectorGain {                
//                 proportional: self.continuous_to_discrete_gain(period, gains.proportional),
//                 integral: self.continuous_to_discrete_gain(period, gains.integral),
//                 },
//             integral: Vector2::zero(),
//         }
//     }
//     fn continuous_to_discrete_gain(period: Float, proportional: Float) -> Result<Float, &'static str> {
//         if proportional > 1.   // we consider any phenomenon faster than this control loop to be out of control
//             {return Err("gain is higher than control loop frequency")}
//         // zero order hold coefficient for getting the same result as a continuous correction on a 1st order system
//         Ok( (1. - (-proportional * period).exp()) / period)
//     }
//     
//     pub fn disable(&mut self) {
//         self.integral = Vector2::zero();
//     }
//     /// take target current (relative) and specify target voltage (relative) to reach it
//     pub fn step(&mut self, current_field: Vector2<Float>, target_torque: Float) -> Vector2<Float> {
//         let current_torque = current_field[1] * self.rated_torque / self.rated_current;
//         let target_current = target_torque / self.rated_torque * self.rated_current;
//         let target_field = Vector2::new(0, target_current);
//         
//         let error = target_torque - current_torque;
//         
//         let conversion = self.phase_resistance / self.power_voltage;
//         let mut target_voltage = (error * self.gains.proportional + self.integral * self.gains.integral) * conversion;
//         if target_voltage.abs() > self.power_voltage {
//             self.integral += (self.power_voltage - target_voltage) / self.gains.integral;
//             target_voltage = self.power_voltage;
//         }
//         self.integral += error * self.period;
//         
//         (current_torque, target_voltage)
//     }
// }
