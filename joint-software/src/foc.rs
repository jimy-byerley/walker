
use core::cell::RefCell;
use esp_hal::{
    gpio::{*, interconnect::PeripheralOutput},
    analog::adc::*,
    mcpwm::{*, operator::*},
    Blocking,
    time::Rate,
};
use embedded_hal::{
    i2c::I2c,
    };
use nalgebra::{Matrix, SMatrix, SVector, Vector2, Matrix2, Rotation2};

use crate::as5600::{self, As5600};


type Float = f32;
const PHASES: usize = 3;

pub struct State {
    position: Float,
    force: Float,
    voltages: [Float; PHASES],
    currents: [Float; PHASES],
}

pub struct Foc<'d, I2C, ADC, ADC1, ADC0, PWM> {
    power_enable: Output<'d>,
    power_pins: (
        PwmPin<'d, PWM, 0, true>,
        PwmPin<'d, PWM, 1, true>,
        PwmPin<'d, PWM, 2, true>,
        ),
    adc: Adc<'d, ADC, Blocking>,
    current_pins: (
        AdcPin<ADC0, ADC>,
        AdcPin<ADC1, ADC>,
        ),
    position_sensor: As5600<'d, I2C>,
    control: TorqueControl,
    transform: SpaceVectorTransform<PHASES>,
}
impl<'d, I2C, ADC, ADC1, ADC0, PWM> Foc<'d, I2C, ADC, ADC1, ADC0, PWM> 
where
    I2C: I2c,
    ADC: RegisterAccess,
    ADC0: AdcChannel + AnalogPin,
    ADC1: AdcChannel + AnalogPin,
    PWM: PwmPeripheral + 'd,
{
    pub fn new(
        bus: &'d RefCell<I2C>,
        
        power_enable: impl OutputPin,
        power_pwm: PWM,
        power_pins: (
            impl PeripheralOutput<'d>,
            impl PeripheralOutput<'d>,
            impl PeripheralOutput<'d>,
            ),
        
        current_adc: ADC,
        current_pins: (ADC0, ADC1),
        ) -> Self
    {
        let power_enable = Output::new(power_enable, Level::Low, OutputConfig::default());
        
        let mut mcpwm = McPwm::new(power_pwm, PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap());
        mcpwm.operator0.set_timer(&mcpwm.timer0);
        mcpwm.operator1.set_timer(&mcpwm.timer0);
        mcpwm.operator2.set_timer(&mcpwm.timer0);
        let power_pins = (
            mcpwm.operator0.with_pin_a(power_pins.0, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            mcpwm.operator1.with_pin_a(power_pins.1, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            mcpwm.operator2.with_pin_a(power_pins.2, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            );
        
        let mut config = AdcConfig::new();
        let current_pins = (
            config.enable_pin(current_pins.0, Attenuation::_0dB),
            config.enable_pin(current_pins.1, Attenuation::_0dB),
            );
        let adc = Adc::new(current_adc, config);
     
        let position_sensor = As5600::new(bus);
        
        let control = todo!();
        let transform = todo!();
        
        Self {
            power_enable,
            power_pins,
            adc,
            current_pins,
            position_sensor,
            control,
            transform,
            }
    }
    
    
    pub fn disable(&mut self) {
        self.power_enable.set_low();
        self.control.disable();
    }
    async fn get_position(&mut self) -> Float {
        self.position_sensor.angle().await.unwrap()
    }
    async fn get_currents(&mut self) -> SVector<Float, PHASES> {
        let mut i0 = None;
        let mut i1 = None;
        while i0.is_none() && i1.is_none() {
            if i0.is_none() {
                i0 = self.adc.read_oneshot(&mut self.current_pins.0).ok();
            }
            if i1.is_none() {
                i1 = self.adc.read_oneshot(&mut self.current_pins.1).ok();
            }
        }
        let i0 = i0.unwrap();
        let i1 = i1.unwrap();
        [i0, i1, -i0-i1].into()
    }
    fn set_modulations(&mut self, modulations: SVector<Float, PHASES>) {
        self.power_pins.0.set_timestamp((modulations[0] * Float::from(self.power_pins.0.period())) as _);
        self.power_pins.1.set_timestamp((modulations[1] * Float::from(self.power_pins.1.period())) as _);
        self.power_pins.2.set_timestamp((modulations[2] * Float::from(self.power_pins.2.period())) as _);
    }
    pub async fn step(&mut self, enable: bool, target_torque: Float) -> State {
        self.power_enable.set_high();
        let current_position = self.get_position().await;
        let current_currents = self.get_currents().await;
        self.transform.set_position(current_position + position_offset);
        let current_field = self.transform.phases_to_rotor(current_currents);
        
        let (current_torque, target_voltage) = self.control.step(current_field, target_torque);
        
        let target_voltages = self.transform.rotor_to_phases(target_voltage);
        let target_modulations = clamp_voltage(target_voltages / self.power_voltage, -1, 1);
        self.set_modulations(target_modulations);
        State {
            position: current_position,
            currents: current_currents, 
            force: current_torque,
            voltage: target_voltage,
            }
    }

}

struct SpaceVectorTransform<const N: usize> {
    // transformation matrices
    rotor_to_stator: Matrix2<Float>,
    phases_to_stator: SMatrix<Float, 2, N>,
    stator_to_phases: SMatrix<Float, N, 2>,
}
impl<const N: usize> SpaceVectorTransform<N> {
    fn new() -> Self {
        let phases = SMatrix::<Float, 2, N>::from_array_storage((0 .. N).map(|i| {
            let angle = Float::consts::PI * (i as Float) / (N as Float);
            Vector2::new(angle.cos(), angle.sin())
            }).collect());
        Self {
            rotor_to_stator: Matrix::from_element(Float::NAN),
            phases_to_stator: phases,
            stator_to_phases: phases.transpose() * (phases * phases.transpose()),
        }
    }
    fn set_position(&mut self, position: Float) {
        self.rotor_to_stator = Matrix2::from(Rotation2::new(position));
    }
    fn phases_to_rotor(&self, phases: SVector<Float, N>) -> Vector2<Float> {
        self.rotor_to_stator.transpose() * self.phases_to_stator * phases
    }
    fn rotor_to_phases(&self, field: Vector2<Float>) -> SVector<Float, N> {
        &self.stator_to_phases * (&self.rotor_to_stator * field)
    }
}

/// if voltage out of bounds, center it to reduce saturations, then saturate
fn clamp_voltage(voltages: SVector<Float, PHASES>, saturation: (Float, Float)) -> SVector<Float, PHASES> {
    
    let range = (voltages.min(), voltages.max());
    if range.0 < saturation.0 || range.1 > saturation.1 {
        let center = (range.1 + range.0) * 0.5;
        voltages.iter()
            .map(|voltage|  (voltage - center).clamp(saturation.0, saturation.1))
            .collect::<_>()
    }
    else {voltages}
}

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
impl TorqueControl {
    pub fn new(period: Float, phase_inductance: Float, phase_resistance: Float, power_voltage: Float, gains: CorrectorGain) -> Self {
        Self {
            phase_inductance,
            phase_resistance,
            power_voltage,
            period,
            gains: CorrectorGain {                
                proportional: Self::continuous_to_discrete_gain(period, gains.proportional),
                integral: Self::continuous_to_discrete_gain(period, gains.integral),
                },
            integral: Vector2::zero(),
        }
    }
    fn continuous_to_discrete_gain(period: Float, proportional: Float) -> Result<Float, &'static str> {
        if proportional > 1.   // we consider any phenomenon faster than this control loop to be out of control
            {return Err("gain is higher than control loop frequency")}
        // zero order hold coefficient for getting the same result as a continuous correction on a 1st order system
        Ok( (1. - (-proportional * period).exp()) / period)
    }
    
    pub fn disable(&mut self) {
        self.integral = Vector2::zero();
    }
    /// take target current (relative) and specify target voltage (relative) to reach it
    pub fn step(&mut self, current_field: Vector2<Float>, target_torque: Float) -> Vector2<Float> {
        let current_torque = current_field[1] * self.rated_torque / self.rated_current;
        let target_current = target_torque / self.rated_torque * self.rated_current;
        let target_field = Vector2::new(0, target_current);
        
        let error = target_torque - current_torque;
        
        let conversion = self.phase_resistance / self.power_voltage;
        let mut target_voltage = (error * self.gains.proportional + self.integral * self.gains.integral) * conversion;
        if target_voltage.abs() > self.power_voltage {
            self.integral += (self.power_voltage - target_voltage) / self.gains.integral;
            target_voltage = self.power_voltage;
        }
        self.integral += error * self.period;
        
        (current_torque, target_voltage)
    }
}
