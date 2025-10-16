
use core::cell::RefCell;
use esp_hal::{
    gpio::{*, interconnect::PeripheralOutput},
    analog::adc::*,
    mcpwm::{*, operator::*},
    Blocking,
    time::Rate,
};
use embedded_hal_async::i2c::I2c;
use nalgebra::{Matrix, SMatrix, SVector, Vector2, Matrix2, Rotation2};
use num_traits::float::Float as FloatTrait;

use crate::{
    as5600::{self, As5600},
    nb_task,
    foc::Driver,
    };

type Float = f32;
const PHASES: usize = 3;



pub struct MKSDualFoc<'d, I2C, ADC, ADC1, ADC0, PWM> {
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
}
impl<'d, I2C, ADC, ADC1, ADC0, PWM> 
MKSDualFoc<'d, I2C, ADC, ADC1, ADC0, PWM> 
where
    I2C: I2c,
    ADC: RegisterAccess + 'd,
    ADC0: AdcChannel + AnalogPin,
    ADC1: AdcChannel + AnalogPin,
    PWM: PwmPeripheral + 'd,
{
    pub fn new(
        bus: &'d RefCell<I2C>,
        
        power_enable: impl OutputPin + 'd,
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
        
        Self {
            power_enable,
            power_pins,
            adc,
            current_pins,
            position_sensor,
            }
    }
}
impl<'d, I2C, ADC, ADC1, ADC0, PWM> 
Driver for MKSDualFoc<'d, I2C, ADC, ADC1, ADC0, PWM> 
where
    I2C: I2c,
    ADC: RegisterAccess + 'd,
    ADC0: AdcChannel + AnalogPin,
    ADC1: AdcChannel + AnalogPin,
    PWM: PwmPeripheral + 'd,
{
    async fn get_position(&mut self) -> Float {
        self.position_sensor.angle().await.unwrap()
    }
    async fn get_currents(&mut self) -> SVector<Float, PHASES> {
        let i0 = nb_task!(self.adc.read_oneshot(&mut self.current_pins.0)).await.unwrap();
        let i1 = nb_task!(self.adc.read_oneshot(&mut self.current_pins.1)).await.unwrap();
        let max_adc_voltage = 1.1; // adc max on esp32 with 0db attenuation
        let i0 = Float::from(i0) / Float::from(i16::MAX) * max_adc_voltage;
        let i1 = Float::from(i1) / Float::from(i16::MAX) * max_adc_voltage;
        [i0, i1, -i0-i1].into()
    }
    fn set_modulations(&mut self, modulations: SVector<Float, PHASES>) {
        self.power_enable.set_high();
        self.power_pins.0.set_timestamp((modulations[0] * Float::from(self.power_pins.0.period())).round() as _);
        self.power_pins.1.set_timestamp((modulations[1] * Float::from(self.power_pins.1.period())).round() as _);
        self.power_pins.2.set_timestamp((modulations[2] * Float::from(self.power_pins.2.period())).round() as _);
    }
    fn disable(&mut self) {
        self.power_enable.set_low();
    }
}

