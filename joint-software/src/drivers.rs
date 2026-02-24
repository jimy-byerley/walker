
use esp_hal::{
    gpio::{*, interconnect::PeripheralOutput},
    analog::adc::*,
    mcpwm::{*, operator::*, timer::PwmWorkingMode},
    Blocking,
    time::Rate,
    };
use embedded_hal_async::i2c::I2c;
use futures_concurrency::future::Join;
use num_traits::float::Float as FloatTrait;
use vecmat::Vector;
use esp_println::dbg;

use crate::prelude::*;
use crate::{
    registers::ControlError,
    nb_task,
    };



/// rotor position with AS5600 and no current sensing
pub struct MksDualFocV32<'d, PWM> {
    power_enable: Output<'d>,
    power_pins: (
        PwmPin<'d, PWM, 0, true>,
        PwmPin<'d, PWM, 1, true>,
        PwmPin<'d, PWM, 2, true>,
        ),
    modulation_to_current: Float,
    modulation_to_voltage: Float,
    current_estimation: Vector<Float, PHASES>,
    voltage_estimation: Vector<Float, PHASES>,
}
impl<'d, PWM> 
MksDualFocV32<'d, PWM> 
where PWM: PwmPeripheral + 'd,
{
    pub fn new(
        power_enable: impl OutputPin + 'd,
        power_pwm: PWM,
        power_pins: (
            impl PeripheralOutput<'d>,
            impl PeripheralOutput<'d>,
            impl PeripheralOutput<'d>,
            ),
            
        phases_resistance: Float,
        power_voltage: Float,
        ) -> Self
    {
        let power_enable = Output::new(power_enable, Level::Low, OutputConfig::default());
        
        let config = PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
        let mut mcpwm = McPwm::new(power_pwm, config);
        mcpwm.operator0.set_timer(&mcpwm.timer0);
        mcpwm.operator1.set_timer(&mcpwm.timer0);
        mcpwm.operator2.set_timer(&mcpwm.timer0);
        let power_pins = (
            mcpwm.operator0.with_pin_a(power_pins.0, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            mcpwm.operator1.with_pin_a(power_pins.1, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            mcpwm.operator2.with_pin_a(power_pins.2, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            );
        let timer_clock_cfg = config
            .timer_clock_with_frequency(99, PwmWorkingMode::UpDown, Rate::from_khz(20)).unwrap(); 
        mcpwm.timer0.start(timer_clock_cfg);
     
        Self {
            power_enable,
            power_pins,
            modulation_to_current: power_voltage / phases_resistance,
            modulation_to_voltage: power_voltage,
            current_estimation: Vector::zero(),
            voltage_estimation: Vector::zero(),
            }
    }
    pub async fn measure(&mut self) -> Result<(Vector<Float, PHASES>, Vector<Float, PHASES>), ControlError> {
        Ok((self.current_estimation, self.voltage_estimation))
    }
    pub fn modulate(&mut self, modulations: Vector<Float, PHASES>) {
        self.power_enable.set_high();
        self.power_pins.0.set_timestamp((modulations[0] * Float::from(self.power_pins.0.period())).round() as _);
        self.power_pins.1.set_timestamp((modulations[1] * Float::from(self.power_pins.1.period())).round() as _);
        self.power_pins.2.set_timestamp((modulations[2] * Float::from(self.power_pins.2.period())).round() as _);
        self.current_estimation = modulations * self.modulation_to_current;
        self.voltage_estimation = modulations * self.modulation_to_voltage;
    }
    pub fn disable(&mut self) {
        self.power_enable.set_low();
        self.current_estimation = Vector::zero();
        self.voltage_estimation = Vector::zero();
        // TODO remove when disable is fixed
        self.modulate(Vector::zero());
    }
}



/// rotor position with AS5600 and two phases current sensing
pub struct MksDualFocV33<'d, ADC, ADC1, ADC0, PWM> {
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
    modulation_to_voltage: Float,
    voltage_estimation: Vector<Float, PHASES>,
    adc_to_current: Float,
    adc_zero: Vector<Float, 2>,
}
impl<'d, ADC, ADC1, ADC0, PWM> 
MksDualFocV33<'d, ADC, ADC1, ADC0, PWM> 
where
    ADC: RegisterAccess + 'd,
    ADC0: AdcChannel + AnalogPin,
    ADC1: AdcChannel + AnalogPin,
    PWM: PwmPeripheral + 'd,
{
    pub fn new(
        power_enable: impl OutputPin + 'd,
        power_pwm: PWM,
        power_pins: (
            impl PeripheralOutput<'d>,
            impl PeripheralOutput<'d>,
            impl PeripheralOutput<'d>,
            ),
        current_adc: ADC,
        current_pins: (ADC0, ADC1),
        
        power_voltage: Float,
        ) -> Self
    {
        let power_enable = Output::new(power_enable, Level::Low, OutputConfig::default());
        
        let config = PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
        let mut mcpwm = McPwm::new(power_pwm, config);
        mcpwm.operator0.set_timer(&mcpwm.timer0);
        mcpwm.operator1.set_timer(&mcpwm.timer0);
        mcpwm.operator2.set_timer(&mcpwm.timer0);
        let power_pins = (
            mcpwm.operator0.with_pin_a(power_pins.0, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            mcpwm.operator1.with_pin_a(power_pins.1, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            mcpwm.operator2.with_pin_a(power_pins.2, PwmPinConfig::UP_DOWN_ACTIVE_HIGH),
            );
        let timer_clock_cfg = config
            .timer_clock_with_frequency(99, PwmWorkingMode::UpDown, Rate::from_khz(20)).unwrap(); 
        mcpwm.timer0.start(timer_clock_cfg);
        
        let mut config = AdcConfig::new();
        let current_pins = (
            config.enable_pin(current_pins.0, Attenuation::_11dB),
            config.enable_pin(current_pins.1, Attenuation::_11dB),
            );
        let adc = Adc::new(current_adc, config);
        
        let max_adc_voltage = 2.45; // adc max on esp32 with 0db attenuation
        let amplifier_gain = 50.; // INA181A2 gain
        let amplifier_ref = (0. + 3.3)*0.5; // voltage received from amplifier when current is zero
        let resistor = 0.01; // sense resistor ohm
     
        Self {
            power_enable,
            power_pins,
            adc,
            current_pins,
            modulation_to_voltage: power_voltage,
            voltage_estimation: Vector::zero(),
            adc_to_current: max_adc_voltage / (amplifier_gain * resistor),
            adc_zero: Vector::fill(amplifier_ref),
            }
    }
    
    /// measure the given amount of current samples on all pins and return the average
    async fn current_samples(&mut self, samples: u16) -> Vector<Float, 2> {   
        let max_adc_value: u16 = (1<<12) -1; // adc precision is 12 bit
        
        let mut i = Vector::<Float, 2>::zero();
        for _ in 0 .. samples {
            i += Vector::from([
                nb_task!(self.adc.read_oneshot(&mut self.current_pins.0)).await.unwrap(), 
                nb_task!(self.adc.read_oneshot(&mut self.current_pins.1)).await.unwrap(),
                ]).map(Float::from);
        }
        i / (Float::from(samples) * Float::from(max_adc_value))
    }
    
    /// calibrate zero current offsets with each current amplifier
    pub async fn calibrate(&mut self) {
        self.disable();
        let measure = self.current_samples(100).await;
        dbg!(measure);
        self.adc_zero = self.adc_to_current * measure;
    }
    
    pub async fn measure(&mut self) -> Result<(Vector<Float,PHASES>, Vector<Float,PHASES>), ControlError> {
        let i = self.current_samples(3).await * self.adc_to_current - self.adc_zero;
        Ok((
            Vector::from([i[0], i[1], -i[0]-i[1]]),
            self.voltage_estimation,
            ))
    }
    pub fn modulate(&mut self, modulations: Vector<Float, PHASES>) {
        self.power_enable.set_high();
        self.power_pins.0.set_timestamp((modulations[0] * Float::from(self.power_pins.0.period()-1)).round() as _);
        self.power_pins.1.set_timestamp((modulations[1] * Float::from(self.power_pins.1.period()-1)).round() as _);
        self.power_pins.2.set_timestamp((modulations[2] * Float::from(self.power_pins.2.period()-1)).round() as _);
        self.voltage_estimation = modulations * self.modulation_to_voltage;
    }
    pub fn disable(&mut self) {
        self.power_enable.set_low();
        self.voltage_estimation = Vector::zero();
        // TODO remove when disable is fixed
        self.modulate(Vector::zero());
    }
}
