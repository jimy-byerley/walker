
use core::{
    cell::RefCell,
    future::Future,
    };
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

use crate::{
    as5600::{self, As5600},
    foc::{Driver, Measure, ControlError},
    nb_task,
    };

type Float = f32;
const PHASES: usize = 3;


/*
/// rotor position with AS5600 and two phases current sensing
pub struct Mk1<'d, I2C, ADC, ADC1, ADC0, PWM> {
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
Mk1<'d, I2C, ADC, ADC1, ADC0, PWM> 
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
Driver for Mk1<'d, I2C, ADC, ADC1, ADC0, PWM> 
where
    I2C: I2c,
    ADC: RegisterAccess + 'd,
    ADC0: AdcChannel + AnalogPin,
    ADC1: AdcChannel + AnalogPin,
    PWM: PwmPeripheral + 'd,
{
    fn check(&mut self) -> impl Future<Output=ControlError> {
        async {
            match self.position_sensor.check().await {
                as5600::Error::I2c(_) => ControlError::PositionSensorNotDetected,
                as5600::Error::Sensor(_) => ControlError::PositionMagnetNotDetected,
            }
        }
    }
    fn measure(&mut self) -> impl Future<Output=Measure> {
        async {
            let (position, currents) = (
                async {
        //             self.position_sensor.check().await.map_err(|e|  dbg!(e)).ok();
                    self.position_sensor.angle().await.unwrap()
                },
                async {
                    let i0 = nb_task!(self.adc.read_oneshot(&mut self.current_pins.0)).await.unwrap();
                    let i1 = nb_task!(self.adc.read_oneshot(&mut self.current_pins.1)).await.unwrap();
                    let max_adc_value = (1<<12) -1;
                    let max_adc_voltage = 2.45; // adc max on esp32 with 0db attenuation
                    let amplifier = 50.; // INA181A2 gain
                    let resistor = 0.01; // sense resistor ohm
                    let i0 = Float::from(max_adc_value - i0) / Float::from(max_adc_value) * max_adc_voltage / (amplifier * resistor);
                    let i1 = Float::from(max_adc_value - i1) / Float::from(max_adc_value) * max_adc_voltage / (amplifier * resistor);
                    [i0, i1, -i0-i1]
                },
            ).join().await;
            Measure {position, currents}
        }
    }
    fn modulate(&mut self, modulations: [Float; PHASES]) {
        self.power_enable.set_high();
        self.power_pins.0.set_timestamp((modulations[0] * Float::from(self.power_pins.0.period())).round() as _);
        self.power_pins.1.set_timestamp((modulations[1] * Float::from(self.power_pins.1.period())).round() as _);
        self.power_pins.2.set_timestamp((modulations[2] * Float::from(self.power_pins.2.period())).round() as _);
    }
    fn disable(&mut self) {
        self.power_enable.set_low();
    }
}*/


/// rotor position with AS5600 and no current sensing
pub struct Mk2<'d, I2C, PWM> {
    power_enable: Output<'d>,
    power_pins: (
        PwmPin<'d, PWM, 0, true>,
        PwmPin<'d, PWM, 1, true>,
        PwmPin<'d, PWM, 2, true>,
        ),
    position_sensor: As5600<'d, I2C>,
    modulation_to_current: Float,
    current_estimation: Vector<Float, PHASES>,
}
impl<'d, I2C, PWM> 
Mk2<'d, I2C, PWM> 
where
    I2C: I2c,
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
     
        let position_sensor = As5600::new(bus);
        
        Self {
            power_enable,
            power_pins,
            position_sensor,
            modulation_to_current: power_voltage / phases_resistance,
            current_estimation: Vector::from([0.; PHASES]),
            }
    }
}
impl<'d, I2C, PWM> 
Driver for Mk2<'d, I2C, PWM> 
where
    I2C: I2c,
    PWM: PwmPeripheral + 'd,
{
    fn check(&mut self) -> impl Future<Output=Result<(), ControlError>> {
        async {
            self.position_sensor.check().await.map_err(|e| match e {
                as5600::Error::I2c(_) => ControlError::PositionSensorNotDetected,
                as5600::Error::Sensor(_) => ControlError::PositionMagnetNotDetected,
            })
        }
    }
    fn measure(&mut self) -> impl Future<Output=Result<Measure, ControlError>> {
        async { Ok(Measure {
            position: self.position_sensor.angle().await.map_err(|_| ControlError::PositionSensorNotDetected)?,
            currents: self.current_estimation,
            modulations: Vector::from([
                Float::from(self.power_pins.0.timestamp()) * Float::from(self.power_pins.0.period()),
                Float::from(self.power_pins.1.timestamp()) * Float::from(self.power_pins.1.period()),
                Float::from(self.power_pins.2.timestamp()) * Float::from(self.power_pins.2.period()),
                ]),
            }) }
    }
    fn modulate(&mut self, modulations: Vector<Float, PHASES>) {
        self.power_enable.set_high();
        self.power_pins.0.set_timestamp((modulations[0] * Float::from(self.power_pins.0.period())).round() as _);
        self.power_pins.1.set_timestamp((modulations[1] * Float::from(self.power_pins.1.period())).round() as _);
        self.power_pins.2.set_timestamp((modulations[2] * Float::from(self.power_pins.2.period())).round() as _);
        self.current_estimation = modulations * self.modulation_to_current;
    }
    fn disable(&mut self) {
        self.power_enable.set_low();
    }
}
