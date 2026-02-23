#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::master::I2c,
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Uart, DataBits, Parity, StopBits, RxConfig},
    mcpwm::PwmPeripheral,
    analog::adc::*,
    gpio::{*, interconnect::PeripheralOutput},
};
use embassy_executor::Spawner;
use embassy_time::{Instant, Duration, Ticker};
use futures_concurrency::future::Join;
use esp_println::dbg;
use num_traits::FloatConst;

use joint_registers as registers;

mod utils;
mod prelude;
pub mod i2c;
pub mod as5600;
// pub mod tcs3472;
pub mod foc;
pub mod drivers;

use crate::{
    prelude::*,
    drivers::*,
    as5600::*,
    foc::*,
    registers::{ControlError, Mode, Status},
    };
    
#[macro_use]
extern crate alloc;


esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {  
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(size: 32 * 1024);
    
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);
    
    
    log::info!("init slave");
    // let serial = Efuse::read_base_mac_address()
    let slave = uartcat::slave::Slave::<_, {registers::END}>::new(
        Uart::new(peripherals.UART1, esp_hal::uart::Config::default()
                .with_baudrate(1_000_000)
                .with_data_bits(DataBits::_8)
                .with_stop_bits(StopBits::_1)
                .with_parity(Parity::Even)
                .with_rx(RxConfig::default() .with_fifo_full_threshold(1))
                ).unwrap()
            .with_rx(peripherals.GPIO16)
            .with_tx(peripherals.GPIO17)
            .into_async(), 
        uartcat::registers::Device {
            serial: "".try_into().unwrap(),
            model: "joint-software".try_into().unwrap(),
            hardware_version: "0.1".try_into().unwrap(),
            software_version: "0.1".try_into().unwrap(),
            });
    
    log::info!("init motor driver");
    let bus = RefCell::new(
        I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default()
                .with_frequency(Rate::from_khz(400))
                ).unwrap()
            .with_sda(peripherals.GPIO19)
            .with_scl(peripherals.GPIO18)
            .into_async()
    );
    
    let power_voltage = 14.; // volts
    let gearbox_ratio = 30.;
    let motor = MotorProfile {
        poles: 2,
        phase_resistance: 12., // ohm
        phase_inductance: 0., // henry
        rated_torque: 62e-3, // N.m
        rated_current: 0.91, // amps
    };
    
    let period = 1e-3; // second
    let gains = CorrectorGains {
        proportional: 10., // Hz
        integral: 5., // Hz
    };
    let velocity_lowpass = 100.; // Hz

    let mut driver = Driver {
        motor,
        power_voltage,
        encoder_to_output: 1. / gearbox_ratio,
        encoder_to_rotor: 1. + 1. / gearbox_ratio,
//         encoder_to_rotor: 1.,
        slave: &slave,
        tick: Ticker::every(Duration::from_micros((period * 1e6) as _)),
//         motor_driver: MksDualFocV32::new(
//             peripherals.GPIO22,
//             peripherals.MCPWM0,
//             (peripherals.GPIO32, peripherals.GPIO25, peripherals.GPIO33),
//             motor.phase_resistance,
//             power_voltage,
//             ),
        motor_driver: MksDualFocV33::new(
            peripherals.GPIO22,
            peripherals.MCPWM0,
            (peripherals.GPIO32, peripherals.GPIO33, peripherals.GPIO25),
            peripherals.ADC1,
            (peripherals.GPIO39, peripherals.GPIO36),
            power_voltage,
            ),
        rotor_sensor: As5600::new(&bus),
        foc: Foc::new(period, motor, gains),
        multiturn: MultiTurnObserver::new(period, velocity_lowpass),
        rotor_offset: 0.,
        mode: Mode::default(),
        error: ControlError::default(),
        status: Status::default()
        };
    driver.motor_driver.calibrate().await;
    
    (driver.run(), slave.run()).join().await;
}


struct Driver<'d, PWM, ADC, ADC1, ADC0, const MEM: usize> {
    motor: MotorProfile,
    power_voltage: Float,
    encoder_to_output: Float,
    encoder_to_rotor: Float,
    tick: Ticker,
    slave: &'d uartcat::slave::Slave<Uart<'d, esp_hal::Async>, MEM>,
//     motor_driver: MksDualFocV32<'d, PWM>,
    motor_driver: MksDualFocV33<'d, ADC, ADC1, ADC0, PWM>,
    rotor_sensor: As5600<'d, I2c<'d, esp_hal::Async>>,
    multiturn: MultiTurnObserver,
    foc: Foc,
    rotor_offset: Float,
    mode: Mode,
    error: ControlError,
    status: Status,
}
impl<'d, PWM, ADC, ADC1, ADC0, const MEM: usize>
Driver<'d, PWM, ADC, ADC1, ADC0, MEM>
where
    PWM: PwmPeripheral + 'd,
    ADC: RegisterAccess + 'd,
    ADC0: AdcChannel + AnalogPin,
    ADC1: AdcChannel + AnalogPin,
{
    async fn run(&mut self) {
        self.write_typical().await;
        loop {
            if self.error != ControlError::None {
                self.control().await.ok();
            }
            else if let Err(err) = match self.mode {
                Mode::Off => self.control().await,
                Mode::Control => self.control().await,
                Mode::CalibrateImpedance => self.calibrate_impedance().await,
                Mode::CalibrateFocContinuous => self.calibrate_foc_continuous().await,
            } {
                self.error = err;
            }
        }
    }
    
    async fn write_typical(&self) {
        // expose characteristics to master
        let mut buffer = self.slave.lock().await;
        buffer.set(registers::typical::RATED_FORCE, self.motor.rated_torque);
        buffer.set(registers::typical::RATED_CURRENT, self.motor.rated_current);
        buffer.set(registers::typical::MAX_VOLTAGE, self.power_voltage);
        buffer.set(registers::typical::MAX_CURRENT, self.power_voltage / (2. * self.motor.phase_resistance));
        buffer.set(registers::typical::MAX_FORCE, self.power_voltage / (2. * self.motor.phase_resistance) * self.motor.rated_torque / self.motor.rated_current);
    }
    
    async fn control(&mut self) -> Result<(), ControlError> {
        let [mut duration_measure, mut duration_period, mut duration_process] = [Duration::from_micros(0); _];
        loop {
            let start_period = Instant::now();
            self.tick.next().await;
            // measures
            let start_measure = Instant::now();
            let (rotor, motor) = (
                self.rotor_sensor.angle(),
                self.motor_driver.measure(),
                ).join().await;
            let position_encoder = rotor.unwrap();
            let (currents, voltages) = motor?;
            duration_measure = start_measure.elapsed();
            
            let start_process = Instant::now();
            // observations
            let (position_multi, velocity_multi) = self.multiturn.observe(position_encoder);
            let position = position_multi * self.encoder_to_output;
            let velocity = velocity_multi * self.encoder_to_output;
            let force = self.foc.observe(position_multi * self.encoder_to_rotor + self.rotor_offset, currents);
            
            // exchanges
            let mut buffer = self.slave.lock().await;
            buffer.set(registers::current::STATUS, self.status);
            buffer.set(registers::current::ERROR, self.error);
            buffer.set(registers::current::POSITION, position);
            buffer.set(registers::current::VELOCITY, velocity);
            buffer.set(registers::current::FORCE, force);
            buffer.set(registers::current::CURRENTS, currents.as_array().map(|x|  (x / registers::CURRENT_UNIT) as _).into());
            buffer.set(registers::current::VOLTAGES, voltages.as_array().map(|x|  (x / registers::VOLTAGE_UNIT) as _).into());
            buffer.set(registers::timing::PERIOD, duration_period.as_micros() as f32 *1e-6);
            buffer.set(registers::timing::MEASURE, duration_measure.as_micros() as f32 *1e-6);
            buffer.set(registers::timing::PROCESS, duration_process.as_micros() as f32 *1e-6);
            self.mode = buffer.get(registers::target::MODE);
            let force_command = buffer.get(registers::target::FORCE);
            let force_constant = buffer.get(registers::target::FORCE_CONSTANT);
            let force_position = buffer.get(registers::target::FORCE_POSITION);
            let force_velocity = buffer.get(registers::target::FORCE_VELOCITY);
            let limit_force = buffer.get(registers::target::LIMIT_FORCE);
            let limit_velocity = buffer.get(registers::target::LIMIT_VELOCITY);
            let limit_position = buffer.get(registers::target::LIMIT_POSITION);
            drop(buffer);
            
            match self.mode {
            Mode::Off => {
                self.status.set_powered(false);
                self.status.set_fault(false);
                self.error = ControlError::None;
                self.foc.disable();
                self.motor_driver.disable();
            },
            Mode::Control => {
                self.status.set_powered(true);
                let target_force = force_command + force_constant + force_position * position + force_velocity * velocity;
                let target_force = target_force.clamp(limit_force.start, limit_force.stop);
                let target_force = soft_command_limit(target_force, position, (limit_position.start, limit_position.stop), 0.1);
                let target_force = soft_command_limit(target_force, velocity, (limit_velocity.start, limit_velocity.stop), 0.1);
                let voltages = self.foc.control(target_force, self.power_voltage);
                let modulations = clamp_voltage(voltages / self.power_voltage, (0., 1.));
                self.motor_driver.modulate(modulations);
            },
            _ => {
                if self.error != ControlError::None {
                    self.foc.disable();
                    self.motor_driver.disable();
                }
                else 
                    {return Ok(())}
            },
            }
            duration_process = start_process.elapsed();
            duration_period = start_period.elapsed();
        }
    }
    
    async fn calibrate_impedance(&mut self) -> Result<(), ControlError> {
        use rand::prelude::*;
        use rand::{Rng, RngExt};
        use rand::rngs::SmallRng;
        use rand::distr::{Distribution, StandardUniform};
        
        self.status.set_powered(true);
    
        let amplitude = self.power_voltage * 0.001;
        let limit = self.power_voltage * 0.1;
        let mut rng = SmallRng::seed_from_u64(1);
        
        let noise_current = 0.05;
        let lowpass = 1.;
        const SAMPLES: usize = 100;
        let period = self.foc.period();
        let mut transform = SpaceVectorTransform::new();
        let mut regression = RollingLinearRegression::new(
            Matrix::from([[self.motor.phase_resistance], [self.motor.phase_inductance/period]]),
            Vector::from([noise_current, noise_current/period]),
            continuous_to_discrete_filter(period, lowpass),
            );
        let mut bcurrent = RollingBuffer::<Vector<Float, 2>, SAMPLES>::new();
        let mut bvoltage = RollingBuffer::<Vector<Float, 2>, SAMPLES>::new();
        let mut icurrent = Vector::zero();
        let mut ivoltage = Vector::zero();
        loop {
            self.tick.next().await;
            // measures
            let (currents, voltages) = self.motor_driver.measure().await?;
            let current = transform.phases_to_stator(currents);
            let voltage = transform.phases_to_stator(voltages);
            
            // estimate impedance
            // difference between end and start of buffer
            let dcurrent = current - bcurrent.push(current);
            let dvoltage = voltage - bvoltage.push(current);
            // integrate difference
            icurrent += dcurrent;
            ivoltage += dvoltage;
            // use integrated/filtered values to regress the model
            regression.add(
                Matrix::from([icurrent, dcurrent]).transpose(),
                Matrix::from([ivoltage]).transpose(),
                );
            let solution = regression.solution();
            let resistance = solution[(0,0)];
            let inductance = solution[(1,0)] * period;
            
            // exchanges
            let mut buffer = self.slave.lock().await;
            buffer.set(registers::current::STATUS, self.status);
            buffer.set(registers::current::RESISTANCE, resistance);
            buffer.set(registers::current::INDUCTANCE, inductance);
            buffer.set(registers::current::CURRENTS, currents.as_array().map(|x|  (x / registers::CURRENT_UNIT) as _).into());
            buffer.set(registers::current::VOLTAGES, voltages.as_array().map(|x|  (x / registers::VOLTAGE_UNIT) as _).into());
            self.mode = buffer.get(registers::target::MODE);
            drop(buffer);
            
            match self.mode {
            Mode::CalibrateImpedance => {
                // generate noise
                let target_voltage = voltage + Vector::from(core::array::from_fn(|_| amplitude * rng.sample::<Float, _>(StandardUniform)));
                let modulations = clamp_voltage(
                    transform.stator_to_phases(target_voltage) / self.power_voltage, 
                    (0., limit / self.power_voltage));
                self.motor_driver.modulate(modulations);
            },
            _ => return Ok(()),
            }
        }
    }
    
    async fn calibrate_foc_continuous(&mut self) -> Result<(), ControlError> {
        self.status.set_powered(true);
        self.status.set_calibrated(false);
        self.slave.lock().await.set(registers::current::STATUS, self.status);

        let voltage = (self.motor.rated_current * self.motor.phase_resistance).min(self.power_voltage);
        let rotations = 1.;
        let velocity = 0.5; // rotation/s

        // measure offset between rotor position from encoder and field position
        // correct the bias assuming they are opposite while moving in opposite directions
        log::debug!("calibrate +");
        let offset_positive = self.calibrate_biased_offset(0., rotations, velocity, voltage).await?;
        log::debug!("calibrate -");
        let offset_negative = self.calibrate_biased_offset(rotations, 0., -velocity, voltage).await?;
        log::debug!("calibrate done");
        self.rotor_offset = (offset_positive + wrap_as(offset_negative, offset_positive, 1.)) /2.;
        log::debug!("offset {}", self.rotor_offset);
        
        // cleanup
        self.motor_driver.disable();
        self.status.set_powered(false);
        self.status.set_calibrated(true);
        self.slave.lock().await.set(registers::current::STATUS, self.status);
        // wait an other mode
        loop {
            self.tick.next().await;
            self.mode = self.slave.lock().await.get(registers::target::MODE);
            if self.mode != Mode::CalibrateFocContinuous 
                {return Ok(())}
        }
    }
    async fn calibrate_biased_offset(&mut self, start: Float, stop: Float, expected_velocity: Float, target_voltage: Float) -> Result<Float, ControlError> {
        let mut transform = SpaceVectorTransform::new();
        
        let mut offsets = 0.;
        let mut correlations = 1e-6;
        
        let elapsed = Instant::now();
        loop {
            self.tick.next().await;
            // measure
            let position_encoder = self.rotor_sensor.angle().await.unwrap();
            let (currents, voltages) = self.motor_driver.measure().await?;
            // observations
            let (position_multi, velocity_multi) = self.multiturn.observe(position_encoder);
            let position = position_multi * self.encoder_to_rotor;
            let velocity = velocity_multi * self.encoder_to_rotor;
            
            {
                // exchanges
                let mut buffer = self.slave.lock().await;
                buffer.set(registers::current::STATUS, self.status);
                buffer.set(registers::current::ERROR, self.error);
                buffer.set(registers::current::POSITION, position_multi * self.encoder_to_output);
                buffer.set(registers::current::VELOCITY, velocity_multi * self.encoder_to_output);
                buffer.set(registers::current::FORCE, 0.);
                buffer.set(registers::current::CURRENTS, currents.as_array().map(|x|  (x / registers::CURRENT_UNIT) as _).into());
                buffer.set(registers::current::VOLTAGES, voltages.as_array().map(|x|  (x / registers::VOLTAGE_UNIT) as _).into());
                self.mode = buffer.get(registers::target::MODE);
            }
            if self.mode != Mode::CalibrateFocContinuous {return Err(ControlError::CalibrationFailed)}
            
            // next move
            let t = (elapsed.elapsed().as_micros() as f32) * 1e-6;
            let expected_position = start + expected_velocity * t;
            if stop > start {
                if expected_position > stop {break}
            }
            else {
                if expected_position < stop {break}
            }
            
            // integrate the possible value of offset between expected rotor position and position according to encoder
            // add wrapping logic to still catch unique offset while catchingup on an other period
            let correlation = (expected_velocity * velocity).max(0.);
            let offset = wrap_as(expected_position - position, offsets / correlations, 1.);
            offsets += offset * correlation;
            correlations += correlation;
            
            // set the field in the direction of position we want the motor to go
            transform.set_position(expected_position * Float::from(self.motor.poles) * Float::PI()*2.);
            let voltages = transform.rotor_to_phases(Vector::from([target_voltage, 0.]));
            self.motor_driver.modulate(clamp_voltage(voltages / self.power_voltage, (0., 1.)));
        }
        
        // average position during velocity peak should be more or less in the middle of the 1/4 dephasing to field position giving max torque
        // this offset is of cours biased by the non linear friction that can exist in the gearbox
        Ok(offsets / correlations + 1./8. /Float::from(self.motor.poles) .copysign(expected_velocity))
    }
}
