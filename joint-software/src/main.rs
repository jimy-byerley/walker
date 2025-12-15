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
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use futures_concurrency::future::Join;
use esp_println::dbg;

mod utils;
mod prelude;
pub mod i2c;
pub mod as5600;
// pub mod tcs3472;
pub mod foc;
pub mod drivers;
pub mod registers;

use crate::{
    prelude::*,
    drivers::*,
    as5600::*,
    foc::*,
    };

type Float = f32;
const PHASES: usize = 3;


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
    let config = esp_hal::uart::Config::default()
        .with_baudrate(1_500_000)
        .with_data_bits(DataBits::_8)
        .with_stop_bits(StopBits::_1)
        .with_parity(Parity::Even)
        .with_rx(RxConfig::default() .with_fifo_full_threshold(1))
        ;
    let bus = Uart::new(peripherals.UART1, config).unwrap()
        .with_rx(peripherals.GPIO16)
        .with_tx(peripherals.GPIO17)
        .into_async();
    // let serial = Efuse::read_base_mac_address()
    let slave = uartcat::slave::Slave::<_, {registers::END}>::new(
        bus, 
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
    
    let period = 0.5e-3; // second
    let power_voltage = 15.; // volts
    let gains = CorrectorGains {
        proportional: 10., // Hz
        integral: 5., // Hz
    };
    let gearbox_ratio = 30.;
    let motor = MotorProfile {
        poles: 2,
        phase_resistance: 12., // ohm
        phase_inductance: 0., // henry
        rated_torque: 62e-3, // N.m
        rated_current: 0.91, // amps
    };

    let mut driver = Driver {
        motor,
        period,
        power_voltage,
        rotor_output_ratio: 1. / gearbox_ratio,
        output_stator_ratio: gearbox_ratio * (1. + 1. / gearbox_ratio),
        slave: &slave,
        motor_driver: MksDualFocV32::new(
            peripherals.GPIO22,
            peripherals.MCPWM0,
            (peripherals.GPIO32, peripherals.GPIO25, peripherals.GPIO33),
            motor.phase_resistance,
            power_voltage,
            ),
        rotor_sensor: As5600::new(&bus),
        foc: Foc::new(period, motor, gains).unwrap(),
        multiturn: MultiTurnObserver::new(period),
        control: registers::Control::default(),
        rotor_offset: 0.,
        };
    
    (driver.run(), slave.run()).join().await;
}


struct Driver<'d, PWM, const MEM: usize> {
    motor: MotorProfile,
    period: Float,
    power_voltage: Float,
    rotor_output_ratio: Float,
    output_stator_ratio: Float,
    slave: &'d uartcat::slave::Slave<Uart<'d, esp_hal::Async>, MEM>,
    motor_driver: MksDualFocV32<'d, PWM>,
    rotor_sensor: As5600<'d, I2c<'d, esp_hal::Async>>,
    multiturn: MultiTurnObserver,
    foc: Foc,
    control: registers::Control,
    rotor_offset: Float,
}
impl<'d, PWM: PwmPeripheral + 'd, const MEM: usize>
Driver<'d, PWM, MEM> {
    async fn run(&mut self) {
        self.write_typical().await;
        loop {
            if self.control.calibrate() {
                self.calibrate_impedance().await;
                self.calibrate_foc_constant().await;
            }
            else {
                self.force().await;
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
    
    async fn force(&mut self) {
        let mut status = registers::Status::default();
        let mut error = registers::ControlError::None;
        let mut voltages = Vector::zero();
        
        loop {
            let step = async {
                // measures
                let position_rotor = self.rotor_sensor.angle().await .unwrap();
                let currents = self.motor_driver.measure().await .unwrap();
                
                // observations
                let (position_multi, velocity_multi) = self.multiturn.observe(position_rotor);
                let position = position_multi * self.rotor_output_ratio;
                let velocity = velocity_multi * self.rotor_output_ratio;
                let force = self.foc.observe(position * self.output_stator_ratio + self.rotor_offset, currents);
                
                // exchanges
                let mut buffer = self.slave.lock().await;
                buffer.set(registers::current::STATUS, status);
                buffer.set(registers::current::ERROR, error);
                buffer.set(registers::current::POSITION, position);
                buffer.set(registers::current::VELOCITY, velocity);
                buffer.set(registers::current::FORCE, force);
                buffer.set(registers::current::CURRENTS, currents.as_array().map(|x|  (x * registers::CURRENT_UNIT) as _).into());
                buffer.set(registers::current::VOLTAGES, voltages.as_array().map(|x|  (x * registers::VOLTAGE_UNIT) as _).into());
                self.control = buffer.get(registers::target::CONTROL);
                let force_command = buffer.get(registers::target::FORCE);
                let force_constant = buffer.get(registers::target::FORCE_CONSTANT);
                let force_position = buffer.get(registers::target::FORCE_POSITION);
                let force_velocity = buffer.get(registers::target::FORCE_VELOCITY);
                let limit_force = buffer.get(registers::target::LIMIT_FORCE);
                let limit_velocity = buffer.get(registers::target::LIMIT_VELOCITY);
                let limit_position = buffer.get(registers::target::LIMIT_POSITION);
                drop(buffer);
                
                // calibration process takes hand over everything
                if self.control.calibrate() {return}
                
                status.set_powered(self.control.power() && ! status.fault());
                
                if status.powered() {
                    let target_force = force_command + force_constant + force_position * position + force_velocity * velocity;
                    let target_force = target_force.clamp(limit_force.start, limit_force.stop);
                    let target_force = control_barrier(target_force, position, (limit_position.start, limit_position.stop), 0.);
                    let target_force = control_barrier(target_force, velocity, (limit_velocity.start, limit_velocity.stop), 0.);
                    voltages = self.foc.control(target_force, self.power_voltage);
                    self.motor_driver.modulate(clamp_voltage(voltages / self.power_voltage, (0., 1.)));
                }
                else {
                    // reset errors needs unpowering motor
                    if self.control.reset() {
                        status.set_fault(false);
                        error = registers::ControlError::None;
                    }
                    voltages = Vector::zero();
                    self.foc.disable();
                    self.motor_driver.disable();
                }
            };
            let timeout = Timer::after(Duration::from_micros((self.period * 1e6) as _));
            (step, timeout).join().await;
        }
    }
    
    async fn calibrate_impedance(&mut self) {todo!()}
    async fn calibrate_foc_constant(&mut self) {todo!()}
    async fn calibrate_foc_vibrations(&mut self) {todo!()}
}
