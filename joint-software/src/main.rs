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
    uart::{DataBits, Parity, StopBits, RxConfig},
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use futures_concurrency::future::Join;

mod utils;
pub mod i2c;
pub mod as5600;
// pub mod tcs3472;
pub mod foc;
pub mod drivers;
pub mod registers;

use crate::foc::{Foc, CorrectorGains, MotorProfile};


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
    let bus = esp_hal::uart::Uart::new(peripherals.UART1, config).unwrap()
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
    
    let power_voltage = 15.; // volts
    let position_offset = 0.; // rad
    let period = 0.5e-3; // second
    let gains = CorrectorGains {
        proportional: 10., // Hz
        integral: 5., // Hz
    };
    let motor = MotorProfile {
        poles: 2,
        phase_resistance: 12., // ohm
        phase_inductance: 0., // henry
        rated_torque: 62e-3, // N.m
        rated_current: 0.91, // amps
    };
//     let mut driver = drivers::Mk1::new(
//         &bus,
//         peripherals.GPIO22,
//         peripherals.MCPWM0,
//         (peripherals.GPIO32, peripherals.GPIO25, peripherals.GPIO33),
//         peripherals.ADC1,
//         (peripherals.GPIO0, peripherals.GPIO2),
//         );
    let mut driver = drivers::Mk2::new(
        &bus,
        peripherals.GPIO22,
        peripherals.MCPWM0,
        (peripherals.GPIO32, peripherals.GPIO25, peripherals.GPIO33),
        motor.phase_resistance,
        power_voltage,
        );
    let mut foc = Foc::new(
        &mut driver,
        power_voltage,
        position_offset,
        period,
        motor,
        gains,
        ).unwrap();
    
//     log::info!("calibrate FOC");
//     foc.calibrate_constant(motor.rated_torque * 0.8, 1.).await.unwrap();

    let fault_check_period_factor = 100;

    log::info!("init typical values");
    {
        // expose characteristics to master
        let mut buffer = slave.lock().await;
        buffer.set(registers::typical::RATED_FORCE, motor.rated_torque);
        buffer.set(registers::typical::RATED_CURRENT, motor.rated_current);
        buffer.set(registers::typical::MAX_VOLTAGE, power_voltage);
        buffer.set(registers::typical::MAX_CURRENT, power_voltage / (2. * motor.phase_resistance));
        buffer.set(registers::typical::MAX_FORCE, power_voltage / (2. * motor.phase_resistance) * motor.rated_torque / motor.rated_current);
    }
    
    log::info!("init done");
    let task = async {
        let mut counter: usize = 0;
        let mut previous_control = registers::Control::default();
        let mut status = registers::Status::default();
        let mut error = registers::ControlError::None;
        
        loop {
            counter += 1;
            
            let step = async {
//                 let state = foc.measure().await;
//                 foc.control(state, motor.rated_torque * 0.4);
                
                if !status.fault() && counter > fault_check_period_factor {
                    counter = 0;
                    error = foc.check().await .or(Err(registers::ControlError::None)).unwrap_err();
                    status.set_fault(error != registers::ControlError::None);
                }
                
                let current = foc.measure().await;
                let current_velocity = 0.;
                
                let mut buffer = slave.lock().await;
                buffer.set(registers::current::STATUS, status);
                buffer.set(registers::current::ERROR, error);
                if let Ok(current) = current {
                    buffer.set(registers::current::POSITION, current.position);
                    buffer.set(registers::current::VELOCITY, current_velocity);
                    buffer.set(registers::current::FORCE, current.force);
                    buffer.set(registers::current::CURRENTS, current.currents.as_array().map(|x|  (x * registers::CURRENT_UNIT) as _).into());
                    buffer.set(registers::current::VOLTAGES, current.voltages.as_array().map(|x|  (x * registers::VOLTAGE_UNIT) as _).into());
                }
                
                let control = buffer.get(registers::target::CONTROL);
                let force = buffer.get(registers::target::FORCE);
                let force_constant = buffer.get(registers::target::FORCE_CONSTANT);
                let force_position = buffer.get(registers::target::FORCE_POSITION);
                let force_velocity = buffer.get(registers::target::FORCE_VELOCITY);
                let limit_force = buffer.get(registers::target::LIMIT_FORCE);
                let limit_velocity = buffer.get(registers::target::LIMIT_VELOCITY);
                let limit_position = buffer.get(registers::target::LIMIT_POSITION);
                drop(buffer);
                
                status.set_powered(control.power() && ! status.fault());
//                 log::debug!("status {:?}", status);
                
                if status.powered() {
                    // calibration process takes hand over everything
                    if control.calibrate() && !previous_control.calibrate() {
                        status.set_calibrated(false);
                        if let Err(e) = foc.calibrate_constant(force, 1.).await {
                            status.set_fault(true);
                            error = e;
                        }
                        else {
                            status.set_calibrated(true);
                        }
                    }
                    // normal control
                    else {
                        let current = current.unwrap();
                        let command = force + force_constant + force_position * current.position + force_velocity * current_velocity;
                        let mut command = command.clamp(limit_force.start, limit_force.stop);
                        if command > 0. {
                            if current.position > limit_position.stop 
                            || current_velocity > limit_velocity.stop
                            {command = 0.;}
                        }
                        else {   
                            if current.position < limit_position.start
                            || current_velocity < limit_velocity.start
                            {command = 0.;}
                        }
                        foc.control(current, command);
                    }
                }
                else {
                    // reset errors needs power off
                    if control.reset() {
                        status.set_fault(false);
                        error = registers::ControlError::None;
                    }
                    foc.disable();
                }
                
                previous_control = control;
            };
            let timeout = Timer::after(Duration::from_micros((period * 1e6) as _));
            (step, timeout).join().await;
        }
    };
    (task, slave.run()).join().await;
}
