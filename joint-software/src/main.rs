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
//     interrupt::software::SoftwareInterruptControl,
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use futures_concurrency::future::Join;
use esp_println::{println, dbg};

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
    
//     // Optionally, start the scheduler on the second core
//     let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
//     esp_rtos::start_second_core(
//         software_interrupt.software_interrupt0,
//         software_interrupt.software_interrupt1,
//         || {}, // Second core's main function.
//     );
    
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
    let period = 0.5e3; // second
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
    
    println!("{}", esp_alloc::HEAP.stats());

    foc.calibrate(motor.rated_torque * 0.8, 1.).await.unwrap();
    loop {
        let (state, _) = (
            foc.step(motor.rated_torque * 0.4),
            Timer::after(Duration::from_micros((period * 1e6) as _)),
        ).join().await;
    
//         dbg!(state);
    }
}

