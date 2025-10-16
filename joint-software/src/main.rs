#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_backtrace as _;

use core::cell::RefCell;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::master::I2c,
    time::Rate,
};
use esp_println::{println, dbg};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

mod utils;
pub mod i2c;
pub mod as5600;
// pub mod tcs3472;
pub mod foc;
pub mod mks;

use crate::{
    foc::{Foc, CorrectorGains, MotorProfile},
    mks::MKSDualFoc,
    };


esp_bootloader_esp_idf::esp_app_desc!();

// #[main]
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    
    let bus = RefCell::new(
        I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default()
                .with_frequency(Rate::from_khz(400))
                ).unwrap()
            .with_sda(peripherals.GPIO19)
            .with_scl(peripherals.GPIO18)
            .into_async()
    );
    let power_voltage = 10.; // volts
    let position_offset = 0.; // rad
    let period = 0.01; // second
    let gains = CorrectorGains {
        proportional: 10., // Hz
        integral: 5., // Hz
    };
    let motor = MotorProfile {
        phase_resistance: 12., // ohm
        phase_inductance: 0., // henry
        rated_torque: 82e-3, // N.m
        rated_current: 0.91, // amps
    };
    let mut driver = MKSDualFoc::new(
        &bus,
        peripherals.GPIO22,
        peripherals.MCPWM0,
        (peripherals.GPIO32, peripherals.GPIO33, peripherals.GPIO25),
        peripherals.ADC1,
        (peripherals.GPIO0, peripherals.GPIO2),
        );
    let mut foc = Foc::new(
        &mut driver,
        power_voltage,
        position_offset,
        period,
        motor,
        gains,
        );

    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}

