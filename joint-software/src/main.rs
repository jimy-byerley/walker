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
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, McPwm, PeripheralClockConfig},
    time::Rate,
};
use esp_println::{println, dbg};

pub mod i2c;
pub mod as5600;
// pub mod tcs3472;
pub mod foc;
use crate::foc::Foc;


esp_bootloader_esp_idf::esp_app_desc!();

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

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
    );
    let foc = Foc::new(
        &bus,
        peripherals.GPIO22,
        peripherals.MCPWM0,
        (peripherals.GPIO32, peripherals.GPIO33, peripherals.GPIO25),
        peripherals.ADC1,
        (peripherals.GPIO0, peripherals.GPIO2),
        );

    loop {
        Timer::after(Duration::from_secs(5)).await;
    }
}

