#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_backtrace as _;

use core::cell::RefCell;
use embedded_hal::{
    delay::DelayNs, 
    pwm::SetDutyCycle,
    };
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    i2c::master::I2c,
    ledc::Ledc,
    main,
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, McPwm, PeripheralClockConfig},
    time::Rate,
};
use esp_println::{println, dbg};

pub mod i2c;
pub mod as5600;
pub mod tcs3472;
pub mod foc;
use crate::foc::Foc;


esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sda = peripherals.GPIO19;
    let scl = peripherals.GPIO18;
    let m0en = peripherals.GPIO22;
    let m0i0 = peripherals.GPIO32;
    let m0i1 = peripherals.GPIO33;
    let m0i2 = peripherals.GPIO25;

    
    let bus = Rc::new(RefCell::new(
        I2cDriver::new(peripherals.I2C0, sda, scl, &I2cConfig::new().baudrate(KiloHertz(400).into())).unwrap()
    ));
    let foc = Foc::new(
        bus,
        peripherals.GPIO22,
        peripherals.ledc.TIMER0,
        [peripherals.ledc.channel0, peripherals.ledc.channel1, peripherals.ledc.channel2],
        [peripherals.GPIO32, peripherals.GPIO33, peripherals.GPIO25],
        peripherals.ADC1,
        [peripherals.GPIOVP, peripherals.GPIOVN],
        )?;

    let delay = Delay::default();
    loop {
        delay.delay_ms(20);
    }
}
