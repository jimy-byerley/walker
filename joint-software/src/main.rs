
pub mod i2c;
pub mod as5600;
pub mod tcs3472;
pub mod foc;


use std::{
    rc::Rc,
    cell::RefCell,
};
use esp_idf_hal::{
    i2c::{I2cConfig, I2cDriver},
    ledc::{LedcDriver, LedcTimerDriver, config::TimerConfig},
    peripherals::Peripherals,
    delay::Delay,
    prelude::*,
};
use crate::foc::Foc;


fn main() {
    esp_idf_hal::sys::link_patches();
//     esp_idf_hal::log::EspLogger::initialize_default();

    println!("1");
    let peripherals = Peripherals::take().unwrap();

    let sda = peripherals.pins.gpio19;
    let scl = peripherals.pins.gpio18;
    let m0en = peripherals.pins.gpio22;
    let m0i0 = peripherals.pins.gpio32;
    let m0i1 = peripherals.pins.gpio33;
    let m0i2 = peripherals.pins.gpio25;

    
    let bus = Rc::new(RefCell::new(
        I2cDriver::new(peripherals.i2c0, sda, scl, &I2cConfig::new().baudrate(KiloHertz(400).into())).unwrap()
    ));
    let foc = Foc::new(
        bus,
        peripherals.pins.gpio22,
        peripherals.ledc.timer0,
        [peripherals.ledc.channel0, peripherals.ledc.channel1, peripherals.ledc.channel2],
        [peripherals.pins.gpio32, peripherals.pins.gpio33, peripherals.pins.gpio25],
        peripherals.adc1,
        [peripherals.pins.gpiovp, peripherals.pins.gpiovn],
        )?;

    let delay = Delay::default();
    loop {
        delay.delay_ms(20);
    }
}
