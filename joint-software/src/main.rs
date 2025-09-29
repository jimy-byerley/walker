
pub mod i2c;
pub mod as5600;
pub mod tcs3472;
// pub mod synchron_motor;

use std::{
    rc::Rc,
    cell::RefCell,
};
use esp_idf_svc::hal::{
    i2c::{I2cConfig, I2cDriver},
    ledc::{LedcTimerDriver, LedcDriver, config::TimerConfig},
    peripherals::Peripherals,
    delay::Delay,
    prelude::*,
};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();


    let m0_in1 = peripherals.pins.gpio32;
    let m0_in2 = peripherals.pins.gpio33;
    let m0_in3 = peripherals.pins.gpio25;
    let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer0, &TimerConfig::default().frequency(25.kHz().into())).unwrap();
    let mut pwm1 = LedcDriver::new(peripherals.ledc.channel0, &timer_driver, m0_in1).unwrap();
    let mut pwm2 = LedcDriver::new(peripherals.ledc.channel1, &timer_driver, m0_in2).unwrap();
    let mut pwm3 = LedcDriver::new(peripherals.ledc.channel2, &timer_driver, m0_in3).unwrap();
    println!("3");
    pwm1.set_duty(pwm1.get_max_duty() * 1 / 4).unwrap();
    pwm2.set_duty(pwm2.get_max_duty() * 2 / 4).unwrap();
    pwm3.set_duty(pwm3.get_max_duty() * 3 / 4).unwrap();
    println!("4");



//     let sda = peripherals.pins.gpio19;
//     let scl = peripherals.pins.gpio18;
//     let delay = Delay::default();
//     let bus = Rc::new(RefCell::new(
//         I2cDriver::new(peripherals.i2c0, sda, scl, &I2cConfig::new().baudrate(KiloHertz(400).into())).unwrap()
//     ));
// 
//     let mut slave = crate::i2c::Slave::new(bus.clone(), as5600::ADDRESS);
// 
//     use bilge::prelude::u12;
//     slave.write(as5600::registers::ZPOS, u12::from_u16(0)).unwrap();
//     slave.write(as5600::registers::MPOS, u12::from_u16(0b111111111111)).unwrap();
// 
//     loop {
//         // log::info!("angle: {:.3e}", rotor.angle().unwrap());
//         log::info!("read {} {} {:#?} {:#?} {} {}",
//             slave.read(as5600::registers::RAW_ANGLE).unwrap(),
//             slave.read(as5600::registers::ANGLE).unwrap(),
//             slave.read(as5600::registers::STATUS).unwrap(),
//             slave.read(as5600::registers::CONF).unwrap(),
//             slave.read(as5600::registers::ZPOS).unwrap(),
//             slave.read(as5600::registers::MPOS).unwrap(),
//         );
//         delay.delay_ms(100);
//     }
}
