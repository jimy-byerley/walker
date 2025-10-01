
pub mod i2c;
pub mod as5600;
pub mod tcs3472;
// pub mod synchron_motor;


use std::{
    rc::Rc,
    cell::RefCell,
};
use esp_idf_hal::{
    i2c::{I2cConfig, I2cDriver},
    ledc::{LedcTimerDriver, LedcDriver, config::TimerConfig},
    peripherals::Peripherals,
    delay::Delay,
    prelude::*,
    ledc::{LedcDriver, LedcTimerDriver, config::TimerConfig},
};


fn main() {
    esp_idf_hal::sys::link_patches();
//     esp_idf_hal::log::EspLogger::initialize_default();

    println!("1");
    let peripherals = Peripherals::take().unwrap();


    let m0_in1 = peripherals.pins.gpio6;
    let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer0, &TimerConfig::default().frequency(25.kHz().into())).unwrap();
    let mut driver = LedcDriver::new(peripherals.ledc.channel0, timer_driver, m0_in1).unwrap();

    let max_duty = driver.get_max_duty();
    log::info!("max duty {}", max_duty);
    driver.set_duty(max_duty * 3 / 4).unwrap();



    let sda = peripherals.pins.gpio19;
    let scl = peripherals.pins.gpio18;

    let m0i0 = peripherals.pins.gpio32;
    let m0i1 = peripherals.pins.gpio33;
    let m0i2 = peripherals.pins.gpio25;

    println!("2");
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &TimerConfig::default().frequency(25.kHz().into())).expect("creating timer driver");
    println!("2.5");
    let mut pwm0 = LedcDriver::new(peripherals.ledc.channel0, &timer, m0i0).expect("creating ledc driver");
    println!("2.5.1");
    let mut pwm1 = LedcDriver::new(peripherals.ledc.channel1, &timer, m0i1).unwrap();
    println!("2.5.2");
    let mut pwm2 = LedcDriver::new(peripherals.ledc.channel2, &timer, m0i2).unwrap();

    println!("3");
    pwm0.set_duty(pwm0.get_max_duty() * 1 / 4).unwrap();
    pwm1.set_duty(pwm1.get_max_duty() * 2 / 4).unwrap();
    pwm2.set_duty(pwm2.get_max_duty() * 3 / 4).unwrap();

    println!("4");
    let delay = Delay::default();
    let bus = Rc::new(RefCell::new(
        I2cDriver::new(peripherals.i2c0, sda, scl, &I2cConfig::new().baudrate(KiloHertz(400).into())).unwrap()
    ));

    let mut slave = crate::i2c::Slave::new(bus.clone(), as5600::ADDRESS);

    use bilge::prelude::u12;
    slave.write(as5600::registers::ZPOS, u12::from_u16(0)).unwrap();
    slave.write(as5600::registers::MPOS, u12::from_u16(0b111111111111)).unwrap();

    loop {
        // log::info!("angle: {:.3e}", rotor.angle().unwrap());
        log::info!("read {} {} {:#?} {:#?} {} {}",
            slave.read(as5600::registers::RAW_ANGLE).unwrap(),
            slave.read(as5600::registers::ANGLE).unwrap(),
            slave.read(as5600::registers::STATUS).unwrap(),
            slave.read(as5600::registers::CONF).unwrap(),
            slave.read(as5600::registers::ZPOS).unwrap(),
            slave.read(as5600::registers::MPOS).unwrap(),
        );
        delay.delay_ms(100);
    }
}



// use esp_idf_hal::delay::FreeRtos;
// use esp_idf_hal::ledc::*;
// use esp_idf_hal::peripherals::Peripherals;
// use esp_idf_hal::units::*;
// 
// fn main() -> anyhow::Result<()> {
//     esp_idf_hal::sys::link_patches();
// 
//     println!("Configuring output channel");
// 
//     let peripherals = Peripherals::take()?;
//     let mut channel = LedcDriver::new(
//         peripherals.ledc.channel0,
//         LedcTimerDriver::new(
//             peripherals.ledc.timer0,
//             &config::TimerConfig::new().frequency(25.kHz().into()),
//         )?,
//         peripherals.pins.gpio32,
//     )?;
// 
//     println!("Starting duty-cycle loop");
// 
//     let max_duty = channel.get_max_duty();
//     for numerator in [0, 1, 2, 3, 4, 5].iter().cycle() {
//         println!("Duty {numerator}/5");
//         channel.set_duty(max_duty * numerator / 5)?;
//         FreeRtos::delay_ms(2000);
//     }
// 
//     loop {
//         FreeRtos::delay_ms(1000);
//     }
// }
