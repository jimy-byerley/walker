
pub mod i2c;
pub mod as5600;
pub mod tcs3472;

use std::{
    rc::Rc,
    cell::RefCell,
};
use esp_idf_svc::hal::{
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    delay::Delay,
    prelude::*,
};
use as5600::AS5600;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let sda = peripherals.pins.gpio19;
    let scl = peripherals.pins.gpio18;

    let delay = Delay::default();
    let slaves_bus = Rc::new(RefCell::new(
        I2cDriver::new(peripherals.i2c0, sda, scl, &I2cConfig::new().baudrate(KiloHertz(400).into())).unwrap()
    ));

    // let mut rotor = AS5600::new(slaves_bus);
    let mut slave = crate::i2c::Slave::new(slaves_bus.clone(), as5600::ADDRESS);

    use bilge::prelude::u12;
    let (min, max) = (0_u16, 0b111111111111_u16);
    slave.write(as5600::registers::ZPOS, u12::from_u16(min)).unwrap();
    slave.write(as5600::registers::MPOS, u12::from_u16(max)).unwrap();

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
