#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

pub mod as5600;
pub mod i2c;
pub mod tcs3472;
// pub mod synchron_motor;

use core::cell::RefCell;
use embedded_hal::{delay::DelayNs, pwm::SetDutyCycle};
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
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // loop {
    //     log::info!("Hello world!");
    //     let delay_start = esp_hal::time::Instant::now();
    //     while delay_start.elapsed() < esp_hal::time::Duration::from_millis(500) {}
    // }

    log::info!("Hello world!");

    //     let m0_in1 = peripherals.pins.gpio6;
    //     let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer0, &TimerConfig::default().frequency(25.kHz().into())).unwrap();
    //     let mut driver = LedcDriver::new(peripherals.ledc.channel0, timer_driver, m0_in1).unwrap();
    //
    //     let max_duty = driver.get_max_duty();
    //     log::info!("max duty {}", max_duty);
    //     driver.set_duty(max_duty * 3 / 4).unwrap();

    log::debug!("test pwm");
    let config = PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, config);
    // connect operator0 to timer0
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    log::debug!("1");
    // connect operator0 to pin
    // let mut m0_in0 = mcpwm
    //     .operator0
    //     .with_pin_a(peripherals.GPIO6, PwmPinConfig::UP_ACTIVE_HIGH);
    // let mut m0_in1 = mcpwm
    //     .operator1
    //     .with_pin_a(peripherals.GPIO7, PwmPinConfig::UP_ACTIVE_HIGH);
    // let mut m0_in2 = mcpwm
    //     .operator2
    //     .with_pin_a(peripherals.GPIO8, PwmPinConfig::UP_ACTIVE_HIGH);
    log::debug!("2");
    // start timer with timestamp values in the range of 0..=99 and a frequency of 20 kHz;
    mcpwm.timer0.start(
        config
            .timer_clock_with_frequency(99, PwmWorkingMode::Increase, Rate::from_khz(20))
            .unwrap(),
    );
    log::debug!("3");
    // pin will be high 50% of the time
    // m0_in0.set_timestamp(20);
    // m0_in1.set_timestamp(50);
    // m0_in2.set_timestamp(70);

    // let mut ledc = Ledc::new(peripherals.LEDC);
    // let mut channel0 = ledc.channel(esp_hal::ledc::channel::Number::Channel0, peripherals.GPIO6);

    // use esp_hal::ledc::{channel, timer, LowSpeed};
    // let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    // lstimer0
    //     .configure_hw(timer::config::Config {
    //         duty: timer::config::Duty::Duty5Bit,
    //         clock_source: timer::LSClockSource::APBClk,
    //         frequency: Rate::from_khz(24),
    //     })
    //     .unwrap();
    // channel0
    //     .configure(channel::config::Config {
    //         timer: &lstimer0,
    //         duty_pct: 10,
    //         pin_config: channel::config::PinConfig::PushPull,
    //     })
    //     .unwrap();
    // channel0.set_duty_cycle_percent(20);

    let mut delay = Delay::default();

    log::debug!("test i2c");
    let config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let bus = RefCell::new(
        I2c::new(peripherals.I2C0, config)
            .unwrap()
            .with_sda(peripherals.GPIO19)
            .with_scl(peripherals.GPIO18),
    );

    let mut slave = crate::i2c::Slave::new(&bus, as5600::ADDRESS);

    use bilge::prelude::u12;
    slave
        .write(as5600::registers::ZPOS, u12::from_u16(0))
        .unwrap();
    slave
        .write(as5600::registers::MPOS, u12::from_u16(0b111111111111))
        .unwrap();

    loop {
        // log::info!("angle: {:.3e}", rotor.angle().unwrap());
        log::info!(
            "read {} {} {:#?} {:#?} {} {}",
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
