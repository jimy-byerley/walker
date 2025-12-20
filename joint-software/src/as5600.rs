use core::cell::RefCell;
use bilge::prelude::*;
use embedded_hal_async::i2c::I2c;
use crate::i2c::{Slave, Register};
use crate::pack_bilge;


/// high level control interface to the AS5600 magnetic encoder sensor
pub struct As5600<'b, I2C> {
    pub slave: Slave<'b, I2C>,
}
impl<'b, I2C:I2c> As5600<'b, I2C>
{
    pub fn new(bus: &'b RefCell<I2C>) -> Self {
        Self{slave: Slave::new(bus, ADDRESS)}
    }
    pub async fn angle(&mut self) -> Result<f32, I2C::Error> {
        let angle = self.slave.read(registers::ANGLE).await?.value();
        Ok(f32::from(u16::from(angle)) / f32::from(1u16<<12))
    }
    pub async fn check(&mut self) -> Result<(), Error<I2C::Error>> {
//         let status = self.slave.read(registers::STATUS).await .map_err(|e| Error::I2c(e))?;
//         if !status.magnet_detected() {Err(Error::Sensor("magnet not detected"))}
//         else if status.magnet_too_high() {Err(Error::Sensor("maget too strong"))}
//         else if status.magnet_too_low() {Err(Error::Sensor("magnet too weak"))}
//         else {Ok(())}
        Ok(())
    }
}
/// high level errors in managing the sensor
#[derive(Copy, Clone, Debug)]
pub enum Error<E> {
    I2c(E),
    Sensor(&'static str),
}


/// the AS5600 has a fixed i2c address
pub const ADDRESS: u8 = 0x36;
/// low level access to the sensor's i2c registers
pub mod registers {
    use super::*;

    pub const ZMCO: Register<u2> = Register::new(0x0);
    /**
        These registers are used to configure the start position (ZPOS)
        and a stop position (MPOS) or maximum angle (MANG) for a
        narrower angular range. The angular range must be greater
        than 18 degrees. In case of narrowed angular range, the
        resolution is not scaled to narrowed range (e.g. 0° to
        360°(full-turn) → 4096dec; 0° to180°→2048dec). To configure
        the angular range, see Angle Programming.
     */
    pub const ZPOS: Register<Angle> = Register::new(0x1);
    pub const MPOS: Register<Angle> = Register::new(0x3);
    pub const MANG: Register<Angle> = Register::new(0x5);
    /**
        The CONF register supports customizing the AS5600.
     */
    pub const CONF: Register<Conf> = Register::new(0x7);

    /**
        The RAW ANGLE register contains the unscaled and unmodified
        angle. The scaled output value is available in the ANGLE register.
        Note(s): The ANGLE register has a 10-LSB hysteresis at the limit
        of the 360 degree range to avoid discontinuity points or
        toggling of the output within one rotation.
     */
    pub const RAW_ANGLE: Register<Angle> = Register::new(0x0c);
    pub const ANGLE: Register<Angle> = Register::new(0x0e);

    /// The STATUS register provides bits that indicate the current state of the AS5600.
    pub const STATUS: Register<Status> = Register::new(0x0b);
    /**
        The AS5600 uses Automatic Gain Control in a closed loop to
        compensate for variations of the magnetic field strength due
        to changes of temperature, airgap between IC and magnet, and
        magnet degradation. The AGC register indicates the gain. For
        the most robust performance, the gain value should be in the
        center of its range. The airgap of the physical system can be
        adjusted to achieve this value.
        In 5V operation, the AGC range is 0-255 counts. The AGC range
        is reduced to 0-128 counts in 3.3V mode.
     */
    pub const AGC: Register<u8> = Register::new(0x0f);
    /// The MAGNITUDE register indicates the magnitude value of the internal CORDIC.
    pub const MAGNITUDE: Register<Angle> = Register::new(0x01a);

    /**
        Non-Volatile Memory (OTP)
        The non-volatile memory is used to permanently program the
        configuration. To program the non-volatile memory, the I2C
        interface is used (Option A, Option C). Alternatively, start and
        stop positions can be programmed through the output pin
        (Option B). The programming can be either performed in the
        5V supply mode or in the 3.3V operation mode but using a
        minimum supply voltage of 3.3V and a 10 μF capacitor at the
        VDD3V3 pin to ground. This 10 μF capacitor is needed only
        during the programming of the device. Two different
        commands are used to permanently program the device:
     */
    pub const BURN: Register<Burn> = Register::new(0x0ff);
    
    #[bitsize(16)]
    #[derive(FromBits, DebugBits, PartialEq, Default)]
    pub struct Angle {
        pub value: u12,
        _padding1: u4,
    }
    pack_bilge!(Angle);

    #[bitsize(16)]
    #[derive(FromBits, DebugBits, PartialEq, Default)]
    pub struct Conf {
        /// Power Mode
        /// 00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3
        pub power_mode: PowerMode,
        /// Hysteresis
        /// 00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs
        pub hysteresis: u2,
        /// mode for output stage (if used)
        pub out_stage: OutputStage,
        /// frequency of PWM (if enabled)
        pub pwm_frequency: PwmFrequency,
        /// slow filter
        pub slow_filter: SlowFilter,
        /// fast filter threshold
        pub fast_filter_threshold: FastFilterThreshold,
        /// watchdog enabled
        pub watchdog: bool,
        _padding1: u2,
    }
    pack_bilge!(Conf);
    
    #[bitsize(2)]
    #[derive(FromBits, Debug, PartialEq, Default)]
    pub enum PowerMode {
        #[default]
        Nominal = 0b00,
        LowPower1 = 0b01,
        LowPower2 = 0b10,
        LowPower3 = 0b11,
    }
    
    #[bitsize(2)]
    #[derive(FromBits, Debug, PartialEq, Default)]
    pub enum OutputStage {
        /// full range from 0% to 100% between GND and VDD
        #[default]
        AnalogFull = 0b00,
        /// reduced range from 10% to 90% between GND and VDD
        AnalogReduced = 0b01,
        /// digital PWM
        Pwm = 0b10,
        Undefined = 0b11,
    }
    
    #[bitsize(2)]
    #[derive(FromBits, Debug, PartialEq, Default)]
    pub enum PwmFrequency {
        #[default]
        Hz115 = 0b00,
        Hz230 = 0b01,
        Hz460 = 0b10,
        Hz920 = 0b11,
    }
    #[bitsize(2)]
    #[derive(FromBits, Debug, PartialEq, Default)]
    pub enum SlowFilter {
        #[default]
        X16 = 0b00,
        X8 = 0b01,
        X4 = 0b10,
        X2 = 0b11,
    }
    #[bitsize(3)]
    #[derive(FromBits, Debug, PartialEq, Default)]
    pub enum FastFilterThreshold {
        #[default]
        Only = 0b000,
        Lsb6 = 0b001,
        Lsb7 = 0b010,
        Lsb9 = 0b011,
        Lsb18 = 0b100,
        Lsb21 = 0b101,
        Lsb24 = 0b110,
        Lsb10 = 0b111,
    }

    #[bitsize(8)]
    #[derive(FromBits, DebugBits, PartialEq, Default)]
    pub struct Status {
        _padding1: u3,
        /// AGC minimum gain overflow, magnet too strong
        pub magnet_too_high: bool,
        /// AGC maximum gain overflow, magnet too weak
        pub magnet_too_low: bool,
        /// Magnet was detected
        pub magnet_detected: bool,
        _padding: u2,
    }
    pack_bilge!(Status);

    #[bitsize(8)]
    #[derive(FromBits, Debug, PartialEq, Default)]
    pub enum Burn {
        /// value when no specific operation is set
        #[fallback]
        #[default]
        Undefined = 0,
        /**
            Burn_Angle Command (ZPOS, MPOS)
            The host microcontroller can perform a permanent
            programming of ZPOS and MPOS with a BURN_ANGLE
            command. To perform a BURN_ANGLE command, write the
            value 0x80 into register 0xFF. The BURN_ANGLE command can
            be executed up to 3 times. ZMCO shows how many times ZPOS
            and MPOS have been permanently written.
            This command may only be executed if the presence of the
            magnet is detected (MD = 1).
         */
        Angle = 0x80,
        /**
            Burn_Setting Command (MANG, CONFIG)
            The host microcontroller can perform a permanent writing of
            MANG and CONFIG with a BURN_SETTING command. To
            perform a BURN_SETTING command, write the value 0x40 into
            register 0xFF.
            MANG can be written only if ZPOS and MPOS have never been
            permanently written (ZMCO = 00). The BURN_ SETTING
            command can be performed only one time.
         */
        Setting = 0x40,
    }
//     pack_bilge!(Burn);
}
