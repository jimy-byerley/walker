// use nalgebra::{Matrix, SMatrix, SVector, Vector2, Matrix2, Rotation2};
use core::future::Future;
use num_traits::float::{Float as FloatTrait, FloatConst};
use vecmat::{
    prelude::{Zero, Dot},
    vector::Vector,
    matrix::Matrix,
    transform::Rotation2,
    };
use embassy_time::{Duration, Instant, Timer};

type Float = f32;
const PHASES: usize = 3;


/// trait for objects allowing to control sensors and power modulation for Field Oriented Control
pub trait Driver {
    fn get_position(&mut self) -> impl Future<Output=Float>;
    fn get_currents(&mut self) -> impl Future<Output=[Float; PHASES]>;
    fn set_modulations(&mut self, modulations: [Float; PHASES]);
    fn disable(&mut self);
}
#[derive(Copy, Clone, Debug)]
pub struct State {
    pub position: Float,
    pub force: Float,
    pub voltages: [Float; PHASES],
    pub currents: [Float; PHASES],
}
#[derive(Copy, Clone, Debug)]
pub struct MotorProfile {
    pub rated_torque: Float,
    pub rated_current: Float,
    pub phase_inductance: Float,
    pub phase_resistance: Float,
}
#[derive(Copy, Clone, Debug)]
pub struct CorrectorGains {
    pub proportional: Float,
    pub integral: Float,
}

/// Field Oriented Control of torque
pub struct Foc<'d, D> {
    power_voltage: Float,
    position_offset: Float,
    driver: &'d mut D,
    control: TorqueControl,
    transform: SpaceVectorTransform<PHASES>,
}
impl<'d, D:Driver> Foc<'d, D> {
    pub fn new(
        driver: &'d mut D, 
        power_voltage: Float, 
        position_offset: Float, 
        period: Float, 
        motor: MotorProfile, 
        gains: CorrectorGains,
    ) -> Result<Self, &'static str> {
        Ok(Self {
            driver,
            power_voltage,
            position_offset,
            control: TorqueControl::new(period, motor, gains)?,
            transform: SpaceVectorTransform::new(),
            })
    }
    pub fn disable(&mut self) {
        self.driver.disable();
        self.control.disable();
    }
    pub async fn step(&mut self, target_torque: Float) -> State {
        let current_position = self.driver.get_position().await.into();
        let current_currents = self.driver.get_currents().await.into();
        self.transform.set_position(current_position + self.position_offset);
        let current_field = self.transform.phases_to_rotor(current_currents);
        let (current_torque, target_voltage) = self.control.step(current_field, target_torque, self.power_voltage);
        let target_voltages = self.transform.rotor_to_phases(target_voltage);
        let target_modulations = clamp_voltage(target_voltages / self.power_voltage + Vector::fill(0.5), (0., 1.));
        self.driver.set_modulations(target_modulations.into());
        State {
            position: current_position,
            currents: current_currents.as_array().clone(), 
            force: current_torque,
            voltages: target_voltages.as_array().clone(),
            }
    }
    pub async fn calibrate(&mut self, torque: Float, duration: Float) -> Result<(), &'static str> {
        // reset field orientation
        let expected = 0.;
        let end = Instant::now() + Duration::from_millis((duration * 1e3) as _);
        self.transform.set_position(expected);
        loop {
            let current_currents = self.driver.get_currents().await.into();
            let current_field = self.transform.phases_to_rotor(current_currents);
            let (current_torque, target_voltage) = self.control.step(current_field, torque, self.power_voltage);
            let target_voltages = self.transform.rotor_to_phases(target_voltage);
            let target_modulations = clamp_voltage(target_voltages / self.power_voltage + Vector::fill(0.5), (0., 1.));
            self.driver.set_modulations(target_modulations.into());
            // wait for position to stabilize
            if (current_torque - torque).abs() / torque < 0.2 {
                return Err("current control failed");
            }
            if Instant::now() > end {
                break;
            }
            Timer::after(Duration::from_micros((self.control.period() * 1e6) as _)).await;
        }
        // current position is offset
        self.position_offset = expected - self.driver.get_position().await;
        self.disable();
        Ok(())
    }
}



pub struct SpaceVectorTransform<const N: usize> {
    // transformation matrices
    rotor_to_stator: Matrix<Float, 2, 2>,
    phases_to_stator: Matrix<Float, 2, N>,
    stator_to_phases: Matrix<Float, N, 2>,
}
impl<const N: usize> SpaceVectorTransform<N> {
    pub fn new() -> Self {
        let mut phases = Matrix::<Float, 2, N>::zero();
        for i in 0 .. N {
            let angle = Float::PI() * (i as Float) / (N as Float);
            phases[(0,i)] = angle.cos();
            phases[(1,i)] = angle.sin();
        }
        Self {
            rotor_to_stator: Matrix::fill(Float::NAN),
            phases_to_stator: phases,
            stator_to_phases: phases.transpose().dot(phases.dot(phases.transpose()).inv()),
        }
    }
    pub fn set_position(&mut self, position: Float) {
        self.rotor_to_stator = Matrix::from(Rotation2::new(position).to_linear().into_matrix());
    }
    pub fn phases_to_rotor(&self, phases: Vector<Float, N>) -> Vector<Float, 2> {
        self.rotor_to_stator.transpose().dot(self.phases_to_stator.dot(phases))
    }
    pub fn rotor_to_phases(&self, field: Vector<Float, 2>) -> Vector<Float, N> {
        self.stator_to_phases.dot(self.rotor_to_stator.dot(field))
    }
}

/// if voltage out of bounds, center it to reduce saturations, then saturate
pub fn clamp_voltage(voltages: Vector<Float, PHASES>, saturation: (Float, Float)) -> Vector<Float, PHASES> {
    
    let range = (voltages.min(), voltages.max());
    if range.0 < saturation.0 || range.1 > saturation.1 {
        let center = (range.1 + range.0) * 0.5;
        (voltages - Vector::fill(center)).clamp(saturation.0, saturation.1)
    }
    else {voltages}
}



pub struct TorqueControl {
    motor: MotorProfile,
    period: Float,
    gains: CorrectorGains,
    integral: Vector<Float, 2>,
}
impl TorqueControl {
    pub fn new(period: Float, motor: MotorProfile, gains: CorrectorGains) -> Result<Self, &'static str> {
        Ok(Self {
            motor,
            period,
            gains: CorrectorGains {                
                proportional: Self::continuous_to_discrete_gain(period, gains.proportional)?,
                integral: Self::continuous_to_discrete_gain(period, gains.integral)?,
                },
            integral: Vector::zero(),
        })
    }
    fn continuous_to_discrete_gain(period: Float, proportional: Float) -> Result<Float, &'static str> {
        let gain = (1. - (-proportional * period).exp()) / period;
        // we consider any phenomenon faster than this control loop to be out of control
        if gain > 1. 
            {return Err("gain is higher than control loop frequency")}
        // zero order hold coefficient for getting the same result as a continuous correction on a 1st order system
        Ok(gain)
    }
    pub fn period(&self) -> Float {
        self.period
    }
    
    pub fn disable(&mut self) {
        self.integral = Vector::zero();
    }
    /// take target current (relative) and specify target voltage (relative) to reach it
    pub fn step(&mut self, current_field: Vector<Float,2>, target_torque: Float, max_voltage: Float) -> (Float, Vector<Float,2>) {
        // only field orthogonal to rotor contributes to torque
        let current_torque = current_field[1] * self.motor.rated_torque / self.motor.rated_current;
        // motor characteristics gives relation between torque and current
        let target_current = target_torque / self.motor.rated_torque * self.motor.rated_current;
        // optimal field for desired torque (no need to waste energy on parallel direction)
        let target_field = Vector::<Float, 2>::from([0., target_current]);
        
        // considering one phase alone:  u = R i + L di/dt
        // use resistance for feed forward
        let mut target_voltage = target_field * self.motor.phase_resistance;
        // use indicutance for correction
        let error = target_field - current_field;
        self.integral += error * self.period;
        // proportional correction
        target_voltage += error * self.gains.proportional * self.motor.phase_inductance;
        if target_voltage.length() > max_voltage {
            target_voltage *= max_voltage / target_voltage.length();
        }
        // integral correction
        target_voltage += self.integral * self.gains.integral * self.motor.phase_inductance;
        if target_voltage.length() > max_voltage {
            let clamped = target_voltage * max_voltage / target_voltage.length();
            self.integral += (clamped - target_voltage) / (self.gains.integral * self.motor.phase_inductance);
            target_voltage = clamped;
        }
        
        (current_torque, target_voltage)
    }
}

