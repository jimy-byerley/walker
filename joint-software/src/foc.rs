use num_traits::float::{Float as FloatTrait, FloatConst};
use vecmat::{
    prelude::{Zero, Dot},
    vector::Vector,
    matrix::Matrix,
    transform::Rotation2,
    };
use esp_println::{println, dbg};

use crate::prelude::*;


// fn flat() {
//     loop
//         // checks
//         check_rotor_encoder().await
//         check_output_encoder().await
//         for _ in 0 .. 10
//             // measures
//             position_output = measure_output_encoder().await
//             for _ in 0 .. 10
//                 currents = measure_currents().await
//                 position_rotor = measure_rotor_encoder().await
//                 // observations
//                 (position_fast, current_velocity) = multiturn.observe(position_rotor) * rotor_output_ratio
//                 current_position = filter.observe(position_output, position_fast)
//                 current_torque = foc.observe(currents, (current_position + position_offset) * rotor_stator_ratio)
//                 // exchanges
//                 buffer = slave.lock().await
//                 buffer.set(registers::current::POSITION, current_position)
//                 force = buffer.get(registers::target::FORCE)
//                 ...
//                 // decisions
//                 if powered ...
//                 // controls
//                 target_torque = force + force_constant + force_position * current_position + force_velocity * current_velocity
//                 target_torque = target_torque.clamp(limit_force)
//                 target_torque = control_barrier(target_torque, position, limit_position)
//                 target_torque = control_barrier(target_torque, position, limit_velocity)
//                 target_voltages = foc.control(target_torque, max_voltage)
//                 target_modulations = clamp_relative(target_voltages / power_voltage, (0, 1))
//                 modulate(modulations)
// }



/// motor characteristics
#[derive(Copy, Clone, Debug)]
pub struct MotorProfile {
    /// motor rated torque in desired unit
    pub rated_torque: Float,
    /// (amps) motor rated current (current at rated torque)
    pub rated_current: Float,
    /// (ohm) electric resistance of one motor coil, needs to be precise to ±20% at least
    pub phase_resistance: Float,
    /// (henry) electric self inductance of one motor coil, set to 0 to disable current feedback loop
    pub phase_inductance: Float,
    /// number of pairs of poles in the motor
    pub poles: u8,
}
/// gains for torque control
#[derive(Copy, Clone, Debug, Default)]
pub struct CorrectorGains {
    /// Hz
    pub proportional: Float,
    /// Hz
    pub integral: Float,
}


pub struct Foc {
    /// motor electrical characteristics
    motor: MotorProfile,
    /// expected control period, step should be called at this rate
    period: Float,
    /// correctior gains
    gains: CorrectorGains,
    /// current integral value
    integral: Vector<Float, 2>,
    /// last observed field
    current_field: Vector<Float, 2>,
    /// transform phases to cartesian
    transform: SpaceVectorTransform<PHASES>,
}
impl Foc {
    /// gains must be continuous time, period will be used to convert them to discrete gains
    pub fn new(period: Float, motor: MotorProfile, gains: CorrectorGains) -> Self {
        Self {
            motor,
            period,
            gains: CorrectorGains {                
                proportional: continuous_to_discrete_gain(period, gains.proportional),
                integral: continuous_to_discrete_gain(period, gains.integral),
                },
            integral: Vector::zero(),
            current_field: Vector::zero(),
            transform: SpaceVectorTransform::new(),
        }
    }
    pub fn period(&self) -> Float {
        self.period
    }
    /// inform the control loop that the motor is disabled until `step` is called again
    pub fn disable(&mut self) {
        self.integral = Vector::zero();
    }
    /// return the current torque produced
    pub fn observe(&mut self, position: Float, currents: Vector<Float, PHASES>) -> Float {
        self.transform.set_position(position * Float::PI()*2. * Float::from(self.motor.poles));
        let current_field = self.transform.phases_to_rotor(currents);
        // only field orthogonal to rotor contributes to torque
        let current_torque = current_field[1] * self.motor.rated_torque / self.motor.rated_current;
        
        current_torque
    }
    /// take target current (relative) and specify target voltage (relative) to reach it
    pub fn control(&mut self, target_torque: Float, max_voltage: Float) -> Vector<Float,PHASES> {
        // motor characteristics gives relation between torque and current
        let target_current = target_torque / self.motor.rated_torque * self.motor.rated_current;
        // optimal field for desired torque (no need to waste energy on parallel direction)
        let target_field = Vector::<Float, 2>::from([0., target_current]);
        
        // considering one phase alone:  u = R i + L di/dt
        // use resistance for feed forward
        let mut target_voltage = target_field * self.motor.phase_resistance;
        // use inductance for correction
        let error = target_field - self.current_field;
        self.integral += error * self.period;
        // proportional correction
        target_voltage += error * self.gains.proportional * self.motor.phase_inductance;
        if target_voltage.length() > max_voltage {
            target_voltage *= max_voltage / target_voltage.length();
        }
        // integral correction
        target_voltage += self.integral * self.gains.integral * self.motor.phase_inductance;
        if target_voltage.length() > max_voltage && self.gains.integral * self.motor.phase_inductance != 0. {
            let clamped = target_voltage * max_voltage / target_voltage.length();
            self.integral += (clamped - target_voltage) / (self.gains.integral * self.motor.phase_inductance);
            target_voltage = clamped;
        }
        
        self.transform.rotor_to_phases(target_voltage)
    }
}


pub struct MultiTurnObserver {
    period: Float,
    lowpass_rate: Float,
    last_absolute: Float,
    last_velocity: Float,
}
impl MultiTurnObserver {
    pub fn new(period: Float, lowpass: Float) -> Self {
        Self {
            period,
            lowpass_rate: continuous_to_discrete_filter(period, lowpass),
            last_absolute: 0.,
            last_velocity: 0.,
        }
    }
    pub fn reset(&mut self, absolute: Float) {
        self.last_absolute = absolute;
    }
    pub fn observe(&mut self, relative: Float) -> (Float, Float) {
    // TODO enavble this when velocity is smoother
//         let expected = self.last_absolute + 0.5*self.last_velocity * self.period;
        let expected = self.last_absolute;
        let error = relative - rem_euclid(expected, 1.);
        // always assume less than 1 revolution occured
        let absolute = expected + error + (
            if error > 0.5 {-1.}
            else if error < -0.5 {1.}
            else {0.}
            );
        // TODO try getting velocity AND acceleration with polynomial least squares regression over a rolling buffer
        // low-pass filter velocity
        if error.abs() < 0.1 {
            self.last_velocity = self.last_velocity*(1.-self.lowpass_rate) + (absolute - self.last_absolute) / self.period * self.lowpass_rate;
        }
        self.last_absolute = absolute;
        (absolute, self.last_velocity)
    }
}

fn rem_euclid(x: Float, r: Float) -> Float {
    if x >= 0. {x % r} else {r + x % r}
}

fn continuous_to_discrete_gain(period: Float, proportional: Float) -> Float {
    (1. - (-proportional * period).exp()) / period
}
fn continuous_to_discrete_filter(period: Float, proportional: Float) -> Float {
    1. - (-proportional * period).exp()
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
            let angle = 2. * Float::PI() * (i as Float) / (N as Float);
            phases[(0,i)] = angle.cos();
            phases[(1,i)] = angle.sin();
        }
        Self {
            rotor_to_stator: Matrix::fill(Float::NAN),
            phases_to_stator: phases,
            stator_to_phases: phases.transpose().dot(phases.dot(phases.transpose()).inv()),
        }
    }
    /// update current rotor to stator transform
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
    let sat_center = (saturation.1 + saturation.0) * 0.5;
    let range_center = (range.1 + range.0) * 0.5;
    (voltages + Vector::fill(sat_center - range_center)).clamp(saturation.0, saturation.1)
}

pub fn control_barrier(command: Float, _limited: Float, _limit: (Float, Float), _rate: Float) -> Float {
    return command
}


