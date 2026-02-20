
use core::ops::{Add, Mul};

use num_traits::float::{Float as FloatTrait, FloatConst};
use esp_println::{println, dbg};

use crate::prelude::*;



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
        self.transform.set_position(position * Float::from(self.motor.poles) * Float::PI()*2.);
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
    last_velocity: [Float; MULTITURN_LOWPASS_ORDER+1],
}
const MULTITURN_LOWPASS_ORDER: usize = 2; // order of the linear low pass filter
impl MultiTurnObserver {
    pub fn new(period: Float, lowpass: Float) -> Self {
        Self {
            period,
            lowpass_rate: continuous_to_discrete_filter(period, lowpass),
            last_absolute: 0.,
            last_velocity: [0.; _],
        }
    }
    /// reset count of turns to the given absolute position
    pub fn reset(&mut self, absolute: Float) {
        self.last_absolute = absolute;
    }
    pub fn observe(&mut self, relative: Float) -> (Float, Float) {
    // TODO enavble this when velocity is smoother
//         let expected = self.last_absolute + 0.5*self.last_velocity * self.period;
        let expected = self.last_absolute;
        let absolute = wrap_as(relative, expected, 1.);
        // TODO try getting velocity AND acceleration with polynomial least squares regression over a rolling buffer
        // low-pass filter velocity
        let diff = absolute - self.last_absolute;
        if diff.abs() < 0.1 {
            self.last_velocity[0] = diff / self.period;
            for i in 1 .. self.last_velocity.len() {
                self.last_velocity[i] = lerp(self.last_velocity[i], self.last_velocity[i-1], self.lowpass_rate);
            }
        }
        self.last_absolute = absolute;
        (absolute, self.last_velocity.last().unwrap().clone())
    }
}

/// euclid remainder
pub fn rem_euclid(x: Float, r: Float) -> Float {
    if x >= 0. {x % r} else {r + x % r}
}

/// bring the given `value` the closest to `reference` by adding any multiple of `period`
pub fn wrap_as(value: Float, reference: Float, period: Float) -> Float {
    let difference = value - rem_euclid(reference, period);
    // always assume less than 1 revolution occured
    reference + difference + (
        if difference > 0.5 {-period}
        else if difference < -0.5 {period}
        else {0.}
        )
}

pub fn continuous_to_discrete_gain(period: Float, proportional: Float) -> Float {
    (1. - (-proportional * period).exp()) / period
}
pub fn continuous_to_discrete_filter(period: Float, proportional: Float) -> Float {
    1. - (-proportional * period).exp()
}

pub fn lerp<T>(a: T, b: T, x: Float) -> T
where T: Add<Output=T> + Mul<Float, Output=T>
{
    a*(1.-x) + b*x
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
    pub fn phases_to_stator(&self, phases: Vector<Float, N>) -> Vector<Float, 2> {
        self.phases_to_stator.dot(phases)
    }
    pub fn stator_to_phases(&self, field: Vector<Float, 2>) -> Vector<Float, N> {
        self.stator_to_phases.dot(field)
    }
}

/// if voltage out of bounds, center it to reduce saturations, then saturate
pub fn clamp_voltage(voltages: Vector<Float, PHASES>, saturation: (Float, Float)) -> Vector<Float, PHASES> {
    let range = (voltages.min(), voltages.max());
    let sat_center = (saturation.1 + saturation.0) * 0.5;
    let range_center = (range.1 + range.0) * 0.5;
    (voltages + Vector::fill(sat_center - range_center)).clamp(saturation.0, saturation.1)
//     (voltages + Vector::fill(sat_center)).clamp(saturation.0, saturation.1)
}

pub fn soft_command_limit(command: Float, limited: Float, limit: (Float, Float), soft: Float) -> Float {
//     return command;
    // TODO handle the case where one or two bounds are inf
    let range = soft * (limit.1 - limit.0);
    if command > 0. {
        command * ((limit.1 - limited) / range).clamp(0., 1.)
    }
    else {
        command * ((limited - limit.0) / range).clamp(0., 1.)
    }
}


// pub struct LowPassFilter<T, const ORDER: usize> {
//     rate: Float,
//     value: [T; ORDER+1],
// }

// pub struct Impedance {
//     pub resistance: Float,
//     pub inductance: Float,
// }
// pub struct ImpedanceObserver<const N: usize> {
//     period: Float,
//     transform: SpaceVectorTransform<PHASES>,
//     regression: RollingLinearRegression<PHASES, PHASES>,
//     bcurrent: RollingBuffer<Vector<Float, 2>, N>,
//     bvoltage: RollingBuffer<Vector<Float, 2>, N>,
//     icurrent: Float,
//     ivoltage: Float,
// }
// impl<const N: usize> ImpedanceObserver<N> {
//     pub fn new(period: Float, initial: Impedance, noise_current: Float, lowpass: Float) -> Self {todo!()}
//     pub fn observe(&mut self, currents: Vector<Float, PHASES>, voltages: Vector<Float, PHASES>) -> Impedance {todo!()}
//     pub fn stimulate(&mut self) -> Vector<Float, PHASES> {todo!()}
// }

/**
    continuously solves `a*x = b`, where 
    - `a` is a matrix (T,A)
    - `b` is a matrix (T,B)
    - `x` is the solution matrix (A,B) to find
    - `T` is the amount of samples until now
    
    it uses a 1th order lowpass filter to lower the weight of samples with time passing
*/
pub struct RollingLinearRegression<const A: usize, const B: usize> {
    lowpass_rate: Float,
    initial_aa: Matrix<Float, A, A>,
    initial_ab: Matrix<Float, A, B>,
    aa: Matrix<Float, A, A>,
    ab: Matrix<Float, A, B>,
    x: Matrix<Float, A, B>,
    err: Vector<Float, B>,
}
impl<const A: usize, const B: usize> RollingLinearRegression<A, B> {
    /**
        - `initial` is the initial value of the model, and the value the regression returns to in case of singular input
        - `prec_a` is a value of smaller order of magnitude than the expected samples, used to detect when to fallback to the initial model
        - `lowpass_rate` is the rate of the 1th order lowpass filter lowering important of old samples
    */
    pub fn new(initial: Matrix<Float, A, B>, prec_a: Vector<Float, A>, lowpass_rate: Float) -> Self {
        let a = Matrix::diagonal(prec_a);
        let b = a.dot(initial);
        Self {
            aa: Matrix::zero(), 
            ab: Matrix::zero(),
            lowpass_rate,
            initial_aa: a.transpose().dot(a),
            initial_ab: a.transpose().dot(b),
            x: initial,
            err: Vector::zero(),
        }
    }
    /** add a batch of samples to the regression
        
        - they all count the same weight
        - all previous samples weights are lowered
    */
    pub fn add<const N: usize>(&mut self, a: Matrix<Float, N, A>, b: Matrix<Float, N, B>) {
        self.aa = lerp(self.aa, a.transpose().dot(a), self.lowpass_rate);
        self.ab = lerp(self.ab, a.transpose().dot(b), self.lowpass_rate);
        self.x = (self.aa + self.initial_aa).inv().dot(self.ab + self.initial_ab);
        let pred = a.dot(self.x);
        for i in 0 .. N {
            self.err = lerp(self.err, (b.row(i) - pred.row(i)) .map(|x| x*x), self.lowpass_rate);
        }
    }
    /// current solution of the regression
    pub fn solution(&self) -> Matrix<Float, A, B> {
        self.x
    }
    /// current standard reprojection error on `b`
    pub fn error(&self) -> Vector<Float, B> {
        self.err.map(Float::sqrt)
    }
}

/// simple discontinuous rolling buffer
pub struct RollingBuffer<T, const N: usize> {
    pub buffer: [T; N],
    pub index: usize,
}
impl<T: Default, const N: usize> RollingBuffer<T,N> {
    pub fn new() -> Self {
        Self {
            buffer: core::array::from_fn(|_| T::default()),
            index: 0,
        }
    }
    pub fn push(&mut self, mut value: T) -> T {
        core::mem::swap(&mut value, &mut self.buffer[self.index]);
        self.index = (self.index + 1) %N;
        value
    }
}
