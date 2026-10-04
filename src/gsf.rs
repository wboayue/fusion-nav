//! Yaw from the IMU and GNSS velocity alone: a bank of small filters, one per yaw hypothesis,
//! weighted by how well each predicts the velocity. Equations (45)–(52).
//!
//! The estimator PX4 and ArduPilot run for a vehicle with no magnetometer and no second
//! antenna (`EKFGSF_yaw`). It is free of [`Eskf`](crate::Eskf), which feeds it and reads
//! [`YawEstimator::yaw`]; nothing here reads the main filter's attitude, which is what lets its
//! answer correct a main filter whose yaw is wrong.

use core::f32::consts::PI;

use nalgebra::{
    ComplexField, Matrix2, Matrix3, Matrix3x2, RealField, UnitQuaternion, Vector2, Vector3,
};

use crate::math::{exp_quat, wrap_pi};
use crate::propagate::ImuSample;

/// Yaw hypotheses: `N_MODELS_EKFGSF` (`EKFGSF_yaw.h:40` at PX4 `c4e4ef98e9`;
/// `AP_Nav_Common.h:123` at ArduPilot `368dc0c428`). Spaced 72° apart, each starts within 36°
/// of the truth at worst, which is what (48)'s small-angle `F` needs.
pub(crate) const MODELS: usize = 5;

/// `k_t`, the gain from tilt error to rate correction in (46), 1/s (`_tilt_gain`,
/// `EKFGSF_yaw.h:91` at `c4e4ef98e9`). Taken from PX4 and ArduPilot, not derived: it is a
/// complementary filter's crossover, a 5 s time constant.
const TILT_GAIN: f32 = 0.2;

/// `k_β`, the gain from the tilt correction to the gyroscope bias in (47), 1/s
/// (`_gyro_bias_gain`, `EKFGSF_yaw.h:92`).
const BIAS_GAIN: f32 = 0.04;

/// The most bias (47) learns, rad/s, per axis (`ekf2_gyr_b_limit`, `EKFGSF_yaw.cpp:209`).
const BIAS_LIMIT: f32 = 0.05;

/// The rate above which (47) leaves the bias alone, 10°/s (`EKFGSF_yaw.cpp:212`): in a turn the
/// tilt correction is centripetal acceleration, not bias.
const BIAS_RATE_MAX: f32 = 0.174_532_93;

/// The low-pass on specific force is ten times faster than the tilt correction it feeds
/// (`EKFGSF_yaw.cpp:71`), a 0.5 s time constant against vibration.
const ACCEL_FILTER_RATIO: f32 = 10.0;

/// Yaw-rate noise of (48), rad s⁻¹ / √Hz.
///
/// PX4 and ArduPilot write a per-step σ of 0.1 rad/s, squared with the step (`_gyro_noise`,
/// `EKFGSF_yaw.h:89`, applied at `EKFGSF_yaw.cpp:280`), so their growth depends on the rate
/// the estimator runs at. This is that σ as a density at PX4's 10 ms step
/// (`EKF2_PREDICT_US`, `module.yaml:15-22`), `0.1 √0.01`, for the reason (21) is in densities.
const GYRO_NOISE: f32 = 0.01;

/// Horizontal acceleration noise of (48), m s⁻² / √Hz: PX4's 2 m/s² per step (`_accel_noise`,
/// `EKFGSF_yaw.h:90`) as a density at 10 ms, as [`GYRO_NOISE`] is. Far above an
/// accelerometer's own, because it prices the tilt error of (46), which leaks gravity into
/// the horizontal.
const ACCEL_NOISE: f32 = 0.2;

/// The floor on each model's variances, as `EKFGSF_yaw.cpp:290` and `:321` floor them.
const VARIANCE_MIN: f32 = 1.0e-6;

/// The floor on a weight before (51) normalizes (`EKFGSF_yaw.cpp:132`): a hypothesis is never
/// ruled out for good, so one that was wrong while the vehicle hovered can still win.
const WEIGHT_MIN: f32 = 1.0e-5;

/// The NIS an innovation is clipped to in (50), 5σ (`EKFGSF_yaw.cpp:333-336`): a velocity
/// spike moves each model by a bounded amount and cannot zero every weight at once.
const NIS_MAX: f32 = 25.0;

/// The least velocity σ (50) reads, m/s (`EKFGSF_yaw.cpp:302`).
const SIGMA_MIN: f32 = 0.01;

/// One yaw hypothesis: an attitude kept level by the accelerometer, and the horizontal
/// velocity and yaw error that attitude implies.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Model {
    /// `q_i`, body to navigation.
    attitude: UnitQuaternion<f32>,
    /// `β_i`, the gyroscope bias (47) learns.
    gyro_bias: Vector3<f32>,
    /// `v_i`, north and east.
    velocity: Vector2<f32>,
    /// `P_i` over `[δv_N δv_E δψ]`, `δψ` a rotation about navigation down.
    covariance: Matrix3<f32>,
    /// `w_i`.
    weight: f32,
}

impl Default for Model {
    fn default() -> Self {
        Self {
            attitude: UnitQuaternion::identity(),
            gyro_bias: Vector3::zeros(),
            velocity: Vector2::zeros(),
            covariance: Matrix3::zeros(),
            weight: 1.0 / MODELS as f32,
        }
    }
}

impl Model {
    /// `ψ_i`, the heading of the model's forward axis: the angle the main filter's heading
    /// is, so the two difference to a rotation about down.
    fn yaw(&self) -> f32 {
        yaw_of(self.attitude)
    }

    /// Turn the attitude about navigation down, composed on the left as a heading adoption
    /// is: tilt is untouched.
    fn turn(&mut self, angle: f32) {
        self.attitude = exp_quat(Vector3::z() * angle) * self.attitude;
        self.attitude.renormalize();
    }
}

/// The heading of an attitude's forward axis, `atan2` of its east and north components.
pub(crate) fn yaw_of(attitude: UnitQuaternion<f32>) -> f32 {
    let forward = attitude * Vector3::x();
    RealField::atan2(forward.y, forward.x)
}

/// A rotation of a horizontal vector by `angle` about down.
fn turned(v: Vector2<f32>, angle: f32) -> Vector2<f32> {
    let (sin, cos) = (ComplexField::sin(angle), ComplexField::cos(angle));
    Vector2::new(cos * v.x - sin * v.y, sin * v.x + cos * v.y)
}

/// The bank of yaw hypotheses and what they agree on. Equations (45)–(52).
///
/// A hover says nothing about yaw: every hypothesis predicts the same velocity, the weights
/// stay equal and the composite variance stays wide. A horizontal acceleration is what
/// separates them, since each rotates the same specific force into a different direction.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct YawEstimator {
    models: [Model; MODELS],
    /// `f̄`, the low-passed specific force (46) levels against.
    accel: Vector3<f32>,
    /// Whether the models' tilt has been set from the accelerometer.
    leveled: bool,
    /// Whether (45) has spread the hypotheses and velocity is being fused.
    fusing: bool,
    /// Seconds of propagation since fusing began.
    active: f32,
    /// `ψ̄` of (52).
    yaw: f32,
    /// `σ²_ψ̄` of (52).
    variance: f32,
}

impl Default for YawEstimator {
    fn default() -> Self {
        Self {
            models: [Model::default(); MODELS],
            accel: Vector3::zeros(),
            leveled: false,
            fusing: false,
            active: 0.0,
            yaw: 0.0,
            variance: f32::INFINITY,
        }
    }
}

impl YawEstimator {
    /// Begin again: nothing leveled, nothing fused.
    pub(crate) fn restart(&mut self) {
        *self = Self::default();
    }

    /// The composite yaw and its variance, (52), once velocity is being fused.
    pub(crate) fn yaw(&self) -> Option<(f32, f32)> {
        self.fusing.then_some((self.yaw, self.variance))
    }

    /// How long velocity has been fused, in seconds of propagation.
    pub(crate) fn active(&self) -> f32 {
        self.active
    }

    /// Step every hypothesis across one IMU sample. Equations (46)–(48).
    ///
    /// `gyro_bias` is the main filter's, which each model starts from until velocity fusion
    /// begins (`setGyroBias`, `EKFGSF_yaw.h:59-70`): a bias about the gravity vector is the one
    /// the accelerometer cannot teach (47), and the main filter's is the best available.
    ///
    /// The sample must be finite with positive intervals, which
    /// [`Eskf::predict`](crate::Eskf::predict) has checked before it integrates one.
    pub(crate) fn predict(&mut self, imu: ImuSample, gravity: f32, gyro_bias: Vector3<f32>) {
        let dt_v = imu.velocity_interval.as_secs();
        let dt_a = imu.angle_interval.as_secs();
        let delta_velocity = imu.delta_velocity.vector();
        let delta_angle = imu.delta_angle.vector();
        let accel = delta_velocity / dt_v;

        let coefficient = (ACCEL_FILTER_RATIO * dt_v * TILT_GAIN).min(1.0);
        self.accel = self.accel * (1.0 - coefficient) + accel * coefficient;

        if !self.leveled {
            // Both the sample and the filtered force within a tenth of 1 g, so that a vehicle
            // being moved is not leveled against its own acceleration (`EKFGSF_yaw.cpp:79-97`).
            let near_one_g = |f: &Vector3<f32>| (0.9 * gravity..1.1 * gravity).contains(&f.norm());
            if !(near_one_g(&accel) && near_one_g(&self.accel)) {
                return;
            }
            let level = level(accel);
            for model in &mut self.models {
                model.attitude = level;
            }
            self.leveled = true;
        }
        if !self.fusing {
            for model in &mut self.models {
                model.gyro_bias = gyro_bias;
            }
        }

        let force = self.accel.norm();
        // (46): unity at 1 g and zero a half g either side, squared so that vibration about
        // 1 g costs little (`ahrsCalcAccelGain`, `EKFGSF_yaw.cpp:417-435`).
        let attenuation = 1.0 - (2.0 * (force - gravity).abs() / gravity).min(1.0);
        let gain = TILT_GAIN * attenuation * attenuation;

        for model in &mut self.models {
            // (46): the rotation that carries the model's down onto the measured one.
            let down = model.attitude.inverse() * Vector3::z();
            let correction = if gain > 0.0 {
                down.cross(&self.accel) * (gain / force)
            } else {
                Vector3::zeros()
            };
            // (47).
            let rate = delta_angle / dt_a - model.gyro_bias;
            if rate.norm() < BIAS_RATE_MAX {
                model.gyro_bias -= correction * (BIAS_GAIN * dt_a);
                model.gyro_bias = model
                    .gyro_bias
                    .map(|bias| bias.clamp(-BIAS_LIMIT, BIAS_LIMIT));
            }
            let corrected = delta_angle + (correction - model.gyro_bias) * dt_a;
            model.attitude *= exp_quat(corrected);
            model.attitude.renormalize();

            if !self.fusing {
                continue;
            }
            // (48). The yaw column of `F` is the velocity increment turned a quarter circle:
            // a yaw error rotates what the accelerometer added.
            let moved = (model.attitude * delta_velocity).xy();
            #[rustfmt::skip]
            let f = Matrix3::new(
                1.0, 0.0, -moved.y,
                0.0, 1.0,  moved.x,
                0.0, 0.0,  1.0,
            );
            let q_v = ACCEL_NOISE * ACCEL_NOISE * dt_v;
            let q_psi = GYRO_NOISE * GYRO_NOISE * dt_a;
            model.covariance = f * model.covariance * f.transpose()
                + Matrix3::from_diagonal(&Vector3::new(q_v, q_v, q_psi));
            condition(&mut model.covariance);
            model.velocity += moved;
        }
        if self.fusing {
            self.active += dt_v;
        }
    }

    /// Weigh every hypothesis against a horizontal GNSS velocity. Equations (45), (49)–(52).
    ///
    /// `velocity` was taken some time before the models' present, and `carried` is how far the
    /// main filter's horizontal velocity has moved since, in that filter's axes, whose heading
    /// is `heading`. (49) turns it into each model's axes. PX4 needs no such step, because its
    /// estimator runs on the delayed horizon the measurement is taken at.
    ///
    /// Until the speed exceeds its own σ the hypotheses are only spread, (45), and nothing is
    /// fused: PX4's test without its in-air arm (`EKFGSF_yaw.cpp:117`), since this filter is
    /// not told when it flies.
    pub(crate) fn fuse_velocity(
        &mut self,
        velocity: Vector2<f32>,
        variance: f32,
        carried: Vector2<f32>,
        heading: f32,
    ) {
        if !self.leveled {
            return;
        }
        let variance = variance.max(SIGMA_MIN * SIGMA_MIN);
        if !self.fusing {
            self.spread(velocity, variance, carried, heading);
            self.fusing = velocity.norm_squared() > variance;
            self.active = 0.0;
            return;
        }

        let mut densities = [0.0; MODELS];
        let mut conditioned = true;
        for (model, density) in self.models.iter_mut().zip(&mut densities) {
            // (49).
            let z = velocity + turned(carried, wrap_pi(model.yaw() - heading));
            // (50).
            let s = model.covariance.fixed_view::<2, 2>(0, 0) + Matrix2::identity() * variance;
            let determinant = s.m11 * s.m22 - s.m12 * s.m21;
            if !(determinant > f32::MIN_POSITIVE) {
                conditioned = false;
                continue;
            }
            let s_inverse = Matrix2::new(s.m22, -s.m12, -s.m21, s.m11) / determinant;
            let mut innovation = model.velocity - z;
            let mut nis = (innovation.transpose() * s_inverse * innovation).x;
            if nis > NIS_MAX {
                innovation *= ComplexField::sqrt(NIS_MAX / nis);
                nis = NIS_MAX;
            }
            let k: Matrix3x2<f32> = model.covariance.fixed_view::<3, 2>(0, 0) * s_inverse;
            let correction = -k * innovation;
            model.velocity += correction.xy();
            model.turn(correction.z);
            model.covariance -= k * s * k.transpose();
            condition(&mut model.covariance);
            // (51).
            *density = ComplexField::exp(-0.5 * nis) / (2.0 * PI * ComplexField::sqrt(determinant));
        }
        // (51). A model that could not be updated leaves every weight where it was
        // (`EKFGSF_yaw.cpp:126`).
        if conditioned {
            let mut total = 0.0;
            for (model, density) in self.models.iter_mut().zip(densities) {
                model.weight = (model.weight * density).max(WEIGHT_MIN);
                total += model.weight;
            }
            for model in &mut self.models {
                model.weight /= total;
            }
        }
        self.compose();
    }

    /// (45): spread the hypotheses evenly over the circle, each at the measured velocity.
    fn spread(
        &mut self,
        velocity: Vector2<f32>,
        variance: f32,
        carried: Vector2<f32>,
        heading: f32,
    ) {
        let increment = 2.0 * PI / MODELS as f32;
        // Half the spacing: the hypotheses' 1σ intervals tile the circle.
        let sigma_yaw = 0.5 * increment;
        for (index, model) in self.models.iter_mut().enumerate() {
            let yaw = -PI + (index as f32 + 0.5) * increment;
            model.turn(wrap_pi(yaw - model.yaw()));
            model.velocity = velocity + turned(carried, wrap_pi(yaw - heading));
            model.covariance =
                Matrix3::from_diagonal(&Vector3::new(variance, variance, sigma_yaw * sigma_yaw));
            model.weight = 1.0 / MODELS as f32;
        }
        self.yaw = 0.0;
        self.variance = f32::INFINITY;
    }

    /// (52): the weighted mean direction, and the weighted variance about it. Summed as unit
    /// vectors so that hypotheses either side of ±π do not average to zero.
    fn compose(&mut self) {
        let mut direction = Vector2::zeros();
        for model in &self.models {
            let yaw = model.yaw();
            direction +=
                Vector2::new(ComplexField::cos(yaw), ComplexField::sin(yaw)) * model.weight;
        }
        self.yaw = RealField::atan2(direction.y, direction.x);
        self.variance = self
            .models
            .iter()
            .map(|model| {
                let delta = wrap_pi(model.yaw() - self.yaw);
                model.weight * (model.covariance.m33 + delta * delta)
            })
            .sum();
        // A bank that has stopped producing numbers starts over (`EKFGSF_yaw.cpp:165-167`).
        if !(self.variance > 0.0 && self.variance.is_finite()) {
            self.restart();
        }
    }
}

/// The attitude with zero heading whose down is where the accelerometer says it is: (5)'s
/// roll and pitch, as [`level_from_accel`](crate::init::level_from_accel) reads them.
fn level(specific_force: Vector3<f32>) -> UnitQuaternion<f32> {
    let (roll, pitch) =
        crate::init::level_from_accel(crate::units::Acceleration::from_vector(specific_force));
    UnitQuaternion::from_euler_angles(roll.as_radians(), pitch.as_radians(), 0.0)
}

/// Keep a model's covariance symmetric and its variances off zero, (42) and (42′) at 3 × 3.
fn condition(p: &mut Matrix3<f32>) {
    *p = (*p + p.transpose()) * 0.5;
    p.m11 = p.m11.max(VARIANCE_MIN);
    p.m22 = p.m22.max(VARIANCE_MIN);
    p.m33 = p.m33.max(VARIANCE_MIN);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::GRAVITY;
    use crate::frames::Body;
    use crate::units::{Acceleration, AngularRate, Seconds, Timestamp};

    const DT: f32 = 0.01;

    /// A vehicle flown from a known attitude and velocity, and the estimator fed what its
    /// sensors would read: truth is integrated here, never read back from the estimator.
    struct Flight {
        estimator: YawEstimator,
        attitude: UnitQuaternion<f32>,
        velocity: Vector3<f32>,
        step: u64,
        /// A deterministic velocity error, so the fixture's measurement is not the truth.
        ripple: f32,
    }

    impl Flight {
        fn heading(yaw: f32) -> Self {
            Self {
                estimator: YawEstimator::default(),
                attitude: UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
                velocity: Vector3::zeros(),
                step: 0,
                ripple: 0.05,
            }
        }

        /// One IMU step under a navigation-frame acceleration and a body rate.
        fn step(&mut self, acceleration: Vector3<f32>, rate: Vector3<f32>) {
            let force = self.attitude.inverse() * (acceleration - Vector3::z() * GRAVITY);
            self.step += 1;
            let imu = ImuSample::from_rates(
                Timestamp::from_micros(10_000 * self.step),
                AngularRate::<Body>::from_vector(rate),
                Acceleration::<Body>::from_vector(force),
                Seconds::from_secs(DT),
            );
            self.estimator.predict(imu, GRAVITY, Vector3::zeros());
            self.attitude *= exp_quat(rate * DT);
            self.velocity += acceleration * DT;
        }

        /// The velocity a receiver reports now, at σ 0.1 m/s.
        fn fix(&mut self) {
            let phase = self.step as f32 * 0.37;
            let error = Vector2::new(ComplexField::sin(phase), ComplexField::cos(1.7 * phase));
            self.estimator.fuse_velocity(
                self.velocity.xy() + error * self.ripple,
                0.01,
                Vector2::zeros(),
                0.0,
            );
        }

        /// `seconds` of flight under `acceleration(t)`, a fix every 0.2 s.
        fn fly(&mut self, seconds: f32, acceleration: impl Fn(f32) -> Vector3<f32>) {
            for k in 0..(seconds / DT) as u32 {
                self.step(acceleration(k as f32 * DT), Vector3::zeros());
                if k % 20 == 19 {
                    self.fix();
                }
            }
        }

        /// Two seconds still, so the bank levels, then a dash north to start the fusion.
        fn airborne(yaw: f32) -> Self {
            let mut flight = Self::heading(yaw);
            flight.fly(2.0, |_| Vector3::zeros());
            flight
        }
    }

    /// A multirotor's legs: two seconds of dash and two of brake at 2 m/s², each leg a
    /// quarter turn from the last.
    fn legs(t: f32) -> Vector3<f32> {
        let leg = (t / 4.0) as u32;
        let sign = if t - leg as f32 * 4.0 < 2.0 {
            1.0
        } else {
            -1.0
        };
        let direction = leg as f32 * core::f32::consts::FRAC_PI_2;
        Vector3::new(
            ComplexField::cos(direction),
            ComplexField::sin(direction),
            0.0,
        ) * (2.0 * sign)
    }

    /// A steady circle: 2 m/s² turning at 0.8 rad/s.
    fn circling(t: f32) -> Vector3<f32> {
        Vector3::new(
            2.0 * ComplexField::cos(0.8 * t),
            2.0 * ComplexField::sin(0.8 * t),
            0.0,
        )
    }

    #[test]
    fn a_maneuver_finds_the_yaw_wherever_it_started() {
        // Headings on a hypothesis, between two, and across the ±π seam.
        for yaw in [0.3_f32, -1.9, 2.6, 3.1, -0.628] {
            let mut flight = Flight::airborne(yaw);
            flight.fly(10.0, legs);
            let (estimate, variance) = flight.estimator.yaw().unwrap();
            assert!(
                wrap_pi(estimate - yaw).abs() < 0.05,
                "yaw {yaw}: estimated {estimate}"
            );
            assert!(variance < 0.262 * 0.262, "yaw {yaw}: variance {variance}");
        }
    }

    #[test]
    fn a_hover_never_claims_a_yaw() {
        // Enough speed to start fusing, then no acceleration: every hypothesis predicts the
        // same velocity, so the spread between them is all the variance there is.
        let mut flight = Flight::airborne(1.0);
        flight.velocity = Vector3::new(1.0, 0.0, 0.0);
        flight.fly(20.0, |_| Vector3::zeros());
        let (_, variance) = flight.estimator.yaw().unwrap();
        assert!(variance > 1.0, "variance {variance}");
    }

    #[test]
    fn nothing_is_fused_below_the_velocitys_own_accuracy() {
        let mut flight = Flight::airborne(1.0);
        assert_eq!(flight.estimator.yaw(), None);
        flight.velocity = Vector3::new(0.05, 0.0, 0.0);
        flight.ripple = 0.0;
        flight.fix();
        assert_eq!(flight.estimator.yaw(), None);
        flight.velocity = Vector3::new(0.5, 0.0, 0.0);
        flight.fix();
        assert!(flight.estimator.yaw().is_some());
    }

    #[test]
    fn the_weights_stay_a_distribution() {
        let mut flight = Flight::airborne(-2.2);
        flight.fly(6.0, legs);
        let total: f32 = flight.estimator.models.iter().map(|m| m.weight).sum();
        assert!((total - 1.0).abs() < 1.0e-5);
        assert!(flight.estimator.models.iter().all(|m| m.weight > 0.0));
    }

    #[test]
    fn the_accelerometer_levels_a_tilted_hypothesis() {
        let mut flight = Flight::airborne(0.4);
        for model in &mut flight.estimator.models {
            model.attitude *= UnitQuaternion::from_euler_angles(0.2, -0.15, 0.0);
        }
        // Six time constants of (46).
        flight.fly(30.0, |_| Vector3::zeros());
        for model in &flight.estimator.models {
            let down = model.attitude.inverse() * Vector3::z();
            assert!(down.xy().norm() < 0.01, "down {down}");
        }
    }

    #[test]
    fn a_turn_carries_every_hypothesis_with_it() {
        let mut flight = Flight::airborne(0.5);
        flight.fly(12.0, legs);
        // A quarter turn at 30°/s, still maneuvering.
        for k in 0..300 {
            flight.step(legs(k as f32 * DT), Vector3::z() * (PI / 6.0));
            if k % 20 == 19 {
                flight.fix();
            }
        }
        let (estimate, _) = flight.estimator.yaw().unwrap();
        assert!(
            wrap_pi(estimate - (0.5 + PI / 2.0)).abs() < 0.1,
            "{estimate}"
        );
    }

    #[test]
    fn an_old_velocity_is_carried_to_now_in_each_hypothesis_own_axes() {
        // Fixes 0.2 s old. The main filter here holds the true velocity history but a heading
        // 2 rad off, so its carry is in axes turned by that much. Fused as if current, the
        // same flight misses the bar this one meets.
        let fly = |carry: bool| {
            let (yaw, wrong) = (0.9_f32, 2.0_f32);
            let mut flight = Flight::airborne(yaw);
            let mut past = std::collections::VecDeque::new();
            for k in 0..1500 {
                flight.step(circling(k as f32 * DT), Vector3::zeros());
                past.push_back(flight.velocity.xy());
                if past.len() > 21 {
                    past.pop_front();
                }
                if k % 20 == 19 && past.len() == 21 {
                    let then = past[0];
                    let moved = turned(flight.velocity.xy() - then, wrong);
                    let carried = if carry { moved } else { Vector2::zeros() };
                    flight
                        .estimator
                        .fuse_velocity(then, 0.01, carried, wrap_pi(yaw + wrong));
                }
            }
            let (estimate, _) = flight.estimator.yaw().unwrap();
            wrap_pi(estimate - yaw)
        };
        std::println!("{} {}", fly(true), fly(false));
        assert!((-0.26..-0.16).contains(&fly(true)), "carried {}", fly(true));
        assert!(fly(false) < -0.3, "not carried {}", fly(false));
    }

    #[test]
    fn a_steady_circle_biases_the_yaw_by_the_tilt_filters_lag() {
        // (46) reads a specific force that keeps turning as a tilt, a quarter cycle late, and
        // the gravity that tilt leaks turns the acceleration every hypothesis sees: about
        // `atan(k_t / ω)`, 0.24 rad at 0.8 rad/s, under a variance that does not know it.
        // Asserted so that a tilt reference that removes it trips the line.
        let mut flight = Flight::airborne(0.3);
        flight.fly(20.0, circling);
        let (estimate, variance) = flight.estimator.yaw().unwrap();
        let error = wrap_pi(estimate - 0.3);
        assert!((-0.26..-0.16).contains(&error), "error {error}");
        assert!(variance < 0.05 * 0.05, "variance {variance}");
    }

    #[test]
    fn a_bank_that_stops_producing_numbers_starts_over() {
        let mut flight = Flight::airborne(0.0);
        flight.fly(3.0, circling);
        flight.estimator.models[2].covariance.m33 = f32::NAN;
        flight.estimator.compose();
        assert_eq!(flight.estimator, YawEstimator::default());
    }
}
