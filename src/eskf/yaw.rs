//! Yaw without a heading sensor: the estimator of (45)–(52) fed from `predict` and
//! `fuse_gnss_velocity`, and its answer adopted, as a first heading or in place of one GNSS
//! velocity contradicts.
//!
//! No entry point: [`Config::yaw_estimator`](crate::Config::yaw_estimator) turns it on, and a
//! multirotor with no magnetometer and no second antenna calls nothing new.

use crate::config::{ALIGNED_HEADING, Config};
use crate::frames::{Body, Ned};
use crate::gsf::YawEstimator;
use crate::math::wrap_pi;
use crate::observation::heading::{has_heading, heading_of};
use crate::propagate::ImuSample;
use crate::units::{Position, Radians, Timestamp, Velocity, VelocityNoise};

use nalgebra::Vector3;

use super::Eskf;

/// The composite yaw σ under which the estimator's answer is used: 15°, PX4's
/// `EKFGSF_yaw_err_max` (`src/modules/ekf2/EKF/common.h:396` at `c4e4ef98e9`) and ArduPilot's
/// `GSF_YAW_ACCURACY_THRESHOLD_DEG` (`AP_NavEKF3_core.h:78` at `368dc0c428`).
const YAW_SIGMA_MAX: Radians = Radians::from_degrees(15.0);

/// How far the filter's yaw must sit from the estimator's before GNSS rejections are put down
/// to the heading: 25° (`isYawFailure`, `gps_control.cpp:561` at `c4e4ef98e9`).
const YAW_FAILURE: Radians = Radians::from_degrees(25.0);

/// What a replaced yaw leaves owing: the GNSS horizontal sources not yet judged against the
/// new one, and a magnetometer not yet seen to agree with it.
///
/// PX4 resets velocity and position to GNSS with the yaw (`do_vel_pos_reset`,
/// `gps_control.cpp:124-128` at `c4e4ef98e9`): both were integrated through the wrong heading,
/// and a gate built on that covariance would hold the next fix off for its whole timeout. So
/// each source's first measurement the gate turns down is adopted at once, as after a position
/// hold ([`recovery_after`](super::hold::recovery_after)), and the debt is settled by that
/// adoption, by a measurement the gate passes, or by a caller's reset.
///
/// A debt needs no expiry. One still standing after the source's own
/// [`Recovery`](crate::Recovery) timeout belongs to a source not accepted for that long,
/// which the ordinary lockout adopts at its first rejection anyway.
///
/// PX4 also declares the magnetometer faulty for the rest of the flight
/// (`gps_control.cpp:442-445`). Here its recovery is withheld until one of its headings
/// passes the gate again: otherwise a GNSS outage lifts the guard on
/// [`Recovery::mag_heading`](crate::Recovery::mag_heading), and seven seconds later the
/// heading that was just replaced is adopted back.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct YawReplaced {
    position: bool,
    velocity: bool,
    magnetometer: bool,
}

impl YawReplaced {
    /// Everything owed, as a replacement leaves it.
    const OWING: Self = Self {
        position: true,
        velocity: true,
        magnetometer: true,
    };

    /// Whether the first GNSS position the gate turns down is to be adopted at once.
    pub(super) fn owes_position(&self) -> bool {
        self.position
    }

    /// [`owes_position`](Self::owes_position) for a GNSS velocity.
    pub(super) fn owes_velocity(&self) -> bool {
        self.velocity
    }

    /// Whether the magnetometer's recovery is withheld.
    pub(super) fn overrules_magnetometer(&self) -> bool {
        self.magnetometer
    }

    /// Horizontal position has been judged, adopted or set since.
    pub(super) fn settle_position(&mut self) {
        self.position = false;
    }

    /// Velocity has been judged, adopted or set since.
    pub(super) fn settle_velocity(&mut self) {
        self.velocity = false;
    }

    /// A magnetic heading has passed the gate since.
    pub(super) fn settle_magnetometer(&mut self) {
        self.magnetometer = false;
    }
}

// The bar an adoption has to clear is inside the one alignment asks of heading.
const _: () = assert!(YAW_SIGMA_MAX.as_radians() < ALIGNED_HEADING.as_radians());

impl Eskf {
    /// Step the yaw estimator across the sample `predict` just integrated. Equations
    /// (46)–(48).
    ///
    /// Out of line, beside the step rather than beneath it, as the position hold is.
    #[inline(never)]
    pub(super) fn step_yaw_estimator(&mut self, imu: ImuSample) {
        let Config {
            yaw_estimator: true,
            gravity,
            ..
        } = self.config
        else {
            return;
        };
        let gyro_bias = self.estimate.state().gyro_bias.vector();
        self.yaw_estimator.predict(imu, gravity, gyro_bias);
    }

    /// Time the estimator did not integrate, a gap coasted or a step refused once the clock
    /// had moved: its hypotheses no longer describe the vehicle, and it starts over, to settle
    /// again before its yaw is taken.
    pub(super) fn restart_yaw_estimator(&mut self) {
        if self.config.yaw_estimator {
            self.yaw_estimator.restart();
        }
    }

    /// Weigh the yaw hypotheses against a GNSS velocity. Equations (49)–(52).
    ///
    /// Called by [`fuse_gnss_velocity`](Self::fuse_gnss_velocity) with a solution it has
    /// screened, before the main filter judges it: the estimator's use is the case where that
    /// judgment is made against a wrong yaw. The velocity is counted once in each estimator,
    /// which is why the answer enters the main filter as an adoption and never as a
    /// measurement fused beside the velocity it was read from.
    #[inline(never)]
    pub(super) fn weigh_yaw(
        &mut self,
        time: Timestamp,
        velocity: Velocity<Ned>,
        noise: VelocityNoise<Ned>,
        antenna: Position<Body>,
    ) {
        if !(self.config.yaw_estimator && self.yaw_estimator.is_leveled()) {
            return;
        }
        let [north, east, _] = noise.variance();
        let Some(variance) = YawEstimator::variance_of(north, east) else {
            return;
        };
        // (49): how far the estimate's velocity has moved since `time`, less the antenna's
        // swing, in the main filter's axes.
        let Some(carried) = self.carried_velocity(Velocity::zero(), antenna, time) else {
            return;
        };
        let heading = heading_of(self.estimate.state().attitude.quaternion());
        self.diagnostics.yaw_estimator.note_arrival(time);
        self.yaw_estimator.fuse_velocity(
            time,
            velocity.vector().xy(),
            variance,
            carried.vector().xy(),
            heading,
            self.config.gravity,
        );
    }

    /// Take the yaw estimator's answer as the first heading, where none was ever established
    /// and the estimator has one. Called after [`weigh_yaw`](Self::weigh_yaw).
    ///
    /// The adoption is the one every first heading takes
    /// ([`fuse_mag_heading`](Self::fuse_mag_heading)), at the composite variance of (52), once
    /// its σ is under [`YAW_SIGMA_MAX`]: `resetYawToEKFGSF`, `gps_control.cpp:564-579` at
    /// `c4e4ef98e9`. Counted in
    /// [`Diagnostics::yaw_estimator`](crate::Diagnostics::yaw_estimator) as adopted.
    ///
    /// Only from a settled bank, where PX4 aligns from one that restarted in flight too
    /// (`gps_control.cpp:415-419`): the reason a restarted bank waits before it overrules a
    /// heading, that it leveled against whatever the vehicle was doing, is no weaker for a
    /// first one.
    pub(super) fn adopt_first_yaw(&mut self) {
        if !(self.unestablished.heading && self.yaw_estimator.is_settled()) {
            return;
        }
        if self.turn_to_estimated_yaw(|_| true) {
            self.diagnostics.yaw_estimator.record_adopted();
            self.note_alignment();
            self.magnetic_north = false;
        }
    }

    /// Replace a heading GNSS velocity contradicts with the yaw estimator's. Called by
    /// [`fuse_gnss_velocity`](Self::fuse_gnss_velocity) with a velocity the gate has just
    /// turned down.
    ///
    /// A yaw that is wrong turns every acceleration the wrong way, so velocity leaves the
    /// truth within a second and GNSS is rejected for it, while the heading source that put
    /// the yaw there goes on agreeing with it. Recovering the velocity alone, after its own
    /// timeout, adopts a fix and leaves the cause. The estimator never read that heading:
    /// where it has converged more than [`YAW_FAILURE`] from the filter's yaw, and no velocity
    /// has been accepted for [`Recovery::yaw_estimator`](crate::Recovery::yaw_estimator), the
    /// yaw is the estimator's from here (`tryYawEmergencyReset`, `gps_control.cpp:426-444` at
    /// `c4e4ef98e9`). Counted in
    /// [`Diagnostics::yaw_estimator`](crate::Diagnostics::yaw_estimator) as recovered.
    ///
    /// PX4 fires on a rejected position as well (`gps_control.cpp:113-125`). Not here: a
    /// position turned down with velocity still accepted is a receiver's offset, not a yaw,
    /// and on `093e806a`, whose receiver claims 0.37 m, the position trigger replaced a good
    /// magnetic heading twice ([measured]).
    ///
    /// PX4 also resets the variance of the gyroscope bias about body z
    /// (`resetGyroBiasZCov`, `gps_control.cpp:440`). Not done here: [`reset_heading_by`]
    /// drops the heading's correlation with the bias, and `yaw_fault` settles without it.
    ///
    /// [`reset_heading_by`]: Self::reset_heading_by
    /// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#yawestimator
    #[inline(never)]
    pub(super) fn replace_failed_yaw(&mut self) {
        if !self.config.yaw_estimator || !self.yaw_estimator.is_settled() {
            return;
        }
        // A second antenna measures heading directly and is the reference where a vehicle
        // carries one: an estimator that disagrees with it is the one at fault.
        let after = self.unless_accepted(
            self.config.recovery.yaw_estimator,
            &[&self.diagnostics.gnss_heading],
        );
        let since_initialized = self.diagnostics.since_initialized;
        let velocity = &self.diagnostics.gnss_velocity;
        // The rejection in hand is at least the second: at 1 Hz the delay alone is met by one
        // solution turned down, and one outlier is not a yaw.
        if !velocity.locked_out(after, since_initialized) || velocity.consecutive_rejections == 0 {
            return;
        }
        let disagrees = |y: f32| y.abs() > YAW_FAILURE.as_radians();
        if self.turn_to_estimated_yaw(disagrees) {
            self.diagnostics.yaw_estimator.record_recovered();
            self.note_alignment();
            self.magnetic_north = false;
            self.yaw_replaced = YawReplaced::OWING;
        }
    }

    /// Turn the estimate to the yaw estimator's answer, if it has one good enough, the
    /// vehicle's forward axis names a heading, and `wanted` takes the turn it would make.
    /// Returns whether it did.
    fn turn_to_estimated_yaw(&mut self, wanted: impl FnOnce(f32) -> bool) -> bool {
        let Some((yaw, variance)) = self.yaw_estimator.yaw() else {
            return false;
        };
        let bar = YAW_SIGMA_MAX.as_radians();
        // Written so that a variance that is not a number is not converged.
        let converged = variance < bar * bar;
        if !converged || !has_heading(self.estimate.state()) {
            return false;
        }
        let attitude = self.estimate.state().attitude.quaternion();
        let y = wrap_pi(yaw - heading_of(attitude));
        if !(y.is_finite() && wanted(y)) {
            return false;
        }
        self.reset_heading_by(y, variance, attitude.inverse() * Vector3::z());
        true
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::super::fixtures::*;
    use crate::config::{Config, GRAVITY, Recovery};
    use crate::eskf::Eskf;
    use crate::gsf::YawEstimator;
    use crate::gsf::tests::{circling, legs};
    use crate::health::{Fusion, Propagation, Status};
    use crate::math::wrap_pi;
    use crate::observation::heading::heading_of;
    use crate::propagate::ImuSample;
    use crate::units::{
        Acceleration, AngularRate, HeadingNoise, MagField, Position, Seconds, Velocity,
        VelocityNoise,
    };

    /// A vehicle held level at a true `yaw`, flown under `acceleration(t)`, the filter handed
    /// what its sensors read: the IMU every step, and every 0.2 s a GNSS velocity at `sigma`
    /// and whatever `also` offers.
    struct Flight {
        yaw: f32,
        velocity: Vector3<f32>,
        sigma: f32,
        /// How old each velocity is when it is fused, in steps.
        age: usize,
        /// IMU steps between velocities: 20 is 5 Hz.
        every: u32,
        /// The last velocity's outcome.
        last: Option<Fusion>,
    }

    impl Flight {
        fn heading(yaw: f32) -> Self {
            Self {
                yaw,
                velocity: Vector3::zeros(),
                sigma: 0.1,
                age: 0,
                every: 20,
                last: None,
            }
        }

        fn fly(
            &mut self,
            filter: &mut Eskf,
            seconds: f32,
            acceleration: impl Fn(f32) -> Vector3<f32>,
            mut also: impl FnMut(&mut Eskf),
        ) {
            let attitude = UnitQuaternion::from_euler_angles(0.0, 0.0, self.yaw);
            let dt = DT.as_secs();
            let mut past = std::collections::VecDeque::from([self.velocity]);
            for k in 0..(seconds / dt).round() as u32 {
                let a = acceleration(k as f32 * dt);
                let force = attitude.inverse() * (a - Vector3::z() * GRAVITY);
                let imu = ImuSample::reading(AngularRate::zero(), Acceleration::from_vector(force));
                assert_eq!(filter.step(imu, DT), Propagation::Propagated);
                self.velocity += a * dt;
                past.push_back(self.velocity);
                if past.len() > self.age + 1 {
                    past.pop_front();
                }
                if k % 20 == 19 {
                    let taken = filter
                        .now()
                        .before(Seconds::from_secs(self.age as f32 * dt));
                    self.last = Some(filter.fuse_gnss_velocity(
                        taken,
                        Velocity::from_vector(past[0]),
                        VelocityNoise::from_speed_accuracy(self.sigma),
                        Position::zero(),
                    ));
                    also(filter);
                }
            }
        }

        fn error(&self, filter: &Eskf) -> f32 {
            wrap_pi(heading_of(filter.state().attitude.quaternion()) - self.yaw).abs()
        }
    }

    fn still(_: f32) -> Vector3<f32> {
        Vector3::zeros()
    }

    /// A magnetometer reading heading zero whatever the vehicle does: right for a vehicle
    /// pointing north, and the fault for any other.
    fn north(filter: &mut Eskf) {
        let _ = filter.fuse_mag_heading(
            filter.now(),
            MagField::body(0.22, 0.0, 0.44),
            HeadingNoise::from_sigma(0.1),
        );
    }

    /// A static start whose window's magnetometer read north.
    fn pointing_north(config: Config) -> Eskf {
        let mut filter = Eskf::new(config).unwrap();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .unwrap();
        assert!(filter.state().validity.heading);
        filter
    }

    #[test]
    fn a_multirotor_with_no_heading_sensor_takes_its_heading_from_its_legs() {
        let mut filter = initialized();
        let mut flight = Flight::heading(2.2);
        flight.fly(&mut filter, 2.0, still, |_| {});
        assert_eq!(filter.state().status, Status::Aligning);
        assert_eq!(filter.diagnostics().yaw_estimator.adopted, 0);

        flight.fly(&mut filter, 12.0, legs, |_| {});
        let yaw = filter.diagnostics().yaw_estimator;
        assert_eq!((yaw.adopted, yaw.recovered), (1, 0));
        assert!(filter.state().validity.heading);
        assert_eq!(filter.state().status, Status::Healthy);
        assert!(flight.error(&filter) < 0.05, "{}", flight.error(&filter));
        assert!(flight.last.is_some_and(|fusion| fusion.is_accepted()));
    }

    #[test]
    fn a_cruise_with_no_acceleration_establishes_nothing() {
        let mut filter = initialized();
        let mut flight = Flight::heading(2.2);
        flight.fly(&mut filter, 2.0, still, |_| {});
        // At speed with no acceleration to get there: every hypothesis predicts the same
        // velocity, so the estimator fuses for twenty seconds and separates nothing.
        flight.velocity = Vector3::new(2.0, 0.0, 0.0);
        let noise = VelocityNoise::from_speed_accuracy(0.1);
        assert!(filter.reset_velocity_to(Velocity::from_vector(flight.velocity), noise));
        flight.fly(&mut filter, 20.0, still, |_| {});
        assert!(filter.yaw_estimator.yaw().is_some());
        assert_eq!(filter.diagnostics().yaw_estimator.adopted, 0);
        assert_eq!(filter.state().status, Status::Aligning);
    }

    #[test]
    fn with_the_estimator_off_nothing_is_stepped_weighed_or_adopted() {
        let mut filter = Eskf::new(Config {
            yaw_estimator: false,
            ..Config::default()
        })
        .unwrap();
        let _ = filter
            .initialize_over(&[crate::init::tests::still(); 8], Seconds::from_secs(0.25))
            .unwrap();
        let mut flight = Flight::heading(2.2);
        flight.fly(&mut filter, 2.0, still, |_| {});
        flight.fly(&mut filter, 12.0, legs, |_| {});
        assert_eq!(filter.yaw_estimator, YawEstimator::default());
        assert_eq!(filter.diagnostics().yaw_estimator, Default::default());
        assert!(!filter.state().validity.heading);
    }

    #[test]
    fn a_velocity_at_half_a_meter_a_second_of_sigma_is_not_weighed() {
        let mut filter = initialized();
        let mut flight = Flight::heading(2.2);
        flight.sigma = 0.5;
        flight.fly(&mut filter, 2.0, still, |_| {});
        flight.fly(&mut filter, 12.0, legs, |_| {});
        assert_eq!(filter.diagnostics().yaw_estimator.period(), None);
        assert_eq!(filter.yaw_estimator.yaw(), None);
    }

    #[test]
    fn a_heading_gnss_contradicts_is_replaced_and_the_velocity_adopted_with_it() {
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(1.4);
        flight.fly(&mut filter, 2.0, still, north);
        assert!(flight.error(&filter) > 1.3);

        flight.fly(&mut filter, 12.0, legs, north);
        let d = filter.diagnostics();
        assert_eq!((d.yaw_estimator.adopted, d.yaw_estimator.recovered), (1, 1));
        // Not on the first rejection: after a second of them, five at this rate.
        assert!(
            d.gnss_velocity.rejected >= 4,
            "{}",
            d.gnss_velocity.rejected
        );
        // The velocity that tripped it is adopted on the spot, not after its 7 s.
        assert_eq!(d.gnss_velocity.recovered, 1);
        assert!(flight.error(&filter) < 0.05, "{}", flight.error(&filter));
        // The magnetometer goes on reading north, and is turned down for it.
        assert!(d.mag_heading.consecutive_rejections > 10);
        assert!(flight.last.is_some_and(|fusion| fusion.is_accepted()));
    }

    #[test]
    fn a_receiver_fault_under_a_good_heading_leaves_the_yaw_alone() {
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(0.0);
        flight.fly(&mut filter, 2.0, still, north);
        flight.fly(&mut filter, 12.0, legs, north);
        // Three seconds of velocities 30 m/s out, too unsure of themselves for the estimator
        // to weigh: rejected for longer than the delay, with the two yaws in agreement.
        // Survives dropping the `YAW_FAILURE` test, which would turn the yaw here.
        let noise = VelocityNoise::from_speed_accuracy(0.6);
        let wild = Velocity::from_vector(flight.velocity + Vector3::new(30.0, 0.0, 0.0));
        hold(&mut filter, 3.0, 20, |filter| {
            let fusion = filter.fuse_gnss_velocity(filter.now(), wild, noise, Position::zero());
            assert!(matches!(fusion, Fusion::Rejected { .. }), "{fusion:?}");
        });
        assert_eq!(filter.diagnostics().yaw_estimator.adopted, 0);
        assert!(filter.yaw_estimator.yaw().is_some());
    }

    #[test]
    fn after_the_yaw_is_replaced_the_first_fix_the_gate_turns_down_is_adopted() {
        // A position recovery far past the flight, so only the latch can adopt.
        let mut filter = pointing_north(Config {
            recovery: Recovery {
                gnss_position: Some(Seconds::from_secs(1000.0)),
                ..Recovery::default()
            },
            ..Config::default()
        });
        let mut flight = Flight::heading(1.4);
        flight.fly(&mut filter, 2.0, still, north);
        flight.fly(&mut filter, 12.0, legs, north);
        assert_eq!(filter.diagnostics().yaw_estimator.recovered, 1);
        let far =
            Position::from_vector(filter.state().position.vector() + Vector3::new(500.0, 0.0, 0.0));
        let fix = |filter: &mut Eskf| {
            filter.fuse_gnss_position(filter.now(), far, one_metre(), Position::zero())
        };
        assert_eq!(fix(&mut filter).horizontal, Fusion::Reset);
        // Once: the next one the gate turns down waits out its own timeout.
        let _ = filter.reset_position_to(Position::zero(), one_metre());
        assert!(matches!(
            fix(&mut filter).horizontal,
            Fusion::Rejected { .. }
        ));

        // And a new start forgets a latch still set.
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(1.4);
        flight.fly(&mut filter, 2.0, still, north);
        flight.fly(&mut filter, 12.0, legs, north);
        assert!(filter.yaw_replaced.owes_position());
        let _ = filter.initialize_over(&window_with_mag(), Seconds::from_secs(0.25));
        assert_eq!(filter.yaw_replaced, super::YawReplaced::default());
    }

    #[test]
    fn an_estimator_begun_again_in_flight_does_not_overrule_until_it_settles() {
        // The fault of `a_heading_gnss_contradicts...`, which that test sees replaced within
        // twelve seconds, behind a coasted gap: eight seconds in, the estimator has converged
        // and GNSS is being rejected, and the yaw is still the magnetometer's.
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(1.4);
        flight.fly(&mut filter, 2.0, still, north);
        let gap = filter.step(crate::init::tests::still().imu, Seconds::from_secs(0.5));
        assert!(matches!(gap, Propagation::Coasted { .. }));
        flight.fly(&mut filter, 8.0, legs, north);
        assert!(filter.yaw_estimator.yaw().is_some_and(|(_, v)| v < 0.01));
        assert!(filter.diagnostics().gnss_velocity.rejected > 5);
        assert_eq!(filter.diagnostics().yaw_estimator.recovered, 0);
    }

    #[test]
    fn an_old_velocity_reaches_the_estimator_carried_to_now() {
        // Fixes 0.2 s old on a steady circle, where a velocity not carried lags the
        // acceleration by `ω τ`, 0.16 rad of yaw. Survives handing the estimator a zero carry.
        let mut filter = initialized();
        let mut flight = Flight::heading(0.9);
        flight.age = 20;
        flight.fly(&mut filter, 2.0, still, |_| {});
        flight.fly(&mut filter, 15.0, circling, |_| {});
        let (yaw, _) = filter.yaw_estimator.yaw().unwrap();
        let error = wrap_pi(yaw - 0.9);
        assert!(error.abs() < 0.03, "{error}");
    }

    #[test]
    fn a_second_antenna_that_is_being_accepted_is_not_overruled() {
        // The fault of `a_heading_gnss_contradicts...`, with a dual-antenna heading reading
        // north beside the magnetometer: the reference, by definition, so the yaw stays.
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(1.4);
        let both = |filter: &mut Eskf| {
            north(filter);
            let zero = crate::units::Radians::from_radians(0.0);
            let _ = filter.fuse_gnss_heading(filter.now(), zero, HeadingNoise::from_sigma(0.02));
        };
        flight.fly(&mut filter, 2.0, still, both);
        flight.fly(&mut filter, 12.0, legs, both);
        assert!(filter.diagnostics().gnss_velocity.rejected > 5);
        assert_eq!(filter.diagnostics().yaw_estimator.adopted, 0);
    }

    #[test]
    fn with_its_recovery_off_a_wrong_heading_is_reported_and_kept() {
        let mut filter = pointing_north(Config {
            recovery: Recovery {
                yaw_estimator: None,
                ..Recovery::default()
            },
            ..Config::default()
        });
        let mut flight = Flight::heading(1.4);
        flight.fly(&mut filter, 2.0, still, north);
        flight.fly(&mut filter, 6.0, legs, north);
        assert_eq!(filter.diagnostics().yaw_estimator.adopted, 0);
        assert!(filter.diagnostics().gnss_velocity.rejected > 0);
        assert!(flight.error(&filter) > 1.0);
    }

    #[test]
    fn a_coasted_gap_starts_the_estimator_over_and_it_must_settle_again() {
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(0.0);
        flight.fly(&mut filter, 2.0, still, north);
        flight.fly(&mut filter, 6.0, legs, north);
        assert!(filter.yaw_estimator.yaw().is_some());
        let gap = filter.step(crate::init::tests::still().imu, Seconds::from_secs(0.5));
        assert!(matches!(gap, Propagation::Coasted { .. }));
        assert_eq!(filter.yaw_estimator.yaw(), None);
        assert!(!filter.yaw_estimator.is_settled());
    }

    #[test]
    fn a_step_refused_after_the_clock_moved_starts_the_estimator_over() {
        // Five seconds the filter would not coast: the hypotheses missed whatever the vehicle
        // did in them. Left running, they would still hold a yaw, and a settled one.
        let mut filter = pointing_north(Config {
            coast: None,
            ..Config::default()
        });
        let mut flight = Flight::heading(0.0);
        flight.fly(&mut filter, 2.0, still, north);
        flight.fly(&mut filter, 6.0, legs, north);
        assert!(filter.yaw_estimator.yaw().is_some());
        let gap = filter.step(crate::init::tests::still().imu, Seconds::from_secs(5.0));
        assert!(matches!(gap, Propagation::StepTooLong { .. }));
        assert_eq!(filter.yaw_estimator.yaw(), None);
        assert!(!filter.yaw_estimator.is_settled());

        // A sample not after the clock moved nothing, and costs the estimator nothing.
        flight.fly(&mut filter, 4.0, legs, north);
        let held = filter.yaw_estimator.clone();
        let stale = crate::init::tests::still().imu.timed(filter.now(), DT);
        assert!(matches!(
            filter.predict(stale),
            Propagation::InvalidStep { .. }
        ));
        assert_eq!(filter.yaw_estimator, held);
    }

    #[test]
    fn a_first_heading_waits_for_an_estimator_begun_in_flight_to_settle() {
        let mut filter = initialized();
        let mut flight = Flight::heading(2.2);
        flight.fly(&mut filter, 2.0, still, |_| {});
        let gap = filter.step(crate::init::tests::still().imu, Seconds::from_secs(0.5));
        assert!(matches!(gap, Propagation::Coasted { .. }));
        // Converged and unsettled, eight seconds in: not adopted.
        flight.fly(&mut filter, 8.0, legs, |_| {});
        assert!(filter.yaw_estimator.yaw().is_some_and(|(_, v)| v < 0.01));
        assert_eq!(filter.diagnostics().yaw_estimator.adopted, 0);
        flight.fly(&mut filter, 8.0, legs, |_| {});
        assert_eq!(filter.diagnostics().yaw_estimator.adopted, 1);
    }

    #[test]
    fn a_start_in_motion_begins_an_unsettled_estimator() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&moving_window_at(100.0), Seconds::from_secs(0.25))
            .unwrap();
        assert!(!filter.yaw_estimator.is_settled());
        assert!(initialized().yaw_estimator.is_settled());
    }

    #[test]
    fn one_velocity_turned_down_is_not_a_yaw_failure() {
        // Velocities 1.2 s apart, so the first one rejected already meets the second's delay.
        // The yaw is replaced on a later one, and that first rejection stands as a rejection.
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(1.4);
        flight.every = 120;
        flight.fly(&mut filter, 2.4, still, north);
        flight.fly(&mut filter, 24.0, legs, north);
        let d = filter.diagnostics();
        assert_eq!(d.yaw_estimator.recovered, 1);
        assert!(
            d.gnss_velocity.rejected >= 1,
            "{}",
            d.gnss_velocity.rejected
        );
    }

    #[test]
    fn an_overruled_magnetometer_is_not_adopted_back_when_gnss_goes() {
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(1.4);
        flight.fly(&mut filter, 2.0, still, north);
        flight.fly(&mut filter, 12.0, legs, north);
        assert_eq!(filter.diagnostics().yaw_estimator.recovered, 1);
        // GNSS stops; the magnetometer goes on reading north for twelve seconds, past the
        // seven its recovery waits once no GNSS is fresh.
        hold(&mut filter, 12.0, 20, north);
        assert_eq!(filter.diagnostics().mag_heading.recovered, 0);
        assert!(flight.error(&filter) < 0.3, "{}", flight.error(&filter));
    }

    #[test]
    fn a_magnetometer_that_agrees_again_is_trusted_again() {
        let mut filter = pointing_north(Config::default());
        let mut flight = Flight::heading(1.4);
        flight.fly(&mut filter, 2.0, still, north);
        flight.fly(&mut filter, 12.0, legs, north);
        assert!(filter.yaw_replaced.overrules_magnetometer());
        // One reading the truth, 1.4 rad from north: body x points at heading 1.4, so the
        // field's horizontal part sits 1.4 rad to the left of it.
        let (sin, cos) = 1.4_f32.sin_cos();
        let field = MagField::body(0.22 * cos, -0.22 * sin, 0.44);
        let fusion = filter.fuse_mag_heading(filter.now(), field, HeadingNoise::from_sigma(0.1));
        assert!(fusion.is_accepted(), "{fusion:?}");
        assert!(!filter.yaw_replaced.overrules_magnetometer());
    }

    #[test]
    fn a_new_start_forgets_the_estimator() {
        let mut filter = initialized();
        let mut flight = Flight::heading(2.2);
        flight.fly(&mut filter, 2.0, still, |_| {});
        flight.fly(&mut filter, 12.0, legs, |_| {});
        let _ = filter
            .initialize_over(&[crate::init::tests::still(); 8], Seconds::from_secs(0.25))
            .unwrap();
        assert_eq!(filter.yaw_estimator, YawEstimator::default());
        assert_eq!(filter.yaw_replaced, super::YawReplaced::default());
    }
}
