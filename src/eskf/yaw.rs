//! Yaw without a heading sensor: the estimator of (45)–(52) fed from `predict` and
//! `fuse_gnss_velocity`, and its answer adopted as a first heading.
//!
//! No entry point: [`Config::yaw_estimator`](crate::Config::yaw_estimator) turns it on, and a
//! multirotor with no magnetometer and no second antenna calls nothing new.

use crate::config::{ALIGNED_HEADING, Config};
use crate::frames::{Body, Ned};
use crate::gsf::yaw_of;
use crate::health::{Diagnostics, SourceHealth};
use crate::math::wrap_pi;
use crate::observation::heading::has_heading;
use crate::propagate::ImuSample;
use crate::units::{Position, Radians, Timestamp, Velocity, VelocityNoise};

use nalgebra::Vector3;

use super::Eskf;

/// The composite yaw σ under which the estimator's answer is used: 15°, PX4's
/// `EKFGSF_yaw_err_max` (`src/modules/ekf2/EKF/common.h:396` at `c4e4ef98e9`) and ArduPilot's
/// `GSF_YAW_ACCURACY_THRESHOLD_DEG` (`AP_NavEKF3_core.h:78` at `368dc0c428`).
const YAW_SIGMA_MAX: Radians = Radians::from_degrees(15.0);

/// The velocity σ at or above which a solution is not weighed, m/s: PX4's `EKF2_REQ_SACC`
/// default (`params_gnss.yaml:117-121`), applied at `gps_control.cpp:388`. A receiver this
/// unsure of its velocity separates no hypotheses, and its error is not the white one (51)
/// takes it for.
const SPEED_SIGMA_MAX: f32 = 0.5;

/// How far the filter's yaw must sit from the estimator's before GNSS rejections are put down
/// to the heading: 25° (`isYawFailure`, `gps_control.cpp:561` at `c4e4ef98e9`).
const YAW_FAILURE: Radians = Radians::from_degrees(25.0);

/// Which GNSS horizontal sources have yet to be judged since the yaw they were being judged
/// against was replaced: each one's first measurement the gate turns down is adopted at once,
/// as after a position hold ([`recovery_after`](super::hold::recovery_after)).
///
/// PX4 resets velocity and position to GNSS with the yaw (`do_vel_pos_reset`,
/// `gps_control.cpp:124-128`): both were integrated through the wrong heading, and a gate
/// built on that covariance would hold the next fix off for its whole timeout.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct YawReplaced {
    pub(super) position: bool,
    pub(super) velocity: bool,
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

    /// A gap the filter coasted: the estimator integrated none of it, so its hypotheses no
    /// longer describe the vehicle, and it starts over.
    pub(super) fn restart_yaw_estimator(&mut self) {
        if self.config.yaw_estimator {
            self.yaw_estimator.restart();
        }
    }

    /// Weigh the yaw hypotheses against a GNSS velocity, and adopt their answer where heading
    /// was never established. Equations (49)–(52).
    ///
    /// Called by [`fuse_gnss_velocity`](Self::fuse_gnss_velocity) with a solution it has
    /// screened, before the main filter judges it: the estimator's use is the case where that
    /// judgment is made against a wrong yaw. The velocity is counted once in each estimator,
    /// which is why the answer enters the main filter as an adoption and never as a
    /// measurement fused beside the velocity it was read from.
    ///
    /// The adoption is the one every first heading takes
    /// ([`fuse_mag_heading`](Self::fuse_mag_heading)), at the composite variance of (52), once
    /// its σ is under [`YAW_SIGMA_MAX`]: `resetYawToEKFGSF`, `gps_control.cpp:564-579` at
    /// `c4e4ef98e9`. Counted in
    /// [`Diagnostics::yaw_estimator`](crate::Diagnostics::yaw_estimator) as adopted.
    #[inline(never)]
    pub(super) fn weigh_yaw(
        &mut self,
        time: Timestamp,
        velocity: Velocity<Ned>,
        noise: VelocityNoise<Ned>,
        antenna: Position<Body>,
    ) {
        if !self.config.yaw_estimator {
            return;
        }
        let [north, east, _] = noise.variance();
        let variance = north.max(east);
        if variance >= SPEED_SIGMA_MAX * SPEED_SIGMA_MAX {
            return;
        }
        // (49): how far the estimate's velocity has moved since `time`, less the antenna's
        // swing, in the main filter's axes.
        let Some(carried) = self.carried_velocity(Velocity::zero(), antenna, time) else {
            return;
        };
        let heading = yaw_of(self.estimate.state().attitude.quaternion());
        self.diagnostics.yaw_estimator.note_arrival(time);
        self.yaw_estimator.fuse_velocity(
            velocity.vector().xy(),
            variance,
            carried.vector().xy(),
            heading,
        );
        if self.unestablished.heading && self.turn_to_estimated_yaw(|_| true) {
            self.diagnostics.yaw_estimator.record_adopted();
            self.note_alignment();
            self.magnetic_north = false;
        }
    }

    /// Replace a heading GNSS contradicts with the yaw estimator's. Called with a GNSS
    /// horizontal position or velocity the gate has just turned down, `source` naming which.
    ///
    /// A yaw that is wrong turns every acceleration the wrong way, so velocity and position
    /// leave the truth and GNSS is rejected for it, while the heading source that put the yaw
    /// there goes on agreeing with it. Recovering the GNSS source alone, after its own
    /// timeout, adopts a fix and leaves the cause. The estimator never read that heading:
    /// where it has converged more than [`YAW_FAILURE`] from the filter's yaw, and `source` has
    /// gone unaccepted for [`Recovery::yaw_estimator`](crate::Recovery::yaw_estimator), the
    /// yaw is the estimator's from here (`tryYawEmergencyReset`, `gps_control.cpp:426-444` at
    /// `c4e4ef98e9`). Counted in
    /// [`Diagnostics::yaw_estimator`](crate::Diagnostics::yaw_estimator) as recovered.
    ///
    /// PX4 also resets the variance of the gyroscope bias about body z
    /// (`resetGyroBiasZCov`, `gps_control.cpp:440`). Not done here: [`reset_heading_by`]
    /// drops the heading's correlation with the bias, and `yaw_fault` settles without it.
    ///
    /// [`reset_heading_by`]: Self::reset_heading_by
    #[inline(never)]
    pub(super) fn replace_failed_yaw(&mut self, source: fn(&Diagnostics) -> &SourceHealth) {
        if !self.config.yaw_estimator || !self.yaw_estimator.is_settled() {
            return;
        }
        let after = self.config.recovery.yaw_estimator;
        let since_initialized = self.diagnostics.since_initialized;
        if !source(&self.diagnostics).locked_out(after, since_initialized) {
            return;
        }
        let disagrees = |y: f32| y.abs() > YAW_FAILURE.as_radians();
        if self.turn_to_estimated_yaw(disagrees) {
            self.diagnostics.yaw_estimator.record_recovered();
            self.note_alignment();
            self.magnetic_north = false;
            self.yaw_replaced = YawReplaced {
                position: true,
                velocity: true,
            };
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
        if !(variance < bar * bar) || !has_heading(self.estimate.state()) {
            return false;
        }
        let attitude = self.estimate.state().attitude.quaternion();
        let y = wrap_pi(yaw - yaw_of(attitude));
        if !(y.is_finite() && wanted(y)) {
            return false;
        }
        self.reset_heading_by(y, variance, attitude.inverse() * Vector3::z());
        true
    }
}
