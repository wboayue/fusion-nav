//! Yaw without a heading sensor: the estimator of (45)–(52) fed from `predict` and
//! `fuse_gnss_velocity`, and its answer adopted as a first heading.
//!
//! No entry point: [`Config::yaw_estimator`](crate::Config::yaw_estimator) turns it on, and a
//! multirotor with no magnetometer and no second antenna calls nothing new.

use crate::config::{ALIGNED_HEADING, Config};
use crate::frames::{Body, Ned};
use crate::gsf::yaw_of;
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
        if self.unestablished.heading && self.adopt_estimated_yaw() {
            self.diagnostics.yaw_estimator.record_adopted();
            self.note_alignment();
            self.magnetic_north = false;
        }
    }

    /// Turn the estimate to the yaw estimator's answer, if it has one good enough and the
    /// vehicle's forward axis names a heading. Returns whether it did.
    fn adopt_estimated_yaw(&mut self) -> bool {
        let Some((yaw, variance)) = self.yaw_estimator.yaw() else {
            return false;
        };
        let bar = YAW_SIGMA_MAX.as_radians();
        if !(variance < bar * bar) || !has_heading(self.estimate.state()) {
            return false;
        }
        let attitude = self.estimate.state().attitude.quaternion();
        let y = wrap_pi(yaw - yaw_of(attitude));
        if !y.is_finite() {
            return false;
        }
        self.reset_heading_by(y, variance, attitude.inverse() * Vector3::z());
        true
    }
}
