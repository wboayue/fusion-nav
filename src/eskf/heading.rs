//! The heading sources, (36): magnetic heading, (34)–(35), dual-antenna GNSS heading, (35′), and
//! course over ground, (35″), with the adoption that turns attitude by (41).
//!
//! Entry points: [`Eskf::fuse_mag_heading`], [`Eskf::fuse_gnss_heading`] and
//! [`Eskf::fuse_course`].

use crate::config::Gate;
use crate::frames::Body;
use crate::health::{Diagnostics, Fusion, SourceHealth};
use crate::math::{correlation_inflation, exp_quat};
use crate::observation::{heading, mag};
use crate::state::{Covariance, State};
use crate::units::{Attitude, HeadingNoise, MagField, Radians, Seconds, Timestamp};
use crate::update::{self, Observation, update};
use nalgebra::Vector3;

use super::Eskf;
use super::fuse::refuse;

impl Eskf {
    /// Fuse magnetic heading from a calibrated three-axis magnetometer.
    /// Equations (34)–(36).
    ///
    /// Heading only: the field is reduced to one scalar, so a magnetic disturbance can
    /// corrupt yaw but cannot reach roll or pitch. `noise` is on the resulting
    /// heading, not on the field components.
    ///
    /// This is also where heading becomes an estimate. A static window with no
    /// magnetometer in it, and any coarse start, leave yaw unobserved — see
    /// [`initialize`](Self::initialize) — and [`validity`](Self::validity) reports
    /// `heading` false until the first heading is accepted, here or from
    /// [`fuse_gnss_heading`](Self::fuse_gnss_heading) or [`fuse_course`](Self::fuse_course),
    /// however tight [`sigma_yaw`](crate::Initialization::sigma_yaw) was.
    ///
    /// That first heading is the point `GOALS.md` names for a yaw **reset** rather than an
    /// ordinary update: the error-state attitude of equation (2) is a small-angle
    /// quantity, so a yaw error of a radian is wrong in a way no variance expresses, and
    /// widening the prior does not fix it. It is adopted rather than gated, which is
    /// [`Fusion::Reset`]'s documented shape — a quantity never established, with no
    /// estimate to step away from. Gating it instead would judge the measurement against
    /// a prior nothing ever measured: a static window with no magnetometer leaves
    /// [`sigma_yaw`](crate::Initialization::sigma_yaw), 0.35 rad, on a yaw that is a
    /// guess, and a correct heading more than about 64° from that guess would be
    /// rejected — locking the filter out of what may be the one source that can ever
    /// establish yaw.
    ///
    /// Ordinary updates thereafter, gated at [`Gates::mag_heading`](crate::Gates) with
    /// one degree of freedom, until a run of rejections outlasts
    /// [`Recovery::mag_heading`](crate::Recovery::mag_heading) while no GNSS position,
    /// velocity or heading is arriving,
    /// when the next heading is adopted the same way. The field is reduced to a scalar heading
    /// before the gate sees it, so a disturbance is tested as the yaw error it is.
    ///
    /// The field must be calibrated: this crate corrects no hard- or soft-iron error and
    /// carries no magnetic-field states to absorb one, so a bias in `field` is a bias in
    /// heading. A field of exactly zero is finite, and reports the vehicle as `D_m` off
    /// rather than producing NaN — gateable once yaw is established, and adopted where it
    /// is not, which is the strongest reason to check a magnetometer before the first
    /// call rather than after it.
    ///
    /// Fused at the variance (24′) leaves for a heading whose error persists from the last,
    /// [`Config::correlation`](crate::Config::correlation)'s `mag_heading`;
    /// the gate reads `noise` itself.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_mag_heading(
        &mut self,
        time: Timestamp,
        field: MagField<Body>,
        noise: HeadingNoise,
    ) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.mag_heading, refusal);
        }
        self.diagnostics.mag_heading.note_arrival(time);
        if !field.is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.mag_heading, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.mag_heading, Fusion::InvalidNoise);
        }
        let declination = self.declination;
        // With GNSS arriving, a magnetometer that disagrees this long is more likely disturbed
        // than right: horizontal aiding is already correcting heading through (20), and a GNSS
        // heading measures it outright.
        let d = &self.diagnostics;
        let recovery = self.unless_accepted(
            self.config.recovery.mag_heading,
            &[&d.gnss_position, &d.gnss_velocity, &d.gnss_heading],
        );
        // Nor after the yaw estimator replaced the heading, until a magnetic heading agrees
        // with the new one: see `YawReplaced`.
        let recovery = recovery.filter(|_| !self.yaw_replaced.overrules_magnetometer());
        let source = HeadingSource {
            health: |diagnostics| &mut diagnostics.mag_heading,
            gate: self.config.gates.mag_heading,
            correlation: self.config.correlation.mag_heading,
            recovery,
            magnetic: true,
        };
        let fusion = self.fuse_heading(time, source, 0.0, |past, covariance| {
            mag::heading_observation(past, covariance, field, declination, noise)
        });
        if matches!(fusion, Fusion::Accepted { .. }) {
            self.yaw_replaced.settle_magnetometer();
        }
        fusion
    }

    /// Fuse a true heading from a dual-antenna (moving-baseline) GNSS receiver.
    /// Equations (35′) and (36).
    ///
    /// The heading of the line from the primary antenna to the secondary, which is taken to
    /// lie along body x: a receiver whose antennas are mounted otherwise has the mounting
    /// angle subtracted by the caller first, as PX4 and ArduPilot each do from a parameter.
    /// True rather than magnetic, so no [declination](Self::set_magnetic_declination) applies,
    /// and nothing is leveled with the estimated attitude, so the (36′) a magnetic heading
    /// carries has no counterpart; see `observation/heading.rs` for the model and why its
    /// Jacobian is (36) rather than the exact one PX4 differentiates.
    ///
    /// `noise` is the receiver's `heading_accuracy`, bounded before it arrives:
    /// [`HeadingNoise::clamped`](crate::HeadingNoise::clamped) holds PX4's and ArduPilot's
    /// floors, and both production estimators apply one because a moving-baseline solution is
    /// as optimistic about itself as a position fix is.
    ///
    /// The first heading after any start that observed no yaw is adopted rather than gated,
    /// as a magnetic one is — [`Fusion::Reset`], with this `noise` as the adopted variance —
    /// and both production estimators align on the first sample the same way (PX4
    /// `aid_sources/gnss/gnss_yaw_control.cpp:100-111` at `c4e4ef98`, ArduPilot
    /// `AP_NavEKF3_MagFusion.cpp:404-410` at `368dc0c4`). Ordinary updates thereafter, gated at
    /// [`Gates::gnss_heading`](crate::Gates), and recovered after
    /// [`Recovery::gnss_heading`](crate::Recovery::gnss_heading).
    ///
    /// Fused beside a magnetometer rather than instead of one, where PX4 stops fusing the
    /// magnetometer while GNSS yaw is active (`aid_sources/magnetometer/mag_control.cpp:189`):
    /// each is gated against an estimate the other has corrected, and a magnetometer that
    /// disagrees is not recovered while this is accepted. A body x within 30° of vertical has
    /// no heading to compare, and is refused as [`Fusion::Unobservable`].
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_gnss_heading(
        &mut self,
        time: Timestamp,
        heading: Radians,
        noise: HeadingNoise,
    ) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.gnss_heading, refusal);
        }
        self.diagnostics.gnss_heading.note_arrival(time);
        if !heading.as_radians().is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.gnss_heading, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.gnss_heading, Fusion::InvalidNoise);
        }
        if !heading::has_heading(&self.past(time).0) {
            return refuse(&mut self.diagnostics.gnss_heading, Fusion::Unobservable);
        }
        let source = HeadingSource {
            health: |diagnostics| &mut diagnostics.gnss_heading,
            gate: self.config.gates.gnss_heading,
            correlation: self.config.correlation.gnss_heading,
            recovery: self.config.recovery.gnss_heading,
            magnetic: false,
        };
        self.fuse_heading(time, source, 0.0, |past, _| {
            heading::gnss_observation(past, heading, noise)
        })
    }

    /// Constrain heading to the direction of travel: the vehicle points along its estimated
    /// velocity, to within `sideslip`. Equations (35″) and (36).
    ///
    /// Yaw from course over ground, for a vehicle with no magnetometer or one it cannot trust.
    /// It is a constraint rather than a measurement: nothing new is read, and what the call
    /// states is that the nose and the track agree, so it is only as true as the vehicle makes
    /// it. A fixed-wing in coordinated flight and a ground vehicle on its wheels hold it to a
    /// few degrees; a crosswind adds its crab angle to `sideslip`. **A multirotor holds no such
    /// relation** — it hovers, and flies in any direction facing any other — and must not call
    /// this.
    ///
    /// The velocity is the estimate's, not a GNSS velocity handed in: that velocity has already
    /// been fused through [`fuse_gnss_velocity`](Self::fuse_gnss_velocity), and taking a course
    /// from it again would count its cross-track error twice. Read off the state, the
    /// velocity's uncertainty reaches the update through `P`, and the update corrects velocity
    /// and heading together. So `time` is when the constraint is claimed to hold, and a caller
    /// fusing one per GNSS velocity passes that fix's time.
    ///
    /// Refused as [`Fusion::NoReference`] where no fresh GNSS velocity holds the estimated one
    /// ([`SourceHealth::is_fresh`](crate::SourceHealth::is_fresh)), which also covers one never
    /// established, and as [`Fusion::Unobservable`] where it names no direction: too slow
    /// against its own uncertainty, which is ArduPilot's 15° bar
    /// on the course read from `P` — see `observation/heading.rs` — and so a speed threshold
    /// the velocity's accuracy sets rather than a parameter. Refused the same way with body x
    /// within 30° of vertical. ArduPilot's plane realigns yaw from course above 5 m/s
    /// (`realignYawGPS`, `AP_NavEKF3_MagFusion.cpp:145-218` at `368dc0c4`); PX4 has no course
    /// source and reaches the same vehicle through its GSF yaw estimator.
    ///
    /// The first course after a start that observed no yaw is adopted, as a first heading is,
    /// at `sideslip`'s variance plus the course's own; ordinary updates thereafter, continuously
    /// rather than ArduPilot's once, gated at [`Gates::course`](crate::Gates) and recovered
    /// after [`Recovery::course`](crate::Recovery::course) while no other heading is accepted.
    ///
    /// The adoption keeps none of the heading's correlation with velocity, though the adopted
    /// heading error is the course error, `∇χᵀδv`, plus the sideslip. Dropping it overstates
    /// the next update's `S` by about `2σ_χ²`, which is conservative; what it costs is that a
    /// velocity fix cannot correct the part of the heading error the velocity lent it until
    /// propagation correlates the two again.
    ///
    /// Not a source [`Status`](crate::Status) counts: it reads the filter's own velocity, so it
    /// aids nothing the GNSS velocity it needs does not, and a vehicle slowing to a stop, where
    /// every course is refused, is not degraded for it.
    pub fn fuse_course(&mut self, time: Timestamp, sideslip: HeadingNoise) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.course, refusal);
        }
        self.diagnostics.course.note_arrival(time);
        if !sideslip.is_finite() {
            return refuse(&mut self.diagnostics.course, Fusion::NotFinite);
        }
        if !sideslip.is_positive() {
            return refuse(&mut self.diagnostics.course, Fusion::InvalidNoise);
        }
        // A velocity no GNSS velocity is holding is a dead-reckoned one, and a course along it
        // would read as aiding while it drifts. It also covers a velocity never established,
        // which only a GNSS velocity's acceptance or `reset_velocity_to` establishes.
        if !self
            .diagnostics
            .gnss_velocity
            .is_fresh(&self.config.timeouts)
        {
            return refuse(&mut self.diagnostics.course, Fusion::NoReference);
        }
        // Screened on the state the observation is built on, the one at `time`.
        let past = self.past(time).0;
        let spread = heading::course_variance(&past, &self.covariance)
            .filter(|_| heading::has_heading(&past));
        let Some(spread) = spread else {
            return refuse(&mut self.diagnostics.course, Fusion::Unobservable);
        };
        // With a heading source arriving, a course that disagrees this long is sideslip the
        // caller did not allow for, not a wrong heading.
        let d = &self.diagnostics;
        let recovery = self.unless_accepted(
            self.config.recovery.course,
            &[&d.mag_heading, &d.gnss_heading],
        );
        let source = HeadingSource {
            health: |diagnostics| &mut diagnostics.course,
            gate: self.config.gates.course,
            correlation: self.config.correlation.course,
            recovery,
            magnetic: false,
        };
        self.fuse_heading(time, source, spread, |past, _| {
            heading::course_observation(past, sideslip)
        })
    }

    /// The update every heading source shares, once its measurement has been screened:
    /// formed at `time` by (23′), fused at (24′)'s variance, adopted where heading was never
    /// established, and otherwise gated and recovered.
    ///
    /// `spread` is the variance an adoption carries beyond `R`: zero for a source whose `H`
    /// reads attitude alone, and the course's own for the constraint, whose `H` reads velocity
    /// too and whose adopted heading is only as good as the velocity it was taken along.
    ///
    /// Out of line for the reason [`observe`](Self::observe) is; the scalar sources' peak stays
    /// under `fuse_gnss_velocity`'s ([measured]).
    ///
    /// [measured]:
    /// https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#measured-cost-by-function
    #[inline(never)]
    fn fuse_heading(
        &mut self,
        time: Timestamp,
        source: HeadingSource,
        spread: f32,
        build: impl FnOnce(&State, &Covariance) -> Observation<1>,
    ) -> Fusion {
        let since_measured = (source.health)(&mut self.diagnostics).since_measured;
        let observation = self
            .observe(time, |past, _| build(past, &self.covariance))
            .correlated(correlation_inflation(since_measured, source.correlation));
        if self.unestablished.heading {
            if !self.adopt_heading(&observation, spread) {
                return refuse((source.health)(&mut self.diagnostics), Fusion::NotFinite);
            }
            (source.health)(&mut self.diagnostics).record_adopted();
            self.note_alignment();
            self.magnetic_north = source.magnetic;
            return Fusion::Reset;
        }
        let outcome = update(
            self.estimate.state(),
            &self.covariance,
            &self.offset,
            &observation,
            source.gate,
        );
        let fused = self.apply_or_recover(outcome, source.health, source.recovery, |filter| {
            filter.adopt_heading(&observation, spread)
        });
        // An adoption replaces the heading with this source's; a true heading accepted means
        // the estimate is no longer referred to north through the declination alone.
        match fused {
            Fusion::Reset => self.magnetic_north = source.magnetic,
            Fusion::Accepted { .. } if !source.magnetic => self.magnetic_north = false,
            _ => {}
        }
        fused
    }

    /// `recovery`, unless any of `arbiters` was accepted recently: a source that disagrees
    /// with better aiding that is arriving is the one at fault, and adopting it would step the
    /// estimate away from what the others say.
    pub(super) fn unless_accepted(
        &self,
        recovery: Option<Seconds>,
        arbiters: &[&SourceHealth],
    ) -> Option<Seconds> {
        if arbiters
            .iter()
            .any(|arbiter| arbiter.is_fresh(&self.config.timeouts))
        {
            None
        } else {
            recovery
        }
    }

    /// Adopt a heading: [`reset_heading_by`](Self::reset_heading_by) with the `y` and `R`
    /// an ordinary update would read. (36′) is what makes that worth saying: the leveling
    /// error is priced on the path where the tilt it comes from is worst.
    ///
    /// `spread` is added to `R`: see [`fuse_heading`](Self::fuse_heading).
    ///
    /// Returns whether it adopted: a variance or an innovation that is not finite commits
    /// nothing. Both are finite products of a finite covariance, and can still overflow: the
    /// leveling variance of (36′) squares a tilt σ a coarse start charged a 1e20 rad/s
    /// gyroscope reading to.
    fn adopt_heading(&mut self, observation: &Observation<1>, spread: f32) -> bool {
        // Navigation down in body axes, `R(q̂)ᵀe₃`, of the present state: the axis the
        // adoption turns about. Not read off the observation's attitude row, which (23′) carries
        // through the error dynamics: a course's gains a tilt component from its velocity block,
        // `∇χᵀR[a_b]× τ`, about 0.06 at 18 m/s and 110 ms, and stops being the unit vector
        // `reset_attitude_direction` needs.
        let down = self.estimate.state().attitude.quaternion().inverse() * Vector3::z();
        let (y, variance) = (observation.y[0], observation.r_m[0] + spread);
        if !(y.is_finite() && variance.is_finite()) {
            return false;
        }
        self.reset_heading_by(y, variance, down);
        true
    }

    /// Turn the estimate by a yaw error and give the result the measurement's variance:
    /// the adoption behind [`Fusion::Reset`] for heading.
    ///
    /// `y` is the innovation of (35), (35′) or (35″), so the corrected attitude is `Exp(y e₃) ⊗ q̂`
    /// — composed on the **left**, because `e₃` is the navigation down axis, where the `δθ` of (2)
    /// that `update` injects is a body-frame rotation composed on the right. Tilt is untouched: a
    /// rotation about navigation down moves the tilt axis and not the tilt angle, so the roll and
    /// pitch gravity established survive a heading any heading source supplies.
    ///
    /// `Exp` charges its caller with a finite argument, which each innovation's wrap
    /// discharges by construction: `y` is in `(-π, π]` whatever was measured and the attitude
    /// were.
    ///
    /// `variance` is the `R` the source's update would read, plus what the heading inherits
    /// that `R` does not carry: for the course, the uncertainty of the velocity it was taken
    /// along ([`fuse_heading`](Self::fuse_heading)'s `spread`). For a magnetic heading it is
    /// `R` from (36′) rather than the caller's `σ_ψ²` alone. The leveling
    /// of (34) is done with the estimated attitude on this path too — on a coarse start,
    /// with the worst tilt the filter ever holds — so an adoption that stored the
    /// magnetometer's own number would report a heading good to
    /// [`Accuracy::heading`](crate::Accuracy::heading) while carrying the window's
    /// leveling error times `tan δ`. That is the falsely-valid attitude (36′) exists to
    /// remove, and the adoption is where it is largest.
    ///
    /// The correlations go with it, which is what fusing against an infinitely uncertain
    /// prior converges to — the same limit [`reset_position_to`](Self::reset_position_to)
    /// takes, one component wide. That component is the rotation about navigation down,
    /// `uᵀδθ` with `u = R(q̂)ᵀe₃` in body axes, not `δθ_z`: the two agree only while the
    /// vehicle is level, and at 90° of pitch `δθ_z` is a tilt. The rotation above leaves
    /// `e₃` where it was, so `u` is the same before the adoption and after it.
    ///
    /// What survives — the tilt block, the biases, and the correlations between them — is
    /// reparameterized first, by the (41) of `update::reparameterize`. `δθ` is referenced
    /// to the nominal's body axes, so `R(q̂⁺)ᵀ R(q̂)` is the exact change of frame this
    /// rotation makes, and it is not the small one (41) approximates: a first heading can
    /// turn the estimate by half a circle. The tilt block is near-isotropic and largely
    /// survives it, but the bias blocks are in physical body axes that do **not** turn, so
    /// their correlations with attitude transform on one side only — left unrotated, a
    /// roll-error/gyro-bias-x correlation is read afterwards as roll-error/gyro-bias-y and
    /// the next velocity update pushes the correction into the wrong axis.
    pub(super) fn reset_heading_by(&mut self, y: f32, variance: f32, down: Vector3<f32>) {
        let before = self
            .estimate
            .state()
            .attitude
            .quaternion()
            .to_rotation_matrix();
        let mut corrected =
            exp_quat(Vector3::z() * y) * self.estimate.state().attitude.quaternion();
        corrected.renormalize();
        self.estimate.commit(State {
            attitude: Attitude::from_quaternion(corrected),
            ..*self.estimate.state()
        });

        let g_theta = corrected.to_rotation_matrix().inverse() * before;
        let g_theta = g_theta.into_inner();
        let mut covariance = update::reparameterize(*self.covariance.as_matrix(), g_theta);
        let mut offset = update::reparameterize_offset(&self.offset, g_theta);
        covariance.reset_attitude_direction(down, variance);
        offset.decorrelate_attitude_direction(down);
        self.commit_covariance(covariance, offset);
        self.unestablished.heading = false;
    }
}

/// What distinguishes one heading source from another inside
/// [`Eskf::fuse_heading`]: where its health is kept, and the gate, `τ` and recovery
/// [`Config`](crate::Config) gives it.
struct HeadingSource {
    health: fn(&mut Diagnostics) -> &mut SourceHealth,
    gate: Gate<1>,
    correlation: Option<Seconds>,
    recovery: Option<Seconds>,
    /// Whether the heading is magnetic, referred to true north through the declination:
    /// what [`Eskf::place_origin`] reads to decide whether a learned declination turns it.
    magnetic: bool,
}

#[cfg(test)]
mod tests {

    use crate::config::Correlation;
    use crate::eskf::Eskf;
    use crate::eskf::fixtures::*;
    use crate::frames::Body;

    use crate::health::{Fusion, Propagation, Refusal};
    use crate::init::Alignment;
    use crate::init::tests::{still, turning};

    use crate::observation::mag::tests::{attitude_of, measured};
    use crate::observation::{heading, mag};

    use crate::state::{AttitudeVariance, Covariance, ErrorState, STATES, State};
    use crate::units::{
        Attitude, HeadingNoise, MagField, Position, Radians, Seconds, Velocity, VelocityNoise,
    };

    use nalgebra::Vector3;

    #[test]
    fn a_static_window_without_a_magnetometer_leaves_heading_unestablished() {
        // The covariance on its own would say otherwise: `Initialization::sigma_yaw` is
        // 0.35 rad against an `Accuracy::heading` of 0.52, so the prior clears the bar
        // comfortably. It is a prior on a yaw nothing ever observed.
        let filter = initialized();
        let validity = filter.validity();
        assert!(validity.tilt, "gravity pins roll and pitch");
        assert!(!validity.heading, "nothing pins the rotation about gravity");
        assert!(!filter.is_aligned());
    }

    #[test]
    fn the_first_accepted_magnetic_heading_establishes_yaw() {
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    MagField::body(0.22, 0.0, 0.44),
                    HeadingNoise::from_sigma(0.1),
                )
                .is_accepted()
        );
        assert!(filter.validity().heading, "something observed it at last");
        assert!(filter.is_aligned());
    }

    #[test]
    fn the_first_magnetic_heading_is_adopted_and_every_one_after_it_is_fused() {
        // A quantity never established is adopted once; after that only a lockout adopts
        // again, and a heading that agrees is no lockout.
        let mut filter = initialized();
        let field = measured(attitude_of(0.0, 0.0, 1.1), 0.0);
        let noise = HeadingNoise::from_sigma(0.05);

        assert!(
            filter
                .fuse_mag_heading(filter.now(), field, noise)
                .is_reset()
        );
        assert_eq!(filter.diagnostics().mag_heading.adopted, 1);

        let second = filter.fuse_mag_heading(filter.now(), field, noise);
        assert!(!second.is_reset(), "adoption happens once: {second:?}");
        assert!(matches!(second, Fusion::Accepted { .. }), "{second:?}");
        assert_eq!(filter.diagnostics().mag_heading.adopted, 1);
    }

    #[test]
    fn the_adopted_yaw_is_the_heading_the_field_reports() {
        // The window leveled with no magnetometer, so yaw starts at zero against a
        // field that says 1.1 rad. A gradual correction would leave it somewhere between
        // the two; an adoption puts it at the measurement.
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 1.1), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw - 1.1).abs() < 1e-4, "adopted {yaw} rather than 1.1");
    }

    #[test]
    fn the_adoption_turns_the_estimate_about_down_and_touches_nothing_else() {
        // A heading is yaw and nothing else: the tilt gravity established and the biases
        // the window measured are the same after it, and so are their variances. What
        // makes this worth asserting is that the rotation is applied to the whole
        // quaternion — about navigation down, where it moves the tilt axis and leaves
        // the tilt angle — rather than to a yaw component.
        let mut filter = initialized();
        let before = filter.state();
        let (roll, pitch, _) = before.attitude.euler_angles();
        let tilt_variance = filter.covariance().variance(ErrorState::AttitudeX);

        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, -2.4), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );

        let after = filter.state();
        let (roll_after, pitch_after, _) = after.attitude.euler_angles();
        assert!((roll_after - roll).abs() < 1e-5, "roll moved");
        assert!((pitch_after - pitch).abs() < 1e-5, "pitch moved");
        assert_eq!(after.gyro_bias, before.gyro_bias);
        assert_eq!(after.accel_bias, before.accel_bias);
        // Within an ulp rather than exactly: the adoption reparameterizes the attitude
        // block by (41), and a 2.4 rad rotation of an isotropic `v I` reconstructs `v`
        // through `v(c² + s²)`. A tolerance wide enough to hide a heading's variance
        // landing on the tilt would be four orders of magnitude wider than this.
        assert!(
            (filter.covariance().variance(ErrorState::AttitudeX) - tilt_variance).abs() < 1e-9,
            "the tilt's own uncertainty is not the heading's to replace"
        );
    }

    #[test]
    fn the_adoption_carries_the_attitude_correlations_into_the_new_body_axes() {
        // (41) on the adoption path. The error state of (2) is referenced to the
        // nominal's body axes, so turning the nominal turns the axes every attitude row
        // is written in — and the bias states it is correlated with are in physical body
        // axes that do not turn, so their cross-blocks transform on one side only.
        //
        // Propagation is what builds one: (20)'s `−I Δt` makes P_θ,βg ≈ −Δt·P_βg,βg, the
        // same correlation that lets a velocity update correct the gyroscope bias. A
        // quarter turn about a level vehicle's down axis maps the roll-error column onto
        // the pitch-error one, so the correlation the x bias had with roll error is the
        // one the y bias should hold afterwards. Skip the reparameterization and it stays
        // where it was, and the next velocity update pushes the bias correction into the
        // wrong axis.
        let mut filter = initialized();
        let holding_still = still().imu;
        for _ in 0..2_000 {
            assert_eq!(
                filter.step(holding_still, Seconds::from_secs(0.005)),
                Propagation::Propagated
            );
        }

        let before = filter
            .covariance()
            .get(ErrorState::AttitudeX, ErrorState::GyroBiasX);
        assert!(
            before.abs() > 1e-9,
            "10 s of (20) should build one: {before}"
        );
        assert!(
            filter
                .covariance()
                .get(ErrorState::AttitudeX, ErrorState::GyroBiasY)
                .abs()
                < 1e-12,
            "and nothing across the axes yet"
        );

        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, core::f32::consts::FRAC_PI_2), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );

        let after = filter.covariance();
        assert!(
            (after.get(ErrorState::AttitudeX, ErrorState::GyroBiasY) - before).abs()
                < 1e-9 * before.abs().max(1.0),
            "the quarter turn should have moved it to the y bias, got {}",
            after.get(ErrorState::AttitudeX, ErrorState::GyroBiasY)
        );
        assert!(
            after
                .get(ErrorState::AttitudeX, ErrorState::GyroBiasX)
                .abs()
                < 1e-9,
            "and off the x bias, got {}",
            after.get(ErrorState::AttitudeX, ErrorState::GyroBiasX)
        );
    }

    #[test]
    fn a_refused_first_heading_establishes_nothing() {
        // A refusal is not a measurement the filter chose to believe, so the quantity is
        // still unestablished and the next usable field is still the one adopted.
        let mut filter = initialized();
        assert_eq!(
            filter.fuse_mag_heading(
                filter.now(),
                MagField::body(f32::NAN, 0.0, 0.44),
                HeadingNoise::from_sigma(0.05),
            ),
            Fusion::NotFinite
        );
        assert!(!filter.validity().heading);
        assert_eq!(filter.diagnostics().mag_heading.adopted, 0);

        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 1.1), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset(),
            "the adoption was waiting for a usable field, not spent on a refused one"
        );
    }

    #[test]
    fn a_heading_far_from_an_established_yaw_is_rejected_and_changes_nothing() {
        // Once yaw is established the gate has a prior worth testing against, and this
        // is the disturbance case: a field turned a quarter circle against a σ of
        // 0.05 rad is hundreds of times the threshold of (37).
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );
        let before = filter.state().attitude;

        let outcome = filter.fuse_mag_heading(
            filter.now(),
            measured(attitude_of(0.0, 0.0, core::f32::consts::FRAC_PI_2), 0.0),
            HeadingNoise::from_sigma(0.05),
        );
        assert!(
            matches!(outcome, Fusion::Rejected { test_ratio } if test_ratio > 1.0),
            "{outcome:?}"
        );
        assert_eq!(filter.state().attitude, before, "a rejection costs nothing");
        assert_eq!(filter.diagnostics().mag_heading.rejected, 1);
    }

    #[test]
    fn an_ordinary_heading_moves_yaw_part_of_the_way_and_shrinks_its_variance() {
        // The other side of the adoption: once established, a heading is fused rather
        // than taken. The distance it moves is (25)'s gain with (36′) in both `P` and
        // `R`, and the whole sum is checkable by hand — the window's tilt prior is
        // `sigma_tilt` = 0.02 and the dip is 2.0, so the leveling is worth
        // 2.0² · 0.02² = 0.0016 on every heading here. The adoption left yaw at
        // 0.05² + 0.0016 = 0.0041, the second field arrives with σ = 0.02, so
        // `R` = 0.02² + 0.0016 = 0.0020 and `K = 0.0041 / (0.0041 + 0.0020)` = 0.672:
        // a 0.05 rad disagreement moves 0.0336.
        //
        // The two σ differ on purpose. Equal ones make `P` and `R` equal, `K` exactly a
        // half, and the test blind to (36′) being dropped from either side — which is
        // what it is here to notice. Unpriced in `R` this moves 0.0456, unpriced in both
        // 0.0431. Both headings arrive with no step between them, which (24′) takes for the
        // same error twice, so the sources are white here.
        let mut filter = initialized();
        filter.config.correlation = Correlation::WHITE;
        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );

        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 0.05), 0.0),
                    HeadingNoise::from_sigma(0.02),
                )
                .is_accepted()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!(
            (yaw - 0.0336).abs() < 1e-3,
            "expected 0.0336 by the arithmetic above, got {yaw}"
        );
        assert!(
            filter.covariance().variance(ErrorState::AttitudeZ) < 0.0041,
            "a fused heading leaves yaw better known than the adoption did"
        );
    }

    #[test]
    fn a_magnetometer_in_the_window_establishes_heading_at_once() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        assert!(filter.validity().heading);
    }

    #[test]
    fn a_moving_start_takes_its_heading_from_the_first_magnetometer() {
        // This window's halves level 0.4 rad apart, and the dip scales that into 0.78 rad
        // of yaw — half as much again as `Accuracy::heading`, so the start is worth no
        // heading at all. The first field is adopted rather than fused: a prior nothing
        // measured is replaced, not averaged with.
        let mut filter = Eskf::default();
        let alignment = filter
            .initialize_over(&turning(0.0, 0.4, 0.0), Seconds::from_secs(0.25))
            .expect("moving, not unusable");
        assert!(matches!(alignment, Alignment::Coarse(..)));
        assert!(!filter.validity().heading, "the window was moving");

        let field = MagField::body(0.22, 0.0, 0.44);
        let noise = HeadingNoise::from_sigma(0.1);
        assert!(
            filter
                .fuse_mag_heading(filter.now(), field, noise)
                .is_reset()
        );

        // (36′) on the adoption path, which is where the term is worth the most: (34)
        // levels with a tilt whose variance is 0.64 here, and the dip carries that into
        // the heading. 0.582 rather than the measurement's own 0.01 — σ = 0.76 rad,
        // outside `Accuracy::heading`. Storing the magnetometer's number alone is what
        // makes the filter claim an attitude it does not have, on the one path where the
        // tilt doing the leveling is worst.
        let yaw_variance = filter.covariance().variance(ErrorState::AttitudeZ);
        assert!(
            (yaw_variance - 0.582).abs() < 1e-2,
            "expected the leveling priced in, got {yaw_variance}"
        );
        assert!(
            !filter.validity().heading,
            "a heading leveled by this tilt is not good to Accuracy::heading"
        );
        assert!(!filter.validity().tilt, "the tilt is still the window's");

        // Established all the same, which is the other half of the claim: the quantity
        // has been observed, so the next field is fused rather than adopted. Established
        // and good are two questions and only the covariance answers the second.
        assert!(matches!(
            filter.fuse_mag_heading(filter.now(), field, noise),
            Fusion::Accepted { .. }
        ));
    }

    #[test]
    fn on_its_tail_a_heading_adoption_resets_heading_and_leaves_tilt() {
        let mut filter = on_its_tail();
        let before = filter.attitude_variance();
        // The same vehicle turned 1.1 rad about down: a heading, and nothing else, differs.
        let truth = Attitude::from_quaternion(
            nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.1)
                * filter.state().attitude.quaternion(),
        );
        let field = measured(truth, 0.0);
        let noise = HeadingNoise::from_sigma(0.05);
        let adopted = mag::heading_observation(
            filter.estimate.state(),
            &filter.covariance,
            field,
            filter.declination,
            noise,
        )
        .r_m[0];

        assert!(
            filter
                .fuse_mag_heading(filter.now(), field, noise)
                .is_reset()
        );

        let after = filter.attitude_variance();
        assert!(
            (after.heading - adopted).abs() < 1e-6 * adopted,
            "heading carries the adopted variance: {after:?} against {adopted}"
        );
        assert!(
            (after.tilt_north - before.tilt_north).abs() < 1e-9,
            "{after:?}"
        );
        assert!(
            (after.tilt_east - before.tilt_east).abs() < 1e-9,
            "{after:?}"
        );
    }

    /// An established yaw of zero, and a magnetometer turned a quarter circle from it.
    fn disturbed_heading() -> (Eskf, MagField<Body>) {
        let mut filter = initialized();
        assert!(
            filter
                .fuse_mag_heading(
                    filter.now(),
                    measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                    HeadingNoise::from_sigma(0.05),
                )
                .is_reset()
        );
        let turned = measured(attitude_of(0.0, 0.0, core::f32::consts::FRAC_PI_2), 0.0);
        (filter, turned)
    }

    #[test]
    fn a_magnetometer_disagreeing_while_gnss_arrives_is_not_adopted() {
        // The guard PX4's `mag_control.cpp` keeps: with horizontal aiding arriving, a heading
        // that disagrees for this long is a disturbance, and adopting it would turn the
        // estimate by it.
        let (mut filter, turned) = disturbed_heading();
        let mut step = 0;
        hold(&mut filter, 20.0, 10, |filter| {
            step += 1;
            if step % 10 == 0 {
                let _ = filter.fuse_gnss_position(
                    filter.now(),
                    Position::zero(),
                    one_metre(),
                    Position::zero(),
                );
            }
            let outcome =
                filter.fuse_mag_heading(filter.now(), turned, HeadingNoise::from_sigma(0.05));
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.diagnostics().mag_heading.recovered, 0);
    }

    #[test]
    fn a_magnetometer_disagreeing_with_a_gnss_heading_is_not_adopted() {
        // The receiver measures heading outright, so the magnetometer is the one disturbed.
        let (mut filter, turned) = disturbed_heading();
        let north = Radians::from_radians(0.0);
        hold(&mut filter, 20.0, 10, |filter| {
            let _ = filter.fuse_gnss_heading(filter.now(), north, HeadingNoise::from_sigma(0.05));
            let outcome =
                filter.fuse_mag_heading(filter.now(), turned, HeadingNoise::from_sigma(0.05));
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.diagnostics().mag_heading.recovered, 0);
    }

    #[test]
    fn a_magnetometer_disagreeing_with_nothing_else_to_say_is_adopted() {
        let (mut filter, turned) = disturbed_heading();
        let mut recovered = false;
        hold(&mut filter, 8.0, 10, |filter| {
            if !recovered {
                recovered = filter
                    .fuse_mag_heading(filter.now(), turned, HeadingNoise::from_sigma(0.05))
                    .is_reset();
            }
        });
        assert!(recovered);
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!(
            (yaw - core::f32::consts::FRAC_PI_2).abs() < 1e-3,
            "yaw = {yaw}"
        );
        assert_eq!(filter.diagnostics().mag_heading.recovered, 1);
    }

    #[test]
    fn the_first_gnss_heading_is_adopted_and_every_one_after_it_is_fused() {
        let mut filter = initialized();
        let noise = HeadingNoise::from_sigma(0.05);
        let heading = Radians::from_radians(1.1);
        assert!(
            filter
                .fuse_gnss_heading(filter.now(), heading, noise)
                .is_reset()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw - 1.1).abs() < 1e-5, "yaw = {yaw}");
        assert!(filter.validity().heading);
        assert!(filter.is_aligned(), "a heading source is a heading source");
        assert_eq!(filter.diagnostics().gnss_heading.adopted, 1);

        let second = filter.fuse_gnss_heading(filter.now(), heading, noise);
        assert!(matches!(second, Fusion::Accepted { .. }), "{second:?}");
    }

    #[test]
    fn a_gnss_heading_is_screened_before_anything_else() {
        let mut filter = initialized();
        let at = |r| Radians::from_radians(r);
        assert_eq!(
            filter.fuse_gnss_heading(filter.now(), at(f32::NAN), HeadingNoise::from_sigma(0.1)),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_gnss_heading(filter.now(), at(0.3), HeadingNoise::from_variance(0.0)),
            Fusion::InvalidNoise
        );
        assert_eq!(filter.diagnostics().gnss_heading.adopted, 0);
        assert!(!filter.validity().heading, "a refusal establishes nothing");
    }

    #[test]
    fn a_gnss_heading_is_true_heading_whatever_the_declination() {
        let mut filter = initialized();
        assert!(filter.set_magnetic_declination(Radians::from_radians(0.3)));
        let _ = filter.fuse_gnss_heading(
            filter.now(),
            Radians::from_radians(1.1),
            HeadingNoise::from_sigma(0.05),
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!((yaw - 1.1).abs() < 1e-5, "yaw = {yaw}");
    }

    #[test]
    fn a_course_is_refused_without_speed_and_adopted_with_it() {
        let slow = Velocity::ned(0.5, 0.5, 0.0);
        let mut filter = cruising(slow);
        assert_eq!(
            filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.05)),
            Fusion::Unobservable
        );
        assert_eq!(
            filter.diagnostics().course.last_refusal,
            Some(Refusal::Unobservable)
        );

        // North-east at 14 m/s: the course is π/4.
        let mut filter = cruising(Velocity::ned(10.0, 10.0, 0.0));
        assert!(
            filter
                .fuse_course(filter.now(), HeadingNoise::from_sigma(0.05))
                .is_reset()
        );
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        assert!(
            (yaw - core::f32::consts::FRAC_PI_4).abs() < 1e-5,
            "yaw = {yaw}"
        );
        assert!(filter.is_aligned());
        let next = filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.05));
        assert!(matches!(next, Fusion::Accepted { .. }), "{next:?}");
    }

    #[test]
    fn an_adopted_course_carries_the_velocity_s_uncertainty_as_well_as_the_sideslip() {
        // About 0.2 m/s across 14.1 m/s of track, once the velocity fix has been fused, on top
        // of 0.05 rad of sideslip.
        let mut filter = cruising(Velocity::ned(10.0, 10.0, 0.0));
        let course =
            heading::course_variance(filter.estimate.state(), &filter.covariance).expect("moving");
        assert!(course > 1e-5 && course < 0.09 / 200.0, "{course}");
        let sideslip = HeadingNoise::from_sigma(0.05);
        assert!(filter.fuse_course(filter.now(), sideslip).is_reset());
        let heading = AttitudeVariance::of(&filter.state().attitude, filter.covariance()).heading;
        let expected = sideslip.variance() + course;
        assert!(
            (heading - expected).abs() < 1e-6,
            "heading variance {heading}, expected {expected}"
        );
    }

    #[test]
    fn a_coarse_start_has_no_course_until_velocity_is_established() {
        let mut filter = coarse();
        assert_eq!(
            filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.05)),
            Fusion::NoReference
        );
    }

    #[test]
    fn the_course_corrects_heading_and_velocity_together() {
        // Heading established at 0, then a course constraint along a velocity 0.2 rad east of
        // it: the update turns the nose toward the track and the track toward the nose, each
        // by its share of the uncertainty.
        let mut filter = cruising(Velocity::ned(20.0 * 0.2f32.cos(), 20.0 * 0.2f32.sin(), 0.0));
        let _ = filter.fuse_gnss_heading(
            filter.now(),
            Radians::from_radians(0.0),
            HeadingNoise::from_sigma(0.1),
        );
        let outcome = filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.1));
        assert!(matches!(outcome, Fusion::Accepted { .. }), "{outcome:?}");
        let state = filter.state();
        let (_, _, yaw) = state.attitude.euler_angles();
        let v = state.velocity.vector();
        let track = v.y.atan2(v.x);
        assert!(
            yaw > 0.0 && yaw < 0.2,
            "the nose turned toward the track: {yaw}"
        );
        assert!(
            track < 0.2 && track > yaw,
            "the track turned toward the nose: {track}"
        );
    }

    #[test]
    fn a_course_disagreeing_while_another_heading_arrives_is_not_adopted() {
        // Nose north on a GNSS heading, tracking east: a 90° crab no sideslip allowed for.
        let mut filter = cruising(Velocity::ned(0.0, 15.0, 0.0));
        let north = Radians::from_radians(0.0);
        let noise = HeadingNoise::from_sigma(0.02);
        assert!(
            filter
                .fuse_gnss_heading(filter.now(), north, noise)
                .is_reset()
        );
        // Velocity aided too, or its uncertainty outgrows the speed and the course is refused.
        let east = Velocity::ned(0.0, 15.0, 0.0);
        hold(&mut filter, 10.0, 10, |filter| {
            let _ = filter.fuse_gnss_heading(filter.now(), north, noise);
            let _ = filter.fuse_gnss_velocity(
                filter.now(),
                east,
                VelocityNoise::from_speed_accuracy(0.3),
                Position::zero(),
            );
            let outcome = filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.02));
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.diagnostics().course.recovered, 0);
    }

    #[test]
    fn a_course_disagreeing_with_nothing_else_to_say_is_adopted() {
        let mut filter = cruising(Velocity::ned(0.0, 15.0, 0.0));
        let noise = HeadingNoise::from_sigma(0.02);
        assert!(
            filter
                .fuse_gnss_heading(filter.now(), Radians::from_radians(0.0), noise)
                .is_reset()
        );
        let mut recovered = false;
        hold(&mut filter, 8.0, 10, |filter| {
            hold_velocity(filter, Velocity::ned(0.0, 15.0, 0.0));
            if !recovered {
                recovered = filter.fuse_course(filter.now(), noise).is_reset();
            }
        });
        assert!(recovered);
        assert_eq!(filter.diagnostics().course.recovered, 1);
    }

    #[test]
    fn a_course_along_a_dead_reckoned_velocity_is_refused() {
        // Moving fast enough, but a 10 Hz velocity has not held it for five of its periods,
        // twice its own timeout: a course along it would read as aiding while it drifts.
        let velocity = Velocity::ned(0.0, 15.0, 0.0);
        let mut filter = cruising(velocity);
        hold(&mut filter, 2.0, 10, |filter| {
            hold_velocity(filter, velocity)
        });
        hold(&mut filter, 0.5, 100, |_| {});
        assert_eq!(
            filter.fuse_course(filter.now(), HeadingNoise::from_sigma(0.05)),
            Fusion::NoReference
        );
    }

    #[test]
    fn a_course_adopted_in_the_past_turns_about_down_and_leaves_tilt_alone() {
        // (23′) carries the course's row back through the error dynamics, and its velocity block
        // lends the attitude row a tilt component; the adoption must still turn about down.
        let mut filter = cruising(Velocity::ned(10.0, 10.0, 0.0));
        hold(&mut filter, 0.2, 10, |filter| {
            hold_velocity(filter, Velocity::ned(10.0, 10.0, 0.0))
        });
        let tilt = |filter: &Eskf| {
            let v = AttitudeVariance::of(&filter.state().attitude, filter.covariance());
            v.tilt_north + v.tilt_east
        };
        let before = tilt(&filter);
        let taken = filter.now().before(Seconds::from_secs(0.11));
        assert!(
            filter
                .fuse_course(taken, HeadingNoise::from_sigma(0.05))
                .is_reset()
        );
        let after = tilt(&filter);
        assert!(
            (after - before).abs() < 1e-3 * before,
            "tilt variance {before} -> {after}"
        );
    }

    #[test]
    fn a_forward_axis_near_vertical_refuses_both_gnss_heading_and_course() {
        // A tailsitter hovering: body x 25° from the sky, moving at 15 m/s with its velocity
        // held, so nothing but the geometry refuses the course.
        let mut filter = Eskf::default();
        let state = State {
            attitude: attitude_of(0.0, 1.134, 0.0),
            velocity: Velocity::ned(15.0, 0.0, 0.0),
            ..State::default()
        };
        let _ = filter
            .seed(state, Covariance::from_sigmas([0.5; STATES]))
            .expect("a sane seed");
        hold_velocity(&mut filter, Velocity::ned(15.0, 0.0, 0.0));
        let noise = HeadingNoise::from_sigma(0.05);
        assert_eq!(
            filter.fuse_gnss_heading(filter.now(), Radians::from_radians(0.3), noise),
            Fusion::Unobservable
        );
        assert_eq!(
            filter.fuse_course(filter.now(), noise),
            Fusion::Unobservable
        );
    }

    #[test]
    fn a_course_disagreeing_with_a_magnetometer_is_not_adopted() {
        // Nose north on the magnetometer, tracking east: a crab no sideslip allowed for.
        let east = Velocity::ned(0.0, 15.0, 0.0);
        let mut filter = cruising(east);
        let north = measured(attitude_of(0.0, 0.0, 0.0), 0.0);
        let noise = HeadingNoise::from_sigma(0.02);
        assert!(
            filter
                .fuse_mag_heading(filter.now(), north, noise)
                .is_reset()
        );
        hold(&mut filter, 10.0, 10, |filter| {
            let _ = filter.fuse_mag_heading(filter.now(), north, noise);
            hold_velocity(filter, east);
            let outcome = filter.fuse_course(filter.now(), noise);
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        });
        assert_eq!(filter.diagnostics().course.recovered, 0);
    }
}
