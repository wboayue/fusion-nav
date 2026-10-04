//! The update every source shares, (23)–(27) at the measurement's own time, (23′), and at its
//! equivalent white noise, (24′), then the sources that use it directly: GNSS position, (28) and
//! (28′), converted from geodetic by (43) about the origin `site.rs` places by (44), GNSS
//! velocity, (29) and (29′), and barometric altitude, (30) and (30′).
//!
//! Entry points: [`Eskf::fuse_gnss_position`], [`Eskf::fuse_gnss_geodetic`],
//! [`Eskf::fuse_gnss_velocity`] and [`Eskf::fuse_baro_altitude`]. The shared path comes first,
//! beside its first caller; `heading.rs` holds the sources that correct heading.

use crate::config::LATENCY_HORIZON;
use crate::frames::{Body, Ned};
use crate::geodetic::{Geodetic, LocalOrigin};
use crate::health::{Diagnostics, Fusion, GnssFusion, SourceHealth};
use crate::math::correlation_inflation;
use crate::observation::{baro, gnss};
use crate::propagate;
use crate::state::State;
use crate::units::{
    Altitude, AltitudeNoise, AngularRate, Position, PositionNoise, Seconds, Timestamp, Velocity,
    VelocityNoise,
};
use crate::update::{Observation, Update, update};

use super::Eskf;
use super::adopt::{HORIZONTAL, POSITION};
use super::hold::recovery_after;

impl Eskf {
    /// Commit what an update produced and record it against its source, handing the outcome
    /// back. Equations (23)–(41) are `update`'s; this is only the bookkeeping.
    ///
    /// One place for the reason [`refuse`] is one place: every `fuse_*` that runs an update
    /// ends here, so none can commit a state without the covariance that goes with it, record
    /// an acceptance without restarting the timer, or forget [`note_alignment`] — which an
    /// update owes as much as a reset does, since a position fix narrows the attitude block
    /// through the correlations (17) builds.
    ///
    /// [`note_alignment`]: Self::note_alignment
    fn apply(
        &mut self,
        outcome: Update,
        source: fn(&mut Diagnostics) -> &mut SourceHealth,
    ) -> Fusion {
        match outcome {
            Update::Accepted {
                state,
                covariance,
                offset,
                offset_correction,
                ratio,
                innovation,
            } => {
                self.estimate.commit(state);
                self.commit_covariance(covariance, offset);
                // (30′): `b` is the error in `α₀`, so its estimate comes off it.
                if let Some(reference) = self.baro_reference {
                    self.baro_reference = Some(Altitude::from_meters(
                        reference.as_meters() - offset_correction,
                    ));
                }
                source(&mut self.diagnostics).record_accepted(ratio, Some(innovation));
                self.note_alignment();
                Fusion::Accepted { test_ratio: ratio }
            }
            Update::Rejected { ratio, innovation } => {
                source(&mut self.diagnostics).record_rejected(ratio, innovation);
                Fusion::Rejected { test_ratio: ratio }
            }
            Update::Invalid => refuse(source(&mut self.diagnostics), Fusion::StateInvalid),
        }
    }

    /// [`apply`](Self::apply), unless the gate rejected a source locked out for `after`, in
    /// which case `adopt` takes the measurement instead: recovery from gate lockout, see
    /// [`Recovery`](crate::Recovery).
    ///
    /// One place, for the reason `apply` is one place: every source recovers through here, so
    /// none can adopt without counting it or forget [`note_alignment`](Self::note_alignment).
    ///
    /// `adopt` returns whether it adopted. One that would write a value that is not finite
    /// commits nothing and the measurement is refused as [`Fusion::NotFinite`]: a finite fix
    /// carried to now by (28′) can still overflow, a lever arm of `f32::MAX` rotated.
    ///
    /// Out of line for the reason [`observe`](Self::observe) is: inlined, the `Update` it takes
    /// sits in each `fuse_*` frame above the update, and a geodetic fix's path then becomes the
    /// crate's high-water mark ([measured]).
    ///
    /// [measured]:
    /// https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#measured-cost-by-function
    #[inline(never)]
    pub(super) fn apply_or_recover(
        &mut self,
        outcome: Update,
        source: fn(&mut Diagnostics) -> &mut SourceHealth,
        after: Option<Seconds>,
        adopt: impl FnOnce(&mut Self) -> bool,
    ) -> Fusion {
        let since_initialized = self.diagnostics.since_initialized;
        let locked_out = source(&mut self.diagnostics).locked_out(after, since_initialized);
        if !(matches!(outcome, Update::Rejected { .. }) && locked_out) {
            return self.apply(outcome, source);
        }
        if !adopt(self) {
            return refuse(source(&mut self.diagnostics), Fusion::NotFinite);
        }
        source(&mut self.diagnostics).record_recovered();
        self.note_alignment();
        Fusion::Reset
    }

    /// Fuse a position fix already expressed in NED meters about the filter's origin.
    /// Equation (28).
    ///
    /// For a receiver reporting latitude and longitude, use
    /// [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic), which converts about the origin
    /// the filter holds. This one is for a caller that owns the conversion, or whose
    /// positions were never geodetic — a local RTK base, motion capture. Mixing the two is
    /// only right if the caller's origin is [`origin`](Self::origin).
    ///
    /// After a coarse start the first fix is adopted rather than fused; see
    /// [`Fusion::Reset`].
    ///
    /// `noise` is the receiver's own accuracy where it reports one, bounded before it
    /// arrives: [`PositionNoise::clamped`](crate::PositionNoise::clamped) takes `eph` and
    /// `epv` and holds each within its own floor and cap, for the reasons recorded there. A
    /// two-dimensional fix instead goes through
    /// [`PositionNoise::horizontal_vertical`](crate::PositionNoise::horizontal_vertical),
    /// which leaves the vertical σ where the caller put it: `clamped` caps both axes, so
    /// it would turn a declined height back into a measurement.
    ///
    /// The filter applies no bound of its own, because `R` describes the measurement and
    /// belongs with it rather than in [`Config`](crate::Config). A caller handing over a raw `eph`
    /// is therefore trusting the receiver further than either production autopilot does. Nor does
    /// it smooth one: ArduPilot runs each accuracy through a decaying envelope with a 5 s time
    /// constant before bounding it (`AP_NavEKF3_Measurements.cpp:609-633` at `368dc0c4`), so a
    /// spike in `eph` deweights the fixes after it for seconds there and only its own fix here. The
    /// replay harness takes the same stance, for the reasons `data/README.md` gives under
    /// `r_policy=`. What the filter does add is the receiver's rather than the fix's: a fix's error
    /// persists into the next one, and the update is computed at the variance that leaves, equation
    /// (24′), with the gate still reading `noise` itself. See
    /// [`Config::correlation`](crate::Config::correlation).
    ///
    /// The interval (24′) reads is the time since the previous fix fused through this method,
    /// so it assumes one receiver. Fixes from a second one, or from motion capture,
    /// interleaved with the first read as the same error arriving sooner and are deweighted
    /// though their errors are independent.
    ///
    /// The fix is two measurements, gated and reported apart: north and east at
    /// [`Gates::gnss_position`](crate::Gates), then down at
    /// [`Gates::gnss_height`](crate::Gates), against the state the first left. A half
    /// inconsistent with the estimate is [`Fusion::Rejected`] and changes nothing but its
    /// own health, so a height the estimate disagrees with costs no horizontal aiding; see
    /// [`GnssFusion`] for the measurement behind that. A half whose numbers are unusable is
    /// refused alone for the same reason — a 2D fix reporting `epv = 0` still fuses its
    /// horizontal position.
    ///
    /// The adoption after a coarse start is the exception, and takes the fix whole: it writes
    /// all three axes onto the covariance, so any unusable number refuses both halves.
    ///
    /// `antenna` is where the antenna sits relative to the IMU in body axes, forward, right,
    /// down ([`Position::body`](crate::Position)), and [`Position::zero`](crate::Position) for
    /// one on top of it. The fix is the antenna's, `p + R r` of (28′), and the estimate stays
    /// the IMU's. Unlike PX4, which subtracts `R̂ r` from the fix and keeps `H` as it was
    /// (`EKF/aid_sources/gnss/gps_control.cpp:351-354` at `c4e4ef98`), the update carries the
    /// arm's dependence on attitude, `−R̂[r]×`, so a fix observes heading through a long mast;
    /// see `observation/gnss.rs` for what that was measured against. An argument, as `noise`
    /// is and as PX4 carries it on each GNSS message (`antenna_offset_x/y/z`), because a
    /// second receiver has its own; from PX4's parameters it is `SENS_GPS0_OFF*` less
    /// `EKF2_IMU_POS*`.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_gnss_position(
        &mut self,
        time: Timestamp,
        position: Position<Ned>,
        noise: PositionNoise<Ned>,
        antenna: Position<Body>,
    ) -> GnssFusion {
        if let Err(refusal) = self.admit(time) {
            return self.refuse_gnss(refusal);
        }
        self.diagnostics.gnss_position.note_arrival(time);
        self.diagnostics.gnss_height.note_arrival(time);
        // An arm that is not a number spoils both halves, and an adoption would write it into
        // the state.
        if !antenna.is_finite() {
            return self.refuse_gnss(Fusion::NotFinite);
        }
        if self.unestablished.position {
            if !position.is_finite() || !noise.is_finite() {
                return self.refuse_gnss(Fusion::NotFinite);
            }
            if !noise.is_positive() {
                return self.refuse_gnss(Fusion::InvalidNoise);
            }
            let Some(adopted) = self.carried_position(position, antenna, time) else {
                return self.refuse_gnss(Fusion::NotFinite);
            };
            self.adopt_position(adopted, noise, POSITION);
            self.unestablished.position = false;
            self.diagnostics.gnss_position.record_adopted();
            self.diagnostics.gnss_height.record_adopted();
            return GnssFusion::both(Fusion::Reset);
        }

        let (z, r) = (position.vector(), noise.variance());
        let horizontal = match screen(&[z[0], z[1]], &[r[0], r[1]]) {
            Some(refusal) => refuse(&mut self.diagnostics.gnss_position, refusal),
            None => {
                let observation = self
                    .observe(time, |past, _| {
                        gnss::horizontal_observation(past, position, noise, antenna)
                    })
                    .correlated(correlation_inflation(
                        self.diagnostics.gnss_position.since_measured,
                        self.config.correlation.gnss_position,
                    ));
                let outcome = update(
                    self.estimate.state(),
                    &self.covariance,
                    &self.offset,
                    &observation,
                    self.config.gates.gnss_position,
                );
                self.apply_or_recover(
                    outcome,
                    |diagnostics| &mut diagnostics.gnss_position,
                    recovery_after(
                        self.hold.holds_position() || self.yaw_replaced.owes_position(),
                        self.config.recovery.gnss_position,
                    ),
                    |filter| {
                        let adopted = filter.carried_position(position, antenna, time);
                        adopted
                            .map(|adopted| filter.adopt_position(adopted, noise, HORIZONTAL))
                            .is_some()
                    },
                )
            }
        };
        // An adoption ended the hold's claim on position on its way. A fix the gate passes
        // measures position too, and one it passes against a position already measured since
        // the hold has checked the velocity that carried the estimate there.
        if matches!(horizontal, Fusion::Accepted { .. }) {
            self.yaw_replaced.settle_position();
        }
        if matches!(horizontal, Fusion::Accepted { .. }) {
            if !self.hold.holds_position() {
                self.hold.end_velocity();
            }
            self.hold.end_position();
        }
        let height = match screen(&[z[2]], &[r[2]]) {
            Some(refusal) => refuse(&mut self.diagnostics.gnss_height, refusal),
            None => {
                let observation = self
                    .observe(time, |past, _| {
                        gnss::height_observation(past, position, noise, antenna)
                    })
                    .correlated(correlation_inflation(
                        self.diagnostics.gnss_height.since_measured,
                        self.config.correlation.gnss_height,
                    ));
                let outcome = update(
                    self.estimate.state(),
                    &self.covariance,
                    &self.offset,
                    &observation,
                    self.config.gates.gnss_height,
                );
                self.apply_or_recover(
                    outcome,
                    |diagnostics| &mut diagnostics.gnss_height,
                    self.config.recovery.gnss_height,
                    |filter| {
                        let adopted = filter.carried_position(position, antenna, time);
                        adopted
                            .map(|adopted| filter.adopt_height(adopted, noise))
                            .is_some()
                    },
                )
            }
        };
        GnssFusion { horizontal, height }
    }

    /// Whether a measurement taken at `time` can be fused, or the refusal that says why not:
    /// [`Fusion::NotInitialized`], or [`Fusion::OutOfHorizon`] for one older than
    /// [`LATENCY_HORIZON`], later than the state by more than
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt), or taken before a start
    /// that did not show the vehicle at rest.
    ///
    /// The history begins at the start, so a measurement from before it would be placed at
    /// the start's state. After a still window that is where the vehicle was; after a start in
    /// motion, a seed or a single sample, it is `v τ` from where the vehicle was, 1.6 m for a
    /// fix 110 ms old at 15 m/s, and the next measurement is a better use of the source.
    ///
    /// The filter's time is its last IMU sample's, so a measurement timed between that sample
    /// and the next arrives ahead of it, which is the ordinary case for a caller fusing a sensor
    /// the moment it reads; [`past`](Self::past) carries the state forward to it. The bound is
    /// the one step (22′) would coast rather than integrate, the longest the filter extrapolates
    /// on one sample.
    pub(super) fn admit(&self, time: Timestamp) -> Result<(), Fusion> {
        if !self.initialized {
            return Err(Fusion::NotInitialized);
        }
        let age = self.time.since(time);
        if age > LATENCY_HORIZON
            || -age.as_secs() > self.config.max_predict_dt.as_secs()
            || time < self.earliest
        {
            return Err(Fusion::OutOfHorizon { age });
        }
        Ok(())
    }

    /// The state at `time`, from the [`History`](crate::history::History), and how long before the
    /// present it was placed: no further back than the history reaches, so that a measurement older
    /// than the initialization is placed at the start rather than described by one time and
    /// linearized at another.
    pub(super) fn past(&self, time: Timestamp) -> (State, Seconds) {
        let omega = self
            .angular_rate
            .unwrap_or_else(|| AngularRate::body(0.0, 0.0, 0.0));
        let (past, placed) = self.estimate.at(time, self.time, omega);
        (past, self.time.since(placed))
    }

    /// Form an observation of the state at `time`, expressed in today's error.
    /// Equation (23′).
    ///
    /// `build` is the observation module's own function, handed the past state instead of the
    /// present: the innovation of (23) is then the measurement against where the vehicle was
    /// when it was taken, and a fix 150 ms old on a vehicle at 20 m/s is not 3 m of error
    /// handed to the gate as truth. The past is read from the [`History`](crate::history::History)
    /// rather than extrapolated from the present, and its `H` is carried to today's error through
    /// (16)–(19) at the mean rates over the age, which the same history gives.
    ///
    /// Out of line, so that it sits beside `update` rather than beneath it: inlined into
    /// `fuse_gnss_velocity`, it raises the crate's high-water mark ([measured]).
    ///
    /// [measured]:
    /// https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#measured-cost-by-function
    #[inline(never)]
    pub(super) fn observe<const M: usize>(
        &self,
        time: Timestamp,
        build: impl FnOnce(&State, AngularRate<Body>) -> Observation<M>,
    ) -> Observation<M> {
        if time == self.time {
            return build(
                self.estimate.state(),
                self.mean_rate(self.estimate.state(), 0.0),
            );
        }
        let (past, age) = self.past(time);
        let tau = age.as_secs();
        let omega = self.mean_rate(&past, tau);
        if tau == 0.0 {
            return build(&past, omega);
        }
        let now = self.estimate.state().attitude.quaternion();
        // Mean rates over the age, rather than the last sample's: one sample's specific force
        // carries the airframe's vibration, which the velocities either side of it average out.
        let a_n = (self.estimate.state().velocity.vector() - past.velocity.vector()) / tau;
        let a_b = now.inverse() * (a_n - propagate::gravity_vector(self.config.gravity));
        let a = propagate::error_dynamics(self.estimate.state(), omega.vector(), a_b);
        build(&past, omega).delayed(age, &a)
    }

    /// The body rate between `past`, `tau` before now, and now: the rotation the attitude
    /// made, over the time it took. Bias-corrected by construction, since it is the estimate's
    /// own turn. With no interval, the last sample's `ω` of (9), or none after a coast.
    ///
    /// What (23′) carries `H` with, and the rate a GNSS velocity's antenna turned at, (29′):
    /// a solution 110 ms old is the antenna's motion then, not now.
    fn mean_rate(&self, past: &State, tau: f32) -> AngularRate<Body> {
        if tau == 0.0 {
            return self.angular_rate.unwrap_or_default();
        }
        let (now, then) = (
            self.estimate.state().attitude.quaternion(),
            past.attitude.quaternion(),
        );
        AngularRate::from_vector((then.inverse() * now).scaled_axis() / tau)
    }

    /// A fix of the antenna taken at `time`, as the IMU's position now: referred to the IMU
    /// by the attitude then, `z − R̂(t − τ) r` of (28′), and carried to now by the state's own
    /// motion since, `+ x̂ − x̂(t − τ)`. For an adoption, which writes the measurement as the
    /// state. A fix 110 ms old on a vehicle at 30 m/s, adopted as it stands, puts the estimate
    /// 3.3 m behind.
    ///
    /// `None` where the sum overflows, which finite inputs can: an adoption would write it
    /// into the state.
    fn carried_position(
        &self,
        taken: Position<Ned>,
        antenna: Position<Body>,
        time: Timestamp,
    ) -> Option<Position<Ned>> {
        let (past, _) = self.past(time);
        let arm = past.attitude.quaternion() * antenna.vector();
        let moved = self.estimate.state().position.vector() - past.position.vector();
        let carried = Position::from_vector(taken.vector() - arm + moved);
        carried.is_finite().then_some(carried)
    }

    /// [`carried_position`](Self::carried_position) for a velocity, referred to the IMU by
    /// (29′) at the mean rate over the measurement's age, or the last sample's for one taken
    /// now.
    pub(super) fn carried_velocity(
        &self,
        taken: Velocity<Ned>,
        antenna: Position<Body>,
        time: Timestamp,
    ) -> Option<Velocity<Ned>> {
        let (past, age) = self.past(time);
        let omega = self.mean_rate(&past, age.as_secs());
        let turning = past.attitude.quaternion() * omega.vector().cross(&antenna.vector());
        let moved = self.estimate.state().velocity.vector() - past.velocity.vector();
        let carried = Velocity::from_vector(taken.vector() - turning + moved);
        carried.is_finite().then_some(carried)
    }

    /// Refuse both halves of a GNSS fix for one reason.
    fn refuse_gnss(&mut self, outcome: Fusion) -> GnssFusion {
        refuse(&mut self.diagnostics.gnss_position, outcome);
        GnssFusion::both(refuse(&mut self.diagnostics.gnss_height, outcome))
    }

    /// Fuse a GNSS fix given as latitude, longitude and height. Equations (43), (44),
    /// then (28).
    ///
    /// The filter holds the navigation origin and converts about it, so the fix and the
    /// estimate are relative to the same point by construction. The first fix places the
    /// origin, unless [`set_origin`](Self::set_origin) already has:
    ///
    /// * With a position estimate — a static start, a seed — the origin goes where that
    ///   estimate says the vehicle started, so the fix lands on the estimate and nothing
    ///   steps. Equation (44). The fix is not fused: it was spent placing the origin, and
    ///   fusing it as well would count it twice. What it does settle is the position
    ///   uncertainty. About the new origin the position error *is* the fix's error, so
    ///   the position covariance block becomes `noise` and its correlations are dropped,
    ///   as [`reset_position_to`](Self::reset_position_to) does, with the value unchanged.
    ///   Reported as accepted with a zero test ratio: nothing was inconsistent.
    /// * Without one — after a coarse start — the origin goes at the fix, and the fix is
    ///   adopted as position zero; see [`Fusion::Reset`].
    ///
    /// Check the receiver's fix type before calling. Many report latitude and longitude
    /// zero until they have a fix, and a finite zero is a usable origin: the first one
    /// would put the navigation frame in the Gulf of Guinea for the rest of the flight.
    ///
    /// `noise` and `antenna` are as for [`fuse_gnss_position`](Self::fuse_gnss_position), floor
    /// included, and so is the split into two gated halves. The origin (44) places goes under
    /// the estimate of the antenna, since the fix is the antenna's.
    ///
    /// A fix that is not a number is refused with [`Fusion::NotFinite`], both halves, since
    /// no conversion survives one; so is a noise that is not, where the fix would place the
    /// origin, which writes all three axes. A fix
    /// with a latitude beyond ±90° cannot place an origin, nor can one near a pole that no
    /// origin puts at the estimate (see [`LocalOrigin::placing`]): with none held it is
    /// refused with [`Fusion::NoReference`], and the next usable fix places it instead.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_gnss_geodetic(
        &mut self,
        time: Timestamp,
        fix: Geodetic,
        noise: PositionNoise<Ned>,
        antenna: Position<Body>,
    ) -> GnssFusion {
        if let Err(refusal) = self.admit(time) {
            return self.refuse_gnss(refusal);
        }
        // Noted again by `fuse_gnss_position` when it delegates, which the same time ignores.
        self.diagnostics.gnss_position.note_arrival(time);
        self.diagnostics.gnss_height.note_arrival(time);
        if !fix.is_finite() || !antenna.is_finite() {
            return self.refuse_gnss(Fusion::NotFinite);
        }
        if let Some(origin) = self.origin {
            return self.fuse_gnss_position(time, origin.to_ned(fix), noise, antenna);
        }
        if !noise.is_finite() {
            return self.refuse_gnss(Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return self.refuse_gnss(Fusion::InvalidNoise);
        }

        if self.unestablished.position {
            let Some(origin) = LocalOrigin::new(fix) else {
                return self.refuse_gnss(Fusion::NoReference);
            };
            // The adoption below refuses an arm that overflows, and placing the origin turns
            // the heading, so ask first. The turn is about down and keeps the arm's length.
            if self
                .carried_position(Position::zero(), antenna, time)
                .is_none()
            {
                return self.refuse_gnss(Fusion::NotFinite);
            }
            self.place_origin(origin);
            return self.fuse_gnss_position(time, Position::zero(), noise, antenna);
        }

        // Equation (44): the origin under the estimate when the fix was taken, and the fix's
        // error as the position's.
        if !self.place_origin_under_estimate(fix, antenna, time) {
            return self.refuse_gnss(Fusion::NoReference);
        }
        let placed = self.reset_position_to(self.estimate.state().position, noise);
        debug_assert!(
            placed,
            "the estimate and the fix's noise are both already checked"
        );
        self.diagnostics.gnss_position.record_accepted(0.0, None);
        self.diagnostics.gnss_height.record_accepted(0.0, None);
        GnssFusion::both(Fusion::Accepted { test_ratio: 0.0 })
    }

    /// Fuse a GNSS velocity solution. Equation (29).
    ///
    /// After a coarse start the first solution is adopted rather than fused; see
    /// [`Fusion::Reset`].
    ///
    /// `noise` is the receiver's speed accuracy, `sacc`, bounded the way
    /// [`fuse_gnss_position`](Self::fuse_gnss_position)'s is:
    /// [`VelocityNoise::clamped`](crate::VelocityNoise::clamped). Where the solution
    /// carries no usable vertical velocity, or the receiver reports the axes separately,
    /// [`VelocityNoise::horizontal_vertical`](crate::VelocityNoise::horizontal_vertical)
    /// is the constructor to reach for: all three axes are fused or none, and a per-axis
    /// σ is what says which of them the receiver actually measured.
    ///
    /// A solution inconsistent with the estimate at
    /// [`Gates::gnss_velocity`](crate::Gates) is [`Fusion::Rejected`] and changes nothing
    /// but the source's health. One joint test over all three axes, as (28)'s is.
    ///
    /// Fused at the variance (24′) leaves for a solution whose error persists from the last,
    /// [`Config::correlation`](crate::Config::correlation)'s `gnss_velocity`;
    /// the gate reads `noise` itself.
    ///
    /// `antenna` is as for [`fuse_gnss_position`](Self::fuse_gnss_position): the solution is
    /// the antenna's velocity, `v + R(ω × r)` of (29′), with `ω` the rate the vehicle turned
    /// at over the solution's age, read off the state's own history rather than the last
    /// sample.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_gnss_velocity(
        &mut self,
        time: Timestamp,
        velocity: Velocity<Ned>,
        noise: VelocityNoise<Ned>,
        antenna: Position<Body>,
    ) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.gnss_velocity, refusal);
        }
        self.diagnostics.gnss_velocity.note_arrival(time);
        if !velocity.is_finite() || !noise.is_finite() || !antenna.is_finite() {
            return refuse(&mut self.diagnostics.gnss_velocity, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.gnss_velocity, Fusion::InvalidNoise);
        }
        self.weigh_yaw(time, velocity, noise, antenna);
        self.adopt_first_yaw();
        if self.unestablished.velocity {
            let Some(adopted) = self.carried_velocity(velocity, antenna, time) else {
                return refuse(&mut self.diagnostics.gnss_velocity, Fusion::NotFinite);
            };
            self.adopt_velocity(adopted, noise);
            self.unestablished.velocity = false;
            self.diagnostics.gnss_velocity.record_adopted();
            return Fusion::Reset;
        }
        let observation = self
            .observe(time, |past, omega| {
                gnss::velocity_observation(past, velocity, noise, antenna, omega)
            })
            .correlated(correlation_inflation(
                self.diagnostics.gnss_velocity.since_measured,
                self.config.correlation.gnss_velocity,
            ));
        let outcome = update(
            self.estimate.state(),
            &self.covariance,
            &self.offset,
            &observation,
            self.config.gates.gnss_velocity,
        );
        if matches!(outcome, Update::Rejected { .. }) {
            self.replace_failed_yaw();
        }
        let fusion = self.apply_or_recover(
            outcome,
            |diagnostics| &mut diagnostics.gnss_velocity,
            recovery_after(
                self.hold.holds_velocity() || self.yaw_replaced.owes_velocity(),
                self.config.recovery.gnss_velocity,
            ),
            |filter| {
                let adopted = filter.carried_velocity(velocity, antenna, time);
                adopted
                    .map(|adopted| filter.adopt_velocity(adopted, noise))
                    .is_some()
            },
        );
        if matches!(fusion, Fusion::Accepted { .. }) {
            self.yaw_replaced.settle_velocity();
        }
        // An adoption ended the hold's claim on velocity in `adopt_velocity`.
        if matches!(fusion, Fusion::Accepted { .. }) {
            self.hold.end_velocity();
        }
        fusion
    }

    /// Fuse a barometric altitude. Equation (30).
    ///
    /// The measurement is `z = -(α - α₀)`, so it needs a reference: one
    /// [`initialize`](Self::initialize) derived from a window taken at rest, one
    /// [`set_baro_reference`](Self::set_baro_reference) named, or — failing both — one this
    /// call establishes from the estimate, below.
    ///
    /// `noise` being per call is what lets it carry a condition neither platform's
    /// parameter can: ArduPilot multiplies the barometer variance by 4 in ground effect
    /// (`gndEffectBaroScaler`, `AP_NavEKF3.h:511`, applied at
    /// `AP_NavEKF3_PosVelFusion.cpp:1419`), and PX4 runs a deadzone around the same
    /// condition. An application here inflates σ on the calls it applies to, and says in
    /// its own code when that is.
    ///
    /// An altitude inconsistent with the estimate at
    /// [`Gates::baro_altitude`](crate::Gates) is [`Fusion::Rejected`] and changes nothing
    /// but the source's health. One degree of freedom, so the threshold is a `Gate<1>` and
    /// a `Gate<3>` in that field does not compile.
    ///
    /// With no reference held, the first altitude offered once position is established
    /// sets one, `α̂₀ = α + p̂_D`, so that it lands on the estimate — PX4's
    /// `baro_height_control.cpp:79` at `c4e4ef98`. That covers every start that leaves no
    /// reference: one in motion after its first GNSS fix, a window with no barometer in it,
    /// and a seed. Leaving it to the caller costs `cd7e0001`, a coarse start, all 3530 of its
    /// altitudes, with nothing but [`Fusion::NoReference`] on a source nobody reads to say
    /// so. [`Config::baro_reference_from_estimate`](crate::Config::baro_reference_from_estimate)
    /// turns it off for a caller that names its own.
    ///
    /// The reference inherits the height error of the estimate it was read against, so it is
    /// seeded correlated with it, `P_bb = P_DD + R_m` and `P_xb = −P[:, D]` of (30′), rather
    /// than as an independent σ. That is what keeps the error the first fix left in `α̂₀`
    /// from reading as a hundred barometer readings' worth of agreement: held constant and
    /// uncorrelated, it took `moving_start`'s `nees_pos` to 112.59.
    ///
    /// The altitude is spent placing the reference rather than fused, as a first fix is
    /// spent placing the origin in [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic), and is
    /// reported the same way: accepted with a zero test ratio, since nothing was
    /// inconsistent. Fusing it as well would count its noise twice, once in `P_bb`.
    ///
    /// Until position is established there is no estimate to read the reference against,
    /// and the altitude is refused with [`Fusion::NoReference`] rather than adopted: a
    /// height is not a quantity this source can establish, because `α₀` is what relates
    /// it to the origin.
    ///
    /// Fused at the variance (24′) leaves for an altitude whose error persists from the last,
    /// [`Config::correlation`](crate::Config::correlation)'s `baro_altitude`;
    /// the gate reads `noise` itself.
    ///
    /// `time` is when the measurement was taken, on the clock the IMU's samples are timed on;
    /// [`Fusion::OutOfHorizon`] says which times cannot be placed.
    pub fn fuse_baro_altitude(
        &mut self,
        time: Timestamp,
        altitude: Altitude,
        noise: AltitudeNoise,
    ) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.baro_altitude, refusal);
        }
        self.diagnostics.baro_altitude.note_arrival(time);
        if !altitude.as_meters().is_finite() || !noise.is_finite() {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.baro_altitude, Fusion::InvalidNoise);
        }
        let Some(reference) = self.baro_reference else {
            if self.unestablished.position || !self.config.baro_reference_from_estimate {
                return refuse(&mut self.diagnostics.baro_altitude, Fusion::NoReference);
            }
            self.reference_from_estimate(altitude, noise, time);
            self.diagnostics.baro_altitude.record_accepted(0.0, None);
            return Fusion::Accepted { test_ratio: 0.0 };
        };
        let observation = self
            .observe(time, |past, _| {
                baro::altitude_observation(past, altitude, reference, noise)
            })
            .correlated(correlation_inflation(
                self.diagnostics.baro_altitude.since_measured,
                self.config.correlation.baro_altitude,
            ));
        let outcome = update(
            self.estimate.state(),
            &self.covariance,
            &self.offset,
            &observation,
            self.config.gates.baro_altitude,
        );
        // Read against the estimate, so only once there is an estimate to read it against, and
        // never over a reference the caller owns.
        let after = if self.unestablished.position || !self.config.baro_reference_from_estimate {
            None
        } else {
            self.config.recovery.baro_altitude
        };
        self.apply_or_recover(
            outcome,
            |diagnostics| &mut diagnostics.baro_altitude,
            after,
            |filter| {
                filter.reference_from_estimate(altitude, noise, time);
                true
            },
        )
    }
}

/// Why one half of a GNSS fix cannot be judged, or `None` if it can: the checks every
/// `fuse_*` makes of a whole measurement, made of the components that half reads.
fn screen(values: &[f32], variances: &[f32]) -> Option<Fusion> {
    if !values.iter().chain(variances).all(|v| v.is_finite()) {
        return Some(Fusion::NotFinite);
    }
    if !variances.iter().all(|v| *v > 0.0) {
        return Some(Fusion::InvalidNoise);
    }
    None
}

/// Record a refusal against the source that produced it, and hand the outcome back to the
/// caller.
///
/// One place maps an outcome to what `Diagnostics` stores, so a `fuse_*` that grows another
/// guard cannot forget to count it. A refusal moves no timer; see
/// [`SourceHealth::record_refused`].
pub(super) fn refuse(source: &mut SourceHealth, outcome: Fusion) -> Fusion {
    if let Some(refusal) = outcome.refusal() {
        source.record_refused(refusal);
    }
    outcome
}

#[cfg(test)]
mod tests {

    use crate::config::{Correlation, GRAVITY, LATENCY_HORIZON};
    use crate::eskf::Eskf;
    use crate::eskf::fixtures::*;
    use crate::frames::Ned;
    use crate::geodetic::Geodetic;
    use crate::health::{Fusion, GnssFusion, Propagation, Refusal};
    use crate::init::tests::still;

    use crate::observation::mag::tests::{attitude_of, measured};

    use crate::propagate::ImuSample;
    use crate::state::{Covariance, ErrorState, STATES, State};
    use crate::units::{
        Acceleration, Altitude, AltitudeNoise, AngularRate, HeadingNoise, MagField, Position,
        PositionNoise, Radians, Seconds, Timestamp, Velocity, VelocityNoise,
    };

    /// The horizon, both ends, at the barometer: a reading older than [`LATENCY_HORIZON`] or
    /// further ahead of the state than a step is refused, counted as a refusal and moves no
    /// timer, and one inside either bound reaches the gate.
    #[test]
    fn a_measurement_the_filter_cannot_place_in_time_is_refused() {
        let mut filter = aided();
        let now = filter.now();
        let altitude = Altitude::from_meters(100.0);
        let noise = AltitudeNoise::from_sigma(2.0);
        let before = filter.diagnostics().baro_altitude;

        let past = Seconds::from_secs(LATENCY_HORIZON.as_secs() + 0.001);
        let ahead = Seconds::from_secs(filter.config().max_predict_dt.as_secs() + 0.001);
        for (time, age) in [
            (now.before(past), past.as_secs()),
            (now.after(ahead), -ahead.as_secs()),
        ] {
            let Fusion::OutOfHorizon { age: refused } =
                filter.fuse_baro_altitude(time, altitude, noise)
            else {
                panic!("not refused");
            };
            assert!(
                (refused.as_secs() - age).abs() < 1e-6,
                "{refused:?} against {age}"
            );
        }
        let health = filter.diagnostics().baro_altitude;
        assert_eq!(health.refused, before.refused + 2);
        assert_eq!(health.last_refusal, Some(Refusal::OutOfHorizon));
        assert_eq!(health.time_since_accepted, before.time_since_accepted);

        let inside = Seconds::from_secs(LATENCY_HORIZON.as_secs() - 0.001);
        assert!(
            filter
                .fuse_baro_altitude(now.before(inside), altitude, noise)
                .is_accepted()
        );
        assert!(
            filter
                .fuse_baro_altitude(now.after(DT), altitude, noise)
                .is_accepted()
        );
    }

    #[test]
    fn a_refused_measurement_is_visible_in_diagnostics_not_only_in_the_return_value() {
        // The failure this guards: a miswired sensor feeding NaN reads as `never accepted`,
        // exactly like one that was never connected, unless the refusal is counted.
        let mut filter = initialized();
        let nan = Velocity::ned(f32::NAN, 0.0, 0.0);
        let noise = VelocityNoise::from_speed_accuracy(0.3);

        assert_eq!(
            filter.fuse_gnss_velocity(filter.now(), nan, noise, Position::zero()),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_gnss_velocity(
                filter.now(),
                Velocity::ned(1.0, 0.0, 0.0),
                VelocityNoise::from_variance(0.0, 1.0, 1.0),
                Position::zero()
            ),
            Fusion::InvalidNoise
        );

        let health = filter.diagnostics().gnss_velocity;
        assert_eq!(health.refused, 2);
        assert_eq!(health.last_refusal, Some(Refusal::InvalidNoise));
        assert_eq!(health.accepted, 0);
        assert_eq!(health.rejected, 0, "neither reached the gate");
        assert_eq!(
            health.time_since_accepted, None,
            "a refusal is not aiding, so no timer starts"
        );
        assert!(!health.has_been_used());
    }

    #[test]
    fn a_refusal_names_which_kind_it_was() {
        let mut filter = coarse();
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::NoReference
        );
        assert_eq!(
            filter.diagnostics().baro_altitude.last_refusal,
            Some(Refusal::NoReference),
            "a missing reference is a different problem from a bad number"
        );

        let mut fresh = Eskf::default();
        assert_eq!(
            fresh.fuse_baro_altitude(
                fresh.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::NotInitialized
        );
        assert_eq!(
            fresh.diagnostics().baro_altitude.last_refusal,
            Some(Refusal::NotInitialized)
        );
    }

    #[test]
    fn an_altitude_above_the_reference_pulls_the_estimate_up_and_moves_nothing_sideways() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_at(100.0), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");

        // Two meters above the reference the window fixed, on a sensor claiming 0.5 m.
        assert!(
            filter
                .fuse_baro_altitude(
                    filter.now(),
                    Altitude::from_meters(102.0),
                    AltitudeNoise::from_sigma(0.5)
                )
                .is_accepted()
        );

        let position = filter.state().position.vector();
        assert!(
            position[2] < -1.0,
            "up is negative down: {} should be near -2 m",
            position[2]
        );
        assert_eq!(
            (position[0], position[1]),
            (0.0, 0.0),
            "(30) observes p_D alone, and a still start correlates it with nothing"
        );
    }

    #[test]
    fn an_altitude_the_gate_turns_down_leaves_the_estimate_where_it_was() {
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_at(100.0), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let before = filter.state().position;

        // A hundred meters of climb the instant the window closed, on a 0.5 m sensor: the
        // shape of a pressure transient, and what `Gates::baro_altitude` is there for.
        let outcome = filter.fuse_baro_altitude(
            filter.now(),
            Altitude::from_meters(200.0),
            AltitudeNoise::from_sigma(0.5),
        );
        assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        assert_eq!(filter.state().position, before);
        assert_eq!(filter.diagnostics().baro_altitude.rejected, 1);
    }

    /// Equation (23′): a fix 250 ms old on a vehicle at 20 m/s is where the vehicle was, 5 m
    /// back along its track. Taken at its own time it agrees with the estimate; taken as
    /// current it is 5 m of error the gate turns down.
    #[test]
    fn an_old_fix_is_judged_against_where_the_vehicle_was() {
        let flying = || {
            // The hold would pull a vehicle nothing aids back toward where it started.
            let mut filter = Eskf::new(crate::Config {
                hold: None,
                ..crate::Config::default()
            })
            .unwrap();
            let state = State {
                velocity: Velocity::ned(20.0, 0.0, 0.0),
                ..State::default()
            };
            let _ = filter
                .seed(state, Covariance::from_sigmas([0.5; STATES]))
                .expect("a sane seed");
            for _ in 0..50 {
                assert!(filter.step(still().imu, DT).is_propagated());
            }
            filter
        };
        let age = Seconds::from_secs(0.25);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);

        let mut filter = flying();
        let there = Position::ned(filter.state().position.x() - 5.0, 0.0, 0.0);
        let taken = filter.now().before(age);
        let aged = filter
            .fuse_gnss_position(taken, there, noise, Position::zero())
            .horizontal;
        assert!(
            aged.test_ratio().is_some_and(|ratio| ratio < 1.0e-3),
            "{aged:?}"
        );

        let mut filter = flying();
        let now = filter.now();
        let current = filter
            .fuse_gnss_position(now, there, noise, Position::zero())
            .horizontal;
        assert!(matches!(current, Fusion::Rejected { .. }), "{current:?}");
    }

    /// A start in motion says nothing about where the vehicle was before it, so a fix taken
    /// before a seed is refused; a still window says the vehicle was where it started, so one
    /// taken before that start is placed there and fused.
    #[test]
    fn a_fix_from_before_the_start_is_placed_only_after_a_start_at_rest() {
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let before = Seconds::from_secs(0.05);

        let mut seeded = Eskf::default();
        let (state, covariance) = seed();
        let start = Timestamp::from_micros(1_000_000);
        let _ = seeded
            .initialize_from(state, covariance, start)
            .expect("a sane seed");
        let taken = start.before(before);
        let refused =
            seeded.fuse_gnss_position(taken, Position::ned(0.0, 0.0, 0.0), noise, Position::zero());
        assert_eq!(
            refused,
            GnssFusion::both(Fusion::OutOfHorizon { age: before })
        );

        let mut still = initialized();
        let taken = still.now().before(before);
        let fused =
            still.fuse_gnss_position(taken, Position::ned(0.0, 0.0, 0.0), noise, Position::zero());
        assert!(fused.is_accepted(), "{fused:?}");
    }

    /// A correction reaches the past it was propagated from, so a second fix of the same moment
    /// is judged against the corrected past and agrees with it better than the first did. Were
    /// the history left behind, the second would innovate as the first did against a covariance
    /// the first had already narrowed: 1.33 against 0.12, where this reads 0.08.
    #[test]
    fn a_correction_reaches_the_history_the_next_old_fix_is_judged_against() {
        let mut filter = Eskf::default();
        let _ = filter
            .seed(State::default(), Covariance::from_sigmas([2.0; STATES]))
            .expect("a sane seed");
        for _ in 0..50 {
            assert!(filter.step(still().imu, DT).is_propagated());
        }
        let taken = filter.now().before(Seconds::from_secs(0.2));
        let off = Position::ned(3.0, 0.0, 0.0);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let first = filter
            .fuse_gnss_position(taken, off, noise, Position::zero())
            .horizontal;
        let second = filter
            .fuse_gnss_position(taken, off, noise, Position::zero())
            .horizontal;
        let (Some(first), Some(second)) = (first.test_ratio(), second.test_ratio()) else {
            panic!("both fixes reach the gate: {first:?}, {second:?}");
        };
        assert!(second < first, "{second} against {first}");
    }

    /// A fix timed ahead of the last IMU sample is where the vehicle will be: carried forward
    /// on the velocity rather than taken as current.
    #[test]
    fn a_fix_ahead_of_the_state_is_judged_against_where_the_vehicle_will_be() {
        let mut filter = Eskf::default();
        let state = State {
            velocity: Velocity::ned(20.0, 0.0, 0.0),
            ..State::default()
        };
        let _ = filter
            .seed(state, Covariance::from_sigmas([0.5; STATES]))
            .expect("a sane seed");
        for _ in 0..50 {
            assert!(filter.step(still().imu, DT).is_propagated());
        }
        let lead = Seconds::from_secs(0.05);
        let there = Position::ned(filter.state().position.x() + 1.0, 0.0, 0.0);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let taken = filter.now().after(lead);
        let outcome = filter
            .fuse_gnss_position(taken, there, noise, Position::zero())
            .horizontal;
        assert!(
            outcome.test_ratio().is_some_and(|ratio| ratio < 1.0e-3),
            "{outcome:?}"
        );
    }

    /// A heading timed ahead of the last IMU sample is judged against the attitude the turn
    /// will have reached, carried forward on the last sample's rate. Taken against the present
    /// instead, 90 ms into a 90°/s turn is 8° of error, which the gate turns down.
    #[test]
    fn a_heading_ahead_of_the_state_is_judged_against_the_turn() {
        let rate = core::f32::consts::FRAC_PI_2;
        let mut sigmas = [0.5; STATES];
        // Attitude and gyroscope bias tight, so that the covariance cannot absorb the turn.
        for axis in [
            ErrorState::AttitudeX,
            ErrorState::AttitudeY,
            ErrorState::AttitudeZ,
        ] {
            sigmas[axis.index()] = 0.005;
        }
        for axis in [
            ErrorState::GyroBiasX,
            ErrorState::GyroBiasY,
            ErrorState::GyroBiasZ,
        ] {
            sigmas[axis.index()] = 0.001;
        }
        let mut filter = Eskf::default();
        let _ = filter
            .seed(State::default(), Covariance::from_sigmas(sigmas))
            .expect("a sane seed");
        let turning = still().imu.with_gyro(AngularRate::body(0.0, 0.0, rate));
        for _ in 0..20 {
            assert!(filter.step(turning, DT).is_propagated());
        }
        let lead = Seconds::from_secs(0.09);
        let (_, _, yaw) = filter.state().attitude.euler_angles();
        let field_at = |yaw: f32| MagField::body(0.22 * yaw.cos(), -0.22 * yaw.sin(), 0.44);
        let noise = HeadingNoise::from_sigma(0.02);

        let mut judged = filter.clone();
        let ahead = field_at(yaw + rate * lead.as_secs());
        let outcome = judged.fuse_mag_heading(filter.now().after(lead), ahead, noise);
        assert!(
            outcome.test_ratio().is_some_and(|ratio| ratio < 0.05),
            "{outcome:?}"
        );
        let current = filter.fuse_mag_heading(filter.now(), ahead, noise);
        assert!(matches!(current, Fusion::Rejected { .. }), "{current:?}");
    }

    #[test]
    fn a_run_of_correlated_fixes_leaves_more_uncertainty_than_a_run_of_white_ones() {
        // Twenty-five fixes at 5 Hz. As white, (24) averages them down; at (24′)'s default,
        // each is worth 1/42 of one horizontally and 1/140 in height, since the receiver's
        // error has barely moved between them. Worth less is not worth nothing: each half reads
        // its own clock, and one read off the other would see the horizontal half restart it
        // an instant before and take every height for the same error twice.
        let mut white = aided();
        white.config.correlation = Correlation::WHITE;
        let (mut correlated, mut unaided) = (aided(), aided());
        for step in 0..500 {
            for filter in [&mut white, &mut correlated, &mut unaided] {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
            if step % 20 == 0 {
                for filter in [&mut white, &mut correlated] {
                    let _ = filter.fuse_gnss_position(
                        filter.now(),
                        Position::ned(0.0, 0.0, 0.0),
                        PositionNoise::from_sigma(1.0, 1.0, 1.0),
                        Position::zero(),
                    );
                }
            }
        }
        for axis in [ErrorState::PositionNorth, ErrorState::PositionDown] {
            let variance = |filter: &Eskf| filter.covariance().variance(axis);
            let (w, c, u) = (variance(&white), variance(&correlated), variance(&unaided));
            assert!(c > 3.0 * w, "{axis:?}: {c} against {w} white");
            assert!(c < 0.95 * u, "{axis:?}: {c} against {u} with no fix at all");
        }
    }

    #[test]
    fn every_source_is_fused_at_its_own_correlation() {
        // Velocity at 5 Hz and heading at 20 Hz for 5 s: as white, (24) averages each down;
        // at their defaults each solution is worth about a fifth, each heading about a
        // thirty-fifth. Each source reads its own `τ` and its own clock.
        let mut white = initialized();
        white.config.correlation = Correlation::WHITE;
        let mut correlated = initialized();
        for step in 0..500 {
            for filter in [&mut white, &mut correlated] {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
                if step % 20 == 0 {
                    let _ = filter.fuse_gnss_velocity(
                        filter.now(),
                        Velocity::ned(0.0, 0.0, 0.0),
                        VelocityNoise::from_speed_accuracy(0.3),
                        Position::zero(),
                    );
                }
                if step % 5 == 0 {
                    let _ = filter.fuse_mag_heading(
                        filter.now(),
                        measured(attitude_of(0.0, 0.0, 0.0), 0.0),
                        HeadingNoise::from_sigma(0.05),
                    );
                }
            }
        }
        for axis in [ErrorState::VelocityNorth, ErrorState::AttitudeZ] {
            let (w, c) = (
                white.covariance().variance(axis),
                correlated.covariance().variance(axis),
            );
            assert!(c > 2.0 * w, "{axis:?}: {c} against {w} white");
        }
    }

    #[test]
    fn a_refused_fix_does_not_restart_the_interval_of_24_prime() {
        // A receiver interleaving unusable fixes with good ones carries no error in the bad
        // ones, so the good ones are as far apart as they were.
        let good = |filter: &mut Eskf| {
            let _ = filter.fuse_gnss_position(
                filter.now(),
                Position::ned(0.0, 0.0, 0.0),
                PositionNoise::from_sigma(1.0, 1.0, 1.0),
                Position::zero(),
            );
        };
        let (mut interleaved, mut clean) = (aided(), aided());
        for filter in [&mut interleaved, &mut clean] {
            good(filter);
            for _ in 0..10 {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
        }
        let refused = interleaved.fuse_gnss_position(
            interleaved.now(),
            Position::ned(f32::NAN, f32::NAN, f32::NAN),
            PositionNoise::from_sigma(1.0, 1.0, 1.0),
            Position::zero(),
        );
        assert_eq!(refused, GnssFusion::both(Fusion::NotFinite));
        for filter in [&mut interleaved, &mut clean] {
            for _ in 0..10 {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
            good(filter);
        }
        let north = ErrorState::PositionNorth;
        assert_eq!(
            interleaved.covariance().variance(north),
            clean.covariance().variance(north)
        );
    }

    #[test]
    fn a_rejected_fix_or_a_refused_half_leaves_the_interval_alone() {
        // (24′) discounts a fix for the error it shares with those already fused, and neither a
        // rejected fix nor a refused half fused anything: the next is timed from the last fix
        // accepted, 110 ms before it, not from the one turned away 10 ms before. The half that
        // was accepted does restart its own clock, which the two halves keep apart.
        let fix = |filter: &mut Eskf, position: Position<Ned>, noise: PositionNoise<Ned>| {
            let _ = filter.fuse_gnss_position(filter.now(), position, noise, Position::zero());
        };
        let one = PositionNoise::from_sigma(1.0, 1.0, 1.0);
        let steps = |filter: &mut Eskf, n: usize| {
            for _ in 0..n {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
        };
        let (mut rejected, mut half, mut clean) = (aided(), aided(), aided());
        for filter in [&mut rejected, &mut half, &mut clean] {
            fix(filter, Position::ned(0.0, 0.0, 0.0), one);
            steps(filter, 10);
        }
        let outcome = rejected.fuse_gnss_position(
            rejected.now(),
            Position::ned(1000.0, 0.0, 1000.0),
            one,
            Position::zero(),
        );
        assert!(
            matches!(outcome.horizontal, Fusion::Rejected { .. }),
            "{outcome:?}"
        );
        let outcome = half.fuse_gnss_position(
            half.now(),
            Position::ned(0.0, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.0, f32::NAN),
            Position::zero(),
        );
        assert_eq!(outcome.height, Fusion::NotFinite);
        for filter in [&mut rejected, &mut half, &mut clean] {
            steps(filter, 1);
            fix(filter, Position::ned(0.0, 0.0, 0.0), one);
        }
        let (north, down) = (ErrorState::PositionNorth, ErrorState::PositionDown);
        let variance = |filter: &Eskf, axis| filter.covariance().variance(axis);
        assert_eq!(variance(&rejected, north), variance(&clean, north));
        assert_eq!(variance(&half, down), variance(&clean, down));
    }

    #[test]
    fn the_interval_of_24_prime_restarts_with_the_filter() {
        // The filter's clock restarts at initialization, so the fix before it is not the
        // previous one. Were it kept, the first fix after 0.2 s of the new clock would be
        // fused at 1/42 of its weight.
        let fix = |filter: &mut Eskf| {
            let _ = filter.fuse_gnss_position(
                filter.now(),
                Position::ned(0.0, 0.0, 0.0),
                PositionNoise::from_sigma(1.0, 1.0, 1.0),
                Position::zero(),
            );
        };
        let mut restarted = aided();
        fix(&mut restarted);
        let _ = restarted
            .initialize_over(&window_at(100.0), Seconds::from_secs(0.25))
            .expect("a 2 s window of stillness");
        let mut fresh = aided();
        for filter in [&mut restarted, &mut fresh] {
            for _ in 0..20 {
                assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            }
            fix(filter);
        }
        let north = ErrorState::PositionNorth;
        assert_eq!(
            restarted.covariance().variance(north),
            fresh.covariance().variance(north)
        );
    }

    #[test]
    fn a_fix_that_agrees_with_the_estimate_narrows_its_uncertainty() {
        let mut filter = initialized();
        let before = filter.covariance().variance(ErrorState::PositionNorth);
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.5, -0.5, 0.2),
            PositionNoise::horizontal_vertical(1.0, 1.0),
            Position::zero(),
        );

        let Fusion::Accepted { test_ratio } = outcome.horizontal else {
            panic!("expected an acceptance, got {outcome:?}");
        };
        assert!(test_ratio > 0.0 && test_ratio <= 1.0);
        assert!(filter.covariance().variance(ErrorState::PositionNorth) < before);
        assert!(filter.state().position.x() > 0.0, "moved toward the fix");
        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.test_ratio, Some(test_ratio));
        assert!(health.innovation.is_some());
    }

    #[test]
    fn a_rejected_fix_changes_nothing_but_the_sources_record_of_it() {
        let mut filter = initialized();
        let noise = PositionNoise::horizontal_vertical(1.0, 1.0);
        assert!(
            filter
                .fuse_gnss_position(
                    filter.now(),
                    Position::ned(0.1, 0.0, 0.0),
                    noise,
                    Position::zero()
                )
                .is_accepted()
        );
        assert!(filter.step(still().imu, DT).is_propagated());
        let (state, covariance) = (filter.state(), *filter.covariance());
        let timer = filter.diagnostics().gnss_position.time_since_accepted;

        // A kilometer out in both halves, so neither changes anything.
        let both = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(1000.0, 0.0, 1000.0),
            noise,
            Position::zero(),
        );
        assert!(matches!(both.height, Fusion::Rejected { .. }), "{both:?}");
        let outcome = both.horizontal;
        assert!(matches!(outcome, Fusion::Rejected { test_ratio } if test_ratio > 1.0));
        assert_eq!(filter.state(), state);
        assert_eq!(filter.covariance(), &covariance);
        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.time_since_accepted, timer);
        assert_eq!((health.rejected, health.consecutive_rejections), (1, 1));
        assert_eq!(health.test_ratio, outcome.test_ratio());
    }

    #[test]
    fn a_covariance_that_is_no_longer_one_refuses_the_update() {
        let mut filter = initialized();
        let mut p = *filter.covariance().as_matrix();
        p[(0, 0)] = -10.0;
        filter.covariance = Covariance::from_matrix(p);
        let state = filter.state();

        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.1, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.0, 1.0),
            Position::zero(),
        );
        assert_eq!(outcome.horizontal, Fusion::StateInvalid);
        let health = filter.diagnostics().gnss_position;
        assert_eq!(health.last_refusal, Some(Refusal::StateInvalid));
        assert_eq!((health.accepted, health.rejected), (0, 0));
        // The broken variance is north's, which a height update never reads, so the height
        // half is judged against a block that is still a covariance.
        assert!(outcome.height.is_accepted(), "{outcome:?}");
        assert_eq!(
            filter.state().position.vector()[0],
            state.position.vector()[0]
        );
    }

    #[test]
    fn a_height_the_estimate_disagrees_with_costs_no_horizontal_aiding() {
        // The failure the split exists for: a receiver 50 m off in height on a 1 m σ,
        // with a horizontal fix that agrees.
        let mut filter = initialized();
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.2, -0.1, -50.0),
            PositionNoise::horizontal_vertical(1.0, 1.0),
            Position::zero(),
        );
        assert!(
            matches!(outcome.horizontal, Fusion::Accepted { .. }),
            "{outcome:?}"
        );
        assert!(
            matches!(outcome.height, Fusion::Rejected { .. }),
            "{outcome:?}"
        );
        let d = filter.diagnostics();
        assert_eq!((d.gnss_position.accepted, d.gnss_position.rejected), (1, 0));
        assert_eq!((d.gnss_height.accepted, d.gnss_height.rejected), (0, 1));
        assert!(
            filter.state().position.vector()[2].abs() < 1e-3,
            "height untouched"
        );
    }

    #[test]
    fn a_2d_fix_with_no_usable_height_still_fuses_its_horizontal_position() {
        let mut filter = initialized();
        let outcome = filter.fuse_gnss_position(
            filter.now(),
            Position::ned(0.2, -0.1, 0.0),
            PositionNoise::<Ned>::from_variance(1.0, 1.0, 0.0),
            Position::zero(),
        );
        assert!(
            matches!(outcome.horizontal, Fusion::Accepted { .. }),
            "{outcome:?}"
        );
        assert_eq!(outcome.height, Fusion::InvalidNoise);
        assert_eq!(
            filter.diagnostics().gnss_height.last_refusal,
            Some(Refusal::InvalidNoise)
        );
    }

    /// Nose east and level, so body forward is navigation east: an antenna 1 m forward and
    /// 0.5 m up is 1 m east of the IMU and 0.5 m above it, worked by hand.
    fn nose_east() -> Eskf {
        seeded(
            attitude_of(0.0, 0.0, core::f32::consts::FRAC_PI_2),
            AngularRate::zero(),
        )
    }

    #[test]
    fn a_fix_of_the_antenna_where_the_estimate_puts_it_moves_nothing() {
        let mut filter = nose_east();
        let before = filter.state().position;
        let at_antenna = Position::ned(0.0, 1.0, -0.5);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let fused = filter.fuse_gnss_position(filter.now(), at_antenna, noise, mast());
        assert!(
            fused
                .horizontal
                .test_ratio()
                .is_some_and(|ratio| ratio < 1e-9),
            "{fused:?}"
        );
        assert!(fused.height.test_ratio().is_some_and(|ratio| ratio < 1e-9));
        assert!(near(filter.state().position, before));

        // The same fix read as the IMU's is a meter east of the estimate.
        let mut unarmed = nose_east();
        let fused = unarmed.fuse_gnss_position(unarmed.now(), at_antenna, noise, Position::zero());
        assert!(
            fused
                .horizontal
                .test_ratio()
                .is_some_and(|ratio| ratio > 0.1)
        );
    }

    #[test]
    fn a_velocity_of_the_antenna_includes_its_swing_about_the_imu() {
        // Yawing at 0.5 rad/s, nose east: an antenna 1 m forward swings south at 0.5 m/s on
        // top of whatever the IMU is doing.
        let mut filter = nose_east();
        let imu = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 0.5),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        );
        assert!(filter.step(imu, DT).is_propagated());
        let v = filter.state().velocity.vector();
        let swung = Velocity::ned(v.x - 0.5, v.y, v.z);
        let noise = VelocityNoise::from_speed_accuracy(0.1);
        let fused = filter.fuse_gnss_velocity(filter.now(), swung, noise, mast());
        let ratio = fused.test_ratio().expect("fused");
        assert!(ratio < 1e-3, "ratio {ratio}");
    }

    #[test]
    fn an_antenna_that_is_not_a_number_is_refused_before_it_reaches_the_state() {
        let broken = Position::body(f32::NAN, 0.0, 0.0);
        let mut filter = coarse();
        let before = filter.state();
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let fused = filter.fuse_gnss_position(filter.now(), Position::zero(), noise, broken);
        assert_eq!(fused, GnssFusion::both(Fusion::NotFinite));
        let noise = VelocityNoise::from_speed_accuracy(0.3);
        let fused = filter.fuse_gnss_velocity(filter.now(), Velocity::zero(), noise, broken);
        assert_eq!(fused, Fusion::NotFinite);
        let noise = PositionNoise::horizontal_vertical(0.5, 0.5);
        let fused = filter.fuse_gnss_geodetic(filter.now(), zurich(), noise, broken);
        assert_eq!(fused, GnssFusion::both(Fusion::NotFinite));
        assert_eq!(filter.state().position, before.position);
        assert_eq!(filter.state().velocity, before.velocity);
        assert_eq!(filter.origin(), None);
    }

    #[test]
    fn a_fix_that_is_not_a_number_is_refused_even_with_an_origin_held() {
        // The case that matters: an origin held and position unestablished, where a NaN
        // would otherwise be adopted outright.
        let mut filter = initialized();
        assert!(filter.set_origin(zurich()));
        let mut window = [still(); 8];
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let _ = filter
            .initialize_over(&window, Seconds::from_secs(0.25))
            .expect("moving, so position is unestablished");
        let noise = PositionNoise::horizontal_vertical(1.5, 1.5);

        let nonsense = Geodetic::from_degrees(f64::NAN, 8.5, 488.0);
        assert_eq!(
            filter.fuse_gnss_geodetic(filter.now(), nonsense, noise, Position::zero()),
            GnssFusion::both(Fusion::NotFinite)
        );
        assert_eq!(
            filter.fuse_gnss_geodetic(
                filter.now(),
                zurich(),
                PositionNoise::from_sigma(f32::NAN, 1.5, 1.5),
                Position::zero()
            ),
            GnssFusion::both(Fusion::NotFinite)
        );
        assert!(filter.state().position.is_finite());
        assert!(
            filter
                .fuse_gnss_geodetic(filter.now(), zurich(), noise, Position::zero())
                .is_reset()
        );
    }

    #[test]
    fn every_source_refuses_a_measurement_that_is_not_a_number() {
        let mut filter = initialized();
        assert!(filter.set_baro_reference(
            Altitude::from_meters(52.0),
            AltitudeNoise::from_sigma(0.001)
        ));
        let nan = f32::NAN;
        assert_eq!(
            filter
                .fuse_gnss_position(
                    filter.now(),
                    Position::ned(nan, 0.0, 0.0),
                    PositionNoise::horizontal_vertical(1.5, 1.5),
                    Position::zero()
                )
                .horizontal,
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_gnss_velocity(
                filter.now(),
                Velocity::ned(0.0, 0.0, 0.0),
                VelocityNoise::from_speed_accuracy(nan),
                Position::zero()
            ),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(nan),
                AltitudeNoise::from_sigma(2.0)
            ),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_mag_heading(
                filter.now(),
                MagField::body(0.2, 0.0, 0.4),
                HeadingNoise::from_variance(f32::INFINITY)
            ),
            Fusion::NotFinite
        );
        assert_eq!(
            filter.fuse_stationary(filter.now(), VelocityNoise::from_speed_accuracy(nan)),
            Fusion::NotFinite
        );
        assert_eq!(filter.diagnostics().gnss_position.accepted, 0);
    }

    #[test]
    fn every_source_measures_its_period_from_what_it_offers() {
        // Refused, every one of them: a period is how often the sensor speaks, not how often
        // it is believed, so a source turning out NaN is still timed against its own rate.
        let mut filter = initialized();
        let nan = f32::NAN;
        hold(&mut filter, 4.0, 20, |filter| {
            let now = filter.now();
            let noise = PositionNoise::horizontal_vertical(1.5, 1.5);
            let _ = filter.fuse_gnss_position(
                now,
                Position::ned(nan, nan, nan),
                noise,
                Position::zero(),
            );
            let _ = filter.fuse_gnss_velocity(
                now,
                Velocity::ned(nan, 0.0, 0.0),
                VelocityNoise::from_speed_accuracy(0.3),
                Position::zero(),
            );
            let _ = filter.fuse_baro_altitude(
                now,
                Altitude::from_meters(nan),
                AltitudeNoise::from_sigma(2.0),
            );
            let heading = HeadingNoise::from_variance(nan);
            let _ = filter.fuse_mag_heading(now, MagField::body(0.2, 0.0, 0.4), heading);
            let _ = filter.fuse_gnss_heading(now, Radians::from_radians(0.0), heading);
            let _ = filter.fuse_course(now, heading);
            let _ = filter.fuse_stationary(now, VelocityNoise::from_speed_accuracy(nan));
        });
        // The hold offers nothing; the filter makes it, and `hold.rs` times it. Nor does the
        // yaw estimator, which weighs the velocities the filter screened: `yaw.rs` times it.
        let offered = filter.diagnostics().sources();
        let offered = offered
            .iter()
            .filter(|&&(name, _)| !matches!(name, "position_hold" | "yaw_estimator"));
        for &(name, source) in offered {
            assert_eq!(source.accepted, 0, "{name}");
            let period = source.period().map(Seconds::as_secs);
            assert!(
                period.is_some_and(|period| (period - 0.2).abs() < 1e-4),
                "{name}: {period:?}"
            );
        }
    }

    #[test]
    fn every_source_refuses_a_variance_no_sensor_could_have() {
        let mut filter = initialized();
        assert!(filter.set_baro_reference(
            Altitude::from_meters(52.0),
            AltitudeNoise::from_sigma(0.001)
        ));
        assert_eq!(
            filter.fuse_gnss_position(
                filter.now(),
                Position::ned(1.0, 2.0, 3.0),
                PositionNoise::<Ned>::from_variance(0.0, 1.0, -1.0),
                Position::zero(),
            ),
            GnssFusion::both(Fusion::InvalidNoise),
            "zero variance claims a perfect measurement and makes S singular"
        );
        assert_eq!(
            filter.fuse_gnss_velocity(
                filter.now(),
                Velocity::ned(0.0, 0.0, 0.0),
                VelocityNoise::<Ned>::from_variance(1.0, -4.0, 1.0),
                Position::zero(),
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(60.0),
                AltitudeNoise::from_variance(0.0)
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_mag_heading(
                filter.now(),
                MagField::body(0.2, 0.0, 0.4),
                HeadingNoise::from_variance(-1.0)
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_stationary(
                filter.now(),
                VelocityNoise::<Ned>::from_variance(1.0, 1.0, 0.0)
            ),
            Fusion::InvalidNoise
        );
        assert_eq!(
            filter.fuse_gnss_geodetic(
                filter.now(),
                zurich(),
                PositionNoise::<Ned>::from_variance(1.0, 1.0, 0.0),
                Position::zero()
            ),
            GnssFusion::both(Fusion::InvalidNoise)
        );
        assert_eq!(filter.origin(), None, "and no origin was placed on the way");

        for (name, source) in filter.diagnostics().sources() {
            assert_eq!(source.accepted, 0, "{name} counted a refused measurement");
            assert_eq!(source.time_since_accepted, None, "{name} started aiding");
        }
    }
}
