//! Holding tilt without aiding: the standstill a caller asserts, (29″), and the position hold
//! the filter fuses itself while nothing aids it, (28″).
//!
//! Entry point: [`Eskf::fuse_stationary`]. The hold has none; `predict` runs it.

use nalgebra::Vector3;

use crate::config::Hold;
use crate::frames::Ned;
use crate::health::Fusion;
use crate::math::correlation_inflation;
use crate::observation::hold;
use crate::units::{Radians, Seconds, Timestamp, VelocityNoise};
use crate::update::update;

use super::Eskf;
use super::fuse::refuse;

/// How often the hold is fused while it lasts: PX4's 5 Hz
/// (`fake_pos_control.cpp:47-48` at `c4e4ef98`).
///
/// A rate rather than every step, because each fusion is a reading of the same assumption, and
/// the update counts it as independent evidence: at the IMU's rate the hold would average its
/// own `σ` down hundreds of times faster than at a sensor's.
const HOLD_INTERVAL: Seconds = Seconds::from_secs(0.2);

/// The tilt σ, per navigation axis, above which an unaided filter holds its position: PX4's 3°
/// (`fake_pos_control.cpp:79-82` at `c4e4ef98`).
///
/// A hold is an assumption, and a filter whose tilt is still good has nothing to gain from it
/// and a moving vehicle to misread. Held from the first unaided step instead, `gnss_outage`'s
/// 20 s gap over the circuit's fastest turns read 3092 epochs of tilt claimed valid and wrong,
/// tilt error 0.340° → 2.68° RMS; behind this bar it holds 5 times and reads 0.344°
/// ([measured]).
///
/// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#hold
const HOLD_TILT: Radians = Radians::from_degrees(3.0);

/// The correlation time the hold is fused at, equation (24′): one assumption read every 0.2 s is
/// one error, not independent ones.
///
/// Fused as white, as PX4 and ArduPilot fuse theirs, five readings a second average `S` down
/// around an error that persists for the whole outage, and the covariance follows them:
/// `hover_outage`'s ensemble read `anees_pos` 61 and `anees_vel` 0.97, every block past its bound
/// and attitude too where GNSS returns. At 2 s every block passes, and the estimate it leaves is
/// better, not only more honest: `pos_h` 38 m to 7.8 against 36 with no hold. 1 s still fails
/// (`anees_pos` 46); longer weakens the hold toward none ([measured]).
///
/// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#hold
const HOLD_TAU: Seconds = Seconds::from_secs(2.0);

/// The position hold's state: where it holds, and when it last fused.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Anchor {
    /// The position estimate when the hold engaged, NED meters about the origin.
    position: Vector3<f32>,
    fused: Timestamp,
}

impl Eskf {
    /// Fuse the caller's claim that the vehicle is still: zero velocity on all three axes.
    /// Equation (29″).
    ///
    /// Without horizontal aiding nothing observes tilt, so a vehicle on the bench with no GNSS
    /// loses its tilt on the schedule [`ImuNoise`](crate::ImuNoise) sets: [`Validity::tilt`]
    /// goes false about 10 s into an unaided start. A zero velocity is what holds it: tilt
    /// error times gravity lands in velocity at once, and the update takes it back out. PX4 and
    /// ArduPilot hold the attitude on the ground the same way.
    ///
    /// Zero velocity rather than PX4's constant position (`EKF2_POS_LOCK`): what the caller
    /// knows is that the vehicle is not moving, which needs no anchor and observes tilt one
    /// integration nearer than a position does. ArduPilot fuses zero velocity for the same case.
    ///
    /// The filter cannot know the vehicle is still; the caller asserts it, as it asserts the
    /// static window's stillness, from a landed detector, a disarmed state or its own test. A
    /// claim made while the vehicle moves reads its acceleration as tilt, and the gate is what
    /// stands between that and the estimate.
    ///
    /// `noise` is how still: the velocity the vibration and the claim's own error leave. Upstream
    /// chooses far apart, ArduPilot 1 m/s for its synthetic zero velocity
    /// (`AP_NavEKF3_PosVelFusion.cpp:769-775` at `368dc0c4`), PX4 0.01 m on the position it holds
    /// at rest (`fake_pos_control.cpp:57-60` at `c4e4ef98`). A tighter σ levels faster and lets
    /// less vibration through as tilt.
    ///
    /// A claim inconsistent with the estimate at [`Gates::stationary`](crate::Gates) is
    /// [`Fusion::Rejected`] and changes nothing. It is never adopted, whatever
    /// [`Config::recovery`](crate::Config::recovery) says: a standstill the gate keeps turning down
    /// is a caller wrong about the vehicle, and adopting it would write that into the state. Nor
    /// does it establish velocity after a coarse start, which only a sensor does. No
    /// [`Config::correlation`](crate::Config::correlation) either: its error is vibration, not
    /// a receiver's error persisting from one solution to the next.
    ///
    /// Not a source [`Status`](crate::Status) counts: it says where the vehicle is not going,
    /// not where it is, so a bench with no GNSS stays `DeadReckoning` while its tilt holds.
    ///
    /// [`Validity::tilt`]: crate::Validity::tilt
    pub fn fuse_stationary(&mut self, time: Timestamp, noise: VelocityNoise<Ned>) -> Fusion {
        if let Err(refusal) = self.admit(time) {
            return refuse(&mut self.diagnostics.stationary, refusal);
        }
        self.diagnostics.stationary.note_arrival(time);
        if !noise.is_finite() {
            return refuse(&mut self.diagnostics.stationary, Fusion::NotFinite);
        }
        if !noise.is_positive() {
            return refuse(&mut self.diagnostics.stationary, Fusion::InvalidNoise);
        }
        let variance = Vector3::from(noise.variance());
        let observation = self.observe(time, |past, _| hold::zero_velocity(past, variance));
        let outcome = update(
            self.estimate.state(),
            &self.covariance,
            &self.offset,
            &observation,
            self.config.gates.stationary,
        );
        // With no timeout `apply_or_recover` is `apply`, out of line, and never adopts.
        self.apply_or_recover(outcome, |d| &mut d.stationary, None, |_| false)
    }

    /// Fuse the position hold if the filter is unaided and one is due. Equation (28″).
    ///
    /// Called by [`predict`](Self::predict) after each committed step, since nothing else is
    /// guaranteed to run while nothing aids the filter. Unaided is the condition
    /// [`Status::DeadReckoning`](crate::Status::DeadReckoning) reads, no horizontal source
    /// accepted within [`Timeouts::dead_reckoning_after`](crate::Timeouts), and no standstill
    /// either: a caller's claim that the vehicle is still is the better constraint, and two at
    /// once would count the vehicle's stillness twice. The anchor is the estimate at the first
    /// step unaided, a few seconds of dead reckoning past where aiding stopped; PX4 keeps the last
    /// aided position instead, which costs a write on every accepted fix.
    ///
    /// Out of line so that `predict`'s own frame does not grow by an update's: the update's
    /// frame then sits beside the propagation's rather than above it.
    #[inline(never)]
    pub(super) fn hold_if_unaided(&mut self) {
        let Some(Hold { sigma }) = self.config.hold else {
            return;
        };
        if !self.is_unaided() {
            self.anchor = None;
            return;
        }
        // Released as PX4 releases it, so the next engagement anchors afresh where the
        // estimate has got to.
        if self.tilt_within(&self.covariance, HOLD_TILT) {
            self.anchor = None;
            return;
        }
        let position = match self.anchor {
            Some(anchor) if self.time.since(anchor.fused) < HOLD_INTERVAL => return,
            Some(anchor) => anchor.position,
            None => self.estimate.state().position.vector(),
        };
        self.anchor = Some(Anchor {
            position,
            fused: self.time,
        });
        self.diagnostics.position_hold.note_arrival(self.time);
        let variance = sigma.as_meters() * sigma.as_meters();
        let observation = hold::position_hold(self.estimate.state(), position, variance)
            .correlated(correlation_inflation(
                self.diagnostics.position_hold.since_measured,
                Some(HOLD_TAU),
            ));
        let outcome = update(
            self.estimate.state(),
            &self.covariance,
            &self.offset,
            &observation,
            self.config.gates.position_hold,
        );
        // Through `apply_or_recover`, out of line, so the `Update` is not copied into this frame;
        // with no timeout it never adopts.
        let _ = self.apply_or_recover(outcome, |d| &mut d.position_hold, None, |_| false);
    }

    /// Whether nothing holds horizontal position or velocity: no horizontal measurement judged,
    /// and no standstill accepted, within [`Timeouts::dead_reckoning_after`](crate::Timeouts).
    ///
    /// Judged rather than accepted, as PX4 starts its fake position only once GNSS fusion has
    /// stopped rather than while its fixes fail the gate: a receiver the gate keeps turning down
    /// is [`Recovery`](crate::Recovery)'s, and a hold engaged under it would adopt the next fix at
    /// once. On `7ce66f0d`, whose fixes are rejected for seconds at a time, holding on acceptance
    /// took recoveries from 27 to 62 ([measured]).
    ///
    /// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#hold
    fn is_unaided(&self) -> bool {
        let after = self.config.timeouts.dead_reckoning_after;
        let d = &self.diagnostics;
        !d.horizontal().any(|source| source.judged_within(after))
            && !d.stationary.accepted_within(after)
    }

    /// Whether the position hold holds the estimate now: engaged, and still unaided. What
    /// [`validity`](Self::validity) reads to keep horizontal position and velocity invalid
    /// under a covariance the hold, not a sensor, has bounded.
    pub(super) fn is_held(&self) -> bool {
        self.anchor.is_some() && self.is_unaided()
    }

    /// The lockout timeout a horizontal source recovers after: none at all while the hold is
    /// engaged, `after` otherwise, and never where `after` turns recovery off for the source.
    ///
    /// The hold's covariance describes the assumption, not the vehicle, so a fix it turns down
    /// is judged against a position nobody measured; waiting out [`Config::recovery`] would hold
    /// a vehicle away from its first fix for seconds. PX4 resets to a GNSS position that fails
    /// its test when fusion starts (`gps_control.cpp:224-245` at `c4e4ef98`). Read on the
    /// anchor rather than on [`is_held`](Self::is_held), because the fix asking is the one
    /// about to end the hold.
    ///
    /// [`Config::recovery`]: crate::Config::recovery
    pub(super) fn recovery_after(&self, after: Option<Seconds>) -> Option<Seconds> {
        if self.anchor.is_some() {
            after.map(|_| Seconds::ZERO)
        } else {
            after
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::fixtures::*;
    use crate::config::{Config, Recovery};
    use crate::eskf::Eskf;
    use crate::health::{Fusion, Propagation, Refusal, Status};
    use crate::init::tests::still;
    use crate::units::{Position, Seconds, VelocityNoise};

    fn fix_at_origin(filter: &mut Eskf) -> crate::GnssFusion {
        filter.fuse_gnss_position(
            filter.now(),
            Position::zero(),
            one_metre(),
            Position::zero(),
        )
    }

    fn holds(filter: &Eskf) -> u32 {
        let hold = filter.diagnostics().position_hold;
        hold.accepted + hold.rejected
    }

    fn standstill() -> VelocityNoise<crate::frames::Ned> {
        VelocityNoise::from_speed_accuracy(0.1)
    }

    /// The control and the claim from one start: an unaided bench loses tilt about 5 s in from
    /// a window whose gyroscope never scattered, and the same bench told it is still keeps it.
    #[test]
    fn a_standstill_holds_tilt_on_a_bench_with_no_aiding() {
        let (mut told, mut unaided) = (initialized(), unheld());
        let mut claims = 0;
        hold(&mut told, 30.0, 10, |filter| {
            let outcome = filter.fuse_stationary(filter.now(), standstill());
            assert!(outcome.is_accepted(), "{outcome:?}");
            claims += 1;
        });
        hold(&mut unaided, 30.0, 10, |_| {});
        assert_eq!(claims, 300);
        assert!(told.validity().tilt);
        assert!(!unaided.validity().tilt);
        // Nothing aids it: a claim is not a sensor.
        assert_eq!(told.state().status, Status::DeadReckoning);
    }

    /// Survives `apply_or_recover` in place of `apply`: the default recovery adopts a horizontal
    /// source after 7 s of rejections, and these run for 8.
    #[test]
    fn a_standstill_the_gate_turns_down_is_never_adopted() {
        // The hold would pull an unaided 20 m/s toward zero and let a claim in.
        let mut filter = flying(Config {
            hold: None,
            ..Config::default()
        });
        let velocity = filter.state().velocity;
        let tight = VelocityNoise::from_speed_accuracy(0.01);
        let dt = Seconds::from_secs(0.1);
        for _ in 0..80 {
            let _ = filter.step(still().imu, dt);
            let outcome = filter.fuse_stationary(filter.now(), tight);
            assert!(matches!(outcome, Fusion::Rejected { .. }), "{outcome:?}");
        }
        let health = filter.diagnostics().stationary;
        assert_eq!(
            (health.rejected, health.adopted, health.recovered),
            (80, 0, 0)
        );
        // Ten seconds of coasting moved it a little; a claim adopted would have stopped it.
        assert!(filter.state().velocity.to_array()[0] > velocity.to_array()[0] - 1.0);
    }

    /// The filter [`initialized`] starts, stepped unaided until the hold first fuses: once its
    /// tilt σ has passed [`HOLD_TILT`](super::HOLD_TILT), a few seconds in.
    fn engaged() -> Eskf {
        let mut filter = initialized();
        for _ in 0..2000 {
            if holds(&filter) > 0 {
                return filter;
            }
            assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
        }
        panic!("no hold in 20 s unaided");
    }

    /// The hold's own control: the same unaided start, with and without it, a minute in. The
    /// vehicle is level throughout, so the estimate's tilt is its error.
    #[test]
    fn the_hold_bounds_tilt_through_an_unaided_minute_and_claims_nothing_else() {
        let (mut held, mut unaided) = (initialized(), unheld());
        hold(&mut held, 60.0, 1, |_| {});
        hold(&mut unaided, 60.0, 1, |_| {});
        let sigma = |filter: &Eskf| {
            let v = filter.attitude_variance();
            v.tilt_north.max(v.tilt_east).sqrt()
        };
        assert!(
            sigma(&held) < 0.5 * sigma(&unaided),
            "held {} rad against {} unaided",
            sigma(&held),
            sigma(&unaided)
        );
        // Bounded honestly: the level truth sits inside the σ it claims.
        let (roll, pitch, _) = held.state().attitude.euler_angles();
        assert!(roll.abs().max(pitch.abs()) < 3.0 * sigma(&held));
        assert_eq!(held.diagnostics().position_hold.rejected, 0);
        let validity = held.validity();
        assert!(!validity.horizontal_position && !validity.horizontal_velocity);
        assert_eq!(held.state().status, Status::DeadReckoning);
    }

    /// Survives the gate removed: the tilt σ just after a start is well under 3°, so an ungated
    /// hold fuses on the first step.
    #[test]
    fn a_hold_waits_for_the_tilt_to_need_it() {
        let mut filter = initialized();
        hold(&mut filter, 1.0, 1, |_| {});
        assert_eq!(holds(&filter), 0);
        assert!(filter.tilt_within(filter.covariance(), super::HOLD_TILT));
        // And once the σ has grown past it, the hold fuses: `engaged` panics otherwise.
        let _ = engaged();
    }

    #[test]
    fn the_hold_waits_out_the_dead_reckoning_timeout_after_the_last_fix() {
        let mut filter = engaged();
        hold(&mut filter, 2.0, 20, |filter| {
            assert!(fix_at_origin(filter).horizontal.is_accepted());
        });
        let before = holds(&filter);
        // Inside the 5 s still aided; past it, held. Neither lands on the boundary.
        hold(&mut filter, 4.9, 1, |_| {});
        assert_eq!(holds(&filter), before);
        hold(&mut filter, 0.2, 1, |_| {});
        assert_eq!(holds(&filter), before + 1);
    }

    /// Survives the condition read on acceptance instead: a fix the gate turns down is still a
    /// receiver speaking, and holding through it would hand every rejection to the hold.
    #[test]
    fn a_receiver_the_gate_turns_down_keeps_the_hold_off() {
        let mut filter = engaged();
        let far = Position::ned(500.0, 0.0, 0.0);
        // The first is adopted, the hold being engaged; the rest are rejected for 6 s.
        let _ = filter.fuse_gnss_position(filter.now(), far, one_metre(), Position::zero());
        let before = holds(&filter);
        let elsewhere = Position::ned(-500.0, 0.0, 0.0);
        hold(&mut filter, 6.0, 20, |filter| {
            let outcome =
                filter.fuse_gnss_position(filter.now(), elsewhere, one_metre(), Position::zero());
            assert!(
                matches!(outcome.horizontal, Fusion::Rejected { .. }),
                "{outcome:?}"
            );
        });
        assert_eq!(holds(&filter), before);
    }

    #[test]
    fn an_accepted_fix_ends_the_hold_and_the_next_outage_anchors_afresh() {
        let mut filter = engaged();
        assert!(!filter.validity().horizontal_position);
        assert!(fix_at_origin(&mut filter).horizontal.is_accepted());
        // Over at once, before the step that clears the anchor.
        assert!(filter.validity().horizontal_position);
        let before = holds(&filter);
        hold(&mut filter, 1.0, 20, |filter| {
            assert!(fix_at_origin(filter).horizontal.is_accepted());
        });
        assert_eq!(holds(&filter), before);
        assert_eq!(filter.anchor, None);
    }

    /// Each filter is the control for the one before: the hold's own adoption, the lockout
    /// timeout it skips, and the switch that turns both off.
    #[test]
    fn the_first_fix_after_a_hold_that_the_gate_turns_down_is_adopted_at_once() {
        let far = Position::ned(500.0, 0.0, 0.0);
        let mut filter = engaged();
        let outcome = filter.fuse_gnss_position(filter.now(), far, one_metre(), Position::zero());
        assert_eq!(outcome.horizontal, Fusion::Reset);
        assert_eq!(filter.diagnostics().gnss_position.recovered, 1);
        assert!(filter.validity().horizontal_position);

        // Without the hold the same fix waits out `Recovery::gnss_position`.
        let mut filter = unheld();
        hold(&mut filter, 5.0, 1, |_| {});
        let outcome = filter.fuse_gnss_position(filter.now(), far, one_metre(), Position::zero());
        assert!(matches!(outcome.horizontal, Fusion::Rejected { .. }));

        // And a source whose recovery is off is never adopted, hold or not.
        let mut filter = Eskf::new(Config {
            recovery: Recovery::OFF,
            ..Config::default()
        })
        .unwrap();
        let _ = filter.initialize_over(&[still(); 8], Seconds::from_secs(0.25));
        hold(&mut filter, 20.0, 1, |_| {});
        assert!(holds(&filter) > 0);
        let outcome = filter.fuse_gnss_position(filter.now(), far, one_metre(), Position::zero());
        assert!(matches!(outcome.horizontal, Fusion::Rejected { .. }));
    }

    #[test]
    fn a_standstill_stands_the_hold_down() {
        let mut filter = engaged();
        let before = holds(&filter);
        hold(&mut filter, 10.0, 10, |filter| {
            let _ = filter.fuse_stationary(filter.now(), standstill());
        });
        // The steps before the first claim are the only ones that could hold.
        assert!(holds(&filter) <= before + 1);
        assert_eq!(filter.anchor, None);
    }

    #[test]
    fn a_standstill_with_no_usable_noise_is_refused() {
        let mut filter = initialized();
        let now = filter.now();
        let nan = VelocityNoise::from_speed_accuracy(f32::NAN);
        assert_eq!(filter.fuse_stationary(now, nan), Fusion::NotFinite);
        let zero = VelocityNoise::from_speed_accuracy(0.0);
        assert_eq!(filter.fuse_stationary(now, zero), Fusion::InvalidNoise);
        let health = filter.diagnostics().stationary;
        assert_eq!(health.refused, 2);
        assert_eq!(health.last_refusal, Some(Refusal::InvalidNoise));
    }
}
