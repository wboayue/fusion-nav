//! The recent past of the nominal state, for fusing a measurement at the time it was taken.
//! Equation (23′).

use nalgebra::{UnitQuaternion, Vector3};

use crate::config::LATENCY_HORIZON;
use crate::state::State;
use crate::units::{Attitude, Position, Timestamp, Velocity};

/// Entries held: enough that [`LATENCY_HORIZON`] is covered at [`SPACING`] with one to spare
/// at each end.
const CAPACITY: usize = 32;

/// The least time between two entries, in microseconds: 10 ms. An IMU faster than 100 Hz is
/// recorded every few samples, and the interpolation between them costs little: a 5 m/s²
/// manoeuvre departs from a straight line by `a Δt² / 8`, 60 µm at 10 ms.
///
/// Compared in integer microseconds, because `f32` puts 0.3 s / 30 a hair above the 10 000 µs
/// a 400 Hz IMU's fourth sample reaches, and the spacing would come out at 12.5 ms instead.
const SPACING: u64 = (LATENCY_HORIZON.as_secs() * 1.0e6) as u64 / (CAPACITY as u64 - 2);

#[derive(Clone, Copy, Debug, PartialEq)]
struct Entry {
    time: Timestamp,
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    attitude: UnitQuaternion<f32>,
}

impl Default for Entry {
    fn default() -> Self {
        Self {
            time: Timestamp::ZERO,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            attitude: UnitQuaternion::identity(),
        }
    }
}

/// A ring of past positions, velocities and attitudes, newest last.
///
/// The state as it stood rather than a quantity integrated from the IMU since: a velocity a
/// measurement's age ago is the current velocity less the acceleration *integrated* over that
/// age, and one sample's specific force is no estimate of that integral on a vibrating
/// airframe. Extrapolating back on the last sample instead took `2c42096b`, a vehicle sitting
/// on the ground, from 0 GNSS rejections to 94.
///
/// Kept consistent with every correction: an update moves the estimate of the past with the
/// present, so [`shift`](Self::shift) applies each one to every entry. Without it a fix taken
/// before the previous fix was fused innovates against a past that fix never corrected, and
/// the same error is corrected twice.
#[derive(Clone, Debug)]
pub(crate) struct History {
    entries: [Entry; CAPACITY],
    /// Entries held, at most [`CAPACITY`].
    len: usize,
    /// Where the next entry goes.
    next: usize,
}

impl Default for History {
    fn default() -> Self {
        Self {
            entries: [Entry::default(); CAPACITY],
            len: 0,
            next: 0,
        }
    }
}

impl History {
    /// Forget everything: a new start has no past.
    pub(crate) fn clear(&mut self) {
        *self = Self::default();
    }

    /// Record `state` at `time`, if [`SPACING`] has passed since the newest entry.
    pub(crate) fn record(&mut self, time: Timestamp, state: &State) {
        if let Some(newest) = self.newest()
            && time.as_micros().saturating_sub(newest.time.as_micros()) < SPACING
        {
            return;
        }
        let entry = Entry {
            time,
            position: state.position.vector(),
            velocity: state.velocity.vector(),
            attitude: state.attitude.quaternion(),
        };
        if let Some(slot) = self.entries.get_mut(self.next) {
            *slot = entry;
        }
        self.next = (self.next + 1) % CAPACITY;
        self.len = (self.len + 1).min(CAPACITY);
    }

    /// Apply the change from `before` to `after` to every entry: a correction to the present
    /// is a correction to the past it was propagated from.
    ///
    /// Position by `δp` and velocity by `δv`, leaving out the velocity correction's own `δv τ`
    /// in position, a correction of a few cm/s times an age of a few hundred milliseconds.
    /// Attitude by the same rotation in the navigation frame, composed on the left: the local
    /// error of (2) turns against the body as it rotates, (18), so the error that stays put
    /// between then and now is the one expressed in navigation axes. A heading adoption, a
    /// rotation about down, is exact that way at any rate of turn; composed on the right it
    /// would tilt the past of a vehicle that rolled.
    pub(crate) fn shift(&mut self, before: &State, after: &State) {
        let dp = after.position.vector() - before.position.vector();
        let dv = after.velocity.vector() - before.velocity.vector();
        let dq = after.attitude.quaternion() * before.attitude.quaternion().inverse();
        // The ring fills from index zero, so the entries held are the first `len` until it wraps.
        for entry in self.entries.iter_mut().take(self.len) {
            entry.position += dp;
            entry.velocity += dv;
            entry.attitude = dq * entry.attitude;
        }
    }

    /// When the oldest entry was recorded: the furthest back a measurement can be placed.
    pub(crate) fn oldest(&self) -> Option<Timestamp> {
        if self.len == 0 {
            return None;
        }
        let index = (self.next + CAPACITY - self.len) % CAPACITY;
        self.entries.get(index).map(|entry| entry.time)
    }

    /// `current` as it stood at `time`: position, velocity and attitude interpolated from the
    /// history, the biases as they are. Before the oldest entry, the oldest.
    ///
    /// After `now`, which a measurement timed between the last IMU sample and the next can be,
    /// the present carried forward on its velocity: an integrated quantity, where one sample's
    /// specific force is not, and the step is short enough that `½ a t²` is millimetres.
    pub(crate) fn at(&self, time: Timestamp, now: Timestamp, current: &State) -> State {
        let lead = time.since(now).as_secs();
        if lead > 0.0 {
            let velocity = current.velocity.vector();
            return State {
                position: Position::from_vector(current.position.vector() + velocity * lead),
                ..*current
            };
        }
        let present = Entry {
            time: now,
            position: current.position.vector(),
            velocity: current.velocity.vector(),
            attitude: current.attitude.quaternion(),
        };
        // Newest first, the present ahead of them all.
        let mut later = present;
        for age in 0..self.len {
            let index = (self.next + CAPACITY - 1 - age) % CAPACITY;
            let Some(&earlier) = self.entries.get(index) else {
                break;
            };
            if earlier.time <= time {
                return with(current, interpolate(earlier, later, time));
            }
            later = earlier;
        }
        with(current, later)
    }

    fn newest(&self) -> Option<Entry> {
        if self.len == 0 {
            return None;
        }
        self.entries
            .get((self.next + CAPACITY - 1) % CAPACITY)
            .copied()
    }
}

/// The entry at `time`, between `earlier` and `later`: linear in position and velocity, the
/// shorter arc in attitude.
fn interpolate(earlier: Entry, later: Entry, time: Timestamp) -> Entry {
    let span = later.time.since(earlier.time).as_secs();
    if span <= 0.0 {
        return later;
    }
    let s = (time.since(earlier.time).as_secs() / span).clamp(0.0, 1.0);
    Entry {
        time,
        position: earlier.position.lerp(&later.position, s),
        velocity: earlier.velocity.lerp(&later.velocity, s),
        attitude: earlier
            .attitude
            .try_slerp(&later.attitude, s, 1.0e-6)
            .unwrap_or(later.attitude),
    }
}

fn with(current: &State, entry: Entry) -> State {
    State {
        position: Position::from_vector(entry.position),
        velocity: Velocity::from_vector(entry.velocity),
        attitude: Attitude::body_to_ned(entry.attitude),
        ..*current
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::Seconds;

    const DT: Seconds = Seconds::from_secs(0.0025);

    fn moving(north: f32) -> State {
        State {
            position: Position::ned(north, 0.0, 0.0),
            velocity: Velocity::ned(20.0, 0.0, 0.0),
            ..State::default()
        }
    }

    /// 400 Hz of a vehicle at 20 m/s, 0.4 s of it: more than the ring spans.
    fn flown() -> (History, Timestamp, State) {
        let mut history = History::default();
        let mut time = Timestamp::ZERO;
        let mut state = moving(0.0);
        for _ in 0..160 {
            history.record(time, &state);
            time = time.after(DT);
            state = moving(state.position.x() + 20.0 * DT.as_secs());
        }
        (history, time, state)
    }

    #[test]
    fn the_past_of_a_straight_line_is_on_it() {
        let (history, now, state) = flown();
        for age in [0.0, 0.001, 0.0137, 0.15, 0.29] {
            let past = history.at(now.before(Seconds::from_secs(age)), now, &state);
            let expected = state.position.x() - 20.0 * age;
            assert!(
                (past.position.x() - expected).abs() < 1e-3,
                "{age} s: {} against {expected}",
                past.position.x()
            );
        }
    }

    /// Recording is thinned to `SPACING`, so 400 Hz does not overrun the ring's span.
    #[test]
    fn the_ring_spans_the_horizon_at_imu_rate() {
        let (history, now, state) = flown();
        let oldest = history.at(Timestamp::ZERO, now, &state);
        let reached = (state.position.x() - oldest.position.x()) / 20.0;
        assert!(reached >= LATENCY_HORIZON.as_secs(), "{reached} s");
    }

    /// A correction to the present moves the past with it, so the next old measurement is
    /// judged against the corrected past rather than corrected a second time.
    /// A heading adoption turns the past about down, whatever the vehicle did since: here it
    /// rolled a quarter turn between the entry and now, and the entry keeps its roll.
    #[test]
    fn a_turn_about_down_reaches_the_past_about_down() {
        let mut history = History::default();
        let rolled = |angle: f32| State {
            attitude: Attitude::body_to_ned(UnitQuaternion::from_euler_angles(angle, 0.0, 0.0)),
            ..State::default()
        };
        history.record(Timestamp::ZERO, &rolled(0.0));
        let now = Timestamp::from_micros(100_000);
        let before = rolled(core::f32::consts::FRAC_PI_2);
        let turned =
            UnitQuaternion::from_euler_angles(0.0, 0.0, 1.0) * before.attitude.quaternion();
        let after = State {
            attitude: Attitude::body_to_ned(turned),
            ..before
        };
        history.shift(&before, &after);
        let past = history.at(Timestamp::ZERO, now, &after);
        let (roll, pitch, yaw) = past.attitude.euler_angles();
        assert!(roll.abs() < 1e-5 && pitch.abs() < 1e-5, "{roll} {pitch}");
        assert!((yaw - 1.0).abs() < 1e-5, "{yaw}");
    }

    #[test]
    fn a_time_ahead_of_the_present_is_carried_forward_on_velocity() {
        let (history, now, state) = flown();
        let ahead = history.at(now.after(Seconds::from_secs(0.01)), now, &state);
        assert!((ahead.position.x() - (state.position.x() + 0.2)).abs() < 1e-4);
    }

    #[test]
    fn a_correction_reaches_the_past() {
        let (mut history, now, state) = flown();
        let corrected = State {
            position: Position::ned(state.position.x() + 3.0, 0.5, 0.0),
            ..state
        };
        history.shift(&state, &corrected);
        let past = history.at(now.before(Seconds::from_secs(0.1)), now, &corrected);
        assert!((past.position.x() - (state.position.x() - 2.0 + 3.0)).abs() < 1e-3);
        assert!((past.position.y() - 0.5).abs() < 1e-6);
    }
}
