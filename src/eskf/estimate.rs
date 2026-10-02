//! The nominal state and its recent past, which move together. Equation (23′).

use crate::frames::Body;
use crate::history::History;
use crate::state::State;
use crate::units::{AngularRate, Timestamp};

/// The nominal state of (1) and the [`History`] of it that (23′) reads, written only through
/// the three methods that keep the two consistent.
///
/// A struct of its own so that the compiler holds the invariant: `Eskf`'s methods live in
/// child modules of `eskf`, which see `Eskf`'s private fields, and none of them sees these.
/// A writer that set the state and skipped the history would leave the next old measurement
/// to correct the same error a second time, which no test of the present state can see.
#[derive(Clone, Debug, Default)]
pub(super) struct Estimate {
    state: State,
    history: History,
}

impl Estimate {
    /// The state as it stands.
    pub(super) const fn state(&self) -> &State {
        &self.state
    }

    /// Store a corrected state, and correct the [`History`] by the same change.
    ///
    /// Every write of the state other than a propagation or a fresh start comes through here:
    /// an update, an adoption, a heading reset, a new origin. The past is only consistent with
    /// the present if each correction reaches both.
    pub(super) fn commit(&mut self, state: State) {
        self.history.shift(&self.state, &state);
        self.state = state;
    }

    /// Store the state a step propagated to `time`, and record it in the [`History`].
    pub(super) fn step(&mut self, time: Timestamp, state: State) {
        self.state = state;
        self.history.record(time, &self.state);
    }

    /// Begin again at `time` from `state`, with no past: a new start has none.
    pub(super) fn restart(&mut self, time: Timestamp, state: State) {
        self.state = state;
        self.history.clear();
        self.history.record(time, &self.state);
    }

    /// The state as it stood at `time`, and the time it was placed at; see [`History::at`].
    pub(super) fn at(
        &self,
        time: Timestamp,
        now: Timestamp,
        omega: AngularRate<Body>,
    ) -> (State, Timestamp) {
        self.history.at(time, now, &self.state, omega)
    }
}
