//! Propagation and gate outcomes, per-source health, and the aggregate status.
//!
//! See [gate lockout](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#gate-lockout)
//! for why the filter reports rather than recovers.

use crate::units::Seconds;

/// How far the estimate can be trusted: whether attitude has converged, and how well
/// aided it is.
///
/// Payload-free, so [`State`](crate::State) stays `Copy` and cheap to read on the hot
/// path. The detail is in [`Diagnostics`].
///
/// Declared in order of increasing severity. When more than one applies the most severe
/// is reported, so [`Aligning`](Self::Aligning) hides [`Degraded`](Self::Degraded) — an
/// attitude that has not converged is the larger problem — and
/// [`DeadReckoning`](Self::DeadReckoning) hides everything, since nothing is arriving
/// that could align the filter anyway.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Status {
    /// Every source that has ever been fused is still being accepted, and attitude has
    /// converged.
    Healthy,
    /// At least one source has timed out, but the estimate is still aided.
    Degraded,
    /// The filter is running and aided, but its attitude has not converged: it was
    /// initialized without a static window, or seeded coarsely, and is still learning.
    ///
    /// Position and velocity are being estimated and are usable to the extent the
    /// covariance says. Attitude is not yet what a static alignment would have given, so
    /// a controller should not fly on it. The filter leaves this state on its own, as
    /// soon as the covariance says the attitude uncertainty has come down to what
    /// [`Initialization`](crate::Initialization) asks of a static start — there is no
    /// timer and nothing to acknowledge.
    Aligning,
    /// Nothing is aiding the filter. Position and velocity error grows without bound.
    #[default]
    DeadReckoning,
}

/// The outcome of one measurement update.
///
/// Carries the dimensionless test ratio `r = ε / γ` of equation (38) in both the accepted
/// and rejected cases, so a rejection is diagnosable rather than a bare failure. `r > 1`
/// means rejected, whatever the dimension of the observation.
#[must_use = "a rejected measurement is silently dropped unless the outcome is inspected"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Fusion {
    /// The measurement passed the gate and was fused.
    Accepted {
        /// Test ratio, in `[0, 1]`.
        test_ratio: f32,
    },
    /// The measurement was adopted outright rather than fused, because the filter had no
    /// estimate of that quantity to fuse it against.
    ///
    /// This happens once per quantity, on the first GNSS fix after a coarse start: a
    /// vehicle that initialized while moving does not know its velocity, and no gate can
    /// judge a measurement against nothing. The state becomes the measurement and its
    /// covariance block becomes the measurement's, which is what fusing against an
    /// infinitely uncertain prior converges to — the limit, taken exactly rather than
    /// approached with an invented variance.
    ///
    /// Worth reacting to: position or velocity stepped, which a controller consuming the
    /// estimate may care about.
    Reset,
    /// The measurement failed the gate and was discarded. The state is unchanged.
    Rejected {
        /// Test ratio, greater than 1.
        test_ratio: f32,
    },
    /// [`Eskf::initialize`](crate::Eskf::initialize) has not been called, so there is no
    /// state to fuse against. The measurement was discarded.
    NotInitialized,
    /// Barometer only: no reference altitude was established, because the static window
    /// carried no barometer sample. The measurement was discarded.
    ///
    /// `α₀` of equation (30) is a constant fixed at initialization, not a state, so a
    /// barometric altitude without one has no origin to be relative to. Fusing it anyway
    /// would silently invent the origin from whichever sample happened to arrive first.
    NoReference,
}

impl Fusion {
    /// Whether the filter took the measurement, whether by fusing it or by adopting it
    /// outright. See [`is_reset`](Self::is_reset) to tell those apart.
    pub const fn is_accepted(self) -> bool {
        matches!(self, Self::Accepted { .. } | Self::Reset)
    }

    /// Whether the measurement replaced the estimate rather than correcting it, stepping
    /// the state.
    pub const fn is_reset(self) -> bool {
        matches!(self, Self::Reset)
    }

    /// The test ratio, where the gate ran at all.
    pub const fn test_ratio(self) -> Option<f32> {
        match self {
            Self::Accepted { test_ratio } | Self::Rejected { test_ratio } => Some(test_ratio),
            // A reset ran no gate: there was nothing to be inconsistent with.
            Self::Reset | Self::NotInitialized | Self::NoReference => None,
        }
    }
}

/// The outcome of one propagation step.
///
/// Returned by [`Eskf::predict`](crate::Eskf::predict), which refuses a step longer than
/// [`Config::max_predict_dt`](crate::Config::max_predict_dt) rather than attempting it.
#[must_use = "a refused propagation leaves the state stale unless the outcome is inspected"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Propagation {
    /// The state and covariance advanced over the full `dt`.
    Propagated,
    /// `dt` exceeded [`Config::max_predict_dt`](crate::Config::max_predict_dt). The state
    /// and covariance are unchanged.
    ///
    /// The per-source timers still advanced: the time really did pass, so the aiding
    /// really is that much staler, and [`Status`] must not claim otherwise.
    StepTooLong {
        /// The `dt` offered.
        dt: Seconds,
        /// The configured limit.
        limit: Seconds,
    },
    /// `dt` was zero, negative, or not a number. Nothing was propagated and no timer
    /// advanced.
    ///
    /// Rejected before the bookkeeping rather than after: a negative `dt` would run the
    /// per-source timers backwards and make stale aiding look fresh, and a NaN would
    /// poison them for the rest of the flight. Zero is included because two IMU samples
    /// sharing a timestamp means duplicated data, which is worth knowing about even
    /// though propagating over it would be harmless.
    InvalidStep {
        /// The `dt` offered.
        dt: Seconds,
    },
    /// [`Eskf::initialize`](crate::Eskf::initialize) has not been called. Nothing was
    /// propagated and no timer advanced.
    NotInitialized,
}

impl Propagation {
    /// Whether the state advanced.
    pub const fn is_propagated(self) -> bool {
        matches!(self, Self::Propagated)
    }
}

/// Health of one observation source.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SourceHealth {
    /// Test ratio of the most recent measurement, or `None` if none has been offered.
    pub test_ratio: Option<f32>,
    /// Time since a measurement from this source was last accepted, or `None` if none
    /// ever has been. Advances with the `dt` passed to
    /// [`Eskf::predict`](crate::Eskf::predict).
    pub time_since_accepted: Option<Seconds>,
    /// Rejections since the last acceptance.
    pub consecutive_rejections: u16,
    /// Measurements accepted over the filter's life.
    pub accepted: u32,
    /// Measurements rejected over the filter's life.
    pub rejected: u32,
}

impl SourceHealth {
    /// Whether this source has ever been fused.
    pub const fn has_been_used(&self) -> bool {
        self.accepted > 0
    }
}

/// Per-source health, off the hot path.
///
/// Returned by [`Eskf::diagnostics`](crate::Eskf::diagnostics).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Diagnostics {
    /// GNSS position updates.
    pub gnss_position: SourceHealth,
    /// GNSS velocity updates.
    pub gnss_velocity: SourceHealth,
    /// Barometric altitude updates.
    pub baro_altitude: SourceHealth,
    /// Magnetic heading updates.
    pub mag_heading: SourceHealth,
}

impl Diagnostics {
    /// Every source, for iteration.
    pub const fn sources(&self) -> [(&'static str, SourceHealth); 4] {
        [
            ("gnss_position", self.gnss_position),
            ("gnss_velocity", self.gnss_velocity),
            ("baro_altitude", self.baro_altitude),
            ("mag_heading", self.mag_heading),
        ]
    }
}

impl SourceHealth {
    /// Advance the fusion clock. Called from `predict`, since the filter has no clock.
    pub(crate) fn advance(&mut self, dt: Seconds) {
        if let Some(elapsed) = self.time_since_accepted {
            self.time_since_accepted = Some(Seconds::from_secs(elapsed.as_secs() + dt.as_secs()));
        }
    }

    pub(crate) fn record_accepted(&mut self, test_ratio: f32) {
        self.test_ratio = Some(test_ratio);
        self.time_since_accepted = Some(Seconds::ZERO);
        self.consecutive_rejections = 0;
        self.accepted = self.accepted.saturating_add(1);
    }

    #[allow(dead_code, reason = "used once gating is implemented")]
    pub(crate) fn record_rejected(&mut self, test_ratio: f32) {
        self.test_ratio = Some(test_ratio);
        self.consecutive_rejections = self.consecutive_rejections.saturating_add(1);
        self.rejected = self.rejected.saturating_add(1);
    }
}

impl Diagnostics {
    pub(crate) fn advance(&mut self, dt: Seconds) {
        self.gnss_position.advance(dt);
        self.gnss_velocity.advance(dt);
        self.baro_altitude.advance(dt);
        self.mag_heading.advance(dt);
    }

    pub(crate) const fn as_array(&self) -> [SourceHealth; 4] {
        [
            self.gnss_position,
            self.gnss_velocity,
            self.baro_altitude,
            self.mag_heading,
        ]
    }
}
