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
    /// covariance says. Attitude is not yet good enough to fly on. The filter leaves this
    /// state on its own, as soon as the covariance says tilt and heading uncertainty are
    /// within [`Config::accuracy`](crate::Config::accuracy) — there is no timer and
    /// nothing to acknowledge.
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
    /// This happens once per quantity, on the first GNSS position fix and the first GNSS
    /// velocity after a coarse start: a vehicle that initialized while moving knows
    /// neither where it is nor how fast it is going, and no gate can judge a measurement
    /// against nothing. A static start and a seed both have an estimate, so their first
    /// fix is fused normally. The state becomes the measurement and its
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
    /// No initialization has succeeded — [`Eskf::initialize`](crate::Eskf::initialize),
    /// `initialize_coarse` or `initialize_from` — so there is no state to fuse against. The measurement was discarded.
    NotInitialized,
    /// The measurement has nothing to be relative to. The measurement was discarded.
    ///
    /// Barometer: no reference altitude was established, because the static window
    /// carried no barometer sample. `α₀` of equation (30) is a constant fixed at
    /// initialization, not a state, so a barometric altitude without one has no origin to
    /// be relative to. Fusing it anyway would silently invent the origin from whichever
    /// sample happened to arrive first.
    ///
    /// Geodetic GNSS: no navigation origin is held and this fix cannot place one, because
    /// a coordinate is not a number or it sits on a pole. The next usable fix will. See
    /// [`Eskf::fuse_gnss_geodetic`](crate::Eskf::fuse_gnss_geodetic).
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
    /// No initialization has succeeded; see [`Fusion::NotInitialized`]. Nothing was
    /// propagated and no timer advanced.
    NotInitialized,
}

impl Propagation {
    /// Whether the state advanced.
    pub const fn is_propagated(self) -> bool {
        matches!(self, Self::Propagated)
    }
}

/// Which parts of the estimate are good enough to use.
///
/// The single [`Status`] answers *how bad is the worst thing*; this answers *which
/// outputs can I use*, which is the question a controller actually has. They are
/// independent axes, and collapsing them loses real information: a filter that started
/// coarse and has adopted a GNSS fix has position as good as the receiver while its
/// attitude is still converging, and `Status::Aligning` alone cannot say so.
///
/// Every flag is derived from the covariance against
/// [`Config::accuracy`](crate::Config::accuracy), plus the requirement that the quantity
/// was ever established at all — a coarse start has no position until a fix arrives, and
/// a tight prior on a number nobody set is not validity.
///
/// Horizontal and vertical are separate because sources are: a vehicle with a barometer
/// and no GNSS has a usable height and no horizontal position at all, which describes two
/// of the five logs in the replay corpus.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Validity {
    /// Roll and pitch.
    pub tilt: bool,
    /// Heading.
    pub heading: bool,
    /// North and east position.
    pub horizontal_position: bool,
    /// Down position.
    pub vertical_position: bool,
    /// North and east velocity.
    pub horizontal_velocity: bool,
    /// Down velocity.
    pub vertical_velocity: bool,
}

impl Validity {
    /// Nothing is valid. What an uninitialized filter reports.
    pub const NONE: Self = Self {
        tilt: false,
        heading: false,
        horizontal_position: false,
        vertical_position: false,
        horizontal_velocity: false,
        vertical_velocity: false,
    };

    /// Whether every part of the estimate is usable.
    pub const fn all(self) -> bool {
        self.tilt
            && self.heading
            && self.horizontal_position
            && self.vertical_position
            && self.horizontal_velocity
            && self.vertical_velocity
    }

    /// Whether attitude is usable: the part a stabilizing controller needs before
    /// anything else.
    pub const fn attitude(self) -> bool {
        self.tilt && self.heading
    }

    /// Whether the full navigation solution — position and velocity, both axes — is
    /// usable.
    pub const fn navigation(self) -> bool {
        self.horizontal_position
            && self.vertical_position
            && self.horizontal_velocity
            && self.vertical_velocity
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

    /// Whether a measurement from this source was accepted no more than `timeout` ago.
    /// Never, for a source that has not been accepted at all.
    pub fn accepted_within(&self, timeout: Seconds) -> bool {
        self.time_since_accepted
            .is_some_and(|elapsed| elapsed <= timeout)
    }

    /// Advance the fusion clock. Called from `predict`, since the filter has no clock.
    pub(crate) fn advance(&mut self, dt: Seconds) {
        if let Some(elapsed) = self.time_since_accepted {
            self.time_since_accepted = Some(Seconds::from_secs(elapsed.as_secs() + dt.as_secs()));
        }
    }

    /// Record a measurement that passed the gate: its test ratio, a restarted fusion
    /// clock, and the end of any run of rejections.
    pub(crate) fn record_accepted(&mut self, test_ratio: f32) {
        self.test_ratio = Some(test_ratio);
        self.time_since_accepted = Some(Seconds::ZERO);
        self.consecutive_rejections = 0;
        self.accepted = self.accepted.saturating_add(1);
    }

    /// Record a measurement the gate refused. The fusion clock keeps running, which is
    /// what lets a source that is only ever rejected time out.
    #[allow(dead_code, reason = "used once gating is implemented")]
    pub(crate) fn record_rejected(&mut self, test_ratio: f32) {
        self.test_ratio = Some(test_ratio);
        self.consecutive_rejections = self.consecutive_rejections.saturating_add(1);
        self.rejected = self.rejected.saturating_add(1);
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

    /// Advance every source's fusion clock.
    pub(crate) fn advance(&mut self, dt: Seconds) {
        self.gnss_position.advance(dt);
        self.gnss_velocity.advance(dt);
        self.baro_altitude.advance(dt);
        self.mag_heading.advance(dt);
    }
}
