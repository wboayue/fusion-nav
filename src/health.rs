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
    /// initialized without a static window, seeded coarsely, or started from a window
    /// with no magnetometer in it, and is still learning.
    ///
    /// Position and velocity are being estimated and are usable to the extent the
    /// covariance says. Attitude is not yet good enough to fly on.
    ///
    /// Two ways out, and neither is a timer or an acknowledgement. Tilt and a widened
    /// yaw leave on their own, as soon as the covariance falls within
    /// [`Config::accuracy`](crate::Config::accuracy). A heading nothing has observed
    /// needs a measurement instead: stillness never supplies yaw, so a filter that
    /// started without a magnetometer stays here however small the covariance is, until
    /// [`Eskf::fuse_mag_heading`](crate::Eskf::fuse_mag_heading) accepts one. A vehicle
    /// carrying no magnetometer at all therefore never leaves, and — since this hides
    /// [`Degraded`](Self::Degraded) — its source timeouts stop showing in `Status` and
    /// have to be read from [`Diagnostics`].
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
///
/// Not `#[must_use]`, unlike [`Propagation`] and the `reset_*` outcomes, because acceptance
/// and rejection reach the caller by a second route: [`Diagnostics`] keeps the latest test
/// ratio, the running counts and the timer for every source, so a loop that reads
/// [`Eskf::state`](crate::Eskf::state) each cycle loses nothing by discarding this value.
///
/// The refusals keep their own counters there rather than a lint here:
/// [`SourceHealth::refused`] and [`SourceHealth::last_refusal`] say how often a source was
/// turned away and why, and [`SourceHealth::adopted`] separates a [`Reset`](Self::Reset) from
/// the ordinary acceptance it otherwise looks like. A refusal still moves no timer — it is not
/// aiding, and pretending otherwise would let a stream of NaN hold off
/// [`Status::DeadReckoning`].
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
    /// fix is fused normally, or, given as latitude and longitude, places the origin
    /// under that estimate; see [`Eskf::fuse_gnss_geodetic`](crate::Eskf::fuse_gnss_geodetic).
    /// The state becomes the measurement and its
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
    /// Barometer: no reference altitude was established, because the window carried no
    /// barometer sample or was taken in motion — a start that cannot claim the altitude it
    /// reads is the ground leaves the reference alone, so a filter that never had one has
    /// none. `α₀` of equation (30) is a constant fixed at initialization, not a state, so
    /// a barometric altitude without one has no origin to be relative to. Fusing it anyway
    /// would silently invent the origin from whichever sample happened to arrive first.
    /// [`Eskf::set_baro_reference`](crate::Eskf::set_baro_reference) names one.
    ///
    /// Geodetic GNSS: no navigation origin is held and this fix cannot place one, because
    /// its latitude is beyond ±90°. The next usable fix will. See
    /// [`Eskf::fuse_gnss_geodetic`](crate::Eskf::fuse_gnss_geodetic).
    NoReference,
    /// A number in the measurement or its noise is NaN or infinite. The measurement was
    /// discarded and no health timer moved.
    ///
    /// Refused before anything else, including the adoption a coarse start allows: one
    /// NaN in the state or covariance spreads to every quantity at the next update and
    /// never leaves.
    NotFinite,
    /// A variance in the measurement noise is zero or negative. The measurement was
    /// discarded and no health timer moved.
    ///
    /// `R` has to be a variance some sensor could have. Zero makes the innovation
    /// covariance `S = H P Hᵀ + R` of equation (24) singular as soon as the state it
    /// observes is itself certain, and a negative one is worse: it claims a measurement
    /// better than perfect, and where a coarse start adopts the measurement outright it
    /// writes that negative variance straight into `P`, which
    /// [`Validity`] then reads as an excellent estimate.
    ///
    /// Usually a floor or a unit mistake at the boundary — a receiver reporting `eph = 0`
    /// while it has no fix, or a variance arrived at by subtracting one σ² from another.
    /// Refused alongside [`NotFinite`](Self::NotFinite), before the adoption a coarse
    /// start allows.
    ///
    /// One bad component refuses the whole measurement, so a receiver in 2D-fix mode
    /// reporting a good `eph` with `epv = 0` loses its horizontal aiding too. The filter
    /// reports rather than repairs: PX4 and ArduPilot clamp such a value into range, and a
    /// caller that wants that behavior floors `noise` before the call, where the policy is
    /// visible. See [`Eskf::fuse_gnss_position`](crate::Eskf::fuse_gnss_position).
    InvalidNoise,
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

    /// Why the measurement was turned away, or `None` if it reached the gate.
    ///
    /// [`Reset`](Self::Reset) is not a refusal: the measurement was taken, just adopted rather
    /// than fused.
    pub const fn refusal(self) -> Option<Refusal> {
        match self {
            Self::NotInitialized => Some(Refusal::NotInitialized),
            Self::NoReference => Some(Refusal::NoReference),
            Self::NotFinite => Some(Refusal::NotFinite),
            Self::InvalidNoise => Some(Refusal::InvalidNoise),
            Self::Accepted { .. } | Self::Rejected { .. } | Self::Reset => None,
        }
    }

    /// The test ratio, where the gate ran at all.
    pub const fn test_ratio(self) -> Option<f32> {
        match self {
            Self::Accepted { test_ratio } | Self::Rejected { test_ratio } => Some(test_ratio),
            // A reset ran no gate: there was nothing to be inconsistent with.
            Self::Reset
            | Self::NotInitialized
            | Self::NoReference
            | Self::NotFinite
            | Self::InvalidNoise => None,
        }
    }
}

/// Why a measurement was turned away before the gate ran.
///
/// The four cases [`Fusion`] reports that are not a verdict on the measurement's *value*: the
/// filter could not form an innovation to judge it against, or the numbers offered were not
/// ones any sensor could produce. Kept on [`SourceHealth::last_refusal`] so that a count of
/// refusals says which kind, which is the difference between a miswired sensor and a missing
/// reference.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// The filter had not been initialized. See [`Fusion::NotInitialized`].
    NotInitialized,
    /// Nothing to measure against: no barometric reference, or no origin the fix could place.
    /// See [`Fusion::NoReference`].
    NoReference,
    /// A number in the measurement or its noise was NaN or infinite. See
    /// [`Fusion::NotFinite`].
    NotFinite,
    /// A variance in the measurement noise was zero or negative. See
    /// [`Fusion::InvalidNoise`].
    InvalidNoise,
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
/// was ever established at all — a coarse start has no position until a fix arrives, a
/// window with no magnetometer has no heading until one is fused, and a tight prior on a
/// number nobody set is not validity.
///
/// Horizontal and vertical are separate because sources are: a vehicle with a barometer
/// and no GNSS has a usable height and no horizontal position at all, which describes one
/// of the five logs in the replay corpus. Another carries neither source, so it has no
/// position of either kind.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Validity {
    /// Roll and pitch.
    pub tilt: bool,
    /// Heading. False until something observes the rotation about gravity: a
    /// magnetometer in the initialization window, or an accepted
    /// [`Eskf::fuse_mag_heading`](crate::Eskf::fuse_mag_heading). Stillness does not
    /// observe it, so a perfect static alignment on a vehicle with no magnetometer
    /// reports `tilt` and not this.
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
///
/// `#[non_exhaustive]`: read the fields, do not construct one. The set grows as the filter
/// learns to report more, and every addition would otherwise break every caller.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
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
    /// Measurements turned away before the gate ran, over the filter's life.
    ///
    /// Distinct from [`rejected`](Self::rejected), which is the gate's verdict on a
    /// measurement it could judge. A refusal means it could not be judged at all, and a
    /// source with a rising count here is misconfigured or miswired rather than noisy: a
    /// source that only ever refuses reads as `never accepted`, exactly like one that was
    /// never connected.
    pub refused: u32,
    /// Why the most recent refusal happened, or `None` if none has.
    pub last_refusal: Option<Refusal>,
    /// Measurements adopted outright rather than fused, over the filter's life.
    ///
    /// At most one, and only after a coarse start; see [`Fusion::Reset`]. Also counted in
    /// [`accepted`](Self::accepted), because the measurement was taken and the timer restarted
    /// — this is what tells the two apart, since an adoption steps the state and an ordinary
    /// acceptance does not.
    pub adopted: u32,
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

    /// Record a measurement adopted outright. An acceptance for every other purpose, so the
    /// timer restarts with it; see [`Fusion::Reset`].
    pub(crate) fn record_adopted(&mut self) {
        self.record_accepted(0.0);
        self.adopted = self.adopted.saturating_add(1);
    }

    /// Record a measurement the gate refused. The fusion clock keeps running, which is
    /// what lets a source that is only ever rejected time out.
    #[allow(dead_code, reason = "used once gating is implemented")]
    pub(crate) fn record_rejected(&mut self, test_ratio: f32) {
        self.test_ratio = Some(test_ratio);
        self.consecutive_rejections = self.consecutive_rejections.saturating_add(1);
        self.rejected = self.rejected.saturating_add(1);
    }

    /// Record a measurement turned away before the gate.
    ///
    /// No timer moves and no test ratio is recorded: nothing was measured against the state,
    /// so the source is no fresher than it was. That is what keeps a stream of NaN from
    /// holding off [`Status::DeadReckoning`].
    pub(crate) fn record_refused(&mut self, refusal: Refusal) {
        self.refused = self.refused.saturating_add(1);
        self.last_refusal = Some(refusal);
    }
}

/// What propagation refused, and the worst of it.
///
/// Not per source — [`Eskf::predict`](crate::Eskf::predict) is the one path that is not a
/// sensor — and the only record that a step was ever turned away. The state stops advancing
/// while `Status` goes on reporting on the aiding, so without this a filter running on refused
/// steps looks healthy.
///
/// `#[non_exhaustive]`; see [`SourceHealth`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct PropagationHealth {
    /// Steps refused as longer than
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt).
    pub refused_too_long: u32,
    /// Steps refused as zero, negative, or not a number.
    pub refused_invalid: u32,
    /// The longest `dt` refused as too long, or `None` if none has been.
    ///
    /// How far past the limit the worst gap ran, which separates a scheduler that overran by a
    /// millisecond from a logger that dropped a second of data. The filter reads no clock, so
    /// it holds the size of the gap and not when it happened; a caller that needs the moment
    /// timestamps it from [`Propagation::StepTooLong`].
    pub longest_refused: Option<Seconds>,
}

impl PropagationHealth {
    /// Record what one step did, if it was refused.
    pub(crate) fn record(&mut self, outcome: Propagation) {
        match outcome {
            Propagation::StepTooLong { dt, .. } => {
                self.refused_too_long = self.refused_too_long.saturating_add(1);
                if self.longest_refused.is_none_or(|worst| dt > worst) {
                    self.longest_refused = Some(dt);
                }
            }
            Propagation::InvalidStep { .. } => {
                self.refused_invalid = self.refused_invalid.saturating_add(1);
            }
            Propagation::Propagated | Propagation::NotInitialized => {}
        }
    }
}

/// Per-source health and what propagation refused, off the hot path.
///
/// Returned by [`Eskf::diagnostics`](crate::Eskf::diagnostics).
///
/// Every count here describes the filter's life since it last initialized, not since it was
/// constructed: committing a window resets these. Measurements offered before then are refused
/// with [`Fusion::NotInitialized`] and counted, and that count goes with the reset — which is
/// what makes a refusal count afterwards a statement about the flight rather than about the
/// caller's startup order.
///
/// `#[non_exhaustive]`; see [`SourceHealth`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct Diagnostics {
    /// GNSS position updates.
    pub gnss_position: SourceHealth,
    /// GNSS velocity updates.
    pub gnss_velocity: SourceHealth,
    /// Barometric altitude updates.
    pub baro_altitude: SourceHealth,
    /// Magnetic heading updates.
    pub mag_heading: SourceHealth,
    /// What [`Eskf::predict`](crate::Eskf::predict) refused. Not a source, so not in
    /// [`sources`](Self::sources).
    pub propagation: PropagationHealth,
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
