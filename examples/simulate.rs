//! Generate seeded synthetic flights with analytic ground truth, in the replay format.
//!
//! Nothing else in the repository can say how *accurate* the filter is. `data/flight.csv` had no
//! generator and no truth, the PX4 corpus has no truth, and `--reference` gives EKF2's own
//! position and velocity rather than the vehicle's. This writes both halves of a benchmark: a log
//! the replay harness reads, and the trajectory it was generated from, so equations (9)–(42) can
//! be scored against truth as they land (GOALS.md differentiator 6). Scoring is `examples/replay.rs`
//! given a truth file as its third argument; this only produces the data.
//!
//! # Truth is analytic, not integrated
//!
//! Position and the Euler angles are closed-form functions of time; velocity, acceleration, body
//! rate and specific force are their exact derivatives. A simulator that integrated a trajectory
//! would hand the filter its own integration error as truth, and every score would then be
//! measuring two integrators against each other.
//!
//! The attitude is prescribed rather than flown: the Euler angles are declared, not derived from
//! the path, so the vehicle is told where to be and how to point rather than being flown there.
//! What scoring needs is that the inertial and the aiding streams follow from one truth, which
//! they do by construction; what it does not get is a dynamically feasible airframe.
//!
//! # And it shares no code with the filter
//!
//! This file imports nothing from `fusion_nav` — the rotations, the gravity convention and the
//! accelerometer model are written out again here. A shared `C_bn` would cancel its own sign
//! error out of the score, which is the inverse crime in its cheapest form. The noise figures
//! avoid the same trap from the other side: they come from the tables below rather than from
//! [`ImuNoise::default()`], so a consistency statistic is not being handed the answer key. Where
//! the two differ, and in which direction, is recorded on [`IMU`] and [`HARSH_IMU`].
//!
//! [`ImuNoise::default()`]: fusion_nav::ImuNoise
//!
//! # Output
//!
//! Two files per scenario:
//!
//! - `<scenario>.csv` — the replay format `examples/replay.rs` documents, one row per
//!   measurement, sorted by time. Positions are NED metres, so the log needs no origin and agrees
//!   with truth by construction.
//! - `<scenario>.truth.csv` — `t_s,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,roll,pitch,yaw,`
//!   `ba_x,ba_y,ba_z,bg_x,bg_y,bg_z` at the IMU rate: the state a perfect filter would report at
//!   that instant, with the biases actually applied to that sample, walk included.
//!
//! ```text
//! cargo run --example simulate                    # every scenario -> target/sim/
//! cargo run --example simulate -- mission         # one of them
//! cargo run --example simulate -- flight data     # regenerate the checked-in CI log, in place
//! cargo run --example simulate -- mission out --seed 17   # the same flight on another draw
//! ```
//!
//! `--seed` replaces every selected scenario's seed, and is how `data/anees.sh` flies the
//! ensemble ANEES is averaged over (#89). The table's seed is the one `data/scenarios.txt` pins.
//!
//! # Reproducibility, and its boundary
//!
//! The same seed gives byte-identical files: the generator is SplitMix64 with Box–Muller on top,
//! seeded per scenario and split per sensor, so nothing depends on iteration order, on the clock,
//! or on how many rows another sensor asked for.
//!
//! That is a promise about repeated runs, not about hosts: this is a host tool calling the
//! platform's `sin` and `cos`, and it inherits none of the cross-architecture guarantee the
//! filter has. `data/README.md` owns that boundary, and it is why `data/flight.csv` is committed
//! rather than regenerated.
//!
//! # A gap in the log
//!
//! `logging_dropout` is the one scenario with a step longer than `Config::max_predict_dt`: it
//! writes no row of any kind for 1.2 s, as a logger that lost its buffer does, and the filter
//! coasts across it (equation (22′)). Its truth file keeps every epoch, and `examples/replay.rs`
//! scores the epochs it has, the first after the gap included — the coasted state is what the
//! filter published.

use std::env;
use std::error::Error;
use std::f64::consts::{FRAC_PI_2, TAU};
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

/// Standard gravity, m s⁻², down-positive in NED.
///
/// The same value `fusion_nav::GRAVITY` carries. A defined physical constant is not part of the
/// model under test, so agreeing on it is not an inverse crime; agreeing on a rotation would be.
const GRAVITY: f64 = 9.806_65;

/// Magnetic declination, rad, east-positive.
///
/// Written into the log's header, which is where `examples/replay.rs` reads the declination it
/// configures the filter with, so a heading fused from these logs is true heading whatever the
/// constant says.
const DECLINATION: f64 = -0.06;

/// Magnetic inclination, rad, down-positive: mid-latitude, and the dip the corpus logs carry.
const INCLINATION: f64 = 1.107;

/// Total field strength, gauss. Only the direction is fused; the unit follows the replay format.
const FIELD: f64 = 0.49;

/// Window the report measures stillness over, in seconds.
///
/// `Initialization::min_duration` defaults to the same 2 s, which is what makes these figures
/// worth printing: a scenario whose opening window exceeds the stationarity tolerances is one the
/// harness will align coarsely. Reported, never depended on — the simulator has no opinion about
/// how the filter is configured.
const OPENING: f64 = 2.0;

// ---------------------------------------------------------------------------------------------
// Pseudo-random numbers
// ---------------------------------------------------------------------------------------------

/// SplitMix64, written out because a simulator that needs a dependency to be reproducible is one
/// more version to pin. Sequential seeds are fine: mixing them is what the algorithm is for.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform on (0, 1]. Open at zero, because Box–Muller takes its logarithm.
    fn next_f64(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 1.0) / 9_007_199_254_740_992.0
    }
}

/// A Gaussian stream: Box–Muller over SplitMix64, keeping the second of each pair.
struct Noise {
    rng: SplitMix64,
    spare: Option<f64>,
}

impl Noise {
    /// One stream per sensor channel, so the streams are independent of each other.
    ///
    /// Without the split, giving GNSS a faster rate would consume different numbers and shift
    /// every later IMU sample, and two scenarios differing in one sensor would differ in all of
    /// them — which would make a scenario pair useless for attributing a score difference.
    fn new(seed: u64, channel: u64) -> Self {
        Self {
            rng: SplitMix64(seed ^ channel.wrapping_mul(0x9E37_79B9_7F4A_7C15)),
            spare: None,
        }
    }

    fn sample(&mut self) -> f64 {
        if let Some(spare) = self.spare.take() {
            return spare;
        }
        let radius = (-2.0 * self.rng.next_f64().ln()).sqrt();
        let angle = TAU * self.rng.next_f64();
        self.spare = Some(radius * angle.sin());
        radius * angle.cos()
    }

    fn vector(&mut self, sigma: f64) -> [f64; 3] {
        self.vector_with([sigma; 3])
    }

    /// Per-axis σ, for a sensor whose axes are not equally good — GNSS height being the standing
    /// example. Hand-rolling the loop at that one call site instead is how its draw order stops
    /// matching every other sensor's.
    fn vector_with(&mut self, sigma: [f64; 3]) -> [f64; 3] {
        [
            sigma[0] * self.sample(),
            sigma[1] * self.sample(),
            sigma[2] * self.sample(),
        ]
    }
}

/// Stream indices for [`Noise::new`]. Appending one leaves every existing stream alone.
mod channel {
    pub const GYRO: u64 = 1;
    pub const ACCEL: u64 = 2;
    pub const GYRO_WALK: u64 = 3;
    pub const ACCEL_WALK: u64 = 4;
    pub const GNSS_POSITION: u64 = 5;
    pub const GNSS_VELOCITY: u64 = 6;
    pub const BARO: u64 = 7;
    pub const MAG: u64 = 8;
    pub const GNSS_HEADING: u64 = 9;
}

// ---------------------------------------------------------------------------------------------
// The flight, and its exact derivatives
// ---------------------------------------------------------------------------------------------

/// A stretch of log time, `[start, end)`.
///
/// One primitive for three jobs — when a source exists at all, when it drops out, and when it is
/// disturbed — because they are the same question and only the field name differs.
#[derive(Clone, Copy)]
struct Window {
    start: f64,
    end: f64,
}

impl Window {
    /// The whole log.
    const ALWAYS: Self = Self {
        start: 0.0,
        end: f64::INFINITY,
    };

    /// Never: a sensor the vehicle does not carry.
    const NEVER: Self = Self {
        start: f64::INFINITY,
        end: f64::INFINITY,
    };

    fn contains(&self, t: f64) -> bool {
        t >= self.start && t < self.end
    }
}

/// One degree of freedom: `offset + s(u) · (rate · u + amplitude · sin(2π u / period + phase))`,
/// where `u` is time since release and `s` is the trajectory's ramp.
///
/// A drift plus one harmonic covers everything the scenarios need — a hold, a steady turn, a
/// circuit, a climb — and differentiates twice by hand, which is the property that matters here.
#[derive(Clone, Copy)]
struct Wave {
    /// Value before release, and the constant the motion is added to.
    offset: f64,
    /// Linear term, per second: a steady climb, or a steady turn.
    rate: f64,
    amplitude: f64,
    period: f64,
    phase: f64,
}

/// Motionless: the identity element of the scenario table.
const STILL: Wave = Wave {
    offset: 0.0,
    rate: 0.0,
    amplitude: 0.0,
    period: 1.0,
    phase: 0.0,
};

impl Wave {
    /// The moving part and its first two derivatives with respect to `u`.
    fn motion(&self, u: f64) -> (f64, f64, f64) {
        let omega = TAU / self.period;
        let angle = omega * u + self.phase;
        (
            self.rate * u + self.amplitude * angle.sin(),
            self.rate + self.amplitude * omega * angle.cos(),
            -self.amplitude * omega * omega * angle.sin(),
        )
    }
}

/// An analytic flight: three position axes and three Euler angles, released together.
#[derive(Clone, Copy)]
struct Trajectory {
    /// Seconds the vehicle sits still before the motion is released.
    hold: f64,
    /// Seconds the motion is blended in over, once released.
    ramp: f64,
    north: Wave,
    east: Wave,
    down: Wave,
    roll: Wave,
    pitch: Wave,
    yaw: Wave,
    /// A vehicle that points where it goes: yaw is the course of the north and east waves plus
    /// this sideslip, and `yaw` above is ignored. `None` for one whose nose and track are
    /// unrelated, as a multirotor's are.
    sideslip: Option<Wave>,
}

/// The state a perfect estimator would report, and what every sensor below is computed from.
struct Truth {
    position: [f64; 3],
    velocity: [f64; 3],
    acceleration: [f64; 3],
    euler: [f64; 3],
    euler_rate: [f64; 3],
}

impl Trajectory {
    /// Position, velocity, acceleration, attitude and attitude rate at `t`, all closed form.
    fn at(&self, t: f64) -> Truth {
        let u = t - self.hold;
        let (tau, dtau, ddtau) = self.release(u);

        // Chain rule on `g(τ(u))`, twice. `τ` is C², so the acceleration this returns — and the
        // specific force taken from it — has no step at release.
        let axis = |wave: &Wave| {
            let (g, dg, ddg) = wave.motion(tau);
            (wave.offset + g, dg * dtau, ddg * dtau * dtau + dg * ddtau)
        };

        let (pos_n, vel_n, acc_n) = axis(&self.north);
        let (pos_e, vel_e, acc_e) = axis(&self.east);
        let (pos_d, vel_d, acc_d) = axis(&self.down);
        let (roll, roll_rate, _) = axis(&self.roll);
        let (pitch, pitch_rate, _) = axis(&self.pitch);
        let (yaw, yaw_rate) = match &self.sideslip {
            None => {
                let (yaw, yaw_rate, _) = axis(&self.yaw);
                (yaw, yaw_rate)
            }
            Some(sideslip) => {
                // The course of the unwarped waves: velocity is `g′(τ) τ′` on every axis, and
                // `τ′ ≥ 0` is shared, so the direction is `g′`'s and is defined through the hold,
                // where the vehicle sits pointing along the track it is about to fly.
                let (_, n1, n2) = self.north.motion(tau);
                let (_, e1, e2) = self.east.motion(tau);
                let course = e1.atan2(n1);
                let course_rate = (n1 * e2 - e1 * n2) / (n1 * n1 + e1 * e1) * dtau;
                let (beta, beta_rate, _) = axis(sideslip);
                (course + beta, course_rate + beta_rate)
            }
        };

        Truth {
            position: [pos_n, pos_e, pos_d],
            velocity: [vel_n, vel_e, vel_d],
            acceleration: [acc_n, acc_e, acc_d],
            euler: [roll, pitch, yaw],
            euler_rate: [roll_rate, pitch_rate, yaw_rate],
        }
    }

    /// Warped time `τ(u)` and its first two derivatives: the clock the waves are evaluated on.
    ///
    /// The ramp slows *time*, not the amplitude. Scaling the amplitude instead would inflate the
    /// speed rather than ease into it — the `s′·g` term of the product rule adds velocity
    /// proportional to how far along the path the vehicle already is, and on the circuit below
    /// that was worth an extra 14 m s⁻¹ during the ramp, in a scenario whose point is to be an
    /// ordinary mission. Warping time cannot do that: `τ′ ≤ 1`, so the vehicle flies exactly the
    /// path the table describes, entering it slowly.
    ///
    /// `τ′` is the quintic smoothstep, so it leaves the hold with zero first *and* second
    /// derivative and a vehicle that has been sitting still does not jerk into motion with a step
    /// in specific force that no airframe produces — which the stationarity check would read as
    /// the still window ending a sample early. `τ` is its integral, and past the ramp it is
    /// `u − ramp/2`: the half-ramp the easing cost, held constant thereafter.
    fn release(&self, u: f64) -> (f64, f64, f64) {
        if u <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        if self.ramp <= 0.0 {
            return (u, 1.0, 0.0);
        }
        if u >= self.ramp {
            return (u - 0.5 * self.ramp, 1.0, 0.0);
        }
        let x = u / self.ramp;
        (
            self.ramp * x * x * x * x * (2.5 + x * (-3.0 + x)),
            x * x * x * (10.0 + x * (-15.0 + 6.0 * x)),
            30.0 * x * x * (1.0 - x) * (1.0 - x) / self.ramp,
        )
    }
}

// ---------------------------------------------------------------------------------------------
// Frames, and what the inertial sensors read
// ---------------------------------------------------------------------------------------------

/// `C_bn`, body (FRD) to navigation (NED), from ZYX Euler angles.
fn body_to_nav(euler: [f64; 3]) -> [[f64; 3]; 3] {
    let (sr, cr) = euler[0].sin_cos();
    let (sp, cp) = euler[1].sin_cos();
    let (sy, cy) = euler[2].sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// `Cᵀ v` — a navigation-frame vector resolved in body axes.
fn resolve_in_body(c: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        c[0][0] * v[0] + c[1][0] * v[1] + c[2][0] * v[2],
        c[0][1] * v[0] + c[1][1] * v[1] + c[2][1] * v[2],
        c[0][2] * v[0] + c[1][2] * v[1] + c[2][2] * v[2],
    ]
}

/// What the accelerometer reads: `f_b = C_bnᵀ (a_n − g_n)`, with gravity down-positive.
///
/// At rest and level this is `[0, 0, −g]`, which is the sanity check [`Report`] prints and the
/// convention `Eskf::initialize` expects.
fn specific_force(truth: &Truth) -> [f64; 3] {
    let a = truth.acceleration;
    resolve_in_body(body_to_nav(truth.euler), [a[0], a[1], a[2] - GRAVITY])
}

/// What the gyroscope reads: Euler rates mapped onto body axes for the ZYX sequence.
fn body_rate(truth: &Truth) -> [f64; 3] {
    let [roll, pitch, _] = truth.euler;
    let [roll_rate, pitch_rate, yaw_rate] = truth.euler_rate;
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    [
        roll_rate - yaw_rate * sp,
        pitch_rate * cr + yaw_rate * cp * sr,
        -pitch_rate * sr + yaw_rate * cp * cr,
    ]
}

/// The local field in NED, turned about the down axis by `disturbance`.
///
/// A field turned by `δ` is indistinguishable from a vehicle whose yaw is `δ` lower, since
/// `Rz(δ)ᵀ C_bn(ψ) = C_bn(ψ − δ)` and the tilt commutes past it. That is exactly what a nearby
/// current or a ferrous mass does, and what makes the disturbance scenario a test of the heading
/// gate rather than of the magnetometer model.
fn magnetic_field(disturbance: f64) -> [f64; 3] {
    let (sd, cd) = (DECLINATION + disturbance).sin_cos();
    let (si, ci) = INCLINATION.sin_cos();
    [FIELD * ci * cd, FIELD * ci * sd, FIELD * si]
}

// ---------------------------------------------------------------------------------------------
// Sensors: one error table and one model each
// ---------------------------------------------------------------------------------------------

/// IMU errors: white noise, a constant bias, and a bias random walk.
///
/// Densities, as [`ImuNoise`] states them, so the per-sample figures are `white / √dt` and
/// `walk · √dt`. Writing the table this way is what lets a scenario change the IMU rate without
/// changing the physics.
///
/// [`ImuNoise`]: fusion_nav::ImuNoise
#[derive(Clone, Copy)]
struct ImuErrors {
    /// rad s⁻¹/√Hz.
    gyro_white: f64,
    /// m s⁻²/√Hz.
    accel_white: f64,
    /// rad s⁻²/√Hz.
    gyro_walk: f64,
    /// m s⁻³/√Hz.
    accel_walk: f64,
    /// rad s⁻¹, at `t = 0`.
    gyro_bias: [f64; 3],
    /// m s⁻², at `t = 0`.
    accel_bias: [f64; 3],
}

/// A well-isolated airframe carrying a current MEMS IMU: a few times a datasheet, and well below
/// what the filter assumes.
///
/// `ImuNoise::default()`'s white noise is ten times PX4's density, far above datasheet
/// deliberately — its `Q` absorbs vibration, scale-factor error, timing jitter and the coning a
/// first-order propagation drops. None of that is in this simulator, so matching those numbers here would be
/// simulating PX4's modelling allowance rather than an IMU. The consequence for scoring is worth
/// stating plainly: against this table the filter's white noise is two orders of magnitude
/// conservative and its bias walks one, so every scenario should come out *under*-confident,
/// and a consistency statistic that does not is a finding rather than a pass. [`HARSH_IMU`]
/// comes closest on attitude: its accelerometer bias is 0.93 of `Initialization`'s prior on y,
/// which (5) levels in as tilt and (8) carries as the tilt's correlation with the bias.
const IMU: ImuErrors = ImuErrors {
    gyro_white: 2.6e-4,
    accel_white: 2.0e-3,
    gyro_walk: 1.0e-5,
    accel_walk: 1.0e-4,
    // Fixed rather than drawn from the seed, so the same bias is being estimated in every
    // scenario and two scores differ by the scenario rather than by the draw. Small, uneven, and
    // none of them zero: a sign error in one axis has nowhere to hide.
    gyro_bias: [0.002_1, -0.003_4, 0.001_2],
    accel_bias: [0.043, -0.062, 0.027],
};

/// A badly isolated airframe: ten times [`IMU`]'s white noise, twenty times its bias walk, and
/// three times its bias.
///
/// The one scenario whose sensors are nothing like the nominal table, so that a statistic taken
/// across the set is not reading the same IMU seven times. At 200 Hz this is 0.28 m s⁻² and
/// 0.04 rad s⁻¹ per sample — a quadrotor with a hard-mounted flight controller, still a sensor
/// rather than a fault.
///
/// It does **not** make the filter overconfident, and the attempt is what showed why. Exceeding
/// `ImuNoise::default()` would take 4.95 m s⁻² of per-sample accelerometer noise at this rate,
/// because PX4's figures are a process-noise allowance for vibration, coning and timing —
/// things absent from an analytic simulator — rather than a sensor specification. No IMU worth
/// flying reaches them. Its gyroscope-bias walk is the one term that does exceed the filter's,
/// 2.0e-4 against 1.1e-4, which costs `tilt` and `yaw` but leaves `nees_att` under 1, the white
/// noise dominating. So `Q` cannot be made optimistic overall from the sensor side here, and a
/// scenario that makes the covariance genuinely overconfident has to do it through `R`: a
/// receiver reporting an accuracy better than it delivers. Nothing in this table does that yet.
const HARSH_IMU: ImuErrors = ImuErrors {
    gyro_white: 2.6e-3,
    accel_white: 2.0e-2,
    gyro_walk: 2.0e-4,
    accel_walk: 2.0e-3,
    gyro_bias: [0.006_3, -0.010_2, 0.003_6],
    accel_bias: [0.129, -0.186, 0.081],
};

/// What the IMU produced this epoch, and the bias it was produced with.
struct ImuReading {
    gyro: [f64; 3],
    accel: [f64; 3],
    gyro_bias: [f64; 3],
    accel_bias: [f64; 3],
}

/// The IMU model. The only stateful sensor, because the bias walks.
struct Imu {
    sigma_gyro: f64,
    sigma_accel: f64,
    step_gyro: f64,
    step_accel: f64,
    gyro_bias: [f64; 3],
    accel_bias: [f64; 3],
    white_gyro: Noise,
    white_accel: Noise,
    walk_gyro: Noise,
    walk_accel: Noise,
}

impl Imu {
    /// Continuous-time densities become per-sample figures here, and nowhere else.
    fn new(errors: ImuErrors, seed: u64, dt: f64) -> Self {
        Self {
            sigma_gyro: errors.gyro_white / dt.sqrt(),
            sigma_accel: errors.accel_white / dt.sqrt(),
            step_gyro: errors.gyro_walk * dt.sqrt(),
            step_accel: errors.accel_walk * dt.sqrt(),
            gyro_bias: errors.gyro_bias,
            accel_bias: errors.accel_bias,
            white_gyro: Noise::new(seed, channel::GYRO),
            white_accel: Noise::new(seed, channel::ACCEL),
            walk_gyro: Noise::new(seed, channel::GYRO_WALK),
            walk_accel: Noise::new(seed, channel::ACCEL_WALK),
        }
    }

    /// One sample, then walk the bias on.
    ///
    /// The reading carries the bias it was taken with rather than the caller reading the bias
    /// back afterwards, so the truth row cannot be written against the next epoch's bias.
    fn sample(&mut self, truth: &Truth) -> ImuReading {
        let reading = ImuReading {
            gyro: add(
                add(body_rate(truth), self.gyro_bias),
                self.white_gyro.vector(self.sigma_gyro),
            ),
            accel: add(
                add(specific_force(truth), self.accel_bias),
                self.white_accel.vector(self.sigma_accel),
            ),
            gyro_bias: self.gyro_bias,
            accel_bias: self.accel_bias,
        };
        self.gyro_bias = add(self.gyro_bias, self.walk_gyro.vector(self.step_gyro));
        self.accel_bias = add(self.accel_bias, self.walk_accel.vector(self.step_accel));
        reading
    }
}

/// GNSS errors. Position and velocity arrive together, at one rate, as a receiver reports them.
#[derive(Clone, Copy)]
struct GnssErrors {
    period: f64,
    /// m, north and east.
    sigma_horizontal: f64,
    /// m, down: GNSS height is the weaker axis, as the geometry requires.
    sigma_vertical: f64,
    /// m s⁻¹, all three axes.
    sigma_velocity: f64,
    /// s: time constants of the position error, north and east then down, and of the velocity
    /// error; see [`GaussMarkov`].
    tau_horizontal: f64,
    tau_vertical: f64,
    tau_velocity: f64,
    /// Seconds between the instant a fix describes and the timestamp it is logged under.
    latency: f64,
    /// When the receiver is in the log at all.
    available: Window,
    /// A stretch inside that with no fixes: the receiver is there and reporting nothing.
    outage: Option<Window>,
    /// rad: a dual-antenna heading's σ, reported with every fix, or `None` for a receiver with
    /// one antenna.
    sigma_heading: Option<f64>,
}

/// A good 5 Hz receiver under open sky.
///
/// The σ the log reports is the σ the noise is drawn from. A receiver that lies about its own
/// accuracy is a worthwhile experiment and a different one; conflating the two would make every
/// gating result unattributable.
const GNSS: GnssErrors = GnssErrors {
    period: 0.2,
    sigma_horizontal: 0.9,
    sigma_vertical: 1.8,
    sigma_velocity: 0.15,
    tau_horizontal: 0.0,
    tau_vertical: 0.0,
    tau_velocity: 0.0,
    latency: 0.0,
    available: Window::ALWAYS,
    outage: None,
    sigma_heading: None,
};

/// One fix: position and velocity, each with the variance the log reports.
struct Fix {
    /// When the fix describes the vehicle: the row's time less the receiver's latency.
    taken: f64,
    position: [f64; 3],
    velocity: [f64; 3],
    position_variance: [f64; 3],
    velocity_variance: [f64; 3],
    /// A dual-antenna heading and its variance, where the receiver has two antennas.
    heading: Option<(f64, f64)>,
}

/// A first-order Gauss-Markov error: `e_k = φ e_{k−1} + √(1 − φ²) w_k`, `φ = exp(−T / τ)`, per
/// axis, for a sensor sampled every `T`.
///
/// `w_k` is the white draw at the stationary σ, so the error's σ is the same whatever `τ` is and
/// the sample the sensor reports stays honest about it; what correlation changes is how much of
/// the error averaging can remove, which is equation (24)'s assumption and #117's subject. The
/// first draw comes from the stationary distribution, `w_0` itself. At `τ = 0`, `φ = 0` and the
/// result is `w_k` bit for bit, which keeps every white scenario paired with one that is not.
struct GaussMarkov<const N: usize> {
    phi: [f64; N],
    last: Option<[f64; N]>,
}

impl<const N: usize> GaussMarkov<N> {
    fn new(period: f64, tau: [f64; N]) -> Self {
        Self {
            phi: tau.map(|tau| {
                if tau > 0.0 {
                    (-period / tau).exp()
                } else {
                    0.0
                }
            }),
            last: None,
        }
    }

    fn next(&mut self, white: [f64; N]) -> [f64; N] {
        let error = match self.last {
            None => white,
            Some(last) => std::array::from_fn(|i| {
                let phi = self.phi[i];
                phi * last[i] + (1.0 - phi * phi).sqrt() * white[i]
            }),
        };
        self.last = Some(error);
        error
    }
}

struct Gnss {
    errors: GnssErrors,
    position: Noise,
    velocity: Noise,
    position_error: GaussMarkov<3>,
    velocity_error: GaussMarkov<3>,
    heading: Noise,
}

impl Gnss {
    fn new(errors: GnssErrors, seed: u64) -> Self {
        Self {
            errors,
            position: Noise::new(seed, channel::GNSS_POSITION),
            velocity: Noise::new(seed, channel::GNSS_VELOCITY),
            position_error: GaussMarkov::new(
                errors.period,
                [
                    errors.tau_horizontal,
                    errors.tau_horizontal,
                    errors.tau_vertical,
                ],
            ),
            velocity_error: GaussMarkov::new(errors.period, [errors.tau_velocity; 3]),
            heading: Noise::new(seed, channel::GNSS_HEADING),
        }
    }

    /// The fix logged at `t`, if the receiver is reporting one.
    ///
    /// A stale fix describes where the vehicle *was*, under the timestamp it arrived — which is
    /// the whole of the latency model, and why it needs the trajectory rather than the current
    /// truth.
    fn fix(&mut self, t: f64, flight: &Trajectory) -> Option<Fix> {
        // Draw first, decide second. A window that suppresses a fix must not also skip its noise,
        // or every fix after the window is drawn from a stream the baseline never reaches:
        // `gnss_outage` is paired against `mission` sample for sample, and skipping the gap's 100
        // draws put the remaining 825 fixes on different numbers. The receiver is still running;
        // what a gap removes is the row.
        let sigma = [
            self.errors.sigma_horizontal,
            self.errors.sigma_horizontal,
            self.errors.sigma_vertical,
        ];
        let white = self.position.vector_with(sigma);
        let position_noise = self.position_error.next(white);
        let velocity_noise = self
            .velocity_error
            .next(self.velocity.vector(self.errors.sigma_velocity));
        // Its own stream, so a receiver with a second antenna draws the first one's numbers.
        let heading_noise = self
            .errors
            .sigma_heading
            .map(|sigma| (self.heading.vector(sigma)[0], sigma));

        if !self.errors.available.contains(t) {
            return None;
        }
        if self.errors.outage.is_some_and(|outage| outage.contains(t)) {
            return None;
        }
        // Before the log starts there is nothing for a stale fix to describe, so the first few
        // are dropped rather than extrapolated from a flight that had not begun.
        let described = t - self.errors.latency;
        if described < 0.0 {
            return None;
        }

        let was = flight.at(described);
        Some(Fix {
            taken: described,
            position: add(was.position, position_noise),
            velocity: add(was.velocity, velocity_noise),
            position_variance: sigma.map(|sigma| sigma * sigma),
            velocity_variance: [self.errors.sigma_velocity.powi(2); 3],
            heading: heading_noise.map(|(noise, sigma)| {
                let heading = was.euler[2] + noise;
                (heading.sin().atan2(heading.cos()), sigma * sigma)
            }),
        })
    }
}

/// Barometer errors.
#[derive(Clone, Copy)]
struct BaroErrors {
    period: f64,
    /// m.
    sigma: f64,
    /// m s⁻¹: reference drift, the quantity equation (30′) estimates.
    drift: f64,
    /// m: what the barometer reads on the ground. Initialization averages this into `α₀` and
    /// every later altitude is relative to it, so a non-zero value makes a scenario that never
    /// establishes a reference visibly wrong rather than accidentally right.
    reference: f64,
    /// s: time constant of the noise; see [`GaussMarkov`].
    tau: f64,
    available: Window,
}

const BARO: BaroErrors = BaroErrors {
    period: 0.05,
    sigma: 0.35,
    drift: 0.0,
    reference: 112.0,
    tau: 0.0,
    available: Window::ALWAYS,
};

/// One altitude reading, up-positive, with the variance the log reports.
struct Altitude {
    meters: f64,
    variance: f64,
}

struct Baro {
    errors: BaroErrors,
    noise: Noise,
    error: GaussMarkov<1>,
}

impl Baro {
    fn new(errors: BaroErrors, seed: u64) -> Self {
        Self {
            errors,
            noise: Noise::new(seed, channel::BARO),
            error: GaussMarkov::new(errors.period, [errors.tau]),
        }
    }

    /// Altitude above the barometer's own reference — and that reference drifts.
    ///
    /// Equation (30) takes `α₀` from the initialization window; everything after that is measured
    /// against a reference that may no longer be where it was, which (30′) is there to follow.
    fn sample(&mut self, t: f64, truth: &Truth) -> Option<Altitude> {
        // Drawn before the availability window is consulted, for the reason [`Gnss::fix`] gives.
        let [noise] = self.error.next([self.errors.sigma * self.noise.sample()]);
        if !self.errors.available.contains(t) {
            return None;
        }
        Some(Altitude {
            meters: self.errors.reference + self.errors.drift * t - truth.position[2] + noise,
            variance: self.errors.sigma.powi(2),
        })
    }
}

/// A magnetic heading error: the field turned about the down axis, for a stretch of the log.
#[derive(Clone, Copy)]
struct Disturbance {
    window: Window,
    /// rad.
    rotation: f64,
}

/// Magnetometer errors, on a field taken as already calibrated — hard- and soft-iron correction
/// is the application's job, so what is left here is noise and interference.
#[derive(Clone, Copy)]
struct MagErrors {
    period: f64,
    /// gauss, per axis.
    sigma_field: f64,
    /// s: time constant of the field noise, every axis; see [`GaussMarkov`].
    tau: f64,
    available: Window,
    disturbance: Option<Disturbance>,
}

const MAG: MagErrors = MagErrors {
    period: 0.05,
    sigma_field: 0.012,
    tau: 0.0,
    available: Window::ALWAYS,
    disturbance: None,
};

/// One field reading in body axes, with the heading variance the log reports.
struct Field {
    body: [f64; 3],
    heading_variance: f64,
}

struct Mag {
    errors: MagErrors,
    noise: Noise,
    error: GaussMarkov<3>,
}

impl Mag {
    fn new(errors: MagErrors, seed: u64) -> Self {
        Self {
            errors,
            noise: Noise::new(seed, channel::MAG),
            error: GaussMarkov::new(errors.period, [errors.tau; 3]),
        }
    }

    fn sample(&mut self, t: f64, truth: &Truth) -> Option<Field> {
        // Drawn before the availability window is consulted, for the reason [`Gnss::fix`] gives.
        let noise = self.error.next(self.noise.vector(self.errors.sigma_field));
        if !self.errors.available.contains(t) {
            return None;
        }
        let turned = self
            .errors
            .disturbance
            .filter(|d| d.window.contains(t))
            .map_or(0.0, |d| d.rotation);
        Some(Field {
            body: add(
                resolve_in_body(body_to_nav(truth.euler), magnetic_field(turned)),
                noise,
            ),
            heading_variance: self.heading_variance(),
        })
    }

    /// Heading variance to report with each sample, rad².
    ///
    /// Derived from the field noise and the horizontal field it acts on rather than pinned: a
    /// `var0` disagreeing with the noise actually injected would turn every heading consistency
    /// number into a statement about this file. Optimistic against a real magnetometer, whose
    /// error is dominated by calibration residual — bench-characterizing that residual is what a
    /// real application owes its own `R`.
    fn heading_variance(&self) -> f64 {
        let horizontal = FIELD * INCLINATION.cos();
        (self.errors.sigma_field / horizontal).powi(2)
    }
}

// ---------------------------------------------------------------------------------------------
// The scenarios
// ---------------------------------------------------------------------------------------------

/// One generated flight.
struct Scenario {
    name: &'static str,
    /// What this scenario covers that no other one does — the rule `data/manifest.txt` holds
    /// corpus logs to. Printed into both files, since a CSV found on its own should say what it
    /// is for, and to the console, so the list lives in exactly one place.
    covers: &'static str,
    /// Sequential; SplitMix64 is a mixer, so neighbouring seeds give unrelated streams.
    seed: u64,
    duration: f64,
    imu_rate: f64,
    trajectory: Trajectory,
    imu: ImuErrors,
    gnss: GnssErrors,
    baro: BaroErrors,
    mag: MagErrors,
    /// A stretch the logger lost: every row in it is drawn and then not written, IMU and aiding
    /// alike, so the streams stay paired with a scenario that has none.
    dropout: Option<Window>,
    /// rad: the sideslip σ the harness fuses the course constraint at, written as the log's
    /// `# Course sideslip` line, or `None` for a vehicle that does not point where it goes.
    course: Option<f64>,
}

/// Sitting on the ground, pointing somewhere unremarkable.
///
/// The heading is neither zero nor a multiple of a right angle, here and in every other
/// trajectory: a yaw convention error that would cancel at 0° stays visible.
fn at_rest() -> Trajectory {
    Trajectory {
        hold: 0.0,
        ramp: 0.0,
        north: STILL,
        east: STILL,
        down: STILL,
        roll: STILL,
        pitch: STILL,
        yaw: Wave {
            offset: 0.6,
            ..STILL
        },
        sideslip: None,
    }
}

/// A circuit with turns and climbs: a 2:1 horizontal figure-eight with a slow climb across it,
/// released after five seconds on the ground.
///
/// Curved rather than a box with corners, because a corner is a step in acceleration: truth would
/// still be exact, but no airframe flies it and the specific force would be a train of impulses
/// rather than a flight. As parameterized it peaks at roughly 21 m s⁻¹ and 3.7 m s⁻², which is a
/// brisk multirotor mission.
fn circuit() -> Trajectory {
    Trajectory {
        hold: 5.0,
        ramp: 4.0,
        north: Wave {
            amplitude: 120.0,
            period: 60.0,
            ..STILL
        },
        east: Wave {
            amplitude: 80.0,
            period: 30.0,
            ..STILL
        },
        down: Wave {
            amplitude: -20.0,
            period: 90.0,
            ..STILL
        },
        roll: Wave {
            amplitude: 0.25,
            period: 30.0,
            ..STILL
        },
        pitch: Wave {
            amplitude: 0.12,
            period: 20.0,
            ..STILL
        },
        yaw: Wave {
            offset: 0.6,
            amplitude: 0.9,
            period: 60.0,
            ..STILL
        },
        sideslip: None,
    }
}

/// A climbing, banked turn already in progress at `t = 0`: 30 m radius at 0.4 rad s⁻¹.
///
/// The yaw rate is a steady term rather than a harmonic so that it never dips: at 0.4 rad s⁻¹ it
/// clears `Initialization::max_gyro_rate` (0.262) continuously, and no two-second window anywhere
/// in the log reads as still. A harmonic of the same peak would pass under the tolerance for
/// three seconds twice a cycle and hand the harness a static window after all.
///
/// The bank is what the turn requires — atan(v²/r / g) = atan(4.8/g) = 0.455 rad — so the attitude
/// and the acceleration describe the same manoeuvre even though neither is derived from the other.
fn banked_turn() -> Trajectory {
    Trajectory {
        hold: 0.0,
        ramp: 0.0,
        north: Wave {
            amplitude: 30.0,
            period: 15.7,
            ..STILL
        },
        east: Wave {
            offset: 30.0,
            amplitude: -30.0,
            period: 15.7,
            phase: FRAC_PI_2,
            ..STILL
        },
        down: Wave {
            rate: -1.5,
            ..STILL
        },
        roll: Wave {
            offset: 0.455,
            ..STILL
        },
        pitch: STILL,
        yaw: Wave {
            offset: 0.6,
            rate: 0.4,
            ..STILL
        },
        sideslip: None,
    }
}

/// A short hop with sources appearing and dropping out, at a slow IMU rate.
///
/// This is `data/flight.csv`: gentle enough that the log stays small, and shaped so the
/// initialization window has a barometer but no magnetometer. See the `flight` entry in
/// [`scenarios`] for what that buys.
fn short_hop() -> Trajectory {
    Trajectory {
        hold: 2.0,
        ramp: 2.0,
        north: Wave {
            amplitude: 20.0,
            period: 20.0,
            ..STILL
        },
        east: Wave {
            amplitude: 8.0,
            period: 12.0,
            ..STILL
        },
        down: Wave {
            amplitude: -5.0,
            period: 24.0,
            ..STILL
        },
        roll: Wave {
            amplitude: 0.12,
            period: 12.0,
            ..STILL
        },
        pitch: Wave {
            amplitude: 0.08,
            period: 9.0,
            ..STILL
        },
        yaw: Wave {
            offset: 0.6,
            amplitude: 0.4,
            period: 20.0,
            ..STILL
        },
        sideslip: None,
    }
}

/// A fixed-wing's orbit: on the ground pointing along its track, then a 150 m circle at 18 m/s,
/// climbing gently, with the nose a few degrees off the track as a crosswind would put it.
///
/// Yaw is the course plus the sideslip wave, so heading is observable from the velocity alone
/// and from nothing else: this is the vehicle `Eskf::fuse_course` is for. The start angle is
/// 0.6 rad, for the reason [`at_rest`] gives. Roll and pitch are not the turn's — the specific
/// force is computed from the acceleration and the attitude whatever they are, as on `circuit`.
fn orbit() -> Trajectory {
    const RADIUS: f64 = 150.0;
    const PERIOD: f64 = TAU * RADIUS / 18.0;
    const START: f64 = 0.6;
    Trajectory {
        hold: 5.0,
        ramp: 4.0,
        north: Wave {
            offset: -RADIUS * 0.564_642_473, // sin 0.6
            amplitude: RADIUS,
            period: PERIOD,
            phase: START,
            ..STILL
        },
        east: Wave {
            offset: RADIUS * 0.825_335_615, // cos 0.6
            amplitude: -RADIUS,
            period: PERIOD,
            phase: START + FRAC_PI_2,
            ..STILL
        },
        down: Wave {
            amplitude: -15.0,
            period: 90.0,
            ..STILL
        },
        roll: Wave {
            amplitude: 0.1,
            period: 30.0,
            ..STILL
        },
        pitch: Wave {
            amplitude: 0.05,
            period: 20.0,
            ..STILL
        },
        yaw: STILL,
        sideslip: Some(Wave {
            offset: 0.02,
            amplitude: 0.035,
            period: 25.0,
            ..STILL
        }),
    }
}

/// The scenario table.
///
/// Eight of them — `harsh_imu`, `gnss_outage`, `baro_drift`, `gnss_latency`, `correlated`,
/// `mag_disturbance`, `gnss_heading`, `logging_dropout` — are one-variable departures from
/// `mission`, and the pairing is by
/// construction rather than by assertion: same trajectory, same duration, and **the same seed**.
/// Streams are split per sensor, so a departure that leaves a sensor alone reproduces the
/// baseline's draws for it bit for bit, and `diff mission.csv <departure>.csv` shows the fault and
/// nothing else. Differing seeds would have left every sensor differing everywhere, which is the
/// attribution this whole arrangement exists to buy.
///
/// The four that are not departures — `static`, `moving_start`, `no_mag`, `flight` — carry their own
/// seeds, so a statistic aggregated across the set still has independent draws to work with.
fn scenarios() -> Vec<Scenario> {
    /// The seed the baseline and its departures share. See above: this is the pairing.
    const PAIRED: u64 = 2;

    let base = Scenario {
        name: "",
        covers: "",
        seed: PAIRED,
        duration: 185.0,
        // 200 Hz: fast enough that first-order propagation is not the thing being measured, slow
        // enough that a scenario is a few megabytes.
        imu_rate: 200.0,
        trajectory: circuit(),
        imu: IMU,
        gnss: GNSS,
        baro: BARO,
        mag: MAG,
        dropout: None,
        course: None,
    };

    vec![
        // Bias observability: at rest the gyro bias is the whole gyro reading, while the
        // horizontal accelerometer bias is indistinguishable from a tilt of b/g. Sixty seconds is
        // long enough for the walk to move the bias measurably against its own white noise.
        Scenario {
            name: "static",
            covers: "60 s on the ground: bias observability, and the at-rest specific force",
            seed: 1,
            duration: 60.0,
            trajectory: at_rest(),
            ..base
        },
        // The baseline every departure below is measured against.
        Scenario {
            name: "mission",
            covers: "5 s static, then a 180 s circuit with turns and climbs: the baseline",
            ..base
        },
        // The only scenario whose sensors are not the nominal table, so a statistic across the
        // set is not reading one IMU seven times. Its aiding is untouched, which is what makes
        // the IMU the single variable — and what keeps `gnss_outage` below a single variable too.
        Scenario {
            name: "harsh_imu",
            covers: "the baseline flown on a badly isolated IMU: ten times the white noise and \
                     twenty times the bias walk of every other scenario",
            imu: HARSH_IMU,
            ..base
        },
        // Dead reckoning: 20 s with no fixes at all, over the fastest part of the circuit, and
        // the one scenario where the gap is what the position has to live on. On the nominal IMU
        // deliberately — pairing it against `mission` says what the gap cost, and pairing
        // `harsh_imu` against `mission` says what the sensor cost. Combining them in one file
        // would measure neither.
        Scenario {
            name: "gnss_outage",
            covers: "20 s without GNSS: dead-reckoning growth against the reported sigma",
            gnss: GnssErrors {
                outage: Some(Window {
                    start: 60.0,
                    end: 80.0,
                }),
                ..GNSS
            },
            ..base
        },
        // No static window anywhere: airborne and turning from the first sample. Covers the
        // coarse start, `Status::Aligning`, all three `Fusion::Reset` adoptions -- the position
        // and velocity a moving start never had, and the heading it could not level -- and the
        // barometric reference a window taken in motion cannot establish, read from the estimate
        // instead (#115).
        Scenario {
            name: "moving_start",
            covers: "airborne from t=0 in a banked turn: coarse alignment, Fusion::Reset, a \
                     barometric reference read from the estimate, and time to aligned",
            seed: 4,
            duration: 90.0,
            trajectory: banked_turn(),
            ..base
        },
        // The reference walks 2 cm/s away from the one initialization fixed, 3.7 m over the log:
        // what GOALS.md's "barometric reference as an estimated offset" exists to follow, and the
        // scenario that measured a constant `α₀` costing 2 m of height.
        Scenario {
            name: "baro_drift",
            covers: "the baseline with the barometric reference drifting 0.02 m/s: what a \
                     reference that is not followed costs in vertical position",
            baro: BaroErrors {
                drift: 0.02,
                ..BARO
            },
            ..base
        },
        // The "Measurement latency" open question, made measurable: fixes describe where the
        // vehicle was 150 ms ago and are timestamped now. At 20 m s⁻¹ that is 3 m of position
        // error correlated with velocity — which is what distinguishes it from GNSS noise, and
        // what a delayed fusion horizon would remove.
        Scenario {
            name: "gnss_latency",
            covers: "the baseline with fixes 150 ms stale: the cost of fusing GNSS against the \
                     current state",
            gnss: GnssErrors {
                latency: 0.15,
                ..GNSS
            },
            ..base
        },
        // Every aiding error persisting between samples, which (24) fuses as independent and
        // averages down. The time constants are the corpus's: each real log's `acf1_` per source
        // with every measurement fused as white, read as τ = −T / ln ρ at that log's own sample
        // interval, over the logs whose autocorrelation is positive. `Config::correlation`
        // takes their median; this takes the median of their upper half, sources slower than
        // the filter assumes. Not the default on purpose: an error drawn at the filter's own `τ`
        // scores it against its assumption, and innovations are whiter than the error behind
        // them, so the corpus understates `τ` and slower is the direction a real sensor errs in.
        Scenario {
            name: "correlated",
            covers: "the baseline with every aiding error correlated in time, slower than the filter \
                     assumes: GNSS position 8.7 s and 38 s, velocity 0.89 s, barometer 1.1 s, \
                     magnetometer 4.3 s",
            gnss: GnssErrors {
                tau_horizontal: 8.7,
                tau_vertical: 38.0,
                tau_velocity: 0.89,
                ..GNSS
            },
            baro: BaroErrors { tau: 1.1, ..BARO },
            mag: MagErrors { tau: 4.3, ..MAG },
            ..base
        },
        // Gating: 10 s of a field turned 30° about the down axis, which reads as a 30° heading
        // error — far outside anything the heading gate should accept, and the scenario that says
        // whether it does.
        Scenario {
            name: "mag_disturbance",
            covers: "10 s of a 30 degree magnetic heading error: whether the heading gate rejects \
                     it, and what it costs if it does not",
            mag: MagErrors {
                disturbance: Some(Disturbance {
                    window: Window {
                        start: 60.0,
                        end: 70.0,
                    },
                    rotation: 30.0_f64.to_radians(),
                }),
                ..MAG
            },
            ..base
        },
        // A second heading source beside the magnetometer, as `a299e722` flies one: a moving-
        // baseline receiver reporting heading with every fix, at 0.01 rad. One variable, on
        // `mission`'s seed: the heading draws come from their own stream, so every other row is
        // `mission`'s.
        Scenario {
            name: "gnss_heading",
            covers: "the baseline with a dual-antenna GNSS heading beside the magnetometer: \
                     fuse_gnss_heading, and two heading sources gated against one estimate",
            gnss: GnssErrors {
                sigma_heading: Some(0.01),
                ..GNSS
            },
            ..base
        },
        // A fixed-wing with no magnetometer, the vehicle #53 is for: nothing observes yaw until
        // it moves, and then only the course does. The sideslip the harness allows, 0.05 rad, is
        // wider than the truth's, 0.02 ± 0.035, as a caller who knows the airframe would state it.
        Scenario {
            name: "no_mag",
            covers: "a fixed-wing with no magnetometer, still and then orbiting: heading from \
                     fuse_course alone, refused until the vehicle moves",
            seed: 3,
            duration: 120.0,
            trajectory: orbit(),
            mag: MagErrors {
                available: Window::NEVER,
                ..MAG
            },
            course: Some(0.05),
            ..base
        },
        // An IMU gap at speed, and the one scenario that reaches (22′). The logger loses 1.2 s at
        // 20 m s⁻¹ in a turn, as `4b473e91`'s did at 30 m s⁻¹. Coasted, the first fix after it is
        // accepted; refused (`Config::coast = None`), the state stays where the gap began under a
        // covariance that did not grow, every fix after it is tens of σ away, and only
        // `Config::recovery` gets it back.
        Scenario {
            name: "logging_dropout",
            covers: "1.2 s of the log lost at speed in a turn: the gap coasted by (22')",
            dropout: Some(Window {
                start: 60.0,
                end: 61.2,
            }),
            ..base
        },
        // `data/flight.csv`, the log CI replays, and the only scenario whose output is committed.
        // What no other one covers is sources that come and go: the magnetometer starts after the
        // initialization window, so yaw begins unobserved and `sigma_yaw` stays inflated, while
        // the barometer is on from the first sample and does fix a reference. GNSS then stops at
        // 6 s and the other two at 8.5 s, which walks the status down through `Degraded` to
        // `DeadReckoning` without the filter resetting itself.
        Scenario {
            name: "flight",
            covers: "sources appearing and dropping out mid-log at 50 Hz: an unobserved initial \
                     heading, then aiding lost one source at a time",
            seed: 8,
            // Five seconds past the last aiding row, which is `Timeouts::dead_reckoning_after`:
            // the log has to outlive its own sources or the walk down through `Degraded` stops
            // one rung short and the smoke test never reaches `DeadReckoning`.
            duration: 14.0,
            imu_rate: 50.0,
            trajectory: short_hop(),
            gnss: GnssErrors {
                available: Window {
                    start: 2.0,
                    end: 6.0,
                },
                ..GNSS
            },
            baro: BaroErrors {
                // 10 Hz: 20 Hz is not a whole number of 50 Hz IMU steps, and `ticks` refuses
                // rather than rounding a rate the log cannot express.
                period: 0.1,
                available: Window {
                    start: 0.0,
                    end: 8.5,
                },
                ..BARO
            },
            mag: MagErrors {
                period: 0.1,
                available: Window {
                    start: 3.5,
                    end: 8.5,
                },
                ..MAG
            },
            ..base
        },
    ]
}

// ---------------------------------------------------------------------------------------------
// Generation
// ---------------------------------------------------------------------------------------------

fn main() {
    if let Err(e) = run() {
        eprintln!("simulate: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let (positional, overrides) = split_options(env::args().skip(1))?;
    let mut positional = positional.into_iter();
    let wanted = positional.next().unwrap_or_else(|| "all".to_string());
    let out_dir = positional
        .next()
        .map_or_else(default_out_dir, PathBuf::from);

    let mut selected = scenarios();
    selected.retain(|scenario| wanted == "all" || wanted == scenario.name);
    if selected.is_empty() {
        let names: Vec<&str> = scenarios().iter().map(|scenario| scenario.name).collect();
        return Err(format!("unknown scenario `{wanted}`; have {}", names.join(", ")).into());
    }
    // Every selected scenario takes the same overrides, so the departures from `mission` stay
    // paired with it at each seed of an ensemble exactly as they are at `PAIRED`.
    for scenario in &mut selected {
        overrides.apply(scenario);
    }

    fs::create_dir_all(&out_dir)?;
    println!("fusion-nav simulator — seeded flights with analytic truth\n");
    for scenario in &selected {
        generate(scenario, &out_dir, &overrides)?.print(scenario);
    }
    Ok(())
}

/// What the command line changes about the table, for sweeps: none of it is a scenario of its
/// own, which is why a run under one says so in the log's header.
#[derive(Default)]
struct Overrides {
    /// `--seed <n>`, in place of the table's.
    seed: Option<u64>,
    /// `--speed <k>`: every translational wave scaled by `k`, so speed and acceleration scale
    /// with it and the attitude does not. #52's "at more than one speed".
    speed: Option<f64>,
    /// `--latency <s>`: the receiver's latency, in place of the table's.
    latency: Option<f64>,
}

impl Overrides {
    fn apply(&self, scenario: &mut Scenario) {
        if let Some(seed) = self.seed {
            scenario.seed = seed;
        }
        if let Some(k) = self.speed {
            let trajectory = &mut scenario.trajectory;
            for wave in [
                &mut trajectory.north,
                &mut trajectory.east,
                &mut trajectory.down,
            ] {
                wave.rate *= k;
                wave.amplitude *= k;
            }
        }
        if let Some(latency) = self.latency {
            scenario.gnss.latency = latency;
        }
    }

    /// The header line naming the sweep overrides, if there are any. The seed is not one: every
    /// header already names the seed.
    fn note(&self) -> Option<String> {
        let mut cells = Vec::new();
        if let Some(k) = self.speed {
            cells.push(format!("speed x{k}"));
        }
        if let Some(latency) = self.latency {
            cells.push(format!("gnss latency {latency} s"));
        }
        (!cells.is_empty()).then(|| format!("# Overridden for a sweep: {}", cells.join(", ")))
    }
}

/// The positional arguments, and the [`Overrides`] named in place of the table's.
///
/// The seed is an override rather than a column of seeds per scenario: `data/anees.sh` flies each
/// scenario on fifty of them (#89), and the table's own seed stays the one `data/scenarios.txt`
/// pins.
fn split_options(args: impl Iterator<Item = String>) -> Result<(Vec<String>, Overrides), String> {
    let mut positional = Vec::new();
    let mut overrides = Overrides::default();
    let mut args = args;
    while let Some(arg) = args.next() {
        let mut value = |what: &str| args.next().ok_or_else(|| format!("{arg} needs {what}"));
        match arg.as_str() {
            "--seed" => {
                let v = value("a value")?;
                overrides.seed = Some(
                    v.parse()
                        .map_err(|_| format!("--seed `{v}` is not an unsigned integer"))?,
                );
            }
            "--speed" | "--latency" => {
                let v = value("a number")?;
                let number: f64 = v
                    .parse()
                    .ok()
                    .filter(|n: &f64| n.is_finite() && *n >= 0.0)
                    .ok_or_else(|| format!("{arg} `{v}` is not a number at least zero"))?;
                if arg == "--speed" {
                    overrides.speed = Some(number);
                } else {
                    overrides.latency = Some(number);
                }
            }
            _ => positional.push(arg),
        }
    }
    Ok((positional, overrides))
}

/// Write one scenario's log and truth files.
fn generate(
    scenario: &Scenario,
    out_dir: &Path,
    overrides: &Overrides,
) -> Result<Report, Box<dyn Error>> {
    let log_path = out_dir.join(format!("{}.csv", scenario.name));
    let truth_path = out_dir.join(format!("{}.truth.csv", scenario.name));
    let mut log = Log::new(BufWriter::new(File::create(&log_path)?));
    let mut truth = BufWriter::new(File::create(&truth_path)?);
    let note = overrides.note();
    write_log_header(&mut log.out, scenario, note.as_deref())?;
    write_truth_header(&mut truth, scenario, note.as_deref())?;

    let dt = 1.0 / scenario.imu_rate;
    let epochs = (scenario.duration * scenario.imu_rate).round() as usize;
    let gnss_every = ticks(scenario.gnss.period, dt, "gnss")?;
    let baro_every = ticks(scenario.baro.period, dt, "baro")?;
    let mag_every = ticks(scenario.mag.period, dt, "mag")?;

    let mut imu = Imu::new(scenario.imu, scenario.seed, dt);
    let mut gnss = Gnss::new(scenario.gnss, scenario.seed);
    let mut baro = Baro::new(scenario.baro, scenario.seed);
    let mut mag = Mag::new(scenario.mag, scenario.seed);
    let mut report = Report::new(log_path, truth_path, epochs);

    for epoch in 0..epochs {
        let t = epoch as f64 * dt;
        let state = scenario.trajectory.at(t);

        // Every draw is made whether or not it is written, so a dropout costs rows and nothing
        // else: the samples either side of it are the ones the paired scenario has.
        let logged = !scenario.dropout.is_some_and(|dropout| dropout.contains(t));

        let reading = imu.sample(&state);
        let [gx, gy, gz] = reading.gyro;
        let [ax, ay, az] = reading.accel;
        if logged {
            log.row(t, "imu", &[gx, gy, gz, ax, ay, az], &[], None)?;
        }
        write_truth_row(&mut truth, t, &state, &reading)?;
        report.epoch(t, &state, &reading);

        if epoch % gnss_every == 0
            && let Some(fix) = gnss.fix(t, &scenario.trajectory)
            && logged
        {
            let taken = Some(fix.taken);
            log.row(t, "gnss_pos", &fix.position, &fix.position_variance, taken)?;
            log.row(t, "gnss_vel", &fix.velocity, &fix.velocity_variance, taken)?;
            if let Some((heading, variance)) = fix.heading {
                log.row(t, "gnss_yaw", &[heading], &[variance], taken)?;
            }
        }
        if epoch % baro_every == 0
            && let Some(altitude) = baro.sample(t, &state)
            && logged
        {
            log.row(t, "baro", &[altitude.meters], &[altitude.variance], None)?;
        }
        if epoch % mag_every == 0
            && let Some(field) = mag.sample(t, &state)
            && logged
        {
            log.row(t, "mag", &field.body, &[field.heading_variance], None)?;
        }
    }

    log.out.flush()?;
    truth.flush()?;
    report.rows = log.rows;
    Ok(report)
}

/// IMU epochs between two samples of a source, refusing a period the rate cannot express.
///
/// A 20 Hz barometer on a 50 Hz IMU would otherwise silently become 25 Hz, and the manifest rule
/// that a log says what it covers would be broken by rounding.
fn ticks(period: f64, dt: f64, source: &str) -> Result<usize, String> {
    let ticks = period / dt;
    if ticks < 1.0 || (ticks - ticks.round()).abs() > 1e-9 {
        return Err(format!(
            "{source} period {period} s is not a whole number of {dt} s IMU steps"
        ));
    }
    Ok(ticks.round() as usize)
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

// ---------------------------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------------------------

/// What the generator measured about the flight it just wrote.
///
/// Measured here rather than derived from the table, because the table holds amplitudes and
/// periods while the questions are about peaks — and because these are the figures that say
/// whether the harness will align statically or coarsely, which no parameter states directly.
struct Report {
    log: PathBuf,
    truth: PathBuf,
    epochs: usize,
    /// Rows the log holds, read back from [`Log`] once it is written rather than tallied here.
    rows: usize,
    /// Running sum of specific force over the first [`OPENING`] seconds, and its count. At rest
    /// and level the mean is `[0, 0, −g]` plus the accelerometer bias, which is the check that
    /// the frames, the gravity sign and the bias all agree.
    opening_force: [f64; 3],
    opening_samples: u32,
    opening_gyro: f64,
    opening_deviation: f64,
    peak_speed: f64,
}

impl Report {
    fn new(log: PathBuf, truth: PathBuf, epochs: usize) -> Self {
        Self {
            log,
            truth,
            epochs,
            rows: 0,
            opening_force: [0.0; 3],
            opening_samples: 0,
            opening_gyro: 0.0,
            opening_deviation: 0.0,
            peak_speed: 0.0,
        }
    }

    fn epoch(&mut self, t: f64, truth: &Truth, reading: &ImuReading) {
        self.peak_speed = self.peak_speed.max(norm(truth.velocity));
        if t < OPENING {
            self.opening_force = add(self.opening_force, reading.accel);
            self.opening_samples += 1;
            self.opening_gyro = self.opening_gyro.max(norm(reading.gyro));
            self.opening_deviation = self
                .opening_deviation
                .max((norm(reading.accel) - GRAVITY).abs());
        }
    }

    fn print(&self, scenario: &Scenario) {
        let samples = f64::from(self.opening_samples.max(1));
        let mean = self.opening_force.map(|sum| sum / samples);
        println!("{}  — {}", scenario.name, scenario.covers);
        println!(
            "  {} rows, {} IMU epochs over {:.0} s at {:.0} Hz, seed {}",
            self.rows, self.epochs, scenario.duration, scenario.imu_rate, scenario.seed
        );
        println!("  log   {}", self.log.display());
        println!("  truth {}", self.truth.display());
        println!(
            "  opening {OPENING:.0} s: mean f_b ({:.4}, {:.4}, {:.4}) m/s^2, peak gyro {:.4} \
             rad/s, peak |f|-g {:.4} m/s^2",
            mean[0], mean[1], mean[2], self.opening_gyro, self.opening_deviation,
        );
        println!("  peak speed {:.2} m/s\n", self.peak_speed);
    }
}

// ---------------------------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------------------------

/// The log file, and the count of what has been written to it.
///
/// The count lives with the writer because it is a fact about the file. A caller tallying rows
/// of its own is one edit away from disagreeing with what is on disk, and nothing downstream
/// would notice: the number only ever reaches a console line.
struct Log<W: Write> {
    out: W,
    rows: usize,
}

impl<W: Write> Log<W> {
    fn new(out: W) -> Self {
        Self { out, rows: 0 }
    }

    /// One measurement row: six value columns, three variance columns, and when the
    /// measurement was taken, blank where the source does not use them. A barometer and a
    /// magnetometer read the instant they are logged, so only a fix carries a time of its own.
    fn row(
        &mut self,
        t: f64,
        source: &str,
        values: &[f64],
        variances: &[f64],
        taken: Option<f64>,
    ) -> io::Result<()> {
        write!(self.out, "{t:.4},{source}")?;
        for column in 0..6 {
            match values.get(column) {
                Some(value) => write!(self.out, ",{value:.6}")?,
                None => write!(self.out, ",")?,
            }
        }
        for column in 0..3 {
            match variances.get(column) {
                Some(variance) => write!(self.out, ",{variance:.6}")?,
                None => write!(self.out, ",")?,
            }
        }
        match taken {
            Some(taken) => write!(self.out, ",{taken:.4}")?,
            None => write!(self.out, ",")?,
        }
        writeln!(self.out)?;
        self.rows += 1;
        Ok(())
    }
}

fn write_truth_row(
    out: &mut impl Write,
    t: f64,
    truth: &Truth,
    reading: &ImuReading,
) -> io::Result<()> {
    write!(out, "{t:.4}")?;
    for value in truth
        .position
        .iter()
        .chain(&truth.velocity)
        .chain(&truth.euler)
        .chain(&reading.accel_bias)
        .chain(&reading.gyro_bias)
    {
        write!(out, ",{value:.6}")?;
    }
    writeln!(out)
}

fn write_log_header(
    out: &mut impl Write,
    scenario: &Scenario,
    note: Option<&str>,
) -> io::Result<()> {
    writeln!(
        out,
        "# fusion-nav simulated flight - scenario `{name}`, seed {seed}\n\
         # {covers}\n\
         #\n\
         # Magnetic declination {declination:.6} rad\n\
{course}\
         #\n\
         # Generated by `cargo run --example simulate -- {name} <dir>`; truth in\n\
         # `{name}.truth.csv`. Do not edit, regenerate. Six decimals throughout, three orders\n\
         # below the smallest sigma in the scenario table.\n\
         #\n\
         # One row per measurement, sorted by time. Blank cells are not applicable.\n\
         #   imu       v0..v2 gyro rad/s          v3..v5 accel m/s^2\n\
         #   gnss_pos  v0..v2 NED position m      var0..var2 m^2\n\
         #   gnss_vel  v0..v2 NED velocity m/s    var0..var2 m^2/s^2\n\
         #   baro      v0     altitude m (up)     var0      m^2\n\
         #   mag       v0..v2 field, calibrated   var0      heading rad^2\n\
         #   gnss_yaw  v0     dual-antenna heading rad  var0  rad^2\n\
         #   t_meas_s  when a fix was taken, t_s less the receiver's latency",
        name = scenario.name,
        seed = scenario.seed,
        covers = scenario.covers,
        declination = DECLINATION,
        course = scenario.course.map_or_else(String::new, |sigma| format!(
            "# Course sideslip {sigma:.6} rad\n"
        )),
    )?;
    if let Some(note) = note {
        writeln!(out, "{note}")?;
    }
    writeln!(out, "t_s,source,v0,v1,v2,v3,v4,v5,var0,var1,var2,t_meas_s")
}

fn write_truth_header(
    out: &mut impl Write,
    scenario: &Scenario,
    note: Option<&str>,
) -> io::Result<()> {
    writeln!(
        out,
        "# fusion-nav truth for `{}.csv`, seed {}\n\
         #\n\
         # The analytic trajectory the log was generated from, at the IMU rate: closed-form\n\
         # position and Euler angles, exact derivatives, and the biases actually applied to that\n\
         # sample. Angles are radians, ZYX, in the NED/FRD convention the crate fixes.",
        scenario.name, scenario.seed,
    )?;
    if let Some(note) = note {
        writeln!(out, "{note}")?;
    }
    writeln!(
        out,
        "t_s,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,roll,pitch,yaw,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z"
    )
}

fn default_out_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("target/sim")
}
