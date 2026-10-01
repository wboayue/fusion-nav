//! Replay a recorded flight from CSV and write the estimate back out as CSV.
//!
//! `predict` propagates the nominal state and its covariance, (9)–(22), and every source the
//! crate carries corrects both, (23)–(41), so the estimate columns hold an aided trajectory and
//! every test ratio on the fusion rows is real. What this establishes is the replay
//! harness and the normalized log format `GOALS.md` commits to. What it exercises, that `basic.rs` and
//! `degradation.rs` cannot, is irregular `dt` taken from timestamps, per-sample variance,
//! a source that appears partway through the log, and an initialization window found in
//! the data rather than asserted.
//!
//! Run with `cargo run --example replay`, or point it at your own files:
//!
//! ```text
//! cargo run --example replay -- data/flight.csv target/replay.csv
//! cargo run --example replay -- data/flight.csv target/replay.csv data/flight.truth.csv
//! cargo run --example replay -- --r-policy px4 data/logs/<log>.csv target/replay.csv
//! ```
//!
//! The third argument is optional and turns on scoring against truth; see *Scoring* below.
//! `--r-policy`, anywhere on the line, picks what GNSS rows are fused with: `raw`, the
//! default, or `px4`; see [`RPolicy`]. `--declination model` ignores the header's declination
//! and leaves the filter to read its own magnetic model at the origin; `header` is the default.
//! `--antenna zero` fuses every GNSS row as the IMU's rather than at the header's antenna.
//! `--recovery off` replays with `Recovery::OFF`, the filter that only reports, which is how a
//! lockout recovery ends is priced (`recovery=` on the `summary` line says which ran).
//! `--set <field>=<value>`, repeatable, sets one `Config` field by its path, `--set
//! correlation.gnss_position=2.5`; `settings.rs` lists the names, and `set=` on the `summary`
//! line repeats them. `--derive` is the mode that works those values out from a log; see
//! `derive.rs`.
//! It also writes `<out>.nees.csv`, `ε` per block per epoch, which `data/anees.sh` averages
//! across seeds (#89), and `<out>.error.csv`, the truth error on navigation axes beside its σ,
//! which `tools/replay_report.py` draws.
//!
//! # Input
//!
//! One row per measurement, sorted by time. `#` comments and the header are skipped, and
//! blank cells are those not applicable to that source. Leading comments that describe the
//! vehicle and its site are read, since those and not the harness decide them: a
//! `# Magnetic declination <rad> rad` line sets `Eskf::set_magnetic_declination` (a log without
//! one is replayed at zero), a `# GNSS antenna <forward> <right> <down> m` line is every GNSS
//! fix's `antenna` (zero without one), and `# Navigation origin <lat> <lon> <height>` names the
//! site the magnetic model is looked up at. A `# GNSS noise parameters` line is read only under
//! `--r-policy px4`, which refuses a log without one.
//!
//! ```text
//! t_s,source,v0,v1,v2,v3,v4,v5,var0,var1,var2,t_meas_s
//! 0.0200,imu,0.0004,0.0020,0.0001,0.0054,0.0299,-9.8067,,,,
//! 2.0000,gnss_pos,0,0,0,,,,2.25,2.25,5.625,1.8900
//! 2.0000,baro,0.0273,,,,,,4,,,
//! ```
//!
//! `t_s` is when a row arrives, which is the order the harness hands it to the filter in, and
//! `t_meas_s` when a measurement was taken, the `time` its `fuse_*` is given: a fix computed
//! 110 ms before it reached the autopilot. Blank, or absent from an older file, it is `t_s`.
//!
//! One interleaved stream rather than a file per sensor, because the interleaving is the
//! thing worth exercising. Converting rosbag, ULog, or a dataflash log into this shape is
//! a separate concern, deliberately: the replay path stays free of ROS and PX4 tooling so
//! it runs in CI with no hardware and no toolchain.
//!
//! # Output
//!
//! Two files, and a third when scoring. `<out>.csv` holds one row per IMU epoch after initialization: the state, the
//! covariance diagonal as standard deviations, and the most recent test ratio per source.
//! The sigma columns are what make the result plottable as estimate ± 3σ against truth; the
//! test ratios are directly comparable with the innovation ratios PX4 publishes.
//!
//! `<out>.fusion.csv` holds one row per verdict — a `fuse_*` call, or each half of a GNSS
//! fix — the resolution the epoch file cannot reach, since it keeps only the *last* ratio per
//! source and so cannot tell two fusions apart or say what became of one:
//!
//! ```text
//! # one row per verdict. gates gnss_pos=13.815511 gnss_hgt=10.827566 gnss_vel=16.266235 baro=10.827566 mag=10.827566
//! # nu* and s* are the filter's published innovation and diag(S), empty where the gate ran no update: an adoption, a refusal, a call before initialization
//! t_s,source,nu0,nu1,nu2,s0,s1,s2,ratio,outcome,truth
//! 0.0000,baro,,,,,,,,not_initialized,
//! 2.2000,gnss_pos,1.350874,1.096309,,1.258352,1.258354,,0.1741,accepted,good
//! 2.2000,gnss_hgt,2.224828,,,4.004997,,,0.1142,accepted,good
//! ```
//!
//! The gates ride in the header because the filter reports `r = ε / γ`: without `γ` a ratio
//! does not go back to `ε`, and `ε` is what a consistency statistic needs.
//!
//! `nu*` and `s*` are `SourceHealth::innovation` as the filter published it: `ν` of (23) and
//! the diagonal of `S` of (24), padded with empty fields past the observation's dimension.
//! They are empty where the gate ran no update — a refusal or an adoption. The
//! harness never works them out from the measurement and the covariance itself: that would be
//! a second implementation of a quantity the update owns, free to disagree with it while
//! somebody chased a filter bug that did not exist.
//!
//! # Scoring
//!
//! Given a third argument — a `<scenario>.truth.csv` from `examples/simulate.rs`, or the
//! truth `tools/urbannav2replay.py` converts beside its log — the harness prints a `score`
//! line beside `summary`, in the same `key=value` shape so one
//! parser reads both. No truth file means no `score` line at all, rather than a line of
//! zeros: a corpus log has no truth, and a missing line says that where `pos_h=0.000` would
//! claim a perfect filter. `data/manifest.txt` is therefore untouched by any of this.
//!
//! Every key describes one error vector, `δx = truth ⊖ estimate` in the error-state
//! ordering of equation (2) — see [`error_state`], which is the only place it is computed.
//! `tilt`, `yaw` and the attitude half of `false_valid` read its attitude on navigation axes
//! instead, from [`attitude_error_ned`]: tilt is rotation about north and east, heading about
//! down, and `δθ`'s body axes are those only while the vehicle is level.
//!
//! | key | what it is |
//! | --- | --- |
//! | `pos_h`, `pos_v`, `vel` | RMSE, m and m s⁻¹ |
//! | `pos_h_max` | worst horizontal position error, m — the excursion an RMSE hides |
//! | `tilt`, `yaw` | RMS attitude error about the horizontal axes and about down, degrees |
//! | `ba`, `bg` | RMS bias error, m s⁻² and rad s⁻¹ — the estimate against the bias applied |
//! | `in3s` | fraction of axis-epochs within 3σ, over all 15 states |
//! | `nees_pos`, `nees_vel`, `nees_att` | mean NEES per degree of freedom; ≈1 is consistent |
//! | `false_valid` | quantity-epochs claimed usable while the error exceeded `Config::accuracy` |
//! | `false_valid_att` | the tilt and heading share of it, pinned apart so a total cannot hide it |
//! | `scored` | epochs that had a truth row, which is what every figure above is a mean over |
//! | `offered_<half>` | GNSS verdicts the gate or an adoption decided, per half of the fix |
//! | `bad_<half>` | of those, fixes truth says their own `R` could not explain; see [`Judged`] |
//! | `rejected_bad_<half>`, `rejected_good_<half>` | rejections split by that verdict |
//! | `accepted_far_<half>` | fixes accepted while past `Accuracy::position` on some axis |
//! | `adopted_bad_<half>` | bad fixes taken by an adoption, a recovery's or a first one |
//! | `recovered_after_bad_<half>`, `recovered_after_lockout_<half>` | recoveries by the run they ended; see [`FixScore`] |
//! | `rms_<half>` | RMS of those fixes' own error against truth, m: `pos_h` and `pos_v` for the receiver |
//!
//! `<half>` is `gnss_pos` or `gnss_hgt`, the two verdicts one fix meets. These are the only keys
//! that say whether a rejection was *right*, which needs hostile fixes and truth together
//! (GOALS.md, "Three questions"): on the simulator they read a handful of bad fixes in a
//! thousand, and on UrbanNav's M8T most of them (`data/urbannav-pins.txt`). The fusion file
//! carries each verdict in its `truth` column. `rms_` is the other thing only truth can say of
//! a fix: how far off the receiver was, so a filter's `pos_h` can be read against what its
//! only position source gave it (on INSANE, `data/insane-pins.txt`).
//!
//! Convergence is **not** here: `aligned_at=` is on the `summary` line, it needs no truth,
//! and the corpus pins it. One statistic, one implementation (`AGENTS.md`).
//!
//! `in3s` and `nees_*` are not the same test twice. `in3s` is marginal — per axis, on the
//! covariance diagonal alone — and `nees_*` is joint, reading each block's correlations. A
//! covariance with the right variances and the wrong correlations passes the first and fails
//! the second. Neither reaches the bias states as an *error*: `in3s` asks only whether the
//! bias error sits inside a 3σ that grows on its own, and no `nees_*` block covers them.
//!
//! `ba` and `bg` are that error, against the bias the truth file carries as *applied* to
//! that sample, walk included. They are the only keys that fail when a bias estimate walks
//! away from the bias in the IMU, which is a failure the position keys absorb for a long
//! time: an accelerometer bias enters position through two integrations, so a filter can
//! hold metre accuracy against fixes while its `β̂ₐ` is wrong by most of the bias. The
//! simulator injects a fixed `[0.043, −0.062, 0.027]` m s⁻² that `ImuNoise::default` does
//! not know about, so a `ba` at that magnitude is a filter estimating nothing, and the
//! distance below it is what the aiding bought.
//!
//! `false_valid` counts quantity-epochs where the filter's own [`Validity`] said a quantity
//! was usable and the truth error was outside `Config::accuracy`. It reads the filter's
//! verdict rather than re-deriving σ ≤ accuracy from the covariance, and it falsifies that
//! verdict the way the filter states it — per axis, not on a 2-D norm. Both halves are the
//! same point: the key is a test of the claim, and re-deriving either half tests a copy of
//! it instead. [`Quantity::falsified`] has the arithmetic.
//!
//! `false_valid_att` is the same count restricted to tilt and heading. A total is an allowance,
//! and an allowance hides whatever it is not being spent on; `data/scenarios.txt` pins this one
//! at zero wherever it is zero, so an attitude regression cannot settle under a velocity
//! allowance granted for something else.
//!
//! Read it as a rate against `epochs=`, not as a defect count. The bar is a 1σ one, so a
//! filter sitting exactly at it — which `Accuracy::default`'s attitude figures do — fails it
//! constantly while being perfectly tuned: about a third of epochs for a single-axis claim
//! (`heading`, `position (v)`, `velocity (v)`) and about half for a two-axis one, which is
//! outside the bar if either axis is. A low count means the filter kept margin; a high one
//! is the evidence for widening either the bar or the report.
//!
//! # What the score measures today
//!
//! Every source the crate carries reaches the state: the two GNSS observations, the barometer
//! and magnetic heading. Attitude is observed directly for the first time in `yaw` alone —
//! (34)–(36) constrain the rotation about gravity and nothing else, so `tilt` is still
//! corrected only through the correlations (20) builds, and a heading is priced for the tilt
//! it was levelled by through (36′).
//!
//! `nees_*` is a ratio of two quantities that both move: the error, and a `P` that (22) grows
//! and (27) shrinks. On these scenarios it approaches 1 from below, because
//! `ImuNoise::default()` is PX4's, an allowance for vibration, scale-factor error and coning
//! that an analytic simulator does not produce, and it sits 17× above even `HARSH_IMU` on
//! accelerometer noise. So a consistency key here fails in the *overconfident* direction only
//! where something reaches the filter that its model does not describe. No scenario shows one
//! on a single seed; `gnss_latency`'s late fixes would, fused as current, and (23′) fuses each
//! at the time it was taken. The ensemble of `data/anees.sh` sees the rest, and
//! `data/anees.txt` names them.
//!
//! Six of the scenarios are one-variable departures from `mission` on `mission`'s seed, so
//! what attributes a fault is `score(departure) − score(mission)` rather than either alone.
//! `gnss_outage`, `gnss_latency` and `correlated` separate through the GNSS they change,
//! and `harsh_imu`
//! through propagation — no longer on the position keys at all, which read `mission`'s figures
//! now that two quantities are aided, but on `tilt` and on `ba`, the keys that read the IMU's
//! own errors. `baro_drift` separates on height — `pos_v` 0.917 m against `mission`'s 0.170,
//! and `nees_pos` 0.44 against 0.09, which is what a drifting reference costs once (30′)
//! estimates it. `mag_disturbance` separates on `yaw` alone — 0.381 deg against `mission`'s
//! 0.361 — which is what a 30 deg field error costs a filter that refuses all 200 samples of
//! it.
//!
//! A refused propagation step is still scored. The epoch row is written either way — the
//! state is simply the one before it — and that stale state is what the filter published, so
//! that is what it answers for. No scenario reaches it today; the corpus log `f16771dd` is
//! the only thing that does, and it has no truth.

use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

use fusion_nav::STATES;
use fusion_nav::prelude::*;

mod derive;
mod settings;
use nalgebra::{Quaternion, SMatrix, SVector, UnitQuaternion, Vector3};

/// Capacity of the initialization window, in samples.
///
/// `Initialization::min_duration` is a span of time, so the samples it takes depend on the
/// log: 2 s is 100 samples at 50 Hz and 800 at 400 Hz. The corpus needs 500 at most, at
/// 250 Hz; this is sized for the faster IMUs a converter will eventually hand it. The harness
/// buffers, where an embedded caller folds samples into a `StaticWindow` as they arrive,
/// because it slides the window forward until it is still and an accumulator cannot drop its
/// oldest sample.
const WINDOW: usize = 1024;

/// Intervals sampled before fixing the IMU rate.
const PROBE: usize = 64;

/// How long to keep sliding the window looking for stillness before accepting a coarse
/// start, in seconds of log time.
///
/// A policy, not a filter constant: the harness would rather align properly, but a log
/// whose vehicle is already moving still has to replay.
const PATIENCE: f64 = 10.0;

/// What the harness hands each GNSS `fuse_*` as `R`, reported as `r_policy=` on the `summary`
/// line.
///
/// `Raw`, the default and the only policy `data/manifest.txt` pins, fuses every measurement
/// with the variance its row carries, unbounded, where both production estimators floor a
/// receiver's reported accuracy first — the parameters and lines are on
/// `PositionNoise::clamped` and `VelocityNoise::clamped`, and `data/README.md` carries why
/// this harness applies neither by default.
///
/// `Px4` applies EKF2's own rule through those same constructors, with the parameters the
/// log's EKF2 flew, so a comparison
/// against its solution (#8) has one `R` policy in it rather than two. A published figure is a
/// claim about an `R` policy as much as about the filter, and nothing else on the `summary`
/// line would distinguish a raw run from a floored one — `rejected=`, `transitions=` and
/// every `nis_` would simply read differently, with no key saying why.
///
/// GNSS only: PX4 logs no barometer or heading variance, so those rows already carry the
/// converter's constants under either policy.
#[derive(Clone, Copy, Debug, PartialEq)]
enum RPolicy {
    Raw,
    Px4(GnssNoiseParameters),
}

/// `EKF2_GPS_P_NOISE`, `EKF2_GPS_V_NOISE` and `EKF2_NOAID_NOISE` as the log's header states
/// them — the converter supplies PX4's default for one the log lacks, and says so.
#[derive(Clone, Copy, Debug, PartialEq)]
struct GnssNoiseParameters {
    position: f32,
    velocity: f32,
    no_aid: f32,
}

impl RPolicy {
    fn name(self) -> &'static str {
        match self {
            RPolicy::Raw => "raw",
            RPolicy::Px4(_) => "px4",
        }
    }

    /// A GNSS position row's `R`, from the variances it carries.
    ///
    /// Under `Px4`, EKF2's rule as `PositionNoise::clamped`'s doc writes it: horizontal held
    /// within `p..=noaid`, vertical within `1.5 p..=noaid`. PX4 caps horizontal only while GNSS
    /// is the sole horizontal aiding, which it always is here, and vertical only while it is
    /// *not* the sole vertical source; the cap is applied to both, and it binds only where a
    /// receiver claims worse than 10 m. PX4's second horizontal floor, 0.01 m on the variance's
    /// root, is folded into the first: `max(p, 0.01)` then the cap reads the same as PX4's
    /// cap then 0.01 whenever `noaid` ≥ 0.01.
    ///
    /// A variance that is not a finite non-negative number passes through untouched, so the
    /// filter refuses the row as it would under `Raw`: the square root of a negative one is a
    /// NaN, and the row's own number is the one worth printing.
    fn position(self, var: [f32; 3]) -> PositionNoise<Ned> {
        let RPolicy::Px4(p) = self else {
            return PositionNoise::from_variance(var[0], var[1], var[2]);
        };
        if !var.iter().all(|v| v.is_finite() && *v >= 0.0) {
            return PositionNoise::from_variance(var[0], var[1], var[2]);
        }
        PositionNoise::clamped(
            var[0].sqrt(),
            var[2].sqrt(),
            SigmaBounds::new(p.position.max(0.01), p.no_aid),
            SigmaBounds::new(1.5 * p.position, p.no_aid),
        )
    }

    /// A GNSS heading row's `R`.
    ///
    /// Under `Px4`, EKF2's hard-coded 0.1 rad floor, `HeadingNoise::clamped`'s first example,
    /// which also reads an accuracy of zero as the floor rather than refusing it.
    fn heading(self, var: f32) -> HeadingNoise {
        match self {
            RPolicy::Px4(_) if var.is_finite() && var >= 0.0 => {
                HeadingNoise::clamped(var.sqrt(), SigmaBounds::at_least(0.1))
            }
            _ => HeadingNoise::from_variance(var),
        }
    }

    /// A GNSS velocity row's `R`.
    ///
    /// Under `Px4`, EKF2's rule as `VelocityNoise::clamped`'s doc writes it: `sacc` floored at
    /// `max(v, 0.01)` and vertical widened by 1.5 after the floor, which is the vertical σ
    /// `1.5 sacc` floored at `1.5 max(v, 0.01)`. The row's first variance is `sacc²`, as the
    /// converter writes it to all three.
    fn velocity(self, var: [f32; 3]) -> VelocityNoise<Ned> {
        let RPolicy::Px4(p) = self else {
            return VelocityNoise::from_variance(var[0], var[1], var[2]);
        };
        if !var.iter().all(|v| v.is_finite() && *v >= 0.0) {
            return VelocityNoise::from_variance(var[0], var[1], var[2]);
        }
        let (sacc, floor) = (var[0].sqrt(), p.velocity.max(0.01));
        VelocityNoise::clamped(
            sacc,
            1.5 * sacc,
            SigmaBounds::at_least(floor),
            SigmaBounds::at_least(1.5 * floor),
        )
    }
}

/// Reads one estimate column out of a `State`.
type Column = fn(&State) -> f32;

/// The error covariance as the harness's own algebra reads it.
type CovarianceMatrix = SMatrix<f32, STATES, STATES>;

/// The filter's covariance as `nalgebra`'s. Built entry by entry: `nalgebra`'s `From` for an
/// array reads columns, and would score the transpose.
fn matrix(covariance: &Covariance) -> CovarianceMatrix {
    let p = covariance.to_rows();
    CovarianceMatrix::from_fn(|row, column| p[row][column])
}

/// The filter's attitude as `nalgebra`'s, for the harness's own algebra. Unchecked, so the
/// rotation scored is the filter's to the last bit: the getter hands back what it stores.
fn rotation(attitude: Attitude) -> UnitQuaternion<f32> {
    let q = attitude.body_to_ned();
    UnitQuaternion::new_unchecked(Quaternion::new(q.w, q.x, q.y, q.z))
}

/// The estimate columns: each name next to the value it reads. Drives both the header and
/// the row, so the two cannot drift apart.
///
/// Attitude is the quaternion `Attitude::from_body_to_ned` names, Hamilton and scalar-first,
/// rather than Euler angles: ZYX Euler cannot separate roll from yaw at 90° of pitch, where a
/// tailsitter cruises, while the quaternion is the rotation itself at every attitude. What a
/// reader wants drawn from it — tilt, heading, the difference from EKF2 — is derived by
/// `tools/replay_report.py`.
const ESTIMATE: [(&str, Column); 16] = [
    ("pos_n", |s| s.position.x()),
    ("pos_e", |s| s.position.y()),
    ("pos_d", |s| s.position.z()),
    ("vel_n", |s| s.velocity.x()),
    ("vel_e", |s| s.velocity.y()),
    ("vel_d", |s| s.velocity.z()),
    ("q0", |s| s.attitude.body_to_ned().w),
    ("q1", |s| s.attitude.body_to_ned().x),
    ("q2", |s| s.attitude.body_to_ned().y),
    ("q3", |s| s.attitude.body_to_ned().z),
    ("ba_x", |s| s.accel_bias.x()),
    ("ba_y", |s| s.accel_bias.y()),
    ("ba_z", |s| s.accel_bias.z()),
    ("bg_x", |s| s.gyro_bias.x()),
    ("bg_y", |s| s.gyro_bias.y()),
    ("bg_z", |s| s.gyro_bias.z()),
];

/// The covariance diagonal, in the error-state ordering. Drives both the header and the
/// row, so the two cannot drift apart.
const SIGMAS: [(ErrorState, &str); 15] = [
    (ErrorState::PositionNorth, "sigma_pos_n"),
    (ErrorState::PositionEast, "sigma_pos_e"),
    (ErrorState::PositionDown, "sigma_pos_d"),
    (ErrorState::VelocityNorth, "sigma_vel_n"),
    (ErrorState::VelocityEast, "sigma_vel_e"),
    (ErrorState::VelocityDown, "sigma_vel_d"),
    (ErrorState::AttitudeX, "sigma_att_x"),
    (ErrorState::AttitudeY, "sigma_att_y"),
    (ErrorState::AttitudeZ, "sigma_att_z"),
    (ErrorState::AccelBiasX, "sigma_ba_x"),
    (ErrorState::AccelBiasY, "sigma_ba_y"),
    (ErrorState::AccelBiasZ, "sigma_ba_z"),
    (ErrorState::GyroBiasX, "sigma_bg_x"),
    (ErrorState::GyroBiasY, "sigma_bg_y"),
    (ErrorState::GyroBiasZ, "sigma_bg_z"),
];

/// Reads one variance out of an `AttitudeVariance`.
type AttitudeColumn = fn(AttitudeVariance) -> f32;

/// The attitude's σ on navigation axes, from `Eskf::attitude_variance`: tilt about north and
/// east, heading about down.
///
/// Beside `SIGMAS`' body-axis attitude rather than instead of it. The body diagonal is what
/// NEES and `in3s` compare `δθ` against; these are what a tilt or heading panel can be banded
/// with, which the body diagonal cannot be at 90° of pitch and cannot be rotated into without
/// the off-diagonals the row does not carry.
const ATTITUDE_SIGMAS: [(&str, AttitudeColumn); 3] = [
    ("sigma_tilt_n", |v| v.tilt_north),
    ("sigma_tilt_e", |v| v.tilt_east),
    ("sigma_heading", |v| v.heading),
];

/// Source names as the input spells them, in `Diagnostics` order, so a fusion row names the
/// row it came from. The constants below index this and `RATIOS` alike.
///
/// `gnss_hgt` is the one name no input row carries: a `gnss_pos` row is one fix and two
/// verdicts, the horizontal half under `gnss_pos` and the height under `gnss_hgt`, because
/// the filter gates them apart (`GnssFusion`). `course` is the other: no row carries a course,
/// which the harness fuses after each `gnss_vel` row when a sideslip is given (`sideslip_of`).
const SOURCES: [&str; 7] = [
    "gnss_pos", "gnss_hgt", "gnss_vel", "baro", "mag", "gnss_yaw", "course",
];

/// What each source's innovation components are, in the order the filter publishes them, so a
/// `nu_` key names an axis rather than a subscript.
///
/// `Innovation` carries values and variances and no names for them, so this table is the only
/// statement of what component 0 of a GNSS position is — which makes it the tenth place
/// `AGENTS.md` lists a fifth source having to reach, and the one that would otherwise be
/// discovered by an index out of range. The lengths are the observation dimensions of
/// (28)–(30) and (34)–(36), and are checked against what the filter publishes rather than
/// trusted.
const AXES: [&[&str]; 7] = [
    &["n", "e"],
    &["d"],
    &["n", "e", "d"],
    &["d"],
    &["yaw"],
    &["yaw"],
    &["yaw"],
];

/// The row sources `--without` can drop: every aiding row an input carries. Not `SOURCES`,
/// which names verdicts, two of which (`gnss_hgt`, `course`) no row carries, and a name that
/// dropped nothing would still print on the `summary` line as if it had.
const DROPPABLE: [&str; 5] = ["gnss_pos", "gnss_vel", "baro", "mag", "gnss_yaw"];

/// Last test ratio per source, in `Diagnostics` order.
const RATIOS: [&str; 7] = [
    "r_gnss_pos",
    "r_gnss_hgt",
    "r_gnss_vel",
    "r_baro",
    "r_mag",
    "r_gnss_yaw",
    "r_course",
];
const GNSS_POS: usize = 0;
const GNSS_HGT: usize = 1;
const GNSS_VEL: usize = 2;
const BARO: usize = 3;
const MAG: usize = 4;
const GNSS_YAW: usize = 5;
const COURSE: usize = 6;

fn main() {
    if let Err(e) = run() {
        eprintln!("replay: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut policy = None;
    let mut options = Options::default();
    let mut derive = false;
    let mut positional = Vec::new();
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--r-policy" {
            policy = Some(args.next().ok_or("--r-policy wants `raw` or `px4`")?);
        } else if arg == "--course" {
            let degrees: f32 = args
                .next()
                .and_then(|value| value.parse().ok())
                .ok_or("--course wants a sideslip sigma in degrees")?;
            options.course = Some(Radians::from_degrees(degrees));
        } else if arg == "--declination" {
            options.model_declination = match args.next().as_deref() {
                Some("header") => false,
                Some("model") => true,
                _ => return Err("--declination wants `header` or `model`".into()),
            };
        } else if arg == "--antenna" {
            options.antenna_from_header = match args.next().as_deref() {
                Some("header") => true,
                Some("zero") => false,
                _ => return Err("--antenna wants `header` or `zero`".into()),
            };
        } else if arg == "--recovery" {
            options.recovery = match args.next().as_deref() {
                Some("on") => true,
                Some("off") => false,
                _ => return Err("--recovery wants `on` or `off`".into()),
            };
        } else if arg == "--set" {
            let pair = args.next().ok_or("--set wants <field>=<value>")?;
            let (name, value) = pair
                .split_once('=')
                .ok_or_else(|| format!("--set `{pair}`: want <field>=<value>"))?;
            options.sets.push((name.to_string(), value.to_string()));
        } else if arg == "--without" {
            let name = args.next().ok_or("--without wants an input source name")?;
            if !DROPPABLE.contains(&name.as_str()) {
                return Err(
                    format!("--without `{name}`: want one of {}", DROPPABLE.join(", ")).into(),
                );
            }
            options.without = Some(name);
        } else if arg == "--derive" {
            derive = true;
        } else {
            positional.push(arg);
        }
    }
    let mut args = positional.into_iter();
    let input = args.next().map_or_else(default_input, PathBuf::from);

    let text = fs::read_to_string(&input).map_err(|e| format!("{}: {e}", input.display()))?;
    options.policy = match policy.as_deref() {
        None | Some("raw") => RPolicy::Raw,
        Some("px4") => RPolicy::Px4(gnss_noise_of(&text).ok_or(
            "--r-policy px4 needs a `# GNSS noise parameters` header line; \
             reconvert the log with tools/ulog2replay.py",
        )?),
        Some(other) => return Err(format!("--r-policy `{other}`: want `raw` or `px4`").into()),
    };
    if derive {
        // Stdout is the `Config` alone, so `> file` captures Rust; the evidence is stderr's.
        if args.next().is_some() || !options.sets.is_empty() || !options.recovery {
            return Err(
                "--derive takes one input and starts from Config::default(): \
                        no output, truth, --set or --recovery"
                    .into(),
            );
        }
        let derived = derive::derive(&input, &text, &options)?;
        eprint!("{}", derived.report());
        print!("{}", derived.render());
        if !derived.holds() {
            return Err(
                "the derived Config coasted steps its max_predict_dt did not predict".into(),
            );
        }
        return Ok(());
    }
    let output = args.next().map_or_else(default_output, PathBuf::from);
    let truth = args.next();

    let config = options.config()?;
    // Optional, and absent on every corpus log: no truth file, no `score` line. Opened with
    // the log in hand, so a truth file belonging to another scenario is refused here rather
    // than scored against this one.
    let scoring = truth
        .map(|path| Scoring::open(path.into(), &text))
        .transpose()?;
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    let fusions = output.with_extension("fusion.csv");
    let mut epoch_out = BufWriter::new(File::create(&output)?);
    let mut fusion_out = BufWriter::new(File::create(&fusions)?);
    write_header(&mut epoch_out)?;
    write_fusion_header(&mut fusion_out, config.gates)?;

    let mut replay = prepare(&text, config, &options, scoring)?;
    drive(
        &mut replay,
        &text,
        &input,
        &mut Sinks {
            epochs: &mut epoch_out,
            fusions: &mut fusion_out,
        },
    )?;
    epoch_out.flush()?;
    fusion_out.flush()?;
    if let Some(scoring) = &replay.scoring {
        let mut nees_out = BufWriter::new(File::create(output.with_extension("nees.csv"))?);
        scoring.write_nees(&mut nees_out)?;
        nees_out.flush()?;
        let mut error_out = BufWriter::new(File::create(output.with_extension("error.csv"))?);
        scoring.write_error(&mut error_out)?;
        error_out.flush()?;
    }

    replay.report(&input, &output, &fusions);
    Ok(())
}

/// What the command line chose about a replay besides its files: everything [`prepare`] reads,
/// so `--derive` runs a log the way the command line would.
#[derive(Clone)]
struct Options {
    policy: RPolicy,
    course: Option<Radians>,
    without: Option<String>,
    model_declination: bool,
    antenna_from_header: bool,
    recovery: bool,
    sets: Vec<(String, String)>,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            policy: RPolicy::Raw,
            course: None,
            without: None,
            model_declination: false,
            antenna_from_header: true,
            recovery: true,
            sets: Vec::new(),
        }
    }
}

impl Options {
    /// `Config::default()`, with `--recovery off` and then each `--set` applied.
    fn config(&self) -> Result<Config, String> {
        let mut config = Config::default();
        if !self.recovery {
            config.recovery = Recovery::OFF;
        }
        // After `--recovery off`, so a source's own timeout can be put back on top of it.
        for (name, value) in &self.sets {
            settings::set(&mut config, name, value)?;
        }
        Ok(config)
    }
}

/// A replay of `text` under `config`, set up from the log's header and `options` and ready for
/// its first row.
fn prepare(
    text: &str,
    config: Config,
    options: &Options,
    scoring: Option<Scoring>,
) -> Result<Replay, Box<dyn Error>> {
    let mut replay = Replay::new(config, options.policy, scoring)?;
    replay.course = options.course.or_else(|| sideslip_of(text));
    if replay
        .course
        .is_some_and(|sideslip| !(sideslip.as_radians() > 0.0 && sideslip.as_radians().is_finite()))
    {
        return Err("the course sideslip is not a positive number".into());
    }
    replay.without = options.without.clone();
    replay.sets = options.sets.clone();
    replay.site = origin_of(text);
    if options.antenna_from_header {
        replay.antenna = antenna_of(text).unwrap_or_default();
        if !replay.antenna.to_array().iter().all(|v| v.is_finite()) {
            return Err("the `# GNSS antenna` header is not a finite offset".into());
        }
    }
    if options.model_declination {
        // The header's declination is not read. The site is handed over as the origin before
        // initializing, which is where the filter reads its own magnetic model, so the window
        // levels at the model's value. A static start then clears the origin, and positions
        // stay the file's NED about it; nothing else moves.
        let site = replay
            .site
            .ok_or("--declination model needs a `# Navigation origin` header line")?;
        if !replay.filter.set_origin(site) {
            return Err("the `# Navigation origin` header is not a usable origin".into());
        }
    } else if !replay.filter.set_magnetic_declination(declination_of(text)) {
        return Err("the `# Magnetic declination` header is not a finite number".into());
    }
    Ok(replay)
}

/// Every row of `text` through `replay`, and what a pending start was holding at the end.
fn drive(
    replay: &mut Replay,
    text: &str,
    input: &Path,
    out: &mut Sinks,
) -> Result<(), Box<dyn Error>> {
    for (n, line) in text.lines().enumerate() {
        replay
            .row(line, out)
            .map_err(|e| format!("{}:{}: {e}", input.display(), n + 1))?;
    }
    replay.finish(out)?;
    Ok(())
}

/// Each source's gate threshold, indexed by the same constants as [`SOURCES`].
///
/// One function serving two readings. Applied to the configured gates it gives the `γ` that
/// turns a published test ratio back into `ε = r γ`, equation (38) read backwards. Applied to
/// `Gates::at(Percentile::P95)` it gives the 95 % chi-square quantile at each source's degrees
/// of freedom — the bound `nis_over95_` counts against, taken from the table `src/config.rs`
/// already checks against the closed-form CDF rather than written out a second time here.
fn thresholds(gates: Gates) -> [f32; SOURCES.len()] {
    [
        gates.gnss_position.threshold(),
        gates.gnss_height.threshold(),
        gates.gnss_velocity.threshold(),
        gates.baro_altitude.threshold(),
        gates.mag_heading.threshold(),
        gates.gnss_heading.threshold(),
        gates.course.threshold(),
    ]
}

/// One axis of one source's innovations, and the statistics defined on that series.
///
/// `ν` is summed as it arrives, because a mean needs no history. `x = ν/√S_ii` is kept,
/// because the lag-1 autocorrelation written as `Σxₜxₜ₊₁ − (n−1)x̄²` over `Σx² − n x̄²` loses
/// its significant digits exactly where `nu_` reports an offset worth having — the case the
/// statistic exists to find. The two-pass definition has no such subtraction. The cost is
/// bounded and small: the longest series in the corpus is the 2 h log's 35 631 magnetometer
/// rows, 143 KB, in a host-side example that already allocates per row.
#[derive(Clone, Default)]
struct Series {
    nu_sum: f64,
    normalized: Vec<f32>,
}

impl Series {
    /// `ν` and the `S_ii` the filter published beside it.
    fn push(&mut self, nu: f32, variance: f32) {
        self.nu_sum += f64::from(nu);
        self.normalized.push(nu / variance.sqrt());
    }

    fn len(&self) -> usize {
        self.normalized.len()
    }

    /// Mean `ν`, in the observation's own units. `None` where the source gated nothing.
    ///
    /// Zero for a well-modelled source. A standing offset is what a declination error, an
    /// uncorrected lever arm and measurement latency each leave behind, and none of them
    /// moves the NIS mean nearly as legibly.
    fn nu_mean(&self) -> Option<f64> {
        (self.len() > 0).then(|| self.nu_sum / self.len() as f64)
    }

    /// Lag-1 autocorrelation of the normalized innovation: `Σ(xₜ−x̄)(xₜ₊₁−x̄) / Σ(xₜ−x̄)²`.
    ///
    /// Zero for a white sequence, which is what an innovation is when the filter's model of
    /// the measurement is right. `None` below two samples, and on a series that does not
    /// vary at all, where the ratio is 0/0 rather than zero.
    fn autocorrelation(&self) -> Option<f64> {
        if self.len() < 2 {
            return None;
        }
        let mean = self.normalized.iter().map(|&x| f64::from(x)).sum::<f64>() / self.len() as f64;
        let centered: Vec<f64> = self
            .normalized
            .iter()
            .map(|&x| f64::from(x) - mean)
            .collect();
        let variance: f64 = centered.iter().map(|x| x * x).sum();
        let lagged: f64 = centered.windows(2).map(|pair| pair[0] * pair[1]).sum();
        (variance > 0.0).then(|| lagged / variance)
    }
}

/// The innovation-consistency statistics of #5, accumulated per source as the replay runs.
///
/// Every number here is built from what the filter published — `ν` and diag(`S`) on
/// [`SourceHealth::innovation`], the test ratio beside them — and never from the measurement
/// and the covariance, which would be a second implementation of (23) and (24) free to
/// disagree with the first. `AGENTS.md`, *one statistic, one implementation*.
///
/// What these say that `rejected_` cannot: a rejection count of zero is equally consistent
/// with an `R` that is right and with one a hundred times too wide, and on most corpus logs
/// most gated sources have never been turned down. A NIS mean is what tells those
/// apart. For the barometer and the magnetometer it tests a constant `tools/ulog2replay.py`
/// invented, PX4 logging no variance for either; for GNSS it tests the receiver's own
/// `eph`/`epv`/`s_variance_m_s`. `data/README.md` carries that caveat beside the keys.
#[derive(Clone)]
struct Consistency {
    /// Per source, per axis, trimmed to the observation's dimension by what is pushed.
    axes: [[Series; 3]; SOURCES.len()],
    /// `Σε` and the dimension the filter reported, per source.
    epsilon: [f64; SOURCES.len()],
    dimension: [usize; SOURCES.len()],
    /// Rows whose `ε` exceeded the 95, 99 and 99.9 % chi-square quantiles at that source's
    /// dimension, in [`PERCENTILES`] order.
    over: [[u32; 3]; SOURCES.len()],
    /// `γ` as configured, and the three bounds, all from [`thresholds`].
    gamma: [f32; SOURCES.len()],
    bounds: [[f32; SOURCES.len()]; 3],
    /// When each source's rows were recorded, so that its lag-one autocorrelation can be read
    /// as a time constant at the interval it was taken over.
    times: [Vec<f64>; SOURCES.len()],
}

/// The percentiles [`Consistency`] counts exceedances at: the one `nis_over95_` reports, and the
/// two above it that `--derive` sets a gate's choice against.
const PERCENTILES: [Percentile; 3] = [Percentile::P95, Percentile::P99, Percentile::P999];

impl Consistency {
    fn new(gates: Gates) -> Self {
        Self {
            axes: Default::default(),
            epsilon: [0.0; SOURCES.len()],
            dimension: [0; SOURCES.len()],
            over: [[0; 3]; SOURCES.len()],
            gamma: thresholds(gates),
            bounds: PERCENTILES.map(|percentile| thresholds(Gates::at(percentile))),
            times: Default::default(),
        }
    }

    /// Record one measurement the gate ran an update for.
    ///
    /// Rejections are included, and that is the decision this population turns on: their `ε`
    /// is computed before the gate has a verdict, so it is as real as an acceptance's, and
    /// leaving them out censors exactly the tail the statistic is measuring. What it costs is
    /// that a rejected fix leaves the state where an accepted one would not, so every later
    /// `ε` on that source is conditioned on the refusal — a NIS mean over a gated source
    /// describes the filter that ran, not the one that would have run.
    fn record(&mut self, t: f64, source: usize, ratio: f32, innovation: &Innovation) {
        let epsilon = ratio * self.gamma[source];
        self.epsilon[source] += f64::from(epsilon);
        self.dimension[source] = innovation.values().len();
        for (count, bounds) in self.over[source].iter_mut().zip(&self.bounds) {
            if epsilon > bounds[source] {
                *count += 1;
            }
        }
        self.times[source].push(t);
        for (axis, (&nu, &variance)) in innovation
            .values()
            .iter()
            .zip(innovation.variances())
            .enumerate()
        {
            self.axes[source][axis].push(nu, variance);
        }
    }

    /// Measurements this source contributed, which is what every mean divides by.
    fn rows(&self, source: usize) -> usize {
        self.axes[source][0].len()
    }

    /// Mean `ε` per degree of freedom: 1 for a filter whose `S` describes its own
    /// innovations, below 1 where the reported `R` is wider than the measurement earns, above
    /// it where the source claims an accuracy it does not deliver.
    fn nis(&self, source: usize) -> Option<f64> {
        let dimension = self.dimension[source];
        (self.rows(source) > 0 && dimension > 0)
            .then(|| self.epsilon[source] / self.rows(source) as f64 / dimension as f64)
    }

    /// The fraction of measurements whose `ε` sat above the 95 % bound — 0.05 for a filter
    /// whose innovations are distributed the way `S` claims.
    ///
    /// Beside the mean rather than instead of it, because the two fail differently: a mean
    /// near 1 built from a body of tiny residuals and a handful of large ones describes no
    /// distribution at all, and it is the tail that the gate's own percentile rests on.
    fn over95_fraction(&self, source: usize) -> Option<f64> {
        self.over_fraction(source, 0)
    }

    /// The fraction of measurements whose `ε` sat above the bound at `PERCENTILES[which]`.
    fn over_fraction(&self, source: usize, which: usize) -> Option<f64> {
        (self.rows(source) > 0)
            .then(|| f64::from(self.over[source][which]) / self.rows(source) as f64)
    }

    /// The median interval between this source's recorded rows, seconds: the lag `acf1_` is
    /// taken at. The median rather than the mean, because a source that bursts or drops out
    /// stretches a mean past every interval it actually has: `2c42096b`'s GNSS height reads a
    /// 1.545 s mean at a 1 s rate. `None` below two rows.
    fn interval(&self, source: usize) -> Option<f64> {
        let times = &self.times[source];
        let mut steps: Vec<f64> = times.windows(2).map(|pair| pair[1] - pair[0]).collect();
        if steps.is_empty() {
            return None;
        }
        steps.sort_by(f64::total_cmp);
        Some(steps[steps.len() / 2])
    }

    /// Lag-1 autocorrelation for a source, averaged over its axes.
    ///
    /// Per source rather than per axis, which is the one aggregate here that hides something
    /// and is worth the hiding: what this tests is whether successive measurements are
    /// independent, and that is a property of the source's own sampling and internal
    /// filtering — a 1 Hz receiver smooths its solution in time, and the smoothing arrives on
    /// every axis of the fix at once. If an axis ever separates from its siblings, the key
    /// splits the way `rejected=` did.
    ///
    /// `None` where no axis had two samples to correlate.
    fn autocorrelation(&self, source: usize) -> Option<f64> {
        let per_axis: Vec<f64> = self.axes[source]
            .iter()
            .take(self.dimension[source])
            .filter_map(Series::autocorrelation)
            .collect();
        (!per_axis.is_empty()).then(|| per_axis.iter().sum::<f64>() / per_axis.len() as f64)
    }
}

/// A statistic, or the word for not having one.
///
/// `none` rather than `0.0000` follows `nees_text`: a zero is a reading, and a source that
/// gated nothing has not produced one. It also keeps an empty source from quietly clearing a
/// range in `data/manifest.txt`, since `data/expect.sh` refuses a value that is not a number
/// where a bound was asked for.
fn measured(value: Option<f64>, places: usize) -> String {
    value.map_or_else(|| "none".to_string(), |value| format!("{value:.places$}"))
}

/// What the vehicle did, for the `summary` line: how far it went, how fast, how far over.
///
/// These say what a log *is* — AGENTS.md, "Know what a log is before reading its figures as
/// accuracy" — so a manifest note's claim that a log flies past a kilometre or pitches to
/// the vertical is pinned rather than asserted. Extent and speed are the receiver's own rows
/// as the log gives them, gated or not and before initialization too, because they describe
/// the flight rather than the filter; a glitch the gate refuses still counts. Tilt is the
/// filter's estimate, the only attitude a replay has, on navigation axes: the angle between
/// body down and navigation down, which no Euler sequence reaches at 90° of pitch.
///
/// So each key carries an error of its own. A glitch the gate refuses still stretches
/// `extent`, and the first fix is the origin whatever it was. `tilt_max` includes the
/// filter's tilt error and the attitude initialization committed, which is how it found
/// bias walks read as densities without conversion (#137); the manifest note has the
/// figures.
#[derive(Clone, Default)]
struct Excursion {
    origin: Option<(f32, f32)>,
    extent: Option<f32>,
    speed_max: Option<f32>,
    tilt_max: Option<f32>,
}

impl Excursion {
    /// Horizontal distance from the first fix in the log.
    fn fix(&mut self, north: f32, east: f32) {
        let (n0, e0) = *self.origin.get_or_insert((north, east));
        let distance = (north - n0).hypot(east - e0);
        self.extent = Some(self.extent.map_or(distance, |d| d.max(distance)));
    }

    /// Horizontal ground speed, the one a fixed-wing's range and a fix's latency scale with.
    fn velocity(&mut self, north: f32, east: f32) {
        let speed = north.hypot(east);
        self.speed_max = Some(self.speed_max.map_or(speed, |s| s.max(speed)));
    }

    fn attitude(&mut self, attitude: Attitude) {
        let down = rotation(attitude) * Vector3::z();
        let tilt = down.z.clamp(-1.0, 1.0).acos().to_degrees();
        self.tilt_max = Some(self.tilt_max.map_or(tilt, |t| t.max(tilt)));
    }

    fn keys(&self) -> String {
        let text = |value: Option<f32>| value.map_or("none".to_string(), |v| format!("{v:.1}"));
        format!(
            "extent={} speed_max={} tilt_max={}",
            text(self.extent),
            text(self.speed_max),
            text(self.tilt_max)
        )
    }
}

/// The two output streams.
///
/// Two files rather than a `row_kind` column: an epoch row and a fusion row share no
/// columns, and the epoch file's width is what `write_header` and the determinism job in CI
/// both rest on. `dyn Write` rather than two type parameters, because `Replay` would
/// otherwise carry them through every method for no gain — the writers are buffered and
/// this is one virtual call per row.
struct Sinks<'a> {
    epochs: &'a mut dyn Write,
    fusions: &'a mut dyn Write,
}

/// An IMU row as the log carries it: a rate gyroscope and an accelerometer, PX4's
/// `sensor_combined`. The interval each reading stands for is the replay's to decide, so the
/// increments the filter takes are formed where that is known.
#[derive(Clone, Copy, Default)]
struct Rates {
    gyro: AngularRate<Body>,
    accel: Acceleration<Body>,
}

/// A window sample before the rate estimator has fixed the interval it stands for.
#[derive(Clone, Copy, Default)]
struct Held {
    time: Timestamp,
    imu: Rates,
    mag: Option<MagField<Body>>,
    baro: Option<Altitude>,
    velocity: Option<Velocity<Ned>>,
}

impl Held {
    fn sample(self, dt: Seconds) -> StaticSample {
        StaticSample {
            imu: ImuSample::from_rates(self.time, self.imu.gyro, self.imu.accel, dt),
            mag: self.mag,
            baro: self.baro,
            velocity: self.velocity,
        }
    }
}

/// A start that could still be committed on its still prefix, kept while the harness
/// waits to see whether a static window arrives. See [`Replay::onset_of_motion`].
#[derive(Clone)]
struct Fallback {
    /// The replay as it stood at the onset of motion.
    at: Box<Replay>,
    /// Timestamp of the last still sample, where the prefix ends.
    t: f64,
    /// Samples in the prefix.
    still: usize,
    dt: Seconds,
    /// The sample that moved, the first the filter would propagate.
    onset: (f64, Rates),
    /// Every line since the onset.
    lines: Vec<String>,
    /// The fusion rows those lines wrote while the filter was not initialized.
    fusions: Vec<u8>,
}

/// One row of the fusion file, kept in memory: when, which source, what became of it.
#[derive(Clone, Copy, Debug)]
struct Verdict {
    t: f64,
    source: usize,
    outcome: Fusion,
}

/// Everything the loop carries between rows.
#[derive(Clone)]
struct Replay {
    filter: Eskf,
    /// Candidate static window, slid forward one sample at a time until `initialize`
    /// accepts it. A log that begins in motion simply initializes later.
    window: [Held; WINDOW],
    filled: usize,
    /// Whether every sample since the log began may still be still, which is what lets a
    /// start shorter than a full window commit at the onset of motion. See
    /// [`Replay::onset_of_motion`].
    still_since_start: bool,
    /// Samples in the longest prefix of the log the filter has read as still.
    still_prefix: usize,
    /// The still prefix and everything since, while `PATIENCE` has yet to say whether a
    /// static window will arrive instead. Boxed because it holds a whole `Replay`.
    fallback: Option<Box<Fallback>>,
    /// Set by `accumulate` when `PATIENCE` ran out with a fallback pending, for `row` to act
    /// on with the sinks the replayed rows belong in.
    fall_back_due: bool,
    /// IMU sample period, taken as the median of the first `PROBE` intervals.
    ///
    /// The log supplies the rate rather than the example assuming one. The median, not
    /// the minimum or the mean: one real log logs in bursts, with a shortest interval of
    /// 2.5 ms against a true period of 20 ms, so a minimum reads it as 400 Hz. Dropouts
    /// stretch intervals and bursts shorten them; only the middle is stable.
    interval: Option<f64>,
    probe: [f64; PROBE],
    probed: usize,
    /// Samples the window needed to cover `min_duration`, for the report.
    window_samples: usize,
    /// Most recent magnetometer reading, attached to static samples so a log that does
    /// carry mag during the window gets an observed initial heading. The bundled log does
    /// not, which is the point: yaw starts unobserved and `sigma_yaw` stays inflated.
    last_mag: Option<MagField<Body>>,
    /// The sideslip σ the course constraint is fused at after each `gnss_vel` row, or `None`
    /// for no course: from `--course`, or the `# Course sideslip` header line the simulator
    /// writes for a vehicle that points where it flies. See `sideslip_of`.
    course: Option<Radians>,
    /// An input source whose rows are skipped, `--without mag`: a vehicle without that sensor,
    /// replayed from a log that has one.
    without: Option<String>,
    /// The `--set` pairs the `Config` was built with, as given, for `set=`.
    sets: Vec<(String, String)>,
    /// Every verdict in log order, kept only where `--derive` asks for it: what the fusion
    /// file writes as text, for a reader in the same process.
    trail: Option<Vec<Verdict>>,
    /// The geodetic point the file's positions are relative to, from its `# Navigation
    /// origin` line; see `origin_of`.
    site: Option<Geodetic>,
    /// Where the GNSS antenna sits relative to the IMU, from the `# GNSS antenna` line, or
    /// zero under `--antenna zero` or for a file with none; see `antenna_of`.
    antenna: Position<Body>,
    /// Most recent barometer reading, attached to static samples the same way. This is
    /// what fixes the reference the filter's altitudes are relative to, and a log whose
    /// barometer starts after initialization leaves it unset for the whole replay.
    last_baro: Option<Altitude>,
    /// GNSS velocity awaiting the next IMU epoch, attached to exactly one static sample
    /// and not held across the epochs that follow, as `last_mag` and `last_baro` are.
    /// `StaticSample::velocity` says why: those are averaged and this is differenced, so
    /// a repeated reading would date the difference from the wrong epoch.
    pending_velocity: Option<Velocity<Ned>>,
    /// Timestamp of the previous IMU row, so `dt` comes from the log rather than from an
    /// assumed rate.
    previous_imu: Option<f64>,
    /// Timestamp of the first IMU row, so the wait for stillness can be bounded.
    first_imu: Option<f64>,
    ratios: [Option<f32>; SOURCES.len()],
    /// The innovation-consistency statistics of #5, fed from `observe`.
    consistency: Consistency,
    initialized_at: Option<f64>,
    alignment: Option<Alignment>,
    mag_at_init: bool,
    /// Whether the window itself fixed `α₀`, so `alpha0=` can say where the reference came
    /// from: a window taken at rest, or the first altitude read against the estimate once
    /// position was established. The filter reports only that it holds one.
    alpha0_from_window: bool,
    /// What `StaticWindow::noise` made of the window the filter initialized on.
    noise: Option<WindowNoise>,
    /// When `Eskf::is_aligned` first read true, in log time.
    ///
    /// The filter's own latch rather than `Validity::attitude`: the two read different bars
    /// (`ALIGNED_TILT` and `ALIGNED_HEADING` against `Config::accuracy`), and re-deriving
    /// alignment from validity would make this key move with a mission bar that
    /// `Status::Aligning` does not read. It says when the filter *claims* to have converged,
    /// not whether the attitude was good then; that needs truth, which the corpus has not
    /// got.
    aligned_at: Option<f64>,
    /// The first epoch at or after alignment where `Validity::attitude` reads false, which
    /// is the covariance growth of (16)–(22) arriving on real data.
    ///
    /// Against `Config::accuracy`, the mission bar, where `aligned_at` reads the alignment
    /// bars — so a mission bar the start never met reads `0.00` here, out of service the
    /// moment the start resolved, rather than `never`, which would claim it held all log.
    ///
    /// `aligned_at` cannot see it: `Status::Aligning` latches, deliberately
    /// (`Eskf::is_aligned`), so nothing else on this line moves when an unaided filter's
    /// attitude stops being usable — and on a corpus with no truth, growth is the only thing
    /// stage 4 of #31 produces that a replay can check at all. It reads a few seconds today
    /// on every log, because the tilt prior is 0.02 rad against a 0.052 bar and nothing
    /// shrinks a covariance; the update of (23)–(28) is what should push it to `never`.
    attitude_lost: Option<f64>,
    /// `Validity::heading` the moment initialization committed — the filter's own verdict
    /// on the window, which is not the same as `mag_at_init`: a coarse start carrying a
    /// magnetometer has observed nothing it could level a heading with.
    heading_at_init: bool,
    /// The attitude equations (5)–(7) committed, captured at that moment rather than read
    /// off the filter at the end. Since (12)–(15) landed the two differ on any log whose
    /// vehicle turns, and reading it off the end would silently report an end-of-log
    /// attitude instead — the trap `heading=` documents, which this capture is what avoids.
    attitude_at_init: Option<Attitude>,
    excursion: Excursion,
    epochs: u32,
    /// Rows written to the fusion file: every verdict the log met, whatever it was — one per
    /// `fuse_*` call, two per GNSS fix.
    fusions: u32,
    /// When the longest gap, coasted or refused, happened. The count and the size come from
    /// `diagnostics()`; the filter reads no clock, so the log timestamp is the harness's to
    /// keep.
    longest_step_at: Option<f64>,
    status: Status,
    transitions: Vec<(f64, Status)>,
    /// The truth file and the accumulators scoring against it, or `None` when none was
    /// given. One field rather than two, so that "no truth, no `score` line" is structural:
    /// there is no way to reach the accumulators without the rows that filled them.
    scoring: Option<Scoring>,
    policy: RPolicy,
}

impl Replay {
    fn new(config: Config, policy: RPolicy, scoring: Option<Scoring>) -> Result<Self, ConfigError> {
        Ok(Self {
            consistency: Consistency::new(config.gates),
            filter: Eskf::new(config)?,
            window: [Held::default(); WINDOW],
            filled: 0,
            still_since_start: true,
            still_prefix: 0,
            fallback: None,
            fall_back_due: false,
            interval: None,
            probe: [0.0f64; PROBE],
            probed: 0,
            window_samples: 0,
            last_mag: None,
            course: None,
            without: None,
            sets: Vec::new(),
            trail: None,
            site: None,
            antenna: Position::zero(),
            last_baro: None,
            pending_velocity: None,
            previous_imu: None,
            first_imu: None,
            ratios: [None; SOURCES.len()],
            initialized_at: None,
            alignment: None,
            mag_at_init: false,
            alpha0_from_window: false,
            noise: None,
            aligned_at: None,
            attitude_lost: None,
            heading_at_init: false,
            attitude_at_init: None,
            excursion: Excursion::default(),
            epochs: 0,
            fusions: 0,
            longest_step_at: None,
            status: Status::default(),
            transitions: Vec::new(),
            scoring,
            policy,
        })
    }

    fn row(&mut self, line: &str, out: &mut Sinks) -> Result<(), Box<dyn Error>> {
        // Held until `PATIENCE` decides between a static window and the still prefix: the
        // line, to replay if it is the prefix, and the fusion rows it writes, to keep if not.
        let Some(fallback) = self.fallback.as_mut() else {
            return self.record(line, out);
        };
        fallback.lines.push(line.to_string());
        let mut fusions = core::mem::take(&mut fallback.fusions);
        let result = self.record(
            line,
            &mut Sinks {
                epochs: out.epochs,
                fusions: &mut fusions,
            },
        );
        let Some(mut fallback) = self.fallback.take() else {
            return result;
        };
        fallback.fusions = fusions;
        if self.fall_back_due {
            return self.fall_back(*fallback, out);
        }
        if self.filter.is_initialized() {
            out.fusions.write_all(&fallback.fusions)?;
        } else {
            self.fallback = Some(fallback);
        }
        result
    }

    /// Replay one row of the log.
    fn record(&mut self, line: &str, out: &mut Sinks) -> Result<(), Box<dyn Error>> {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("t_s") {
            return Ok(());
        }
        let r = Record::parse(line).ok_or("malformed row")?;
        if self.without.as_deref() == Some(r.source) {
            return Ok(());
        }

        match r.source {
            "imu" => {
                let imu = Rates {
                    gyro: AngularRate::body(r.value(0)?, r.value(1)?, r.value(2)?),
                    accel: Acceleration::body(r.value(3)?, r.value(4)?, r.value(5)?),
                };
                if self.filter.is_initialized() {
                    self.propagate(r.t, imu, out)?;
                } else {
                    self.accumulate(r.t, imu)?;
                }
            }
            "gnss_pos" => {
                self.excursion.fix(r.value(0)?, r.value(1)?);
                let fix = Position::ned(r.value(0)?, r.value(1)?, r.value(2)?);
                // Variance comes from the log, per sample: a real GNSS reports its own
                // accuracy, and it degrades before it drops out.
                let variance = [r.variance(0)?, r.variance(1)?, r.variance(2)?];
                let recovered = |filter: &Eskf| {
                    let sources = filter.diagnostics().sources();
                    [sources[GNSS_POS].1.recovered, sources[GNSS_HGT].1.recovered]
                };
                let before = recovered(&self.filter);
                let outcome = self.filter.fuse_gnss_position(
                    r.taken(),
                    fix,
                    self.policy.position(variance),
                    self.antenna,
                );
                let after = recovered(&self.filter);
                // Judged on the row's own variance whatever `--r-policy` fused, so every
                // policy is scored against the same set of bad fixes.
                let judged = self.scoring.as_ref().and_then(|scoring| {
                    scoring.judge(
                        r.measured.unwrap_or(r.t),
                        fix,
                        variance,
                        self.antenna,
                        &self.filter.config().accuracy,
                    )
                });
                let [horizontal, height] = judged.map_or([None; 2], |[h, v]| [Some(h), Some(v)]);
                if let Some(scoring) = &mut self.scoring {
                    for (half, outcome, judged) in [
                        (0, outcome.horizontal, horizontal),
                        (1, outcome.height, height),
                    ] {
                        let recovery = after[half] > before[half];
                        scoring.score.fixes[half].record(outcome, judged, recovery);
                    }
                }
                self.observe(r.t, GNSS_POS, outcome.horizontal, horizontal, out)?;
                self.observe(r.t, GNSS_HGT, outcome.height, height, out)?;
            }
            "gnss_vel" => {
                let velocity = Velocity::ned(r.value(0)?, r.value(1)?, r.value(2)?);
                self.excursion.velocity(r.value(0)?, r.value(1)?);
                // Before initialization the fusion below is refused, and this is the one
                // thing in the row that a window taken in motion can still use.
                self.pending_velocity = Some(velocity);
                let outcome = self.filter.fuse_gnss_velocity(
                    r.taken(),
                    velocity,
                    self.policy
                        .velocity([r.variance(0)?, r.variance(1)?, r.variance(2)?]),
                    self.antenna,
                );
                self.observe(r.t, GNSS_VEL, outcome, None, out)?;
                if let Some(sideslip) = self.course {
                    let sideslip = HeadingNoise::from_sigma(sideslip.as_radians());
                    let outcome = self.filter.fuse_course(r.taken(), sideslip);
                    self.observe(r.t, COURSE, outcome, None, out)?;
                }
            }
            "baro" => {
                let altitude = Altitude::from_meters(r.value(0)?);
                self.last_baro = Some(altitude);
                let outcome = self.filter.fuse_baro_altitude(
                    r.taken(),
                    altitude,
                    AltitudeNoise::from_variance(r.variance(0)?),
                );
                self.observe(r.t, BARO, outcome, None, out)?;
            }
            "mag" => {
                let field = MagField::body(r.value(0)?, r.value(1)?, r.value(2)?);
                self.last_mag = Some(field);
                let outcome = self.filter.fuse_mag_heading(
                    r.taken(),
                    field,
                    HeadingNoise::from_variance(r.variance(0)?),
                );
                self.observe(r.t, MAG, outcome, None, out)?;
            }
            "gnss_yaw" => {
                let outcome = self.filter.fuse_gnss_heading(
                    r.taken(),
                    Radians::from_radians(r.value(0)?),
                    self.policy.heading(r.variance(0)?),
                );
                self.observe(r.t, GNSS_YAW, outcome, None, out)?;
            }
            other => return Err(format!("unknown source `{other}`").into()),
        }
        Ok(())
    }

    /// Fill the static window, sliding it forward until the filter accepts one.
    ///
    /// Before initialization the filter refuses measurements with
    /// [`Fusion::NotInitialized`], so aiding rows in this stretch of the log are simply
    /// recorded and dropped.
    fn accumulate(&mut self, t: f64, imu: Rates) -> Result<(), Box<dyn Error>> {
        self.first_imu.get_or_insert(t);
        let previous = self.previous_imu;
        self.probe_rate(t);
        let time = Timestamp::from_secs_f64(t);
        // A sample not after the last is one `predict` would refuse as `InvalidStep`, and a
        // window holding it is one `initialize` refuses: it describes no time, so it is left
        // out, as the filter leaves it out once running.
        if self.filled > 0 && self.window[self.filled - 1].time >= time {
            return Ok(());
        }
        let sample = Held {
            time,
            imu,
            mag: self.last_mag,
            baro: self.last_baro,
            velocity: self.pending_velocity.take(),
        };
        self.push_to_window(sample);

        // One interval is needed before the window can be sized at all.
        let Some(interval) = self.interval else {
            return Ok(());
        };
        let needed = self.samples_needed(interval)?;
        let dt = Seconds::from_secs(interval as f32);
        if self.still_since_start && self.filled <= needed {
            // Nothing has slid out yet, so the window is every sample since the log began.
            let alignment = self.alignment_of(0..self.filled, dt)?;
            if let Some(still) = self.onset_of_motion(alignment)
                && let Some(previous) = previous
            {
                self.fallback = Some(Box::new(Fallback {
                    at: Box::new(self.clone()),
                    t: previous,
                    still,
                    dt,
                    onset: (t, imu),
                    lines: Vec::new(),
                    fusions: Vec::new(),
                }));
            }
        }
        if self.filled < needed {
            return Ok(());
        }
        let window = self.filled - needed..self.filled;
        let alignment = self.alignment_of(window.clone(), dt)?;
        if !self.worth_committing(t, alignment) {
            return Ok(());
        }
        if !alignment.is_static() && self.fallback.is_some() {
            // `row` rewinds, since it holds the sinks the buffered rows belong in.
            self.fall_back_due = true;
            return Ok(());
        }
        self.commit(t, window, dt)
    }

    /// Initialize on the still prefix a [`Fallback`] kept, then replay what came after it.
    ///
    /// The state is the one captured at the onset of motion, so everything the waiting
    /// did — the window it slid, the fusion rows it refused as not initialized, the
    /// counters those rows moved — is discarded rather than undone.
    fn fall_back(&mut self, fallback: Fallback, out: &mut Sinks) -> Result<(), Box<dyn Error>> {
        let Fallback {
            at,
            t,
            still,
            dt,
            onset: (onset_t, onset_imu),
            lines,
            ..
        } = fallback;
        *self = *at;
        self.commit(t, 0..still, dt)?;
        // The sample that moved is the first the filter propagates.
        self.previous_imu = Some(t);
        self.propagate(onset_t, onset_imu, out)?;
        for line in &lines {
            self.row(line, out)?;
        }
        Ok(())
    }

    /// Release what a pending [`Fallback`] was holding when the log ends before
    /// `PATIENCE` decides it: the log never initialized, and its rows are written as the
    /// refusals they were.
    fn finish(&mut self, out: &mut Sinks) -> io::Result<()> {
        match self.fallback.take() {
            Some(fallback) => out.fusions.write_all(&fallback.fusions),
            None => Ok(()),
        }
    }

    /// Initialize on `self.window[range]`, the window ending at the sample stamped `t`.
    fn commit(
        &mut self,
        t: f64,
        range: core::ops::Range<usize>,
        dt: Seconds,
    ) -> Result<(), Box<dyn Error>> {
        self.window_samples = range.len();
        let window = self.window(range.clone(), dt)?;
        let alignment = self.filter.initialize(&window)?;
        self.noise = window.noise(self.filter.config());
        self.alignment = Some(alignment);
        self.initialized_at = Some(t);
        // The filter's own rule: one magnetometer sample anywhere in the window observes
        // heading, and none at all leaves yaw a prior until a heading is fused.
        self.mag_at_init = self.window[range].iter().any(|held| held.mag.is_some());
        self.alpha0_from_window = self.filter.baro_reference().is_some();
        let state = self.filter.state();
        self.note_alignment(t, self.filter.is_aligned(), state.validity);
        self.heading_at_init = state.validity.heading;
        self.attitude_at_init = Some(state.attitude);
        self.excursion.attitude(state.attitude);
        self.status = state.status;
        self.transitions.push((t, self.status));
        Ok(())
    }

    /// Record one IMU interval toward the rate estimate, and fix the rate as the median
    /// once `PROBE` of them are in. See [`Replay::interval`] for why the median.
    fn probe_rate(&mut self, t: f64) {
        let step = self.previous_imu.replace(t).map(|previous| t - previous);
        if let Some(step) = step.filter(|step| *step > 0.0)
            && self.probed < PROBE
        {
            self.probe[self.probed] = step;
            self.probed += 1;
            if self.probed == PROBE {
                let mut sorted = self.probe;
                sorted.sort_by(f64::total_cmp);
                self.interval = Some(sorted[PROBE / 2]);
            }
        }
    }

    /// `self.window[range]` as the filter takes it, every sample standing for `dt`.
    ///
    /// `dt` is the interval the rate estimator fixed rather than each row's own step, which
    /// on a burst log is zero as often as not. The harness slides a buffered window, which a
    /// `StaticWindow` cannot do, so each candidate is folded in anew.
    fn window(
        &self,
        range: core::ops::Range<usize>,
        dt: Seconds,
    ) -> Result<StaticWindow, SampleRefusal> {
        let mut window = StaticWindow::new();
        window.try_extend(self.window[range].iter().map(|held| held.sample(dt)))?;
        Ok(window)
    }

    /// What the filter would make of `self.window[range]`.
    fn alignment_of(
        &self,
        range: core::ops::Range<usize>,
        dt: Seconds,
    ) -> Result<Alignment, InitError> {
        self.filter.alignment_of(&self.window(range, dt)?)
    }

    /// Append a sample, dropping the oldest once the window is full.
    fn push_to_window(&mut self, sample: Held) {
        if self.filled == WINDOW {
            self.window.copy_within(1.., 0);
            self.window[WINDOW - 1] = sample;
        } else {
            self.window[self.filled] = sample;
            self.filled += 1;
        }
    }

    /// Samples it takes to span `Initialization::min_duration` at this interval.
    fn samples_needed(&self, interval: f64) -> Result<usize, String> {
        let required = f64::from(self.filter.config().init.min_duration.as_secs());
        let needed = (required / interval).ceil() as usize;
        if needed > WINDOW {
            return Err(format!(
                "{required} s at {:.0} Hz needs {needed} samples; the window holds {WINDOW}",
                1.0 / interval
            ));
        }
        Ok(needed)
    }

    /// The wait-for-stillness policy: commit a static window at once, a coarse one only
    /// after `PATIENCE` seconds of log have gone by without a static one.
    ///
    /// Prefer a static window, but do not wait forever for one: a log that begins in
    /// motion, or a vehicle that never gets a quiet moment, should still fly. This is the
    /// policy an application has to choose, which is why the filter reports the alignment
    /// rather than deciding this itself.
    fn worth_committing(&self, t: f64, alignment: Alignment) -> bool {
        let waited = t - self.first_imu.unwrap_or(t);
        alignment.is_static() || waited >= PATIENCE
    }

    /// The fallback half of the policy: a vehicle still since the log began that starts
    /// to move before a full window has passed leaves a still prefix behind, and if no
    /// static window arrives within `PATIENCE` the start is that prefix rather than a
    /// window taken in motion. A static window still wins when one comes: a vehicle
    /// bumped 0.33 s in and then left alone is a static start, not a 0.33 s one.
    ///
    /// `alignment` is the prefix from the first sample through the newest. Returns how
    /// many samples the prefix keeps — all but the newest — when the newest is the one that
    /// turned it `NotStationary`, and `None` otherwise: the prefix before it must itself
    /// have been read still, so a log that moved before the rate was fixed commits
    /// nothing here. Stillness is a peak test (`init::at_rest`), so once a prefix has
    /// moved no longer one is still, and the check stops. The rate probe is therefore the
    /// floor on how short a start this keeps.
    ///
    /// What an application does when it arms a vehicle that has been sitting for less
    /// than `Initialization::min_duration`: the filter reports the start as
    /// `Coarse::WindowTooShort`, and keeps what a still window establishes.
    fn onset_of_motion(&mut self, alignment: Alignment) -> Option<usize> {
        match alignment {
            Alignment::Coarse(Coarse::NotStationary { .. }) => {
                self.still_since_start = false;
                (self.still_prefix > 0 && self.still_prefix == self.filled - 1)
                    .then_some(self.still_prefix)
            }
            _ => {
                self.still_prefix = self.filled;
                None
            }
        }
    }

    fn propagate(&mut self, t: f64, imu: Rates, out: &mut Sinks) -> io::Result<()> {
        if self.previous_imu.replace(t).is_some() {
            // A rate stands for the time since the last sample the filter took, which is the
            // step the filter measures itself, so the two cannot disagree. The filter decides
            // what is too long, not the example.
            let time = Timestamp::from_secs_f64(t);
            let dt = time.since(self.filter.time().unwrap_or_default());
            let worst_before = self.filter.diagnostics().propagation.longest_gap;
            let propagated = self
                .filter
                .predict(ImuSample::from_rates(time, imu.gyro, imu.accel, dt))
                .is_propagated();
            // Timestamp the step that set a new worst, which is the one the filter kept.
            // Comparing to `dt` instead would also match a later step that merely ties it,
            // and move the report's timestamp off the gap the size belongs to.
            if !propagated && self.filter.diagnostics().propagation.longest_gap != worst_before {
                self.longest_step_at = Some(t);
            }
        }
        let state = self.filter.state();
        self.note_alignment(t, self.filter.is_aligned(), state.validity);
        self.excursion.attitude(state.attitude);
        if state.status != self.status {
            self.status = state.status;
            self.transitions.push((t, state.status));
        }
        self.epochs += 1;
        // Scored from the same `state` the row is written from, and after a coasted or refused
        // step as well as a propagated one: that state is what the filter published. Both reads
        // sit inside the `if let` so a replay with no truth file — which is every CI run and
        // every corpus log — does no work for a feature it is not using.
        if let Some(scoring) = &mut self.scoring {
            scoring.epoch(
                t,
                // A period: see `Truth::at`.
                self.interval.unwrap_or(TRUTH_TOLERANCE),
                &state,
                self.filter.covariance(),
                self.filter.attitude_variance(),
                &self.filter.config().accuracy,
            );
        }
        self.write_row(t, state, out)
    }

    /// Carry the test ratio into the epoch row, and record the fusion itself.
    ///
    /// Counting is the filter's job — `AGENTS.md`, one statistic, one implementation — so
    /// everything else this row did is read back out of `diagnostics()` at the end. What
    /// the fusion file adds is the one thing totals cannot reconstruct: which measurement,
    /// at what time, met what verdict.
    fn observe(
        &mut self,
        t: f64,
        source: usize,
        outcome: Fusion,
        judged: Option<Judged>,
        out: &mut Sinks,
    ) -> io::Result<()> {
        self.ratios[source] = outcome.test_ratio();
        self.fusions += 1;
        if let Some(trail) = &mut self.trail {
            trail.push(Verdict { t, source, outcome });
        }
        // `ν` and the diagonal of `S` as the filter published them, and only for a call the
        // gate judged: a refusal or an adoption leaves the last update's values in place,
        // which would be written against a measurement they do not describe. Never computed
        // here — `AGENTS.md`, one statistic, one implementation.
        let innovation = outcome
            .test_ratio()
            .and_then(|_| self.filter.diagnostics().sources()[source].1.innovation);
        // The consistency statistics take the same rows the columns do, and the predicate is
        // the innovation rather than the ratio, which is a narrower claim than it looks. An
        // adoption already fails both — `Fusion::Reset` publishes no ratio — so the two agree
        // on every path this harness drives. They part on `fuse_gnss_geodetic`, which spends
        // its first fix placing the origin and reports `Accepted { test_ratio: 0.0 }` with
        // nothing beside it: a population keyed on the ratio would average in an `ε = 0` that
        // no update produced. Nothing here calls it, so this forecloses that rather than
        // fixing it, and the reason to choose it anyway is that `ε` is defined on the
        // innovation and the ratio is a proxy for having one.
        if let (Some(ratio), Some(innovation)) = (outcome.test_ratio(), innovation.as_ref()) {
            self.consistency.record(t, source, ratio, innovation);
        }
        writeln!(
            out.fusions,
            "{t:.4},{},{},{},{},{}",
            SOURCES[source],
            innovation_fields(innovation),
            match outcome.test_ratio() {
                Some(ratio) => format!("{ratio:.4}"),
                None => String::new(),
            },
            verdict(outcome),
            // Only where the gate or an adoption decided: a refusal judged nothing.
            match judged {
                Some(judged) if decided(outcome) => {
                    if judged.bad { "bad" } else { "good" }
                }
                _ => "",
            },
        )
    }

    /// Sum one `SourceHealth` count over every source.
    ///
    /// A count of verdicts, not of measurements: a GNSS fix is two sources, `gnss_pos` and
    /// `gnss_hgt`, so one refused, rejected or adopted whole counts twice here. The
    /// per-source keys are where the two halves are told apart.
    fn total(&self, count: fn(&SourceHealth) -> u32) -> u32 {
        self.filter
            .diagnostics()
            .sources()
            .iter()
            .map(|(_, health)| count(health))
            .sum()
    }

    /// Verdicts the gate turned down, over every source; see [`Replay::total`].
    fn rejections(&self) -> u32 {
        self.total(|health| health.rejected)
    }

    /// The same count split per source, as ` rejected_<source>=<n>` pairs.
    ///
    /// The total alone averages two sources moving in opposite directions, which is the
    /// reading `ba=` and `bg=` were split to avoid: fusing velocity halved one bias and
    /// worsened the other, and one key would have reported a clean win. A gate makes the
    /// same shape of claim — `a299e722` turns down 266 velocity solutions, and an altitude
    /// its barometer gate also turned down would arrive on that line as a larger number
    /// with nothing saying which source grew.
    ///
    /// The total stays beside these rather than being replaced by them, and is not
    /// redundant: `data/expect.sh` fails a key named in the expectations and absent from
    /// the line, but never a key on the line that no expectation names, so a source whose
    /// own key nobody added to a manifest entry goes unwatched. The sum is what notices.
    ///
    /// Names come from `SOURCES`, so a key reads as its own `r_<source>` column and the
    /// fusion rows of that source spell it the same way; `Diagnostics` order is what pairs
    /// the two lists, as it does for `RATIOS`.
    fn rejections_by_source(&self) -> String {
        SOURCES
            .iter()
            .zip(self.filter.diagnostics().sources())
            .map(|(name, (_, health))| format!(" rejected_{name}={}", health.rejected))
            .collect()
    }

    /// What the static window measured, `StaticWindow::noise` of the window the filter started
    /// on: `noise_gyro=` and `noise_accel=` the worst axis's white-noise density, rad s⁻¹/√Hz
    /// and m s⁻²/√Hz, and `noise_baro=` the barometer readings' σ in metres.
    ///
    /// Floors, not the noise to configure (`WindowNoise`), and pinned to say how far below
    /// `ImuNoise::default()` each airframe's still sensors sit. The simulator's scenarios
    /// check the other way: `data/scenarios.txt` bounds them about the densities the
    /// simulator injected. `none` for a window that moved or held too few readings, the
    /// `alpha0=none` convention. Fixed decimals, for `data/expect.sh`.
    fn noise_keys(&self) -> String {
        let noise = self.noise.as_ref();
        format!(
            "noise_gyro={} noise_accel={} noise_baro={}",
            noise.map_or("none".into(), |n| format!("{:.7}", n.worst_gyro_white())),
            noise.map_or("none".into(), |n| format!("{:.6}", n.worst_accel_white())),
            noise
                .and_then(|n| n.baro)
                .map_or("none".into(), |b| format!("{:.4}", b.variance().sqrt())),
        )
    }

    /// The innovation-consistency keys of #5, as ` key=value` pairs.
    ///
    /// Grouped by statistic rather than by source, so the manifest reads as four claims
    /// across the corpus rather than four unrelated numbers per sensor. Each family answers
    /// something `rejected_` cannot: `nis_` whether the reported `R` is the size the residuals
    /// say it is, `nis_over95_` whether the tail agrees with the mean that the gate's own
    /// percentile is chosen against, `nu_` whether a standing offset is being fused as noise,
    /// `acf1_` whether successive innovations are independent the way (24) assumes.
    ///
    /// `none` where a source gated nothing, and for `acf1_` below two samples — the
    /// `nees_text` convention, a word rather than a zero, because a zero is a reading and this
    /// is the absence of one. A word also keeps a manifest range from clearing itself on an
    /// empty source: `data/expect.sh` refuses a non-number where a bound is asked for.
    ///
    /// Fixed decimals throughout and never scientific notation, which the comparator's number
    /// pattern does not admit — a `1e-7` would arrive there as `NOT A NUMBER`.
    fn consistency_keys(&self) -> String {
        let per_source = |name: &str, places: usize, of: &dyn Fn(usize) -> Option<f64>| -> String {
            SOURCES
                .iter()
                .enumerate()
                .map(|(source, spelling)| {
                    format!(" {name}_{spelling}={}", measured(of(source), places))
                })
                .collect()
        };

        let nis = per_source("nis", 4, &|source| self.consistency.nis(source));
        let over95 = per_source("nis_over95", 4, &|source| {
            self.consistency.over95_fraction(source)
        });
        let acf1 = per_source("acf1", 4, &|source| {
            self.consistency.autocorrelation(source)
        });
        let nu: String = SOURCES
            .iter()
            .enumerate()
            .flat_map(|(source, spelling)| {
                AXES[source].iter().enumerate().map(move |(axis, name)| {
                    let mean = self.consistency.axes[source][axis].nu_mean();
                    format!(" nu_{spelling}_{name}={}", measured(mean, 6))
                })
            })
            .collect();

        format!("{nis}{over95}{nu}{acf1}")
    }

    /// Keep the first moment the filter reported itself aligned, and only the first.
    ///
    /// Taken at the commit as well as at every epoch: a window that carried a magnetometer
    /// leaves the filter aligned the instant it closes, and that is zero seconds rather
    /// than one `dt`.
    fn note_alignment(&mut self, t: f64, aligned: bool, validity: Validity) {
        if self.aligned_at.is_none() && aligned {
            self.aligned_at = Some(t);
        }
        // Only after alignment, and only the first time: before it, "not valid" is the
        // start the filter has not finished, which `aligned_at` already reports.
        if self.aligned_at.is_some() && self.attitude_lost.is_none() && !validity.attitude() {
            self.attitude_lost = Some(t);
        }
    }

    /// Seconds from the end of the initialization window to the first valid attitude, or
    /// `never`.
    ///
    /// `never` stays a word rather than a large number. A vehicle carrying no magnetometer
    /// never aligns by construction — stillness observes tilt and never yaw — and that is
    /// a correct answer, not a slow one.
    fn aligned_after(&self) -> String {
        match (self.initialized_at, self.aligned_at) {
            (Some(window_closed), Some(aligned)) => format!("{:.2}", aligned - window_closed),
            _ => "never".to_string(),
        }
    }

    /// Seconds from the end of the initialization window to the first epoch whose attitude
    /// stopped being valid, or `never`.
    ///
    /// `never` covers two different things, as `aligned_at`'s does: a filter that never
    /// aligned has nothing to lose, and one whose attitude held for the whole log has lost
    /// nothing. The pair reads unambiguously — `aligned_at=never attitude_lost=never` is the
    /// first, any number in `aligned_at` with `attitude_lost=never` the second.
    fn attitude_lost_after(&self) -> String {
        match (self.initialized_at, self.attitude_lost) {
            (Some(window_closed), Some(lost)) => format!("{:.2}", lost - window_closed),
            _ => "never".to_string(),
        }
    }

    /// Verdicts the filter could not reach at all, over every source; see [`Replay::total`].
    ///
    /// The gate's verdict is [`Replay::rejections`]; this is everything that never reached
    /// it — a variance of zero or less, a NaN in the measurement, an altitude with no
    /// reference to be relative to. Each one is aiding that silently is not there, which
    /// is what `alpha0=` was added to notice for one source and one variant.
    ///
    /// `Diagnostics` resets when a window commits, so on a log that initializes this
    /// counts only what happened afterwards: the measurements refused before then were
    /// refused for having arrived early, which is the caller's startup order and not the
    /// flight. A log that never initializes has nothing else to report, and this is that
    /// count instead.
    fn discarded(&self) -> u32 {
        self.total(|health| health.refused)
    }

    /// Measurements adopted outright: because initialization left nothing to fuse them
    /// against, or to recover from lockout. A GNSS fix adopted whole counts twice; see
    /// [`Replay::total`].
    fn resets(&self) -> u32 {
        self.total(|health| health.adopted)
    }

    /// The part of [`resets`](Self::resets) that recovered from gate lockout, a rejected
    /// measurement adopted after `Config::recovery`'s timeout. Pinned apart because it is a
    /// finding against the covariance where `resets` is a property of the start: a log that
    /// recovers on good data has a `P` too small for its sources.
    fn recoveries(&self) -> u32 {
        self.total(|health| health.recovered)
    }

    /// What the vehicle's own acceleration was over the initialization window, as the
    /// filter measured it from the GNSS velocities the window carried. `None` from a
    /// start the filter did not classify as moving, which does not report one.
    fn inertial_accel(&self) -> Option<Acceleration<Ned>> {
        match self.alignment {
            Some(Alignment::Coarse(Coarse::NotStationary { inertial_accel, .. })) => inertial_accel,
            _ => None,
        }
    }

    /// The attitude initialization committed, as roll, pitch and yaw in degrees, rounded
    /// to what the `summary` line prints. Zero everywhere if the log never initialized.
    ///
    /// `roll0`, `pitch0` and `yaw0` are the only keys on that line that look at attitude,
    /// so they are what would notice a sign inverted in the down-positive convention, the
    /// levelling dropped out of (6), or a declination that stopped reaching the filter.
    ///
    /// Rounded here rather than by the format string so that an angle rounding to zero
    /// from below prints `0.00` and not `-0.00`: the same angle either way, and the
    /// manifest matches these as substrings, so the sign alone would read as a moved
    /// expectation. IEEE addition makes `-0.0 + 0.0` positive zero, which is the whole of
    /// the correction.
    fn angles_at_init(&self) -> (f32, f32, f32) {
        let attitude = self.attitude_at_init.unwrap_or_default();
        let (roll, pitch, yaw) = attitude.euler_angles();
        let rounded = |angle: f32| (angle.to_degrees() * 100.0).round() / 100.0 + 0.0;
        (rounded(roll), rounded(pitch), rounded(yaw))
    }

    fn write_row(&self, t: f64, state: State, out: &mut Sinks) -> io::Result<()> {
        let out = &mut out.epochs;
        write!(out, "{t:.4},{:?}", state.status)?;
        for (_, value) in ESTIMATE {
            write!(out, ",{:.6}", value(&state))?;
        }
        let covariance = self.filter.covariance();
        for (component, _) in SIGMAS {
            write!(out, ",{:.6}", covariance.variance(component).sqrt())?;
        }
        let attitude = self.filter.attitude_variance();
        for (_, variance) in ATTITUDE_SIGMAS {
            write!(out, ",{:.6}", variance(attitude).sqrt())?;
        }
        for ratio in self.ratios {
            match ratio {
                Some(ratio) => write!(out, ",{ratio:.4}")?,
                None => write!(out, ",")?,
            }
        }
        writeln!(out)
    }

    /// Print what happened. `summary` is the only line anything parses.
    fn report(&self, input: &Path, output: &Path, fusions: &Path) {
        self.report_initialization(input, output, fusions);
        self.report_steps();
        self.report_transitions();
        self.report_summary();
        self.report_validity();
        self.report_sources();
        self.report_score();
    }

    /// Accuracy against truth, and the `score` line — both silent without a truth file.
    ///
    /// Last, because it is the only section that is usually absent: the corpus has no truth,
    /// and a reader of those logs should not have to scroll past a gap where it would be.
    fn report_score(&self) {
        let Some(scoring) = &self.scoring else { return };
        let score = &scoring.score;
        println!(
            "\naccuracy against {} ({} epochs scored)",
            scoring.path.display(),
            score.scored
        );
        if scoring.unmatched() > 0 {
            // Not a rounding question: the two files are written together by one run of
            // `examples/simulate.rs`, so an epoch with no truth row means they were not.
            println!(
                "  {} epochs had no truth row — the log and the truth are from different runs",
                scoring.unmatched()
            );
        }
        if score.scored == 0 {
            // Nothing below would mean anything, and printing it as zeros would read as a
            // perfect filter rather than an unmeasured one.
            println!("  nothing scored\n\n{}", score.line());
            return;
        }
        println!(
            "  position     RMSE {:.3} m horizontal, {:.3} m vertical, worst {:.3} m",
            score.rms(score.position_horizontal),
            score.rms(score.position_vertical),
            score.position_horizontal_max,
        );
        println!("  velocity     RMSE {:.3} m/s", score.rms(score.velocity));
        println!(
            "  attitude     RMS {:.3} deg tilt, {:.3} deg yaw",
            score.rms(score.tilt).to_degrees(),
            score.rms(score.yaw).to_degrees(),
        );
        println!(
            "  bias         RMS {} m/s^2 accelerometer, {} rad/s gyroscope",
            Score::bias_text(score.accel_bias, score.accel_bias_scored, 5),
            Score::bias_text(score.gyro_bias, score.gyro_bias_scored, 6),
        );
        println!(
            "  consistency  {:.1}% of axis-epochs within 3 sigma; NEES/dof {} pos, \
             {} vel, {} att",
            100.0 * score.in3s(),
            score.nees_text(0),
            score.nees_text(1),
            score.nees_text(2),
        );
        if score.false_valid() > 0 {
            // Per quantity, because the key is a sum and the sum does not say which claim
            // failed — and against a 1σ bar, so a share of these is expected rather than
            // wrong. See the module docs.
            println!(
                "  {} quantity-epochs claimed usable while the error exceeded \
                 Config::accuracy:",
                score.false_valid()
            );
            for (quantity, count) in QUANTITIES.iter().zip(score.false_valid) {
                if count > 0 {
                    println!("    {:<13} {count}", quantity.name);
                }
            }
        }
        println!("\n{}", score.line());
    }

    /// The input, the output, and how initialization went.
    fn report_initialization(&self, input: &Path, output: &Path, fusions: &Path) {
        println!("fusion-nav replay — no filtering is performed\n");
        println!("in   {}", input.display());
        println!("out  {}", output.display());
        println!("     {} ({} fusions)", fusions.display(), self.fusions);

        match self.initialized_at {
            Some(t) => {
                let heading = if self.mag_at_init {
                    "magnetometer present, heading observed"
                } else {
                    "no magnetometer in the window, heading unobserved until one is fused"
                };
                let reference = match self.filter.baro_reference() {
                    Some(reference) => format!(
                        "barometric reference {:.2} m, altitudes are relative to it",
                        reference.as_meters()
                    ),
                    None => "no barometric reference from the window (no barometer in it, or \
                             it was taken in motion); the first altitude once position is \
                             established reads one from the estimate"
                        .to_string(),
                };
                let alignment = match self.alignment {
                    Some(Alignment::Static) => "static alignment".to_string(),
                    Some(Alignment::Coarse(Coarse::NotStationary {
                        peak_gyro,
                        peak_accel_deviation,
                        span,
                        inertial_accel,
                    })) => format!(
                        "COARSE: peak gyro {:.3} rad/s over {:.2} s, peak |a|-g {:.3} m/s^2\n  {}",
                        peak_gyro.as_rad_per_s(),
                        span.as_secs(),
                        peak_accel_deviation.as_m_per_s2(),
                        match inertial_accel {
                            Some(accel) => format!(
                                "GNSS puts the vehicle's own acceleration at {:.3} m/s^2 \
                                 ({:.2}, {:.2}, {:.2} NED), which in-motion levelling would \
                                 subtract",
                                Vector3::from(accel.to_array()).norm(),
                                accel.x(),
                                accel.y(),
                                accel.z(),
                            ),
                            None => "no two GNSS velocities in the window, so none of the \
                                     specific force is accounted for"
                                .to_string(),
                        },
                    ),
                    Some(Alignment::Coarse(Coarse::WindowTooShort { .. })) => {
                        "COARSE: window too short".to_string()
                    }
                    Some(Alignment::Seeded) => "seeded".to_string(),
                    // Unreachable while this harness only ever calls `initialize`, and
                    // labelled rather than assumed away: `summary` says `align=none` here.
                    None => "no alignment recorded".to_string(),
                };
                let interval = self.interval.unwrap_or(f64::NAN);
                println!(
                    "\ninitialized at {t:.2} s from {:.2} s of stillness \
                     ({} samples at {:.0} Hz)\n  {alignment}\n  {heading}\n  {reference}",
                    self.window_samples as f64 * interval,
                    self.window_samples,
                    1.0 / interval,
                );
            }
            None => println!("\nnever initialized: no window covering min_duration"),
        }
    }

    /// Epochs written, measurements rejected or adopted, and steps refused.
    fn report_steps(&self) {
        let propagation = self.filter.diagnostics().propagation;
        println!(
            "\n{} epochs written, {} measurements rejected",
            self.epochs,
            self.rejections()
        );
        if self.discarded() > 0 {
            // Which source, and why, is in `report_sources` — `SourceHealth` keeps a count
            // and the last refusal rather than a tally per variant, so per source is the
            // breakdown that exists.
            println!(
                "{} never reached the gate at all: a variance of zero or less, a NaN, \
                 an altitude with no reference, or a time the filter cannot place — per \
                 source below",
                self.discarded()
            );
        }
        if self.resets() > self.recoveries() {
            println!(
                "{} adopted outright: initialization established no such quantity to fuse \
                 them against",
                self.resets() - self.recoveries()
            );
        }
        if self.recoveries() > 0 {
            println!(
                "{} adopted to recover from gate lockout — per source below",
                self.recoveries()
            );
        }
        if propagation.refused_invalid > 0 {
            println!(
                "{} steps refused as zero, negative or NaN — with rows sorted by \
                 timestamp, in practice duplicates",
                propagation.refused_invalid
            );
        }
        // No log in the corpus carries a non-finite IMU row, so this line is silent on all
        // of them; it is here for the next converted log, where a NaN reaching `predict`
        // is a converter bug and otherwise looks like the filter quietly not advancing.
        if propagation.refused_not_finite > 0 {
            println!(
                "{} steps refused as carrying a NaN or an infinity — the IMU or the \
                 conversion, not the timing",
                propagation.refused_not_finite
            );
        }
        if let Some(worst) = propagation.longest_gap {
            let at = self.longest_step_at.unwrap_or(f64::NAN);
            println!(
                "{} propagation steps coasted and {} refused as longer than {} s, worst {:.3} s \
                 at {at:.2} s\n  a gap is usually the logger missing messages, not the IMU \
                 stopping",
                propagation.coasted,
                propagation.refused_too_long,
                self.filter.config().max_predict_dt.as_secs(),
                worst.as_secs(),
            );
        }
    }

    /// Status transitions, the first few of them.
    fn report_transitions(&self) {
        // A real log can flap hundreds of times; print enough to see the pattern.
        const SHOWN: usize = 12;
        println!("\nstatus ({} transitions)", self.transitions.len());
        for (t, status) in self.transitions.iter().take(SHOWN) {
            println!("  {t:>6.2} s  {status:?}");
        }
        if let Some(rest) = self.transitions.len().checked_sub(SHOWN)
            && rest > 0
        {
            println!("  … {rest} more");
        }
    }

    /// One machine-readable line, which `data/fetch.sh --check` asserts against the
    /// expectations recorded in the manifest.
    fn report_summary(&self) {
        println!("\n{}", self.summary());
    }

    /// The `summary` line itself.
    ///
    /// Built rather than printed, so the tests below can assert on the keys the manifest
    /// pins. Every number the corpus is guarded by passes through here, and until it
    /// returned a value nothing could check one except by replaying a log and reading the
    /// expectation it was supposed to be checking.
    fn summary(&self) -> String {
        let state = self.filter.state();
        let (roll0, pitch0, yaw0) = self.angles_at_init();
        format!(
            "summary rate={:.0} window={} {} align={} an={} alpha0={} heading={} \
             roll0={roll0:.2} pitch0={pitch0:.2} yaw0={yaw0:.2} declination={:.2} declination_model={} antenna={} resets={} \
             recovered={} aligned_at={} attitude_lost={} r_policy={} recovery={} course={} without={} set={} rejected={}{} discarded={} coasted={} refused={} \
             invalid={} floored={} epochs={}{} {} \
             degraded_s={:.2} dead_reckoning_s={:.2} transitions={} status={:?}",
            self.interval.map_or(0.0, |interval| 1.0 / interval),
            self.window_samples,
            // What the vehicle did, so a note's "past a kilometre" is a pin. See `Excursion`.
            self.excursion.keys(),
            match self.alignment {
                Some(Alignment::Static) => "static",
                Some(Alignment::Coarse(Coarse::WindowTooShort { .. })) => "short",
                Some(Alignment::Coarse(Coarse::NotStationary { .. })) => "coarse",
                Some(Alignment::Seeded) => "seeded",
                None => "none",
            },
            // `ā_n` of (5′): only a moving start reports one, and only when its window
            // carried two dated GNSS velocities. `none` therefore covers both "the start
            // was static" and "the window had no GNSS in it", which is why it is pinned
            // rather than derived — on the one log that starts in motion, `cd7e0001`, it is the only
            // key that would notice the velocity disappearing out of the window, and
            // in-motion levelling has nothing to correct with when it does.
            if self.inertial_accel().is_some() {
                "measured"
            } else {
                "none"
            },
            // Where the barometric reference came from: the window, or the estimate once a
            // fix established position. Pinned here because nothing else in this line would
            // notice barometric aiding vanishing, nor a coarse log's reference moving from
            // one source to the other.
            match (self.filter.baro_reference(), self.alpha0_from_window) {
                (Some(_), true) => "window",
                (Some(_), false) => "estimate",
                (None, _) => "none",
            },
            // `Validity::heading` as initialization left it, not as the log ended.
            // Nothing pins the rotation about gravity except a magnetometer, and the
            // covariance would report it good either way — `Initialization::sigma_yaw` is
            // 0.35 rad inside an `Accuracy::heading` of 0.52 — so what is worth pinning is
            // the verdict on the window. At the end of the log this says only that some
            // magnetic heading was fused at some point, which `transitions` already
            // notices.
            if self.heading_at_init {
                "valid"
            } else {
                "invalid"
            },
            // Degrees, from the log's header (`declination_of`): a log converted before the
            // converter wrote one reads 0.00, which is a site nobody named.
            self.filter.magnetic_declination().as_radians().to_degrees(),
            // The crate's magnetic model at the log's site, beside the header's value: on a log
            // whose EKF2 read PX4's table at its first fix, the two are one table read by two
            // ports at one point, and the manifest pins them equal. `none` without a site.
            self.site
                .and_then(Geodetic::magnetic_declination)
                .map_or_else(
                    || "none".to_string(),
                    |d| format!("{:.2}", d.as_radians().to_degrees())
                ),
            // The lever arm every GNSS fix was fused with, forward, right, down in metres: a
            // header that stopped reaching the filter reads `0.00,0.00,0.00` here and moves
            // nothing else a reader would think to check.
            {
                let [forward, right, down] = self.antenna.to_array();
                format!("{forward:.2},{right:.2},{down:.2}")
            },
            self.resets(),
            self.recoveries(),
            // When the filter first called its own attitude usable. It is what settled
            // `Accuracy`'s attitude defaults off the corpus the way replay settled
            // `SourceHealth::timeout`: with the bars equal to the priors they were
            // compared against, every static log read 0.00 and lost the claim one step
            // later, which is the reading that produced the numbers those defaults now
            // carry.
            self.aligned_after(),
            // What the covariance growth of (16)–(22) does on a log with no truth. Pinned
            // because it is the only key that moves when propagation's uncertainty model
            // changes, and because the update of (23)–(28) should push it to `never`.
            self.attitude_lost_after(),
            self.policy.name(),
            if self.filter.config().recovery == Recovery::OFF {
                "off"
            } else {
                "on"
            },
            // Choices rather than counts, as `r_policy=` is: whether the course constraint was
            // fused and at what sideslip, and which input source was dropped, so a figure from
            // a vehicle replayed without its magnetometer names that it was.
            self.course.map_or_else(
                || "off".to_string(),
                |s| format!("{:.1}", s.as_radians().to_degrees())
            ),
            self.without.as_deref().unwrap_or("none"),
            // A config other than the default names itself, so a figure taken under `--set`
            // cannot pass for the default's.
            if self.sets.is_empty() {
                "none".to_string()
            } else {
                self.sets
                    .iter()
                    .map(|(name, value)| format!("{name}:{value}"))
                    .collect::<Vec<_>>()
                    .join(",")
            },
            // The gate's verdict, which no other key on this line reports: `refused=` and
            // `invalid=` are propagation steps, not measurements, so a change that started
            // turning down every fix in the corpus would pass `--check` unmoved without
            // this. The total, then the same count per source; see `rejections_by_source`
            // for why both.
            self.rejections(),
            self.rejections_by_source(),
            // Everything that never reached the gate. See `Replay::discarded`.
            self.discarded(),
            // Gaps past `max_predict_dt`: coasted by (22′), or refused with coasting off.
            self.filter.diagnostics().propagation.coasted,
            self.filter.diagnostics().propagation.refused_too_long,
            self.filter.diagnostics().propagation.refused_invalid,
            // Equation (42′)'s diagonal floor, pinned at zero on every log because that is
            // the claim: the floor is set well below anything the filter reaches here, so
            // it is a guard against a collapse rather than part of the arithmetic. The
            // margin is measured in `math.rs`'s `FLOOR`, which is the one place it is
            // written. A log that starts flooring is one whose covariance is being
            // driven to zero by something upstream, and no other key on this line would
            // notice — a floored variance makes the estimate *more* conservative, so it
            // moves neither `rejected=` nor `transitions=`.
            self.filter.diagnostics().floored,
            self.epochs,
            // The four consistency families of #5. They read the rows the gate judged, which
            // no count on this line describes: `rejected_` says how often the gate refused
            // and nothing about whether the `R` it was judging against is the size the
            // residuals say it is. See `consistency_keys`.
            self.consistency_keys(),
            self.noise_keys(),
            // How long the filter said so, beside how often: `transitions=` counts a flap
            // and a minute alike, and #60 asks for the time a hostile receiver costs.
            self.seconds_in(Status::Degraded),
            self.seconds_in(Status::DeadReckoning),
            self.transitions.len(),
            state.status,
        )
    }

    /// Log time spent in one [`Status`], to the last IMU sample; see [`seconds_in`].
    fn seconds_in(&self, status: Status) -> f64 {
        seconds_in(
            &self.transitions,
            self.previous_imu.unwrap_or_default(),
            status,
        )
    }

    /// Per-quantity validity now, and predicted at takeoff.
    ///
    /// The second column names the horizon it asked over, because that is a `Config` choice
    /// and the answer means nothing without it: `predicted_validity` projects the covariance
    /// that far with nothing fusing, so the same log reads differently at 1 s and at 6 s.
    fn report_validity(&self) {
        let validity = self.filter.state().validity;
        let predicted = self.filter.predicted_validity();
        let horizon = self.filter.config().accuracy.horizon.as_secs();
        println!("\nvalidity at end of log        now  +{horizon:.1} s unaided");
        for quantity in &QUANTITIES {
            let mark = |flag| if flag { "yes" } else { " no" };
            println!(
                "  {:<13} {}        {}",
                quantity.name,
                mark((quantity.flag)(validity)),
                mark((quantity.flag)(predicted)),
            );
        }
    }

    /// Per-source health at the end of the log.
    fn report_sources(&self) {
        println!("\nper-source health at end of log");
        for (name, health) in self.filter.diagnostics().sources() {
            match health.time_since_accepted {
                Some(elapsed) => println!(
                    "  {name:<13} last accepted {:>5.1} s ago, {} accepted, {} rejected",
                    elapsed.as_secs(),
                    health.accepted,
                    health.rejected
                ),
                None => println!("  {name:<13} never accepted"),
            }
            // What separates a miswired sensor from one that was never connected: both read
            // `never accepted`, and only this says the filter was turned down and why.
            if health.recovered > 0 {
                println!(
                    "  {:<13} {} adopted to recover from lockout",
                    "", health.recovered
                );
            }
            if let Some(refusal) = health.last_refusal {
                // "measurements", because `summary`'s `refused=` counts propagation steps.
                println!(
                    "  {:<13} {} measurements refused, last {refusal:?}",
                    "", health.refused
                );
            }
        }
    }
}

/// One parsed row. Values and variances are positional; which ones a source uses is
/// documented in the log header.
struct Record<'a> {
    t: f64,
    source: &'a str,
    values: [Option<f32>; 6],
    variances: [Option<f32>; 3],
    /// `t_meas_s`, where the row carries one.
    measured: Option<f64>,
}

impl<'a> Record<'a> {
    fn parse(line: &'a str) -> Option<Self> {
        let mut fields = line.split(',');
        // `f64`, not `f32`. A timestamp is large and a `dt` is small, and in `f32` the
        // magnitude eats the mantissa: at 1200 s into a flight the ULP is 0.12 ms, so a
        // 2.5 ms step comes back 2.3% wrong; at 18000 s the ULP is 1.95 ms and the same
        // step reads 22% short; past 65536 s it exceeds the step entirely and consecutive
        // samples collapse onto one value, so the step reads as zero. That noise would
        // look like filter error during validation. Values stay `f32`; only time is
        // widened.
        let t = fields.next()?.trim().parse().ok()?;
        let source = fields.next()?.trim();
        let mut values = [None; 6];
        let mut variances = [None; 3];
        for slot in values.iter_mut().chain(variances.iter_mut()) {
            let Some(field) = fields.next() else { break };
            let field = field.trim();
            if !field.is_empty() {
                *slot = Some(field.parse().ok()?);
            }
        }
        let measured = match fields.next().map(str::trim) {
            None | Some("") => None,
            Some(field) => Some(field.parse().ok()?),
        };
        Some(Self {
            t,
            source,
            values,
            variances,
            measured,
        })
    }

    /// When the measurement was taken: `t_meas_s`, or the row's own time.
    fn taken(&self) -> Timestamp {
        Timestamp::from_secs_f64(self.measured.unwrap_or(self.t))
    }

    fn value(&self, i: usize) -> Result<f32, String> {
        self.values[i].ok_or_else(|| format!("`{}` row has no v{i}", self.source))
    }

    fn variance(&self, i: usize) -> Result<f32, String> {
        self.variances[i].ok_or_else(|| format!("`{}` row has no var{i}", self.source))
    }
}

// ---------------------------------------------------------------------------------------------
// Scoring against truth
// ---------------------------------------------------------------------------------------------

/// The truth file's columns, in order.
///
/// Checked against the header rather than assumed, because nothing connects this list to
/// `write_truth_header` in `examples/simulate.rs` at compile time. A column renamed or
/// reordered there would otherwise score one quantity against another and publish a number
/// for it.
const TRUTH_COLUMNS: [&str; STATES + 1] = [
    "t_s", "pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d", "roll", "pitch", "yaw", "ba_x",
    "ba_y", "ba_z", "bg_x", "bg_y", "bg_z",
];

/// How near a truth row must be to a GNSS fix to be taken as the truth at its time, in
/// seconds.
///
/// A fix is judged against truth taken at its own time, never at the nearest row alone: the
/// simulator writes both at the same instants, and UrbanNav's receivers and its SPAN-CPT
/// both report on the UTC second. So this only absorbs a writer's rounding, the simulator's
/// four decimals. A fix between truth rows is judged against the two interpolated, within
/// [`TRUTH_SPAN`]. Epochs are matched within an IMU period instead, and never interpolated;
/// see [`Truth::at`].
const TRUTH_TOLERANCE: f64 = 1e-4;

/// The widest pair of truth rows a fix between them is interpolated across, in seconds.
///
/// INSANE's truth is at RTK2's epochs, 125 ms apart and 250 ms at up to one interval in
/// eight (12 % on `outdoor_1`, 6 % on `mars_19`, 0.5 % on `mars_1`), and its fixes land
/// between them: without interpolation 1799 of `mars_19`'s 1801 went unjudged. 0.3 s spans
/// one dropped row and not a gap. Linear over it, a vehicle accelerating at `a` is misplaced
/// by at most `a h² / 8`, 3 cm at 3 m s⁻². On the three sequences `rms_gnss_pos` moves by at
/// most 0.019 m across a 0.2 s span, 0.3 s and none at all, while 0.2 s judges 753 of
/// `outdoor_1`'s fixes where 0.3 s judges 958 and no limit 968.
const TRUTH_SPAN: f64 = 0.3;

/// Components in one NEES block: position, velocity and attitude are three each.
const BLOCK: usize = 3;

/// One row of `<scenario>.truth.csv`: the state a perfect filter would report at that
/// instant, with the biases actually applied to that sample.
///
/// A bias group may be blank, all three cells, where nothing knows it: a real vehicle's
/// reference (`tools/urbannav2replay.py`'s SPAN-CPT) reports a trajectory and no bias of the
/// IMU being scored. `None` then, and every key that reads it says `none` rather than
/// scoring the estimate against a zero nobody measured. Position, velocity and attitude are
/// never optional: they are what a truth file is for.
#[derive(Clone, Copy)]
struct TruthRow {
    t: f64,
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    attitude: UnitQuaternion<f32>,
    accel_bias: Option<Vector3<f32>>,
    gyro_bias: Option<Vector3<f32>>,
}

impl TruthRow {
    fn parse(line: &str) -> Option<Self> {
        let mut fields = line.split(',');
        // `f64` for the same reason `Record::parse` widens it: a timestamp is large and the
        // interval between two of them is small.
        let t = fields.next()?.trim().parse().ok()?;
        let mut values = [None; STATES];
        for slot in &mut values {
            let cell = fields.next()?.trim();
            if !cell.is_empty() {
                *slot = Some(cell.parse::<f32>().ok()?);
            }
        }
        // Five groups of three, in `TRUTH_COLUMNS` order after `t_s` — which the header this
        // file was accepted on has already been checked against. A group is all there or all
        // blank; a partial one is a malformed row, not a partial truth.
        let group = |first: usize| match values[first..first + BLOCK] {
            [Some(x), Some(y), Some(z)] => Some(Some(Vector3::new(x, y, z))),
            [None, None, None] => Some(None),
            _ => None,
        };
        let [roll, pitch, yaw] = [values[6]?, values[7]?, values[8]?];
        Some(Self {
            t,
            position: group(0)??,
            velocity: group(3)??,
            // `from_euler_angles` is the ZYX sequence `Attitude::euler_angles` reads back,
            // which is what makes the two ends of this comparison the same convention.
            attitude: UnitQuaternion::from_euler_angles(roll, pitch, yaw),
            accel_bias: group(9)?,
            gyro_bias: group(12)?,
        })
    }
}

/// Where the truth puts the IMU and how it is turned: all a fix is judged on, since the
/// antenna is `p + R(q) a`.
#[derive(Clone, Copy)]
struct Pose {
    position: Vector3<f32>,
    attitude: UnitQuaternion<f32>,
}

impl TruthRow {
    fn pose(&self) -> Pose {
        Pose {
            position: self.position,
            attitude: self.attitude,
        }
    }

    /// The pose at `t` between this row and a later one: linear in position, and along the
    /// shortest rotation in attitude.
    fn pose_between(&self, later: &Self, t: f64) -> Pose {
        let f = ((t - self.t) / (later.t - self.t)) as f32;
        Pose {
            position: self.position + (later.position - self.position) * f,
            attitude: self.attitude.slerp(&later.attitude, f),
        }
    }
}

/// The magnetic declination a log's header names, or zero where it names none.
///
/// Read from the log because the site sets it: `tools/ulog2replay.py` writes the one the log's
/// EKF2 used and `examples/simulate.rs` the one its field was generated for. One constant for
/// every log put a standing 17° between `cd7e0001`'s heading and EKF2's, and the error would
/// score as heading bias with nothing failing to say so. `declination=` on the `summary` line
/// is what notices it stop arriving.
fn declination_of(text: &str) -> Radians {
    header_radians(text, "# Magnetic declination ").unwrap_or(Radians::ZERO)
}

/// The antenna offset a leading `# GNSS antenna <forward> <right> <down> m` line names: where
/// the receiver's antenna sits relative to the IMU in body axes, metres, which every GNSS
/// `fuse_*` takes as `antenna`. `tools/ulog2replay.py` writes the one the log's EKF2 applied
/// (`SENS_GPS0_OFF*` less `EKF2_IMU_POS*`), and `examples/simulate.rs` the one its fixes were
/// generated at. `None` for a file with none, replayed at zero.
fn antenna_of(text: &str) -> Option<Position<Body>> {
    let rest = text
        .lines()
        .take_while(|line| line.starts_with('#'))
        .find_map(|line| line.strip_prefix("# GNSS antenna "))?;
    let mut numbers = rest.split_whitespace().map(|word| word.parse::<f32>().ok());
    let (forward, right, down) = (numbers.next()??, numbers.next()??, numbers.next()??);
    Some(Position::body(forward, right, down))
}

/// The site a leading `# Navigation origin <lat> <lon> <height>` line names, degrees and
/// metres: the geodetic point the file's NED positions are relative to. `None` for a file
/// with no fix (`none`) or none of the line, which is a simulator's, whose positions were
/// never geodetic.
fn origin_of(text: &str) -> Option<Geodetic> {
    let rest = text
        .lines()
        .take_while(|line| line.starts_with('#'))
        .find_map(|line| line.strip_prefix("# Navigation origin "))?;
    let mut numbers = rest.split_whitespace().map(|word| word.parse::<f64>().ok());
    let (latitude, longitude, height) = (numbers.next()??, numbers.next()??, numbers.next()??);
    Some(Geodetic::from_degrees(latitude, longitude, height))
}

/// The sideslip a leading `# Course sideslip <rad> rad` line names, which turns on the course
/// constraint. The simulator writes one for a scenario whose vehicle points where it flies; no
/// converted log carries one, since whether a vehicle does is not in its log.
fn sideslip_of(text: &str) -> Option<Radians> {
    header_radians(text, "# Course sideslip ")
}

/// The number a leading `#` line starting with `prefix` carries, in radians.
fn header_radians(text: &str, prefix: &str) -> Option<Radians> {
    text.lines()
        .take_while(|line| line.starts_with('#'))
        .find_map(|line| line.strip_prefix(prefix))
        .and_then(|rest| rest.split_whitespace().next()?.parse().ok())
        .map(Radians::from_radians)
}

/// The floors a `# GNSS noise parameters` header line names, which `--r-policy px4` fuses
/// with. `None` for a log converted before the converter wrote one, which that policy refuses
/// rather than filling in a default the log's EKF2 may not have run.
fn gnss_noise_of(text: &str) -> Option<GnssNoiseParameters> {
    let rest = text
        .lines()
        .take_while(|line| line.starts_with('#'))
        .find_map(|line| line.strip_prefix("# GNSS noise parameters: "))?;
    let value = |name: &str| -> Option<f32> {
        rest.split(',')
            .find_map(|cell| cell.trim().strip_prefix(name)?.split_whitespace().next())?
            .parse()
            .ok()
    };
    Some(GnssNoiseParameters {
        position: value("EKF2_GPS_P_NOISE")?,
        velocity: value("EKF2_GPS_V_NOISE")?,
        no_aid: value("EKF2_NOAID_NOISE")?,
    })
}

/// The run a generated file names in its `#` header, as `(name, tag)`.
///
/// `examples/simulate.rs` stamps both halves of a scenario with this — the log carries
/// ``scenario `mission`, seed 2`` and the truth ``truth for `mission.csv`, seed 2`` — and a
/// converter pairing a log with its reference does the same with the sources' digest,
/// ``converted `urbannav-tst-m8t` from …, source 39a76af5d675``. The tag is what follows the
/// last `, `, compared as text, so either form is one check. It is the only thing that can
/// say a truth file belongs to the log being scored against it. The row count cannot: a
/// 50 Hz log lands on every fourth row of 200 Hz truth, so the wrong file matches every epoch
/// and publishes an accuracy figure with nothing tying it to what produced it.
///
/// `None` for a file carrying no marker — a corpus log, a hand-written one. Those are taken
/// on trust, because there is nothing in them that could be checked.
fn scenario_of(text: &str) -> Option<(String, String)> {
    let line = text
        .lines()
        .take_while(|line| line.starts_with('#'))
        .find(|line| line.contains("fusion-nav"))?;
    // The backticked name, which the truth header spells with its `.csv` and the log's
    // without, and the trailing tag.
    let name = line.split('`').nth(1)?.trim_end_matches(".csv").to_string();
    let (_, tag) = line.rsplit_once(", ")?;
    let tag = tag.trim();
    (tag.starts_with("seed ") || tag.starts_with("source ")).then(|| (name, tag.to_string()))
}

/// The truth file, and how far through it the replay has got.
#[derive(Clone)]
struct Truth {
    rows: Vec<TruthRow>,
    cursor: usize,
    /// The row the last answer came from, and the epoch it answered: so a row is scored at
    /// one epoch, and a repeated epoch is answered again.
    claimed: Option<(usize, f64)>,
    /// The epoch asked about before this one.
    previous: Option<f64>,
    /// Rows passed unscored although an epoch fell within half a period of them: a truth
    /// file denser than the log, or offset from it.
    missed: u32,
}

impl Truth {
    fn parse(text: &str) -> Result<Self, String> {
        let mut rows = Vec::new();
        let mut header = false;
        for (n, line) in text.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if !header {
                let columns: Vec<&str> = line.split(',').map(str::trim).collect();
                if columns.as_slice() != TRUTH_COLUMNS.as_slice() {
                    return Err(format!(
                        "line {}: truth columns are `{}`, not `{line}`",
                        n + 1,
                        TRUTH_COLUMNS.join(",")
                    ));
                }
                header = true;
                continue;
            }
            rows.push(
                TruthRow::parse(line).ok_or_else(|| format!("line {}: malformed row", n + 1))?,
            );
        }
        if rows.is_empty() {
            return Err("no truth rows".to_string());
        }
        Ok(Self {
            rows,
            cursor: 0,
            claimed: None,
            previous: None,
            missed: 0,
        })
    }

    /// The truth at one epoch, or `None` if this file has no row for it.
    ///
    /// The row nearest the epoch, if it is within `tolerance`, one IMU period, and no earlier
    /// epoch was scored against it: so each row is scored at the first epoch it is nearest,
    /// within a period. The simulator writes a row per epoch at the epoch's own time, so
    /// every epoch finds its own row exactly. A vehicle's reference is sparser than its IMU
    /// (UrbanNav's SPAN-CPT at 1 Hz against a 400 Hz Xsens) and never lands on an epoch, so
    /// a row is scored up to 2.5 ms from its epoch, 29 mm at the segment's 11.5 m/s. Half a
    /// period would be the nearest epoch alone, and missed 21 of the segment's 787 rows to
    /// the Xsens' timestamp jitter. Nothing is interpolated: the epochs between rows are
    /// unscored, as they are.
    ///
    /// A cursor rather than a search: epochs arrive in order, so the whole file is walked
    /// once. It moves on only to a row strictly nearer, which is what lets a repeated
    /// timestamp — a duplicate IMU row, refused as an invalid step and still written as an
    /// epoch — ask twice and be answered twice.
    fn at(&mut self, t: f64, tolerance: f64) -> Option<TruthRow> {
        while self
            .rows
            .get(self.cursor + 1)
            .is_some_and(|next| (next.t - t).abs() < (self.rows[self.cursor].t - t).abs())
        {
            // A row left behind unscored between two epochs, within half a period of either,
            // had an epoch to be scored at. One in a gap in the log did not, nor one before
            // the first epoch scored, nor the neighbour of a row an epoch sits on.
            let passed = self.rows[self.cursor].t;
            let claimed = self.claimed.is_some_and(|(index, _)| index == self.cursor);
            let reachable = self.previous.is_some_and(|previous| {
                let near = |epoch: f64| (passed - epoch).abs() <= tolerance / 2.0;
                near(previous) || near(t)
            });
            if reachable && !claimed {
                self.missed += 1;
            }
            self.cursor += 1;
        }
        self.previous = Some(t);
        let row = self
            .rows
            .get(self.cursor)
            .copied()
            .filter(|row| (row.t - t).abs() <= tolerance)?;
        match self.claimed {
            Some((index, at)) if index == self.cursor => (at == t).then_some(row),
            _ => {
                self.claimed = Some((self.cursor, t));
                Some(row)
            }
        }
    }

    /// The truth's pose at a fix's instant, found by search: a fix is asked for out of step
    /// with the epochs' cursor, and at most once.
    ///
    /// The row within [`TRUTH_TOLERANCE`] where there is one, so a file written at the fixes'
    /// own times is read as written. Otherwise the two rows either side interpolated, where
    /// they are at most [`TRUTH_SPAN`] apart, and `None` across a gap or outside the file.
    fn at_fix(&self, t: f64) -> Option<Pose> {
        let after = self.rows.partition_point(|row| row.t < t - TRUTH_TOLERANCE);
        if let Some(row) = self
            .rows
            .get(after)
            .filter(|row| (row.t - t).abs() <= TRUTH_TOLERANCE)
        {
            return Some(row.pose());
        }
        let (a, b) = (self.rows.get(after.checked_sub(1)?)?, self.rows.get(after)?);
        (b.t - a.t <= TRUTH_SPAN).then(|| a.pose_between(b, t))
    }
}

/// The error state of equation (2), `δx = truth ⊖ estimate`, in the [`ErrorState`] ordering.
///
/// One vector with four readers — RMSE, `in3s`, NEES and `false_valid` all take slices of
/// it — so every key on the `score` line describes the same error. Computed in one place
/// because the attitude part is what is easy to get differently twice: a tilt error
/// differenced from Euler angles and a `δθ` taken from the quaternions agree near level and
/// part company in a bank.
///
/// That attitude part is `δθ` of (3) and (4). `q = q̂ ⊗ δq`, so `δq = q̂⁻¹ ⊗ q` and `δθ` is
/// its rotation vector: a body-frame perturbation, which is what `P`'s attitude block is the
/// covariance of and therefore the only form NEES can compare against. A rotation vector's
/// norm is at most π, so its yaw component arrives wrapped and none is applied here.
fn error_state(state: &State, truth: &TruthRow) -> SVector<f32, STATES> {
    let mut error = SVector::<f32, STATES>::zeros();
    // Each block beside the state it starts at, rather than relying on the order of this
    // array to land it in the right rows. The offsets are `ErrorState`'s own, so the
    // ordering of equation (2) is cited here and declared in `src/state.rs`.
    let blocks = [
        (
            ErrorState::PositionNorth,
            truth.position - Vector3::from(state.position.to_array()),
        ),
        (
            ErrorState::VelocityNorth,
            truth.velocity - Vector3::from(state.velocity.to_array()),
        ),
        (
            ErrorState::AttitudeX,
            (rotation(state.attitude).inverse() * truth.attitude).scaled_axis(),
        ),
        // Zero where the truth knows no bias: [`Score::epoch`] reads the bias rows only where
        // it does, so the zero is never scored as a perfect estimate.
        (
            ErrorState::AccelBiasX,
            truth.accel_bias.map_or_else(Vector3::zeros, |bias| {
                bias - Vector3::from(state.accel_bias.to_array())
            }),
        ),
        (
            ErrorState::GyroBiasX,
            truth.gyro_bias.map_or_else(Vector3::zeros, |bias| {
                bias - Vector3::from(state.gyro_bias.to_array())
            }),
        ),
    ];
    for (first, values) in &blocks {
        error
            .fixed_rows_mut::<BLOCK>(first.index())
            .copy_from(values);
    }
    error
}

/// One component of the error vector, named by its error state rather than its offset.
fn at(error: &SVector<f32, STATES>, state: ErrorState) -> f32 {
    error[state.index()]
}

/// The norm of two components — the horizontal pair of a position or velocity, or the two
/// tilt axes of an attitude. Both named, so neither is "the one after the other".
fn horizontal(error: &SVector<f32, STATES>, x: ErrorState, y: ErrorState) -> f32 {
    at(error, x).hypot(at(error, y))
}

/// The attitude error on navigation axes, `(north, east, down)`: tilt about the first two,
/// heading about the third.
///
/// `R(q̂) δθ`, which is how `Eskf::validity` resolves the covariance of `δθ` before testing
/// it — and computed here without that rotation, as the rotation vector of `q ⊗ q̂⁻¹`. The two
/// are the same vector exactly (`q ⊗ q̂⁻¹ = q̂ ⊗ δq ⊗ q̂⁻¹`), so the audit states the claim in
/// the filter's shape without borrowing its geometry: a transposed `R` in the filter would
/// be mirrored by a harness that reused it, and is not by this.
fn attitude_error_ned(state: &State, truth: &TruthRow) -> Vector3<f32> {
    (truth.attitude * rotation(state.attitude).inverse()).scaled_axis()
}

/// `ε = δxᵀ P⁻¹ δx` over the three-component block starting at `first`, or `None` if that
/// block is singular.
///
/// The joint test: it reads the block's correlations, where `in3s` reads only its diagonal.
/// A singular block is reported rather than unwrapped — it means a covariance that has
/// collapsed, which is a finding and not a reason to stop replaying the log.
fn nees(error: &SVector<f32, STATES>, p: &CovarianceMatrix, first: ErrorState) -> Option<f64> {
    let at = first.index();
    let inverse = p
        .fixed_view::<BLOCK, BLOCK>(at, at)
        .into_owned()
        .try_inverse()?;
    let delta = error.fixed_rows::<BLOCK>(at).into_owned();
    Some(f64::from(delta.dot(&(inverse * delta))))
}

/// One of the six quantities [`Validity`] answers for.
///
/// A table rather than parallel lists. The names, the flags and the bars were written out
/// separately for `report_validity`, for the `false_valid` count and for the breakdown under
/// it, with nothing but a doc comment holding the three orders together — reordering one
/// silently relabelled the others.
struct Quantity {
    name: &'static str,
    /// Which flag on [`Validity`] this is. Read off the filter rather than re-derived from
    /// the covariance: a recomputation would test a copy of the claim instead of the claim.
    flag: fn(Validity) -> bool,
    /// The components the claim covers.
    components: &'static [Component],
    /// The [`Accuracy`] field those components are judged against.
    bar: fn(&Accuracy) -> f32,
}

impl Quantity {
    /// Whether the filter claimed this quantity and the truth error says otherwise.
    ///
    /// Per axis, because that is the shape of the claim: `Eskf::validity` asks
    /// `within(PositionNorth) && within(PositionEast)`, so what falsifies it is either axis
    /// outside the bar. Testing the 2-D norm instead would hold the filter to a bar √2
    /// tighter than the one it asserted, and would diverge from the claim exactly as the
    /// estimate approached it — the regime this count exists to watch.
    ///
    /// Attitude is per navigation axis for the same reason: the filter tests tilt about
    /// north and east and heading about down, so those are what a truth error falsifies.
    fn falsified(
        &self,
        error: &SVector<f32, STATES>,
        attitude_ned: &Vector3<f32>,
        validity: Validity,
        accuracy: &Accuracy,
    ) -> bool {
        (self.flag)(validity)
            && self
                .components
                .iter()
                .any(|component| component.of(error, attitude_ned).abs() > (self.bar)(accuracy))
    }
}

/// One component a [`Validity`] claim is stated on.
#[derive(Clone, Copy)]
enum Component {
    /// A position or velocity axis, which the error state already carries on navigation axes.
    State(ErrorState),
    /// Tilt about north, from [`attitude_error_ned`].
    TiltNorth,
    /// Tilt about east.
    TiltEast,
    /// Heading: rotation about down.
    Heading,
}

impl Component {
    fn of(self, error: &SVector<f32, STATES>, attitude_ned: &Vector3<f32>) -> f32 {
        match self {
            Self::State(state) => at(error, state),
            Self::TiltNorth => attitude_ned.x,
            Self::TiltEast => attitude_ned.y,
            Self::Heading => attitude_ned.z,
        }
    }

    /// The filter's variance on the same axis, so an error and the σ it is banded with are
    /// read in one frame: the covariance diagonal for position and velocity, which the error
    /// state carries on navigation axes already, and `Eskf::attitude_variance` for tilt and
    /// heading, which the body-axis diagonal is not.
    fn variance(self, covariance: &Covariance, attitude: AttitudeVariance) -> f32 {
        match self {
            Self::State(state) => covariance.variance(state),
            Self::TiltNorth => attitude.tilt_north,
            Self::TiltEast => attitude.tilt_east,
            Self::Heading => attitude.heading,
        }
    }
}

/// The components `<out>.error.csv` writes, each beside its σ, and the column names they are
/// written under.
///
/// Navigation axes throughout, the way [`Validity`] states its claims, so a panel can draw the
/// error inside its own ±3σ band. The band travels in the same row as the error rather than
/// being joined from the epoch file afterwards: a reader pairing two files it decimates
/// separately draws an error against a σ from another epoch, which is how
/// `tools/replay_report.py`'s first track figure came to plot positions the vehicle never held.
const ERRORS: [(Component, &str); 9] = [
    (Component::State(ErrorState::PositionNorth), "pos_n"),
    (Component::State(ErrorState::PositionEast), "pos_e"),
    (Component::State(ErrorState::PositionDown), "pos_d"),
    (Component::State(ErrorState::VelocityNorth), "vel_n"),
    (Component::State(ErrorState::VelocityEast), "vel_e"),
    (Component::State(ErrorState::VelocityDown), "vel_d"),
    (Component::TiltNorth, "tilt_n"),
    (Component::TiltEast, "tilt_e"),
    (Component::Heading, "heading"),
];

/// One component of [`ERRORS`] at one epoch: the truth error and the filter's σ on that axis.
type Deviation = [(f32, f32); ERRORS.len()];

/// The six quantities, in the order [`Score::false_valid`] counts them.
const QUANTITIES: [Quantity; 6] = [
    Quantity {
        name: "tilt",
        flag: |v| v.tilt,
        components: &[Component::TiltNorth, Component::TiltEast],
        bar: |a| a.tilt.as_radians(),
    },
    Quantity {
        name: "heading",
        flag: |v| v.heading,
        components: &[Component::Heading],
        bar: |a| a.heading.as_radians(),
    },
    Quantity {
        name: "position (h)",
        flag: |v| v.horizontal_position,
        components: &[
            Component::State(ErrorState::PositionNorth),
            Component::State(ErrorState::PositionEast),
        ],
        bar: |a| a.position.as_meters(),
    },
    Quantity {
        name: "position (v)",
        flag: |v| v.vertical_position,
        components: &[Component::State(ErrorState::PositionDown)],
        bar: |a| a.position.as_meters(),
    },
    Quantity {
        name: "velocity (h)",
        flag: |v| v.horizontal_velocity,
        components: &[
            Component::State(ErrorState::VelocityNorth),
            Component::State(ErrorState::VelocityEast),
        ],
        bar: |a| a.velocity.as_m_per_s(),
    },
    Quantity {
        name: "velocity (v)",
        flag: |v| v.vertical_velocity,
        components: &[Component::State(ErrorState::VelocityDown)],
        bar: |a| a.velocity.as_m_per_s(),
    },
];

/// What scoring the replay against truth has found so far.
///
/// Sums in `f64` while the states are `f32`: squared position errors over tens of thousands
/// of epochs lose precision in `f32` fast enough that the RMSE would depend on how long the
/// log ran.
#[derive(Clone, Default)]
struct Score {
    scored: u32,
    position_horizontal: f64,
    position_vertical: f64,
    position_horizontal_max: f64,
    velocity: f64,
    tilt: f64,
    yaw: f64,
    accel_bias: f64,
    gyro_bias: f64,
    /// Epochs whose truth knew each bias, the mean `ba` and `bg` are taken over: a truth file
    /// with blank bias columns scores the trajectory and leaves both `none`.
    accel_bias_scored: u32,
    gyro_bias_scored: u32,
    within_3s: u64,
    axes: u64,
    nees: [f64; 3],
    nees_epochs: [u32; 3],
    /// Per claim, in `claims` order.
    false_valid: [u32; 6],
    /// Each GNSS verdict against truth, `gnss_pos` and `gnss_hgt` in that order.
    fixes: [FixScore; 2],
}

impl Score {
    /// Accumulate one epoch.
    ///
    /// The claim `false_valid` tests comes off `state.validity` rather than a second
    /// argument, so there is no way to score an epoch against a verdict the estimate did not
    /// carry.
    fn epoch(
        &mut self,
        state: &State,
        covariance: &Covariance,
        truth: &TruthRow,
        accuracy: &Accuracy,
    ) -> (Epsilon, [f32; ERRORS.len()]) {
        use ErrorState::*;
        let error = error_state(state, truth);
        let attitude_ned = attitude_error_ned(state, truth);
        self.scored += 1;

        let horizontal_error = f64::from(horizontal(&error, PositionNorth, PositionEast));
        self.position_horizontal += horizontal_error * horizontal_error;
        self.position_horizontal_max = self.position_horizontal_max.max(horizontal_error);
        self.position_vertical += f64::from(at(&error, PositionDown)).powi(2);
        // All three axes together, where position is split: a speed error has no horizontal
        // and vertical bar to be judged against the way `Accuracy::position` gives position.
        self.velocity += f64::from(
            error
                .fixed_rows::<BLOCK>(VelocityNorth.index())
                .norm_squared(),
        );
        self.tilt += f64::from(attitude_ned.x.hypot(attitude_ned.y)).powi(2);
        self.yaw += f64::from(attitude_ned.z).powi(2);
        // Whole blocks, the way `velocity` is scored: a bias is estimated per axis but
        // wrong as one vector, and no part of `Accuracy` splits it.
        if truth.accel_bias.is_some() {
            self.accel_bias_scored += 1;
            self.accel_bias +=
                f64::from(error.fixed_rows::<BLOCK>(AccelBiasX.index()).norm_squared());
        }
        if truth.gyro_bias.is_some() {
            self.gyro_bias_scored += 1;
            self.gyro_bias +=
                f64::from(error.fixed_rows::<BLOCK>(GyroBiasX.index()).norm_squared());
        }

        let p = matrix(covariance);
        for i in 0..STATES {
            let unknown = match i / BLOCK {
                3 => truth.accel_bias.is_none(),
                4 => truth.gyro_bias.is_none(),
                _ => false,
            };
            if unknown {
                continue;
            }
            self.axes += 1;
            if error[i].abs() <= 3.0 * p[(i, i)].sqrt() {
                self.within_3s += 1;
            }
        }

        // Position, velocity and attitude, in the order `nees_pos`, `nees_vel`, `nees_att`
        // are printed. The bias blocks have no NEES key; `in3s` above is what reaches them.
        let mut epsilon = [None; 3];
        for (block, first) in [PositionNorth, VelocityNorth, AttitudeX]
            .into_iter()
            .enumerate()
        {
            epsilon[block] = nees(&error, &p, first);
            if let Some(nees) = epsilon[block] {
                self.nees[block] += nees;
                self.nees_epochs[block] += 1;
            }
        }

        for (count, quantity) in self.false_valid.iter_mut().zip(&QUANTITIES) {
            if quantity.falsified(&error, &attitude_ned, state.validity, accuracy) {
                *count += 1;
            }
        }
        (
            epsilon,
            ERRORS.map(|(component, _)| component.of(&error, &attitude_ned)),
        )
    }

    /// Root mean square of one accumulated sum of squares.
    fn rms(&self, sum: f64) -> f64 {
        (sum / f64::from(self.scored.max(1))).sqrt()
    }

    /// The bias RMS as the `score` line prints it, `places` decimals, or `none` where the
    /// truth never knew that bias.
    fn bias_text(sum: f64, scored: u32, places: usize) -> String {
        match scored {
            0 => "none".to_string(),
            n => format!("{:.places$}", (sum / f64::from(n)).sqrt()),
        }
    }

    /// Mean NEES per degree of freedom for one block — ≈1 where the covariance describes the
    /// error it actually made — or `None` where the block was singular at every epoch and
    /// there is no such figure.
    fn nees_per_dof(&self, block: usize) -> Option<f64> {
        match self.nees_epochs[block] {
            0 => None,
            epochs => Some(self.nees[block] / f64::from(epochs) / BLOCK as f64),
        }
    }

    /// That figure as both the report and the `score` line print it.
    ///
    /// A word where it could not be computed, as `aligned_at=never` and `alpha0=none` are.
    /// `0.0000` would read as an extremely conservative covariance — the opposite of the
    /// collapsed one that produced it — and nothing else published would tell the two apart.
    fn nees_text(&self, block: usize) -> String {
        self.nees_per_dof(block)
            .map_or_else(|| "none".to_string(), |nees| format!("{nees:.4}"))
    }

    /// Fraction of axis-epochs within 3σ, over all 15 states.
    fn in3s(&self) -> f64 {
        self.within_3s as f64 / self.axes.max(1) as f64
    }

    /// Quantity-epochs claimed usable while the error was outside `Config::accuracy`.
    fn false_valid(&self) -> u32 {
        self.false_valid.iter().sum()
    }

    /// The attitude's share of it: tilt and heading, the first two of [`QUANTITIES`].
    ///
    /// Published beside the total because a total is an allowance, and an allowance hides
    /// whatever it is not being spent on. `moving_start` carries 108 of these epochs on
    /// its vertical velocity, which is a coarse start's velocity prior rather than the
    /// attitude bound of (8′) — and a total pinned at 110 to admit them would pass a
    /// hundred epochs of falsely valid *attitude* without a word. This one is pinned at
    /// zero on every scenario, which is what makes "the tighter attitude prior is honest"
    /// a claim `data/bench.sh` enforces rather than a sentence in a header.
    fn false_valid_attitude(&self) -> u32 {
        self.false_valid[0] + self.false_valid[1]
    }

    /// The `score` line, in `summary`'s `key=value` shape so one parser reads both.
    ///
    /// With nothing scored it is `score scored=0` and no more. A full line of zeros would
    /// put `pos_h=0.000`, the best possible value, beside `in3s=0.0000`, the worst, and
    /// claim a perfect filter for a log that was never measured — the same reason a log with
    /// no truth file gets no line at all.
    fn line(&self) -> String {
        if self.scored == 0 {
            return "score scored=0".to_string();
        }
        let fixes: String = [GNSS_POS, GNSS_HGT]
            .iter()
            .zip(&self.fixes)
            .map(|(&source, fixes)| fixes.keys(SOURCES[source]))
            .collect();
        format!(
            "score pos_h={:.3} pos_v={:.3} vel={:.3} pos_h_max={:.3} tilt={:.3} yaw={:.3} \
             ba={} bg={} in3s={:.4} nees_pos={} nees_vel={} nees_att={} \
             false_valid={} false_valid_att={} scored={}{fixes}",
            self.rms(self.position_horizontal),
            self.rms(self.position_vertical),
            self.rms(self.velocity),
            self.position_horizontal_max,
            self.rms(self.tilt).to_degrees(),
            self.rms(self.yaw).to_degrees(),
            Self::bias_text(self.accel_bias, self.accel_bias_scored, 5),
            Self::bias_text(self.gyro_bias, self.gyro_bias_scored, 6),
            self.in3s(),
            self.nees_text(0),
            self.nees_text(1),
            self.nees_text(2),
            self.false_valid(),
            self.false_valid_attitude(),
            self.scored,
        )
    }
}

/// Time spent in `status` over a list of transitions ending at `end`.
///
/// Read off the transitions rather than kept beside them, so the two cannot disagree.
fn seconds_in(transitions: &[(f64, Status)], end: f64, status: Status) -> f64 {
    let until = transitions.iter().skip(1).map(|(t, _)| *t).chain([end]);
    transitions
        .iter()
        .zip(until)
        .filter(|((_, s), _)| *s == status)
        .map(|((from, _), to)| to - from)
        // A fold from +0.0: `Sum` on floats starts at −0.0, which prints as `-0.00`.
        .fold(0.0, |total, span| total + span)
}

/// What truth says of one half of a GNSS fix: `bad` where the receiver's own `R` could not
/// have produced the error, `far` where the error is past the mission's bar.
///
/// `bad` is the gate a perfect state would run: `ε = eᵀ R⁻¹ e` with `e` the fix less the true
/// antenna position, per half the way the filter gates it, `Gate<2>` on the horizontal pair
/// and `Gate<1>` on height (#118), at [`Percentile::P999`] and the row's own variance. Fixed
/// there rather than at `Config::gates`, `--r-policy` or `--recovery`, so a sweep of any
/// scores its rejections against one set of bad fixes. The antenna is the one the replay
/// fuses at, so `--antenna zero` judges at the IMU as it fuses there. The truth's own σ is left out: SPAN-CPT reports
/// at most 0.40 m on this segment, 3 % of the smallest variance a receiver claims.
///
/// `far` is the answer to "wrong by metres" in the shape the filter states it:
/// [`Accuracy::position`] per axis, the bar `Validity` reads, so an accepted fix that is
/// `far` is one that could move a valid estimate out of its own claim.
///
/// `error` is how far the fix is from the truth at the antenna, m: the horizontal distance
/// for the horizontal half, the height difference's magnitude for the other. It is what
/// `rms_<half>` reads, the receiver's own accuracy beside the filter's `pos_h` and `pos_v`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Judged {
    bad: bool,
    far: bool,
    error: f32,
}

/// The gates [`Judged::bad`] reads, fixed at [`Percentile::P999`].
const BAD_HORIZONTAL: Gate<2> = Gate::<2>::at(Percentile::P999);
const BAD_HEIGHT: Gate<1> = Gate::<1>::at(Percentile::P999);

/// One half of every GNSS fix, scored against truth (#60): what the gate did with the fixes
/// that were bad and with those that were not.
///
/// Counts of verdicts, like `rejected_`, and only of those the gate or an adoption decided:
/// a refusal (before initialization, outside the history) judged nothing. A decided fix with
/// no truth at its time ([`Truth::at_fix`]) is `unjudged_`, so a clock or a truth rate that
/// misses the fixes reads as that rather than as nothing bad.
///
/// The recovery split reads the run of rejections since the last accepted or adopted fix,
/// the adopted fix included: an adoption ending a run mostly of good fixes ended a lockout,
/// one ending a run of bad ones ended a glitch, and a tie is counted a glitch. The adopted
/// fix counts because `Config::recovery` times from the last *acceptance*: after a gap in
/// the fixes the first one can be rejected and adopted at once, with no run before it. So
/// the two sum to that source's `SourceHealth::recovered`, where `recovered=` on the
/// `summary` line sums every source and splits none.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct FixScore {
    offered: u32,
    bad: u32,
    rejected_bad: u32,
    rejected_good: u32,
    accepted_far: u32,
    adopted_bad: u32,
    recovered_after_bad: u32,
    recovered_after_lockout: u32,
    unjudged: u32,
    /// The sum of every judged fix's squared [`Judged::error`], over the `offered` fixes.
    squared_error: f64,
    /// Bad and good fixes rejected since the last one accepted or adopted.
    run: (u32, u32),
}

impl FixScore {
    /// Count one verdict. `judged` is `None` where truth had nothing at the fix's time;
    /// `recovery` says an adoption was `Config::recovery`'s rather than a first one.
    fn record(&mut self, outcome: Fusion, judged: Option<Judged>, recovery: bool) {
        if !decided(outcome) {
            return;
        }
        let Some(judged) = judged else {
            self.unjudged += 1;
            return;
        };
        match outcome {
            Fusion::Accepted { .. } => {
                self.accepted_far += u32::from(judged.far);
                self.run = (0, 0);
            }
            Fusion::Rejected { .. } => {
                if judged.bad {
                    self.rejected_bad += 1;
                    self.run.0 += 1;
                } else {
                    self.rejected_good += 1;
                    self.run.1 += 1;
                }
            }
            Fusion::Reset => {
                self.adopted_bad += u32::from(judged.bad);
                if recovery {
                    let (bad, good) = if judged.bad {
                        (self.run.0 + 1, self.run.1)
                    } else {
                        (self.run.0, self.run.1 + 1)
                    };
                    if good > bad {
                        self.recovered_after_lockout += 1;
                    } else {
                        self.recovered_after_bad += 1;
                    }
                }
                self.run = (0, 0);
            }
            Fusion::NotInitialized
            | Fusion::NoReference
            | Fusion::Unobservable
            | Fusion::NotFinite
            | Fusion::InvalidNoise
            | Fusion::StateInvalid
            | Fusion::OutOfHorizon { .. } => return,
        }
        self.offered += 1;
        self.bad += u32::from(judged.bad);
        self.squared_error += f64::from(judged.error).powi(2);
    }

    /// ` offered_<source>=… unjudged_<source>=… rms_<source>=…`, for the `score` line.
    ///
    /// `rms_` is `none` where no fix was judged, rather than a zero that would claim a
    /// perfect receiver.
    fn keys(&self, source: &str) -> String {
        let rms = if self.offered == 0 {
            "none".to_string()
        } else {
            format!(
                "{:.3}",
                (self.squared_error / f64::from(self.offered)).sqrt()
            )
        };
        let counts: String = [
            ("offered", self.offered),
            ("bad", self.bad),
            ("rejected_bad", self.rejected_bad),
            ("rejected_good", self.rejected_good),
            ("accepted_far", self.accepted_far),
            ("adopted_bad", self.adopted_bad),
            ("recovered_after_bad", self.recovered_after_bad),
            ("recovered_after_lockout", self.recovered_after_lockout),
            ("unjudged", self.unjudged),
        ]
        .iter()
        .map(|(key, value)| format!(" {key}_{source}={value}"))
        .collect();
        format!("{counts} rms_{source}={rms}")
    }
}

/// Whether the gate or an adoption decided a measurement, rather than a refusal turning it
/// away before either could.
fn decided(outcome: Fusion) -> bool {
    match outcome {
        Fusion::Accepted { .. } | Fusion::Rejected { .. } | Fusion::Reset => true,
        Fusion::NotInitialized
        | Fusion::NoReference
        | Fusion::Unobservable
        | Fusion::NotFinite
        | Fusion::InvalidNoise
        | Fusion::StateInvalid
        | Fusion::OutOfHorizon { .. } => false,
    }
}

/// `ε` per block for one epoch, in the order `nees_pos`, `nees_vel`, `nees_att`: `None` where
/// the block was singular, or where the truth file had no row.
type Epsilon = [Option<f64>; 3];

/// A truth file and the score being accumulated against it.
#[derive(Clone)]
struct Scoring {
    path: PathBuf,
    /// The `(name, tag)` both headers named, carried into the `.nees.csv` header so
    /// `tools/anees.py` can refuse an ensemble that mixes scenarios or repeats a seed.
    scenario: Option<(String, String)>,
    truth: Truth,
    score: Score,
    /// Epochs outside the truth file's time range: truth that stops before the log does.
    outside: u32,
    /// `ε` and the error per epoch, kept rather than written as they arrive: [`Replay`] is
    /// cloned and rewound while it waits for a static window, and a buffer inside it rewinds
    /// with it. The error is `None` where the truth file had no row.
    epochs: Vec<(f64, Epsilon, Option<Deviation>)>,
}

impl Scoring {
    fn new(path: PathBuf, truth: Truth) -> Self {
        Self {
            path,
            scenario: None,
            truth,
            score: Score::default(),
            outside: 0,
            epochs: Vec::new(),
        }
    }

    /// Read a truth file and check it belongs to the log it will be scored against.
    fn open(path: PathBuf, log: &str) -> Result<Self, Box<dyn Error>> {
        let text = fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let truth = Truth::parse(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        if let (Some(from_log), Some(from_truth)) = (scenario_of(log), scenario_of(&text))
            && from_log != from_truth
        {
            return Err(format!(
                "{}: truth for `{}`, {}, but the log is `{}`, {}",
                path.display(),
                from_truth.0,
                from_truth.1,
                from_log.0,
                from_log.1,
            )
            .into());
        }
        Ok(Self {
            scenario: scenario_of(log),
            ..Self::new(path, truth)
        })
    }

    /// Score one epoch against the truth row within `tolerance` of it, one IMU period, or
    /// record that there was none.
    fn epoch(
        &mut self,
        t: f64,
        tolerance: f64,
        state: &State,
        covariance: &Covariance,
        attitude: AttitudeVariance,
        accuracy: &Accuracy,
    ) {
        // Outside by more than half the truth's own spacing: an IMU that runs on for 22 ms
        // past a 1 Hz reference's last row is not a mismatch; one that runs on past a
        // reference written per epoch is.
        let rows = &self.truth.rows;
        let margin = match rows.as_slice() {
            [a, b, ..] => (b.t - a.t) / 2.0,
            _ => 0.0,
        };
        let covered = rows.first().is_some_and(|row| t >= row.t - margin)
            && rows.last().is_some_and(|row| t <= row.t + margin);
        if !covered {
            self.outside += 1;
        }
        let Some(truth) = self.truth.at(t, tolerance) else {
            self.epochs.push((t, [None; 3], None));
            return;
        };
        let (epsilon, error) = self.score.epoch(state, covariance, &truth, accuracy);
        let mut deviation = [(0.0, 0.0); ERRORS.len()];
        for ((slot, value), (component, _)) in deviation.iter_mut().zip(error).zip(ERRORS) {
            *slot = (value, component.variance(covariance, attitude).sqrt());
        }
        self.epochs.push((t, epsilon, Some(deviation)));
    }

    /// Judge a GNSS fix taken at `t` against the truth at that instant, or `None` where the
    /// truth has no row there or the fix's variance could not be divided by.
    ///
    /// At the antenna, `p + R(q) a` with the truth's own attitude, since that is where the
    /// receiver was: on UrbanNav 0.86 m ahead of the IMU.
    fn judge(
        &self,
        t: f64,
        fix: Position<Ned>,
        variance: [f32; 3],
        antenna: Position<Body>,
        accuracy: &Accuracy,
    ) -> Option<[Judged; 2]> {
        if !variance.iter().all(|v| v.is_finite() && *v > 0.0) {
            return None;
        }
        let truth = self.truth.at_fix(t)?;
        let e = Vector3::from(fix.to_array())
            - (truth.position + truth.attitude * Vector3::from(antenna.to_array()));
        let horizontal = e.x * e.x / variance[0] + e.y * e.y / variance[1];
        let height = e.z * e.z / variance[2];
        let bar = accuracy.position.as_meters();
        Some([
            Judged {
                bad: horizontal > BAD_HORIZONTAL.threshold(),
                far: e.x.abs() > bar || e.y.abs() > bar,
                error: e.xy().norm(),
            },
            Judged {
                bad: height > BAD_HEIGHT.threshold(),
                far: e.z.abs() > bar,
                error: e.z.abs(),
            },
        ])
    }

    /// Truth the log and the file do not share: epochs outside the truth's time range, and
    /// rows an epoch could have been scored at and was not ([`Truth::missed`]).
    ///
    /// Zero on a paired file, a log with a gap in it included. The first catches truth that
    /// stops early; the second a file denser than the log or offset from it, which is how a
    /// 50 Hz log scored against 200 Hz truth shows up here as well as in its header. Between
    /// two rows of a sparse truth an epoch is simply unscored, which is not a mismatch.
    fn unmatched(&self) -> u32 {
        self.outside + self.truth.missed
    }

    /// `<out>.nees.csv`: `ε` per block per epoch, the input `tools/anees.py` averages across
    /// seeds.
    ///
    /// `ε` itself rather than per degree of freedom, because a consistent block's is χ²(3) and
    /// the ensemble bound is a χ² quantile on their sum. The harness computes it and the
    /// aggregator only averages (`AGENTS.md`, one statistic, one implementation), so the
    /// time-averaged `nees_*` on the `score` line and the ensemble bound read one `ε`.
    fn write_nees(&self, out: &mut dyn Write) -> io::Result<()> {
        match &self.scenario {
            Some((name, tag)) => writeln!(out, "# fusion-nav nees for `{name}`, {tag}")?,
            None => writeln!(out, "# fusion-nav nees, no scenario named")?,
        }
        writeln!(
            out,
            "# eps = dx' P^-1 dx per 3-state block, chi-square(3) if consistent; empty where \
             singular or unscored"
        )?;
        writeln!(out, "t_s,nees_pos,nees_vel,nees_att")?;
        let field = |value: Option<f64>| value.map_or_else(String::new, |v| format!("{v:.6}"));
        for (t, [pos, vel, att], _) in &self.epochs {
            writeln!(
                out,
                "{t:.4},{},{},{}",
                field(*pos),
                field(*vel),
                field(*att)
            )?;
        }
        Ok(())
    }

    /// `<out>.error.csv`: the truth error on navigation axes per epoch, each beside the σ the
    /// filter claimed on that axis, which `tools/replay_report.py` draws as error inside ±3σ.
    ///
    /// The same error vector every `score` key reads, so a figure and the RMSE printed under
    /// it cannot describe two different errors. One row per epoch, like `<out>.nees.csv`, and
    /// empty where the truth file had no row rather than dropped.
    fn write_error(&self, out: &mut dyn Write) -> io::Result<()> {
        match &self.scenario {
            Some((name, tag)) => writeln!(out, "# fusion-nav error for `{name}`, {tag}")?,
            None => writeln!(out, "# fusion-nav error, no scenario named")?,
        }
        writeln!(
            out,
            "# err_ = truth - estimate on navigation axes (attitude: rotation vector of q * q^-1, \
             rad), sigma_ the filter's on the same axis; empty where unscored"
        )?;
        write!(out, "t_s")?;
        for (_, name) in ERRORS {
            write!(out, ",err_{name},sigma_{name}")?;
        }
        writeln!(out)?;
        for (t, _, deviation) in &self.epochs {
            write!(out, "{t:.4}")?;
            match deviation {
                Some(deviation) => {
                    for (error, sigma) in deviation {
                        write!(out, ",{error:.6},{sigma:.6}")?;
                    }
                }
                None => write!(out, "{}", ",,".repeat(ERRORS.len()))?,
            }
            writeln!(out)?;
        }
        Ok(())
    }
}

/// The `nu0..nu2,s0..s2` fields of one fusion row: the published innovation and the diagonal
/// of its covariance, padded with empty fields past the observation's dimension, and all six
/// empty where the gate ran no update.
fn innovation_fields(innovation: Option<Innovation>) -> String {
    let column = |values: &[f32], i: usize| {
        values
            .get(i)
            .map_or_else(String::new, |value| format!("{value:.6}"))
    };
    let (nu, s) = innovation
        .as_ref()
        .map_or((&[][..], &[][..]), |i| (i.values(), i.variances()));
    (0..3)
        .map(|i| column(nu, i))
        .chain((0..3).map(|i| column(s, i)))
        .collect::<Vec<_>>()
        .join(",")
}

/// The name the fusion file records for one outcome.
///
/// Exhaustive on purpose. `Fusion` is not `#[non_exhaustive]` — an outcome is matched, and a
/// wildcard arm is the integrator bug the typed outcomes exist to prevent (`AGENTS.md`) — so
/// a variant added later stops here rather than being written out as something else.
fn verdict(outcome: Fusion) -> &'static str {
    match outcome {
        Fusion::Accepted { .. } => "accepted",
        Fusion::Reset => "reset",
        Fusion::Rejected { .. } => "rejected",
        Fusion::NotInitialized => "not_initialized",
        Fusion::NoReference => "no_reference",
        Fusion::Unobservable => "unobservable",
        Fusion::NotFinite => "not_finite",
        Fusion::InvalidNoise => "invalid_noise",
        Fusion::StateInvalid => "state_invalid",
        Fusion::OutOfHorizon { .. } => "out_of_horizon",
    }
}

/// The fusion file's header, and the gates that make its ratios readable.
///
/// The filter reports `r = ε / γ`, so a ratio only becomes `ε` — and `ε` only becomes NIS —
/// if `γ` is known. Recording the gates here rather than expecting a reader to look them up
/// keeps the file self-describing: a `Config` change moves the number in the file that the
/// ratios were produced under.
fn write_fusion_header(out: &mut impl Write, gates: Gates) -> io::Result<()> {
    // Through `thresholds` rather than field by field, so this header and the `γ` the
    // consistency keys divide by cannot disagree about which gate belongs to which source.
    // The pairing is otherwise unguarded: swapping the two `Gate<3>` fields, or the two
    // `Gate<1>`s, compiles and passes every fixture, because one percentile gives equal
    // thresholds within a dimension. The fixture below varies one gate alone, which is what
    // makes a mispaired list visible — and it now covers both readers at once.
    writeln!(
        out,
        "# one row per verdict. gates {}",
        SOURCES
            .iter()
            .zip(thresholds(gates))
            .map(|(name, gamma)| format!("{name}={gamma}"))
            .collect::<Vec<_>>()
            .join(" ")
    )?;
    writeln!(
        out,
        "# nu* and s* are the filter's published innovation and diag(S), empty where the \
         gate ran no update: an adoption, a refusal, a call before initialization"
    )?;
    writeln!(out, "t_s,source,nu0,nu1,nu2,s0,s1,s2,ratio,outcome,truth")
}

fn write_header(out: &mut impl Write) -> io::Result<()> {
    write!(out, "t_s,status")?;
    for (name, _) in ESTIMATE {
        write!(out, ",{name}")?;
    }
    for (_, name) in SIGMAS {
        write!(out, ",{name}")?;
    }
    for (name, _) in ATTITUDE_SIGMAS {
        write!(out, ",{name}")?;
    }
    for name in RATIOS {
        write!(out, ",{name}")?;
    }
    writeln!(out)
}

fn default_input() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("data/flight.csv")
}

fn default_output() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("target/replay.csv")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A test's rotation as the components every `Attitude` constructor takes.
    fn components(q: UnitQuaternion<f32>) -> fusion_nav::Quaternion {
        fusion_nav::Quaternion {
            w: q.w,
            x: q.i,
            y: q.j,
            z: q.k,
        }
    }

    /// The rate the fixtures run at, which is `data/flight.csv`'s.
    const DT: f64 = 0.02;

    /// A still IMU row: no rotation, and specific force reading gravity alone, so
    /// `classify` sees a peak gyro and a peak deviation of exactly zero.
    const STILL: ([f32; 3], [f32; 3]) = ([0.0, 0.0, 0.0], [0.0, 0.0, -GRAVITY]);

    /// A turning one, past `Initialization::max_gyro_rate` of 0.262 rad/s.
    const TURNING: ([f32; 3], [f32; 3]) = ([0.5, 0.0, 0.0], [0.0, 0.0, -GRAVITY]);

    /// Builds a log one row at a time.
    ///
    /// Rows only — it returns no count, no rate and no verdict — so every expected value
    /// below is a literal written beside the assertion that reads it. A fixture that
    /// derives the answer is a fixture checking itself.
    ///
    /// Timestamps carry six decimals, as the converter and `examples/simulate.rs` write
    /// them, so a rate like 4 kHz survives the formatting rather than rounding into a
    /// different one.
    struct Log(String);

    impl Log {
        fn new() -> Self {
            Self("t_s,source,v0,v1,v2,v3,v4,v5,var0,var1,var2\n".to_string())
        }

        fn imu(mut self, t: f64, (gyro, accel): ([f32; 3], [f32; 3])) -> Self {
            self.0 += &format!(
                "{t:.6},imu,{},{},{},{},{},{},,,\n",
                gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]
            );
            self
        }

        /// A raw row, for the malformed and non-finite cases a typed builder would refuse
        /// to express.
        fn raw(mut self, line: &str) -> Self {
            self.0 += line;
            self.0 += "\n";
            self
        }

        fn mag(mut self, t: f64) -> Self {
            self.0 += &format!("{t:.6},mag,0.21,0.02,0.43,,,,0.0025,,\n");
            self
        }

        fn gnss_yaw(mut self, t: f64, heading: f32) -> Self {
            self.0 += &format!("{t:.6},gnss_yaw,{heading},,,,,,0.0025,,\n");
            self
        }

        /// The `# Course sideslip` header line, which has to lead the file to be read.
        fn course(self, sideslip: f32) -> Self {
            Self(format!("# Course sideslip {sideslip} rad\n") + &self.0)
        }

        fn baro(mut self, t: f64, altitude: f32) -> Self {
            self.0 += &format!("{t:.6},baro,{altitude},,,,,,4,,\n");
            self
        }

        fn gnss_pos(mut self, t: f64, north: f32, east: f32, down: f32) -> Self {
            self.0 += &format!("{t:.6},gnss_pos,{north},{east},{down},,,,2.25,2.25,5.625\n");
            self
        }

        fn gnss_vel(mut self, t: f64, north: f32, east: f32, down: f32) -> Self {
            self.0 += &format!("{t:.6},gnss_vel,{north},{east},{down},,,,0.09,0.09,0.09\n");
            self
        }

        /// `rows` IMU rows at a fixed interval, the first at `start`.
        fn run(
            mut self,
            start: f64,
            rows: usize,
            interval: f64,
            sample: ([f32; 3], [f32; 3]),
        ) -> Self {
            for i in 0..rows {
                self = self.imu(start + i as f64 * interval, sample);
            }
            self
        }
    }

    /// Drive a fixture through the harness, keeping the fusion rows it wrote.
    fn drive(log: &Log, scoring: Option<Scoring>) -> Result<(Replay, String), String> {
        drive_at(log, Accuracy::default(), scoring)
    }

    /// [`drive`] against a mission bar other than the default.
    fn drive_at(
        log: &Log,
        accuracy: Accuracy,
        scoring: Option<Scoring>,
    ) -> Result<(Replay, String), String> {
        drive_with(
            log,
            Config {
                accuracy,
                ..Config::default()
            },
            scoring,
        )
    }

    /// [`drive`] against a whole `Config`, for the fixtures whose subject is a tuning knob
    /// rather than the log.
    fn drive_with(
        log: &Log,
        config: Config,
        scoring: Option<Scoring>,
    ) -> Result<(Replay, String), String> {
        drive_under(log, config, RPolicy::Raw, scoring)
    }

    /// [`drive_with`] under an `R` policy other than the default.
    fn drive_under(
        log: &Log,
        config: Config,
        policy: RPolicy,
        scoring: Option<Scoring>,
    ) -> Result<(Replay, String), String> {
        let mut replay = Replay::new(config, policy, scoring).map_err(|e| e.to_string())?;
        assert!(
            replay
                .filter
                .set_magnetic_declination(Radians::from_radians(-0.06))
        );
        // Read from the log as `run` reads it, so a fixture turns the course on the way the
        // simulator does.
        replay.course = sideslip_of(&log.0);
        let mut fusions = Vec::new();
        {
            let mut out = Sinks {
                epochs: &mut io::sink(),
                fusions: &mut fusions,
            };
            for line in log.0.lines() {
                replay.row(line, &mut out).map_err(|e| e.to_string())?;
            }
            replay.finish(&mut out).map_err(|e| e.to_string())?;
        }
        Ok((replay, String::from_utf8(fusions).expect("utf-8")))
    }

    /// Replay a fixture, or report the row that stopped it.
    fn try_replay(log: &Log) -> Result<Replay, String> {
        drive(log, None).map(|(replay, _)| replay)
    }

    /// The fusion rows a fixture produced, header excluded — `write_fusion_header` is not
    /// part of what `Replay` writes, and is checked on its own.
    fn fusion_rows(log: &Log) -> Vec<String> {
        match drive(log, None) {
            Ok((_, rows)) => rows.lines().map(str::to_string).collect(),
            Err(e) => panic!("fixture replays: {e}"),
        }
    }

    fn replay(log: &Log) -> Replay {
        match try_replay(log) {
            Ok(replay) => replay,
            Err(e) => panic!("fixture replays: {e}"),
        }
    }

    /// The error a fixture stops on. A function rather than `expect_err`, which would want
    /// `Debug` on `Replay` — a derive the example carries only for these two tests.
    fn replay_error(log: &Log) -> String {
        match try_replay(log) {
            Ok(_) => panic!("the fixture was expected to stop"),
            Err(e) => e,
        }
    }

    /// One `key=value` pair off a `summary` line.
    ///
    /// Per key rather than by comparing whole lines: #20 and #64 add keys next, and
    /// expectations are matched pair by pair as substrings anyway (`AGENTS.md`), so a
    /// whole-line assertion would pin something the manifest itself does not.
    fn key<'a>(summary: &'a str, name: &str) -> &'a str {
        summary
            .split_whitespace()
            .find_map(|pair| pair.strip_prefix(&format!("{name}=")))
            .unwrap_or_else(|| panic!("summary has no `{name}=`: {summary}"))
    }

    /// A still log long enough to initialize: 100 samples covers the 2 s
    /// `Initialization::min_duration` at 50 Hz, and the first 65 of them fix the rate.
    fn still_start() -> Log {
        Log::new().run(0.0, 100, DT, STILL)
    }

    /// [`still_start`], rounded up to whole runs, with the barometer reading each of `altitudes` in turn, spread through
    /// it. Interleaved rather than written up front, because the harness holds the last reading
    /// on every IMU sample, and readings that all arrive before the first one are one reading
    /// held — which sets no reference.
    fn still_start_with_baro(altitudes: &[f32]) -> Log {
        let rows = 100_usize.div_ceil(altitudes.len());
        let mut log = Log::new();
        for (k, altitude) in altitudes.iter().enumerate() {
            let t = (k * rows) as f64 * DT;
            log = log.baro(t, *altitude).run(t, rows, DT, STILL);
        }
        log
    }

    // ---- the row parser ----

    /// The two magnitudes the comment on `Record::parse` quantifies, pinned so the figures
    /// stay checkable: at 18000 s a 400 Hz step reads short, and past 65536 s it is gone.
    #[test]
    fn a_timestamp_past_the_f32_mantissa_keeps_its_step() {
        for (t, reads) in [("18000", 0.001_953_125_f32), ("65536", 0.0)] {
            // Bound rather than inlined: a `Record` borrows the line it parsed.
            let (first, second) = (
                format!("{t}.000000,imu,0,0,0,0,0,-9.80665,,,"),
                format!("{t}.002500,imu,0,0,0,0,0,-9.80665,,,"),
            );
            let a = Record::parse(&first).expect("parses");
            let b = Record::parse(&second).expect("parses");
            assert!(
                (b.t - a.t - 0.0025).abs() < 1e-9,
                "at {t} s the parse keeps a 2.5 ms step: {} s",
                b.t - a.t
            );
            // What the widening bought, named so the test rules out the alternative rather
            // than only confirming the choice.
            assert_eq!(
                b.t as f32 - a.t as f32,
                reads,
                "at {t} s the same step in f32 reads {reads} s"
            );
        }
    }

    #[test]
    fn a_blank_cell_is_absent_rather_than_zero() {
        let r = Record::parse("1.0,baro,0.0273,,,,,,4,,").expect("parses");
        assert_eq!(r.value(0), Ok(0.0273));
        assert!(r.value(1).is_err(), "v1 is blank, not zero");
        assert_eq!(r.variance(0), Ok(4.0));
    }

    #[test]
    fn a_row_missing_a_variance_names_the_column_it_wants() {
        let r = Record::parse("1.0,gnss_pos,0,0,0,,,,2.25,2.25,").expect("parses");
        assert_eq!(
            r.variance(2),
            Err("`gnss_pos` row has no var2".to_string()),
            "the message names the column, because the converter is what has to fix it"
        );
    }

    #[test]
    fn columns_past_the_measurement_time_are_ignored() {
        let r = Record::parse("1.0,imu,0,0,0,0,0,-9.80665,,,,,extra,columns").expect("parses");
        assert_eq!(r.value(5), Ok(-9.80665));
    }

    #[test]
    fn a_measurement_is_taken_at_its_own_time_or_else_at_its_rows() {
        let late = Record::parse("2.0,baro,1,,,,,,4,,,1.89").expect("parses");
        assert_eq!(late.taken(), Timestamp::from_micros(1_890_000));
        for row in ["2.0,baro,1,,,,,,4,,,", "2.0,baro,1,,,,,,4,,"] {
            let r = Record::parse(row).expect("parses");
            assert_eq!(r.taken(), Timestamp::from_micros(2_000_000), "{row}");
        }
    }

    #[test]
    fn a_short_row_leaves_the_remaining_columns_absent() {
        let r = Record::parse("1.0,imu,0,0,0").expect("parses");
        assert_eq!(r.value(2), Ok(0.0));
        assert!(r.value(3).is_err(), "the row stopped before v3");
    }

    #[test]
    fn a_non_numeric_field_fails_the_row() {
        assert!(Record::parse("1.0,imu,0,0,0,0,0,-,,,").is_none());
        assert!(Record::parse("not-a-time,imu,0,0,0,0,0,-9.8,,,").is_none());
    }

    // ---- the rate estimator ----

    #[test]
    fn the_rate_is_the_median_of_the_intervals_not_the_mean() {
        // The burst one corpus log logs in: intervals of 2.5 ms against a true period of
        // 20 ms. 30 short and 34 long over the 64 the probe takes, so the median is 20 ms
        // and the mean is 11.8 ms — 50 Hz against the 85 Hz a mean would report.
        let mut log = Log::new();
        let mut t = 0.0;
        log = log.imu(t, STILL);
        for i in 0..64 {
            t += if i % 2 == 1 && i < 60 { 0.0025 } else { 0.02 };
            log = log.imu(t, STILL);
        }
        // Enough further samples to fill the window, so the burst is shown not to disturb
        // the sizing either.
        for _ in 0..40 {
            t += 0.02;
            log = log.imu(t, STILL);
        }
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "rate"), "50");
        assert_eq!(key(&summary, "align"), "static", "a burst is not motion");
    }

    #[test]
    fn a_dropout_does_not_stretch_the_estimated_rate() {
        // Four half-second SD-card dropouts among 60 ordinary intervals. The median is
        // untouched; a mean would read 20 Hz.
        let mut log = Log::new();
        let mut t = 0.0;
        log = log.imu(t, STILL);
        for i in 0..64 {
            t += if i % 16 == 15 { 0.5 } else { 0.02 };
            log = log.imu(t, STILL);
        }
        assert_eq!(key(&replay(&log).summary(), "rate"), "50");
    }

    #[test]
    fn a_repeated_timestamp_is_not_an_interval() {
        // Two duplicates per distinct timestamp, so zero steps outnumber real ones two to
        // one. Counted, they would take the median to zero and the rate to infinity.
        let mut log = Log::new();
        for i in 0..70 {
            let t = i as f64 * DT;
            log = log.imu(t, STILL).imu(t, STILL).imu(t, STILL);
        }
        assert_eq!(key(&replay(&log).summary(), "rate"), "50");
    }

    #[test]
    fn a_log_shorter_than_the_probe_never_fixes_a_rate() {
        let log = Log::new().run(0.0, 2, DT, STILL);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "rate"), "0", "one interval is not 64");
        assert_eq!(key(&summary, "window"), "0");
        assert_eq!(key(&summary, "align"), "none");
        assert_eq!(key(&summary, "epochs"), "0");
    }

    #[test]
    fn a_rate_whose_window_would_not_fit_the_buffer_is_an_error() {
        // 4 kHz needs 8000 samples to cover 2 s, and the window holds 1024. Reported
        // rather than silently truncated: a short window is a worse alignment, quietly.
        let log = Log::new().run(0.0, 66, 0.00025, STILL);
        let error = replay_error(&log);
        assert!(
            error.contains("needs 8000 samples") && error.contains("holds 1024"),
            "the error says how short the buffer is: {error}"
        );
    }

    // ---- the verdict keys ----

    #[test]
    fn the_antenna_comes_from_its_header_line_in_body_axes() {
        let named = "# GNSS antenna 0.100 -0.300 -0.057 m (forward, right, down)\nt_s\n";
        assert_eq!(antenna_of(named), Some(Position::body(0.1, -0.3, -0.057)));
        assert_eq!(antenna_of("# GNSS antenna 1 2\nt_s\n"), None);
        assert_eq!(antenna_of("t_s,source\n# GNSS antenna 1 2 3\n"), None);
    }

    #[test]
    fn the_site_comes_from_the_origin_header_and_a_log_without_a_fix_has_none() {
        let named = "# Navigation origin 56.410000000 43.760000000 150.000 (lat deg, ...)\nt_s\n";
        let site = origin_of(named).expect("three numbers");
        assert!((site.latitude_deg() - 56.41).abs() < 1e-9);
        assert!((site.longitude_deg() - 43.76).abs() < 1e-9);
        assert_eq!(site.height(), 150.0);
        assert_eq!(
            origin_of("# Navigation origin none (no 3D fix)\nt_s\n"),
            None
        );
        assert_eq!(origin_of("t_s,source\n# Navigation origin 1 2 3\n"), None);
        assert_eq!(origin_of("# nothing\n"), None);
    }

    #[test]
    fn the_declination_comes_from_the_header_and_defaults_to_zero() {
        let named =
            "# Converted from x.ulg\n# Magnetic declination 0.239919 rad (13.75 deg)\nt_s\n";
        assert!((declination_of(named).as_radians() - 0.239_919).abs() < 1e-6);
        // Only the header: a data row carrying the same words is not a declination.
        let late = "t_s,source\n# Magnetic declination 0.5 rad\n";
        assert_eq!(declination_of(late), Radians::ZERO);
        assert_eq!(declination_of("# nothing\n"), Radians::ZERO);
    }

    #[test]
    fn the_gnss_noise_parameters_come_from_the_header_or_not_at_all() {
        let header = "# GNSS noise parameters: EKF2_GPS_P_NOISE 0.5, EKF2_GPS_V_NOISE 0.25, \
                      EKF2_NOAID_NOISE 10 (PX4 default; not in the log)\nt_s\n";
        assert_eq!(
            gnss_noise_of(header),
            Some(GnssNoiseParameters {
                position: 0.5,
                velocity: 0.25,
                no_aid: 10.0,
            })
        );
        // A line missing one parameter is refused whole rather than defaulted here: the
        // converter is where a default is chosen and said.
        let partial = "# GNSS noise parameters: EKF2_GPS_P_NOISE 0.5, EKF2_GPS_V_NOISE 0.3\n";
        assert_eq!(gnss_noise_of(partial), None);
        assert_eq!(
            gnss_noise_of("t_s\n# GNSS noise parameters: EKF2_GPS_P_NOISE 1\n"),
            None
        );
    }

    #[test]
    fn px4_floors_a_gnss_heading_at_ekf2_s_0_1_rad() {
        let px4 = RPolicy::Px4(GnssNoiseParameters {
            position: 0.5,
            velocity: 0.3,
            no_aid: 10.0,
        });
        // `a299e722` logs no accuracy, which the converter writes as the floor already; a
        // receiver claiming zero is floored here as PX4 floors it, and raw passes it through to
        // be refused.
        assert_eq!(px4.heading(0.0), HeadingNoise::from_sigma(0.1));
        assert_eq!(px4.heading(0.25), HeadingNoise::from_variance(0.25));
        assert_eq!(RPolicy::Raw.heading(0.0), HeadingNoise::from_variance(0.0));
        assert!(!px4.heading(f32::NAN).variance().is_finite());
    }

    #[test]
    fn px4_floors_each_axis_by_its_own_rule() {
        let px4 = RPolicy::Px4(GnssNoiseParameters {
            position: 0.5,
            velocity: 0.3,
            no_aid: 10.0,
        });
        let sigmas = |noise: [f32; 3]| noise.map(|v| (v.sqrt() * 1000.0).round() / 1000.0);
        // A 5 cm RTK fix: horizontal floored at p, vertical at 1.5 p — not at p, which is
        // what one floor for both axes would give.
        let fix = px4.position([0.0025, 0.0025, 0.0025]).variance();
        assert_eq!(sigmas(fix), [0.5, 0.5, 0.75]);
        // A 40 m claim is capped at `no_aid` on both axes.
        let wild = px4.position([1600.0, 1600.0, 1600.0]).variance();
        assert_eq!(sigmas(wild), [10.0, 10.0, 10.0]);
        // Velocity floors first, then widens vertical: 0.1 m/s reads 0.3 / 0.45, where
        // widening first and flooring after would read 0.3 / 0.3.
        let slow = px4.velocity([0.01, 0.01, 0.01]).variance();
        assert_eq!(sigmas(slow), [0.3, 0.3, 0.45]);
        // A NaN is not floored into a fix: `f32::max` would return the floor.
        assert!(px4.position([f32::NAN, 1.0, 1.0]).variance()[0].is_nan());
        assert!(px4.velocity([-1.0, 1.0, 1.0]).variance()[0] < 0.0);
        // `Raw` is the row, untouched.
        let raw = RPolicy::Raw.velocity([0.01, 0.01, 0.01]).variance();
        assert_eq!(sigmas(raw), [0.1, 0.1, 0.1]);
    }

    #[test]
    fn a_still_window_aligns_statically() {
        let summary = replay(&still_start()).summary();
        assert_eq!(key(&summary, "align"), "static");
        assert_eq!(key(&summary, "window"), "100", "2 s at 50 Hz");
        assert_eq!(key(&summary, "an"), "none", "a static start reports no ā_n");
    }

    #[test]
    fn a_magnetometer_in_the_window_makes_the_heading_valid() {
        let log = Log::new().mag(0.0).run(0.0, 100, DT, STILL);
        assert_eq!(key(&replay(&log).summary(), "heading"), "valid");
    }

    #[test]
    fn no_magnetometer_in_the_window_leaves_the_heading_invalid() {
        // Stillness observes tilt and never yaw, and the covariance cannot say so: the
        // 0.35 rad of `Initialization::sigma_yaw` clears `Accuracy::heading`'s 0.52 outright.
        assert_eq!(key(&replay(&still_start()).summary(), "heading"), "invalid");
    }

    #[test]
    fn the_attitude_keys_report_the_tilt_the_window_was_held_at() {
        // A vehicle parked 10° right wing down and 5° nose down reads
        // `f = R₀ᵀ(−g)`, and (5) must give those two angles back. Written out rather
        // than rotated here, so the fixture is not the equation checking itself.
        const TILTED: ([f32; 3], [f32; 3]) =
            ([0.0, 0.0, 0.0], [-0.854_706, -1.696_427, -9.620_915]);
        let summary = replay(&Log::new().run(0.0, 100, DT, TILTED)).summary();
        assert_eq!(key(&summary, "align"), "static", "a tilt is not motion");
        assert_eq!(key(&summary, "roll0"), "10.00");
        assert_eq!(key(&summary, "pitch0"), "-5.00");
    }

    #[test]
    fn the_excursion_keys_read_the_receiver_horizontally_from_its_first_fix() {
        // Extent from the first fix, not from the origin: measured from (0, 0) the last fix
        // would read 7.3. Speed horizontal: the 10 m/s climb would make it 11.2 in 3-D.
        let log = still_start()
            .gnss_pos(2.0, 1.0, 1.0, 0.0)
            .gnss_pos(3.0, 4.0, 5.0, -50.0)
            .gnss_pos(4.0, -2.0, 7.0, 0.0)
            .gnss_vel(4.0, 3.0, 4.0, -10.0);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "extent"), "6.7");
        assert_eq!(key(&summary, "speed_max"), "5.0");
    }

    #[test]
    fn the_tilt_key_is_the_angle_between_body_down_and_navigation_down() {
        // The parked vehicle of the test above, 10° roll and 5° pitch: acos(cos 10° cos 5°)
        // is 11.17°, where the larger Euler angle alone would read 10.0.
        const TILTED: ([f32; 3], [f32; 3]) =
            ([0.0, 0.0, 0.0], [-0.854_706, -1.696_427, -9.620_915]);
        let summary = replay(&Log::new().run(0.0, 100, DT, TILTED)).summary();
        assert_eq!(key(&summary, "tilt_max"), "11.2");
        assert_eq!(key(&summary, "extent"), "none", "no receiver, no extent");
        assert_eq!(key(&summary, "speed_max"), "none");
    }

    #[test]
    fn the_heading_key_carries_the_declination_the_filter_was_configured_with() {
        // The fixture's field, levelled, is 0.095 rad east of its own north, and the
        // harness configures −0.06 rad of declination: −8.88° of true heading. A
        // declination that stopped reaching the filter would read −5.45° here.
        let log = Log::new().mag(0.0).run(0.0, 100, DT, STILL);
        assert_eq!(key(&replay(&log).summary(), "yaw0"), "-8.88");
    }

    #[test]
    fn a_window_with_no_magnetometer_reports_a_heading_of_zero() {
        assert_eq!(key(&replay(&still_start()).summary(), "yaw0"), "0.00");
    }

    #[test]
    fn an_angle_just_below_zero_does_not_report_itself_as_negative_zero() {
        // 10 µm s⁻² of specific force on the forward axis is −5.8e-5° of pitch, which
        // `{:.2}` alone renders as `-0.00`. The manifest matches these pairs as
        // substrings, so that sign would read as a moved expectation and is not one.
        const BARELY: ([f32; 3], [f32; 3]) = ([0.0, 0.0, 0.0], [-1.0e-5, 0.0, -GRAVITY]);
        let summary = replay(&Log::new().run(0.0, 100, DT, BARELY)).summary();
        assert_eq!(key(&summary, "pitch0"), "0.00");
    }

    #[test]
    fn a_barometer_in_a_still_window_sets_the_reference() {
        // Three readings, since one held across the window is one reading and sets no
        // reference: it has no scatter to give the reference a variance from.
        let log = still_start_with_baro(&[41.5, 42.5, 42.0]);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "alpha0"), "window");
        let reference = replay.filter.baro_reference().expect("α₀ established");
        assert!(
            (reference.as_meters() - 42.0).abs() < 1e-3,
            "the reference is the window's mean altitude: {} m",
            reference.as_meters()
        );
    }

    /// Airborne from the first sample, so the harness starts coarse at `PATIENCE`.
    fn coarse_start() -> Log {
        Log::new().run(0.0, 501, DT, TURNING)
    }

    #[test]
    fn a_course_is_fused_after_each_velocity_once_a_sideslip_is_given() {
        // A coarse start adopts the first velocity and the first course; the second of each
        // reaches the gate, which is where the dimension `AXES` names is published.
        let moving = |log: Log| {
            log.gnss_vel(10.02, 15.0, 0.0, 0.0)
                .imu(10.04, TURNING)
                .gnss_vel(10.06, 15.0, 0.0, 0.0)
                .imu(10.08, TURNING)
                .gnss_vel(10.10, 15.0, 0.0, 0.0)
        };
        let fused = replay(&moving(coarse_start().course(0.05)));
        let summary = fused.summary();
        assert_eq!(key(&summary, "course"), "2.9");
        assert_eq!(fused.filter.diagnostics().course.adopted, 1, "{summary}");
        assert_eq!(fused.consistency.dimension[COURSE], AXES[COURSE].len());

        // No sideslip, no course: the neutral replay every corpus log is pinned under.
        let neutral = replay(&moving(coarse_start()));
        assert_eq!(key(&neutral.summary(), "course"), "off");
        assert!(!neutral.filter.diagnostics().course.has_been_used());
    }

    #[test]
    fn a_source_named_by_without_is_never_offered() {
        let log = still_start().mag(2.0).mag(2.1);
        let mut kept = Replay::new(Config::default(), RPolicy::Raw, None).unwrap();
        let mut dropped = Replay::new(Config::default(), RPolicy::Raw, None).unwrap();
        dropped.without = Some("mag".to_string());
        for replay in [&mut kept, &mut dropped] {
            let mut sinks = Sinks {
                epochs: &mut io::sink(),
                fusions: &mut io::sink(),
            };
            for line in log.0.lines() {
                replay.row(line, &mut sinks).expect("fixture replays");
            }
        }
        assert!(kept.filter.diagnostics().mag_heading.has_been_used());
        assert!(!dropped.filter.diagnostics().mag_heading.has_been_used());
        assert_eq!(key(&dropped.summary(), "without"), "mag");
    }

    #[test]
    fn a_replay_under_set_names_what_it_set() {
        let log = still_start().mag(2.0);
        assert_eq!(key(&replay(&log).summary(), "set"), "none");
        let options = Options {
            sets: vec![("gravity".into(), "9.79".into())],
            ..Options::default()
        };
        let mut set = prepare(&log.0, options.config().expect("a field"), &options, None)
            .expect("a usable header");
        super::drive(
            &mut set,
            &log.0,
            Path::new("fixture"),
            &mut Sinks {
                epochs: &mut io::sink(),
                fusions: &mut io::sink(),
            },
        )
        .expect("fixture replays");
        assert_eq!(set.filter.config().gravity, 9.79);
        assert_eq!(key(&set.summary(), "set"), "gravity:9.79");
    }

    #[test]
    fn a_window_without_a_barometer_takes_the_reference_from_the_estimate() {
        let log = still_start().baro(2.0, 42.0);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "alpha0"), "estimate");
        assert_eq!(key(&replay.summary(), "discarded"), "0");
    }

    #[test]
    fn a_coarse_start_sets_the_reference_at_the_first_altitude_after_a_fix() {
        // `cd7e0001`'s shape: a coarse start, a fix, then the barometer.
        let log = coarse_start()
            .gnss_pos(10.02, 1.0, 2.0, -3.0)
            .baro(10.04, 42.0)
            .imu(10.04, TURNING);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "alpha0"), "estimate");
        assert_eq!(key(&replay.summary(), "discarded"), "0");
    }

    #[test]
    fn a_coarse_start_with_no_fix_refuses_altitude() {
        // Nothing establishes position, so there is no estimate to read `α₀` against, and
        // `alpha0=` is the only key that would notice the barometer going unused.
        let log = coarse_start().baro(10.02, 42.0).imu(10.02, TURNING);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "alpha0"), "none");
        let (_, baro) = replay
            .filter
            .diagnostics()
            .sources()
            .into_iter()
            .find(|(name, _)| *name == "baro_altitude")
            .expect("a barometer source");
        assert_eq!(baro.last_refusal, Some(Refusal::NoReference));
        assert_eq!(
            baro.accepted, 0,
            "no altitude is referred to an invented origin"
        );
    }

    #[test]
    fn a_log_still_for_less_than_a_window_falls_back_to_its_still_start() {
        // 80 still samples, 1.58 s, then turning: never a full window to be static, so at
        // `PATIENCE` the start is the still prefix rather than a window taken in motion,
        // dated where the prefix ended. Three barometer readings in the still stretch, so
        // `alpha0=` can say the short window was taken at rest.
        let log = Log::new()
            .baro(0.0, 41.5)
            .run(0.0, 30, DT, STILL)
            .baro(0.6, 42.5)
            .run(0.6, 30, DT, STILL)
            .baro(1.2, 42.0)
            .run(1.2, 20, DT, STILL)
            .run(1.6, 450, DT, TURNING);
        let replay = replay(&log);
        let summary = replay.summary();
        assert_eq!(key(&summary, "align"), "short");
        assert_eq!(key(&summary, "window"), "80");
        assert_eq!(key(&summary, "alpha0"), "window");
        let at = replay.initialized_at.expect("committed");
        assert!(
            (at - 1.58).abs() < 1e-9,
            "at the last still sample, not {at}"
        );
    }

    #[test]
    fn a_static_window_after_a_bump_beats_the_still_start_before_it() {
        // Still for 80 samples, bumped for one, then still for 100: the prefix is kept as
        // a fallback and never used, because a full static window arrives inside
        // `PATIENCE`. The fix that arrived while it waited is released as the refusal it
        // was; drop the release and the fusion file loses it.
        let log = Log::new()
            .run(0.0, 80, DT, STILL)
            .imu(1.6, TURNING)
            .gnss_pos(1.61, 1.0, 2.0, -3.0)
            .run(1.62, 100, DT, STILL);
        let (replay, rows) = drive(&log, None).expect("fixture replays");
        assert_eq!(key(&replay.summary(), "align"), "static");
        assert_eq!(key(&replay.summary(), "window"), "100");
        assert!(
            replay.fallback.is_none(),
            "released once the static start committed"
        );
        assert_eq!(rows.matches("not_initialized").count(), 2, "{rows}");
    }

    #[test]
    fn a_fallback_replays_what_came_after_the_prefix_once() {
        // A fix during the wait is refused as not initialized, then replayed onto the
        // committed prefix: one fusion row for it, not two, and the fusion file carries no
        // trace of the refusal the rewind discarded.
        let log = Log::new()
            .run(0.0, 80, DT, STILL)
            .run(1.6, 200, DT, TURNING)
            .gnss_pos(5.6, 1.0, 2.0, -3.0)
            .run(5.6, 250, DT, TURNING);
        let (replay, rows) = drive(&log, None).expect("fixture replays");
        assert_eq!(key(&replay.summary(), "align"), "short");
        assert_eq!(
            rows.lines().count(),
            2,
            "one row each for position and height"
        );
        assert!(!rows.contains("not_initialized"), "{rows}");
    }

    #[test]
    fn a_log_that_ends_before_patience_writes_its_refusals() {
        // Nothing decides the fallback, so the log never initializes and the fix it held
        // is written as the refusal it was.
        let log = Log::new()
            .run(0.0, 80, DT, STILL)
            .run(1.6, 50, DT, TURNING)
            .gnss_pos(2.6, 1.0, 2.0, -3.0);
        let (replay, rows) = drive(&log, None).expect("fixture replays");
        assert!(!replay.filter.is_initialized());
        assert_eq!(rows.lines().count(), 2, "{rows}");
    }

    #[test]
    fn a_log_that_moves_before_its_rate_is_fixed_waits_for_patience() {
        // 30 still samples, fewer than the 65 the rate probe needs: no prefix was ever read
        // still, so there is nothing to commit at the onset and the harness waits.
        let log = Log::new()
            .run(0.0, 30, DT, STILL)
            .run(0.6, 471, DT, TURNING);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "align"), "coarse");
        assert_eq!(replay.initialized_at, Some(PATIENCE));
    }

    #[test]
    fn a_moving_log_waits_for_patience_then_starts_coarse() {
        // Turning throughout, so no window is ever static and the harness's own policy —
        // not the filter's — decides when to stop waiting.
        let log = Log::new().run(0.0, 501, DT, TURNING);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "align"), "coarse");
        assert_eq!(
            replay.initialized_at,
            Some(PATIENCE),
            "committed on the first sample past PATIENCE, not before"
        );
    }

    #[test]
    fn two_gnss_velocities_in_a_moving_window_measure_the_inertial_acceleration() {
        // The window is the 100 samples before the commit at t = 10 s, so these two land
        // inside it, 1 s apart, 2 m/s apart: ā_n is 2 m/s² north.
        let mut log = Log::new();
        for i in 0..=500 {
            let t = i as f64 * DT;
            log = match i {
                425 => log.gnss_vel(t, 1.0, 0.0, 0.0),
                475 => log.gnss_vel(t, 3.0, 0.0, 0.0),
                _ => log,
            };
            log = log.imu(t, TURNING);
        }
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "an"), "measured");
        let accel = replay.inertial_accel().expect("ā_n measured");
        assert!(
            (accel.x() - 2.0).abs() < 1e-3 && Vector3::from(accel.to_array()).norm() > 1.9,
            "2 m/s over 1 s, north: {:?}",
            Vector3::from(accel.to_array())
        );
    }

    #[test]
    fn a_moving_window_without_two_velocities_measures_nothing() {
        // One velocity spans no time, and a difference over zero seconds is not an
        // acceleration. `an=none` covers this as well as a static start, which is why the
        // manifest pins it per log rather than deriving it.
        let mut log = Log::new();
        for i in 0..=500 {
            let t = i as f64 * DT;
            if i == 450 {
                log = log.gnss_vel(t, 1.0, 0.0, 0.0);
            }
            log = log.imu(t, TURNING);
        }
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "align"), "coarse");
        assert_eq!(key(&summary, "an"), "none");
    }

    // ---- convergence ----

    #[test]
    fn a_window_carrying_a_magnetometer_is_aligned_when_it_closes() {
        // Zero seconds, not one `dt`: the window observed both tilt and heading, so the
        // filter is aligned the instant it commits.
        let log = Log::new().mag(0.0).run(0.0, 100, DT, STILL);
        assert_eq!(key(&replay(&log).summary(), "aligned_at"), "0.00");
    }

    #[test]
    fn a_heading_nobody_observed_aligns_when_one_is_fused() {
        // Stillness observes tilt and never yaw, so this window leaves `heading=invalid`
        // and the attitude waits for a magnetometer — 0.52 s later here.
        let mut log = still_start();
        for i in 0..100 {
            let t = 2.0 + i as f64 * DT;
            if i == 25 {
                log = log.mag(t);
            }
            log = log.imu(t, STILL);
        }
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "heading"), "invalid", "as the window left it");
        assert_eq!(
            key(&summary, "aligned_at"),
            "0.52",
            "2.50 s, less the 1.98 s the window closed at: {summary}"
        );
    }

    #[test]
    fn a_vehicle_with_no_magnetometer_never_aligns() {
        // A correct answer rather than a slow one, which is why it stays a word: no
        // covariance shrinks yaw, so no number would ever arrive.
        let log = still_start().run(2.0, 100, DT, STILL);
        assert_eq!(key(&replay(&log).summary(), "aligned_at"), "never");
    }

    #[test]
    fn an_unaided_attitude_stops_being_valid_and_the_line_says_when() {
        // The covariance growth of (16)–(22) on a log with no truth: a window that observed
        // both tilt and heading aligns at 0.00 and the tilt variance then crosses
        // `Accuracy::tilt` 3.83 s later, which is the figure `Accuracy`'s defaults cite. Six
        // seconds of stillness at 50 Hz is enough to see it.
        let log = Log::new().mag(0.0).run(0.0, 400, DT, STILL);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "aligned_at"), "0.00");
        assert_eq!(
            key(&summary, "attitude_lost"),
            "3.84",
            "one epoch past 3.83 s at this rate: {summary}"
        );
    }

    #[test]
    fn a_mission_bar_moves_attitude_lost_and_leaves_alignment_alone() {
        // `aligned_at=` and `transitions=` answer whether the start resolved, which the
        // filter decides against `ALIGNED_TILT`; `attitude_lost=` answers whether the output
        // met the mission's bar. A 1° tilt bar sits under the 20 mrad prior, so the output is
        // out of service at 0.00 while nothing about alignment moves. The mutation it
        // catches: the harness reading `aligned_at` off `Validity::attitude`, which reads
        // `never` here.
        let log = Log::new().mag(0.0).run(0.0, 400, DT, STILL);
        let default = replay(&log).summary();
        let tight = match drive_at(
            &log,
            Accuracy {
                tilt: Radians::from_degrees(1.0),
                ..Accuracy::default()
            },
            None,
        ) {
            Ok((replay, _)) => replay.summary(),
            Err(e) => panic!("fixture replays: {e}"),
        };

        for unmoved in ["aligned_at", "transitions", "status"] {
            assert_eq!(
                key(&tight, unmoved),
                key(&default, unmoved),
                "{unmoved}: {tight}"
            );
        }
        assert_eq!(key(&default, "attitude_lost"), "3.84");
        assert_eq!(key(&tight, "attitude_lost"), "0.00", "{tight}");
    }

    #[test]
    fn a_log_too_short_to_lose_its_attitude_says_never() {
        // The other `never`: aligned at 0.00 and still valid when the log ended, which is
        // not the same answer as a filter that never aligned, and reads differently from it
        // only because `aligned_at` carries a number.
        let log = Log::new().mag(0.0).run(0.0, 100, DT, STILL);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "aligned_at"), "0.00");
        assert_eq!(key(&summary, "attitude_lost"), "never", "{summary}");
    }

    #[test]
    fn a_filter_that_never_aligns_loses_nothing() {
        let log = still_start().run(2.0, 100, DT, STILL);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "aligned_at"), "never");
        assert_eq!(key(&summary, "attitude_lost"), "never", "{summary}");
    }

    #[test]
    fn a_log_that_never_initializes_never_aligns() {
        let log = Log::new().run(0.0, 2, DT, STILL);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "align"), "none");
        assert_eq!(key(&summary, "aligned_at"), "never");
    }

    // ---- the counters ----

    /// The three variants that reached no gate. Each is aiding that is silently not there:
    /// `alpha0=` was added because 35575 barometer rows went from fused to `NoReference`
    /// and nothing on this line noticed.
    #[test]
    fn everything_that_never_reached_the_gate_is_discarded() {
        let no_reference = coarse_start()
            .raw("10.020000,baro,42,,,,,,4,,")
            .imu(10.02, TURNING);
        let summary = replay(&no_reference).summary();
        assert_eq!(key(&summary, "discarded"), "1", "no reference: {summary}");
        assert_eq!(key(&summary, "rejected"), "0", "{summary}");
        for (row, what) in [
            (
                "2.000000,gnss_pos,0,0,0,,,,0,2.25,5.625",
                "a variance of zero",
            ),
            (
                "2.000000,gnss_pos,nan,0,0,,,,2.25,2.25,5.625",
                "a NaN in the measurement",
            ),
        ] {
            let summary = replay(&still_start().raw(row)).summary();
            assert_eq!(key(&summary, "discarded"), "1", "{what}: {summary}");
            assert_eq!(
                key(&summary, "rejected"),
                "0",
                "{what} is not a gate verdict: {summary}"
            );
        }
    }

    #[test]
    fn aiding_offered_before_initialization_is_not_discarded() {
        // `Diagnostics` resets when the window commits, and this is what that buys: the
        // count afterwards is a statement about the flight, not about how early the
        // harness started offering rows.
        let mut log = Log::new();
        for i in 0..100 {
            let t = i as f64 * DT;
            log = log.gnss_pos(t, 0.0, 0.0, 0.0).imu(t, STILL);
        }
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "align"), "static");
        assert_eq!(
            key(&summary, "discarded"),
            "0",
            "100 fixes were refused as early, and none of them is a fault: {summary}"
        );
    }

    #[test]
    fn a_fix_the_gate_turns_down_is_counted_as_rejected() {
        // A kilometre off a still start whose position σ is metres. The consistent fix
        // beside it is what shows the count is the gate's and not every fix's.
        let log = still_start()
            .gnss_pos(2.0, 1.0, 2.0, -3.0)
            .gnss_pos(2.04, 1000.0, 0.0, 0.0);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "rejected"), "1", "{summary}");
        assert_eq!(key(&replay(&still_start()).summary(), "rejected"), "0");
    }

    #[test]
    fn r_is_fused_as_the_row_reports_it_however_small() {
        // `r_policy=raw` as a fixture rather than a restatement of its own constant: the
        // same 1 m/s innovation twice, differing only in the variance beside it. At
        // σ_v = 0.01 m/s the gate turns it down; at 0.5, the highest `EKF2_GPS_V_NOISE` the
        // corpus flies, the same innovation is accepted. So a floor reaching a `Raw` row
        // would flip the first assertion, which is the mutation this guards and the one
        // `a299e722` runs 266 times.
        let moving = |var: &str| {
            still_start().raw(&format!(
                "2.000000,gnss_vel,1.0,0.0,0.0,,,,{var},{var},{var}"
            ))
        };

        let raw = replay(&moving("0.0001")).summary();
        assert_eq!(key(&raw, "rejected"), "1", "{raw}");
        assert_eq!(key(&raw, "rejected_gnss_vel"), "1", "{raw}");
        assert_eq!(key(&raw, "r_policy"), "raw", "{raw}");

        // `rejected=0` alone would also pass if the row never reached the gate at all —
        // discarded for a bad variance, or refused before fusion — so the acceptance is
        // pinned by the two keys that only a completed update can move.
        let floored = replay(&moving("0.25")).summary();
        assert_eq!(key(&floored, "rejected"), "0", "{floored}");
        assert_eq!(key(&floored, "discarded"), "0", "{floored}");
        assert_ne!(key(&floored, "nis_gnss_vel"), "none", "{floored}");

        // The first row again, under `--r-policy px4`: floored to 0.5 m/s in the harness,
        // the same innovation is accepted, and the line says which policy did it.
        let px4 = RPolicy::Px4(GnssNoiseParameters {
            position: 0.5,
            velocity: 0.5,
            no_aid: 10.0,
        });
        let (replay, _) =
            drive_under(&moving("0.0001"), Config::default(), px4, None).expect("fixture replays");
        let px4 = replay.summary();
        assert_eq!(key(&px4, "rejected"), "0", "{px4}");
        assert_ne!(key(&px4, "nis_gnss_vel"), "none", "{px4}");
        assert_eq!(key(&px4, "r_policy"), "px4", "{px4}");
    }

    #[test]
    fn px4_reaches_gnss_position_rows_too() {
        // `S = H P Hᵀ + R`, so on the same fix from the same state the published `S_n` differs
        // by exactly the `R` each policy fused: 0.5² − 0.01² under PX4's `p` of 0.5 m. The
        // velocity fixture above cannot see a position row fused raw under `px4`.
        let log = still_start().raw("2.000000,gnss_pos,0.1,0.1,0,,,,0.0001,0.0001,0.0001");
        let s_n = |policy| {
            let (_, rows) =
                drive_under(&log, Config::default(), policy, None).expect("fixture replays");
            let row = rows
                .lines()
                .find(|row| row.contains(",gnss_pos,"))
                .expect("a position row")
                .to_string();
            row.split(',')
                .nth(5)
                .expect("S_n")
                .parse::<f64>()
                .expect("a number")
        };
        let px4 = RPolicy::Px4(GnssNoiseParameters {
            position: 0.5,
            velocity: 0.3,
            no_aid: 10.0,
        });
        let widened = s_n(px4) - s_n(RPolicy::Raw);
        assert!(
            (widened - (0.25 - 0.0001)).abs() < 1e-4,
            "S_n widened by {widened}"
        );
    }

    #[test]
    fn a_rejection_is_attributed_to_the_source_that_earned_it() {
        // The same kilometre-off fix as above, so the total is 1 and exactly one source
        // may claim it. Asserting the other three are zero is the half that matters: a
        // fragment built off the wrong list would still total correctly while naming the
        // source beside the right one. Survives zipping `SOURCES` against a `sources()`
        // in a different order, which `RATIOS` would not have caught either.
        let log = still_start()
            .gnss_pos(2.0, 1.0, 2.0, -3.0)
            .gnss_pos(2.04, 1000.0, 0.0, 0.0);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "rejected_gnss_pos"), "1", "{summary}");
        for source in ["gnss_vel", "baro", "mag"] {
            assert_eq!(
                key(&summary, &format!("rejected_{source}")),
                "0",
                "{source} rejected nothing: {summary}"
            );
        }
    }

    #[test]
    fn every_source_carries_its_own_rejection_key() {
        // The keys are generated from `SOURCES`, so what a reader cannot see by reading
        // the format string is that all four are actually on the line. A source added to
        // `Diagnostics` without a `SOURCES` entry loses its key silently, and `zip` is
        // what makes that silent rather than a compile error.
        let summary = replay(&still_start()).summary();
        for source in SOURCES {
            assert!(
                summary.contains(&format!(" rejected_{source}=")),
                "no rejected_{source}= on the line: {summary}"
            );
        }
        assert_eq!(SOURCES.len(), Diagnostics::default().sources().len());
    }

    #[test]
    fn a_series_computes_its_three_statistics_from_its_definitions() {
        // `ν` of 1, −1, 2 against a variance of 4, so `x` is 0.5, −0.5, 1.0 and every
        // number below can be done on paper: mean `ν` is 2/3, and the lag-1 autocorrelation
        // is Σ(xₜ−x̄)(xₜ₊₁−x̄) / Σ(xₜ−x̄)² = (−25/36)/(42/36) = −25/42.
        let mut series = Series::default();
        for nu in [1.0, -1.0, 2.0] {
            series.push(nu, 4.0);
        }
        assert_eq!(series.len(), 3);
        assert!((series.nu_mean().expect("three samples") - 2.0 / 3.0).abs() < 1e-9);
        let acf1 = series.autocorrelation().expect("three samples");
        assert!((acf1 - (-25.0 / 42.0)).abs() < 1e-6, "acf1 was {acf1}");

        // Below two samples there is no pair to correlate, and a series that does not vary
        // has a zero denominator rather than a zero correlation. Both are `none` on the
        // line, not a number a range could clear.
        let mut one = Series::default();
        one.push(1.0, 4.0);
        assert_eq!(one.autocorrelation(), None);
        let mut flat = Series::default();
        flat.push(1.0, 4.0);
        flat.push(1.0, 4.0);
        assert_eq!(flat.autocorrelation(), None);
    }

    #[test]
    fn every_source_carries_all_four_consistency_keys() {
        // Generated from `SOURCES` and `AXES` the way `rejected_` is, so the same silence a
        // missing `SOURCES` entry buys applies here. `AXES` is the only statement of what
        // component 0 of a fix is called, so its lengths are checked against the dimensions
        // the filter actually publishes rather than trusted.
        // The barometer needs a reading inside the window as well as after it: with no `α₀`
        // the first reading after it is spent reading one from the estimate, the gate never
        // runs, and the source publishes no dimension to check `AXES` against.
        let log = still_start_with_baro(&[41.5, 42.5, 42.0])
            .gnss_pos(2.0, 1.0, 2.0, -3.0)
            .gnss_vel(2.0, 0.1, 0.0, 0.0)
            .baro(2.0, 42.5)
            .mag(2.0)
            .mag(2.1)
            .gnss_yaw(2.0, 0.0)
            .gnss_yaw(2.1, 0.0);
        let replay = replay(&log);
        let summary = replay.summary();
        for (source, spelling) in SOURCES.iter().enumerate() {
            for family in ["nis", "nis_over95", "acf1"] {
                assert!(
                    summary.contains(&format!(" {family}_{spelling}=")),
                    "no {family}_{spelling}= on the line: {summary}"
                );
            }
            for axis in AXES[source] {
                assert!(
                    summary.contains(&format!(" nu_{spelling}_{axis}=")),
                    "no nu_{spelling}_{axis}= on the line: {summary}"
                );
            }
            // A course needs speed, which a still start has not got; the test below moves.
            if source == COURSE {
                continue;
            }
            assert_eq!(
                AXES[source].len(),
                replay.consistency.dimension[source],
                "{spelling} publishes a different dimension than AXES names"
            );
        }
        assert_eq!(AXES.len(), SOURCES.len());
    }

    #[test]
    fn nis_is_recovered_with_the_gate_the_filter_was_configured_with() {
        // `ε = r γ`, so a harness that wrote its own `γ` instead of reading `Config::gates`
        // would be wrong by the ratio of two percentiles and no other fixture here would
        // notice: they all replay at the default. Nothing in this log is near either gate,
        // so both runs accept the same measurements and follow the same trajectory, and the
        // only thing that changed is the number `r` was divided by on the way out.
        let log = still_start()
            .gnss_pos(2.0, 1.0, 2.0, -3.0)
            .gnss_pos(2.2, 1.1, 2.1, -3.1)
            .gnss_pos(2.4, 0.9, 1.9, -2.9);
        let at = |percentile| {
            let config = Config {
                gates: Gates::at(percentile),
                ..Config::default()
            };
            let (replay, _) = drive_with(&log, config, None).expect("fixture replays");
            let summary = replay.summary();
            assert_eq!(key(&summary, "rejected"), "0", "{summary}");
            key(&summary, "nis_gnss_pos").to_string()
        };
        assert_eq!(at(Percentile::P999), at(Percentile::P95));
    }

    #[test]
    fn an_adopted_measurement_contributes_no_epsilon() {
        // A window with no magnetometer leaves yaw unobserved, so the first heading is
        // adopted rather than fused: `Fusion::Reset`, ratio 0, no innovation.
        //
        // This pins a property rather than guarding a branch, which is worth the sentence
        // because the other fixtures here do the opposite. #5 asked whether a NIS mean
        // should exclude adoptions or average them in as zeros; the answer is that it
        // cannot average them in, since an adoption publishes no innovation and therefore
        // no dimension to record a row against. No mutation of the predicate in `observe`
        // puts an adopted row into a population. What this is for is the reader who asks
        // #5's question and wants it answered by something that runs.
        let one_fused = replay(&still_start().mag(2.0).mag(2.02));
        let summary = one_fused.summary();
        assert_eq!(key(&summary, "resets"), "1", "{summary}");
        assert_eq!(one_fused.consistency.rows(MAG), 1, "{summary}");

        // The same start with one more heading. Two fusions and still one adoption, so the
        // population grows by exactly the measurement that was fused.
        let two_fused = replay(&still_start().mag(2.0).mag(2.02).mag(2.04));
        assert_eq!(key(&two_fused.summary(), "resets"), "1");
        assert_eq!(two_fused.consistency.rows(MAG), 2);
    }

    #[test]
    fn a_rejected_measurement_does_contribute_its_epsilon() {
        // The gate's verdict is not the statistic's: `ε` is computed before the gate has
        // one, and a rejection is the largest `ε` a source produces. Dropping those would
        // censor the tail `nis_over95_` exists to count and flatter every NIS mean on a
        // source that is being turned down — which on the corpus is the one source whose
        // reported accuracy is in question.
        let clean = still_start().gnss_pos(2.0, 1.0, 2.0, -3.0);
        let with_outlier = still_start()
            .gnss_pos(2.0, 1.0, 2.0, -3.0)
            .gnss_pos(2.04, 1000.0, 0.0, 0.0);

        let clean = replay(&clean);
        let with_outlier = replay(&with_outlier);
        assert_eq!(key(&with_outlier.summary(), "rejected"), "1");
        assert_eq!(clean.consistency.rows(GNSS_POS), 1);
        assert_eq!(with_outlier.consistency.rows(GNSS_POS), 2);

        // A kilometre off a metre-scale `S`, so it lands far above the 95 % bound and drags
        // the mean with it. Both keys move; neither would if the row were dropped.
        let nis = |replay: &Replay| {
            key(&replay.summary(), "nis_gnss_pos")
                .parse::<f64>()
                .expect("a number")
        };
        assert!(
            nis(&with_outlier) > nis(&clean),
            "{} against {}",
            nis(&with_outlier),
            nis(&clean)
        );
        assert_eq!(
            key(&with_outlier.summary(), "nis_over95_gnss_pos"),
            "0.5000"
        );
        assert_eq!(key(&clean.summary(), "nis_over95_gnss_pos"), "0.0000");
    }

    #[test]
    fn a_source_that_gated_nothing_reads_none_rather_than_zero() {
        // A still start with no aiding at all. Zero would be a reading — a filter whose
        // innovations were perfectly consistent reads a NIS near 1, and one whose `R` is
        // enormous reads near 0 — so a source that produced no measurement has to say so in
        // a word. It also keeps an empty source from clearing a manifest range, since
        // `data/expect.sh` refuses a non-number where a bound is asked for.
        let summary = replay(&still_start()).summary();
        for source in SOURCES {
            assert_eq!(key(&summary, &format!("nis_{source}")), "none", "{summary}");
            assert_eq!(
                key(&summary, &format!("nis_over95_{source}")),
                "none",
                "{summary}"
            );
            assert_eq!(
                key(&summary, &format!("acf1_{source}")),
                "none",
                "{summary}"
            );
        }
        assert_eq!(key(&summary, "nu_gnss_pos_n"), "none", "{summary}");
        assert_eq!(key(&summary, "nu_mag_yaw"), "none", "{summary}");
    }

    #[test]
    fn epochs_count_every_imu_row_after_initialization() {
        let log = still_start().run(2.0, 10, DT, STILL);
        assert_eq!(key(&replay(&log).summary(), "epochs"), "10");
    }

    #[test]
    fn the_diagonal_floor_of_42_reports_what_it_raised() {
        // Zero is the value the manifest pins on every log, so a key wired to the wrong
        // field reads correctly on the whole corpus. What makes this a fixture rather than
        // a restatement is the second half: a receiver claiming 1e-12 m² — a micron of
        // eph — is accepted, gated and fused like any other, and it drives the position
        // variances six decades below the floor within a few fixes. Three states raised,
        // once each, which is what tells a per-entry count from a per-covariance one.
        let quiet = still_start().run(2.0, 10, DT, STILL);
        assert_eq!(key(&replay(&quiet).summary(), "floored"), "0");

        let certain = |fixes: usize| {
            let mut log = still_start();
            for i in 0..fixes {
                let t = 2.0 + i as f64 * DT;
                // Taken at the IMU sample before it, so the fix has no age and informs
                // position alone.
                log = log
                    .imu(t, STILL)
                    .raw(&format!("{t:.6},gnss_pos,0,0,0,,,,1e-12,1e-12,1e-12"));
            }
            replay(&log).summary()
        };

        let one = certain(1);
        assert_eq!(key(&one, "discarded"), "0", "{one}");
        assert_eq!(key(&one, "floored"), "3", "{one}");

        // Counted per raise, not per state that ever collapsed: the three position
        // variances are floored, (16) grows them back from velocity over the step, and the
        // next fix collapses the same three again.
        assert_eq!(key(&certain(2), "floored"), "6");
    }

    #[test]
    fn a_repeated_timestamp_after_initialization_is_an_invalid_step() {
        // With rows sorted by time, a zero `dt` is a duplicate in practice. The epoch is
        // still written: the row existed, and the state is simply the one before it.
        let log = still_start().imu(99.0 * DT, STILL);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "invalid"), "1");
        assert_eq!(key(&summary, "refused"), "0", "nothing here was too long");
        assert_eq!(key(&summary, "coasted"), "0", "nothing here was too long");
        assert_eq!(key(&summary, "epochs"), "1");
    }

    #[test]
    fn a_gap_longer_than_the_limit_is_coasted_and_timestamped() {
        // 0.5 s against a `max_predict_dt` of 0.1 s: a logging dropout, which the filter
        // coasts across while the timers run on.
        let gap_at = 99.0 * DT + 0.5;
        let log = still_start().imu(gap_at, STILL);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "coasted"), "1");
        assert_eq!(
            key(&replay.summary(), "refused"),
            "0",
            "a coast is not a refusal"
        );
        assert_eq!(
            replay.longest_step_at,
            Some(gap_at),
            "the filter holds the size of the gap; the harness holds when it happened"
        );
        let worst = replay
            .filter
            .diagnostics()
            .propagation
            .longest_gap
            .expect("a coasted step");
        assert!(
            (worst.as_secs() - 0.5).abs() < 1e-3,
            "{} s",
            worst.as_secs()
        );
    }

    #[test]
    fn a_coarse_start_adopts_one_position_and_one_velocity() {
        // Nothing was ever established, so the first fix is adopted rather than fused —
        // once per quantity. None of it is a recovery.
        let mut log = Log::new().run(0.0, 501, DT, TURNING);
        log = log
            .gnss_pos(10.02, 1.0, 2.0, -3.0)
            .gnss_vel(10.02, 0.5, 0.0, 0.0)
            .imu(10.02, TURNING)
            .gnss_pos(10.04, 1.1, 2.1, -3.1)
            .gnss_vel(10.04, 0.6, 0.0, 0.0)
            .imu(10.04, TURNING);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "align"), "coarse");
        // Three: the fix is adopted whole, and counted by both of the sources it feeds.
        assert_eq!(
            key(&summary, "resets"),
            "3",
            "the second pair is fused, not adopted"
        );
        assert_eq!(key(&summary, "recovered"), "0");
    }

    #[test]
    fn a_locked_out_receiver_is_recovered_once_and_counted_apart() {
        // A still start, then a receiver 100 m from where the window put the vehicle, once a
        // second: every fix is rejected until none has been accepted for
        // `Recovery::gnss_position`, then one is adopted and the rest fuse. Its height agrees,
        // so only the horizontal half recovers. The IMU runs on without a break: a gap would be
        // coasted, and a covariance grown across it accepts the receiver instead.
        let mut log = still_start().run(2.0, 50, DT, STILL);
        for second in 3..16 {
            let t = f64::from(second);
            log = log.gnss_pos(t, 100.0, 0.0, 0.0).run(t, 50, DT, STILL);
        }
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "recovered"), "1");
        assert_eq!(key(&summary, "resets"), "1", "a recovery is an adoption");
        assert_ne!(
            key(&summary, "rejected_gnss_pos"),
            "0",
            "it was locked out first"
        );
    }

    #[test]
    fn a_still_start_adopts_nothing() {
        let log = still_start()
            .gnss_pos(2.0, 1.0, 2.0, -3.0)
            .gnss_vel(2.0, 0.5, 0.0, 0.0)
            .imu(2.0, STILL);
        assert_eq!(key(&replay(&log).summary(), "resets"), "0");
    }

    #[test]
    fn a_non_finite_imu_row_after_initialization_is_refused_as_the_sensor_not_the_timing() {
        // `nan` parses, so the converter can hand one through. Counted apart from a bad
        // `dt`, because otherwise a failed IMU looks like the filter quietly not advancing.
        let log = still_start().raw("2.000000,imu,0,0,0,0,0,nan,,,");
        let propagation = replay(&log).filter.diagnostics().propagation;
        assert_eq!(propagation.refused_not_finite, 1);
        assert_eq!(propagation.refused_invalid, 0, "the timing was fine");
    }

    #[test]
    fn a_non_finite_sample_inside_the_window_stops_the_replay() {
        // Initialization refuses genuinely unusable input rather than aligning to it, and
        // the harness has no state it could keep replaying from. Worth pinning as the
        // asymmetry it is: the same row after initialization is merely counted above.
        let mut log = Log::new().run(0.0, 99, DT, STILL);
        log = log.raw("1.980000,imu,0,0,0,0,0,nan,,,");
        let error = replay_error(&log);
        assert!(
            error.contains("not finite"),
            "the error names the fault: {error}"
        );
    }

    // ---- the fusion file ----

    #[test]
    fn every_verdict_writes_one_row_whatever_it_was() {
        // Three aiding rows before the window closes and three after, and a GNSS fix is two
        // verdicts, so eight. The summary counts only what was accepted or refused; this is
        // the record that they happened at all.
        let mut log = Log::new();
        for i in 0..100 {
            let t = i as f64 * DT;
            if i == 10 {
                log = log
                    .gnss_pos(t, 0.0, 0.0, 0.0)
                    .gnss_vel(t, 0.0, 0.0, 0.0)
                    .mag(t);
            }
            log = log.imu(t, STILL);
        }
        log = log
            .gnss_pos(2.0, 1.0, 2.0, -3.0)
            .gnss_vel(2.0, 0.5, 0.0, 0.0)
            .mag(2.0)
            .imu(2.0, STILL);
        assert_eq!(fusion_rows(&log).len(), 8);
    }

    #[test]
    fn a_fusion_row_names_the_verdict_it_got() {
        // One fixture per outcome a log can produce. `state_invalid` is not among them: it
        // needs a covariance that has stopped being one, which no row of input can write, so
        // `verdict` being exhaustive on `Fusion` is what keeps its name from going missing.
        let coarse = Log::new()
            .run(0.0, 501, DT, TURNING)
            .gnss_pos(10.02, 1.0, 2.0, -3.0)
            .imu(10.02, TURNING);
        for (log, expected) in [
            (still_start().gnss_pos(2.0, 0.0, 0.0, 0.0), "accepted"),
            (still_start().gnss_pos(2.0, 1000.0, 0.0, 1000.0), "rejected"),
            (Log::new().gnss_pos(0.0, 0.0, 0.0, 0.0), "not_initialized"),
            (
                coarse_start()
                    .raw("10.020000,baro,42,,,,,,4,,")
                    .imu(10.02, TURNING),
                "no_reference",
            ),
            (
                still_start().raw("2.000000,gnss_pos,0,0,0,,,,0,2.25,0"),
                "invalid_noise",
            ),
            (
                still_start().raw("2.000000,gnss_pos,nan,0,nan,,,,2.25,2.25,5.625"),
                "not_finite",
            ),
            (coarse, "reset"),
            // `t_meas_s` a second before arrival: read, and handed to the filter as the time.
            (
                still_start().raw("2.000000,gnss_pos,0,0,0,,,,2.25,2.25,5.625,1.000000"),
                "out_of_horizon",
            ),
        ] {
            let last = fusion_rows(&log).pop().expect("a fusion row");
            // Second from the end: `truth` follows the outcome, empty with no truth file.
            assert_eq!(
                last.rsplit(',').nth(1),
                Some(expected),
                "expected {expected}: {last}"
            );
        }
    }

    #[test]
    fn a_gated_fix_carries_the_innovation_the_filter_published() {
        // A still start sits at the origin, so `ν` is the fix itself. `S` is `H P Hᵀ + R`,
        // so each entry is at least the row's own variance.
        // Two rows, one per half, each padded past its own dimension.
        let log = still_start().gnss_pos(2.0, 1.0, 2.0, -3.0);
        let rows = fusion_rows(&log);
        let halves = &rows[rows.len() - 2..];
        for (row, (source, nu, r)) in halves.iter().zip([
            ("gnss_pos", ["1.000000", "2.000000", ""], &[2.25, 2.25][..]),
            ("gnss_hgt", ["-3.000000", "", ""], &[5.625][..]),
        ]) {
            let fields: Vec<&str> = row.split(',').collect();
            assert_eq!(fields[1], source, "{row}");
            assert_eq!(&fields[2..5], &nu, "ν: {row}");
            for (s, r) in fields[5..8].iter().zip(r) {
                let s: f32 = s.parse().unwrap_or_else(|_| panic!("S missing: {row}"));
                assert!(s > *r, "S below R: {row}");
            }
            assert_eq!(
                fields[5 + r.len()..8]
                    .iter()
                    .filter(|f| f.is_empty())
                    .count(),
                3 - r.len()
            );
            assert_eq!(fields[9], "accepted", "{row}");
        }
    }

    #[test]
    fn a_call_the_gate_did_not_judge_leaves_the_innovation_columns_empty() {
        // A refusal and an adoption both follow a gated fix, whose values must not be
        // written again against a measurement they do not describe. No source is a stub
        // any more, so those two are the whole of what reaches a fusion row without a
        // verdict from the gate. The magnetometer is the adoption here: this window
        // carries none, so yaw is unestablished and the first field is taken rather than
        // tested. The verdicts are pinned because a barometer with no reference, refused
        // `NoReference` before it reaches an update, passes the column check while
        // testing only refusals.
        let log = still_start()
            .gnss_pos(2.0, 1.0, 2.0, -3.0)
            .raw("2.020000,gnss_pos,1,2,-3,,,,0,2.25,0")
            .mag(2.04);
        let rows = fusion_rows(&log);
        let tail = &rows[rows.len() - 3..];
        for (row, verdict) in tail.iter().zip(["invalid_noise", "invalid_noise", "reset"]) {
            let fields: Vec<&str> = row.split(',').collect();
            assert_eq!(&fields[2..8], &["", "", "", "", "", ""], "ν and S: {row}");
            assert_eq!(fields[9], verdict, "{row}");
        }
    }

    #[test]
    fn the_fusion_header_carries_the_gates_the_ratios_were_produced_under() {
        // `r = ε / γ`, so a ratio without its `γ` is not recoverable to NIS.
        let mut out = Vec::new();
        let gates = Gates {
            baro_altitude: Gate::new(2.71).expect("a positive threshold"),
            ..Gates::default()
        };
        write_fusion_header(&mut out, gates).expect("header");
        let text = String::from_utf8(out).expect("utf-8");
        assert!(text.contains("baro=2.71"), "the gate in force: {text}");
        assert_eq!(
            text.lines().last(),
            Some("t_s,source,nu0,nu1,nu2,s0,s1,s2,ratio,outcome,truth")
        );
    }

    #[test]
    fn the_fusion_header_names_one_column_per_field_in_a_row() {
        let mut out = Vec::new();
        write_fusion_header(&mut out, Gates::default()).expect("header");
        let text = String::from_utf8(out).expect("utf-8");
        let header = text.lines().last().expect("a header");
        let row = fusion_rows(&still_start().gnss_pos(2.0, 0.0, 0.0, 0.0))
            .pop()
            .expect("a fusion row");
        assert_eq!(header.split(',').count(), row.split(',').count(), "{row}");
    }

    // ---- scoring against truth ----

    /// Builds a truth file one row at a time.
    ///
    /// Rows only, like [`Log`]: it computes no error and no score, so every expected figure
    /// below is a literal beside the assertion that reads it.
    struct TruthLog(String);

    impl TruthLog {
        fn new() -> Self {
            Self(format!("# a truth fixture\n{}\n", TRUTH_COLUMNS.join(",")))
        }

        /// One row: position, velocity, roll/pitch/yaw, then the two biases.
        fn row(mut self, t: f64, values: [f32; STATES]) -> Self {
            self.0 += &format!("{t:.4}");
            for value in values {
                self.0 += &format!(",{value:.6}");
            }
            self.0 += "\n";
            self
        }

        /// `rows` rows of a vehicle sitting at the origin, level and pointing north — what
        /// the harness's own still fixtures are the truth for.
        fn still(mut self, start: f64, rows: usize, interval: f64) -> Self {
            for i in 0..rows {
                self = self.row(start + i as f64 * interval, [0.0; STATES]);
            }
            self
        }

        fn parse(&self) -> Result<Truth, String> {
            Truth::parse(&self.0)
        }

        fn scoring(self) -> Scoring {
            Scoring::new(
                PathBuf::from("fixture.truth.csv"),
                self.parse().expect("fixture parses"),
            )
        }
    }

    /// The error a truth fixture is refused with. A function rather than `expect_err`,
    /// which would want `Debug` on `Truth` — the derive this example declines for `Replay`
    /// for the same reason.
    fn truth_error(text: &str) -> String {
        match Truth::parse(text) {
            Ok(_) => panic!("the fixture was expected to be refused"),
            Err(e) => e,
        }
    }

    /// Truth at an attitude, everything else at the origin.
    ///
    /// The three angle columns of `TRUTH_COLUMNS`, which follow position and velocity — Euler
    /// angles, not `δθ`, which is what `error_state` turns them into.
    fn truth_at(roll: f32, pitch: f32, yaw: f32) -> TruthRow {
        let mut values = [0.0f32; STATES];
        values[6..9].copy_from_slice(&[roll, pitch, yaw]);
        truth_row(0.0, values)
    }

    /// Truth at a position, level and at rest — the first three columns after `t_s`.
    fn truth_offset(north: f32, east: f32, down: f32) -> TruthRow {
        let mut values = [0.0f32; STATES];
        values[0..3].copy_from_slice(&[north, east, down]);
        truth_row(0.0, values)
    }

    /// One truth row, written by [`TruthLog::row`] and read back through the real parser, so
    /// a fixture never hands the scorer a row the file format could not carry.
    fn truth_row(t: f64, values: [f32; STATES]) -> TruthRow {
        let log = TruthLog::new().row(t, values);
        let row = log.0.lines().last().expect("a row");
        TruthRow::parse(row).expect("a truth row parses")
    }

    /// Score one epoch on its own, away from the harness.
    fn score_one(state: &State, covariance: &Covariance, truth: &TruthRow) -> Score {
        let mut score = Score::default();
        score.epoch(state, covariance, truth, &Accuracy::default());
        score
    }

    /// A state at a given attitude, everything else at the origin, every quantity claimed
    /// valid — so a `false_valid` count is about the error and not about what was claimed.
    fn state_at(roll: f32, pitch: f32, yaw: f32) -> State {
        State {
            attitude: Attitude::from_body_to_ned(components(UnitQuaternion::from_euler_angles(
                roll, pitch, yaw,
            ))),
            validity: Validity {
                tilt: true,
                heading: true,
                horizontal_position: true,
                vertical_position: true,
                horizontal_velocity: true,
                vertical_velocity: true,
            },
            ..State::default()
        }
    }

    #[test]
    fn an_estimate_equal_to_truth_scores_zero() {
        // The check the whole score line rests on: no error anywhere, and every key that
        // measures one says so. `in3s` goes the other way — a zero error is inside any
        // sigma — and `false_valid` finds nothing to contradict.
        let state = state_at(0.0, 0.0, 0.0);
        let covariance = Covariance::from_sigmas([0.5; STATES]);
        let score = score_one(&state, &covariance, &truth_row(0.0, [0.0; STATES]));
        assert_eq!(score.rms(score.position_horizontal), 0.0);
        assert_eq!(score.rms(score.position_vertical), 0.0);
        assert_eq!(score.rms(score.velocity), 0.0);
        assert_eq!(score.position_horizontal_max, 0.0);
        assert_eq!(score.rms(score.tilt), 0.0);
        assert_eq!(score.rms(score.yaw), 0.0);
        assert_eq!(
            Score::bias_text(score.accel_bias, score.accel_bias_scored, 5),
            "0.00000"
        );
        assert_eq!(
            Score::bias_text(score.gyro_bias, score.gyro_bias_scored, 6),
            "0.000000"
        );
        assert_eq!(score.false_valid(), 0);
        assert_eq!(score.within_3s, score.axes, "all 15 axes, not just the 9");
        for block in 0..3 {
            assert_eq!(score.nees_per_dof(block), Some(0.0));
        }
    }

    #[test]
    fn every_state_reaches_the_error_vector() {
        // One unit of error on each of the fifteen, so a block wired to the wrong offset —
        // or left out of `error_state` — shows up as a zero where a one belongs.
        let state = state_at(0.0, 0.0, 0.0);
        let mut values = [0.0f32; 15];
        for (i, value) in values.iter_mut().enumerate() {
            // Attitude is the exception: the truth columns are Euler angles, and a unit
            // radian is not a unit of `δθ`. Those three are checked below instead.
            *value = if (6..9).contains(&i) { 0.0 } else { 1.0 };
        }
        let error = error_state(&state, &truth_row(0.0, values));
        for i in (0..STATES).filter(|i| !(6..9).contains(i)) {
            assert_eq!(error[i], 1.0, "state {i} is not in the error vector");
        }
    }

    #[test]
    fn the_bias_keys_read_one_block_each() {
        // Two keys, two blocks. An error in the accelerometer bias alone must leave `bg` at
        // zero: wired to one offset, or to each other's, both move together and the pair
        // stops saying which bias walked — which is the whole reason for publishing two.
        // 3 and 4 on two axes rather than ones, so a key summing the block instead of
        // taking its norm reads 7 here and 2 under a fixture of ones.
        let mut values = [0.0f32; STATES];
        values[ErrorState::AccelBiasX.index()] = 3.0;
        values[ErrorState::AccelBiasY.index()] = 4.0;
        let score = score_one(
            &state_at(0.0, 0.0, 0.0),
            &Covariance::from_sigmas([0.5; STATES]),
            &truth_row(0.0, values),
        );
        assert_eq!(score.rms(score.accel_bias), 5.0, "the block's norm");
        assert_eq!(score.rms(score.gyro_bias), 0.0, "a block of its own");
        // And through the serialized line, where the two are positional arguments among
        // fifteen: a key inserted in the wrong place reads its neighbour's value, and the
        // accumulators above cannot see that.
        let line = score.line();
        assert_eq!(key(&line, "ba"), "5.00000");
        assert_eq!(key(&line, "bg"), "0.000000");
    }

    #[test]
    fn the_attitude_error_is_a_body_frame_rotation_vector() {
        // Level, so body z is the vertical and `δθ` reads as the Euler angles do. 30 deg of
        // heading is 0.5236 rad of yaw error and no tilt error.
        let error = error_state(&state_at(0.0, 0.0, 0.0), &truth_at(0.0, 0.0, 0.5236));
        let tilt = horizontal(&error, ErrorState::AttitudeX, ErrorState::AttitudeY);
        assert!(tilt.abs() < 1e-5, "no tilt error: {tilt}");
        let yaw = at(&error, ErrorState::AttitudeZ);
        assert!((yaw - 0.5236).abs() < 1e-5, "yaw: {yaw}");
    }

    #[test]
    fn a_banked_vehicles_heading_error_is_not_its_yaw_difference() {
        // Why `error_state` takes `δθ` from the quaternions rather than differencing Euler
        // angles, and why one function owns it. Both attitudes are rolled 0.4 rad and the
        // truth is 0.1 rad further round in yaw. `δq = Rx(-φ) Rz(ψ) Rx(φ)` is 0.1 rad about
        // `(0, sin φ, cos φ)`, so the error the covariance describes is 0.0921 about body z
        // and 0.0389 about body y — not 0.1 of yaw and nothing of tilt, which is what
        // differencing the angles would have said.
        let error = error_state(&state_at(0.4, 0.0, 1.0), &truth_at(0.4, 0.0, 1.1));
        let (y, z) = (
            at(&error, ErrorState::AttitudeY),
            at(&error, ErrorState::AttitudeZ),
        );
        assert!((y - 0.0389).abs() < 1e-3, "body y: {y}");
        assert!((z - 0.0921).abs() < 1e-3, "body z: {z}");
    }

    #[test]
    fn on_its_tail_a_heading_error_is_scored_as_heading() {
        // Pitched 90° nose-up and the truth 0.6 rad further round about down. Body x is the
        // vertical there, so `δθ` is 0.6 about body x: read by body axis that is a tilt
        // error and no heading error. On navigation axes it is heading alone — outside
        // `Accuracy::heading`'s 0.52 rad, and nowhere near the tilt bar's. Reverting the
        // score to `error_state`'s body axes swaps both counts.
        let pitch = core::f32::consts::FRAC_PI_2;
        let state = state_at(0.0, pitch, 0.0);
        let turned =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.6) * rotation(state.attitude);
        let mut truth = truth_at(0.0, 0.0, 0.0);
        truth.attitude = turned;
        let score = score_one(&state, &Covariance::from_sigmas([0.5; STATES]), &truth);
        assert!(
            score.rms(score.tilt) < 1e-5,
            "tilt: {}",
            score.rms(score.tilt)
        );
        assert!(
            (score.rms(score.yaw) - 0.6).abs() < 1e-5,
            "yaw: {}",
            score.rms(score.yaw)
        );
        assert_eq!(score.false_valid[0], 0, "tilt was not wrong");
        assert_eq!(score.false_valid[1], 1, "heading was");
    }

    #[test]
    fn nees_reads_the_correlations_that_in3s_cannot() {
        // The two consistency keys are not the same test twice. One metre of error on north
        // and east each, against a covariance whose two position axes are 1 m and almost
        // perfectly correlated: every axis is inside 1 sigma, so `in3s` is content, while
        // the joint test sees an error the correlation says should not happen.
        let mut p = CovarianceMatrix::identity();
        p[(0, 1)] = 0.99;
        p[(1, 0)] = 0.99;
        let covariance = Covariance::from_rows(p.transpose().into());
        let truth = truth_offset(1.0, -1.0, 0.0);
        let score = score_one(&state_at(0.0, 0.0, 0.0), &covariance, &truth);
        assert_eq!(
            score.within_3s, score.axes,
            "every marginal is inside 3 sigma"
        );
        let joint = score.nees_per_dof(0).expect("position inverts");
        assert!(joint > 30.0, "the joint test is not: {joint}");
    }

    #[test]
    fn a_singular_block_is_counted_out_rather_than_unwrapped() {
        // `try_inverse` returns an `Option` and a collapsed covariance is a finding, not a
        // reason to stop replaying. The other two blocks are still scored.
        let mut p = CovarianceMatrix::identity();
        for i in 0..3 {
            p[(i, i)] = 0.0;
        }
        let covariance = Covariance::from_rows(p.transpose().into());
        let score = score_one(
            &state_at(0.0, 0.0, 0.0),
            &covariance,
            &truth_row(0.0, [0.0; STATES]),
        );
        assert_eq!(
            score.nees_epochs,
            [0, 1, 1],
            "position could not be inverted"
        );
        assert_eq!(score.nees_per_dof(0), None);
        assert_eq!(
            score.nees_text(0),
            "none",
            "a word, so it is not read as an extremely conservative covariance"
        );
        assert_eq!(
            score.nees_text(1),
            "0.0000",
            "the other blocks still report"
        );
    }

    #[test]
    fn a_claim_is_falsified_per_axis_rather_than_on_the_norm() {
        // `Eskf::validity` asks `within(PositionNorth) && within(PositionEast)`, so 4 m on
        // each axis against a 5 m bar is inside the claim the filter actually made — even
        // though the 2-D norm is 5.66 m. Testing the norm would hold it to a bar √2 tighter
        // than the one it asserted, and would diverge from the claim exactly as the estimate
        // approached it.
        let covariance = Covariance::from_sigmas([0.5; STATES]);
        let state = state_at(0.0, 0.0, 0.0);
        let inside = score_one(&state, &covariance, &truth_offset(4.0, 4.0, 0.0));
        assert_eq!(
            inside.false_valid(),
            0,
            "5.66 m of norm, 4 m on either axis"
        );

        let outside = score_one(&state, &covariance, &truth_offset(6.0, 0.0, 0.0));
        assert_eq!(
            outside.false_valid(),
            1,
            "one axis past the bar falsifies it"
        );
    }

    #[test]
    fn nothing_scored_publishes_no_figures() {
        // The same argument as a log with no truth file at all: a full line would put
        // `pos_h=0.000`, the best possible value, beside `in3s=0.0000`, the worst, for a log
        // nothing measured. A consumer looking for `pos_h` finds no key rather than a good
        // one.
        assert_eq!(Score::default().line(), "score scored=0");
    }

    // ---- a GNSS fix judged against truth (#60) ----

    /// Truth at the origin heading `yaw`, level and still, with no bias known: what
    /// `tools/urbannav2replay.py` writes, blank bias columns and all.
    fn heading_truth(t: f64, yaw: f32) -> Scoring {
        let row = format!("{t:.4},0,0,0,0,0,0,0,0,{yaw},,,,,,\n");
        TruthLog(TruthLog::new().0 + &row).scoring()
    }

    fn judge(
        scoring: &Scoring,
        fix: [f32; 3],
        variance: [f32; 3],
        antenna: [f32; 3],
    ) -> [Judged; 2] {
        scoring
            .judge(
                1.0,
                Position::ned(fix[0], fix[1], fix[2]),
                variance,
                Position::body(antenna[0], antenna[1], antenna[2]),
                &Accuracy::default(),
            )
            .expect("a truth row at the fix's time")
    }

    #[test]
    fn a_fix_is_judged_at_the_antenna_on_the_truth_attitude() {
        // Heading east, an antenna a metre forward sits a metre east of the IMU. A fix there
        // is exact; the same fix read on the IMU, or on a north-facing antenna, is a metre
        // out against a 0.1 m sigma, ε = 100 past γ = 13.8.
        let scoring = heading_truth(1.0, core::f32::consts::FRAC_PI_2);
        let tight = [0.01, 0.01, 0.01];
        let [horizontal, height] = judge(&scoring, [0.0, 1.0, 0.0], tight, [1.0, 0.0, 0.0]);
        assert!(!horizontal.bad && !height.bad);
        let [horizontal, _] = judge(&scoring, [1.0, 0.0, 0.0], tight, [1.0, 0.0, 0.0]);
        assert!(
            horizontal.bad,
            "the antenna rotated onto east, not left on north"
        );
    }

    #[test]
    fn a_fix_is_bad_against_its_own_variance_and_per_half() {
        // 4 m north against a 1 m sigma north and a 100 m sigma east: ε = 16, past the
        // 2-DOF γ = 13.8. The same 4 m on east reads ε = 0.0016. Asymmetric so a swapped
        // axis cannot pass, and height judged alone at the 1-DOF γ = 10.8: 3.2 m against 1 m
        // is ε = 10.24, under it, and 3.3 m is 10.89, over.
        let scoring = heading_truth(1.0, 0.0);
        let variance = [1.0, 10_000.0, 1.0];
        let [north, level] = judge(&scoring, [4.0, 0.0, 0.0], variance, [0.0; 3]);
        let [east, _] = judge(&scoring, [0.0, 4.0, 0.0], variance, [0.0; 3]);
        assert!(north.bad && !east.bad);
        assert!(
            !level.bad,
            "the horizontal error does not reach the height half"
        );
        let [horizontal, under] = judge(&scoring, [0.0, 0.0, 3.2], variance, [0.0; 3]);
        let [_, over] = judge(&scoring, [0.0, 0.0, 3.3], variance, [0.0; 3]);
        assert!(!horizontal.bad, "height does not reach the horizontal half");
        assert!(!under.bad && over.bad);
    }

    #[test]
    fn far_is_per_axis_against_the_mission_bar() {
        // `Accuracy::position` is 5 m per axis, the way `Validity` states it: 4.5 m north and
        // 4.5 m east is 6.4 m off and not far, and 5.5 m on one axis is.
        let scoring = heading_truth(1.0, 0.0);
        let loose = [1.0e6; 3];
        let [both, _] = judge(&scoring, [4.5, 4.5, 0.0], loose, [0.0; 3]);
        let [one, _] = judge(&scoring, [0.0, 5.5, 0.0], loose, [0.0; 3]);
        let [_, height] = judge(&scoring, [0.0, 0.0, -5.5], loose, [0.0; 3]);
        assert!(!both.far && one.far && height.far);
    }

    #[test]
    fn a_fix_past_the_truth_or_without_a_variance_is_not_judged() {
        let scoring = heading_truth(1.0, 0.0);
        let at = |t: f64, variance: [f32; 3]| {
            scoring.judge(
                t,
                Position::ned(0.0, 0.0, 0.0),
                variance,
                Position::zero(),
                &Accuracy::default(),
            )
        };
        assert!(at(1.0, [1.0; 3]).is_some());
        assert!(at(1.01, [1.0; 3]).is_none(), "no truth after 1.0 s");
        assert!(
            at(1.0, [1.0, 0.0, 1.0]).is_none(),
            "a zero variance divides nothing"
        );
    }

    #[test]
    fn verdicts_are_counted_against_what_truth_said_of_the_fix() {
        const GOOD: Option<Judged> = Some(Judged {
            bad: false,
            far: false,
            error: 1.0,
        });
        const BAD: Option<Judged> = Some(Judged {
            bad: true,
            far: true,
            error: 3.0,
        });
        let accepted = Fusion::Accepted { test_ratio: 0.1 };
        let rejected = Fusion::Rejected { test_ratio: 3.0 };
        let mut fixes = FixScore::default();
        // A first adoption, which is no recovery. A bad fix accepted, then a run of two good
        // fixes and one bad rejected, ended by recovering onto a good one: a lockout. A run of
        // one bad, ended by recovering onto a good one: a tie, counted a glitch. A recovery
        // after a gap, whose own fix is the whole run: bad, a glitch. A refusal counts for
        // nothing, a refused one with no truth row included, and a decided fix with no truth
        // is unjudged. A run an accepted fix ended
        // leaves the next recovery with only its own fix: good, a lockout.
        for (outcome, judged, recovery) in [
            (Fusion::Reset, GOOD, false),
            (accepted, BAD, false),
            (rejected, GOOD, false),
            (rejected, BAD, false),
            (rejected, GOOD, false),
            (Fusion::Reset, GOOD, true),
            (rejected, BAD, false),
            (Fusion::Reset, GOOD, true),
            (Fusion::Reset, BAD, true),
            (Fusion::NotInitialized, BAD, false),
            (
                Fusion::OutOfHorizon {
                    age: Seconds::from_secs(1.0),
                },
                None,
                false,
            ),
            (accepted, None, false),
            (rejected, BAD, false),
            (accepted, GOOD, false),
            (Fusion::Reset, GOOD, true),
        ] {
            fixes.record(outcome, judged, recovery);
        }
        assert_eq!(
            fixes,
            FixScore {
                offered: 12,
                bad: 5,
                rejected_bad: 3,
                rejected_good: 2,
                accepted_far: 1,
                adopted_bad: 1,
                recovered_after_bad: 2,
                recovered_after_lockout: 2,
                unjudged: 1,
                // Seven good fixes at 1 m and five bad at 3 m, the unjudged and the refused
                // left out: √(52 / 12) = 2.082 m.
                squared_error: 52.0,
                run: (0, 0),
            }
        );
        assert_eq!(
            fixes.keys("gnss_pos"),
            " offered_gnss_pos=12 bad_gnss_pos=5 rejected_bad_gnss_pos=3 \
             rejected_good_gnss_pos=2 accepted_far_gnss_pos=1 adopted_bad_gnss_pos=1 \
             recovered_after_bad_gnss_pos=2 recovered_after_lockout_gnss_pos=2 \
             unjudged_gnss_pos=1 rms_gnss_pos=2.082"
        );
    }

    #[test]
    fn a_receiver_nothing_judged_has_no_rms() {
        // `rms_gnss_pos=0.000` would claim a perfect receiver on a run whose fixes all fell
        // outside the truth.
        let mut fixes = FixScore::default();
        fixes.record(Fusion::Accepted { test_ratio: 0.1 }, None, false);
        assert!(fixes.keys("gnss_pos").ends_with(" rms_gnss_pos=none"));
    }

    #[test]
    fn a_fix_between_truth_rows_is_judged_on_the_two_interpolated() {
        // Rows 0.25 s apart, north 0 → 4 m and heading 0 → 90°. A fix a quarter of the way,
        // at 1.0625 s, sits on truth 1 m north heading 22.5°, where a metre-forward antenna
        // is 0.924 m north and 0.383 m east: a fix at 1.924 m north is exactly there. Not
        // halfway, so a fraction taken from the wrong end reads 3 m, not 1. The error is the
        // horizontal distance and the height apart.
        let row = |t: f64, north: f32, yaw: f32| {
            let mut values = [0.0; STATES];
            values[0] = north;
            values[8] = yaw;
            (t, values)
        };
        let mut log = TruthLog::new();
        for (t, values) in [
            row(1.0, 0.0, 0.0),
            row(1.25, 4.0, core::f32::consts::FRAC_PI_2),
        ] {
            log = log.row(t, values);
        }
        let scoring = log.scoring();
        let judge = |t: f64, fix: [f32; 3]| {
            scoring.judge(
                t,
                Position::ned(fix[0], fix[1], fix[2]),
                [1.0; 3],
                Position::body(1.0, 0.0, 0.0),
                &Accuracy::default(),
            )
        };
        let [horizontal, height] =
            judge(1.0625, [1.923_880, 0.382_683, 0.5]).expect("rows 0.25 s apart");
        assert!(horizontal.error < 1e-3, "{}", horizontal.error);
        assert!((height.error - 0.5).abs() < 1e-6, "{}", height.error);
        // 3 m north and 4 m east of it is 5 m off: the distance, not either axis.
        let [off, _] = judge(1.0625, [4.923_880, 4.382_683, 0.5]).expect("the same instant");
        assert!((off.error - 5.0).abs() < 1e-3, "{}", off.error);
        // A row at the fix's own time is read as written, not blended with its neighbour.
        let [at_row, _] = judge(1.25, [4.0, 1.0, 0.0]).expect("a row at 1.25 s");
        assert!(at_row.error < 1e-6, "{}", at_row.error);
    }

    #[test]
    fn a_fix_across_a_gap_in_the_truth_is_not_judged_and_one_on_a_row_is() {
        // Rows 0.35 s apart, past `TRUTH_SPAN`: a gap, not a dropped row, and what the
        // vehicle did inside it is not the line between its ends. A fix on a row is judged
        // however far its neighbour is, rounding included, which is all of UrbanNav's 1 Hz
        // truth.
        let scoring = TruthLog::new().still(1.0, 2, 0.35).scoring();
        let judged = |t: f64| {
            scoring
                .judge(
                    t,
                    Position::zero(),
                    [1.0; 3],
                    Position::zero(),
                    &Accuracy::default(),
                )
                .is_some()
        };
        assert!(!judged(1.1), "inside the gap");
        assert!(
            judged(1.35005),
            "on the later row, within `TRUTH_TOLERANCE`"
        );
    }

    #[test]
    fn a_gap_in_the_log_is_not_a_mismatch_and_a_denser_truth_is() {
        // Truth every period, epochs every period with five missing: the rows in the gap had
        // no epoch to be scored at. Truth four times as dense as the epochs: three rows in
        // four had one within a period, and were passed.
        let covariance = Covariance::from_sigmas([0.5; STATES]);
        let state = state_at(0.0, 0.0, 0.0);
        // The tolerance is the log's own period, as the harness hands it over.
        let unmatched = |truth: TruthLog, epochs: &[f64]| {
            let mut scoring = truth.scoring();
            let period = epochs[1] - epochs[0];
            for &t in epochs {
                scoring.epoch(
                    t,
                    period,
                    &state,
                    &covariance,
                    AttitudeVariance::default(),
                    &Accuracy::default(),
                );
            }
            (scoring.score.scored, scoring.unmatched())
        };
        let gap: Vec<f64> = (0..20)
            .filter(|i| !(8..13).contains(i))
            .map(|i| i as f64 * DT)
            .collect();
        assert_eq!(unmatched(TruthLog::new().still(0.0, 20, DT), &gap), (15, 0));
        let sparse: Vec<f64> = (0..5).map(|i| i as f64 * 4.0 * DT).collect();
        assert_eq!(
            unmatched(TruthLog::new().still(0.0, 17, DT), &sparse),
            (5, 12)
        );
    }

    #[test]
    fn a_truth_without_biases_scores_the_trajectory_and_leaves_ba_and_bg_none() {
        // Blank bias columns are unknown, not zero: the estimate's zero bias would otherwise
        // score perfect against them, and `in3s` would count six axes nobody measured.
        let scoring = heading_truth(1.0, 0.0);
        let row = scoring.truth.rows[0];
        assert!(row.accel_bias.is_none() && row.gyro_bias.is_none());
        let score = score_one(
            &state_at(0.0, 0.0, 0.0),
            &Covariance::from_sigmas([0.5; STATES]),
            &row,
        );
        assert_eq!(key(&score.line(), "ba"), "none");
        assert_eq!(key(&score.line(), "bg"), "none");
        assert_eq!(score.axes, 9, "position, velocity and attitude only");
        let partial = format!("{}0.0000{},1,,\n", TruthLog::new().0, ",0".repeat(12));
        assert_eq!(truth_error(&partial), "line 3: malformed row");
    }

    #[test]
    fn sparse_truth_is_scored_once_a_row_and_is_not_a_mismatch() {
        // Rows 10 epochs apart, one of them 0.4 of a period off an epoch: every row is scored
        // at one epoch, the epochs between are unscored, and nothing is unmatched.
        let mut scoring = TruthLog::new()
            .row(0.0, [0.0; STATES])
            .row(10.4 * DT, [0.0; STATES])
            .row(20.0 * DT, [0.0; STATES])
            .scoring();
        let state = state_at(0.0, 0.0, 0.0);
        let covariance = Covariance::from_sigmas([0.5; STATES]);
        for epoch in 0..=20 {
            scoring.epoch(
                epoch as f64 * DT,
                DT,
                &state,
                &covariance,
                AttitudeVariance::default(),
                &Accuracy::default(),
            );
        }
        assert_eq!(scoring.score.scored, 3);
        assert_eq!(scoring.unmatched(), 0);
    }

    #[test]
    fn time_in_a_status_runs_to_the_next_transition_and_the_last_ends_the_log() {
        let transitions = [
            (1.0, Status::Aligning),
            (3.0, Status::Degraded),
            (4.5, Status::Healthy),
            (7.0, Status::Degraded),
        ];
        assert_eq!(seconds_in(&transitions, 10.0, Status::Degraded), 4.5);
        assert_eq!(seconds_in(&transitions, 10.0, Status::Aligning), 2.0);
        let never = seconds_in(&transitions, 10.0, Status::DeadReckoning);
        assert_eq!(format!("{never:.2}"), "0.00", "not -0.00");
    }

    #[test]
    fn a_converted_log_pairs_with_its_truth_by_source_digest() {
        let log = "# fusion-nav converted `urbannav-tst-m8t` from UrbanNav, source 39a76af5d675\n";
        let truth = "# fusion-nav truth for `urbannav-tst-m8t.csv`, source 39a76af5d675\n";
        let other = "# fusion-nav truth for `urbannav-tst-m8t.csv`, source 000000000000\n";
        assert_eq!(scenario_of(log), scenario_of(truth));
        assert_ne!(scenario_of(log), scenario_of(other));
        // A marker whose tail names neither a seed nor a source pairs with nothing.
        assert_eq!(
            scenario_of("# fusion-nav note on `x`, with a comma\n"),
            None
        );
    }

    // ---- the truth file belongs to the log ----

    #[test]
    fn a_log_and_its_truth_name_the_same_scenario() {
        // The two headers spell it differently — the log without the `.csv`, the truth with
        // — and the whole check rests on them coming out equal anyway.
        let log = "# fusion-nav simulated flight - scenario `mission`, seed 2\n# covers\n";
        let truth = "# fusion-nav truth for `mission.csv`, seed 2\n#\n";
        assert_eq!(
            scenario_of(log),
            Some(("mission".to_string(), "seed 2".to_string()))
        );
        assert_eq!(scenario_of(log), scenario_of(truth));
    }

    #[test]
    fn truth_from_another_scenario_does_not_match_the_log() {
        // The failure this exists for: nine `*.truth.csv` one tab-completion apart, and
        // `unmatched` blind to the mix-up because a 50 Hz log lands on every fourth row of
        // 200 Hz truth and matches every epoch.
        let log = "# fusion-nav simulated flight - scenario `flight`, seed 8\n";
        let truth = "# fusion-nav truth for `mission.csv`, seed 2\n";
        assert_ne!(scenario_of(log), scenario_of(truth));
    }

    #[test]
    fn the_same_scenario_regenerated_on_another_seed_does_not_match() {
        // Same trajectory, different draws. The name alone would let this through.
        let a = "# fusion-nav truth for `mission.csv`, seed 2\n";
        let b = "# fusion-nav truth for `mission.csv`, seed 3\n";
        assert_ne!(scenario_of(a), scenario_of(b));
    }

    #[test]
    fn a_file_with_no_marker_is_taken_on_trust() {
        // A corpus log, a converted one, a hand-written one. Nothing in them could be
        // checked, so the check stands down rather than refusing the file.
        assert_eq!(scenario_of("t_s,source,v0,v1,v2\n"), None);
        assert_eq!(
            scenario_of("# converted from somewhere else\nt_s,source\n"),
            None
        );
    }

    #[test]
    fn false_valid_counts_a_claim_only_where_the_filter_made_one() {
        // Ten metres of horizontal position error against a 5 m bar. Claimed, it is a false
        // valid; unclaimed, the filter said so itself and there is nothing to report.
        let truth = truth_offset(10.0, 0.0, 0.0);
        let covariance = Covariance::from_sigmas([0.5; STATES]);
        let claimed = score_one(&state_at(0.0, 0.0, 0.0), &covariance, &truth);
        assert_eq!(claimed.false_valid(), 1);
        assert_eq!(
            claimed.false_valid[2], 1,
            "horizontal position, in QUANTITIES order"
        );

        let mut honest = state_at(0.0, 0.0, 0.0);
        honest.validity.horizontal_position = false;
        let honest = score_one(&honest, &covariance, &truth);
        assert_eq!(honest.false_valid(), 0, "the filter claimed nothing");
    }

    #[test]
    fn a_truth_header_that_is_not_the_agreed_columns_is_refused() {
        // Nothing connects this list to `write_truth_header` in `examples/simulate.rs` at
        // compile time, so a column renamed or reordered there has to stop here rather than
        // score one quantity against another.
        let swapped = TRUTH_COLUMNS
            .join(",")
            .replace("pos_n,pos_e", "pos_e,pos_n");
        let error = truth_error(&format!("{swapped}\n0.0000{}\n", ",0.0".repeat(15)));
        assert!(error.contains("truth columns are"), "{error}");
    }

    #[test]
    fn a_truth_file_with_no_rows_is_refused() {
        assert_eq!(truth_error(&TruthLog::new().0), "no truth rows");
    }

    #[test]
    fn a_repeated_epoch_is_answered_from_the_same_truth_row() {
        // A duplicate IMU row is refused as an invalid step and still writes an epoch, so
        // the same timestamp is asked for twice. The cursor moves on only to a strictly nearer row, which is what
        // makes the second answer the first.
        let mut truth = TruthLog::new().still(0.0, 3, DT).parse().expect("parses");
        assert!(truth.at(DT, DT).is_some());
        assert!(truth.at(DT, DT).is_some(), "asked twice, answered twice");
        assert!(truth.at(2.0 * DT, DT).is_some(), "and still moves on");
    }

    #[test]
    fn an_epoch_with_no_truth_row_is_counted_rather_than_guessed() {
        // Truth stops at 0.04 s and the epochs do not. Nothing is interpolated: the two
        // files come from one run of the simulator, so a gap means they came from two.
        let mut scoring = TruthLog::new().still(0.0, 3, DT).scoring();
        let state = state_at(0.0, 0.0, 0.0);
        let covariance = Covariance::from_sigmas([0.5; STATES]);
        for epoch in 0..5 {
            scoring.epoch(
                epoch as f64 * DT,
                DT,
                &state,
                &covariance,
                AttitudeVariance::default(),
                &Accuracy::default(),
            );
        }
        assert_eq!(scoring.score.scored, 3);
        assert_eq!(scoring.unmatched(), 2);
    }

    #[test]
    fn the_nees_file_has_a_row_per_epoch_and_leaves_an_unscored_one_empty() {
        // Truth for two epochs of three. The third is written, empty, rather than dropped:
        // `tools/anees.py` aligns seeds row by row and refuses an empty field, so a missing
        // truth row stops the ensemble instead of shifting every later epoch by one.
        let mut scoring = TruthLog::new().still(0.0, 2, DT).scoring();
        scoring.scenario = Some(("fixture".to_string(), "seed 7".to_string()));
        let state = state_at(0.0, 0.0, 0.0);
        let covariance = Covariance::from_sigmas([0.5; STATES]);
        for epoch in 0..3 {
            scoring.epoch(
                epoch as f64 * DT,
                DT,
                &state,
                &covariance,
                AttitudeVariance::default(),
                &Accuracy::default(),
            );
        }
        let mut out = Vec::new();
        scoring.write_nees(&mut out).expect("written");
        let text = String::from_utf8(out).expect("utf-8");
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], "# fusion-nav nees for `fixture`, seed 7");
        assert_eq!(lines[2], "t_s,nees_pos,nees_vel,nees_att");
        // A zero error has zero NEES in every block.
        assert_eq!(lines[3], "0.0000,0.000000,0.000000,0.000000");
        assert_eq!(lines[5], "0.0400,,,");
        assert_eq!(lines.len(), 6);
    }

    #[test]
    fn the_error_file_bands_each_error_with_the_sigma_on_its_own_axis() {
        // Truth 1.5 m north and 0.25 rad of heading away from a level estimate at the origin.
        // The body-axis attitude sigmas (0.7, 0.8, 0.9 rad) differ from the navigation-axis
        // ones handed in (0.1, 0.2, 0.3), so a band read off the covariance diagonal for
        // tilt or heading fails here rather than drawing a plausible-looking figure.
        let mut values = [0.0f32; STATES];
        values[0] = 1.5;
        values[8] = 0.25;
        let mut scoring = TruthLog::new().row(0.0, values).scoring();
        scoring.scenario = Some(("fixture".to_string(), "seed 7".to_string()));
        let state = state_at(0.0, 0.0, 0.0);
        let covariance = Covariance::from_sigmas([
            0.5, 0.6, 0.7, 0.1, 0.2, 0.3, 0.7, 0.8, 0.9, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
        ]);
        let attitude = AttitudeVariance {
            tilt_north: 0.01,
            tilt_east: 0.04,
            heading: 0.09,
        };
        for epoch in 0..2 {
            scoring.epoch(
                epoch as f64 * DT,
                DT,
                &state,
                &covariance,
                attitude,
                &Accuracy::default(),
            );
        }
        let mut out = Vec::new();
        scoring.write_error(&mut out).expect("written");
        let text = String::from_utf8(out).expect("utf-8");
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], "# fusion-nav error for `fixture`, seed 7");
        assert_eq!(
            lines[2],
            "t_s,err_pos_n,sigma_pos_n,err_pos_e,sigma_pos_e,err_pos_d,sigma_pos_d,\
             err_vel_n,sigma_vel_n,err_vel_e,sigma_vel_e,err_vel_d,sigma_vel_d,\
             err_tilt_n,sigma_tilt_n,err_tilt_e,sigma_tilt_e,err_heading,sigma_heading"
        );
        assert_eq!(
            lines[3],
            "0.0000,1.500000,0.500000,0.000000,0.600000,0.000000,0.700000,\
             0.000000,0.100000,0.000000,0.200000,0.000000,0.300000,\
             0.000000,0.100000,0.000000,0.200000,0.250000,0.300000"
        );
        // No truth for the second epoch: written, empty, so rows stay one per epoch.
        assert_eq!(lines[4], format!("0.0200{}", ",".repeat(18)));
        assert_eq!(lines.len(), 5);
    }

    #[test]
    fn a_log_scored_against_its_own_truth_counts_every_epoch() {
        // End to end: the harness reaches the scorer once per epoch written, and with the
        // same state the epoch row carries. A still log against still truth is exact.
        let log = still_start().run(2.0, 10, DT, STILL);
        let scoring = TruthLog::new().still(0.0, 110, DT).scoring();
        let (replay, _) = drive(&log, Some(scoring)).expect("fixture replays");
        let score = &replay.scoring.as_ref().expect("scoring").score;
        assert_eq!(key(&replay.summary(), "epochs"), "10");
        assert_eq!(score.scored, 10, "one scored epoch per written epoch");
        assert_eq!(replay.scoring.as_ref().expect("scoring").unmatched(), 0);
        assert_eq!(key(&score.line(), "pos_h"), "0.000");
        assert_eq!(key(&score.line(), "scored"), "10");
    }

    #[test]
    fn a_log_with_no_truth_file_has_no_score_at_all() {
        // A missing line, never a line of zeros: `pos_h=0.000` on a corpus log would claim a
        // perfect filter where the honest answer is that nothing knows.
        let replay = replay(&still_start().run(2.0, 10, DT, STILL));
        assert!(replay.scoring.is_none());
        assert!(
            !replay.summary().contains("pos_h"),
            "and the summary line is untouched"
        );
    }

    // ---- the output shape ----

    #[test]
    fn the_header_names_one_column_per_field_in_a_row() {
        // `ESTIMATE`, `SIGMAS`, `ATTITUDE_SIGMAS` and `RATIOS` drive both sides, and this is what says they
        // still do.
        let mut out = Vec::new();
        write_header(&mut out).expect("header");
        let mut replay = Replay::new(Config::default(), RPolicy::Raw, None).unwrap();
        let log = still_start().run(2.0, 1, DT, STILL);
        {
            let mut sinks = Sinks {
                epochs: &mut out,
                fusions: &mut io::sink(),
            };
            for line in log.0.lines() {
                replay.row(line, &mut sinks).expect("fixture replays");
            }
        }
        let text = String::from_utf8(out).expect("utf-8");
        let mut lines = text.lines();
        let header = lines.next().expect("a header");
        let row = lines.next().expect("an epoch");
        assert_eq!(
            header.split(',').count(),
            row.split(',').count(),
            "header:\n{header}\nrow:\n{row}"
        );
        assert_eq!(header.split(',').count(), 2 + 16 + 15 + 3 + 7);
    }

    #[test]
    fn the_attitude_columns_are_the_quaternion_scalar_first() {
        // Pitched 90° about body y: `q = (cos 45°, 0, sin 45°, 0)`. A scalar-last write
        // still reads as a valid rotation — a half turn about a tilted axis — so only the
        // position of the one non-zero pair says which convention reached the file.
        let state =
            State {
                attitude: Attitude::from_body_to_ned(components(
                    UnitQuaternion::from_euler_angles(0.0, core::f32::consts::FRAC_PI_2, 0.0),
                )),
                ..State::default()
            };
        let column = |name: &str| {
            let (_, read) = ESTIMATE
                .iter()
                .find(|(n, _)| *n == name)
                .expect("column exists");
            read(&state)
        };
        let half = core::f32::consts::FRAC_1_SQRT_2;
        assert!((column("q0") - half).abs() < 1e-6, "q0 {}", column("q0"));
        assert!(column("q1").abs() < 1e-6, "q1 {}", column("q1"));
        assert!((column("q2") - half).abs() < 1e-6, "q2 {}", column("q2"));
        assert!(column("q3").abs() < 1e-6, "q3 {}", column("q3"));
    }
}
