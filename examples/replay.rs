//! Replay a recorded flight from CSV and write the estimate back out as CSV.
//!
//! Nothing here estimates anything after initialization — `predict` propagates nothing and
//! every `fuse_*` accepts unconditionally, so every estimate column holds whatever the
//! window put there and every test ratio comes out zero. What this establishes is the replay
//! harness and the normalized log format `GOALS.md` commits to. What it exercises, that `basic.rs` and
//! `degradation.rs` cannot, is irregular `dt` taken from timestamps, per-sample variance,
//! a source that appears partway through the log, and an initialization window found in
//! the data rather than asserted.
//!
//! Run with `cargo run --example replay`, or point it at your own files:
//!
//! ```text
//! cargo run --example replay -- data/flight.csv target/replay.csv
//! ```
//!
//! # Input
//!
//! One row per measurement, sorted by time. `#` comments and the header are skipped, and
//! blank cells are those not applicable to that source.
//!
//! ```text
//! t_s,source,v0,v1,v2,v3,v4,v5,var0,var1,var2
//! 0.0200,imu,0.0004,0.0020,0.0001,0.0054,0.0299,-9.8067,,,
//! 2.0000,gnss_pos,0,0,0,,,,2.25,2.25,5.625
//! 2.0000,baro,0.0273,,,,,,4,,
//! ```
//!
//! One interleaved stream rather than a file per sensor, because the interleaving is the
//! thing worth exercising. Converting rosbag, ULog, or a dataflash log into this shape is
//! a separate concern, deliberately: the replay path stays free of ROS and PX4 tooling so
//! it runs in CI with no hardware and no toolchain.
//!
//! # Output
//!
//! Two files. `<out>.csv` holds one row per IMU epoch after initialization: the state, the
//! covariance diagonal as standard deviations, and the most recent test ratio per source.
//! The sigma columns are what make the result plottable as estimate ± 3σ against truth; the
//! test ratios are directly comparable with the innovation ratios PX4 publishes.
//!
//! `<out>.fusion.csv` holds one row per `fuse_*` call — the resolution the epoch file cannot
//! reach, since it keeps only the *last* ratio per source and so cannot tell two fusions
//! apart or say what became of one:
//!
//! ```text
//! # one row per fuse_* call. gates gnss_pos=7.81 gnss_vel=7.81 baro=3.84 mag=3.84
//! t_s,source,nu0,nu1,nu2,s0,s1,s2,ratio,outcome
//! 2.0000,gnss_pos,,,,,,,0.0000,accepted
//! 0.1000,baro,,,,,,,,not_initialized
//! ```
//!
//! The gates ride in the header because the filter reports `r = ε / γ`: without `γ` a ratio
//! does not go back to `ε`, and `ε` is what a consistency statistic needs.
//!
//! `nu*` and `s*` are empty. The filter publishes no innovation or innovation covariance,
//! and the harness deliberately does not work them out from the measurement and the
//! covariance itself — the update of equations (23)–(28) is about to own that quantity, and
//! a second implementation of it would disagree eventually, while somebody chased a filter
//! bug that did not exist. The columns are here so the shape is settled before there are
//! values for them.

use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

use fusion_nav::prelude::*;

/// Capacity of the initialization window, in samples.
///
/// `Initialization::min_duration` is a span of time, so the samples it takes depend on the
/// log: 2 s is 100 samples at 50 Hz and 800 at 400 Hz. The corpus needs 500 at most, at
/// 250 Hz; this is sized for the faster IMUs a converter will eventually hand it. A fixed
/// array rather than a `Vec`, to stay honest about what the filter itself is allowed to
/// assume — though an embedded caller at 400 Hz would decimate rather than carry 40 KB of
/// window.
const WINDOW: usize = 1024;

/// Intervals sampled before fixing the IMU rate.
const PROBE: usize = 64;

/// How long to keep sliding the window looking for stillness before accepting a coarse
/// start, in seconds of log time.
///
/// A policy, not a filter constant: the harness would rather align properly, but a log
/// whose vehicle is already moving still has to replay.
const PATIENCE: f64 = 10.0;

/// Reads one estimate column out of a `State`.
type Column = fn(&State) -> f32;

/// The estimate columns: each name next to the value it reads. Drives both the header and
/// the row, so the two cannot drift apart.
const ESTIMATE: [(&str, Column); 15] = [
    ("pos_n", |s| s.position.x()),
    ("pos_e", |s| s.position.y()),
    ("pos_d", |s| s.position.z()),
    ("vel_n", |s| s.velocity.x()),
    ("vel_e", |s| s.velocity.y()),
    ("vel_d", |s| s.velocity.z()),
    ("roll", |s| s.attitude.euler_angles().0),
    ("pitch", |s| s.attitude.euler_angles().1),
    ("yaw", |s| s.attitude.euler_angles().2),
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

/// Source names as the input spells them, in `Diagnostics` order, so a fusion row names the
/// row it came from. The constants below index this and `RATIOS` alike.
const SOURCES: [&str; 4] = ["gnss_pos", "gnss_vel", "baro", "mag"];

/// Last test ratio per source, in `Diagnostics` order.
const RATIOS: [&str; 4] = ["r_gnss_pos", "r_gnss_vel", "r_baro", "r_mag"];
const GNSS_POS: usize = 0;
const GNSS_VEL: usize = 1;
const BARO: usize = 2;
const MAG: usize = 3;

fn main() {
    if let Err(e) = run() {
        eprintln!("replay: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let input = args.next().map_or_else(default_input, PathBuf::from);
    let output = args.next().map_or_else(default_output, PathBuf::from);

    // `examples/simulate.rs` writes its magnetic field for this same declination, and says what
    // divergence costs: a heading fused from a generated log would carry the difference as a
    // bias in every score, with nothing failing to say so.
    let config = Config {
        magnetic_declination: Radians::from_radians(-0.06),
        ..Config::default()
    };
    let text = fs::read_to_string(&input).map_err(|e| format!("{}: {e}", input.display()))?;
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    let fusions = output.with_extension("fusion.csv");
    let mut epoch_out = BufWriter::new(File::create(&output)?);
    let mut fusion_out = BufWriter::new(File::create(&fusions)?);
    write_header(&mut epoch_out)?;
    write_fusion_header(&mut fusion_out, config.gates)?;

    let mut replay = Replay::new(config);
    {
        let mut out = Sinks {
            epochs: &mut epoch_out,
            fusions: &mut fusion_out,
        };
        for (n, line) in text.lines().enumerate() {
            replay
                .row(line, &mut out)
                .map_err(|e| format!("{}:{}: {e}", input.display(), n + 1))?;
        }
    }
    epoch_out.flush()?;
    fusion_out.flush()?;

    replay.report(&input, &output, &fusions);
    Ok(())
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

/// Everything the loop carries between rows.
struct Replay {
    filter: Eskf,
    /// Candidate static window, slid forward one sample at a time until `initialize`
    /// accepts it. A log that begins in motion simply initializes later.
    window: [StaticSample; WINDOW],
    filled: usize,
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
    ratios: [Option<f32>; 4],
    initialized_at: Option<f64>,
    alignment: Option<Alignment>,
    mag_at_init: bool,
    /// When `Validity::attitude` first read true, in log time.
    ///
    /// `Accuracy::tilt` equals `Initialization::sigma_tilt` and `Accuracy::heading` equals
    /// `Initialization::sigma_yaw`, compared with `<=`, so a static start passes by exactly
    /// zero margin (`src/config.rs`). Widening those two is a measurement rather than a
    /// guess, and this is the measurement: how long a real log takes before the filter
    /// claims its attitude is usable. It says when the filter *claims* to have converged,
    /// not whether the attitude was good then — that needs truth, which the corpus has not
    /// got.
    aligned_at: Option<f64>,
    /// `Validity::heading` the moment initialization committed — the filter's own verdict
    /// on the window, which is not the same as `mag_at_init`: a coarse start carrying a
    /// magnetometer has observed nothing it could level a heading with.
    heading_at_init: bool,
    /// The attitude equations (5)–(7) committed, captured at that moment rather than read
    /// off the filter at the end. The stub propagates nothing, so the two agree today and
    /// would keep agreeing until (12)–(15) land — at which point this key would silently
    /// become an end-of-log attitude, which is the trap `heading=` already documents.
    attitude_at_init: Option<Attitude>,
    epochs: u32,
    /// Rows written to the fusion file: every `fuse_*` call the log made, whatever its
    /// outcome.
    fusions: u32,
    /// When the worst refused step happened. The count and the size of the gap come from
    /// `diagnostics()`; the filter reads no clock, so the log timestamp is the harness's to
    /// keep.
    longest_step_at: Option<f64>,
    status: Status,
    transitions: Vec<(f64, Status)>,
}

impl Replay {
    fn new(config: Config) -> Self {
        Self {
            filter: Eskf::new(config),
            window: [StaticSample::default(); WINDOW],
            filled: 0,
            interval: None,
            probe: [0.0f64; PROBE],
            probed: 0,
            window_samples: 0,
            last_mag: None,
            last_baro: None,
            pending_velocity: None,
            previous_imu: None,
            first_imu: None,
            ratios: [None; 4],
            initialized_at: None,
            alignment: None,
            mag_at_init: false,
            aligned_at: None,
            heading_at_init: false,
            attitude_at_init: None,
            epochs: 0,
            fusions: 0,
            longest_step_at: None,
            status: Status::default(),
            transitions: Vec::new(),
        }
    }

    fn row(&mut self, line: &str, out: &mut Sinks) -> Result<(), Box<dyn Error>> {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("t_s") {
            return Ok(());
        }
        let r = Record::parse(line).ok_or("malformed row")?;

        match r.source {
            "imu" => {
                let imu = ImuSample {
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
                // Variance comes from the log, per sample: a real GNSS reports its own
                // accuracy, and it degrades before it drops out.
                let outcome = self.filter.fuse_gnss_position(
                    Position::ned(r.value(0)?, r.value(1)?, r.value(2)?),
                    PositionNoise::from_variance(r.variance(0)?, r.variance(1)?, r.variance(2)?),
                );
                self.observe(r.t, GNSS_POS, outcome, out)?;
            }
            "gnss_vel" => {
                let velocity = Velocity::ned(r.value(0)?, r.value(1)?, r.value(2)?);
                // Before initialization the fusion below is refused, and this is the one
                // thing in the row that a window taken in motion can still use.
                self.pending_velocity = Some(velocity);
                let outcome = self.filter.fuse_gnss_velocity(
                    velocity,
                    VelocityNoise::from_variance(r.variance(0)?, r.variance(1)?, r.variance(2)?),
                );
                self.observe(r.t, GNSS_VEL, outcome, out)?;
            }
            "baro" => {
                let altitude = Altitude::from_meters(r.value(0)?);
                self.last_baro = Some(altitude);
                let outcome = self
                    .filter
                    .fuse_baro_altitude(altitude, AltitudeNoise::from_variance(r.variance(0)?));
                self.observe(r.t, BARO, outcome, out)?;
            }
            "mag" => {
                let field = MagField::body(r.value(0)?, r.value(1)?, r.value(2)?);
                self.last_mag = Some(field);
                let outcome = self
                    .filter
                    .fuse_mag_heading(field, HeadingNoise::from_variance(r.variance(0)?));
                self.observe(r.t, MAG, outcome, out)?;
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
    fn accumulate(&mut self, t: f64, imu: ImuSample) -> Result<(), Box<dyn Error>> {
        self.first_imu.get_or_insert(t);
        self.probe_rate(t);
        let sample = StaticSample {
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
        if self.filled < needed {
            return Ok(());
        }
        self.window_samples = needed;
        let window = &self.window[self.filled - needed..self.filled];

        let dt = Seconds::from_secs(interval as f32);
        let alignment = self.filter.alignment_of(window, dt)?;
        if !self.worth_committing(t, alignment) {
            return Ok(());
        }

        self.alignment = Some(alignment);
        // Already classified above; committing it cannot disagree.
        let _ = self.filter.initialize(window, dt)?;
        self.initialized_at = Some(t);
        // The filter's own rule: one magnetometer sample anywhere in the window observes
        // heading, and none at all leaves yaw a prior until a heading is fused.
        self.mag_at_init = window.iter().any(|s| s.mag.is_some());
        let state = self.filter.state();
        self.note_alignment(t, state.validity);
        self.heading_at_init = state.validity.heading;
        self.attitude_at_init = Some(state.attitude);
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

    /// Append a sample, dropping the oldest once the window is full.
    fn push_to_window(&mut self, sample: StaticSample) {
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

    fn propagate(&mut self, t: f64, imu: ImuSample, out: &mut Sinks) -> io::Result<()> {
        if let Some(previous) = self.previous_imu.replace(t) {
            // The filter decides what is too long, not the example.
            let worst_before = self.filter.diagnostics().propagation.longest_refused;
            let propagated = self
                .filter
                .predict(imu, Seconds::from_secs((t - previous) as f32))
                .is_propagated();
            // Timestamp the step that set a new worst, which is the one the filter kept.
            // Comparing to `dt` instead would also match a later step that merely ties it,
            // and move the report's timestamp off the gap the size belongs to.
            if !propagated && self.filter.diagnostics().propagation.longest_refused != worst_before
            {
                self.longest_step_at = Some(t);
            }
        }
        let state = self.filter.state();
        self.note_alignment(t, state.validity);
        if state.status != self.status {
            self.status = state.status;
            self.transitions.push((t, state.status));
        }
        self.epochs += 1;
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
        out: &mut Sinks,
    ) -> io::Result<()> {
        self.ratios[source] = outcome.test_ratio();
        self.fusions += 1;
        // `ν` and the diagonal of `S` are left empty rather than computed here. The filter
        // publishes neither, and the harness working them out from the measurement and the
        // covariance would be a second implementation of a quantity the update of (23)-(28)
        // is about to own — the disagreement `AGENTS.md` keeps one implementation to avoid.
        // The columns exist so the shape is fixed before #36 has values to put in it.
        writeln!(
            out.fusions,
            "{t:.4},{},,,,,,,{},{}",
            SOURCES[source],
            match outcome.test_ratio() {
                Some(ratio) => format!("{ratio:.4}"),
                None => String::new(),
            },
            verdict(outcome),
        )
    }

    /// Sum one `SourceHealth` count over every source.
    fn total(&self, count: fn(&SourceHealth) -> u32) -> u32 {
        self.filter
            .diagnostics()
            .sources()
            .iter()
            .map(|(_, health)| count(health))
            .sum()
    }

    /// Measurements the gate turned down, over every source.
    fn rejections(&self) -> u32 {
        self.total(|health| health.rejected)
    }

    /// Keep the first moment the attitude read valid, and only the first.
    ///
    /// Taken at the commit as well as at every epoch: a window that carried a magnetometer
    /// leaves the filter aligned the instant it closes, and that is zero seconds rather
    /// than one `dt`.
    fn note_alignment(&mut self, t: f64, validity: Validity) {
        if self.aligned_at.is_none() && validity.attitude() {
            self.aligned_at = Some(t);
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

    /// Measurements the filter could not judge at all, over every source.
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

    /// Measurements adopted outright because a coarse start left nothing to fuse them
    /// against. At most one per source.
    fn resets(&self) -> u32 {
        self.total(|health| health.adopted)
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
                    None => "no barometric reference (no barometer in the window, or it was \
                             taken in motion), altitude fusion refused"
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
                                accel.vector().norm(),
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
                "{} never reached the gate at all: a variance of zero or less, a NaN, or \
                 an altitude with no reference — per source below",
                self.discarded()
            );
        }
        if self.resets() > 0 {
            println!(
                "{} adopted outright: a coarse start had no position or velocity to fuse \
                 them against",
                self.resets()
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
        if let Some(worst) = propagation.longest_refused {
            let at = self.longest_step_at.unwrap_or(f64::NAN);
            println!(
                "{} propagation steps refused as longer than {} s, worst {:.3} s at \
                 {at:.2} s\n  a gap is usually the logger missing messages, not the IMU \
                 stopping",
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
            "summary rate={:.0} window={} align={} an={} alpha0={} heading={} \
             roll0={:.2} pitch0={:.2} yaw0={:.2} resets={} \
             aligned_at={} rejected={} discarded={} refused={} invalid={} epochs={} \
             transitions={} status={:?}",
            self.interval.map_or(0.0, |interval| 1.0 / interval),
            self.window_samples,
            match self.alignment {
                Some(Alignment::Static) => "static",
                Some(Alignment::Coarse(..)) => "coarse",
                Some(Alignment::Seeded) => "seeded",
                None => "none",
            },
            // `ā_n` of (5′): only a moving start reports one, and only when its window
            // carried two dated GNSS velocities. `none` therefore covers both "the start
            // was static" and "the window had no GNSS in it", which is why it is pinned
            // rather than derived — on the one log that starts in motion it is the only
            // key that would notice the velocity disappearing out of the window, and
            // in-motion levelling has nothing to correct with when it does.
            if self.inertial_accel().is_some() {
                "measured"
            } else {
                "none"
            },
            // Only a static start establishes the barometric reference, so a coarse log
            // fuses no altitude at all unless the application names one. Pinned here
            // because nothing else in this line would notice barometric aiding vanishing.
            if self.filter.baro_reference().is_some() {
                "set"
            } else {
                "none"
            },
            // `Validity::heading` as initialization left it, not as the log ended.
            // Nothing pins the rotation about gravity except a magnetometer, and the
            // covariance would report it good either way — `Initialization::sigma_yaw`
            // and `Accuracy::heading` are the same number — so what is worth pinning is
            // the verdict on the window. At the end of the log this says only that some
            // magnetic heading was fused at some point, which `transitions` already
            // notices.
            if self.heading_at_init {
                "valid"
            } else {
                "invalid"
            },
            // The attitude of (5)-(7), in degrees: the only keys on this line that would
            // notice a sign inverted in the down-positive convention, a levelling dropped
            // out of (6), or a declination that stopped reaching the filter. Nothing else
            // here looks at attitude at all.
            roll0,
            pitch0,
            yaw0,
            self.resets(),
            // When the filter first called its own attitude usable, which is what would
            // settle `Accuracy`'s attitude defaults off the corpus the way replay settled
            // `Timeouts::degraded_after`. It measures the stub until covariance
            // propagation lands, and is pinned meanwhile.
            self.aligned_after(),
            // The gate's verdict, which nothing on this line reported before: `refused=`
            // and `invalid=` are propagation steps, not measurements, and a change that
            // started turning down every fix in the corpus would have passed `--check`
            // unmoved. It reads zero until the χ² gate of (37) is real, and is pinned
            // from now so that the first non-zero is a diff and not a discovery.
            self.rejections(),
            // Everything that never reached the gate. See `Replay::discarded`.
            self.discarded(),
            self.filter.diagnostics().propagation.refused_too_long,
            self.filter.diagnostics().propagation.refused_invalid,
            self.epochs,
            self.transitions.len(),
            state.status,
        )
    }

    /// Per-quantity validity now, and predicted at takeoff.
    fn report_validity(&self) {
        let validity = self.filter.state().validity;
        let predicted = self.filter.predicted_validity();
        println!("\nvalidity at end of log        now  at takeoff");
        for (name, now, then) in [
            ("tilt", validity.tilt, predicted.tilt),
            ("heading", validity.heading, predicted.heading),
            (
                "position (h)",
                validity.horizontal_position,
                predicted.horizontal_position,
            ),
            (
                "position (v)",
                validity.vertical_position,
                predicted.vertical_position,
            ),
            (
                "velocity (h)",
                validity.horizontal_velocity,
                predicted.horizontal_velocity,
            ),
            (
                "velocity (v)",
                validity.vertical_velocity,
                predicted.vertical_velocity,
            ),
        ] {
            let mark = |flag| if flag { "yes" } else { " no" };
            println!("  {name:<13} {}        {}", mark(now), mark(then));
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
        Some(Self {
            t,
            source,
            values,
            variances,
        })
    }

    fn value(&self, i: usize) -> Result<f32, String> {
        self.values[i].ok_or_else(|| format!("`{}` row has no v{i}", self.source))
    }

    fn variance(&self, i: usize) -> Result<f32, String> {
        self.variances[i].ok_or_else(|| format!("`{}` row has no var{i}", self.source))
    }
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
        Fusion::NotFinite => "not_finite",
        Fusion::InvalidNoise => "invalid_noise",
    }
}

/// The fusion file's header, and the gates that make its ratios readable.
///
/// The filter reports `r = ε / γ`, so a ratio only becomes `ε` — and `ε` only becomes NIS —
/// if `γ` is known. Recording the gates here rather than expecting a reader to look them up
/// keeps the file self-describing: a `Config` change moves the number in the file that the
/// ratios were produced under.
fn write_fusion_header(out: &mut impl Write, gates: Gates) -> io::Result<()> {
    writeln!(
        out,
        "# one row per fuse_* call. gates gnss_pos={} gnss_vel={} baro={} mag={}",
        gates.gnss_position, gates.gnss_velocity, gates.baro_altitude, gates.mag_heading
    )?;
    writeln!(
        out,
        "# nu* and s* are empty: the filter publishes no innovation yet, and the harness \
         does not compute one it would have to agree with later"
    )?;
    writeln!(out, "t_s,source,nu0,nu1,nu2,s0,s1,s2,ratio,outcome")
}

fn write_header(out: &mut impl Write) -> io::Result<()> {
    write!(out, "t_s,status")?;
    for (name, _) in ESTIMATE {
        write!(out, ",{name}")?;
    }
    for (_, name) in SIGMAS {
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
    fn drive(log: &Log) -> Result<(Replay, String), String> {
        let mut replay = Replay::new(Config {
            magnetic_declination: Radians::from_radians(-0.06),
            ..Config::default()
        });
        let mut fusions = Vec::new();
        {
            let mut out = Sinks {
                epochs: &mut io::sink(),
                fusions: &mut fusions,
            };
            for line in log.0.lines() {
                replay.row(line, &mut out).map_err(|e| e.to_string())?;
            }
        }
        Ok((replay, String::from_utf8(fusions).expect("utf-8")))
    }

    /// Replay a fixture, or report the row that stopped it.
    fn try_replay(log: &Log) -> Result<Replay, String> {
        drive(log).map(|(replay, _)| replay)
    }

    /// The fusion rows a fixture produced, header excluded — `write_fusion_header` is not
    /// part of what `Replay` writes, and is checked on its own.
    fn fusion_rows(log: &Log) -> Vec<String> {
        match drive(log) {
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
    fn columns_past_the_last_variance_are_ignored() {
        let r = Record::parse("1.0,imu,0,0,0,0,0,-9.80665,,,,extra,columns").expect("parses");
        assert_eq!(r.value(5), Ok(-9.80665));
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
        // Stillness observes tilt and never yaw, and the covariance cannot say so:
        // `Initialization::sigma_yaw` and `Accuracy::heading` are the same 0.35 rad.
        assert_eq!(key(&replay(&still_start()).summary(), "heading"), "invalid");
    }

    #[test]
    fn a_barometer_in_a_still_window_sets_the_reference() {
        let log = Log::new().baro(0.0, 42.0).run(0.0, 100, DT, STILL);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "alpha0"), "set");
        let reference = replay.filter.baro_reference().expect("α₀ established");
        assert!(
            (reference.as_meters() - 42.0).abs() < 1e-3,
            "the reference is the window's mean altitude: {} m",
            reference.as_meters()
        );
    }

    #[test]
    fn a_window_without_a_barometer_refuses_altitude() {
        // The LPE corpus log (`7592c9b2…`) yields no barometer rows at all; this is that
        // path, and `alpha0=` is the only key that would notice it.
        let log = still_start().baro(2.0, 42.0);
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
            (accel.x() - 2.0).abs() < 1e-3 && accel.vector().norm() > 1.9,
            "2 m/s over 1 s, north: {:?}",
            accel.vector()
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
        for (row, what) in [
            (
                "2.000000,baro,42,,,,,,4,,",
                "no reference to be relative to",
            ),
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
    fn a_gate_that_turns_nothing_down_reports_zero_rejected() {
        // Pinned while it is a constant, so that the first non-zero is a diff rather than
        // a discovery. Nothing rejects until the χ² gate of (37) is real.
        assert_eq!(key(&replay(&still_start()).summary(), "rejected"), "0");
    }

    #[test]
    fn epochs_count_every_imu_row_after_initialization() {
        let log = still_start().run(2.0, 10, DT, STILL);
        assert_eq!(key(&replay(&log).summary(), "epochs"), "10");
    }

    #[test]
    fn a_repeated_timestamp_after_initialization_is_an_invalid_step() {
        // With rows sorted by time, a zero `dt` is a duplicate in practice. The epoch is
        // still written: the row existed, and the state is simply the one before it.
        let log = still_start().imu(99.0 * DT, STILL);
        let summary = replay(&log).summary();
        assert_eq!(key(&summary, "invalid"), "1");
        assert_eq!(key(&summary, "refused"), "0", "nothing here was too long");
        assert_eq!(key(&summary, "epochs"), "1");
    }

    #[test]
    fn a_gap_longer_than_the_limit_is_refused_and_timestamped() {
        // 0.5 s against a `max_predict_dt` of 0.1 s: a logging dropout, which the filter
        // refuses while the timers run on.
        let gap_at = 99.0 * DT + 0.5;
        let log = still_start().imu(gap_at, STILL);
        let replay = replay(&log);
        assert_eq!(key(&replay.summary(), "refused"), "1");
        assert_eq!(
            replay.longest_step_at,
            Some(gap_at),
            "the filter holds the size of the gap; the harness holds when it happened"
        );
        let worst = replay
            .filter
            .diagnostics()
            .propagation
            .longest_refused
            .expect("a refused step");
        assert!(
            (worst.as_secs() - 0.5).abs() < 1e-3,
            "{} s",
            worst.as_secs()
        );
    }

    #[test]
    fn a_coarse_start_adopts_one_position_and_one_velocity() {
        // Nothing was ever established, so the first fix is adopted rather than fused —
        // once per quantity, and never for recovery.
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
        assert_eq!(
            key(&summary, "resets"),
            "2",
            "the second pair is fused, not adopted"
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
    fn every_fuse_call_writes_one_row_whatever_it_returned() {
        // Three aiding rows before the window closes and three after. The summary counts
        // only what was accepted or refused; this is the record that they happened at all.
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
        assert_eq!(fusion_rows(&log).len(), 6);
    }

    #[test]
    fn a_fusion_row_names_the_verdict_it_got() {
        // One fixture per outcome the stub can actually produce. `rejected` needs the χ²
        // gate of (37); `verdict` is exhaustive on `Fusion`, so the variant cannot be
        // dropped silently while it waits.
        let coarse = Log::new()
            .run(0.0, 501, DT, TURNING)
            .gnss_pos(10.02, 1.0, 2.0, -3.0)
            .imu(10.02, TURNING);
        for (log, expected) in [
            (still_start().gnss_pos(2.0, 0.0, 0.0, 0.0), "accepted"),
            (Log::new().gnss_pos(0.0, 0.0, 0.0, 0.0), "not_initialized"),
            (
                still_start().raw("2.000000,baro,42,,,,,,4,,"),
                "no_reference",
            ),
            (
                still_start().raw("2.000000,gnss_pos,0,0,0,,,,0,2.25,5.625"),
                "invalid_noise",
            ),
            (
                still_start().raw("2.000000,gnss_pos,nan,0,0,,,,2.25,2.25,5.625"),
                "not_finite",
            ),
            (coarse, "reset"),
        ] {
            let last = fusion_rows(&log).pop().expect("a fusion row");
            assert_eq!(
                last.rsplit(',').next(),
                Some(expected),
                "expected {expected}: {last}"
            );
        }
    }

    #[test]
    fn the_innovation_columns_stay_empty_until_the_filter_publishes_one() {
        // Pinned rather than left to be noticed. The harness could work `ν` out from the
        // measurement and the covariance, and must not: the update of (23)-(28) is about to
        // own that quantity, and two implementations of it would disagree while somebody
        // chases a filter bug that does not exist.
        let log = still_start().gnss_pos(2.0, 1.0, 2.0, -3.0);
        let row = fusion_rows(&log).pop().expect("a fusion row");
        let fields: Vec<&str> = row.split(',').collect();
        assert_eq!(&fields[2..8], &["", "", "", "", "", ""], "ν and S: {row}");
        assert_eq!(fields[8], "0.0000", "the ratio is published: {row}");
    }

    #[test]
    fn the_fusion_header_carries_the_gates_the_ratios_were_produced_under() {
        // `r = ε / γ`, so a ratio without its `γ` is not recoverable to NIS.
        let mut out = Vec::new();
        let config = Config {
            gates: Gates {
                baro_altitude: 2.71,
                ..Gates::default()
            },
            ..Config::default()
        };
        write_fusion_header(&mut out, config.gates).expect("header");
        let text = String::from_utf8(out).expect("utf-8");
        assert!(text.contains("baro=2.71"), "the gate in force: {text}");
        assert_eq!(
            text.lines().last(),
            Some("t_s,source,nu0,nu1,nu2,s0,s1,s2,ratio,outcome")
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

    // ---- the output shape ----

    #[test]
    fn the_header_names_one_column_per_field_in_a_row() {
        // `ESTIMATE`, `SIGMAS` and `RATIOS` drive both sides, and this is what says they
        // still do.
        let mut out = Vec::new();
        write_header(&mut out).expect("header");
        let mut replay = Replay::new(Config::default());
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
        assert_eq!(header.split(',').count(), 2 + 15 + 15 + 4);
    }
}
