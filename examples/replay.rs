//! Replay a recorded flight from CSV and write the estimate back out as CSV.
//!
//! Nothing here estimates anything — `predict` propagates nothing and every `fuse_*`
//! accepts unconditionally, so every estimate column comes out constant and every test
//! ratio comes out zero. What this establishes is the replay harness and the normalized
//! log format `GOALS.md` commits to. What it exercises, that `basic.rs` and
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
//! One row per IMU epoch after initialization: the state, the covariance diagonal as
//! standard deviations, and the most recent test ratio per source. The sigma columns are
//! what make the result plottable as estimate ± 3σ against truth; the test ratios are
//! directly comparable with the innovation ratios PX4 publishes.

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

/// Last test ratio per source, in `Diagnostics` order. The constants below index it.
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

    let config = Config {
        magnetic_declination: Radians::from_radians(-0.06),
        ..Config::default()
    };
    let text = fs::read_to_string(&input).map_err(|e| format!("{}: {e}", input.display()))?;
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut out = BufWriter::new(File::create(&output)?);
    write_header(&mut out)?;

    let mut replay = Replay::new(config);
    for (n, line) in text.lines().enumerate() {
        replay
            .row(line, &mut out)
            .map_err(|e| format!("{}:{}: {e}", input.display(), n + 1))?;
    }
    out.flush()?;

    replay.report(&input, &output);
    Ok(())
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
    /// `Validity::heading` the moment initialization committed — the filter's own verdict
    /// on the window, which is not the same as `mag_at_init`: a coarse start carrying a
    /// magnetometer has observed nothing it could level a heading with.
    heading_at_init: bool,
    epochs: u32,
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
            heading_at_init: false,
            epochs: 0,
            longest_step_at: None,
            status: Status::default(),
            transitions: Vec::new(),
        }
    }

    fn row(&mut self, line: &str, out: &mut impl Write) -> Result<(), Box<dyn Error>> {
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
                self.observe(GNSS_POS, outcome);
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
                self.observe(GNSS_VEL, outcome);
            }
            "baro" => {
                let altitude = Altitude::from_meters(r.value(0)?);
                self.last_baro = Some(altitude);
                let outcome = self
                    .filter
                    .fuse_baro_altitude(altitude, AltitudeNoise::from_variance(r.variance(0)?));
                self.observe(BARO, outcome);
            }
            "mag" => {
                let field = MagField::body(r.value(0)?, r.value(1)?, r.value(2)?);
                self.last_mag = Some(field);
                let outcome = self
                    .filter
                    .fuse_mag_heading(field, HeadingNoise::from_variance(r.variance(0)?));
                self.observe(MAG, outcome);
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
        self.heading_at_init = state.validity.heading;
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

    fn propagate(&mut self, t: f64, imu: ImuSample, out: &mut impl Write) -> io::Result<()> {
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
        if state.status != self.status {
            self.status = state.status;
            self.transitions.push((t, state.status));
        }
        self.epochs += 1;
        self.write_row(t, state, out)
    }

    /// Carry the test ratio into the output row. Counting is the filter's job — `AGENTS.md`,
    /// one statistic, one implementation — so everything else this row did is read back out
    /// of `diagnostics()` at the end.
    fn observe(&mut self, source: usize, outcome: Fusion) {
        self.ratios[source] = outcome.test_ratio();
    }

    /// Measurements the gate turned down, over every source.
    fn rejections(&self) -> u32 {
        self.filter
            .diagnostics()
            .sources()
            .iter()
            .map(|(_, health)| health.rejected)
            .sum()
    }

    /// Measurements adopted outright because a coarse start left nothing to fuse them
    /// against. At most one per source.
    fn resets(&self) -> u32 {
        self.filter
            .diagnostics()
            .sources()
            .iter()
            .map(|(_, health)| health.adopted)
            .sum()
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

    fn write_row(&self, t: f64, state: State, out: &mut impl Write) -> io::Result<()> {
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
    fn report(&self, input: &Path, output: &Path) {
        self.report_initialization(input, output);
        self.report_steps();
        self.report_transitions();
        self.report_summary();
        self.report_validity();
        self.report_sources();
    }

    /// The input, the output, and how initialization went.
    fn report_initialization(&self, input: &Path, output: &Path) {
        println!("fusion-nav replay — no filtering is performed\n");
        println!("in   {}", input.display());
        println!("out  {}", output.display());

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
        let state = self.filter.state();
        println!(
            "\nsummary rate={:.0} window={} align={} an={} alpha0={} heading={} resets={} \
             refused={} invalid={} epochs={} transitions={} status={:?}",
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
            self.resets(),
            self.filter.diagnostics().propagation.refused_too_long,
            self.filter.diagnostics().propagation.refused_invalid,
            self.epochs,
            self.transitions.len(),
            state.status,
        );
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
        // 2.5 ms step comes back 2.3% wrong, and past ~5 hours consecutive samples
        // collapse onto the same value and the step reads as zero. That noise would look
        // like filter error during validation. Values stay `f32`; only time is widened.
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
