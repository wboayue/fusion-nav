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

/// Capacity of the initialization window. Must be at least `Config::init.min_samples`.
/// A fixed array rather than a `Vec`, to keep the example honest about what the filter
/// itself is allowed to assume.
const WINDOW: usize = 100;

/// Estimate columns, in the order `write_row` emits them.
const ESTIMATE: [&str; 15] = [
    "pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d", "roll", "pitch", "yaw", "ba_x", "ba_y",
    "ba_z", "bg_x", "bg_y", "bg_z",
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

/// A `dt` above this is not an IMU interval. Real logs contain them: an SD card that
/// misses messages leaves a hole the replay sees as one long step, and propagating a
/// discretization built for milliseconds across a second of it is meaningless. The
/// example reports them rather than hiding them.
const LONG_STEP: f32 = 0.1;

/// Last test ratio per source, in `Diagnostics` order.
const RATIOS: [&str; 4] = ["r_gnss_pos", "r_gnss_vel", "r_baro", "r_mag"];

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
    if config.init.min_samples > WINDOW {
        return Err(format!(
            "window holds {WINDOW} samples, config requires {}",
            config.init.min_samples
        )
        .into());
    }

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
    /// Most recent magnetometer reading, attached to static samples so a log that does
    /// carry mag during the window gets an observed initial heading. The bundled log does
    /// not, which is the point: yaw starts unobserved and `sigma_yaw` stays inflated.
    last_mag: Option<MagField<Body>>,
    /// Timestamp of the previous IMU row, so `dt` comes from the log rather than from an
    /// assumed rate.
    previous_imu: Option<f32>,
    ratios: [Option<f32>; 4],
    initialized_at: Option<f32>,
    mag_at_init: bool,
    epochs: u32,
    rejections: u32,
    long_steps: u32,
    longest_step: (f32, f32),
    status: Status,
    transitions: Vec<(f32, Status)>,
}

impl Replay {
    fn new(config: Config) -> Self {
        Self {
            filter: Eskf::new(config),
            window: [StaticSample::default(); WINDOW],
            filled: 0,
            last_mag: None,
            previous_imu: None,
            ratios: [None; 4],
            initialized_at: None,
            mag_at_init: false,
            epochs: 0,
            rejections: 0,
            long_steps: 0,
            longest_step: (0.0, 0.0),
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
                    gyro: AngularRate::from_rad_per_s(r.value(0)?, r.value(1)?, r.value(2)?),
                    accel: Acceleration::from_m_per_s2(r.value(3)?, r.value(4)?, r.value(5)?),
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
                    Position::<Ned>::from_meters(r.value(0)?, r.value(1)?, r.value(2)?),
                    PositionVariance::from_m2(r.variance(0)?, r.variance(1)?, r.variance(2)?),
                );
                self.observe(0, outcome);
            }
            "gnss_vel" => {
                let outcome = self.filter.fuse_gnss_velocity(
                    Velocity::<Ned>::from_m_per_s(r.value(0)?, r.value(1)?, r.value(2)?),
                    VelocityVariance::from_m2_per_s2(
                        r.variance(0)?,
                        r.variance(1)?,
                        r.variance(2)?,
                    ),
                );
                self.observe(1, outcome);
            }
            "baro" => {
                let outcome = self.filter.fuse_baro_altitude(
                    Altitude::from_meters(r.value(0)?),
                    AltitudeVariance::from_m2(r.variance(0)?),
                );
                self.observe(2, outcome);
            }
            "mag" => {
                let field =
                    MagField::<Body>::from_components(r.value(0)?, r.value(1)?, r.value(2)?);
                self.last_mag = Some(field);
                let outcome = self
                    .filter
                    .fuse_mag_heading(field, HeadingVariance::from_rad2(r.variance(0)?));
                self.observe(3, outcome);
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
    fn accumulate(&mut self, t: f32, imu: ImuSample) -> Result<(), Box<dyn Error>> {
        let sample = StaticSample {
            imu,
            mag: self.last_mag,
        };
        if self.filled == WINDOW {
            self.window.copy_within(1.., 0);
            self.window[WINDOW - 1] = sample;
        } else {
            self.window[self.filled] = sample;
            self.filled += 1;
        }
        if self.filled < WINDOW {
            return Ok(());
        }

        match self.filter.initialize(&self.window) {
            Ok(()) => {
                self.initialized_at = Some(t);
                self.mag_at_init = self.window.iter().all(|s| s.mag.is_some());
                // The window's last sample is this one, so the next row's `dt` is
                // measured from here.
                self.previous_imu = Some(t);
                self.status = self.filter.state().status;
                self.transitions.push((t, self.status));
            }
            // Still moving. Slide by one and try again on the next IMU row.
            Err(InitError::NotStationary) => {}
            Err(e) => return Err(e.into()),
        }
        Ok(())
    }

    fn propagate(&mut self, t: f32, imu: ImuSample, out: &mut impl Write) -> io::Result<()> {
        if let Some(previous) = self.previous_imu.replace(t) {
            let dt = t - previous;
            if dt > LONG_STEP {
                self.long_steps += 1;
                if dt > self.longest_step.1 {
                    self.longest_step = (t, dt);
                }
            }
            self.filter.predict(imu, Seconds::from_secs(dt));
        }
        let state = self.filter.state();
        if state.status != self.status {
            self.status = state.status;
            self.transitions.push((t, state.status));
        }
        self.epochs += 1;
        self.write_row(t, state, out)
    }

    fn observe(&mut self, source: usize, outcome: Fusion) {
        self.ratios[source] = outcome.test_ratio();
        if matches!(outcome, Fusion::Rejected { .. }) {
            self.rejections += 1;
        }
    }

    fn write_row(&self, t: f32, state: State, out: &mut impl Write) -> io::Result<()> {
        let (roll, pitch, yaw) = state.attitude.euler_angles();
        let position = state.position.as_meters();
        let velocity = state.velocity.as_m_per_s();
        let accel_bias = state.accel_bias.as_m_per_s2();
        let gyro_bias = state.gyro_bias.as_rad_per_s();

        write!(out, "{t:.4},{:?}", state.status)?;
        for value in [
            position.x,
            position.y,
            position.z,
            velocity.x,
            velocity.y,
            velocity.z,
            roll,
            pitch,
            yaw,
            accel_bias.x,
            accel_bias.y,
            accel_bias.z,
            gyro_bias.x,
            gyro_bias.y,
            gyro_bias.z,
        ] {
            write!(out, ",{value:.6}")?;
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

    fn report(&self, input: &Path, output: &Path) {
        println!("fusion-nav replay — no filtering is performed\n");
        println!("in   {}", input.display());
        println!("out  {}", output.display());

        match self.initialized_at {
            Some(t) => {
                let heading = if self.mag_at_init {
                    "magnetometer present, heading observed"
                } else {
                    "no magnetometer in the window, heading unobserved"
                };
                println!("\ninitialized at {t:.2} s from {WINDOW} static samples\n  {heading}");
            }
            None => println!("\nnever initialized: no stationary window of {WINDOW} samples"),
        }

        println!(
            "\n{} epochs written, {} measurements rejected",
            self.epochs, self.rejections
        );
        if self.long_steps > 0 {
            let (at, dt) = self.longest_step;
            println!(
                "{} steps longer than {LONG_STEP} s, worst {dt:.3} s at {at:.2} s\n  \
                 a gap is usually the logger missing messages, not the IMU stopping",
                self.long_steps
            );
        }

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
        }
    }
}

/// One parsed row. Values and variances are positional; which ones a source uses is
/// documented in the log header.
struct Record<'a> {
    t: f32,
    source: &'a str,
    values: [Option<f32>; 6],
    variances: [Option<f32>; 3],
}

impl<'a> Record<'a> {
    fn parse(line: &'a str) -> Option<Self> {
        let mut fields = line.split(',');
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
    for name in ESTIMATE {
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
