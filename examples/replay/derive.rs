//! `--derive`: work a `Config` out from one log, and print it as Rust.
//!
//! The offline half of `GOALS.md` differentiator 7: what the static window cannot measure,
//! derived from a replay log at a defined moment and printed for a reader to commit, never
//! applied in flight. Each value is printed with where it came from, and what the log could not
//! measure is printed at its default with the reason.
//!
//! In the harness rather than beside it in Python, for three reasons. The tool builds the
//! `Config` it prints, so it replays the log under that `Config` before printing it and says
//! what moved. [`render`](Derived::render) destructures `Config` without `..`, so a field added
//! to it does not compile until the tool prints it. And every figure is read off the same
//! statistics the `summary` line is built from, in the same process: `acf1_`'s series for `τ`,
//! the gate's own `ε` for the percentiles, the verdicts the fusion file writes for the runs of
//! rejections and the coasted gaps.
//!
//! Three layers. The runs ([`derive`]) replay the log under the configurations a reading needs.
//! The readings take a run or the rows and return numbers: what a *run* produced (innovations,
//! verdicts) is read off a [`Replay`], what the *measurements* hold by themselves (IMU
//! intervals, the barometer against GNSS height, the site) off the rows, and nothing else
//! computes those. The rules (`*_from`) turn readings into a value, and are what the tests
//! below hold to their evidence in `DESIGN.md`, "Defaults and their evidence".

use std::error::Error;
use std::io;
use std::path::Path;
use std::thread;

use fusion_nav::prelude::*;
use fusion_nav::{Seconds, Timestamp};

use super::settings::{self, PerSource};
use super::{
    GNSS_POS, GNSS_VEL, Options, Record, Replay, SOURCES, Sinks, Verdict, decided, drive,
    origin_of, prepare,
};

/// Where the sorted IMU intervals stop being the sensor's and start being dropouts: the first
/// step up by at least this factor. On the corpus, after each log's start, the widest step
/// among ordinary intervals and the narrowest to a dropout straddle it
/// ([evidence](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#max_predict_dt)).
const DROPOUT_STEP: f64 = 5.0;

/// The margin `max_predict_dt` keeps over the longest ordinary interval.
const INTERVAL_MARGIN: f64 = 1.1;

/// Shortest overlap of barometer and GNSS height a drift is read from, seconds: where the
/// estimate's scatter on simulated walks comes down to a fifth
/// ([evidence](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#baro_offset_walk)).
const DRIFT_SPAN: f64 = 1800.0;

/// The shortest lag the drift is read at, seconds, where ten of the log's GNSS-height `τ` do
/// not push it further. GNSS height's correlated error lifts `D(L)` to a plateau over a few of
/// its `τ`, which a slope read inside reports as walk. The lags run to four times the
/// shortest, past which they hold too few independent increments to read
/// ([evidence](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#baro_offset_walk)).
const DRIFT_LAG: f64 = 60.0;

/// How long after a gap the GNSS verdicts are read for its cost, seconds: past the 7 s
/// recovery timeout, so that a lockout the gap started has been adopted inside it.
const SETTLE: f64 = 10.0;

/// The `baro_offset_walk` values the rejections are counted at besides the derived one: zero,
/// the two the default's doc comment measured, and PX4's.
const WALKS: [f32; 4] = [0.0, 0.02, 0.05, 0.13];

/// Multiples of a `Coast::default()` field each gap is tried at, the other field held at its
/// default. Zero asks whether a gap needs that field's noise at all.
const COAST_SCALES: [f32; 7] = [0.0, 0.125, 0.25, 0.5, 1.0, 2.0, 4.0];

/// The name `Correlation`, `Recovery` and `Diagnostics` give each of [`SOURCES`], in that order:
/// `Diagnostics::sources()`'s, the list `SOURCES` is paired with everywhere in the harness.
fn field(source: usize) -> &'static str {
    Diagnostics::default().sources()[source].0
}

/// What `--derive` worked out from a log, and the evidence for each value.
pub struct Derived {
    name: String,
    config: Config,
    noise: Option<WindowNoise>,
    gates: [Exceedance; SOURCES.len()],
    runs: [Option<f64>; SOURCES.len()],
    recovered: [u32; SOURCES.len()],
    correlation: [Tau; SOURCES.len()],
    intervals: Option<Intervals>,
    site: Option<(Geodetic, f32)>,
    drift: Option<Drift>,
    walks: Vec<(f32, u32, u32)>,
    gaps: Vec<Gap>,
    check: Check,
}

/// One source's `ε` against the three bounds: the fraction over each, and the rows.
#[derive(Clone, Copy, Default)]
struct Exceedance {
    rows: usize,
    over: [f64; 3],
}

/// One source's `τ` as the white run read it.
#[derive(Clone, Copy, Debug)]
enum Tau {
    /// No two rows to correlate: the log does not carry the source.
    Absent,
    /// `|ρ|` inside `2/√n`, two standard errors of a lag-one autocorrelation of `n` independent
    /// rows: the log cannot tell the source from white, and the default stands.
    Unresolved { rho: f64, rows: usize },
    /// `ρ` below `−2/√n`: successive innovations alternate, which no `τ` describes, and the
    /// default stands.
    Alternating { rho: f64 },
    /// `τ = −T / ln ρ`.
    Measured { rho: f64, interval: f64, tau: f64 },
}

/// One step the filter takes after the start: the log times either side, and the interval as
/// [`Eskf::predict`] differences it, in whole microseconds.
#[derive(Clone, Copy)]
struct Span {
    start: f64,
    end: f64,
    dt: Seconds,
}

/// The IMU's intervals after the first epoch, sorted, and what they say.
struct Intervals {
    count: usize,
    median: f64,
    p999: f64,
    /// The longest interval below the first [`DROPOUT_STEP`].
    ordinary: f64,
    /// The widest step between neighboring sorted intervals among the ordinary ones, and the
    /// step from the longest ordinary one to the shortest dropout, where there is one: the two
    /// figures [`DROPOUT_STEP`] sits between.
    widest_step: f64,
    break_step: Option<f64>,
    /// Every interval past the ordinary ones.
    dropouts: Vec<Span>,
    spans: Vec<Span>,
}

impl Intervals {
    /// The steps a filter with this `max_predict_dt` coasts: the comparison `predict` makes.
    fn over(&self, limit: Seconds) -> Vec<Span> {
        self.spans
            .iter()
            .copied()
            .filter(|s| s.dt > limit)
            .collect()
    }
}

/// The barometer against GNSS height: `q_b` from the structure function's slope, and the
/// change from the first minute to the last.
struct Drift {
    span: f64,
    start_to_end: f64,
    /// The shortest lag read.
    shortest: f64,
    /// `√slope`, or `None` where the slope is not positive: no drift the log resolves.
    walk: Option<f64>,
}

/// One coasted gap, and for each `Coast` field the smallest multiple of its default, the other
/// field at its own, after which no GNSS position or velocity was rejected or adopted for
/// [`SETTLE`] seconds: the rule `Coast::default()` was set by on `4b473e91`, a field at a time
/// (`DESIGN.md`, `Coast`).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Gap {
    start: f64,
    end: f64,
    acceleration_at: Option<f32>,
    rotation_at: Option<f32>,
    /// No GNSS position was decided inside the window, so the gap says nothing about coasting.
    unaided: bool,
}

/// The derived `Config` replayed against the default: what moved.
struct Check {
    predicted_coasted: usize,
    coasted: [u32; 2],
    refused: [u32; 2],
    rejected: [u32; 2],
    recovered: [u32; 2],
    transitions: [usize; 2],
}

impl Check {
    fn holds(&self) -> bool {
        self.coasted[1] as usize == self.predicted_coasted
    }
}

/// Run `text` under `config` with no output files, keeping its verdicts.
///
/// Boxed, so that a derivation holding several finished replays holds pointers to them.
fn run(text: &str, config: Config, options: &Options) -> Result<Box<Replay>, String> {
    let mut replay = prepare(text, config, options, None).map_err(|e| e.to_string())?;
    replay.trail = Some(Vec::new());
    drive(
        &mut replay,
        text,
        Path::new("--derive"),
        &mut Sinks {
            epochs: &mut io::sink(),
            fusions: &mut io::sink(),
        },
    )
    .map_err(|e| e.to_string())?;
    Ok(Box::new(replay))
}

/// A thread's stack for one replay. A `Replay` carries its candidate window and a boxed copy of
/// itself while a start is pending, and in a debug build the frames that move them overflow
/// the 2 MB a spawned thread gets by default.
const STACK: usize = 64 << 20;

/// [`run`] under each of `configs`, at once, each on a thread of its own.
fn runs(text: &str, configs: Vec<Config>, options: &Options) -> Result<Vec<Box<Replay>>, String> {
    thread::scope(|scope| {
        let handles = configs
            .into_iter()
            .map(|config| {
                thread::Builder::new()
                    .stack_size(STACK)
                    .spawn_scoped(scope, move || run(text, config, options))
                    .map_err(|e| e.to_string())
            })
            .collect::<Result<Vec<_>, String>>()?;
        handles
            .into_iter()
            .map(|handle| handle.join().map_err(|_| "a replay panicked".to_string())?)
            .collect()
    })
}

/// Work out a `Config` from the log `text`, read from `input`.
pub fn derive(input: &Path, text: &str, options: &Options) -> Result<Derived, Box<dyn Error>> {
    let defaults = Config::default();
    let rows = Rows::of(text);
    let site = origin_of(text).map(|site| (site, site.normal_gravity()));
    let has_height = !rows.baro.is_empty() && !rows.gnss_down.is_empty();

    // First the runs that need nothing but the defaults: the baseline, every source white,
    // recovery off, and the offset walks.
    let mut walks: Vec<f32> = if has_height {
        WALKS.to_vec()
    } else {
        Vec::new()
    };
    let mut configs = vec![
        defaults,
        Config {
            correlation: Correlation::WHITE,
            ..defaults
        },
        Config {
            recovery: Recovery::OFF,
            ..defaults
        },
    ];
    configs.extend(walks.iter().map(|&walk| Config {
        baro_offset_walk: walk,
        ..defaults
    }));
    let mut first = runs(text, configs, options)?.into_iter();
    let (Some(baseline), Some(white), Some(off)) = (first.next(), first.next(), first.next())
    else {
        return Err("--derive: a run went missing".into());
    };
    let mut walk_rows: Vec<(f32, u32, u32)> = walks
        .iter()
        .zip(first)
        .map(|(&walk, replay)| walk_counts(walk, &replay))
        .collect();

    let started = baseline
        .initialized_at
        .ok_or("--derive: the log never initialized, so it has no run to derive from")?;
    let correlation = taus(&white);
    let derived_correlation = correlation_from(defaults.correlation, &correlation);
    let drift = drift(
        &rows.baro,
        &rows.gnss_down,
        shortest_lag(derived_correlation.gnss_height),
    );
    let derived_walk = walk_from(drift.as_ref());
    let intervals = Intervals::of(&rows.imu, started);
    let max_predict_dt = max_predict_dt_from(intervals.as_ref(), defaults.max_predict_dt);

    // Then the runs those readings ask for: the derived walk, where the first batch did not
    // run it, and each `Coast` field at each scale, where the derived limit coasts a gap.
    let mut gaps: Vec<Gap> = intervals
        .as_ref()
        .map_or_else(Vec::new, |i| i.over(max_predict_dt))
        .iter()
        .map(|span| Gap {
            start: span.start,
            end: span.end,
            acceleration_at: None,
            rotation_at: None,
            unaided: false,
        })
        .collect();
    let new_walk = derived_walk.filter(|walk| has_height && !walks.contains(walk));
    let coast = Coast::default();
    let scaled: Vec<(bool, f32)> = if gaps.is_empty() {
        Vec::new()
    } else {
        [false, true]
            .into_iter()
            .flat_map(|rotation| COAST_SCALES.iter().map(move |&scale| (rotation, scale)))
            .collect()
    };
    let mut configs: Vec<Config> = new_walk
        .map(|walk| Config {
            baro_offset_walk: walk,
            ..defaults
        })
        .into_iter()
        .collect();
    configs.extend(scaled.iter().map(|&(rotation, scale)| Config {
        max_predict_dt,
        coast: Some(if rotation {
            Coast {
                rotation: coast.rotation * scale,
                ..coast
            }
        } else {
            Coast {
                acceleration: coast.acceleration * scale,
                ..coast
            }
        }),
        ..defaults
    }));
    let mut second = runs(text, configs, options)?.into_iter();
    if let Some(walk) = new_walk {
        let replay = second.next().ok_or("--derive: a walk run went missing")?;
        walk_rows.push(walk_counts(walk, &replay));
        walks.push(walk);
    }
    for (&(rotation, scale), replay) in scaled.iter().zip(second) {
        let trail = replay.trail.as_deref().unwrap_or_default();
        let next: Vec<f64> = gaps.iter().skip(1).map(|g| g.start).collect();
        for (index, gap) in gaps.iter_mut().enumerate() {
            let until = next
                .get(index)
                .map_or(gap.end + SETTLE, |n| n.min(gap.end + SETTLE));
            let at = if rotation {
                &mut gap.rotation_at
            } else {
                &mut gap.acceleration_at
            };
            match settled(trail, gap.end, until) {
                None => gap.unaided = true,
                Some(true) if at.is_none() => *at = Some(scale),
                _ => {}
            }
        }
    }

    let config = Config {
        imu: imu_from(defaults.imu, baseline.noise.as_ref()),
        correlation: derived_correlation,
        gravity: site.map_or(defaults.gravity, |(_, gamma)| rounded(f64::from(gamma), 6)),
        max_predict_dt,
        coast: Some(coast_from(coast, &gaps)),
        baro_offset_walk: derived_walk.unwrap_or(defaults.baro_offset_walk),
        ..defaults
    };

    // The derived `Config` replayed: what moved against the baseline, and whether the steps
    // it coasts are the ones the intervals said it would.
    let checked = runs(text, vec![config], options)?
        .pop()
        .ok_or("--derive: the check run went missing")?;
    let predicted_coasted = intervals.as_ref().map_or(0, |i| {
        let started = checked.initialized_at.unwrap_or(started);
        i.over(max_predict_dt)
            .iter()
            .filter(|span| span.start >= started)
            .count()
    });
    let pair = |of: &dyn Fn(&Replay) -> u32| [of(&baseline), of(&checked)];
    let check = Check {
        predicted_coasted,
        coasted: pair(&|r| r.filter.diagnostics().propagation.coasted),
        refused: pair(&|r| {
            let p = &r.filter.diagnostics().propagation;
            p.refused_too_long
                + p.refused_invalid
                + p.refused_not_finite
                + p.refused_state_not_finite
        }),
        rejected: pair(&|r| r.rejections()),
        recovered: pair(&|r| r.total(|health| health.recovered)),
        transitions: [baseline.transitions.len(), checked.transitions.len()],
    };

    Ok(Derived {
        name: input.file_name().map_or_else(
            || input.display().to_string(),
            |n| n.to_string_lossy().into(),
        ),
        config,
        noise: baseline.noise,
        gates: exceedances(&baseline),
        runs: honest_runs(off.trail.as_deref().unwrap_or_default()),
        recovered: core::array::from_fn(|source| {
            baseline.filter.diagnostics().sources()[source].1.recovered
        }),
        correlation,
        intervals,
        site,
        drift,
        walks: walk_rows,
        gaps,
        check,
    })
}

/// GNSS height's and the barometer's rejections in a run at `walk`.
fn walk_counts(walk: f32, replay: &Replay) -> (f32, u32, u32) {
    let d = replay.filter.diagnostics();
    (walk, d.gnss_height.rejected, d.baro_altitude.rejected)
}

/// The white noise: the default, or the window's floor where it sits above it, since no `Q`
/// should claim a sensor quieter than it measured at rest.
fn imu_from(default: ImuNoise, floor: Option<&WindowNoise>) -> ImuNoise {
    floor.map_or(default, |floor| ImuNoise {
        gyro_white: default.gyro_white.max(floor.worst_gyro_white()),
        accel_white: default.accel_white.max(floor.worst_accel_white()),
        ..default
    })
}

/// Each source's `τ`: the reading where it exceeds the default, and the default otherwise.
///
/// A reading through the filter is a lower bound, so one above the default shows the default
/// too short and one below it shows nothing; read as the value it made the `correlated`
/// scenario more overconfident than the defaults do
/// ([measured](https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#correlation)). An
/// estimator the filter does not bias is #195.
fn correlation_from(default: Correlation, taus: &[Tau; SOURCES.len()]) -> Correlation {
    let mut derived = default;
    for (source, tau) in taus.iter().enumerate() {
        let Some(slot) = derived.field_mut(field(source)) else {
            continue;
        };
        if let Tau::Measured { tau, .. } = *tau {
            let read = Seconds::from_secs(rounded(tau, 2));
            if slot.is_some_and(|default| read > default) {
                *slot = Some(read);
            }
        }
    }
    derived
}

/// The longest ordinary interval with [`INTERVAL_MARGIN`], never below the default: a lower
/// limit catches no dropout on the corpus that the default misses, and the same bound limits
/// how far a measurement is carried forward.
fn max_predict_dt_from(intervals: Option<&Intervals>, default: Seconds) -> Seconds {
    intervals.map_or(default, |i| {
        let needed = (i.ordinary * INTERVAL_MARGIN / 0.005).ceil() * 0.005;
        Seconds::from_secs((needed as f32).max(default.as_secs()))
    })
}

/// Each `Coast` field at twice the largest multiple of its default that a gap with GNSS after it
/// needed: the default where a gap never settled at any multiple, where none had GNSS after it,
/// or where every gap settled with none of that field's noise.
fn coast_from(default: Coast, gaps: &[Gap]) -> Coast {
    let aided: Vec<&Gap> = gaps.iter().filter(|g| !g.unaided).collect();
    let scale = |at: fn(&Gap) -> Option<f32>| -> Option<f32> {
        if aided.is_empty() || aided.iter().any(|g| at(g).is_none()) {
            return None;
        }
        let largest = aided.iter().filter_map(|g| at(g)).fold(0.0, f32::max);
        (largest > 0.0).then_some(2.0 * largest)
    };
    Coast {
        acceleration: scale(|g| g.acceleration_at).map_or(default.acceleration, |s| {
            rounded(f64::from(default.acceleration * s), 2)
        }),
        rotation: scale(|g| g.rotation_at).map_or(default.rotation, |s| {
            rounded(f64::from(default.rotation * s), 2)
        }),
    }
}

/// `q`, where the drift resolves one.
fn walk_from(drift: Option<&Drift>) -> Option<f32> {
    drift
        .and_then(|d| d.walk)
        .map(|walk| rounded(walk, 2))
        .filter(|walk| *walk > 0.0)
}

/// The drift's shortest lag: [`DRIFT_LAG`], or ten of GNSS height's `τ` as the derived `Config`
/// holds it, where that is longer. That `τ` is a lower bound; where the lags still sit inside
/// the plateau GNSS height's error makes, `q` reads high, and it is an upper bound either way.
fn shortest_lag(gnss_height: Option<Seconds>) -> f64 {
    gnss_height.map_or(DRIFT_LAG, |tau| {
        DRIFT_LAG.max((10.0 * f64::from(tau.as_secs())).ceil())
    })
}

/// Whether GNSS settled over `[from, until)`: a position was accepted and no position or
/// velocity was rejected or adopted. `None` where no position was decided at all.
fn settled(trail: &[Verdict], from: f64, until: f64) -> Option<bool> {
    let window: Vec<&Verdict> = trail
        .iter()
        .filter(|v| v.t >= from && v.t < until)
        .filter(|v| v.source == GNSS_POS || v.source == GNSS_VEL)
        .filter(|v| decided(v.outcome))
        .collect();
    if !window.iter().any(|v| v.source == GNSS_POS) {
        return None;
    }
    Some(
        window
            .iter()
            .all(|v| matches!(v.outcome, Fusion::Accepted { .. })),
    )
}

/// Each source's `ε` against the 95, 99 and 99.9 % bounds, as the baseline gated it.
fn exceedances(baseline: &Replay) -> [Exceedance; SOURCES.len()] {
    core::array::from_fn(|source| {
        let c = &baseline.consistency;
        Exceedance {
            rows: c.rows(source),
            over: core::array::from_fn(|which| c.over_fraction(source, which).unwrap_or(0.0)),
        }
    })
}

/// `τ = −T / ln ρ` per source, from the run that fused every source white: with (24′) on, the
/// gain moves `ρ` and the reading would describe the filter rather than the sensor.
fn taus(white: &Replay) -> [Tau; SOURCES.len()] {
    core::array::from_fn(|source| {
        let c = &white.consistency;
        let rows = c.rows(source);
        let resolved = 2.0 / (rows as f64).sqrt();
        match (c.autocorrelation(source), c.interval(source)) {
            (Some(rho), Some(interval)) if rho > resolved && rho < 1.0 && interval > 0.0 => {
                Tau::Measured {
                    rho,
                    interval,
                    tau: -interval / libm::log(rho),
                }
            }
            (Some(rho), Some(_)) if rho < -resolved => Tau::Alternating { rho },
            (Some(rho), Some(_)) => Tau::Unresolved { rho, rows },
            _ => Tau::Absent,
        }
    })
}

/// Per source, the longest run from a rejection to the next acceptance, seconds: how long an
/// honest disagreement lasted. Read with recovery off, so that no run is cut short by an
/// adoption; a run the log ends inside is not one that resolved, and is left out.
fn honest_runs(trail: &[Verdict]) -> [Option<f64>; SOURCES.len()] {
    let mut longest = [None; SOURCES.len()];
    let mut since: [Option<f64>; SOURCES.len()] = [None; SOURCES.len()];
    for verdict in trail {
        let source = verdict.source;
        match verdict.outcome {
            Fusion::Rejected { .. } => {
                since[source].get_or_insert(verdict.t);
            }
            Fusion::Accepted { .. } => {
                if let Some(start) = since[source].take() {
                    let run = verdict.t - start;
                    longest[source] = Some(longest[source].map_or(run, |l: f64| l.max(run)));
                }
            }
            Fusion::Reset => since[source] = None,
            _ => {}
        }
    }
    longest
}

/// The measurements a derivation reads directly.
struct Rows {
    /// IMU row times.
    imu: Vec<f64>,
    /// Barometric altitude, up, m.
    baro: Vec<(f64, f64)>,
    /// GNSS down, m.
    gnss_down: Vec<(f64, f64)>,
}

impl Rows {
    fn of(text: &str) -> Self {
        let mut rows = Self {
            imu: Vec::new(),
            baro: Vec::new(),
            gnss_down: Vec::new(),
        };
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') || line.starts_with("t_s") {
                continue;
            }
            let Some(r) = Record::parse(line) else {
                continue;
            };
            match (r.source, r.values[0], r.values[2]) {
                ("imu", _, _) => rows.imu.push(r.t),
                ("baro", Some(altitude), _) => rows.baro.push((r.t, f64::from(altitude))),
                ("gnss_pos", _, Some(down)) => rows.gnss_down.push((r.t, f64::from(down))),
                _ => {}
            }
        }
        rows
    }
}

impl Intervals {
    /// The intervals between successive IMU rows from `started` on, as [`Eskf::predict`] steps
    /// them: each time rounded to its microsecond, and a row not after the last one refused
    /// without moving the clock, so that it opens no interval.
    fn of(imu: &[f64], started: f64) -> Option<Self> {
        let mut spans = Vec::new();
        let mut previous: Option<(f64, Timestamp)> = None;
        for &t in imu {
            let time = Timestamp::from_secs_f64(t);
            match previous {
                Some((_, clock)) if time <= clock => continue,
                Some((p, clock)) if p >= started => spans.push(Span {
                    start: p,
                    end: t,
                    dt: time.since(clock),
                }),
                _ => {}
            }
            previous = Some((t, time));
        }
        let mut sorted: Vec<f64> = spans.iter().map(|s| f64::from(s.dt.as_secs())).collect();
        if sorted.is_empty() {
            return None;
        }
        sorted.sort_by(f64::total_cmp);
        let n = sorted.len();
        let mut i = n / 2;
        let mut widest_step: f64 = 1.0;
        while i + 1 < n && sorted[i + 1] < DROPOUT_STEP * sorted[i] {
            widest_step = widest_step.max(sorted[i + 1] / sorted[i]);
            i += 1;
        }
        let ordinary = sorted[i];
        Some(Self {
            count: n,
            median: sorted[n / 2],
            p999: sorted[((n as f64 * 0.999) as usize).min(n - 1)],
            ordinary,
            widest_step,
            break_step: sorted.get(i + 1).map(|next| next / ordinary),
            dropouts: spans
                .iter()
                .copied()
                .filter(|s| f64::from(s.dt.as_secs()) > ordinary)
                .collect(),
            spans,
        })
    }

    /// The dropouts a filter at `limit` integrates as one step that one at `needed` would
    /// coast.
    fn integrated_between(&self, needed: Seconds, limit: Seconds) -> usize {
        self.dropouts
            .iter()
            .filter(|s| s.dt > needed && s.dt <= limit)
            .count()
    }
}

/// The barometer's drift against GNSS height, as a random walk: `D(L) = c + q² L`, the mean
/// squared change of `baro + down` over a lag `L`, fitted over eight lags from `shortest` to
/// four times it, which has to fit in half the overlap. A walk's structure function grows linearly in the lag, and the constant takes
/// both sensors' white noise. GNSS height's own slow wander is in the slope as well, so `q` is
/// an upper bound on the barometer's.
fn drift(baro: &[(f64, f64)], gnss_down: &[(f64, f64)], shortest: f64) -> Option<Drift> {
    drift_over(baro, gnss_down, shortest, DRIFT_SPAN)
}

/// [`drift`] on an overlap of at least `least` seconds.
fn drift_over(
    baro: &[(f64, f64)],
    gnss_down: &[(f64, f64)],
    shortest: f64,
    least: f64,
) -> Option<Drift> {
    // `baro + down` at each fix, the barometer interpolated between readings at most 1 s apart.
    let mut difference = Vec::new();
    for &(t, down) in gnss_down {
        let after = baro.partition_point(|(tb, _)| *tb < t);
        let (Some(&(t0, a0)), Some(&(t1, a1))) =
            (after.checked_sub(1).map(|i| &baro[i]), baro.get(after))
        else {
            continue;
        };
        if t1 - t0 > 1.0 {
            continue;
        }
        let altitude = if t1 > t0 {
            a0 + (a1 - a0) * (t - t0) / (t1 - t0)
        } else {
            a0
        };
        difference.push((t, altitude + down));
    }
    let (&(start, _), &(end, _)) = (difference.first()?, difference.last()?);
    let span = end - start;
    if span < least {
        return None;
    }
    // On a 1 s grid, linear between fixes at most 5 s apart and a hole elsewhere.
    let cells = span as usize + 1;
    let mut grid = vec![f64::NAN; cells];
    for pair in difference.windows(2) {
        let ((t0, d0), (t1, d1)) = (pair[0], pair[1]);
        if t1 - t0 > 5.0 || t1 <= t0 {
            continue;
        }
        let from = (t0 - start).ceil() as usize;
        let to = ((t1 - start).floor() as usize).min(cells - 1);
        for (cell, value) in grid.iter_mut().enumerate().take(to + 1).skip(from) {
            let t = start + cell as f64;
            *value = d0 + (d1 - d0) * (t - t0) / (t1 - t0);
        }
    }
    let mean_over = |cells: &[f64]| {
        let kept: Vec<f64> = cells.iter().copied().filter(|v| !v.is_nan()).collect();
        (!kept.is_empty()).then(|| kept.iter().sum::<f64>() / kept.len() as f64)
    };
    let minute = 60;
    let longest = 4.0 * shortest;
    if longest > span / 2.0 {
        return None;
    }
    let start_to_end = mean_over(&grid[cells - minute..])? - mean_over(&grid[..minute])?;

    // Eight lags, geometric from `shortest` to four times it, each a whole second.
    let lags: Vec<usize> = (0..8)
        // `libm` rather than `powf`, so the lags do not depend on the host's transcendentals.
        .map(|k| (shortest * libm::pow(longest / shortest, f64::from(k) / 7.0)).round() as usize)
        .collect();
    let mut points = Vec::new();
    for &lag in &lags {
        let (sum, count) = grid
            .iter()
            .zip(&grid[lag..])
            .filter(|(a, b)| !a.is_nan() && !b.is_nan())
            .fold((0.0, 0usize), |(sum, count), (a, b)| {
                (sum + (b - a) * (b - a), count + 1)
            });
        if count > 0 {
            points.push((lag as f64, sum / count as f64));
        }
    }
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let (mean_l, mean_d) = (
        points.iter().map(|p| p.0).sum::<f64>() / n,
        points.iter().map(|p| p.1).sum::<f64>() / n,
    );
    let slope = points
        .iter()
        .map(|(l, d)| (l - mean_l) * (d - mean_d))
        .sum::<f64>()
        / points
            .iter()
            .map(|(l, _)| (l - mean_l) * (l - mean_l))
            .sum::<f64>();
    Some(Drift {
        span,
        start_to_end,
        shortest,
        walk: (slope > 0.0).then(|| slope.sqrt()),
    })
}

/// `x` to `significant` figures, as the `f32` a literal of that text parses to: the `Config`
/// that is replayed is the one printed, digit for digit.
fn rounded(x: f64, significant: usize) -> f32 {
    format!("{:.*e}", significant.saturating_sub(1), x)
        .parse()
        .unwrap_or(x as f32)
}

/// An `f32` as a Rust literal: `{:?}` keeps the decimal point a whole number needs.
fn literal(x: f32) -> String {
    format!("{x:?}")
}

fn seconds(value: Option<Seconds>) -> String {
    value.map_or("None".into(), |s| {
        format!(
            "Some(fusion_nav::Seconds::from_secs({}))",
            literal(s.as_secs())
        )
    })
}

fn percent(x: f64) -> String {
    format!("{:.1} %", x * 100.0)
}

impl Derived {
    /// Whether the derived `Config` coasted exactly the steps its `max_predict_dt` predicts.
    pub fn holds(&self) -> bool {
        self.check.holds()
    }

    /// The `Config` as a Rust expression, every value commented with where it came from.
    pub fn render(&self) -> String {
        // Without `..`: a field added to `Config` does not compile until it is printed.
        let Config {
            imu:
                ImuNoise {
                    gyro_white,
                    accel_white,
                    gyro_bias_walk,
                    accel_bias_walk,
                },
            gates,
            timeouts,
            recovery,
            correlation,
            init,
            accuracy,
            gravity,
            max_predict_dt,
            coast,
            baro_offset_walk,
            baro_reference_from_estimate,
        } = self.config;
        let defaults = Config::default();
        let mut out = String::new();
        let mut line = |text: String| {
            out.push_str(&text);
            out.push('\n');
        };
        line(format!(
            "// A Config derived from `{}` by `cargo run --example replay -- --derive`.",
            self.name
        ));
        line("// Each value says where it came from; the run's evidence is on stderr.".into());
        let arguments = settings::arguments(&self.config);
        line(format!(
            "// As --set: {}",
            if arguments.is_empty() {
                "nothing; every value is the default".into()
            } else {
                arguments.join(" ")
            }
        ));
        line("fusion_nav::Config {".into());

        line("    imu: fusion_nav::ImuNoise {".into());
        let floor = |field: &str, value: f32, default: f32, floor: Option<f32>| match floor {
            Some(floor) if value > default => format!(
                "        // The window's floor, {floor:.3e}, above the default {}: no `Q` below what the sensor measured at rest.\n        {field}: {},",
                literal(default),
                literal(value)
            ),
            Some(floor) => format!(
                "        // The default; the window measured a floor of {floor:.3e}, {:.0}x under it.\n        {field}: {},",
                default / floor,
                literal(value)
            ),
            None => format!(
                "        // The default; the window moved or held too few readings to measure a floor.\n        {field}: {},",
                literal(value)
            ),
        };
        line(floor(
            "gyro_white",
            gyro_white,
            defaults.imu.gyro_white,
            self.noise.map(|n| n.worst_gyro_white()),
        ));
        line(floor(
            "accel_white",
            accel_white,
            defaults.imu.accel_white,
            self.noise.map(|n| n.worst_accel_white()),
        ));
        line("        // Bias walks: the default. Not measured: an Allan variance needs a soak of hours.".into());
        line(format!(
            "        gyro_bias_walk: {},",
            literal(gyro_bias_walk)
        ));
        line(format!(
            "        accel_bias_walk: {},",
            literal(accel_bias_walk)
        ));
        line("    },".into());
        if let Some(sigma) = self.noise.and_then(|n| n.baro).map(|b| b.variance().sqrt()) {
            line(format!(
                "    // Barometer `R` is per call to fuse_baro_altitude, not Config: the window's σ, {sigma:.3} m, is its floor."
            ));
        }

        line(
            "    // Gates: the default percentile. ε over the 95 / 99 / 99.9 % bounds, against 5 / 1 / 0.1:"
                .into(),
        );
        for (source, exceedance) in SOURCES.iter().zip(&self.gates) {
            if exceedance.rows > 0 {
                line(format!(
                    "    //   {source}: {} / {} / {} of {}",
                    percent(exceedance.over[0]),
                    percent(exceedance.over[1]),
                    percent(exceedance.over[2]),
                    exceedance.rows
                ));
            }
        }
        // What a log cannot derive is printed by name rather than value, so it has to be what
        // the name says.
        assert_eq!(
            gates,
            Gates::at(Percentile::P999),
            "--derive keeps the gates"
        );
        assert_eq!(timeouts, defaults.timeouts, "--derive keeps the timeouts");
        assert_eq!(init, defaults.init, "--derive keeps the initialization");
        assert_eq!(accuracy, defaults.accuracy, "--derive keeps the accuracy");
        line("    gates: fusion_nav::Gates::at(fusion_nav::Percentile::P999),".into());
        line("    timeouts: fusion_nav::Timeouts::default(), // the mission's".into());

        line("    recovery: fusion_nav::Recovery {".into());
        line("        // The defaults, PX4's; beside each, the longest rejection run that ended in an acceptance with recovery off, and recoveries at the default.".into());
        for (source, (name, value)) in recovery.fields().into_iter().enumerate() {
            debug_assert_eq!(name, field(source));
            line(format!(
                "        {name}: {}, // {}; recovered {}",
                seconds(value),
                self.runs[source].map_or("no rejection resolved".into(), |run| format!(
                    "longest honest run {run:.1} s"
                )),
                self.recovered[source]
            ));
        }
        line("    },".into());

        line("    correlation: fusion_nav::Correlation {".into());
        line("        // τ = −T / ln ρ from each source's lag-one autocorrelation fused white, a lower bound since an innovation is whiter than the error behind it: printed only where it exceeds the default.".into());
        let defaults_correlation = defaults.correlation.fields().map(|(_, value)| value);
        for (source, (name, value)) in correlation.fields().into_iter().enumerate() {
            debug_assert_eq!(name, field(source));
            let why = match self.correlation[source] {
                Tau::Absent => "not in this log: the default".to_string(),
                Tau::Unresolved { rho, rows } => {
                    format!("ρ {rho:.3} over {rows} rows, white within 2/√n: the default")
                }
                Tau::Alternating { rho } => format!("ρ {rho:.3}: alternates, which no τ describes"),
                Tau::Measured { rho, interval, tau } => {
                    let default = defaults_correlation[source];
                    if default.is_some_and(|d| value != Some(d)) {
                        format!("ρ {rho:.3} at {interval:.3} s, τ ≥ {tau:.2} s")
                    } else {
                        format!(
                            "ρ {rho:.3} at {interval:.3} s, τ ≥ {tau:.2} s: under the default, which stands"
                        )
                    }
                }
            };
            line(format!("        {name}: {}, // {why}", seconds(value)));
        }
        line("    },".into());

        line(
            "    init: fusion_nav::Initialization::default(), // the start's tolerances and priors"
                .into(),
        );
        line("    accuracy: fusion_nav::Accuracy::default(), // the mission's".into());

        line(match self.site {
            Some((site, _)) => format!(
                "    // WGS-84 normal gravity at the log's origin, {:.4}° {:.4}° {:.0} m.",
                site.latitude_deg(),
                site.longitude_deg(),
                site.height()
            ),
            None => "    // The default: the log names no origin.".into(),
        });
        line(format!("    gravity: {},", literal(gravity)));

        match &self.intervals {
            Some(i) => {
                line(format!(
                    "    // IMU interval after the start: median {:.1} ms, 99.9 % {:.1} ms, longest ordinary {:.1} ms; widest step among ordinary intervals {:.1}x{}; {} dropouts of {} intervals{}.",
                    i.median * 1e3,
                    i.p999 * 1e3,
                    i.ordinary * 1e3,
                    i.widest_step,
                    i.break_step
                        .map_or(String::new(), |b| format!(", to the first dropout {b:.1}x")),
                    i.dropouts.len(),
                    i.count,
                    i.dropouts
                        .iter()
                        .map(|s| s.dt.as_secs())
                        .reduce(f32::max)
                        .map_or(String::new(), |worst| format!(", longest {worst:.3} s"))
                ));
                let needed = max_predict_dt_from(Some(i), Seconds::from_secs(0.0));
                if max_predict_dt == defaults.max_predict_dt {
                    let between = i.integrated_between(needed, max_predict_dt);
                    line(if between == 0 {
                        format!(
                            "    // The default: {} s with margin is under it, and no dropout falls between the two.",
                            literal(needed.as_secs())
                        )
                    } else {
                        format!(
                            "    // The default: {between} dropouts between {} s, the ordinary with margin, and it are integrated as one step each.",
                            literal(needed.as_secs())
                        )
                    });
                }
            }
            None => line("    // The default: no IMU interval after the start.".into()),
        }
        line(format!(
            "    max_predict_dt: fusion_nav::Seconds::from_secs({}),",
            literal(max_predict_dt.as_secs())
        ));

        line(if self.gaps.is_empty() {
            "    // The default: the log has no gap to coast.".into()
        } else {
            let at = |at: Option<f32>| at.map_or("never".to_string(), |s| format!("×{s}"));
            let needed: Vec<String> = self
                .gaps
                .iter()
                .map(|g| {
                    let what = if g.unaided {
                        "no GNSS after it".to_string()
                    } else {
                        format!(
                            "acceleration {}, rotation {}",
                            at(g.acceleration_at),
                            at(g.rotation_at)
                        )
                    };
                    format!("{:.2} s at {:.1} s: {what}", g.end - g.start, g.start)
                })
                .collect();
            format!(
                "    // Twice the largest multiple of each default, the other at its own, after which GNSS settles for {SETTLE} s past a gap; the default where none is needed or one never settles. Per gap: {}.",
                needed.join("; ")
            )
        });
        line(format!(
            "    coast: {},",
            match coast {
                None => "None".to_string(),
                Some(Coast {
                    acceleration,
                    rotation,
                }) => format!(
                    "Some(fusion_nav::Coast {{ acceleration: {}, rotation: {} }})",
                    literal(acceleration),
                    literal(rotation)
                ),
            }
        ));

        line(match &self.drift {
            Some(Drift {
                span,
                start_to_end,
                shortest,
                walk: Some(walk),
            }) => format!(
                "    // Barometer against GNSS height over {span:.0} s: {start_to_end:+.2} m first minute to last; q = {walk:.3} at lags from {shortest:.0} s, an upper bound."
            ),
            Some(Drift {
                span,
                start_to_end,
                walk: None,
                ..
            }) => format!(
                "    // The default: over {span:.0} s the barometer moved {start_to_end:+.2} m against GNSS height, and no walk resolves."
            ),
            None => format!(
                "    // The default: under {DRIFT_SPAN:.0} s of barometer beside GNSS height, too short to read a drift."
            ),
        });
        if !self.walks.is_empty() {
            let counts: Vec<String> = self
                .walks
                .iter()
                .map(|(walk, height, baro)| format!("{walk}: {height}/{baro}"))
                .collect();
            line(format!(
                "    // GNSS height / barometer rejections at each walk: {}.",
                counts.join(", ")
            ));
        }
        line(format!(
            "    baro_offset_walk: {},",
            literal(baro_offset_walk)
        ));
        line(format!(
            "    baro_reference_from_estimate: {baro_reference_from_estimate}, // policy"
        ));
        line("}".into());
        out
    }

    /// What the derived `Config` did to the log, beside the default's run.
    pub fn report(&self) -> String {
        let c = &self.check;
        let mut out = format!(
            "derive {}\n  replayed under the derived Config, default -> derived:\n",
            self.name
        );
        for (name, [before, after]) in [
            ("coasted", c.coasted),
            ("refused", c.refused),
            ("rejected", c.rejected),
            ("recovered", c.recovered),
        ] {
            out.push_str(&format!("    {name:<12}{before:>8} -> {after}\n"));
        }
        out.push_str(&format!(
            "    {:<12}{:>8} -> {}\n",
            "transitions", c.transitions[0], c.transitions[1]
        ));
        out.push_str(&format!(
            "  coasted {} steps where its max_predict_dt predicts {}: {}\n",
            c.coasted[1],
            c.predicted_coasted,
            if c.holds() { "ok" } else { "MISMATCH" }
        ));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BARO, GNSS_HGT};

    #[test]
    fn the_ordinary_intervals_end_at_the_first_fivefold_step() {
        // Ordinary 5 ms with a 14.8 ms straggler (3x, kept), then a 340 ms dropout.
        let mut t = vec![0.0];
        for _ in 0..100 {
            t.push(t.last().unwrap() + 0.005);
        }
        t.push(t.last().unwrap() + 0.0148);
        t.push(t.last().unwrap() + 0.340);
        t.push(t.last().unwrap() + 0.005);
        let i = Intervals::of(&t, 0.0).expect("intervals");
        assert!((i.ordinary - 0.0148).abs() < 1e-9, "{}", i.ordinary);
        assert_eq!(i.dropouts.len(), 1);
        assert_eq!(i.over(Seconds::from_secs(0.1)).len(), 1);
        assert!((i.widest_step - 0.0148 / 0.005).abs() < 1e-6);
        assert!((i.break_step.expect("a dropout") - 0.340 / 0.0148).abs() < 1e-6);
    }

    #[test]
    fn intervals_start_at_the_first_epoch_and_skip_a_repeated_time() {
        // A 1 s gap before the start is the window's, and a repeated row opens no interval, nor
        // does one 0.4 µs later, which `predict` reads as the same microsecond.
        let t = [0.0, 1.0, 1.01, 1.01, 1.010_000_4, 1.02, 1.03];
        let i = Intervals::of(&t, 1.0).expect("intervals");
        assert_eq!(i.count, 3);
        assert!(i.over(Seconds::from_secs(0.1)).is_empty());
    }

    #[test]
    fn a_walking_barometer_reads_back_its_density() {
        // The barometer walks at 0.1 m/√s (a deterministic sequence with that increment
        // variance), GNSS holds still, both at 1 Hz for 2 h. The structure function's slope is
        // q² = 0.01 m²/s.
        let mut level = 0.0;
        let mut baro = Vec::new();
        let mut gnss = Vec::new();
        let mut state: u64 = 12345;
        for second in 0..7200 {
            let t = f64::from(second);
            // ±0.1 m steps, a fair coin from a linear congruential generator.
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            level += if state >> 63 == 0 { 0.1 } else { -0.1 };
            baro.push((t, level));
            gnss.push((t + 0.5, 0.0));
        }
        let walk = drift_walk(&baro, &gnss).expect("a slope");
        assert!((walk - 0.1).abs() < 0.03, "{walk}");
        // A constant barometer has nothing to walk.
        let still: Vec<(f64, f64)> = baro.iter().map(|&(t, _)| (t, 3.0)).collect();
        assert_eq!(drift_walk(&still, &gnss), None);
        // Half an hour is the least it reads.
        assert!(drift(&baro[..1700], &gnss[..1700], DRIFT_LAG).is_none());
    }

    fn drift_walk(baro: &[(f64, f64)], gnss: &[(f64, f64)]) -> Option<f64> {
        drift(baro, gnss, DRIFT_LAG).and_then(|d| d.walk)
    }

    #[test]
    fn a_rejection_run_counts_only_when_an_acceptance_ends_it() {
        let accepted = Fusion::Accepted { test_ratio: 0.1 };
        let rejected = Fusion::Rejected { test_ratio: 2.0 };
        let at = |t, source, outcome| Verdict { t, source, outcome };
        let trail = [
            at(1.0, BARO, rejected),
            at(2.0, BARO, rejected),
            at(4.5, BARO, accepted),
            // An adoption ends a run without resolving it: kept open, this one would read 5.0.
            at(5.0, BARO, rejected),
            at(5.5, BARO, Fusion::Reset),
            at(10.0, BARO, accepted),
            at(7.0, GNSS_POS, rejected),
        ];
        let runs = honest_runs(&trail);
        assert_eq!(runs[BARO], Some(3.5));
        assert_eq!(
            runs[GNSS_POS], None,
            "a run the log ends inside did not resolve"
        );
    }

    /// `data/flight.config.rs` is what `--derive data/flight.csv` prints, and it is Rust that
    /// builds a `Config` the filter accepts. The second half is what keeps the printer honest
    /// as `Config` changes; the first, that the committed file is the current printer's.
    #[test]
    fn the_committed_derivation_is_the_printers_and_compiles() {
        let config: Config = include!("../../data/flight.config.rs");
        assert!(Eskf::new(config).is_ok());
        let text = include_str!("../../data/flight.csv");
        let derived =
            derive(Path::new("flight.csv"), text, &Options::default()).expect("flight.csv derives");
        assert_eq!(
            derived.config, config,
            "the printed Config is the one derived"
        );
        assert!(derived.holds());
        assert_eq!(
            derived.render(),
            include_str!("../../data/flight.config.rs"),
            "regenerate: cargo run --example replay -- --derive data/flight.csv > data/flight.config.rs"
        );
    }

    #[test]
    fn gnss_settles_only_when_nothing_in_the_window_was_turned_down() {
        let accepted = Fusion::Accepted { test_ratio: 0.1 };
        let rejected = Fusion::Rejected { test_ratio: 2.0 };
        let at = |t, source, outcome| Verdict { t, source, outcome };
        let trail = [
            at(1.0, GNSS_POS, accepted),
            at(2.0, crate::GNSS_VEL, rejected),
            at(3.0, BARO, rejected),
            at(12.0, GNSS_POS, accepted),
        ];
        // A velocity rejected inside the window unsettles it; a barometer does not.
        assert_eq!(settled(&trail, 0.0, 10.0), Some(false));
        assert_eq!(settled(&trail, 2.5, 13.0), Some(true));
        // After the window's end, and with no position inside it, it says nothing.
        assert_eq!(settled(&trail, 4.0, 11.0), None);
        // A refusal is not a verdict on the coast.
        let refused = [at(
            1.0,
            GNSS_POS,
            Fusion::OutOfHorizon {
                age: Seconds::from_secs(0.4),
            },
        )];
        assert_eq!(settled(&refused, 0.0, 10.0), None);
    }

    #[test]
    fn the_per_source_names_are_the_diagnostics_order() {
        // `render` pairs `correlation.fields()` and `recovery.fields()` with `SOURCES` by
        // position; this is what makes the position mean the source.
        let names: Vec<&str> = (0..SOURCES.len()).map(field).collect();
        let of = |fields: [(&'static str, Option<Seconds>); 7]| fields.map(|(n, _)| n).to_vec();
        assert_eq!(of(Correlation::default().fields()), names);
        assert_eq!(of(Recovery::default().fields()), names);
    }

    #[test]
    fn a_tau_read_through_the_filter_only_raises_the_default() {
        let read = |tau| Tau::Measured {
            rho: 0.9,
            interval: 0.2,
            tau,
        };
        let mut taus = [Tau::Absent; SOURCES.len()];
        taus[GNSS_POS] = read(11.0); // over the default 8.5: taken
        taus[GNSS_HGT] = read(30.0); // under the default 37: shows nothing
        taus[GNSS_VEL] = Tau::Alternating { rho: -0.4 };
        taus[BARO] = Tau::Unresolved { rho: 0.1, rows: 20 };
        let defaults = Correlation::default();
        let derived = correlation_from(defaults, &taus);
        assert_eq!(derived.gnss_position, Some(Seconds::from_secs(11.0)));
        assert_eq!(derived.gnss_height, defaults.gnss_height);
        assert_eq!(derived.gnss_velocity, defaults.gnss_velocity);
        assert_eq!(derived.baro_altitude, defaults.baro_altitude);
        assert_eq!(derived.mag_heading, defaults.mag_heading);
    }

    #[test]
    fn each_coast_field_is_twice_the_most_any_gap_needed_of_it() {
        let gap = |acceleration_at, rotation_at, unaided| Gap {
            start: 0.0,
            end: 1.0,
            acceleration_at,
            rotation_at,
            unaided,
        };
        let default = Coast::default();
        let derived = coast_from(
            default,
            &[
                gap(Some(0.25), Some(0.5), false),
                gap(Some(0.125), Some(0.0), false),
                // No GNSS after it: says nothing, whatever it read.
                gap(Some(4.0), None, true),
            ],
        );
        assert_eq!(derived.acceleration, 1.0, "2 × 0.25 × 2.0");
        assert_eq!(derived.rotation, 0.1, "2 × 0.5 × 0.1");
        // A gap that never settled leaves that field at the default; one that needed none of
        // it, too.
        let never = coast_from(default, &[gap(None, Some(0.0), false)]);
        assert_eq!(never, default);
        // One gap settling is not the log settling.
        let one_of_two = coast_from(
            default,
            &[gap(Some(0.25), Some(0.5), false), gap(None, None, false)],
        );
        assert_eq!(one_of_two, default);
        assert_eq!(coast_from(default, &[]), default);
    }

    #[test]
    fn max_predict_dt_covers_the_ordinary_tail_and_never_falls_below_the_default() {
        // A straggler under five times the period, so ordinary.
        let at = |period_ms: f64, ordinary_ms: f64| {
            let mut t = vec![0.0];
            for _ in 0..100 {
                t.push(t.last().unwrap() + period_ms / 1000.0);
            }
            t.push(t.last().unwrap() + ordinary_ms / 1000.0);
            t.push(t.last().unwrap() + period_ms / 1000.0);
            Intervals::of(&t, 0.0).expect("intervals")
        };
        let default = Seconds::from_secs(0.1);
        // 18.0 ms with 10 % margin is 19.8, rounded up to 20: under the default.
        assert_eq!(max_predict_dt_from(Some(&at(4.0, 18.0)), default), default);
        // 92.0 ms is 101.2, rounded up to 105.
        let raised = max_predict_dt_from(Some(&at(20.0, 92.0)), default).as_secs();
        assert!((raised - 0.105).abs() < 1e-6, "{raised}");
    }

    #[test]
    fn the_dropouts_a_limit_integrates_are_counted() {
        // 10 ms ordinary, a 60 ms stall: a limit derived at 15 ms would coast it, 100 ms does
        // not.
        let mut t = vec![0.0];
        for _ in 0..100 {
            t.push(t.last().unwrap() + 0.010);
        }
        t.push(t.last().unwrap() + 0.060);
        t.push(t.last().unwrap() + 0.010);
        let i = Intervals::of(&t, 0.0).expect("intervals");
        assert_eq!(
            i.integrated_between(Seconds::from_secs(0.015), Seconds::from_secs(0.1)),
            1
        );
        assert_eq!(
            i.integrated_between(Seconds::from_secs(0.015), Seconds::from_secs(0.05)),
            0
        );
    }

    /// The scatter of [`drift`]'s `q` on simulated walks, the figures `DESIGN.md`,
    /// `baro_offset_walk`, quotes: `cargo test --example replay -- --ignored drift_scatter
    /// --nocapture`. Each seed walks at 0.1 m/√s with 0.5 m of white noise on top, at 1 Hz.
    #[test]
    #[ignore]
    fn drift_scatter_on_simulated_walks() {
        let mut state: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut uniform = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut normal = move || {
            let (u, v) = (uniform().max(1e-300), uniform());
            (-2.0 * u.ln()).sqrt() * (2.0 * core::f64::consts::PI * v).cos()
        };
        for (span, shortest) in [
            (1200, 60.0),
            (1800, 60.0),
            (3600, 60.0),
            (7200, 60.0),
            (7200, 780.0),
        ] {
            let estimates: Vec<f64> = (0..40)
                .filter_map(|_| {
                    let mut level = 0.0;
                    let baro: Vec<(f64, f64)> = (0..span)
                        .map(|t| {
                            level += 0.1 * normal();
                            (f64::from(t), level + 0.5 * normal())
                        })
                        .collect();
                    let gnss: Vec<(f64, f64)> =
                        (0..span).map(|t| (f64::from(t) + 0.5, 0.0)).collect();
                    // Read at any span, to see the scatter below the floor as well.
                    drift_over(&baro, &gnss, shortest, 0.0).and_then(|d| d.walk)
                })
                .collect();
            let n = estimates.len() as f64;
            let mean = estimates.iter().sum::<f64>() / n;
            let sd = (estimates
                .iter()
                .map(|q| (q - mean) * (q - mean))
                .sum::<f64>()
                / (n - 1.0))
                .sqrt();
            println!(
                "{span} s from {shortest} s: {} seeds, mean {mean:.4}, sd {sd:.4}, {:.0} %",
                estimates.len(),
                100.0 * sd / 0.1
            );
        }
    }

    #[test]
    fn rounding_keeps_the_figures_asked_for() {
        assert_eq!(rounded(4.2371, 2), 4.2);
        assert_eq!(rounded(0.012_49, 2), 0.012);
        assert_eq!(literal(7.0), "7.0");
    }
}
