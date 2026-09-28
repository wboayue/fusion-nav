# Replay data

`cargo run --example replay` reads a recorded flight from CSV and writes the estimate back out as
CSV — the normalized log format the [validation harness](../GOALS.md#harness-constraint) is built
on. The format itself is documented in `examples/replay.rs`.

It writes two files. `<out>.csv` is one row per IMU epoch: the state, the covariance diagonal as
standard deviations, and the last test ratio per source. `<out>.fusion.csv` is one row per
`fuse_*` call — `t_s,source,nu0..nu2,s0..s2,ratio,outcome` — which is the resolution the epoch
file cannot reach, since it keeps only the most recent ratio and so cannot tell two fusions apart
or say what became of either. The per-source gates ride in that file's header, because the filter
reports `r = ε / γ` and a ratio without its `γ` does not go back to `ε`.

`nu*` and `s*` carry the filter's own published innovation and the diagonal of `S`. They are empty
only where the gate ran no update — an adoption, a refusal, or a call before initialization — and
the harness never works them out for itself; see the rule below.

## One statistic, one implementation

The Rust replay harness is the only thing that **computes** a statistic. It emits the per-fusion
rows above and the scalar keys on the `summary` and `score` lines. The Python tools under `tools/`
read those rows and those keys — they aggregate across logs, plot, and compare against the EKF2
reference — and never recompute a number the harness already defines.

The test is whether a quantity could ever be produced by both paths. If it could, it belongs to
the harness, because that is the one CI runs. Where a statistic is only meaningful across logs or
against the reference — distance from EKF2, the corpus table — it lives in Python and is defined
there once: `tools/agreement.py`, which reads no file and computes nothing the harness prints.

The same rule is why `nu*` and `s*` above are published by the filter rather than computed here
from the measurement and the covariance. The update of equations (23)–(28) owns that quantity, and
two implementations of it would eventually disagree — discovered, as these things are, while
somebody chases a filter bug that does not exist.

`tools/replay_report.py` is the rule applied to a whole document: it plots the per-fusion rows and
prints the `summary` and `score` keys beside them, and computes no statistic of its own; the
agreement table it prints beside a reference is `tools/agreement.py`'s. Its NIS
histogram is `ε = r γ` recovered from what the filter published, the way the harness's own
`nis_is_recovered_with_the_gate_the_filter_was_configured_with` recovers it — never `ν` and
diag(`S`), which would need the off-diagonals the fusion CSV does not carry and so would be a
second implementation and a wrong one. Lag-1 autocorrelation is not plotted at all, because
`acf1_` is already on the `summary` line and pinned per log.

## Simulated flights

`examples/simulate.rs` generates flights with **analytic ground truth**, which neither of the
corpora below has: the PX4 logs carry no truth and `--reference` only gives EKF2's own answer. It
writes two files per scenario — `<name>.csv` in the replay format, and `<name>.truth.csv`, the
state a perfect filter would report at each IMU epoch.

```console
$ cargo run --example simulate                         # every scenario -> target/sim/
$ cargo run --example simulate -- mission              # one of them
```

What each scenario covers is in the table in `examples/simulate.rs` — printed by the run above and
carried in both generated files' headers — rather than restated here. Its sensor noise is
deliberately **not** `ImuNoise::default()`: a filter scored against its own assumptions is being
handed the answer key.

### Scoring against truth

Hand the replay harness a truth file as a third argument and it prints a `score` line beside
`summary`, in the same `key=value` shape:

```console
$ cargo run --example replay -- \
    target/sim/mission.csv target/sim/mission.replay.csv target/sim/mission.truth.csv
```

```text
score pos_h=0.240 pos_v=0.273 vel=0.190 pos_h_max=0.878 tilt=0.509 yaw=0.636 …
```

Those are figures for a filter aided by **GNSS position and velocity**: (23)–(29) correct the
state at each fix and each velocity solution, and the barometer and the magnetometer are not
fused yet. Aided by position alone the same 185 s read `pos_h=0.702`; unaided, `pos_h=1248.627`,
because a 2° tilt error leaks gravity into the horizontal channel and integrates twice. They
move again when a stage of #31 lands.

What each key means, and what it can and cannot say on these scenarios, is in the module docs of
`examples/replay.rs`, which owns the definitions. Two things about *using* it belong here:

- **No truth file, no `score` line** — not a line of zeros. Every log in the PX4 corpus below has
  no truth, and `pos_h=0.000` on one of them would claim a perfect filter where the honest answer
  is that nothing knows. `manifest.txt` is untouched by scoring, and `fetch.sh --check` reads
  `summary` exactly as before. A truth file that was scored but matched no epoch is the same case:
  the line is `score scored=0` and no more.
- **The truth has to belong to the log.** Both files carry their scenario and seed in a `#`
  header, and a mismatch is refused rather than scored — nine `*.truth.csv` sit one
  tab-completion apart in `target/sim/`, and the epoch timestamps of the wrong one line up
  perfectly often enough that nothing else would notice. A log with no such header — a corpus
  log, a converted one — is taken on trust, because there is nothing in it to check.

### Ceilings, and what they gate

`data/scenarios.txt` holds a measured ceiling per key per scenario, and `data/bench.sh` asserts
them — the only gate here that reads accuracy rather than self-consistency, and the reason the
simulator exists:

```console
$ data/bench.sh                                        # every scenario; runs in CI
$ data/bench.sh mission static                         # only these
$ data/expect.sh --self-test                           # the comparator's own fixtures
```

It generates every scenario first rather than reusing `target/sim/`, so no ceiling can be met by
a flight produced before the change under test; it fails on a scenario the simulator generated and
this file does not gate, and on a line carrying no ceilings at all, since either reads as a green
run over a gate that is not there. It runs the debug build: the eleven scenarios
are about ten seconds all told, against a minute to build the crate again under a second profile,
and `score` is identical either way. `fetch.sh --check` uses `--release` for a reason that does
not apply here — a two-hour log at 1.4M epochs.

A breach names the pair and prints the whole `score` line, so a ceiling that moved for a good
reason is re-measured by copying:

```text
  BREACH     mission: tilt=2.125, wanted tilt<=2.000
    got pos_h=1248.627 pos_v=204.617 vel=22.866 … tilt=2.125 yaw=2.936 …
```

Each bound is the measured value plus 1 % — the header of `data/scenarios.txt` gives the
arithmetic, and the short version is that the simulator's `sin` and `cos` come from the platform
rather than from the `libm` crate that pins the filter, and propagation integrates a last-bit
difference over tens of thousands of steps.

Moving one is the same commitment as moving a manifest expectation: a sentence beside it saying
what the data said. Both directions count. Stage 3 tightened `flight` across the board and
loosened every 185 s scenario by an order of magnitude, and both were the same change: a filter
that propagates nothing holds its position at zero, which beats integrating a tilt error for
three minutes and loses to it over fourteen seconds.

The `seed` column pins which flight produced the numbers, checked against the header the
generator wrote. It is the same guard as the truth-file header above, one level out: that one
stops the wrong truth being scored against a log, this one stops the right truth being scored
against ceilings that belong to another flight.

**What a ceiling cannot say.** It pins what the filter did last run, so it fails a filter that got
worse and passes one that got *more accurate and more overconfident at once* — which is the
failure that matters, because `P` is not a diagnostic. It sets `S = H P Hᵀ + R` and so which
measurements the gate rejects, it sets the gain, and `Validity` is derived from it. The next
section is the test of that claim.

### The covariance's own promise

```bash
data/anees.sh                  # every scenario on 50 seeds, against data/anees.txt; a CI gate
data/anees.sh moving_start     # only these
python3 tools/anees.py --self-test
```

`nees_*` on the `score` line is one flight's NEES averaged over time, and no chi-square bound
applies to it: consecutive epochs share their error, and a 7 s lockout is diluted into 185 s.
`data/anees.sh` flies each scenario on seeds 1–50 (`simulate --seed`), each replay writes `ε` per
block per epoch to `<out>.nees.csv`, and `tools/anees.py` averages the fifty at every epoch. If
`P` is honest, fifty times that mean is χ²(150), so its bound is a quantile rather than a
measurement. Two keys per block are gated, one per shape of fault. `any_` counts epochs past the
bound made family-wise over the log by Bonferroni, which holds however correlated the epochs are
and is what catches a transient. `over_` is the fraction past the per-epoch 95 % bound, which
catches a mild overconfidence that never spikes. `data/anees.txt` says why each is set where it is, what
`ImuNoise` scaled down by ten does to them, and which four scenarios fail today and why.

One-sided, because the simulator's IMU sits below `ImuNoise::default()` on purpose and every
honest scenario is underconfident. The aggregator is standard-library Python run by `python3`,
not `uv run`: it reads the harness's output and nothing else, so it has no dependency to pin and
none of the converter's reasons to stay off the CI path. It computes no NEES; the harness does,
once, in `nees`.

### The log CI replays

`flight.csv` is the `flight` scenario's output, committed with its truth beside it, and the only
log CI replays. Regenerate it in place, never edit it:

```console
$ cargo run --example simulate -- flight data          # data/flight.csv + data/flight.truth.csv
$ cargo run --example replay                           # data/flight.csv -> target/replay.csv
$ cargo run --example replay -- <input.csv> <output.csv> [truth.csv]
$ cargo run --example replay -- data/flight.csv target/replay.csv data/flight.truth.csv
```

That last one is the shortest accuracy check in the repository: it needs no network, no PX4
tooling and no generated scenario, because both halves are committed.

Committed rather than generated on demand because the generator is a host tool: it calls the
platform's `sin` and `cos`, which are free to differ in the last bit between machines. In practice
they do not — an x86-64 and an arm64 build write identical scenarios on one machine, and the
ceilings above, measured on macOS/arm64, hold on CI's Linux/x86-64 runner, which is a second libm
as well as a second architecture. That second half is weaker than it looks: a ceiling is an
inequality, so it says the scores did not get worse, not that the same bytes were generated. Both
are observations rather than the guarantee the filter has below, and they are now load-bearing:
`data/bench.sh` regenerates every scenario in CI, so a platform whose `sin` differs in the last
bit would move a ceiling with no diff to point at. The committed `flight.csv` is the one file
that sidesteps it, which is why the determinism job compares that and not a generated one.

A given seed always gives byte-identical files on one machine, and each sensor draws from its own
stream, so changing one sensor's rate or model does not shift another's noise.

## Determinism

Replay output is bit-reproducible across architectures: the same input and the same toolchain give a
byte-identical CSV on x86-64 and on aarch64. CI asserts it — the `replay determinism` jobs replay
`flight.csv` twice on each of an x86-64 and an aarch64 Linux runner, then compare a sha256
of the result across the two. So a diff in `target/replay.csv` is your change, not your laptop,
which is what lets a stage of #31 be reviewed by asserting the file did not move.

It holds by construction rather than by luck. The filter is `no_std` and takes `nalgebra` with
`default-features = false`, so its transcendentals come from the `libm` crate — pure Rust, the same
bits on every target — and not from a platform libm, which is free to differ in the last bit on
`sin` or `atan2`. Rust contracts no FMAs, so a target with fused multiply-add rounds like one
without; the remaining arithmetic and `sqrt` are IEEE-exact.

The promise is across architectures, not across toolchains. A rustc or `libm` upgrade may move the
last bit, and that is a legitimate reason for the hash to change — unlike a difference between two
machines running the same one, which is a bug.

Matching hashes are a slightly weaker statement than matching state: the CSV is written at fixed
precision, so a last-bit difference usually rounds away before it reaches the file. The check also
only covers as much arithmetic as the harness exercises, and that grows with each stage that
lands: before (9)–(15) the estimate columns were constant after the initialization window, so the
hash could only see the window. Every row is now a propagated one.

## PX4 corpus

Real flights come from [PX4 Flight Review](https://review.px4.io/). They are fetched rather than
committed: `logs/` is gitignored, and `manifest.txt` pins each log by sha256.

```console
$ data/fetch.sh                 # fetch + verify the manifest
$ data/fetch.sh --verify        # checksums only, no network
$ uv run tools/ulog2replay.py data/logs/<log-id>.ulg -o data/logs/<log-id>.csv --reference
$ cargo run --example replay -- data/logs/<log-id>.csv
```

A converted log dates each measurement as the log's own EKF2 did. `t_s` is when a row was logged
and `t_meas_s` when it was taken: `t_s` less `EKF2_GPS_DELAY` for a fix (`SENS_GPS0_DELAY` on a
build that has it), `EKF2_BARO_DELAY` and `EKF2_MAG_DELAY` for the other two, blank where the
delay is zero. A `# Measurement delays` header line says which parameter each figure came from.
No corpus receiver logs a sample time of its own: `sensor_gps.timestamp_sample` is zero or equal
to `timestamp` wherever it appears.

A `gnss_yaw` row, `v0` the heading in radians and `var0` its variance, is written only where the
log's own EKF2 fused a dual-antenna heading: `EKF2_GPS_CTRL` bit 3, or `EKF2_AID_MASK` bit 7 on a
build before it, *and* a finite `sensor_gps.heading`. The field alone is no evidence, since a ULog
records a name and never what a driver put in it. `a299e722` is the one corpus log with both, a
real moving-baseline yaw within 0.010 rad of EKF2's at rest; it logs no `heading_accuracy`, so the
variance is PX4's 0.1 rad floor, the value its EKF2 fused at, and a header line says so. It is
dated by the GNSS delay, as its fix is.

### Finding a candidate

An entry exists because it covers something no other log does, and #86 names the gaps. A
candidate is found in three passes, each cheaper than the one after it, so most are refused
before anything is downloaded whole.

**Metadata.** Flight Review publishes every public log's metadata as one gzipped JSON array —
hundreds of thousands of entries, `sys_hw`, `ver_sw_release`, `mav_type`, `estimator`,
`duration_s`, `airframe_name`, `description` and `download_url` among them. It narrows by
airframe, firmware and length, and says nothing about the sensors:

```console
$ curl -sL https://review.px4.io/dbinfo | gunzip > dbinfo.json
$ python3 -c '
import json, re
for e in json.load(open("dbinfo.json")):
    v = re.match(r"v?(\d+)\.(\d+)", e.get("ver_sw_release") or "")
    if (e["estimator"] == "EKF2" and "SITL" not in str(e["sys_hw"])
            and v and (int(v[1]), int(v[2])) >= (1, 14)
            and e["mav_type"] == "Fixed Wing" and e["duration_s"] > 300):
        print(e["download_url"], e["sys_hw"], e["duration_s"], e["airframe_name"])'
```

Thousands of real EKF2 logs on current firmware pass for multirotors, fixed-wings and standard
VTOLs, and hundreds for tailsitters. Descriptions are sparse: a few mention a catapult, and on
the order of a hundred mention RTK or vibration.

**`--screen`.** Download the candidate somewhere other than `logs/` — `fetch.sh --add` writes the
manifest — and read what the ULog alone can say about it:

```console
$ uv run tools/ulog2replay.py candidate.ulg --screen
screen sitl=no hw=PX4_FMU_V5 sw=v1.11.3 duration=7127 imu_hz=199 gnss=vehicle_gps_position
  fix_max=4 eph_min=1.17 eph_max=2.18 sats_min=17 sats_max=26 clip=0 vib_p95=0.094 vib_metric=dv
  ekf2=quat24 vehicle_imu=yes type=mc mode_changes=0
```

Every value is one number or one word, so most of a gap's criteria are `expect.sh` pairs, checked
with `compare_pairs` after sourcing the file. `sw=` carries the firmware type (`v1.16.0-rc`). `imu_hz=`
is `sensor_combined`'s median rate: a logger profile can sample it at 5 Hz, which replays as a
coasted step at every epoch.
`ekf2=` names the covariance layout `--reference` will read — `err24`, `err23` or `quat24`, the
table below — or says `unmapped` (LPE) or `none`. `vib_metric=` says which quantity `vib_p95=`
is, since PX4 `f2ae8ae814` changed it under one field name: `dv`, a filtered Δv difference in m/s
before v1.13, `accel`, an acceleration difference in m/s² from v1.13's betas on, or `unknown`
for a v1.12 or v1.13 build short of beta, which could be either, or a vendor's own version. `2c42096b`'s 0.094 `dv` and a v1.15 log's 32.8 `accel` are not
a quiet airframe and a loud one, and no rescaling makes them one quantity, so compare `vib_p95`
only within one metric. What a pair cannot say is in prose:

| gap | on the `screen` line | then, on the `summary` line |
| --- | --- | --- |
| every entry | `sitl=no`, `imu_hz>=100`, and `eph_min` differing from `eph_max` | |
| real baseline | `vehicle_imu=yes duration>=600`, and `ekf2=` a layout | `align=static` |
| short-and-still start | | `align=short alpha0=window` |
| RTK receiver | `fix_max>=6` | `nis_gnss_pos=`, recorded in the note |
| high vibration | `clip>=1` | `rejected_mag=`, if it moves |
| fixed-wing | `type=fw` | `extent>=1000` |
| VTOL, tailsitter | `type=vtol mode_changes>=2` | `tilt_max>=80` for a tailsitter |

**Replay.** What the vehicle did — `extent=`, `speed_max=`, `tilt_max=` — and whether its start
was still are the harness's to say, not the screen's: the harness is where a statistic is
computed, and stillness is `init::at_rest`'s claim. Convert, replay, open the
`replay_report.py` page, write the manifest note's sentence naming the gap, and only then
`fetch.sh --add` and `fetch.sh --pin`.

### Converter changes are batched

A change to `tools/ulog2replay.py`'s output costs a corpus regeneration: every log reconverted
and replayed, and every moved `manifest.txt` expectation explained. Land the changes that move
that output together — one `--check` run and one manifest diff that names, per moved
expectation, the change that moved it. Separately, each diff obscures the last: the second
regeneration's manifest diff mixes its own movement with whatever the first left unexplained.

Every column now reaches something the manifest pins. A variance the converter writes is the `R`
a `fuse_*` gates against, so moving it moves `rejected_<source>=` and the consistency keys, where
before the update existed it changed CSV bytes and no key.

### What `--reference` writes, and what it cannot

`--reference` writes EKF2's own solution to a second file on the replay timebase, for a
side-by-side diff. It is never filter input. Four row kinds, each at its own publication rate
rather than resampled onto a common grid — interpolation is a statistic, and not a converter's:
`ekf2_local` (position and velocity), `ekf2_att` (attitude as the quaternion `q0..q3`, and
`quat_reset_counter` where the topic carries it), `ekf2_states` (biases and the covariance diagonal
as standard deviations), and `ekf2_ratio` (the four aggregate innovation test ratios). Column
names match the epoch file's, so a diff is by name.

Attitude is a quaternion in both files — Hamilton, scalar-first, body to NED, the convention
`Attitude::body_to_ned` names and `vehicle_attitude.q` already logs — rather than Euler angles,
because ZYX Euler cannot separate roll from yaw at 90° of pitch, where a tailsitter cruises (#129).
On the `mission` scenario with the circuit's pitch amplitude raised from 0.12 rad to 1.9 rad (a
local edit to `examples/simulate.rs`, default seed), which pitches to 109°, ZYX roll and yaw read
off the epoch file's quaternion step 125° in one epoch at t = 113.885 s while the quaternion moves
by a fraction of a degree. The report derives what it draws.

A fifth kind, `vehicle_mode`, is not EKF2's: it is the vehicle's flight regime — `mc`, `fw`,
`to_fw`, `to_mc`, `undefined` or `other` — one row per change, for shading the report. PX4 tells
EKF2 its regime and this filter is told nothing, so a reader of a VTOL log needs to see where it
changed; it goes in the reference, never the replay input, so the filter cannot read it. It is
taken from MAVLink's `MAV_VTOL_STATE` and `MAV_TYPE`, never from `vehicle_status.vehicle_type`,
whose constants were renumbered under one field name — `3949f175` logs its quadrotor as 0
(AGENTS.md, "A ULog field name does not pin its meaning"). Eight corpus logs read only `mc` and
two only `fw`; `285ee2e7` and `4b473e91` are VTOLs and read all four regimes; `a299e722` reports
`MAV_TYPE` 202, outside MAVLink's enum, and reads `other` rather than a guess.

EKF2's state vector has been laid out the same way in every version that logs one, but **its
covariance has not**, three times, and neither count alone says which: `n_states=24` is both the
state-indexed covariance and the first error-state one, and a 24-entry covariance is both the
state-indexed one and the error-state one with terrain. The pair `(n_states, entries)` separates
all three, and `tools/ulog2replay.py` keys the index map on it. That table is the only place the
map lives, with the PX4 commits that moved it cited above it. What it means per log:

| `ekf2=` | `n_states`, entries | covariance | bias states | corpus |
|---|---|---|---|---|
| `err24` | 25, 24 | error-state, terrain last | rad/s and m/s² | `3949f175`, `eb799954`, `285ee2e7`, `4b473e91`, `7ce66f0d`, `093e806a` |
| `err23` | 24, 23 | error-state, no terrain (v1.15) | rad/s and m/s² | `89a498ce`, `cd7e0001` |
| `quat24` | 24, 24 | indexed like the state vector, four quaternion entries first | delta-angle and delta-velocity per filter update | `a299e722`, `2c42096b`, `f16771dd` |
| `unmapped` | anything else | refused | refused | `7592c9b2`, an LPE log reporting 10 |

Four boundaries follow, and they are properties of the logs rather than of the converter:

- **The quaternion era supplies no attitude σ**, on three of the thirteen corpus logs. Four quaternion
  variances become a rotation-vector σ only through the full 4×4 block, and the log carries the
  diagonal alone. The cells are blank rather than filled.
- **Where there is one, it is in NED.** PX4 stores the error-state attitude covariance in the
  navigation frame — `getRotVarNed` returns the diagonal as stored while `getRotVarBody` rotates
  it by `Rᵀ(·)R`, `EKF/ekf_helper.cpp:926-937` at `c4e4ef98` — while this crate's `δθ` is a local
  body-frame perturbation and (36) gives the navigation-frame error as `R(q̂) δθ`. The columns are
  named `sigma_att_n/e/d` for that reason, and they compare with the epoch file's
  `sigma_tilt_n`, `sigma_tilt_e` and `sigma_heading`, which the replay reads off
  `Eskf::attitude_variance` in the same frame. `sigma_att_total` is emitted beside them: a trace
  is invariant under rotation, so it also compares with the body-axis `sigma_att_x/y/z`. A
  single tilt σ is deliberately *not* emitted — PX4's `getTiltVariance` sums the two horizontal
  variances where (36′) and `Validity` read each against the bar, and naming those alike would
  compare two different quantities.
- **EKF2's position is written in the replay frame**, not as EKF2 logged it. PX4's local x/y are
  its `MapProjection`, azimuthal equidistant on a 6371 km sphere (cited at `px4_reproject` in
  `tools/ulog2replay.py`), where the replay frame is the exact tangent plane of (43). The two
  differ in scale as well as origin, by about 0.2 % of the distance out, so no single shift
  aligns them: shifted by one, EKF2 reads a median 3.7 m north of its own fixes on `89a498ce`,
  4.07 km out. The converter reprojects each row about that row's own EKF2 origin, which
  `093e806a` moves once mid-log and `7ce66f0d` moves in height, and places it as it places a
  fix. `EKF2 position in replay frame:` says which axes it placed;
  `EKF2 origin in replay frame: N E D m` still records where EKF2's first origin sits.
- **Two of thirteen logs report no origin** (`xy_global` false, the reference fields all zero), so
  EKF2's `x,y,z` there are origin-relative with no origin, and stay in EKF2's frame.
- **The two origins are on different vertical datums.** The replay input's origin is the first
  fix at its ellipsoidal height where the receiver logs one, and EKF2's `ref_alt` is MSL
  (`msg/versioned/VehicleLocalPosition.msg:57` at `c4e4ef98`). The converter moves EKF2's heights
  onto the ellipsoid by the first fix's own two heights, and where the fix logs no MSL height it
  places north and east only. Read without that shift, the geoid height is the whole of the down
  offset: −25.41 m on `eb799954` in Oklahoma and +20.58 m on `89a498ce` in Korea, and
  `eb799954`'s `pos_d_rms` read 27 m where it reads 0.60.

Three more header lines say what the rows came from rather than what they hold. `Estimator:`
names which PX4 estimator published them, from `SYS_MC_EST_GROUP` or `EKF2_EN`/`LPE_EN`: the row
kinds are `ekf2_*` on every log, and on `7592c9b2` they are LPE's. `EKF2 aiding:` gives the height
reference EKF2 was configured to converge to (`EKF2_HGT_REF`, `EKF2_HGT_MODE` before it) and the
share of `control_mode_flags` samples on which each GNSS, barometer and magnetometer bit is set,
because a build with `EKF2_HGT_REF` fuses barometer and GNSS height at once and only the parameter
says which one it follows. `ekf2_local` also carries EKF2's `xy`, `z`, `vxy` and `vz` reset
counters, beside `att_reset`, since a step across one is an event in EKF2 rather than divergence.

The delta-angle era is scaled to rates so the column means the same thing on every log, and the
scale factor is stated in the header with where it came from. It has to be the quantity PX4 itself
divides by, which is `_dt_ekf_avg` — `getGyroBias() { return _state.delta_ang_bias / _dt_ekf_avg; }`
with the variance over `sq(_dt_ekf_avg)`, `EKF/ekf.h:239-244` at `ae3070bbf1^` — and that is a
running **mean** of the realized step, not an integer multiple of anything.

It is **not** the topic's publication interval: `estimator_status` publishes at 5 Hz on two corpus
logs and `estimator_states` at 1 Hz on a third. It is `max(target, imu_dt)`, where `target` is
`EKF2_PREDICT_US` when the log carries it and otherwise the `FILTER_UPDATE_PERIOD_MS{10}` that
preceded that parameter (`EKF/estimator_interface.h:267` at `ae3070bbf1^`). The down-sampler is
built to hold that mean rather than to round up to a sample boundary: it fires when the
accumulated `delta_ang_dt` reaches `_target_dt - _imu_collection_time_adj` and then moves the
adjustment by `0.01f * (delta_ang_dt - _target_dt)`, a feedback term whose own comment says it is
there "so that we meet the average EKF update rate requirement"
(`EKF/imu_down_sampler.cpp:36-43` at `ae3070bbf1^`). On a 250 Hz IMU against a 10 ms target it
alternates two- and three-sample steps and averages 10 ms; it does not settle at 12.

Both halves of the `max` are load-bearing. Rounding a 4 ms IMU up to 12 ms scales every bias and
bias σ in that log 20 % low, and a 50 Hz log genuinely does run a 20 ms period against a 10 ms
target, because nothing can subdivide a sample longer than the target.

That `imu_dt` is the one place the converter computes something the harness also computes — the
IMU interval, which the `summary` line publishes as `rate=` and `manifest.txt` pins. Two
estimators of one quantity is what *one statistic, one implementation* forbids, and the failure
would be silent, moving a published bias figure with nothing to point at. So it is guarded the way
`write_truth_header` and `TRUTH_COLUMNS` are: the converter writes the interval it used into the
header, and `replay_report.py` refuses a set of files whose interval and `rate=` disagree.

Reading the columns back is what verifies them — a wrong index is silence, not a failure. On
`2c42096b`, 35583 samples over 2 h, this filter's gyro-bias estimate and EKF2's scaled
delta-angle bias agree to 5.3e-4 rad/s (0.03 °/s), which is what confirms the units and the index
map. The rounding itself rests on PX4 source rather than on that number: it moves `2c42096b` by
0.5 %, and the log where it would matter carries 58 bias samples against a velocity source this
filter rejects 266 of 609 solutions from, so its own bias wanders by ±0.01 rad/s and cannot
adjudicate anything.

Tilt agreed with EKF2 within 0.13° on the five logs it was checked on (#129) at EKF2's first attitude sample after the
initialization window — read with the report's own `tilt_heading` and `rotation_difference`, so
no second implementation of either — which is what confirms the quaternion convention and the
timebase rebasing at once. A magnitude cannot see a tilt in the wrong direction, so the direction is read
off the rotation between the two: its body x and y components are within 0.42° on the three logs
whose headings there agree within 17°. On the other two they are not a tilt comparison — a
heading difference about navigation down lands partly on body x and y, by the sine of the tilt,
and `f16771dd` reads 1.84° and 2.39° there with its headings 162° apart at 1° of tilt.
**Absolute yaw does not compare at an instant** and should not be read as
divergence: EKF2 resets yaw in the first seconds — on `3949f175` it moves 41.60° to 16.66°
between t = 2 s and t = 4 s, which is what `ekf2_att`'s `att_reset` column is for — and the
residue after its reset was mostly a declination difference while this harness fixed −0.06 rad
for every log. It now configures the one the log names (`declination=`), and the median heading
difference to EKF2 on the five real logs with GNSS is 0.00–2.45°, where it was 4.39–13.34°.

### Per-log reports

`tools/replay_report.py` renders one replay into a single self-contained HTML file: horizontal
track against EKF2 and the raw fixes, every state with its ±3σ band and `Status` shaded behind it,
attitude as tilt and heading and as its rotation from EKF2's in body axes,
all the σ on one log axis with GNSS gaps shaded, per-axis normalized innovations with the gate and
its rejections, the NIS histogram and QQ plot against χ², and the published keys.

```console
$ cargo run --release --example replay -- data/logs/<log-id>.csv target/replay.csv > target/<id>.summary
$ uv run tools/replay_report.py data/logs/<log-id>.csv target/replay.csv \
      --reference data/logs/<log-id>.reference.csv --summary target/<id>.summary \
      -o target/report/<log-id>.html
```

One HTML file with the PNGs inlined, and no JavaScript, for the same reason `README.md` carries no
mermaid: it renders wherever it lands. The 2 h log takes 27 s and produces 2.4 MB, decimating
1.4 M epochs to about 4000 points per trace with a min/max envelope per bucket, so a transient
survives the stride. Sources, their gates and their degrees of freedom are discovered from the
files, so adding a measurement source to the crate needs no edit in that tool.

`--summary` is how the report reads the statistics rather than recomputing them, and a captured
stdout has nothing in it tying it to the CSVs beside it — the hazard `Scoring::open` already
refuses for a truth file from the wrong scenario, where the timestamps line up often enough that
nothing else notices. So the tool refuses a mismatched set, on three reads: `epochs=` against the
epoch CSV's row count, each `rejected_<source>=` against that source's `rejected` rows in the
fusion CSV, and the reference's IMU interval against `rate=`.

Two caveats are printed beside the distribution plots rather than left to a reader, because both
are measured and both look like filter faults. **No source in this corpus is white** — `acf1_`
runs 0.5736–0.9885 on GNSS position, 0.1712–0.8511 on the barometer and 0.1793–0.9851 on the
magnetometer — because a 1 Hz receiver filters its own solution in time and a 250 Hz magnetometer
is sampled far faster than the field it reads changes. And **`R` for the barometer and the
magnetometer is a converter constant**, so their distribution tests those constants; only GNSS
tests a receiver's own reported accuracy.

`data/fetch.sh --check` converts and replays the whole pinned corpus and asserts the per-log
expectations recorded beside each checksum against the `summary` line the replay example prints,
through the same `data/expect.sh` that `data/bench.sh` reads ceilings with — so `key=value` here
and `key<=value` there are one language, and a key named in the manifest but missing from the
`summary` line fails instead of passing unnoticed.

A fourth form, `key=lo..hi`, pins a value between two inclusive bounds, and the consistency
statistics below are what it is for. An exact pin on a statistic says *this number* where the
claim is *this receiver reports six times the accuracy its own solutions support*: it makes every
filter change that moves a digit a manifest edit, and it states nothing a reader can disagree
with. A range states the finding and survives the digit. Both endpoints go through the same
numeric check a ceiling gets, so `none` cannot clear a bound the filter never met. The band a
range gets is stated in `manifest.txt`'s header, and `data/fetch.sh --pin <name>` applies it, so a
new entry's ranges are derived rather than typed.

The keys are `rate=` and `window=` (the IMU rate and the samples it takes to cover
`min_duration`), `extent=`, `speed_max=` and `tilt_max=` (what the vehicle did: the farthest
horizontal GNSS row from the first and the fastest horizontal GNSS velocity, as reported, and the
filter's own largest tilt from the vertical — what a manifest note's "past a kilometre" is pinned
by), `align=`, `an=` and `alpha0=` (what initialization achieved — `static`, `short` for a still
window that never reached `min_duration`, or `coarse` for a moving one — whether a moving
window measured the vehicle's own acceleration from GNSS velocity — `ā_n` of equation (5′), which
only a moving start reports — and where the barometric reference came from: `window`, `estimate`
once a fix has established position, or `none`), `heading=` (`Validity::heading` **as initialization left it** — not as the
log ended, which would only restate `transitions=`), `declination=` (the magnetic declination
the harness configured, in degrees, read from the log's `# Magnetic declination` header line —
zero where it has none), `resets=` (adoptions, per source, so a GNSS fix adopted whole counts in both its halves),
`aligned_at=` (seconds from the end of the window to the first epoch `Eskf::is_aligned` read true,
or `never`), `attitude_lost=` (seconds to the first epoch at or after it where `Validity::attitude`
read false against `Config::accuracy` — the mission's bar, where `aligned_at=` reads the fixed
alignment bars — which is where
the covariance growth of (16)–(22) shows up on logs with no truth — `Status::Aligning` latches, so
nothing else on the line moves when it happens), `r_policy=` (what the harness handed each GNSS
`fuse_*` as `R` — `raw` on every entry, and the paragraph below the caveats says why it is not a
floor; `px4` under `--r-policy px4`, which only `data/ekf2.txt` pins), `course=` and `without=`
(choices too: the sideslip in degrees the course constraint was fused at after each `gnss_vel`
row, from `--course` or a `# Course sideslip` header line, or `off`; and the input source
`--without` dropped, or `none`; `off` and `none` on every manifest entry, so a figure from a log
replayed as a vehicle without its magnetometer says so), `rejected=` and `discarded=` (the gate's verdict, and everything that never reached it — a
variance of zero or less, a NaN, an altitude with no reference; both count verdicts, so a GNSS
fix judged or refused whole counts once per half), `refused=` and `invalid=` (steps
refused as too long or as not a step at all — propagation, not measurements), `floored=`
(variances raised to the diagonal floor of equation (42′), pinned at zero on every log because
that is the claim — the floor sits far below anything the filter reaches, so a non-zero says a
covariance is being driven toward zero by something upstream and the floor is masking it), and
`epochs=`, `transitions=` and `status=`, followed by four families of consistency statistic, one
set per source: `nis_` (mean normalized innovation squared per degree of freedom, 1 when `S`
describes its own innovations), `nis_over95_` (the fraction above the 95 % χ² quantile, 0.05 when
it does), `nu_` per axis (mean innovation, 0 when nothing is biased) and `acf1_` (lag-1
autocorrelation of the normalized innovation, 0 when successive measurements are independent).
Each is built from `ν` and diag(`S`) as the filter published them, and from `ε = r γ` with `γ` off
`Config::gates`, so none re-derives (23) or (24) — it is the gate's own `ε`, not a copy of it.

**What those four are testing is not always this filter**, and two caveats belong beside every
figure they produce:

- PX4 logs **no variance at all** for the barometer or the magnetic heading, so
  `tools/ulog2replay.py` substitutes a constant. `nis_baro` and `nis_mag` therefore measure that
  constant rather than the filter's tuning, and the corpus reads 0.0309–0.5164 and 0.0026–0.9046
  on the real logs whose estimate held, conservative by 1.4–5.7× in σ for the barometer and
  1.1–19.6× for the magnetometer, stated separately because one range across both understates the
  magnetometer's low end. `data/manifest.txt` names the logs set aside.
  Nothing here had ever tested them; that is the first measurement
  [#50](https://github.com/wboayue/fusion-nav/issues/50) can argue from.
- For GNSS the statistic tests the receiver's own `eph`/`epv`/`s_variance_m_s`, which the harness
  passes through unfloored where both production estimators bound theirs. Horizontal position
  reads 0.0184–0.2251 on the real logs whose position gate stays quiet and height 0.0865–0.9207,
  so those figures are wider than their residuals earn; `a299e722`'s velocity reads 6.6705, a
  receiver contradicting its own differenced positions.

**Unfloored is the policy, and it is deliberate.** Both production estimators bound a receiver's
reported accuracy before fusing, and this crate ships the same facility for an integrator who
wants it — `PositionNoise::clamped` and `VelocityNoise::clamped`, whose doc comments carry both
platforms' parameters, `file:line` and the shas they were read at. The harness applies neither,
for three measured reasons:

- **A floor erases the figure rather than bounding it.** The `EKF2_GPS_V_NOISE` each log flew,
  0.25–0.3 m/s (the 0.5 in `EKF/common.h:370` is an initializer the parameter overrides), sits
  above 9988 of the corpus's 15021 velocity solutions: every one on `093e806a`, `2b2ad123`,
  `4b473e91`, `89a498ce` and `a299e722`, 98.9 % or more on `285ee2e7`, `cd7e0001` and `eb799954`, 55.9 % on
  `7ce66f0d`, 12.4 % on `2c42096b` and none on the SITL log. Honest receivers included, it is the
  operative value on most logs rather than a backstop. The position floors bind on four receivers:
  reported σ_h falls to 0.297 m on `093e806a` and 0.436 m on `4b473e91` against a 0.5 m bound,
  `89a498ce`'s RTK receiver reports 0.014 m horizontally and 0.01 m vertically on every fix, and
  `2b2ad123`'s 0.014-0.020 m; σ_h
  is 0.622 m and up on the rest, and σ_v averages 0.51–6.26 m against 0.75 where there is no
  RTK. Since the barometer and the magnetometer already carry
  converter constants, flooring would leave almost no receiver-reported variance in the corpus.
- **It costs most or all of the only GNSS rejections the multirotors have.** `a299e722` (314
  velocities, 70 positions) and `2b2ad123` (18 positions, 1 velocity) are the only non-zero GNSS
  counts across the nine multirotor and SITL logs, and under their own floors they read 40 and 0.
  `2b2ad123`'s show what that erases in both directions: nine refuse fixes that step and revert,
  which the floors would fuse, and nine are this filter lagging an acceleration (#169), which the
  floors would have prevented (its manifest entry). (Of
  the four airframe logs, measured at `Recovery::OFF` with every fix fused as white, the floors
  correct one: `093e806a`'s 860 position rejections read 92 under them, where recovery alone read
  291, and 278 with (24′). They leave `4b473e91`'s
  lockout, which recovery removes, and `7ce66f0d`'s divergence, which neither removes.) Replayed with the floors applied,
  the 278 `a299e722` read before #137 read 0 under PX4's treatment — the 0.5 m/s floor *and* the separate `sq(1.5f)` vertical
  widening — 2 under that floor alone, and 44 under ArduPilot's per-axis 0.3/0.5, which is the one
  policy that would leave the gate of (37)–(38) still exercised by real data. `transitions=` went
  4 to 2 under all three in that measurement (#113), so this is not the only key a floor would move.
  Re-measured with (24′) and each log's own parameters (`--r-policy px4`), the 266 read 37 at
  `a299e722`'s 0.25 m/s and 6 at 0.5, and `transitions=` still 4 to 2.
- **It moves the accuracy gate.** `examples/simulate.rs` draws GNSS velocity noise at σ = 0.15 m/s
  and `data/bench.sh` scores every scenario through this same harness, so a floor would hand every
  simulated fix an `R` 11× too wide and move the ceilings in `data/scenarios.txt` — distrusting a
  receiver the simulator defines as honest.

The policy belongs to the corpus rather than to one log: a source added later reports its own
accuracy the same way and is fused the same way. What it costs is that a figure here is not
directly comparable with EKF2's on the same log, which fuses a floored `R`. So the comparison with
EKF2 runs both: `--r-policy px4` replays each log at EKF2's own floors, with the parameters it
flew, and [Agreement with EKF2](#agreement-with-ekf2) reads the distance under each. `r_policy=`
on the `summary` line carries the verdict, so a published figure travels with the policy that
produced it.

`data/fetch.sh --check` needs `pyulog`, so it is a local tool rather than a CI job:
`data/fetch.sh --venv` once, which installs the version the converter pins, and `fetch.sh`
finds the gitignored `.venv` on its own.
The converter declares its own dependency inline (PEP 723), so `uv run tools/ulog2replay.py`
needs no virtualenv at all.

That dependency is pinned to an exact version, and `--check` stops if the interpreter it is about
to convert with holds a different one. The two paths only produce the same CSV for the same
pyulog, and a converter that changed underneath the corpus would move the expectations below with
nothing in the repository to blame. `tools/ulog2replay.py` holds the pin; `fetch.sh` reads it from
there.

`tools/replay_report.py` pins its own dependencies the same way and nothing enforces those, which
is not an oversight to correct. The enforcement exists because converter output reaches
`manifest.txt`, where a silent change has no diff to point at; a report reaches nobody's
expectations, and a matplotlib that renders a line differently moves no pinned number.

`data/fetch.sh --add <url> [name]` downloads a log once and appends a manifest line to commit; the
files stay out of the repo, the checksums do not. Each entry should cover something no other log
does.

Flight Review logs are [CC BY 4.0](https://review.px4.io/).

### Agreement with EKF2

EKF2 is not truth, so this measures agreement, never accuracy: two estimators fed one log, and a
divergence is a finding to explain before it is an error on either side (GOALS.md, "Three
questions, three kinds of source").

```console
$ data/fetch.sh --compare          # replay every log raw and px4, assert data/ekf2.txt
$ data/fetch.sh --compare --pin    # print the lines to commit instead
```

`--compare` converts each log with `--reference`, replays it under both `R` policies into
`target/compare/<log>/`, and runs `tools/replay_report.py --corpus` over the directory, which
writes `agreement.html` there, one table per family with a row per log and policy, and prints one
`agreement` line per run for `data/expect.sh` to compare. About a minute for the corpus. The
statistics are `tools/agreement.py`'s and nothing else computes them; the report reads the files
and hands it arrays. A single report with `--reference` carries the same table for its one run.

What each key is, and what it cannot say:

- `pos_{n,e,d}_{rms,max}` and `vel_…`: EKF2's estimate less this filter's, each EKF2 sample paired
  with the nearest epoch within two epoch intervals, never interpolated. Position after adding
  EKF2's origin in this filter's frame from the reference header; `none` where EKF2 reports no
  origin. `pos_d` is in one frame, but the two filters may follow different height references.
- `_nd2`: the mean of `d² / (σ²_ours + σ²_EKF2)`, on EKF2's covariance timeline. Near 1 or below
  where each covariance covers the other's estimate. A scale for comparing logs, not a χ² to test
  against: both filters read the same sensors, so the two errors are not independent. A sample
  where EKF2 reports σ = 0 is a state it is not estimating and is left out.
- `climb`, `climb_ekf2`: each filter's own height change, last 60 s mean less first, up positive,
  over the span both cover, beside `height_reference_ekf2`. As change because each filter
  converges to its own reference. `none` on a log under 120 s, where the two windows overlap.
- `tilt_diff_{rms,max}`: tilt difference from the swing-twist split, degrees, named apart from the
  `summary` line's `tilt_max`, which is the vehicle's own peak tilt. `heading_diff_med`: median
  absolute heading difference after EKF2's first attitude reset once this filter runs, which is
  its yaw alignment; later resets stay in. `att_nd2`: the rotation between the two over the sum
  of the traces, the one attitude scalar both files carry in one frame; `none` on the quat24 era.
- `ba_…`, `bg_…`: `_rms` and `_nd2` per body axis, EKF2's delta era already scaled to rates.
- `rej_s_<source>`, `…_ekf2`, `…_both`: seconds each filter spent over its gate, and seconds both
  did. As time rather than counts, because EKF2 publishes its test ratios at 1–5 Hz and this filter
  judges every fusion. A verdict holds until the source's next sample and for at most five of its
  median intervals (`agreement.HOLD`, the multiple the report shades a GNSS outage at), so a
  logging dropout does not stretch one rejection across the gap. `baro`
  compares only against a barometer height reference, since EKF2's height ratio belongs to
  whichever source is active; `gnss_hgt` has no EKF2 counterpart at all.
- `ekf2_{xy,z,vxy,vz,att}_resets`: how often each EKF2 counter moved, beside this filter's
  `resets=` and `recovered=`. `estimator`: which PX4 estimator the reference is.

Every line carries every key, the harness's (`nis_`, `rejected_`, `resets=`) read off the
`summary` beside the agreement ones. `--pin` decides what `data/ekf2.txt` holds: a `raw` line
without the keys its log's manifest entry already pins, a `px4` line whole, since nothing else pins
it. It bands whatever is printed with a decimal point (`pin_pairs --decimal`), which is every
statistic and no count, so a new statistic needs no edit in `fetch.sh`. `data/ekf2.txt`'s header
records what the first run said.
