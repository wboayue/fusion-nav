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
there once.

The same rule is why `nu*` and `s*` above are published by the filter rather than computed here
from the measurement and the covariance. The update of equations (23)–(28) owns that quantity, and
two implementations of it would eventually disagree — discovered, as these things are, while
somebody chases a filter bug that does not exist.

`tools/replay_report.py` is the rule applied to a whole document: it plots the per-fusion rows and
prints the `summary` and `score` keys beside them, and computes no statistic of its own. Its NIS
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
run over a gate that is not there. It runs the debug build: the nine scenarios
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
measurements the gate rejects, it sets the gain, and `Validity` is derived from it. Testing that
claim needs ANEES over N seeds against a chi-square bound, and a covariance that moves:
[#89](https://github.com/wboayue/fusion-nav/issues/89), after
[#35](https://github.com/wboayue/fusion-nav/issues/35). `nees_pos`, `nees_vel` and `nees_att` are
already on the `score` line and already ratcheted here; what is missing is the bound, not the
statistic.

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
`ekf2_local` (position and velocity), `ekf2_att` (attitude, and `quat_reset_counter` where the
topic carries it), `ekf2_states` (biases and the covariance diagonal as standard deviations), and
`ekf2_ratio` (the four aggregate innovation test ratios). Column names match the epoch file's, so
a diff is by name.

EKF2's state vector has been laid out the same way in every version that logs one, but **its
covariance has not**, and the two eras both report 24 entries meaning different things — so no
field spelling distinguishes them and `tools/ulog2replay.py` keys the index map on `n_states`.
That table is the only place the map lives, with the PX4 commits that moved it cited above it.
What it means per log:

| `n_states` | covariance | bias states | corpus |
|---|---|---|---|
| 25 | error-state | rad/s and m/s² | `3949f175` |
| 24 | indexed like the state vector, four quaternion entries first | delta-angle and delta-velocity per filter update | `a299e722`, `2c42096b`, `f16771dd` |
| anything else | refused | refused | `7592c9b2`, an LPE log reporting 10 |

Three boundaries follow, and they are properties of the logs rather than of the converter:

- **The 24-state era supplies no attitude σ**, on three of the five corpus logs. Four quaternion
  variances become a rotation-vector σ only through the full 4×4 block, and the log carries the
  diagonal alone. The cells are blank rather than filled.
- **Where there is one, it is in NED.** PX4 stores the error-state attitude covariance in the
  navigation frame — `getRotVarNed` returns the diagonal as stored while `getRotVarBody` rotates
  it by `Rᵀ(·)R`, `EKF/ekf_helper.cpp:926-937` at `c4e4ef98` — while this crate's `δθ` is a local
  body-frame perturbation and (36) gives the navigation-frame error as `R(q̂) δθ`. The columns are
  named `sigma_att_n/e/d` for that reason, and `sigma_att_total` is emitted beside them: a trace
  is invariant under rotation, so it is the one attitude scalar comparable to the epoch file's
  `sigma_att_x/y/z` with no off-diagonal needed on either side. A tilt or a yaw σ is deliberately
  *not* emitted — PX4's `getTiltVariance` sums two NED variances where (36′) takes the larger of
  two body-frame ones, and naming those alike would compare two different quantities.
- **Two of five logs report no origin** (`xy_global` false, the reference fields all zero), so
  EKF2's `x,y,z` there are origin-relative with no origin and cannot be aligned to this filter's.
  The origin is a `#` header line, not a column, since it is one geodetic point per log.

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
filter rejects 278 of 609 solutions from, so its own bias wanders by ±0.01 rad/s and cannot
adjudicate anything.

Roll and pitch agree with EKF2 within 0.28° on all five logs at the end of the initialization
window, which is what confirms the quaternion convention, the ZYX order and the timebase
rebasing at once. **Absolute yaw does not compare at an instant** and should not be read as
divergence: EKF2 resets yaw in the first seconds — on `3949f175` it moves 41.60° to 16.66°
between t = 2 s and t = 4 s, which is what `ekf2_att`'s `att_reset` column is for — and the
residue after its reset is a declination difference, this harness overriding declination to a
fixed −0.06 rad while EKF2 reads the world magnetic model at an origin two corpus logs do not
have.

### Per-log reports

`tools/replay_report.py` renders one replay into a single self-contained HTML file: horizontal
track against EKF2 and the raw fixes, every state with its ±3σ band and `Status` shaded behind it,
all the σ on one log axis with GNSS gaps shaded, per-axis normalized innovations with the gate and
its rejections, the NIS histogram and QQ plot against χ², and the published keys.

```console
$ cargo run --release --example replay -- data/logs/<log-id>.csv target/replay.csv > target/<id>.summary
$ uv run tools/replay_report.py data/logs/<log-id>.csv target/replay.csv \
      --reference data/logs/<log-id>.reference.csv --summary target/<id>.summary \
      -o target/report/<log-id>.html
```

One HTML file with the PNGs inlined, and no JavaScript, for the same reason `README.md` carries no
mermaid: it renders wherever it lands. The 2 h log takes 16 s and produces 2.1 MB, decimating
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
numeric check a ceiling gets, so `none` cannot clear a bound the filter never met.

The keys are `rate=` and `window=` (the IMU rate and the samples it takes to cover
`min_duration`), `align=`, `an=` and `alpha0=` (what initialization achieved, whether a moving
window measured the vehicle's own acceleration from GNSS velocity — `ā_n` of equation (5′), which
only a moving start reports — and where the barometric reference came from: `window`, `estimate`
once a fix has established position, or `none`), `heading=` (`Validity::heading` **as initialization left it** — not as the
log ended, which would only restate `transitions=`), `resets=` (adoptions, per source, so a GNSS fix adopted whole counts in both its halves),
`aligned_at=` (seconds from the end of the window to the first epoch `Eskf::is_aligned` read true,
or `never`), `attitude_lost=` (seconds to the first epoch at or after it where `Validity::attitude`
read false against `Config::accuracy` — the mission's bar, where `aligned_at=` reads the fixed
alignment bars — which is where
the covariance growth of (16)–(22) shows up on logs with no truth — `Status::Aligning` latches, so
nothing else on the line moves when it happens), `r_policy=` (what the harness handed each
`fuse_*` as `R` — `raw` on every entry, and the paragraph below the caveats says why it is not a
floor), `rejected=` and `discarded=` (the gate's verdict, and everything that never reached it — a
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
  constant rather than the filter's tuning, and the corpus reads 0.0067–0.2093 and 0.0025–0.1984
  — conservative by 2.2–12.2× in σ for the barometer and 2.2–20.0× for the magnetometer, stated
  separately because one range across both understates the magnetometer's low end by two thirds.
  Nothing here had ever tested them; that is the first measurement
  [#50](https://github.com/wboayue/fusion-nav/issues/50) can argue from.
- For GNSS the statistic tests the receiver's own `eph`/`epv`/`s_variance_m_s`, which the harness
  passes through unfloored where both production estimators bound theirs. Horizontal position
  reads 0.0056, 0.0110 and 0.1221 and height 0.0048, 0.0547 and 0.1024, so those figures are
  wider than their residuals earn; `a299e722`'s velocity reads 7.5592, a receiver contradicting its
  own differenced positions.

**Unfloored is the policy, and it is deliberate.** Both production estimators bound a receiver's
reported accuracy before fusing, and this crate ships the same facility for an integrator who
wants it — `PositionNoise::clamped` and `VelocityNoise::clamped`, whose doc comments carry both
platforms' parameters, `file:line` and the shas they were read at. The harness applies neither,
for three measured reasons:

- **A floor erases the figure rather than bounding it.** PX4's `ekf2_gps_v_noise`, 0.5 m/s, sits
  above 4885 of the corpus's 5348 velocity solutions — every solution on two logs and 90 % on the
  third, honest receivers included — so it is the operative value rather than a backstop. The
  position floors are a measured no-op in the other direction: reported σ_h never falls below
  0.900 m against a 0.5 m bound, and σ_v averages 1.78–3.59 m against 0.75. Since the barometer
  and the magnetometer already carry converter constants, flooring would leave no
  receiver-reported variance anywhere in the corpus.
- **It costs most or all of the only rejection the corpus has.** `rejected_gnss_vel=278` on
  `a299e722` is the single non-zero count across five logs. Replayed with the floors applied it
  reads 0 under PX4's treatment — the 0.5 m/s floor *and* the separate `sq(1.5f)` vertical
  widening — 2 under that floor alone, and 44 under ArduPilot's per-axis 0.3/0.5, which is the one
  policy that would leave the gate of (37)–(38) still exercised by real data. `transitions=` goes
  4 to 2 under all three, so this is not the only key a floor would move.
- **It moves the accuracy gate.** `examples/simulate.rs` draws GNSS velocity noise at σ = 0.15 m/s
  and `data/bench.sh` scores every scenario through this same harness, so a floor would hand every
  simulated fix an `R` 11× too wide and move the ceilings in `data/scenarios.txt` — distrusting a
  receiver the simulator defines as honest.

The policy belongs to the corpus rather than to one log: a source added later reports its own
accuracy the same way and is fused the same way. What it costs is that a figure here is not
directly comparable with EKF2's on the same log, which fuses a floored `R` — a comparison states
that difference or matches the policy, and
[#8](https://github.com/wboayue/fusion-nav/issues/8) owns which. `r_policy=` on the `summary` line
carries the verdict, so a published figure travels with the policy that produced it.

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
