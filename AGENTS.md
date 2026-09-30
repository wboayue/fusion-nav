# AGENTS.md

This file provides guidance to coding agents working with code in this repository.

## Status

**`EQUATIONS.md` is implemented.** #31 closed with stage 9 (#110, #111): (1)–(44) are built
except (31)–(33), the three-axis magnetometer, which is out of scope, and (5′)'s subtraction,
which is #59's. The `**Stub.**` marker survives on those two and nowhere else, and every status
banner says built rather than intended. What is left is mostly measurement and publication —
#41 (cost on hardware) and #47 (the release); #8, the EKF2 comparison, landed in #156, and
#123, the validation report that publishes it, in #161. The defect the
consistency keys of #112 surfaced, no corpus source white while (24) fused each as white, is
#117's, and #152 answered it with equation (24′): every source is fused at
`R_m (1+ρ)/(1−ρ)`, `ρ = exp(−Δt/τ)`, gated on `R_m`, with `Δt` from the source's last fused
measurement and one `τ` per source in `Config::correlation` (corpus medians of `−T/ln acf1`
read as white). A posterior floor, the issue's option 3, was measured and lost: honest, but it
raised the gain. The `correlated` scenario's residual (`anees_pos` 1.45, sources slower than the
defaults) is #51's per-sensor τ to remove, and #117 is closed on that basis. Overconfidence is the
lockout precondition, and #116 (#143) recovers from lockout by default: per-source
`Config::recovery` at PX4's timeouts, `Recovery::OFF` byte-identical to the filter that only
reported, `recovered=` pinned on every scenario and log so a recovery masking #117 is a diff.
`093e806a` (29) and `7ce66f0d` (28) recover from causes recovery does not remove, and their
entries say so. #144 (#158) removed the lockout recovery was covering on `4b473e91`: a step past
`max_predict_dt` is coasted by equation (22′), position on `v̂ Δt` and `P` grown by
`Config::coast`'s acceleration and rotation densities, so the first fix after a gap is accepted
(`4b473e91` recovered 12 → 0, rejected 67 → 4; `logging_dropout` `pos_h_max` 23.25 → 2.74 m).
The rotation term is the corpus's finding, not the simulator's: the course turns 44° across
`4b473e91`'s 3.1 s gap, and without it 7 recoveries remain. The 4 rejections it left, fixes
timestamped inside a gap, are refused as `Fusion::OutOfHorizon` since #52: ahead of the state. #118 gated GNSS height apart from horizontal
position, which removed the lockout fusing that barometer caused. #119 (#122) estimates the
barometric offset beside the 15-state covariance, equation (30′), walking at
`Config::baro_offset_walk` (PX4's 0.13); GOALS' "Barometric reference as an estimated offset"
records why estimated rather than a consider state — 1.035 on `moving_start`, but 29310 barometer
rejections on `2c42096b`. #115 (#127) seeds that offset from the estimate on any start that
leaves no reference, correlated with the height it was read against (`P_xb = −P[:, D]`), behind
`Config::baro_reference_from_estimate`: `2c42096b` fused its barometer that way while it started
coarse (35575 rows refused → 4; 3825 rejected at `baro_offset_walk = 0`), and since it starts
`short` it reads `alpha0=window`, so `cd7e0001` and `7ce66f0d` are the logs that cover the seed;
`moving_start` `nees_pos` 1.0877.
#8 landed (#156): `data/fetch.sh --compare` replays every log `raw` and `px4` (`--r-policy px4`,
EKF2's own GNSS floors from the log's parameters) and asserts 26 `agreement` lines against
`data/ekf2.txt`; the statistics are `tools/agreement.py`'s, and `replay_report.py --corpus` draws
the table #123 consumes. On `2c42096b`, horizontal agrees to 0.30 / 0.27 m RMS north / east
(EKF2's origin sits 3.67 m S, 2.14 m E, 4.21 m below ours), and height does not: `climb`,
first 60 s mean to last, is +11.97 m for EKF2 on its barometer and −7.28 m for this filter on GNSS
height's low frequencies (`height_reference_ekf2=baro`). The 24.2 m origin gap once quoted was
mostly the geoid, the replay origin being ellipsoidal and `ref_alt` MSL; the converter now puts
both on one datum. The `R` policy is most of the rejection disagreement (`093e806a` 278.8 s raw,
27.1 px4, EKF2 304.9). `89a498ce`'s "EKF2 3.6 m north of its own fixes" was the converter (#167,
#145 section 4): PX4's local x/y are `MapProjection`, azimuthal equidistant on a 6371 km sphere,
not the tangent plane of (43), ~0.2 % apart in scale, so no origin shift aligns them. EKF2's
position is now reprojected per row about its own origin and placed as a fix is; raw `pos_n_rms`
4.67 → 0.130 m, and every log that flies far moved (`4b473e91` `pos_e_rms` 1.81 → 0.61).
The sensor-boundary pass landed (#159): `clamped` takes a `SigmaBounds` per axis, so PX4's and
ArduPilot's GNSS `R` rules are one call each and `RPolicy::Px4` holds no floor of its own (#157;
`px4` replay byte-identical on all twelve logs); declination moved off `Config` onto `Eskf`,
`set_magnetic_declination`, a site property like the origin (#125 part 1; the model landed in #170);
`Eskf::angular_rate` is the bias-corrected `ω`, committed with the state through `Propagated`, so a
caller can move the estimate to another point (the filter applies the antenna itself since #170); and #27's
two boundary notes. No corpus or scenario output moved.
#89 landed (#151): `data/anees.sh` gates per-epoch ensemble NEES on 50 seeds against χ² in CI,
and asserts failures by cause: `correlated` (#117's residual) on position; `gnss_latency`'s
assertion tripped when #52 landed, and it now passes all three blocks;
`logging_dropout`'s assertion tripped when #144 coasted its gap, and it now passes all three. Of the two it found that no single seed showed, #150
(`moving_start`'s first 0.2 s) was fixed by (24′), and #149 (`harsh_imu` attitude) by #160.
#160 made (8) correlate tilt with accelerometer bias, since (5) levels a biased accelerometer
(`P_θβa = −(σ_βa²/γ)[d̂]×`), raised `sigma_accel_bias` 0.1 → 0.2 (PX4's and ArduPilot's), and
floors the tilt the bias does not explain at the window's own scatter across gravity, without
which the defaults' `P₀` was singular. The evidence is the run that could fail, `harsh_imu`
fused white on 50 seeds: `any_att` 2281 → 0 (214 at the prior alone). Tilt tightened on every
scenario (`mission` 0.414° → 0.330); `7ce66f0d` recovers 28 times rather than 69.
#123 landed (#161): `VALIDATION.md` and `validation/{accuracy,honesty,robustness,ekf2}.md`, for
a reader who has never computed a NEES. Every number and table comes through a placeholder in
`validation/src/`, and every figure through `replay_report.py --figures`. `tools/validation.sh`
regenerates them from the gates it runs, and `--check` re-derives them. The pages state the
losses: `correlated` (#51) is overconfident and `7ce66f0d` levels wrong (#59). #47, the release,
is next: the *API frozen* milestone is closed. #174 became bad vertical-accelerometer detection,
internal and reported through `Diagnostics`, so it left the milestone (clipping, its first shape,
is 22 samples on the corpus); #41 needs a board.
#183 landed (#185): no public item names an `nalgebra` type, since it is 0.x and each minor is a
semver break. Vectors and noise cross as `[f32; 3]`, the covariance as rows (`from_rows`,
`to_rows`), and a quaternion as `Quaternion { w, x, y, z }`, at the root and out of the prelude.
The plan was a scalar-first `[f32; 4]`; review found glam's `to_array` and `nalgebra`'s
`From<[f32; 4]>` are scalar-last, so `.into()` compiled and seeded a finite, wrong attitude. An
array's order is a convention too, and a named field is the type that states it. `mint` was
deferred as additive. The constructors normalize prescaled by the largest component: `nalgebra`
alone turns one `f32::MAX` component into a finite zero, which a seed accepts. Every corpus log,
scenario and page was byte-identical.
#44 landed (#179), with the attitude renaming #47 asked for before the freeze. `Eskf::new` returns
`Result<_, ConfigError>`, and `Config::validate` destructures `Config` without `..`, so a new field
does not compile until it is bounded. The constructors are `Attitude::from_*`, and the getters carry
the bare frame-pair names. `src/eskf/adversarial.rs` is a proptest suite (`hostile`, and `ordinary`
with `floored == 0`) that checks the invariants on `[P, P_xb; P_xbᵀ, P_bb]` after every public
call. It found seven defects, all fixed and pinned by literals. Each one was a finite input the
arithmetic overflows (one `f32::MAX` reading, a rotated `f32::MAX` arm, a 1e20 rad/s gyro) or a
refusal that had already committed something. Only the (42) fix moved output: three corpus logs
in the fourth figure of their EKF2 agreement.
#50 landed (#178): `StaticWindow::noise(&init)` reports the white noise a still window measured,
per axis, as `WindowNoise`: a **floor** under `Config::imu` and a barometer's `R`, never applied,
and asked of the window so a `Config` can be derived before any filter exists. The IMU densities
are equation (8″), the weighted scatter of increments over 50 ms blocks (`WindowNoise::BLOCK`),
because real still windows are not white (lag-one −0.98 to +0.96 on the window rows) and one
sample's scatter misread the density 6× high to 2.9× low; a lag-one `(1+ρ)/(1−ρ)` correction
missed by up to 3.3×. The block length is the figure's real uncertainty, up to 2.3× between 50 and
125 ms, against ±11 % from 40 blocks. `WindowNoise::MIN_READINGS` (9) gates each sensor; `α₀`
keeps two, because (30′) refines it and holding it to nine left the corpus no real short still
start with its own reference. The floor is a floor on the corpus in both directions it could fail:
`ImuNoise::default` sits 9 to 180× above the gyroscope's and 6 to 230× above the accelerometer's,
and fusing the barometer's as `R` took `nis_baro` to 1.54 to 34.56 on five of six real logs.
`noise_gyro=`, `noise_accel=`, `noise_baro=` are pinned on all thirteen logs and bounded in
`data/scenarios.txt` about the simulator's injected densities, the one two-sided truth bound
there. `a299e722`'s rows average 2.5 ms while standing for 20 ms, so its figures read about √8
high and are left out of the quoted ranges until #177 carries the converter's integral intervals.
#60 landed (#180): UrbanNav Medium-Urban-1 is the gate benchmark, the one source with hostile GNSS
*and* truth. It states no licence, so `data/urbannav.txt` is its own manifest
(`data/fetch.sh --manifest`), nothing drawn from it is committed, and the pages publish scalars.
`tools/urbannav2replay.py` reads the IMU CSV, each u-blox `$PUBX,00` and SPAN-CPT text, no rosbag.
The harness judges each GNSS fix against truth (`Judged`: the gate's own test at P999 on the row's
`R`, fixed whatever the run's gates, policy or `--recovery`), and the `score` line carries
`bad_`, `rejected_bad_`, `rejected_good_`, `accepted_far_`, `adopted_bad_`,
`recovered_after_{bad,lockout}_` and `unjudged_` per half; truth may leave bias columns blank and
be sparser than the IMU. The F9P rejects none of its 654 good fixes. The M8T, 200–470 m out while
claiming 5–25 m, captures the filter: 32 of 338 bad fixes rejected, 59 good ones, 258 m RMS,
heading lost; 12 of 16 position recoveries end lockouts, and `--recovery off` is 61 km. No gate
percentile helps, so P999 and recovery-on stand; the persistent-error failure is #181.
#9 landed (#182): INSANE is the accuracy benchmark on a real UAV for position, height and
velocity, **not attitude**. `data/insane.txt` is its own manifest (BSD-2 with a no-Sell
condition and a citation requirement), three sequences, scalars published. The dataset's truth
pipeline fits attitude to the RTK baseline *and the PX4 magnetometer the filter fuses*: yaw truth
is the dual-RTK baseline, so `fuse_gnss_heading` cannot be scored there. At rest the truth tilts
gravity 5–17° from vertical, and its timeline lags the IMU by 80–170 ms, which
`tools/insane2replay.py` measures per sequence and removes. It fuses the PX4 receiver,
barometer and magnetometer (on its logged axes: the calibration's extrinsic made heading
worse), no GNSS velocity (horizontal only, no accuracy). The finding is (24′)'s: `nees_pos`
0.47 / 0.35 / 0.44, where fused white reads 39.2 / 11.6 / 16.1, at 0.05–0.37 m of horizontal
RMS over the fixes' own error. That comparison is hand-computed until #184 makes it a key.
`tools/replay_format.py` is the one replay/truth writer for all three converters,
`tools/rotations.py` their geometry, and `data/truth-runs.sh` the runner `urbannav.sh` and
`insane.sh` share.
#81, #125, #25 and #62 landed together (#170), the sensor boundary. The edge converts both ways
(`flu_to_enu`, `to_enu`, `to_flu`). The filter reads PX4's WMM table (`src/magnetic.rs`, the
`magnetic-model` feature, 2.5 KB) where it places its origin unless the caller set a declination,
and turns a heading only the magnetometer set; fixed epoch, since the table is within 0.26° and
drifts 0.42° in five years at the corpus origins, against a 3.1° heading σ (GOALS). Every GNSS
`fuse_*` takes `antenna: Position<Body>`, (28′) and (29′), with the arm's attitude and gyro-bias
terms in `H` where PX4 corrects the fix alone: `a299e722` px4 `pos_e_rms` to EKF2 0.650 → 0.408 m
(PX4's form 0.448), `cd7e0001` heading gap 1.61° → 0.81 (0.85). Four logs carry an arm, and replayed
`--antenna zero` each reproduces the old manifest; `eb799954` moves away from EKF2 (0.213 → 0.262 m),
unexplained. `lever_arm` is the one scenario whose fixes are not the IMU's. The README's "Coming
from PX4 or ArduPilot" maps every parameter; its audit found PX4 firmware's `EKF2_BARO_NOISE` is
3.5 m, that PX4 gates GNSS height apart too, and that commander's `COM_POS_FS_EPH`/`COM_VEL_FS_EVH`
are `Accuracy`'s counterparts. The review found a NaN `antenna` reaching the state on adoption and
(44) placing the origin before the declination turn; both are tested.
#21 and #52 landed together (#162), one time model for `predict` and `fuse_*`. `ImuSample` is
PX4's `imuSample`: a `Timestamp` (u64 µs), delta angle and delta velocity, each with its own
interval; `predict(imu)` differences timestamps for the step the timers and the gap test read,
and integrates each increment over its own interval. Every `fuse_*` takes the measurement's time,
and equation (23′) fuses it there: the innovation against `src/history.rs`, a 32-entry ring of
the nominal state with every correction shifted through it, and `H` carried to today's error by
`H(I − Aτ + ½A²τ²)` at the mean rates over the age. `gnss_latency` `pos_h` 2.197 → 0.295 m
(`mission` 0.292), flat across 50–300 ms and half to twice the speed. Extrapolating back from
the last IMU sample matched it in simulation and was refused on the corpus (`eb799954` 2 → 917
rejections: one sample's vibration). Fixes are dated by each log's `EKF2_GPS_DELAY` (`t_meas_s`),
which moves agreement with EKF2 closer on most logs; `a299e722` (266 → 396 rejections, 15.6 m
from EKF2 under raw `R`) and `7ce66f0d` (1828 → 1966) prefer no delay in a per-log sweep, and
both entries name causes latency does not touch. GOALS, "Measurement latency", owns the decision
and why not PX4's delayed horizon. The four-lens review of #162 added what the corpus could not
ask for: a measurement ahead of the state is carried forward on attitude (the last sample's rate)
as well as position, since a heading 100 ms ahead in a 90°/s turn was 9° off (five logs re-pinned
at noise level, all from this: zeroed, the old manifest passes); `OutOfHorizon { age }` names how
far out; a window whose timestamps do not run forward is `InitError::InvalidStep`; a measurement
dated before a start not at rest is refused, which no corpus log or scenario reaches; and
`ImuSample::accumulate` sums a driver's batch, coning left out and cited against PX4's
`ImuDownSampler`. `Eskf::commit_state` is the one writer that keeps the history in step, and a
test fails without it.
#24 and #53 landed together (#163): heading without a magnetometer, and the source set is 7.
`fuse_gnss_heading` is (35′), a dual-antenna true heading on (36)'s row with no (36′);
`fuse_course` is (35″), a *constraint* on the estimated velocity rather than a GNSS velocity
handed in, which would count the fix's cross-track error twice. Both run through one private
`Eskf::fuse_heading` with the magnetometer (`src/observation/heading.rs` holds (35′), (35″) and
the shared (36)), both adopt a first heading, and `Fusion::Unobservable` refuses body x within 30°
of vertical and a course under ArduPilot's 15° bar on `σ_χ`, so the speed threshold is the
velocity's own accuracy. A course with no fresh GNSS velocity is
`NoReference`, and `Status` does not count the course (`Diagnostics::aiding()`). `a299e722` turned
out to be a real dual-antenna log (`EKF2_AID_MASK` bit 7, 0.010 rad from EKF2's yaw at rest): the
converter writes `gnss_yaw` rows only where the log's own EKF2 enables GNSS yaw, and fusing them
took its heading gap to EKF2 4.86° → 1.03° and `nu_mag_yaw` −0.127 → −0.002 rad, while px4
`pos_e_rms` went 0.33 → 0.66 m, most of it the 0.30 m antenna lever arm #170 applied (0.41). #53's bar
was met on the simulator's `no_mag` (heading 0.8 s after takeoff, 1.82° against 1.83), which
measures the simulated sideslip as much as the filter; `093e806a --without mag --course 3` aligns
where it never did, and the VTOL `4b473e91` shows the multirotor case it is not for. GOALS records
option 6 (GSF) as not needed for option 5's vehicles.
#56 landed (#164): `Status` times each source against its own rate, and `DeadReckoning` is
horizontal. A source's timeout is 2.5 × `SourceHealth::period()`, a running mean of its arrival
intervals (15, then weight 1/15), or `dead_reckoning_after` before it has one, and uncapped after;
`degraded_after` is gone. `DeadReckoning` reads GNSS position and velocity alone
(`Diagnostics::horizontal()`), as PX4's `inertial_dead_reckoning` and ArduPilot's
`dead_reckoning` do, so a vehicle awaiting its first fix reads `DeadReckoning`, not `Aligning`.
A median was the plan and lost on the corpus: `eb799954`'s magnetometer bursts, and a median of
nine read its burst spacing as its rate (14025 transitions against 2). A neutral replay (2.5 s
fixed, the old any-source rule) reproduced every summary, and only `transitions=`/`status=` moved,
on six logs, each noted; `2c42096b` 888 → 48, since its mean interval is 1.54 s. The period
reaches the estimate through the recovery guards and the course's `NoReference`, which GOALS'
derived-configuration row records. `diagnostics()` and `sources()` return references
(`Eskf::state`'s frame 688 → 56 bytes on `thumbv6m`).
#155 landed (#166): the initialization window is folded in as it arrives. `StaticWindow` (936 B
on `thumbv6m` since #50's sums, where a buffered 2 s window at 400 Hz is 64 KB of 80-byte `StaticSample`s) takes
`push`, `try_extend` or `TryFrom<&[StaticSample]>`, refuses a sample with `SampleRefusal` and
leaves the window as it was, and answers `is_long_enough` and `is_at_rest` per sample;
`initialize` and `alignment_of` take it and can refuse only an empty one. `examples/embedded.rs`
collects at 400 Hz. The halves `window_drift` compares split at the nearest of 8 block
boundaries, since a stream does not know its middle: exact at 4096 blocks, every output is
byte-identical to main, and at 8 only `7ce66f0d` moves, in a last digit, while the drift itself
moves up to 4° (`BLOCKS` owns the table). That last figure is why the outputs alone were not the
evidence: the drift reaches `coarse_sigmas` only where it is the largest bound, so an unchanged
line could not have shown the approximation. The barometer's scatter is summed about the first
reading; the naive one-pass form is 5 × 10⁻⁴ out at 10 km, and the first fixture, 11 levels,
squared exactly and could not tell the two apart. `push`'s cost on a board is #41's.
#48 and #49 landed (#154): `Display` on every outcome, an optional `defmt` feature, and
`examples/embedded.rs`, built for both thumb targets in CI. `Display` prints numbers through
`src/display.rs`'s `Fixed`, because core's `f32` formatting reaches `core::panicking` (the
existing `InitError` impl did, unguarded). `panic-check` now formats every `Display` impl. #86's
tailsitter is no longer blocked: #131 (#133) reads tilt and heading on navigation axes, `diag(R P_θθ Rᵀ)` through
`AttitudeVariance`, in `Validity`, the latch, the heading adoption and (8)'s prior, and the
`tilt`/`yaw`/`false_valid` score keys moved with it — tilt² + yaw² unchanged, the body split had
booked heading error as tilt (`gnss_latency` `false_valid_att` 120 → 50). #134 (#135) made (36′)
axis-free: its tilt variance is the largest eigenvalue of the horizontal tilt block, not the larger
of two diagonals. The exact field-axis term `f̂ᵀPf̂` lost while headings were fused as white
(`gnss_outage` `pos_h` 2.630 m against 2.227), because consecutive headings share a tilt error;
under (24′) the two agree (1.255 against 1.252, #153), and the eigenvalue stays as the bound
rather than for a margin. Only `f16771dd` re-pinned (`nu_mag_yaw` −0.025485 → −0.024121).
#86's PR 2 (#141) made the corpus eight logs: `89a498ce` is the real baseline (RTK, 4.07 km, #124's
log), `eb799954` carries the first `rejected_mag` (3), and `cd7e0001` is the coarse start and the
first `floored=` (21, a raw 0.43 mm/s σ_v). `2c42096b` now starts `short` on a 0.80 s still prefix:
the harness waits `PATIENCE` for a static window and falls back to the prefix, and `classify`
measures motion before length. Declination is read per log (`declination=`), which took the median
heading gap to EKF2 on real GNSS logs from 4–13° to 0–2.5°. #142 closed #86 at twelve logs, the
first fast vehicles, each pinned with its cause named: `285ee2e7` (tailsitter, 125° with every
heading fused, the first `rejected_baro` at back-transitions), `4b473e91` (VTOL locked out after a
1.18 s logging dropout, #116's acceptance line), `7ce66f0d` (hand launch levelled 12° wrong, (5′)'s
first real check) and `093e806a` (fixed-wing, 1.14 km, a receiver PX4's R floors would correct).
Declination now follows EKF2's own rule, PX4's table at the first fix. What #86 left open is #145:
`9eb08bdb`'s unexplained position rejections and a heavy-lift log pairing vibration with a
magnetometer glitch. Its `2b2ad123` entered in #168, the thirteenth log: its old lockout was the
IMU gaps, which coasting removes, and it is the corpus's second RTK receiver, 5.13 km out. Its 18
raw rejections split evenly. Nine are right, fixes stepping metres beyond the receiver's own
velocity and returning, held through while EKF2 under its floor is pulled toward them. Nine are
this filter lagging a hard acceleration after a GNSS velocity rejection and refusing fixes EKF2
agrees with until recovery (#169). The review of #168 found the first draft had called all 18
right; comparing each refusal with EKF2 at the fix's own time is what split them.

**Every source the crate publishes is fused; no `fuse_*` is a stub.** Initialization is real —
equations (5)–(8), so the filter starts at the attitude and biases the window yields — `predict`
propagates the nominal state *and* its covariance, (9)–(22), and a GNSS position, a GNSS velocity,
a barometric altitude or a magnetic heading corrects both: the update of (23)–(27) in Joseph form,
the observation models (28), (29), (30) and (34)–(36) with the levelling variance (36′), the gate
of (37)–(38) and the injection and reset of (39)–(41) all exist, in `src/update.rs` and
`src/observation/{gnss,baro,mag}.rs`. So `mission` scores 0.244 m of horizontal RMSE, 0.187 m/s of
velocity and 0.249 m of height where dead reckoning scored 1261 — 0.083 before (30′), whose
walking offset hands the low frequencies to GNSS height, which a simulated barometer that never
drifts reads as pure loss (`data/scenarios.txt`).
The gate turns a fix down rather than taking everything offered — on the corpus, where
`a299e722` refuses 287 of its 609 velocity solutions, a receiver its own differenced positions
contradict (#105 settled that the harness does *not* floor `R` as both production estimators do,
and `r_policy=raw` pins that). The multirotors barely reach the other gates: the magnetometer
three times on `eb799954`, single samples 1.1 rad out, and never the barometer. The four
airframe logs of #86's PR 3 reach all of them, each for a cause its manifest entry names:
a dishonest receiver at speed, a logging dropout that locks a 30 m/s vehicle out (#116's
case), a hand launch levelled wrong ((5′)'s), and a tailsitter's back-transitions reaching the
barometer. Between
fixes — and on every axis a fix reaches only through the covariance — the estimate is still dead
reckoning, which is what `Validity` and `attitude_lost=` stay honest about.

**What (34)–(36) bought, and what (36′) cost to get it.** Yaw is the key that moved, and what a
heading buys is a property of the start rather than of the filter. Where the window never observed
yaw the magnetometer is the whole estimate: `moving_start` `yaw` 3.228° → 0.917 and `bg`
0.004853 → 0.002660, `static` 1.715 → 0.508. Where a static window already fixed yaw and the flight
is short it buys nothing and costs a little — `mission` `yaw` 0.644 → 0.651, `tilt` 0.519 → 0.576,
`bg` 0.000677 → 0.001013 — because a heading carrying 3.1° of noise has nothing to tell an
alignment that knew yaw to 0.6°, and its corrections reach tilt and gyroscope bias through (20).
`nees_att` rose on every line (`mission` 0.0353 → 0.2005) and is still under 1 everywhere: an
attitude that is finally observed approaches 1 from below, which `data/anees.txt` reads as a bound.

(36′) is what made the stage shippable rather than a refinement of it. Pricing the levelling error
into `R` — `tan²δ · σ_tilt²`, the `tan δ` of (8′) read off the field rather than configured — is
worth `moving_start` `tilt` 2.653° → 1.665, `yaw` 3.777 → 0.917, `nees_att` 1.634 → 0.226 and 840
falsely-valid attitude quantity-epochs → 0. It widens `S` without claiming the heading observes the
tilt that spoiled it — `R` inflation, which holds because velocity fusion keeps correcting the
tilt it prices, where an error that persists needs (30′)'s cross-covariance; the alternative was measured, and the exact Jacobian took `mission` to 9.0° of tilt.
The same `R` prices the **adoption**, where the tilt doing the levelling is worst — on
`moving_start` the adopted yaw variance is 0.582 rather than 0.01, σ = 0.76 rad against an
`Accuracy::heading` of 0.5236, so the heading is reported established and *invalid* rather than
established and believed. One line loosened on a claim rather than a number: `gnss_latency`
`false_valid` 340 → 413 and `false_valid_att` 57 → 120, the attitude covariance coming down where a
late fix leaves the error (removed by (23′), #52: both read 0).

**What (30) bought, and what it cost.** Height was the key that moved — `mission` `pos_v` 0.273 m →
0.083, `flight` 0.819 → 0.414 — and `moving_start` is byte-identical on all fifteen keys, because
a start in motion fixes no `α₀` and fuses no altitude. Two figures loosened and both are honest.
`gnss_outage`'s `pos_h_max` went 11.93 m → 29.69 while its RMSE barely moved: 20 s into the gap
`σ_pos_n` is 95 m against a barometer-held `σ_pos_d` of 0.14, so a height measurement's horizontal
gain is ~94× the correlation the two carry and the sensor's own noise arrives sideways in metres.
It is still worth having — the end-of-gap error is 1.39 m against 11.93 without it. And `static`'s
`nees_pos` went 1.06 → 2.04, more accurate and more overconfident in one diff, which is #89's case:
a 20 Hz barometer averages its noise down faster than the true error falls, and the accelerometer
bias walk is what stops the error following — or so it read; #119 showed the reference was most of
it, `static` 2.04 → 1.02 once the offset walks. `baro_drift` was the same at full size, `nees_pos`
257 → 1.19 under (30′). **What stage 9 added.** (42′), a per-group diagonal variance floor, applied at
`Eskf::commit_covariance` so the invariant belongs to the filter — *every covariance it commits
has been floored* — and reaches the ones no product built: an adoption, a `reset_*_to`, the (8) a
window commits. An honest source never reaches it, and that is measured: `floored=0` on all thirteen
corpus logs, 1.4 M epochs on the 2 h one. `cd7e0001`, whose receiver claims 0.43 mm/s after
touchdown and is fused raw, read 21 until (23′)'s carried-back `H` spread each velocity fix over
attitude and bias. `math.rs`'s `FLOOR` owns the headroom figures and every other mention cites it.
And `predicted_validity` stopped meaning *aiding is arriving*: `P` is projected
`Accuracy::horizon` forward with nothing fusing and each quantity tested at the far end, **or**
counted because a constraining source is being accepted. Tilt is what it bought — a static start
holds tilt 3.83 s, so a 1 s horizon arms and a 6 s one does not, where before it predicted its own
current value. `Accuracy::horizon` is the one knob no data could settle.

Also real: the health bookkeeping (timers,
`Status`, `Diagnostics`), the typed API surface, and the replay harness. The `**Stub.**` marker is
now a convention with two live users, both (5′)'s (`src/init.rs`) — keep it accurate, and keep the
status banners in `README.md`, `DESIGN.md`, `EQUATIONS.md` and `GOALS.md` saying what is true;
`src/lib.rs` inherits the README's, since it includes the file. `EQUATIONS.md`'s
is the one to watch: it sits above a mapping table that separately marks functions as unbuilt, so
the two can contradict each other, and has twice — the banner once claimed no implementation
existed while the table listed the geodetic origin as built, and stage 9's own first draft added
(42′) to the table while leaving the banner describing symmetry alone. The example module docs
carried the same rot: `basic.rs` and `degradation.rs` still told a reader heading and velocity
accepted without correcting, two stages after they stopped. `GLOSSARY.md`
repeats no caveat by design — it defines terms and defers status to the document that owns it —
but a few entries do name what is unbuilt (the GSF yaw estimator), and those are on the
list.

## Backlog

One tracking issue is still open and owns ordering for its area; read it before starting work
there.

- **#10** — replay validation. Carries which issue answers which question, and the measured
  numbers those answers are argued from. The questions themselves — self-consistency without
  truth, accuracy with it, a correct rejection needing truth *and* hostile measurements, and the
  covariance's own honesty underneath all three — are `GOALS.md`, "Three questions, three kinds
  of source".

**#31 is closed**, having landed all nine stages of `EQUATIONS.md` (#32–#40). Its closing comment
carries what stage 9 left; *why* the order was what it was stayed in `DESIGN.md`, "Staging the
implementation", which is the point of putting reasoning somewhere a tracker's closure cannot take
it. What is left in that area is measurement and publication rather than mathematics.

**A merge is not finished until the issues it falsified are updated.** A tracker is not a
changelog: it carries ordering, what each stage leaves its successors, and the measured numbers a
later decision gets argued from, so landed work makes some of it false rather than merely
incomplete. Nor is it the home for reasoning that outlives it — a tracker closes and takes
whatever is only written there with it, which is why the two above cite `DESIGN.md` and `GOALS.md`
for *why* rather than restating it. The update immediately follows the merge rather than riding in
the PR: an issue body is not code, it has no review to pass and no branch to rebase, and the
figures it should quote are the ones the merged run printed. The closing keyword is the easy half;
the rest is the tracker covering the area and every issue whose *premise* moved. #97 is the
measure of that cost: afterwards #31 and #10 still called #77 and #85 the next work, #86 still
claimed to block a number that had shipped on harness fixtures instead, #89 still said these
scenarios could not fail overconfident while `moving_start` was reading `nees_pos=10.20` on a
committed line, #59 still argued from a tilt prior that no longer exists, and this file's own
`Status` bullet had inverted. Six documents wrong from one merge, each reading as evidence until
someone checked. Quote what the run printed, not the ceiling: `data/scenarios.txt` carries 1 % of
margin, so a figure copied out of it is wrong in the direction of flattery.

Issues here are worth the length they run to. The convention: cite `file:line` at a pinned
upstream revision rather than paraphrasing PX4 or ArduPilot, state what it depends on and what it
blocks, name the replay/manifest impact, and say what the data must say before the issue can close.
Each carries an area label (`equations`, `validation`, `api`, `perf`, `docs`, `tooling`, `ci`) and
one of three milestones — *API frozen*, *Equations implemented and scored*, *Measured and
published*. Trackers carry the area label but no milestone, since they span all three.

**When looking for gaps, audit `GOALS.md` rather than the issue list.** The six differentiators,
the two open design questions and the derived-configuration table are commitments, and a
commitment with no issue against it is the gap — differentiator 1, the one GOALS calls most
defensible, had zero representation in the backlog until #41 and #42.

Differentiators are cited **by number** here, in issue bodies and in `data/manifest.txt`, so
`GOALS.md` numbers them 1–4, 6, 7: 5 was ecosystem coherence, dropped in #63, and the gap stays.
Never renumber them. A renumber silently repoints every citation, including closed issues that
cannot be corrected.

**Nothing is released, so nothing is a break.** The crate is `0.0.0` and has never been
published; no integrator holds a struct literal, a match arm or a signature that a change could
break. Design each type for the most usable, idiomatic shape it can have — rename, restructure,
add fields and variants, change signatures — and never pick a worse API to avoid a break, bundle
changes to "take the break once", or defer an API improvement to a version. Compatibility starts
at #47's first published version, and the attributes and semver policy exist to protect *that*
surface, not this one.

Put a query on the type whose data it reads. `Eskf::noise_of` read nothing of the filter but its
`Initialization`, and its purpose, deriving a `Config`, comes before any filter exists; it
became `StaticWindow::noise(&init)`, beside `is_at_rest(&init)` (#178). A method that forwards
one field of `self` to another type is the sign.

**Sequencing hazard, and what it taught:** #31's stages were stacked branches while the
signature-changing issues (#21, #25) changed the API underneath them, so an API change had to land
*before* the stage that built on it. The hazard is rebase cost between branches, not users: the
stages are done and #21 and #25 have landed, but a branch stacked on a surface
another branch is changing still inherits it. It
held throughout: #58 landed before stage 5, the first code to read
`Config::gates`, so the gate reads a `Gate<M>` typed by its degrees of freedom rather than a bare
`f32`, and stage 5 then decided the default percentile from replay (`P999`) in the diff that first
turned a fix down at all — the corpus stayed at `rejected=0` on all five logs until (29) reached a
receiver whose velocity it refuses; #61 landed before stage 2, so a quaternion reaches `Attitude` only through
a constructor naming its convention (`from_body_to_ned`, `from_ned_to_body`, `from_flu_to_enu`, `from_flu_to_nwu`) and
the `q̂₀` of (5)–(7) is committed through the final shape; #59's signature landed with it, so
`StaticSample` carries GNSS velocity and `Coarse::NotStationary` reports `ā_n`. What is left of #59
is equation (5′), and the attitude it needs to rotate `ā_n` into body axes now exists.

## Goal: a reference to learn from

Clarity and readability are goals, not side effects. This crate should work as a reference
implementation someone can learn an ESKF from by reading it — GOALS.md differentiator 3,
"Readable mathematics", is the contrast with PX4's generated code. In practice:

- Prefer the obvious form of an equation over a clever or fused one; name variables after the
  symbols in `EQUATIONS.md` and cite the equation number.
- A reader should follow a function top to bottom without jumping files. Split for meaning,
  not line count.
- Explain *why* in doc comments (the derivation, the source, the evidence), not *what* the next
  line does.
- When performance and readability conflict, keep the readable version unless a measured cost
  says otherwise, and record the measurement where the trade was made.

### What a doc comment owes the reader

Doc comments are the reference half of "reference implementation": `EQUATIONS.md` holds the
mathematics, the comment holds why *this* code and not the obvious alternative. Length is earned
per question answered, never per importance.

- **Summary line first, and standing alone.** One indicative sentence naming what the item does
  ("Fuse magnetic heading from a calibrated three-axis magnetometer"), then the equation numbers.
  Rustdoc prints that line by itself in the index, so it cannot lean on the paragraph below it.
- **Then one paragraph per question a reader would actually ask.** Three earn their space: *why
  this and not the obvious alternative* (`Fusion::Reset` takes the zero-information limit exactly
  rather than approaching it with an invented variance), *what breaks otherwise* (`InvalidNoise`:
  a negative variance written into `P` reads back as an excellent estimate), *where the number
  came from* (`SourceHealth::timeout`: 2826 status flaps at two periods on `eb799954`, 2 at
  two and a half). A
  paragraph that is none of the three, or that the signature already answers, is cut.
- **Cite instead of restating.** An equation number, a `file:line` at a pinned PX4/ArduPilot
  revision, a GOALS.md differentiator by number, a measured figure. A citation is what lets a
  short sentence be checked; paraphrase is what makes a long one rot.
- **Derivations stay in `EQUATIONS.md`.** The comment says which equation, and where the code
  departs from it — saturation, ordering, refusal. Never re-derives. This is the crate's main
  concision lever: the reader who wants the algebra has somewhere to go.
- **Write for a reader with the equations open and no git history.** No "previously", "now also",
  "changed to"; present tense about present code. History belongs in the commit message. A
  rejected value and what it measured is evidence, not history: "at 1.0 s the status flaps 76
  times in 124 s" stays, "the previous default was 1.0 s" goes.
- **Inline `//` justifies the line beneath it** — an ordering constraint, a tolerance, the
  subtraction that needed f64 — in one or two sentences. Narration of what the next line does is
  deleted; anything longer is a doc comment or a `DESIGN.md` paragraph.
- **A stale *why* is worse than none**, because it reads as evidence. Evidence in a comment (a
  corpus count, a PX4 default, a `**Stub.**` marker) moves in the commit that moves the thing it
  describes.

## Commands

```bash
cargo test --all-targets          # unit tests: inline `mod tests` across src/, and in examples/replay.rs
cargo test --doc                  # README.md (included by lib.rs), plus the item doctests
cargo test --lib eskf::tests::a_gap_is_coasted_on_the_estimated_velocity_and_the_time_still_passes  # one test
cargo fmt --all -- --check
cargo clippy --all-targets --no-deps   # CI runs with RUSTFLAGS=-D warnings
cargo build --lib --target thumbv7em-none-eabihf   # also thumbv6m-none-eabi; both gate CI
panic-check/run.sh                # no reachable panic, both thumb targets; needs llvm-tools
tools/check-anchors.sh            # every `.md#anchor` resolves; --self-test runs its fixtures
cargo +1.89 build --lib           # MSRV

# Stack frame per function, which `propagate_covariance` and `enforce_symmetry` both cite a
# measured figure from. Needs `rustup target add --toolchain nightly <target>` once.
# llvm-readobj is not on PATH here: it ships with rustup's llvm-tools, at
# ~/.rustup/toolchains/*/lib/rustlib/*/bin/llvm-readobj.
RUSTFLAGS=-Zemit-stack-sizes cargo +nightly build --lib --release --target thumbv6m-none-eabi
llvm-readobj --stack-sizes target/thumbv6m-none-eabi/release/libfusion_nav.rlib | grep -A1 predict
# When a frame grows, bisect it: strip one candidate per build and re-measure. #122's +4168
# bytes on `update::<3>` was not only the 16 x 16 it looked like -- returning `(Covariance,
# Offset)` as a tuple through `reset` cost a 900-byte copy of P on its own.

cargo run --example basic         # minimal integration loop
cargo run --example degradation   # timeouts, status transitions, application-driven recovery
cargo build --example embedded --target thumbv7em-none-eabihf [--features defmt]   # no_std; CI builds, never runs
cargo run --example replay        # replays data/flight.csv -> target/replay.csv (CI smoke test)
cargo run --example replay -- <input.csv> <output.csv> [truth.csv]   # truth adds a `score` line
cargo run --example replay -- data/flight.csv target/replay.csv data/flight.truth.csv

cargo run --example simulate      # seeded flights with truth -> target/sim/<scenario>{,.truth}.csv
cargo run --example simulate -- flight data   # regenerate the committed data/flight.csv

data/bench.sh                     # score every scenario against data/scenarios.txt; a CI gate
data/bench.sh mission static      # only these
data/expect.sh --self-test        # the comparator both bench.sh and the manifest rules read
cargo test --lib adversarial      # the proptest suite of #44, seeded; ~6 s in debug
data/anees.sh                     # every scenario on 50 seeds against data/anees.txt; a CI gate
python3 tools/anees.py --self-test   # the ensemble aggregator's fixtures (stdlib, no uv)
uv run tools/agreement.py --self-test    # agreement with EKF2; replay_report's self-test runs it too
tools/validation.sh               # regenerate VALIDATION.md, validation/*.md and their figures; local
tools/validation.sh --check       # fail if a committed page is not what a fresh run renders
tools/validation.sh --allow-dirty # rewrite from an uncommitted tree, while editing templates
python3 tools/validation.py --self-test   # the page renderer's fixtures
# Regenerate on a clean tree: the stamp is `git describe --dirty` taken at the start. So commit
# first, run it, and commit the result. That diff should be the stamp lines alone, since the PNGs
# are byte-identical on one machine; any other line that moved is a number that moved.
```

## Replay corpus

Real PX4 logs are fetched, not committed (`data/logs/` is gitignored). `data/manifest.txt`
pins each by sha256 *and* by expectations (`rate=`, `window=`, `refused=`, `transitions=`,
`status=`) matched against the `summary` line `examples/replay.rs` prints.

```bash
data/fetch.sh --venv              # once, for --check; .venv is gitignored and found automatically
                                  # installs the pin in tools/ulog2replay.py, which --check asserts
data/fetch.sh                     # fetch + verify the manifest
data/fetch.sh --verify            # checksums only, no network
data/fetch.sh --check             # convert each .ulg and replay it, assert expectations
data/fetch.sh --add <url> [name]  # download once, append a manifest line to commit
data/fetch.sh --pin <name>        # replay one log, print the expectations to append
data/fetch.sh --compare [--pin]   # every log beside EKF2, raw and px4, against data/ekf2.txt
data/fetch.sh --manifest data/urbannav.txt   # UrbanNav's segment into data/urbannav; no licence, never commit
data/urbannav.sh [--pin]          # both receivers against truth, raw and px4 (M8T also recovery off)
uv run tools/urbannav2replay.py --self-test   # the UrbanNav converter's fixtures (stdlib)
data/fetch.sh --manifest data/insane.txt     # INSANE's three sequences into data/insane; never commit
data/insane.sh [--pin]            # each sequence against truth, raw, --declination model
uv run tools/insane2replay.py --self-test     # the INSANE converter's fixtures (stdlib)
uv run tools/ulog2replay.py log.ulg --screen   # what a candidate could cover; data/README.md
uv run tools/ulog2replay.py log.ulg -o log.csv [--reference]   # ULog -> replay CSV
uv run tools/replay_report.py in.csv out.csv [truth.csv] \
    --reference ref.csv --summary summary.txt -o report.html   # one HTML per log
```

`--reference` writes EKF2's own solution beside the replay input, never into it. Its state/covariance
index map is keyed on the pair `(n_states, covariance entries)`, because EKF2's covariance layout
changed three times and neither count alone separates the eras: `n_states=24` is both the
state-indexed layout and v1.15's error-state one, and no field spelling distinguishes them either.
Three of the thirteen corpus logs therefore supply no attitude σ at all, two supply no origin, and the
LPE log's rows are LPE's, which its `Estimator:` header line says. EKF2's position arrives already
in the replay frame, reprojected out of PX4's spherical `MapProjection` and moved from MSL onto the
ellipsoidal datum the replay origin is on; `EKF2 position in replay frame:` names the axes placed.
`data/README.md`, "What `--reference` writes, and what it cannot", owns
those boundaries and the bias-scaling factor; the map itself lives in `tools/ulog2replay.py` and
nowhere else, so a consumer reads column names and never the layout.

**A ULog field name does not pin its meaning.** A ULog file records field names and types, never
enum constants, so a renumbered enum reads the same as the old one. `vehicle_status.vehicle_type`
is the case the corpus already holds. PX4 `7cb6464cfb` renumbered it to rotary wing 0 and fixed
wing 1, and `a150fc05af` (v1.16.0-rc2) and `50626f6848` (main) put back 1 and 2.
`3949f175` was built inside that window (`v1.16.0-rc1-154`) and logs a quadrotor as 0, while its
`ver_sw_release` reads as a v1.16.0 build, so a mapping keyed on the version misreads it too.
Before the converter reads a numeric enum, check the `.msg` history in `~/projects/PX4-Autopilot`.
Prefer a field whose values an outside standard fixes, such as MAVLink's `MAV_VTOL_STATE`, or one
whose name changed when its meaning did. #129 applies this to flight mode.

The report tool computes no statistic: it plots the per-fusion rows and prints the `summary` and
`score` keys, and the agreement-with-EKF2 table `tools/agreement.py` returns. It refuses a set of files that do not describe one run — `epochs=` against the epoch
row count, `rejected_<source>=` against the fusion CSV's tally, the source `.ulg` both the
reference and the replay input name, the scenario and seed in a truth file's header, and the
reference's IMU interval against `rate=` — that last one guarding the only quantity the converter
and the harness both estimate.

**`uv` is the package manager and the runner for the Python tools under `tools/`.** The converter
declares `pyulog` inline (PEP 723), so `uv run tools/ulog2replay.py` resolves it with no
virtualenv to create or keep current — a tool run a few times a year is the one whose setup
instructions rot. `data/fetch.sh` cannot use `uv run` (it invokes a single interpreter), so it
takes `.venv/bin/python` when that exists and honours `PYTHON=` otherwise, and owns
`--venv` so that the one documented way to create that interpreter installs the pin
rather than whatever is current. Nothing here is on the
Rust test path.

Both paths produce byte-identical CSVs **for the same pyulog**, which is why the inline
dependency is pinned to an exact version and `--check` refuses to run against an interpreter
holding a different one. Unpinned, `uv run` would take whatever pyulog is current while `.venv`
kept whatever was installed, and a change in its dropout merging or field exposure would move the
`summary` expectations with no diff anywhere in the repository — the one way corpus output can
move without a commit to point at. The pin lives in `tools/ulog2replay.py` and nowhere else;
`fetch.sh` parses it for both `--venv` and `--check`, and the converter reads its own header
back for the remedy it prints, so moving it is one edit and the version appears in no prose
and in no instruction.

`--check` is a **local** tool, run before a release or after touching the converter, replay
example, or any default it asserts. Converters stay out of the test path on purpose (GOALS.md,
"Harness constraint"): CI must need no network, no PX4 tooling, and no hardware, so it replays
the checked-in synthetic `data/flight.csv` only. Never add a pyulog or ROS dependency to the
Rust test path.

CI also replays `flight.csv` on an x86-64 and an aarch64 runner and compares a sha256 of the
output. It pins nothing about the *content* — only that the content does not depend on the host,
which is what makes #32's before/after diff of `target/replay.csv` mean anything: a diff is the
change, not the laptop. `data/README.md` owns the verdict and its boundaries (a rustc or `libm`
upgrade may legitimately move the hash; the fixed-precision CSV hides a last-bit difference).
Keep the filter's transcendentals on the `libm` crate — enabling `nalgebra`'s `std` feature would
swap them for a platform libm, which is free to differ in the last bit on `sin` or `atan2`, and
the property is gone.

Each manifest entry exists because it covers something nothing else does (SD-card dropouts
driving a coast, burst logging that forced a median rate estimator, a 2 h log guarding the
f64 timestamp parse, old field spellings). Adding or dropping a log means saying which behavior
it uniquely covers — and invalidates every sentence that quantifies the corpus. "Four of the five
logs", "the worst ordinary interval is 65 ms", "rates from 50 Hz to 400 Hz": these sit in doc
comments and `GOALS.md`, nothing pins them, and by the time anyone re-derived them two were wrong
and one described a corpus of a different size. Grep for `five logs`, `corpus` and `manifest.txt`,
and re-derive from `data/logs/*.csv`, before committing a new entry. Changing a `Config` default or the `summary` line requires updating the
affected expectations.

The same reasoning makes lost coverage a cost when no number worsens. #50 measured holding the
barometric reference to nine readings: no rejection or transition count moved, and it would have
left the corpus no real vehicle whose short still start sets its own reference, the claim #85 and
#97 had shipped on harness fixtures and `2c42096b` then showed on a real vehicle. The rule was
kept at two for that. When a change moves a manifest note's "what nothing else covers" line,
that is the finding to weigh, not only the moved keys.

Expectations are matched pair by pair as substrings, so **adding** a key to the `summary` line is
safe and pinning new behavior there is cheap — `align=`, `resets=`, `alpha0=` and `heading=` were
added that way, and each now guards a decision that would otherwise rot into a comment (`alpha0=`
caught the coarse log's 35575 barometer rows going from fused to `NoReference`, which no other key
noticed, and now says where each log's reference came from — `window`, `estimate` or `none`; `heading=` is the validity verdict on the initialization window, which catches a yaw
reported valid that no magnetometer ever observed — taken at the end of the log it would only
restate `transitions=`). `floored=` is the odd one: it pins behaviour that must
**not** happen, a count of variances raised to the floor of (42′) that reads zero on every log, and
nothing else on the line would notice if it started — a floored variance only makes the estimate
more conservative, so it moves neither `rejected=` nor `transitions=`. A key whose interesting
value is the one it does not have still earns its place. `r_policy=` is odder still and earns it
differently: a choice rather than a count, `raw` on every manifest entry and `px4` only under
`--r-policy px4`, so it cannot regress and fixtures in `examples/replay.rs` carry the guard
instead. What it buys is that a published figure names the `R` policy that produced it, which the
comparison against EKF2 (`data/ekf2.txt`) runs under both.
Renaming or removing a key breaks every
entry at once.

**One manifest per licence.** The PX4 logs are CC BY 4.0 and could be redistributed;
they are fetched rather than committed for size, not for terms. INSANE is BSD-2 with a
condition forbidding sale of what derives from it, so it cannot be bundled into an MIT crate at
all: `data/insane.txt`, fetched only when named (GOALS.md, Validation). UrbanNav states no licence at all,
which is stricter still: `data/urbannav.txt`, fetched only when named, and only scalars
published. Adding a data source means saying which behavior it uniquely covers **and** under
what licence — and for a restricted one, that measured scalars are publishable while converted
CSVs and plots stay out of the repository.

**One corpus is generated rather than fetched.** `examples/simulate.rs` writes seeded flights
with analytic truth, so it needs no licence, no manifest and no network — and it is the only
source that can say how *accurate* the filter is rather than how self-consistent. The same rule
still applies: a scenario exists because it covers something no other one does, and it says so in
the `covers` field of the table in that file, which is the single place the list lives. Its noise
tables are deliberately not `Config`'s; matching them would score the filter against its own
assumptions. `data/flight.csv` is the `flight` scenario's committed output, with
`data/flight.truth.csv` beside it — regenerate it with the command above, never hand-edit it. Its contents reach the
determinism hash through every column now that (9)–(15) propagate: the window still sets where
the attitude and gyroscope-bias columns start, and the rest of the file is a dead-reckoned
trajectory rather than a constant. So a change anywhere in propagation moves the hash, which is
what makes the byte-for-byte diff of `target/replay.csv` the reviewable artifact it was written to
be — before stage 3 it could only see the window.

**Scenario ceilings ratchet, in both directions.** `data/scenarios.txt` holds a measured ceiling
per `score` key per scenario and `data/bench.sh` asserts them in CI, which is the only gate in the
repository that reads *accuracy* rather than self-consistency. Same discipline as the manifest: a
number that moves needs a sentence saying what the data said, and the whole `score` line is
printed on a breach so re-measuring is a copy rather than a second run. Tightening is the usual
direction as each stage of #31 improves on the filter the ceilings were taken from, but a
loosening is honest where the equation that landed is the reason — `static`'s position ceilings
were exactly zero while nothing propagated, and stage 3 raised them to what dead reckoning on a
stationary vehicle actually drifts. What a ceiling cannot do is
test the covariance's own promise: it passes a filter that grew more accurate and more
overconfident at once. `data/anees.sh` is that test (each scenario on 50 seeds, NEES averaged
per epoch against a chi-square bound, `data/anees.txt`), and its bounds are quantiles rather than
measurements, so they do not ratchet. A scenario the filter fails asserts the failure with its
cause named, so fixing the cause trips the line.

The comparison itself is shared, not copied: `data/expect.sh` owns `key=value`, `key<=value`,
`key>=value` and `key=lo..hi`, and both readers source it — `data/bench.sh` for
`data/scenarios.txt`, `data/fetch.sh --check` for `data/manifest.txt`. So the pair syntax is one
language, and the two-sided bound a statistic wants landed as one arm in one `case` rather than as
a second comparator, which `data/anees.sh` also reads `data/anees.txt` with. It carries fixtures with
literal verdicts for the same reason `examples/replay.rs` does — the expectations in both files
were produced by the harness they guard, so a comparator that waves something through turns a
miscount into the baseline everything later is measured against.

Two things it refuses that a substring match does not: a key named in the expectations and absent
from the line, which is what notices a renamed `summary` or `score` key, and a value that is not a
number where one is required — `nees_pos=none` is what the harness prints when a block does not
invert, and awk reads it as zero, which passes every ceiling.

**Ceilings carry 1 % of margin**, `scored` excepted, and the header of `data/scenarios.txt` says
why: the simulator is a host tool on `std`'s transcendentals rather than the `libm` crate that
pins the filter, and since (9)–(15) every figure is an integral over tens of thousands of steps,
where a last-bit difference accumulates to roughly `n·eps`. Re-measure with the margin, never
without it, and never widen it to make a breach go away — that is the change needing a sentence.

**Converter changes are batched.** Regenerating the corpus is not free — logs fetched, `pyulog`
installed, every log reconverted, replayed, and every moved expectation explained. Land changes
that move `tools/ulog2replay.py` output together, with one `--check` run and one manifest diff
naming, per moved expectation, which change moved it. Three separate diffs each obscure the last.

The harness has its own `#[cfg(test)] mod tests`, and the reason is that the manifest alone is a
circular guard: its expectations were produced by the harness they are meant to protect, so a
miscount becomes a pinned number and then the baseline every later change is measured against.
The tests cover what the corpus cannot check about itself — the median rate estimator on a burst
and a dropout, the `f64` timestamp parse past the `f32` mantissa, the counters, and each verdict
key — against fixtures whose answer is known by construction. Keep them fixtures: the builder
there emits rows and never a count, a rate or a verdict, so the expected values stay literals
beside their assertions.

**A dev-dependency's features reach every test and example build.** Cargo unifies features across
the graph, and examples take dev-dependencies. proptest's `std` turns on `num-traits/std`, which
switched nalgebra from `libm` to the platform's transcendentals in the harness, and moved
`flight.csv`, the corpus and the validation pages in the last digit with no `src/` line changed.
`cargo tree -e features -i num-traits` is the check. A new dev-dependency is also built for the
thumb targets through `examples/embedded.rs`, hence proptest's `cfg(not(target_os = "none"))`
table. And `flight.csv` alone is not a neutral replay: #179's (42) fix left it byte-identical
and still moved three corpus logs, which only `tools/validation.sh --check` noticed.

**A test of a guard is worth mutating.** Break the guard, run the test, and see it fail before
believing it. A passing test proves nothing about a guard that was never exercised, and the
failure mode is not hypothetical: `data/expect.sh`'s fixture for pathname expansion passed against
code with the protection removed, because with one matching file both the line and the
expectations expand the same way and the bug cancels itself out. Two files is what made it bite.
The same pass caught a prefix match that read `pos_h_max` where `pos_h` was asked for, and a
missing numeric check that let `nees_pos=none` read as zero and clear every ceiling. Each took one
`sed` and one run. Where the mutation is not obvious, the fixture's comment says which one it
survives — that is the sentence a later reader needs, not the assertion, which they can see.
Commit the test before mutating, and restore with `git checkout HEAD -- <file>`: restoring an
uncommitted file wipes the test along with the mutation, and every later mutation then "passes"
a test that no longer exists — which is how #122's first mutation pass reported three survivors.

A fixture can fail to bite through symmetry as well as through a cancelling bug. The simulator's
IMU axes are identical, so a harness reporting the best axis where the worst was asked for passed
every scenario bound (#178); an asymmetric unit fixture, the worst axis in the middle, is what
kills it. And a kill that rests on an exact floating-point tie, `0.025 × 2 == 0.05` in `f32`, is
a property of the representation: pick values no boundary lands on.

**An artifact nobody looked at is unverified, whatever the pipeline says.** The rule above, one
level out: a passing test says nothing about a guard never exercised, and a passing pipeline says
nothing about an output nobody opened. `tools/replay_report.py` landed with every guard
mutation-tested, `fetch.sh --check` green on all five logs and CI green on nine jobs, and four of
its figures were wrong — a track built by pairing two columns decimated independently, so 3217 of
3994 buckets drew a position the vehicle never held; a gap detector fed decimated timestamps,
shading 6700 s of a 7127 s log as outage; a caption reporting a count filled in during a draw that
had not happened, since a caption is an argument; and an attitude σ panel labelled in degrees
while plotting radians. None of it errored. A wrong plot renders, sizes and times exactly like a
right one, so **the only detector is opening it** — one figure per section, on a log that
exercises that section, before the work is called done.

Opening them for #123 found three more, and all three were ways a figure can disagree with a
number printed beside it:
- **A figure must draw in the frame its statistic compares in.** The `4b473e91` track drew EKF2 at
  its own origin, so it looked 60 m from this filter while `pos_e_rms` read 2.7 m. The converter
  now writes EKF2's position in the replay frame, so neither the statistic nor the figure shifts
  anything.
- **A frame is a projection, not only an origin.** One origin shift aligned EKF2 at its origin and
  nowhere else: PX4's local x/y are on a sphere, ours on the ellipsoid's tangent plane, and the
  0.2 % scale between them read as EKF2 sitting 3.6 m north of its own RTK fixes on `89a498ce`,
  published as open for a release. EKF2's own innovations (millimetres) said it sat on its fixes;
  checking them first would have named the cause. Before calling another estimator's disagreement
  with its own sensor a finding, read that estimator's innovations for the sensor.
- **An axis sized to the widest band hides the error.** `gnss_outage`'s ±250 m band drew a 10 m
  error as a flat line.
- **Prose written before the figure is opened is a guess.** The robustness page said the outage
  reached `DeadReckoning`; the shading showed `Degraded`, because any accepted source then counted as
  aiding. #56 changed that, and the page was rewritten looking at the new shading.

Write the sentence about a figure while looking at it, and check a surprising pixel against the
rows before claiming it. `gnss_outage`'s error looked outside its band after the gap; the CSV put
it at −2.37 m inside a 3σ of 2.63.

**An issue's "what the data must say" was written before the data.** Meet it, then measure the
alternatives it did not propose, on the corpus as well as the simulator. #119 asked for a consider
state and listed its success criteria; the consider state met them — `moving_start` `nees_pos`
112.59 → 1.035 — and on `2c42096b`, the one real barometer that drifts, it rejected 29310
barometer readings. The simulator's barometer never drifts, so no scenario could have said so. An
estimated offset shipped instead (#122), and GOALS records both measurements. The same run showed
a criterion that could not fail: once a second height source is fused, `σ_pos_d` under the
receiver's `epv` is legitimate, so the 4603-of-4604 count #117 was to close on reads the same
whether or not the covariance is honest.

**Evidence has to be able to come out the other way.** Before a number is offered as confirmation,
ask what it would read if the thing were broken. The corrected EKF2 bias scaling was argued from
35583 samples on `2c42096b` agreeing to 5.3e-4 rad/s — and that log's update period moved 0.5 %
under the correction, so the figure was identical either way and could not have caught the 20 %
error it was cited as ruling out. `f16771dd`, where the period moved 12 ms → 10 ms, was the log
that could have. A statistic that reads the same whether or not the code is right is not weak
evidence, it is none, and quoting it is worse than quoting nothing because it reads as checked.

**A simulator's IMU cannot judge a method that reads one sample.** Its noise is ~58 times under
`ImuNoise`'s defaults and it has no vibration, so a method that uses the last IMU sample and one
that averages across many score alike on every scenario. #52's extrapolation back on `v − a_n τ`
matched the state history everywhere in simulation, and on the corpus took `eb799954` from 2
GNSS rejections to 917 and `2c42096b`, which never moves, from 0 to 94. Replay every finalist on
the corpus before choosing, and prefer a quantity integrated over time to a raw sample.

The same holds for a *statistic*, and for a plan's premise. The simulator's IMU noise is white;
a real still window's is not, lag-one autocorrelation −0.98 to +0.96 across the corpus's axes.
#50's plan read a noise density off one sample's scatter, which recovered the simulator's
injected σ exactly and misread the corpus 6× high to 2.9× low, and the estimator became 50 ms
blocks halfway through the implementation. Before building on a statistic that assumes
independent samples, compute the corpus's lag-one autocorrelation for it: one line of Python, and
the cheapest point to learn the plan is wrong.

**Measure a claim on the rows the code reads.** A figure taken on a convenient proxy, "the first
500 rows", is a claim about the proxy. On #50 two of them ran past a still window into flight
(`4b473e91`'s window is 254 rows, `2c42096b`'s 160) and one read the opposite way on the window
itself; a review re-measuring on the harness window found it. The harness knows which rows it
used (`window=` on the `summary` line); take the evidence from those.

**Set a new input to its neutral value and check the old pins come back.** #52 added a
measurement time; replaying the corpus with every delay at zero reproduced the previous manifest
to within two counts, which is what showed every other move was the age and not the plumbing. A
neutral replay that does not reproduce the baseline is a finding about the change, and names
the part of it that is not what it claims (here, barometer readings timed between two IMU
samples being carried forward rather than taken as current).

**A figure quoted in prose is a measurement of a specific commit.** #52's review fixes moved
numbers after `data/manifest.txt`, `GOALS.md`, `data/scenarios.txt` and `data/ekf2.txt` had
quoted them, and every quote had to be re-measured: a sweep, a delay scan, an ANEES, a mechanism
check. Quote after the last code change, and re-run what the prose cites when the code moves
again. On a rendered page the same rot can hide inside placeholders: `a299e722`'s agreement
sentence argued "closer" from two heading figures that, once the numbers moved, said the
opposite, with every placeholder still resolving.

**An absence is measured only on what was fused.** `Gates`' doc comment dismissed the cost of a
joint GNSS gate because "the corpus shows no such fix" — true, because `2c42096b`'s barometer was
being discarded. The first change that fused it (#115) rejected 3945 of its 4616 fixes, every one
on height, and #118 had to split the gate. A claim that the corpus shows *no* X is conditional on
every source that could produce X reaching the filter. When a change makes a source reach a log
it did not before, grep for the claims that rested on its absence — "shows no", "never",
"none", `rejected_<source>=0` — and re-measure them in the same diff.

**Compare figures in the same measure.** The barometer's 13.6 m on `2c42096b` was its min–max
range; EKF2's ~12 m was start to end. Set side by side, they said EKF2 followed most of the drift
when it followed all of it (start to end, the barometer climbs ~12 m too). That survived a GOALS
rewrite, a manifest note and three issue comments before review caught it. Name the statistic — range, start to
end, RMS, mean — whenever two numbers are put next to each other.

**Know what a log is before reading its figures as accuracy.** `2c42096b` is a grounded,
vibrating vehicle under a poor sky view for two hours, not a flight. Its numbers pin *behaviour*
(what the filter does when two height sources disagree), and tuning toward them would be fitting
a bench test. Check peak speed and extent from the CSV before a log's figures argue for a change,
and say what the log is in its manifest note, as that entry now does. Check first whether it is
real. `3949f175` was the corpus's "baseline" and the evidence for a status timeout until
`ver_hw=PX4_SITL` showed it was a simulation. Its receiver reports `eph` 0.90 and 10 satellites on
every message, so its fix jitter was the scheduler's. Flight Review hosts SITL logs beside real
ones, and a SITL log is synthetic data without the simulator's truth.

**One statistic, one implementation.** The Rust replay harness is the only thing that *computes* a
statistic; it emits per-fusion rows and scalar keys on the `summary` and `score` lines. The Python
tools read those and aggregate, plot, or compare against the EKF2 reference — they never recompute
a number the harness already defines. A quantity that could be produced by both paths belongs to
the harness, because that is the one CI runs. Cross-log and against-reference metrics are defined
once, in Python: distance from EKF2 in `tools/agreement.py`, which reads no file, so the report
and `--corpus` both hand it arrays.

The rule reaches past *computing* a number, and the extension is the one that has actually bitten.
A statistic that **audits a filter claim** must falsify it in the shape the filter states it, not
merely read its verdict. `false_valid` took `Validity` off the filter exactly as intended and then
tested the 2-D norm of the horizontal error, while `Eskf::validity` states the claim per axis
(`within(PositionNorth) && within(PositionEast)`, `src/eskf.rs:1170-1171`) — a bar √2 tighter than
the one the filter asserted, diverging from it precisely as the estimate approaches it, which is
the only regime where such a count says anything. Reading the verdict and re-deriving the geometry
is still two implementations of one claim. It applies to every scoring statistic still to land:
NIS against the gates (#5), distance from EKF2 (#8), and scoring a rejection as correct (#60).

**Look at `tools/replay_report.py` before drawing a figure.** It renders one run per log, with
EKF2's reference beside it, and nothing else in the repository draws corpus figures. A bespoke
page that bins or averages the replay CSVs is a second implementation of statistics the harness
owns. Comparing *runs* (before and after a change on one log) is what it does not do. Until it
does, render one report per run and set them side by side; a cross-run mode belongs in the tool,
not in a page. The published pages follow the same rule: `VALIDATION.md` and `validation/*.md`
are rendered from `validation/src/` by `tools/validation.py`, every number through a placeholder
naming the line it is copied from and every figure drawn by `replay_report.py --figures`. A
number typed into a template is the one `--check` cannot re-derive, so it is the one that rots;
state a scenario's parameters in prose, never a result.

**Say which file a published number came from.** A score is a claim about a specific run, and
`examples/replay.rs` refuses a truth file whose `#` header names a different scenario or seed than
the log's. That check exists because the obvious guard does not work: an epoch counter that fails
to match truth rows catches nothing when a 50 Hz log lands on every fourth row of 200 Hz truth, so
the wrong file scored cleanly and published a figure with nothing tying it to what produced it.
UrbanNav's and INSANE's converters write the same kind of marker, a `source` digest of their
inputs, and any later source that pairs two generated files needs one too; a file carrying no
marker is taken on trust because there is nothing in it to check.

**A dataset's truth is a pipeline: read it before choosing what to score.** INSANE's paper
promises sub-degree attitude; its `gps_mag_orientation.m` builds that attitude partly from the
magnetometer the filter fuses, and at rest it tilts gravity 17° off vertical. Its sync step left
the truth 80–170 ms behind the IMU. Neither shows in a score line, which reads a coarse or late
truth as filter error; both showed in a check that could fail, truth rotation over one second
against the integrated gyroscope, and truth against the accelerometer at rest. Run those before
the first score, and state what the truth cannot judge.

## How defaults get decided

Three defaults are no longer placeholders, and each records its evidence in its doc comment:
`SourceHealth::timeout`'s 2.5 periods (replay showed `eb799954`'s bursting magnetometer flapping
2826 times at 2.0, and a median period flapping it 14025), `ImuNoise`
(ArduPilot's bias walks converted from per-step σ to density, which took `2c42096b`'s tilt
peak from 5.2° to 2.5°; white noise at ten times PX4's density, which vibration, a raw `R` and
GNSS latency were each measured spending), and `Initialization`'s
stationarity tolerances (the old ones failed four of five corpus logs on vehicles sitting on the
ground). Follow that pattern rather than adjusting a number quietly.

The loop that produced all three: implement the check, run it over the corpus, read what real
logs actually do, then pick the number — and write down what the data said. Anything touching
replay output should be measured this way before it is committed, because the manifest
expectations are the regression guard and a change that moves them needs a reason in words.

Read PX4 and ArduPilot source rather than their docs or memory, for behavior as much as for
numbers. Defaults: PX4 `src/modules/ekf2/EKF/common.h`, ArduPilot
`libraries/AP_NavEKF3/AP_NavEKF3.cpp`. Alignment: PX4 `EKF/ekf.cpp` (`initialiseTilt`),
ArduPilot `AP_NavEKF3_core.cpp` (`InitialiseFilterBootstrap`). Baro reference: PX4
`EKF/aid_sources/barometer/`, ArduPilot `AP_NavEKF3_Measurements.cpp`. GNSS `R` floors: PX4
`EKF/aid_sources/gnss/gps_control.cpp`, ArduPilot `AP_NavEKF3_PosVelFusion.cpp`. Status models:
PX4 `filter_control_status_u` in `common.h` plus `msg/versioned/VehicleLocalPosition.msg`,
ArduPilot `libraries/AP_NavEKF/AP_Nav_Common.h`. Read the firmware default, not the library's:
`EKF2.cpp` binds every `EKF2_*` parameter over `EKF/common.h`'s initialisers, so PX4's barometer
noise is the 3.5 m of `params_barometer.yaml`, not `common.h`'s 2.0, and its default height
reference is GNSS.

Cite `file:line` at the sha you read; `~/projects/PX4-Autopilot` and `~/projects/ardupilot` are
current checkouts, and a bare path rots as the tree moves. Cite it in **one** place — the document
that owns the claim — and have the others point there. PX4's reset timeouts were stated in three
files with two different descriptions of the condition before anyone checked the source: they are
`reset_timeout_max` (7 s of horizontal inertial dead reckoning) and `hgt_fusion_timeout_max` (5 s
of failed height fusion) at `src/modules/ekf2/EKF/common.h:515-517`, and `Recovery`'s `Default`
impl in `src/config.rs` owns them, since it is the code that uses them.

Comparing against them is a design tool, not just a fact check. `Validity` and
`predicted_validity` exist because a comparison showed both estimators answer *which output can I
use* per quantity while this crate answered only *how bad is the worst thing*; the gap was real
and had already cost the corpus a regression guard. When a reporting or API question comes up,
look at what those two publish before inventing something.

## Architecture

One published crate, `no_std`, `forbid(unsafe_code)`, `deny(missing_docs)`, allocation-free,
edition 2024, MSRV 1.89, one dependency (`nalgebra` with `libm`), plus `defmt` behind an
off-by-default feature of the same name. `magnetic-model`, on by default, pulls no crate: it links
the declination table, and CI builds and tests without it too.

- `src/eskf.rs` — `Eskf`, the whole public filter: `initialize`, `initialize_from`, `predict`,
  `fuse_*`, `state`, `reset_*_to`. `initialize_from` is stage 1
  of GOALS.md's "Alignment beyond the static window": the static window stays the preferred path,
  a moving or short window starts coarse under `Status::Aligning`, yaw from course is
  `fuse_course` (#53), and in-motion levelling (5′, #59) is the option still unbuilt — read that
  section before touching initialization.
- `src/init.rs` — initialization's types (`StaticSample`, `StaticWindow`, `Alignment`, `Coarse`,
  `InitError`) and pure functions (`level_from_accel`, `heading_from_mag`, `nominal_state`,
  `classify`, `attitude_sigmas`, `initial_covariance`). `StaticWindow` folds each sample in as
  it is pushed, so a caller never buffers the window; its doc comment owns how each statistic
  is taken in one pass, and `StaticWindow::noise` reports the sensors' noise floor, (8″). The `initialize*` methods on `Eskf` call these and commit the result.
  Tests for the pure functions live here; tests of what the filter does with them stay in
  `eskf.rs`.
- `src/propagate.rs` — `ImuSample` and equations (9)–(22), in increments; `error_dynamics`, the
  `A` of (16)–(19) that (23′) carries `H` through.
- `src/history.rs` — the recent past of the nominal state for (23′): 32 entries 10 ms apart,
  interpolated, carried forward on velocity and the last sample's rate for a time ahead of the
  present, and shifted by every correction in navigation axes through `Eskf::commit_state`, the
  one writer of the state outside a propagation or a start. `Eskf::observe` and `Eskf::past` are
  its only readers, and it costs 1.5 KB of `Eskf`.
- `src/update.rs` — the update every observation shares: (23)–(27) in Joseph form, the gate of
  (37)–(38), the injection and reset of (39)–(41). Generic over `dim(z)` and free of `Eskf`, so it
  is tested against a synthetic `H`. One Cholesky factorization of `S` serves the gate and the
  gain, and the gate runs first, so a rejection computes nothing it could commit. `Eskf::apply` is
  the one path that commits what it returns and records it. Its stack frame is the largest in the
  crate — `update::<3>`, 8088 bytes on `thumbv6m`, `Eskf::apply` itself
  inlining away, and the high-water mark is `fuse_gnss_velocity` into it at 9504, with `observe`
  and `apply_or_recover` kept out of line beside it rather than beneath (inlined, 10800) — which is why `reparameterize` applies `G P Gᵀ` block-wise and (30′)'s offset
  enters (27) in blocks rather than as a 16 × 16 (+4168 bytes measured); the figure
  and the #41 that would revisit it are in the doc comments.
- `src/observation/` — one module per sensor, each forming `y`, `H` and diagonal `R_m` and
  nothing else. `gnss.rs` holds (28) and (29) at the antenna, (28′) and (29′), `baro.rs` holds (30), `mag.rs` holds (34), (35)
  and the levelling variance (36′), and `heading.rs` holds (36), which every heading source
  shares, with (35′) and (35″); (31)–(33), the three-axis magnetometer, are deliberately unbuilt.
- `src/math.rs` — the primitives the equations share: `skew`, `exp_quat`, `wrap_pi`,
  `enforce_symmetry` (42). Pure, stateless, and unit-tested against their definitions. All four
  have callers as of (16)–(22), so none carries a dead-code allowance any more. Neither does
  anything else in `src/`: the gate of (37) took the last one when it became
  `record_rejected`'s first caller.
- `src/state.rs` — `State` (nominal, 16 values), `Covariance` (15×15, public as rows), and
  `ErrorState`, whose discriminants define the covariance ordering `[δp δv δθ δβa δβg]`; also
  `Offset`, the barometric offset's row of (30′) (`P_xb`, `P_bb`), crate-private and beside the
  covariance rather than in it, so the public 15 × 15 stays the navigation state's.
- `src/units.rs` — typed scalars/vectors. Types carry the claims that cause bugs — frame,
  value vs noise, sign convention — not units: SI throughout, and a constructor names a unit
  only where sources commonly supply another (`Radians::from_degrees`, `body_deg_per_s`).
  Vector constructors name the frame (`Position::ned`, `AngularRate::body`) and convert ENU/FLU
  input (`Position::enu(..).to_ned()`, `AngularRate::flu`); noise types are built `from_sigma`
  or `from_variance`. No `From<[f32; 3]>` on framed types: `.into()` would claim a frame
  silently. Payloads are `nalgebra` `Vector3<f32>` / `UnitQuaternion<f32>` inside and never in a
  public signature: `from_array`/`to_array` and `Quaternion` at the boundary, crate-private
  `from_vector`/`vector()` and `from_quaternion`/`quaternion()` for the equations.
- `src/frames.rs` — `Ned`, `Enu`, `Body` as sealed zero-sized type parameters on quantities.
- `src/geodetic.rs` — `Geodetic` (f64 lat/lon/height) and `LocalOrigin`, the tangent plane of
  equations (43)–(44), exact via ECEF (fixed-iteration inverse, no data-dependent loops). The filter owns the origin: `fuse_gnss_geodetic` places it on the first
  fix (under the estimate of the antenna, or at the fix after a coarse start), a static start clears it.
- `src/magnetic.rs` — PX4's WMM declination table and its lookup, behind the `magnetic-model`
  feature; `Eskf::place_origin` reads it at every origin placement.
- `src/config.rs` — tuning. Defaults are **placeholders** except the three listed under "How
  defaults get decided"; each doc comment records why. Preserve that habit: a default
  justified by data says so.
- `src/health.rs` — `Propagation` (`#[must_use]`), `Fusion` (not, deliberately — see its doc
  comment), `SourceHealth`, `Status`.
- `examples/replay.rs` — the normalized CSV format and the offline harness. Two output files:
  one row per IMU epoch, and one row per `fuse_*` call in `<out>.fusion.csv` with the gates in
  its header. `ν` and `S` are columns without values until the update of (23)–(28) publishes
  them — the harness must not compute them itself, which is "one statistic, one implementation"
  applied to the filter rather than to Python. A third argument is a truth CSV, and adds a
  `score` line beside `summary`: RMSE, NEES, `in3s`, `false_valid`. No truth file means no
  `score` line rather than a line of zeros, so the corpus and `manifest.txt` are untouched by
  it, and a truth file from another scenario is refused rather than scored — both files carry
  their scenario and seed in a `#` header, and the wrong one's timestamps line up often enough
  that nothing else notices. Every key reads one error vector, built in `error_state` and
  nowhere else. `false_valid` reads the filter's own `Validity` *and* falsifies it per axis,
  the way `Eskf::validity` states it: re-deriving either half tests a copy of the claim
  instead of the claim.
- `examples/simulate.rs` — the scenario table, and the only source of truth to score against.
  An example rather than a workspace member on purpose: `panic-check` is a crate out of
  necessity (its own target, profile and link step), while this one imports nothing, needs no
  dependency and costs 16 KB of the packaged crate. The first dependency scoring wants, or a
  second binary that would share its trajectory code, is when to move it, and the order to try
  is `examples/common/mod.rs` first, an unpublished member taken as a dev-dependency second, a
  feature-gated module in `src/` never — that last one qualifies the crate's `no_std` and
  single-dependency claims in four documents, and puts the simulator one `use` away from sharing
  the filter's rotations, which is the property its numbers rest on. See #16 and #17.
  Tests in an example do run under `cargo test --all-targets`, so none of this is about coverage.
  Their `main`s do not: `basic` and `degradation` compile in CI and never run, so a behaviour
  change that breaks them shows only when they are run and their output diffed against main's.
  #122's held-reading rule turned every `degradation` altitude into `NoReference` with every
  test green.
- `examples/embedded.rs` — the integration loop as firmware: `no_std` and `no_main` on
  `target_os = "none"`, an empty `main` on a host so `clippy --all-targets` reads it. CI builds it
  for both thumb targets, with and without `defmt`, and never runs it. Its `log!` macro is the
  one place both logging routes are exercised at a call site; a `std` call in it is a build
  error there and nowhere else.
- `panic-check/` — a second, unpublished crate: a bare-metal binary calling the whole public
  API, plus `run.sh`, which links it and reads the panic paths back out of the ELF. A workspace
  member so that one `Cargo.lock` covers both, but not a *default* member, so `cargo test`,
  `cargo clippy` and `cargo package` at the root never try to build a `no_std` binary for the
  host. Its build knobs live in `run.sh`, not in its `Cargo.toml`, where a member's `[profile]`
  would be ignored. The only way in is `panic-check/run.sh`.

### Invariants worth knowing before editing

- **A window sample's `baro` and `mag` may be held.** A slower sensor's last value repeats across
  IMU epochs, and the harness does exactly that. A mean survives it; any other statistic over
  the window has to count distinct readings, which is why `init::Scatter` counts a value
  only where it differs from the previous sample's. The corollary for fixtures: `[sample; N]`
  with a barometer is *one reading held*, which sets no reference — the fixtures, both examples
  and the `prelude` doctest all had to be given real scatter.
- **The filter never reads a clock; time is supplied.** Every `ImuSample` and every measurement
  carries a `Timestamp`, a seed names its own, and the step and a measurement's age are
  differenced in integer microseconds. A window's span is the time its samples integrated,
  summed in `f64`: 100 intervals of 20 ms sum to 1.9999987 s in `f32`, and `flight.csv` started
  `short`.
- **`predict` refuses or coasts, never fakes.** A sample not after the clock → `InvalidStep`,
  before the timers and the clock move. An interval under 1 µs or past `max_predict_dt` →
  `InvalidInterval`. `dt > Config::max_predict_dt` is never integrated from the one sample: it is
  `Coasted` by (22′), an assumption the covariance prices, or `StepTooLong` with `Config::coast`
  off. Both come *after* `Diagnostics::advance`, because the time really passed and `Status` must
  not claim otherwise, and both note the gap in `longest_gap` whatever becomes of the step.
- **`Status` is derived on read**, not cached, so mutating methods have no invariant to keep.
  Only sources that have ever been accepted count toward it.
- **Initialization does not refuse a usable window.** A short or moving one gives
  `Alignment::Coarse` with what it measured, the filter runs, and `Status::Aligning` says the
  attitude has not converged. Only genuinely unusable input errors (`NoSamples`, `InvalidStep`,
  `NotFinite`, `InvalidVariance`). Three entry points — `initialize`, `initialize_coarse`,
  `initialize_from` — and each reports an `Alignment`; `alignment_of` classifies without
  mutating. `is_aligned` reads the covariance against `ALIGNED_TILT` and `ALIGNED_HEADING` —
  constants, never `Config::accuracy`, which is the mission's and moves only `Validity` — so
  promotion is measured rather than timed — except heading, which no covariance can promote because
  stillness never observes yaw; that one waits for the first heading from `fuse_mag_heading`,
  `fuse_gnss_heading` or `fuse_course`. Promotion only: the flag
  **latches**, so `Status::Aligning` reports a start that has not been resolved and never
  returns, while `Validity::tilt` stays live and does fall back. Read live the bar was crossed
  703 times on `2c42096b` while it started coarse, its tilt σ over `ALIGNED_TILT` for 79 % of the log, and
  `7592c9b2` and `f16771dd` end `Aligning`, on body axes and navigation ones alike; PX4 and
  ArduPilot latch theirs for the same reason. The latch is the one thing about `Status` that is not derived on
  read, which is why every path that can move the attitude covariance calls `note_alignment`.
- **`Status` is the summary; `State::validity` is the detail.** Six per-quantity flags derived
  from the covariance against `Config::accuracy` (the one knob meant to be supplied, since
  mission accuracy is not derivable), plus "was this ever established" — the `Unestablished` flags
  a coarse start sets on position and velocity, and the one a window with no magnetometer sets on
  heading, since stillness observes tilt but never yaw — each cleared by the adoption of the first
  measurement of that quantity. A prior is not an estimate, and
  `sigma_yaw` (0.35 rad) sits inside `Accuracy::heading` (0.5236, 30°), so the covariance reports a yaw
  nobody measured as good. `Eskf::predicted_validity` answers the arming question instead: valid now,
  or a constraining source is being accepted. Both exist because PX4 and ArduPilot answer
  per-quantity validity and a single ladder cannot.
- **An unaided filter loses its outputs on a schedule the defaults set**, and the schedule is
  measured: at `ImuNoise`'s defaults a static start holds tilt for 3.83 s and heading for 37.8 s.
  Those two figures are cited by `Accuracy`'s doc comment and pinned by a test; the gyroscope-bias
  prior entering attitude through (20)'s `−I Δt` is what sets them, not the white-noise density,
  which alone would give 10.4 s and 674 s. They move `Validity` only — `Status` is answering on
  the aiding timers well before then, and the alignment latch keeps `Aligning` out of it.
- **`Status` precedence is most-severe-first**: `DeadReckoning` > `Aligning` > `Degraded` >
  `Healthy`. Aligning outranking Degraded is deliberate, and what it costs is measured: the
  masking lasts exactly as long as a start goes unresolved. The 2 h corpus log showed 2 transitions
  rather than 888 while its coarse attitude prior was charged the window's worst sample, and reports
  all 888 now that (8′) bounds that prior by the average (5)–(6) level and the start resolves 0.20 s
  in.
- **The filter recovers by adoption, per source, and only by adoption.** A source the gate has
  rejected for longer than `Config::recovery` allows has its next measurement adopted
  (`Fusion::Reset`, counted in `SourceHealth::recovered`), through the same path as a first
  adoption; `Recovery::OFF` reproduces the filter that only reports, byte for byte on every
  scenario and corpus log, and `reset_position_to` / `reset_velocity_to` stay for an application
  that owns the decision. A new correction of any kind gets its own switch, default on, rather
  than a shared "auto" flag — GOALS.md, "Rejection handling". Only a *rejection* triggers it:
  silence and refusals have nothing to adopt. Heading waits while GNSS is arriving (PX4's guard),
  the barometer re-reads `α₀` rather than stepping a state, and a GNSS height recovery drops a
  reference read from the estimate. The other adoption is the first measurement of a quantity
  initialization never established, once per quantity, because there is no estimate to step away
  from. After a coarse start that is the first GNSS position and velocity; for heading it is any
  start that observed no yaw, which includes a *static* window carrying no magnetometer, since
  stillness observes tilt and never yaw. Only a seed escapes, having vouched for every quantity.
  The heading adoption is the one that steps **attitude**, by up to half a circle, so it
  reparameterizes the surviving attitude rows by (41) with the exact `R(q̂⁺)ᵀR(q̂)` rather than the
  small-angle Jacobian an update takes, and it carries the `R` of (36′) rather than the caller's
  σ_ψ² — a heading levelled by a coarse tilt is not worth the magnetometer's own variance. A
  heading recovery takes the same path. The barometric reference is the same
  shape without the step: a start that leaves none reads it from the estimate at the first
  altitude once position is established, which moves no state.
- **A `Config` is valid by construction, and nothing is committed on a refusal.** `Eskf::new`
  refuses a value outside its bound, so code past it may trust `Config`. A start is worked out
  whole (`Startup`), checked, then committed, and `alignment_of` reads the same `Startup`. An
  adoption, a declination turn or an origin placement that could still refuse is asked first
  (`declination_at`, `carried_*` returning `None`). The adversarial suite holds both: a refused
  call must leave the estimate, clock, origin, declination, reference and latches bit-identical.
- **Nothing in `src/` panics.** No `unwrap`, `expect`, `panic!` or `unreachable!` outside
  `#[cfg(test)]`. On `thumbv6m` a panic is a `udf` and the vehicle is a brick, which is why bad
  input is reported through a typed outcome rather than asserted on. The matrix arithmetic the
  equations bring is where this gets broken — `try_inverse` returns an `Option` and `nalgebra`
  indexing panics out of range. Refuse or saturate; never unwrap. `panic-check/run.sh` gates it
  in CI by linking the public API for both thumb targets and failing on a surviving
  `core::panicking` reference, which catches the indexing nobody wrote down as well as the
  `unwrap` somebody did. It also refuses to run if `panic-check/src/main.rs` is missing any
  `pub fn` in `src/`, so a new entry point has to be linked into it — matched on the name, so
  one call per name is enough and a rename is what breaks it. Every `Display` impl in `src/` is
  held the same way, through a `show::<T>(` call, because core's `f32` formatting reaches
  `core::panicking`: an impl printing a float with `{}` fails the gate, and `src/display.rs`'s
  `Fixed` is what prints one instead. `README.md` owns the two
  boundaries: `debug-assertions = false`, and `opt-level = 3` or `"s"`.
- Frames and units are fixed at the boundary: NED navigation frame, FRD body, Hamilton
  quaternion scalar-first, down-positive gravity. Not configurable.

### Adding a measurement source costs more than a `fuse_*`

Every source touches the same ten places, and three of them are public:

- `src/observation/` gains a module forming `y`, `H` and `R_m` from a `&State`, and its `fuse_*`
  takes the measurement's `Timestamp`, checks it with `admit`, builds the observation through
  `Eskf::observe` (so it is formed against the state at that time, (23′)) and calls
  `update::update` and `Eskf::apply_or_recover`. An adoption carries the measurement forward with
  `carried_position`/`carried_velocity`. This is the cheap part, and the only one the compiler checks:
  a `Gate<M>` of the wrong dimension does not build.
- `Diagnostics` gains a field and `sources()`'s return type changes length
  (`src/health.rs:1030`) — `#[non_exhaustive]` covers the new field, but not the array length,
  so settle the source set before publishing. `aiding()` beside it derives the sources `Status`
  counts from that list, so a source that aids nothing a sensor does not (the course) is excluded
  there by name, and `advance()` is a third list to extend.
- `Gates` gains a field, a `Gate<DOF>` at the observation's dimension — the type states the degrees
  of freedom, and `Gates::at` needs a line for the new field.
- A source is a verdict, not a sensor: a GNSS fix is two, `gnss_position` and `gnss_height`, gated
  apart (#118), so a sensor whose components fail independently wants a source per component.
- A `fuse_*` calls `note_arrival` on its source once `admit` passes, which is what gives it a
  `period()` and so its own timeout; `every_source_measures_its_period_from_what_it_offers`
  fails for a source that does not. `Timeouts` has nothing per source to add. `Recovery` *is*
  per source, like `Gates`,
  so it gains a field and a decision about what adopting the source resets. So does
  `Correlation`: a `τ` for (24′), from the source's `acf1_` on the corpus read as white, and the
  source's `fuse_*` calls `.correlated(...)` on its observation.
- `Validity` and `predicted_validity`: decide whether the source constrains a quantity, and say so.
- A `summary` key in `examples/replay.rs`, pinned per log in `data/manifest.txt`, plus a corpus log
  that uniquely covers the source — or an honest note that none does.
- `AXES` in `examples/replay.rs`, naming the source's innovation components, since `Innovation`
  carries values and variances and no names for them. Twenty-three consistency keys are generated
  from it and `SOURCES` together, so a source added to one and not the other is an index out of range
  rather than a missing key — caught by an `assert_eq!` in the same file, which is the weakest
  guard on this list.
- The README fusion table, the `Eskf` and `prelude` doctests, and `EQUATIONS.md`'s mapping
  table.
- `tools/ulog2replay.py`, which has to find the source in a ULog and name its variance columns.
- The comparison with EKF2, if EKF2 judges the source: its test ratio in `EKF2_RATIOS`
  (`tools/ulog2replay.py`) and `REFERENCE_KINDS["ratio"]` (`tools/replay_report.py`), the pairing
  in `RATIO_OF` (`tools/agreement.py`), and a new `rej_s_` key in every line of `data/ekf2.txt`.
  If the source reports its own `R` and EKF2 floors it, `RPolicy` in `examples/replay.rs` owns the
  floor. Harness keys reach the table by prefix (`rejected_`, `nis_`), so those need nothing.
- `examples/simulate.rs`, which has to model it — an error table, a `sample`, a row — and the
  column legend its generated headers carry. Nothing connects these two to the replay format at
  compile time, which is why they are on this list rather than left to be discovered.

The *truth* format is the one coupling of this kind that is guarded: `write_truth_header` in
`examples/simulate.rs` and `TRUTH_COLUMNS` in `examples/replay.rs` are still two lists, but the
scorer checks the header against its own and refuses the file, so a column renamed or reordered
stops the run instead of scoring one quantity against another. A new *state* — not a new source —
is what moves those columns.

The converter's `#` header lines are the other coupling, and they are guarded less. Each is an
f-string in `tools/ulog2replay.py` and a parser elsewhere: `# Magnetic declination` and
`# GNSS noise parameters` read by `examples/replay.rs`, `Estimator:`, `EKF2 position in replay
frame:` and `EKF2 aiding:` by `tools/replay_report.py`, which also reads the bounds off
`tools/anees.py --series`'s `# fusion-nav anees for` line. Both sides carry fixtures on the same literal
strings, so rewording one side fails a self-test; nothing stops the two sets of literals drifting
apart together. A parser that finds nothing reads its absence (declination zero, no origin)
rather than failing, which is why the report's fixtures exist.

### Reports are `#[non_exhaustive]`, outcomes are not

`Diagnostics`, `SourceHealth` and `PropagationHealth` carry the attribute; `Propagation`,
`Fusion`, `Status`, `Refusal`, `Validity`, `Alignment` and `InitError` do not. The split is by
what the type is for, not by how likely it is to change.

A report grows, and is read: `SourceHealth` gained `refused`, `last_refusal` and `adopted` in
#69, `PropagationHealth` gained `refused_not_finite` in #73, and every new source adds a
`Diagnostics` field. A reader loses nothing to a new field; only a caller *constructing* one
breaks, which is what the attribute refuses.

An outcome is matched, and a wildcard arm is the integrator bug the typed outcomes exist to
prevent — an application that silently ignores a refusal the filter grew flies on a stale state
and reports nothing. So after the first release, adding a variant is a breaking change on
purpose, raised by the compiler at every call site. Before it, a variant is free: add it when the
design wants it.

Neither attribute substitutes for settling the API: `#[non_exhaustive]` does nothing for the
length of `sources()`'s array.

### Three kinds of noise number, easily confused

- **`R`, measurement noise** — a per-call argument to every `fuse_*`, never stored in `Config`,
  because the accuracy of a fix is a property of that fix. Passed as `PositionNoise` and friends,
  built from σ or variance as the source reports it. GNSS supplies its own (`eph`/`epv`,
  `s_variance_m_s` — a σ despite the name — squared into the replay CSV's variance columns by
  `tools/ulog2replay.py`). The harness fuses each row's variance **unfloored**, where both
  production estimators bound a receiver's first — a decision rather than an oversight, reported
  as `r_policy=` on the `summary` line and argued in `data/README.md`; the clamping facility is
  `PositionNoise::clamped` / `VelocityNoise::clamped`, which own the PX4 and ArduPilot citations.
  PX4 logs no barometer or
  magnetic-heading variance, so the converter substitutes constants and says so; the honest
  source for those is bench characterization of the residual after calibration, which is a
  different activity from calibration itself (this crate does no calibration — that is the
  application's job).
- **`Q`, process noise** — `Config::imu` (`ImuNoise`), continuous-time densities, and
  `Config::baro_offset_walk`, the random walk of (30′)'s barometric offset.
- **`P0`, initial state uncertainty** — `Initialization::sigma_*`. A prior on the *state*, not on
  any measurement; `sigma_yaw` >> `sigma_tilt` because gravity pins tilt and yaw inherits the
  magnetometer's error.

A fourth thing is often mistaken for `R`: **an error the readings share.** Inflating `R` cannot
represent it, because (24) treats each reading's error as independent, so N readings average `S`
down by about N however large `R` is. Measured on #115: `α₀` read from the estimate carries the
fix's height error into every barometer reading, and adding that variance to `R` (PX4's
`baro_height_control.cpp:87`) takes `moving_start`'s `nees_pos` from 112.59 only to 14.58, while
`σ_pos_d` still falls 1.80 m → 0.16 m under a constant 1.1 m error. A shared error belongs in the
covariance with its correlations — (30′), where #119 measured a consider state against an estimated
one and the corpus chose the second. (36′) is `R` inflation too,
and it works because velocity fusion keeps correcting the tilt it prices. Before pricing an
error into `R`, ask whether it persists across readings. An error that persists *between*
readings for a time rather than forever is the case `R` can carry after all, as (24′)'s
equivalent white noise, because it is the interval that sets the factor; it is the constant
share that still needs the covariance.

A window taken **at rest** also fixes `α₀`, the barometric reference (`StaticSample::baro` →
`Eskf::baro_reference`, equation (30)); one taken in motion keeps whatever reference the flight
already had, since it cannot claim the altitude it reads is the ground. Measured by
`init::at_rest`, not read off the `Alignment`: `classify` reports a short window as
`Coarse::WindowTooShort` *before* it ever measures motion, so a still 0.8 s window on the ground
has an honest reference while a moving window of any length does not. Its variance is the
window's own scatter over its count of distinct readings — a held value is one reading, and one
reading sets no reference — and from then on (30′) estimates it, so it is none of the three above:
an initialization output that the filter goes on refining. A window
that fixes none — in motion, or with no barometer sample — leaves the first altitude after
position is established to read one from the estimate (#115), correlated with the height it was
read against; until then `fuse_baro_altitude` returns `Fusion::NoReference` rather than referring
altitudes to an invented origin. `cd7e0001` is the corpus log that covers the seed (`2c42096b` did
while it started coarse); the LPE log
(`7592c9b2…`) yields no barometer rows at all.

The same window also *measures* the barometer and IMU noise, `StaticWindow::noise` (#50, equation
(8″)), under GOALS.md differentiator 7, "Configuration derived, not demanded". What it hands back
is a floor, not a starting `R`/`Q`: a vehicle on the ground is quieter than one in flight, and the
corpus measured both directions (`WindowNoise`'s doc). Keep its boundary: derived at a defined
moment and reported, never silently retuned in flight, which would cost the determinism claim; a
recovery is an adoption on a schedule `Config::recovery` fixes in advance, not a retuning.
Anything the window cannot honestly measure, the noise to configure above the floor included,
goes to an offline tool that prints a `Config` (#51), not into the filter.

### Documentation is the specification

`EQUATIONS.md` holds numbered equations (1)–(44) and an equation-to-code mapping table naming
the module and function intended to implement each. Implementation work follows that layout
(`init.rs`, `propagate.rs`, `update.rs`, `observation/{gnss,baro,mag}.rs`, `math.rs`), cites its
equation numbers in the doc comment (existing stubs already do), and updates the table when the
layout changes. `README.md` is the user guide (why an ESKF, how to initialize, run, and read
health); architecture and implementation detail go in `DESIGN.md`, replay/corpus usage in
`data/README.md`. `GLOSSARY.md` defines the vocabulary the rest assume — one entry per term,
pointing at the document that owns the thing rather than restating it, so a definition cannot
drift from the equation it describes. A term a newcomer would have to look up belongs there, not
expanded inline in the document using it. It carries *this* crate's vocabulary rather than the
field's, with one exception: a term PX4 or ArduPilot uses for a different thing, which is where a
reader arrives already holding the wrong definition — `Reset` is recovery there and adoption
here. `GOALS.md` records positioning, the six differentiators, and decisions already made — check
it before changing scope; the non-goals list is deliberate. Its landscape says what each
alternative is and why it is not this crate, and carries no versions, release dates, download
counts or dependency lag: "~36 in the last 90 days" was already 33 when someone looked, and
"nine minor versions behind" became ten when `nalgebra` shipped. crates.io and the linked
repositories own those figures.

Prose in the documents avoids em dashes: a comma, colon, parenthesis or a new sentence carries
the same aside. `GOALS.md` is written that way; the rest converts as it is edited.

The README **is** the crate's front page: `src/lib.rs` is one `#![doc = include_str!]` and no
prose of its own, so `cargo test --doc` compiles every snippet the user guide shows. A snippet
that cannot stand alone takes hidden `# ` setup lines rather than a `text` fence — a snippet
that opts out is the one that rots — and few of them, because rustdoc hides those lines while
GitHub and crates.io print them. The integration loop is `no_run`: it compiles, which is the
part that catches a renamed method, and it never terminates. Assertions live on the items they
document, `Eskf` and `prelude`, so the README carries shape and those carry behavior.

What the compiler still does not read is the prose around the snippets. The variant tables for
`Status`, `Propagation` and `Fusion` are written by hand, and a missing row is how the old API
block lost `Status::Aligning`, `Fusion::Reset` and `Fusion::NoReference`. Snippets handle every
`#[must_use]` outcome rather than `let _ =`; the README is where integrators copy from. `Fusion`
no longer carries the lint, because `Diagnostics` covers acceptance and rejection, but a snippet
still shows the refusals it does not cover rather than dropping them.

The README reaches the other documents through **absolute** GitHub URLs, since rustdoc serves it
with no siblings beside it and a relative `DESIGN.md` is a 404 on docs.rs. Both forms, and
in-page anchors, are resolved against the headings they name by `tools/check-anchors.sh`, which
CI runs together with its `--self-test` fixtures — so a renamed heading fails there rather than
in a reader's browser, and nobody greps for `.md#` first. It refuses a relative sibling link in
any file rustdoc includes, which is the docs.rs 404 nobody reading GitHub can see. It is shell
rather than Python, and outside `uv`'s remit above, because it reads the repository's Markdown
and nothing else.

For the same reason the README carries **no mermaid**: mermaid renders through client-side
JavaScript that GitHub ships and docs.rs and crates.io do not, so a diagram there is a picture on
one surface and ten lines of `flowchart TD` on the other two. A diagram the README needs goes in
a `text` fence, which renders the same everywhere. `DESIGN.md` and `GOALS.md` keep theirs —
nothing includes them, and GitHub is where they are read.

### The sources are fetched, and the citations to them are checked

`reference/` holds the primary sources, fetched rather than committed for the same reason as the
corpus logs — arXiv's licence covers distribution *there*, and a standards body owns its own
document. The PDFs are gitignored; `reference/README.md` is the record, and an entry carries a
URL, a sha256 and a line saying what the file is the source *of*, so a claim can be traced to one
and a source that turns out to answer nothing can be dropped.

Fetching them is what makes a citation checkable, and the check pays. `EQUATIONS.md`'s
correspondence table maps thirteen of its equations onto Solà's, and reading the paper against
the prose around them found two sentences that contradicted it: a global attitude-error
perturbation moves `R` rather than changing signs, and the `Δt²` form of `Q` is his as much as
PX4's, differing in which σ is held fixed rather than in the exponent. Solà numbers are v1's,
arXiv holds no other version, and a bare number in that document is always its own.

Three habits follow, and each cost something before it was written down:

* **Cite what a claim actually rests on, and drop the rest.** Farrell and Markley & Crassidis sat
  in the References with no equation, doc comment or number pointing at them. A bibliography
  entry nothing rests on reads as support and supplies none.
* **Say when a source cannot be checked.** Groves (2.112) needs a book, and the References and
  `reference/README.md` both say so rather than leaving a reader to discover it.
* **Check what a fetched source actually contains before citing it.** The WGS 84 standard
  defines the ellipsoid but never writes the geodetic-to-ECEF conversion, so it is the source for
  the constants in `src/geodetic.rs` (Tables 3.1 and 3.5) and not for (43) — which is what the
  entry in `reference/README.md` says.
