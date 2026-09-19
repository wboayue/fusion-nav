# AGENTS.md

This file provides guidance to coding agents working with code in this repository.

## Status

**API sketch.** Types and signatures compile; the estimation mathematics does not exist.
`predict` propagates nothing, every `fuse_*` accepts with a zero test ratio. What is real is
the health bookkeeping (timers, `Status`, `Diagnostics`), the typed API surface, and the replay
harness. Anything stubbed says so in its doc comment with a `**Stub.**` paragraph — keep that
marker accurate when landing real math, and keep the same caveat in `README.md`, `DESIGN.md`,
`src/lib.rs`, and the example module docs, which all repeat it.

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

## Commands

```bash
cargo test --all-targets          # unit tests (inline `mod tests` in src/eskf.rs and src/init.rs)
cargo test --doc                  # doctests in lib.rs / prelude carry the usage contract
cargo test --lib eskf::tests::a_step_over_the_limit_is_refused_but_the_time_still_passes  # one test
cargo fmt --all -- --check
cargo clippy --all-targets --no-deps   # CI runs with RUSTFLAGS=-D warnings
cargo build --lib --target thumbv7em-none-eabihf   # also thumbv6m-none-eabi; both gate CI
cargo +1.89 build --lib           # MSRV

cargo run --example basic         # minimal integration loop
cargo run --example degradation   # timeouts, status transitions, application-driven recovery
cargo run --example replay        # replays data/flight.csv -> target/replay.csv (CI smoke test)
cargo run --example replay -- <input.csv> <output.csv>
```

## Replay corpus

Real PX4 logs are fetched, not committed (`data/logs/` is gitignored). `data/manifest.txt`
pins each by sha256 *and* by expectations (`rate=`, `window=`, `refused=`, `transitions=`,
`status=`) matched against the `summary` line `examples/replay.rs` prints.

```bash
data/fetch.sh                     # fetch + verify the manifest
data/fetch.sh --verify            # checksums only, no network
data/fetch.sh --check             # convert each .ulg and replay it, assert expectations (needs pyulog)
data/fetch.sh --add <url> [name]  # download once, append a manifest line to commit
tools/ulog2replay.py log.ulg -o log.csv [--reference]   # ULog -> replay CSV
```

`--check` is a **local** tool, run before a release or after touching the converter, replay
example, or any default it asserts. Converters stay out of the test path on purpose (GOALS.md,
"Harness constraint"): CI must need no network, no PX4 tooling, and no hardware, so it replays
the checked-in synthetic `data/flight.csv` only. Never add a pyulog or ROS dependency to the
Rust test path.

Each manifest entry exists because it covers something nothing else does (SD-card dropouts
driving `StepTooLong`, burst logging that forced a median rate estimator, a 2 h log guarding the
f64 timestamp parse, old field spellings). Adding or dropping a log means saying which behavior
it uniquely covers. Changing a `Config` default or the `summary` line requires updating the
affected expectations.

Expectations are matched pair by pair as substrings, so **adding** a key to the `summary` line is
safe and pinning new behavior there is cheap — `align=` and `resets=` were added that way, and
each now guards a decision that would otherwise rot into a comment. Renaming or removing a key
breaks every entry at once.

## How defaults get decided

Three defaults are no longer placeholders, and each records its evidence in its doc comment:
`Timeouts::degraded_after` (replay showed 76 status flaps in 124 s), `ImuNoise` (PX4 and
ArduPilot agree within 2x and both sit 10-15x above datasheet), and `Initialization`'s
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
ArduPilot `libraries/AP_NavEKF/AP_Nav_Common.h`. Published figures drift — PX4's barometer noise
default is 2.0 m in current source, not the 3.5 m widely quoted.

Comparing against them is a design tool, not just a fact check. `Validity` and
`predicted_validity` exist because a comparison showed both estimators answer *which output can I
use* per quantity while this crate answered only *how bad is the worst thing*; the gap was real
and had already cost the corpus a regression guard. When a reporting or API question comes up,
look at what those two publish before inventing something.

## Architecture

Single crate, `no_std`, `forbid(unsafe_code)`, `deny(missing_docs)`, allocation-free, edition
2024, MSRV 1.89, one dependency (`nalgebra` with `libm`).

- `src/eskf.rs` — `Eskf`, the whole public filter: `initialize`, `initialize_from`, `predict`,
  `fuse_*`, `state`, `reset_*_to`. `initialize_from` is stage 1
  of GOALS.md's "Alignment beyond the static window": the static window stays the preferred path,
  and coarse in-motion alignment plus a `Status::Aligning` phase is designed there but unbuilt —
  read that section before touching initialization.
- `src/init.rs` — initialization's types (`StaticSample`, `Alignment`, `Coarse`, `InitError`)
  and pure functions (`classify`, `attitude_sigmas`, `initial_covariance`, `baro_reference`).
  The `initialize*` methods on `Eskf` call these and commit the result. Tests for the pure
  functions live here; tests of what the filter does with them stay in `eskf.rs`.
- `src/propagate.rs` — `ImuSample` today; equations (9)–(22) land here.
- `src/state.rs` — `State` (nominal, 16 values), `Covariance`/`CovarianceMatrix` (15×15), and
  `ErrorState`, whose discriminants define the covariance ordering `[δp δv δθ δβa δβg]`.
- `src/units.rs` — typed scalars/vectors; every constructor names its unit (`from_meters`,
  `from_rad_per_s`). Payloads are `nalgebra` `Vector3<f32>` / `UnitQuaternion<f32>`.
- `src/frames.rs` — `Ned`, `Enu`, `Body` as sealed zero-sized type parameters on quantities.
- `src/geodetic.rs` — `Geodetic` (f64 lat/lon/height) and `LocalOrigin`, the tangent plane of
  equations (43)–(44). The filter owns the origin: `fuse_gnss_geodetic` places it on the first
  fix (under the estimate, or at the fix after a coarse start), a static start clears it.
- `src/config.rs` — tuning. Defaults are **placeholders** except the three listed under "How
  defaults get decided"; each doc comment records why. Preserve that habit: a default
  justified by data says so.
- `src/health.rs` — `Propagation`, `Fusion` (both `#[must_use]`), `SourceHealth`, `Status`.
- `examples/replay.rs` — the normalized CSV format and the offline harness.

### Invariants worth knowing before editing

- **The filter never reads a clock.** `dt` is a parameter everywhere, including `initialize`,
  which measures the static window in seconds rather than samples.
- **`predict` refuses rather than fakes.** Zero/negative/NaN `dt` → `InvalidStep`, before the
  timers move. `dt > Config::max_predict_dt` → `StepTooLong`, but *after* `Diagnostics::advance`,
  because the time really passed and `Status` must not claim otherwise.
- **`Status` is derived on read**, not cached, so mutating methods have no invariant to keep.
  Only sources that have ever been accepted count toward it.
- **Initialization does not refuse a usable window.** A short or moving one gives
  `Alignment::Coarse` with what it measured, the filter runs, and `Status::Aligning` says the
  attitude has not converged. Only genuinely unusable input errors (`NoSamples`, `InvalidStep`,
  `NotFinite`, `NegativeVariance`). Three entry points — `initialize`, `initialize_coarse`,
  `initialize_from` — and each reports an `Alignment`; `alignment_of` classifies without
  mutating. `is_aligned` reads the covariance against `Initialization`'s sigmas, so promotion is
  measured rather than timed.
- **`Status` is the summary; `State::validity` is the detail.** Six per-quantity flags derived
  from the covariance against `Config::accuracy` (the one knob meant to be supplied, since
  mission accuracy is not derivable), plus "was this ever established" — the `unknown` flags a
  coarse start sets. `Eskf::predicted_validity` answers the arming question instead: valid now,
  or a constraining source is being accepted. Both exist because PX4 and ArduPilot answer
  per-quantity validity and a single ladder cannot.
- **`Status` precedence is most-severe-first**: `DeadReckoning` > `Aligning` > `Degraded` >
  `Healthy`. Aligning outranking Degraded is deliberate and is why the 2 h corpus log now shows
  2 transitions rather than 888.
- **The filter gates but never self-recovers.** On sustained rejection it reports
  `DeadReckoning`; `reset_position_to` / `reset_velocity_to` exist for the application to
  decide. Do not add automatic resets — that is a documented decision in GOALS.md. The single
  exception, and its boundary: after a *coarse* start the first GNSS position and velocity are
  adopted (`Fusion::Reset`, once per quantity), because those were never established and there
  is no estimate to step away from. Never for recovery, never for a quantity that was once
  known.
- Frames and units are fixed at the boundary: NED navigation frame, FRD body, Hamilton
  quaternion scalar-first, down-positive gravity. Not configurable.

### Three kinds of noise number, easily confused

- **`R`, measurement noise** — a per-call argument to every `fuse_*`, never stored in `Config`,
  because the accuracy of a fix is a property of that fix. GNSS supplies its own (`eph`/`epv`,
  `s_variance_m_s`, squared into variances by `tools/ulog2replay.py`). PX4 logs no barometer or
  magnetic-heading variance, so the converter substitutes constants and says so; the honest
  source for those is bench characterization of the residual after calibration, which is a
  different activity from calibration itself (this crate does no calibration — that is the
  application's job).
- **`Q`, process noise** — `Config::imu` (`ImuNoise`), continuous-time densities.
- **`P0`, initial state uncertainty** — `Initialization::sigma_*`. A prior on the *state*, not on
  any measurement; `sigma_yaw` >> `sigma_tilt` because gravity pins tilt and yaw inherits the
  magnetometer's error.

The static window also fixes `α₀`, the barometric reference (`StaticSample::baro` →
`Eskf::baro_reference`, equation (30)). It is neither of the three above: a constant, not noise
and not a state, and the only initialization output an application may need to keep. A window
with no barometer sample leaves it unset and `fuse_baro_altitude` returns `Fusion::NoReference`
rather than referring altitudes to an invented origin — the LPE log in the corpus
(`7592c9b2…`) yields no barometer rows at all and covers that path.

Still open: the same window could *measure* the barometer and IMU noise and hand back a starting
`R`/`Q` instead of making the caller guess, as another output of `initialize`. That sits under
GOALS.md differentiator 7, "Configuration derived, not demanded" — do not ask for a value the
system could measure. Note its boundary before acting on it: derived at a defined moment and
reported, never silently retuned in flight, which would cost the determinism claim and
contradict report-do-not-self-recover. Anything the static window cannot honestly measure goes
to an offline tool that prints a `Config`, not into the filter.

### Documentation is the specification

`EQUATIONS.md` holds numbered equations (1)–(44) and an equation-to-code mapping table naming
the module and function intended to implement each. Implementation work follows that layout
(`init.rs`, `propagate.rs`, `update.rs`, `observation/{gnss,baro,mag}.rs`, `math.rs`), cites its
equation numbers in the doc comment (existing stubs already do), and updates the table when the
layout changes. `README.md` is the user guide (why an ESKF, how to initialize, run, and read
health); architecture and implementation detail go in `DESIGN.md`, replay/corpus usage in
`data/README.md`. `GOALS.md` records positioning, the six differentiators, and decisions already
made — check it before changing scope; the non-goals list is deliberate.

The README is not compiled — `lib.rs` does not `include_str!` it — so its snippets rot silently;
its old API block went missing `Status::Aligning`, `Fusion::Reset`, and `Fusion::NoReference`
without anything failing. When an enum or signature changes, check the README's tables and quick
start against the `lib.rs` doctest, which is the compiled source of truth. Snippets handle every
`#[must_use]` outcome rather than `let _ =`; the README is where integrators copy from, and
discarding outcomes is the one habit the API is built to prevent. Cross-doc anchors
(`DESIGN.md#measurement-rejection` from `GOALS.md`) break on heading renames — grep for
`.md#` before renaming one.
