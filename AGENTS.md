# AGENTS.md

This file provides guidance to coding agents working with code in this repository.

## Status

**API sketch.** Types and signatures compile; the estimation mathematics does not exist.
`predict` propagates nothing, every `fuse_*` accepts with a zero test ratio. What is real is
the health bookkeeping (timers, `Status`, `Diagnostics`), the typed API surface, and the replay
harness. Anything stubbed says so in its doc comment with a `**Stub.**` paragraph — keep that
marker accurate when landing real math, and keep the same caveat in `README.md`, `DESIGN.md`,
`EQUATIONS.md`, `src/lib.rs`, and the example module docs, which all repeat it. `EQUATIONS.md`'s
is the one to watch: it sits above a mapping table that separately marks functions as unbuilt, so
the two can contradict each other, and did — the banner claimed no implementation existed while
the table below it listed the geodetic origin and the initial covariance as built.

## Backlog

Two tracking issues own ordering and hold rules the individual issues do not repeat. Read the one
covering the area before starting work in it.

- **#31** — implementing `EQUATIONS.md`, staged as #32–#40. Carries the stage table, the
  dependency reasoning (why the simulator and `rejected=` come *before* the math), and standing
  rules that apply to every stage.
- **#10** — replay validation. Consistency on the corpus, which has no truth; accuracy on
  simulation and INSANE, which do. The two measure different things and are not collapsed.

Issues here are worth the length they run to. The convention: cite `file:line` at a pinned
upstream revision rather than paraphrasing PX4 or ArduPilot, state what it depends on and what it
blocks, name the replay/manifest impact, and say what the data must say before the issue can close.
Each carries an area label (`equations`, `validation`, `api`, `perf`, `docs`, `tooling`, `ci`) and
one of three milestones — *API frozen*, *Equations implemented and scored*, *Measured and
published*. Trackers carry neither, since they span all three.

**When looking for gaps, audit `GOALS.md` rather than the issue list.** The six differentiators,
the two open design questions and the derived-configuration table are commitments, and a
commitment with no issue against it is the gap — differentiator 1, the one GOALS calls most
defensible, had zero representation in the backlog until #41 and #42.

Differentiators are cited **by number** here, in issue bodies and in `data/manifest.txt`, so
`GOALS.md` numbers them 1–4, 6, 7: 5 was ecosystem coherence, dropped in #63, and the gap stays.
Never renumber them. A renumber silently repoints every citation, including closed issues that
cannot be corrected.

**Sequencing hazard:** #31's stages are stacked branches, while the signature-changing issues
(#21, #22, #25, #59, #61) change the API underneath them. Land an API change before the stage
that builds on it, not after — #59 before #33, #61 before whichever stage seeds an attitude.

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
  came from* (`degraded_after`: 76 status flaps in 124 s). A paragraph that is none of the three,
  or that the signature already answers, is cut.
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
cargo test --all-targets          # unit tests (inline `mod tests` in src/eskf.rs and src/init.rs)
cargo test --doc                  # doctests in lib.rs / prelude carry the usage contract
cargo test --lib eskf::tests::a_step_over_the_limit_is_refused_but_the_time_still_passes  # one test
cargo fmt --all -- --check
cargo clippy --all-targets --no-deps   # CI runs with RUSTFLAGS=-D warnings
cargo build --lib --target thumbv7em-none-eabihf   # also thumbv6m-none-eabi; both gate CI
panic-check/run.sh                # no reachable panic, both thumb targets; needs llvm-tools
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
uv venv && uv pip install pyulog  # once, for --check; .venv is gitignored and found automatically
data/fetch.sh                     # fetch + verify the manifest
data/fetch.sh --verify            # checksums only, no network
data/fetch.sh --check             # convert each .ulg and replay it, assert expectations
data/fetch.sh --add <url> [name]  # download once, append a manifest line to commit
uv run tools/ulog2replay.py log.ulg -o log.csv [--reference]   # ULog -> replay CSV
```

**`uv` is the package manager and the runner for everything under `tools/`.** The converter
declares `pyulog` inline (PEP 723), so `uv run tools/ulog2replay.py` resolves it with no
virtualenv to create or keep current — a tool run a few times a year is the one whose setup
instructions rot. `data/fetch.sh` cannot use `uv run` (it invokes a single interpreter), so it
takes `.venv/bin/python` when that exists and honours `PYTHON=` otherwise. Both paths produce
byte-identical CSVs. Nothing here is on the Rust test path.

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
driving `StepTooLong`, burst logging that forced a median rate estimator, a 2 h log guarding the
f64 timestamp parse, old field spellings). Adding or dropping a log means saying which behavior
it uniquely covers — and invalidates every sentence that quantifies the corpus. "Four of the five
logs", "the worst ordinary interval is 65 ms", "rates from 50 Hz to 400 Hz": these sit in doc
comments and `GOALS.md`, nothing pins them, and by the time anyone re-derived them two were wrong
and one described a corpus of a different size. Grep for `five logs`, `corpus` and `manifest.txt`,
and re-derive from `data/logs/*.csv`, before committing a new entry. Changing a `Config` default or the `summary` line requires updating the
affected expectations.

Expectations are matched pair by pair as substrings, so **adding** a key to the `summary` line is
safe and pinning new behavior there is cheap — `align=`, `resets=`, `alpha0=` and `heading=` were
added that way, and each now guards a decision that would otherwise rot into a comment (`alpha0=`
catches the coarse log's 35575 barometer rows going from fused to `NoReference`, which no other key
noticed; `heading=` is the validity verdict on the initialization window, which catches a yaw
reported valid that no magnetometer ever observed — taken at the end of the log it would only
restate `transitions=`). Renaming or removing a key breaks every entry at once.

**Two corpora, two licences, two manifests.** The PX4 logs are CC BY 4.0 and could be redistributed;
they are fetched rather than committed for size, not for terms. INSANE is BSD-2 with a
non-commercial rider, so it cannot be bundled into an MIT crate at all and needs its own manifest
rather than riding the default fetch (GOALS.md, Validation). Adding a data source means saying
which behavior it uniquely covers **and** under what licence — and for a restricted one, that
measured scalars are publishable while converted CSVs and plots stay out of the repository.

**Converter changes are batched.** Regenerating the corpus is not free — logs fetched, `pyulog`
installed, every log reconverted, replayed, and every moved expectation explained. Land changes
that move `tools/ulog2replay.py` output together, with one `--check` run and one manifest diff
naming, per moved expectation, which change moved it. Three separate diffs each obscure the last.

**One statistic, one implementation.** The Rust replay harness is the only thing that *computes* a
statistic; it emits per-fusion rows and scalar keys on the `summary` and `score` lines. The Python
tools read those and aggregate, plot, or compare against the EKF2 reference — they never recompute
a number the harness already defines. A quantity that could be produced by both paths belongs to
the harness, because that is the one CI runs. Cross-log and against-reference metrics are defined
once, in Python.

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

Cite `file:line` at the sha you read; `~/projects/PX4-Autopilot` and `~/projects/ardupilot` are
current checkouts, and a bare path rots as the tree moves. Cite it in **one** place — the document
that owns the claim — and have the others point there. PX4's reset timeouts were stated in three
files with two different descriptions of the condition before anyone checked the source: they are
`reset_timeout_max` (7 s of horizontal inertial dead reckoning) and `hgt_fusion_timeout_max` (5 s
of failed height fusion) at `src/modules/ekf2/EKF/common.h:515-517`, and `README.md` owns them.

Comparing against them is a design tool, not just a fact check. `Validity` and
`predicted_validity` exist because a comparison showed both estimators answer *which output can I
use* per quantity while this crate answered only *how bad is the worst thing*; the gap was real
and had already cost the corpus a regression guard. When a reporting or API question comes up,
look at what those two publish before inventing something.

## Architecture

One published crate, `no_std`, `forbid(unsafe_code)`, `deny(missing_docs)`, allocation-free,
edition 2024, MSRV 1.89, one dependency (`nalgebra` with `libm`).

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
- `src/math.rs` — the primitives the equations share: `skew`, `exp_quat`, `wrap_pi`,
  `enforce_symmetry` (42). Pure, stateless, and unit-tested against their definitions; no filter
  path calls them yet, which a module-level `expect(dead_code)` says and the last caller to land
  must delete.
- `src/state.rs` — `State` (nominal, 16 values), `Covariance`/`CovarianceMatrix` (15×15), and
  `ErrorState`, whose discriminants define the covariance ordering `[δp δv δθ δβa δβg]`.
- `src/units.rs` — typed scalars/vectors. Types carry the claims that cause bugs — frame,
  value vs noise, sign convention — not units: SI throughout, and a constructor names a unit
  only where sources commonly supply another (`Radians::from_degrees`, `body_deg_per_s`).
  Vector constructors name the frame (`Position::ned`, `AngularRate::body`) and convert ENU/FLU
  input (`Position::enu(..).to_ned()`, `AngularRate::flu`); noise types are built `from_sigma`
  or `from_variance`. No `From<[f32; 3]>` on framed types: `.into()` would claim a frame
  silently. Payloads are `nalgebra` `Vector3<f32>` / `UnitQuaternion<f32>`, with `.to_array()`
  for callers on another version.
- `src/frames.rs` — `Ned`, `Enu`, `Body` as sealed zero-sized type parameters on quantities.
- `src/geodetic.rs` — `Geodetic` (f64 lat/lon/height) and `LocalOrigin`, the tangent plane of
  equations (43)–(44), exact via ECEF (fixed-iteration inverse, no data-dependent loops). The filter owns the origin: `fuse_gnss_geodetic` places it on the first
  fix (under the estimate, or at the fix after a coarse start), a static start clears it.
- `src/config.rs` — tuning. Defaults are **placeholders** except the three listed under "How
  defaults get decided"; each doc comment records why. Preserve that habit: a default
  justified by data says so.
- `src/health.rs` — `Propagation` (`#[must_use]`), `Fusion` (not, deliberately — see its doc
  comment), `SourceHealth`, `Status`.
- `examples/replay.rs` — the normalized CSV format and the offline harness.
- `panic-check/` — a second, unpublished crate: a bare-metal binary calling the whole public
  API, plus `run.sh`, which links it and reads the panic paths back out of the ELF. A workspace
  member so that one `Cargo.lock` covers both, but not a *default* member, so `cargo test`,
  `cargo clippy` and `cargo package` at the root never try to build a `no_std` binary for the
  host. Its build knobs live in `run.sh`, not in its `Cargo.toml`, where a member's `[profile]`
  would be ignored. The only way in is `panic-check/run.sh`.

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
  `NotFinite`, `InvalidVariance`). Three entry points — `initialize`, `initialize_coarse`,
  `initialize_from` — and each reports an `Alignment`; `alignment_of` classifies without
  mutating. `is_aligned` reads the covariance against `Initialization`'s sigmas, so promotion is
  measured rather than timed — except heading, which no covariance can promote because
  stillness never observes yaw; that one waits for a magnetometer.
- **`Status` is the summary; `State::validity` is the detail.** Six per-quantity flags derived
  from the covariance against `Config::accuracy` (the one knob meant to be supplied, since
  mission accuracy is not derivable), plus "was this ever established" — the `Unestablished` flags
  a coarse start sets on position and velocity, and the one a window with no magnetometer sets on
  heading, since stillness observes tilt but never yaw. A prior is not an estimate, and
  `sigma_yaw` equals `Accuracy::heading` exactly, so the covariance cannot tell them apart. `Eskf::predicted_validity` answers the arming question instead: valid now,
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
- **Nothing in `src/` panics.** No `unwrap`, `expect`, `panic!` or `unreachable!` outside
  `#[cfg(test)]`. On `thumbv6m` a panic is a `udf` and the vehicle is a brick, which is why bad
  input is reported through a typed outcome rather than asserted on. The matrix arithmetic the
  equations bring is where this gets broken — `try_inverse` returns an `Option` and `nalgebra`
  indexing panics out of range. Refuse or saturate; never unwrap. `panic-check/run.sh` gates it
  in CI by linking the public API for both thumb targets and failing on a surviving
  `core::panicking` reference, which catches the indexing nobody wrote down as well as the
  `unwrap` somebody did. It also refuses to run if `panic-check/src/main.rs` is missing any
  `pub fn` in `src/`, so a new entry point has to be linked into it — matched on the name, so
  one call per name is enough and a rename is what breaks it. `README.md` owns the two
  boundaries: `debug-assertions = false`, and `opt-level = 3` or `"s"`.
- Frames and units are fixed at the boundary: NED navigation frame, FRD body, Hamilton
  quaternion scalar-first, down-positive gravity. Not configurable.

### Adding a measurement source costs more than a `fuse_*`

Every source touches the same six places, and three of them are public:

- `Diagnostics` gains a field and `sources()`'s return type changes length
  (`src/health.rs:473-500`) — `#[non_exhaustive]` covers the new field, but not the array length,
  so settle the source set before publishing.
- `Gates` gains a threshold, with its degrees of freedom stated.
- `Timeouts` is **global**, not per-source (`src/config.rs:84-95`): there is no per-source entry to
  add, and giving a source its own threshold is a design change. See #56.
- `Validity` and `predicted_validity`: decide whether the source constrains a quantity, and say so.
- A `summary` key in `examples/replay.rs`, pinned per log in `data/manifest.txt`, plus a corpus log
  that uniquely covers the source — or an honest note that none does.
- The README fusion table, the `lib.rs` doctest, and `EQUATIONS.md`'s mapping table.

### Three kinds of noise number, easily confused

- **`R`, measurement noise** — a per-call argument to every `fuse_*`, never stored in `Config`,
  because the accuracy of a fix is a property of that fix. Passed as `PositionNoise` and friends,
  built from σ or variance as the source reports it. GNSS supplies its own (`eph`/`epv`,
  `s_variance_m_s` — a σ despite the name — squared into the replay CSV's variance columns by
  `tools/ulog2replay.py`). PX4 logs no barometer or
  magnetic-heading variance, so the converter substitutes constants and says so; the honest
  source for those is bench characterization of the residual after calibration, which is a
  different activity from calibration itself (this crate does no calibration — that is the
  application's job).
- **`Q`, process noise** — `Config::imu` (`ImuNoise`), continuous-time densities.
- **`P0`, initial state uncertainty** — `Initialization::sigma_*`. A prior on the *state*, not on
  any measurement; `sigma_yaw` >> `sigma_tilt` because gravity pins tilt and yaw inherits the
  magnetometer's error.

A window taken **at rest** also fixes `α₀`, the barometric reference (`StaticSample::baro` →
`Eskf::baro_reference`, equation (30)); one taken in motion keeps whatever reference the flight
already had, since it cannot claim the altitude it reads is the ground. Measured by
`init::at_rest`, not read off the `Alignment`: `classify` reports a short window as
`Coarse::WindowTooShort` *before* it ever measures motion, so a still 0.8 s window on the ground
has an honest reference while a moving window of any length does not. It is neither of the three above: a constant, not noise
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
made — check it before changing scope; the non-goals list is deliberate. Its landscape carries a
survey date, so write competitor facts in a shape that survives re-checking: "a few dozen
downloads a quarter" keeps, "~36 in the last 90 days" was already 33 when someone looked, and
"nine minor versions behind" became ten when `nalgebra` shipped.

The README is not compiled — `lib.rs` does not `include_str!` it — so its snippets rot silently;
its old API block went missing `Status::Aligning`, `Fusion::Reset`, and `Fusion::NoReference`
without anything failing. When an enum or signature changes, check the README's tables and quick
start against the `lib.rs` doctest, which is the compiled source of truth. Snippets handle every
`#[must_use]` outcome rather than `let _ =`; the README is where integrators copy from. `Fusion`
no longer carries the lint, because `Diagnostics` covers acceptance and rejection, but a snippet
still shows the refusals it does not cover rather than dropping them. Cross-doc anchors
(`DESIGN.md#measurement-rejection` from `GOALS.md`) break on heading renames — grep for
`.md#` before renaming one.
