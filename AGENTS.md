# AGENTS.md

This file provides guidance to coding agents working with code in this repository.

## Status

**`EQUATIONS.md` is implemented.** #31 closed with stage 9 (#110, #111): (1)–(44) are built
except (31)–(33), the three-axis magnetometer, which is out of scope, and (5′)'s subtraction,
which is #59's. The `**Stub.**` marker survives on those two and nowhere else, and every status
banner says built rather than intended. What is left is mostly measurement and publication —
#41 (cost on hardware), #8 (the EKF2 comparison), #47 (the release), #89 (ANEES) — plus two
defects the consistency keys of #112 surfaced on `2c42096b`: every corpus source is correlated
(`acf1_` 0.10–0.99) while (24) fuses it as white (#117); and that overconfidence is the lockout
precondition "report, do not self-recover" accepted, which #116 replaces with recovery on by
default behind per-correction `Config` opt-outs. #118 gated GNSS height apart from horizontal
position, which removed the lockout fusing that barometer caused. #119 (#122) estimates the
barometric offset beside the 15-state covariance, equation (30′), walking at
`Config::baro_offset_walk` (PX4's 0.13); GOALS' "Barometric reference as an estimated offset"
records why estimated rather than a consider state — 1.035 on `moving_start`, but 29310 barometer
rejections on `2c42096b`. #115 (#127) seeds that offset from the estimate on any start that
leaves no reference, correlated with the height it was read against (`P_xb = −P[:, D]`), behind
`Config::baro_reference_from_estimate`: `2c42096b` fuses its barometer (35575 rows refused → 4,
`rejected_gnss_hgt=0`, `Healthy`; 3825 rejected at `baro_offset_walk = 0`), `moving_start`
`nees_pos` 1.0877. Beside EKF2 there, horizontal agrees within metres and height does not: EKF2
follows the barometer's ~12 m climb, this filter GNSS height's low frequencies — #8's to explain.
Order: #89 (the gate the remedies are judged by), #117, #116, then #8. Apart from that order,
#131 blocks #86's tailsitter: `Validity`, the alignment latch, (36′) and heading adoption split
attitude by body axis, which is tilt and heading only near level. #129 (#132) already made the
tooling read 90° of pitch, as quaternions and tilt/heading panels.

**Every source the crate publishes is fused; no `fuse_*` is a stub.** Initialization is real —
equations (5)–(8), so the filter starts at the attitude and biases the window yields — `predict`
propagates the nominal state *and* its covariance, (9)–(22), and a GNSS position, a GNSS velocity,
a barometric altitude or a magnetic heading corrects both: the update of (23)–(27) in Joseph form,
the observation models (28), (29), (30) and (34)–(36) with the levelling variance (36′), the gate
of (37)–(38) and the injection and reset of (39)–(41) all exist, in `src/update.rs` and
`src/observation/{gnss,baro,mag}.rs`. So `mission` scores 0.240 m of horizontal RMSE, 0.188 m/s of
velocity and 0.249 m of height where dead reckoning scored 1261 — 0.083 before (30′), whose
walking offset hands the low frequencies to GNSS height, which a simulated barometer that never
drifts reads as pure loss (`data/scenarios.txt`).
The gate turns a fix down rather than taking everything offered — on the corpus, where
`a299e722` refuses 278 of its 609 velocity solutions, a receiver its own differenced positions
contradict (#105 settled that the harness does *not* floor `R` as both production estimators do,
and `r_policy=raw` pins that); the
barometer and the magnetometer have never been turned down there, 42 298 altitudes and 49 229
headings accepted, so `rejected_mag=0` is a measured zero rather than a structural one. Between
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
attitude that is finally observed approaches 1 from below, which is #89's to read as a bound.

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
late fix leaves the error (#52 owns the cause).

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
window commits. It is unreachable and that is measured: `floored=0` on all five corpus logs,
1 398 114 epochs on the 2 h one, and every replay CSV byte-identical to the same log replayed
without it. `math.rs`'s `FLOOR` owns the headroom figures and every other mention cites it.
And `predicted_validity` stopped meaning *aiding is arriving*: `P` is projected
`Accuracy::horizon` forward with nothing fusing and each quantity tested at the far end, **or**
counted because a constraining source is being accepted. Tilt is what it bought — a static start
holds tilt 3.82 s, so a 1 s horizon arms and a 6 s one does not, where before it predicted its own
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
but a few entries do name what is unbuilt (ANEES, the GSF yaw estimator), and those are on the
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

**Sequencing hazard, and what it taught:** #31's stages were stacked branches while the
signature-changing issues (#21, #25) changed the API underneath them, so an API change had to land
*before* the stage that built on it. The hazard is rebase cost between branches, not users: the
stages are done, but #25 and #21 are still open, and a branch stacked on a surface another branch
is changing inherits it. It
held throughout: #58 landed before stage 5, the first code to read
`Config::gates`, so the gate reads a `Gate<M>` typed by its degrees of freedom rather than a bare
`f32`, and stage 5 then decided the default percentile from replay (`P999`) in the diff that first
turned a fix down at all — the corpus stayed at `rejected=0` on all five logs until (29) reached a
receiver whose velocity it refuses; #61 landed before stage 2, so a quaternion reaches `Attitude` only through
a constructor naming its convention (`body_to_ned`, `ned_to_body`, `flu_to_enu`, `flu_to_nwu`) and
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
  came from* (`degraded_after`: 7992 status flaps at 1.0 s on `2c42096b`, 888 at 2.5 s). A
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
cargo test --lib eskf::tests::a_step_over_the_limit_is_refused_but_the_time_still_passes  # one test
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
cargo run --example replay        # replays data/flight.csv -> target/replay.csv (CI smoke test)
cargo run --example replay -- <input.csv> <output.csv> [truth.csv]   # truth adds a `score` line
cargo run --example replay -- data/flight.csv target/replay.csv data/flight.truth.csv

cargo run --example simulate      # seeded flights with truth -> target/sim/<scenario>{,.truth}.csv
cargo run --example simulate -- flight data   # regenerate the committed data/flight.csv

data/bench.sh                     # score every scenario against data/scenarios.txt; a CI gate
data/bench.sh mission static      # only these
data/expect.sh --self-test        # the comparator both bench.sh and the manifest rules read
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
uv run tools/ulog2replay.py log.ulg -o log.csv [--reference]   # ULog -> replay CSV
uv run tools/replay_report.py in.csv out.csv [truth.csv] \
    --reference ref.csv --summary summary.txt -o report.html   # one HTML per log
```

`--reference` writes EKF2's own solution beside the replay input, never into it. Its state/covariance
index map is keyed on `n_states`, because EKF2's covariance layout changed while the entry count did
not — both eras report 24 entries meaning different things, so no field spelling distinguishes them.
Three of the five corpus logs therefore supply no attitude σ at all, two supply no origin, and the
LPE log is refused outright. `data/README.md`, "What `--reference` writes, and what it cannot", owns
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
`score` keys. It refuses a set of files that do not describe one run — `epochs=` against the epoch
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
caught the coarse log's 35575 barometer rows going from fused to `NoReference`, which no other key
noticed, and now says where each log's reference came from — `window`, `estimate` or `none`; `heading=` is the validity verdict on the initialization window, which catches a yaw
reported valid that no magnetometer ever observed — taken at the end of the log it would only
restate `transitions=`). `floored=` is the odd one: it pins behaviour that must
**not** happen, a count of variances raised to the floor of (42′) that reads zero on every log, and
nothing else on the line would notice if it started — a floored variance only makes the estimate
more conservative, so it moves neither `rejected=` nor `transitions=`. A key whose interesting
value is the one it does not have still earns its place. `r_policy=` is odder still and earns it
differently: a compile-time constant rather than a count, so it cannot regress and a fixture in
`examples/replay.rs` carries the guard instead. What it buys is that a published figure names the
`R` policy that produced it, which #8's comparison against EKF2 has to state either way.
Renaming or removing a key breaks every
entry at once.

**Two corpora, two licences, two manifests.** The PX4 logs are CC BY 4.0 and could be redistributed;
they are fetched rather than committed for size, not for terms. INSANE is BSD-2 with a
non-commercial rider, so it cannot be bundled into an MIT crate at all and needs its own manifest
rather than riding the default fetch (GOALS.md, Validation). Adding a data source means saying
which behavior it uniquely covers **and** under what licence — and for a restricted one, that
measured scalars are publishable while converted CSVs and plots stay out of the repository.

**A third corpus is generated rather than fetched.** `examples/simulate.rs` writes seeded flights
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
overconfident at once, which is #89 (ANEES over N seeds), after #35 gives it a covariance that
moves. The format is already indifferent to several seeds per scenario.

The comparison itself is shared, not copied: `data/expect.sh` owns `key=value`, `key<=value`,
`key>=value` and `key=lo..hi`, and both readers source it — `data/bench.sh` for
`data/scenarios.txt`, `data/fetch.sh --check` for `data/manifest.txt`. So the pair syntax is one
language, and the two-sided bound a statistic wants landed as one arm in one `case` rather than as
a second comparator — which is what #89's ANEES band against a chi-square bound inherits. It carries fixtures with
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
real. `3949f175` was the corpus's "baseline" and the evidence for `degraded_after` until
`ver_hw=PX4_SITL` showed it was a simulation. Its receiver reports `eph` 0.90 and 10 satellites on
every message, so its fix jitter was the scheduler's. Flight Review hosts SITL logs beside real
ones, and a SITL log is synthetic data without the simulator's truth.

**One statistic, one implementation.** The Rust replay harness is the only thing that *computes* a
statistic; it emits per-fusion rows and scalar keys on the `summary` and `score` lines. The Python
tools read those and aggregate, plot, or compare against the EKF2 reference — they never recompute
a number the harness already defines. A quantity that could be produced by both paths belongs to
the harness, because that is the one CI runs. Cross-log and against-reference metrics are defined
once, in Python.

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
not in a page.

**Say which file a published number came from.** A score is a claim about a specific run, and
`examples/replay.rs` refuses a truth file whose `#` header names a different scenario or seed than
the log's. That check exists because the obvious guard does not work: an epoch counter that fails
to match truth rows catches nothing when a 50 Hz log lands on every fourth row of 200 Hz truth, so
the wrong file scored cleanly and published a figure with nothing tying it to what produced it.
Any later source that pairs two generated files — INSANE (#9), UrbanNav (#60) — needs the same
kind of check, and a file carrying no marker is taken on trust because there is nothing in it to
check.

## How defaults get decided

Three defaults are no longer placeholders, and each records its evidence in its doc comment:
`Timeouts::degraded_after` (replay showed 7992 status flaps at 1.0 s on `2c42096b`), `ImuNoise`
(PX4 and ArduPilot agree within 2x and both sit 10-15x above datasheet), and `Initialization`'s
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
  and pure functions (`level_from_accel`, `heading_from_mag`, `nominal_state`, `classify`,
  `attitude_sigmas`, `initial_covariance`, `baro_reference`, `inertial_acceleration`).
  The `initialize*` methods on `Eskf` call these and commit the result. Tests for the pure
  functions live here; tests of what the filter does with them stay in `eskf.rs`.
- `src/propagate.rs` — `ImuSample` and equations (9)–(22).
- `src/update.rs` — the update every observation shares: (23)–(27) in Joseph form, the gate of
  (37)–(38), the injection and reset of (39)–(41). Generic over `dim(z)` and free of `Eskf`, so it
  is tested against a synthetic `H`. One Cholesky factorization of `S` serves the gate and the
  gain, and the gate runs first, so a rejection computes nothing it could commit. `Eskf::apply` is
  the one path that commits what it returns and records it. Its stack frame is the largest in the
  crate — `update::<3>`, 7816 bytes on `thumbv6m` and 7936 on `thumbv7em`, `Eskf::apply` itself
  inlining away — which is why `reparameterize` applies `G P Gᵀ` block-wise and (30′)'s offset
  enters (27) in blocks rather than as a 16 × 16 (+4168 bytes measured); the figure
  and the #41 that would revisit it are in the doc comments.
- `src/observation/` — one module per sensor, each forming `y`, `H` and diagonal `R_m` and
  nothing else. `gnss.rs` holds (28) and (29), `baro.rs` holds (30), and `mag.rs` holds (34)–(36)
  and the levelling variance (36′); (31)–(33), the three-axis magnetometer, are deliberately unbuilt.
- `src/math.rs` — the primitives the equations share: `skew`, `exp_quat`, `wrap_pi`,
  `enforce_symmetry` (42). Pure, stateless, and unit-tested against their definitions. All four
  have callers as of (16)–(22), so none carries a dead-code allowance any more. Neither does
  anything else in `src/`: the gate of (37) took the last one when it became
  `record_rejected`'s first caller.
- `src/state.rs` — `State` (nominal, 16 values), `Covariance`/`CovarianceMatrix` (15×15), and
  `ErrorState`, whose discriminants define the covariance ordering `[δp δv δθ δβa δβg]`; also
  `Offset`, the barometric offset's row of (30′) (`P_xb`, `P_bb`), crate-private and beside the
  covariance rather than in it, so the public 15 × 15 stays the navigation state's.
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
- `panic-check/` — a second, unpublished crate: a bare-metal binary calling the whole public
  API, plus `run.sh`, which links it and reads the panic paths back out of the ELF. A workspace
  member so that one `Cargo.lock` covers both, but not a *default* member, so `cargo test`,
  `cargo clippy` and `cargo package` at the root never try to build a `no_std` binary for the
  host. Its build knobs live in `run.sh`, not in its `Cargo.toml`, where a member's `[profile]`
  would be ignored. The only way in is `panic-check/run.sh`.

### Invariants worth knowing before editing

- **A window sample's `baro` and `mag` may be held.** A slower sensor's last value repeats across
  IMU epochs, and the harness does exactly that. A mean survives it; any other statistic over
  the window has to count distinct readings, which is why `init::baro_reference` counts a value
  only where it differs from the previous sample's. The corollary for fixtures: `[sample; N]`
  with a barometer is *one reading held*, which sets no reference — the fixtures, both examples
  and the `prelude` doctest all had to be given real scatter.
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
  mutating. `is_aligned` reads the covariance against `ALIGNED_TILT` and `ALIGNED_HEADING` —
  constants, never `Config::accuracy`, which is the mission's and moves only `Validity` — so
  promotion is measured rather than timed — except heading, which no covariance can promote because
  stillness never observes yaw; that one waits for the first `fuse_mag_heading`. Promotion only: the flag
  **latches**, so `Status::Aligning` reports a start that has not been resolved and never
  returns, while `Validity::tilt` stays live and does fall back. Read live the bar is crossed
  703 times on `2c42096b`, whose tilt σ sits over `ALIGNED_TILT` for 79 % of the log, and
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
  measured: at `ImuNoise`'s defaults a static start holds tilt for 3.82 s and heading for 35.8 s.
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
- **The filter gates but never self-recovers.** On sustained rejection it reports
  `DeadReckoning`; `reset_position_to` / `reset_velocity_to` exist for the application to
  decide. Do not add automatic resets — that is a documented decision in GOALS.md. The single
  exception, and its boundary: the first measurement of a quantity initialization never established
  is **adopted** (`Fusion::Reset`), once per quantity, because there is no estimate to step away
  from. After a coarse start that is the first GNSS position and velocity; for heading it is any
  start that observed no yaw, which includes a *static* window carrying no magnetometer, since
  stillness observes tilt and never yaw. Only a seed escapes, having vouched for every quantity.
  The heading adoption is the one that steps **attitude**, by up to half a circle, so it
  reparameterizes the surviving attitude rows by (41) with the exact `R(q̂⁺)ᵀR(q̂)` rather than the
  small-angle Jacobian an update takes, and it carries the `R` of (36′) rather than the caller's
  σ_ψ² — a heading levelled by a coarse tilt is not worth the magnetometer's own variance. Never
  for recovery, never for a quantity that was once known. The barometric reference is the same
  shape without the step: a start that leaves none reads it from the estimate at the first
  altitude once position is established, which moves no state.
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

Every source touches the same ten places, and three of them are public:

- `src/observation/` gains a module forming `y`, `H` and `R_m`, and its `fuse_*` calls
  `update::update` and `Eskf::apply`. This is the cheap part, and the only one the compiler checks:
  a `Gate<M>` of the wrong dimension does not build.
- `Diagnostics` gains a field and `sources()`'s return type changes length
  (`src/health.rs:703-744`) — `#[non_exhaustive]` covers the new field, but not the array length,
  so settle the source set before publishing.
- `Gates` gains a field, a `Gate<DOF>` at the observation's dimension — the type states the degrees
  of freedom, and `Gates::at` needs a line for the new field.
- A source is a verdict, not a sensor: a GNSS fix is two, `gnss_position` and `gnss_height`, gated
  apart (#118), so a sensor whose components fail independently wants a source per component.
- `Timeouts` is **global**, not per-source (`src/config.rs:297-308`): there is no per-source entry to
  add, and giving a source its own threshold is a design change. See #56.
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
- `examples/simulate.rs`, which has to model it — an error table, a `sample`, a row — and the
  column legend its generated headers carry. Nothing connects these two to the replay format at
  compile time, which is why they are on this list rather than left to be discovered.

The *truth* format is the one coupling of this kind that is guarded: `write_truth_header` in
`examples/simulate.rs` and `TRUTH_COLUMNS` in `examples/replay.rs` are still two lists, but the
scorer checks the header against its own and refuses the file, so a column renamed or reordered
stops the run instead of scoring one quantity against another. A new *state* — not a new source —
is what moves those columns.

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
error into `R`, ask whether it persists across readings.

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
altitudes to an invented origin. `2c42096b` is the corpus log that covers the seed; the LPE log
(`7592c9b2…`) yields no barometer rows at all.

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
`data/README.md`. `GLOSSARY.md` defines the vocabulary the rest assume — one entry per term,
pointing at the document that owns the thing rather than restating it, so a definition cannot
drift from the equation it describes. A term a newcomer would have to look up belongs there, not
expanded inline in the document using it. It carries *this* crate's vocabulary rather than the
field's, with one exception: a term PX4 or ArduPilot uses for a different thing, which is where a
reader arrives already holding the wrong definition — `Reset` is recovery there and adoption
here. `GOALS.md` records positioning, the six differentiators, and decisions already made — check
it before changing scope; the non-goals list is deliberate. Its landscape carries a
survey date, so write competitor facts in a shape that survives re-checking: "a few dozen
downloads a quarter" keeps, "~36 in the last 90 days" was already 33 when someone looked, and
"nine minor versions behind" became ten when `nalgebra` shipped.

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
