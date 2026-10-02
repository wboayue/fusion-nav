# AGENTS.md

This file provides guidance to coding agents working with code in this repository.

## Status

**`EQUATIONS.md` (1)–(44) is built**, except (31)–(33), the three-axis magnetometer, which is
out of scope, and (5′)'s subtraction, in-motion levelling (#59). Those two carry the `**Stub.**`
marker and nothing else does. Every source the crate publishes is fused: GNSS position, GNSS
height (gated apart from horizontal), GNSS velocity, dual-antenna heading, course over ground,
barometric altitude and magnetic heading. What remains is measurement and publication: #47, the
release, and #41, cost on a board.

What the filter does, and where each decision's evidence lives:

- **Start.** `StaticWindow` folds a still window in as it arrives and seeds attitude, biases and
  the barometric reference, (5)–(8), with tilt correlated to accelerometer bias; a short or moving
  window starts coarse under `Status::Aligning`. `StaticWindow::noise` reports a noise floor,
  (8″), never applied. Evidence: `DESIGN.md`, "Defaults and their evidence".
- **Propagation.** (9)–(22) in increments; a step past `max_predict_dt` is coasted by (22′) on
  `Config::coast`.
- **Time.** Every measurement carries its own `Timestamp` and is fused at that time, (23′),
  against `src/history.rs`; one beyond the history is `Fusion::OutOfHorizon`. `GOALS.md`,
  "Measurement latency".
- **Update.** (23)–(27) in Joseph form, gated at P999 per source, each source fused at its
  equivalent white noise, (24′), with one `τ` per source in `Config::correlation`. The antenna
  lever arm is in `H`, (28′)–(29′). The barometric offset is estimated beside the covariance,
  (30′), seeded from the estimate when no window set one (`GOALS.md`, "Barometric reference as an
  estimated offset"). A diagonal floor, (42′), holds every committed covariance and reads
  `floored=0` on the whole corpus (`DESIGN.md`'s `FLOOR` section).
- **Recovery.** Per source, by adoption, at PX4's timeouts (`Config::recovery`); `Recovery::OFF`
  reproduces the filter that only reports, byte for byte.
- **Health.** `Status` times each source against its own measured period; `DeadReckoning` reads
  horizontal GNSS alone. `Validity` and `predicted_validity` answer per quantity.
- **Boundary.** No public item names an `nalgebra` type. `Eskf::new` validates `Config`, a refused
  call commits nothing, and `src/eskf/adversarial.rs` holds both. Declination comes from a WMM2025
  table `tools/declination.py` generates (`magnetic-model`) unless the caller sets one.

Validation: thirteen PX4 logs (`data/manifest.txt`), the simulator's scenarios
(`data/scenarios.txt`, and `data/anees.txt` for the covariance's honesty on 50 seeds), agreement
with EKF2 on every log (`data/ekf2.txt`), UrbanNav for hostile GNSS against truth and INSANE for
accuracy on a real UAV. `VALIDATION.md` and `validation/*.md` publish them, rendered by
`tools/validation.sh`. Stack frames, type sizes and flash are pinned (`data/footprint.txt`) and
published on `validation/cost.md`, which CI checks against the pins; the whole stack path is not
pinned yet (#202). Replay derives a `Config` from a log (`--derive`).

Known losses, stated in the published pages:

- `correlated` is overconfident on position (`data/anees.txt` asserts the failure): a `τ` read
  through the filter cannot remove it, and an unbiased estimator is #195.
- `7ce66f0d`, a hand launch, levels 12° wrong: (5′), #59.
- A receiver that is persistently wrong while claiming accuracy captures the filter (UrbanNav's
  M8T); no gate percentile prevents it. Detecting it is #181, with `2b2ad123`'s offset epochs
  the case to fire on and `89a498ce` the one not to.
- On `2c42096b`, height disagrees with EKF2 because EKF2 follows its barometer and this filter
  follows GNSS height's low frequencies; horizontal agrees to 0.3 m RMS.

Keep the status banners in `README.md`, `DESIGN.md`, `EQUATIONS.md` and `GOALS.md` saying what is
true (`src/lib.rs` inherits the README's). `EQUATIONS.md`'s is the one to watch: it sits above a
mapping table that separately marks functions as unbuilt, so the two can contradict each other.
The example module docs rot the same way, since `basic` and `degradation` compile in CI and never
run. `GLOSSARY.md` repeats no caveat by design, but entries that name unbuilt work (the GSF yaw
estimator, #165) are on the list.

## Backlog

One tracking issue is open and owns ordering for its area; read it before starting work there.

- **#10** — replay validation. Carries which issue answers which question, and the measured
  numbers those answers are argued from. The questions themselves — self-consistency without
  truth, accuracy with it, a correct rejection needing truth *and* hostile measurements, and the
  covariance's own honesty underneath all three — are `GOALS.md`, "Three questions, three kinds
  of source".

Why `EQUATIONS.md` was built in the order it was is `DESIGN.md`, "Staging the implementation":
reasoning goes somewhere a tracker's closure cannot take it.

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

**Stacked branches inherit the surface beneath them.** When a branch builds on an API another
branch is changing, land the API change first: the cost is rebase work between branches, not
users. A typed surface landed early is what later stages are written against (`Gate<M>` typed by
its degrees of freedom; `Attitude` reachable only through a constructor naming its convention).

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
- **One sentence of evidence, and a link to the rest** (#187). Where the number came from is the
  deciding figure in a sentence; the per-log breakdown, the table, the alternatives measured and
  rejected go to the document that owns the decision, and the comment links the heading. That
  document is one of three: the `GOALS.md` decision, where one exists; `DESIGN.md`, "Defaults and
  their evidence", under a heading named for the item (a repeated heading's bare anchor is refused
  by `tools/check-anchors.sh`, since it moves); or a `data/manifest.txt` note. What a function
  costs now (its frame, a type's size, `.text`) is `validation/cost.md`, through a `{{footprint}}`
  placeholder on the pin and never typed; what decided its form (the rejected form's cost, host
  timings, operation counts) goes to `DESIGN.md`, "Measured cost, by function", and the comment
  keeps why its form was chosen. #200 re-pinned `.text` and left DESIGN's copy of the flash table
  behind, which is why the current figures have no hand-written home. Pending work is one line,
  "Not built: #N", with at most a clause naming what would build it; the argument for waiting
  belongs to the issue, or to `GOALS.md` if it outlives it. The test is a paragraph's subject: a
  log or a byte count, rather than the code, is the paragraph that moves. Test modules are
  exempt: a fixture's comment saying which mutation it survives stays. Links are absolute
  (docs.rs serves no siblings) and reference-style (`[measured]: https://…` at the block's end),
  and `tools/check-anchors.sh` resolves them in `src/` as in the Markdown.

**Moving evidence is an audit, not a cut and paste.** #187 moved 263 figures, and the move found
three things a copy would have carried over:
- **Two copies had already diverged.** `mag.rs` and `EQUATIONS.md` both quoted `moving_start`'s
  gain from (36′), from the same commit, and disagreed (1.720° against 1.665°); only
  `data/scenarios.txt` said why. A figure with two homes is the rot one home prevents: when a
  destination already states it, cut the copy rather than moving it.
- **Context was carrying a date.** "The five logs in `data/manifest.txt`", "the thirteen scenarios",
  fixes "still rejected" that #52 now refuses as `OutOfHorizon`: each was true where it sat, beside
  code of its time, and read as current once lifted into a section of its own. A moved figure
  takes a label for what it was measured on, or is re-measured.
- **Deleted evidence fails silently**, so the PR proves nothing was lost. A script took every log
  hash, `#N` and multi-digit number on the removed comment lines and required each to appear in
  `src/` or a tracked document, and every removed sentence's figures to share one paragraph
  there. Name each miss in the PR as placed or as a cut duplicate, with where the original lives.

## Commands

```bash
cargo test --all-targets          # unit tests: inline `mod tests` across src/, and in examples/replay/main.rs
cargo test --doc                  # README.md (included by lib.rs), plus the item doctests
cargo test --lib eskf::tests::a_gap_is_coasted_on_the_estimated_velocity_and_the_time_still_passes  # one test
cargo fmt --all -- --check
cargo clippy --all-targets --no-deps   # CI runs with RUSTFLAGS=-D warnings
cargo build --lib --target thumbv7em-none-eabihf   # also thumbv6m-none-eabi; both gate CI
panic-check/run.sh                # no reachable panic, both thumb targets; needs llvm-tools
tools/check-anchors.sh            # every `.md#anchor` resolves; --self-test runs its fixtures
uv run tools/declination.py       # regenerate src/magnetic.rs's table at the model and epoch it names
uv run tools/declination.py --model WMM_2025 --epoch 2027.5   # at another; a new model needs its sha pinned
uv run tools/declination.py --check   # the committed table is what it generates
uv run tools/declination.py --drift [--at <year>] [log.csv ...]   # stale? corpus, INSANE and the ±60° grid
uv run tools/declination.py --px4 ~/projects/PX4-Autopilot   # rebuild PX4's table, check the converter's copy
python3 tools/declination.py --self-test   # its fixtures; CI
cargo +1.89 build --lib           # MSRV

tools/footprint.sh                # type sizes, stack frames, flash on both thumb targets vs data/footprint.txt; CI
tools/footprint.sh --pin          # the file's keys at their measured values: re-pin by copying
python3 tools/validation.py render validation/src target/validation . --only cost.md   # after a re-pin
tools/footprint.sh --all          # every key measured, to choose what to pin
tools/footprint.sh --install      # once: its pinned nightly, llvm-tools and both thumb targets
python3 tools/footprint.py --self-test   # the parser's fixtures
# When a frame grows, bisect it: strip one candidate per build and re-measure. #122's +4168 bytes on `update::<3>`
# was not only the 16 x 16 it looked like -- returning `(Covariance, Offset)` as a tuple through
# `reset` cost a 900-byte copy of P on its own.
cargo bench -p bench              # host timings of predict, every fuse_* and a start; never gated
cargo test -p bench --benches     # each benchmark once, in debug, as CI runs them

cargo run --example basic         # minimal integration loop
cargo run --example degradation   # timeouts, status transitions, application-driven recovery
cargo build --example embedded --target thumbv7em-none-eabihf [--features defmt]   # no_std; CI builds, never runs
cargo run --example replay        # replays data/flight.csv -> target/replay.csv (CI smoke test)
cargo run --example replay -- <input.csv> <output.csv> [truth.csv]   # truth adds a `score` line
cargo run --example replay -- data/flight.csv target/replay.csv data/flight.truth.csv
cargo run --release --example replay -- --derive <log.csv> > config.rs   # a Config from a log; evidence on stderr
cargo run --example replay -- --derive data/flight.csv > data/flight.config.rs  # regenerate the fixture a test pins
cargo run --example replay -- --set correlation.gnss_height=29 <in.csv> <out.csv>   # any derivable field; `set=` names it
REPLAY_ARGS="--set ..." data/anees.sh correlated   # the ANEES gate under a derived Config
cargo test --example replay -- --ignored drift_scatter --nocapture   # the drift estimator's scatter DESIGN quotes

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
`status=`) matched against the `summary` line `examples/replay/main.rs` prints.

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
`--r-policy px4`, so it cannot regress and fixtures in `examples/replay/main.rs` carry the guard
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
literal verdicts for the same reason `examples/replay/main.rs` does — the expectations in both files
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

**A quantity read through the filter is the filter's as much as the sensor's.** #51 read each
source's correlation time off its innovations, as the `Correlation` defaults had been read, and on
the one scenario with known `τ` it came out 1.7–6× short at every lag: the filter follows part of
the error, so its innovations decorrelate faster than the error does. Taken as the value it made
`correlated` more overconfident than the defaults (`anees_pos` 3.16 against 1.45, as #51 measured
it). Before
deriving a sensor property from innovations, check it on a simulated source whose answer is
known, and treat what survives as a bound, not a value.

**A reference with errors of its own puts their shape in the statistic.** #51's barometer walk
was read against GNSS height, whose correlated error lifts the structure function to a plateau
over a few of its `τ`: a slope read at 60–240 s reported that rise as barometer drift, 0.15 on
`2c42096b` and 0.20 on `7ce66f0d`, and both moved when the lag range did. Plot the statistic
across its whole range before fitting a piece of it, and suspect a figure that flips with a
window you chose.

**An ablation that removes the rows it counts proves nothing about them.** #169's first draft
dropped eleven off-level fixes and quoted 19 rejections falling to 2, but seven of the eleven
were rejections themselves. The review's version drops only the four *accepted* ones, the
claimed cause, and a control of four ordinary fixes: 19 → 9 against 19 → 19. Remove the cause,
count the effect elsewhere, and run the same removal on rows that should not matter.

**A log's timestamp is when the message arrived, not when it was true.** At 1 Hz a filter
predicts a second across tens of milliseconds of delivery jitter; at 5–10 Hz the same jitter
reads as along-track position error a centimetre receiver cannot absorb, and #194 first read it
as RTK receivers rejected by the hundred. Before a rate change is read as the sensor, date the
measurement by the source's own clock where it logs one (`time_utc_usec`), and check that the
dating is never later than the log's own.

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

**A comparison is two figures from one build.** Evidence for a form, "inlined it cost 2384
against 1400", ages one number at a time: a later commit re-quoted the current half and kept the
old one, and the subtraction then measured nothing. Two of DESIGN.md's stack pairs had split this
way before #201 found them, `apply_or_recover`'s 984 read as 968 and `fuse_heading`'s 7872 as
7848. So a pair carries the commit it was measured together at, once the code has moved past it;
a current figure that a decision rests on renders from a pin instead (`validation/cost.md`), and
a re-measurement is quoted as a pair of its own. Name a commit `main` holds: a branch's commits
vanish in the squash merge, so a figure measured on a branch whose `src/` matches `main` cites
`main`'s commit.

**A sum of named frames is not a path.** The crate's stack peak was quoted for months as
`fuse_gnss_velocity` plus `update::<3>`, 9504 B. Walked through the rlib's call edges it is
11408, because the deepest frame below `update` is `nalgebra`'s 15 × 15 product, which
`-Zemit-stack-sizes` measures and `tools/footprint.py` keys out as another crate's. A figure built
from the functions someone thought to name is bounded by that list; #202 makes the path a pin.

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
`examples/replay/main.rs` refuses a truth file whose `#` header names a different scenario or seed than
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

Three defaults were the first to stop being placeholders, and each carries its deciding figure in
its doc comment (the rest of `ImuNoise`'s and `Initialization`'s in `DESIGN.md`, "Defaults and
their evidence"):
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
  crate (`update::<3>`, `Eskf::apply` itself inlining away), and the high-water mark is
  `fuse_gnss_velocity` into it, with `observe` and `apply_or_recover` kept out of line beside it
  rather than beneath; which is why `reparameterize` applies `G P Gᵀ` block-wise and (30′)'s
  offset enters (27) in blocks rather than as a 16 × 16. The current frames are
  `validation/cost.md`, which is where #41's will land, and what decided each form is `DESIGN.md`,
  "Measured cost, by function".
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
- `src/magnetic.rs` — the WMM declination table and its bilinear lookup, behind the
  `magnetic-model` feature; `Eskf::place_origin` reads it at every origin placement. The table
  is generated, between marker comments, by `tools/declination.py`; never hand-edit it.
  **It ages, so expect to regenerate it periodically**, about once per model: the grid within
  60° of the equator drifts past GOALS.md's 1° near 2029.0, and NCEI replaces the model every
  five years (WMM2030 is expected in December 2029). Run `--check` and `--drift` before each
  release and regenerate on GOALS.md's rule ("Magnetic declination from a table"). A regeneration
  moves `declination_model=` in `data/manifest.txt` and the INSANE pins, which replay under
  `--declination model`, and nothing else: re-pin both (`--drift` refuses until the manifest
  agrees with its lookup) and re-render the pages. The lookup exists twice, `declination_at` and
  the script's `lookup`, and `rust_max=` is what ties them. The converter's copy in
  `tools/ulog2replay.py` is PX4's table, not this one, and follows PX4.
- `src/config.rs` — tuning. Each default's doc comment says where its number came from (a cited
  PX4/ArduPilot source, a replay measurement, or **placeholder**, as `Accuracy`'s are). Preserve
  that habit: a default justified by data says so, in a sentence, and `DESIGN.md`, "Defaults and
  their evidence", holds the measurement.
- `src/health.rs` — `Propagation` (`#[must_use]`), `Fusion` (not, deliberately — see its doc
  comment), `SourceHealth`, `Status`.
- `examples/replay/` — `main.rs`, the normalized CSV format and the offline harness;
  `derive.rs`, `--derive`, which reads a `Config` off a log as runs, readings and pure `*_from`
  rules and prints it; `settings.rs`, the `--set` names and their inverse. Two output files:
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
  host. Its build knobs live in `profile.sh`, not in its `Cargo.toml`, where a member's
  `[profile]` would be ignored; `run.sh` and `tools/footprint.sh` both source it, and are the
  only ways in.
- `bench/`: a third unpublished member, criterion benchmarks of the hot path on the host. A
  member rather than a root dev-dependency because criterion turns on `num-traits/std`, which at
  the root would move every replay output (the dev-dependency rule under Replay corpus). Not a
  default member; `cargo bench -p bench` times it, and CI runs `cargo test -p bench --benches`.

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
- A `summary` key in `examples/replay/main.rs`, pinned per log in `data/manifest.txt`, plus a corpus log
  that uniquely covers the source — or an honest note that none does.
- `AXES` in `examples/replay/main.rs`, naming the source's innovation components, since `Innovation`
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
  If the source reports its own `R` and EKF2 floors it, `RPolicy` in `examples/replay/main.rs` owns the
  floor. Harness keys reach the table by prefix (`rejected_`, `nis_`), so those need nothing.
- `examples/simulate.rs`, which has to model it — an error table, a `sample`, a row — and the
  column legend its generated headers carry. Nothing connects these two to the replay format at
  compile time, which is why they are on this list rather than left to be discovered.

The *truth* format is the one coupling of this kind that is guarded: `write_truth_header` in
`examples/simulate.rs` and `TRUTH_COLUMNS` in `examples/replay/main.rs` are still two lists, but the
scorer checks the header against its own and refuses the file, so a column renamed or reordered
stops the run instead of scoring one quantity against another. A new *state* — not a new source —
is what moves those columns.

The converter's `#` header lines are the other coupling, and they are guarded less. Each is an
f-string in `tools/ulog2replay.py` and a parser elsewhere: `# Magnetic declination`,
`# GNSS noise parameters` and `# IMU averaging interval` read by `examples/replay/main.rs`, `Estimator:`, `EKF2 position in replay
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
corpus measured both directions (`DESIGN.md`, `WindowNoise`). Keep its boundary: derived at a defined
moment and reported, never silently retuned in flight, which would cost the determinism claim; a
recovery is an adoption on a schedule `Config::recovery` fixes in advance, not a retuning.
Anything the window cannot honestly measure, the noise to configure above the floor included,
goes to `replay --derive`, which prints a `Config` from a log (#51), not into the filter.

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
