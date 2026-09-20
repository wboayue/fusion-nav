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

`nu*` and `s*` are empty for now: the filter publishes no innovation or innovation covariance, and
the harness does not work them out for itself — see the rule below.

## One statistic, one implementation

The Rust replay harness is the only thing that **computes** a statistic. It emits the per-fusion
rows above and the scalar keys on the `summary` line. The Python tools under `tools/` read those
rows and those keys — they aggregate across logs, plot, and compare against the EKF2 reference —
and never recompute a number the harness already defines.

The test is whether a quantity could ever be produced by both paths. If it could, it belongs to
the harness, because that is the one CI runs. Where a statistic is only meaningful across logs or
against the reference — distance from EKF2, the corpus table — it lives in Python and is defined
there once.

The same rule is why `nu*` and `s*` above stay empty rather than being computed here from the
measurement and the covariance. The update of equations (23)–(28) is about to own that quantity,
and two implementations of it would eventually disagree — discovered, as these things are, while
somebody chases a filter bug that does not exist.

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
score pos_h=101.112 pos_v=14.025 vel=14.527 pos_h_max=125.000 tilt=11.323 yaw=36.041 …
```

What each key means, and what it can and cannot say on these scenarios, is in the module docs of
`examples/replay.rs`, which owns the definitions. Three things about *using* it belong here:

- **No truth file, no `score` line** — not a line of zeros. Every log in the PX4 corpus below has
  no truth, and `pos_h=0.000` on one of them would claim a perfect filter where the honest answer
  is that nothing knows. `manifest.txt` is untouched by scoring, and `fetch.sh --check` reads
  `summary` exactly as before.
- **Convergence is on `summary`, not here.** `aligned_at=` needs no truth and the corpus pins it;
  one statistic, one implementation.
- **Five scenarios are one-variable departures from `mission` on `mission`'s seed**, so the figure
  that attributes a fault is `score(departure) − score(mission)`, not either alone.

The per-scenario ceilings CI would gate these against are
[#17](https://github.com/wboayue/fusion-nav/issues/17).

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
they do not — an x86-64 and an arm64 build of the simulator write identical scenarios here — but
that is an observation, not the guarantee the filter has below, and the committed file is what CI
compares against itself.

A given seed always gives byte-identical files on one machine, and each sensor draws from its own
stream, so changing one sensor's rate or model does not shift another's noise.

## Determinism

Replay output is bit-reproducible across architectures: the same input and the same toolchain give a
byte-identical CSV on x86-64 and on aarch64. CI asserts it — the `replay determinism` jobs replay
`flight.csv` twice on each of an x86-64 Linux runner and an aarch64 macOS one, then compare a sha256
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
only covers as much arithmetic as the harness exercises, which is little while the equations are
stubs and grows with each stage that lands.

## PX4 corpus

Real flights come from [PX4 Flight Review](https://review.px4.io/). They are fetched rather than
committed: `logs/` is gitignored, and `manifest.txt` pins each log by sha256.

```console
$ data/fetch.sh                 # fetch + verify the manifest
$ data/fetch.sh --verify        # checksums only, no network
$ uv run tools/ulog2replay.py data/logs/<log-id>.ulg -o data/logs/<log-id>.csv --reference
$ cargo run --example replay -- data/logs/<log-id>.csv
```

`--reference` writes EKF2's own solution and innovation test ratios to a second file, for a
side-by-side diff.

`data/fetch.sh --check` converts and replays the whole pinned corpus and asserts the per-log
expectations recorded beside each checksum against the `summary` line the replay example prints.
The keys are `rate=` and `window=` (the IMU rate and the samples it takes to cover
`min_duration`), `align=`, `an=` and `alpha0=` (what initialization achieved, whether a moving
window measured the vehicle's own acceleration from GNSS velocity — `ā_n` of equation (5′), which
only a moving start reports — and whether it fixed a barometric reference), `heading=` (`Validity::heading` **as initialization left it** — not as the
log ended, which would only restate `transitions=`), `resets=` (measurements adopted outright),
`aligned_at=` (seconds from the end of the window to the first valid attitude, or `never`),
`rejected=` and `discarded=` (the gate's verdict, and everything that never reached it — a
variance of zero or less, a NaN, an altitude with no reference), `refused=` and `invalid=` (steps
refused as too long or as not a step at all — propagation, not measurements), and `epochs=`,
`transitions=` and `status=`. It needs `pyulog`, so it is a local tool rather than a CI job:
`data/fetch.sh --venv` once, which installs the version the converter pins, and `fetch.sh`
finds the gitignored `.venv` on its own.
The converter declares its own dependency inline (PEP 723), so `uv run tools/ulog2replay.py`
needs no virtualenv at all.

That dependency is pinned to an exact version, and `--check` stops if the interpreter it is about
to convert with holds a different one. The two paths only produce the same CSV for the same
pyulog, and a converter that changed underneath the corpus would move the expectations below with
nothing in the repository to blame. `tools/ulog2replay.py` holds the pin; `fetch.sh` reads it from
there.

`data/fetch.sh --add <url> [name]` downloads a log once and appends a manifest line to commit; the
files stay out of the repo, the checksums do not. Each entry should cover something no other log
does.

Flight Review logs are [CC BY 4.0](https://review.px4.io/).
