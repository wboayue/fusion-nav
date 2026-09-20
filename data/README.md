# Replay data

`cargo run --example replay` reads a recorded flight from CSV and writes the estimate back out as
CSV — the normalized log format the [validation harness](../GOALS.md#harness-constraint) is built
on. The format itself is documented in `examples/replay.rs`.

## Simulated flights

`examples/simulate.rs` generates flights with **analytic ground truth**, which neither of the
corpora below has: the PX4 logs carry no truth and `--reference` only gives EKF2's own answer. It
writes two files per scenario — `<name>.csv` in the replay format, and `<name>.truth.csv`, the
state a perfect filter would report at each IMU epoch.

```console
$ cargo run --example simulate                         # every scenario -> target/sim/
$ cargo run --example simulate -- mission              # one of them
$ cargo run --example replay -- target/sim/mission.csv target/sim/mission.replay.csv
```

What each scenario covers is in the table in `examples/simulate.rs` — printed by the run above and
carried in both generated files' headers — rather than restated here. Its sensor noise is
deliberately **not** `ImuNoise::default()`: a filter scored against its own assumptions is being
handed the answer key.

Nothing reads the truth files yet. Scoring against them — RMSE, NEES, and the per-scenario
ceilings CI would gate on — is [#16](https://github.com/wboayue/fusion-nav/issues/16) and
[#17](https://github.com/wboayue/fusion-nav/issues/17).

### The log CI replays

`flight.csv` is the `flight` scenario's output, committed with its truth beside it, and the only
log CI replays. Regenerate it in place, never edit it:

```console
$ cargo run --example simulate -- flight data          # data/flight.csv + data/flight.truth.csv
$ cargo run --example replay                           # data/flight.csv -> target/replay.csv
$ cargo run --example replay -- <input.csv> <output.csv>
```

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
