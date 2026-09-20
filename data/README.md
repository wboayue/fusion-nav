# Replay data

`cargo run --example replay` reads a recorded flight from CSV and writes the estimate back out as
CSV — the normalized log format the [validation harness](../GOALS.md#harness-constraint) is built
on. The format itself is documented in `examples/replay.rs`.

## Synthetic log

`flight.csv` is a small synthetic log checked in so the example runs with no network. It is the
only log CI replays.

```console
$ cargo run --example replay                           # data/flight.csv -> target/replay.csv
$ cargo run --example replay -- <input.csv> <output.csv>
```

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
`min_duration`), `align=` and `alpha0=` (what initialization achieved, and whether it fixed a
barometric reference), `heading=` (`Validity::heading` **as initialization left it** — not as the
log ended, which would only restate `transitions=`), `resets=`, `refused=` and `invalid=` (adopted
measurements, and steps refused as too long or as not a step at all), and `epochs=`,
`transitions=` and `status=`. It needs `pyulog`, so it is a local tool rather than a CI job:
`uv venv && uv pip install pyulog` once, and `fetch.sh` finds the gitignored `.venv` on its own.
The converter declares its own dependency inline (PEP 723), so `uv run tools/ulog2replay.py`
needs no virtualenv at all.

`data/fetch.sh --add <url> [name]` downloads a log once and appends a manifest line to commit; the
files stay out of the repo, the checksums do not. Each entry should cover something no other log
does.

Flight Review logs are [CC BY 4.0](https://review.px4.io/).
