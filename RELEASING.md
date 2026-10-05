# Releasing

How a version of `fusion-nav` reaches crates.io. What the version number promises is
[GUIDE.md, "Versioning"](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#versioning);
this file is the procedure, for a maintainer, and is not packaged.

Each step is on `main` at the commit to be published, with a clean tree. A step that fails stops
the release: fix it in a PR, then start again.

## Before the version bump

1. **CI is green on that commit**: both thumb targets, MSRV, doctests, the packaged crate and the
   docs.rs build all run there.
2. **The corpus still says what the manifest pins.** `data/fetch.sh --check`, which CI cannot run.
   A moved expectation is a finding about the code, explained in its own PR before the release.
3. **The declination table is current.** `uv run tools/declination.py --check` and `--drift`.
   Regenerate it if GOALS.md's rule
   ([magnetic declination from a table](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#magnetic-declination-from-a-table-read-where-the-origin-is-placed))
   says it is stale; that moves `declination_model=` and the INSANE pins, and the changelog
   names it.
4. **The positioning still holds.** Re-survey GOALS.md's
   [what would falsify this positioning](https://github.com/wboayue/fusion-nav/blob/main/GOALS.md#what-would-falsify-this-positioning):
   its triggers are events, and nothing else watches them. Update the survey date and say in
   the changelog which triggers moved, if any.
5. **`Cargo.toml`'s `description` names every source fused**, against GUIDE.md's measurement
   table.
6. **An API change is decided, not pending.** Every open issue that would change the public
   surface either lands or says on the issue that it is deferred, and whether its change would be
   additive or a minor bump.

## The release commit

7. Set `version` in `Cargo.toml`, move the changelog's `(unreleased)` heading to the version
   and date, and point the README's quick start at the published version.
8. Commit, then regenerate the validation pages on that commit: `tools/validation.sh`, which
   refuses a dirty tree, then commit its output. The diff should be stamp lines alone; any
   other moved line is a number that moved and needs a sentence. `tools/validation.sh --check`
   passes.
9. `cargo publish --dry-run`, and `cargo package --list` reads as the files `include` names.
10. `cargo +nightly doc --no-deps --lib --target thumbv7em-none-eabihf --features defmt` with
    `RUSTDOCFLAGS="--cfg docsrs"`, opened, as docs.rs will render it.

## Publishing

11. `cargo publish`. It cannot be undone: a version can be yanked, never replaced.
12. Tag the commit `v<version>` and push the tag; a GitHub release carries the changelog entry.
13. Check the crates.io page and docs.rs once each has built, and update the issues the release
    falsified, as AGENTS.md asks of a merge.
