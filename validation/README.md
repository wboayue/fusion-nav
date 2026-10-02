# validation/

The pages linked from [VALIDATION.md](../VALIDATION.md), and what produces them.

- `src/*.md`: the templates. **Edit these**, never the pages beside them. A number or figure
  enters a page only through a placeholder, which `tools/validation.py`'s docstring lists, and
  it names the run it came from.
- `*.md` here and `../VALIDATION.md`: the rendered pages, each opening with a comment naming
  its template.
- `figures/<run>/<slug>.png`: one directory per scenario (`mission`) or corpus run
  (`89a498ce-raw`), drawn by `tools/replay_report.py --figures`. Only the figures a page places
  are drawn.

## Regenerating

```console
$ tools/validation.sh           # replay everything, redraw, rewrite the pages
$ tools/validation.sh --check   # fail if a page's text is not what a fresh run renders
```

Local, never CI. It needs the fetched corpus, pyulog and uv (`data/README.md`), and it runs
every gate it publishes: `data/anees.sh` and `data/fetch.sh --compare`. It takes a few minutes.
Run it after a change that moves a `score`, `summary`, `anees` or `agreement` line, and before
a release (#47).

`--check` compares text, not PNGs, because matplotlib's output is not byte-stable across
builds. It checks that every placed figure exists.

## The cost page

`cost.md` is the exception to "local, never CI". Its numbers are `{{footprint}}` placeholders
on `data/footprint.txt`'s pins and need no replay, so it renders and checks alone:

```console
$ python3 tools/validation.py render validation/src target/validation . --only cost.md
$ python3 tools/validation.py check validation/src target/validation . --only cost.md
```

CI's `check` job runs it, beside the other stdlib self-tests, so a re-pin that leaves the page
behind fails there. `tools/footprint.sh` points here when a pin moves.

## Size

The committed figures have a budget, which `tools/validation.sh` enforces and states. The
crate's `Cargo.toml` excludes this directory, so the figures cost the repository and never a
download.

## Licenses

The corpus figures are drawn from PX4 Flight Review logs, CC BY 4.0, credited on the page that
shows them. The simulator's are this repository's. INSANE and UrbanNav contribute measured
scalars only: INSANE's license and UrbanNav's lack of one keep plots and converted data out of
the repository.
