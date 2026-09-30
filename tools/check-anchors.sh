#!/usr/bin/env bash
#
# Resolve every cross-document Markdown link in the repository against the file and the
# heading it names.
#
#   tools/check-anchors.sh              every tracked `.md`
#   tools/check-anchors.sh --self-test  fixtures with known verdicts
#
# The documents cite each other by anchor -- `DESIGN.md#measurement-rejection` from
# `GOALS.md`, `GOALS.md#alignment-beyond-the-static-window` from `README.md` -- and a heading
# rename breaks every one of them silently, since Markdown has no link resolution and a dead
# anchor renders as a working link that lands at the top of the page. The instruction this
# replaces was "grep for `.md#` before renaming a heading", which is a check a machine does
# better and a reader should not have to carry.
#
# Three link shapes are resolved, and the rest ignored:
#
#   [text](DESIGN.md#anchor)   relative to the linking file's own directory, so
#                              `data/README.md` reaching `../GOALS.md` is resolved from
#                              `data/` rather than from the root.
#   [text](#anchor)            a heading in the linking file itself.
#   [text](https://github.com/<owner>/<repo>/blob/<ref>/DESIGN.md#anchor)
#                              the absolute form `README.md` uses, since the README is
#                              included in the crate's rustdoc (`src/lib.rs`), where a
#                              relative link to a sibling document is served by nothing.
#                              Resolved repo-root-relative, past whatever ref it pins, and
#                              recognised by the `repository` URL in `Cargo.toml` so the
#                              address lives in one place.
#
# A document that rustdoc includes is held to that third form: a relative sibling link in it
# resolves here and 404s on docs.rs, which is the one failure a reader cannot see and this
# tool can.
#
# Anything else -- an external URL, a link to a file that is not Markdown -- is somebody
# else's to check, and checking it would need the network that `GOALS.md`'s harness
# constraint keeps out of CI. That includes a source permalink with a `#L12-L20` fragment,
# which is a citation rather than an anchor.
#
# Slugs follow GitHub's rule: lowercase the heading text, drop ASCII punctuation other than
# `-` and `_`, then turn spaces into hyphens. A repeated heading takes the `-1`, `-2` suffix
# GitHub gives it. Characters above ASCII are kept as written, which is what both GitHub and
# rustdoc do with the `α₀` and `γ` this crate's prose is full of; the two disagree only about
# an *uppercase* non-ASCII letter, which GitHub lowercases and rustdoc does not, and which no
# heading here has. Two constructs are slugged more literally than GitHub slugs them -- a
# heading holding a link, and one with trailing whitespace after its text -- and both fail a
# link that was written correctly rather than passing one that was not.
#
# mawk on `ubuntu-latest` and macOS awk both run this; CI's own output is the evidence
# ("65 links in 6 documents", run 35557144917), which is worth having because the interval
# expression below is the kind of thing mawk has historically wanted a flag for.

set -euo pipefail

# Byte-wise character classes and no locale-dependent case folding, so a heading's `α₀` is
# passed through as the bytes it is rather than being folded or dropped.
LC_ALL=C
export LC_ALL

# Documents rustdoc includes verbatim, and which therefore may not use relative links.
# `src/lib.rs` carries the `include_str!`; this is the list it implies.
INCLUDED_IN_RUSTDOC="README.md"

# ASCII punctuation GitHub drops from a slug. `-` and `_` are not here because it keeps them.
SLUG_DROPS='!"#$%&'\''()*+,./:;<=>?@[\]^`{|}~'

workspace=$(mktemp -d)
trap 'rm -rf "$workspace"' EXIT

# Every heading in a file, as the anchor it can be reached by, in document order.
#
# Fenced blocks are skipped, which is the whole reason this is a parser and not a grep:
# `AGENTS.md` and `data/README.md` are full of shell in fences, and every comment in it
# starts with `#`.
heading_slugs() {
    awk -v drops="$SLUG_DROPS" '
        function slug(text,   i, c, out) {
            gsub(/\t/, " ", text)
            sub(/^#+ +/, "", text)
            sub(/ +#+ *$/, "", text)
            sub(/ +$/, "", text)
            text = tolower(text)
            out = ""
            for (i = 1; i <= length(text); i++) {
                c = substr(text, i, 1)
                if (c == " ") out = out "-"
                else if (index(drops, c) == 0) out = out c
            }
            return out
        }
        /^[ \t]*(```|~~~)/ { fence = !fence; next }
        fence { next }
        /^#{1,6}[ \t]/ {
            anchor = slug($0)
            seen[anchor]++
            if (seen[anchor] > 1) anchor = anchor "-" (seen[anchor] - 1)
            print anchor
        }
    ' "$1"
}

# document_links <file> <blob prefix>
#
# Every resolvable link in the file, as `line<TAB>kind<TAB>path<TAB>anchor`, where kind is
# `rel` (the path is relative to the linking file) or `root` (relative to the repository
# root). The blob prefix is what makes an absolute link into this repository resolvable at
# all; with an empty one every such link reads as external. Links inside fenced blocks are
# not links.
#
# An in-page `#anchor` is emitted as the file's own path rather than as an empty one, because
# tab is an IFS whitespace character: `read` collapses a run of them, and an empty field in
# the middle would arrive as a missing one, shifting the anchor into the path.
document_links() {
    awk -v blob="$2" '
        /^[ \t]*(```|~~~)/ { fence = !fence; next }
        fence { next }
        {
            rest = $0
            while (match(rest, /\]\([^()]*\)/)) {
                target = substr(rest, RSTART + 2, RLENGTH - 3)
                rest = substr(rest, RSTART + RLENGTH)

                if (blob != "" && index(target, blob) == 1) {
                    target = substr(target, length(blob) + 1)
                    ref = index(target, "/")
                    if (ref == 0) continue
                    target = substr(target, ref + 1)
                    if (target !~ /\.md($|#)/) continue
                    kind = "root"
                } else if (substr(target, 1, 1) == "#") {
                    kind = "root"
                    target = FILENAME target
                } else if (target ~ /:\/\//) {
                    continue
                } else if (target ~ /\.md($|#)/) {
                    kind = "rel"
                } else {
                    continue
                }

                anchor = ""
                if (index(target, "#") > 0) {
                    anchor = substr(target, index(target, "#") + 1)
                    target = substr(target, 1, index(target, "#") - 1)
                }
                print FNR "\t" kind "\t" target "\t" anchor
            }
        }
    ' "$1"
}

# check_documents <root> <blob prefix> <file>...
#
# Paths are given relative to <root> and reported that way, so a failure names a file the
# reader can open rather than the temporary directory a fixture holds it in. Reports every
# broken link rather than the first: a heading rename breaks several at once, and one run
# should name all of them.
check_documents() {
    local root=$1 blob=$2 doc file line kind target anchor rustdoc rc=0 links=0
    shift 2
    local cache=$workspace/cache
    rm -rf "$cache"
    mkdir -p "$cache"

    for doc in "$@"; do
        # A file awk cannot read contributes no links, and without this it would contribute
        # no complaint either -- the "0 links, all resolved" this tool refuses below.
        if ! (cd "$root" && document_links "$doc" "$blob") > "$cache/links"; then
            echo "  UNREADABLE $doc" >&2
            rc=1
            continue
        fi

        while IFS=$'\t' read -r line kind target anchor; do
            links=$((links + 1))
            case $kind in
                rel) file=$(dirname "$doc")/$target ;;
                root) file=$target ;;
            esac

            # A Rust source is rustdoc by definition.
            case " $INCLUDED_IN_RUSTDOC " in
                *" $doc "*) rustdoc=1 ;;
                *) case $doc in *.rs) rustdoc=1 ;; *) rustdoc=0 ;; esac ;;
            esac
            case $rustdoc in
                1)
                    if [ "$kind" = rel ]; then
                        echo "  RELATIVE   $doc:$line -> $target${anchor:+#$anchor} (rustdoc" \
                            "includes this file; use the repository URL)" >&2
                        rc=1
                        continue
                    fi
                    ;;
            esac

            if [ ! -f "$root/$file" ]; then
                echo "  NO FILE    $doc:$line -> $target${anchor:+#$anchor}" >&2
                rc=1
                continue
            fi
            [ -n "$anchor" ] || continue

            local key=$cache/${file//\//_}
            [ -f "$key" ] || heading_slugs "$root/$file" > "$key"
            if ! grep -qxF -- "$anchor" "$key"; then
                echo "  NO ANCHOR  $doc:$line -> $target#$anchor" >&2
                rc=1
            fi
        done < "$cache/links"
    done

    [ "$rc" = 0 ] && echo "check-anchors.sh: $links links in $# documents, all resolved"
    return $rc
}

# The `<repository>/blob/` prefix this repository's own absolute links carry, from
# `Cargo.toml`'s `repository`, so the address lives in one place. The ref after it is
# whatever the link pins -- a branch, a tag, a sha -- and the path is resolved against the
# working tree either way, which is the file a reader following that link will be shown.
blob_prefix_from_cargo() {
    local repository
    repository=$(awk -F'"' '/^repository[ \t]*=/ { print $2; exit }' "$1")
    [ -n "$repository" ] || return 0
    printf '%s/blob/' "${repository%/}"
}

# The guards here are the fence tracking, the slug rule and the refusals, and none of them is
# visible in a passing run over documents that happen to be correct. These fixtures are
# literals with the verdict written beside them, the way `data/expect.sh`'s are: each one was
# checked by breaking the rule it names and watching it fail.
self_test() {
    local sandbox=$workspace/sandbox passed=0 failed=0
    mkdir -p "$sandbox/sub"

    cat > "$sandbox/target.md" <<'EOF'
# Target

## Health reporting

### `Status` — how bad is the worst thing

## The barometric reference α₀

## :: the operator

## Repeated

## Repeated

```bash
# Not a heading
data/fetch.sh --check
```
EOF

    # Appended with `printf` rather than written in the heredoc above, because trailing
    # whitespace in a source file is what every editor and formatter strips -- and the
    # fixture would then be asking about a heading that has none.
    printf '\n## Trailing spaces   \n' >> "$sandbox/target.md"

    # t <expected rc> <name> <markdown> [document path] [message pattern]
    #
    # The document path is where the markdown is written, which is what lets one fixture ask
    # about a subdirectory or about a file rustdoc includes. The message pattern, where given,
    # is what separates a verdict reached for the right reason from one reached by accident.
    t() {
        local want=$1 name=$2 body=$3 doc=${4:-} want_msg=${5:-} got=0 message
        [ -n "$doc" ] || doc=doc.md
        mkdir -p "$(dirname "$sandbox/$doc")"
        printf '%s\n' "$body" > "$sandbox/$doc"
        message=$(check_documents "$sandbox" "$blob_prefix" "$doc" target.md 2>&1 > /dev/null) || got=$?
        rm -f "$sandbox/$doc"

        if [ "$got" != "$want" ]; then
            failed=$((failed + 1))
            echo "  FAIL  $name: rc=$got, wanted $want" >&2
            return
        fi
        if [ -n "$want_msg" ]; then
            case "$message" in
                $want_msg) ;;
                *)
                    failed=$((failed + 1))
                    echo "  FAIL  $name: message was '$message'" >&2
                    return
                    ;;
            esac
        fi
        passed=$((passed + 1))
    }

    t 0 'file and anchor resolve'     '[a](target.md#health-reporting)'
    t 0 'file with no anchor'         '[a](target.md)'
    t 0 'two links on one line'       '[a](target.md) and [b](target.md#repeated)'

    # Each failure is asserted by message as well as by status, because the two refusals reach
    # the same exit code by different paths: drop the existence check and a missing file still
    # fails, as a missing anchor, with an empty slug list nobody wrote.
    t 1 'no such file'   '[a](missing.md#health-reporting)' '' '*NO FILE*missing.md#health-reporting*'
    t 1 'no such anchor' '[a](target.md#health-report)'     '' '*NO ANCHOR*target.md#health-report*'

    # An anchor is matched whole. A prefix match would resolve `#health` against `## Health
    # reporting` and call a link that lands nowhere good.
    t 1 'an anchor is not a prefix'   '[a](target.md#health)'

    # Backticks and asterisks are formatting and vanish with the rest of the punctuation; the
    # em dash is above ASCII and is kept, which is what GitHub and rustdoc both do -- so the
    # anchor carries it, and the hyphens around it come from the spaces.
    t 0 'formatting and punctuation'  '[a](target.md#status-—-how-bad-is-the-worst-thing)'

    # The crate's prose is `α₀`, `γ`, `ν`; a heading carrying one keeps it. Dropping bytes
    # above ASCII instead would fail this link, which is correctly written.
    t 0 'a non-ASCII heading'         '[a](target.md#the-barometric-reference-α₀)'

    # Trailing whitespace on a heading is invisible in review and GitHub ignores it.
    t 0 'trailing space on a heading' '[a](target.md#trailing-spaces)'

    # A heading opening with punctuation slugs to an anchor opening with a hyphen, which
    # `grep` reads as options unless the pattern comes after `--`.
    t 0 'an anchor opening with a hyphen' '[a](target.md#-the-operator)'

    # GitHub numbers a repeated heading from the second one.
    t 0 'repeated heading, first'     '[a](target.md#repeated)'
    t 0 'repeated heading, second'    '[a](target.md#repeated-1)'
    t 1 'repeated heading, third'     '[a](target.md#repeated-2)'

    # The mutation that matters: without fence tracking, `# Not a heading` in the shell block
    # above is a heading, this link resolves, and the comments in `AGENTS.md`'s command
    # blocks become anchors nobody wrote.
    t 1 'a comment in a fence is not a heading' '[a](target.md#not-a-heading)'

    # ... and the same rule on the reading side: a link inside a fence is a sample of
    # Markdown, not a link. `data/README.md` shows link syntax in a fence today.
    t 0 'a link in a fence is not a link' '```
[a](missing.md#nowhere)
```'

    # An in-page anchor is resolved against the linking file. The pass matters as much as the
    # failure: emit the path for such a link as an empty field and `read` loses it to the run
    # of tabs, so the anchor lands in the path and a link that resolves reports no file.
    t 0 'an in-page anchor resolves' '## Own heading
[a](#own-heading)'
    t 1 'an in-page anchor that does not' '## Own heading
[a](#other-heading)'

    # A relative link resolves from the linking file's directory. Resolving from the root
    # instead would report `data/README.md`'s `../GOALS.md` as missing, and -- worse -- would
    # resolve a `data/`-relative path that does not exist.
    t 0 'relative to the linking file' '[a](../target.md#repeated)' sub/doc.md

    # The absolute form, one link per fixture so that a verdict names which half failed, and
    # past whatever ref the link pins. A tag is checked too: the ref is skipped rather than
    # matched, so a link pinned to a release resolves against the working tree.
    t 0 'an absolute link'         "[a](${blob_prefix}main/target.md#repeated)" sub/doc.md
    t 0 'an absolute link, tagged' "[a](${blob_prefix}v1.2.3/target.md#repeated)" sub/doc.md
    t 1 'an absolute link, bad anchor' "[a](${blob_prefix}main/target.md#gone)" sub/doc.md

    # A source permalink is a citation, not an anchor: `#L696-L698` names lines in a file
    # this tool does not read. Checking it as a heading would fail every citation the
    # repository's own conventions ask for.
    t 0 'an absolute link to source' "[a](${blob_prefix}main/src/eskf.rs#L696-L698)"

    # A document rustdoc includes must link absolutely. Relative works on GitHub and 404s on
    # docs.rs, where the sibling file is not served -- invisible to everything else here.
    t 1 'an included document, relative link' '[a](target.md#repeated)' README.md '*RELATIVE*README.md:1*'
    t 0 'an included document, absolute link' "[a](${blob_prefix}main/target.md#repeated)" README.md

    # A doc comment is rustdoc too, and cites the documents that own its evidence.
    t 1 'a doc comment, relative link' '/// [a](target.md#repeated)' src/lib.rs '*RELATIVE*src/lib.rs:1*'
    t 1 'a doc comment, bad anchor' "/// [a](${blob_prefix}main/target.md#gone)" src/lib.rs '*NO ANCHOR*'
    t 0 'a doc comment, absolute link' "/// [a](${blob_prefix}main/target.md#repeated)" src/lib.rs

    # A document that cannot be read contributes no links, and must not therefore contribute
    # no complaint: "0 links, all resolved" is the shape of every check that checks nothing.
    local got=0
    check_documents "$sandbox" "$blob_prefix" absent.md > /dev/null 2>&1 || got=$?
    if [ "$got" = 1 ]; then
        passed=$((passed + 1))
    else
        failed=$((failed + 1))
        echo "  FAIL  an unreadable document: rc=$got, wanted 1" >&2
    fi

    echo "check-anchors.sh: $passed passed, $failed failed"
    [ "$failed" = 0 ]
}

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
blob_prefix=$(blob_prefix_from_cargo "$repo_root/Cargo.toml")

# Refused rather than degraded: with no prefix to recognise, every absolute link into this
# repository reads as an external URL and is skipped, which is the whole README checking
# nothing and saying it resolved 50 links.
if [ -z "$blob_prefix" ]; then
    echo "check-anchors.sh: no \`repository\` in Cargo.toml, so an absolute link into this" >&2
    echo "                  repository cannot be told from an external one" >&2
    exit 2
fi

case "${1:-}" in
    --self-test)
        self_test
        ;;
    -h | --help)
        sed -n '2,/^set -e/p' "$0" | sed -e 's/^#//' -e 's/^ //' -e '$d'
        ;;
    '')
        # `git ls-files` rather than a list here, so a document added to the repository is
        # checked without this script being edited -- and an empty answer is a refusal for
        # the same reason the missing prefix above is.
        # Less validation/src/: a template's links are written for the path its page is
        # published to (tools/validation.py), so the rendered page is the one that resolves.
        # And every Rust source, whose doc comments cite the documents that own their
        # evidence, and are rustdoc itself, so held to the absolute form.
        documents=$(cd "$repo_root" && git ls-files '*.md' ':!validation/src/*' 'src/*.rs')
        if [ -z "$documents" ]; then
            echo "check-anchors.sh: no tracked Markdown files to check" >&2
            exit 2
        fi
        # The word split below is the point; pathname expansion on the same line is not.
        set -f
        # shellcheck disable=SC2086
        check_documents "$repo_root" "$blob_prefix" $documents
        ;;
    *)
        echo "usage: check-anchors.sh [--self-test]" >&2
        exit 2
        ;;
esac
