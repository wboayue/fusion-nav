#!/usr/bin/env python3
"""Read the compiler's and the linker's own reports of what the filter costs, as `key=value` pairs.

    tools/footprint.py TYPES FRAMES FLASH    print one line of pairs
    tools/footprint.py --self-test           run the fixtures

`tools/footprint.sh` builds and writes the three inputs, and compares the line against
`data/footprint.txt`; this reads text and computes nothing about the filter:

    TYPES   rustc's `-Zprint-type-sizes` output for this crate alone
    FRAMES  `llvm-readobj --stack-sizes --demangle` of the rlib built with `-Zemit-stack-sizes`
    FLASH   per opt-level, a `level <l>` line, `llvm-size -A` and `llvm-nm --demangle
            --print-size --size-sort` of `panic-check`'s ELF

Keys are paths in this crate with `::` as `.`:

    size.<type>=          bytes; a generic type (`units::Position<frames::Ned>`) is left out
    frame.<function>=     bytes of the function's own frame. A method is keyed by its type,
                          `eskf.Eskf.predict`, so moving an `impl` between modules keeps it. A
                          generic parameter list is dropped and a leading constant kept,
                          `update::<3>` as `update.update.3`, so a function generic over a
                          closure is one key holding its **largest** instance, and a chain
                          summed from keys is a bound on the deepest path, not a path. Trait
                          impls, closures and methods on generic types are left out.
    text.<l>=, rodata.<l>=           the ELF's sections
    text_<crate>.<l>=                symbols `libm`, `compiler_builtins` or `nalgebra` kept
                                     out of line. An unmangled name is `compiler_builtins`'s:
                                     the soft-float, memory and C math routines it exports.

Names are read as nightly demangles Rust's v0 mangling, which keeps generic arguments; the legacy
scheme drops them and would fold every instance into one name.

Standard library only, run by `python3`, for the reason `tools/anees.py` gives.
"""

import re
import sys

OWNED = ("libm", "compiler_builtins", "nalgebra")
OPEN, CLOSE = "<([", ">)]"


def closes(name, i):
    """Whether `name[i]` closes a bracket. The `>` of `->` closes nothing."""
    return name[i] in CLOSE and not (name[i] == ">" and i > 0 and name[i - 1] == "-")


def depths(name):
    """Bracket depth before each character."""
    depth, out = 0, []
    for i, ch in enumerate(name):
        out.append(depth)
        if ch in OPEN:
            depth += 1
        elif closes(name, i):
            depth -= 1
    return out


def split_top(names):
    """`a::<1, b>, c` -> [`a::<1, b>`, `c`]: a folded entry's names, split at depth 0 only."""
    level, parts, start = depths(names), [], 0
    for i in range(len(names)):
        if names.startswith(", ", i) and level[i] == 0:
            parts.append(names[start:i])
            start = i + 2
    parts.append(names[start:])
    return parts


def generics(name):
    """Drop each `::<...>`, keeping a leading constant: `observe::<3, {closure}>` -> `observe::3`."""
    level, out, i = depths(name), "", 0
    while i < len(name):
        if name.startswith("::<", i):
            j = i + 3
            while j < len(name) and not (closes(name, j) and level[j] == level[i] + 1):
                j += 1
            if j == len(name):
                raise ValueError(f"unbalanced generic list in {name!r}")
            constant = re.match(r"\d+", name[i + 3 : j])
            out += "::" + constant.group(0) if constant else ""
            i = j + 1
        else:
            out += name[i]
            i += 1
    return out


def path(name):
    """`<fusion_nav::eskf::Eskf>::predict` -> `eskf.Eskf.predict`; None for anything not keyed."""
    name = re.sub(r" \(\.llvm\.\d+\)$", "", name)
    if not name.startswith(("fusion_nav::", "<fusion_nav::")):
        return None
    name = generics(name)
    m = re.fullmatch(r"<fusion_nav::([A-Za-z0-9_:]+)>::([A-Za-z0-9_:]+)", name)
    if m:
        name = m.group(1) + "::" + m.group(2)
    elif name.startswith("fusion_nav::"):
        name = name[len("fusion_nav::") :]
    else:
        return None
    if not re.fullmatch(r"[A-Za-z0-9_:]+", name):
        return None
    return name.replace("::", ".")


def crate(symbol):
    """The crate a linked symbol belongs to; None for the driver's entry point."""
    symbol = symbol.lstrip("<&")
    if "::" not in symbol:
        return None if symbol == "_start" else "compiler_builtins"
    return symbol.split("::", 1)[0]


def parse(types, frames, flash):
    """The three reports as a dict of pairs. Refuses an empty report: a build cargo considered
    fresh compiles nothing and prints nothing, and its keys would read as renamed."""
    pairs = {}

    def largest(key, value):
        pairs[key] = max(value, pairs.get(key, 0))

    for line in types.splitlines():
        m = re.match(r"print-type-size type: `([A-Za-z0-9_:]+)`: (\d+) bytes", line)
        if m:
            largest("size." + m.group(1).replace("::", "."), int(m.group(2)))
    if not any(k.startswith("size.") for k in pairs):
        raise ValueError("no type sizes: was the crate compiled with -Zprint-type-sizes?")

    names = None
    for line in frames.splitlines():
        m = re.search(r"Functions: \[(.*)\]", line)
        if m:
            names = m.group(1)
            continue
        m = re.search(r"Size: 0x([0-9A-Fa-f]+)", line)
        if m and names is not None:
            # A folded entry names every function sharing the code; each has the frame.
            for function in split_top(names):
                key = path(function)
                if key is not None:
                    largest("frame." + key, int(m.group(1), 16))
            names = None
    if not any(k.startswith("frame.") for k in pairs):
        raise ValueError("no stack sizes: was the rlib built with -Zemit-stack-sizes, without LTO?")

    level = None
    for line in flash.splitlines():
        if line.startswith("level "):
            level = line.split()[1]
            for owner in OWNED:
                pairs[f"text_{owner}.{level}"] = 0
            continue
        m = re.match(r"(\.text|\.rodata)\s+(\d+)\s", line)
        if m:
            if level is None:
                raise ValueError("a section before any `level` line")
            pairs[f"{m.group(1)[1:]}.{level}"] = int(m.group(2))
            continue
        m = re.match(r"[0-9a-f]+ ([0-9a-f]+) [tT] (.*)$", line)
        if m:
            if level is None:
                raise ValueError("a symbol before any `level` line")
            owner = crate(m.group(2))
            if owner in OWNED:
                pairs[f"text_{owner}.{level}"] += int(m.group(1), 16)
    if level is None:
        raise ValueError("no flash: no `level` line")
    return pairs


def line(pairs):
    return " ".join(f"{k}={v}" for k, v in sorted(pairs.items()))


def self_test():
    failures = []

    def check(name, got, want):
        if got != want:
            failures.append(f"{name}: got {got!r}, want {want!r}")

    check("method", path("<fusion_nav::eskf::Eskf>::predict"), "eskf.Eskf.predict")
    check("const generic", path("fusion_nav::update::update::<3>"), "update.update.3")
    check("llvm suffix", path("fusion_nav::propagate::coast (.llvm.1234)"), "propagate.coast")
    check(
        "generic over a closure",
        path(
            "<fusion_nav::eskf::Eskf>::observe::<1, <fusion_nav::eskf::Eskf>::fuse_heading"
            "<<fusion_nav::eskf::Eskf>::fuse_course::{closure#2}>::{closure#0}>"
        ),
        "eskf.Eskf.observe.1",
    )
    # The `>` of `->` closes nothing; counted, the list ends early and the name is dropped.
    check("fn pointer argument", path("fusion_nav::math::apply::<fn(f32) -> f32>"), "math.apply")
    check("trait impl", path("<fusion_nav::init::SampleRefusal as core::fmt::Display>::fmt"), None)
    check("closure", path("<fusion_nav::eskf::Eskf>::fuse_gnss_velocity::{closure#0}"), None)
    check("generic type", path("<fusion_nav::units::Position<fusion_nav::frames::Ned>>::zero"), None)
    check("other crate", path("nalgebra::base::matrix::Matrix::mul"), None)
    check("unmangled", crate("__aeabi_fadd"), "compiler_builtins")
    check("driver", crate("_start"), None)
    check("reference impl", crate("<&nalgebra::geometry::Quaternion<f32> as Mul>::mul"), "nalgebra")

    types = "\n".join([
        "print-type-size type: `eskf::Eskf`: 3776 bytes, alignment: 8 bytes",
        "print-type-size     field `.config`: 244 bytes",
        "print-type-size type: `units::Position<frames::Ned>`: 12 bytes, alignment: 4 bytes",
        "print-type-size type: `{closure@src/eskf.rs:1:1: 1:2}`: 4 bytes, alignment: 4 bytes",
    ])
    # A comma inside a generic list is not a fold: split there, `fusion_nav::frames::Ned` would
    # come out as a function. The two `observe::<1, ...>` instances differ; the key holds the
    # larger whichever comes first.
    frames = "\n".join([
        "  Entry {",
        "    Functions: [<fusion_nav::eskf::Eskf>::observe::<1, fusion_nav::frames::Ned>]",
        "    Size: 0x4F8",
        "  }",
        "  Entry {",
        "    Functions: [<fusion_nav::eskf::Eskf>::observe::<1, fusion_nav::frames::Enu>]",
        "    Size: 0x500",
        "  }",
        "  Entry {",
        "    Functions: [fusion_nav::math::skew, fusion_nav::math::wrap_pi]",
        "    Size: 0x10",
        "  }",
    ])
    flash = "\n".join([
        "level s",
        "section               size     addr",
        ".rodata               4455    65768",
        ".text               107754   135760",
        "0002d2a8 00000298 t fusion_nav::update::inject",
        "0003644c 000002a4 t libm::math::sqrt::sqrt",
        "00039024 00000338 t compiler_builtins::float::mul::__muldf3",
        "00039400 00000010 T __aeabi_memcpy",
        "00039500 00000020 t <&nalgebra::base::Matrix<f32> as Mul>::mul",
        "00021360 00004fac T _start",
        "level 3",
        ".text               167570   135760",
        ".rodata               4383    65768",
    ])
    got = parse(types, frames, flash)
    check("pairs", line(got), " ".join([
        "frame.eskf.Eskf.observe.1=1280",
        "frame.math.skew=16",
        "frame.math.wrap_pi=16",
        "rodata.3=4383",
        "rodata.s=4455",
        "size.eskf.Eskf=3776",
        "text.3=167570",
        "text.s=107754",
        "text_compiler_builtins.3=0",
        "text_compiler_builtins.s=840",
        "text_libm.3=0",
        "text_libm.s=676",
        "text_nalgebra.3=0",
        "text_nalgebra.s=32",
    ]))

    def refused(name, *inputs):
        try:
            parse(*inputs)
            failures.append(f"{name}: accepted")
        except ValueError:
            pass

    refused("no types", "", frames, flash)
    refused("no frames", types, "", flash)
    refused("no flash", types, frames, "")
    refused("a section before a level", types, frames, ".text  1  0")
    refused("an unbalanced list", types, "Functions: [fusion_nav::f::<1]\nSize: 0x10", flash)

    for failure in failures:
        print(f"footprint self-test: {failure}", file=sys.stderr)
    if failures:
        sys.exit(1)
    print("footprint self-test: ok")


def main(argv):
    if argv == ["--self-test"]:
        self_test()
        return
    if len(argv) != 3:
        sys.exit(__doc__.split("\n\n")[1])
    try:
        texts = []
        for name in argv:
            with open(name) as f:
                texts.append(f.read())
        print(line(parse(*texts)))
    except (OSError, ValueError) as error:
        sys.exit(f"footprint: {error}")


if __name__ == "__main__":
    main(sys.argv[1:])
