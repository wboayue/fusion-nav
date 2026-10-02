#!/usr/bin/env python3
"""Read the compiler's and the linker's own reports of what the filter costs, as `key=value` pairs.

    tools/footprint.py TYPES FRAMES FLASH CODE              print one line of pairs
    tools/footprint.py --path <key> TYPES FRAMES FLASH CODE  print one chain's path, or why
                                                             it was refused
    tools/footprint.py --self-test                          run the fixtures

`tools/footprint.sh` builds and writes the four inputs, and compares the line against
`data/footprint.txt`; this reads text and computes nothing about the filter:

    TYPES   rustc's `-Zprint-type-sizes` output for this crate alone
    FRAMES  `llvm-readobj --stack-sizes --demangle` of this crate's rlib and of every rlib its
            code can call into, each built with `-Zemit-stack-sizes`
    FLASH   per opt-level, a `level <l>` line, `llvm-size -A` and `llvm-nm --demangle
            --print-size --size-sort` of `panic-check`'s ELF
    CODE    `llvm-objdump -d -r -t --demangle` of the same rlibs as FRAMES, in the same order

`tools/footprint.sh` leaves the four in `target/footprint/<target>/`, so `--path` runs on them
by hand.

Keys are paths in this crate with `::` as `.`:

    size.<type>=          bytes; a generic type (`units::Position<frames::Ned>`) is left out
    frame.<function>=     bytes of the function's own frame. A method is keyed by its type,
                          `eskf.Eskf.predict`, so moving an `impl` between modules keeps it. A
                          generic parameter list is dropped and a leading constant kept,
                          `update::<3>` as `update.update.3`, so a function generic over a
                          closure is one key holding its **largest** instance, and a chain
                          summed from keys is a bound on the deepest path, not a path. Trait
                          impls, closures and methods on generic types are left out.
    chain.<function>=     bytes of the deepest stack the function reaches: its frame and its
                          deepest callee's chain, walked through the call relocations into
                          `nalgebra`, `libm`, `core` and `compiler_builtins`, whose frames are
                          as much a part of the path as this crate's. Keyed as `frame.` is, a
                          generic function holding its largest instance. `refused` where the
                          walk cannot bound it (`walk` says when); `--path` says why.
    stack_peak=           the largest `chain.`: the stack an integrator plans for.
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


CALLS = ("R_ARM_THM_CALL", "R_ARM_THM_JUMP24", "R_ARM_CALL", "R_ARM_JUMP24")


class Refused(Exception):
    """A path the walk cannot bound. Printed as the chain's value, and its reason on stderr."""


class Function:
    """One function's code: where it is, what it calls, and what its prologue takes."""

    def __init__(self, member, section, label):
        self.member, self.section, self.label = member, section, label
        self.names = [label]
        self.calls = []          # every call relocation's symbol
        self.references = set()  # every other relocation's symbol: addresses it takes, data
        self.indirect = 0        # `blx rN`, `bx rN`: calls through a register
        self.jumps = 0           # `mov pc, rN`: a jump table, if its entries are its own section
        self.pushed = 0          # bytes its pushes and `sub sp` take, for a function with no frame
        self.dynamic = False     # it moves `sp` by a register: no prologue read bounds it

    def indirect_calls(self):
        own = self.section in self.references
        return self.indirect + (0 if own else self.jumps)


def registers(operand):
    """The registers `{r4, r5, lr}` or `{d8-d11}` names, as (count, bytes each)."""
    count, width = 0, 4
    for part in operand.strip("{}").split(","):
        part = part.strip()
        m = re.fullmatch(r"([rsd])(\d+)-[rsd](\d+)", part)
        count += int(m.group(3)) - int(m.group(2)) + 1 if m else 1
        if part.startswith("d"):
            width = 8
    return count, width


def instruction(function, mnemonic, operands):
    """Note what one instruction says about calls and the stack."""
    register = re.fullmatch(r"r(\d+)", operands)
    if mnemonic in ("blx", "bx") and register:
        function.indirect += 1
    elif mnemonic == "mov" and operands.startswith("pc,"):
        function.jumps += 1
    elif mnemonic.startswith("ldr") and operands.startswith("pc,"):
        # `ldr pc, [sp], #4` is a return; any other load into pc is a jump the walk cannot see.
        if not re.match(r"pc, \[sp\], #", operands):
            function.indirect += 1
    elif mnemonic.split(".")[0] in ("push", "vpush"):
        count, width = registers(operands)
        function.pushed += count * width
    elif re.fullmatch(r"subs?(\.w)?", mnemonic) and operands.startswith("sp,"):
        m = re.search(r"#(0x[0-9a-f]+|\d+)$", operands)
        if m:
            function.pushed += int(m.group(1), 0)
        else:
            function.dynamic = True
    elif re.fullmatch(r"(adds?|mov)(\.w)?", mnemonic) and re.match(r"sp, (sp, )?r\d", operands):
        function.dynamic = True
    elif mnemonic.startswith(("str", "stm")) and re.search(r"\[sp, #-|sp!", operands):
        m = re.search(r"#-(0x[0-9a-f]+|\d+)\]!", operands)
        if m:
            function.pushed += int(m.group(1), 0)
        else:
            function.dynamic = True


def program(code, frames):
    """Every function in `llvm-objdump -d -r -t --demangle` of the rlibs, with its frame from
    `llvm-readobj --stack-sizes` of the same files.

    Returns the functions and a resolver from (calling member, symbol) to the functions a call
    to it may reach: a local symbol only in its own object file, a global one in every file
    defining it, since a weak definition can stand in more than one place."""
    aliases = {}   # (member, section, value) -> [(binding, name)]
    defined = {}   # name -> "F" or "O", over every file: a reference names code or data
    functions = {}
    member = section = current = None
    for line in code.splitlines():
        m = re.match(r"(\S.*\)):\s+file format ", line)
        if m:
            member, current = m.group(1), None
            continue
        m = re.match(r"Disassembly of section (\S+):$", line)
        if m:
            section = m.group(1)
            continue
        m = re.match(r"([0-9a-f]{8}) <(.*)>:$", line)
        if m:
            key = (member, section, int(m.group(1), 16))
            current = functions[key] = Function(member, section, m.group(2))
            names = aliases.get(key)
            if names:
                current.names = [name for _, name in names]
            continue
        m = re.match(r"\s+[0-9a-f]+:\s+(R_ARM_\w+)\s+(.+?)(?:[+-]0x[0-9a-f]+)?$", line)
        if m:
            if current is not None:
                (current.calls.append if m.group(1) in CALLS else current.references.add)(m.group(2))
            continue
        m = re.match(r"\s+[0-9a-f]+:\s+(?:[0-9a-f]{2,8} )+\s*\t(\S+)(?:\t(.*))?$", line)
        if m:
            if current is not None:
                instruction(current, m.group(1), (m.group(2) or "").split(" @ ")[0].strip())
            continue
        m = re.match(r"([0-9a-f]{8}) (.{7}) (\S+)\t[0-9a-f]+ (?:\.hidden )?(.+)$", line)
        if m and m.group(3) != "*UND*":
            flags, name = m.group(2), m.group(4)
            kind = "F" if "F" in flags else "O"
            defined[name] = "F" if kind == "F" or defined.get(name) == "F" else kind
            if kind == "F":
                aliases.setdefault((member, m.group(3), int(m.group(1), 16)), []).append(
                    (flags[0], name)
                )

    local, exported = {}, {}
    for (where, section, value), names in aliases.items():
        function = functions.get((where, section, value))
        if function is None:
            continue
        for binding, name in names:
            if binding == "l":
                local[(where, name)] = function
            else:
                exported.setdefault(name, []).append(function)

    sizes, member, names = {}, None, None
    for line in frames.splitlines():
        m = re.match(r"File: (.*)$", line)
        if m:
            member = m.group(1)
            continue
        m = re.search(r"Functions: \[(.*)\]", line)
        if m:
            names = m.group(1)
            continue
        m = re.search(r"Size: 0x([0-9A-Fa-f]+)", line)
        if m and names is not None:
            for name in split_top(names):
                sizes[(member, name)] = int(m.group(1), 16)
            names = None
    for function in functions.values():
        found = [sizes[(function.member, n)] for n in function.names if (function.member, n) in sizes]
        function.frame = max(found) if found else None

    sections = {}
    for function in functions.values():
        sections.setdefault((function.member, function.section), []).append(function)

    def resolve(caller, symbol):
        """The functions a relocation against `symbol` from `caller`'s file may reach; None if
        no file defines it as code."""
        if (caller.member, symbol) in local:
            return [local[(caller.member, symbol)]]
        if symbol in exported:
            return exported[symbol]
        # A section symbol: an address inside that section, which the walk takes as any of it.
        return sections.get((caller.member, symbol))

    return functions, resolve, defined


def frame_of(function):
    """The compiler's figure, or, for the hand-written assembly it has none for, every byte the
    function's pushes and `sub sp` take: a bound on straight-line code, refused where `sp`
    moves by a register."""
    if function.frame is not None:
        return function.frame
    if function.dynamic:
        raise Refused(f"{function.label} has no measured frame and moves sp by a register")
    return function.pushed


def walk(root, resolve, defined):
    """The deepest stack `root` reaches, as (bytes, [(function, frame), ...]) along the path.

    An indirect call is bounded by the deepest function whose address any function `root`
    reaches takes: a function pointer reaches a call only from where it was taken, which holds
    while no structure in the crate stores one. Refused for a call to code no input holds, a
    cycle, or an indirect call with nothing taken to bound it."""
    reached, taken, unknown, pending = set(), set(), set(), [root]
    while pending:
        function = pending.pop()
        if function in reached:
            continue
        reached.add(function)
        for symbol in function.calls:
            callees = resolve(function, symbol)
            if callees is None:
                raise Refused(f"{function.label} calls {symbol}, which no input holds")
            pending.extend(callees)
        for symbol in function.references:
            targets = resolve(function, symbol)
            if targets is not None and symbol != function.section:
                taken.update(targets)
                pending.extend(targets)
            elif targets is None and symbol not in defined and not symbol.startswith("."):
                unknown.add(symbol)
    indirect = [f for f in reached if f.indirect_calls()]
    if indirect and unknown:
        raise Refused(f"{indirect[0].label} calls through a register, and {sorted(unknown)[0]} is undefined")
    if indirect and not taken:
        raise Refused(f"{indirect[0].label} calls through a register, and nothing reached takes an address")

    memo, active = {}, set()

    def depth(function):
        if function in memo:
            return memo[function]
        if function in active:
            raise Refused(f"a cycle through {function.label}")
        active.add(function)
        callees = [c for symbol in function.calls for c in resolve(function, symbol)]
        if function.indirect_calls():
            callees += sorted(taken, key=lambda f: f.label)
        deepest = max((depth(c) for c in callees), key=lambda d: d[0], default=(0, []))
        own = frame_of(function)
        active.discard(function)
        memo[function] = (own + deepest[0], [(function.label, own)] + deepest[1])
        return memo[function]

    return depth(root)


def chains(code, frames):
    """`chain.<function>=` for every function `frame.` keys: its frame and the deepest path of
    calls beneath it, through `nalgebra`, `libm`, `core` and `compiler_builtins` as well as this
    crate. A key holds its largest instance, as `frame.` does. Also the paths, for `--path`."""
    functions, resolve, defined = program(code, frames)
    pairs, paths = {}, {}
    for function in functions.values():
        keys = {k for k in map(path, function.names) if k is not None}
        if not keys:
            continue
        try:
            bytes_, steps = walk(function, resolve, defined)
        except Refused as reason:
            bytes_, steps = "refused", [(str(reason), 0)]
        for key in keys:
            key = "chain." + key
            held = pairs.get(key)
            if held == "refused" or (isinstance(held, int) and bytes_ != "refused" and held >= bytes_):
                continue
            pairs[key], paths[key] = bytes_, steps
    bounded = [v for v in pairs.values() if v != "refused"]
    if not bounded:
        raise ValueError("no chains: was the disassembly taken of this crate's rlib?")
    pairs["stack_peak"] = max(bounded)
    return pairs, paths


def parse(types, frames, flash, code=None):
    """The reports as a dict of pairs. Refuses an empty report: a build cargo considered
    fresh compiles nothing and prints nothing, and its keys would read as renamed. Without
    `code`, no chain: the fixtures of the other three readers need none."""
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
    if code is not None:
        pairs.update(chains(code, frames)[0])
    return pairs


def line(pairs):
    return " ".join(f"{k}={v}" for k, v in sorted(pairs.items()))


def objects(*files):
    """`llvm-objdump -d -r -t` and `llvm-readobj --stack-sizes` text for synthetic object files.

    Each file is (member, [(binding, names, frame, body)]): a function's first name labels its
    code, the rest alias it, a frame of None writes no stack-size entry, and each body line is
    an instruction (`push\t{r7, lr}`) or a relocation (`R_ARM_THM_CALL\tsymbol`)."""
    code, frames = [], []
    for member, functions in files:
        member = f"lib.rlib({member})"
        code += [f"{member}:\tfile format elf32-littlearm", "", "SYMBOL TABLE:"]
        frames += [f"File: {member}", "StackSizes ["]
        for binding, names, frame, _ in functions:
            for name in names:
                code.append(f"00000000 {binding}     F .text.{names[0]}\t00000010 {name}")
            if frame is not None:
                frames += ["  Entry {", f"    Functions: [{', '.join(names)}]",
                           f"    Size: 0x{frame:X}", "  }"]
        for _, names, _, body in functions:
            code += ["", f"Disassembly of section .text.{names[0]}:", "", f"00000000 <{names[0]}>:"]
            for i, step in enumerate(body):
                if step.startswith("R_ARM"):
                    code.append(f"\t\t\t{i:08x}:  {step}")
                else:
                    code.append(f"{i * 2:8x}: b5f0         \t{step}")
        frames.append("]")
    return "\n".join(code), "\n".join(frames)


def walk_fixtures(check):
    """The walk against call graphs whose depth is known by construction. Each value is written
    out as the sum it is, so a reader can see which frames the path takes."""
    f = "fusion_nav::f::"
    call = "R_ARM_THM_CALL\t" + f
    a = [
        # `big` has the largest frame under `root`, but `mid` and its leaf are deeper: the case
        # summing named frames gets wrong.
        ("g", [f + "root"], 100, [call + "big", call + "mid", call + "dup"]),
        ("g", [f + "big"], 300, []),
        ("g", [f + "mid"], 50, ["R_ARM_THM_JUMP24\t" + f + "leaf"]),
        ("g", [f + "leaf"], 400, []),
        # Local in both files, deeper in `b`: a call reaches its own file's copy.
        ("l", [f + "dup"], 8, []),
        # `apply` calls through a register; the only address `fuse` takes is `select`'s, so the
        # bound is `select` and not `deep`, whose address an unreached `other` takes.
        ("g", [f + "fuse"], 20, ["R_ARM_ABS32\t" + f + "select", call + "apply"]),
        ("g", [f + "apply"], 30, ["blx\tr2"]),
        ("g", [f + "select"], 4, ["bx\tlr"]),
        ("g", [f + "other"], 1, ["R_ARM_ABS32\t" + f + "deep"]),
        ("g", [f + "deep"], 1000, []),
        # A jump table: `mov pc` through entries in its own section is no call.
        ("g", [f + "table"], 12, ["mov\tpc, r2", f"R_ARM_ABS32\t.text.{f}table"]),
        # Hand-written assembly with no stack-size entry, read from its pushes, called through
        # an alias the label does not show.
        ("g", [f + "asm"], 0, [call + "naked_alias"]),
        ("g", [f + "naked", f + "naked_alias"], None,
         ["push\t{r4, lr}", "sub\tsp, #0x10", "add\tsp, #0x10", "pop\t{r4, pc}"]),
        ("g", [f + "cycle"], 4, [call + "again"]),
        ("g", [f + "again"], 4, [call + "cycle"]),
        ("g", [f + "missing"], 4, [call + "nowhere"]),
        ("g", [f + "blind"], 4, ["blx\tr3"]),
        ("g", [f + "pointer"], 4, [call + "moved"]),
        ("g", [f + "moved"], None, ["push\t{r4, lr}", "add\tsp, r6"]),
    ]
    b = [("l", [f + "dup"], 800, [])]
    pairs, paths = chains(*objects(("a.o", a), ("b.o", b)))
    want = {
        "root": 100 + 50 + 400,
        "fuse": 20 + 30 + 4,
        "table": 12,
        "asm": 0 + (2 + 4) * 4,
        # Keyed by the alias as well as the label: one function, two names in the index.
        "naked_alias": (2 + 4) * 4,
        "cycle": "refused",
        "missing": "refused",
        "blind": "refused",
        "pointer": "refused",
    }
    for name, value in want.items():
        check(f"chain {name}", pairs.get("chain.f." + name), value)
    check("path", [s for _, s in paths["chain.f.root"]], [100, 50, 400])
    # The deepest chain, `deep` alone; a refused one is not a figure to take the largest of.
    check("peak", pairs["stack_peak"], 1000)
    check("registers", registers("{d8-d11}"), (4, 8))


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
    # come out as a function. The two `observe::<1, ...>` instances differ, the larger first, so
    # a key that kept the last instance would read the smaller.
    frames = "\n".join([
        "  Entry {",
        "    Functions: [<fusion_nav::eskf::Eskf>::observe::<1, fusion_nav::frames::Enu>]",
        "    Size: 0x500",
        "  }",
        "  Entry {",
        "    Functions: [<fusion_nav::eskf::Eskf>::observe::<1, fusion_nav::frames::Ned>]",
        "    Size: 0x4F8",
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

    walk_fixtures(check)

    for failure in failures:
        print(f"footprint self-test: {failure}", file=sys.stderr)
    if failures:
        sys.exit(1)
    print("footprint self-test: ok")


def main(argv):
    if argv == ["--self-test"]:
        self_test()
        return
    show = None
    if argv[:1] == ["--path"] and len(argv) == 6:
        show, argv = argv[1], argv[2:]
    if len(argv) != 4:
        sys.exit(__doc__.split("\n\n")[1])
    try:
        texts = []
        for name in argv:
            with open(name) as f:
                texts.append(f.read())
        if show is None:
            print(line(parse(*texts)))
            return
        pairs, paths = chains(texts[3], texts[1])
        if show not in paths:
            raise ValueError(f"no {show}")
        print(f"{show}={pairs[show]}")
        for function, frame in paths[show]:
            print(f"{frame:8} {function}")
    except (OSError, ValueError) as error:
        sys.exit(f"footprint: {error}")


if __name__ == "__main__":
    main(sys.argv[1:])
