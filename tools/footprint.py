#!/usr/bin/env python3
"""Read the compiler's and the linker's own reports of what the filter costs, as `key=value` pairs.

    tools/footprint.py TYPES FRAMES FLASH CODE SYSROOT    print one line of pairs
    tools/footprint.py --path <key> FRAMES CODE SYSROOT   print one chain's path, or why it was
                                                          refused
    tools/footprint.py --self-test                        run the fixtures

`tools/footprint.sh` builds and writes the five inputs, and compares the line against
`data/footprint.txt`; this reads text and computes nothing about the filter:

    TYPES   rustc's `-Zprint-type-sizes` output for this crate alone
    FRAMES  `llvm-readobj --stack-sizes --demangle` of this crate's rlib and of every rlib its
            code can call into, each built with `-Zemit-stack-sizes`
    FLASH   per opt-level, a `level <l>` line, `llvm-size -A` and `llvm-nm --demangle
            --print-size --size-sort` of `panic-check`'s ELF
    CODE    `llvm-objdump -d -r -t --demangle` of the same rlibs as FRAMES, then `llvm-objdump
            -r --demangle` of them for the relocations of their data
    SYSROOT `llvm-objdump -d -r -t --demangle` of the sysroot's prebuilt `core` and
            `compiler_builtins`, which FRAMES and CODE take from a `-Zbuild-std` rebuild

`tools/footprint.sh` leaves the five in `target/footprint/<target>/`, so `--path` runs on them
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
    stack_peak=           the largest chain of a function the crate exports, `refused` if any
                          is: the deepest stack a call into the filter takes, before the
                          caller's own frames and any exception stacked on top. Frames are the
                          library build's; an application's LTO can inline them differently.
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


def unsuffixed(name):
    """`f (.llvm.123)` -> `f`: the suffix LLVM gives a local symbol it promotes, which differs
    between two builds of the same code."""
    return re.sub(r" \(\.llvm\.\d+\)$", "", name)


def path(name):
    """`<fusion_nav::eskf::Eskf>::predict` -> `eskf.Eskf.predict`; None for anything not keyed."""
    name = unsuffixed(name)
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


# A relocation on a branch of any range is a call or a tail call, and counts as a call: that
# overstates a tail call by its caller's frame and never understates one.
CALL = re.compile(r"R_ARM_(THM_)?(CALL|JUMP\d*|PC24|PLT32)$")
# The condition an IT block or an ARM instruction can append to a mnemonic.
COND = r"(eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|al)?"
# A file of the `-Zbuild-std` rebuild, whose code must be the sysroot's (`same_as_sysroot`).
REBUILT = re.compile(r"/lib(core|compiler_builtins)-[0-9a-f]+\.rlib\(")

# `llvm-objdump`'s lines, one pattern per kind. A relocation's symbol drops its addend.
RELOCATION = r"(R_ARM_\w+)\s+(.+?)(?:[+-]0x[0-9a-f]+)?$"
FILE = re.compile(r"(\S.*\)):\s+file format ")
SECTION = re.compile(r"Disassembly of section (\S+):$")
RECORDS = re.compile(r"RELOCATION RECORDS FOR \[(\S+)\]:$")
RECORD = re.compile(r"[0-9a-f]{8} " + RELOCATION)
LABEL = re.compile(r"([0-9a-f]{8}) <(.*)>:$")
INLINE = re.compile(r"\s+[0-9a-f]+:\s+" + RELOCATION)
INSTRUCTION = re.compile(r"\s+[0-9a-f]+:\s+(?:[0-9a-f]{2,8} )+\s*\t(\S+)(?:\t(.*))?$")
SYMBOL = re.compile(r"([0-9a-f]{8}) (.{7}) (\S+)\t[0-9a-f]+ (\.hidden )?(.+)$")

# An instruction's mnemonic and operands, one pattern per question `instruction` asks.
REGISTER_CALL = re.compile(f"bl?x{COND}")
JUMP = re.compile(f"mov{COND}")
OFFSET_JUMP = re.compile(f"add{COND}")
LOAD = re.compile(f"ldr{COND}(\\.w)?")
STORE_MULTIPLE = re.compile(r"stmdb(\.w)?")
ADJUST = re.compile(r"(sub|add)s?w?(\.w)?")
MOVE = re.compile(r"mov(\.w)?")
IMMEDIATE = re.compile(r"#(-?(?:0x[0-9a-f]+|\d+))$")
PRE_INDEXED = re.compile(r"#-(0x[0-9a-f]+|\d+)\]!")


class Refused(Exception):
    """A path the walk cannot bound: the chain's value is `refused`, and `--path` gives why."""


class Function:
    """One function's code: where it is, what it calls, and what its prologue takes."""

    def __init__(self, member, section, label):
        self.member, self.section, self.label = member, section, label
        self.names = [label]
        self.exported = False    # visible outside the crate: a caller there passes no pointer
        self.frame = None
        self.calls = []          # every call relocation's symbol
        self.references = set()  # every other relocation's symbol: addresses it takes, data
        self.indirect = 0        # `blx rN`, `bx rN`, `ldr pc`: calls through a register
        self.jumps = 0           # `mov pc, rN`: a jump table, if its entries are its own section
        self.pushed = 0          # bytes its pushes and `sub sp` take, for a function with no frame
        self.dynamic = False     # it moves `sp` by a register: no prologue read bounds it

    def is_table(self, symbol):
        """Whether a reference to `symbol` is an entry of the function's own jump table: an
        address in its own section, which neither takes an address nor makes `mov pc` a call."""
        return symbol == self.section

    def indirect_calls(self):
        has_table = any(self.is_table(symbol) for symbol in self.references)
        return self.indirect + (0 if has_table else self.jumps)

    def signature(self):
        """What two builds of the same function share: callees, pushes, indirect calls."""
        return (sorted(map(unsuffixed, self.calls)), self.pushed, self.dynamic,
                self.indirect_calls())


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
    if REGISTER_CALL.fullmatch(mnemonic) and re.fullmatch(r"r\d+", operands):
        function.indirect += 1
    elif JUMP.fullmatch(mnemonic) and operands.startswith("pc,"):
        function.jumps += 1
    elif OFFSET_JUMP.fullmatch(mnemonic) and operands.startswith("pc,"):
        # Thumb-1's jump table: `pc` plus an offset read from a table inline in the function,
        # so a branch inside its own code.
        pass
    elif LOAD.fullmatch(mnemonic) and operands.startswith("pc,"):
        # `ldr pc, [sp], #4` is a return; any other load into pc is a jump the walk cannot see.
        if not operands.startswith("pc, [sp], #"):
            function.indirect += 1
    elif mnemonic.split(".")[0] in ("push", "vpush") or (
        STORE_MULTIPLE.fullmatch(mnemonic) and operands.startswith("sp!,")
    ):
        count, width = registers(operands.removeprefix("sp!, "))
        function.pushed += count * width
    elif ADJUST.fullmatch(mnemonic) and operands.startswith("sp,"):
        immediate = IMMEDIATE.search(operands)
        if re.match(r"sp, (sp, )?r\d", operands) or not immediate:
            function.dynamic = True
        else:
            moved = int(immediate.group(1), 0)
            function.pushed += moved if mnemonic.startswith("sub") else max(0, -moved)
    elif MOVE.fullmatch(mnemonic) and operands.startswith("sp,"):
        function.dynamic = True
    elif mnemonic.startswith(("str", "stm")) and ("[sp, #-" in operands or "sp!" in operands):
        m = PRE_INDEXED.search(operands)
        if m:
            function.pushed += int(m.group(1), 0)
        else:
            function.dynamic = True


def stack_sizes(frames):
    """`llvm-readobj --stack-sizes --demangle` as (file, names, bytes) per entry. A folded entry
    names every function sharing the code; each has the frame."""
    member, names = None, None
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
            yield member, split_top(names), int(m.group(1), 16)
            names = None


class Program:
    """Every function in `llvm-objdump -d -r -t --demangle` of the rlibs, followed by
    `llvm-objdump -r` of the same files for the relocations of their data. `attach` gives each
    function its frame from `llvm-readobj --stack-sizes`.

    A call resolves to a local symbol only in its own object file, and to a global one in every
    file defining it, since a weak definition can stand in more than one place."""

    def __init__(self, code):
        self.functions = {}  # (file, section, value) -> Function
        self.data = {}       # (file, section) -> symbols its relocations name
        self.homes = {}      # (file, name) for a local symbol, name for a global -> (file, section)
        self.defined = set()
        aliases = self.read(code)
        self.index(aliases)

    def read(self, code):
        """Read the dump line by line into functions, data relocations and symbols; return the
        function symbols by address, (exported, local, name), for `index`."""
        aliases = {}
        member = section = current = None
        held = None  # the data section whose relocation records follow
        for line in code.splitlines():
            if m := FILE.match(line):
                member, current, held = m.group(1), None, None
            elif m := SECTION.match(line):
                section, held = m.group(1), None
            elif m := RECORDS.match(line):
                current = None
                held = None if m.group(1).startswith(".text") else (member, m.group(1))
            elif m := RECORD.match(line):
                if held is not None:
                    self.data.setdefault(held, set()).add(m.group(2))
            elif m := LABEL.match(line):
                key = (member, section, int(m.group(1), 16))
                current = self.functions[key] = Function(member, section, m.group(2))
                if key in aliases:
                    current.names = [name for _, _, name in aliases[key]]
                    current.exported = any(e for e, _, _ in aliases[key])
            elif m := INLINE.match(line):
                if current is not None:
                    if CALL.match(m.group(1)):
                        current.calls.append(m.group(2))
                    else:
                        current.references.add(m.group(2))
            elif m := INSTRUCTION.match(line):
                if current is not None:
                    instruction(current, m.group(1), (m.group(2) or "").split(" @ ")[0].strip())
            elif (m := SYMBOL.match(line)) and m.group(3) != "*UND*":
                flags, where, name = m.group(2), m.group(3), m.group(5)
                local = flags[0] == "l"
                self.defined.add(name)
                self.homes[(member, name) if local else name] = (member, where)
                if "F" in flags:
                    exported = not local and not m.group(4)
                    aliases.setdefault((member, where, int(m.group(1), 16)), []).append(
                        (exported, local, name)
                    )
        return aliases

    def index(self, aliases):
        """The lookups `resolve` reads: local names per file, global names, and sections."""
        self.local, self.exported_names, self.sections = {}, {}, {}
        for key, names in aliases.items():
            function = self.functions.get(key)
            if function is None:
                continue
            for _, local, name in names:
                if local:
                    self.local[(key[0], name)] = function
                else:
                    self.exported_names.setdefault(name, []).append(function)
        for function in self.functions.values():
            self.sections.setdefault((function.member, function.section), []).append(function)

    def attach(self, frames):
        """Give each function its frame, the largest any of its names has. Refuses a frame no
        function holds: an alias renamed out from under its entry."""
        named = {(f.member, n): f for f in self.functions.values() for n in f.names}
        for where, names, size in stack_sizes(frames):
            matched = [named[(where, n)] for n in names if (where, n) in named]
            if not matched:
                raise ValueError(f"a frame for {names[0]} in {where}, which the code holds nowhere")
            for function in matched:
                function.frame = max(size, function.frame or 0)
        return self

    def resolve(self, member, symbol):
        """The functions a relocation against `symbol` in file `member` may reach; None if no
        file holds it as code."""
        if (member, symbol) in self.local:
            return [self.local[(member, symbol)]]
        if symbol in self.exported_names:
            return self.exported_names[symbol]
        # A section symbol: an address inside that section, which the walk takes as any of it.
        return self.sections.get((member, symbol))

    def home(self, member, symbol):
        """The (file, section) holding `symbol` as named from file `member`; None if none does."""
        found = self.homes.get((member, symbol)) or self.homes.get(symbol)
        if found is None and (member, symbol) in self.data:
            found = (member, symbol)
        return found

    def held(self, member, symbol):
        """The functions whose addresses the data `symbol` names holds, through any data it
        points at in turn: a table of function pointers, a vtable, a constant structure. None
        if no file defines the symbol at all."""
        start = self.home(member, symbol)
        if start is None:
            return None if symbol not in self.defined else []
        found, seen, pending = [], set(), [start]
        while pending:
            home = pending.pop()
            if home in seen:
                continue
            seen.add(home)
            for target in self.data.get(home, ()):
                code = self.resolve(home[0], target)
                if code is not None:
                    found += code
                elif (inner := self.home(home[0], target)) is not None:
                    pending.append(inner)
        return found


def frame_of(function):
    """The compiler's figure, or, for hand-written assembly it has none for, every byte the
    function's pushes and `sub sp` take: a bound on straight-line code, refused where `sp`
    moves by a register. A Rust path with no frame is compiled code whose entry went missing,
    so it is refused rather than read from its pushes."""
    if function.frame is not None:
        return function.frame
    if any("::" in name for name in function.names):
        raise Refused(f"{function.label} is compiled code with no measured frame")
    if function.dynamic:
        raise Refused(f"{function.label} has no measured frame and moves sp by a register")
    return function.pushed


def reach(root, program):
    """Everything `root` can run: (reached, callees per function, taken, unknown). `taken` is
    every function whose address reached code takes, in code or through the data it points at;
    `unknown`, every symbol it names that no file defines. Refused for a call to code no input
    holds."""
    reached, callees, taken, unknown, pending = set(), {}, set(), set(), [root]
    while pending:
        function = pending.pop()
        if function in reached:
            continue
        reached.add(function)
        callees[function] = []
        for symbol in function.calls:
            found = program.resolve(function.member, symbol)
            if found is None:
                raise Refused(f"{function.label} calls {symbol}, which no input holds")
            callees[function] += found
        pending.extend(callees[function])
        for symbol in function.references:
            if function.is_table(symbol):
                continue
            targets = program.resolve(function.member, symbol)
            if targets is None:
                targets = program.held(function.member, symbol)
            if targets is None:
                unknown.add(symbol)
                continue
            taken.update(targets)
            pending.extend(targets)
    return reached, callees, taken, unknown


def bound(root, reached, taken, unknown):
    """Refuse unless every indirect call `root` reaches is bounded by `taken`.

    A pointer reaches a call from where it was taken or from `root`'s caller, so `taken` bounds
    it under an exported root, whose caller is an integrator's and passes none: no public
    function takes a function pointer or a trait object. Under a root the crate keeps to itself,
    a caller can pass a pointer the root never took."""
    indirect = sorted(f.label for f in reached if f.indirect_calls())
    if not indirect:
        return
    if not root.exported:
        raise Refused(f"{indirect[0]} calls through a register, under a root a caller in the "
                      "crate can pass a pointer")
    if unknown:
        raise Refused(f"{indirect[0]} calls through a register, and {sorted(unknown)[0]} is "
                      "undefined")
    if not taken:
        raise Refused(f"{indirect[0]} calls through a register, and nothing reached takes an "
                      "address")


def deepest(root, callees, taken):
    """The deepest stack beneath `root`, as (bytes, [(function, frame), ...]) along the path;
    an indirect call's callees are `taken`. Refused for a cycle."""
    memo, active = {}, set()

    def depth(function):
        if function in memo:
            return memo[function]
        if function in active:
            raise Refused(f"a cycle through {function.label}")
        active.add(function)
        beneath = list(callees[function])
        if function.indirect_calls():
            beneath += sorted(taken, key=lambda f: f.label)
        down = max((depth(c) for c in beneath), key=lambda d: d[0], default=(0, []))
        own = frame_of(function)
        active.discard(function)
        memo[function] = (own + down[0], [(function.label, own)] + down[1])
        return memo[function]

    return depth(root)


def same_as_sysroot(visited, sysroot):
    """Refuse unless every function the walks visited in the `-Zbuild-std` rebuild of `core` and
    `compiler_builtins` is the sysroot's prebuilt one, which is what links: the same callees,
    pushes and indirect calls. Its frames are read off the rebuild, which only the sysroot's
    code makes them a measure of."""
    shipped = {}
    for function in Program(sysroot).functions.values():
        for name in function.names:
            shipped[unsuffixed(name)] = function
    for function in sorted(visited, key=lambda f: f.label):
        if not REBUILT.search(function.member):
            continue
        prebuilt = shipped.get(unsuffixed(function.label))
        if prebuilt is None:
            raise ValueError(f"build-std's {function.label} is not in the sysroot")
        if function.signature() != prebuilt.signature():
            raise ValueError(f"build-std's {function.label} differs from the sysroot's")


def chains(code, frames):
    """`chain.<function>=` for every function `frame.` keys: its frame and the deepest path of
    calls beneath it, through `nalgebra`, `libm`, `core` and `compiler_builtins` as well as this
    crate. A key holds its largest instance, as `frame.` does, and `refused` if any instance is.
    `stack_peak=` is the largest exported chain, `refused` if any exported one is.

    Returns the pairs, each chain's path for `--path`, and every function a walk visited, for
    `same_as_sysroot`."""
    program = Program(code).attach(frames)
    pairs, paths, visited, peak = {}, {}, set(), []
    for function in program.functions.values():
        keys = {k for k in map(path, function.names) if k is not None}
        if not keys:
            continue
        try:
            reached, callees, taken, unknown = reach(function, program)
            visited |= reached
            bound(function, reached, taken, unknown)
            bytes_, steps = deepest(function, callees, taken)
        except Refused as reason:
            bytes_, steps = "refused", [(str(reason), 0)]
        if function.exported:
            peak.append(bytes_)
        for key in keys:
            key = "chain." + key
            held = pairs.get(key)
            if held == "refused" or (held is not None and bytes_ != "refused" and held >= bytes_):
                continue
            pairs[key], paths[key] = bytes_, steps
    if not peak:
        raise ValueError("no exported chain: was the disassembly taken of this crate's rlib?")
    pairs["stack_peak"] = "refused" if "refused" in peak else max(peak)
    return pairs, paths, visited


def parse(types, frames, flash):
    """The reports as a dict of pairs. Refuses an empty report: a build cargo considered
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

    for _, names, size in stack_sizes(frames):
        for function in names:
            key = path(function)
            if key is not None:
                largest("frame." + key, size)
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


def refused(check, name, call, *inputs):
    """Check that `call(*inputs)` refuses its input with a ValueError."""
    try:
        call(*inputs)
        check(name, "accepted", "refused")
    except ValueError:
        pass


def objects(*files, data=(), entries=()):
    """`llvm-objdump` and `llvm-readobj --stack-sizes` text for synthetic object files.

    Each file is (member, [(binding, names, frame, body)]): a member without an archive is put
    in `lib.rlib`; a binding `g`, `h` (global and hidden: the crate's own) or `l`; a function's
    first name labels its code, the rest alias it; a frame of None writes no stack-size entry;
    and each body line is an instruction (`push\t{r7, lr}`) or a relocation
    (`R_ARM_THM_CALL\tsymbol`). `data` is (member, section, [symbol]): a data section of that
    file and the symbols its relocations name. `entries` is (member, names, frame): a stack-size
    entry more, read after the files' own."""
    code, frames, records = [], [], []

    def archive(member):
        return member if "(" in member else f"lib.rlib({member})"

    def text_of(name):
        return ".text." + re.sub(r"[^\w:.]", "_", name)

    for member, functions in files:
        member = archive(member)
        code += [f"{member}:\tfile format elf32-littlearm", "", "SYMBOL TABLE:"]
        frames += [f"File: {member}", "StackSizes ["]
        for binding, names, frame, _ in functions:
            flag, hidden = ("g", ".hidden ") if binding == "h" else (binding, "")
            for name in names:
                code.append(f"00000000 {flag}     F {text_of(names[0])}\t00000010 {hidden}{name}")
            if frame is not None:
                frames += ["  Entry {", f"    Functions: [{', '.join(names)}]",
                           f"    Size: 0x{frame:X}", "  }"]
        for where, section, _ in data:
            if archive(where) == member:
                code.append(f"00000000 l    d  {section}\t00000000 {section}")
        for _, names, _, body in functions:
            code += ["", f"Disassembly of section {text_of(names[0])}:", "", f"00000000 <{names[0]}>:"]
            for i, step in enumerate(body):
                if step.startswith("R_ARM"):
                    code.append(f"\t\t\t{i:08x}:  {step}")
                else:
                    code.append(f"{i * 2:8x}: b5f0         \t{step}")
        frames.append("]")
    for where, section, symbols in data:
        records += [f"{archive(where)}:\tfile format elf32-littlearm", "",
                    f"RELOCATION RECORDS FOR [{section}]:", "OFFSET   TYPE                     VALUE"]
        records += [f"{i * 4:08x} R_ARM_ABS32              {s}" for i, s in enumerate(symbols)]
    for where, names, frame in entries:
        frames += [f"File: {archive(where)}", "StackSizes [", "  Entry {",
                   f"    Functions: [{', '.join(names)}]", f"    Size: 0x{frame:X}", "  }", "]"]
    return "\n".join(code + [""] + records), "\n".join(frames)


def walk_fixtures(check):
    """The walk against call graphs whose depth is known by construction. Each value is written
    out as the sum it is, so a reader can see which frames the path takes. Where a guard's
    mutation is not obvious, the fixture says which one it kills."""
    f = "fusion_nav::f::"
    call = "R_ARM_THM_CALL\t" + f
    a = [
        # `big` has the largest frame under `root`, but `mid` and its leaf are deeper: the case
        # summing named frames gets wrong.
        ("g", [f + "root"], 100, [call + "big", call + "mid", call + "dup"]),
        ("h", [f + "big"], 300, []),
        ("h", [f + "mid"], 50, ["R_ARM_THM_JUMP24\t" + f + "leaf"]),
        ("h", [f + "leaf"], 400, []),
        # Local in both files, deeper in `b`: a call reaches its own file's copy.
        ("l", [f + "dup"], 8, []),
        # A conditional tail call's relocation, which a list of the two call types misses.
        ("g", [f + "near"], 10, ["R_ARM_THM_JUMP19\t" + f + "leaf"]),
        # `apply` calls through a register; the only address `fuse` takes is `select`'s, so the
        # bound is `select` and not `deep`, whose address an unreached `other` takes.
        ("g", [f + "fuse"], 20, ["R_ARM_ABS32\t" + f + "select", call + "apply"]),
        ("h", [f + "apply"], 30, ["blx\tr2"]),
        ("h", [f + "select"], 4, ["bx\tlr"]),
        ("g", [f + "other"], 1, ["R_ARM_ABS32\t" + f + "deep"]),
        ("h", [f + "deep"], 1000, []),
        # The same `apply` from a root the crate does not export: its caller can pass any
        # pointer, so the address `select` it takes bounds nothing. Kills an exported-only
        # rule that reads every root as exported.
        ("h", [f + "inner"], 6, ["R_ARM_ABS32\t" + f + "select", call + "apply"]),
        # The pointer reaches `table_user` through a table in data, as a constant structure
        # holding a `fn` would: the bound is `heavy`, which no code relocation names.
        ("g", [f + "table_user"], 2, ["R_ARM_ABS32\t.rodata.table", "bx\tr3"]),
        ("h", [f + "heavy"], 900, []),
        # A tail call and a conditional call through a register, and a jump loaded into pc:
        # each bounded by `select`, the one address taken.
        ("g", [f + "tail"], 2, ["R_ARM_ABS32\t" + f + "select", "bx\tr1"]),
        ("g", [f + "cond"], 2, ["R_ARM_ABS32\t" + f + "select", "blxne\tr1"]),
        ("g", [f + "load"], 2, ["R_ARM_ABS32\t" + f + "select", "ldr.w\tpc, [r1, #4]"]),
        # A jump table: `mov pc` through entries in its own section is no call.
        ("g", [f + "table"], 12, ["mov\tpc, r2", f"R_ARM_ABS32\t.text.{f}table"]),
        # A section symbol: a call into `.text.<leaf>` reaches `leaf`.
        ("g", [f + "by_section"], 3, [f"R_ARM_THM_CALL\t.text.{f}leaf"]),
        # Hand-written assembly with no stack-size entry, read from its pushes, called through
        # an alias the label does not show. Each push form counts.
        ("g", [f + "asm"], 0, ["R_ARM_THM_CALL\t__naked_alias"]),
        ("g", ["__naked", "__naked_alias"], None,
         ["push\t{r4, lr}", "sub\tsp, #0x10", "add\tsp, #0x10", "pop\t{r4, pc}"]),
        ("g", [f + "asm2"], 0, ["R_ARM_THM_CALL\t__wide"]),
        ("g", ["__wide"], None,
         ["str\tr4, [sp, #-8]!", "subw\tsp, sp, #0x100", "add.w\tsp, sp, #-0x20",
          "stmdb\tsp!, {r5, r6}", "vpush\t{d8-d9}"]),
        # Aliases with frames in two entries, the smaller added below and read last; the
        # function has the larger.
        ("g", [f + "twice"], 0, [call + "folded_b"]),
        ("h", [f + "folded_a", f + "folded_b"], 60, []),
        ("g", [f + "cycle"], 4, [call + "again"]),
        ("h", [f + "again"], 4, [call + "cycle"]),
        ("g", [f + "missing"], 4, [call + "nowhere"]),
        ("g", [f + "blind"], 4, ["blx\tr3"]),
        ("g", [f + "unknown"], 4, ["R_ARM_ABS32\tnowhere_data", "R_ARM_ABS32\t" + f + "select",
                                   "blx\tr3"]),
        ("g", [f + "pointer"], 4, ["R_ARM_THM_CALL\t__moved"]),
        ("g", ["__moved"], None, ["push\t{r4, lr}", "add\tsp, r6"]),
        ("g", [f + "lost"], 4, [call + "frameless"]),
        ("h", [f + "frameless"], None, ["bx\tlr"]),
        # Two instances of one key, one refused: the key is refused, whichever comes first.
        ("g", [f + "gen::<1, A>"], 4, []),
        ("g", [f + "gen::<1, B>"], 4, [call + "nowhere"]),
        ("g", [f + "gen::<2, B>"], 4, [call + "nowhere"]),
        ("g", [f + "gen::<2, A>"], 4, []),
        # A global defined in two files: either may be the one that links, so the deeper.
        ("g", [f + "weak_user"], 1, [call + "weak"]),
        ("g", [f + "weak"], 5, []),
    ]
    b = [
        ("l", [f + "dup"], 800, []),
        ("g", [f + "weak"], 70, []),
    ]
    data = [("a.o", ".rodata.table", [f + "heavy"])]
    entries = [("a.o", [f + "folded_b"], 40)]
    pairs, paths, _ = chains(*objects(("a.o", a), ("b.o", b), data=data, entries=entries))
    want = {
        "root": 100 + 50 + 400,
        "near": 10 + 400,
        "fuse": 20 + 30 + 4,
        "inner": "refused",
        "table_user": 2 + 900,
        "tail": 2 + 4,
        "cond": 2 + 4,
        "load": 2 + 4,
        "table": 12,
        "by_section": 3 + 400,
        "asm": 0 + (2 + 4) * 4,
        "asm2": 8 + 0x100 + 0x20 + 2 * 4 + 2 * 8,
        "twice": 60,
        # Keyed by the alias as well as the label: one function, two names in the index.
        "folded_b": 60,
        "cycle": "refused",
        "missing": "refused",
        "blind": "refused",
        "unknown": "refused",
        "pointer": "refused",
        "lost": "refused",
        "gen.1": "refused",
        "gen.2": "refused",
        "weak_user": 1 + 70,
        "stack_peak": "refused",
    }
    for name, value in want.items():
        key = name if name == "stack_peak" else "chain.f." + name
        check(f"chain {name}", pairs.get(key), value)
    check("path", [s for _, s in paths["chain.f.root"]], [100, 50, 400])
    check("registers", registers("{d8-d11}"), (4, 8))

    # The peak is the deepest exported chain; a deeper one the crate keeps to itself is read
    # through its exported callers, and an exported one refused refuses the peak above.
    pairs, _, _ = chains(*objects(("a.o", [
        ("g", [f + "entry"], 10, [call + "inside"]),
        ("h", [f + "inside"], 20, []),
        ("h", [f + "orphan"], 5000, []),
    ])))
    check("peak", pairs["stack_peak"], 10 + 20)

    # A frame the code holds nowhere: an alias renamed out from under its entry.
    refused(check, "an unmatched frame", chains,
            *objects(("a.o", [("g", [f + "entry"], 10, [])]), entries=[("a.o", [f + "renamed"], 16)]))

    # A rebuilt builtin compared with the sysroot's: the same, then pushing one more register.
    rebuilt = "std/libcompiler_builtins-0123abcd.rlib(cb.o)"
    pairs, _, visited = chains(*objects(
        ("a.o", [("g", [f + "entry"], 10, ["R_ARM_THM_CALL\t__aeabi_x"])]),
        (rebuilt, [("g", ["__aeabi_x"], 8, ["push\t{r7, lr}"])]),
    ))
    same, _ = objects(("sysroot/libcompiler_builtins-ffff.rlib(cb.o)",
                       [("g", ["__aeabi_x"], None, ["push\t{r7, lr}"])]))
    differs, _ = objects(("sysroot/libcompiler_builtins-ffff.rlib(cb.o)",
                          [("g", ["__aeabi_x"], None, ["push\t{r4, r7, lr}"])]))
    check("rebuilt builtin", pairs["chain.f.entry"], 10 + 8)
    check("sysroot same", same_as_sysroot(visited, same), None)
    refused(check, "a rebuilt builtin unlike the sysroot's", same_as_sysroot, visited, differs)
    refused(check, "a rebuilt builtin the sysroot lacks", same_as_sysroot, visited, "")


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

    refused(check, "no types", parse, "", frames, flash)
    refused(check, "no frames", parse, types, "", flash)
    refused(check, "no flash", parse, types, frames, "")
    refused(check, "a section before a level", parse, types, frames, ".text  1  0")
    refused(check, "an unbalanced list", parse, types, "Functions: [fusion_nav::f::<1]\nSize: 0x10",
            flash)

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
    show = argv[1] if argv[:1] == ["--path"] and len(argv) == 5 else None
    if show is None and len(argv) != 5:
        sys.exit(__doc__.split("\n\n")[1])
    try:
        texts = []
        for name in argv[2:] if show else argv:
            with open(name) as f:
                texts.append(f.read())
        if show is None:
            types, frames, flash, code, sysroot = texts
            pairs = parse(types, frames, flash)
        else:
            frames, code, sysroot = texts
        walked, paths, visited = chains(code, frames)
        same_as_sysroot(visited, sysroot)
        if show is None:
            print(line(pairs | walked))
            return
        if show not in paths:
            raise ValueError(f"no {show}")
        print(f"{show}={walked[show]}")
        for function, frame in paths[show]:
            print(f"{frame:8} {function}")
    except (OSError, ValueError) as error:
        sys.exit(f"footprint: {error}")


if __name__ == "__main__":
    main(sys.argv[1:])
