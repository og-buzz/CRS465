"""The RAPL-3 compiler on the PC: CRS's own rpp and rc1, run only inside the Unicorn emulator.

This does what the controller's /bin/r3c driver does: preprocess with rpp, then compile with rc1, each
under a465/crosemu.py (which answers every system call itself; nothing of the vendor's runs natively).
Nothing here talks to the controller. See docs/RAPL3-COMPILER.md for the evidence and what is unproven.

    from a465 import r3c
    res = r3c.compile("hello.r3", out="hello", options=["-s"])
    print(res.messages, res.status)      # res.output is the path of the object file, or None

Library sets (`libs=`): "ours" is this unit's CROS 1.16.471 syslib and robotlib (vendor/crs/c500-1.16.471/lib),
so the program carries this unit's stamps and robotlib routine numbers; "c500c" is the C500C's CROS 3.1
libraries (syslib, robotlib, v3access, acid), which gives output that matches the vendor's own builds
byte for byte but is wrong for a robotlib user on our unit. Extra libraries go in `extra_libs`; files the
source `.include`s go in `extra`. Options in `options` are handed to rc1 unchanged (-Wall -s -x -l -fstack=N ...).
"""
import os
import shutil
import tempfile
from dataclasses import dataclass, field

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENDOR = os.path.join(REPO, "vendor", "crs")
BIN = os.path.join(VENDOR, "c500c-3.1", "bin")
LIBSETS = {
    "ours": [os.path.join(VENDOR, "c500-1.16.471", "lib", n) for n in ("syslib.r", "robotlib.r")],
    "c500c": [os.path.join(VENDOR, "c500c-3.1", "lib", n) for n in ("syslib.r", "robotlib.r", "v3access.r", "acid.r")],
}


@dataclass
class Result:
    status: int                     # 0 = compiled; rpp's or rc1's exit status otherwise (125: the emulated guest died)
    messages: str                   # what rpp and rc1 printed (stdout and stderr, in order)
    output: str = None              # path of the object file, None on failure
    work: str = None                # work directory, only when keep_work
    libs: list = field(default_factory=list)   # library files passed to rc1, in order


def _emu(exe, outdir, reads, argv, time=None, log=None):
    from a465.crosemu import Emu         # imported late: needs the unicorn module
    emu = Emu(exe, outdir, reads, None, time, log, [], argv, echo=False)
    st = emu.run()
    text = (bytes(emu.captured[1]) + bytes(emu.captured[2])).decode("latin-1")
    if emu.died:
        text += f"[crosemu] guest died: {emu.died}\n"
    return st, text


def compile(source, out=None, libs="ours", extra_libs=(), extra=(), options=(), time=None,
            use_rpp=True, defines=(), keep_work=False):
    """Compile `source` (a .r3 file) and write the object file to `out` (default: the source's name
    without extension, in the current directory). `time` pins the build time stored in the header
    (default: now). Returns a Result; never raises for a compile error."""
    source = os.path.abspath(source)
    if not os.path.isfile(source):
        raise FileNotFoundError(source)
    if libs not in LIBSETS:
        raise ValueError(f"libs must be one of {sorted(LIBSETS)}")
    libfiles = [os.path.abspath(p) for p in LIBSETS[libs]] + [os.path.abspath(p) for p in extra_libs]
    for p in libfiles + [os.path.join(BIN, "rc1"), os.path.join(BIN, "rpp")]:
        if not os.path.isfile(p):
            raise FileNotFoundError(p)
    name = os.path.splitext(os.path.basename(source))[0]
    out = os.path.abspath(out or name)
    work = tempfile.mkdtemp(prefix="r3c-")
    messages = ""
    try:
        # The source (and `.include` files) are copied into the work directory and named relative to it:
        # rc1 keeps only 60 characters of a file name in its line markers.
        copies = []
        for f in [source] + [os.path.abspath(e) for e in extra]:
            dst = os.path.join(work, os.path.basename(f))
            shutil.copyfile(f, dst)
            copies.append(dst)
        src = os.path.basename(source)
        rc1_in = src
        if use_rpp:
            rc1_in = name + ".i"
            argv = ["rpp", "-D__C500__", "-D__RAPL3__"] + ["-D" + d for d in defines] + [src, rc1_in]
            st, text = _emu(os.path.join(BIN, "rpp"), work, copies, argv)
            messages += text
            if st != 0:
                return Result(st or 1, messages, libs=libfiles, work=work if keep_work else None)
        obj = "r3c.out"
        argv = ["rc1"] + ["-L" + p for p in libfiles] + list(options) + ["-o" + obj, rc1_in]
        st, text = _emu(os.path.join(BIN, "rc1"), work, [os.path.join(work, rc1_in)] + libfiles, argv, time=time)
        messages += text
        done = os.path.join(work, obj)
        if st == 0 and os.path.isfile(done):
            os.makedirs(os.path.dirname(out), exist_ok=True)
            shutil.copyfile(done, out)
            return Result(0, messages, out, work if keep_work else None, libfiles)
        return Result(st or 1, messages, None, work if keep_work else None, libfiles)
    finally:
        if not keep_work:
            shutil.rmtree(work, ignore_errors=True)


def stamp_report(path, libfiles):
    """Lines describing the library stamps a compiled file carries, and whether each matches `libfiles`."""
    from a465 import crosprog
    data = open(path, "rb").read()
    have = {os.path.splitext(os.path.basename(p))[0]: crosprog.stamp(open(p, "rb").read()) for p in libfiles}
    lines = [f"{path}: {len(data)} bytes, check word {'ok' if crosprog.check_ok(data) else 'BAD'}"]
    for lib in crosprog.parse(data).libs:
        want = have.get(lib.name)
        verdict = "no such library in the chosen set" if want is None else (
            "matches" if want == (lib.build_time, lib.signature) else "DIFFERS from the chosen set")
        lines.append(f"  {lib.name}: build time {lib.build_time:#x}, signature {lib.signature:#010x}  ({verdict})")
    return lines
