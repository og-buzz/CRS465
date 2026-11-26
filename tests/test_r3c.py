"""The RAPL-3 compiler on the PC (a465/r3c.py, a465/crosemu.py, scripts/a465-r3c): vendor rc1/rpp under Unicorn.

Skipped when the unicorn module (python3-unicorn) is missing. Nothing here touches the controller.
"""
import errno
import os
import subprocess
import sys

import pytest

pytest.importorskip("unicorn")

from a465 import crosprog, r3dis, r3vm, r3c            # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "r3")
VENDOR = r3c.VENDOR
APP = os.path.join(VENDOR, "c500c-3.1", "app", "test")
OURLIB = os.path.join(VENDOR, "c500-1.16.471", "lib")
SCRIPT = os.path.join(os.path.dirname(HERE), "scripts", "a465-r3c")

FIB_TIME, SIEVE_TIME = 0x3B2A12BD, 0x3B2A12BF     # the build times of the vendor's own fib and sieve


def rd(path):
    with open(path, "rb") as f:
        return f.read()


def build(tmp_path, name, **kw):
    out = str(tmp_path / name)
    res = r3c.compile(kw.pop("source", os.path.join(SRC, name + ".r3")), out=out, **kw)
    assert res.status == 0 and res.output == out, res.messages
    return rd(out), res


def test_fib_and_sieve_match_the_vendors_own_builds(tmp_path):
    fib, _ = build(tmp_path, "fib", source=os.path.join(APP, "fib.r3"), libs="c500c", options=["-s"], time=FIB_TIME)
    assert fib == rd(os.path.join(APP, "fib"))
    sieve, _ = build(tmp_path, "sieve", source=os.path.join(APP, "sieve.r3"), libs="c500c", options=["-s"],
                     time=SIEVE_TIME)                       # sieve uses .define: goes through rpp
    assert sieve == rd(os.path.join(APP, "sieve"))


@pytest.mark.parametrize("name,want", [
    ("hello", "hello from rc1\n"),
    ("calc", "sum of fib(1..10) = 143\nquarter = 35.75\ndone\n"),
])
def test_programs_built_against_our_libraries_decode_and_run(tmp_path, name, want):
    data, _ = build(tmp_path, name, options=["-Wall"], time=0x692600C0)
    assert crosprog.check_ok(data)
    ins = r3dis.decode(r3dis.Module(data))
    assert not [i for i in ins if i.kind == "data" and i.text.startswith("; data")]
    vm = r3vm.VM(data, argv=[name])
    assert vm.run() == 0
    assert vm.stdout() == want


def test_syntax_error_gives_messages_and_failure(tmp_path):
    res = r3c.compile(os.path.join(SRC, "bad.r3"), out=str(tmp_path / "bad"), options=["-Wall"])
    assert res.status != 0 and res.output is None
    assert "bad.r3:3: Error: Syntax error" in res.messages
    assert not (tmp_path / "bad").exists()
    p = subprocess.run([sys.executable, SCRIPT, os.path.join(SRC, "bad.r3"), "-o", str(tmp_path / "bad2")],
                       capture_output=True, text=True)
    assert p.returncode != 0 and "Syntax error" in p.stdout and "compile failed" in p.stderr


def test_our_libraries_stamp_the_output(tmp_path):
    data, res = build(tmp_path, "hello", options=["-s"])
    libs = {lib.name: (lib.build_time, lib.signature) for lib in crosprog.parse(data).libs}
    assert libs["syslib"][1] == 0x0A0D4875
    ours = {n: crosprog.stamp(rd(os.path.join(OURLIB, n + ".r"))) for n in ("syslib", "robotlib")}
    assert libs == {k: ours[k] for k in libs}
    # no sample program calls robotlib, so its stamp is checked on the library file itself
    assert crosprog.stamp(rd(os.path.join(OURLIB, "robotlib.r")))[1] == 0xAE952686
    p = subprocess.run([sys.executable, SCRIPT, os.path.join(SRC, "hello.r3"), "-o", str(tmp_path / "h"), "-s",
                        "--stamp-check"], capture_output=True, text=True)
    assert p.returncode == 0 and "syslib" in p.stdout and "0x0a0d4875" in p.stdout and "matches" in p.stdout


# --- the shim's sandbox -----------------------------------------------------------------------------------

def make_emu(tmp_path, reads=()):
    from a465.crosemu import Emu
    out = tmp_path / "out"
    out.mkdir()
    emu = Emu(os.path.join(r3c.BIN, "rpp"), str(out), list(reads), None, None, None, [], ["rpp"], echo=False)
    return emu, out


def sys_open(emu, path, flags):
    addr = emu.end_bss + 0x100
    emu.uc.mem_write(addr, path.encode() + b"\0")
    rec = {}
    return emu.dispatch(2, [addr, flags, 7, 0], rec), rec


def test_shim_refuses_reads_outside_the_allowed_files(tmp_path):
    allowed, other = tmp_path / "allowed.txt", tmp_path / "other.txt"
    allowed.write_text("a")
    other.write_text("secret")
    emu, _ = make_emu(tmp_path, reads=[allowed])
    fd, _ = sys_open(emu, str(allowed), 1)
    assert fd >= 3
    ret, rec = sys_open(emu, str(other), 1)
    assert ret == -errno.ENOENT and "denied" in rec
    for p in ("/etc/passwd", "../other.txt"):
        assert sys_open(emu, p, 1)[0] == -errno.ENOENT
    assert emu.dispatch(19, [emu.end_bss + 0x100, emu.end_bss + 0x200, 0, 0], {}) in (0, -errno.ENOENT)


def test_shim_refuses_writes_outside_the_output_directory(tmp_path):
    emu, out = make_emu(tmp_path)
    target = tmp_path / "escape.txt"
    for p in (str(target), "../escape.txt", "/tmp/claude-1000/never-created"):
        ret, rec = sys_open(emu, p, 0x602)
        assert ret == -errno.EACCES and "denied" in rec
    assert not target.exists()
    ok, _ = sys_open(emu, "inside.txt", 0x602)               # create + truncate + write, inside: allowed
    assert ok >= 3 and (out / "inside.txt").exists()
    # unlink and chmod outside the directory are refused too
    victim = tmp_path / "victim.txt"
    victim.write_text("x")
    addr = emu.end_bss + 0x100
    emu.uc.mem_write(addr, str(victim).encode() + b"\0")
    assert emu.dispatch(13, [addr, 0, 0, 0], {}) == -errno.EACCES and victim.exists()
    assert emu.dispatch(21, [addr, 7, 0, 0], {}) == -errno.EACCES
    # an unknown system call stops the guest instead of passing through
    with pytest.raises(Exception):
        emu.dispatch(2000, [0, 0, 0, 0], {})
