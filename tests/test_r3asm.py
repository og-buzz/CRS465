"""a465/r3asm.py: lifting every vendor program of the standard shape to source and assembling it again
gives the original file byte for byte; the stream encoder reproduces every data stream and frame
initialiser of every sample; the header, library records and errors behave as documented.

The vendor files are this unit's own copies under sessions/controller-files/ (git-ignored) and the
two stock examples under controller/backup-2026-11-23/app/test/; tests that need them skip without.
"""
import re
import struct
from pathlib import Path

import pytest

from a465 import crosprog, r3asm, r3dis

REPO = Path(__file__).resolve().parent.parent
FILES = REPO / "sessions" / "controller-files"
TEST_APP = REPO / "controller" / "backup-2026-11-23" / "app" / "test"

# Every program of the standard shape among the samples (bin/pendant and the libraries are not).
PROGRAMS = [
    "patched/homezc-plain", "patched/homezc-offset", "patched/homezc", "patched/calzc",
    "diag/zero", "diag/xzero", "diag/cal", "diag/calseek", "diag/setup",
    "bin/home", "bin/calrdy", "bin/cp", "bin/more", "bin/siocfg", "bin/v3make", "bin/shell", "bin/ash",
    "sbin/f3adjust", "sbin/fastacid", "sbin/simsockd", "sbin/robotsrv",
]


def sample(rel):
    path = TEST_APP / rel if rel in ("fib", "sieve") else FILES / rel
    if not path.exists():
        pytest.skip(f"{path.relative_to(REPO)} is not here")
    return r3dis.load(path)


def all_samples():
    paths = sorted(p for p in FILES.glob("*/*") if p.is_file()) + sorted(TEST_APP.glob("*"))
    out = []
    for p in paths:
        data = r3dis.load(p)
        if data[:4] == r3dis.MAGIC:
            out.append((p, data))
    if not out:
        pytest.skip("no compiled samples here")
    return out


@pytest.mark.parametrize("rel", PROGRAMS + ["fib", "sieve"])
def test_lifted_source_assembles_to_the_same_bytes(rel):
    original = sample(rel)
    source = r3asm.lift(original, rel)
    assert ".word" not in source                     # every code word came back as an instruction
    assert r3asm.assemble(source, rel) == original


def test_homezc_lifts_to_symbolic_source():
    source = r3asm.lift(sample("patched/homezc-plain"), "homezc")
    lines = [line.strip() for line in source.splitlines()]
    assert lines[1:5] == [".build 0x377a89a8", ".entry func_0001",
                          ".lib syslib 2 0x377a8993 0x0a0d4875 5", ".lib robotlib 3 0x377a899b 0xae952686 2"]
    assert 'G7: string "{}: invalid command line arguments.\\n"' in lines
    for want in ("enter 10", "eqc L5, L6, 2", "ifnot L5, loc_0026", "pushlea L9", "builtin str_to_int",
                 "push L8 tag 19", "pushleag G0", "builtinx fprintf", "try loc_0044", "libcall robotlib:0x4a",
                 "chkpop", "goto loc_005e", "builtin error_code", "neg L5, L6", "libcall syslib:0x7", "trypop",
                 "retv 0"):
        assert want in lines, want


def test_sources_move_with_their_labels():
    """Code and data are addressed through labels: inserting a word moves every target and the
    file still disassembles to the same instructions after it."""
    original = sample("diag/xzero")
    source = r3asm.lift(original, "xzero")
    moved = re.sub(r"(func_0001:\n    enter 41[^\n]*\n)", r"\1    ldi L6, 0\n", source, count=1)
    moved = moved.replace(".data\n", ".data\n    zeros 1\n", 1)
    image = r3asm.assemble(moved)
    old = [i.text for i in r3dis.decode(r3dis.Module(original))]
    new = [i.text for i in r3dis.decode(r3dis.Module(image))]
    assert len(new) == len(old) + 1 and new[2] == "L6 = 0"
    assert "try handler 0x00d4" in new and "try handler 0x00d3" in old     # moved by the inserted word
    strings = {r3dis.Module(image).string_at(n + 1) for n in range(96)} - {None}
    assert "Usage: {} [-v] axislist\n" in strings


def test_stream_encoder_reproduces_every_sample():
    """Data streams and `enter` initialisers of all files, libraries included."""
    n_streams = n_frames = 0
    for path, data in all_samples():
        w = struct.unpack_from("<16I", data, 0)
        decoded, relocs, used = r3dis.init_stream(data, w[6])
        assert r3asm.encode_stream(decoded, relocs) == data[w[6]:w[6] + used], path.name
        n_streams += 1
        mod = r3dis.Module(data, path.name)
        for ins in r3dis.decode(mod):
            if ins.op == 0x54 and ins.kind != "data":
                body = data[64 + 4 * (ins.addr + 1):64 + 4 * (ins.addr + ins.words)]
                init, rel, used = r3dis.init_stream(body)
                assert r3asm.encode_stream(init, rel) == body[:used], (path.name, ins.addr)
                assert len(init) == 4 * (-r3dis.s16(ins.word >> 16) - 1)
                assert not any(body[used:])
                n_frames += 1
    assert n_streams >= 1 and n_frames >= 1


def test_stream_encoding_rules():
    enc = r3asm.encode_stream
    assert enc(b"") == b"\xff"
    assert enc(b"a\0b\0\0c") == b"a\0b\0\0c\xff"                     # 1 or 2 zeros as themselves
    assert enc(bytes(3)) == b"\xfd\x03\xff" and enc(bytes(255)) == b"\xfd\xff\xff"
    assert enc(bytes(256)) == b"\xfe\x00\x01\xff"
    assert enc(b"\xfa\xfb\xfc\xfd\xfe\xff") == b"\xfa\xfc\xfb\xfc\xfc\xfc\xfd\xfc\xfe\xfc\xff\xff"
    assert enc(bytes(8), relocs=[4]) == b"\xfd\x04\xfb\0\0\0\0\xff"   # a pointer ends a zero run
    for data, relocs in ((bytes(70000), []), (b"\x01\0\0\0\xff\0\0\0\0\0", [4])):
        decoded, got, _ = r3dis.init_stream(enc(data, relocs))
        assert (decoded, got) == (data, relocs)


def test_header_and_library_records():
    source = """
.build 0x12345678
.entry main
.lib syslib 2 0x377a8993 0x0a0d4875 5
.lib robotlib 3 0x377a899b 0xae952686 2
.data
msg: string "hi\\n"
.code
helper:
    enter 5
    ret
main:
    enter 6
    pushi 0
    pushleag msg
    pushi 2
    builtinx printf
    pushi 0
    libcall robotlib:ctl_rel
    pop L4
    retv 0
"""
    image = r3asm.assemble(source)
    w = struct.unpack_from("<16I", image, 0)
    mod = r3dis.Module(image)
    assert w[0] == 0x10ADAB1F and w[2] == 0x12345678 and w[3] == 0x00810004      # entry: main at word 4
    assert w[4] == (mod.data_words << 16 | mod.code_words) and w[6] == 64 + 4 * mod.code_words
    assert w[13] == 1 and w[1] == w[5] == w[7] == w[9] == w[10] == w[11] == w[15] == 0
    assert crosprog.check_ok(image)
    prog = crosprog.parse(image)
    assert [(lib.name, lib.slot, lib.build_time, lib.signature) for lib in prog.libs] == [
        ("syslib", 2, 0x377A8993, 0x0A0D4875), ("robotlib", 3, 0x377A899B, 0xAE952686)]
    assert w[12] + 4 + struct.unpack_from("<I", image, w[12])[0] == len(image)
    assert mod.code[0] == 0x44 and mod.string_at(0) == "hi\n"
    assert any("call robotlib:0x4" in line for line in r3dis.listing(mod))
    assert r3asm.lift(image) and r3asm.assemble(r3asm.lift(image)) == image


def test_library_stamps_are_this_controllers():
    for lib, want in (("syslib.r", r3asm.SYSLIB), ("robotlib.r", r3asm.ROBOTLIB)):
        path = FILES / "lib" / lib
        if not path.exists():
            pytest.skip(f"{lib} is not here")
        assert r3asm.library_record(path.read_bytes(), want.name, want.slot) == want


@pytest.mark.parametrize("rel,why", [("bin/pendant", "word 1 is 0x10000"), ("lib/syslib.r", "word 3 is 0x0")])
def test_other_shapes_are_refused(rel, why):
    with pytest.raises(r3asm.LiftError, match=why):
        r3asm.lift(sample(rel), rel)


@pytest.mark.parametrize("body,message", [
    ("    frob L1", "unknown mnemonic"),
    ("    eqc L1, L2, 300", "does not fit a signed byte"),
    ("    ldi L1, 40000", "does not fit a signed 16-bit"),
    ("    goto nowhere", "not a number or a defined name"),
    ("    push L1 tag 1\n    pop L1 tag 2", "takes no tag"),
    ("x:\nx:\n    ret", "defined twice"),
    ("    libcall nolib:1", "no .lib"),
    ("    enter 3 init 00", "has 8 bytes"),
    ("    add L1, L2", "takes 3 operand"),
])
def test_errors_name_the_line(body, message):
    source = ".build 0\n.entry main\n.lib syslib 2 0 0 5\n.code\nmain:\n" + body + "\n"
    with pytest.raises(r3asm.AsmError, match=message):
        r3asm.assemble(source)


def test_assembled_instructions_disassemble_as_written():
    source = """.build 1
.entry main
.lib syslib 2 0 0 5
.equ LIMIT 300
.data
buf: string 4 ""
v: word 7, -1
p: ptr text:main
.code
main:
    enter 4
    ldi L4, LIMIT
    ldlit L3, 0x12345678
    gtc L2, L4, 'A'
    sub L4, L3, L2
    stp L4, L3, 1
    ldx L4, L3, L2
    getg L4, v
    putg L4, v
    lea L3, A4
    leag L3, buf
    pushaddr text:main
    switch L4, table
    goto main
table:
    case main, 1, 3
    case main, 'q', 'q'
    endcases
    libcall syslib:delay
    builtin 0x84
    retv L4
"""
    mod = r3dis.Module(r3asm.assemble(source))
    texts = [i.text for i in r3dis.decode(mod, {"syslib": r3dis.SYSLIB})][1:]
    assert texts == [
        "enter 4", "L4 = 300", "L3 = 305419896 (0x12345678)", "L2 = L4 > 65", "L4 = L3 - L2", "L3@[1] = L4",
        "L4 = L3@[L2]", "L4 = G2", "G2 = L4", "L3 = &A4", "L3 = &G0", "push addr <lib1 text 0x1>",
        "switch L4 table 0x0012", "goto 0x0001", "; switch table: 1..3 -> 0x0001, 113..113 -> 0x0001; end",
        "call syslib:0x14 delay", "builtin 0x84 str_to_int", "return L4"]
    assert mod.data_relocs == [16] and mod.data_word(4) == 0x00810001 and mod.data_word(3) == 0xFFFFFFFF
