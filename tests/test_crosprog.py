"""a465/crosprog.py: the check value and library stamps of compiled RAPL-3 programs.

The synthetic program below is built here from the layout the module documents. The tests against real
files read this unit's own copies under sessions/controller-files/ (git-ignored, copied off the controller
on 2026-11-23) and skip when those are absent.
"""
import struct
from pathlib import Path

import pytest

from a465 import crosprog

FILES = Path(__file__).resolve().parent.parent / "sessions" / "controller-files"


def make_lib(build_time, signature):
    """A library-shaped file: only the header matters to stamp()."""
    words = [0x10ADAB1F, 0, build_time, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, signature, 0, 0]
    data = bytearray(struct.pack("<16I", *words))
    struct.pack_into("<I", data, crosprog.CHECK_AT, crosprog.compute_check(data))
    return bytes(data)


def make_program(build_time, libs, call=0x4A):
    """Header, three code words (one of them library call `call`), a library table naming `libs`, the names block."""
    code = bytes.fromhex("44000000") + bytes((0x56, 0x83, call, 0x00)) + bytes.fromhex("58000000")
    table_at = crosprog.HEADER_BYTES + len(code)
    names = b"".join(name.encode() + b"\0" for name, _, _ in libs)
    table = struct.pack("<HH", len(libs), 0)
    at = 0
    for i, (name, built, signature) in enumerate(libs):
        table += struct.pack("<HHIHHI", at, 2 + i, built, 0, 5 if name == "syslib" else 2, signature)
        at += len(name) + 1
    names_at = table_at + len(table)
    words = [0x10ADAB1F, 0, build_time, 0x810001, (3 << 0) | (0x13 << 16), 0, table_at, 0, table_at, 0, 0, 0, names_at, 1, 0, 0]
    words[6] = table_at
    data = bytearray(struct.pack("<16I", *words)) + code + table + struct.pack("<HH", (len(names) + 3) & ~3, 0) + names
    struct.pack_into("<I", data, crosprog.CHECK_AT, crosprog.compute_check(data))
    return bytes(data)


SYS_99, ROBOT_99 = make_lib(0x377A8993, 0x0A0D4875), make_lib(0x377A899B, 0xAE952686)
SYS_01, ROBOT_01 = make_lib(0x3B2A12B0, 0x0A0D4875), make_lib(0x3B2A12B3, 0x51B97334)
PROG_01 = make_program(0x3B2A12B8, [("syslib", 0x3B2A12B0, 0x0A0D4875), ("robotlib", 0x3B2A12B3, 0x51B97334)])


def test_the_check_makes_the_first_fifteen_header_words_add_to_zero():
    assert sum(struct.unpack_from("<15I", PROG_01, 0)) & crosprog.MASK == 0
    assert crosprog.check_ok(PROG_01)


def test_one_changed_build_time_second_changes_the_check_by_the_opposite_amount():
    later = bytearray(PROG_01)
    later[8] += 2
    assert not crosprog.check_ok(later)
    assert crosprog.compute_check(later) == (crosprog.parse(PROG_01).check - 2) & crosprog.MASK


def test_the_check_does_not_cover_the_body():
    body = bytearray(PROG_01)
    body[crosprog.HEADER_BYTES + 6] -= 1                # a different library call number
    assert crosprog.check_ok(body)


def test_library_records_are_read():
    prog = crosprog.parse(PROG_01)
    assert prog.build_time == 0x3B2A12B8
    assert [(lib.name, lib.slot, lib.build_time, lib.signature) for lib in prog.libs] == [
        ("syslib", 2, 0x3B2A12B0, 0x0A0D4875), ("robotlib", 3, 0x3B2A12B3, 0x51B97334)]


def test_calls_are_found_in_the_code_only():
    assert crosprog.calls(PROG_01) == [(68, 0x4A)]


def test_stamp_is_a_librarys_build_time_and_signature():
    assert crosprog.stamp(ROBOT_99) == (0x377A899B, 0xAE952686)


def test_restamp_changes_only_the_build_dependent_fields_and_keeps_the_check_right():
    out = crosprog.restamp(PROG_01, {"syslib": SYS_99, "robotlib": ROBOT_99}, build_time=0x377A89A8)
    assert crosprog.check_ok(out) and len(out) == len(PROG_01)
    changed = {i for i in range(len(out)) if out[i] != PROG_01[i]}
    records = {lib.at for lib in crosprog.parse(PROG_01).libs}
    allowed = set(range(8, 12)) | set(range(56, 60))
    for at in records:
        allowed |= set(range(at + 4, at + 8)) | set(range(at + 12, at + 16))
    assert changed <= allowed
    prog = crosprog.parse(out)
    assert prog.build_time == 0x377A89A8
    assert [(lib.build_time, lib.signature) for lib in prog.libs] == [(0x377A8993, 0x0A0D4875), (0x377A899B, 0xAE952686)]


def test_restamp_there_and_back_is_the_identity():
    there = crosprog.restamp(PROG_01, {"syslib": SYS_99, "robotlib": ROBOT_99}, build_time=0x377A89A8)
    assert crosprog.restamp(there, {"syslib": SYS_01, "robotlib": ROBOT_01}, build_time=0x3B2A12B8) == PROG_01


def test_restamp_without_the_library_it_names_is_refused():
    with pytest.raises(crosprog.ProgramError, match="no robotlib"):
        crosprog.restamp(PROG_01, {"syslib": SYS_99})


@pytest.mark.parametrize("bad, why", [
    (PROG_01[:63], "at least 64"),
    (b"\0\0\0\0" + PROG_01[4:], "magic"),
])
def test_what_is_not_a_program_is_refused(bad, why):
    with pytest.raises(crosprog.ProgramError, match=why):
        crosprog.parse(bad)


real = pytest.mark.skipif(not (FILES / "bin" / "home").exists(), reason="sessions/controller-files is git-ignored and absent")


@real
@pytest.mark.parametrize("name", ["bin/home", "bin/calrdy", "bin/ash", "bin/pendant", "bin/shell", "bin_joint", "diag/cal", "diag/zero",
                                  "diag/xzero", "diag/calseek", "diag/setup", "sbin/robotsrv", "lib/robotlib.r", "lib/syslib.r"])
def test_the_check_holds_on_every_file_from_the_controller(name):
    assert crosprog.check_ok((FILES / name).read_bytes())


@real
def test_the_stamps_in_this_units_programs_are_its_librarys_build_time_and_signature():
    libs = {"syslib": (FILES / "lib" / "syslib.r").read_bytes(), "robotlib": (FILES / "lib" / "robotlib.r").read_bytes()}
    for name in ("bin/home", "diag/cal", "diag/zero", "sbin/robotsrv"):
        for lib in crosprog.parse((FILES / name).read_bytes()).libs:
            assert (lib.build_time, lib.signature) == crosprog.stamp(libs[lib.name])


@real
def test_home_and_cal_differ_in_only_a_call_number_and_the_build_time_and_check():
    home, cal = (FILES / "bin" / "home").read_bytes(), (FILES / "diag" / "cal").read_bytes()
    assert {i for i in range(len(home)) if home[i] != cal[i]} == {8, 56, 390, 442}
    assert crosprog.compute_check(cal) == crosprog.compute_check(home) - 2 & crosprog.MASK


def test_homezc_offset_program_reproduces_the_hand_patch():
    plain, offset = FILES / "patched" / "homezc-plain", FILES / "patched" / "homezc-offset"
    if not (plain.exists() and offset.exists()):
        pytest.skip("patched homezc files not present")
    made = crosprog.homezc_offset_program(plain.read_bytes())
    assert made == offset.read_bytes() and sum(made) & 0xFFFF == 0xA234


def test_homezc_offset_program_checks_the_words_it_relies_on():
    data = bytearray(640)
    data[:4] = crosprog.MAGIC
    for index, value in crosprog.HOMEZC_PLAIN_WORDS.items():
        struct.pack_into("<I", data, 0x40 + 4 * index, value)
    data[0x200:0x200 + 22] = crosprog.HOMEZC_OLD_TEXT
    made = crosprog.homezc_offset_program(bytes(data))
    assert len(made) == 640 and b"zero-cross offset: {}\n" in made and crosprog.HOMEZC_OLD_TEXT not in made
    assert struct.unpack_from("<I", made, 0x40 + 4 * 0x19)[0] == 0x00260044
    assert struct.unpack_from("<I", made, 0x40 + 4 * 0x5F)[0] == 0x001A0044
    bad = bytearray(data)
    struct.pack_into("<I", bad, 0x40 + 4 * 0x5E, 1)
    with pytest.raises(crosprog.ProgramError):
        crosprog.homezc_offset_program(bytes(bad))
    with pytest.raises(crosprog.ProgramError):
        crosprog.homezc_offset_program(bytes(data[:600]))
