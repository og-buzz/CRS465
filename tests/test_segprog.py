"""a465/segprog.py: the segtest bench program, run in a465/r3vm.py against a fake robot library.

The fake's pos_get returns a location built as robotsrv builds one (0x0264..0x0277): the 8 axis
words, flags 1, machine type, class 2 and the check byte. The committed build under
controller/programs/ must be what segprog builds now.
"""
import re
import struct

import pytest

from a465 import calfile, crosprog, r3asm, r3dis, r3vm, segprog

IMAGE = segprog.build()
NAMES = {v: k for k, v in segprog.ROUTINES.items()}
SAVED = 40
START = [1000, -2000, 3000, -4000, 5000, -6000, 0, 0]     # commanded and actual counts at the start
CPD100 = [round(c * 100) for c in calfile.COUNTS_PER_DEGREE]


def location(vm, ptr, counts):
    r3vm.store_location(vm, ptr, counts)
    vm.store(ptr, 0, 2 | 0x07 << 8 | 1 << 16)             # class 2, machine type 7, flags 1
    vm.b_loc_re_check([ptr])


class Robot(r3vm.FakeRobot):
    """Records calls with times; pos_get answers START (type 2) or START with joint 6 moving (type 1)."""

    def __init__(self, results=None, latency_ms=10, finished_after=3):
        def speed_get(vm, args):
            vm.store(args[0], 0, SAVED)
            return 0

        def pos_get(vm, args):
            counts = list(START)
            if args[0] == 1:
                counts[5] += vm.clock // 10
            location(vm, args[1], counts)
            return 0

        def move(vm, args):
            self.locations.append([s for s in args])
            return 0

        def cpath(vm, args):
            locs, start, finish, trig = args
            self.path = [[vm.load(locs, 9 * k + w) for w in range(9)] for k in range(start, finish + 1)]
            self.triggers = [vm.load(trig, k) for k in range(32)]
            return 0
        res = {0x43: speed_get, 0x0E: pos_get, 0x2C: move, 0x39: cpath, 0x3E: [0] * (finished_after - 1) + [1]}
        res.update(results or {})
        super().__init__(res, latency_ms=latency_ms)
        self.locations, self.path, self.triggers = [], None, None


class Run:
    def __init__(self, *argv, results=None, latency_ms=10, finished_after=3, image=IMAGE, until=60_000):
        self.robot = Robot(results, latency_ms, finished_after)
        self.syslib = r3vm.FakeSyslib({5: "I/O error", 13: "not the controlling client"})
        self.vm = r3vm.VM(image, argv=["/tmp/segtest", *argv], libs={"robotlib": self.robot, "syslib": self.syslib})
        self.code = self.vm.run(until_ms=until)
        self.calls = [(NAMES[n],) + tuple(a) for n, a in self.robot.calls]
        self.names = [c[0] for c in self.calls if not (c[0] == "pos_get" and c[1] == 1)]
        self.lines = self.vm.stdout().splitlines()

    def of(self, letter):
        return [ln.split() for ln in self.lines if ln.startswith(letter + " ")]


def checksum_ok(words):
    return sum(struct.pack("<9I", *words)) & 0xFF == 0


def float_of(word):
    return struct.unpack("<f", struct.pack("<I", word))[0]


START_SEQ = ["ctl_get", "speed_get", "speed_set"]
END_SEQ = ["robotisfinished"] * 3 + ["finish", "online", "speed_set", "ctl_rel"]


def test_built_files_are_current():
    out = segprog.OUT
    if not (out / "segtest").exists():
        pytest.skip("controller/programs/segtest has not been built")
    assert (out / "segtest").read_bytes() == IMAGE
    assert (out / "segtest.r3s").read_text() == segprog.source()
    assert (out / "segtest.lst").read_text() == segprog.listing(IMAGE)


def test_file_is_a_program_for_this_controller():
    assert IMAGE[:4] == bytes.fromhex("1fabad10") and crosprog.check_ok(IMAGE)
    stamps = [(lib.name, lib.slot, lib.build_time, lib.signature) for lib in crosprog.parse(IMAGE).libs]
    assert stamps == [("syslib", 2, 0x377A8993, 0x0A0D4875), ("robotlib", 3, 0x377A899B, 0xAE952686)]
    assert r3asm.assemble(r3asm.lift(IMAGE, "segtest")) == IMAGE
    assert len(IMAGE) < 6000


def test_every_function_frame_covers_its_locals():
    frame = None
    for ins in r3dis.decode(r3dis.Module(IMAGE))[1:]:
        if ins.op == 0x54 and ins.kind != "data":
            frame = -r3dis.s16(ins.word >> 16)
            continue
        for n in re.findall(r"\bL(\d+)", ins.text):
            assert 1 <= int(n) <= frame, (hex(ins.addr), ins.text)       # Ln..+8 reaches down to L(n-8)


def test_joint_kind_calls_in_order():
    r = Run("j", "6", "2", "6", "5", "15")
    assert r.code == 0 and r.lines[-1] == "done"
    assert r.names == START_SEQ + ["online"] + ["joint"] * 3 + ["speed_set"] + ["joint"] * 3 + END_SEQ
    joints = [c for c in r.calls if c[0] == "joint"]
    assert {(c[1], float_of(c[2])) for c in joints} == {(6, 2.0)}
    speeds = [c[1] for c in r.calls if c[0] == "speed_set"]
    assert speeds == [5, 15, SAVED]
    assert [c[1] for c in r.calls if c[0] == "online"] == [1, 0]
    assert r.lines[0] == "I j 6 2 6 5 15 0"


def test_motor_kind():
    r = Run("m", "6", "-280", "5", "3", "12")
    assert r.code == 0
    assert r.names == START_SEQ + ["online"] + ["motor"] * 3 + ["speed_set"] + ["motor"] * 2 + END_SEQ
    assert {c[1:] for c in r.calls if c[0] == "motor"} == {(6, -280, 0)}          # MSTOP_NONE


@pytest.mark.parametrize("axis,step", [(6, 2), (3, -3), (1, 1)])
def test_location_kind_targets(axis, step):
    r = Run("l", str(axis), str(step), "4", "5", "10")
    assert r.code == 0
    assert r.names == START_SEQ + ["pos_get", "online"] + ["move"] * 2 + ["speed_set"] + ["move"] * 2 + END_SEQ
    assert [c[1] for c in r.calls if c[0] == "pos_get"][0] == 2                     # the commanded position
    base = r.robot.locations[0]
    for k, loc in enumerate(r.robot.locations, 1):
        assert len(loc) == 9 and checksum_ok(loc)
        assert loc[0] & 0xFFFFFF == base[0] & 0xFFFFFF == 2 | 7 << 8 | 1 << 16       # class, machine type, flags kept
        want = list(START)
        want[axis - 1] += int(k * step * CPD100[axis - 1] / 100)                    # truncated, as RAPL divides
        assert [r3vm.s32(w) for w in loc[1:]] == want
    per_step = int(step * CPD100[axis - 1] / 100)
    assert r.lines[0] == f"I l {axis} {step} 4 5 10 {per_step}"


def test_path_kind_sends_one_cpath_with_the_same_points_and_zero_triggers():
    r = Run("c", "6", "2", "6", "5", "15")
    assert r.code == 0
    assert r.names == START_SEQ + ["pos_get", "online", "cpath", "speed_set"] + END_SEQ
    (_, locs, start, finish, trig), = [c for c in r.calls if c[0] == "cpath"]
    assert (start, finish) == (0, 5)
    assert r.robot.triggers == [0] * 32
    assert len(r.robot.path) == 6 and all(checksum_ok(loc) for loc in r.robot.path)
    assert [r3vm.s32(loc[6]) for loc in r.robot.path] == [START[5] + int(k * 2 * CPD100[5] / 100) for k in range(1, 7)]


@pytest.mark.parametrize("count,half", [(1, 1), (2, 1), (5, 3), (8, 4)])
def test_the_second_speed_is_set_after_the_first_half(count, half):
    r = Run("j", "6", "1", str(count), "5", "15")
    moves = [n for n in r.names if n in ("joint", "speed_set")][1:-1]
    assert moves == ["joint"] * half + ["speed_set"] + ["joint"] * (count - half)


def test_m_lines_time_every_call_and_p_lines_carry_positions():
    r = Run("j", "6", "2", "2", "5", "15", latency_ms=95)
    ms = r.of("M")
    assert [int(m[1]) for m in ms] == list(range(len(ms)))
    assert [m[2] for m in ms] == [n for n in r.names]
    assert all(int(m[4]) - int(m[3]) == 95 for m in ms)                      # each call's blocking time
    ps = r.of("P")
    assert ps and all(len(p) == 8 for p in ps)
    for p in ps:
        t = int(p[1])
        assert [int(x) for x in p[2:7]] == START[:5]
    times = [int(p[1]) for p in ps]
    assert times == sorted(times) and min(b - a for a, b in zip(times, times[1:])) >= 50


def test_p_lines_at_most_every_50_ms_when_calls_are_fast():
    r = Run("j", "6", "2", "2", "5", "15", latency_ms=0, finished_after=400)
    times = [int(p[1]) for p in r.of("P")]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert len(times) > 10 and min(gaps[:-1]) >= 50


def test_polling_ends_at_6_s_when_never_finished_and_finish_still_waits():
    r = Run("j", "6", "2", "2", "5", "15", latency_ms=10, finished_after=10_000)
    assert r.code == 0 and r.lines[-1] == "done"
    (finish,) = [m for m in r.of("M") if m[2] == "finish"]
    polls = [m for m in r.of("M") if m[2] == "robotisfinished"]
    assert 5900 <= int(finish[3]) - int(polls[0][3]) <= 6400
    assert all(m[5] == "0" for m in polls)


@pytest.mark.parametrize("argv", [
    (), ("j", "6", "2", "6", "5"), ("x", "6", "2", "6", "5", "15"), ("j", "0", "2", "6", "5", "15"),
    ("j", "7", "2", "6", "5", "15"), ("j", "6", "0", "6", "5", "15"), ("j", "6", "2", "0", "5", "15"),
    ("j", "6", "2", "9", "5", "15"), ("j", "6", "4", "6", "5", "15"), ("l", "6", "-3", "7", "5", "15"),
    ("m", "6", "501", "6", "5", "15"), ("j", "6", "2", "6", "0", "15"), ("j", "6", "2", "6", "5", "26"),
    ("j", "6", "x", "6", "5", "15"),
])
def test_limits_are_refused_before_any_call(argv):
    r = Run(*argv)
    assert r.code == 1 and r.calls == []
    assert r.vm.stderr() == ("/tmp/segtest: invalid command line arguments.\n"
                             "Usage: /tmp/segtest j|m|l|c axis step count speed1 speed2\n")


def test_limits_at_the_edge_are_allowed():
    assert Run("j", "6", "-20", "1", "25", "1").code == 0
    assert Run("m", "6", "375", "8", "1", "25").code == 0


def test_an_error_after_the_start_halts_and_restores():
    r = Run("l", "6", "2", "6", "5", "15", results={0x2C: [0, -5, 0]})
    assert r.code == 1
    assert r.names == START_SEQ + ["pos_get", "online", "move", "move", "halt", "online", "speed_set", "ctl_rel"]
    assert [c[1] for c in r.calls if c[0] == "online"] == [1, 0]
    assert [c[1] for c in r.calls if c[0] == "speed_set"] == [5, SAVED]
    assert "E -5 I/O error" in r.lines and "done" not in r.lines


def test_no_control_no_halt():
    r = Run("j", "6", "2", "6", "5", "15", results={0x02: -13})
    assert r.code == 1 and r.names == ["ctl_get"]
    assert r.lines[-1] == "E -13 not the controlling client"


def test_error_before_online_does_not_turn_it_off():
    r = Run("j", "6", "2", "6", "5", "15", results={0x44: -22})
    assert r.code == 1
    assert r.names == ["ctl_get", "speed_get", "speed_set", "halt", "speed_set", "ctl_rel"]


def test_a_failing_position_read_is_an_error_too():
    r = Run("j", "6", "2", "6", "5", "15", results={0x0E: -5})
    assert r.code == 1
    assert [m for m in r.of("M") if m[2] == "pos_get"][0][5] == "-5"
    assert r.names[-4:] == ["halt", "online", "speed_set", "ctl_rel"]
