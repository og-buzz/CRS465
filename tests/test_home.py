"""a465/home.py (scripts/a465-home, a465-joint, a465-xzero) against a fake `$` shell that models counts, the
`joint` and `/diag/xzero` programs and an index-pulse homing program that prints its offset."""
import struct
from pathlib import Path

import pytest

from a465 import calfile, console, home
from fake_acid import FakeClock
from test_homezc import BUTTON, PROGRAM, FakeShell

HOMES = (47145, 676, 16701, -24395, -3106, 25133, 0, 0)
CPD = home.COUNTS_PER_DEGREE
TURN = home.COUNTS_PER_TURN
COAST = 15
REAL_CAL = Path(__file__).resolve().parent.parent / "controller" / "backup-2026-11-23" / "conf" / "robot.cal"


def make_cal(homes=HOMES):
    return (bytes([2, 9, 2, 0]) + struct.pack("<8i", *homes)) + (bytes([2, 9, 4, 0]) + struct.pack("<8i", *([0] * 8)))


class HomeShell(FakeShell):
    """counts[j] is what the joint reads; err[j] = reading minus truth, which decides the offset the homing program prints."""

    def __init__(self, clock, start=None, err=None, homing="ok", scale=1.0, button_on=None, pulse_skew=0, **kw):
        super().__init__(clock, **kw)
        self.counts = list(start or [0] * 6)
        self.err = list(err or [0] * 6)
        self.homing, self.scale, self.button_on, self.pulse_skew = homing, scale, button_on, pulse_skew
        self.files["/tmp/homezo"] = PROGRAM
        self.files["/conf/robot.cal"] = make_cal()
        self.moves = []             # (joint, degrees) that were really carried out

    def _run(self, line):
        w = line.split()
        if w and w[0] == "w2":
            if self.clock() < self.reset_until:
                return super()._run(line)
            c = self.counts
            deg = [c[i] / CPD[i + 1] + 0.0 for i in range(6)]
            self._put((f"\r\nActual Position :\r\nPULSES  {' '.join(f'{x + (self.pulse_skew if self.moves else 0):+10d}' for x in c)}\r\n          +0 +0\r\n"
                       f"JOINTS  {' '.join(f'{x:10.3f}' for x in deg)}\r\n             0.000      0.000\r\n\r\nWORLD ...\r\n$ ").encode(), 0.2)
        elif w and w[0] in ("joint", "/tmp/homezo", "/diag/xzero") and self.button_on == w[0]:
            self._put(b"\r\n" + BUTTON.encode(), 0.5)
            self.running = (int(w[1]), "button")
        elif w and w[0] == "joint":
            j = int(w[1])
            n = round(float(w[2]) * CPD[j] * self.scale)
            self.counts[j - 1] += n
            self.moves.append((j, float(w[2])))
            self._put(b"$ ", 1.0)
        elif w and w[0] == "/diag/xzero":
            self.counts[int(w[1]) - 1] = 0
            self._put(b"$ ", 0.5)
        elif w and w[0] == "/tmp/homezo":
            j = int(w[1])
            if self.homing == "failed":
                self._put(f"Homing axis {j}...Failed.\r\n$ ".encode(), 2.0)
                return
            k = (HOMES[j - 1] + self.err[j - 1] - self.counts[j - 1]) % TURN[j]
            self.counts[j - 1] = HOMES[j - 1] + COAST
            self.err[j - 1] = 0
            self._put(f"Homing axis {j}...Ok.\r\n".encode() + (b"" if self.homing == "no-offset" else f"zero-cross offset: {k}\r\n".encode()) + b"$ ", 2.0)
        else:
            super()._run(line)


@pytest.fixture
def rig(tmp_path):
    clock = FakeClock()
    local = tmp_path / "homezo"
    local.write_bytes(PROGRAM)
    cal = tmp_path / "robot.cal"
    cal.write_bytes(make_cal())

    def go(*argv, shell=None, tool=home.main, **kw):
        shell = shell or HomeShell(clock, **kw)
        extra = ["--local", str(local), "--cal", str(cal)] if tool is home.main else []
        code = tool([*extra, *argv] if tool is home.main else list(argv), repo=str(tmp_path), open_port=lambda a: shell,
                    clock=clock, sleep=clock.sleep, log_dir=str(tmp_path))
        return code, shell
    go.clock, go.cal, go.tmp = clock, cal, tmp_path
    return go


def test_parse_pulses():
    text = "x\r\nPULSES          +0       +173     +24461     -24830         +0     +25151\r\n  +0  +0\r\nJOINTS 0 0\r\n"
    assert console.parse_pulses(text) == [0, 173, 24461, -24830, 0, 25151]
    assert console.parse_pulses("JOINTS 1 2 3 4 5 6") is None


def test_the_real_calibration_file_has_the_home_points_of_the_table():
    if not REAL_CAL.exists():
        pytest.skip("controller backup not present")
    data = REAL_CAL.read_bytes()
    assert calfile.parse(data).home_counts[:6] == HOMES[:6] and sum(data) == 0xE00
    for j, home_point in enumerate(HOMES[:6], 1):
        assert home.park_point(home_point, j) == (46645, 176, 16201, -24645, -3356, 24883)[j - 1]


def test_plan_prints_and_moves_nothing(rig, capsys):
    code, shell = rig("--plan", start=[0, 173, 24461, -24830, 0, 25151])
    out = capsys.readouterr().out
    assert code == 0 and out.splitlines()[0].startswith("PRECONDITION")
    assert shell.moves == [] and not any(s.startswith(("joint", "speed", "/tmp/homezo")) for s in shell.sent)
    assert "cksum /conf/robot.cal" in shell.sent and "46645" in out and "16201" in out and "-24645" in out
    assert "WARNING" not in out or "180" in out


def test_plan_warns_about_a_move_over_180_degrees(rig, capsys):
    code, shell = rig("--plan", "6", start=[0, 0, 0, 0, 0, -30000])
    assert "WARNING: joint 6's parking move" in capsys.readouterr().out and code == 0


def test_clean_run_of_all_six_joints(rig, capsys):
    code, shell = rig()
    out = capsys.readouterr().out
    assert code == 0 and "^Z" not in shell.sent
    homed = [s for s in shell.sent if s.startswith("/tmp/homezo")]
    assert homed == [f"/tmp/homezo {j}" for j in (2, 4, 5, 3, 1, 6)]
    assert "speed 10" in shell.sent and "speed 5" in shell.sent
    assert all(abs(c) < 0.5 * abs(CPD[i + 1]) * 0.01 + 5 for i, c in enumerate(shell.counts))   # back at zero within a few counts
    assert "all joints ran clean and centred" in out and "centred" in out.split("joint offset")[-1]
    # each joint was parked, homed, and returned before the next one started
    order = [m[0] for m in shell.moves]
    assert order == [2, 2, 4, 4, 5, 5, 3, 3, 1, 1, 6, 6]
    assert shell.moves[0] == (2, pytest.approx((176 - 0) / 277.78, abs=0.001))
    assert shell.moves[1][1] < 0       # joint 2 comes back (home 676 + coast = positive reading)


def test_stay_leaves_the_joint_at_the_home_point(rig):
    code, shell = rig("--stay", "4")
    assert code == 0 and shell.counts[3] == -24395 + COAST and len(shell.moves) == 1


def test_calibration_mismatch_stops_before_motion(rig, capsys):
    shell = HomeShell(rig.clock)
    shell.files["/conf/robot.cal"] = make_cal((1, 2, 3, 4, 5, 6, 0, 0))
    code, shell = rig(shell=shell)
    out = capsys.readouterr().out
    assert code == 2 and "STOPPED" in out and "robot.cal" in out
    assert not any(s.startswith(("joint", "speed", "/tmp/homezo")) for s in shell.sent)


def test_parking_move_that_falls_short_stops_everything(rig, capsys):
    code, shell = rig("2", "4", scale=0.8)          # the joint only covers 80% of the move
    out = capsys.readouterr().out
    assert code == 1 and "the parking move did not end cleanly" in out
    assert not any(s.startswith("/tmp/homezo") for s in shell.sent)


def test_parking_that_misses_by_more_than_40_counts_stops_everything(rig, capsys):
    code, shell = rig("2", "4", pulse_skew=60)      # the JOINTS row is right but the PULSES row is 60 counts off
    out = capsys.readouterr().out
    assert code == 1 and "not within 40 counts of the parking point" in out
    assert not any(s.startswith("/tmp/homezo") for s in shell.sent)


def test_homing_failed_stops_everything(rig, capsys):
    code, shell = rig("2", "4", homing="failed")
    out = capsys.readouterr().out
    assert code == 1 and "Failed." in out and "STOPPED" in out
    assert [s for s in shell.sent if s.startswith("/tmp/homezo")] == ["/tmp/homezo 2"]


def test_missing_offset_stops_everything(rig, capsys):
    code, shell = rig("2", homing="no-offset")
    assert code == 1 and "zero-cross offset" in capsys.readouterr().out


def test_edge_offset_gives_exit_code_3(rig, capsys):
    err = [0, 0, 0, 0, 187, 0]                       # joint 5 as measured: 437 of 500
    code, shell = rig("4", "5", "6", err=err)
    out = capsys.readouterr().out
    assert code == 3 and "NEAR THE EDGE" in out and "check this joint by eye" in out.lower() and "3.6 degrees" in out
    assert "437" in out and "near the edge" in out.split("verdict")[-1]
    assert "-1.35" in out
    assert [s for s in shell.sent if s.startswith("/tmp/homezo")] == ["/tmp/homezo 4", "/tmp/homezo 5", "/tmp/homezo 6"]


@pytest.mark.parametrize("which", ["joint", "/tmp/homezo"])
def test_button_request_at_a_motion_step_moves_nothing(rig, capsys, which):
    code, shell = rig("2", button_on=which)
    out = capsys.readouterr().out
    assert code == 1 and "^Z" not in shell.sent and "KILLED before anything moved" in out
    assert shell.sent[shell.sent.index(next(s for s in shell.sent if s.startswith(which))) + 1] == "^E"
    if which == "joint":
        assert shell.moves == [] and shell.counts == [0] * 6 and not any(s.startswith("/tmp/homezo") for s in shell.sent)
    else:
        assert len(shell.moves) == 1 and shell.counts[1] == 176       # parked, then the homing program never started


def test_move_above_max_move_is_refused(rig, capsys):
    code, shell = rig("1", "--max-move", "100")
    out = capsys.readouterr().out
    assert code == 1 and "more than --max-move 100" in out
    assert not any(s.startswith(("joint", "speed", "/tmp/homezo")) for s in shell.sent) and shell.counts == [0] * 6


def test_program_is_uploaded_when_different(rig):
    shell = HomeShell(rig.clock)
    shell.files["/tmp/homezo"] = b"old"
    done = []
    code = home.main(["--local", str(rig.tmp / "homezo"), "--cal", str(rig.cal), "2", "--stay"], repo=str(rig.tmp), open_port=lambda a: shell,
                     clock=rig.clock, sleep=rig.clock.sleep, log_dir=str(rig.tmp),
                     upload=lambda link, args, say: (done.append(args.remote), shell.files.__setitem__(args.remote, PROGRAM)))
    assert code == 0 and done == ["/tmp/homezo"]


def test_log_file_is_written(rig):
    rig("--plan")
    assert list(rig.tmp.glob("*-home.log"))


# ---- scripts/a465-joint and scripts/a465-xzero through the shared functions

def test_joint_script_moves_and_prints_the_summary(rig, capsys):
    code, shell = rig("2", "10", tool=home.joint_main, start=[0, 100, 0, 0, 0, 0])
    out = capsys.readouterr().out
    assert code == 0 and "joint 2 10.000" in shell.sent and "speed 5" in shell.sent
    assert "SUMMARY: joint 2 moved +10.00" in out and "halted by this tool: no" in out


def test_joint_script_refuses_over_max_before_the_port(rig):
    opened = []
    with pytest.raises(SystemExit):
        home.joint_main(["1", "40"], repo=".", open_port=lambda a: opened.append(a))
    assert not opened


def test_joint_script_button_at_the_motion_step(rig, capsys):
    code, shell = rig("1", "5", tool=home.joint_main, button_on="joint")
    assert code == 1 and "NOTHING MOVED" in capsys.readouterr().out and shell.counts == [0] * 6


def test_xzero_script(rig, capsys):
    code, shell = rig("3", tool=home.xzero_main, start=[0, 0, 5000, 0, 0, 0])
    out = capsys.readouterr().out
    assert code == 0 and "/diag/xzero 3" in shell.sent
    assert "SUMMARY: joint 3 read -18.000 and now reads 0.000; other joints changed by at most 0.000" in out
    assert shell.counts == [0] * 6
