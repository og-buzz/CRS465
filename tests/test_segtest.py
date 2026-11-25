"""a465/segtest.py against a fake controller: a `$` shell (cksum, w2) that answers `/tmp/segtest ...`
with scripted `I`/`M`/`P` lines from a simple motion model: the moves either run through (one speed per
half, no stop) or stop at every join; and the analysis on its own."""
import re

import pytest

from a465 import segtest
from fake_acid import FakeClock

CPD = segtest.COUNTS_PER_DEGREE
PROGRAM = b"segtest program bytes"


def profile(step, count, v1, v2, stop_ms=0.0, ramp_ms=0.0):
    """position(ms) in degrees from 0: moves of `step` degrees, the first (count+1)//2 at v1 deg/s, the
    rest at v2; stop_ms of standstill at each join (0: blended)."""
    half = (count + 1) // 2
    segs, t = [], 0.0
    for k in range(count):
        v = v1 if k < half else v2
        dur = abs(step) / v * 1000
        segs.append((t, t + dur, k * step, (k + 1) * step))
        t += dur + (stop_ms if k < count - 1 else 0)

    def pos(ms):
        for a, b, p0, p1 in segs:
            if ms < a:
                return p0
            if ms <= b:
                return p0 + (p1 - p0) * (ms - a) / (b - a)
        return segs[-1][3]
    return pos, t


class FakeController:
    def __init__(self, clock, kind="j", axis=6, step=2, count=6, v1=2.0, v2=6.0, stop_ms=0.0, has_file=True,
                 prompt=True, runaway=False, error=None, button=False, other=None):
        self.clock = clock
        self.files = {"/tmp/segtest": PROGRAM} if has_file else {}
        self.kind, self.axis, self.step, self.count = kind, axis, step, count
        self.v1, self.v2, self.stop_ms = v1, v2, stop_ms
        self.prompt, self.runaway, self.error, self.button = prompt, runaway, error, button
        self.other = other             # (joint, degrees per second): another joint drifts during the moves
        self.out, self.line, self.sent = [], b"", []
        self.joints = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        self.program = None            # pending scripted lines: [time, bytes]
        self.ctrlz = self.ctrle = 0
        self.closed = False
        self.stuck = False             # prompt=False: the console stays with the program until Ctrl-E

    def close(self):
        self.closed = True

    def _put(self, data, delay=0.0):
        self.out.append([self.clock() + delay, data])

    def write(self, data):
        self.sent.append(bytes(data))
        for b in data:
            if b == 0x1A:
                self.ctrlz += 1
                self.out = [e for e in self.out if not e[2:]]          # the program's pending lines are cut
                self._put(b"^C equivalent - halting robot...\r\n[301 killed by SIGABRT.]\r\n" + (b"$ " if self.prompt else b""), 0.3)
            elif b == 0x05:
                self.ctrle += 1
                self.stuck = False
                self._put(b"\r\n$ ")
            elif self.stuck:
                continue
            elif b == 0x0D:
                self._put(b"\r\n")
                line, self.line = self.line.decode(), b""
                self._shell(line.strip())
            else:
                self.line += bytes([b])
                self._put(bytes([b]))
        return len(data)

    def _shell(self, line):
        w = line.split()
        if not w:
            self._put(b"$ ")
        elif w[0] == "cksum":
            body = self.files.get(w[1])
            self._put((f"0x{sum(body):08x} {len(body)} {w[1]}\r\n" if body is not None else f"cksum: {w[1]}: file not found\r\n").encode() + b"$ ", 0.2)
        elif w[0] == "w2":
            j = self.joints
            self._put(("\r\nActual Position :\r\n          Axis 1/7   Axis 2/8     Axis 3     Axis 4     Axis 5     Axis 6\r\n"
                       f"PULSES  {' '.join(f'{round(x * CPD[i + 1]):+10d}' for i, x in enumerate(j))}\r\n          +0 +0\r\n"
                       f"JOINTS  {' '.join(f'{x:10.3f}' for x in j)}\r\n             0.000      0.000\r\n\r\nWORLD ...\r\n$ ").encode(), 0.2)
        elif w[0] == "/tmp/segtest":
            self._launch(w)
        else:
            self._put(f"{w[0]}: not found\r\n$ ".encode())

    def _launch(self, w):
        self.stuck = not self.prompt
        t0 = self.clock() + (2.0 if self.button else 0.0)
        if self.button:
            self._put(b"\r\nPress the PAUSE/CONTINUE button to gain robot point of control.\r\n")
        lines = []

        def say(ms, text):
            lines.append([t0 + ms / 1000 + 0.05, (text + "\r\n").encode(), "program"])
        step_deg = self.step / CPD[self.axis] if self.kind == "m" else float(self.step)
        pos, t_end = profile(step_deg * (30 if self.runaway else 1), self.count, self.v1, self.v2, self.stop_ms)
        say(0, f"I {self.kind} {self.axis} {self.step} {self.count} 5 15 0")
        calls = [(0, "ctl_get"), (95, "speed_get"), (190, "speed_set"), (285, "online")]
        for i, (ms, name) in enumerate(calls):
            say(ms + 95, f"M {i} {name} {ms} {ms + 95} 0")
        start = 400
        p0 = self.joints[self.axis - 1]
        if self.error:
            say(500, f"M 4 joint 400 495 {self.error}")
            say(500, f"E {self.error} I/O error")
            for i, name in enumerate(("halt", "online", "speed_set", "ctl_rel"), 5):
                say(600 + 95 * i, f"M {i} {name} {500 + 95 * i} {600 + 95 * i} 0")
            self._end(lines, t0, 1200)
            return
        for k in range(self.count):
            say(start + 95 * k + 95, f"M {4 + k} joint {start + 95 * k} {start + 95 * k + 95} 0")
        ms = 0
        while ms < t_end + 500:
            d = pos(ms)
            counts = [round(x * CPD[i + 1]) for i, x in enumerate(self.joints)]
            counts[self.axis - 1] = round((p0 + d) * CPD[self.axis])
            if self.other:
                j, rate = self.other
                counts[j - 1] = round((self.joints[j - 1] + rate * ms / 1000) * CPD[j])
            say(start + ms, "P " + " ".join(str(c) for c in [start + round(ms)] + counts))
            ms += 100
        self.joints[self.axis - 1] = p0 + pos(t_end)
        n = 4 + self.count
        end = start + t_end + 600
        for i, name in enumerate(("robotisfinished", "finish", "online", "speed_set", "ctl_rel")):
            say(end + 95 * i, f"M {n + i} {name} {end + 95 * (i - 1)} {end + 95 * i} {1 if name == 'robotisfinished' else 0}")
        say(end + 500, "done")
        self._end(lines, t0, end + 600)

    def _end(self, lines, t0, ms):
        if self.prompt:
            lines.append([t0 + ms / 1000 + 0.1, b"$ ", "program"])
        self.out += lines

    def read(self, count=1):
        self.out.sort(key=lambda e: e[0])
        data = b""
        while self.out and self.out[0][0] <= self.clock() and len(data) < count:
            data += self.out.pop(0)[1]
        return data


@pytest.fixture
def rig(tmp_path):
    clock = FakeClock()
    local = tmp_path / "segtest"
    local.write_bytes(PROGRAM)

    def go(*argv, upload=None, **kw):
        ctl = FakeController(clock, **kw)
        code = segtest.main(["--local", str(local), *argv], repo=str(tmp_path), open_port=lambda a: ctl,
                            clock=clock, sleep=clock.sleep, upload=upload, log_dir=str(tmp_path))
        go.text = "\n".join(p.read_text() for p in sorted(tmp_path.glob("*-segtest.log")))
        return code, ctl
    return go


def test_blended_run(rig):
    code, ctl = rig("j", "6", "2", "6", "5", "15")
    assert code == 0, rig.text
    assert b"/tmp/segtest j 6 2 6 5 15\r" in b"".join(ctl.sent)
    assert ctl.ctrlz == 0 and ctl.ctrle == 0
    assert "answer (1): 0 of 5 joins stopped, 5 ran through" in rig.text
    ratio = float(re.search(r"answer \(2\): the second speed is ([\d.]+) x the first", rig.text).group(1))
    assert ratio == pytest.approx(3.0, rel=0.05)
    assert "joint 6 moved +12.000 degrees by `w2`" in rig.text
    assert "    index  call" in rig.text and "joint" in rig.text and "move calls blocked: 95 ms" in rig.text


def test_clean_run_prints_the_five_other_joints_at_zero(rig):
    code, ctl = rig("j", "6", "2", "6", "5", "15")
    assert code == 0
    assert ("largest movement of the other joints during the run (degrees from their first P line): "
            "joint 1 0.000, joint 2 0.000, joint 3 0.000, joint 4 0.000, joint 5 0.000") in rig.text


@pytest.mark.parametrize("joint", [1, 5])
def test_another_joint_moving_writes_ctrl_z_at_once(rig, joint):
    code, ctl = rig("l", "6", "2", "6", "5", "15", kind="l", other=(joint, 2.0))
    assert code == 1 and ctl.ctrlz == 1
    assert f"OTHER JOINT MOVED: joint {joint} (not the commanded joint 6)" in rig.text
    assert "Ctrl-Z WRITTEN AT ONCE" in rig.text and "the controller is back at its `$` prompt" in rig.text
    after = rig.text.split("largest movement of the other joints")[1]
    moved = float(re.search(rf"joint {joint} ([\d.]+)", after).group(1))
    assert 1.0 < moved < 1.5                                           # stopped at the first sample over the limit


def test_max_other_option(rig):
    code, ctl = rig("l", "6", "2", "6", "5", "15", "--max-other", "3", kind="l", other=(2, 0.5))
    assert code == 0 and ctl.ctrlz == 0                                # 0.5 deg/s for about 5 s stays under 3
    assert "joint 2 2." in rig.text.split("largest movement of the other joints")[1]
    with pytest.raises(SystemExit):
        rig("l", "6", "2", "6", "5", "15", "--max-other", "0")


def test_stop_at_each_join(rig):
    code, ctl = rig("j", "6", "2", "6", "5", "15", stop_ms=400)
    assert code == 0, rig.text
    assert "answer (1): 5 of 5 joins stopped, 0 ran through" in rig.text
    assert rig.text.count(": STOP at the join") == 5


def test_motor_kind_measures_in_pulses(rig):
    code, ctl = rig("m", "6", "-280", "4", "5", "15", kind="m", step=-280, count=4)
    assert code == 0, rig.text
    assert "answer (1): 0 of 3 joins stopped" in rig.text


def test_travel_limit_writes_ctrl_z_at_once(rig):
    code, ctl = rig("j", "6", "2", "6", "5", "15", runaway=True, v1=40.0, v2=40.0)
    assert code == 1
    assert ctl.ctrlz == 1 and "TRAVEL LIMIT: joint 6" in rig.text and "Ctrl-Z WRITTEN AT ONCE" in rig.text
    assert "the controller is back at its `$` prompt" in rig.text


def test_missing_prompt_ctrl_z_then_ctrl_e(rig):
    code, ctl = rig("j", "6", "2", "2", "5", "15", count=2, prompt=False)
    assert code == 1
    assert ctl.ctrlz == 1 and ctl.ctrle == 1
    assert "NO `$` PROMPT" in rig.text and "Ctrl-E sent: YES" in rig.text


def test_an_e_line_is_a_failure(rig):
    code, ctl = rig("j", "6", "2", "6", "5", "15", error=-5)
    assert code == 1 and ctl.ctrlz == 0
    assert "E lines: E -5 I/O error" in rig.text


def test_button_request_is_waited_for_loudly(rig):
    code, ctl = rig("j", "6", "2", "6", "5", "15", button=True)
    assert code == 0, rig.text
    assert "WAITING: PRESS THE PAUSE/CONTINUE BUTTON" in rig.text


def test_upload_when_missing(rig):
    code, ctl = rig("j", "6", "2", "2", "5", "15", count=2, has_file=False,
                    upload=lambda link, args, say: link.port.port.files.__setitem__(args.remote, PROGRAM))
    assert code == 0, rig.text
    assert "Uploading it." in rig.text and "now matches" in rig.text


@pytest.mark.parametrize("argv", [
    ("j", "0", "2", "6", "5", "15"), ("j", "6", "4", "6", "5", "15"), ("m", "6", "600", "6", "5", "15"),
    ("j", "6", "0", "6", "5", "15"), ("j", "6", "2", "9", "5", "15"), ("j", "6", "2", "6", "30", "15"),
    ("x", "6", "2", "6", "5", "15"),
])
def test_limits_refused_before_the_port(rig, argv, tmp_path):
    opened = []
    with pytest.raises(SystemExit):
        segtest.main(["--local", str(tmp_path / "x"), *argv], repo=str(tmp_path), open_port=lambda a: opened.append(a))
    assert not opened


# ---- the analysis alone

def rows_of(pos, t_end, every=100.0):
    out, ms = [], 0.0
    while ms <= t_end + 300:
        out.append((round(ms), pos(ms)))
        ms += every
    return out


def test_joins_and_means_ran_through():
    pos, t = profile(2.0, 6, 2.0, 4.0)
    rows = rows_of(pos, t)
    found = segtest.joins(rows, 0.0, 2.0, 6)
    assert [f[5] for f in found] == ["ran through"] * 5
    first, second = segtest.mean_speeds(rows, 0.0, 2.0, 6, 3)
    assert first[2] == pytest.approx(2.0, rel=0.05) and second[2] == pytest.approx(4.0, rel=0.05)


def test_joins_stop():
    pos, t = profile(-2.0, 4, 3.0, 3.0, stop_ms=300)
    found = segtest.joins(rows_of(pos, t), 0.0, -2.0, 4)
    assert [f[5] for f in found] == ["STOP at the join"] * 3


def test_joins_not_reached():
    pos, t = profile(2.0, 2, 2.0, 2.0)
    found = segtest.joins(rows_of(pos, t)[:5], 0.0, 2.0, 3)        # stopped early
    assert found[-1][5] == "not reached"


def test_report_with_one_move():
    class L:
        def of(self, kind):
            return []
    out = segtest.report(L(), "j", 6, 2, 1, 5, 15)
    assert any("fewer than two P lines" in line for line in out)


# ---- the report from real runs (joint 6, 2026-11-25): P samples (ms, degrees from the first sample)
# copied from the SUMMARY blocks of the bench output

REAL_J = ((463, 0.0), (655, 0.107), (1103, 2.082), (1565, 4.113), (2049, 6.330), (2454, 8.105), (2901, 10.180),
          (2996, 11.156), (3203, 11.990), (3308, 12.026), (3586, 12.019))          # j 6 2 6 5 15: one P per move
REAL_J1 = ((464, 0.0), (656, -0.078), (831, -1.625), (1039, -3.358), (1142, -4.277), (1334, -5.945), (1439, -6.772),
           (1636, -8.512), (1741, -9.353), (1936, -11.042), (2125, -12.005), (2231, -12.047), (2512, -12.033))   # j 6 -12 1 5 5
REAL_C = ((578, 0.0), (3122, -2.025), (3330, -2.224), (3434, -2.431), (3632, -2.844), (3739, -3.129), (3936, -3.685),
          (4134, -4.234), (4240, -4.548), (4435, -5.133), (4541, -5.418), (4741, -5.981), (4937, -6.544), (5045, -6.858),
          (5242, -7.392), (5441, -7.948), (5548, -8.262), (5746, -8.832), (5942, -9.381), (6049, -9.688), (6247, -10.265),
          (6446, -10.821), (6555, -11.113), (6750, -11.491), (6855, -11.734), (7052, -11.976), (7155, -12.012),
          (7435, -12.012))                                                          # c 6 -2 6 5 15: a P every 0.1-0.2 s


class FakeLines:
    """What report() reads from Lines: of("P") and of("M") as regex matches."""

    def __init__(self, samples, start=20.0, axis=6):
        self.p = []
        for ms, d in samples:
            counts = [0] * 6
            counts[axis - 1] = round((start + d) * CPD[axis])
            self.p.append(segtest.LINE_PATTERNS[2][1].match(f"P {ms} " + " ".join(map(str, counts))))

    def of(self, kind):
        return self.p if kind == "P" else []


def run_report(samples, kind, step, count, speed1, speed2, cruise=None):
    return "\n".join(segtest.report(FakeLines(samples), kind, 6, step, count, speed1, speed2, cruise))


def test_one_p_per_move_is_not_judged_without_cruise():
    text = run_report(REAL_J, "j", 2, 6, 5, 15)
    assert "ran through" not in text.replace("0 ran through", "") and "blended" not in text
    assert text.count("cannot be judged from the samples") >= 5
    assert "answer (1): 0 of 5 joins stopped, 0 ran through, 5 cannot be judged from the samples" in text
    assert "scripts/a465-segtest j 6 12 1 5 5" in text and "--cruise" in text
    assert re.search(r"total: 2\.\d\d s from first movement .* mean speed 4\.[3-5]\d deg/s", text)


def test_one_p_per_move_with_cruise_is_slowed_or_stopped():
    text = run_report(REAL_J, "j", 2, 6, 5, 15, cruise=8.6)
    assert "slowed or stopped at the joins (mean 4.4" in text and "against cruise 8.60)" in text
    assert "(from mean speeds): ran through" not in text
    assert "scripts/a465-segtest" not in text


def test_one_p_per_move_running_at_cruise_ran_through():
    text = run_report(REAL_J, "j", 2, 6, 5, 15, cruise=3.5)
    assert "answer (1, from mean speeds): ran through" in text


def test_dense_samples_are_still_judged_from_samples():
    text = run_report(REAL_C, "c", -2, 6, 5, 15)
    assert re.search(r"answer \(1\): 0 of 5 joins stopped, [2-5] ran through", text)
    for k in (3, 4):
        assert re.search(rf"join {k} \(.*\) at \d+ ms: [\d.]+ deg/s through it.*: ran through", text)
    assert "blended" not in text


def test_one_move_prints_cruise_speed():
    text = run_report(REAL_J1, "j", -12, 1, 5, 5)
    m = re.search(r"cruise speed: ([\d.]+) deg/s \(median of the speeds of 10 samples.*paste into --cruise ([\d.]+)", text)
    assert m and float(m.group(1)) == pytest.approx(8.6, abs=0.3) and m.group(1) == m.group(2)
    assert "one move only: no join" in text
    assert "total: " in text


def test_one_move_with_few_samples_has_no_cruise():
    text = run_report(REAL_J1[:4], "j", -12, 1, 5, 5)
    assert "cruise speed: too few samples" in text
