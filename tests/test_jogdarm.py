"""JogdArm against a protocol-level fake of the controller's `$` shell and of jogd, with a fake clock."""
import re

import pytest

from a465 import jogd
from a465.jogd import JogdArm, command_bytes, speed_for
from a465.sim import RobotError

PROGRAM = b"fake jogd program bytes"
CPD = (277.78, 277.78, -277.78, -140.28, -138.89, -140.28)
BUTTON = b"Press the PAUSE/CONTINUE button to gain robot point of control.\r\n"
CTRL_Z, CTRL_E = b"\x1a", b"\x05"


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class FakeController:
    """The serial port. Reads return what is due by the clock. Plays `$` and jogd (see a465/jogd.py for the protocol)."""

    def __init__(self, clock, button_after=None, ready_after=1.1, has_p=True, echo=False, q_works=True, ctrlz_kills=True, ignore_stop=False,
                 errors=(), cksum_ok=True):
        self.clock = clock
        self.out = []                    # (due, bytes)
        self.writes = []                 # (time, bytes)
        self.mode = "shell"
        self.line = b""
        self.button_after, self.ready_after, self.has_p, self.echo = button_after, ready_after, has_p, echo
        self.q_works, self.ctrlz_kills, self.errors, self.cksum_ok = q_works, ctrlz_kills, set(errors), cksum_ok
        self.ignore_stop = ignore_stop    # `.` is not answered (no `-`)
        self.need_button = False          # a killed program leaves control held
        self.mute_shell = False           # the shell does not answer
        self.speed_changes = []           # when a command with another speed reached a moving joint
        self.deaf_until = 0.0
        self.no_restart = False           # ... and jogd printed `-` but never started again
        self.joints = [10.0, -20.0, 30.0, 40.0, -50.0, 60.0]
        self.counts = [round(a * k) for a, k in zip(self.joints, CPD)]
        self.move = None                 # (axis, sign, speed)
        self.last_cmd = 0.0
        self.closed = False
        self.ready = False
        self.stuck = False               # jogd ignores everything (for no-ready tests)

    # ---- the port
    def put(self, data, after=0.0):
        self.out.append((self.clock() + after, data))

    def inject(self, data):
        self.put(data)

    def read(self, count=1):
        self._deadman()
        now = self.clock()
        due = [o for o in self.out if o[0] <= now]
        if not due:
            return b""
        self.out = [o for o in self.out if o[0] > now]
        return b"".join(d for _, d in sorted(due, key=lambda o: o[0]))

    def flush(self):
        pass

    def close(self):
        self.closed = True

    def write(self, data):
        self.writes.append((self.clock(), bytes(data)))
        for b in bytes(data):
            self._byte(bytes([b]))

    # ---- behaviour
    def _deadman(self):
        if self.mode == "jogd" and self.move and not self.ignore_stop and self.clock() - self.last_cmd > 0.3:
            self.move = None
            self.put(b"T\r\n")

    def _byte(self, b):
        if self.mode == "shell":
            if b == b"\r":
                self._shell(self.line.decode())
                self.line = b""
            elif b in (CTRL_E, CTRL_Z):
                pass
            else:
                self.line += b
            return
        if b == CTRL_Z and self.ctrlz_kills:
            self.kill()
            return
        if self.stuck:
            if b == CTRL_E:
                self.mode = "shell"
                self.put(b"\r\n$ ", 0.05)
            elif b == b"q" and self.q_works:
                self.mode = "shell"
                self.put(b"q\r\n$ ", 0.05)
            return
        if b == CTRL_Z:
            self.move = None
            return
        if b == CTRL_E:
            self.mode = "shell"
            self.put(b"\r\n$ ", 0.05)
            return
        if self.echo:
            self.put(b)
        if b in (b"q",):
            if self.q_works:
                self.move = None
                self.mode = "shell"
                self.put(b"\r\n$ ", 0.05)
            else:
                self.stuck = True
        elif b == b".":
            if self.move and not self.ignore_stop:
                self.move = None
                self.put(b"-\r\n", 0.2)
        elif b == b"p":
            if self.has_p:
                self.put(("P " + " ".join(str(c) for c in self.counts) + "\r\n").encode(), 0.02)
        else:
            self.line += b
            if re.fullmatch(rb"[A-Fa-f]\d\d", self.line):
                letter, speed = chr(self.line[0]), int(self.line[1:])
                self.line = b""
                self._command(letter, speed)
            elif len(self.line) >= 3:
                self.line = b""

    def kill(self):
        """What Ctrl-Z does on the real controller: stops the arm and kills the program that holds control."""
        self.move = None
        self.ignore_stop = False          # a restarted jogd answers again
        self.mode = "shell"
        self.line = b""
        self.need_button = True
        self.killed_at = self.clock()
        self.put(b"^C equivalent - halting robot...\r\n[301 killed by SIGABRT.]\r\n$ ", 0.05)

    def _command(self, letter, speed):
        axis, sign = "abcdef".index(letter.lower()), 1 if letter.isupper() else -1
        self.last_cmd = self.clock()
        if letter in self.errors:
            self.put(b"E 5 joint at its limit\r\n", 0.1)
            return
        new = (axis, sign, speed)
        if self.clock() < self.deaf_until:
            return
        if self.move == new:
            return
        if self.move and self.move[:2] == new[:2]:        # variant `s`: another speed = stop, then start again
            self.move = new
            self.speed_changes.append(self.clock())
            if self.no_restart:
                self.move = None
                self.deaf_until = self.clock() + 1.4
                self.put(b"-\r\n", 0.2)
            else:
                self.put(b"-\r\n", 0.2)
                self.put(f"+{letter}{speed:02d}\r\n".encode(), 0.5)
            return
        self.move = new
        self.put(f"+{letter}{speed:02d}\r\n".encode(), 0.28)

    def die(self, text):
        """jogd is gone (killed by something else): the controller printed `text` and is at the shell."""
        self.move = None
        self.mode = "shell"
        self.line = b""
        self.need_button = True
        self.put(text)

    def _shell(self, text):
        if self.mute_shell:
            return
        if text == "":
            self.put(b"\r\n$ ", 0.01)
        elif text.startswith("cksum"):
            bad = 1 if not self.cksum_ok else 0
            self.put(f"{text}\r\n0x{sum(PROGRAM) + bad:x} {len(PROGRAM)} /tmp/jogd\r\n$ ".encode(), 0.05)
        elif text == "w2":
            counts = "".join(f"{c:+11d}" for c in self.counts)
            joints = "".join(f"{a:11.3f}" for a in self.joints)
            self.put(f"w2\r\nPULSES{counts}\r\n   +0 +0\r\nJOINTS{joints}\r\n  0.000 0.000\r\n$ ".encode(), 0.05)
        elif text.startswith("/tmp/jogd"):
            self.put(text.encode() + b"\r\n")
            self.mode = "jogd"
            self.line = b""
            button = self.button_after
            if self.need_button and button is None:
                button = 3.0
            if button is not None:
                self.put(BUTTON)
                self.need_button = False
            if self.ready_after is not None:
                self.put(b"jogd ready\r\n", (button or 0.0) + self.ready_after)
                self.ready = True
        else:
            self.put(f"{text}: not found\r\n$ ".encode(), 0.01)

    # ---- what tests look at
    def sent(self, since=0.0):
        return [(t, d) for t, d in self.writes if t >= since]

    def moves(self, since=0.0):
        return [(t, d) for t, d in self.sent(since) if re.fullmatch(rb"[A-Fa-f]\d\d", d)]


@pytest.fixture
def rig(tmp_path):
    local = tmp_path / "jogd"
    local.write_bytes(PROGRAM)

    class Rig:
        pass

    r = Rig()
    r.clock, r.said, r.tmp = Clock(), [], tmp_path

    def build(**kw):
        opts = {k: kw.pop(k) for k in list(kw) if k in ("button_after", "ready_after", "has_p", "echo", "q_works",
                                                        "ctrlz_kills", "ignore_stop", "errors", "cksum_ok")}
        r.port = FakeController(r.clock, **opts)
        r.arm = JogdArm(serial_port=r.port, say=r.said.append, log_dir=str(tmp_path), clock=r.clock, sleep=r.clock.sleep,
                        threaded=False, local=str(local), **kw)
        r.arm.speed = 25
        return r.arm

    def run(seconds, vec=None, every=0.02):
        """Pump the thread's step every 2 ms; call jog with `vec` every `every` seconds (None: no pad calls)."""
        end, next_jog = r.clock() + seconds, r.clock()
        while r.clock() < end:
            if vec is not None and r.clock() >= next_jog:
                r.arm.jog("joint", vec, ttl=0.25, source="gamepad")
                next_jog += every
            assert r.arm.step()
            r.clock.sleep(0.002)

    r.build, r.run = build, run
    return r


def said(rig, text):
    return [s for s in rig.said if text in s]


V1 = [0.6, 0, 0, 0, 0, 0]


def test_speed_curve():
    assert speed_for(0.2, 25) == 1                         # at the threshold
    assert speed_for(0.6, 25) == 9                         # middle of the span 0.2 .. 1: fine control below the linear 13
    assert speed_for(1.0, 25) == 25                        # full deflection: the pad's speed setting
    assert speed_for(1.0, 12) == 12 and speed_for(0.0, 25) == 1
    values = [speed_for(0.2 + 0.8 * i / 20, 25) for i in range(21)]
    assert values == sorted(values) and values[3] <= 3


def test_start_and_ready(rig):
    arm = rig.build()
    port = rig.port
    assert [d for _, d in port.writes if d.startswith(b"/tmp/jogd")] == [b"/tmp/jogd 1 25 s\r"]
    assert arm.joints == port.joints and port.mode == "jogd"
    assert not port.moves()
    log = list(rig.tmp.glob("*-jogd.log"))
    assert len(log) == 1 and "jogd ready" in log[0].read_text() and "->" in log[0].read_text()
    assert arm.state()["command"] is None


def test_max_speed_is_passed_as_the_clamp(rig):
    rig.build(max_speed=40)
    assert [d for _, d in rig.port.writes if d.startswith(b"/tmp/jogd")] == [b"/tmp/jogd 1 40 s\r"]


def test_upload_when_the_program_differs(rig):
    ups = []
    port = FakeController(rig.clock, cksum_ok=False)
    JogdArm(serial_port=port, say=rig.said.append, log_dir=str(rig.tmp), clock=rig.clock, sleep=rig.clock.sleep, threaded=False,
            local=str(rig.tmp / "jogd"), upload=lambda link, say: (ups.append(1), setattr(port, "cksum_ok", True)))
    assert ups == [1] and port.mode == "jogd"


def test_start_with_the_button_request(rig):
    rig.build(button_after=6.0)
    assert said(rig, "WAITING: PRESS THE PAUSE/CONTINUE BUTTON")
    assert rig.port.mode == "jogd" and not rig.port.moves()


def test_ready_never_comes_leaves_the_prompt(rig):
    with pytest.raises(RobotError):
        rig.build(ready_after=None)
    assert said(rig, "no `jogd ready` within 10 s")
    sent = [d for _, d in rig.port.writes]
    assert b"q" in sent and CTRL_E not in sent and CTRL_Z not in sent
    assert rig.port.mode == "shell" and rig.port.closed


def test_button_never_pressed_sends_only_ctrl_e(rig):
    with pytest.raises(RobotError) as err:
        rig.build(ready_after=None, button_after=0.0)
    assert "button" in str(err.value)
    sent = [d for _, d in rig.port.writes]
    assert CTRL_E in sent and CTRL_Z not in sent and b"q" not in sent and rig.port.closed
    assert rig.port.sent()[-1][0] - rig.port.writes[0][0] >= 180


def test_not_at_the_prompt_refuses(rig):
    class Mute(FakeController):
        def _shell(self, text):
            pass
    port = Mute(rig.clock)
    with pytest.raises(RobotError):
        JogdArm(serial_port=port, say=rig.said.append, log_dir=str(rig.tmp), clock=rig.clock, sleep=rig.clock.sleep,
                threaded=False, local=str(rig.tmp / "jogd"))
    assert port.closed and not port.moves()


def test_deflection_to_speed_on_the_wire(rig):
    rig.build()
    for vec, text in (([0.2, 0, 0, 0, 0, 0], b"A01"), ([0.6, 0, 0, 0, 0, 0], b"A09"), ([1.0, 0, 0, 0, 0, 0], b"A25")):
        rig.run(0.5, vec)
        rig.run(0.4, [0] * 6)
        assert rig.port.moves(rig.clock() - 0.9)[0][1] == text
    rig.arm.speed = 12                                      # the pad's speed setting is full deflection
    rig.run(0.4, [-1.0, 0, 0, 0, 0, 0])
    assert rig.port.moves(rig.clock() - 0.4)[0][1] == b"a12"


def test_picks_joint_and_direction_like_realarm(rig):
    rig.build()
    rig.run(0.3, [0.1, 0.19, 0, 0, 0, 0])                  # below the threshold: nothing
    assert not rig.port.moves()
    rig.run(0.3, [0.25, 0, 0, 0, -0.5, 0.3])               # the biggest one, J5 negative
    assert rig.port.moves()[0][1].startswith(b"e")
    rig.run(0.3, [0.25, 0, 0, 0, -0.15, 0.9])              # J5 still pushed above STICK_OFF: it keeps the joint
    assert {d[:1] for _, d in rig.port.moves()} == {b"e"}


def test_a_new_joint_must_settle(rig):
    rig.build()
    rig.arm.jog("joint", V1, ttl=0.25)
    rig.run(0.03)
    assert not rig.port.moves()                             # SETTLE_S has not passed


def stick(rig, values, every=0.02):
    """One jog call per `every` seconds with joint 1 at each deflection in turn."""
    for v in values:
        rig.run(every, [v, 0, 0, 0, 0, 0], every=every)


def test_speed_is_the_peak_of_the_first_120_ms(rig):
    rig.build()
    stick(rig, [0.3, 0.5, 0.9, 0.7, 0.5, 0.4])               # 120 ms of a stick on its way up and back
    stick(rig, [0.4] * 20)
    assert not rig.port.moves(rig.clock() + 1) and rig.port.moves()
    assert rig.port.moves()[0][1] == command_bytes(0, 1, speed_for(0.9, 25))
    assert {d for _, d in rig.port.moves()} == {command_bytes(0, 1, speed_for(0.9, 25))}      # one command, kept alive


def test_nothing_starts_before_120_ms(rig):
    rig.build()
    stick(rig, [0.6] * 5)                                    # 100 ms
    assert not rig.port.moves()
    stick(rig, [0.6] * 3)
    assert rig.port.moves()


def test_no_command_stream_while_the_stick_wanders_within_the_factor(rig):
    rig.build()
    rig.run(0.5, V1)                                         # speed 9
    for _ in range(4):
        stick(rig, [0.5] * 25 + [0.7] * 25)                  # wanders between 7 and 13: under a factor of 1.6
    assert {d for _, d in rig.port.moves()} == {b"A09"}
    assert b"." not in [d for _, d in rig.port.sent()] and not rig.port.speed_changes


def test_a_sustained_big_change_gives_exactly_one_new_command(rig):
    rig.build()
    rig.run(1.2, V1)                                         # A09 since 0.12 s
    start = rig.port.moves()[0][0]
    t = rig.clock()
    rig.run(3.0, [1.0, 0, 0, 0, 0, 0])
    commands = [(tt, d) for tt, d in rig.port.moves() if d != b"A09"]
    assert {d for _, d in commands} == {b"A25"}
    first = commands[0][0]
    assert first - t >= 0.4 - 1e-9 and first - start >= 1.0 - 1e-9 and first - t < 0.5     # after 0.4 s of difference
    assert rig.port.speed_changes == [first]
    assert all(d == b"A25" for tt, d in rig.port.moves() if tt >= first)
    assert not said(rig, "stopped at")                       # the `-` then `+` of the change is not the end of the jog
    assert rig.arm.state()["moving"] and b"." not in [d for _, d in rig.port.sent()]
    assert said(rig, "9% -> 25%")


def test_a_short_spike_changes_nothing(rig):
    rig.build()
    rig.run(1.5, V1)
    rig.run(0.3, [1.0, 0, 0, 0, 0, 0])
    rig.run(1.5, V1)
    assert {d for _, d in rig.port.moves()} == {b"A09"}


def test_a_big_change_down_works_too(rig):
    rig.build()
    rig.run(1.2, [1.0, 0, 0, 0, 0, 0])
    rig.run(2.0, [0.4, 0, 0, 0, 0, 0])
    assert [d for _, d in rig.port.moves()][-1] == command_bytes(0, 1, speed_for(0.4, 25)) and len(rig.port.speed_changes) == 1


def test_no_respeed_keeps_the_speed_until_release(rig):
    rig.build(respeed=False)
    rig.run(1.2, V1)
    rig.run(3.0, [1.0, 0, 0, 0, 0, 0])
    assert {d for _, d in rig.port.moves()} == {b"A09"} and not rig.port.speed_changes
    rig.run(0.5, [0] * 6)
    rig.run(1.0, [1.0, 0, 0, 0, 0, 0])
    assert rig.port.moves()[-1][1] == b"A25"                 # a new push picks a new speed


def test_a_missing_plus_after_a_speed_change_is_taken_as_stopped(rig):
    rig.build()
    rig.port.no_restart = True
    rig.run(1.2, V1)
    t = rig.clock()
    rig.run(2.5, [1.0, 0, 0, 0, 0, 0])
    assert said(rig, "no `+` within 1 s of the speed change")
    assert said(rig, "stopped at") and rig.arm.state()["moving"]          # and it started again with the stick still held
    assert rig.port.moves()[-1][1] == b"A25" and b"." not in [d for _, d in rig.port.sent(t)]


def test_wait_for_the_first_plus_is_a_second(rig):
    rig.build()
    rig.run(0.2, V1)
    assert rig.arm._await_since is not None
    rig.run(0.4, V1)                                         # `+` came after 0.28 s
    assert rig.arm._await_since is None and rig.arm.state()["moving"]


def test_keepalives_every_100ms(rig):
    rig.build()
    rig.run(2.0, V1)
    times = [t for t, d in rig.port.moves()]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert len(times) >= 18 and max(gaps) <= 0.151 and min(gaps) >= 0.099
    assert {d for _, d in rig.port.moves()} == {b"A09"}
    assert rig.arm.state()["moving"] and rig.arm.state()["command"] == {"kind": "jog", "source": "gamepad"}


def test_echo_does_not_confuse_the_lines(rig):
    rig.build(echo=True)
    rig.run(1.0, V1)
    assert rig.arm.state()["moving"] and not rig.arm.notice
    rig.run(0.5, [0] * 6)
    assert not rig.arm.state()["moving"]


def test_release_writes_stop_at_once(rig):
    rig.build()
    rig.run(0.6, V1)
    t = rig.clock()
    rig.run(0.0, V1)
    rig.arm.jog("joint", [0] * 6, ttl=0.25)
    rig.run(0.01)
    stops = [(tt, d) for tt, d in rig.port.sent(t) if d == b"."]
    assert len(stops) == 1 and stops[0][0] - t < 0.02
    rig.run(0.5)
    assert not rig.arm.state()["moving"] and rig.arm.state()["command"] is None
    assert [d for _, d in rig.port.sent(t) if d == b"."] == [b"."]         # and only the one


def test_stopped_at_line_with_live_position(rig):
    rig.build()
    rig.port.counts = [1000, 2000, -3000, -4000, -5000, -6000]
    rig.run(0.6, V1)
    rig.run(1.0, [0] * 6)
    assert said(rig, "stopped at")
    line = said(rig, "stopped at")[-1]
    assert f"J1 {1000 / CPD[0]:8.2f}" in line and f"J3 {-3000 / CPD[2]:8.2f}" in line
    assert rig.arm.joints == pytest.approx([c / k for c, k in zip(rig.port.counts, CPD)])


def test_change_of_joint_stops_first(rig):
    rig.build()
    rig.run(0.6, V1)
    t = rig.clock()
    rig.run(0.6, [0, 0.6, 0, 0, 0, 0])
    seq = [d for _, d in rig.port.sent(t) if d in (b".",) or re.fullmatch(rb"[A-Fa-f]\d\d", d)]
    first_b = seq.index(b"B09")
    assert seq[first_b - 1] == b"." and b"A09" not in seq[first_b - 1:first_b + 2] and seq.count(b".") == 1
    stop_t = [tt for tt, d in rig.port.sent(t) if d == b"."][0]
    b_t = [tt for tt, d in rig.port.sent(t) if d == b"B09"][0]
    assert b_t - stop_t >= 0.1                              # it waited for jogd's `-`


def test_stale_input_stops(rig):
    rig.build()
    rig.run(0.6, V1)
    t = rig.clock()
    rig.run(0.6)                                            # the pad has gone silent
    stop = [tt for tt, d in rig.port.sent(t) if d == b"."]
    assert len(stop) == 1 and stop[0] - t < 0.26 + 0.1
    assert not [1 for tt, d in rig.port.moves(stop[0]) if tt > stop[0]]


def test_halt_writes_only_the_stop(rig):
    rig.build()
    rig.run(0.6, V1)
    t = rig.clock()
    rig.arm.halt()
    rig.run(0.01, V1)
    sent = rig.port.sent(t)
    assert [d for _, d in sent][0] == b"."
    rig.run(1.0, V1)                                        # the stick is still held
    assert CTRL_Z not in [d for _, d in rig.port.writes]
    assert [d for _, d in rig.port.sent(t) if d == b"."] == [b"."]
    moves = rig.port.moves(t)
    assert moves and 0.09 <= moves[0][0] - t < 0.4          # restarted after jogd's `-`, no 2.5 s hold-off
    assert rig.arm.state()["moving"] and not rig.arm._dead


def test_halt_with_nothing_moving_writes_nothing(rig):
    rig.build()
    t = rig.clock()
    rig.arm.halt()
    rig.run(0.1)
    assert [d for _, d in rig.port.sent(t) if d in (b".", CTRL_Z)] == []


def test_missing_dash_leads_to_ctrl_z_and_the_dead_state(rig):
    rig.build(ignore_stop=True)
    rig.arm._attempts = jogd.MAX_RESTARTS                   # so that it is left dead for the checks below
    rig.run(0.6, V1)
    t = rig.clock()
    rig.arm.halt()
    rig.run(0.7)
    dot = [tt for tt, d in rig.port.sent(t) if d == b"."]
    z = [tt for tt, d in rig.port.sent(t) if d == CTRL_Z]
    assert len(dot) == 1 and len(z) == 1 and 0.6 <= z[0] - dot[0] < 0.62
    assert said(rig, "NO `-`, `T` OR `E` WITHIN 0.6 s")
    assert rig.arm.notice.startswith("jogd is not running")
    assert rig.arm.state()["command"] is None and not rig.arm.state()["moving"]
    quiet = [d for tt, d in rig.port.sent(z[0] + 0.001) if tt < z[0] + 2.5]
    assert quiet == []                                      # nothing at all while the controller refuses


def test_same_rule_for_a_release(rig):
    rig.build(ignore_stop=True)
    rig.run(0.6, V1)
    rig.run(1.0, [0] * 6)
    assert CTRL_Z in [d for _, d in rig.port.writes] and said(rig, "Ctrl-Z WRITTEN")


def test_restart_after_ctrl_z_with_the_button(rig):
    rig.build(ignore_stop=True)
    rig.run(0.6, V1)
    rig.arm.halt()
    rig.run(0.7)
    z = [tt for tt, d in rig.port.writes if d == CTRL_Z][0]
    rig.run(8.0, V1)
    seq = [(tt, d) for tt, d in rig.port.sent(z) if d != b"p"]
    assert seq[1][1] == b"\r" and seq[1][0] - z >= 2.5 and seq[2][1] == b"/tmp/jogd 1 25 s\r"
    assert said(rig, "WAITING: PRESS THE PAUSE/CONTINUE BUTTON") and said(rig, "jogd is running again")
    assert not rig.arm._dead and rig.arm.notice == ""
    assert rig.arm.state()["moving"] and rig.port.moves(seq[2][0])         # jogging again, stick still held
    assert not [d for tt, d in rig.port.sent(z) if tt < seq[2][0] and re.fullmatch(rb"[A-Fa-f]\d\d|\.", d)][1:]


@pytest.mark.parametrize("text", [b"[301 killed by SIGABRT.]\r\n", b"^C equivalent - halting robot...\r\n", b"\r\n$ "])
def test_death_is_noticed_from_each_sign(rig, text):
    rig.build()
    rig.run(0.6, V1)
    t = rig.clock()
    rig.port.die(text)
    rig.arm.step()
    rig.arm.step()
    assert said(rig, "jogd is running again")             # and it restarted by itself
    assert said(rig, "JOGD IS NOT RUNNING")
    first = [d for _, d in rig.port.sent(t)][0]
    assert first == b"\r"                                   # no command, keep-alive or `p` after the death: only the restart's `\r`


def test_nothing_is_written_while_dead_and_the_stick_is_ignored(rig):
    rig.build()
    rig.run(0.6, V1)
    rig.arm._attempts = jogd.MAX_RESTARTS                   # no restart, so it stays dead
    rig.port.die(b"[301 killed by SIGABRT.]\r\n$ ")
    t = rig.clock()
    rig.run(3.0, V1)
    assert rig.arm._dead and rig.port.sent(t) == []
    assert said(rig, "giving up")
    assert rig.arm.state()["command"] is None and rig.arm.notice


def test_restart_that_cannot_get_a_prompt(rig):
    rig.build()
    rig.run(0.6, V1)
    rig.port.mute_shell = True
    rig.port.die(b"[301 killed by SIGABRT.]\r\n")
    rig.run(0.2, V1)
    assert rig.arm._dead and said(rig, "RESTART FAILED: no `$` prompt")
    t = rig.clock()
    rig.run(1.0, V1)                                        # the stick is held: still one try only per push
    assert rig.port.sent(t) == [] and rig.arm._attempts == 1


def test_restart_failure_streak_stops_after_three_tries(rig):
    rig.build()
    rig.run(0.6, V1)
    rig.port.ready_after = None                             # jogd never says it is ready again
    rig.port.die(b"[301 killed by SIGABRT.]\r\n$ ")
    rig.run(0.2, V1)
    assert rig.arm._attempts == 1 and said(rig, "RESTART FAILED: jogd did not come up")
    t = rig.clock()
    rig.run(0.5, V1)
    assert rig.port.sent(t) == []                           # the same push does not try again
    for n in (2, 3):
        rig.run(0.3, [0] * 6)
        rig.run(0.3, V1)
        assert rig.arm._attempts == n
    rig.run(0.3, [0] * 6)
    t = rig.clock()
    rig.run(0.5, V1)
    assert rig.arm._attempts == 3 and rig.port.sent(t) == [] and said(rig, "giving up") and rig.arm._dead


def test_position_polled_every_500ms_while_moving(rig):
    rig.build()
    rig.run(0.3, V1)
    t = rig.clock()
    rig.run(3.0, V1)
    times = [tt for tt, d in rig.port.writes if d == b"p" and tt >= t]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert len(times) >= 5 and all(0.499 <= g <= 0.51 for g in gaps)


def test_close_while_dead_skips_stop_and_quit(rig):
    rig.build()
    rig.run(0.6, V1)
    rig.arm._attempts = jogd.MAX_RESTARTS
    rig.port.die(b"[301 killed by SIGABRT.]\r\n$ ")
    rig.run(0.1)
    rig.port.counts = [100, 200, 300, 400, 500, 600]
    rig.port.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    t = rig.clock()
    rig.arm.close()
    tail = [d for _, d in rig.port.sent(t)]
    assert tail == [b"w2\r"] and rig.port.closed
    assert rig.arm.joints == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] and said(rig, "left at")


def test_exception_in_the_thread_halts(rig):
    rig.build()
    rig.run(0.6, V1)
    t = rig.clock()

    def boom(now):
        raise ValueError("boom")
    rig.arm._positions = boom
    assert rig.arm.step() is False
    tail = [d for _, d in rig.port.sent(t)]
    assert tail[0] == CTRL_Z and b"q" not in tail and tail[-1] == b"w2\r"       # Ctrl-Z killed jogd: the prompt is checked, not `q`
    assert "internal error" in rig.arm.notice and rig.port.closed and rig.port.mode == "shell"
    assert rig.arm.step() is False


def test_error_line_stops_and_does_not_retry_until_release(rig):
    rig.build(errors="A")
    rig.run(2.0, V1)
    assert rig.arm.notice.startswith("controller refused: E 5 joint at its limit")
    assert said(rig, "E 5 joint at its limit")
    assert [d for _, d in rig.port.moves()] == [b"A09"]     # exactly one try
    assert b"." in [d for _, d in rig.port.writes]
    assert not rig.arm.state()["moving"]
    rig.run(0.5, [0] * 6)                                   # release
    rig.run(1.0, V1)
    assert [d for _, d in rig.port.moves()] == [b"A09", b"A09"]
    rig.run(0.5, [-0.6, 0, 0, 0, 0, 0])                     # the other way is not blocked
    assert rig.port.moves()[-1][1].startswith(b"a")


def test_dead_man_line_restarts(rig):
    rig.build()
    rig.run(0.6, V1)
    t = rig.clock()
    rig.port.move = None
    rig.port.inject(b"T\r\n")
    rig.run(0.5, V1)
    assert said(rig, "dead-man")
    after = [d for _, d in rig.port.sent(t)]
    assert b"." not in after and after.count(b"A09") >= 3
    assert rig.arm.state()["moving"]


def test_no_p_line_is_tolerated(rig):
    rig.build(has_p=False)
    start = list(rig.arm.joints)
    rig.run(0.6, V1)
    rig.run(2.0, [0] * 6)
    assert rig.arm.joints == start
    assert len(said(rig, "live position is not available")) >= 1
    assert sum(1 for s in rig.said if s.strip().startswith("jogd has sent no `P`")) == 1
    assert said(rig, "stopped (live position is not available")
    assert [1 for _, d in rig.port.writes if d == b"p"]


def test_p_requested_every_250ms(rig):
    rig.build()
    rig.run(2.0)
    times = [t for t, d in rig.port.writes if d == b"p"]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert len(times) >= 7 and all(0.249 <= g <= 0.26 for g in gaps)


def test_close_clean(rig):
    rig.build()
    rig.run(0.6, V1)
    rig.port.counts = [100, 200, 300, 400, 500, 600]
    rig.port.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    t = rig.clock()
    rig.arm.close()
    tail = [d for _, d in rig.port.sent(t)]
    assert tail[:2] == [b".", b"q"] and b"w2\r" in tail and CTRL_Z not in tail and CTRL_E not in tail
    assert rig.port.mode == "shell" and rig.port.closed
    assert rig.arm.joints == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] and said(rig, "left at")
    assert not [s for s in rig.said if "!!!" in s]
    rig.arm.close()                                         # twice is harmless


def test_close_escalates(rig):
    rig.build(q_works=False, ctrlz_kills=False)
    t = rig.clock()
    rig.arm.close()
    seq = [(tt - t, d) for tt, d in rig.port.sent(t)]
    assert [d for _, d in seq][:4] == [b".", b"q", CTRL_Z, CTRL_E]
    assert 5.0 <= seq[2][0] < 5.1 and seq[3][0] - seq[2][0] >= 2.5
    assert said(rig, "NO `$` PROMPT WITHIN 5 s") and said(rig, "KILLED with Ctrl-E")
    assert rig.port.closed


def test_close_escalation_stops_at_ctrl_z_when_that_works(rig):
    rig.build(q_works=False)
    rig.arm.close()
    sent = [d for _, d in rig.port.writes]
    assert CTRL_Z in sent and CTRL_E not in sent


def test_realarm_refusals_and_state_keys(rig):
    from a465 import crs
    arm = rig.build()
    with pytest.raises(RobotError, match="joint mode only"):
        arm.jog("world", [1, 0, 0, 0, 0, 0])
    arm.jog("world", [0, 0, 0, 0, 0, 0])                      # nothing pushed: fine, as for RealArm
    for call in (lambda: arm.set_arm_power(True), lambda: arm.set_gripper(True), lambda: arm.goto("x"),
                 lambda: arm.home(), lambda: arm.cycle_start(), lambda: arm.set_speed(0), lambda: arm.set_speed(True)):
        with pytest.raises(RobotError):
            call()
    with pytest.raises(RobotError, match="held to 25%"):
        arm.set_speed(60)
    assert arm.speed == 25
    arm.set_speed(7)
    assert arm.speed == 7
    realkeys = {"backend", "joints", "true_joints", "speed", "command", "moving", "notice", "mode", "settling", "home_switches"}
    assert set(arm.state()) == realkeys
    assert arm.state()["true_joints"] is None


def test_true_joints_with_zero_at(rig):
    arm = rig.build(zero_at=[1, 2, 3, 4, 5, 6])
    assert arm.true_joints() == [9.0, -22.0, 27.0, 36.0, -55.0, 54.0]


def test_the_cap_is_the_constructor_argument(rig):
    arm = rig.build(max_speed=40)
    arm.set_speed(40)
    assert arm.speed == 40
    with pytest.raises(RobotError):
        arm.set_speed(41)
    rig.run(0.5, [1.0, 0, 0, 0, 0, 0])
    assert rig.port.moves()[0][1] == b"A40"


def test_threaded_smoke(tmp_path):
    """The real thread, the real clock, a quick fake."""
    import time
    local = tmp_path / "jogd"
    local.write_bytes(PROGRAM)
    port = FakeController(time.monotonic, ready_after=0.05)
    said_ = []
    arm = JogdArm(serial_port=port, say=said_.append, log_dir=str(tmp_path), local=str(local))
    arm.speed = 25
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        arm.jog("joint", V1, ttl=0.25, source="gamepad")
        time.sleep(0.02)
    assert port.moves() and arm.state()["moving"]
    arm.close()
    assert port.mode == "shell" and port.closed and not arm._thread.is_alive()


def test_direction_only_runs_every_jog_at_the_speed_setting(rig):
    rig.build(stick_speed=False)                            # the stick gives the direction; the speed is the pad's setting
    for vec, text in (([0.2, 0, 0, 0, 0, 0], b"A25"), ([0.6, 0, 0, 0, 0, 0], b"A25"), ([-1.0, 0, 0, 0, 0, 0], b"a25")):
        rig.run(0.5, vec)
        rig.run(0.4, [0] * 6)
        assert {d for _, d in rig.port.moves(rig.clock() - 0.9)} == {text}
    rig.arm.speed = 12
    rig.run(0.4, [0, 0.3, 0, 0, 0, 0])
    assert rig.port.moves(rig.clock() - 0.4)[0][1] == b"B12"


def test_direction_only_does_not_restart_when_the_stick_moves(rig):
    rig.build(stick_speed=False)
    stick(rig, [0.3] * 10 + [1.0] * 40 + [0.25] * 40)       # a light push, then full, then light again: one jog, one speed
    assert {d for _, d in rig.port.moves()} == {b"A25"}
