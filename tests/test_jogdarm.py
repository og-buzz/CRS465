"""JogdArm against a protocol-level fake of the controller's `$` shell and of jogd, with a fake clock."""
import re

import pytest

from a465 import jogd
from a465.jogd import JogdArm, command_bytes, speed_for
from a465.sim import RobotError

PROGRAM = b"fake jogd program bytes"
ALL_FRAMES = ("joint", "world", "tool")
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
                 errors=(), cksum_ok=True, ctrlz_aborts=None):
        self.clock = clock
        self.out = []                    # (due, bytes)
        self.writes = []                 # (time, bytes)
        self.mode = "shell"
        self.line = b""
        self.button_after, self.ready_after, self.has_p, self.echo = button_after, ready_after, has_p, echo
        self.q_works, self.ctrlz_kills, self.errors, self.cksum_ok = q_works, ctrlz_kills, set(errors), cksum_ok
        self.ignore_stop = ignore_stop    # `.` is not answered (no `-`)
        self.ctrlz_aborts = ctrlz_aborts  # jogd-c survives Ctrl-Z: it prints its `A` line this many seconds later (None: dies / is plain jogd)
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
        self.cart, self.cmode = False, 1  # started as jogd-c (modes 3, 4: one bounded move per letter / vector)
        self.move_s = None               # jogd-c: a move ends by itself (a bare `-`) this long after its `+` (None: never)
        self.end_at = None
        self.ended = None                # the command whose move ended by itself: jogd-c ignores it until a `.`
        self.started = 0                 # moves jogd-c has started (`+` lines put)
        self.vectors_seen = []           # the vectors accepted, as bytes
        self.has_w = True                # jogd-c answers `w` with `W x y z yaw pitch roll` (hundredths)
        self.world = [-10650, -1280, 98770, 9290, -7030, 200]
        self.gt = None                   # jogd-c go-to in force: (letter, numbers, speed)
        self.gline = None                # a `G` / `L` line being received
        self.gt_s = None                 # a go-to arrives (a bare `-`) this long after its `+` (None: never)
        self.gt_end = None
        self.gt_last = 0.0               # when the line or the last `g` came
        self.gt_lines = []               # every start line accepted, as bytes (with the `\r`)
        self.g_moves = 0                 # `g` keep-alives written while a letter jog / vector was in force
        self.g_count = 0                 # `g` keep-alives written while a go-to was in force

    def set_joints(self, angles):
        """What the next `p` reports (and `w2`)."""
        self.joints = [float(a) for a in angles]
        self.counts = [round(a * k) for a, k in zip(self.joints, CPD)]

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
        if self.gt and self.gt_end is not None and self.clock() >= self.gt_end:
            letter, nums, _ = self.gt
            if letter == "G":
                self.set_joints([c / k for c, k in zip(nums, CPD)])
            self.gt, self.gt_end = None, None
            self.put(b"-\r\n")
            return
        if self.gt and not self.ignore_stop and self.clock() - self.gt_last > 0.3:
            self.gt, self.gt_end = None, None
            self.put(b"T\r\n")
            return
        if self.move and self.end_at is not None and self.clock() >= self.end_at:
            self.ended, self.move, self.end_at = self.move, None, None
            self.put(b"-\r\n")
            return
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
        if b == CTRL_Z and self.ctrlz_aborts is not None:
            self.abort(after=self.ctrlz_aborts, line=b"A abort\r\n")
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
            if self.q_works:                    # as measured on the controller: `-` and a line end only if a move
                moving = self.move is not None  # was in force; otherwise the prompt comes alone, with no line end
                self.move = None
                self.mode = "shell"
                self.put(b"-\r\n$ " if moving else b"$ ", 0.05)
            else:
                self.stuck = True
        elif b == b"." and self.gline is None:
            self.ended = None
            self.line = b""
            if self.gt and not self.ignore_stop:
                self.gt, self.gt_end = None, None
                self.put(b"-\r\n", 0.2)
            if self.move and not self.ignore_stop:
                self.move, self.end_at = None, None
                self.put(b"-\r\n", 0.2)
        elif b == b"p":
            if self.has_p:
                self.put(("P " + " ".join(str(c) for c in self.counts) + "\r\n").encode(), 0.02)
        elif self.cart and (self.gline is not None or (b in (b"G", b"L") and not self.line)):
            self.gline = (self.gline or b"") + b
            if b == b"\r":
                text, self.gline = self.gline[:-1], None
                self._goto_line(text)
        elif self.cart and b == b"w":
            if self.has_w:
                self.put(("W " + " ".join(str(v) for v in self.world) + "\r\n").encode(), 0.02)
        elif self.cart and b == b"g":
            if self.gt:
                self.gt_last = self.clock()
                self.g_count += 1
            elif self.move:                     # the keep-alive of a letter jog or a vector: restarts the dead-man clock only
                self.last_cmd = self.clock()
                self.g_moves += 1
        elif self.cart and (self.line.startswith(b"@") or b == b"@"):
            if b == b"@" or not re.match(rb"[A-Fa-f]", b):
                self.line = b"@" if b == b"@" else self.line + b
            else:
                self.line = b
            if len(self.line) == 15:
                text, self.line = self.line, b""
                if re.fullmatch(rb"@([+-]\d){6}\d\d", text):
                    self._vector(text)
        else:
            self.line += b
            if re.fullmatch(rb"[A-Fa-f]\d\d", self.line):
                letter, speed = chr(self.line[0]), int(self.line[1:])
                self.line = b""
                self._command(letter, speed)
            elif len(self.line) >= 3:
                self.line = b""

    def _goto_line(self, text):
        self.gt_last = self.clock()
        m = re.fullmatch(rb"([GL])((?: -?\d+){6}) (\d\d)", text)
        if not m:
            self.put(b"E -22 invalid argument\r\n", 0.05)
            return
        if self.gt:
            self.put(b"-\r\n", 0.2)
        self.gt = (m.group(1).decode(), [int(n) for n in m.group(2).split()], int(m.group(3)))
        self.gt_lines.append(text + b"\r")
        self.put(b"+" + m.group(1) + b"\r\n", 0.28)
        self.gt_end = self.clock() + 0.28 + self.gt_s if self.gt_s is not None else None

    def abort(self, after=0.0, line=b"A -67108870 Path error\r\n"):
        """The controller aborts the move in force (a joint limit, a collision, Ctrl-Z): jogd-c survives, ends the move, prints ONE
        `A` line after `after` seconds and then ignores repeats of the command that was in force until a `.` or another command."""
        self.ended, self.move, self.end_at = self.move, None, None
        self.gt, self.gt_end = None, None
        self.put(line, after)
        self.aborts = getattr(self, "aborts", 0) + 1

    def kill(self):
        """What Ctrl-Z does on the real controller: stops the arm and kills the program that holds control."""
        self.move = None
        self.gt = None
        self.ignore_stop = False          # a restarted jogd answers again
        self.mode = "shell"
        self.line = b""
        self.need_button = True
        self.killed_at = self.clock()
        self.put(b"^C equivalent - halting robot...\r\n[301 killed by SIGABRT.]\r\n$ ", 0.05)

    def _vector(self, text):
        self.last_cmd = self.clock()
        comps = [int(text[1 + 2 * i:3 + 2 * i]) for i in range(6)]
        if any(comps[3:]):
            self.put(b"E -22 invalid argument\r\n", 0.1)
            return
        new = ("@", tuple(comps[:3]), int(text[13:15]))
        if self.move and self.move[0] == "@":
            return                                          # a keep-alive, whatever the vector
        if self.move:
            self.put(b"-\r\n", 0.2)
        elif self.ended is not None and self.ended[:2] == new[:2]:
            return                                          # the vector last started has ended: ignored
        self.move = new
        self.started += 1
        self.vectors_seen.append(text)
        self.put(b"+" + text[:13] + f"{new[2]:02d}".encode() + b"\r\n", 0.28)
        self._plan_end()

    def _plan_end(self):
        self.end_at = self.clock() + 0.28 + self.move_s if self.move_s is not None else None

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
        if self.cart and self.cmode != 1:
            if self.move and self.move[:2] == new[:2]:
                return                                      # the speed is fixed per move: a keep-alive
            if self.ended is not None and self.ended[:2] == new[:2]:
                return                                      # the move that ended is not started by its own repeats
            if self.move:
                self.put(b"-\r\n", 0.2)
            self.move = new
            self.started += 1
            self.put(f"+{letter}{speed:02d}\r\n".encode(), 0.28)
            self._plan_end()
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
            self.put(f"{text}\r\n0x{sum(PROGRAM) + bad:x} {len(PROGRAM)} {text.split()[-1]}\r\n$ ".encode(), 0.05)
        elif text == "w2":
            counts = "".join(f"{c:+11d}" for c in self.counts)
            joints = "".join(f"{a:11.3f}" for a in self.joints)
            self.put(f"w2\r\nPULSES{counts}\r\n   +0 +0\r\nJOINTS{joints}\r\n  0.000 0.000\r\n$ ".encode(), 0.05)
        elif text.startswith("/tmp/jogd"):
            self.put(text.encode() + b"\r\n")
            self.mode = "jogd"
            self.line = b""
            argv = text.split()
            self.cart, self.cmode = argv[0] == "/tmp/jogd-c", int(argv[1]) if len(argv) > 1 else 1
            self.move, self.ended, self.end_at = None, None, None
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

    def vectors(self, since=0.0):
        return [(t, d) for t, d in self.sent(since) if re.fullmatch(rb"@([+-]\d){6}\d\d", d)]


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
                                                        "ctrlz_kills", "ignore_stop", "errors", "cksum_ok", "ctrlz_aborts")}
        move_s = kw.pop("move_s", None)
        r.port = FakeController(r.clock, **opts)
        r.port.move_s = move_s
        kw.setdefault("local_c", str(local))
        kw.setdefault("frames", ("joint",))             # jogd, as before; the jogd-c tests ask for ALL_FRAMES
        kw.setdefault("wall", r.clock)
        r.arm = JogdArm(serial_port=r.port, say=r.said.append, log_dir=str(tmp_path), clock=r.clock, sleep=r.clock.sleep,
                        threaded=False, local=str(local), **kw)
        r.arm.speed = 25
        return r.arm

    def run(seconds, vec=None, every=0.02, frame="joint"):
        """Pump the thread's step every 2 ms; call jog with `vec` every `every` seconds (None: no pad calls)."""
        end, next_jog = r.clock() + seconds, r.clock()
        while r.clock() < end:
            if vec is not None and r.clock() >= next_jog:
                r.arm.jog(frame, vec, ttl=0.25, source="gamepad")
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
            local=str(rig.tmp / "jogd"), frames=("joint",), upload=lambda link, say: (ups.append(1), setattr(port, "cksum_ok", True)))
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
                threaded=False, local=str(rig.tmp / "jogd"), frames=("joint",))
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
    assert rig.arm.state()["moving"] and rig.arm.state()["command"] == {"kind": "jog", "source": "gamepad", "axis": 1, "sign": 1, "speed": 9, "frame": "joint", "vector": None}


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
    with pytest.raises(RobotError, match="not available"):
        arm.jog("camera", [1, 0, 0, 0, 0, 0])
    arm.jog("camera", [0, 0, 0, 0, 0, 0])                     # nothing pushed: fine, as for RealArm
    for call in (lambda: arm.set_arm_power(True), lambda: arm.set_gripper(True), lambda: arm.goto("x"),
                 lambda: arm.home(), lambda: arm.cycle_start(), lambda: arm.set_speed(0), lambda: arm.set_speed(True)):
        with pytest.raises(RobotError):
            call()
    with pytest.raises(RobotError, match="held to 25%"):
        arm.set_speed(60)
    assert arm.speed == 25
    arm.set_speed(7)
    assert arm.speed == 7
    realkeys = {"backend", "joints", "true_joints", "speed", "command", "moving", "notice", "mode", "settling", "home_switches",
                "frame", "max_speed", "joint_caps", "frame_caps", "joint_speeds", "position_time", "jogd", "controller_message", "limit", "guard_limits", "goto", "counts"}
    assert set(arm.state()) == realkeys
    assert arm.state()["limit"] is None
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
    arm = JogdArm(serial_port=port, say=said_.append, log_dir=str(tmp_path), local=str(local), frames=("joint",))
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


# ---- frames, caps, controller messages, state ---------------------------------------------------------

def starts(rig):
    return [d for _, d in rig.port.writes if d.startswith(b"/tmp/jogd")]


@pytest.mark.parametrize("frame, mode", [("world", 3), ("tool", 4)])
def test_a_frame_change_quits_and_restarts_jogd(rig, frame, mode):
    arm = rig.build(frames=ALL_FRAMES)
    rig.run(0.3, [0] * 6)
    before = len(rig.port.writes)
    arm.set_frame(frame)
    assert arm.state()["notice"] == f"changing to {frame.upper()} frame" and arm.state()["jogd"] == "changing frame"
    arm.jog(frame, [1, 0, 0, 0, 0, 0], ttl=0.25)                # ignored while it runs
    rig.run(6.0)
    assert arm.frame == frame and arm.state()["frame"] == frame and arm.state()["jogd"] == "running"
    assert arm.state()["notice"] == ""
    sent = [d for _, d in rig.port.writes[before:]]
    assert sent[0] == b"q" and sent[1] == f"/tmp/jogd-c {mode} 25 s\r".encode()
    assert not rig.port.moves() and rig.port.mode == "jogd"
    assert b"p" not in sent[:2] and starts(rig)[-1] == f"/tmp/jogd-c {mode} 25 s\r".encode()


def test_a_held_stick_does_not_start_after_a_frame_change(rig):
    arm = rig.build(frames=ALL_FRAMES)
    arm.set_frame("world")
    rig.run(4.0, V1, frame="world")                           # the stick is held through the whole change
    assert arm.frame == "world" and not rig.port.moves()
    rig.run(0.3, [0] * 6, frame="world")                      # let go once
    rig.run(0.5, V1, frame="world")
    assert rig.port.moves() and rig.port.moves()[0][1].startswith(b"A")
    assert arm.state()["command"]["frame"] == "world"


def test_a_jog_in_another_frame_changes_the_frame_and_is_dropped(rig):
    arm = rig.build(frames=ALL_FRAMES)
    rig.run(0.3, [0] * 6)
    arm.jog("tool", V1, ttl=0.25)
    assert arm.state()["jogd"] == "changing frame"
    for _ in range(3000):
        arm.jog("tool", V1, ttl=0.25)
        assert arm.step()
        rig.clock.sleep(0.002)
    assert arm.frame == "tool" and starts(rig)[-1] == b"/tmp/jogd-c 4 25 s\r" and not rig.port.moves()


def test_a_frame_change_waits_for_a_move_to_stop(rig):
    arm = rig.build(frames=ALL_FRAMES)
    rig.run(0.8, V1)
    assert rig.port.move
    arm.set_frame("world")
    rig.run(3.0)
    sent = [d for _, d in rig.port.writes]
    assert sent.index(b".") < sent.index(b"q") and arm.frame == "world"


def test_a_failed_frame_change_goes_to_the_restart(rig):
    arm = rig.build(q_works=False, frames=ALL_FRAMES)
    arm.set_frame("world")
    rig.run(0.2)
    assert arm.state()["jogd"] in ("changing frame", "dead")
    rig.run(20.0)
    assert said(rig, "FRAME CHANGE FAILED") and said(rig, "JOGD IS NOT RUNNING")
    assert "frame change to world failed" in arm.state()["notice"] or said(rig, "restarting jogd")


def test_world_axes_table_is_used(rig, monkeypatch):
    monkeypatch.setitem(jogd.FRAME_AXES, "world", [(2, -1), (1, +1), (3, +1), (4, +1), (5, +1), (6, +1)])
    arm = rig.build(stick_speed=False, frames=ALL_FRAMES)
    arm.set_frame("world")
    rig.run(4.0, [0] * 6, frame="world")
    rig.run(0.6, [1.0, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.moves()[0][1] == b"b15"


def test_caps(rig):
    arm = rig.build(max_speed=70, stick_speed=False)
    arm.speed = 60
    for vec, text in (([1.0, 0, 0, 0, 0, 0], b"A60"), ([0, 1.0, 0, 0, 0, 0], b"B25"), ([0, 0, -1.0, 0, 0, 0], b"c25"), ([0, 0, 0, 1.0, 0, 0], b"D60")):
        rig.run(0.6, vec)
        rig.run(0.5, [0] * 6)
        assert rig.port.moves(rig.clock() - 1.1)[0][1] == text
    arm.speed = 5
    rig.run(0.6, [0, 1.0, 0, 0, 0, 0])
    assert rig.port.moves(rig.clock() - 0.6)[0][1] == b"B05"
    s = arm.state()
    assert s["joint_caps"] == {"2": 25, "3": 25} and s["frame_caps"] == {"world": 15, "tool": 15} and s["max_speed"] == 70


def test_frame_cap_and_custom_caps(rig):
    arm = rig.build(max_speed=70, stick_speed=False, joint_caps={}, frame_caps={"world": 12}, frames=ALL_FRAMES)
    arm.speed = 60
    assert arm.state()["joint_caps"] == {} and arm.state()["frame_caps"] == {"world": 12, "tool": 15}
    rig.run(0.6, [0, 1.0, 0, 0, 0, 0])
    assert rig.port.moves()[0][1] == b"B60"
    rig.run(0.5, [0] * 6)
    arm.set_frame("world")
    rig.run(4.0, [0] * 6, frame="world")
    rig.run(0.6, [1.0, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.moves(rig.clock() - 0.6)[0][1] == b"A12"
    arm.speed = 3
    rig.run(0.5, [0] * 6, frame="world")
    rig.run(0.6, [0, 1.0, 0, 0, 0, 0], frame="world")
    assert rig.port.moves(rig.clock() - 0.6)[0][1] == b"B03"


def test_cap_parsing():
    assert jogd.parse_joint_caps("2=25,3=25") == {2: 25, 3: 25} and jogd.parse_joint_caps("none") == {}
    assert jogd.parse_frame_caps("tool=9") == {"world": 15, "tool": 9}
    for bad in ("2=0", "7=5", "2=100", "x", "2"):
        with pytest.raises(ValueError):
            jogd.parse_joint_caps(bad)
    for bad in ("world=0", "joint=5", "tool=100"):
        with pytest.raises(ValueError):
            jogd.parse_frame_caps(bad)
    with pytest.raises(RobotError):
        JogdArm(serial_port=None, joint_caps={2: 0})


KILL = (b"S: Arm power is OFF\r\nError 0x7000204\r\nCollision error on axis 2\r\n[376 killed by SIGABRT.]\r\n$ ")


def test_controller_messages_are_reported_with_the_kill(rig):
    arm = rig.build()
    rig.run(0.5, V1)
    assert arm.state()["controller_message"] is None
    rig.port.die(KILL)
    rig.run(0.05)
    want = "controller said: S: Arm power is OFF | Error 0x7000204 | Collision error on axis 2"
    assert said(rig, want)
    notice = arm.state()["notice"]
    assert want in notice and "restarted" in notice and "arm power is off: turn it on at the controller's front panel" in notice
    assert said(rig, "arm power is off: turn it on at the controller's front panel")
    msg = arm.state()["controller_message"]
    assert msg["text"] == "S: Arm power is OFF | Error 0x7000204 | Collision error on axis 2" and msg["at"] == pytest.approx(rig.clock(), abs=6.0)
    rig.run(8.0, [0] * 6)
    assert arm.state()["jogd"] == "running" and arm.state()["controller_message"] is not None
    rig.run(4.0, V1)
    assert rig.port.moves() and arm.state()["controller_message"] is None      # cleared by the next successful start


def test_old_lines_and_protocol_lines_are_not_messages(rig):
    arm = rig.build()
    rig.run(0.8, V1)
    rig.run(0.5, [0] * 6)
    rig.port.inject(b"something odd\r\n")
    rig.run(1.5, [0] * 6)
    assert arm.state()["controller_message"]["text"] == "something odd"
    rig.port.die(b"[9 killed by SIGABRT.]\r\n$ ")
    rig.run(0.05)
    assert "controller said" not in arm.state()["notice"]      # the odd line was more than a second old


def test_controller_error_reaches_the_notice(rig):
    arm = rig.build(errors={"A"})
    rig.run(0.6, V1)
    assert "controller refused: E 5 joint at its limit" in arm.state()["notice"] and said(rig, "controller refused: E 5 joint at its limit")
    assert arm.state()["controller_message"]["text"] == "E 5 joint at its limit"


def test_new_state_values(rig):
    arm = rig.build()
    s = arm.state()
    assert s["frame"] == "joint" and s["jogd"] == "running" and s["joint_speeds"] == [0.0] * 6
    rig.run(0.6)
    s = arm.state()
    assert s["position_time"] is not None and s["joint_speeds"] == [0.0] * 6
    rig.port.counts[0] += 277                                  # about 1 degree between two samples 0.25 s apart
    rig.run(0.3)
    s = arm.state()
    assert 2.0 < s["joint_speeds"][0] < 6.0 and s["joint_speeds"][1] == 0.0
    rig.clock.sleep(2.0)
    assert arm.state()["joint_speeds"] == [0.0] * 6


def test_a_frame_that_is_switched_off_moves_nothing(rig):
    rig.build(frames=("joint",))                            # world and tool are off until a bench run has confirmed the axes
    rig.arm.set_frame("world")
    rig.run(0.6, [0.9, 0, 0, 0, 0, 0], frame="world")
    assert not rig.port.moves() and rig.arm.frame == "joint"
    assert "WORLD frame is switched off" in rig.arm.state()["notice"]
    rig.run(0.5, [0.9, 0, 0, 0, 0, 0], frame="joint")       # the stick is still held: nothing may start
    assert not rig.port.moves()
    rig.run(0.1, [0] * 6, frame="joint")                    # let go once
    rig.run(0.5, [0.9, 0, 0, 0, 0, 0], frame="joint")
    assert rig.port.moves() and rig.port.moves()[0][1][:1] == b"A"
    assert "switched off" not in rig.arm.state()["notice"]


# ---- jogd-c: world and tool frames, bounded moves, `@` vectors ----------------------------------------------------------

import math

ZERO = [0.0] * 6


def cart(rig, frame="world", **kw):
    """An arm in world (or tool) frame on jogd-c, ready for a push; the stick has been let go once."""
    kw.setdefault("frames", ALL_FRAMES)
    arm = rig.build(**kw)
    arm.set_frame(frame)
    rig.run(4.0, ZERO, frame=frame)
    assert arm.frame == frame and rig.port.cart and rig.port.cmode == (3 if frame == "world" else 4)
    return arm


def writes_of(rig, since=0.0):
    return [d for _, d in rig.port.sent(since)]


def polar(angle_deg, r=1.0):
    return [r * math.cos(math.radians(angle_deg)), r * math.sin(math.radians(angle_deg)), 0, 0, 0, 0]


def test_jogd_c_is_chosen_and_uploaded_when_frames_include_world(rig):
    ups = []
    port = FakeController(rig.clock, cksum_ok=False)
    JogdArm(serial_port=port, say=rig.said.append, log_dir=str(rig.tmp), clock=rig.clock, sleep=rig.clock.sleep, threaded=False,
            local=str(rig.tmp / "no-jogd"), local_c=str(rig.tmp / "jogd"), frames=("joint", "world"),
            upload=lambda link, say: (ups.append(1), setattr(port, "cksum_ok", True)))
    assert ups == [1] and port.cart
    sent = [d for _, d in port.writes]
    assert b"cksum /tmp/jogd-c\r" in sent and b"/tmp/jogd-c 1 25 s\r" in sent
    assert not [d for d in sent if d.startswith(b"/tmp/jogd ")]
    assert jogd.REMOTE_C == "/tmp/jogd-c" and jogd.LOCAL_C.endswith("controller/programs/jogd-c")


def test_joint_only_keeps_jogd(rig):
    rig.build(frames=("joint",))
    assert not rig.port.cart and [d for _, d in rig.port.writes if d.startswith(b"/tmp/jogd")] == [b"/tmp/jogd 1 25 s\r"]


@pytest.mark.parametrize("frame", ["world", "tool"])
@pytest.mark.parametrize("index, letter", [(0, b"A"), (1, b"B"), (2, b"C"), (3, b"F"), (4, b"E"), (5, b"D")])
def test_axis_letters_for_each_pad_component(rig, frame, index, letter):
    cart(rig, frame)
    for sign, text in ((1, letter), (-1, letter.lower())):
        vec = [0.0] * 6
        vec[index] = 0.6 * sign
        t = rig.clock()
        rig.run(0.5, vec, frame=frame)
        assert rig.port.moves(t)[0][1] == text + b"%02d" % speed_for(0.6, 15)
        rig.run(0.6, ZERO, frame=frame)
        assert not rig.port.move
    assert not rig.port.vectors()


def test_a_move_that_ends_by_itself_starts_again_while_the_stick_is_held(rig):
    arm = cart(rig, move_s=0.5)
    t = rig.clock()
    rig.run(3.0, [0.6, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.started >= 3 and arm.state()["moving"] and not arm._dead
    assert said(rig, "move ended by itself") and CTRL_Z not in writes_of(rig)
    assert arm.notice == ""
    seq = writes_of(rig, t)
    assert seq.count(b".") == rig.port.started - 1                 # one `.` before each start but the first (jogd-c ignores a repeat)
    for i, d in enumerate(seq):
        if d == b".":
            assert seq[i + 1] == b"A06"
    assert {d for d in seq if d != b"." and d != b"p"} == {b"A06", b"g"}      # the letter once per start, `g` between


def test_a_move_that_ends_by_itself_stays_stopped_when_the_stick_is_let_go(rig):
    arm = cart(rig)
    rig.run(0.7, [0.6, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.move and arm.state()["moving"]
    t = rig.clock()
    rig.port.end_at = rig.clock()                                  # jogd-c's move ends right now ...
    arm.jog("world", ZERO, ttl=0.25)                               # ... as the stick is let go
    rig.run(1.5, ZERO, frame="world")
    assert said(rig, "move ended by itself") and rig.port.started == 1
    assert [d for d in writes_of(rig, t) if d != b"p"] == []
    assert not arm.state()["moving"] and arm.state()["command"] is None and not arm._dead and not arm.notice


def test_a_move_that_ends_at_once_is_not_started_in_a_loop(rig):
    arm = cart(rig, move_s=0.1)                                    # the `-` comes 0.1 s after the `+`: a refusal, not a finished move
    rig.run(2.0, [0.6, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.started == 1 and "ended the move at once" in arm.notice
    rig.run(0.2, ZERO, frame="world")
    rig.run(0.6, [0.6, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.started == 2


def test_a_diagonal_push_sends_a_vector_and_keeps_it_alive(rig):
    arm = cart(rig)
    t = rig.clock()
    rig.run(2.0, [0.6, 0.4, 0, 0, 0, 0], frame="world")
    sp = speed_for(0.6, 15)
    want = b"@+9+6+0+0+0+0%02d" % sp
    assert len(want) == 15
    vectors = rig.port.vectors(t)
    assert {d for _, d in vectors} == {want} and not rig.port.moves(t)
    assert len(vectors) == 1                                        # the vector once; `g` is the keep-alive
    gs = [t_ for t_, d in rig.port.sent(t) if d == b"g"]
    gaps = [b - a for a, b in zip(gs, gs[1:])]
    assert len(gs) >= 15 and max(gaps) <= 0.151 and min(gaps) >= 0.099
    assert rig.port.vectors_seen == [want] and rig.port.started == 1
    cmd = arm.state()["command"]
    assert arm.state()["moving"] and cmd["vector"] == [9, 6, 0] and cmd["axis"] is None and cmd["sign"] is None and cmd["speed"] == sp
    assert b"." not in writes_of(rig, t) and not arm.notice


def test_vector_signs_and_second_example(rig):
    cart(rig, "tool")
    t = rig.clock()
    rig.run(1.0, [1.0, -0.35, 0.2, 0, 0, 0], frame="tool")
    assert {d for _, d in rig.port.vectors(t)} == {b"@+9-3+2+0+0+0%02d" % speed_for(1.0, 15)}


def test_a_stick_below_the_threshold_is_not_part_of_a_vector(rig):
    cart(rig)
    t = rig.clock()
    rig.run(0.8, [0.6, 0.15, 0.1, 0, 0, 0], frame="world")        # only x is past crs.STICK_ON: one axis, a letter
    assert rig.port.moves(t)[0][1] == b"A%02d" % speed_for(0.6, 15) and not rig.port.vectors(t)


def test_a_wobble_of_10_degrees_sends_nothing_new(rig):
    arm = cart(rig)
    t = rig.clock()
    rig.run(0.8, polar(35, 0.9), frame="world")
    first = rig.port.vectors(t)[0][1]
    t2 = rig.clock()
    for i in range(6):
        rig.run(0.25, polar(35 + 10, 0.9), frame="world")
        rig.run(0.25, polar(35 - 10, 0.9), frame="world")
    assert {d for _, d in rig.port.vectors(t)} == {first} and rig.port.started == 1
    assert b"." not in writes_of(rig, t2) and not rig.port.speed_changes and arm.state()["moving"]


def test_a_sustained_change_of_60_degrees_gives_one_stop_and_one_new_vector(rig):
    arm = cart(rig)
    t = rig.clock()
    rig.run(1.0, polar(12), frame="world")
    first = rig.port.vectors(t)[0][1]
    assert first[:13] == b"@+9+2+0+0+0+0"
    t2 = rig.clock()
    rig.run(0.25, polar(72), frame="world")                         # shorter than DIR_CHANGE_S: nothing yet
    assert b"." not in writes_of(rig, t2)
    rig.run(0.25, polar(12), frame="world")                         # and back: the clock starts over
    rig.run(2.5, polar(72), frame="world")
    seq = [d for d in writes_of(rig, t2) if d != b"p"]
    dots = [tt for tt, d in rig.port.sent(t2) if d == b"."]
    assert len(dots) == 1 and not rig.port.moves(t)
    vectors = [d for d in dict.fromkeys(d for _, d in rig.port.vectors(t))]
    assert vectors == [first, b"@+3+9+0+0+0+0" + first[13:]]
    assert seq.count(vectors[1]) == 1 and seq.count(b"g") >= 5 and rig.port.started == 2 and arm.state()["command"]["vector"] == [3, 9, 0]
    assert said(rig, "stopping, then starting vector +3 +9 +0")
    assert not arm.notice and not arm._dead


def test_a_change_between_one_axis_and_several_follows_the_same_rule(rig):
    cart(rig)
    t = rig.clock()
    rig.run(0.8, [0.9, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.moves(t)
    rig.run(0.2, [0.9, 0.5, 0, 0, 0, 0], frame="world")             # 29 degrees off the X axis, but not for 0.3 s yet
    assert not rig.port.vectors(t) and b"." not in writes_of(rig, t)
    rig.run(1.5, [0.9, 0.5, 0, 0, 0, 0], frame="world")
    assert [d for _, d in rig.port.vectors(t)][-1][:13] == b"@+9+5+0+0+0+0" and writes_of(rig, t).count(b".") == 1
    t2 = rig.clock()
    rig.run(1.5, [0.9, 0.0, 0, 0, 0, 0], frame="world")             # and back to one axis
    assert writes_of(rig, t2).count(b".") == 1 and rig.port.moves(t2)[-1][1][:1] == b"A"


def test_a_vector_is_stopped_by_letting_go(rig):
    arm = cart(rig)
    rig.run(1.0, [0.6, 0.4, 0, 0, 0, 0], frame="world")
    t = rig.clock()
    arm.jog("world", ZERO, ttl=0.25)
    rig.run(0.01)
    assert [tt for tt, d in rig.port.sent(t) if d == b"."] and writes_of(rig, t)[0] == b"."
    rig.run(0.5, ZERO, frame="world")
    assert writes_of(rig, t).count(b".") == 1 and not arm.state()["moving"] and arm.state()["command"] is None
    assert not rig.port.move and not arm._dead


def test_rotation_with_translation_gives_a_letter_for_the_larger(rig):
    cart(rig)
    t = rig.clock()
    rig.run(0.8, [0.5, 0.4, 0, 0, 0, 0.9], frame="world")           # yaw is the largest: one axis (D), no vector
    assert rig.port.moves(t)[0][1].startswith(b"D") and not rig.port.vectors(t)
    rig.run(0.8, ZERO, frame="world")
    t = rig.clock()
    rig.run(0.8, [0.9, 0.5, 0, 0.6, 0, 0], frame="world")           # x is the largest: the rotation takes no part in the vector
    assert rig.port.vectors(t)[0][1][:13] == b"@+9+5+0+0+0+0" and not rig.port.moves(t)


def test_no_respeed_in_a_cart_frame(rig):
    cart(rig)
    t = rig.clock()
    rig.run(1.2, [0.6, 0, 0, 0, 0, 0], frame="world")
    rig.run(3.0, [1.0, 0, 0, 0, 0, 0], frame="world")
    assert {d for _, d in rig.port.moves(t)} == {b"A%02d" % speed_for(0.6, 15)} and not rig.port.speed_changes


def test_the_emergency_rule_after_a_stop_still_fires(rig):
    arm = cart(rig, ignore_stop=True)
    arm._attempts = jogd.MAX_RESTARTS
    rig.run(1.0, [0.6, 0.4, 0, 0, 0, 0], frame="world")
    t = rig.clock()
    rig.run(0.0, ZERO)
    arm.jog("world", ZERO, ttl=0.25)
    rig.run(1.0)                                            # jogd-c's `-` takes up to 0.65 s on the arm: no alarm yet
    assert not [d for _, d in rig.port.sent(t) if d == CTRL_Z]
    rig.run(0.8)
    dot = [tt for tt, d in rig.port.sent(t) if d == b"."]
    z = [tt for tt, d in rig.port.sent(t) if d == CTRL_Z]
    wait = jogd.STOP_WAIT_CART_S
    assert len(dot) == 1 and len(z) == 1 and wait <= z[0] - dot[0] < wait + 0.02
    assert said(rig, f"NO `-`, `T` OR `E` WITHIN {wait} s") and arm.notice.startswith("jogd is not running")


def test_an_error_after_a_vector_reaches_the_notice(rig):
    arm = cart(rig)
    rig.run(0.8, [0.6, 0.4, 0, 0, 0, 0], frame="world")
    t = rig.clock()
    rig.port.put(b"E -22 invalid argument\r\n")
    rig.run(0.5, [0.6, 0.4, 0, 0, 0, 0], frame="world")
    assert "controller refused: E -22 invalid argument" in arm.notice
    assert arm.state()["controller_message"]["text"] == "E -22 invalid argument"
    assert b"." in writes_of(rig, t)
    n = len(rig.port.vectors(t))
    rig.run(1.0, [0.6, 0.4, 0, 0, 0, 0], frame="world")             # blocked until the stick is let go
    assert len(rig.port.vectors(t)) == n


def test_the_fake_rejects_a_vector_with_rotation_components(rig):
    cart(rig)
    rig.port.write(b"@+1+0+0+0+1+002")
    rig.run(0.3)
    assert not rig.port.move and rig.port.started == 0 and rig.port.vectors_seen == []
    assert rig.arm.notice == "" or "E -22" in rig.arm.notice       # the fake answers E -22 as jogd-c does; the arm never sends one itself


# ---- joint-limit guard (world and tool frames) ------------------------------------------------------------------------

POSE = [44.0, -4.0, -111.0, -8.0, -48.5, -4.0]        # J3 inside the margin of its low limit (-114.993); where the operator pulled in
NEG_Y, POS_Y = [0, -0.6, 0, 0, 0, 0], [0, 0.6, 0, 0, 0, 0]
NEG_VEC, POS_VEC = [0, -0.6, -0.33, 0, 0, 0], [0, 0.6, 0.33, 0, 0, 0]   # the vector (0, -9, -5): J3 toward its low limit


def cart_at(rig, joints, frame, **kw):
    kw.setdefault("frames", ALL_FRAMES)
    arm = rig.build(**kw)
    rig.port.set_joints(joints)
    arm.set_frame(frame)
    rig.run(4.0, ZERO, frame=frame)
    assert arm.frame == frame and [round(a, 1) for a in arm.joints] == [round(a, 1) for a in joints]
    return arm


def drift(rig, start_j3, rate, seconds, vec, frame="world", base=POSE):
    """Run with J3 reading start_j3 + rate * t (degrees, degrees per second)."""
    t0, end = rig.clock(), rig.clock() + seconds
    while rig.clock() < end:
        rig.port.set_joints([base[0], base[1], start_j3 + rate * (rig.clock() - t0)] + list(base[3:]))
        rig.run(0.02, vec, frame=frame)


def test_guard_constants_and_state_key_when_far_from_limits(rig):
    assert jogd.GUARD_LIMITS_DEG[2] == (-114.993, 114.993) and jogd.GUARD_MARGIN_DEG == 4.0 and jogd.GUARD_LOOKAHEAD_S == 0.8
    arm = cart_at(rig, [10.0, -20.0, 30.0, 40.0, -50.0, 60.0], "world")
    st = arm.state()
    assert st["limit"] is None and st["guard_limits"][2] == [-114.993, 114.993]
    assert {"joints", "joint_speeds", "notice", "command", "moving"} <= set(st)


def test_world_move_toward_a_limit_is_stopped_with_the_normal_stop(rig):
    arm = cart_at(rig, [44.0, -4.0, -95.0, -8.0, -48.5, -4.0], "world")
    t = rig.clock()
    drift(rig, -95.0, 0.0, 0.6, NEG_Y)                      # standing still far from the limit: the move runs
    assert rig.port.move and b"." not in writes_of(rig, t) and not arm.notice
    t = rig.clock()
    drift(rig, -95.0, -10.0, 1.2, NEG_Y)                    # 10 deg/s: -100 + 10 * -0.8 = -108 is inside the margin
    seq = writes_of(rig, t)
    assert seq.count(b".") == 1 and CTRL_Z not in seq and not arm._dead
    assert "J3 is at its limit (-114.993)" in arm.notice and "blocked" in arm.notice and said(rig, "J3 is at its limit")
    assert len(said(rig, "J3 is at its limit")) == 1
    assert not rig.port.move
    n = len(rig.port.moves(t))
    drift(rig, -105.0, 0.0, 0.8, NEG_Y)                     # held: not restarted until the stick is let go
    assert len(rig.port.moves(t)) == n and arm._guard_joint == 2


def test_a_move_away_from_a_limit_is_not_stopped(rig):
    arm = cart_at(rig, [44.0, -4.0, -100.0, -8.0, -48.5, -4.0], "world")
    drift(rig, -100.0, 10.0, 1.2, POS_Y)
    assert rig.port.move and b"." not in writes_of(rig) and not arm.notice


def test_a_start_that_pushes_further_into_a_limit_is_refused(rig):
    arm = cart_at(rig, POSE, "world")
    t = rig.clock()
    rig.run(1.5, NEG_Y, frame="world")
    assert not rig.port.moves(t) and not rig.port.vectors(t) and rig.port.started == 0
    assert [d for d in writes_of(rig, t) if d != b"p"] == []
    assert "J3 is at its limit (-114.993)" in arm.notice and len(said(rig, "J3 is at its limit")) == 1
    lim = arm.state()["limit"]
    assert lim == {"joint": 3, "limit": -114.993, "angle": -111.0, "blocked": True}
    rig.run(0.5, ZERO, frame="world")                       # let go: tried again, still refused (same direction)
    rig.run(1.0, NEG_Y, frame="world")
    assert rig.port.started == 0 and len(said(rig, "J3 is at its limit")) == 2


def test_the_opposite_direction_is_allowed(rig):
    arm = cart_at(rig, POSE, "world")
    rig.run(1.0, NEG_Y, frame="world")
    assert rig.port.started == 0
    rig.run(0.5, ZERO, frame="world")
    t = rig.clock()
    rig.run(1.0, POS_Y, frame="world")
    assert rig.port.started == 1 and rig.port.moves(t)[0][1][:1] == b"B"
    assert arm.notice == "" and arm.state()["limit"]["blocked"] is False


def test_a_vector_is_treated_the_same_way(rig):
    arm = cart_at(rig, POSE, "world")
    rig.run(1.0, NEG_VEC, frame="world")
    assert rig.port.started == 0 and not rig.port.vectors() and "J3 is at its limit" in arm.notice
    rig.run(0.5, ZERO, frame="world")
    rig.run(1.0, POS_VEC, frame="world")
    assert rig.port.started == 1 and rig.port.vectors()


def test_a_vector_is_stopped_at_the_look_ahead(rig):
    arm = cart_at(rig, [44.0, -4.0, -95.0, -8.0, -48.5, -4.0], "world")
    rig.run(0.5, NEG_VEC, frame="world")
    assert rig.port.vectors()
    t = rig.clock()
    drift(rig, -95.0, -10.0, 1.2, NEG_VEC)
    assert writes_of(rig, t).count(b".") == 1 and "J3 is at its limit" in arm.notice


def test_tool_frame_is_guarded_too(rig):
    arm = cart_at(rig, POSE, "tool")
    assert arm._guard_joint_motion((0, 1))[2] < 0 < arm._guard_joint_motion((0, -1))[2]     # tool +X pushes J3 toward -114.993
    rig.run(1.0, [0.6, 0, 0, 0, 0, 0], frame="tool")
    assert rig.port.started == 0 and "J3 is at its limit" in arm.notice
    rig.run(0.5, ZERO, frame="tool")
    rig.run(1.0, [-0.6, 0, 0, 0, 0, 0], frame="tool")
    assert rig.port.started == 1 and arm.notice == ""


def test_the_guard_does_nothing_in_joint_frame(rig):
    arm = rig.build()
    rig.port.set_joints([0, 0, -111.0, 0, 0, 0])
    rig.run(1.0, ZERO)
    t = rig.clock()
    rig.run(0.6, [0, 0, -0.9, 0, 0, 0])
    assert rig.port.moves(t) and not arm.notice
    drift(rig, -111.0, -10.0, 1.0, [0, 0, -0.9, 0, 0, 0], frame="joint", base=[0, 0, 0, 0, 0, 0])
    assert b"." not in writes_of(rig, t) and not arm.notice


def test_no_limit_guard_turns_it_off(rig):
    arm = cart_at(rig, POSE, "world", limit_guard=False)
    rig.run(1.0, NEG_Y, frame="world")
    assert rig.port.started == 1 and not arm.notice and arm.state()["limit"] is None and arm.state()["guard_limits"] is None
    t = rig.clock()
    drift(rig, -111.0, -10.0, 1.0, NEG_Y)
    assert b"." not in writes_of(rig, t)


def test_a_near_singular_pose_does_not_block_or_raise(rig):
    arm = cart_at(rig, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "world")
    assert arm._guard_joint_motion((1, 1)) is None
    rig.run(1.0, [0.6, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.started == 1 and not arm._dead and arm._crashed is None
    rig.port.set_joints([0.0, 0.0, -111.0, 0.0, 0.0, 0.0])      # singular AND inside a margin: the start check is skipped
    rig.run(0.5, ZERO, frame="world")
    rig.run(1.0, [0.6, 0, 0, 0, 0, 0], frame="world")
    assert arm._crashed is None


def test_the_direction_check_on_the_operators_pose(rig):
    arm = cart_at(rig, POSE, "world")
    neg = arm._guard_joint_motion((1, -1))                       # world -Y
    pos = arm._guard_joint_motion((1, 1))
    assert neg[2] < -0.1 and pos[2] > 0.1 and abs(neg[2] + pos[2]) < 1e-9


# ---------------------------------------------------------------------------------------- the controller aborts a move: `A` line

A_LINE = b"A -67108870 Path error\r\n"
HELD = [0.6, 0, 0, 0, 0, 0]


def launches(rig):
    return [d for _, d in rig.port.writes if d.startswith(b"/tmp/jogd")]


def test_classify_a_lines():
    assert jogd.classify("A -67108870 Path error")[0] == "A"
    assert jogd.classify("A abort")[0] == "A"
    assert jogd.classify("$ A abort")[0] == "A"                      # an echo in front does no harm
    for text in ("+A07", "jogd ready", "S: Arm power is OFF", "E -42 device is being reset", "-", "Path error"):
        assert jogd.classify(text)[0] != "A"
    kind, found = jogd.classify("A -67108870 Path error")
    assert (found.group(1), found.group(2)) == ("-67108870", "Path error")


def test_an_abort_during_a_move_ends_it_blocks_the_direction_and_restarts_nothing(rig):
    arm = cart(rig)
    rig.run(1.0, HELD, frame="world")
    assert arm.state()["moving"]
    t = rig.clock()
    rig.port.abort()
    rig.run(1.5, HELD, frame="world")                                  # the stick is still held
    assert arm.notice == "the controller stopped the move: Path error"
    assert arm.state()["controller_message"]["text"] == "A -67108870 Path error"
    assert said(rig, "the controller stopped the move: Path error")
    assert not arm.state()["moving"] and arm._cmd is None and not arm._dead and arm.state()["jogd"] == "running"
    assert arm._stopping_since is None
    seq = writes_of(rig, t)
    assert CTRL_Z not in seq and CTRL_E not in seq and b"." not in seq             # no `.` owed, no emergency, nothing yet to re-arm
    after = [d for tt, d in rig.port.sent(t) if tt > t + 0.15 and re.fullmatch(rb"[A-Fa-f]\d\d", d)]
    assert after == []                                                   # blocked: no start while the stick is held
    assert len(launches(rig)) == 2 and rig.port.started == 1             # jogd-c (start, frame change) was never restarted
    rig.run(0.5, ZERO, frame="world")                                    # let go: the block ends
    t2 = rig.clock()
    rig.run(1.0, HELD, frame="world")
    seq = writes_of(rig, t2)
    assert rig.port.started == 2 and arm.state()["moving"] and arm.notice == ""
    assert seq.count(b".") == 1 and seq[seq.index(b".") + 1] == b"A06"   # a single `.` clears jogd-c's ignore-repeats, then the start


def test_a_blocked_direction_does_not_block_the_other_way(rig):
    arm = cart(rig)
    rig.run(1.0, HELD, frame="world")
    rig.port.abort()
    rig.run(0.5, HELD, frame="world")
    rig.run(0.3, ZERO, frame="world")
    t = rig.clock()
    rig.run(1.0, [-0.6, 0, 0, 0, 0, 0], frame="world")
    assert rig.port.moves(t)[0][1] == b"a06" and arm.state()["moving"]


def test_no_abort_text_for_the_old_kill_path(rig):
    arm = cart(rig)
    arm._attempts = jogd.MAX_RESTARTS
    rig.run(1.0, HELD, frame="world")
    rig.port.die(b"[301 killed by SIGABRT.]\r\n$ ")
    rig.run(0.5, HELD, frame="world")
    assert arm._dead and arm.notice.startswith("jogd is not running")


def test_the_power_line_before_the_a_line_gives_the_front_panel_advice(rig):
    arm = cart(rig)
    rig.run(1.0, HELD, frame="world")
    rig.port.put(b"S: Arm power is OFF\r\n")
    rig.run(0.2, HELD, frame="world")
    rig.port.abort()
    rig.run(0.5, HELD, frame="world")
    assert arm.notice == "the controller stopped the move: Path error. " + jogd.POWER_OFF_ADVICE
    assert not arm._dead and len(launches(rig)) == 2


def test_the_power_line_after_the_a_line_adds_the_advice(rig):
    arm = cart(rig)
    rig.run(1.0, HELD, frame="world")
    rig.port.abort(line=b"A abort\r\n")
    rig.run(0.5, HELD, frame="world")
    assert arm.notice == "the controller stopped the move: abort"
    rig.port.put(b"S: Arm power is OFF\r\n")
    rig.run(0.5, HELD, frame="world")
    assert arm.notice == "the controller stopped the move: abort. " + jogd.POWER_OFF_ADVICE


def test_an_a_line_with_no_move_is_only_shown(rig):
    arm = cart(rig)
    rig.run(0.5, ZERO, frame="world")
    t = rig.clock()
    rig.port.put(A_LINE)
    rig.run(1.0, ZERO, frame="world")
    assert "A -67108870 Path error" in arm.notice and "no move was in force" in arm.notice
    assert arm.state()["controller_message"]["text"] == "A -67108870 Path error"
    assert not arm._dead and arm._blocked is None and not arm._rearm
    assert [d for d in writes_of(rig, t) if d != b"p"] == []
    rig.run(1.0, HELD, frame="world")                                    # the next move starts as usual, with no `.` before it
    assert arm.state()["moving"] and rig.port.started == 1


def test_an_a_line_while_dot_is_in_flight_completes_the_stop(rig):
    arm = cart(rig, ignore_stop=True)                                    # the `.` is never answered by a `-`
    arm._attempts = jogd.MAX_RESTARTS
    rig.run(1.0, HELD, frame="world")
    rig.run(0.0, ZERO, frame="world")
    arm.jog("world", ZERO, ttl=0.25)
    t = rig.clock()
    rig.run(0.2, ZERO, frame="world")
    assert arm._stopping_since is not None and b"." in writes_of(rig, t)
    rig.port.abort(line=b"A abort\r\n")
    rig.run(2.5, ZERO, frame="world")                                    # past STOP_WAIT_CART_S: the emergency timer must be gone
    assert arm._stopping_since is None and CTRL_Z not in writes_of(rig) and not arm._dead
    assert arm.notice == "the controller stopped the move: abort" and not said(rig, "Ctrl-Z WRITTEN")


def test_an_a_line_while_a_start_is_not_yet_answered(rig):
    arm = cart(rig)
    t = rig.clock()
    rig.run(0.1, HELD, frame="world")
    rig.run(0.2, HELD, frame="world")
    rig.port.out.clear()
    rig.port.put(A_LINE)
    rig.run(0.5, HELD, frame="world")
    assert arm.notice.startswith("the controller stopped the move") or "no move was in force" in arm.notice
    assert not arm._dead


def test_the_abort_in_joint_frame_of_jogd_c_is_handled_too(rig):
    arm = rig.build(frames=ALL_FRAMES)
    rig.run(1.0, ZERO)
    assert rig.port.cart and rig.port.cmode == 1
    rig.run(1.0, V1)
    assert arm.state()["moving"]
    rig.port.abort()
    rig.run(0.5, V1)
    assert arm.notice == "the controller stopped the move: Path error" and not arm._dead and not arm.state()["moving"]


def test_ctrl_z_that_jogd_c_survives_is_not_a_death(rig):
    arm = cart(rig, ignore_stop=True, ctrlz_aborts=0.4)
    n = len(launches(rig))
    rig.run(1.0, HELD, frame="world")
    t = rig.clock()
    arm.jog("world", ZERO, ttl=0.25)
    rig.run(0.0, ZERO, frame="world")
    rig.run(3.0, ZERO, frame="world")
    z = [tt for tt, d in rig.port.sent(t) if d == CTRL_Z]
    assert len(z) == 1 and said(rig, "Ctrl-Z WRITTEN")
    assert said(rig, "jogd-c is alive after Ctrl-Z")
    assert not arm._dead and not arm._gave_up and arm.state()["jogd"] == "running"
    assert arm.notice == "the controller stopped the move: abort" and arm.state()["controller_message"]["text"] == "A abort"
    assert len(launches(rig)) == n and CTRL_E not in writes_of(rig)               # not restarted: no `\r`, no start command
    assert b"\r" not in writes_of(rig, z[0] + 0.01)
    assert arm._stopping_since is None and not said(rig, "JOGD IS NOT RUNNING")
    t = rig.clock()
    rig.run(1.0, HELD, frame="world")                                          # a start works at once
    assert arm.state()["moving"]


def test_ctrl_z_with_neither_death_nor_a_line_goes_on_as_before(rig):
    arm = cart(rig, ignore_stop=True, ctrlz_kills=False)
    arm._attempts = jogd.MAX_RESTARTS
    rig.run(1.0, HELD, frame="world")
    t = rig.clock()
    arm.jog("world", ZERO, ttl=0.25)
    rig.run(0.0, ZERO, frame="world")
    while not arm._dead and rig.clock() < t + 10:
        rig.run(0.05, ZERO, frame="world")
    z = [tt for tt, d in rig.port.sent(t) if d == CTRL_Z][0]
    assert arm._dead and arm.notice.startswith("jogd is not running") and "Ctrl-Z written" in arm.notice
    assert rig.clock() - z >= jogd.CTRL_Z_WAIT_S                      # it waited the whole 2.5 s for an `A` line or the death signs
    assert said(rig, "Ctrl-Z WRITTEN") and said(rig, "jogd will be restarted by itself") and not said(rig, "jogd-c is alive")


def test_a_death_sign_after_ctrl_z_ends_the_wait_at_once(rig):
    arm = cart(rig, ignore_stop=True)                                  # the fake kills jogd on Ctrl-Z, 50 ms later
    arm._attempts = jogd.MAX_RESTARTS
    rig.run(1.0, HELD, frame="world")
    arm.jog("world", ZERO, ttl=0.25)
    rig.run(0.0, ZERO, frame="world")
    rig.run(3.0, ZERO, frame="world")
    assert arm._dead and said(rig, "Ctrl-Z WRITTEN") and not said(rig, "jogd-c is alive")


def test_plain_jogd_does_not_wait_for_an_a_line_after_ctrl_z(rig):
    arm = rig.build(ignore_stop=True, ctrlz_aborts=0.4)                # frames=("joint",): the plain jogd has no handler
    arm._attempts = jogd.MAX_RESTARTS
    rig.run(0.6, V1)
    arm.halt()
    t = rig.clock()
    rig.run(0.7)
    assert CTRL_Z in writes_of(rig, t) and arm._dead and rig.clock() - t < 1.5
    assert not said(rig, "jogd-c is alive")


def test_close_after_ctrl_z_finds_jogd_c_alive_and_quits_it(rig):
    arm = cart(rig, ctrlz_aborts=0.3)
    rig.port.q_works = False
    t = rig.clock()
    arm.close()
    seq = [d for d in writes_of(rig, t)]
    assert CTRL_Z in seq and said(rig, "jogd-c is alive after Ctrl-Z")
    assert seq[seq.index(CTRL_Z) + 1] == b"q"


# ---- teach and replay: saved poses, go-to (G / L / g) -----------------------------------------------------------------

from a465.poses import PoseStore                                  # noqa: E402

TARGET = [20.0, -10.0, 40.0, 30.0, -40.0, 50.0]                   # inside the limits; the fake starts at [10, -20, 30, 40, -50, 60]


def teach(rig, **kw):
    """An arm with a pose store (so on jogd-c), pose A saved at TARGET, and a fresh position."""
    store = PoseStore(str(rig.tmp / "poses.json"))
    store.save("A", TARGET, [round(a * k) for a, k in zip(TARGET, CPD)])
    arm = rig.build(poses=store, **kw)
    rig.run(0.6)
    return arm, store


def hold(rig, seconds, name="A", straight=False, every=0.02, stick=None):
    """Pump the thread and call goto_hold every `every` seconds (and jog with `stick` when given)."""
    end, next_call = rig.clock() + seconds, rig.clock()
    while rig.clock() < end:
        if rig.clock() >= next_call:
            if stick is not None:
                rig.arm.jog("joint", stick, ttl=0.25, source="gamepad")
            try:
                rig.arm.goto_hold(name, straight=straight, source="gamepad")
            except RobotError as err:
                rig.refused = str(err)
            next_call += every
        assert rig.arm.step()
        rig.clock.sleep(0.002)


def test_a_pose_store_makes_the_driver_use_jogd_c(rig):
    arm, _ = teach(rig)
    assert rig.port.cart and b"/tmp/jogd-c 1 25 s\r" in [d for _, d in rig.port.writes]
    rig.build()
    assert not rig.port.cart                         # no store, joint only: jogd as before


def test_the_guard_limits_are_the_controllers_own():
    assert jogd.GUARD_LIMITS_DEG == ((-171.887, 171.887), (-90.012, 90.012), (-114.993, 114.993), (-180.0, 180.0),
                                     (-104.995, 104.995), (-180.0, 180.0))
    assert jogd.GUARD_MARGIN_DEG == 4.0


def test_state_has_goto_and_counts(rig):
    arm = rig.build(poses=PoseStore(str(rig.tmp / "poses.json")))
    assert arm.state()["goto"] is None and arm.state()["counts"] is None
    rig.run(0.6)
    assert arm.state()["counts"] == rig.port.counts and arm.state()["goto"] is None


def test_save_pose_names_selects_and_stores_counts(rig):
    arm, store = teach(rig)
    pose = arm.save_pose()
    assert pose["name"] == "P1" and pose["counts"] == rig.port.counts and store.selected() == "P1"
    assert [round(a, 2) for a in pose["joints"]] == [round(a, 2) for a in rig.port.joints]
    assert arm.save_pose()["name"] == "P2"
    assert arm.save_pose("Home")["name"] == "Home" and arm.selected_pose() == "Home"
    with pytest.raises(RobotError, match="already exists"):
        arm.save_pose("Home")
    assert arm.next_pose() == "A" and arm.next_pose() == "P1"      # saved order, wrapping


def test_save_pose_is_refused_while_moving_and_without_a_fresh_position(rig):
    arm, store = teach(rig)
    rig.run(0.5, V1)
    with pytest.raises(RobotError, match="moving"):
        arm.save_pose()
    rig.run(1.0, [0] * 6)
    assert arm.save_pose()["name"] == "P1"
    rig.clock.sleep(2.0)                                           # no `P` for longer than 1.5 s
    with pytest.raises(RobotError, match="fresh position"):
        arm.save_pose()
    plain = rig.build()
    with pytest.raises(RobotError, match="no pose store"):
        plain.save_pose()


def test_go_to_start_line_keepalive_and_state(rig):
    arm, _ = teach(rig)
    t = rig.clock()
    hold(rig, 0.6)
    counts = [round(a * k) for a, k in zip(TARGET, CPD)]
    expected = ("G " + " ".join(str(c) for c in counts) + " 10\r").encode()
    assert rig.port.gt_lines == [expected] and [d for d in writes_of(rig, t) if d.startswith(b"G")] == [expected]
    gs = [at for at, d in rig.port.sent(t) if d == b"g"]
    assert len(gs) >= 3 and all(0.09 <= b - a <= 0.13 for a, b in zip(gs, gs[1:]))
    goto = arm.state()["goto"]
    assert goto["name"] == "A" and goto["straight"] is False and goto["speed"] == 10 and goto["running"] is True and goto["arrived"] is False
    assert goto["remaining_deg"] == pytest.approx(max(abs(a - b) for a, b in zip(rig.port.joints, TARGET)), abs=0.1)
    assert arm.state()["command"]["kind"] == "goto" and arm.state()["command"]["source"] == "gamepad"
    assert not rig.port.moves(t) and rig.port.mode == "jogd"


def test_straight_go_to_sends_l_from_the_tool_point(rig):
    arm, store = teach(rig)
    pose = store.get("A")
    assert pose["world_hundredths"] is None and pose["world_source"] == "model"
    hold(rig, 0.5, straight=True)
    nums = [pose[k] for k in ("x", "y", "z", "yaw", "pitch", "roll")]
    assert rig.port.gt_lines == [("L " + " ".join(str(int(round(v * 100))) for v in nums) + " 10\r").encode()]
    assert arm.state()["goto"]["straight"] is True and arm.state()["goto"]["running"]


def test_speed_is_goto_speed_capped_by_max_speed(rig):
    arm, _ = teach(rig, goto_speed=40, max_speed=25)
    hold(rig, 0.4)
    assert rig.port.gt_lines[0].endswith(b" 25\r")
    with pytest.raises(RobotError):
        rig.build(goto_speed=0)
    with pytest.raises(RobotError):
        rig.build(goto_speed=100)


def test_arrival_is_reported_once_and_the_same_hold_does_not_restart_it(rig):
    arm, _ = teach(rig)
    rig.port.gt_s = 0.5
    hold(rig, 2.0)
    assert len(rig.port.gt_lines) == 1
    goto = arm.state()["goto"]
    assert goto["arrived"] is True and goto["running"] is False and goto["remaining_deg"] < 0.05
    assert len(said(rig, "arrived")) == 1 and arm.state()["command"] is None
    n = rig.port.g_count
    hold(rig, 0.5)
    assert rig.port.g_count == n and len(rig.port.gt_lines) == 1       # still nothing: the button has not been let go
    rig.run(0.6)                                                       # let go of the button
    hold(rig, 0.6)
    assert len(rig.port.gt_lines) == 2                                 # pressed again: starts again


def test_go_to_stops_when_the_holds_stop(rig):
    arm, _ = teach(rig)
    hold(rig, 0.6)
    t = rig.clock()
    rig.run(0.6)
    seq = writes_of(rig, t)
    assert seq.count(b".") == 1 and CTRL_Z not in seq and rig.port.gt is None
    assert arm.state()["goto"] is None and arm.state()["command"] is None and not arm._dead
    assert b"g" not in seq[seq.index(b"."):]


def test_halt_stops_a_go_to_and_a_held_button_does_not_restart_it(rig):
    arm, _ = teach(rig)
    hold(rig, 0.5)
    t = rig.clock()
    arm.halt()
    rig.run(0.4)
    seq = writes_of(rig, t)
    assert seq.count(b".") == 1 and rig.port.gt is None and arm.state()["goto"] is None
    hold(rig, 0.4)                                                     # halt() let the button count as released: pressing starts again
    assert len(rig.port.gt_lines) == 2


def test_a_stick_wins_over_a_go_to(rig):
    arm, _ = teach(rig)
    hold(rig, 0.5)
    t = rig.clock()
    hold(rig, 0.8, stick=V1)
    seq = writes_of(rig, t)
    assert seq.index(b".") < min(i for i, d in enumerate(seq) if re.fullmatch(rb"[A-Fa-f]\d\d", d))
    assert rig.port.gt is None and rig.port.move and len(rig.port.gt_lines) == 1
    hold(rig, 0.5)                                                     # sticks let go, button still down: the go-to does not start
    assert len(rig.port.gt_lines) == 1 and not rig.port.gt


def test_refusals(rig):
    arm, store = teach(rig)
    store.save("Far", [175.0, 0, 0, 0, 0, 0], [0] * 6)                 # J1 beyond +/-171.887
    store.save("Low", [0, 0, -115.0, 0, 0, 0], [0] * 6)
    for name, text in (("Nope", "no pose"), ("Far", "outside the joint limits"), ("Low", "J3")):
        with pytest.raises(RobotError, match=text):
            arm.goto_hold(name)
    assert not rig.port.gt_lines
    rig.run(0.5, V1)                                                   # a jog in force
    with pytest.raises(RobotError, match="jog"):
        arm.goto_hold("A")
    rig.run(0.8, [0] * 6)
    rig.run(0.5)
    plain = rig.build()
    with pytest.raises(RobotError):
        plain.goto_hold("A")


def test_a_frame_change_refuses_a_go_to(rig):
    arm, _ = teach(rig, frames=ALL_FRAMES)
    arm.set_frame("world")
    with pytest.raises(RobotError, match="frame"):
        arm.goto_hold("A")
    rig.run(4.0)
    arm.goto_hold("A")                                                 # world frame is fine once it is up
    rig.run(0.2)
    assert len(rig.port.gt_lines) == 1


@pytest.mark.parametrize("line,text", [(b"T\r\n", "dead-man"), (b"A -67108870 Path error\r\n", "Path error"),
                                       (b"E 5 joint at its limit\r\n", "refused")])
def test_t_a_and_e_end_a_go_to_and_the_same_hold_does_not_restart_it(rig, line, text):
    arm, _ = teach(rig)
    hold(rig, 0.5)
    rig.port.gt = None
    rig.port.inject(line)
    hold(rig, 0.6)
    assert arm.state()["goto"] is None and text in arm.notice and not arm._dead
    assert len(rig.port.gt_lines) == 1
    rig.run(0.6)
    hold(rig, 0.5)
    assert len(rig.port.gt_lines) == 2


def test_a_malformed_line_answer_is_a_refusal(rig):
    arm, _ = teach(rig)
    hold(rig, 0.3)
    rig.port.gt = None
    rig.port.inject(b"E -22 invalid argument\r\n")
    hold(rig, 0.4)
    assert "E -22" in arm.notice and arm.state()["goto"] is None


def test_no_dash_after_the_stop_is_the_emergency_stop(rig):
    arm, _ = teach(rig, ignore_stop=True)
    arm._attempts = jogd.MAX_RESTARTS                       # left dead for the checks
    hold(rig, 0.5)
    t = rig.clock()
    rig.run(1.8)
    dot = [tt for tt, d in rig.port.sent(t) if d == b"."]
    z = [tt for tt, d in rig.port.sent(t) if d == CTRL_Z]
    assert len(dot) == 1 and len(z) == 1 and 1.5 <= z[0] - dot[0] < 1.52
    assert said(rig, "NO `-`, `T` OR `E` WITHIN 1.5 s") and arm.notice.startswith("jogd is not running")
    assert arm.state()["goto"] is None


def test_save_pose_asks_the_controller_for_its_own_pose(rig):
    arm, store = teach(rig)
    t = rig.clock()
    pose = arm.save_pose("W1")
    assert writes_of(rig, t).count(b"w") == 1
    assert pose["world_hundredths"] == rig.port.world and pose["world_source"] == "controller"
    assert store.get("W1")["world_hundredths"] == rig.port.world
    assert not arm.controller_message and arm.notice == ""            # a `W` line is protocol, not a controller message
    rig.run(0.5)
    hold(rig, 0.5, name="W1", straight=True)                          # the L line carries exactly those numbers
    assert rig.port.gt_lines[-1] == ("L " + " ".join(str(v) for v in rig.port.world) + " 10\r").encode()


def test_save_pose_without_a_w_answer_uses_the_model(rig):
    arm, _ = teach(rig)
    rig.port.has_w = False
    t = rig.clock()
    pose = arm.save_pose("M1")
    assert rig.clock() - t >= 1.0 and pose["world_hundredths"] is None and pose["world_source"] == "model"
    rig.run(0.5)
    hold(rig, 0.5, name="M1", straight=True)
    nums = [int(round(pose[k] * 100)) for k in ("x", "y", "z", "yaw", "pitch", "roll")]
    assert rig.port.gt_lines[-1] == ("L " + " ".join(str(v) for v in nums) + " 10\r").encode()


def test_a_letter_jog_on_jogd_c_sends_the_letter_once_and_then_g(rig):
    arm = rig.build(frames=ALL_FRAMES)
    rig.run(4.0, ZERO)
    assert rig.port.cart
    t = rig.clock()
    rig.run(2.0, [0.6, 0, 0, 0, 0, 0], frame="joint")
    assert len(rig.port.moves(t)) == 1 and rig.port.g_moves >= 15 and arm.state()["moving"]


def test_a_vector_on_jogd_c_sends_the_vector_once_and_then_g(rig):
    arm = cart(rig)
    t = rig.clock()
    rig.run(2.0, [0.6, 0.4, 0, 0, 0, 0], frame="world")
    assert len(rig.port.vectors(t)) == 1 and rig.port.g_moves >= 15 and arm.state()["moving"]
    gs = [t_ for t_, d in rig.port.sent(t) if d == b"g"]
    assert gs and gs[0] > rig.port.vectors(t)[0][0]


def test_the_fake_jogd_c_dead_man_trips_when_g_stops(rig):
    rig.build(frames=ALL_FRAMES)
    port = rig.port
    port.mode, port.cart, port.cmode = "jogd", True, 3
    port.write(b"A06")
    for _ in range(5):                              # `g` keeps it alive past the 0.3 s
        rig.clock.sleep(0.1)
        port.write(b"g")
        port._deadman()
    assert port.move is not None and port.g_moves == 5
    rig.clock.sleep(0.31)                           # `g` stops
    port._deadman()
    assert port.move is None and b"T" in b"".join(d for _, d in port.out)


def test_plain_jogd_still_repeats_the_letter(rig):
    rig.build()
    rig.run(4.0, ZERO)
    t = rig.clock()
    rig.run(1.0, [0.6, 0, 0, 0, 0, 0])
    assert len(rig.port.moves(t)) >= 8 and b"g" not in writes_of(rig, t)


def test_e_minus_4_during_a_move_is_handled_as_an_abort(rig):
    arm = cart(rig)
    rig.run(1.0, [0.6, 0, 0, 0, 0, 0], frame="world")
    assert arm.state()["moving"]
    rig.port.move = None
    rig.port.put(b"E -4 Interrupted system call\r\n")
    rig.run(0.5, [0.6, 0, 0, 0, 0, 0], frame="world")
    assert arm.notice.startswith("the controller stopped the move: Interrupted system call")
    assert not arm.state()["moving"] and not arm._dead


def test_e_minus_4_with_no_move_is_not_an_abort(rig):
    arm = cart(rig)
    rig.port.put(b"E -4 Interrupted system call\r\n")
    rig.run(0.5, ZERO, frame="world")
    assert arm.notice.startswith("the controller reported:") or "stopped the move" not in arm.notice
