"""a465/homezc.py and a465/console.py against a small fake `$` shell (no serial port, no waiting: a FakeClock)."""
import pytest

from a465 import aci, console, homezc
from fake_acid import FakeClock

PROGRAM = b"\x1f\xab\xad\x10" + bytes(range(60))
BUTTON = "Press the PAUSE/CONTINUE button to gain robot point of control.\r\n"


class FakeShell:
    """Echo, `$ ` prompt, cksum, speed, w2, axst (until Ctrl-E), Ctrl-Z, and /tmp/homezc in several moods."""

    def __init__(self, clock, mode="returns", press_after=None, error=None, ctrlz_prompt=True):
        self.clock, self.mode, self.press_after, self.error, self.ctrlz_prompt = clock, mode, press_after, error, ctrlz_prompt
        self.files = {"/tmp/homezc": PROGRAM}
        self.events = []                # [time, bytes]
        self.line = b""
        self.joints = [0.0, 0.0, -88.056, 177.006, 0.0, 0.0]
        self.words = [0x8407] * 6 + [0x8808] * 2
        self.running = None             # (axis, started, kind)
        self.speed = 10
        self.streaming = False
        self.next_stream = 0.0
        self.reset_until = 0.0
        self.sent = []
        self.stopped_by = []
        self.closed = False

    def _put(self, data, delay=0.0):
        self.events.append([self.clock() + delay, data])

    def close(self):
        self.closed = True

    def write(self, data):
        for b in data:
            if b == 0x1a:
                self.sent.append("^Z")
                self.stopped_by.append("^Z")
                self._finish(partial=True)
                self.reset_until = self.clock() + 2.0
                self._put(b"^C equivalent - halting robot...\r\n" + (b"$ " if self.ctrlz_prompt else b""))
            elif b == 0x05:
                self.sent.append("^E")
                self.stopped_by.append("^E")
                self.streaming = False
                self._finish(partial=True)
                self._put(b"$ ")
            elif b == 0x0d:
                self._put(b"\r\n")
                line, self.line = self.line.decode(), b""
                self.sent.append(line)
                self._run(line.strip())
            else:
                self.line += bytes([b])
                self._put(bytes([b]))
        return len(data)

    def _finish(self, partial=False):
        if self.running:
            axis, kind = self.running
            if kind != "button":
                self.joints[axis - 1] += 3.0 if partial else 10.0
            if not partial:
                self.words[axis - 1] |= 0x40 | 0x800
            self.running = None

    def _run(self, line):
        w = line.split()
        if not w:
            self._put(b"$ ")
        elif w[0] == "cksum":
            body = self.files.get(w[1])
            self._put((f"0x{sum(body):08x} {len(body)} {w[1]}\r\n" if body is not None else f"cksum: {w[1]}: file not found\r\n").encode() + b"$ ", 0.3)
        elif w[0] == "speed":
            if self.clock() < self.reset_until:
                self._put(b"ash: device is being reset\r\n$ ")
            elif self.error == "speed":
                self._put(b"speed: error: bad value\r\n$ ")
            elif len(w) == 1:
                self._put(b"Current Speed: %d%% of max.\r\n$ " % self.speed)
            elif self.mode == "speed-button":           # control is held: the button is wanted here, then nothing is printed
                self._put(b"\r\n" + BUTTON.encode(), 0.3)
                self.waiting_since = self.clock()
                if self.press_after is not None and self.press_after <= 180:
                    self.speed = int(w[1])
                    self._put(b"$ ", self.press_after)
            else:
                self.speed = self.speed if self.error == "speed-ignored" else int(w[1])
                self._put(b"$ ")
        elif w[0] == "w2":
            if self.clock() < self.reset_until:
                self._put(b"ash: device is being reset\r\n$ ")
                return
            j = self.joints
            self._put(("\r\nActual Position :\r\n          Axis 1/7   Axis 2/8     Axis 3     Axis 4     Axis 5     Axis 6\r\n"
                       f"JOINTS  {j[0]:10.3f} {j[1]:10.3f} {j[2]:10.3f} {j[3]:10.3f} {j[4]:10.3f} {j[5]:10.3f}\r\n"
                       "             0.000      0.000\r\n\r\nWORLD ...\r\n$ ").encode(), 0.2)
        elif w[0] == "axst":
            self.streaming, self.next_stream = True, self.clock() + 0.2
        elif w[0] == "/tmp/homezc":
            self._program(w[1:])
        else:
            self._put(f"{w[0]}: not found\r\n$ ".encode())

    def _program(self, args):
        if len(args) != 1:
            self._put(b"homezc: invalid command line arguments.\r\nUsage: homezc [newspeed]\r\n$ ", 1.2)
            return
        axis = int(args[0])
        if self.mode == "hangs":
            self.running = (axis, "hang")
            return
        delay = 2.0
        if self.mode == "button":                    # asks for the button at the motion step and waits
            self._put(b"\r\n" + BUTTON.encode(), 0.5)
            self.running = (axis, "button")
            return
        self.joints[axis - 1] += 10.0
        self.words[axis - 1] |= 0x40 | 0x800
        text = b"homezc: motor fault on axis %d\r\n" % axis if self.error == "program" else b""
        self._put(text + b"$ ", delay)

    def read(self, count=1):
        now = self.clock()
        if self.streaming:
            while self.next_stream <= now:
                self.events.append([self.next_stream, (" ".join(f"{x:08x}" for x in self.words) + "\r\n").encode()])
                self.next_stream += 0.2
        self.events.sort(key=lambda e: e[0])
        out = b""
        while self.events and self.events[0][0] <= now and len(out) < count:
            out += self.events.pop(0)[1]
        return out


@pytest.fixture
def rig(tmp_path):
    clock = FakeClock()
    local = tmp_path / "homezc"
    local.write_bytes(PROGRAM)

    def go(*argv, shell=None, upload=None, **kw):
        shell = shell or FakeShell(clock, **kw)
        code = homezc.main(["--local", str(local), *argv], repo=str(tmp_path), open_port=lambda a: shell,
                           clock=clock, sleep=clock.sleep, upload=upload, log_dir=str(tmp_path))
        return code, shell, list(tmp_path.glob("*-homezc.log"))
    go.clock, go.local = clock, local
    return go


def test_parse_helpers():
    assert console.parse_status("junk\r\n" + " ".join(["00008c07"] * 6 + ["00008808"] * 2) + "\r\n") == [0x8c07] * 6 + [0x8808] * 2
    assert console.parse_status("0000 ") is None
    w2 = "x\r\nJOINTS       0.000      0.000    -88.056    177.006      0.000      0.000\r\n             0.000      0.000\r\n"
    assert console.parse_joints(w2) == [0.0, 0.0, -88.056, 177.006, 0.0, 0.0]
    assert console.parse_joints("no joints") is None


def test_check_runs_without_an_axis(rig, tmp_path, capsys):
    code, shell, _ = rig("--check")
    assert code == 0
    assert "the controller loads and runs the program" in capsys.readouterr().out
    assert [s for s in shell.sent if s.startswith("/tmp/homezc")] == ["/tmp/homezc"]
    assert "speed" not in " ".join(shell.sent) and shell.closed


def test_check_reports_unexpected_output(rig, capsys):
    shell = FakeShell(rig.clock)
    shell._program = lambda args: shell._put(b"/tmp/homezc: not an executable\r\n$ ", 0.5)
    code, _, _ = rig("--check", shell=shell)
    out = capsys.readouterr().out
    assert code == 1 and "/tmp/homezc: not an executable" in out and "loads and runs" not in out


def test_normal_run_returns_by_itself(rig, capsys):
    code, shell, _ = rig("6")
    out = capsys.readouterr().out
    assert code == 0 and "^Z" not in shell.sent
    assert "speed 5" in shell.sent and "/tmp/homezc 6" in shell.sent
    assert "JOINT 6 IS ABOUT TO MOVE" in out and "Time limit 8 s" in out
    assert "moved +10.000 degrees" in out
    assert "zero-cross seen (bit 6): no -> yes" in out and "homed (bit 11): no -> yes" in out
    assert "halted by this tool: no" in out
    assert shell.sent.index("speed 5") < shell.sent.index("/tmp/homezc 6")
    assert shell.sent.count("w2") == 2 and shell.sent.count("axst") == 2


def test_run_that_never_returns_is_halted(rig, capsys):
    code, shell, _ = rig("2", "--time-limit", "5", mode="hangs")
    out = capsys.readouterr().out
    assert code == 1 and shell.sent.count("^Z") == 1
    z = shell.sent.index("^Z")
    assert "^E" not in shell.sent[z:shell.sent.index("w2", z)]
    assert "HALTED" in out and "HALTED by this tool: YES" in out
    # the after-readings still happened, after the stop
    assert shell.sent.count("w2") == 2 and shell.sent.index("^Z") < len(shell.sent) - 1
    assert "moved +3.000 degrees" in out


def test_ctrl_e_when_no_prompt_after_halt(rig, capsys):
    code, shell, _ = rig("2", mode="hangs", ctrlz_prompt=False)
    assert code == 1 and shell.sent.count("^Z") == 1 and shell.sent.count("^E") >= 1


def test_button_at_the_speed_step_is_waited_for_then_the_run_goes_on(rig, capsys):
    code, shell, _ = rig("1", "--time-limit", "8", mode="speed-button", press_after=60.0)
    out = capsys.readouterr().out
    assert code == 0 and "^Z" not in shell.sent and "^E" not in shell.sent[shell.sent.index("speed 5"):shell.sent.index("/tmp/homezc 1")]
    assert "PRESS THE PAUSE/CONTINUE BUTTON" in out and "Nothing moves at this step" in out
    assert "speed set to 5% (read back)" in out and "moved +10.000 degrees" in out


def test_button_never_pressed_at_the_speed_step_starts_nothing(rig, capsys):
    code, shell, _ = rig("1", mode="speed-button", press_after=1000.0)
    out = capsys.readouterr().out
    assert code == 1 and "/tmp/homezc 1" not in shell.sent and "^Z" not in shell.sent
    assert "was not pressed within 180 s" in out and "Nothing was started" in out


def test_button_at_the_motion_step_kills_the_program_before_it_moves(rig, capsys):
    code, shell, _ = rig("1", mode="button")
    out = capsys.readouterr().out
    assert code == 1 and "^Z" not in shell.sent
    asked = shell.sent.index("/tmp/homezc 1")
    assert shell.sent[asked + 1] == "^E"                 # killed at once, no waiting
    assert "KILLED before anything moved" in out and "NOTHING MOVED" in out
    assert "moved +0.000 degrees" in out


def test_speed_not_taken_stops_before_the_program(rig, capsys):
    code, shell, _ = rig("4", error="speed-ignored")
    out = capsys.readouterr().out
    assert code == 1 and "/tmp/homezc 4" not in shell.sent and "read back" in out


def test_error_line_from_program_is_nonzero(rig, capsys):
    code, shell, _ = rig("3", error="program")
    out = capsys.readouterr().out
    assert code == 1 and "homezc: motor fault on axis 3" in out and "^Z" not in shell.sent


def test_speed_refused_stops_before_the_program(rig, capsys):
    code, shell, _ = rig("3", error="speed")
    assert code == 1 and not any(s.startswith("/tmp/homezc") for s in shell.sent)


@pytest.mark.parametrize("axis", ["7", "8", "0", "x"])
def test_refused_axis_never_touches_the_port(rig, axis):
    opened = []
    with pytest.raises(SystemExit):
        homezc.main(["--local", str(rig.local), axis], open_port=lambda a: opened.append(a), clock=rig.clock, sleep=rig.clock.sleep)
    assert not opened


def test_check_and_axis_together_refused(rig):
    with pytest.raises(SystemExit):
        homezc.parse_args(["--local", str(rig.local), "--check", "2"], ".")


def test_different_remote_file_is_uploaded_again(rig, capsys):
    shell = FakeShell(rig.clock)
    shell.files["/tmp/homezc"] = b"old"
    uploads = []

    def upload(link, args, say):
        uploads.append(args.remote)
        shell.files[args.remote] = PROGRAM

    code, _, _ = rig("--check", shell=shell, upload=upload)
    assert code == 0 and uploads == ["/tmp/homezc"]
    assert "different" in capsys.readouterr().out


def test_missing_remote_file_uploaded_and_still_wrong_stops(rig, capsys):
    shell = FakeShell(rig.clock)
    del shell.files["/tmp/homezc"]
    code, _, _ = rig("6", shell=shell, upload=lambda link, args, say: None)
    out = capsys.readouterr().out
    assert code == 2 and "missing" in out and "STOPPED" in out
    assert not any(s.startswith("/tmp/homezc") for s in shell.sent)


def test_keyboard_interrupt_sends_ctrl_z_then_reads_again(rig, capsys):
    shell = FakeShell(rig.clock, mode="hangs")
    real = shell.read
    state = {"n": 0}

    def read(count=1):
        if "/tmp/homezc 4" in shell.sent:
            state["n"] += 1
            if state["n"] == 20:
                raise KeyboardInterrupt
        return real(count)
    shell.read = read
    code, _, _ = rig("4", shell=shell)
    out = capsys.readouterr().out
    assert code == 1 and shell.sent.count("^Z") == 1 and shell.sent.count("w2") == 2
    assert "interrupted by the operator" in out
