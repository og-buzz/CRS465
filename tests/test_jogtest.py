"""a465/jogtest.py against a fake controller: a `$` shell that runs the REAL jogd binary in the interpreter
(a465/r3vm.py), with the console bytes and the output lines moving on the tests' FakeClock."""
import pytest

from a465 import jogprog, jogtest, r3vm
from fake_acid import FakeClock

IMAGE = jogprog.build()
NO_DEADMAN_IMAGE = jogprog.build(jogprog.Options(deadman_ms=30_000))
BUTTON = "Press the PAUSE/CONTINUE button to gain robot point of control.\r\n"
CPD = 100.0


class FakeController:
    """modes: button_after=seconds (button request, then the program starts that long after the command),
    echo (bytes sent to jogd are echoed), drop (bytes that never reach jogd), start_error (man_move_start result),
    image (the jogd binary), ctrlz_ends (Ctrl-Z makes jogd exit)."""

    def __init__(self, clock, image=IMAGE, has_file=True, button_after=None, echo=False, drop=b"", start_error=0,
                 ctrlz_ends=False, crlf=True, speed_degrees_per_percent=1.0):
        self.clock, self.image = clock, image
        self.files = {"/tmp/jogd": IMAGE} if has_file else {}
        self.button_after, self.echo, self.drop, self.start_error = button_after, echo, drop, start_error
        self.ctrlz_ends, self.nl = ctrlz_ends, b"\r\n" if crlf else b"\n"
        self.rate = speed_degrees_per_percent
        self.out = []                  # [time, bytes]
        self.line = b""
        self.vm = None
        self.cin = None
        self.sent = []                 # everything written to the port, as bytes in order
        self.to_jogd = bytearray()     # what reached jogd's console
        self.motion_before_ready = []
        self.ctrlz = self.ctrle = 0
        self.joints = [0.0] * 6
        self.moving = {}               # joint -> (sign, speed, vm time)
        self.vm_t0 = 0.0
        self.delivered = 0
        self.exited = False
        self.closed = False
        self.ready_at = None
        self.exit_prompt = False

    # -- port
    def close(self):
        self.closed = True

    def _put(self, data, delay=0.0):
        self.out.append([self.clock() + delay, data])

    def write(self, data):
        self.sent.append(bytes(data))
        for b in data:
            if b in (0x1a, 0x05):
                self._signal(b)
            elif self.vm is not None and not self.exited:
                self._to_jogd(bytes([b]))
            elif b == 0x0d:
                self._put(b"\r\n")
                line, self.line = self.line.decode(), b""
                self._shell(line.strip())
            else:
                self.line += bytes([b])
                self._put(bytes([b]))
        return len(data)

    def _signal(self, b):
        if b == 0x1a:
            self.ctrlz += 1
            self.moving.clear()
            if self.vm is not None and not self.exited and self.ctrlz_ends:
                self._exit()
        else:
            self.ctrle += 1
            if self.vm is not None and not self.exited:
                self._exit()
            else:
                self._put(b"$ ")

    def _exit(self):
        self.exited = True
        self._put(b"\r\n$ ")

    def _to_jogd(self, b):
        self._sync()
        if self.echo:
            self._put(b)
        if b in self.drop:
            return
        self.to_jogd += b
        self.cin.pending.append(((self.clock() - self.vm_t0) * 1000, b))

    # -- shell
    def _shell(self, line):
        w = line.split()
        if not w:
            self._put(b"$ ")
        elif w[0] == "cksum":
            body = self.files.get(w[1])
            self._put((f"0x{sum(body):08x} {len(body)} {w[1]}\r\n" if body is not None else f"cksum: {w[1]}: file not found\r\n").encode() + b"$ ", 0.3)
        elif w[0] == "w2":
            j = self.joints
            self._put(("\r\nActual Position :\r\n          Axis 1/7   Axis 2/8     Axis 3     Axis 4     Axis 5     Axis 6\r\n"
                       f"PULSES  {' '.join(f'{round(x * CPD):+10d}' for x in j)}\r\n          +0 +0\r\n"
                       f"JOINTS  {' '.join(f'{x:10.3f}' for x in j)}\r\n             0.000      0.000\r\n\r\nWORLD ...\r\n$ ").encode(), 0.2)
        elif w[0] == "/tmp/jogd":
            self._launch(w)
        else:
            self._put(f"{w[0]}: not found\r\n$ ".encode())

    def _launch(self, w):
        self.cin = r3vm.Input()
        self.vm = r3vm.VM(self.image, argv=w, libs={"robotlib": r3vm.FakeRobot(self._results()), "syslib": r3vm.FakeSyslib({13: "not the controlling client"})},
                          files={"/dev/cin": self.cin})
        self.vm.start()
        self.exited = False
        self.delivered = 0
        self.vm_t0 = self.clock()
        if self.button_after is not None:
            self._put(b"\r\n" + BUTTON.encode(), 0.4)
            self.vm_t0 += self.button_after

    def _results(self):
        def speed_get(vm, args):
            vm.store(args[0], 0, 40)
            return 0

        def start(vm, args):
            if self.ready_at is None:
                self.motion_before_ready.append(vm.clock)
            if self.start_error:
                return self.start_error
            joint, speed = jogprog_s32(args[0]), jogprog_s32(args[1])
            self._close_move(abs(joint), vm.clock)
            self.moving[abs(joint)] = (1 if joint > 0 else -1, speed, vm.clock)
            return 0

        def stop(vm, args):
            self._close_move(jogprog_s32(args[0]), vm.clock)
            return 0
        return {0x43: speed_get, 0x53: start, 0x54: stop}

    def _close_move(self, joint, now):
        if joint in self.moving:
            sign, speed, t = self.moving.pop(joint)
            self.joints[joint - 1] += sign * speed * self.rate * (now - t) / 1000

    # -- the interpreter follows the tests' clock
    def _target(self):
        return (self.clock() - self.vm_t0) * 1000

    def _sync(self):
        if self.vm is None or self.exited:
            return
        target = self._target()
        try:
            while self.vm.clock <= target:
                self.vm.step()
        except r3vm.ProgramExit:
            self.exited = True
            self.exit_prompt = True

    def _deliver(self):
        if self.vm is None:
            return
        target = self._target()
        if self.exit_prompt:                       # the program has ended: its last lines, then the prompt
            target = self.vm.clock
        while self.delivered < len(self.vm.events) and self.vm.events[self.delivered][0] <= target:
            t, fd, text = self.vm.events[self.delivered]
            self.delivered += 1
            if self.ready_at is None and text.startswith("jogd ready"):
                self.ready_at = t
            self.out.append([self.vm_t0 + t / 1000, text.replace("\n", "\r\n" if self.nl == b"\r\n" else "\n").encode()])
        if self.exit_prompt:
            self.exit_prompt = False
            self.out.append([self.vm_t0 + self.vm.clock / 1000, b"\r\n$ "])

    def read(self, count=1):
        self._sync()
        self._deliver()
        self.out.sort(key=lambda e: e[0])
        data = b""
        while self.out and self.out[0][0] <= self.clock() and len(data) < count:
            data += self.out.pop(0)[1]
        return data


def jogprog_s32(v):
    return v - (1 << 32) if v & 0x80000000 else v


@pytest.fixture
def rig(tmp_path):
    clock = FakeClock()
    local = tmp_path / "jogd"
    local.write_bytes(IMAGE)

    def go(*argv, upload=None, **kw):
        ctl = FakeController(clock, **kw)
        code = jogtest.main(["--local", str(local), *argv], repo=str(tmp_path), open_port=lambda a: ctl,
                            clock=clock, sleep=clock.sleep, upload=upload, log_dir=str(tmp_path))
        go.text = "\n".join(p.read_text() for p in sorted(tmp_path.glob("*-jogtest.log")))
        return code, ctl
    go.clock, go.local, go.tmp = clock, local, tmp_path
    return go


def commands_sent(ctl):
    """Bytes the tool wrote after the line that started jogd."""
    data = b"".join(ctl.sent)
    if b"/tmp/jogd 1" not in data:
        return b""
    return data.split(b"/tmp/jogd 1", 1)[1].split(b"\r", 1)[1].split(b"w2\r")[0]


def motion_bytes(ctl):
    return [b for b in commands_sent(ctl) if chr(b) in "ABCDEFabcdef"]


# ------------------------------------------------------------------------------------------- dry run

@pytest.mark.parametrize("echo", [False, True])
def test_dry_run_is_clean_and_sends_no_motion(rig, echo):
    code, ctl = rig("--dry", echo=echo)
    assert code == 0, rig.text
    assert commands_sent(ctl) == b"q"
    assert not ctl.ctrlz and not ctl.ctrle
    assert "jogd ready" in rig.text and "DRY RUN CLEAN" in rig.text
    assert "/tmp/jogd 1 5 s" in b"".join(ctl.sent).decode()
    assert ctl.closed


def test_dry_run_with_the_button_request_waits_and_sends_nothing_meanwhile(rig):
    code, ctl = rig("--dry", "--variant", "r", "--max", "3", button_after=40.0)
    assert code == 0, rig.text
    assert "WAITING: PRESS THE PAUSE/CONTINUE BUTTON" in rig.text
    assert commands_sent(ctl) == b"q"
    assert "/tmp/jogd 1 3 r" in b"".join(ctl.sent).decode()


def test_dry_run_button_never_pressed_kills_with_ctrl_e_only(rig):
    code, ctl = rig("--dry", button_after=10_000.0)
    assert code == 1
    assert commands_sent(ctl).startswith(b"\x05")
    assert ctl.ctrle == 1 and ctl.ctrlz == 0
    assert "no `jogd ready` within 180 s" in rig.text


def test_dry_run_reports_a_change_of_the_position(rig):
    class Moving(FakeController):
        def _shell(self, line):
            if line.strip() == "/tmp/jogd 1 5 s":
                self.joints[0] += 0.5
            super()._shell(line)
    clock = rig.clock
    ctl = Moving(clock)
    code = jogtest.main(["--local", str(rig.local), "--dry"], repo=str(rig.tmp), open_port=lambda a: ctl, clock=clock,
                        sleep=clock.sleep, log_dir=str(rig.tmp))
    assert code == 1


# ------------------------------------------------------------------------------------------ motion

@pytest.mark.parametrize("echo", [False, True])
def test_test_1_lines_and_times(rig, echo):
    code, ctl = rig("F", "--no-deadman-test", echo=echo)
    assert code == 0, rig.text
    sent = commands_sent(ctl)
    assert sent.startswith(b"F02" * 10 + b".") and sent.endswith(b"q")
    assert ctl.joints[5] == pytest.approx(2 * 1.0, abs=0.3)          # 2 % for about a second
    assert "test 1: start and stop: as expected" in rig.text
    assert "keep-alive gaps: max" in rig.text and "none late" in rig.text
    assert "joint 6: +" in rig.text and "counts" in rig.text
    assert ctl.motion_before_ready == []


def test_negative_letter_moves_the_other_way_and_lf_only_line_ends_work(rig):
    code, ctl = rig("f", "--no-deadman-test", "--hold", "0.5", crlf=False)
    assert code == 0, rig.text
    assert ctl.joints[5] < -0.5


def test_deadman_is_observed(rig):
    code, ctl = rig("C", "--hold", "0.5")
    assert code == 0, rig.text
    assert "test 2: dead-man: as expected" in rig.text
    assert "`T` arrived" in rig.text
    sent = commands_sent(ctl)
    assert b"c02" in sent and b"." in sent
    import re
    ms = float(re.search(r"`T` arrived ([\d.]+) ms after the last keep-alive", rig.text).group(1))
    assert 290 <= ms <= 330


def test_deadman_missing_is_reported_and_dot_sent(rig):
    code, ctl = rig("C", "--hold", "0.3", image=NO_DEADMAN_IMAGE)
    assert code == 1
    assert "THE DEAD-MAN DID NOT FIRE" in rig.text
    sent = commands_sent(ctl)
    assert sent.count(b".") == 2                                   # one after test 1, one after the missing T
    assert sent.endswith(b"q") and not ctl.ctrlz


@pytest.mark.parametrize("variant,expected", [("s", "['+B02', '-', '+B04', '-']"), ("r", "['+B02', '+B04', '-']")])
def test_speed_change_both_variants(rig, variant, expected):
    code, ctl = rig("B", "--speed2", "4", "--variant", variant, "--no-deadman-test", "--hold", "0.4")
    assert code == 0, rig.text
    assert f"expected lines {expected}, got {expected}" in rig.text
    assert "test 3: speed change: as expected" in rig.text
    assert b"B04" in commands_sent(ctl)


def test_error_on_start_stops_the_sequence_and_quits(rig):
    code, ctl = rig("A", start_error=-13)
    assert code == 1
    sent = commands_sent(ctl)
    assert sent.endswith(b".q") and sent.count(b"A02") <= 3
    assert b"a02" not in sent                                       # test 2 never started
    assert "E -13 not the controlling client" in rig.text
    assert "STOPPING" in rig.text
    assert not ctl.ctrlz and not ctl.ctrle


def test_no_answer_to_a_start_sends_dot_and_q(rig):
    code, ctl = rig("A", drop=b"ABCDEFabcdef")
    assert code == 1
    sent = commands_sent(ctl)
    assert sent.endswith(b".q")
    assert "NoAnswer" in rig.text
    n = sent.count(b"A02")
    assert 14 <= n <= 17                                           # 1.5 s of keep-alives


def test_prompt_not_back_after_q_leads_to_ctrl_z_then_ctrl_e(rig):
    code, ctl = rig("--dry", drop=b"q")
    assert code == 1
    assert ctl.ctrlz == 1 and ctl.ctrle == 1
    assert "NO `$` PROMPT" in rig.text and "sending Ctrl-E" in rig.text
    assert "Ctrl-Z sent: YES; Ctrl-E sent: YES" in rig.text


def test_ctrl_z_alone_when_the_prompt_comes_back_after_it(rig):
    code, ctl = rig("--dry", drop=b"q", ctrlz_ends=True)
    assert code == 1
    assert ctl.ctrlz == 1 and ctl.ctrle == 0


def test_keyboard_interrupt_writes_ctrl_z_first_then_dot_then_q(rig):
    class Interrupting(FakeController):
        def write(self, data):
            if data == b"F02" and len([s for s in self.sent if s == b"F02"]) == 3:
                raise KeyboardInterrupt
            return super().write(data)
    clock = rig.clock
    ctl = Interrupting(clock)
    code = jogtest.main(["--local", str(rig.local), "F"], repo=str(rig.tmp), open_port=lambda a: ctl, clock=clock,
                        sleep=clock.sleep, log_dir=str(rig.tmp))
    assert code == 1
    sent = commands_sent(ctl)
    assert sent.endswith(b"\x1a.q")
    assert ctl.ctrlz == 1


def test_late_keepalives_are_reported(rig):
    clock = rig.clock

    class Slow(FakeController):
        def write(self, data):
            if data == b"F02" and len([s for s in self.sent if s == b"F02"]) == 4:
                clock.now += 0.2
            return super().write(data)
    ctl = Slow(clock)
    code = jogtest.main(["--local", str(rig.local), "F", "--no-deadman-test"], repo=str(rig.tmp), open_port=lambda a: ctl,
                        clock=clock, sleep=clock.sleep, log_dir=str(rig.tmp))
    assert "LATE" in "\n".join(p.read_text() for p in rig.tmp.glob("*-jogtest.log"))


# ------------------------------------------------------------------------------ arguments, upload

@pytest.mark.parametrize("argv", [
    ["F", "--max", "0"], ["F", "--max", "11"], ["F", "--speed", "0"], ["F", "--speed", "6"],
    ["F", "--speed", "2", "--speed2", "6"], ["F", "--speed2", "2"], ["F", "--hold", "0.1"], ["F", "--hold", "3.1"],
    ["G"], ["FF"], [], ["--dry", "F"], ["F", "--variant", "x"],
])
def test_bad_arguments_are_refused_before_the_port_is_opened(rig, argv):
    opened = []
    with pytest.raises(SystemExit):
        jogtest.main(["--local", str(rig.local), *argv], repo=str(rig.tmp), open_port=lambda a: opened.append(a),
                     clock=rig.clock, sleep=rig.clock.sleep, log_dir=str(rig.tmp))
    assert opened == []


def test_limits_accept_the_edges(rig):
    code, ctl = rig("F", "--max", "10", "--speed", "10", "--speed2", "1", "--hold", "0.2", "--no-deadman-test")
    assert code == 0, rig.text
    assert b"F10" in commands_sent(ctl) and b"F01" in commands_sent(ctl)


def test_program_missing_on_the_controller_is_uploaded(rig):
    uploaded = []

    def upload(link, args, say):
        uploaded.append(args.remote)
        ctl.files[args.remote] = IMAGE
    ctl = FakeController(rig.clock, has_file=False)
    code = jogtest.main(["--local", str(rig.local), "--dry"], repo=str(rig.tmp), open_port=lambda a: ctl, clock=rig.clock,
                        sleep=rig.clock.sleep, upload=upload, log_dir=str(rig.tmp))
    assert code == 0
    assert uploaded == ["/tmp/jogd"]


def test_a_failed_upload_stops_before_jogd_is_started(rig):
    ctl = FakeController(rig.clock, has_file=False)
    code = jogtest.main(["--local", str(rig.local), "F"], repo=str(rig.tmp), open_port=lambda a: ctl, clock=rig.clock,
                        sleep=rig.clock.sleep, upload=lambda link, args, say: None, log_dir=str(rig.tmp))
    assert code == 2
    assert b"/tmp/jogd 1" not in b"".join(ctl.sent)
