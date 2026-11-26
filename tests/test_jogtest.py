"""a465/jogtest.py against a fake controller: a `$` shell that runs the REAL jogd binary in the interpreter
(a465/r3vm.py), with the console bytes and the output lines moving on the tests' FakeClock."""
import pytest

from a465 import home, jogprog, jogtest, r3vm
from fake_acid import FakeClock

IMAGE = jogprog.build()
NO_DEADMAN_IMAGE = jogprog.build(jogprog.Options(deadman_ms=30_000))
BUTTON = "Press the PAUSE/CONTINUE button to gain robot point of control.\r\n"
CPD = 100.0
SAMPLE_CPD = home.COUNTS_PER_DEGREE
FAST = 100.0                           # degrees/s of the reversal fault's lunge


class FakeController:
    """modes: button_after=seconds (button request, then the program starts that long after the command),
    echo (bytes sent to jogd are echoed), drop (bytes that never reach jogd), start_error (man_move_start result),
    image (the jogd binary), ctrlz_ends (Ctrl-Z makes jogd exit)."""

    def __init__(self, clock, image=IMAGE, has_file=True, button_after=None, echo=False, drop=b"", start_error=0,
                 ctrlz_ends=False, crlf=True, speed_degrees_per_percent=1.0, fault=False, p_delay=0.095):
        self.clock, self.image = clock, image
        self.files = {"/tmp/jogd": IMAGE} if has_file else {}
        self.button_after, self.echo, self.drop, self.start_error = button_after, echo, drop, start_error
        self.ctrlz_ends, self.nl = ctrlz_ends, b"\r\n" if crlf else b"\n"
        self.rate = speed_degrees_per_percent
        self.fault, self.p_delay = fault, p_delay      # fault: a reversal jumps to the mirror of its start, then creeps
        self.last_sign = {}
        self.p_count = 0
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
        if b == b"p":                       # the fake answers `p` itself (the interpreter has no location data)
            self.p_count += 1
            pos = [self._pos(j, self._target()) for j in range(1, 7)]
            counts = [round(x * SAMPLE_CPD[j]) for j, x in zip(range(1, 7), pos)]
            self._put(("P " + " ".join(str(c) for c in counts)).encode() + self.nl, self.p_delay)
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
        elif w[0].startswith("/tmp/jogd"):
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
            sign = 1 if joint > 0 else -1
            self._close_move(abs(joint), vm.clock)
            p0 = self.joints[abs(joint) - 1]
            fast = -p0 if self.fault and self.last_sign.get(abs(joint), sign) != sign else None
            self.last_sign[abs(joint)] = sign
            self.moving[abs(joint)] = (sign, speed, vm.clock, p0, fast)
            return 0

        def stop(vm, args):
            self._close_move(jogprog_s32(args[0]), vm.clock)
            return 0
        return {0x43: speed_get, 0x53: start, 0x54: stop}

    def _close_move(self, joint, now):
        if joint in self.moving:
            self.joints[joint - 1] = self._pos(joint, now)
            self.moving.pop(joint)

    def _pos(self, joint, now):
        """Degrees of a joint at VM time `now` (ms)."""
        if joint not in self.moving:
            return self.joints[joint - 1]
        sign, speed, t, p0, fast = self.moving[joint]
        dt = max(now - t, 0) / 1000
        if fast is None:
            return p0 + sign * speed * self.rate * dt
        t_fast = abs(fast - p0) / FAST
        if dt <= t_fast:
            return p0 + (1 if fast > p0 else -1) * FAST * dt
        return fast + sign * speed * self.rate * (dt - t_fast)

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


# --------------------------------------------------------------------------- sampling, reversal, travel limit

def test_sampled_start_stop_prints_a_table_and_speeds(rig):
    code, ctl = rig("F", "--sample", "--no-deadman-test", "--speed", "5", "--hold", "1.0")
    assert code == 0, rig.text
    assert ctl.p_count >= 8
    assert "samples of joint 6" in rig.text and "deg/s since previous" in rig.text
    assert "phase F05: travel +" in rig.text
    assert "largest speed in this test: +" in rig.text
    import re
    mean = float(re.search(r"phase F05: travel [+-][\d.]+ degrees in \d+ ms, mean ([+-][\d.]+) deg/s", rig.text).group(1))
    assert mean == pytest.approx(-5.0 * 1.0 * 1.0, abs=1.5) or abs(mean) > 3        # joint 6 counts per degree are negative
    assert commands_sent(ctl).count(b"p") >= 8


def test_samples_never_delay_keepalives(rig):
    code, ctl = rig("F", "--sample", "--no-deadman-test", "--hold", "2.0")
    assert code == 0, rig.text
    assert "none late" in rig.text


def test_samples_are_not_written_without_the_option(rig):
    code, ctl = rig("F", "--no-deadman-test")
    assert code == 0 and ctl.p_count == 0


def test_p_never_answered_is_reported_and_the_test_goes_on(rig):
    code, ctl = rig("F", "--sample", "--no-deadman-test", drop=b"p")
    assert code == 0, rig.text
    assert "`p` WAS NOT ANSWERED" in rig.text
    assert b"p" not in commands_sent(ctl).split(b"F02", 1)[1]


def test_dry_run_with_sampling_still_sends_no_motion(rig):
    code, ctl = rig("--dry", "--sample")
    assert code == 0, rig.text
    assert motion_bytes(ctl) == [] and ctl.p_count >= 1


def test_reversal_test_reports_four_moves(rig):
    code, ctl = rig("F", "--reverse", "--sample", "--speed", "5", "--hold", "1.0")
    assert code == 0, rig.text
    sent = commands_sent(ctl)
    order = [sent.index(b"F05"), sent.index(b"f05"), sent.rindex(b"F05")]
    assert order == sorted(order)
    assert sent.count(b".") == 4 and sent.endswith(b"q")
    assert rig.text.count("MOVE REPORT") == 4
    for tag in "abcd":
        assert f"reverse ({tag})" in rig.text
    assert rig.text.count("crossed the starting position: no") == 4         # an ideal reversal returns to the start, not past it
    assert "test 1:" not in rig.text


def test_reversal_fault_is_visible_in_the_report(rig):
    code, ctl = rig("F", "--reverse", "--sample", "--speed", "5", "--hold", "1.0", "--max-travel", "30", fault=True, p_delay=0.0)
    assert code == 0, rig.text
    import re
    peaks = [float(x) for x in re.findall(r"peak ([+-][\d.]+) deg/s", rig.text)]
    assert len(peaks) == 4 and max(abs(x) for x in peaks) > 20           # the lunge is reported, not judged
    assert "crossed the starting position: YES" in rig.text


def test_travel_limit_writes_dot_at_once_and_ends_the_sequence(rig):
    code, ctl = rig("F", "--sample", "--speed", "5", "--hold", "3.0", "--max-travel", "3")
    assert code == 1
    assert "TRAVEL LIMIT" in rig.text and "TravelLimit" in rig.text
    sent = commands_sent(ctl)
    assert sent.endswith(b".q") or sent.endswith(b"..q") or b".q" in sent[-4:]
    assert b"f05" not in sent                                            # test 2 never started
    assert abs(ctl.joints[5]) < 3 + 5 * 1.0                              # stopped soon after the limit
    assert not ctl.ctrlz


def test_travel_limit_catches_the_reversal_lunge(rig):
    code, ctl = rig("F", "--reverse", "--sample", "--speed", "5", "--hold", "1.0", "--max-travel", "6", fault=True, p_delay=0.0)
    assert code == 1
    assert "TRAVEL LIMIT" in rig.text
    assert commands_sent(ctl).count(b"F05") < 30


@pytest.mark.parametrize("argv", [["F", "--reverse", "--speed2", "3"], ["--dry", "--reverse"], ["F", "--max-travel", "0"],
                                  ["F", "--max-travel", "91"]])
def test_new_option_limits(rig, argv):
    opened = []
    with pytest.raises(SystemExit):
        jogtest.main(["--local", str(rig.local), *argv], repo=str(rig.tmp), open_port=lambda a: opened.append(a),
                     clock=rig.clock, sleep=rig.clock.sleep, log_dir=str(rig.tmp))
    assert opened == []


def test_program_and_remote_name_options(rig):
    other = rig.tmp / "jogd-b"
    other.write_bytes(IMAGE)
    ctl = FakeController(rig.clock)
    ctl.files["/tmp/jogd-b"] = IMAGE
    code = jogtest.main(["--program", str(other), "--remote-name", "/tmp/jogd-b", "--dry"], repo=str(rig.tmp),
                        open_port=lambda a: ctl, clock=rig.clock, sleep=rig.clock.sleep, log_dir=str(rig.tmp))
    assert code == 0
    assert "/tmp/jogd-b 1 5 s" in b"".join(ctl.sent).decode()


# ---- --maxvel: jogd-v, the moving joint's maxvels entry changed mid-jog

VIMAGE = jogprog.build(jogprog.BUILDS["jogd-v"])
START_MV = [180.0, 180.0, 180.0, 171.089, 172.8, 172.089, 2368.57, 350.0]


class MaxvelController(FakeController):
    """Runs jogd-v. maxvels_get answers START_MV; a maxvels_set while a joint moves scales that joint's
    speed by new / start value from that moment (applies=True), or changes nothing (applies=False)."""

    def __init__(self, clock, applies=True, **kw):
        super().__init__(clock, image=VIMAGE, **kw)
        self.files = {"/tmp/jogd-v": VIMAGE}
        self.applies = applies
        self.sets = []

    def _results(self):
        res = super()._results()

        def get(vm, args):
            for k, x in enumerate(START_MV):
                vm.store(args[0], k, r3vm.from_float(x))
            return 0

        def put(vm, args):
            vals = [r3vm.to_float(vm.load(args[0], k)) for k in range(8)]
            self.sets.append([round(v, 3) for v in vals])
            if self.applies:
                for j, (sign, speed, t, p0, fast) in list(self.moving.items()):
                    self.moving[j] = (sign, speed * vals[j - 1] / START_MV[j - 1], vm.clock, self._pos(j, vm.clock), None)
            return 0
        res.update({0x15: get, 0x16: put})
        return res


@pytest.fixture
def vrig(tmp_path):
    clock = FakeClock()
    local = tmp_path / "jogd-v"
    local.write_bytes(VIMAGE)

    def go(*argv, **kw):
        ctl = MaxvelController(clock, **kw)
        code = jogtest.main(["--local", str(local), *argv], repo=str(tmp_path), open_port=lambda a: ctl,
                            clock=clock, sleep=clock.sleep, log_dir=str(tmp_path))
        go.text = "\n".join(p.read_text() for p in sorted(tmp_path.glob("*-jogtest.log")))
        return code, ctl
    return go


def maxvel_ratio(text):
    import re
    found = re.search(r"MAXVEL REPORT: .*speed before ([-+\d.]+) deg/s .* after ([-+\d.]+) deg/s .* ratio ([\d.]+)", text)
    return tuple(float(x) for x in found.groups())


def test_maxvel_applied_mid_jog_halves_the_sampled_speed(vrig):
    code, ctl = vrig("F", "--maxvel", "50", "--speed", "5", "--hold", "1.5")
    assert code == 0, vrig.text
    data = b"".join(ctl.sent)
    assert b"/tmp/jogd-v 1 5 s\r" in data and b"v50" in data
    assert data.index(b"v50") > data.index(b"F05")
    before, after, ratio = maxvel_ratio(vrig.text)
    assert before == pytest.approx(5.0, rel=0.1) and ratio == pytest.approx(0.5, abs=0.06)
    assert ctl.sets == [[180.0, 180.0, 180.0, 171.089, 172.8, 86.045, 2368.57, 350.0], START_MV]   # set, then restored
    assert "maxvel: F05, v50 after 1.5 s" in vrig.text and "as expected" in vrig.text


def test_maxvel_not_applied_shows_no_change(vrig):
    code, ctl = vrig("c", "--maxvel", "30", "--speed", "4", "--hold", "1.0", applies=False)
    assert code == 0, vrig.text
    before, after, ratio = maxvel_ratio(vrig.text)
    assert ratio == pytest.approx(1.0, abs=0.06)
    assert ctl.sets[-1] == START_MV


@pytest.mark.parametrize("argv", [["--maxvel", "50"], ["--dry", "--maxvel", "50"], ["F", "--maxvel", "50", "--reverse"],
                                  ["F", "--maxvel", "50", "--speed2", "3"], ["F", "--maxvel", "0"], ["F", "--maxvel", "100"]])
def test_maxvel_arguments_are_checked_before_the_port(vrig, argv, tmp_path):
    opened = []
    (tmp_path / "jv").write_bytes(VIMAGE)
    with pytest.raises(SystemExit):
        jogtest.main(["--local", str(tmp_path / "jv"), *argv], repo=str(tmp_path), open_port=lambda a: opened.append(a))
    assert not opened


def test_maxvel_defaults_to_the_jogd_v_files(tmp_path):
    (tmp_path / "controller" / "programs").mkdir(parents=True)
    (tmp_path / "controller" / "programs" / "jogd-v").write_bytes(VIMAGE)
    args = jogtest.parse_args(["F", "--maxvel", "40"], str(tmp_path))
    assert args.remote == "/tmp/jogd-v" and args.local.endswith("controller/programs/jogd-v") and args.sample


def test_dry_run_of_jogd_v_is_clean(vrig, tmp_path):
    code, ctl = vrig("--dry", "--remote", "/tmp/jogd-v")
    assert code == 0, vrig.text
    assert "jogd line (MV): MV 180 180 180 171.089 172.8 172.089 2368.57 350" in vrig.text and ctl.sets == []


# ---- modes 3 and 4: a letter moves the fake joints by a fixed joint-space step per percent and second

import re

import numpy as np

from a465 import model

# joint velocity (degrees per second per percent of speed) for each letter in the fake's mode 3 / 4
DIRS = {1: [1, 0, 0, 0, 0, 0], 2: [0, 1, 0.5, 0, 0, 0], 3: [0, 0, 1, 0, 0, 0],
        4: [0, 0, 0, 1, 0, 0], 5: [0, 0, 0, 0, 1, 0], 6: [0, 0, 0, 0, 0, 1]}
START6 = [0.0, -30.0, 60.0, 0.0, 30.0, 0.0]


class SixController(FakeController):
    """A letter (axis 1..6) moves ALL the fake joints along DIRS[axis]; `fail_axis` makes that axis's start return -13."""

    def __init__(self, clock, fail_axis=None, **kw):
        super().__init__(clock, **kw)
        self.joints = list(START6)
        self.fail_axis = fail_axis

    def _results(self):
        res = super()._results()

        def start(vm, args):
            joint, speed = jogprog_s32(args[0]), jogprog_s32(args[1])
            if abs(joint) == self.fail_axis:
                return -13
            self._close_move(abs(joint), vm.clock)
            self.moving.clear()
            self.moving[abs(joint)] = (1 if joint > 0 else -1, speed, vm.clock, list(self.joints), None)
            return 0
        res[0x53] = start
        return res

    def _close_move(self, joint, now):
        if joint in self.moving:
            self.joints = [self._pos(j, now) for j in range(1, 7)]
            self.moving.pop(joint)

    def _pos(self, joint, now):
        if not self.moving:
            return self.joints[joint - 1]
        (axis, (sign, speed, t, p0, _)), = self.moving.items()
        return p0[joint - 1] + sign * speed * self.rate * max(now - t, 0) / 1000 * DIRS[axis][joint - 1]


@pytest.fixture
def six(tmp_path):
    clock = FakeClock()
    local = tmp_path / "jogd"
    local.write_bytes(IMAGE)

    def go(*argv, **kw):
        ctl = SixController(clock, **kw)
        start = list(ctl.joints)
        code = jogtest.main(["--local", str(local), *argv], repo=str(tmp_path), open_port=lambda a: ctl,
                            clock=clock, sleep=clock.sleep, log_dir=str(tmp_path))
        go.text = "\n".join(p.read_text() for p in sorted(tmp_path.glob("*-jogtest.log")))
        go.start = start
        return code, ctl
    return go


def sent6(ctl, mode=3):
    data = b"".join(ctl.sent)
    return data.split(f"/tmp/jogd {mode} ".encode(), 1)[1].split(b"\r", 1)[1].split(b"w2\r")[0]


def nums(text, label, n=3):
    found = re.search(label + r"((?:\s+[+-]?\d+\.\d+){%d})" % n, text)
    return [float(x) for x in found.group(1).split()]


def test_mode_reaches_the_jogd_command_line_and_the_plan_is_printed_first(six):
    code, ctl = six("--mode", "3", "A")
    assert code == 0, six.text
    assert "/tmp/jogd 3 5 s\r" in b"".join(ctl.sent).decode()
    assert "the exact jogd command line: /tmp/jogd 3 5 s" in six.text
    assert "limits in force" in six.text and "40 mm" in six.text and "10 degrees" in six.text
    assert six.text.index("limits in force") < six.text.index("starting: /tmp/jogd 3 5 s")
    code, ctl = six("--mode", "4", "--dry", "--max", "3", "--variant", "r")
    assert code == 0, six.text
    assert "/tmp/jogd 4 3 r\r" in b"".join(ctl.sent).decode()
    assert ctl.p_count >= 1 and b"A" not in sent6(ctl, 4)


def test_mode3_move_reports_the_displacement_the_model_gives(six):
    code, ctl = six("--mode", "3", "B", "--speed", "2")
    assert code == 0, six.text
    sent = sent6(ctl)
    assert b"p" in sent and sent.endswith(b"q")
    assert sent.replace(b"p", b"").startswith(b"B02" * 10 + b".")
    p0 = model.forward(six.start)[0]
    p1 = model.forward(ctl.joints)[0]
    assert np.linalg.norm(p1 - p0) > 5
    found = re.search(r"dx ([+-][\d.]+)  dy ([+-][\d.]+)  dz ([+-][\d.]+) mm; length ([\d.]+) mm", six.text)
    got = np.array([float(x) for x in found.groups()[:3]])
    assert got == pytest.approx(p1 - p0, abs=0.15)
    assert float(found.group(4)) == pytest.approx(np.linalg.norm(p1 - p0), abs=0.15)
    dj = re.search(r"joint change \(degrees, from counts\): (.*)", six.text).group(1)
    assert [float(x) for x in re.findall(r"J\d ([+-][\d.]+)", dj)] == pytest.approx([b - a for a, b in zip(six.start, ctl.joints)], abs=0.01)
    ypr0, ypr1 = model.yaw_pitch_roll(model.forward(six.start)[1]), model.yaw_pitch_roll(model.forward(ctl.joints)[1])
    m = re.search(r"yaw / pitch / roll: ([+-][\d.]+) / ([+-][\d.]+) / ([+-][\d.]+)", six.text)
    assert [float(x) for x in m.groups()] == pytest.approx([b - a for a, b in zip(ypr0, ypr1)], abs=0.02)
    assert re.search(r"mean tool-point speed [\d.]+ mm/s, mean rotation rate [\d.]+ deg/s", six.text)
    assert "VERDICT: " in six.text
    assert "from the PC's model (a465/model.py), whose frame has not been checked against the controller's." in six.text
    assert "tool's own frame" not in six.text                      # tool mode only


def test_mode4_move_also_reports_the_tool_frame_displacement(six):
    code, ctl = six("--mode", "4", "C")
    assert code == 0, six.text
    p0, r0, _, _ = model.forward(six.start)
    d = model.forward(ctl.joints)[0] - p0
    m = re.search(r"tool's own frame at the start \(rot.T @ d\): ([+-][\d.]+)  ([+-][\d.]+)  ([+-][\d.]+) mm", six.text)
    assert [float(x) for x in m.groups()] == pytest.approx(list(r0.T @ d), abs=0.15)


def test_the_verdict_names_what_dominated(six):
    code, ctl = six("--mode", "3", "E", "--speed", "4")          # wrist pitch joint: mostly rotation at the tool point
    assert code == 0, six.text
    line = re.search(r"VERDICT: (.*)", six.text).group(1)
    assert line.startswith(("mostly", "mixed")), line
    assert "deg" in line and "mm" in line


def test_verdict_for_a_pure_translation_and_a_pure_rotation(six, monkeypatch):
    monkeypatch.setitem(DIRS, 1, [0, 1, -1, 0, 0, 0])               # shoulder and elbow cancel: the tool keeps its attitude
    code, ctl = six("--mode", "3", "A", "--speed", "3")
    assert code == 0, six.text
    m = re.search(r"VERDICT: mostly ([+-])([XYZ]) translation \(([\d.]+) mm; off-axis ([\d.]+) mm; rotation ([\d.]+) deg\)", six.text)
    assert m, six.text
    assert float(m.group(3)) > 5 and float(m.group(5)) < 0.1
    monkeypatch.setitem(DIRS, 1, [0, 0, 0, 0, 1, 0])
    code, ctl = six("--mode", "4", "A", "--speed", "3")
    assert code == 0, six.text
    assert re.search(r"VERDICT: mostly rotation \([\d.]+ deg about [+-][XYZ] \(tool\); translation [\d.]+ mm\)", six.text), six.text


def test_tcp_limit_writes_dot_at_once(six):
    code, ctl = six("--mode", "3", "A", "--speed", "5", "--hold", "3.0", "--max-tcp", "3")
    assert code == 1
    assert "LIMIT: the tool point is" in six.text and "--max-tcp 3" in six.text and "LimitTrip" in six.text
    sent = sent6(ctl)
    assert b".q" in sent[-4:] or sent.endswith(b".q")
    assert sent.count(b"A05") < 15
    assert not ctl.ctrlz and ctl.joints[0] - six.start[0] < 4


def test_joint_limit_writes_dot_at_once(six):
    code, ctl = six("--mode", "3", "D", "--speed", "5", "--hold", "3.0", "--max-joint", "1")
    assert code == 1
    assert "LIMIT: joint 4 is" in six.text and "--max-joint 1" in six.text
    assert sent6(ctl).count(b"D05") < 15 and abs(ctl.joints[3]) < 4
    assert not ctl.ctrlz


def test_sweep_runs_twelve_moves_in_order_and_prints_one_table(six):
    code, ctl = six("--mode", "3", "--sweep")
    assert code == 0, six.text
    sent = sent6(ctl)
    order = [m.group(1).decode() for m in re.finditer(rb"([A-Fa-f])02", re.sub(rb"([A-Fa-f]02)(?:\1)+", rb"\1", sent.replace(b"p", b"")))]
    assert order == list("AaBbCcDdEeFf")
    assert sent.count(b".") == 12 and sent.endswith(b"q")
    assert six.text.count("SWEEP TABLE") == 1
    rows = [ln for ln in six.text.splitlines() if re.match(r"\s+[A-Fa-f]\s+[+-]\d", ln)]
    assert len(rows) == 12
    assert "d-yaw d-pitch d-roll" in six.text and "mm/s" in six.text and "deg/s" in six.text
    assert "where the arm ended against where the sweep began" in six.text
    ended = re.search(r"joints \(degrees\): (.*)", six.text.split("where the arm ended")[1]).group(1)
    assert all(abs(float(x)) < 0.3 for x in re.findall(r"J\d ([+-][\d.]+)", ended))


def test_sweep_letters_limits_the_sweep_and_mode4_adds_tool_frame_rows(six):
    code, ctl = six("--mode", "4", "--sweep", "--letters", "ab")
    assert code == 0, six.text
    sent = sent6(ctl, 4)
    assert sent.count(b".") == 4 and b"C02" not in sent and b"A02" in sent and b"b02" in sent
    assert "tool-frame displacement at each move's start" in six.text


def test_sweep_stops_at_the_first_move_that_does_not_answer_as_expected(six):
    code, ctl = six("--mode", "3", "--sweep", fail_axis=2)
    assert code == 1
    sent = sent6(ctl)
    assert b"A02" in sent and b"a02" in sent and sent.count(b"B02") <= 3
    assert b"b02" not in sent and b"C02" not in sent
    assert "STOPPED EARLY" in six.text and "E -13" in six.text
    assert six.text.count("SWEEP TABLE") == 1                       # the moves that did run are tabulated


def test_a_limit_trip_in_a_sweep_ends_it(six):
    code, ctl = six("--mode", "3", "--sweep", "--max-joint", "1.5")
    assert code == 1 and "LIMIT: joint" in six.text
    assert b"B02" not in sent6(ctl)


def test_modes_3_4_never_move_if_p_is_not_answered(six):
    code, ctl = six("--mode", "3", "A", drop=b"p")
    assert code == 1
    assert "`p` WAS NOT ANSWERED" in six.text
    assert [b for b in sent6(ctl) if chr(b) in "ABCDEFabcdef"] == []


@pytest.mark.parametrize("argv", [
    ["--mode", "3", "A", "--reverse"], ["--mode", "3", "A", "--maxvel", "50"], ["--mode", "4", "A", "--speed2", "3"],
    ["--mode", "3", "A", "--no-deadman-test"], ["--mode", "3", "A", "--max-travel", "5"],
    ["--mode", "2", "A"], ["--mode", "3", "--sweep", "A"], ["--mode", "3", "--sweep", "--dry"], ["--sweep"],
    ["--mode", "1", "--sweep"], ["--mode", "3", "--sweep", "--letters", "AG"], ["--mode", "3", "--sweep", "--letters", "AA"],
    ["--mode", "3", "A", "--letters", "AB"], ["--mode", "3"], ["A", "--max-tcp", "10"], ["A", "--max-joint", "5"],
    ["--mode", "3", "A", "--max-tcp", "0"], ["--mode", "3", "A", "--max-tcp", "201"], ["--mode", "3", "A", "--max-joint", "46"],
    ["--mode", "3", "A", "--speed", "6"], ["--mode", "3", "A", "--hold", "5.1"],
])
def test_mode_3_4_refused_combinations_never_open_the_port(rig, argv, capsys):
    opened = []
    with pytest.raises(SystemExit):
        jogtest.main(["--local", str(rig.local), *argv], repo=str(rig.tmp), open_port=lambda a: opened.append(a),
                     clock=rig.clock, sleep=rig.clock.sleep, log_dir=str(rig.tmp))
    assert opened == []


def test_refusal_message_names_the_option_and_the_mode(rig, capsys):
    with pytest.raises(SystemExit):
        jogtest.main(["--local", str(rig.local), "--mode", "3", "A", "--reverse"], repo=str(rig.tmp))
    assert "--reverse cannot be used with --mode 3" in capsys.readouterr().err


def test_mode_3_defaults():
    a = jogtest.parse_args(["--mode", "3", "A"], "/home/buzz/dev/crs-a465")
    assert (a.mode, a.speed, a.hold, a.max, a.max_tcp, a.max_joint, a.sample) == (3, 2, 1.0, 5, 40.0, 10.0, True)
    a = jogtest.parse_args(["A"], "/home/buzz/dev/crs-a465")
    assert (a.mode, a.max_travel, a.sample) == (1, 12.0, False)


# ---------------------------------------------------------------------------------- jogd-c: modes 3 / 4 as bounded moves
import numpy as np      # noqa: E402  (already imported above; kept for this section's readability)


class CartController(FakeController):
    """A protocol-level stand-in for jogd-c in mode 3 or 4 (no interpreter): a letter or a 15-byte `@` vector starts ONE move at
    `mm_per_s` mm/s per percent (`deg_per_s` for D E F); the tool point goes along the direction (inverse kinematics from START6),
    `.` ends it with `-` after `minus_delay` s. Knobs: ack_delay (s before `+`; None = never), end_after (s after the start the move
    ends BY ITSELF with a bare `-`), coast (s the arm keeps moving after `.`), echo_wrong (a wrong echo of the vector)."""

    def __init__(self, clock, ack_delay=0.05, end_after=None, coast=0.0, minus_delay=0.02, mm_per_s=5.0, deg_per_s=2.0,
                 no_minus=False, abort_after=None, abort_line=b"A -67108870 Path error\r\n", dead_after_abort=False,
                 button_after_abort=False, e_after=None, abort_delay=0.0, **kw):
        super().__init__(clock, **kw)
        self.abort_after, self.abort_line, self.abort_delay = abort_after, abort_line, abort_delay   # the first move is aborted this long after its start
        self.dead_after_abort = dead_after_abort        # later moves are acknowledged but the arm does not move
        self.button_after_abort = button_after_abort    # the next start prints the PAUSE/CONTINUE request, and is acknowledged 1 s later
        self.e_after = e_after                          # an `E` line this long after the first start
        self.aborted = False
        self.joints = list(START6)
        self.ack_delay, self.end_after, self.coast, self.minus_delay = ack_delay, end_after, coast, minus_delay
        self.mm_per_s, self.deg_per_s, self.no_minus = mm_per_s, deg_per_s, no_minus
        self.cur = None
        self.buf = bytearray()
        self.writes = []                   # (time, bytes) of everything the tool wrote
        self.commands = []                 # every complete command that reached the program (starts and keep-alives)
        self.starts = []

    def write(self, data):
        self.writes.append((self.clock(), bytes(data)))
        return super().write(data)

    # -- the program
    def _launch(self, w):
        self.vm, self.exited, self.mode = True, False, int(w[1])
        self._put(b"jogd ready" + self.nl, 0.05)

    def _sync(self):
        pass

    def _deliver(self):
        pass

    def _exit(self):
        self._freeze()
        super()._exit()

    def _freeze(self):
        if self.cur:
            self.joints = self._pose(self.clock())
            self.cur = None

    def _shell(self, line):
        self._freeze()
        super()._shell(line)

    def _tool_dir(self, name):
        """(unit tool-point direction in the world frame, rotation joint or None)."""
        if name[0] == "@":
            comps = np.array([int(x) for x in re.findall(r"[+-]\d", name[1:7])], dtype=float)
            v = comps / np.linalg.norm(comps)
        else:
            axis = ord(name[0].upper()) - 64
            if axis > 3:
                return None, (axis, 1 if name[0].isupper() else -1)
            v = np.zeros(3)
            v[axis - 1] = 1 if name[0].isupper() else -1
        return v, None

    def _pose(self, t):
        c = self.cur
        if c is None:
            return list(self.joints)
        dt = max(min(t, c["phys_end"]) - c["t0"], 0.0)
        q0 = c["q0"]
        if c["rot"]:
            q = list(q0)
            axis, sign = c["rot"]
            q[axis - 1] += sign * c["speed"] * self.deg_per_s * dt
            return q
        p0, r0 = model.forward(q0)[:2]
        v = r0 @ c["dir"] if self.mode == 4 else c["dir"]
        target = p0 + v * c["speed"] * self.mm_per_s * dt
        q = np.array(q0, dtype=float)
        for _ in range(30):
            err = target - model.forward(q)[0]
            if np.linalg.norm(err) < 1e-4:
                break
            q += np.degrees(np.linalg.pinv(model.jacobian(q)[:3]) @ err)
        return list(q)

    def _to_jogd(self, b):
        if self.echo:
            self._put(b)
        if b == b"p":
            self.p_count += 1
            counts = [round(x * SAMPLE_CPD[j]) for j, x in zip(range(1, 7), self._pose(self.clock()))]
            self._put(("P " + " ".join(str(c) for c in counts)).encode() + self.nl, self.p_delay)
            return
        ch = b.decode("latin-1")
        self.to_jogd += b
        if self.buf:
            self.buf += b
            if len(self.buf) == 15:
                name, self.buf = bytes(self.buf).decode(), bytearray()
                self._command(name)
            return
        if ch == "@":
            self.buf = bytearray(b)
        elif ch == ".":
            self._dot()
        elif ch == "q":
            self._exit()
        elif ch in "ABCDEFabcdef":
            self._letter = ch
            self.buf2 = getattr(self, "buf2", b"") + b
            if len(self.buf2) == 3:
                name, self.buf2 = self.buf2.decode(), b""
                self._command(name)
        elif ch.isdigit() and getattr(self, "buf2", b""):
            self.buf2 += b
            if len(self.buf2) == 3:
                name, self.buf2 = self.buf2.decode(), b""
                self._command(name)

    def _command(self, name):
        self.commands.append((self.clock(), name))
        if self.cur and self.cur["name"] == name:
            return                                                    # a keep-alive
        now = self.clock()
        self._freeze()
        d, rot = self._tool_dir(name)
        speed = int(name[-2:])
        self.cur = dict(name=name, t0=now, q0=list(self.joints), dir=d, rot=rot, speed=speed, phys_end=float("inf"), end_entry=None)
        self.starts.append(now)
        ack = self.ack_delay
        if self.aborted and self.dead_after_abort:
            self.cur["speed"] = 0                                   # acknowledged, but the arm stays where it is
        if self.aborted and self.button_after_abort and len(self.starts) == 2:
            self._put(b"\r\n" + BUTTON.encode(), 0.02)
            ack = 1.0
        if ack is not None:
            self._put(b"+" + name.encode() + self.nl, ack)
        if self.e_after is not None and len(self.starts) == 1:
            self._put(b"E -22 invalid argument" + self.nl, self.e_after)
        if self.abort_after is not None and not self.aborted:
            self.aborted = True
            self.cur["phys_end"] = now + self.abort_after
            self.cur["aborted"] = True
            self._put(self.abort_line.replace(b"\r\n", self.nl), self.abort_after + self.abort_delay)
        if self.end_after is not None:
            self.cur["phys_end"] = now + self.end_after
            self.cur["end_entry"] = [now + self.end_after, b"-" + self.nl]
            self.out.append(self.cur["end_entry"])

    def _dot(self):
        c = self.cur
        if c is None:
            return
        now = self.clock()
        if c["end_entry"] is not None and c["end_entry"] in self.out:
            self.out.remove(c["end_entry"])
        c["phys_end"] = min(c["phys_end"], now + self.coast)
        c["name"] = None
        if c.get("aborted") and now >= c["phys_end"]:
            return                                                    # jogd-c has ended the move: `.` prints nothing
        if not self.no_minus and not (c["end_entry"] is not None and c["end_entry"][0] <= now):
            self._put(b"-" + self.nl, self.minus_delay)


@pytest.fixture
def cart(tmp_path):
    clock = FakeClock()
    local = tmp_path / "jogd"
    local.write_bytes(IMAGE)

    def go(*argv, **kw):
        ctl = CartController(clock, **kw)
        start = list(ctl.joints)
        code = jogtest.main(["--local", str(local), *argv], repo=str(tmp_path), open_port=lambda a: ctl,
                            clock=clock, sleep=clock.sleep, log_dir=str(tmp_path))
        go.text = "\n".join(p.read_text() for p in sorted(tmp_path.glob("*-jogtest.log")))
        go.start = start
        return code, ctl
    go.clock, go.local, go.tmp = clock, local, tmp_path
    return go


def tcp(joints):
    return model.forward(joints)[0]


def test_cart_letter_reports_start_ack_and_stop_figures(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--hold", "1.0", coast=0.3, ack_delay=0.08)
    assert code == 0, cart.text
    m = re.search(r"start acknowledged after (\d+) ms", cart.text)
    assert m and 70 <= int(m.group(1)) <= 130
    assert "may be BLOCKING" not in cart.text
    m = re.search(r"STOP: `-` came (\d+) ms after `\.`; the tool point travelled ([\d.]+) mm \(([\d.]+) degrees of rotation\) after `\.` was written; "
                  r"tool-point speed just before the `\.`: ([\d.]+) mm/s", cart.text)
    assert m, cart.text
    assert 15 <= int(m.group(1)) <= 60
    assert float(m.group(2)) == pytest.approx(0.3 * 10.0, abs=1.0)          # 2 % x 5 mm/s per % x 0.3 s of coasting
    assert float(m.group(4)) == pytest.approx(10.0, abs=1.5)
    up = tcp(ctl.joints) - tcp(cart.start)
    assert up[2] > 10 and abs(up[0]) < 0.5 and abs(up[1]) < 0.5            # straight up, C = +Z
    assert sent6(ctl).replace(b"p", b"").endswith(b".q")


def test_cart_stop_figure_without_coasting_is_small(cart):
    code, ctl = cart("--mode", "3", "A", "--hold", "1.0")
    assert code == 0, cart.text
    assert float(re.search(r"travelled ([\d.]+) mm", cart.text).group(1)) < 1.0


def test_start_over_600_ms_warns_that_the_call_may_block(cart):
    code, ctl = cart("--mode", "3", "A", "--hold", "1.0", ack_delay=0.7)
    assert code == 0, cart.text
    assert re.search(r"start acknowledged after 7\d\d ms", cart.text)
    assert "WARNING" in cart.text and "may be BLOCKING" in cart.text and "150 mm" in cart.text
    assert "START ACKNOWLEDGED LATE" in cart.text


def test_start_at_the_limit_of_600_ms_does_not_warn(cart):
    code, ctl = cart("--mode", "3", "A", "--hold", "1.0", ack_delay=0.5)
    assert code == 0 and "BLOCKING" not in cart.text


def test_no_plus_after_1500_ms_writes_dot_at_once(cart):
    code, ctl = cart("--mode", "3", "A", "--hold", "2.0", ack_delay=None)
    assert code == 1
    assert "NoAnswer" in cart.text and "may be blocking" in cart.text and "NOT ACKNOWLEDGED" in cart.text
    t0 = next(t for t, d in ctl.writes if d == b"A02")
    dots = [t for t, d in ctl.writes if d == b"."]
    assert len(dots) == 1 and 1.5 <= dots[0] - t0 <= 1.65
    assert sum(1 for _, d in ctl.writes if d == b"A02") <= 17
    assert [d for _, d in ctl.writes if d in (b".", b"q")] == [b".", b"q"] and not ctl.ctrlz


def test_a_bare_minus_before_dot_is_the_move_ending_by_itself(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "3", "--hold", "2.0", end_after=0.6)
    assert code == 0, cart.text
    m = re.search(r"the move ended by itself (\d+) ms after the start, after ([\d.]+) mm", cart.text)
    assert m and 580 <= int(m.group(1)) <= 700
    assert float(m.group(2)) == pytest.approx(0.6 * 15.0, abs=1.5)
    sent = sent6(ctl)
    assert b"." not in sent and sent.count(b"C03") <= 8 and sent.endswith(b"q")
    assert "STOP:" not in cart.text and "STOPPED EARLY" not in cart.text


def test_a_rotation_move_that_ends_by_itself_is_reported_in_degrees(cart):
    code, ctl = cart("--mode", "3", "E", "--speed", "3", "--hold", "2.0", end_after=0.5)
    assert code == 0, cart.text
    m = re.search(r"ended by itself (\d+) ms after the start, after ([\d.]+) degrees", cart.text)
    assert m and float(m.group(2)) == pytest.approx(0.5 * 6.0, abs=0.6)


def test_a_sweep_carries_on_after_a_move_that_ended_by_itself(cart):
    code, ctl = cart("--mode", "3", "--sweep", "--letters", "AB", "--hold", "1.0", end_after=0.4, coast=0.2)
    assert code == 0, cart.text
    sent = sent6(ctl)
    assert b"." not in sent
    order = [m.group(1).decode() for m in re.finditer(rb"([A-Fa-f])02", re.sub(rb"([A-Fa-f]02)(?:\1)+", rb"\1", sent.replace(b"p", b"")))]
    assert order == list("AaBb")
    rows = [ln for ln in cart.text.splitlines() if re.match(r"\s+[A-Fa-f]\s+[+-]\d", ln)]
    assert len(rows) == 4 and all("yes, " in r for r in rows)


def test_a_missing_minus_after_dot_is_still_a_failure(cart):
    code, ctl = cart("--mode", "3", "A", "--hold", "0.5", no_minus=True)
    assert code == 1
    assert "NotStopped" in cart.text and ctl.ctrlz == 1


def test_hold_up_to_5_seconds_in_modes_3_and_4(cart):
    a = jogtest.parse_args(["--mode", "4", "A", "--hold", "5.0"], "/home/buzz/dev/crs-a465")
    assert a.hold == 5.0
    code, ctl = cart("--mode", "3", "A", "--hold", "4.5", speed_degrees_per_percent=1.0)
    assert code == 1 and "LIMIT" in cart.text                      # 4.5 s x 10 mm/s is past --max-tcp 40: the limit still works
    code, ctl = cart("--mode", "3", "A", "--hold", "4.5", "--max-tcp", "100", "--max-joint", "20")
    assert code == 0, cart.text
    assert sent6(ctl).count(b"A02") >= 44


def test_vector_bytes_expected_echo_and_keepalives(cart):
    code, ctl = cart("--mode", "3", "--vector", "+1+1+0", "--speed", "2", "--hold", "1.0")
    assert code == 0, cart.text
    sent = sent6(ctl).replace(b"p", b"")
    assert sent.startswith(b"@+1+1+0+0+0+002" * 10 + b".") and sent.endswith(b".q")
    assert {d for _, d in ctl.commands} == {"@+1+1+0+0+0+002"}
    times = [t for t, d in ctl.writes if d == b"@+1+1+0+0+0+002"]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert all(0.09 <= g <= 0.13 for g in gaps)
    assert "move 1/1: @+1+1+0+0+0+002: as expected" in cart.text
    assert "expected lines ['+@+1+1+0+0+0+002', '-']" in cart.text


def test_vector_report_angle_and_off_line_distance_in_the_world_frame(cart):
    code, ctl = cart("--mode", "3", "--vector", "+1+1+0", "--hold", "1.0")
    assert code == 0, cart.text
    m = re.search(r"VECTOR: requested \(\+1 \+1 \+0\) in the world frame; measured displacement is ([\d.]+) degrees from it; off-line distance ([\d.]+) mm", cart.text)
    assert m and float(m.group(1)) < 1.0 and float(m.group(2)) < 0.5
    assert "start acknowledged after" in cart.text and "STOP: `-` came" in cart.text
    d = tcp(ctl.joints) - tcp(cart.start)
    assert d[0] > 5 and d[0] == pytest.approx(d[1], abs=0.5)


def test_vector_report_shows_a_wrong_direction(cart, monkeypatch):
    orig = CartController._tool_dir
    monkeypatch.setattr(CartController, "_tool_dir", lambda self, name: orig(self, "@+1+0+0+0+0+0" if name[0] == "@" else name))
    code, ctl = cart("--mode", "3", "--vector", "+1+1+0", "--hold", "1.0")
    assert code == 0, cart.text
    m = re.search(r"measured displacement is ([\d.]+) degrees from it; off-line distance ([\d.]+) mm", cart.text)
    assert 44 <= float(m.group(1)) <= 46 and float(m.group(2)) > 5


def test_vector_in_mode_4_is_compared_in_the_tools_own_frame(cart):
    code, ctl = cart("--mode", "4", "--vector", "+1+0-1", "--hold", "1.0", "--max-tcp", "60")
    assert code == 0, cart.text
    m = re.search(r"in the tool frame \(the tool's own frame at the start\); measured displacement is ([\d.]+) degrees", cart.text)
    assert m and float(m.group(1)) < 1.0
    p0, r0, _, _ = model.forward(cart.start)
    d = tcp(ctl.joints) - p0
    assert (r0.T @ d)[0] > 5 and (r0.T @ d)[2] < -5


def test_vector_return_runs_the_opposite_vector_and_reports_the_distance_from_the_start(cart):
    code, ctl = cart("--mode", "3", "--vector", "+2-1+0", "--hold", "1.0", "--return")
    assert code == 0, cart.text
    sent = sent6(ctl).replace(b"p", b"")
    assert b"@+2-1+0+0+0+002" in sent and b"@-2+1+0+0+0+002" in sent
    assert sent.index(b"@+2-1") < sent.index(b".") < sent.index(b"@-2+1") and sent.count(b".") == 2
    assert [d for _, d in ctl.commands if d.startswith("@")][-1] == "@-2+1+0+0+0+002"
    assert "move 2/2: @-2+1+0+0+0+002" in cart.text
    m = re.search(r"RETURN: .* ended ([\d.]+) mm from where it started", cart.text)
    assert m and float(m.group(1)) < 1.5
    assert float(m.group(1)) == pytest.approx(float(np.linalg.norm(tcp(ctl.joints) - tcp(cart.start))), abs=0.3)
    assert cart.text.count("start acknowledged after") == 2


def test_return_zero_components_stay_plus_zero():
    assert jogtest.opposite_vector("+1-2+0") == "-1+2+0"
    assert jogtest.opposite_vector("-0+0-3") == "+0+0+3"
    assert jogtest.vector13("+1+1+0") == "@+1+1+0+0+0+0"


def test_a_wrong_vector_echo_is_not_as_expected(cart):
    class Wrong(CartController):
        def _command(self, name):
            super()._command(name)
            if self.starts and len(self.starts) == 1 and name.startswith("@") and self.out:
                self.out[:] = [e for e in self.out if not e[1].startswith(b"+@")]
                self._put(b"+@+9+9+0+0+0+002" + self.nl, 0.05)
    clock = cart.clock
    ctl = Wrong(clock)
    code = jogtest.main(["--local", str(cart.local), "--mode", "3", "--vector", "+1+1+0"], repo=str(cart.tmp), open_port=lambda a: ctl,
                        clock=clock, sleep=clock.sleep, log_dir=str(cart.tmp))
    assert code == 1


def test_vector_move_that_ends_by_itself(cart):
    code, ctl = cart("--mode", "3", "--vector", "+0+0+1", "--hold", "2.0", end_after=0.5)
    assert code == 0, cart.text
    assert re.search(r"ended by itself (\d+) ms after the start, after [\d.]+ mm", cart.text)
    assert b"." not in sent6(ctl)


def test_sweep_table_has_the_new_columns(cart):
    code, ctl = cart("--mode", "3", "--sweep", "--letters", "CD", "--hold", "1.0", coast=0.2, end_after=None)
    assert code == 0, cart.text
    head = next(ln for ln in cart.text.splitlines() if "d-yaw d-pitch d-roll" in ln)
    for col in ("start ms", "stop ms", "after `.`", "ended by itself"):
        assert col in head
    rows = [ln for ln in cart.text.splitlines() if re.match(r"\s+[A-Fa-f]\s+[+-]\d", ln)]
    assert len(rows) == 4
    assert re.search(r"\s5\d\s+\d+\s+[\d.]+ mm\s+no ", rows[0]), rows[0]          # C: ms to `+`, ms to `-`, mm after `.`
    assert re.search(r"[\d.]+ deg\s+no ", rows[2]), rows[2]                       # D: degrees after `.`
    assert "!" not in "".join(rows)


def test_sweep_table_marks_a_slow_start(cart):
    code, ctl = cart("--mode", "3", "--sweep", "--letters", "A", "--hold", "1.0", ack_delay=0.7)
    assert code == 0, cart.text
    assert re.search(r"7\d\d!", cart.text)


@pytest.mark.parametrize("argv", [
    ["--mode", "3", "--vector", "+1+1+0", "C"], ["--mode", "3", "--vector", "+1+1+0", "--sweep"], ["--vector", "+1+1+0"],
    ["--mode", "1", "--vector", "+1+1+0"], ["--mode", "3", "--vector", "+1+1"], ["--mode", "3", "--vector", "1+1+0"],
    ["--mode", "3", "--vector", "+0+0+0"], ["--mode", "3", "--vector", "+10+1+0"], ["--mode", "3", "--vector", "+1+1+0", "--dry"],
    ["--mode", "3", "--vector", "+1+1+0", "--reverse"], ["--mode", "3", "C", "--return"], ["--mode", "3", "--sweep", "--return"],
    ["--mode", "3", "--vector", "+1+1+0", "--letters", "AB"], ["--mode", "4", "--vector", "+1+1+0", "--speed", "6"],
    ["--mode", "4", "--vector", "+1+1+0", "--hold", "5.1"], ["--return"],
])
def test_vector_refused_combinations_never_open_the_port(rig, argv):
    opened = []
    with pytest.raises(SystemExit):
        jogtest.main(["--local", str(rig.local), *argv], repo=str(rig.tmp), open_port=lambda a: opened.append(a),
                     clock=rig.clock, sleep=rig.clock.sleep, log_dir=str(rig.tmp))
    assert opened == []


def test_mode_1_hold_is_still_limited_to_3_seconds(rig):
    with pytest.raises(SystemExit):
        jogtest.main(["--local", str(rig.local), "A", "--hold", "3.5"], repo=str(rig.tmp), open_port=lambda a: None)


# ---------------------------------------------------------------------------------- the controller aborts the move: `A` line

A_TEXT = "A -67108870 Path error"


def test_a_lines_are_classified():
    assert jogtest.classify(A_TEXT) == ("A", A_TEXT)
    assert jogtest.classify("A abort") == ("A", "A abort")
    assert jogtest.classify("$ A abort") == ("A", "A abort")
    for text in ("+A02", "E -22 invalid", "S: Arm power is OFF", "-", "P 1 2 3 4 5 6"):
        assert jogtest.classify(text)[0] != "A"


def test_an_abort_ends_the_move_in_a_letter_run_and_is_reported(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--hold", "3.0", abort_after=0.8)
    assert code == 0, cart.text
    m = re.search(r"ABORTED BY THE CONTROLLER: A -67108870 Path error, (\d+) ms after the start, after ([\d.]+) mm", cart.text)
    assert m, cart.text
    assert 800 <= int(m.group(1)) <= 1000 and float(m.group(2)) == pytest.approx(0.8 * 10.0, abs=1.5)
    sent = sent6(ctl)
    assert b"." not in sent and sent.endswith(b"q") and sent.count(b"C02") <= 12      # no keep-alives after the A line, no `.`
    assert "prompt after `q`: back" in cart.text and not ctl.ctrlz and not ctl.ctrle
    assert "STOPPED EARLY" not in cart.text and "NOT AS EXPECTED" not in cart.text


def test_an_abort_of_a_rotation_is_reported_in_degrees(cart):
    code, ctl = cart("--mode", "3", "D", "--speed", "3", "--hold", "3.0", abort_after=0.9, abort_line=b"A abort\r\n")
    assert code == 0, cart.text
    assert re.search(r"ABORTED BY THE CONTROLLER: A abort, \d+ ms after the start, after [\d.]+ degrees", cart.text)


def test_an_abort_of_a_vector_move_ends_the_run_and_no_return_is_run(cart):
    code, ctl = cart("--mode", "3", "--vector", "+1+1+0", "--return", "--hold", "3.0", abort_after=0.8)
    assert code == 0, cart.text
    assert "ABORTED BY THE CONTROLLER: A -67108870 Path error" in cart.text
    assert len(ctl.starts) == 1 and b"." not in sent6(ctl)


def test_the_abort_report_comes_without_samples_too(cart):
    code, ctl = cart("--mode", "3", "A", "--hold", "3.0", abort_after=0.5, p_delay=0.095)
    assert code == 0 and cart.text.count("ABORTED BY THE CONTROLLER") >= 1


def test_an_abort_while_dot_is_in_flight_completes_the_stop(cart):
    code, ctl = cart("--mode", "3", "A", "--hold", "1.0", abort_after=1.06, no_minus=True)
    assert code == 0, cart.text                                  # the `.` was answered by the `A` line, not `-`
    assert [d for _, d in ctl.writes if d == b"."] == [b"."]
    assert "an `A` line came while `.` was in flight: the stop is complete" in cart.text
    assert "NotStopped" not in cart.text and not ctl.ctrlz


def test_a_run_without_an_abort_is_unchanged(cart):
    code, ctl = cart("--mode", "3", "A", "--hold", "1.0")
    assert code == 0 and "ABORTED BY THE CONTROLLER" not in cart.text and "AFTER THE ABORT" not in cart.text


def test_an_a_line_between_moves_is_listed_and_does_not_fail_the_run(cart):
    class Between(CartController):
        def _dot(self):
            super()._dot()
            self._put(A_TEXT.encode() + self.nl, 0.5)           # in the pause after the first move: nothing in force

    ctl = Between(cart.clock)
    local = cart.local
    code = jogtest.main(["--local", str(local), "--mode", "3", "--sweep", "--letters", "A"], repo=str(cart.tmp), open_port=lambda a: ctl,
                        clock=cart.clock, sleep=cart.clock.sleep, log_dir=str(cart.tmp))
    text = "\n".join(p.read_text() for p in sorted(cart.tmp.glob("*-jogtest.log")))
    assert code == 0, text
    assert "an `A` line came between moves" in text and "lines during the pauses" not in text


# ---- --after-abort

def test_after_abort_with_a_fake_that_moves(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--hold", "1.5", "--after-abort", "c", abort_after=0.8)
    assert code == 0, cart.text
    assert "AFTER THE ABORT (--after-abort c)" in cart.text
    assert "MOVED AFTER THE ABORT: yes" in cart.text and "PAUSE/CONTINUE text seen after the abort: no" in cart.text
    assert re.search(r"move after the abort: c02: start acknowledged after \d+ ms", cart.text)
    a_line = next(t for t, k in [(t, d) for t, d in ctl.writes if d == b"C02"][:1])
    c_cmd = [t for t, d in ctl.writes if d == b"c02"]
    abort_t = a_line + 0.8
    assert c_cmd and 1.5 <= c_cmd[0] - abort_t <= 1.75                 # 1.5 s after the `A` line
    seq = [d for _, d in ctl.writes if d in (b".", b"q")]
    assert seq == [b".", b"q"]                                         # the aborted move got no `.`; the after-abort move did
    assert tcp(ctl.joints)[2] < tcp(cart.start)[2]                  # up 8 mm, then down 15 mm
    assert "STOPPED EARLY" not in cart.text


def test_after_abort_with_a_fake_that_does_not_move(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--hold", "1.5", "--after-abort", "c", abort_after=0.8, dead_after_abort=True)
    assert "MOVED AFTER THE ABORT: no" in cart.text
    assert code == 0, cart.text                                        # movement is reported, not judged
    assert re.search(r"after the abort: c02: start acknowledged", cart.text)


def test_after_abort_same_letter_writes_one_dot_first(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--hold", "1.5", "--after-abort", "C", abort_after=0.8)
    assert code == 0, cart.text
    seq = [d for _, d in ctl.writes if d in (b".", b"q") or d.startswith(b"C")]
    first_dot = seq.index(b".")
    assert seq[first_dot + 1] == b"C02" and seq.count(b".") == 2       # `.` then the new start, and the `.` that stops it
    assert "MOVED AFTER THE ABORT: yes" in cart.text


def test_after_abort_reports_pause_continue_text(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--hold", "2.0", "--after-abort", "c", abort_after=0.8, button_after_abort=True)
    assert "PAUSE/CONTINUE text seen after the abort: YES" in cart.text
    assert "MOVED AFTER THE ABORT: yes" in cart.text


def test_after_abort_without_an_abort_is_not_run_and_fails(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--hold", "1.0", "--after-abort", "c")
    assert code == 1
    assert "no `A` line came, so the after-abort move was NOT run" in cart.text and len(ctl.starts) == 1


def test_after_abort_with_a_vector(cart):
    code, ctl = cart("--mode", "3", "--vector", "+1+0+0", "--hold", "2.0", "--after-abort", "a", abort_after=0.8)
    assert code == 0, cart.text
    assert "MOVED AFTER THE ABORT: yes" in cart.text and len(ctl.starts) == 2


def test_after_abort_a_second_abort_is_reported_not_a_failure(cart):
    class Twice(CartController):
        def _command(self, name):
            self.aborted = False if len(self.starts) == 1 else self.aborted     # abort the second move too
            super()._command(name)

    ctl = Twice(cart.clock, abort_after=0.7)
    code = jogtest.main(["--local", str(cart.local), "--mode", "3", "C", "--hold", "1.5", "--after-abort", "c"], repo=str(cart.tmp),
                        open_port=lambda a: ctl, clock=cart.clock, sleep=cart.clock.sleep, log_dir=str(cart.tmp))
    text = "\n".join(p.read_text() for p in sorted(cart.tmp.glob("*-jogtest.log")))
    assert code == 0 and text.count("ABORTED BY THE CONTROLLER") >= 2, text


# ---- --toward-limit

def test_toward_limit_ends_on_an_a_line(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--toward-limit", "--max-tcp", "100", abort_after=3.0)
    assert code == 0, cart.text
    assert "WARNING: --toward-limit is meant to END IN A CONTROLLER ABORT" in cart.text
    assert "TOWARD-LIMIT ENDED BY: an `A` line" in cart.text and "ABORTED BY THE CONTROLLER" in cart.text
    assert b"." not in sent6(ctl) and sent6(ctl).count(b"C02") > 20                  # kept going well past any fixed hold


def test_toward_limit_ends_on_a_dash_by_itself(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--toward-limit", end_after=2.0)
    assert code == 0, cart.text
    assert "TOWARD-LIMIT ENDED BY: a `-` by itself" in cart.text and b"." not in sent6(ctl)


def test_toward_limit_ends_on_an_e_line(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--toward-limit", e_after=1.0)
    assert code == 1 and "TOWARD-LIMIT ENDED BY: an `E` line" in cart.text and "EError" in cart.text


def test_toward_limit_ends_on_the_tool_limit(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--toward-limit", "--max-tcp", "30")
    assert code == 1 and "TOWARD-LIMIT ENDED BY: a tool limit" in cart.text and "LIMIT: the tool point is" in cart.text


def test_toward_limit_ends_on_the_joint_limit(cart):
    code, ctl = cart("--mode", "3", "D", "--speed", "3", "--toward-limit", "--max-joint", "5", "--max-tcp", "200")
    assert code == 1 and "TOWARD-LIMIT ENDED BY: a tool limit" in cart.text and "LIMIT: joint" in cart.text


def test_toward_limit_ends_on_the_time_limit(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "1", "--toward-limit", "--time-limit", "2.5")
    assert code == 0, cart.text
    assert "TOWARD-LIMIT ENDED BY: --time-limit 2.5 s" in cart.text
    t0 = next(t for t, d in ctl.writes if d == b"C01")
    dot = next(t for t, d in ctl.writes if d == b".")
    assert 2.5 <= dot - t0 <= 2.8 and [d for _, d in ctl.writes if d in (b".", b"q")] == [b".", b"q"]


def test_toward_limit_with_after_abort_is_the_bench_check(cart):
    code, ctl = cart("--mode", "3", "C", "--speed", "2", "--toward-limit", "--max-tcp", "100", "--after-abort", "c", abort_after=2.0)
    assert code == 0, cart.text
    assert "MOVED AFTER THE ABORT: yes" in cart.text
    assert re.search(r"after the abort: c02", cart.text)
    c = [t for t, d in ctl.writes if d == b"c02"]
    assert c and 1.9 <= (c[-1] - c[0]) <= 2.3                                          # the after-abort hold defaults to 2.0 s


def test_toward_limit_time_limit_default_and_range():
    a = jogtest.parse_args(["--mode", "3", "A", "--toward-limit"], "/home/buzz/dev/crs-a465")
    assert a.time_limit == 20.0 and a.toward_limit and a.hold == 2.0
    a = jogtest.parse_args(["--mode", "3", "A", "--toward-limit", "--time-limit", "60", "--max-tcp", "200", "--max-joint", "45"], "/home/buzz/dev/crs-a465")
    assert (a.time_limit, a.max_tcp, a.max_joint) == (60.0, 200.0, 45.0)


@pytest.mark.parametrize("argv", [
    ["--mode", "3", "A", "--toward-limit", "--time-limit", "61"], ["--mode", "3", "A", "--toward-limit", "--time-limit", "0"],
    ["A", "--toward-limit"], ["--mode", "3", "--toward-limit"], ["--mode", "3", "--vector", "+1+0+0", "--toward-limit"],
    ["--mode", "3", "--sweep", "--toward-limit"], ["--mode", "3", "--dry", "--toward-limit"],
    ["--mode", "3", "A", "--time-limit", "5"], ["--mode", "3", "A", "--toward-limit", "--hold", "2"],
    ["--mode", "3", "A", "--toward-limit", "--max-tcp", "201"], ["--mode", "3", "A", "--toward-limit", "--max-joint", "46"],
    ["--mode", "3", "A", "--after-abort", "AB"], ["--mode", "3", "A", "--after-abort", "1"], ["--mode", "3", "--sweep", "--after-abort", "a"],
    ["--mode", "3", "--dry", "--after-abort", "a"], ["--mode", "3", "--after-abort", "a"], ["A", "--after-abort", "b"],
    ["A", "--reverse", "--after-abort", "a"],
])
def test_the_new_options_are_refused_before_the_port_is_opened(rig, argv, capsys):
    opened = []
    with pytest.raises(SystemExit):
        jogtest.main(["--local", str(rig.local), *argv], repo=str(rig.tmp), open_port=lambda a: opened.append(a))
    assert not opened


def test_mode_1_after_abort_takes_the_same_joint_and_switches_sampling_on():
    a = jogtest.parse_args(["F", "--after-abort", "f"], "/home/buzz/dev/crs-a465")
    assert a.sample and a.after_abort == "f"


def test_an_abort_in_a_mode_1_run_ends_it_without_a_dot(tmp_path):
    clock = FakeClock()
    local = tmp_path / "jogd"
    local.write_bytes(IMAGE)

    class Aborting(FakeController):
        def _launch(self, w):
            super()._launch(w)
            self._put(A_TEXT.encode() + self.nl, 1.6)          # about 0.5 s into the first move

    ctl = Aborting(clock)
    code = jogtest.main(["--local", str(local), "F", "--speed", "2", "--hold", "3.0"], repo=str(tmp_path), open_port=lambda a: ctl,
                        clock=clock, sleep=clock.sleep, log_dir=str(tmp_path))
    text = "\n".join(p.read_text() for p in sorted(tmp_path.glob("*-jogtest.log")))
    assert "ABORTED BY THE CONTROLLER: A -67108870 Path error" in text, text
    assert b"." not in commands_sent(ctl).replace(b"p", b"") and not ctl.ctrlz and "prompt after `q`: back" in text
