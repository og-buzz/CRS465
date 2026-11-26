"""jogd-c (a465/jogprog.py, option cart): mode 1 exactly as jogd; modes 3 and 4 as straight-line online
moves (jog_ws / jog_ts) stopped with halt(); `@` vectors (moves / move). Run in a465/r3vm.py against a
fake robot server whose online moves advance a fake pose with the clock and stop on halt().
"""
import pytest

from a465 import calfile, jogprog, r3vm

NAMES = {0x02: "ctl_get", 0x04: "ctl_rel", 0x0E: "pos_get", 0x21: "halt", 0x2C: "move", 0x2D: "moves",
         0x34: "jog_ws", 0x36: "jog_ts", 0x3E: "robotisfinished", 0x3F: "online", 0x43: "speed_get",
         0x44: "speed_set", 0x52: "man_mode_set", 0x53: "man_move_start", 0x54: "man_move_stop",
         0x55: "robot_mode_get", 0x62: "here", 0x7A: "robot_error_get", 0x80: "shift_t"}
CBUILD = jogprog.build(jogprog.BUILDS["jogd-c"])
JOGD = jogprog.build()
SAVED_SPEED = 40
POSE = [300.0, -20.0, 500.0, 10.0, 80.0, -5.0]          # x y z (mm), yaw pitch roll (deg)
PULSES = [1000, -2000, 3000, -400, 500, -600, 0, 0]
CPD100 = [round(c * 100) for c in calfile.COUNTS_PER_DEGREE]
F = r3vm.from_float


def f(word):
    return r3vm.to_float(word)


class Arm:
    """A fake server. Online moves run at `mm_s` (translations; `deg_s` for rotations; joint moves take
    `joint_ms`), halt() freezes the pose where it is, robotisfinished() is 1 once the travel is done."""

    def __init__(self, mm_s=10.0, deg_s=10.0, joint_ms=2000, results=None, halt_ms=0, error_word=0, error_results=(0,)):
        self.pose, self.pulses = list(POSE), list(PULSES)
        self.halt_ms, self.stopping_until = halt_ms, None     # after halt(): robotisfinished 0 for halt_ms
        self.error_word, self.error_results = error_word, list(error_results)
        self.mm_s, self.deg_s, self.joint_ms = mm_s, deg_s, joint_ms
        self.motion = None                       # (t0, from, to, duration ms, "cart" | "joint")
        self.online = 0
        self.halts = 0
        self.override = dict(results or {})      # number -> result instead of the fake's own

    def now(self, vm):
        if self.motion is None:
            return
        t0, a, b, dur, kind = self.motion
        k = 1.0 if dur <= 0 else min(1.0, (vm.clock - t0) / dur)
        cur = [x + (y - x) * k for x, y in zip(a, b)]
        if kind == "cart":
            self.pose = cur
        else:
            self.pulses = [round(v) for v in cur]
        if k >= 1.0:
            self.motion = None

    def start_cart(self, vm, target):
        self.now(vm)
        d = [abs(t - p) for t, p in zip(target, self.pose)]
        dur = max(max(d[:3]) / self.mm_s, max(d[3:]) / self.deg_s) * 1000
        self.motion = (vm.clock, list(self.pose), list(target), dur, "cart")

    def results(self):
        def wrap(number, fn):
            def call(vm, args):
                if number in self.override:
                    r = self.override[number]
                    return r(vm, args) if callable(r) else r
                return fn(vm, args)
            return call
        own = {0x43: self.speed_get, 0x0E: self.pos_get, 0x21: self.halt, 0x34: self.jog, 0x36: self.jog,
               0x3E: self.finished, 0x3F: self.set_online, 0x62: self.here, 0x2D: self.moves, 0x2C: self.move,
               0x80: self.shift_t, 0x55: self.mode_get, 0x7A: self.error_get}
        return {n: wrap(n, fn) for n, fn in own.items()} | {n: wrap(n, lambda vm, a: 0) for n in self.override
                                                             if n not in own}

    def speed_get(self, vm, args):
        vm.store(args[0], 0, SAVED_SPEED)
        return 0

    def mode_get(self, vm, args):
        vm.store(args[0], 0, 0)
        return 0

    def pos_get(self, vm, args):
        self.now(vm)
        r3vm.store_location(vm, args[1], self.pulses, header=2 | 1 << 16)
        return 0

    def halt(self, vm, args):
        self.now(vm)
        self.motion = None
        self.halts += 1
        self.stopping_until = vm.clock + self.halt_ms
        return 0

    def error_get(self, vm, args):
        r = self.error_results.pop(0) if len(self.error_results) > 1 else self.error_results[0]
        if r >= 0:
            vm.store(args[0], 0, self.error_word)
        return r

    def jog(self, vm, args):                      # jog_ws / jog_ts(axis, distance): the fake's tool frame is the world's
        axis, dist = r3vm.s32(args[0]), f(args[1])
        assert 1 <= abs(axis) <= 6
        self.now(vm)
        target = list(self.pose)
        target[abs(axis) - 1] += dist if axis > 0 else -dist
        self.start_cart(vm, target)
        return 0

    def finished(self, vm, args):
        self.now(vm)
        if self.stopping_until is not None and vm.clock < self.stopping_until:
            return 0
        return int(self.motion is None)

    def set_online(self, vm, args):
        self.online = r3vm.s32(args[0])
        return 0

    def here(self, vm, args):
        assert vm.load(args[0], 0) & 0xFF == 1, "here() needs a cloc"
        self.now(vm)
        vm.store(args[0], 0, 1 | 0x10 << 16)
        for k in range(8):
            vm.store(args[0], 1 + k, F(self.pose[k]) if k < 6 else 0)
        return 0

    def moves(self, vm, args):                    # the location by value: args[0..8] are its words 0..8
        assert args[0] & 0xFF == 1, "moves() of a cloc"
        self.start_cart(vm, [f(args[1 + k]) for k in range(6)])
        return 0

    def move(self, vm, args):
        assert args[0] & 0xFF == 2, "move() of a ploc"
        self.now(vm)
        self.motion = (vm.clock, list(self.pulses), [r3vm.s32(args[1 + k]) for k in range(8)], self.joint_ms, "joint")
        return 0

    def shift_t(self, vm, args):                  # (&loc, 1, tag 5, the displacement's 9 words)
        assert r3vm.s32(args[1]) == 1 and r3vm.s32(args[2]) == 5
        for k in range(3):
            vm.store(args[0], 1 + k, F(f(vm.load(args[0], 1 + k)) + f(args[4 + k])))
        return 0


class Run:
    def __init__(self, inputs, argv=("3",), arm=None, image=CBUILD, until=60_000, files=None, latency_ms=0,
                 signals=()):
        self.arm = arm or Arm()
        self.robot = r3vm.FakeRobot(self.arm.results(), latency_ms=latency_ms)
        self.syslib = r3vm.FakeSyslib({5: "I/O error", 22: "invalid argument", 13: "not the controlling client",
                                       42: "device is being reset", 0x4000006: "Path error"})
        self.vm = r3vm.VM(image, argv=["/tmp/jogd-c", *argv], libs={"robotlib": self.robot, "syslib": self.syslib},
                          files=files or {"/dev/cin": r3vm.Input(inputs)})
        for t, sig in signals:
            self.vm.post_signal(t, sig)
        self.killed = None
        try:
            self.code = self.vm.run(until_ms=until)
        except r3vm.Deadline:
            self.code = None
        except r3vm.Killed as e:
            self.code, self.killed = None, e.sig
        self.raw = list(self.robot.calls)
        self.names = [NAMES[n] for n, _ in self.raw]
        self.lines = self.vm.stdout().splitlines()

    def args(self, name):
        return [a for n, a in self.raw if NAMES[n] == name]

    def at(self, name):
        return [t for (n, _), t in zip(self.robot.calls, self.robot.times) if NAMES[n] == name]

    def printed(self, text):
        return [t for t, fd, s in self.vm.events if fd == 1 and s == text + "\n"]


def keepalives(cmd, start, stop, every=100):
    return [(t, cmd) for t in range(start, stop, every)]


def without_polls(names):
    return [n for n in names if n not in ("robotisfinished", "pos_get")]


# ---- builds --------------------------------------------------------------------------------------

def test_the_committed_jogd_c_is_what_jogprog_builds():
    assert (jogprog.OUT / "jogd-c").read_bytes() == CBUILD


def test_the_other_builds_do_not_carry_any_of_it():
    for name in ("jogd", "jogd-nospeedset", "jogd-v"):
        src = jogprog.source(jogprog.BUILDS[name])
        assert "jog_ws" not in src and "vcommand" not in src and "CART_MM" not in src


def test_cart_needs_speed_set():
    with pytest.raises(ValueError):
        jogprog.source(jogprog.Options(cart=True, speed_set=False))


# ---- modes 3 and 4: letters ----------------------------------------------------------------------

def test_world_letter_start_keepalive_stop():
    r = Run([(100, b"A02")] + keepalives(b"A02", 200, 1000) + [(1050, b"."), (1500, b"q")])
    assert r.code == 0
    assert r.lines == ["jogd ready", "+A02", "-"]
    assert without_polls(r.names) == ["ctl_get", "speed_get", "online", "speed_set", "jog_ws", "halt",
                                      "online", "speed_set", "ctl_rel"]
    assert r.args("online") == [(1,), (0,)]
    (axis, dist), = r.args("jog_ws")
    assert axis == 1 and f(dist & 0xFFFFFFFF) == 150.0
    assert r.args("speed_set") == [(2,), (SAVED_SPEED,)]
    assert "man_mode_set" not in r.names and "man_move_start" not in r.names


def test_finished_is_polled_with_no_input_waiting_at_most_every_fin_ms():
    r = Run([(100, b"A02")] + keepalives(b"A02", 200, 3500) + [(3550, b"."), (4000, b"q")])
    t_halt, = r.at("halt")
    polls = [t for t in r.at("robotisfinished") if t < t_halt]
    start = r.at("jog_ws")[0]
    assert len(polls) == 3, "one a second while moving, none on the keep-alives in between"
    gaps = [b - a for a, b in zip([start] + polls, polls)]
    assert min(gaps) >= jogprog.Options().fin_ms
    assert r.lines == ["jogd ready", "+A02", "-"]


@pytest.mark.parametrize("letter,axis,dist", [(b"b", 2, -150.0), (b"C", 3, 150.0), (b"d", 4, -45.0),
                                              (b"E", 5, 45.0), (b"f", 6, -45.0)])
def test_tool_letters_use_jog_ts_and_the_library_axis_numbers(letter, axis, dist):
    r = Run([(100, letter + b"05"), (200, b"."), (400, b"q")], argv=("4",))
    (a, d), = r.args("jog_ts")
    assert a == axis and f(d & 0xFFFFFFFF) == dist
    assert "jog_ws" not in r.names
    assert r.lines == ["jogd ready", "+" + letter.decode() + "05", "-"]


def test_the_deadman_halts_and_prints_T():
    r = Run([(100, b"A02"), (1500, b"q")])
    assert r.lines == ["jogd ready", "+A02", "T"]
    t_halt, = r.at("halt")
    assert 300 <= t_halt - 100 <= 330


def test_a_move_that_ends_by_itself_prints_a_dash_and_later_keepalives_are_ignored():
    arm = Arm(mm_s=500.0)                         # 150 mm in 300 ms
    r = Run([(100, b"A02")] + keepalives(b"A02", 200, 1500) + [(1550, b"."), (1800, b"q")], arm=arm)
    assert r.lines == ["jogd ready", "+A02", "-"]
    assert r.names.count("jog_ws") == 1 and "halt" not in r.names
    t_dash, = r.printed("-")
    assert t_dash <= 100 + jogprog.Options().fin_ms + 20
    assert arm.pose[0] == pytest.approx(POSE[0] + 150.0)


def test_another_letter_halts_then_starts():
    r = Run([(100, b"A02"), (200, b"A02"), (300, b"c03"), (400, b"."), (600, b"q")])
    assert r.lines == ["jogd ready", "+A02", "-", "+c03", "-"]
    assert [n for n in r.names if n != "pos_get"][2:9] == ["online", "speed_set", "jog_ws", "halt", "robotisfinished",
                                                         "speed_set", "jog_ws"]
    assert r.args("jog_ws")[1][0] == 3 and f(r.args("jog_ws")[1][1] & 0xFFFFFFFF) == -150.0


def test_the_same_letter_at_another_speed_is_a_keepalive():
    r = Run([(100, b"A02"), (200, b"A05"), (300, b"A02"), (350, b"."), (600, b"q")])
    assert r.lines == ["jogd ready", "+A02", "-"]
    assert r.names.count("jog_ws") == 1 and r.args("speed_set") == [(2,), (SAVED_SPEED,)]


def test_a_failed_move_call_prints_E_then_halts_and_dashes():
    arm = Arm(results={0x34: -5})
    r = Run([(100, b"A02"), (200, b"A02"), (300, b"q")], arm=arm)
    assert r.lines == ["jogd ready", "E -5 I/O error", "-"]
    assert r.names.count("jog_ws") == 1, "a repeat of the failed command is a keep-alive"
    assert arm.halts == 1


def test_p_during_a_move():
    r = Run([(100, b"A02"), (150, b"p"), (200, b"A02"), (250, b"."), (500, b"q")])
    assert r.lines[:3] == ["jogd ready", "+A02", "P " + " ".join(str(v) for v in PULSES[:6])]
    assert r.lines[3:] == ["-"]


def test_online_failing_at_start_releases_and_exits_1():
    arm = Arm(results={0x3F: -13})
    r = Run([(100, b"A02")], arm=arm)
    assert r.code == 1
    assert r.lines == ["E -13 not the controlling client"]
    assert r.names == ["ctl_get", "speed_get", "online", "ctl_rel"]


class Broken(r3vm.Input):
    def read(self, now, n):
        return -5 if now >= 300 else super().read(now, n)


def test_an_input_error_halts_and_puts_online_and_the_speed_back():
    r = Run([], files={"/dev/cin": Broken([(100, b"A02"), (200, b"A02")])})
    assert r.code == 1
    assert r.lines == ["jogd ready", "+A02", "E -5 I/O error", "-"]
    assert without_polls(r.names)[-4:] == ["halt", "online", "speed_set", "ctl_rel"]
    assert r.args("online")[-1] == (0,)


# ---- mode 1 is jogd's ----------------------------------------------------------------------------

@pytest.mark.parametrize("inputs", [
    [(100, b"A02"), (200, b"A02"), (300, b"b05"), (400, b"."), (500, b"p"), (700, b"q")],
    [(100, b"F03"), (1200, b"A02"), (1300, b"A07"), (1400, b"q")],
])
def test_mode_1_behaves_as_jogd(inputs):
    def run(image):
        arm = Arm()
        return Run(inputs, argv=("1",), arm=arm, image=image)
    a, b = run(JOGD), run(CBUILD)
    assert a.lines == b.lines and a.code == b.code
    assert a.raw == b.raw and a.robot.times == b.robot.times


# ---- `@` vectors ---------------------------------------------------------------------------------

def vec(*comps, speed=2):
    return ("@" + "".join(f"{'-' if c < 0 else '+'}{abs(c)}" for c in comps) + f"{speed:02d}").encode()


def test_world_vector_moves_straight_to_the_scaled_target():
    v = vec(3, 4, 0, 0, 0, 0)
    r = Run([(100, v)] + keepalives(v, 200, 700) + [(750, b"."), (1000, b"q")])
    assert r.lines == ["jogd ready", "+" + v.decode(), "-"]
    assert without_polls(r.names) == ["ctl_get", "speed_get", "online", "speed_set", "here", "moves", "halt",
                                      "online", "speed_set", "ctl_rel"]
    (loc,) = r.args("moves")
    target = [f(w & 0xFFFFFFFF) for w in loc[1:7]]
    assert target[0] == pytest.approx(POSE[0] + 90.0) and target[1] == pytest.approx(POSE[1] + 120.0)
    assert target[2:] == pytest.approx(POSE[2:])


def test_tool_vector_uses_shift_t_with_a_cloc_displacement():
    v = vec(0, -1, 1, 0, 0, 0, speed=3)
    r = Run([(100, v), (200, b"."), (400, b"q")], argv=("4",))
    assert without_polls(r.names)[3:7] == ["speed_set", "here", "shift_t", "moves"]
    (args,) = r.args("shift_t")
    assert args[1] == 1 and args[2] == 5 and args[3] & 0xFF == 1
    disp = [f(w & 0xFFFFFFFF) for w in args[4:12]]
    k = 150.0 / 2 ** 0.5
    assert disp == pytest.approx([0.0, -k, k, 0, 0, 0, 0, 0], rel=1e-6)
    (loc,) = r.args("moves")
    assert [f(w & 0xFFFFFFFF) for w in loc[1:4]] == pytest.approx([POSE[0], POSE[1] - k, POSE[2] + k], rel=1e-6)


def test_joint_vector_is_one_move_with_joints_in_proportion():
    v = vec(1, 0, -2, 0, 0, 0)
    r = Run([(100, v), (200, v), (300, b"."), (600, b"A02"), (700, b"."), (900, b"q")], argv=("1",))
    assert r.lines == ["jogd ready", "+" + v.decode(), "-", "+A02", "-"]
    names = without_polls(r.names)
    assert names[:9] == ["ctl_get", "speed_get", "speed_set", "online", "move", "halt", "online",
                         "man_mode_set", "man_move_start"]
    assert r.args("online") == [(1,), (0,)]
    (loc,) = r.args("move")
    want = list(PULSES)
    want[0] += int(1 * 30 * CPD100[0] / 200)
    want[2] += int(-2 * 30 * CPD100[2] / 200)
    assert [r3vm.s32(w) for w in loc[1:9]] == want


def test_vector_rotations_are_refused_in_modes_3_and_4():
    r = Run([(100, vec(1, 0, 0, 0, 2, 0)), (300, b"q")])
    assert r.lines == ["jogd ready", "E -22 invalid argument"]
    assert "moves" not in r.names and "here" not in r.names


def test_a_vector_while_a_vector_moves_is_its_keepalive_and_one_after_its_end_is_ignored():
    arm = Arm(mm_s=500.0)
    v, w = vec(1, 0, 0, 0, 0, 0), vec(0, 1, 0, 0, 0, 0)
    r = Run([(100, v), (200, w)] + keepalives(v, 300, 1200) + [(1300, b"q")], arm=arm)
    assert r.lines == ["jogd ready", "+" + v.decode(), "-"]
    assert r.names.count("moves") == 1


def test_a_vector_while_a_letter_moves_stops_it_first():
    v = vec(1, 1, 0, 0, 0, 0)
    r = Run([(100, b"A02"), (200, v), (300, b"."), (500, b"q")])
    assert r.lines == ["jogd ready", "+A02", "-", "+" + v.decode(), "-"]
    assert without_polls(r.names)[4:8] == ["jog_ws", "halt", "here", "moves"]


def test_a_half_received_vector_is_dropped_by_a_letter_and_kept_across_p():
    r = Run([(100, b"@+3+4"), (150, b"A02"), (250, b"."), (300, b"@+1+0+0"), (320, b"p"),
             (340, b"+0+0+005"), (450, b"."), (700, b"q")])
    assert r.lines[:3] == ["jogd ready", "+A02", "-"]
    assert r.lines[3].startswith("P ")
    assert r.lines[4:] == ["+@+1+0+0+0+0+005", "-"]


def test_a_malformed_vector_is_dropped():
    r = Run([(100, b"@+3+x+0+0+0+002"), (300, b"q")])
    assert r.lines == ["jogd ready"]
    assert "moves" not in r.names


def test_vector_speed_is_clamped_and_echoed_as_used():
    r = Run([(100, b"@-0+0+1+0+0+060"), (200, b"."), (400, b"q")], argv=("3", "25"))
    assert r.lines[1] == "+@-0+0+1+0+0+025"
    assert r.args("speed_set")[0] == (25,)


def test_cylindrical_mode_refuses_vectors():
    r = Run([(100, vec(1, 0, 0, 0, 0, 0)), (300, b"q")], argv=("2",))
    assert r.lines == ["jogd ready", "E -22 invalid argument"]


# ---- the stop: halt first, the line, then the rest -------------------------------------------------

def calls_after(r, t):
    return [NAMES[n] for (n, _), tt in zip(r.raw, r.robot.times) if tt >= t]


def test_after_a_dot_halt_is_the_first_call_and_the_dash_comes_when_it_returns():
    r = Run([(100, b"A02")] + keepalives(b"A02", 200, 1000) + [(1000, b"."), (1500, b"q")], latency_ms=80)
    t_halt, = r.at("halt")
    assert 1000 <= t_halt <= 1010, "nothing in front of the halt but the loop's 10 ms wait"
    assert calls_after(r, 1000)[:3] == ["halt", "robotisfinished", "online"]
    t_dash, = r.printed("-")
    assert t_dash == t_halt + 80 and t_dash <= r.at("robotisfinished")[-1]


@pytest.mark.parametrize("burst", [b"p.", b"A02p.", b"pA02."])
def test_waiting_input_is_read_before_a_p_answer(burst):
    r = Run([(100, b"A02")] + keepalives(b"A02", 200, 1100) + [(1150, burst), (1600, b"q")], latency_ms=80)
    t_halt, = r.at("halt")
    assert 1150 <= t_halt <= 1160
    answered = [t for t in r.at("pos_get") if t > 1150]
    assert answered and answered[0] > t_halt, "the p is answered after the halt"
    assert r.lines[:4] == ["jogd ready", "+A02", "-", "P " + " ".join(str(v) for v in PULSES[:6])]


@pytest.mark.parametrize("at", [1150, 1170, 1195, 1210])
def test_a_poll_is_not_started_while_input_waits(at):
    fast = jogprog.build(jogprog.Options(cart=True, fin_ms=100))      # a poll due every 100 ms
    r = Run([(100, b"A02")] + keepalives(b"A02", 200, 1100) + [(at, b".")], latency_ms=80, image=fast, until=1800)
    t_halt, = r.at("halt")
    polls = [t for t in r.at("robotisfinished") if t < t_halt]
    assert len(polls) >= 5, "the polls run while nothing waits"
    assert [t for t in polls if t >= at] == [], "none is started once the `.` is there"
    assert t_halt - at <= 80 + 10, "at most the call already under way, and the loop's 10 ms wait"


def test_settle_waits_for_finished_before_the_next_start():
    arm = Arm(halt_ms=300)
    r = Run([(100, b"A02"), (200, b"c02")] + keepalives(b"c02", 300, 1000) + [(1000, b"."), (1500, b"q")], arm=arm)
    assert r.lines == ["jogd ready", "+A02", "-", "+c02", "-"]
    t_halt = r.at("halt")[0]
    t_next = r.at("jog_ws")[1]
    assert t_next >= t_halt + 300
    assert r.printed("-")[0] == t_halt


def test_settle_gives_up_after_settle_ms():
    arm = Arm(halt_ms=100_000)
    r = Run([(100, b"A02"), (200, b"c02")] + keepalives(b"c02", 300, 5000) + [(5000, b"."), (8000, b"q")], arm=arm,
            until=20_000)
    t_halt = r.at("halt")[0]
    t_next = r.at("jog_ws")[1]
    settle = jogprog.Options().settle_ms
    assert settle <= t_next - t_halt <= settle + 20
    assert r.lines[:4] == ["jogd ready", "+A02", "-", "+c02"]


def test_the_dead_man_still_fires_with_the_idle_work():
    r = Run([(100, b"A02"), (150, b"p"), (1500, b"q")], latency_ms=80)
    t_halt, = r.at("halt")
    started = r.at("jog_ws")[0] + 80
    assert 300 <= t_halt - started <= 320
    assert r.lines == ["jogd ready", "+A02", "P " + " ".join(str(v) for v in PULSES[:6]), "T"]


# ---- SIGABRT ---------------------------------------------------------------------------------------

def test_jogd_is_killed_by_sigabrt_and_jogd_c_survives_it():
    killed = Run([(100, b"A02")] + keepalives(b"A02", 200, 600), argv=("1",), image=JOGD, signals=[(450, 6)])
    assert killed.killed == 6
    arm = Arm(error_word=-0x4000006)
    r = Run([(100, b"A02")] + keepalives(b"A02", 200, 600) + [(700, b"."), (800, b"b02"), (900, b"."), (1200, b"q")],
            arm=arm, signals=[(450, 6)])
    assert r.killed is None and r.code == 0
    assert r.lines == ["jogd ready", "+A02", "A -67108870 Path error", "+b02", "-"]
    names = [n for n in r.names if n not in ("robotisfinished", "pos_get")]
    assert names[:10] == ["ctl_get", "speed_get", "online", "speed_set", "jog_ws", "robot_error_get", "online",
                          "jog_ws", "halt", "online"]
    assert r.args("online")[:2] == [(1,), (1,)]


def test_sigabrt_during_the_lockout_waits_it_out():
    arm = Arm(error_word=0, error_results=(-42, 0))
    r = Run([(100, b"A02"), (200, b"A02"), (2000, b"q")], arm=arm, signals=[(250, 6)])
    assert r.lines == ["jogd ready", "+A02", "A abort"]
    t1, t2 = r.at("robot_error_get")
    assert t2 - t1 >= jogprog.Options().abort_wait_ms
    assert r.at("online")[1] >= t2


def test_sigabrt_in_joint_mode_puts_manual_mode_off():
    r = Run([(100, b"A02"), (200, b"A02"), (1000, b"q")], argv=("1",), signals=[(250, 6)])
    assert r.lines == ["jogd ready", "+A02", "A abort"]
    names = [n for n in r.names if n not in ("robotisfinished", "pos_get")]
    assert names == ["ctl_get", "speed_get", "speed_set", "man_mode_set", "man_move_start", "robot_error_get",
                     "man_mode_set", "speed_set", "ctl_rel"]
    assert r.args("man_mode_set") == [(1,), (0,)]


def test_sigint_still_kills_jogd_c():
    r = Run([(100, b"A02")] + keepalives(b"A02", 200, 600), signals=[(450, 7)])
    assert r.killed == 7
