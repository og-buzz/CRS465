"""a465/jogprog.py: the jogd program, run in a465/r3vm.py against a fake robot library and a fake
console whose bytes arrive at given times (milliseconds on the interpreter's clock, which only the
program's own delay() and the fakes move).

Every jog must be the teach pendant's sequence (bin/pendant 0x0d40/0x06d7 to start, 0x07d9 to
stop): speed_set only for a new speed, man_mode_set(mode), man_move_start; then man_move_stop,
halt only if that failed, man_mode_set(0). The committed builds under controller/programs/ must be
what jogprog builds now.
"""
import re

import pytest

from a465 import crosprog, jogprog, r3asm, r3dis, r3vm

NAMES = {0x02: "ctl_get", 0x04: "ctl_rel", 0x0E: "pos_get", 0x21: "halt", 0x43: "speed_get", 0x44: "speed_set",
         0x52: "man_mode_set", 0x53: "man_move_start", 0x54: "man_move_stop", 0x55: "robot_mode_get"}
IMAGE = jogprog.build()
NOSPEEDSET = jogprog.build(jogprog.Options(speed_set=False))
COUNTS = [4225, 0, -1, -24830, 2147483647, -2147483648, 77, 88]     # axes 7 and 8 are not printed
SAVED_SPEED = 40
SETUP = [("ctl_get",), ("speed_get",)]
P_LINE = "P 4225 0 -1 -24830 2147483647 -2147483648"


def go(joint, speed, speed_set=True, mode=1):
    """The pendant's start of one jog."""
    return ([("speed_set", speed)] if speed_set else []) + [("man_mode_set", mode), ("man_move_start", joint, speed)]


def stop(joint, halt=False):
    """The pendant's stop of one jog."""
    return [("man_move_stop", abs(joint))] + ([("halt",)] if halt else []) + [("man_mode_set", 0)]


def end(speed_was_set=True):
    return ([("speed_set", SAVED_SPEED)] if speed_was_set else []) + [("ctl_rel",)]


def speed_get(vm, args):
    vm.store(args[0], 0, SAVED_SPEED)
    return 0


def pos_get(vm, args):
    r3vm.store_location(vm, args[1], COUNTS)
    return 0


def mode_reads(value):
    def robot_mode_get(vm, args):
        vm.store(args[0], 0, value)
        return 0
    return robot_mode_get


class Run:
    def __init__(self, inputs, argv=(), results=None, image=IMAGE, until=60_000, files=None, latency_ms=0, step_us=0):
        res = {0x43: speed_get, 0x0E: pos_get}
        res.update(results or {})
        self.robot = r3vm.FakeRobot(res, latency_ms=latency_ms)
        self.syslib = r3vm.FakeSyslib({13: "not the controlling client", 5: "I/O error"})
        self.vm = r3vm.VM(image, argv=["/tmp/jogd", *argv], libs={"robotlib": self.robot, "syslib": self.syslib},
                          files=files or {"/dev/cin": r3vm.Input(inputs)}, step_us=step_us)
        try:
            self.code = self.vm.run(until_ms=until)
        except r3vm.Deadline:
            self.code = None
        self.calls = [(NAMES[n],) + (() if n in (0x43, 0x55) else args[:1] if n == 0x0E else args)
                      for n, args in self.robot.calls]
        self.lines = self.vm.stdout().splitlines()

    def at(self, name):
        """Clock times of the calls to `name`."""
        return [t for (n, _), t in zip(self.robot.calls, self.robot.times) if NAMES[n] == name]

    def printed(self, text):
        return [t for t, fd, s in self.vm.events if fd == 1 and s == text + "\n"]


def assert_pendant_order(calls, restart=False):
    """man_move_start only right after man_mode_set(mode) (or, in variant r, on the moving joint);
    after every man_move_stop (and its halt, if any) man_mode_set(0); no start in mode 0."""
    motion = [c for c in calls if c[0] not in ("pos_get",)]
    manual = False
    for k, c in enumerate(motion):
        if c[0] == "man_mode_set":
            manual = c[1] != 0
        if c[0] == "man_move_start":
            assert manual, motion[:k + 1]
            assert motion[k - 1][0] == "man_mode_set" or (restart and motion[k - 1][0] in ("speed_set", "man_move_start"))
        if c[0] == "man_move_stop":
            after = motion[k + 1:k + 3]
            assert after[0] == ("man_mode_set", 0) or (after[0] == ("halt",) and after[1] == ("man_mode_set", 0)), after
    assert not manual, "manual mode left on"


@pytest.mark.parametrize("name,image", [("jogd", IMAGE), ("jogd-nospeedset", NOSPEEDSET)])
def test_built_files_are_current(name, image):
    out = jogprog.OUT
    if not (out / name).exists():
        pytest.skip(f"controller/programs/{name} has not been built")
    assert (out / name).read_bytes() == image
    assert (out / f"{name}.r3s").read_text() == jogprog.source(jogprog.BUILDS[name])
    assert (out / f"{name}.lst").read_text() == jogprog.listing(image, name)
    assert jogprog.build(jogprog.BUILDS[name]) == image


@pytest.mark.parametrize("image", [IMAGE, NOSPEEDSET])
def test_file_is_a_program_for_this_controller(image):
    assert image[:4] == bytes.fromhex("1fabad10")                    # not gzip: the firmware refuses that
    assert crosprog.check_ok(image)
    stamps = [(lib.name, lib.slot, lib.build_time, lib.signature) for lib in crosprog.parse(image).libs]
    assert stamps == [("syslib", 2, 0x377A8993, 0x0A0D4875), ("robotlib", 3, 0x377A899B, 0xAE952686)]
    assert r3asm.assemble(r3asm.lift(image, "jogd")) == image
    assert len(image) < 4096


def test_the_nospeedset_build_has_no_speed_calls():
    listing = jogprog.listing(NOSPEEDSET)
    assert "speed_set" not in listing and "speed_get" not in listing
    assert "call robotlib:0x44 speed_set" in jogprog.listing(IMAGE)


@pytest.mark.parametrize("image", [IMAGE, NOSPEEDSET])
def test_every_function_frame_covers_its_locals(image):
    insns = r3dis.decode(r3dis.Module(image))
    frame = None
    for ins in insns[1:]:
        if ins.op == 0x54 and ins.kind != "data":
            frame = -r3dis.s16(ins.word >> 16)
            continue
        for n in re.findall(r"\bL(\d+)", ins.text):
            assert 1 <= int(n) <= frame, (hex(ins.addr), ins.text)


def test_ready_and_quit_touch_no_mode():
    r = Run([(50, b"q")])
    assert r.code == 0
    assert r.lines == ["jogd ready"]
    assert r.calls == SETUP + end(speed_was_set=False)
    assert r.vm.fds == {3: ("/dev/cin", 5)}                          # O_RDONLY | O_NONBLOCK


def test_one_jog_is_the_pendants_sequence():
    r = Run([(0, b"A07"), (100, b"A07"), (200, b"A07"), (260, b"."), (300, b"q")])
    assert r.code == 0
    assert r.lines == ["jogd ready", "+A07", "-"]
    assert r.calls == SETUP + go(1, 7) + stop(1) + end()
    assert r.at("man_move_stop") == [260] and r.at("man_mode_set") == [0, 260]
    assert r.printed("+A07") == [0] and r.printed("-") == [260]     # printed after the whole sequence
    assert_pendant_order(r.calls)


def test_speed_set_only_for_a_speed_not_in_force():
    r = Run([(0, b"A07"), (100, b"."), (200, b"A07"), (300, b"."), (400, b"A40"), (500, b"q")], argv=("1", "40"))
    assert r.calls == SETUP + go(1, 7) + stop(1) + go(1, 7, speed_set=False) + stop(1) + go(1, 40) + stop(1) + [("ctl_rel",)]
    r = Run([(0, b"A40"), (100, b"q")], argv=("1", "40"))             # the speed found at start
    assert r.calls == SETUP + go(1, 40, speed_set=False) + stop(1) + [("ctl_rel",)]


def test_negative_direction_and_other_joints():
    r = Run([(0, b"f15"), (100, b"."), (200, b"C03"), (300, b"."), (400, b"q")])
    assert r.lines == ["jogd ready", "+f15", "-", "+C03", "-"]
    assert r.calls == SETUP + go(-6, 15) + stop(6) + go(3, 3) + stop(3) + end()


def test_change_of_joint_stops_with_the_whole_sequence_first():
    r = Run([(0, b"A07"), (100, b"b07"), (200, b"B07"), (300, b"q")])
    assert r.lines == ["jogd ready", "+A07", "-", "+b07", "-", "+B07", "-"]
    assert r.calls == SETUP + go(1, 7) + stop(1) + go(-2, 7, False) + stop(2) + go(2, 7, False) + stop(2) + end()
    assert_pendant_order(r.calls)


def test_reversal_is_stop_mode_off_then_mode_on_start():
    r = Run([(0, b"C05"), (100, b"c05"), (200, b"C05"), (300, b"."), (400, b"q")])
    assert r.lines == ["jogd ready", "+C05", "-", "+c05", "-", "+C05", "-"]
    assert r.calls == SETUP + go(3, 5) + stop(3) + go(-3, 5, False) + stop(3) + go(3, 5, False) + stop(3) + end()
    assert_pendant_order(r.calls)


def test_speed_change_is_stop_then_start_by_default():
    r = Run([(0, b"A07"), (100, b"A09"), (200, b"q")])
    assert r.lines == ["jogd ready", "+A07", "-", "+A09", "-"]
    assert r.calls == SETUP + go(1, 7) + stop(1) + go(1, 9) + stop(1) + end()
    assert Run([(0, b"A07"), (100, b"A09"), (200, b"q")], argv=("1", "25", "s")).calls == r.calls
    assert_pendant_order(r.calls)


def test_variant_r_only_when_asked():
    r = Run([(0, b"A07"), (100, b"A09"), (150, b"A09"), (200, b"q")], argv=("1", "25", "r"))
    assert r.lines == ["jogd ready", "+A07", "+A09", "-"]
    assert r.calls == SETUP + go(1, 7) + [("speed_set", 9), ("man_move_start", 1, 9)] + stop(1) + end()
    assert_pendant_order(r.calls, restart=True)
    r = Run([(0, b"A07"), (100, b"B07"), (200, b"q")], argv=("1", "25", "r"))   # another joint: stop first
    assert r.calls == SETUP + go(1, 7) + stop(1) + go(2, 7, False) + stop(2) + end()


def test_deadman_does_not_fire_while_keepalives_come_every_250ms():
    keep = [(t, b"D20") for t in range(0, 2001, 250)]
    r = Run(keep + [(2100, b"."), (2200, b"q")])
    assert r.lines == ["jogd ready", "+D20", "-"]
    assert r.calls == SETUP + go(4, 20) + stop(4) + end()
    assert r.at("man_move_start") == [0] and r.at("man_move_stop") == [2100]


def test_deadman_fires_300ms_after_the_last_keepalive_with_the_whole_sequence():
    keep = [(t, b"D20") for t in range(0, 1001, 250)]                  # the last at 1000
    r = Run(keep + [(3000, b"q")])
    assert r.lines == ["jogd ready", "+D20", "T"]
    assert r.calls == SETUP + go(4, 20) + stop(4) + end()
    (stopped,) = r.at("man_move_stop")
    assert 1300 <= stopped <= 1300 + jogprog.Options.poll_ms
    assert r.at("man_mode_set")[-1] == stopped and r.printed("T") == [stopped]


def test_deadman_measures_silence_not_time_since_the_start():
    r = Run([(0, b"A05"), (290, b"A05"), (580, b"A05"), (870, b"A05"), (1500, b"q")])
    (stopped,) = r.at("man_move_stop")
    assert 1170 <= stopped <= 1180                                    # 300 after 870, not after 0


def test_after_the_deadman_the_command_starts_afresh():
    r = Run([(0, b"A05"), (1000, b"A05"), (1100, b"q")])
    assert r.lines == ["jogd ready", "+A05", "T", "+A05", "-"]
    assert r.calls == SETUP + go(1, 5) + stop(1) + go(1, 5, False) + stop(1) + end()


# ---- the dead-man with a controller whose calls take time (about 0.1 s each on the arm)

SLOW = 100


def test_slow_start_with_keepalives_every_100ms_gives_no_T():
    """On the arm the first move printed `+F05` 295 ms after the command and then `T`: the
    keep-alives that queued during the start sequence were not read before the 300 ms check."""
    keep = [(t, b"F05") for t in range(500, 2500, 100)]
    r = Run(keep + [(2600, b"."), (2800, b"q")], latency_ms=SLOW)
    assert r.lines == ["jogd ready", "+F05", "-"]
    assert r.printed("+F05") == [800]                                 # speed_set, man_mode_set, man_move_start
    assert r.calls == SETUP + go(6, 5) + stop(6) + end()


def test_slow_stop_then_start_on_a_change_with_keepalives_gives_no_T():
    keep = [(t, b"F05") for t in range(500, 1500, 100)] + [(t, b"f07") for t in range(1500, 3000, 100)]
    r = Run(keep + [(3100, b"."), (3300, b"q")], latency_ms=SLOW)
    assert r.lines == ["jogd ready", "+F05", "-", "+f07", "-"]


def test_p_every_200ms_during_a_move_with_keepalives_gives_no_T():
    keep = [(t, b"C04") for t in range(500, 3000, 100)]
    polls = [(t, b"p") for t in range(950, 2900, 200)]
    r = Run(keep + polls + [(3100, b"."), (3300, b"q")], latency_ms=SLOW)
    assert "T" not in r.lines
    assert r.lines[:2] == ["jogd ready", "+C04"] and r.lines[-1] == "-"
    assert r.lines.count(P_LINE) == len(polls)
    assert r.at("man_move_start") == [r.at("man_mode_set")[0] + SLOW]  # one start, no restart


def test_real_silence_still_gives_T_at_300ms_with_a_slow_controller():
    keep = [(t, b"C04") for t in range(500, 1501, 100)]                # the last at 1500
    r = Run(keep + [(3000, b"q")], latency_ms=SLOW)
    assert r.lines == ["jogd ready", "+C04", "T"]
    (stopped,) = r.at("man_move_stop")
    assert 1800 <= stopped <= 1800 + 2 * jogprog.Options.poll_ms


def test_a_stream_of_junk_cannot_hold_the_move():
    """Instructions cost 50 us here, so 1500 junk bytes keep input waiting for seconds: the 300 ms
    check never runs, the 1000 ms limit after every byte stops the move."""
    r = Run([(500, b"F05"), (600, b"x" * 1500 + b"q")], step_us=50)
    assert r.lines == ["jogd ready", "+F05", "T"]
    (started,) = r.printed("+F05")
    (stopped,) = r.at("man_move_stop")
    assert 1000 <= stopped - started <= 1010
    assert r.vm.clock - started > 2000                                 # the junk lasted well past both limits


def test_deadman_options():
    image = jogprog.build(jogprog.Options(deadman_ms=500))
    r = Run([(0, b"A05"), (400, b"A05"), (2000, b"q")], image=image)
    (stopped,) = r.at("man_move_stop")
    assert 900 <= stopped <= 910
    image = jogprog.build(jogprog.Options(deadman_long_ms=600))
    r = Run([(500, b"F05"), (600, b"x" * 1500 + b"q")], image=image, step_us=50)
    (started,) = r.printed("+F05")
    (stopped,) = r.at("man_move_stop")
    assert 600 <= stopped - started <= 610


def test_q_while_moving_stops_with_the_whole_sequence_and_releases():
    r = Run([(0, b"E12"), (100, b"q"), (200, b"A05")])
    assert r.code == 0
    assert r.lines == ["jogd ready", "+E12", "-"]
    assert r.calls == SETUP + go(5, 12) + stop(5) + end()
    assert r.at("ctl_rel") == [100]


def test_failed_start_is_followed_by_the_whole_stop_sequence():
    r = Run([(0, b"A07"), (100, b"A07"), (200, b"A07"), (700, b"B05"), (800, b"."), (900, b"q")],
            results={0x53: [-13, 0]})
    assert r.code == 0
    assert r.lines == ["jogd ready", "E -13 not the controlling client", "-", "+B05", "-"]
    assert r.calls == SETUP + go(1, 7) + stop(1) + go(2, 5) + stop(2) + end()
    assert r.at("man_move_start") == [0, 700]                         # repeats of the failed command: ignored
    assert r.syslib.calls[0] == (0x07, (13,))                          # str_error(-r)
    assert_pendant_order(r.calls)


def test_failed_command_can_be_retried_after_a_dot():
    r = Run([(0, b"A07"), (100, b"."), (200, b"A07"), (300, b"q")], results={0x53: [-13, 0]})
    assert r.lines == ["jogd ready", "E -13 not the controlling client", "-", "+A07", "-"]
    assert r.calls == SETUP + go(1, 7) + stop(1) + go(1, 7, False) + stop(1) + end()


def test_failed_restart_in_variant_r_stops_the_move_in_force():
    r = Run([(0, b"A07"), (100, b"A09"), (200, b"A09"), (300, b"q")], argv=("1", "25", "r"), results={0x53: [0, -22, 0]})
    assert r.lines == ["jogd ready", "+A07", "E -22 error 22", "-"]
    assert r.calls == SETUP + go(1, 7) + [("speed_set", 9), ("man_move_start", 1, 9)] + stop(1) + end()


def test_failed_speed_set_starts_nothing():
    r = Run([(0, b"A07"), (100, b"q")], results={0x44: -22})
    assert r.lines == ["jogd ready", "E -22 error 22"]
    assert r.calls == SETUP + [("speed_set", 7), ("ctl_rel",)]


def test_refused_mode_asks_robot_mode_get_and_starts_nothing():
    r = Run([(0, b"A07"), (100, b"A07"), (200, b"q")], results={0x52: -13})
    assert r.code == 0
    assert r.lines == ["jogd ready", "E -13 not the controlling client"]
    assert r.calls == SETUP + [("speed_set", 7), ("man_mode_set", 1), ("robot_mode_get",)] + end()


def test_when_robot_mode_get_says_manual_mode_is_on_it_is_turned_off_at_exit():
    r = Run([(0, b"A07"), (100, b"."), (200, b"q")], results={0x52: [-13, 0], 0x55: mode_reads(2)})
    assert r.lines == ["jogd ready", "E -13 not the controlling client"]
    assert r.calls == SETUP + [("speed_set", 7), ("man_mode_set", 1), ("robot_mode_get",), ("man_mode_set", 0)] + end()


def test_stop_failure_calls_halt_then_mode_off():
    r = Run([(0, b"A07"), (100, b"."), (200, b"q")], results={0x54: -5})
    assert r.lines == ["jogd ready", "+A07", "E -5 I/O error", "-"]
    assert r.calls == SETUP + go(1, 7) + stop(1, halt=True) + end()


def test_halt_failure_is_printed_too():
    r = Run([(0, b"A07"), (500, b"q")], results={0x54: -5, 0x21: -10})
    assert r.lines == ["jogd ready", "+A07", "E -5 I/O error", "E -10 error 10", "T"]
    assert r.calls == SETUP + go(1, 7) + stop(1, halt=True) + end()


def test_mode_off_failure_is_printed_and_retried_at_exit():
    r = Run([(0, b"A07"), (100, b"."), (200, b"q")], results={0x52: [0, -13, 0], 0x55: mode_reads(2)})
    assert r.lines == ["jogd ready", "+A07", "E -13 not the controlling client", "-"]
    assert r.calls == SETUP + go(1, 7) + stop(1) + [("robot_mode_get",), ("man_mode_set", 0)] + end()


def test_speed_is_clamped_to_the_maximum():
    r = Run([(0, b"A50"), (100, b"A99"), (200, b"q")])
    assert r.lines == ["jogd ready", "+A25", "-"]                      # A99 is the same command once clamped
    assert r.calls == SETUP + go(1, 25) + stop(1) + end()


def test_garbage_is_ignored_and_a_letter_resynchronises():
    r = Run([(0, b"xyz\r\n#12G55g55 "), (100, b"A0"), (150, b"B1"), (160, b"!"), (170, b"2"), (300, b"\x7f."), (400, b"q")])
    assert r.lines == ["jogd ready", "+B12", "-"]
    assert r.calls == SETUP + go(2, 12) + stop(2) + end()


def test_speed_00_and_a_dot_when_idle_do_nothing():
    r = Run([(0, b"A00"), (100, b"."), (200, b"q")])
    assert r.lines == ["jogd ready"]
    assert r.calls == SETUP + end(speed_was_set=False)


USAGE = ("/tmp/jogd: invalid command line arguments.\nUsage: /tmp/jogd [mode [maxspeed [s|r]]]\n"
         "  r: speed change without a stop (does not change the speed here)\n")


@pytest.mark.parametrize("argv", [("0",), ("5",), ("x",), ("1", "0"), ("1", "100"), ("1", "25", "x"), ("1", "25", "r", "y")])
def test_bad_arguments_print_usage_and_touch_nothing(argv):
    r = Run([], argv=argv)
    assert r.code == 1
    assert r.vm.stderr() == USAGE
    assert r.calls == [] and r.vm.fds == {}


def test_mode_argument_is_passed_to_man_mode_set():
    r = Run([(0, b"B10"), (100, b"q")], argv=("3",))
    assert r.calls == SETUP + go(2, 10, mode=3) + stop(2) + end()


def test_no_control_no_motion():
    r = Run([(0, b"A07")], results={0x02: -13})
    assert r.code == 1
    assert r.lines == ["E -13 not the controlling client"]
    assert r.calls == [("ctl_get",)]


def test_speed_get_failure_releases_control():
    r = Run([(0, b"A07")], results={0x43: -13})
    assert r.code == 1
    assert r.lines == ["E -13 not the controlling client"]
    assert r.calls == SETUP + [("ctl_rel",)]


def test_no_console_no_control():
    r = Run([], files={"/dev/other": r3vm.Input()})
    assert r.code == 1 and r.lines == ["E -2 error 2"] and r.calls == []


class BrokenInput(r3vm.Input):
    def read(self, now, n):
        return -5 if now >= 200 else super().read(now, n)


def test_input_error_stops_with_the_whole_sequence_releases_and_exits_1():
    r = Run([], files={"/dev/cin": BrokenInput([(0, b"A07"), (100, b"A07")])})
    assert r.code == 1
    assert r.lines == ["jogd ready", "+A07", "E -5 I/O error", "-"]
    assert r.calls == SETUP + go(1, 7) + stop(1) + end()


@pytest.mark.parametrize("variant,lines,calls", [
    ("s", ["+A07", "-", "+A09", P_LINE, "-", "+a09", "-"],
     go(1, 7, False) + stop(1) + go(1, 9, False) + [("pos_get", 1)] + stop(1) + go(-1, 9, False) + stop(1)),
    ("r", ["+A07", "+A09", P_LINE, "-", "+a09", "-"],
     go(1, 7, False) + [("man_move_start", 1, 9), ("pos_get", 1)] + stop(1) + go(-1, 9, False) + stop(1)),
])
def test_build_without_speed_set(variant, lines, calls):
    r = Run([(0, b"A07"), (100, b"A09"), (150, b"p"), (200, b"a09"), (300, b"q")], argv=("1", "25", variant),
            image=NOSPEEDSET)
    assert r.lines == ["jogd ready"] + lines
    assert r.calls == [("ctl_get",)] + calls + [("ctl_rel",)]
    assert_pendant_order(r.calls, restart=variant == "r")


# ---- p: the position readout

def test_p_prints_the_six_counts_from_pos_get_actual():
    r = Run([(0, b"p"), (100, b"q")])
    assert r.code == 0
    assert r.lines == ["jogd ready", P_LINE]
    assert r.calls == SETUP + [("pos_get", 1)] + end(speed_was_set=False)
    (number, (kind, loc)), = [c for c in r.robot.calls if c[0] == 0x0E]
    assert loc >> 16 == r3vm.DATA                                      # a location in jogd's data segment
    assert re.fullmatch(r"P( -?\d+){6}", P_LINE)


def test_p_while_moving_does_not_stop_restart_or_feed_the_deadman():
    """Keep-alives every 250 ms until 500, then only p every 100 ms: the dead-man fires 300 ms after
    the last keep-alive, not after the last p, and the p's in between change nothing."""
    keep = [(0, b"C09"), (250, b"C09"), (500, b"C09")]
    polls = [(t, b"p") for t in range(100, 1000, 100)]
    r = Run(keep + polls + [(1200, b"q")])
    assert r.at("man_move_start") == [0]
    (stopped,) = r.at("man_move_stop")
    assert 800 <= stopped <= 800 + jogprog.Options.poll_ms
    assert r.lines[:2] == ["jogd ready", "+C09"] and r.lines.count("T") == 1
    assert r.lines.count(P_LINE) == len(polls)
    assert [c for c in r.calls if c[0] != "pos_get"] == SETUP + go(3, 9) + stop(3) + end()


def test_p_inside_a_command_leaves_it_intact():
    r = Run([(0, b"Ap0p5"), (100, b"."), (200, b"q")])
    assert r.lines == ["jogd ready", P_LINE, P_LINE, "+A05", "-"]
    assert r.calls == SETUP + [("pos_get", 1), ("pos_get", 1)] + go(1, 5) + stop(1) + end()


def test_p_error_prints_E_and_changes_nothing():
    r = Run([(0, b"B03"), (100, b"p"), (150, b"B03"), (200, b"p"), (250, b"."), (300, b"q")],
            results={0x0E: [-5, -13]})
    assert r.lines == ["jogd ready", "+B03", "E -5 I/O error", "E -13 not the controlling client", "-"]
    assert [c for c in r.calls if c[0] != "pos_get"] == SETUP + go(2, 3) + stop(2) + end()
    assert r.at("man_move_stop") == [250]


def test_p_when_idle_and_the_position_type_option():
    r = Run([(0, b"pp"), (100, b"q")], image=jogprog.build(jogprog.Options(pos_type=2)))
    assert r.lines == ["jogd ready", P_LINE, P_LINE]
    assert [c for c in r.calls if c[0] == "pos_get"] == [("pos_get", 2), ("pos_get", 2)]


def test_idle_loop_waits_with_delay_and_never_blocks():
    r = Run([(1000, b"q")])
    delays = [args for n, args in r.syslib.calls if n == 0x14]
    assert delays and set(delays) == {(jogprog.Options.poll_ms,)}
    assert 1000 // jogprog.Options.poll_ms <= len(delays) <= 1000 // jogprog.Options.poll_ms + 1
