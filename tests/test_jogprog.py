"""a465/jogprog.py: the jogd program, run in a465/r3vm.py against a fake robot library and a fake
console whose bytes arrive at given times (milliseconds on the interpreter's clock, which only the
program's own delay() and the fakes move).

The committed build under controller/programs/ must be what jogprog builds now.
"""
import re

import pytest

from a465 import crosprog, jogprog, r3asm, r3dis, r3vm

NAMES = {0x02: "ctl_get", 0x04: "ctl_rel", 0x21: "halt", 0x43: "speed_get", 0x44: "speed_set",
         0x52: "man_mode_set", 0x53: "man_move_start", 0x54: "man_move_stop"}
IMAGE = jogprog.build()
SAVED_SPEED = 40
SETUP = [("ctl_get",), ("speed_get",), ("man_mode_set", 1)]


def end(speed_was_set=True):
    return [("man_mode_set", 0)] + ([("speed_set", SAVED_SPEED)] if speed_was_set else []) + [("ctl_rel",)]


def speed_get(vm, args):
    vm.store(args[0], 0, SAVED_SPEED)
    return 0


class Run:
    def __init__(self, inputs, argv=(), results=None, image=IMAGE, until=60_000, files=None):
        res = {0x43: speed_get}
        res.update(results or {})
        self.robot = r3vm.FakeRobot(res)
        self.syslib = r3vm.FakeSyslib({13: "not the controlling client", 5: "I/O error"})
        self.vm = r3vm.VM(image, argv=["/tmp/jogd", *argv], libs={"robotlib": self.robot, "syslib": self.syslib},
                          files=files or {"/dev/cin": r3vm.Input(inputs)})
        try:
            self.code = self.vm.run(until_ms=until)
        except r3vm.Deadline:
            self.code = None
        self.calls = [(NAMES[n],) + (() if n == 0x43 else args) for n, args in self.robot.calls]
        self.lines = self.vm.stdout().splitlines()

    def at(self, name):
        """Clock times of the calls to `name`."""
        return [t for (n, _), t in zip(self.robot.calls, self.robot.times) if NAMES[n] == name]

    def printed(self, text):
        return [t for t, fd, s in self.vm.events if fd == 1 and s == text + "\n"]


def test_built_files_are_current():
    out = jogprog.OUT
    if not (out / "jogd").exists():
        pytest.skip("controller/programs/jogd has not been built")
    assert (out / "jogd").read_bytes() == IMAGE
    assert (out / "jogd.r3s").read_text() == jogprog.source()
    assert (out / "jogd.lst").read_text() == jogprog.listing(IMAGE)


def test_file_is_a_program_for_this_controller():
    assert IMAGE[:4] == bytes.fromhex("1fabad10")                    # not gzip: the firmware refuses that
    assert crosprog.check_ok(IMAGE)
    stamps = [(lib.name, lib.slot, lib.build_time, lib.signature) for lib in crosprog.parse(IMAGE).libs]
    assert stamps == [("syslib", 2, 0x377A8993, 0x0A0D4875), ("robotlib", 3, 0x377A899B, 0xAE952686)]
    assert r3asm.assemble(r3asm.lift(IMAGE, "jogd")) == IMAGE
    assert len(IMAGE) < 4096


def test_every_function_frame_covers_its_locals():
    insns = r3dis.decode(r3dis.Module(IMAGE))
    frame = None
    for ins in insns[1:]:
        if ins.op == 0x54 and ins.kind != "data":
            frame = -r3dis.s16(ins.word >> 16)
            continue
        for n in re.findall(r"\bL(\d+)", ins.text):
            assert 1 <= int(n) <= frame, (hex(ins.addr), ins.text)


def test_ready_and_quit():
    r = Run([(50, b"q")])
    assert r.code == 0
    assert r.lines == ["jogd ready"]
    assert r.calls == SETUP + end(speed_was_set=False)
    assert r.vm.fds == {3: ("/dev/cin", 5)}                          # O_RDONLY | O_NONBLOCK


def test_start_keepalive_stop():
    r = Run([(0, b"A07"), (100, b"A07"), (200, b"A07"), (260, b"."), (300, b"q")])
    assert r.code == 0
    assert r.lines == ["jogd ready", "+A07", "-"]
    assert r.calls == SETUP + [("speed_set", 7), ("man_move_start", 1, 7), ("man_move_stop", 1)] + end()
    assert r.at("man_move_stop") == [260]


def test_negative_direction_and_other_joints():
    r = Run([(0, b"f15"), (100, b"."), (200, b"C03"), (300, b"."), (400, b"q")])
    assert r.lines == ["jogd ready", "+f15", "-", "+C03", "-"]
    moves = [c for c in r.calls if c[0].startswith("man_move")]
    assert moves == [("man_move_start", -6, 15), ("man_move_stop", 6), ("man_move_start", 3, 3), ("man_move_stop", 3)]


def test_change_of_joint_or_direction_stops_first():
    r = Run([(0, b"A07"), (100, b"b07"), (200, b"B07"), (300, b"q")])
    assert r.lines == ["jogd ready", "+A07", "-", "+b07", "-", "+B07", "-"]
    moves = [c for c in r.calls if c[0].startswith("man_move")]
    assert moves == [("man_move_start", 1, 7), ("man_move_stop", 1), ("man_move_start", -2, 7),
                     ("man_move_stop", 2), ("man_move_start", 2, 7), ("man_move_stop", 2)]


def test_speed_change_variant_s_stops_then_starts():
    r = Run([(0, b"A07"), (100, b"A09"), (200, b"q")])                 # s is the default
    assert r.lines == ["jogd ready", "+A07", "-", "+A09", "-"]
    assert r.calls == SETUP + [("speed_set", 7), ("man_move_start", 1, 7), ("man_move_stop", 1),
                               ("speed_set", 9), ("man_move_start", 1, 9), ("man_move_stop", 1)] + end()
    assert Run([(0, b"A07"), (100, b"A09"), (200, b"q")], argv=("1", "25", "s")).calls == r.calls


def test_speed_change_variant_r_restarts_without_stopping():
    r = Run([(0, b"A07"), (100, b"A09"), (150, b"A09"), (200, b"q")], argv=("1", "25", "r"))
    assert r.lines == ["jogd ready", "+A07", "+A09", "-"]
    assert r.calls == SETUP + [("speed_set", 7), ("man_move_start", 1, 7),
                               ("speed_set", 9), ("man_move_start", 1, 9), ("man_move_stop", 1)] + end()


def test_variant_r_still_stops_on_a_change_of_joint():
    r = Run([(0, b"A07"), (100, b"B07"), (200, b"q")], argv=("1", "25", "r"))
    assert r.lines == ["jogd ready", "+A07", "-", "+B07", "-"]


def test_deadman_does_not_fire_while_keepalives_come_every_250ms():
    keep = [(t, b"D20") for t in range(0, 2001, 250)]
    r = Run(keep + [(2100, b"."), (2200, b"q")])
    assert r.lines == ["jogd ready", "+D20", "-"]
    assert r.at("man_move_start") == [0] and r.at("man_move_stop") == [2100]


def test_deadman_fires_300ms_after_the_last_keepalive():
    keep = [(t, b"D20") for t in range(0, 1001, 250)]                  # the last at 1000
    r = Run(keep + [(3000, b"q")])
    assert r.lines == ["jogd ready", "+D20", "T"]
    (stopped,) = r.at("man_move_stop")
    assert 1300 <= stopped <= 1300 + jogprog.Options.poll_ms
    assert r.printed("T") == [stopped]
    assert ("man_move_stop", 4) in r.calls


def test_deadman_measures_silence_not_time_since_the_start():
    r = Run([(0, b"A05"), (290, b"A05"), (580, b"A05"), (870, b"A05"), (1500, b"q")])
    (stopped,) = r.at("man_move_stop")
    assert 1170 <= stopped <= 1180                                    # 300 after 870, not after 0


def test_after_the_deadman_the_command_starts_afresh():
    r = Run([(0, b"A05"), (1000, b"A05"), (1100, b"q")])
    assert r.lines == ["jogd ready", "+A05", "T", "+A05", "-"]
    assert r.at("man_move_start") == [0, 1000]


def test_deadman_options():
    image = jogprog.build(jogprog.Options(deadman_ms=500))
    r = Run([(0, b"A05"), (400, b"A05"), (2000, b"q")], image=image)
    (stopped,) = r.at("man_move_stop")
    assert 900 <= stopped <= 910


def test_q_while_moving_stops_and_releases_control():
    r = Run([(0, b"E12"), (100, b"q"), (200, b"A05")])
    assert r.code == 0
    assert r.lines == ["jogd ready", "+E12", "-"]
    assert r.calls[-4:] == [("man_move_stop", 5)] + end()
    assert r.at("ctl_rel") == [100]


def test_start_error_is_printed_and_leaves_jogd_idle_and_responsive():
    r = Run([(0, b"A07"), (100, b"A07"), (200, b"A07"), (700, b"B05"), (800, b"."), (900, b"q")],
            results={0x53: [-13, 0]})
    assert r.code == 0
    assert r.lines == ["jogd ready", "E -13 not the controlling client", "+B05", "-"]
    moves = [c for c in r.calls if c[0].startswith("man_move")]
    assert moves == [("man_move_start", 1, 7), ("man_move_start", 2, 5), ("man_move_stop", 2)]
    assert r.at("man_move_start") == [0, 700]                         # repeats of the failed command: ignored
    assert r.syslib.calls[0] == (0x07, (13,))                          # str_error(-r)


def test_failed_command_can_be_retried_after_a_stop():
    r = Run([(0, b"A07"), (100, b"."), (200, b"A07"), (300, b"q")], results={0x53: [-13, 0]})
    assert r.lines == ["jogd ready", "E -13 not the controlling client", "+A07", "-"]


def test_failed_restart_in_variant_r_stops_the_move_in_force():
    r = Run([(0, b"A07"), (100, b"A09"), (200, b"A09"), (300, b"q")], argv=("1", "25", "r"), results={0x53: [0, -22, 0]})
    assert r.lines == ["jogd ready", "+A07", "E -22 error 22", "-"]
    moves = [c for c in r.calls if c[0].startswith("man_move")]
    assert moves == [("man_move_start", 1, 7), ("man_move_start", 1, 9), ("man_move_stop", 1)]


def test_failed_speed_set_does_not_start():
    r = Run([(0, b"A07"), (100, b"q")], results={0x44: -22})
    assert r.lines == ["jogd ready", "E -22 error 22"]
    assert not r.at("man_move_start")


def test_stop_failure_calls_halt():
    r = Run([(0, b"A07"), (100, b"."), (200, b"q")], results={0x54: -5})
    assert r.lines == ["jogd ready", "+A07", "E -5 I/O error", "-"]
    i = r.calls.index(("man_move_stop", 1))
    assert r.calls[i + 1] == ("halt",)


def test_halt_failure_is_printed_too():
    r = Run([(0, b"A07"), (500, b"q")], results={0x54: -5, 0x21: -10})
    assert r.lines == ["jogd ready", "+A07", "E -5 I/O error", "E -10 error 10", "T"]


def test_speed_is_clamped_to_the_maximum():
    r = Run([(0, b"A50"), (100, b"A99"), (200, b"q")])
    assert r.lines == ["jogd ready", "+A25", "-"]                      # A99 is the same command once clamped
    assert ("man_move_start", 1, 25) in r.calls and ("speed_set", 25) in r.calls
    r = Run([(0, b"A50"), (100, b"A10"), (200, b"q")], argv=("1", "40"))
    assert r.lines == ["jogd ready", "+A40", "-", "+A10", "-"]


def test_garbage_is_ignored_and_a_letter_resynchronises():
    r = Run([(0, b"xyz\r\n#12G55g55 "), (100, b"A0"), (150, b"B1"), (160, b"!"), (170, b"2"), (300, b"\x7f."), (400, b"q")])
    assert r.lines == ["jogd ready", "+B12", "-"]
    moves = [c for c in r.calls if c[0].startswith("man_move")]
    assert moves == [("man_move_start", 2, 12), ("man_move_stop", 2)]


def test_speed_00_and_a_dot_when_idle_do_nothing():
    r = Run([(0, b"A00"), (100, b"."), (200, b"q")])
    assert r.lines == ["jogd ready"]
    assert r.calls == SETUP + end(speed_was_set=False)


@pytest.mark.parametrize("argv", [("0",), ("5",), ("x",), ("1", "0"), ("1", "100"), ("1", "25", "x"), ("1", "25", "r", "y")])
def test_bad_arguments_print_usage_and_touch_nothing(argv):
    r = Run([], argv=argv)
    assert r.code == 1
    assert r.vm.stderr() == "/tmp/jogd: invalid command line arguments.\nUsage: /tmp/jogd [mode [maxspeed [r|s]]]\n"
    assert r.calls == [] and r.vm.fds == {}


def test_mode_argument_is_passed_to_man_mode_set():
    r = Run([(0, b"q")], argv=("3",))
    assert r.calls[:3] == [("ctl_get",), ("speed_get",), ("man_mode_set", 3)]


def test_no_control_no_motion():
    r = Run([(0, b"A07")], results={0x02: -13})
    assert r.code == 1
    assert r.lines == ["E -13 not the controlling client"]
    assert r.calls == [("ctl_get",)]


def test_mode_set_failure_releases_control():
    r = Run([(0, b"A07")], results={0x52: -13})
    assert r.code == 1
    assert r.lines == ["E -13 not the controlling client"]
    assert r.calls == SETUP + [("ctl_rel",)]


def test_no_console_no_control():
    r = Run([], files={"/dev/other": r3vm.Input()})
    assert r.code == 1 and r.lines == ["E -2 error 2"] and r.calls == []


class BrokenInput(r3vm.Input):
    def read(self, now, n):
        return -5 if now >= 200 else super().read(now, n)


def test_input_error_stops_releases_and_exits_1():
    r = Run([], files={"/dev/cin": BrokenInput([(0, b"A07"), (100, b"A07")])})
    assert r.code == 1
    assert r.lines == ["jogd ready", "+A07", "E -5 I/O error", "-"]
    assert r.calls[-4:] == [("man_move_stop", 1)] + end()


def test_build_without_speed_set():
    image = jogprog.build(jogprog.Options(speed_set=False))
    r = Run([(0, b"A07"), (100, b"A09"), (200, b"q")], image=image)
    assert r.lines == ["jogd ready", "+A07", "-", "+A09", "-"]
    assert r.calls == [("ctl_get",), ("man_mode_set", 1), ("man_move_start", 1, 7), ("man_move_stop", 1),
                       ("man_move_start", 1, 9), ("man_move_stop", 1), ("man_mode_set", 0), ("ctl_rel",)]


def test_idle_loop_waits_with_delay_and_never_blocks():
    r = Run([(1000, b"q")])
    delays = [args for n, args in r.syslib.calls if n == 0x14]
    assert delays and set(delays) == {(jogprog.Options.poll_ms,)}
    assert 1000 // jogprog.Options.poll_ms <= len(delays) <= 1000 // jogprog.Options.poll_ms + 1
