"""a465/r3vm.py agrees with the controller on vendor programs whose behaviour was seen on 2026-11-24:
the re-stamped homezc with no argument prints its usage and exits 1; with `6` it calls robotlib 0x4a
with (6, address of its result) and releases control; homezc-offset then prints the offset the
library wrote. The error path runs the program's own try/handler code.

The programs are this unit's git-ignored copies under sessions/controller-files/patched/.
"""
from pathlib import Path

import pytest

from a465 import r3asm, r3dis, r3vm

PATCHED = Path(__file__).resolve().parent.parent / "sessions" / "controller-files" / "patched"
HOMEZC, CTL_REL = 0x4A, 0x04


def program(name):
    path = PATCHED / name
    if not path.exists():
        pytest.skip(f"{name} is not here")
    return path.read_bytes()


def writes_offset(value, result=0):
    def homezc(vm, args):
        vm.store(args[1], 0, value)
        return result
    return homezc


@pytest.mark.parametrize("name", ["homezc-plain", "homezc-offset"])
def test_no_argument_prints_usage_and_exits_1(name):
    vm = r3vm.VM(program(name), argv=["homezc"])
    assert vm.run() == 1
    assert vm.stderr() == "homezc: invalid command line arguments.\nUsage: homezc [newspeed]\n"
    assert vm.stdout() == "" and vm.libs["robotlib"].calls == []


def test_axis_6_calls_homezc_with_the_axis_and_an_address():
    robot = r3vm.FakeRobot({HOMEZC: writes_offset(251)})
    vm = r3vm.VM(program("homezc-plain"), argv=["homezc", "6"], libs={"robotlib": robot})
    assert vm.run() == 0
    (number, (axis, address)), release = robot.calls
    assert (number, axis, release) == (HOMEZC, 6, (CTL_REL, ()))
    assert address >> 16 == r3vm.STACK                       # &L10, a word of main's frame
    assert vm.stderr() == "" and vm.stdout() == ""


def test_offset_version_prints_what_the_library_wrote():
    robot = r3vm.FakeRobot({HOMEZC: writes_offset(251)})
    vm = r3vm.VM(program("homezc-offset"), argv=["/tmp/homezo", "6"], libs={"robotlib": robot})
    assert vm.run() == 0
    assert vm.stderr() == "zero-cross offset: 251\n"
    assert [n for n, _ in robot.calls] == [HOMEZC, CTL_REL]


@pytest.mark.parametrize("arg", ["0", "9", "x"])
def test_bad_axis_is_refused_before_any_call(arg):
    vm = r3vm.VM(program("homezc-plain"), argv=["homezc", arg])
    assert vm.run() == 1
    assert vm.stderr() == f"homezc: invalid axis '{arg}'\nUsage: homezc [newspeed]\n"
    assert vm.libs["robotlib"].calls == []


def test_library_error_goes_through_the_programs_handler():
    """homezc raises the negative result with `check pop >= 0`; its handler prints
    `<name>: <str_error(-code)>`, releases control and exits 1."""
    robot = r3vm.FakeRobot({HOMEZC: -0x04000005})
    syslib = r3vm.FakeSyslib({0x04000005: "robot not homed"})
    vm = r3vm.VM(program("homezc-offset"), argv=["homezo", "3"], libs={"robotlib": robot, "syslib": syslib})
    assert vm.run() == 1
    assert vm.stderr() == "homezo: robot not homed\n"
    assert syslib.calls == [(0x07, (0x04000005,))]
    assert [n for n, _ in robot.calls] == [HOMEZC, CTL_REL]


def test_try_state_is_restored_on_the_normal_path():
    image = program("homezc-plain")
    vm = r3vm.VM(image, argv=["homezc", "2"])
    assert vm.run() == 0 and vm.handler == 0


def test_unsupported_things_fail_loudly():
    source = """.build 0
.entry main
.lib syslib 2 0 0 5
.data
f: string "{2}\\n"
.code
main:
    enter 4
    ldi L4, 5
    push L4 tag 1
    pushi 1
    pushleag f
    pushi 4
    builtinx printf
    retv 0
"""
    with pytest.raises(r3vm.R3VMError, match="format spec"):
        r3vm.VM(r3asm.assemble(source)).run()
    uncaught = source.replace("    ldi L4, 5\n", "    ldi L4, -3\n    raise L4\n")
    with pytest.raises(r3vm.Uncaught) as e:
        r3vm.VM(r3asm.assemble(uncaught)).run()
    assert e.value.code == -3
