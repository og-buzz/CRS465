"""a465/jlimprog.py: the read-only jointlim program, run in a465/r3vm.py against a fake jointlim_get."""
from a465 import jlimprog, r3vm

IMAGE = jlimprog.build()
LIMITS = {1: (175.0, -175.0), 2: (90.0, -90.0), 3: (110.5, -114.25), 4: (180.0, -180.0), 5: (105.0, -105.0),
          6: (180.0, -180.0)}


def run(results):
    robot = r3vm.FakeRobot({0x6E: results})
    vm = r3vm.VM(IMAGE, argv=["/tmp/jointlim"], libs={"robotlib": robot,
                                                       "syslib": r3vm.FakeSyslib({22: "invalid argument"})})
    code = vm.run()
    return code, vm.stdout().splitlines(), robot.calls


def test_the_committed_build_is_what_jlimprog_builds():
    assert (jlimprog.OUT / "jointlim").read_bytes() == IMAGE


def test_prints_each_axis_limits_and_takes_no_control():
    def jointlim_get(vm, args):
        axis = r3vm.s32(args[0])
        vm.store(args[1], 0, r3vm.from_float(LIMITS[axis][0]))
        vm.store(args[2], 0, r3vm.from_float(LIMITS[axis][1]))
        return 0
    code, lines, calls = run(jointlim_get)
    assert code == 0
    assert lines == [f"J {k} {LIMITS[k][0]:g} {LIMITS[k][1]:g}" for k in range(1, 7)] + ["done"]
    assert [n for n, _ in calls] == [0x6E] * 6, "jointlim_get only: no ctl_get, nothing that moves"
    assert [a[0] for _, a in calls] == [1, 2, 3, 4, 5, 6]


def test_an_error_is_printed_per_axis_and_the_rest_still_read():
    def jointlim_get(vm, args):
        if r3vm.s32(args[0]) == 4:
            return -22
        vm.store(args[1], 0, r3vm.from_float(1.0))
        vm.store(args[2], 0, r3vm.from_float(-1.0))
        return 0
    code, lines, _ = run(jointlim_get)
    assert code == 0
    assert lines[3] == "E 4 -22 invalid argument"
    assert len(lines) == 7 and lines[-1] == "done"
