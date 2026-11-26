"""a465-padjog's option checks (they fail before the port is touched)."""
import os
import subprocess
import sys

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts", "a465-padjog")


def padjog(*args):
    return subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True, timeout=30)


def test_bad_caps_are_refused():
    for args in (("--jogd", "--joint-cap", "2=0"), ("--jogd", "--joint-cap", "9=5"), ("--jogd", "--frame-cap", "world=100"),
                 ("--jogd", "--frame-cap", "joint=5"), ("--joint-cap", "2=5"), ("--jogd", "--status-port", "70000")):
        done = padjog(*args)
        assert done.returncode == 2, args


def test_program_c_needs_a_cart_frame():
    for args in (("--program-c", "x"), ("--jogd", "--program-c", "x"), ("--jogd", "--frames", "joint", "--program-c", "x")):
        assert padjog(*args).returncode == 2, args


def test_no_limit_guard_needs_jogd():
    done = padjog("--no-limit-guard")
    assert done.returncode == 2 and "--no-limit-guard" in done.stderr


def test_pose_options_need_jogd_and_are_checked():
    for args in (("--poses", "x.json"), ("--goto-speed", "20"), ("--jogd", "--goto-speed", "0"), ("--jogd", "--goto-speed", "51"),
                 ("--jogd", "--goto-speed", "x")):
        assert padjog(*args).returncode == 2, args
    assert "--poses" in padjog("--poses", "x.json").stderr


def test_the_pose_options_are_in_the_help():
    done = padjog("--help")
    assert done.returncode == 0 and "--poses FILE" in done.stdout and "--goto-speed 1-50" in done.stdout
