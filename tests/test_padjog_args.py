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
