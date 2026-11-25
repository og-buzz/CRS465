"""A supervised first run of the controller's `homezc` program (index-pulse homing of one joint).

The command line is scripts/a465-homezc; the steps live here so that tests can drive them with a
fake console. Every function takes an `aci.Link` (the serial port, a read-ahead buffer, and a clock
and sleep that tests replace) and works at the `$` system prompt.

What is assumed about the console, beyond what was measured (see docs/CONTROLLER.md):
  * the shell echoes what is typed, so a program's output starts after the first line end;
  * a program that has returned is recognised by the `$ ` prompt ending the text;
  * on success the controller prints `Homing axis N...Ok.` (and the offset-printing copy of the program
    adds `zero-cross offset: K`); any other output line, apart from the PAUSE/CONTINUE request, is
    treated as an error line. `run_axis(..., allowed=...)` names the lines that are not errors.
"""
import re
import sys

from . import aci, console

BUTTON = "PAUSE/CONTINUE"
BUTTON_WAIT_S = 180.0              # how long a person has to press the front-panel button
RESET_WAIT_S = 2.5                 # the controller answers "device is being reset" for about 2 s after Ctrl-Z
DEFAULT_SPEED = 5
DEFAULT_TIME_LIMIT_S = 8.0
CHECK_LIMIT_S = 15.0
MAX_SPEED = 25                     # as a465/crs.py: the ceiling while the arm is unhomed
AXES = range(1, 7)
CHECK_EXPECT = (r"invalid command line arguments\.", r"Usage: .*\[newspeed\]")


class HomeError(RuntimeError):
    pass


class LoggedPort:
    """Wraps a serial-like port: every byte written or read goes to `say` (log file and screen)."""

    def __init__(self, port, say, clock):
        self.port, self.say, self.clock, self.t0 = port, say, clock, clock()

    def _stamp(self):
        return f"{self.clock() - self.t0:8.2f}"

    def write(self, data):
        self.say(f"{self._stamp()} -> {aci.shown(data)}")
        return self.port.write(data)

    def read(self, count=1):
        data = self.port.read(count)
        if data:
            self.say(f"{self._stamp()} <- {aci.shown(data)}")
        return data

    def flush(self):
        if hasattr(self.port, "flush"):
            self.port.flush()


def make_link(port, say=print, clock=None, sleep=None):
    import time
    clock, sleep = clock or time.monotonic, sleep or time.sleep
    return aci.Link(LoggedPort(port, say, clock), say, clock, sleep)


def ensure_program(link, remote, local, upload, say=print):
    """Make the program on the controller equal the local file by size and byte sum, uploading it if needed.
    `upload()` puts the local file on the controller. Raises HomeError if the controller still differs."""
    with open(local, "rb") as source:
        data = source.read()
    want = (sum(data) & 0xFFFFFFFF, len(data))
    there = aci.remote_cksum(link, remote)
    if there == want:
        say(f"{remote} is on the controller: {want[1]} bytes, sum {want[0]:#x}, the same as {local}")
        return False
    say(f"{remote} is " + ("missing" if there is None else f"different ({there[1]} bytes, sum {there[0]:#x})")
        + f"; {local} is {want[1]} bytes, sum {want[0]:#x}. Uploading it.")
    upload()
    there = aci.remote_cksum(link, remote)
    if there != want:
        raise HomeError(f"after the upload the controller's cksum of {remote} is {there}, not {want}: stopping")
    say(f"{remote} now matches: {want[1]} bytes, sum {want[0]:#x}")
    return True


def recover(link, say=print):
    """After Ctrl-Z: wait out the reset; if the `$` prompt is not back, Ctrl-E. True if the prompt shows."""
    seen = link.drain(quiet=RESET_WAIT_S, limit=RESET_WAIT_S + 2.0)
    if seen.rstrip(b" ").endswith(b"$"):
        return True
    link.port.write(b"\r")
    if link.drain(quiet=1.0, limit=3.0).rstrip(b" ").endswith(b"$"):
        return True
    say("no prompt after the stop: sending Ctrl-E (kill the foreground program)")
    link.port.write(console.CTRL_E)
    link.drain(quiet=1.0, limit=3.0)
    return console.at_prompt(link)


class Outcome:
    def __init__(self):
        self.text = ""              # the program's own output, verbatim (echo and prompt removed)
        self.returned = False       # back at the `$` prompt by itself
        self.halted = False         # Ctrl-Z was sent (robot stop)
        self.killed = False         # Ctrl-E was sent instead (program killed while nothing was moving)
        self.asked_button = False   # the controller asked for PAUSE/CONTINUE during this program
        self.reason = ""
        self.elapsed = 0.0          # from sending the command to the prompt (or the stop)
        self.waited = 0.0           # of which: waiting for the PAUSE/CONTINUE button
        self.prompt_back = True

    def error_lines(self, allowed=()):
        """Output lines that are neither the button request nor matched by one of the `allowed` regular expressions."""
        return [ln.strip() for ln in self.text.split("\n")
                if ln.strip() and BUTTON not in ln and not any(re.fullmatch(a, ln.strip()) for a in allowed)]


def program_output(text, command):
    lines = text.replace("\r", "").split("\n")
    if lines and lines[0].strip().endswith(command.strip()):
        lines = lines[1:]
    body = "\n".join(lines)
    return re.sub(r"\$ *$", "", body).strip("\n")


def run_program(link, command, limit, say=print, button_wait=BUTTON_WAIT_S, out=None, on_button="wait"):
    """Type `command` at the `$` prompt and watch it until the prompt returns.

    If the prompt is not back `limit` seconds after the command was sent, the robot is halted (Ctrl-Z).
    The controller may instead ask for the PAUSE/CONTINUE button. It prints nothing when the button is
    then pressed (measured 2026-11-23), so the moment the program really starts cannot be seen from here:

      on_button="wait"   for a program that cannot move the arm: tell the operator and allow
                         `button_wait` seconds from the request for the program to finish;
      on_button="abort"  for a program that will move the arm: kill it at once (Ctrl-E) while it is
                         still waiting for the button, so that a move never runs without its time limit.

    On any exception (Ctrl-C from the operator included) it sends Ctrl-Z, fills in `out` and passes the
    exception on."""
    out = out or Outcome()
    link.drain(quiet=0.1, limit=1.0)
    sent = link.clock()
    link.port.write(command.encode() + b"\r")
    button_at = None
    try:
        while True:
            link._more()
            text = link.ahead.decode("latin-1")
            now = link.clock()
            if text.rstrip(" ").endswith("$") and "\n" in text:
                out.returned = True
                break
            if BUTTON in text:
                if button_at is None:
                    button_at, out.asked_button = now, True
                    if on_button == "abort":
                        out.reason = "the controller asked for the PAUSE/CONTINUE button at the motion step"
                        break
                    say("WAITING: PRESS THE PAUSE/CONTINUE BUTTON ON THE CONTROLLER'S FRONT PANEL NOW "
                        f"(up to {button_wait:.0f} s). Nothing moves at this step.")
                if now - button_at > button_wait:
                    out.reason = f"the PAUSE/CONTINUE button was not pressed within {button_wait:.0f} s"
                    break
            elif now - sent > limit:
                out.reason = f"no return to the prompt within {limit:.0f} s"
                break
        out.elapsed = link.clock() - sent
        if button_at is not None:
            out.waited = link.clock() - button_at
        if not out.returned:
            stop(link, out, say, halt=not out.asked_button)
    except BaseException as err:
        out.elapsed = link.clock() - sent
        if not out.halted:
            out.reason = "interrupted by the operator" if isinstance(err, KeyboardInterrupt) else f"{type(err).__name__}: {err}"
            stop(link, out, say)
        raise
    finally:
        out.text = program_output(link.ahead.decode("latin-1"), command)
        link.ahead = b""
    return out


def stop(link, out, say, halt=True):
    """halt=True: Ctrl-Z once (robot stop), wait out the reset, Ctrl-E if the prompt is not back.
    halt=False: the program is only waiting for the button and nothing is moving, so Ctrl-E alone."""
    if halt:
        link.port.write(console.CTRL_Z)
        out.halted = True
        say(f"HALTED: Ctrl-Z sent ({out.reason}). Waiting for the controller to finish resetting.")
        out.prompt_back = recover(link, say)
    else:
        link.port.write(console.CTRL_E)
        out.killed = True
        say(f"KILLED before anything moved: Ctrl-E sent ({out.reason}).")
        link.drain(quiet=1.0, limit=3.0)
        out.prompt_back = console.at_prompt(link)
    say("the controller is back at its `$` prompt" if out.prompt_back else "THE CONTROLLER IS NOT AT ITS `$` PROMPT")


def snapshot(link, say, label):
    joints, words = console.read_joints(link), console.read_status(link)
    say(f"{label}: joints " + (" ".join(f"{j:.3f}" for j in joints) if joints else "(not read)"))
    say(f"{label}: status " + (" ".join(f"{w:08x}" for w in words) if words else "(not read)"))
    return joints, words


SPEED_NOW = re.compile(r"Current Speed:\s*(\d+)\s*%")


def set_speed(link, percent, say=print):
    """`speed <percent>`, then `speed` alone to read it back. Returns (accepted, what the controller said).

    Setting the speed takes the robot's point of control, so this is where the controller asks for the
    PAUSE/CONTINUE button if another program left control held (measured 2026-11-24). Nothing can move
    at this step, which is why the wait for the button is done here and never at the motion step."""
    out = run_program(link, f"speed {percent}", 10.0, say, on_button="wait")
    said = "\n".join(out.error_lines())
    if not out.returned or (said and console.PROBLEM.search(said)):
        return False, said or out.reason
    link.drain(quiet=0.1, limit=1.0)
    answer = link.shell("speed", lambda g: g.endswith(b"$ ") and b"\n" in g, 10.0).decode("latin-1")
    found = SPEED_NOW.search(answer)
    if not found or int(found.group(1)) != percent:
        return False, "read back: " + (program_output(answer, "speed") or "(nothing)")
    return True, f"Current Speed: {percent}% of max."


def check(link, remote, say=print):
    """No axis argument is ever sent: the program only has to load and complain. True if it did."""
    out = run_program(link, remote, CHECK_LIMIT_S, say)
    if out.halted or out.killed or not out.prompt_back:
        say(f"CHECK FAILED: {out.reason}. Output: {out.text!r}")
        return False
    if all(re.search(p, out.text) for p in CHECK_EXPECT):
        say("the controller loads and runs the program. It printed:\n" + out.text)
        return True
    say("the program printed something other than the two expected lines. Exactly:\n" + (out.text or "(nothing)"))
    return False


def words_of(before, after, axis, number, names):
    i = axis - 1
    if not before or not after:
        return "not read"
    f = lambda w: names[console.bit(w[i], number)]
    return f"{f(before)} -> {f(after)}"


def faults(word):
    # Bit 10 is left out: it is set in every status word this controller has shown, moving or still.
    names = [n for b, n in ((8, "error"), (9, "servo fault")) if console.bit(word, b)]
    return ", ".join(names) or "none"


def summary(axis, speed, limit, b_joints, b_words, a_joints, a_words, out):
    i = axis - 1
    lines = ["", f"SUMMARY: joint {axis}"]
    if b_joints and a_joints:
        lines.append(f"  moved {a_joints[i] - b_joints[i]:+.3f} degrees ({b_joints[i]:.3f} -> {a_joints[i]:.3f})")
    else:
        lines.append("  moved: unknown (the joint angles could not be read both times)")
    yn = {False: "no", True: "yes"}
    for number, label in ((5, "armed for zero-cross (bit 5)"), (6, "zero-cross seen (bit 6)"), (11, "homed (bit 11)")):
        lines.append(f"  {label}: {words_of(b_words, a_words, axis, number, yn)}")
    if b_words and a_words:
        lines.append(f"  status word: {b_words[i]:04x} -> {a_words[i]:04x}")
        lines.append(f"  error / servo fault (bits 8, 9): {faults(b_words[i])} -> {faults(a_words[i])}")
    else:
        lines.append("  status word, error / servo fault: not read")
    lines.append("  the program's own output:")
    lines.extend("    " + ln for ln in (out.text.split("\n") if out.text.strip() else ["(nothing)"]))
    lines.append(f"  ran {out.elapsed:.1f} s (time limit {limit:.0f} s, speed {speed}%)")
    if out.halted:
        lines.append("  HALTED by this tool: YES (" + out.reason + ")")
    elif out.killed:
        lines.append("  KILLED before it could move: " + out.reason)
    else:
        lines.append("  halted by this tool: no (the program returned by itself)")
    return "\n".join(lines)


def run_axis(link, remote, axis, speed, limit, say=print, allowed=()):
    """The supervised run. Returns (exit code, Outcome or None). Output lines matching a regular expression in `allowed`
    are not errors (the offset-printing program prints a success line and the offset)."""
    b_joints, b_words = snapshot(link, say, "before")
    ok, reply = set_speed(link, speed, say)
    if not ok:
        say(f"the controller did not accept `speed {speed}`: {reply!r}. Nothing was started.")
        return 1, None
    say(f"speed set to {speed}% (read back)")
    say(f"JOINT {axis} IS ABOUT TO MOVE. Time limit {limit:.0f} s, then Ctrl-Z. Keep a hand on the E-stop.")
    out, problem = Outcome(), None
    try:
        run_program(link, f"{remote} {axis}", limit, say, out=out, on_button="abort")
    except KeyboardInterrupt:
        problem = "interrupted by the operator (Ctrl-Z was sent)"
    except Exception as err:            # noqa: BLE001 -- the after-readings must still happen
        problem = f"{type(err).__name__}: {err}"
    finally:
        try:
            a_joints, a_words = snapshot(link, say, "after")
        except KeyboardInterrupt:
            a_joints = a_words = None
        except Exception as err:        # noqa: BLE001
            say(f"the after-readings failed: {err}")
            a_joints = a_words = None
    say(summary(axis, speed, limit, b_joints, b_words, a_joints, a_words, out))
    if problem:
        say("PROBLEM: " + problem)
    if out.asked_button and not out.returned:
        say("NOTHING MOVED: the controller wanted the button again at the motion step, where this tool will not "
            "wait for it. Run the tool again; the button is asked for at the speed step.")
    code = 0 if (out.returned and not out.halted and not out.killed and not out.error_lines(allowed)) else 1
    return code, out


def parse_args(argv, repo):
    import argparse
    import os
    ap = argparse.ArgumentParser(prog="a465-homezc", description="A supervised first run of the controller's homezc program.")
    ap.add_argument("axis", nargs="?", help="joint to index-home, 1 to 6")
    ap.add_argument("--check", action="store_true", help="no motion: make sure the program is on the controller and that the controller loads it")
    ap.add_argument("--remote", default="/tmp/homezc", help="the program on the controller (default %(default)s)")
    ap.add_argument("--local", default=os.path.join(repo, "sessions", "controller-files", "patched", "homezc"),
                    help="the local copy it must equal (default %(default)s)")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED, help=f"percent, 1 to {MAX_SPEED} (default %(default)s)")
    ap.add_argument("--time-limit", type=float, default=DEFAULT_TIME_LIMIT_S, help="seconds before Ctrl-Z (default %(default)s)")
    ap.add_argument("--port", default=os.environ.get("A465_PORT", "/dev/ttyUSB0"))
    ap.add_argument("--baud", type=int, default=38400)
    args = ap.parse_args(argv)
    if args.check == (args.axis is not None):
        ap.error("give either --check or an axis (1 to 6)")
    if args.axis is not None:
        if not re.fullmatch(r"[1-6]", args.axis):
            ap.error(f"axis must be 1 to 6, not {args.axis!r} (7 and 8 are refused)")
        args.axis = int(args.axis)
        if not 1 <= args.speed <= MAX_SPEED:
            ap.error(f"--speed must be 1 to {MAX_SPEED} while the arm is unhomed")
        if not 0 < args.time_limit <= 120:
            ap.error("--time-limit must be above 0 and at most 120 seconds")
    if not os.path.isfile(args.local):
        ap.error(f"{args.local} does not exist")
    if re.search(r"\s", args.remote):
        ap.error("--remote must not contain spaces")
    return args


def main(argv=None, repo=None, open_port=None, clock=None, sleep=None, upload=None, log_dir=None):
    """Returns the exit code. `open_port(args)` and `upload(link, args, say)` are replaced in tests."""
    import datetime
    import os
    import signal
    import subprocess
    import time
    repo = repo or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    args = parse_args(argv, repo)                       # refuses a bad axis before the port is touched
    log_dir = log_dir or os.path.join(repo, "sessions")
    path = os.path.join(log_dir, datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S") + "-homezc.log")
    with open(path, "w") as log:
        def say(text):
            print(text, flush=True)
            log.write(text + "\n")
            log.flush()
        if open_port is None:
            running = subprocess.run(["pgrep", "-f", "python3 scripts/a465-padjog"], capture_output=True, text=True).stdout.split()
            for pid in running:
                os.kill(int(pid), signal.SIGTERM)
            if running:
                say("stopping the pad program...")
                time.sleep(8)
            import serial
            port = serial.Serial(args.port, args.baud, timeout=0, exclusive=True)
        else:
            port = open_port(args)
        say(f"log: {path}")
        try:
            link = make_link(port, say, clock, sleep)
            if not aci.at_system_prompt(link):
                say("the controller is not at its `$` prompt")
                return 2
            do_upload = (lambda: upload(link, args, say)) if upload else (
                lambda: aci.send_file(link, args.local, args.remote, report=say))
            try:
                ensure_program(link, args.remote, args.local, do_upload, say)
            except (HomeError, aci.TransferError) as err:
                say(f"STOPPED: {err}")
                return 2
            if args.check:
                return 0 if check(link, args.remote, say) else 1
            code, _ = run_axis(link, args.remote, args.axis, args.speed, args.time_limit, say)
            return code
        finally:
            if hasattr(port, "close"):
                port.close()
