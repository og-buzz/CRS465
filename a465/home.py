"""Homing the arm joint by joint on the index pulse, and the single-joint moves it is built from.

    scripts/a465-home --plan            # no motion: what would be done
    scripts/a465-home                   # joints 2 4 5 3 1 6, one after the other
    scripts/a465-home 6 5               # only these joints, in this order

PRECONDITION: each joint's PRESENT READING MUST ALREADY BE WITHIN LESS THAN HALF A MOTOR TURN OF THE
TRUTH (about 1.8 degrees), for example because the arm was put on its zero marks and zeroed, or because
it is already homed. This tool cannot check that. The offsets it prints afterwards tell.

Why: the index step moves the joint in the + counts direction to the NEXT encoder index pulse and then
sets the joint's counter to that joint's home point (record 1 of /conf/robot.cal). So the joint is first
moved to a parking point half a motor turn (in counts) before its home point; the distance travelled to
the pulse (`zero-cross offset: K`, in counts) should then be half a turn, and the difference from half a
turn is how far the joint's reading was from the truth.

For each joint, in order: read the counts; move to the parking point with the controller's `joint`
program (--move-speed); read the counts again and stop everything if the joint is not within 40 counts
of the parking point; run the homing program (--index-speed) and require `Ok.`, a parsed offset, no error
line and no halt; classify the offset ("centred" = within a quarter turn of half a turn); unless --stay,
move the joint back to 0.000 degrees. At the end one table is printed.

Exit code: 0 every joint ran clean and centred; 3 every joint ran but one or more offsets were near the
edge (check that joint by eye); 1 a joint failed and everything was stopped; 2 nothing was started
(no `$` prompt, the controller's /conf/robot.cal differs from --cal, the program could not be put on
the controller, a joint could not be read).

The same module holds the shared single-joint moves used by scripts/a465-joint and scripts/a465-xzero.
Every byte goes to sessions/<time>-home.log (-joint.log, -xzero.log).
"""
import contextlib
import datetime
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field

from . import aci, calfile, console, homezc

ORDER = (2, 4, 5, 3, 1, 6)
# signed counts per degree and counts per motor turn, as measured on this arm (2026-11-24)
COUNTS_PER_DEGREE = {1: 277.78, 2: 277.78, 3: -277.78, 4: -140.28, 5: -138.89, 6: -140.28}
COUNTS_PER_TURN = {1: 1000, 2: 1000, 3: 1000, 4: 500, 5: 500, 6: 500}
PARK_TOLERANCE_COUNTS = 40
WARN_DEGREES = 180.0
ARRIVED_DEGREES = 0.01      # one reading this close to the target ends the watch of a move
NEAR_DEGREES = 0.05         # at the time limit a joint this close to the target counts as arrived
DEFAULT_CAL = os.path.join("controller", "backup-2026-11-23", "conf", "robot.cal")
DEFAULT_REMOTE = "/tmp/homezo"
DEFAULT_LOCAL = os.path.join("sessions", "controller-files", "patched", "homezc-offset")
CAL_REMOTE = "/conf/robot.cal"
PRECONDITION = ("PRECONDITION: each joint's present reading must already be within less than half a motor turn "
                "(1.8 degrees) of the truth, e.g. the arm was put on its zero marks and zeroed, or it is already "
                "homed. This tool cannot check that; the offsets printed afterwards tell.")
HOMING_OK = r"Homing axis {axis}\.\.\.Ok\."
OFFSET_LINE = re.compile(r"zero-cross offset:\s*(-?\d+)")
ALLOWED_HOMING_LINES = (r"Homing axis \d\.\.\.Ok\.", r"zero-cross offset:\s*-?\d+")


class Stop(RuntimeError):
    """Everything stops here; `code` is the exit code."""

    def __init__(self, message, code=1):
        super().__init__(message)
        self.code = code


# ---------------------------------------------------------------------------------------------- sessions

@contextlib.contextmanager
def session(tag, args, repo, open_port=None, clock=None, sleep=None, log_dir=None, first_line=None):
    """Log file, port and Link, the way every tool here starts. Yields (link, say). `open_port(args)` replaces the serial port in tests."""
    log_dir = log_dir or os.path.join(repo, "sessions")
    path = os.path.join(log_dir, datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S") + f"-{tag}.log")
    with open(path, "w") as log:
        def say(text):
            print(text, flush=True)
            log.write(text + "\n")
            log.flush()
        if first_line:
            say(first_line)
        if open_port is None:
            running = subprocess.run(["pgrep", "-f", "python3 scripts/a465-padjog"], capture_output=True, text=True).stdout.split()
            for pid in running:
                os.kill(int(pid), signal.SIGTERM)
            if running:
                say("stopping the pad program...")
                time.sleep(8)
            import serial
            port = serial.Serial(args.port, 38400, timeout=0, exclusive=True)
        else:
            port = open_port(args)
        say(f"log: {path}")
        try:
            yield homezc.make_link(port, say, clock, sleep), say
        finally:
            if hasattr(port, "close"):
                port.close()


# ----------------------------------------------------------------------------------------------- moves

@dataclass
class MoveResult:
    code: int = 1                   # 0 ok; 1 failed; 2 nothing could be started (no reading)
    started: bool = False           # the `joint` command was sent
    before: list = None
    after: list = None
    moved: float = 0.0
    out: homezc.Outcome = field(default_factory=homezc.Outcome)


def move_time_limit(degrees, speed):
    """Seconds after the command: twice the time the move should take (at least 100 deg/s at 100%), plus 6 s, plus
    5 s so that at least 3 more `w2` readings (about 1.5 s each on this controller) come after the move itself."""
    expected = 2.0 * abs(degrees) / speed
    return max(6.0 + expected, expected + 5.0) + 5.0


def move_joint(link, joint, degrees, speed, say, max_move=30.0):
    """Move one joint by `degrees` (relative) with the controller's `joint <n> <degrees>` and supervise it.

    Sets the speed first (a PAUSE/CONTINUE request is waited for there, where nothing can move); a request at
    the motion step kills the program before anything moves. Only the commanded joint's reading decides when
    the move has ended (the others flicker by a few thousandths while holding). code 0 only if the joint ended
    within 0.1 degree of the commanded move and the program returned by itself."""
    res = MoveResult()
    if abs(degrees) > max_move:
        say(f"refused: {degrees} degrees is more than --max {max_move}")
        return res
    command = f"joint {joint} {degrees:.3f}"
    limit = move_time_limit(degrees, speed)
    before = res.before = console.read_joints(link)
    if before is None:
        say("could not read the joints. Nothing was started.")
        res.code = 2
        return res
    say("before: joints " + " ".join(f"{j:.3f}" for j in before))
    ok, reply = homezc.set_speed(link, speed, say)
    if not ok:
        say(f"the controller did not accept `speed {speed}`: {reply!r}. Nothing was started.")
        return res
    want = before[joint - 1] + degrees
    say(f"JOINT {joint} IS ABOUT TO MOVE {degrees:+.3f} degrees to {want:.3f}. "
        f"Speed {speed}%, time limit {limit:.0f} s, then Ctrl-Z.")
    out = res.out = homezc.Outcome()
    start = link.clock()
    res.started = True
    try:
        homezc.run_program(link, command, limit, say, out=out, on_button="abort")
    except KeyboardInterrupt:
        say("interrupted by the operator (Ctrl-Z was sent)")
    after, last = None, None
    while out.returned:
        after = console.read_joints(link)
        if after is None or (last is not None and abs(after[joint - 1] - last[joint - 1]) < 0.005):
            break
        miss = abs(after[joint - 1] - want)
        if miss <= ARRIVED_DEGREES:                     # on the target: no second reading needed
            break
        if link.clock() - start > limit:
            if miss <= NEAR_DEGREES:                    # creeping over its last hundredths: it has arrived
                break
            say("STILL MOVING at the time limit: sending Ctrl-Z")
            homezc.stop(link, out, say, halt=True)
            after = console.read_joints(link)
            break
        last = after
        link.sleep(0.3)
    if after is None:
        after = console.read_joints(link)
    res.after = after
    say("after:  joints " + (" ".join(f"{j:.3f}" for j in after) if after else "(not read)"))
    text = homezc.program_output(out.text, command)
    if text.strip():
        say("the controller said: " + text.strip())
    if out.asked_button and not out.returned:
        say("NOTHING MOVED: the controller wanted the button at the motion step. Run it again.")
    if not after:
        return res
    res.moved = after[joint - 1] - before[joint - 1]
    others = max(abs(a - b) for i, (a, b) in enumerate(zip(after, before)) if i != joint - 1)
    say(f"SUMMARY: joint {joint} moved {res.moved:+.3f} of {degrees:+.3f} degrees and reads {after[joint - 1]:.3f}"
        f" (other joints changed by at most {others:.3f}); halted by this tool: {'yes' if out.halted else 'no'}")
    res.code = 0 if (out.returned and not out.halted and abs(res.moved - degrees) < 0.1) else 1
    return res


def xzero_joint(link, joint, say):
    """`/diag/xzero <joint>`: zero that joint's counter. Nothing moves. Returns the exit code (0 only if it then reads 0.000)."""
    before = console.read_joints(link)
    say("before: joints " + (" ".join(f"{j:.3f}" for j in before) if before else "(not read)"))
    if before is None:
        return 2
    out = homezc.Outcome()
    command = f"/diag/xzero {joint}"
    homezc.run_program(link, command, 10.0, say, out=out, on_button="wait")
    say("the controller said: " + (homezc.program_output(out.text, command).strip() or "(nothing)"))
    after = console.read_joints(link)
    say("after:  joints " + (" ".join(f"{j:.3f}" for j in after) if after else "(not read)"))
    if not after:
        return 1
    others = max(abs(a - b) for i, (a, b) in enumerate(zip(after, before)) if i != joint - 1)
    say(f"SUMMARY: joint {joint} read {before[joint - 1]:.3f} and now reads {after[joint - 1]:.3f}; "
        f"other joints changed by at most {others:.3f}")
    return 0 if (out.returned and abs(after[joint - 1]) < 0.01 and others < 0.05) else 1


def joint_main(argv=None, repo=None, **session_kw):
    import argparse
    repo = repo or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(prog="a465-joint", description="Move one joint by a number of degrees with the controller's own `joint` program, supervised.")
    ap.add_argument("joint", type=int, choices=range(1, 7))
    ap.add_argument("degrees", type=float)
    ap.add_argument("--speed", type=int, default=5, choices=range(1, 26))
    ap.add_argument("--max", type=float, default=30.0, help="largest move allowed, degrees")
    ap.add_argument("--port", default=os.environ.get("A465_PORT", "/dev/ttyUSB0"))
    args = ap.parse_args(argv)
    if abs(args.degrees) > args.max:
        sys.exit(f"refused: {args.degrees} degrees is more than --max {args.max}")
    with session("joint", args, repo, **session_kw) as (link, say):
        if not aci.at_system_prompt(link):
            say("the controller is not at its `$` prompt")
            return 2
        return move_joint(link, args.joint, args.degrees, args.speed, say, args.max).code


def xzero_main(argv=None, repo=None, **session_kw):
    import argparse
    repo = repo or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(prog="a465-xzero", description="Zero one joint's position counter with the controller's own `/diag/xzero <axis>`. Nothing moves.")
    ap.add_argument("joint", type=int, choices=range(1, 7))
    ap.add_argument("--port", default=os.environ.get("A465_PORT", "/dev/ttyUSB0"))
    args = ap.parse_args(argv)
    with session("xzero", args, repo, **session_kw) as (link, say):
        if not aci.at_system_prompt(link):
            say("the controller is not at its `$` prompt")
            return 2
        return xzero_joint(link, args.joint, say)


# ---------------------------------------------------------------------------------------------- homing

def park_point(home, joint):
    return home - COUNTS_PER_TURN[joint] // 2


def classify(offset, joint):
    """(ideal, starting error in counts, centred?) for a measured offset in counts."""
    turn = COUNTS_PER_TURN[joint]
    ideal = turn // 2
    return ideal, offset - ideal, abs(offset - ideal) <= turn // 4


def error_degrees(counts, joint):
    """Counts as degrees in the joint's own sign. For an offset K this is (reading - truth) before homing, as derived from how the offset is defined."""
    return counts / COUNTS_PER_DEGREE[joint]


def check_calibration(link, cal_path, say):
    """The controller's /conf/robot.cal must equal the local file. Returns the home points (counts, eight). Raises Stop(2)."""
    with open(cal_path, "rb") as source:
        data = source.read()
    want = (sum(data) & 0xFFFFFFFF, len(data))
    there = aci.remote_cksum(link, CAL_REMOTE)
    if there != want:
        raise Stop(f"the controller's {CAL_REMOTE} (cksum {there}) is not {cal_path} ({want[1]} bytes, sum {want[0]:#x}). "
                   "Nothing was started.", 2)
    say(f"{CAL_REMOTE} on the controller equals {cal_path}: {want[1]} bytes, sum {want[0]:#x}")
    return calfile.parse(data).home_counts


def plan_lines(counts, joints, homes, joint_order, max_move):
    lines = [f"{'joint':>5} {'now (counts)':>12} {'now (deg)':>10} {'home point':>11} {'parking point':>14} {'park move (deg)':>16} {'return (deg)':>13}"]
    for j in joint_order:
        home = homes[j - 1]
        park = park_point(home, j)
        move = (park - counts[j - 1]) / COUNTS_PER_DEGREE[j]
        back = home / COUNTS_PER_DEGREE[j]
        lines.append(f"{j:>5} {counts[j - 1]:>12} {joints[j - 1]:>10.3f} {home:>11} {park:>14} {move:>+16.3f} {back:>+13.3f}")
        if abs(move) > WARN_DEGREES:
            lines.append(f"      WARNING: joint {j}'s parking move is {abs(move):.1f} degrees, more than {WARN_DEGREES:.0f}")
        if abs(back) > WARN_DEGREES:
            lines.append(f"      WARNING: joint {j}'s return to zero is {abs(back):.1f} degrees, more than {WARN_DEGREES:.0f}")
        if abs(move) > max_move or abs(back) > max_move:
            lines.append(f"      WARNING: that move is above --max-move {max_move:g} and would be refused")
    return lines


def home_one(link, joint, home, args, say):
    """Park, home, classify and (unless --stay) return one joint. Returns the table row. Raises Stop."""
    counts = console.read_pulses(link)
    if counts is None:
        raise Stop(f"joint {joint}: could not read the counts. Nothing was started.", 2)
    park = park_point(home, joint)
    degrees = (park - counts[joint - 1]) / COUNTS_PER_DEGREE[joint]
    say(f"JOINT {joint}: reads {counts[joint - 1]} counts, home point {home}, parking point {park}: move {degrees:+.3f} degrees")
    if abs(degrees) > args.max_move:
        raise Stop(f"joint {joint}: the parking move of {degrees:+.3f} degrees is more than --max-move {args.max_move:g}. Nothing was moved.", 1)
    if abs(degrees) >= 0.005:
        res = move_joint(link, joint, degrees, args.move_speed, say, args.max_move)
        if res.code != 0:
            raise Stop(f"joint {joint}: the parking move did not end cleanly (exit {res.code}). Stopping everything.", 1)
    else:
        say(f"joint {joint} is already at the parking point")
    counts = console.read_pulses(link)
    if counts is None or abs(counts[joint - 1] - park) > PARK_TOLERANCE_COUNTS:
        now = "not read" if counts is None else f"{counts[joint - 1]} counts"
        raise Stop(f"joint {joint} is at {now}, not within {PARK_TOLERANCE_COUNTS} counts of the parking point {park}. "
                   "The homing program was not run. Stopping everything.", 1)
    say(f"joint {joint} is at {counts[joint - 1]} counts, parking point {park}: within {PARK_TOLERANCE_COUNTS}")
    code, out = homezc.run_axis(link, args.remote, joint, args.index_speed, args.time_limit, say, allowed=ALLOWED_HOMING_LINES)
    if out is None or code != 0:
        raise Stop(f"joint {joint}: the homing program did not end cleanly. Stopping everything.", 1)
    found = OFFSET_LINE.search(out.text)
    if not re.search(HOMING_OK.format(axis=joint), out.text) or not found:
        raise Stop(f"joint {joint}: the program did not print `Homing axis {joint}...Ok.` and `zero-cross offset: K`. "
                   f"It printed {out.text!r}. Stopping everything.", 1)
    offset = int(found.group(1))
    ideal, error, centred = classify(offset, joint)
    err_deg = error_degrees(error, joint)
    turn_deg = COUNTS_PER_TURN[joint] / abs(COUNTS_PER_DEGREE[joint])
    if centred:
        say(f"joint {joint}: offset {offset} of {COUNTS_PER_TURN[joint]} per turn (ideal {ideal}): centred")
    else:
        say(f"joint {joint}: offset {offset} of {COUNTS_PER_TURN[joint]} per turn (ideal {ideal}): NEAR THE EDGE. "
            f"The starting reading was off by {err_deg:+.2f} degrees. Check this joint by eye: the neighbouring index "
            f"is one motor turn ({turn_deg:.1f} degrees) away.")
    if not args.stay:
        now = console.read_joints(link)
        if now is None:
            raise Stop(f"joint {joint}: could not read the joints to return it to zero. Stopping everything.", 1)
        back = -now[joint - 1]
        if abs(back) >= 0.0005:
            res = move_joint(link, joint, back, args.move_speed, say, args.max_move)
            if res.code != 0:
                raise Stop(f"joint {joint}: the return to zero did not end cleanly (exit {res.code}). Stopping everything.", 1)
    return joint, offset, ideal, err_deg, "centred" if centred else "near the edge"


def table(rows):
    lines = [f"{'joint':>5} {'offset':>7} {'ideal':>6} {'start error (deg)':>18}  verdict"]
    lines += [f"{j:>5} {k:>7} {i:>6} {e:>+18.2f}  {v}" for j, k, i, e, v in rows]
    return "\n".join(lines)


def parse_args(argv, repo):
    import argparse
    ap = argparse.ArgumentParser(prog="a465-home", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("joints", nargs="*", type=int, choices=range(1, 7), metavar="N", help="joints to home, in this order (default 2 4 5 3 1 6)")
    ap.add_argument("--plan", action="store_true", help="no motion: print what would be done")
    ap.add_argument("--stay", action="store_true", help="leave each joint where the index step put it instead of returning it to 0.000")
    ap.add_argument("--move-speed", type=int, default=10, choices=range(1, homezc.MAX_SPEED + 1), metavar="1-25", help="percent, for parking and returning (default %(default)s)")
    ap.add_argument("--index-speed", type=int, default=5, choices=range(1, homezc.MAX_SPEED + 1), metavar="1-25", help="percent, for the index step (default %(default)s)")
    ap.add_argument("--max-move", type=float, default=185.0, help="refuse any single move above this many degrees (default %(default)s)")
    # measured on 2026-11-24: after a power cycle (the "not homed" state) the index step took 7 to 10 s per joint, against about 1.5 s when the controller already counted the arm as homed
    ap.add_argument("--time-limit", type=float, default=30.0, help="seconds for the index step before Ctrl-Z (default %(default)s)")
    ap.add_argument("--cal", default=os.path.join(repo, DEFAULT_CAL), help="local copy of /conf/robot.cal (default %(default)s)")
    ap.add_argument("--remote", default=DEFAULT_REMOTE, help="the homing program on the controller (default %(default)s)")
    ap.add_argument("--local", default=os.path.join(repo, DEFAULT_LOCAL), help="the local copy it must equal (default %(default)s)")
    ap.add_argument("--port", default=os.environ.get("A465_PORT", "/dev/ttyUSB0"))
    args = ap.parse_args(argv)
    if len(set(args.joints)) != len(args.joints):
        ap.error("a joint is listed twice")
    args.joints = args.joints or list(ORDER)
    if not 0 < args.time_limit <= 120:
        ap.error("--time-limit must be above 0 and at most 120 seconds")
    for path in (args.cal,) + (() if args.plan else (args.local,)):
        if not os.path.isfile(path):
            ap.error(f"{path} does not exist")
    if re.search(r"\s", args.remote):
        ap.error("--remote must not contain spaces")
    return args


def main(argv=None, repo=None, open_port=None, clock=None, sleep=None, upload=None, log_dir=None):
    """Returns the exit code. `open_port(args)` and `upload(link, args, say)` are replaced in tests."""
    repo = repo or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    args = parse_args(argv, repo)
    with session("home", args, repo, open_port, clock, sleep, log_dir, first_line=PRECONDITION) as (link, say):
        rows = []
        try:
            if not aci.at_system_prompt(link):
                raise Stop("the controller is not at its `$` prompt", 2)
            homes = check_calibration(link, args.cal, say)
            counts, joints = console.read_pulses(link), console.read_joints(link)
            if counts is None or joints is None:
                raise Stop("could not read the counts and joints with `w2`. Nothing was started.", 2)
            say("\n".join(plan_lines(counts, joints, homes, args.joints, args.max_move)))
            if args.plan:
                say("--plan: nothing was moved.")
                return 0
            do_upload = (lambda: upload(link, args, say)) if upload else (lambda: aci.send_file(link, args.local, args.remote, report=say))
            try:
                homezc.ensure_program(link, args.remote, args.local, do_upload, say)
            except (homezc.HomeError, aci.TransferError) as err:
                raise Stop(str(err), 2)
            for joint in args.joints:
                rows.append(home_one(link, joint, homes[joint - 1], args, say))
        except Stop as stop:
            say(f"STOPPED: {stop}")
            if rows:
                say("joints finished before the stop:\n" + table(rows))
            return stop.code
        say(table(rows))
        edge = [r[0] for r in rows if r[4] != "centred"]
        if edge:
            say(f"joints near the edge: {' '.join(map(str, edge))}. Check them by eye.")
            return 3
        say("all joints ran clean and centred")
        return 0
