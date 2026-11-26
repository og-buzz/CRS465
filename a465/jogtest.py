"""Bench test of `jogd` (a465/jogprog.py) on the real controller: its first runs.

    scripts/a465-jogtest --dry [--variant s|r] [--max 5]
    scripts/a465-jogtest <A..F|a..f> [--speed 2] [--hold 1.0] [--speed2 N] [--variant s|r] [--max 5] [--no-deadman-test]
    scripts/a465-jogtest <A..F|a..f> --maxvel PCT [--speed 5] [--hold 1.5]     # jogd-v: maxvels mid-jog

    scripts/a465-jogtest --mode 3 A --speed 2 --hold 1.0         # world-frame letter: what does A do?
    scripts/a465-jogtest --mode 3 --sweep [--letters ABC]        # A..F each + then -, one table (--mode 4: tool frame)
    scripts/a465-jogtest --mode 3 --vector +1+1+0 [--return]     # jogd-c: a straight move along (x, y, z); --return comes back

Stand at the E-stop. --dry starts jogd, waits for `jogd ready`, sends `q` and sends NO motion command.
A letter runs, in order: test 1 (start, keep-alives every 100 ms, `.`), test 2 (the opposite direction,
then silence: the dead-man must print `T`), test 3 (only with --speed2: a speed change in the same
direction). Then `q`. The protocol and the output lines are in the docstring of a465/jogprog.py.

What is assumed about the console (nothing beyond what was measured):
  * jogd's lines are found by pattern at the END of a line, so it does not matter whether the bytes
    sent to jogd are echoed (an echo with no line end in front of a jogd line is skipped over);
  * the `$` prompt is a `$` (and spaces) at the end of the text after the last line end, looked for only
    in what arrived since `q` (or since the command that started jogd) was written;
  * the PAUSE/CONTINUE request is the text homezc.BUTTON; the controller prints nothing when it is pressed.
What is NOT assumed: how long any library call takes, whether Ctrl-Z ends jogd, what the prompt looks like
after jogd exits. The tool reports what it saw.

Choices the specification did not fix (also in the final report of whoever wrote this):
  * the answer to a start command is the first `+` or `E` line after it; this applies to every phase that
    sends a new command (also the speed change). If it is missing at the end of --hold the keep-alives go on
    until 1.5 s after the first command of that phase;
  * after `.` the `-` is waited for 1.0 s. If it does not come the move may still be in force: Ctrl-Z first,
    then `.`, `q`, as for Ctrl-C;
  * if the button request is pending when this tool gives up or is interrupted, only Ctrl-E is sent (nothing
    can be moving; same as homezc), not Ctrl-Z. If jogd never printed `jogd ready` for any other reason, `q` and the usual
    steps follow;
  * `.` and `q` are written only while jogd is believed to be running (no `$` prompt at the end of the
    text), so that they never reach the shell as typing;
  * --sample writes `p` every 200 ms from `jogd ready` to `q` (and 1.0 s after the last stop), never within 4 ms
    before a keep-alive is due; the first `p` must be answered (`P c1..c6`) within 1 s or sampling is dropped.
    Sample times are ARRIVAL times on this tool's clock (jogd answers about 95 ms after the `p`). Speeds are
    differences of consecutive samples; a phase runs from its first command to the next command or the `.`
    (the `T` in test 2). --max-travel: when a sample shows the commanded joint more than that many degrees from
    its first sampled position, `.` is written at once, the sequence ends, then `q`. Not active without --sample;
  * --reverse replaces tests 1 to 3 by four moves (letter, opposite, letter, letter), each with `.` and a
    1.0 s pause; speeds are reported, never judged. A move "crossed" if its window held positions more than
    0.05 degrees on both sides of the sequence's starting position;
  * a dry run counts as failed if any encoder count differs afterwards (exit code 1);
  * --maxvel PCT runs jogd-v (controller/programs/jogd-v -> /tmp/jogd-v unless --local/--remote are given)
    instead of tests 1 to 3: one jog with keep-alives for --hold, then `vPCT` once (the moving joint's maxvels
    entry to PCT percent of the value jogd-v read at start; jogd-v puts it back at the stop), keep-alives for
    --hold more, `.`. It implies --sample. The report gives the mean speed in the 0.5 s before the `v` and from
    0.4 s after it to the `.`, and their ratio; it judges only that `+`, `V`, `-` came in that order.

--mode 3|4 (jogd's first argument: 3 world, 4 tool; default 1 joint): what the letters do there has never been run on this arm.
Only the start / hold / stop move runs (--speed default 2, --hold default 1.0; no dead-man test, speed change, --reverse or
--maxvel: refused). Sampling is always on and every sample keeps all six joints. On EVERY sample: --max-tcp MM (default 40,
the model's tool point from where it was at the first sample) and --max-joint DEG (default 10, any joint from the first
sample); past either, `.` is written at once and the run ends (then `q`). A move is reported from the PC's model (a465/model.py,
frame not checked against the controller's): joint changes, tool point dx dy dz and length (model world frame; in mode 4 also
rot.T @ d), change of yaw/pitch/roll, mean tool speed and rotation rate between the first and last moving sample, and a verdict
(1 degree weighs like ROT_MM_PER_DEG mm; one kind is "mostly" when it is DOMINATES times the other). --sweep: per letter the
positive move, 1 s, the negative move, 1 s; it ends at the first move whose lines are not `+X02`, `-` or that trips a limit. The
move's baseline is the last sample before its first command and its end the last sample before the next move starts (after the
pause); the limits count from the FIRST sample of the whole run. If the first `p` is not answered nothing is moved; if no `P`
comes for 1 s during the run, `.` is written and the run ends.

jogd-c (--program controller/programs/jogd-c --remote /tmp/jogd-c): in modes 3 and 4 a letter (or an `@` vector) starts ONE
bounded straight move (150 mm, 45 degrees for D E F) and `.` halts it. --hold may be up to 5.0 s in these modes. For every such move the
report prints, first: "start acknowledged after N ms" (over 600 ms: WARNING, the move call may be blocking, i.e. not returning until
the move is done, so the program could not read `.`; if no `+` came 1.5 s after the command `.` is written at once and the usual
no-answer handling follows); "stop: `-` came N ms after `.`; the tool point travelled X mm (Y degrees) after `.` was written" and the tool
speed just before the `.` (sample times are arrival minus 95 ms, the stated delay of a `P` line); and, if a `-` arrives BY ITSELF before this
tool wrote `.` (not a failure: the bounded length was reached, or the controller ended it), "the move ended by itself N ms after the start,
after X mm": no more keep-alives, no `.` for it, and a sweep carries on. A missing `-` after `.` is still a failure.
`A` lines (jogd-c built at 6120 bytes and later; the other programs never print one): when the controller aborts a move (a "Path error" at a
joint limit, a collision, a Ctrl-Z at the console) jogd-c survives, prints ONE line `A <code> <text>` (or `A abort`; it can take 1.1 s to appear in the
controller's 1000 ms lock-out), ends the move and reads on. In EVERY mode this tool treats that line as the end of the move in force: no more
keep-alives, no `.` for it, and the report says "ABORTED BY THE CONTROLLER: A <code> <text>, N ms after the start, after X mm" (X degrees for D E F;
for mode 1 the commanded joint's degrees, when sampled). An `A` line is never a failure by itself: the exit code stays 0 when the run's own
expectations were met, which for a move that was aborted are "`+` came, then the `A` line" (a `-` before or after it is also accepted), and a stop
that was under way (`.` written) is complete at the `A` line. An `A` line that comes when no move is in force (during a pause) is listed ("an `A`
line came between moves") and does not fail the run. Without --after-abort the run ends at the abort: sampling goes on for 1 s, then `q`
(the report says whether the prompt came back). Arm power may be OFF after a collision: the controller's own line `S: Arm power is OFF` shows
under "other lines seen"; switch it on at the front panel (this tool cannot).
  --after-abort LETTER (modes 3/4 and mode 1, with a letter or --vector, not --sweep, --dry, --reverse, --maxvel or --speed2): after the `A` line wait
1.5 s, then run that letter at the same speed for the same hold (one `.` first when it is the very letter that was aborted, because jogd-c ignores
repeats of the command that was in force until a `.` or a different command; with another letter nothing is sent first). It is a normal move
(start, keep-alives, `.`, `-`) and the summary prints "AFTER THE ABORT" with whether it was acknowledged, the stop figures, "MOVED AFTER THE ABORT:
yes/no" (the PC model moved the tool point 1 mm or rotated it 0.2 degree; mode 1: the commanded joint 0.05 degree) and whether any
"PAUSE/CONTINUE" text was seen after the abort (the arm needed the front-panel button again). Movement is reported, not judged: the exit code needs
the abort to have come, the after-abort move to be acknowledged (`+`) and stopped by `-` (or aborted again), the usual limits not tripped. If no `A`
line comes, --after-abort is not run, that is said, and the exit code is 1. In mode 1 the letter must be the same joint as the first letter (the
travel limit follows one joint) and sampling is switched on.
  --toward-limit (modes 3/4, one letter; not with --vector, --sweep, --dry): no fixed --hold. The move is kept going (keep-alives) until the FIRST of:
an `A` line, a `-` by itself, an `E` line (stopped, run fails as always), --max-tcp / --max-joint tripped (the usual LIMIT stop; run fails as always;
both may be raised to 200 mm and 45 degrees, which are the largest values this tool takes in any run), or --time-limit SECONDS (default 20, at most 60;
`.` follows as usual). A warning is printed first: it is meant to end in a controller abort. The line "TOWARD-LIMIT ENDED BY: ..." says which condition
ended it. The exit code is 0 for an abort, a `-` by itself or the time limit (the run's expectations are met), 1 for E or a tool limit. --hold then
means only the hold of the --after-abort move (default 2.0 s).

--vector XYZ (three signed digits, e.g. +1+1+0; modes 3/4 only; not with a letter, --sweep, --dry or mode 1) sends `@`, XYZ, `+0+0+0`
and the two speed digits (15 bytes, also the 100 ms keep-alive), expects the line `+@+1+1+0+0+0+0NN` and then `-` after `.`, and reports the
angle between the measured tool-point displacement and the direction (world frame in mode 3; the tool's own frame at the start in mode
4) and the off-line distance. --return then runs the opposite vector (zero components stay +0) for the same hold, 1 s later, and reports
how far from the first sample the arm ended.
"""
import argparse
import math
import os
import re

import numpy as np

from . import aci, console, home, homezc, model

READY_WAIT_S = 10.0
BUTTON_WAIT_S = homezc.BUTTON_WAIT_S
ANSWER_WAIT_S = 1.5              # a start command must be answered with `+` or `E` within this
STOP_WAIT_S = 1.0                # `-` after `.`
DEADMAN_WAIT_S = 2.0
QUIT_WAIT_S = 5.0
PAUSE_S = 1.0
KEEPALIVE_S = 0.100
LATE_S = 0.150                   # the dead-man is 300 ms; a keep-alive gap above this is reported
POLL_S = 0.002
MAX_RANGE = (1, 10)
HOLD_RANGE = (0.2, 3.0)
SAMPLE_S = 0.200                 # --sample: `p` every 200 ms (never delaying a keep-alive)
SAMPLE_GUARD_S = 0.004           # a `p` is not written within this of a keep-alive that is due
P_WAIT_S = 1.0                   # the first `p` must be answered within this, or sampling is dropped
TRAIL_S = 1.0                    # sampling goes on this long after the last stop
MAX_TRAVEL = (0.0, 90.0)
MAX_TCP_RANGE = (0.0, 200.0)     # --max-tcp, mm (modes 3 and 4)
MAX_JOINT_RANGE = (0.0, 45.0)    # --max-joint, degrees (modes 3 and 4)
DEFAULT_MAX_TCP, DEFAULT_MAX_JOINT = 40.0, 10.0
ACK_WARN_S = 0.600               # modes 3/4: a start answered later than this: the start call may be blocking
SAMPLE_LAG_S = 0.095             # a `P` line arrives about this long after the position it holds was read
SIX_HOLD_RANGE = (0.2, 5.0)      # --hold in modes 3/4
AFTER_ABORT_WAIT_S = 1.5         # --after-abort: from the `A` line to the next move's first command
AFTER_ABORT_HOLD_S = 2.0         # --toward-limit: the hold of the --after-abort move (--hold)
TIME_LIMIT_DEFAULT_S, TIME_LIMIT_MAX_S = 20.0, 60.0
MOVED_MM, MOVED_DEG, MOVED_JOINT_DEG = 1.0, 0.2, 0.05    # what counts as "the arm moved"
VECTOR_RE = re.compile(r"(?:[+-]\d){3}")
STALE_S = 1.0                    # modes 3/4: no `P` answer for this long while sampling is on: `.` and stop
ROT_MM_PER_DEG = 10.0            # the verdict weighs 1 degree of rotation like this many mm of translation
DOMINATES = 3.0                  # ... and calls a move "mostly" one kind when it is this many times the other
MODEL_NOTE = ("These axes are from the PC's model (a465/model.py), whose frame has not been checked against "
              "the controller's.")
CTRL_Z, CTRL_E = console.CTRL_Z, console.CTRL_E
COUNTS_PER_DEGREE = home.COUNTS_PER_DEGREE

LINE_PATTERNS = (                # searched at the end of a line, in this order
    ("E", re.compile(r"E (-?\d+) (.*)$")),
    ("A", re.compile(r"(?<![A-Za-z0-9+@])A (?:-?\d+ .*|abort)$")),       # jogd-c: the controller aborted the move
    ("MV", re.compile(r"MV(?: \S+){8}$")),
    ("V", re.compile(r"(?<![A-Za-z])V (\d+) (\S+)$")),
    ("P", re.compile(r"P(?: -?\d+){6}$")),
    ("ready", re.compile(r"jogd ready$")),
    ("+", re.compile(r"\+([A-Fa-f]\d\d)$")),
    ("+", re.compile(r"\+@(?:[+-]\d){6}\d\d$")),      # jogd-c: an `@` vector echoed as used
    ("T", re.compile(r"(?<![A-Za-z])T$")),
    ("-", re.compile(r"-$")),
)


class Abort(Exception):
    """The test sequence stops; `.` and `q` follow."""


class EError(Abort):
    pass


class NoAnswer(Abort):
    pass


class TravelLimit(Abort):
    """The commanded joint moved more than --max-travel from where the sequence started."""


class LimitTrip(Abort):
    """Modes 3/4: a sample is beyond --max-tcp or --max-joint."""


class NotStopped(Abort):
    """`.` was sent and no `-` came: the move may still be in force."""


def classify(text):
    text = text.strip()
    for kind, pattern in LINE_PATTERNS:
        found = pattern.search(text)
        if found:
            return kind, found.group(0)
    return "other", text


class Reader:
    """Everything the port gives, from the moment jogd was started: raw bytes and parsed jogd lines."""

    def __init__(self, link, say):
        self.link, self.say = link, say
        self.reset()

    def reset(self):
        self.raw, self.partial, self.lines, self.t0 = b"", b"", [], self.link.clock()
        self.a_raw = {}                # index in lines of an `A` line -> where in raw its line ended

    def pump(self):
        chunk = self.link.port.read(4096)
        if not chunk:
            return False
        t = self.link.clock()
        self.link.ahead += chunk
        self.raw += chunk
        self.partial += chunk
        while b"\n" in self.partial:
            raw, self.partial = self.partial.split(b"\n", 1)
            kind, text = classify(raw.decode("latin-1").replace("\r", ""))
            if kind == "other" and not text:
                continue
            self.lines.append((t, kind, text))
            if kind == "A":
                self.a_raw[len(self.lines) - 1] = len(self.raw) - len(self.partial)
            if kind != "P":               # samples are in the log and in the summary tables
                self.say(f"  [{(t - self.t0) * 1000:9.1f} ms] jogd line ({kind}): {text}")
        return True

    def of_kind(self, kind, since=0):
        return [ln for ln in self.lines[since:] if ln[1] == kind]

    def button_text(self):
        return homezc.BUTTON.encode() in self.raw

    def prompt_since(self, mark=0):
        text = self.raw[mark:]
        tail = text.rsplit(b"\n", 1)[-1]
        return tail.rstrip(b" ").endswith(b"$")


def rot_axis_angle(rot):
    """(angle in degrees, unit axis) of a rotation matrix; axis is zeros when there is no rotation."""
    ang = math.acos(max(-1.0, min(1.0, (np.trace(rot) - 1.0) / 2.0)))
    v = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    n = float(np.linalg.norm(v))
    return math.degrees(ang), (v / n if n > 1e-9 else np.zeros(3))


def read_position(link, tries=4):
    """(joint angles, encoder counts) from one `w2`; (None, None) if never answered."""
    for _ in range(tries):
        link.drain(quiet=0.1, limit=1.0)
        answer = link.shell("w2", lambda g: g.endswith(b"$ ") and b"\n" in g, 10.0).decode("latin-1")
        joints, pulses = console.parse_joints(answer), console.parse_pulses(answer)
        if joints is not None:
            return joints, pulses
        link.sleep(1.0)
    return None, None


def ms(seconds):
    return f"{seconds * 1000:.1f} ms"


def vector13(xyz):
    """`@` + the three components + `+0+0+0`: the 13 bytes before the two speed digits of a vector command."""
    return "@" + xyz + "+0+0+0"


def opposite_vector(xyz):
    """The same components with their signs turned (a zero stays `+0`)."""
    return "".join("+0" if d == "0" else ("-" if sg == "+" else "+") + d for sg, d in re.findall(r"([+-])(\d)", xyz))


class Test:
    def __init__(self, name):
        self.name = name
        self.t0 = None
        self.sent = []               # (time, label)
        self.lines = []              # (time, kind, text)
        self.expected = []
        self.notes = []
        self.ok = False
        self.late = []               # keep-alive gaps above LATE_S
        self.gaps = []
        self.phases = []             # (time of the first write of a start command, its text)
        self.stop_t = None           # when `.` was written (or `T` arrived)
        self.t_end = None            # end of the window sampled for this test (the next test's start)
        self.move_report = False
        self.label = None            # modes 3/4: the letter, or `@+1+1+0` for a vector
        self.self_end_t = None       # modes 3/4: a `-` arrived by itself, before this tool wrote `.`
        self.abort = None            # an `A` line ended the move: {"t", "text", "raw"} (raw: offset in the console bytes after it)
        self.end_reason = None       # --toward-limit: what ended the move

    @property
    def got(self):
        return [t for _, k, t in self.lines if k not in ("other", "P", "A")]


class JogTest:
    def __init__(self, link, say, args):
        self.link, self.say, self.args = link, say, args
        self.rd = Reader(link, say)
        self.started = False         # `/tmp/jogd` was typed
        self.ready = False
        self.running = False         # believed: jogd has not returned to the prompt
        self.ctrlz = self.ctrle = False
        self.tests = []
        self.between = []            # lines that came during the pauses
        self.aborted = None
        self.prompt_back = None
        self.errors = []             # E lines
        self.err_seen = 0
        self.interrupted = False
        self.button_pending = False
        self.six = args.mode in (3, 4)   # modes 3 and 4: all six joints sampled, model-based report and limits
        self.joint = ord(args.letter.upper()) - 64 if args.letter and not self.six else None
        self.joints0 = self.tcp0 = None  # modes 3/4: the first sample's joints (degrees) and tool point
        self.sample_on = False
        self.samples = []            # (arrival time, [six counts])
        self.origin = None           # the commanded joint's first sampled position, degrees
        self.next_p = 0.0
        self._li = 0
        self.sampling_dropped = False
        self.dot_written = False     # `.` already written because no `+` came (finish() need not write it again)
        self.stray_a = []            # `A` lines that came between moves (no move in force)
        self.after = None            # --after-abort: the Test that ran after the abort
        self.toward = bool(getattr(args, "toward_limit", False))

    # ---- low level
    def now(self):
        return self.link.clock()

    def write(self, data):
        self.link.port.write(data)
        if hasattr(self.link.port, "flush"):
            self.link.port.flush()
        return self.now()

    def pump(self, sample_before=None):
        got = self.rd.pump()
        errs = self.rd.of_kind("E")
        if len(errs) > self.err_seen:
            self.errors, self.err_seen = errs, len(errs)
            raise EError(errs[-1][2])
        if got:
            self.take_samples()
        if self.sample_on:
            now = self.now()
            if self.six and self.samples and now - self.samples[-1][0] > STALE_S:
                self.write(b".")
                self.sample_on = False
                self.say(f"NO `P` ANSWER for {STALE_S:g} s: the limits cannot be checked. `.` WRITTEN AT ONCE; the sequence ends, then `q`.")
                raise NoAnswer(f"no `P` answer for {STALE_S:g} s")
            if now >= self.next_p and (sample_before is None or now < sample_before - SAMPLE_GUARD_S):
                self.write(b"p")
                self.next_p = now + SAMPLE_S if now - self.next_p > SAMPLE_S else self.next_p + SAMPLE_S
        return got

    def degrees(self, counts):
        return counts[self.joint - 1] / COUNTS_PER_DEGREE[self.joint]

    def degrees6(self, counts):
        return [c / COUNTS_PER_DEGREE[i + 1] for i, c in enumerate(counts)]

    def check_limits(self, counts):
        """Modes 3/4, on every sample: the model's tool point and each joint against the first sample."""
        joints = self.degrees6(counts)
        tcp = model.forward(joints)[0]
        dist = float(np.linalg.norm(tcp - self.tcp0))
        away = [j - j0 for j, j0 in zip(joints, self.joints0)]
        worst = max(range(6), key=lambda i: abs(away[i]))
        what = None
        if dist > self.args.max_tcp:
            what = f"the tool point is {dist:.1f} mm from where it was at the first sample (limit --max-tcp {self.args.max_tcp:g})"
        elif abs(away[worst]) > self.args.max_joint:
            what = f"joint {worst + 1} is {away[worst]:+.2f} degrees from the first sample (limit --max-joint {self.args.max_joint:g})"
        if what:
            self.write(b".")
            self.sample_on = False
            self.say(f"LIMIT: {what}. `.` WRITTEN AT ONCE; the sequence ends, then `q`.")
            raise LimitTrip(what)

    def take_samples(self):
        new = [(t, [int(n) for n in text.split()[1:]]) for t, k, text in self.rd.lines[self._li:] if k == "P"]
        self._li = len(self.rd.lines)
        for t, counts in new:
            self.samples.append((t, counts))
            if self.sample_on and self.six and self.joints0 is not None:
                self.check_limits(counts)
            if self.sample_on and self.joint and self.origin is not None:
                away = self.degrees(counts) - self.origin
                if abs(away) > self.args.max_travel:
                    self.write(b".")
                    self.sample_on = False
                    self.say(f"TRAVEL LIMIT: joint {self.joint} is {away:+.2f} degrees from where the sequence started "
                             f"(limit {self.args.max_travel:g}). `.` WRITTEN AT ONCE; the sequence ends, then `q`.")
                    raise TravelLimit(f"joint {self.joint} moved {away:+.2f} degrees (limit {self.args.max_travel:g})")

    def probe_sampling(self):
        """From `jogd ready`: the first `p`; it must be answered within P_WAIT_S or sampling is dropped."""
        self.sample_on, self.next_p = True, self.now()
        if not self.wait(P_WAIT_S, until=lambda: self.samples):
            self.sample_on, self.sampling_dropped = False, True
            self.say(f"`p` WAS NOT ANSWERED within {P_WAIT_S:.0f} s: continuing WITHOUT sampling (no travel limit either).")
            return
        if self.six:
            self.joints0 = self.degrees6(self.samples[-1][1])
            self.tcp0 = model.forward(self.joints0)[0]
            self.say(f"sampling every {SAMPLE_S * 1000:.0f} ms; first sample: joints " + " ".join(f"{j:.3f}" for j in self.joints0)
                     + " tool point (model) " + " ".join(f"{x:.1f}" for x in self.tcp0) + " mm")
        elif self.joint:
            self.origin = self.degrees(self.samples[-1][1])
            self.say(f"sampling every {SAMPLE_S * 1000:.0f} ms; joint {self.joint} starts at {self.origin:.3f} degrees; travel limit {self.args.max_travel:g}")

    def wait(self, seconds, until=None):
        """Poll the port every POLL_S for `seconds`; True as soon as until() is true."""
        end = self.now() + seconds
        while True:
            self.pump()
            if until and until():
                return True
            if self.now() >= end:
                return False
            self.link.sleep(POLL_S)

    # ---- start
    def start(self):
        args = self.args
        self.link.drain(quiet=0.1, limit=1.0)
        self.rd.reset()
        self.started = self.running = True
        command = f"{args.remote} {args.mode} {args.max} {args.variant}"
        if self.six:
            for line in self.plan_lines(command):
                self.say(line)
        self.say(f"starting: {command}")
        sent = self.write(command.encode() + b"\r")
        end = sent + READY_WAIT_S
        try:
            while True:
                self.pump()
                if self.rd.of_kind("ready"):
                    self.ready = True
                    self.button_pending = False
                    self.say(f"jogd ready after {ms(self.now() - sent)}")
                    return True
                if b"\n" in self.rd.raw and self.rd.prompt_since():
                    self.running = False
                    self.say("jogd returned to the prompt without printing `jogd ready`")
                    return False
                if self.rd.button_text() and not self.button_pending:
                    self.button_pending = True
                    end = self.now() + BUTTON_WAIT_S
                    self.say("WAITING: PRESS THE PAUSE/CONTINUE BUTTON ON THE CONTROLLER'S FRONT PANEL NOW "
                             f"(up to {BUTTON_WAIT_S:.0f} s). Nothing moves, and nothing is sent, until `jogd ready` is seen.")
                if self.now() >= end:
                    self.say("no `jogd ready` within " + (f"{BUTTON_WAIT_S:.0f} s of the button request" if self.button_pending
                                                           else f"{READY_WAIT_S:.0f} s"))
                    return False
                self.link.sleep(POLL_S)
        except EError:
            self.say("jogd printed an E line before `jogd ready`")
            return False

    # ---- holds
    def hold(self, test, phases):
        """phases: [(letter, speed, seconds)]. Writes the command now and every 100 ms. Each phase must be answered."""
        last = due = None
        for letter, speed, seconds in phases:
            cmd = f"{letter}{speed:02d}".encode()
            mark, first = len(self.rd.lines), None
            while True:
                out = cmd if first is None or self.args.keepalive != "g" else b"g"      # `g`: jogd-c's one-byte keep-alive
                t = self.write(out)
                test.sent.append((t, out.decode() + (" (first)" if first is None else " (keep-alive)")))
                if first is None:
                    test.phases.append((t, cmd.decode()))
                if last is not None:
                    gap = t - last
                    test.gaps.append(gap)
                    if gap > LATE_S:
                        test.late.append(gap)
                last = t
                first = first if first is not None else t
                due = (due if due is not None else t) + KEEPALIVE_S
                self.wait_until(due)
                if self.note_abort(test, mark):                      # the controller aborted the move: no more keep-alives, no `.`
                    return last
                if self.six and self.rd.of_kind("-", mark):         # the move ended by itself: no more keep-alives, no `.`
                    test.self_end_t = self.rd.of_kind("-", mark)[0][0]
                    return last
                answered = any(k in ("+", "E") for _, k, _ in self.rd.lines[mark:])
                elapsed = self.now() - first
                if not answered and elapsed >= ANSWER_WAIT_S:
                    if self.six:
                        t = self.write(b".")
                        test.sent.append((t, "."))
                        test.stop_t, self.dot_written = t, True
                        self.say(f"NO `+` {ANSWER_WAIT_S:g} s after {cmd.decode()}: the start call may be blocking. `.` WRITTEN AT ONCE; the sequence ends, then `q`.")
                    raise NoAnswer(f"no `+` or `E` line within {ANSWER_WAIT_S} s of {cmd.decode()}")
                if answered and elapsed >= seconds:
                    break
        return last

    def note_abort(self, test, mark):
        """An `A` line since line `mark`: record it on the test (first one only). True when the test has an abort."""
        if test.abort is None:
            for i in range(mark, len(self.rd.lines)):
                if self.rd.lines[i][1] == "A":
                    t, _, text = self.rd.lines[i]
                    test.abort = dict(t=t, text=text, raw=self.rd.a_raw.get(i, len(self.rd.raw)))
                    break
        return test.abort is not None

    def wait_until(self, due):
        while self.now() < due:
            self.pump(sample_before=due)
            self.link.sleep(POLL_S)
        self.pump(sample_before=due)

    def stop_move(self, test, why):
        """`.` and wait for `-`."""
        mark = len(self.rd.lines)
        t = self.write(b".")
        test.sent.append((t, "."))
        test.stop_t = t
        if not self.wait(STOP_WAIT_S, until=lambda: self.rd.of_kind("-", mark) or self.rd.of_kind("A", mark)):
            raise NotStopped(f"no `-` within {STOP_WAIT_S} s of `.` ({why})")
        if self.note_abort(test, mark):                              # the operator-side stop and the abort met: the stop is complete
            self.say(f"{test.name}: an `A` line came while `.` was in flight: the stop is complete")

    def end_move(self, test, why):
        """After the hold of a move: nothing more to do when the controller aborted it (the `A` line ended it) or it ended by itself;
        else `.` and wait for `-`."""
        if test.abort:
            self.say(f"{test.name}: ABORTED BY THE CONTROLLER ({test.abort['text']}); no `.` written for it")
        elif test.self_end_t is not None:
            self.say(f"{test.name}: the move ended by itself (a `-` came {ms(test.self_end_t - test.sent[0][0])} after the start); no `.` written for it")
        else:
            self.stop_move(test, why)

    # ---- the tests
    def begin(self, name, expected):
        test = Test(name)
        test.expected, test.t0 = expected, self.now()
        if self.tests:
            self.tests[-1].t_end = test.t0
        self.tests.append(test)
        self._mark = len(self.rd.lines)
        return test

    def end(self, test):
        test.lines = self.rd.lines[self._mark:]
        got, expected = test.got, test.expected
        if test.abort:                    # `+`, then the `A` line; a `-` next to it (a `.` answered before the abort) is accepted
            got, expected = [g for g in got if g != "-"], [e for e in expected if e != "-"]
        test.ok = got == expected and not test.notes

    def pause(self):
        mark = len(self.rd.lines)
        self.wait(PAUSE_S)
        self.between += [ln for ln in self.rd.lines[mark:] if ln[1] not in ("P", "A")]
        self.stray_a += [ln for ln in self.rd.lines[mark:] if ln[1] == "A"]

    def end_sequence(self):
        """Sampling goes on TRAIL_S after the last stop; that closes the last test's window."""
        if self.sample_on:
            self.wait(TRAIL_S)
        if self.tests:
            self.tests[-1].t_end = self.now()

    def run_tests(self):
        a = self.args
        letter, speed = a.letter, a.speed
        opposite = letter.swapcase()
        # test 1
        t1 = self.begin("test 1: start and stop", [f"+{letter}{speed:02d}", "-"])
        try:
            self.hold(t1, [(letter, speed, a.hold)])
            self.end_move(t1, "test 1")
        finally:
            self.end(t1)
        if t1.abort:
            return self.abort_finish(t1)
        self.pause()
        # test 2
        if not a.no_deadman_test:
            t2 = self.begin("test 2: dead-man", [f"+{opposite}{speed:02d}", "T"])
            try:
                last = self.hold(t2, [(opposite, speed, a.hold)])
                mark = len(self.rd.lines)
                if t2.abort:
                    self.end_move(t2, "test 2")
                elif self.wait(DEADMAN_WAIT_S, until=lambda: self.rd.of_kind("T", mark)):
                    t2.deadman = self.rd.of_kind("T", mark)[0][0] - last
                    t2.stop_t = self.rd.of_kind("T", mark)[0][0]
                    t2.notes_info = f"`T` arrived {ms(t2.deadman)} after the last keep-alive was written"
                else:
                    t2.deadman = None
                    t2.notes.append(f"THE DEAD-MAN DID NOT FIRE: no `T` within {DEADMAN_WAIT_S:.0f} s of the last keep-alive")
                    self.say("THE DEAD-MAN DID NOT FIRE. Sending `.`.")
                    self.stop_move(t2, "dead-man missing")
            finally:
                self.end(t2)
            if t2.abort:
                return self.abort_finish(t2)
            self.pause()
        # test 3
        if a.speed2:
            first, second = f"+{letter}{speed:02d}", f"+{letter}{a.speed2:02d}"
            expected = [first, "-", second, "-"] if a.variant == "s" else [first, second, "-"]
            t3 = self.begin("test 3: speed change", expected)
            try:
                self.hold(t3, [(letter, speed, a.hold), (letter, a.speed2, a.hold)])
                self.end_move(t3, "test 3")
            finally:
                self.end(t3)
            if t3.abort:
                return self.abort_finish(t3)
        self.end_sequence()

    def keepalive(self, test, cmd, seconds):
        """Keep-alives of the command in force every KEEPALIVE_S for `seconds`, expecting no answer (an `A` line ends them)."""
        end, due = self.now() + seconds, self.now()
        mark = len(self.rd.lines)
        while self.now() < end:
            t = self.write(cmd)
            if test.sent:
                gap = t - test.sent[-1][0]
                test.gaps.append(gap)
                if gap > LATE_S:
                    test.late.append(gap)
            test.sent.append((t, cmd.decode() + " (keep-alive)"))
            due += KEEPALIVE_S
            self.wait_until(due)
            if self.note_abort(test, mark):
                return

    def run_maxvel(self):
        a = self.args
        cmd, vcmd = f"{a.letter}{a.speed:02d}", f"v{a.maxvel:02d}"
        test = self.begin(f"maxvel: {cmd}, {vcmd} after {a.hold:g} s, `.` {a.hold:g} s later", [f"+{cmd}", "V", "-"])
        try:
            self.hold(test, [(a.letter, a.speed, a.hold)])
            if not test.abort:
                t = self.write(vcmd.encode())
                test.sent.append((t, vcmd))
                test.phases.append((t, vcmd))
                test.maxvel_t = t
                self.keepalive(test, cmd.encode(), a.hold)
            self.end_move(test, "maxvel")
        finally:
            self.end(test)
            kinds = [k for _, k, _ in test.lines if k in ("+", "V", "-", "T")]
            if test.abort:
                kinds = [k for k in kinds if k != "-"]
            if kinds != (["+", "V"] if test.abort else ["+", "V", "-"]):
                test.notes.append(f"lines {kinds}, expected ['+', 'V', '-']")
            test.ok = not test.notes
        self.end_sequence()

    def maxvel_lines(self, test):
        """Mean speed of the commanded joint just before the `v` and after it settled."""
        if not getattr(test, "maxvel_t", None) or not self.samples or not self.joint:
            return []
        tv, te = test.maxvel_t, test.stop_t or self.samples[-1][0]

        def mean(a, b):
            pts = [(t, self.degrees(c)) for t, c in self.samples if a <= t <= b]
            if len(pts) < 2 or pts[-1][0] <= pts[0][0]:
                return None
            return (pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0])
        before, after = mean(tv - 0.5, tv), mean(tv + 0.4, te)
        if before is None or after is None:
            return ["    MAXVEL REPORT: too few samples before or after the `v`"]
        v = [t for _, k, t in test.lines if k == "V"]
        return [f"    MAXVEL REPORT: {self.args.letter}{self.args.speed:02d}, then v{self.args.maxvel:02d}"
                + (f" ({v[0]})" if v else "") + f": speed before {before:+.2f} deg/s (the 0.5 s before), after "
                f"{after:+.2f} deg/s (0.4 s after to the `.`), ratio {after / before if before else 0:.2f}"]

    def run_reverse(self):
        a = self.args
        first, opposite = a.letter, a.letter.swapcase()
        for i, (letter, tag) in enumerate(((first, "a"), (opposite, "b"), (first, "c"), (first, "d"))):
            t = self.begin(f"reverse ({tag}): {letter}{a.speed:02d}", [f"+{letter}{a.speed:02d}", "-"])
            t.move_report = True
            try:
                self.hold(t, [(letter, a.speed, a.hold)])
                self.end_move(t, f"reverse ({tag})")
            finally:
                self.end(t)
            if t.abort:
                return self.abort_finish(t)
            if i < 3:
                self.pause()
        self.end_sequence()

    # ---- modes 3 and 4: Cartesian letters, found out one at a time
    def abort_finish(self, test):
        """The controller aborted `test`: the sequence ends here (no `.` was written for it). With --after-abort the next move follows."""
        if self.args.after_abort:
            self.run_after_abort(test)
        self.end_sequence()

    def run_after_abort(self, aborted):
        """--after-abort LETTER: 1.5 s after the `A` line, that letter at the same speed for the same hold; a normal move."""
        a = self.args
        letter = a.after_abort
        cmd = f"{letter}{a.speed:02d}"
        same = aborted.phases and aborted.phases[0][1] == cmd
        self.say(f"AFTER THE ABORT: waiting {AFTER_ABORT_WAIT_S:g} s, then {cmd} for {a.hold:g} s (the same speed and hold)")
        end = aborted.abort["t"] + AFTER_ABORT_WAIT_S
        self.wait(max(end - self.now(), 0.0))
        if same:                       # jogd-c ignores repeats of the command that was in force until a `.` or another command
            self.write(b".")
            self.say("  the same letter as the aborted move: one `.` written first (jogd-c prints nothing for it)")
            self.wait(0.2)
        t = self.begin(f"after the abort: {cmd}", [f"+{cmd}", "-"])
        t.label, t.move_report = letter, True
        self.after = t
        try:
            self.hold(t, [(letter, a.speed, a.hold)])
            self.end_move(t, t.name)
        finally:
            self.end(t)

    def sweep_letters(self):
        return list(self.args.letters)

    def plan_lines(self, command):
        a = self.args
        if a.sweep:
            what = f"sweep of letters {a.letters}, each + then - ({2 * len(a.letters)} moves)"
        elif a.dry:
            what = "no motion (--dry)"
        elif a.vector:
            what = (f"one vector move: {vector13(a.vector)}{a.speed:02d} for {a.hold:g} s"
                    + (", then the opposite vector for the same time" if a.ret else ""))
        elif self.toward:
            what = (f"one move: {a.letter}{a.speed:02d} kept going until an `A` line, a `-` by itself, an E line, a limit below or "
                    f"--time-limit {a.time_limit:g} s")
        else:
            what = f"one move: {a.letter}{a.speed:02d} for {a.hold:g} s"
        extra = []
        if self.toward:
            extra.append("WARNING: --toward-limit is meant to END IN A CONTROLLER ABORT (a `Path error` at a joint limit; the controller may "
                         "also switch arm power off). Keep a hand on the E-stop.")
        if a.after_abort:
            extra.append(f"--after-abort {a.after_abort}: 1.5 s after an `A` line, {a.after_abort}{a.speed:02d} for {a.hold:g} s")
        extra.append("keep-alive: " + ("the single byte `g` every 100 ms (jogd-c)" if a.keepalive == "g"
                                       else "the start command repeated every 100 ms"))
        return extra + [f"MODE {a.mode} ({'world' if a.mode == 3 else 'tool'} frame): {what}",
                f"limits in force (checked on every sample, `.` at once): tool point (model) {a.max_tcp:g} mm from the first sample, "
                f"any joint {a.max_joint:g} degrees from the first sample; speed {a.speed}% (jogd ceiling {a.max}); "
                + (f"no fixed hold (--time-limit {a.time_limit:g} s)" if self.toward else f"hold {a.hold:g} s per move"),
                f"the exact jogd command line: {command}"]

    def run_six(self):
        a = self.args
        if a.vector:
            seq = [vector13(a.vector)] + ([vector13(opposite_vector(a.vector))] if a.ret else [])
        else:
            seq = [x for L in self.sweep_letters() for x in (L, L.lower())] if a.sweep else [a.letter]
        for i, letter in enumerate(seq):
            cmd = f"{letter}{a.speed:02d}"
            t = self.begin(f"move {i + 1}/{len(seq)}: {cmd}", [f"+{cmd}", "-"])
            t.label = letter if len(letter) == 1 else letter[:7]
            try:
                self.hold(t, [(letter, a.speed, a.time_limit if self.toward else a.hold)])
                if self.toward:
                    t.end_reason = ("an `A` line" if t.abort else "a `-` by itself" if t.self_end_t is not None
                                    else f"--time-limit {a.time_limit:g} s")
                    self.say(f"TOWARD-LIMIT ENDED BY: {t.end_reason}")
                self.end_move(t, t.name)
            finally:
                self.end(t)
            if not t.ok:
                raise Abort(f"{t.name}: lines {t.got}, expected {t.expected}")
            if t.abort:
                return self.abort_finish(t)
            if i < len(seq) - 1:
                self.pause()
        self.end_sequence()

    def analyse(self, test):
        """One move from the samples: baseline = last sample written before its first command, end = last sample
        before the window closed. None if there is no sample after the first command."""
        if not test.sent or not self.samples:
            return None
        tf = test.sent[0][0]
        t_end = test.t_end if test.t_end is not None else self.samples[-1][0]
        before = [x for x in self.samples if x[0] <= tf]
        base = before[-1] if before else self.samples[0]
        win = [x for x in self.samples if tf < x[0] <= t_end]
        if not win:
            return None
        j0, j1 = self.degrees6(base[1]), self.degrees6(win[-1][1])
        p0, r0, _, _ = model.forward(j0)
        p1, r1, _, _ = model.forward(j1)
        d = p1 - p0
        a0, a1 = model.yaw_pitch_roll(r0), model.yaw_pitch_roll(r1)
        dypr = [(y - x + 180.0) % 360.0 - 180.0 for x, y in zip(a0, a1)]
        ang_w, ax_w = rot_axis_angle(r1 @ r0.T)
        ang_t, ax_t = rot_axis_angle(r0.T @ r1)
        # moving samples: counts differ from the one before; rates between the first and the last of them
        prev, moving = base, []
        for x in win:
            if x[1] != prev[1]:
                moving.append(x)
            prev = x
        speed = rate = None
        if len(moving) >= 2 and moving[-1][0] > moving[0][0]:
            (ta, ca), (tb, cb) = moving[0], moving[-1]
            pa, ra, _, _ = model.forward(self.degrees6(ca))
            pb, rb, _, _ = model.forward(self.degrees6(cb))
            dt = tb - ta
            speed = float(np.linalg.norm(pb - pa)) / dt
            rate = rot_axis_angle(rb @ ra.T)[0] / dt
        res = dict(name=test.name, letter=test.phases[0][1][0] if test.phases else "?", dj=[b - a for a, b in zip(j0, j1)],
                   d=d, dist=float(np.linalg.norm(d)), d_tool=r0.T @ d, dypr=dypr, ang_w=ang_w, ax_w=ax_w, ang_t=ang_t, ax_t=ax_t,
                   speed=speed, rate=rate, pitch0=a0[1], n=len(moving))
        res["verdict"] = self.verdict(res)
        res.update(self.timing(test))
        res["label"] = test.label or res["letter"]
        res["unit"] = "degrees" if res["label"][0] in "DEFdef" else "mm"
        res["stop_fig"] = self.stop_figures(test, t_end)
        res["vec"] = self.vector_figures(test, res)
        return res

    def timing(self, test):
        """Times that need no sample: start acknowledged after, `-` after `.`, `-` by itself."""
        first = test.sent[0][0] if test.sent else None
        acks = [t for t, k, _ in test.lines if k == "+"]
        minus = [t for t, k, _ in test.lines if k == "-"]
        out = dict(ack_ms=None, stop_ms=None, self_ms=None, abort_ms=None)
        if first is not None and acks:
            out["ack_ms"] = (acks[0] - first) * 1000
        if test.abort is not None and first is not None:
            out["abort_ms"] = (test.abort["t"] - first) * 1000
        elif test.self_end_t is not None and first is not None:
            out["self_ms"] = (test.self_end_t - first) * 1000
        elif test.stop_t is not None and minus and minus[0] >= test.stop_t:
            out["stop_ms"] = (minus[0] - test.stop_t) * 1000
        return out

    def interp(self, rows, t):
        """Counts at measure time t from (time, counts) rows, interpolated; the end rows hold outside."""
        if t <= rows[0][0]:
            return rows[0][1]
        for (t0, a), (t1, b) in zip(rows, rows[1:]):
            if t0 <= t <= t1:
                k = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return [x + (y - x) * k for x, y in zip(a, b)]
        return rows[-1][1]

    def stop_figures(self, test, t_end):
        """Around the `.`: how far the tool point and the tool's attitude went after it was written, and the speed just before.
        Sample times are taken as their arrival minus SAMPLE_LAG_S (the position was read that long before the line came)."""
        if test.stop_t is None or not self.samples:
            return None
        rows = [(t - SAMPLE_LAG_S, c) for t, c in self.samples]
        ts = test.stop_t
        pose = lambda counts: model.forward(self.degrees6(counts))[:2]      # noqa: E731
        p_s, r_s = pose(self.interp(rows, ts))
        later = [x for x in rows if x[0] <= t_end]
        out = dict(travel=None, rot=None, speed=None, rate=None)
        if later and later[-1][0] > ts:
            p_e, r_e = pose(later[-1][1])
            out["travel"], out["rot"] = float(np.linalg.norm(p_e - p_s)), rot_axis_angle(r_e @ r_s.T)[0]
        pre = [x for x in rows if x[0] <= ts]
        if len(pre) >= 2 and pre[-1][0] > pre[-2][0]:
            (ta, ca), (tb, cb) = pre[-2], pre[-1]
            pa, ra = pose(ca)
            pb, rb = pose(cb)
            out["speed"] = float(np.linalg.norm(pb - pa)) / (tb - ta)
            out["rate"] = rot_axis_angle(rb @ ra.T)[0] / (tb - ta)
        return out

    def vector_figures(self, test, r):
        """A vector move: the angle between the measured displacement and the requested direction, and how far off the line."""
        if not test.phases or not test.phases[0][1].startswith("@"):
            return None
        want = np.array([int(x) for x in re.findall(r"[+-]\d", test.phases[0][1][1:7])], dtype=float)
        got = r["d_tool"] if self.args.mode == 4 else r["d"]
        n = float(np.linalg.norm(got))
        if n < 1.0:
            return dict(want=want, angle=None, off=None, frame="tool" if self.args.mode == 4 else "world")
        u = want / float(np.linalg.norm(want))
        along = float(got @ u)
        return dict(want=want, frame="tool" if self.args.mode == 4 else "world",
                    angle=math.degrees(math.acos(max(-1.0, min(1.0, along / n)))), off=math.sqrt(max(n * n - along * along, 0.0)))

    def timing_lines(self, test, r):
        """The lines the first runs of jogd-c are for, for one mode 3 / 4 move."""
        out = []
        if r["ack_ms"] is None:
            out.append("    START: NOT ACKNOWLEDGED (no `+` line)")
        else:
            out.append(f"    START: start acknowledged after {r['ack_ms']:.0f} ms")
            if r["ack_ms"] > ACK_WARN_S * 1000:
                out.append(f"    WARNING: the start was acknowledged after {r['ack_ms']:.0f} ms, over {ACK_WARN_S * 1000:.0f} ms: the start call may be "
                           "BLOCKING. If the controller's move call does not return until the move is finished, the program cannot read `.` "
                           "and the move would run its whole 150 mm (45 degrees)")
        if r["abort_ms"] is not None:
            out.append("    " + self.abort_text(test, r))
        elif r["self_ms"] is not None:
            unit = "degrees" if r["unit"] == "degrees" else "mm"
            far = r["ang_t" if self.args.mode == 4 else "ang_w"] if unit == "degrees" else r["dist"]
            out.append(f"    ENDED BY ITSELF: the move ended by itself {r['self_ms']:.0f} ms after the start, after {far:.1f} {unit} "
                       f"(tool point {r['dist']:.1f} mm, rotation {r['ang_t' if self.args.mode == 4 else 'ang_w']:.1f} degrees); no `.` was written for it")
        elif r["stop_ms"] is not None or test.stop_t is not None:
            f = r["stop_fig"] or {}
            head = (f"`-` came {r['stop_ms']:.0f} ms after `.`" if r["stop_ms"] is not None else "no `-` seen after `.`")
            after = (f"the tool point travelled {f['travel']:.1f} mm ({f['rot']:.2f} degrees of rotation) after `.` was written"
                     if f.get("travel") is not None else "travel after `.` not known (no sample after it)")
            before = (f"tool-point speed just before the `.`: {f['speed']:.1f} mm/s ({f['rate']:.2f} deg/s)"
                      if f.get("speed") is not None else "speed just before the `.` not known")
            out.append(f"    STOP: {head}; {after}; {before}")
        v = r["vec"]
        if v:
            if v["angle"] is None:
                out.append(f"    VECTOR: requested ({' '.join(f'{x:+.0f}' for x in v['want'])}) in the {v['frame']} frame; the tool moved under 1 mm: direction not known")
            else:
                out.append(f"    VECTOR: requested ({' '.join(f'{x:+.0f}' for x in v['want'])}) in the {v['frame']} frame"
                           + (" (the tool's own frame at the start)" if v["frame"] == "tool" else "")
                           + f"; measured displacement is {v['angle']:.1f} degrees from it; off-line distance {v['off']:.1f} mm")
        return out

    def abort_text(self, test, r=None):
        """"ABORTED BY THE CONTROLLER: A <code> <text>, N ms after the start, after X mm" (X degrees for a rotation / a joint)."""
        first = test.sent[0][0] if test.sent else test.abort["t"]
        r = r if r is not None else (self.analyse(test) if self.six else None)
        if r is not None and self.six:
            far = f"{r['ang_t' if self.args.mode == 4 else 'ang_w']:.1f} degrees" if r["unit"] == "degrees" else f"{r['dist']:.1f} mm"
        else:
            far = self.joint_travel(test)
            far = "distance not known" if far is None else f"{far:.2f} degrees"
        return f"ABORTED BY THE CONTROLLER: {test.abort['text']}, {(test.abort['t'] - first) * 1000:.0f} ms after the start, after {far}"

    def joint_travel(self, test):
        """Mode 1: degrees the commanded joint moved from the last sample before the first command to the end of the test's window."""
        if not self.joint or not test.sent or not self.samples:
            return None
        tf = test.sent[0][0]
        t_end = test.t_end if test.t_end is not None else self.samples[-1][0]
        before = [x for x in self.samples if x[0] <= tf]
        base = before[-1] if before else self.samples[0]
        win = [x for x in self.samples if tf < x[0] <= t_end]
        return abs(self.degrees(win[-1][1]) - self.degrees(base[1])) if win else None

    def after_lines(self):
        """The --after-abort report: acknowledged, stopped, MOVED AFTER THE ABORT, PAUSE/CONTINUE seen."""
        a = self.args
        if not a.after_abort:
            return []
        out = ["", f"AFTER THE ABORT (--after-abort {a.after_abort}):"]
        first = next((t for t in self.tests if t.abort), None)
        if first is None:
            return out + ["  no `A` line came, so the after-abort move was NOT run (nothing was checked)"]
        t = self.after
        if t is None:
            return out + ["  the sequence ended before the after-abort move could start (see STOPPED EARLY)"]
        r = self.timing(t)
        out.append(f"  move {t.name}: " + ("start acknowledged after %.0f ms" % r["ack_ms"] if r["ack_ms"] is not None else "START NOT ACKNOWLEDGED (no `+` line)"))
        if t.abort:
            out.append("  " + self.abort_text(t))
        elif r["stop_ms"] is not None:
            out.append(f"  stop: `-` came {r['stop_ms']:.0f} ms after `.`")
        moved, how = self.moved(t)
        out.append("  MOVED AFTER THE ABORT: " + ("not known" if moved is None else "yes" if moved else "no") + (f" ({how})" if how else ""))
        seen = homezc.BUTTON.encode() in self.rd.raw[first.abort["raw"]:]
        out.append("  PAUSE/CONTINUE text seen after the abort: " + ("YES (the arm asked for the front-panel button again)" if seen else "no"))
        return out

    def moved(self, test):
        """(True/False/None, how much) for the after-abort move."""
        if self.six:
            r = self.analyse(test)
            if r is None:
                return None, "no sample after the first command"
            ang = r["ang_t" if self.args.mode == 4 else "ang_w"]
            return (r["dist"] >= MOVED_MM or ang >= MOVED_DEG), f"tool point {r['dist']:.1f} mm, rotation {ang:.2f} degrees, from the PC's model"
        deg = self.joint_travel(test)
        if deg is None:
            return None, "no sample after the first command"
        return deg >= MOVED_JOINT_DEG, f"joint {self.joint} moved {deg:.3f} degrees"

    def verdict(self, r):
        tool = self.args.mode == 4
        d = r["d_tool"] if tool else r["d"]
        ang, ax = (r["ang_t"], r["ax_t"]) if tool else (r["ang_w"], r["ax_w"])
        frame = "tool" if tool else "world"
        trans, rot_equiv = r["dist"], ang * ROT_MM_PER_DEG
        if trans < 1.0 and ang < 0.2:
            return f"no motion seen (translation {trans:.1f} mm; rotation {ang:.1f} deg)"
        i = int(np.argmax(np.abs(d)))
        off = float(np.linalg.norm(np.delete(d, i)))
        if trans > DOMINATES * rot_equiv:
            return f"mostly {'+' if d[i] >= 0 else '-'}{'XYZ'[i]} translation ({trans:.1f} mm; off-axis {off:.1f} mm; rotation {ang:.1f} deg)" + (" in the tool frame" if tool else "")
        if rot_equiv > DOMINATES * trans:
            k = int(np.argmax(np.abs(ax)))
            about = (f"{'+' if ax[k] >= 0 else '-'}{'XYZ'[k]} ({frame})" if abs(ax[k]) > 0.9 * np.linalg.norm(ax)
                     else "(" + " ".join(f"{x:+.2f}" for x in ax) + f") ({frame})")
            return f"mostly rotation ({ang:.1f} deg about {about}; translation {trans:.1f} mm)"
        return f"mixed: translation {trans:.1f} mm (largest {'+' if d[i] >= 0 else '-'}{'XYZ'[i]}), rotation {ang:.1f} deg"

    def six_lines(self):
        results = [(t, self.analyse(t)) for t in self.tests]
        results = [(t, r) for t, r in results if r]
        if not results and not self.args.dry:
            return ["MOVE REPORT (modes 3/4): no sample after the first command"]
        if not results:
            return []
        tool = self.args.mode == 4
        out = ["", f"MOVE REPORT, mode {self.args.mode} ({'tool' if tool else 'world'} frame). " + MODEL_NOTE]
        for t, r in results:
            out.append(f"  {r['name']}")
            out += self.timing_lines(t, r)
            out.append("    joint change (degrees, from counts): " + "  ".join(f"J{i + 1} {x:+.3f}" for i, x in enumerate(r["dj"])))
            out.append(f"    tool point displacement (model world frame): dx {r['d'][0]:+.1f}  dy {r['d'][1]:+.1f}  dz {r['d'][2]:+.1f} mm; length {r['dist']:.1f} mm")
            if tool:
                out.append(f"    displacement in the tool's own frame at the start (rot.T @ d): {r['d_tool'][0]:+.1f}  {r['d_tool'][1]:+.1f}  {r['d_tool'][2]:+.1f} mm")
            out.append(f"    change of yaw / pitch / roll: {r['dypr'][0]:+.2f} / {r['dypr'][1]:+.2f} / {r['dypr'][2]:+.2f} degrees"
                       + (f"  (pitch was {r['pitch0']:+.1f}: near +/-90 yaw and roll are not meaningful; trust the axis)" if abs(r["pitch0"]) > 85 else ""))
            out.append("    mean tool-point speed " + (f"{r['speed']:.1f} mm/s, mean rotation rate {r['rate']:.2f} deg/s (first to last moving sample, {r['n']} moving samples)"
                                                       if r["speed"] is not None else f"not known (only {r['n']} moving sample(s))"))
            out.append(f"    total rotation {r['ang_t' if tool else 'ang_w']:.2f} degrees about " + " ".join(f"{x:+.2f}" for x in (r['ax_t'] if tool else r['ax_w'])) + f" ({'tool' if tool else 'world'} frame)")
            out.append(f"    VERDICT: {r['verdict']}")
        if self.args.sweep:
            out += self.sweep_table(results)
        if self.args.vector and self.args.ret:
            out += self.return_lines()
        return out

    def end_lines(self, indent):
        """Where the arm ended against where the run began (first sample to last sample)."""
        if not (self.samples and self.joints0 is not None):
            return []
        j1 = self.degrees6(self.samples[-1][1])
        end = model.forward(j1)[0] - self.tcp0
        return [f"{indent}joints (degrees): " + "  ".join(f"J{i + 1} {b - a:+.3f}" for i, (a, b) in enumerate(zip(self.joints0, j1))),
                f"{indent}tool point (mm): dx {end[0]:+.1f}  dy {end[1]:+.1f}  dz {end[2]:+.1f}; length {float(np.linalg.norm(end)):.1f}"]

    def return_lines(self):
        lines = self.end_lines("    ")
        if not lines:
            return []
        end = model.forward(self.degrees6(self.samples[-1][1]))[0] - self.tcp0
        return ["", f"RETURN: after the vector and the opposite vector the tool point ended {float(np.linalg.norm(end)):.1f} mm from where it started "
                    "(first sample to last sample):"] + lines

    def sweep_table(self, results):
        tool = self.args.mode == 4
        out = ["", f"SWEEP TABLE, mode {self.args.mode} ({'tool' if tool else 'world'} frame); mm and degrees are the PC model's, per move",
               "  letter     dx     dy     dz  d-yaw d-pitch d-roll   mm/s  deg/s  start ms stop ms after `.` ended by itself  verdict",
               "         (start ms: start acknowledged after, '!' = over 600 ms; stop ms: `-` after `.`; after `.`: travel after `.` was written)"]
        for _, r in results:
            sp = f"{r['speed']:6.1f}" if r["speed"] is not None else "     -"
            rt = f"{r['rate']:6.2f}" if r["rate"] is not None else "     -"
            if r["ack_ms"] is None:
                start = "-"
            else:
                start = f"{r['ack_ms']:.0f}" + ("!" if r["ack_ms"] > ACK_WARN_S * 1000 else "")
            stop = f"{r['stop_ms']:.0f}" if r["stop_ms"] is not None else "-"
            fig = r["stop_fig"] or {}
            if fig.get("travel") is None:
                after = "-"
            else:
                after = f"{fig['rot']:.1f} deg" if r["unit"] == "degrees" else f"{fig['travel']:.1f} mm"
            if r["self_ms"] is None:
                itself = "no"
            else:
                itself = f"yes, {r['self_ms']:.0f} ms, " + (f"{r['ang_t' if tool else 'ang_w']:.1f} deg" if r["unit"] == "degrees" else f"{r['dist']:.1f} mm")
            out.append(f"  {r['letter']:>6} {r['d'][0]:+6.1f} {r['d'][1]:+6.1f} {r['d'][2]:+6.1f} {r['dypr'][0]:+6.1f} {r['dypr'][1]:+7.1f} {r['dypr'][2]:+6.1f} {sp} {rt}"
                       f"  {start:>8} {stop:>7} {after:>9} {itself:<15}  {r['verdict']}")
        if tool:
            out.append("  tool-frame displacement at each move's start (rot.T @ d), mm:")
            out += [f"    {r['letter']:>4}: {r['d_tool'][0]:+6.1f} {r['d_tool'][1]:+6.1f} {r['d_tool'][2]:+6.1f}" for _, r in results]
        if self.samples and self.joints0 is not None:
            out.append("  where the arm ended against where the sweep began (first sample to last sample):")
            out += self.end_lines("    ")
        out.append("  rows come from the moves that ran; a sweep that stopped early has fewer than "
                   f"{2 * len(self.sweep_letters())} (see STOPPED EARLY).")
        return out

    # ---- the end
    def finish(self, emergency):
        """Get jogd stopped and out, and the prompt back. emergency: Ctrl-Z first."""
        say = self.say
        self.sample_on = False
        if not self.started:
            return
        if self.button_pending and not self.ready:
            if not self.rd.prompt_since():
                self.write(CTRL_E)
                self.ctrle = True
                say("KILLED while waiting for the button (nothing moved): Ctrl-E sent")
                self.link.drain(quiet=1.0, limit=3.0)
                self.prompt_back = console.at_prompt(self.link)
            else:
                self.prompt_back = True
            self.running = False
            return
        if emergency:
            self.write(CTRL_Z)
            self.ctrlz = True
            say("HALT: Ctrl-Z written FIRST (the arm is stopped by the controller), then `.` and `q`")
        if not self.rd.prompt_since():
            try:
                self.wait(0.3 if not emergency else 0.0, until=self.rd.prompt_since)
            except Abort:
                pass
        if self.rd.prompt_since():
            self.prompt_back = True
            self.running = False
        else:
            if not self.dot_written:
                self.write(b".")
            mark = len(self.rd.raw)
            self.write(b"q")
            say("sent `.` then `q`" if emergency or self.aborted else "sent `q`")
            try:
                got = self.wait(QUIT_WAIT_S, until=lambda: self.rd.prompt_since(mark))
            except Abort:
                got = self.rd.prompt_since(mark)
            self.prompt_back = got
            if not got:
                self.escalate()
        self.running = not self.prompt_back

    def quit_clean(self):
        """The normal end: `q`, the prompt within 5 s."""
        self.sample_on = False
        if self.rd.prompt_since():
            self.prompt_back, self.running = True, False
            return
        mark = len(self.rd.raw)
        self.write(b"q")
        self.say("sent `q`")
        try:
            self.prompt_back = self.wait(QUIT_WAIT_S, until=lambda: self.rd.prompt_since(mark))
        except Abort:
            self.prompt_back = self.rd.prompt_since(mark)
        self.running = not self.prompt_back
        if not self.prompt_back:
            self.escalate()

    def escalate(self):
        self.say(f"NO `$` PROMPT within {QUIT_WAIT_S:.0f} s of `q`: HALTING with Ctrl-Z (and Ctrl-E if needed). THE ARM MAY STILL BE UNDER JOGD.")
        if not self.ctrlz:
            self.write(CTRL_Z)
            self.ctrlz = True
        self.link.ahead = b""

        def spy(text):
            if "sending Ctrl-E" in text:
                self.ctrle = True
            self.say(text)
        self.prompt_back = homezc.recover(self.link, spy)
        self.running = not self.prompt_back
        self.say("the controller is back at its `$` prompt" if self.prompt_back else "THE CONTROLLER IS NOT AT ITS `$` PROMPT")

    # ---- report
    def summary(self, before, after, dry):
        out = ["", "SUMMARY"]
        if self.six:
            out += self.six_lines()
        for test in self.tests:
            out.append(f"{test.name}: " + ("as expected" if test.ok else "NOT AS EXPECTED"))
            events = [(t, f"sent {label}") for t, label in test.sent if "keep-alive" not in label]
            alive = [t for t, label in test.sent if "keep-alive" in label]
            if alive:
                events.append((alive[-1], f"sent the last keep-alive ({len(alive)} keep-alives after the first command)"))
            events += [(t, f"jogd: {text}") for t, k, text in test.lines if k != "P"]
            for t, what in sorted(events, key=lambda e: e[0]):
                out.append(f"    {ms(t - test.t0):>12}  {what}")
            out.append(f"    expected lines {test.expected}, got {test.got}")
            if test.abort and not (self.six and self.analyse(test)):
                out.append("    " + self.abort_text(test))
            if test.gaps:
                out.append(f"    keep-alive gaps: max {ms(max(test.gaps))}" + (f"; LATE (over {ms(LATE_S)}): {len(test.late)} time(s), worst {ms(max(test.late))}" if test.late else "; none late"))
            if getattr(test, "notes_info", None):
                out.append("    " + test.notes_info)
            out += [f"    {n}" for n in test.notes]
            out += self.sample_lines(test)
            out += self.maxvel_lines(test)
        if self.stray_a:
            out.append("an `A` line came between moves (no move in force; not a failure): " + ", ".join(t for _, _, t in self.stray_a))
        if self.between:
            out.append("lines during the pauses (not expected): " + ", ".join(t for _, _, t in self.between))
        if True:
            others = [t for _, _, t in self.rd.of_kind("other") if self.args.remote not in t and homezc.BUTTON not in t]
            if others:
                out.append("other lines seen: " + " | ".join(others[:10]))
        if before[0] and after[0]:
            out.append("change from before to after:")
            for i in range(6):
                dc = f", {after[1][i] - before[1][i]:+d} counts" if before[1] and after[1] else ""
                out.append(f"    joint {i + 1}: {after[0][i] - before[0][i]:+.3f} degrees{dc}   ({before[0][i]:.3f} -> {after[0][i]:.3f})")
        else:
            out.append("change from before to after: NOT KNOWN (the joints could not be read both times)")
        out.append("E lines: " + (", ".join(t for _, _, t in self.rd.of_kind('E')) if self.rd.of_kind("E") else "none"))
        out += self.after_lines()
        if self.aborted:
            out.append(f"STOPPED EARLY: {self.aborted}")
        late = sum(len(t.late) for t in self.tests)
        if late:
            out.append(f"KEEP-ALIVES LATE: {late} gap(s) over {ms(LATE_S)} on this tool's clock")
        if self.six:
            slow = [(t.name, r["ack_ms"]) for t in self.tests for r in [self.timing(t)] if r["ack_ms"] is not None and r["ack_ms"] > ACK_WARN_S * 1000]
            if slow:
                out.append("START ACKNOWLEDGED LATE (over 600 ms; the start call may be blocking): " + "; ".join(f"{n}: {a:.0f} ms" for n, a in slow))
        out.append(f"jogd ready seen: {'yes' if self.ready else 'NO'}")
        out.append("prompt after `q`: " + ("back" if self.prompt_back else "NOT BACK" if self.prompt_back is False else "unknown"))
        out.append(f"Ctrl-Z sent: {'YES' if self.ctrlz else 'no'}; Ctrl-E sent: {'YES' if self.ctrle else 'no'}")
        return "\n".join(out)

    def position_at(self, rows, t):
        """Degrees at time t from (time, degrees) rows, interpolated; the end rows hold outside."""
        if t <= rows[0][0]:
            return rows[0][1]
        for (t0, a), (t1, b) in zip(rows, rows[1:]):
            if t0 <= t <= t1:
                return a + (b - a) * (t - t0) / (t1 - t0) if t1 > t0 else b
        return rows[-1][1]

    def sample_lines(self, test):
        """Table of the commanded joint's samples for one test, per-phase speeds, the peak, and for the reversal moves a report."""
        if not self.joint or not test.sent:
            return []
        if not self.samples:
            return ["    samples: none" + (" (`p` was never answered)" if self.sampling_dropped else " (--sample was not given)")] if self.args.sample else []
        j, tf = self.joint, test.sent[0][0]
        t_end = test.t_end if test.t_end is not None else self.samples[-1][0]
        before = [x for x in self.samples if x[0] <= tf]
        picked = ([before[-1]] if before else []) + [x for x in self.samples if tf < x[0] <= t_end]
        rows = [(t, self.degrees(c)) for t, c in picked]
        if len(rows) < 2:
            return [f"    samples of joint {j}: fewer than two in this test's window"]
        out = [f"    samples of joint {j} (time since the first command; arrival times, the controller adds its own delay of about 95 ms):",
               "        time ms      counts      degrees      deg/s since previous"]
        speeds = []
        for i, ((t, c), (_, d)) in enumerate(zip(picked, rows)):
            v = (d - rows[i - 1][1]) / (t - picked[i - 1][0]) if i and t > picked[i - 1][0] else None
            if v is not None:
                speeds.append((abs(v), t, v))
            out.append(f"      {(t - tf) * 1000:9.1f} {c[j - 1]:11d} {d:12.3f}      " + (f"{v:+9.2f}" if v is not None else "        -"))
        peak = max(speeds)
        out.append(f"    largest speed in this test: {peak[2]:+.2f} deg/s at {(peak[1] - tf) * 1000:.1f} ms")
        for k, (ts, label) in enumerate(test.phases):
            te = test.phases[k + 1][0] if k + 1 < len(test.phases) else (test.stop_t or t_end)
            travel = self.position_at(rows, te) - self.position_at(rows, ts)
            out.append(f"    phase {label}: travel {travel:+.3f} degrees in {(te - ts) * 1000:.0f} ms, mean {travel / (te - ts):+.2f} deg/s")
        if test.move_report and self.origin is not None and test.phases:
            ts = test.phases[0][0]
            te = test.stop_t or t_end
            move = self.position_at(rows, te) - self.position_at(rows, ts)
            rel = [d - self.origin for _, d in rows]
            crossed = any(r > 0.05 for r in rel) and any(r < -0.05 for r in rel)
            out.append(f"    MOVE REPORT {test.name}: travel during the move {move:+.3f} degrees, mean {move / (te - ts):+.2f} deg/s, "
                       f"peak {peak[2]:+.2f} deg/s; travel up to the end of the window {rows[-1][1] - rows[0][1]:+.3f} degrees; "
                       f"position relative to the sequence's start {rel[0]:+.3f} -> {rel[-1]:+.3f} (extremes {min(rel):+.3f}, {max(rel):+.3f}); "
                       f"crossed the starting position: {'YES' if crossed else 'no'}")
        return out

    def exit_ok(self, before, after, dry):
        if not self.ready or self.rd.of_kind("E") or self.aborted or self.interrupted:
            return False
        if self.ctrlz or self.ctrle or not self.prompt_back:
            return False
        if dry:
            lines_ok = [t for _, k, t in self.rd.lines if k not in ("other", "P", "MV")] == ["jogd ready"]
            changed = not before[1] or not after[1] or before[1] != after[1]
            return lines_ok and not changed
        if self.args.after_abort and (self.after is None or not any(t.abort for t in self.tests)):
            return False                  # asked for the check and it was not done
        return all(t.ok for t in self.tests) and not self.between and bool(self.tests)


def parse_args(argv, repo):
    ap = argparse.ArgumentParser(prog="a465-jogtest", description="Bench test of the jogd program on the controller.")
    ap.add_argument("letter", nargs="?", help="A..F (joint 1..6, positive; in modes 3/4 the letters' own axes) or a..f (negative): the first move")
    ap.add_argument("--dry", action="store_true", help="start jogd, wait for `jogd ready`, send `q`; no motion command")
    ap.add_argument("--speed", type=int, default=2, help="percent, 1..--max (default %(default)s)")
    ap.add_argument("--hold", type=float, default=None, help=f"seconds of keep-alives per phase, {HOLD_RANGE[0]}..{HOLD_RANGE[1]} (default 1.0; "
                    f"with --toward-limit only the hold of the --after-abort move, default {AFTER_ABORT_HOLD_S:g})")
    ap.add_argument("--speed2", type=int, default=None, help="test 3: a second speed, different from --speed")
    ap.add_argument("--variant", choices=("s", "r"), default="s", help="jogd's speed-change variant (default %(default)s)")
    ap.add_argument("--max", type=int, default=5, help=f"jogd's speed ceiling, {MAX_RANGE[0]}..{MAX_RANGE[1]} (default %(default)s)")
    ap.add_argument("--no-deadman-test", action="store_true", help="skip test 2")
    ap.add_argument("--sample", action="store_true", help="write `p` every 200 ms and tabulate the commanded joint's counts")
    ap.add_argument("--reverse", action="store_true", help="the reversal test (letter, opposite, letter, letter) instead of tests 1 to 3")
    ap.add_argument("--maxvel", type=int, default=None, metavar="PCT",
                    help="jogd-v: one jog, then `vPCT` (1..99) mid-jog; implies --sample (see the docstring)")
    ap.add_argument("--max-travel", type=float, default=None, help="mode 1 with --sample: degrees from the sequence's start at which `.` is written (default 12)")
    ap.add_argument("--mode", type=int, choices=(1, 3, 4), default=1,
                    help="jogd's first argument, the controller's manual mode: 1 joint (default), 3 world, 4 tool. 3 and 4 run one "
                         "start/hold/stop move with sampling always on and the --max-tcp/--max-joint limits")
    ap.add_argument("--sweep", action="store_true", help="modes 3/4, no letter: A..F each + then - (to come back), one table at the end")
    ap.add_argument("--vector", default=None, metavar="XYZ",
                    help="jogd-c, modes 3/4: instead of a letter, a straight move along (x, y, z): three signed digits like +1+1+0 "
                         "(each -9..+9); sends `@`+XYZ+`+0+0+0`+speed")
    ap.add_argument("--return", dest="ret", action="store_true",
                    help="with --vector: then the opposite vector for the same hold, so the arm comes back; reports how far from the start it ended")
    ap.add_argument("--letters", default=None, help="with --sweep: only these letters, e.g. ABC (default ABCDEF)")
    ap.add_argument("--max-tcp", type=float, default=None, metavar="MM",
                    help=f"modes 3/4: `.` when the model's tool point is this far from the first sample (default {DEFAULT_MAX_TCP:g})")
    ap.add_argument("--max-joint", type=float, default=None, metavar="DEG",
                    help=f"modes 3/4: `.` when any joint is this far from the first sample (default {DEFAULT_MAX_JOINT:g})")
    ap.add_argument("--after-abort", default=None, metavar="LETTER",
                    help="if the controller aborts the move (an `A` line): 1.5 s later run this letter (A..F, a..f) at the same speed for the same hold, "
                         "and report whether it was acknowledged and whether the arm moved (see the docstring)")
    ap.add_argument("--toward-limit", action="store_true",
                    help="modes 3/4, one letter: no fixed hold; keep the move going until an `A` line, a `-` by itself, an E line, --max-tcp / --max-joint "
                         "or --time-limit; it is meant to end in a controller abort")
    ap.add_argument("--time-limit", type=float, default=None, metavar="SECONDS",
                    help=f"with --toward-limit: stop after this long (default {TIME_LIMIT_DEFAULT_S:g}, at most {TIME_LIMIT_MAX_S:g})")
    ap.add_argument("--remote-name", "--remote", dest="remote", default="/tmp/jogd", help="jogd on the controller (default %(default)s)")
    ap.add_argument("--program", "--local", dest="local", default=os.path.join(repo, "controller", "programs", "jogd"),
                    help="the local copy it must equal (default %(default)s)")
    ap.add_argument("--keepalive", choices=("g", "repeat"), default=None,
                    help="the keep-alive after the start command: `g` (jogd-c: one byte) or `repeat` (the command again; plain jogd). "
                         "Default g with --mode 3/4 or a jogd-c program, else repeat")
    ap.add_argument("--port", default=os.environ.get("A465_PORT", "/dev/ttyUSB0"))
    args = ap.parse_args(argv)
    six = args.mode in (3, 4)
    if args.keepalive is None:
        is_c = os.path.basename(args.local).startswith("jogd-c") or os.path.basename(args.remote).startswith("jogd-c")
        args.keepalive = "g" if six or is_c else "repeat"
    if args.vector is not None:
        if not six:
            ap.error("--vector needs --mode 3 or --mode 4 (it cannot be combined with mode 1)")
        if args.sweep or args.letter is not None or args.dry or args.letters is not None:
            ap.error("--vector cannot be combined with a letter, --sweep, --letters or --dry")
        if not VECTOR_RE.fullmatch(args.vector):
            ap.error("--vector must be three signed digits, e.g. +1+1+0 (X, Y, Z; each -9..+9)")
        if not any(int(x) for x in re.findall(r"[+-]\d", args.vector)):
            ap.error("--vector must not be all zeros")
    hold_given = args.hold is not None
    if args.toward_limit:
        if not six or args.letter is None:
            ap.error("--toward-limit needs --mode 3 or --mode 4 and one letter (A..F or a..f)")
        if args.vector is not None or args.sweep or args.dry:
            ap.error("--toward-limit cannot be combined with --vector, --sweep or --dry")
        if hold_given and args.after_abort is None:
            ap.error("--toward-limit has no fixed hold: --hold is only the hold of the --after-abort move")
        args.time_limit = TIME_LIMIT_DEFAULT_S if args.time_limit is None else args.time_limit
        if not 0 < args.time_limit <= TIME_LIMIT_MAX_S:
            ap.error(f"--time-limit must be above 0 and at most {TIME_LIMIT_MAX_S:g} seconds")
    elif args.time_limit is not None:
        ap.error("--time-limit goes with --toward-limit")
    args.hold = args.hold if hold_given else (AFTER_ABORT_HOLD_S if args.toward_limit else 1.0)
    if args.after_abort is not None:
        if not re.fullmatch(r"[A-Fa-f]", args.after_abort):
            ap.error(f"--after-abort must be one letter A..F or a..f, not {args.after_abort!r}")
        if args.sweep or args.dry or args.reverse or args.maxvel is not None or args.speed2 is not None:
            ap.error("--after-abort goes with one letter (or --vector) run, not --sweep, --dry, --reverse, --maxvel or --speed2")
        if args.letter is None and args.vector is None:
            ap.error("--after-abort needs a letter or --vector")
        if not six and args.letter is not None and args.after_abort.upper() != args.letter.upper():
            ap.error("--after-abort in mode 1 must be the same joint as the letter (the travel limit follows one joint)")
        args.sample = True
    if args.ret and args.vector is None:
        ap.error("--return goes with --vector")
    if args.sweep:
        if not six:
            ap.error("--sweep needs --mode 3 or --mode 4")
        if args.dry or args.letter is not None:
            ap.error("--sweep takes no letter and no --dry")
        args.letters = (args.letters or "ABCDEF").upper()
        if not re.fullmatch(r"[A-F]+", args.letters) or len(set(args.letters)) != len(args.letters):
            ap.error("--letters must be distinct letters from A..F, e.g. ABC")
    elif args.letters is not None:
        ap.error("--letters goes with --sweep")
    elif args.vector is None and args.dry == (args.letter is not None):
        ap.error("give either --dry or a joint letter (A..F or a..f)")
    if six:
        bad = [name for name, on in (("--reverse", args.reverse), ("--maxvel", args.maxvel is not None), ("--speed2", args.speed2 is not None),
                                     ("--no-deadman-test", args.no_deadman_test), ("--max-travel", args.max_travel is not None)) if on]
        if bad:
            ap.error(f"{', '.join(bad)} cannot be used with --mode {args.mode}: modes 3/4 run only the start/hold/stop move "
                     "(limits are --max-tcp and --max-joint)")
        args.sample = True
        args.max_tcp = DEFAULT_MAX_TCP if args.max_tcp is None else args.max_tcp
        args.max_joint = DEFAULT_MAX_JOINT if args.max_joint is None else args.max_joint
        if not MAX_TCP_RANGE[0] < args.max_tcp <= MAX_TCP_RANGE[1]:
            ap.error(f"--max-tcp must be above 0 and at most {MAX_TCP_RANGE[1]:g} mm")
        if not MAX_JOINT_RANGE[0] < args.max_joint <= MAX_JOINT_RANGE[1]:
            ap.error(f"--max-joint must be above 0 and at most {MAX_JOINT_RANGE[1]:g} degrees")
    else:
        if args.max_tcp is not None or args.max_joint is not None:
            ap.error("--max-tcp and --max-joint are for --mode 3 or 4")
        if args.max_travel is None:
            args.max_travel = 12.0
    if args.letter is not None and not re.fullmatch(r"[A-Fa-f]", args.letter):
        ap.error(f"the letter must be A..F or a..f, not {args.letter!r}")
    if args.maxvel is not None:
        if args.dry or args.reverse or args.speed2 is not None or args.letter is None:
            ap.error("--maxvel takes a letter and --speed, not --dry, --reverse or --speed2")
        if not 1 <= args.maxvel <= 99:
            ap.error("--maxvel must be 1 to 99 (percent of the value read at start)")
        args.sample = True
        if args.remote == "/tmp/jogd":
            args.remote = "/tmp/jogd-v"
        if args.local == os.path.join(repo, "controller", "programs", "jogd"):
            args.local = os.path.join(repo, "controller", "programs", "jogd-v")
    if args.reverse and (args.dry or args.speed2 is not None):
        ap.error("--reverse takes a letter and --speed, not --dry or --speed2")
    if not six and not MAX_TRAVEL[0] < args.max_travel <= MAX_TRAVEL[1]:
        ap.error(f"--max-travel must be above 0 and at most {MAX_TRAVEL[1]:g} degrees")
    if not MAX_RANGE[0] <= args.max <= MAX_RANGE[1]:
        ap.error(f"--max must be {MAX_RANGE[0]} to {MAX_RANGE[1]}")
    if not 1 <= args.speed <= args.max:
        ap.error(f"--speed must be 1 to --max ({args.max})")
    if args.speed2 is not None and (not 1 <= args.speed2 <= args.max or args.speed2 == args.speed):
        ap.error(f"--speed2 must be 1 to --max ({args.max}) and different from --speed")
    hold_range = SIX_HOLD_RANGE if six else HOLD_RANGE
    if not hold_range[0] <= args.hold <= hold_range[1]:
        ap.error(f"--hold must be {hold_range[0]} to {hold_range[1]} seconds")
    if re.search(r"\s", args.remote):
        ap.error("--remote must not contain spaces")
    if not os.path.isfile(args.local):
        ap.error(f"{args.local} does not exist")
    return args


def main(argv=None, repo=None, open_port=None, clock=None, sleep=None, upload=None, log_dir=None):
    """Returns the exit code. `open_port(args)` and `upload(link, args, say)` are replaced in tests."""
    repo = repo or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    args = parse_args(argv, repo)                       # every limit is checked before the port is touched
    with home.session("jogtest", args, repo, open_port, clock, sleep, log_dir) as (link, say):
        if not aci.at_system_prompt(link):
            say("the controller is not at its `$` prompt")
            return 2
        do_upload = (lambda: upload(link, args, say)) if upload else (
            lambda: aci.send_file(link, args.local, args.remote, report=say))
        try:
            homezc.ensure_program(link, args.remote, args.local, do_upload, say)
        except (homezc.HomeError, aci.TransferError) as err:
            say(f"STOPPED: {err}")
            return 2
        before = read_position(link)
        if before[0] is None:
            say("could not read the joints and counts. Nothing was started.")
            return 2
        say("before: joints " + " ".join(f"{j:.3f}" for j in before[0]) + (" counts " + " ".join(f"{c:+d}" for c in before[1]) if before[1] else " (counts not read)"))
        jt = JogTest(link, say, args)
        try:
            try:
                if jt.start():
                    if args.sample:
                        jt.probe_sampling()
                    if args.dry:
                        pass
                    else:
                        if jt.six:
                            if jt.sampling_dropped:
                                raise NoAnswer("`p` was not answered: modes 3/4 never move without the limits being checked")
                            what = (f"VECTOR {args.vector}" if args.vector else f"LETTER{'S ' + args.letters if args.sweep else ' ' + args.letter}")
                            say(f"MODE {args.mode} {what} ABOUT TO MOVE THE ARM (axes unknown). Keep a hand on the E-stop.")
                            jt.run_six()
                        else:
                            say(f"JOINT {args.letter.upper()} ({'+' if args.letter.isupper() else '-'}) IS ABOUT TO MOVE. Keep a hand on the E-stop.")
                            jt.run_maxvel() if args.maxvel is not None else jt.run_reverse() if args.reverse else jt.run_tests()
                    jt.quit_clean()
                else:
                    jt.aborted = "jogd did not reach `jogd ready`" if not jt.rd.of_kind("E") else "E line before `jogd ready`"
                    jt.finish(emergency=False)
            except Abort as err:
                jt.aborted = f"{type(err).__name__}: {err}"
                say("STOPPING: " + jt.aborted)
                if jt.toward:
                    say("TOWARD-LIMIT ENDED BY: " + ("an `E` line" if isinstance(err, EError) else "a tool limit (--max-tcp / --max-joint)"
                                                    if isinstance(err, LimitTrip) else f"a failure ({type(err).__name__})"))
                emergency = isinstance(err, NotStopped)
                jt.finish(emergency=emergency)
        except KeyboardInterrupt:
            jt.interrupted = True
            jt.aborted = "interrupted by the operator"
            say("INTERRUPTED: Ctrl-Z, `.`, `q`")
            try:
                jt.finish(emergency=True)
            except KeyboardInterrupt:
                link.port.write(CTRL_Z)
        except Exception as err:        # noqa: BLE001 -- the arm must be stopped whatever happened
            jt.aborted = f"{type(err).__name__}: {err}"
            say("UNEXPECTED ERROR: " + jt.aborted)
            jt.finish(emergency=True)
        if jt.tests and jt.tests[-1].t_end is None:
            jt.tests[-1].t_end = link.clock()
        after = (None, None)
        if jt.prompt_back:
            after = read_position(link)
            if after[0] is not None:
                say("after: joints " + " ".join(f"{j:.3f}" for j in after[0]) + (" counts " + " ".join(f"{c:+d}" for c in after[1]) if after[1] else " (counts not read)"))
        say(jt.summary(before, after, args.dry))
        code = 0 if jt.exit_ok(before, after, args.dry) else 1
        if args.dry and code == 0:
            say("DRY RUN CLEAN: jogd started, printed `jogd ready`, quit on `q`; nothing changed.")
        return code
