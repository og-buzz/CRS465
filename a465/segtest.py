"""Bench run of `segtest` (a465/segprog.py) on the real controller: do back-to-back moves run through, and
does a speed change between queued moves take effect. Stand at the E-stop.

    scripts/a465-segtest <kind> <axis> <step> <count> <speed1> <speed2> [--cruise DEG_PER_S] [--max-travel 25] [--max-other 1] [--time-limit 12]
    scripts/a465-segtest j 6 2 6 5 15

kind j (joint, step degrees), m (motor, step pulses), l (move to locations), c (one continuous path).

What it does: makes sure controller/programs/segtest is /tmp/segtest on the controller (cksum, upload
if not), reads the joints (`w2`), starts the program, answers nothing (the program reads no input),
collects its lines with arrival times, and stops it with Ctrl-Z at once if the commanded axis goes
more than --max-travel degrees from where its first `P` line put it, if any OTHER joint goes more than
--max-other degrees (default 1) from its own first `P` line (a move to a location carries every joint),
or if --time-limit seconds pass
(the PAUSE/CONTINUE request, if the controller asks for it, is waited for with a loud WAITING line and
does not count). At the end the `$` prompt must come back, else Ctrl-Z and then Ctrl-E as the other
tools do. Then `w2` again, and the report:
  * the M table: every library call the program made, when, and how long it blocked;
  * the commanded axis from the P lines (controller ms, degrees from the first sample, deg/s);
  * the total time and mean speed from first movement to the end of movement; for a one-move run
    also the cruise speed (median of the per-sample speeds without the first and last two samples);
  * (1) each join between consecutive moves: the speed through it against the speed just before;
    below 20 percent is called a stop at the join, else ran through. Only judged from the P samples
    when two or more lie inside the move before it and two inside the move after (kind c); for j, l, m
    (one P line per move) the join is reported as not judgeable with each move's mean speed, and with
    `--cruise DEG_PER_S` (from a one-move run) each move's mean is compared with it: 85 percent or more
    of cruise = ran through, else slowed or stopped;
  * (2) the mean speed while the moves issued at speed1 ran and while those at speed2 ran (for c,
    the first and second half of the path; the second speed was set after cpath returned);
  * the largest movement of each of the other five joints during the run, always (zero included).
No pass/fail beyond that: exit 0 if the program printed `done` and no `E`, and nothing had to be stopped.

What is assumed (nothing else): the lines are found by pattern at the end of a line; the prompt is `$`
at the end of the text after the last line end; positions between P samples are linear; a join is where
the axis passes start + k x step. The P times are the controller's (the middle of each pos_get, about
95 ms long on this arm), so the speeds do not depend on the serial line.
"""
import argparse
import os
import re

from . import aci, calfile, console, home, homezc

TIME_LIMIT_S = 12.0
MAX_TRAVEL = 25.0
MAX_OTHER = 1.0                 # degrees any joint but the commanded one may move before Ctrl-Z
PROMPT_WAIT_S = 3.0
POLL_S = 0.002
STOP_RATIO = 0.20               # speed through a join below this fraction of the speed before: a stop
JOIN_WINDOW_MS = 100.0          # the speed through a join: the slowest interval within this of it
MIN_SAMPLES = 2                 # P samples needed inside the move before a join and inside the move after it
CRUISE_RATIO = 0.85             # a move whose mean speed is at least this fraction of the cruise speed ran through
BEFORE_MS = 500.0               # the speed before a join: the fastest interval in the BEFORE_MS before
LIMITS = {"axis": (1, 6), "count": (1, 8), "speed": (1, 25)}
MAX_DEGREES, MAX_PULSES = 20, 3000
CTRL_Z, CTRL_E = console.CTRL_Z, console.CTRL_E
COUNTS_PER_DEGREE = dict(zip(range(1, 7), calfile.COUNTS_PER_DEGREE))

LINE_PATTERNS = (
    ("E", re.compile(r"E (-?\d+) (.*)$")),
    ("M", re.compile(r"M (\d+) (\w+) (-?\d+) (-?\d+) (-?\d+)$")),
    ("P", re.compile(r"P (-?\d+)((?: -?\d+){6})$")),
    ("I", re.compile(r"I ([jmlc]) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)$")),
    ("done", re.compile(r"(?<!\S)done$")),
)
MOVES = ("joint", "motor", "move", "cpath")


def classify(text):
    text = text.strip()
    for kind, pattern in LINE_PATTERNS:
        found = pattern.search(text)
        if found:
            return kind, found
    return "other", text


class Lines:
    """Everything the port gives from the start of the program: raw bytes and parsed lines."""

    def __init__(self, link, say):
        self.link, self.say = link, say
        self.raw, self.partial, self.lines = b"", b"", []      # (arrival time, kind, match or text)

    def pump(self):
        chunk = self.link.port.read(4096)
        if not chunk:
            return False
        t = self.link.clock()
        self.raw += chunk
        self.partial += chunk
        while b"\n" in self.partial:
            raw, self.partial = self.partial.split(b"\n", 1)
            kind, got = classify(raw.decode("latin-1").replace("\r", ""))
            if kind == "other" and not got:
                continue
            self.lines.append((t, kind, got))
            if kind not in ("P", "M"):
                self.say(f"  segtest line ({kind}): {got if kind == 'other' else got.group(0)}")
        return True

    def of(self, kind):
        return [g for _, k, g in self.lines if k == kind]

    def prompt(self, mark=0):
        tail = self.raw[mark:].rsplit(b"\n", 1)[-1]
        return b"\n" in self.raw[mark:] and tail.rstrip(b" ").endswith(b"$")


# ---------------------------------------------------------------------------------- the analysis

def m_rows(lines):
    return [(int(g.group(1)), g.group(2), int(g.group(3)), int(g.group(4)), int(g.group(5))) for g in lines.of("M")]


def all_degrees(lines):
    """(controller ms, [degrees of joints 1..6]) from the P lines."""
    return [(int(g.group(1)), [int(n) / COUNTS_PER_DEGREE[j] for j, n in zip(range(1, 7), g.group(2).split())])
            for g in lines.of("P")]


def other_excursions(lines, axis):
    """{joint: largest |degrees from its first P line|} for the five joints that are not `axis`."""
    rows = all_degrees(lines)
    out = {j: 0.0 for j in range(1, 7) if j != axis}
    for _, d in rows:
        for j in out:
            out[j] = max(out[j], abs(d[j - 1] - rows[0][1][j - 1]))
    return out


def p_rows(lines, axis):
    """(controller ms, degrees of the axis) from the P lines."""
    out = []
    for g in lines.of("P"):
        counts = [int(n) for n in g.group(2).split()]
        out.append((int(g.group(1)), counts[axis - 1] / COUNTS_PER_DEGREE[axis]))
    return out


def step_degrees(kind, axis, step):
    return step / COUNTS_PER_DEGREE[axis] if kind == "m" else float(step)


def crossing(rows, level, sign):
    """The first time (ms, interpolated) the position reaches `level` going in direction `sign`."""
    if rows and sign * (rows[0][1] - level) >= 0:
        return rows[0][0]
    for (t0, a), (t1, b) in zip(rows, rows[1:]):
        if sign * (a - level) < 0 <= sign * (b - level):
            return t0 + (t1 - t0) * (level - a) / (b - a) if b != a else t1
    return None


def speeds(rows):
    """(start ms, end ms, deg/s) for each pair of consecutive samples."""
    return [(t0, t1, (b - a) * 1000 / (t1 - t0)) for (t0, a), (t1, b) in zip(rows, rows[1:]) if t1 > t0]


def move_means(rows, p0, step_deg, count):
    """For each move k = 1..count: (from ms, to ms, mean deg/s, number of P samples inside it), None
    where the axis did not get that far. The first move starts when the axis has moved 2 percent of a
    step and the last ends 2 percent of a step short of its end (the same as mean_speeds)."""
    sign = 1 if step_deg > 0 else -1
    levels = [0.02] + list(range(1, count)) + [count - 0.02]
    times = [crossing(rows, p0 + x * step_deg, sign) for x in levels]
    out = []
    for k in range(count):
        a, b = times[k], times[k + 1]
        if a is None or b is None or b <= a:
            out.append(None)
            continue
        inside = sum(1 for t, _ in rows if a <= t < b)
        out.append((a, b, abs((levels[k + 1] - levels[k]) * step_deg) * 1000 / (b - a), inside))
    return out


def joins(rows, p0, step_deg, count):
    """For k = 1..count-1: (k, level, time, speed through, speed before, verdict). A join is judged
    only with at least two P samples inside the move before it and two inside the move after it;
    otherwise the verdict is "cannot be judged"."""
    sign = 1 if step_deg > 0 else -1
    v = speeds(rows)
    mv = move_means(rows, p0, step_deg, count)
    out = []
    for k in range(1, count):
        level = p0 + k * step_deg
        t = crossing(rows, level, sign)
        if t is None:
            out.append((k, level, None, None, None, "not reached"))
            continue
        if not (mv[k - 1] and mv[k] and mv[k - 1][3] >= MIN_SAMPLES and mv[k][3] >= MIN_SAMPLES):
            out.append((k, level, t, None, None, "cannot be judged"))
            continue
        through = [abs(s) for a, b, s in v if a <= t + JOIN_WINDOW_MS and b >= t - JOIN_WINDOW_MS]
        before = [abs(s) for a, b, s in v if b <= t - JOIN_WINDOW_MS and b >= t - JOIN_WINDOW_MS - BEFORE_MS]
        if not before:
            before = [abs(s) for a, b, s in v if b <= t][-1:]
        if not through or not before:
            out.append((k, level, t, None, None, "too few samples"))
            continue
        vt, vb = min(through), max(before)
        verdict = "STOP at the join" if vt < STOP_RATIO * vb else "ran through"
        out.append((k, level, t, vt, vb, verdict))
    return out


def mean_speeds(rows, p0, step_deg, count, half):
    """((from ms, to ms, deg/s) for the first `half` steps, the same for the rest), None where unknown."""
    sign = 1 if step_deg > 0 else -1
    start = crossing(rows, p0 + 0.02 * step_deg, sign)
    mid = crossing(rows, p0 + half * step_deg, sign)
    end = crossing(rows, p0 + (count - 0.02) * step_deg, sign)

    def mean(a, b, steps):
        if a is None or b is None or b <= a:
            return None
        return (a, b, abs(steps * step_deg) * 1000 / (b - a))
    first = mean(start, mid, half - 0.02)
    second = mean(mid, end, count - half - 0.02) if count > half else None
    return first, second


def report(lines, kind, axis, step, count, speed1, speed2, cruise=None):
    out = ["", "SUMMARY"]
    ms = m_rows(lines)
    others = other_excursions(lines, axis)
    out.append("largest movement of the other joints during the run (degrees from their first P line): "
               + ", ".join(f"joint {j} {d:.3f}" for j, d in others.items())
               + ("" if lines.of("P") else " (no P lines)"))
    out.append("library calls (controller ms since the program started):")
    out.append("    index  call              at ms   returned   blocked ms   result")
    for i, name, a, b, r in ms:
        out.append(f"    {i:5d}  {name:15s} {a:8d} {b:10d} {b - a:12d} {r:8d}")
    rows = p_rows(lines, axis)
    if len(rows) < 2:
        out.append(f"positions of joint {axis}: fewer than two P lines, no speeds")
        return out
    p0 = rows[0][1]
    out.append(f"joint {axis} from the P lines (degrees from the first sample, {rows[0][1]:.3f}):")
    out.append("        ms     degrees      deg/s since previous")
    for n, (t, d) in enumerate(rows):
        v = (d - rows[n - 1][1]) * 1000 / (t - rows[n - 1][0]) if n and t > rows[n - 1][0] else None
        out.append(f"    {t:6d} {d - p0:+11.3f}      " + (f"{v:+9.2f}" if v is not None else "        -"))
    sd = step_degrees(kind, axis, step)
    half = (count + 1) // 2
    sign = 1 if sd > 0 else -1
    t_a, t_b = crossing(rows, p0 + 0.02 * sd, sign), crossing(rows, p0 + (count - 0.02) * sd, sign)
    total_mean = None
    if t_a is not None and t_b is not None and t_b > t_a:
        total_mean = (count - 0.04) * abs(sd) * 1000 / (t_b - t_a)
        out.append(f"total: {(t_b - t_a) / 1000:.2f} s from first movement ({t_a:.0f} ms) to the end of movement ({t_b:.0f} ms), "
                   f"mean speed {total_mean:.2f} deg/s over {(count - 0.04) * abs(sd):.3f} degrees")
    else:
        out.append("total: the axis did not complete the movement, no total time")
    if count == 1:
        per = [abs((rows[n][1] - rows[n - 1][1]) * 1000 / (rows[n][0] - rows[n - 1][0]))
               for n in range(1, len(rows) - 2) if rows[n][0] > rows[n - 1][0]]
        if len(rows) >= 5 and per:
            per.sort()
            med = per[len(per) // 2] if len(per) % 2 else (per[len(per) // 2 - 1] + per[len(per) // 2]) / 2
            out.append(f"cruise speed: {med:.2f} deg/s (median of the speeds of {len(per)} samples, the first and the last two left out); "
                       f"paste into --cruise {med:.2f}")
        else:
            out.append("cruise speed: too few samples (need 5 P samples) to give one")
    out.append(f"(1) the joins (start + k x {sd:+.3f} degrees); judged from the P samples only when two or more lie inside the move before "
               f"and two inside the move after; a stop is a speed through it below {STOP_RATIO:.0%} of the speed before:")
    found = joins(rows, p0, sd, count)
    mv = move_means(rows, p0, sd, count)

    def mean_of(m):
        return f"{m[2]:.2f} deg/s over {m[1] - m[0]:.0f} ms ({m[3]} samples)" if m else "not measured"
    for k, level, t, vt, vb, verdict in found:
        if verdict == "cannot be judged":
            out.append(f"    join {k} ({level - p0:+.3f} degrees) at {t:.0f} ms: cannot be judged from the samples "
                       f"(fewer than {MIN_SAMPLES} inside the move before or after it); mean speed over the move before it "
                       f"{mean_of(mv[k - 1])}, over the move after it {mean_of(mv[k])}")
        elif vt is None:
            out.append(f"    join {k} ({level - p0:+.3f} degrees): {verdict}" + (f" at {t:.0f} ms" if t is not None else ""))
        else:
            out.append(f"    join {k} ({level - p0:+.3f} degrees) at {t:.0f} ms: {vt:.2f} deg/s through it, {vb:.2f} deg/s before "
                       f"({vt / vb if vb else 0:.0%}): {verdict}")
    if not found:
        out.append("    one move only: no join")
    stops = [x for x in found if x[5].startswith("STOP")]
    ran = [x for x in found if x[5] == "ran through"]
    unjudged = [x for x in found if x[5] == "cannot be judged"]
    if found:
        out.append(f"    answer (1): {len(stops)} of {len(found)} joins stopped, {len(ran)} ran through"
                   + (f", {len(unjudged)} cannot be judged from the samples" if unjudged else "")
                   + (", the rest could not be judged" if len(stops) + len(ran) + len(unjudged) < len(found) else ""))
    if unjudged:
        measured = [m for m in mv if m]
        if cruise is not None and measured:
            out.append(f"    each move's mean speed against the cruise speed {cruise:.2f} deg/s (ran through at {CRUISE_RATIO:.0%} or more):")
            for n, m in enumerate(mv, 1):
                if m:
                    out.append(f"        move {n}: {m[2]:.2f} deg/s ({m[2] / cruise:.0%} of cruise): "
                               + ("ran through" if m[2] >= CRUISE_RATIO * cruise else "slowed or stopped"))
            if all(m[2] >= CRUISE_RATIO * cruise for m in measured) and len(measured) == len(mv):
                out.append(f"    answer (1, from mean speeds): ran through ({total_mean:.2f} deg/s mean against cruise {cruise:.2f})"
                           if total_mean else "    answer (1, from mean speeds): ran through")
            else:
                out.append(f"    answer (1, from mean speeds): slowed or stopped at the joins (mean {total_mean or 0:.2f} deg/s against cruise {cruise:.2f})")
        else:
            out.append("    answer (1): the joins cannot be told from this run, one P sample per move is too few. Run a single move "
                       "of the same total distance at the same speed setting and give its cruise speed with --cruise:")
            out.append(f"        scripts/a465-segtest {kind} {axis} {step * count} 1 {speed1} {speed1}")
    first, second = mean_speeds(rows, p0, sd, count, half)
    label1 = f"moves 1..{half} (issued at {speed1}%)" if kind != "c" else f"the first {half} steps of the path (at {speed1}%)"
    label2 = f"moves {half + 1}..{count} (issued at {speed2}%)" if kind != "c" else \
        f"the rest of the path (speed_set({speed2}) was called after cpath returned)"
    out.append("(2) mean speeds:")
    for n, (label, m) in enumerate(((label1, first), (label2, second))):
        if n and count == half:
            out.append(f"    {label}: none (one move)")
        elif m is None:
            out.append(f"    {label}: not measured (the axis did not get that far, or too few samples)")
        else:
            out.append(f"    {label}: {m[2]:.2f} deg/s ({m[0]:.0f} to {m[1]:.0f} ms)")
    if first and second:
        out.append(f"    answer (2): the second speed is {second[2] / first[2]:.2f} x the first; the speed settings were {speed2} / {speed1} = {speed2 / speed1:.2f}")
    moves = [(i, a, b) for i, name, a, b, r in ms if name in MOVES]
    if moves:
        out.append("move calls blocked: " + ", ".join(f"{b - a} ms" for _, a, b in moves))
    return out


# ---------------------------------------------------------------------------------- the run

class Run:
    def __init__(self, link, say, args):
        self.link, self.say, self.args = link, say, args
        self.lines = Lines(link, say)
        self.started = self.ctrlz = self.ctrle = False
        self.prompt_back = None
        self.stopped = None              # why Ctrl-Z was written by this tool
        self.button = False
        self.origin = None

    def now(self):
        return self.link.clock()

    def write(self, data):
        self.link.port.write(data)
        if hasattr(self.link.port, "flush"):
            self.link.port.flush()

    def halt(self, why):
        self.write(CTRL_Z)
        self.ctrlz = True
        self.stopped = why
        self.say(f"HALT: {why}. Ctrl-Z WRITTEN AT ONCE (the controller stops the arm and the program).")

    def travel(self):
        rows = p_rows(self.lines, self.args.axis)
        if not rows:
            return False
        for j, away in other_excursions(self.lines, self.args.axis).items():
            if away > self.args.max_other:
                self.halt(f"OTHER JOINT MOVED: joint {j} (not the commanded joint {self.args.axis}) is {away:.2f} degrees "
                          f"from its first sample (limit {self.args.max_other:g})")
                return True
        if self.origin is None:
            self.origin = rows[0][1]
            self.say(f"joint {self.args.axis} starts at {self.origin:.3f} degrees; travel limit {self.args.max_travel:g}")
        away = rows[-1][1] - self.origin
        if abs(away) > self.args.max_travel:
            self.halt(f"TRAVEL LIMIT: joint {self.args.axis} is {away:+.2f} degrees from its first sample (limit {self.args.max_travel:g})")
            return True
        return False

    def run(self):
        a = self.args
        command = f"{a.remote} {a.kind} {a.axis} {a.step} {a.count} {a.speed1} {a.speed2}"
        self.link.drain(quiet=0.1, limit=1.0)
        self.say(f"JOINT {a.axis} IS ABOUT TO MOVE ({a.count} x {a.step} {'pulses' if a.kind == 'm' else 'degrees'}). "
                 "Keep a hand on the E-stop.")
        self.say(f"starting: {command}")
        self.write(command.encode() + b"\r")
        self.started = True
        end = self.now() + a.time_limit
        while True:
            if self.lines.pump() and self.travel():
                break
            if homezc.BUTTON.encode() in self.lines.raw and not self.button:
                self.button = True
                end = self.now() + homezc.BUTTON_WAIT_S + a.time_limit
                self.say("WAITING: PRESS THE PAUSE/CONTINUE BUTTON ON THE CONTROLLER'S FRONT PANEL NOW "
                         f"(up to {homezc.BUTTON_WAIT_S:.0f} s). The run starts when it is pressed.")
            if self.lines.prompt():
                self.prompt_back = True
                return
            if self.lines.of("done") or self.lines.of("E"):
                if self.wait_prompt(PROMPT_WAIT_S):
                    return
                self.say(f"NO `$` PROMPT within {PROMPT_WAIT_S:.0f} s of the program's last line")
                self.halt("no prompt after the program's end")
                break
            if self.now() >= end:
                self.halt(f"TIME LIMIT: {a.time_limit:g} s since the program was started")
                break
            self.link.sleep(POLL_S)
        self.recover()

    def wait_prompt(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            self.lines.pump()
            if self.lines.prompt():
                self.prompt_back = True
                return True
            self.link.sleep(POLL_S)
        return False

    def recover(self):
        """After Ctrl-Z: the prompt, else Ctrl-E (homezc.recover)."""
        def spy(text):
            if "Ctrl-E" in text:
                self.ctrle = True
            self.say(text)
        self.lines.pump()
        self.link.ahead = b""
        self.prompt_back = homezc.recover(self.link, spy)
        self.say("the controller is back at its `$` prompt" if self.prompt_back else "THE CONTROLLER IS NOT AT ITS `$` PROMPT")

    def ok(self):
        return bool(self.lines.of("done")) and not self.lines.of("E") and not self.stopped and bool(self.prompt_back)


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


def parse_args(argv, repo):
    ap = argparse.ArgumentParser(prog="a465-segtest", description=__doc__.split("\n")[0])
    ap.add_argument("kind", choices=("j", "m", "l", "c"))
    ap.add_argument("axis", type=int)
    ap.add_argument("step", type=int, help="degrees (j, l, c) or pulses (m), not 0")
    ap.add_argument("count", type=int)
    ap.add_argument("speed1", type=int)
    ap.add_argument("speed2", type=int)
    ap.add_argument("--max-travel", type=float, default=MAX_TRAVEL, help="degrees from the first P sample (default %(default)s)")
    ap.add_argument("--max-other", type=float, default=MAX_OTHER,
                    help="degrees any other joint may move from its first P sample (default %(default)s)")
    ap.add_argument("--cruise", type=float, default=None, metavar="DEG_PER_S",
                    help="cruise speed measured with one long move at the same speed setting (a one-move run prints it)")
    ap.add_argument("--time-limit", type=float, default=TIME_LIMIT_S, help="seconds (default %(default)s)")
    ap.add_argument("--remote", default="/tmp/segtest")
    ap.add_argument("--local", default=os.path.join(repo, "controller", "programs", "segtest"))
    ap.add_argument("--port", default=os.environ.get("A465_PORT", "/dev/ttyUSB0"))
    a = ap.parse_args(argv)
    for name, value, (lo, hi) in (("axis", a.axis, LIMITS["axis"]), ("count", a.count, LIMITS["count"]),
                                  ("speed1", a.speed1, LIMITS["speed"]), ("speed2", a.speed2, LIMITS["speed"])):
        if not lo <= value <= hi:
            ap.error(f"{name} must be {lo} to {hi}")
    limit = MAX_PULSES if a.kind == "m" else MAX_DEGREES
    if a.step == 0 or abs(a.step) * a.count > limit:
        ap.error(f"step must not be 0 and |step| x count at most {limit} {'pulses' if a.kind == 'm' else 'degrees'}")
    if not 0 < a.max_travel <= 30 or not 0 < a.time_limit <= 30:
        ap.error("--max-travel and --time-limit must be above 0 and at most 30")
    if not 0 < a.max_other <= 5:
        ap.error("--max-other must be above 0 and at most 5 degrees")
    if re.search(r"\s", a.remote):
        ap.error("--remote must not contain spaces")
    if not os.path.isfile(a.local):
        ap.error(f"{a.local} does not exist")
    return a


def main(argv=None, repo=None, open_port=None, clock=None, sleep=None, upload=None, log_dir=None):
    """Returns the exit code. `open_port(args)` and `upload(link, args, say)` are replaced in tests."""
    repo = repo or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    args = parse_args(argv, repo)                       # every limit is checked before the port is touched
    with home.session("segtest", args, repo, open_port, clock, sleep, log_dir) as (link, say):
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
            say("could not read the joints. Nothing was started.")
            return 2
        say("before: joints " + " ".join(f"{j:.3f}" for j in before[0]))
        run = Run(link, say, args)
        try:
            run.run()
        except KeyboardInterrupt:
            run.halt("interrupted by the operator")
            run.recover()
        except Exception as err:        # noqa: BLE001 -- the arm must be stopped whatever happened
            run.halt(f"UNEXPECTED ERROR {type(err).__name__}: {err}")
            run.recover()
        after = read_position(link) if run.prompt_back else (None, None)
        if after[0] is not None:
            say("after:  joints " + " ".join(f"{j:.3f}" for j in after[0]))
            say(f"joint {args.axis} moved {after[0][args.axis - 1] - before[0][args.axis - 1]:+.3f} degrees by `w2`")
        for line in report(run.lines, args.kind, args.axis, args.step, args.count, args.speed1, args.speed2, args.cruise):
            say(line)
        errs = run.lines.of("E")
        say("E lines: " + (", ".join(g.group(0) for g in errs) if errs else "none"))
        say(f"`done` seen: {'yes' if run.lines.of('done') else 'NO'}; prompt back: {'yes' if run.prompt_back else 'NO'}; "
            f"Ctrl-Z sent: {'YES (' + run.stopped + ')' if run.ctrlz else 'no'}; Ctrl-E sent: {'YES' if run.ctrle else 'no'}")
        return 0 if run.ok() else 1
