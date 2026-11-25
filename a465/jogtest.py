"""Bench test of `jogd` (a465/jogprog.py) on the real controller: its first runs.

    scripts/a465-jogtest --dry [--variant s|r] [--max 5]
    scripts/a465-jogtest <A..F|a..f> [--speed 2] [--hold 1.0] [--speed2 N] [--variant s|r] [--max 5] [--no-deadman-test]

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
  * a dry run counts as failed if any encoder count differs afterwards (exit code 1).
"""
import argparse
import os
import re

from . import aci, console, home, homezc

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
CTRL_Z, CTRL_E = console.CTRL_Z, console.CTRL_E
COUNTS_PER_DEGREE = home.COUNTS_PER_DEGREE

LINE_PATTERNS = (                # searched at the end of a line, in this order
    ("E", re.compile(r"E (-?\d+) (.*)$")),
    ("P", re.compile(r"P(?: -?\d+){6}$")),
    ("ready", re.compile(r"jogd ready$")),
    ("+", re.compile(r"\+([A-Fa-f]\d\d)$")),
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

    @property
    def got(self):
        return [t for _, k, t in self.lines if k not in ("other", "P")]


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
        self.joint = ord(args.letter.upper()) - 64 if args.letter else None
        self.sample_on = False
        self.samples = []            # (arrival time, [six counts])
        self.origin = None           # the commanded joint's first sampled position, degrees
        self.next_p = 0.0
        self._li = 0
        self.sampling_dropped = False

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
            if now >= self.next_p and (sample_before is None or now < sample_before - SAMPLE_GUARD_S):
                self.write(b"p")
                self.next_p = now + SAMPLE_S if now - self.next_p > SAMPLE_S else self.next_p + SAMPLE_S
        return got

    def degrees(self, counts):
        return counts[self.joint - 1] / COUNTS_PER_DEGREE[self.joint]

    def take_samples(self):
        new = [(t, [int(n) for n in text.split()[1:]]) for t, k, text in self.rd.lines[self._li:] if k == "P"]
        self._li = len(self.rd.lines)
        for t, counts in new:
            self.samples.append((t, counts))
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
        if self.joint:
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
        command = f"{args.remote} 1 {args.max} {args.variant}"
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
                t = self.write(cmd)
                test.sent.append((t, cmd.decode() + (" (first)" if first is None else " (keep-alive)")))
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
                answered = any(k in ("+", "E") for _, k, _ in self.rd.lines[mark:])
                elapsed = self.now() - first
                if not answered and elapsed >= ANSWER_WAIT_S:
                    raise NoAnswer(f"no `+` or `E` line within {ANSWER_WAIT_S} s of {cmd.decode()}")
                if answered and elapsed >= seconds:
                    break
        return last

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
        if not self.wait(STOP_WAIT_S, until=lambda: self.rd.of_kind("-", mark)):
            raise NotStopped(f"no `-` within {STOP_WAIT_S} s of `.` ({why})")

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
        test.ok = test.got == test.expected and not test.notes

    def pause(self):
        mark = len(self.rd.lines)
        self.wait(PAUSE_S)
        self.between += [ln for ln in self.rd.lines[mark:] if ln[1] != "P"]

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
            self.stop_move(t1, "test 1")
        finally:
            self.end(t1)
        self.pause()
        # test 2
        if not a.no_deadman_test:
            t2 = self.begin("test 2: dead-man", [f"+{opposite}{speed:02d}", "T"])
            try:
                last = self.hold(t2, [(opposite, speed, a.hold)])
                mark = len(self.rd.lines)
                if self.wait(DEADMAN_WAIT_S, until=lambda: self.rd.of_kind("T", mark)):
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
            self.pause()
        # test 3
        if a.speed2:
            first, second = f"+{letter}{speed:02d}", f"+{letter}{a.speed2:02d}"
            expected = [first, "-", second, "-"] if a.variant == "s" else [first, second, "-"]
            t3 = self.begin("test 3: speed change", expected)
            try:
                self.hold(t3, [(letter, speed, a.hold), (letter, a.speed2, a.hold)])
                self.stop_move(t3, "test 3")
            finally:
                self.end(t3)
        self.end_sequence()

    def run_reverse(self):
        a = self.args
        first, opposite = a.letter, a.letter.swapcase()
        for i, (letter, tag) in enumerate(((first, "a"), (opposite, "b"), (first, "c"), (first, "d"))):
            t = self.begin(f"reverse ({tag}): {letter}{a.speed:02d}", [f"+{letter}{a.speed:02d}", "-"])
            t.move_report = True
            try:
                self.hold(t, [(letter, a.speed, a.hold)])
                self.stop_move(t, f"reverse ({tag})")
            finally:
                self.end(t)
            if i < 3:
                self.pause()
        self.end_sequence()

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
            if test.gaps:
                out.append(f"    keep-alive gaps: max {ms(max(test.gaps))}" + (f"; LATE (over {ms(LATE_S)}): {len(test.late)} time(s), worst {ms(max(test.late))}" if test.late else "; none late"))
            if getattr(test, "notes_info", None):
                out.append("    " + test.notes_info)
            out += [f"    {n}" for n in test.notes]
            out += self.sample_lines(test)
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
        if self.aborted:
            out.append(f"STOPPED EARLY: {self.aborted}")
        late = sum(len(t.late) for t in self.tests)
        if late:
            out.append(f"KEEP-ALIVES LATE: {late} gap(s) over {ms(LATE_S)} on this tool's clock")
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
            lines_ok = [t for _, k, t in self.rd.lines if k not in ("other", "P")] == ["jogd ready"]
            changed = not before[1] or not after[1] or before[1] != after[1]
            return lines_ok and not changed
        return all(t.ok for t in self.tests) and not self.between and bool(self.tests)


def parse_args(argv, repo):
    ap = argparse.ArgumentParser(prog="a465-jogtest", description="Bench test of the jogd program on the controller.")
    ap.add_argument("letter", nargs="?", help="A..F (joint 1..6, positive) or a..f (negative): the first move")
    ap.add_argument("--dry", action="store_true", help="start jogd, wait for `jogd ready`, send `q`; no motion command")
    ap.add_argument("--speed", type=int, default=2, help="percent, 1..--max (default %(default)s)")
    ap.add_argument("--hold", type=float, default=1.0, help=f"seconds of keep-alives per phase, {HOLD_RANGE[0]}..{HOLD_RANGE[1]} (default %(default)s)")
    ap.add_argument("--speed2", type=int, default=None, help="test 3: a second speed, different from --speed")
    ap.add_argument("--variant", choices=("s", "r"), default="s", help="jogd's speed-change variant (default %(default)s)")
    ap.add_argument("--max", type=int, default=5, help=f"jogd's speed ceiling, {MAX_RANGE[0]}..{MAX_RANGE[1]} (default %(default)s)")
    ap.add_argument("--no-deadman-test", action="store_true", help="skip test 2")
    ap.add_argument("--sample", action="store_true", help="write `p` every 200 ms and tabulate the commanded joint's counts")
    ap.add_argument("--reverse", action="store_true", help="the reversal test (letter, opposite, letter, letter) instead of tests 1 to 3")
    ap.add_argument("--max-travel", type=float, default=12.0, help="with --sample: degrees from the sequence's start at which `.` is written (default %(default)s)")
    ap.add_argument("--remote-name", "--remote", dest="remote", default="/tmp/jogd", help="jogd on the controller (default %(default)s)")
    ap.add_argument("--program", "--local", dest="local", default=os.path.join(repo, "controller", "programs", "jogd"),
                    help="the local copy it must equal (default %(default)s)")
    ap.add_argument("--port", default=os.environ.get("A465_PORT", "/dev/ttyUSB0"))
    args = ap.parse_args(argv)
    if args.dry == (args.letter is not None):
        ap.error("give either --dry or a joint letter (A..F or a..f)")
    if args.letter is not None and not re.fullmatch(r"[A-Fa-f]", args.letter):
        ap.error(f"the letter must be A..F or a..f, not {args.letter!r}")
    if args.reverse and (args.dry or args.speed2 is not None):
        ap.error("--reverse takes a letter and --speed, not --dry or --speed2")
    if not MAX_TRAVEL[0] < args.max_travel <= MAX_TRAVEL[1]:
        ap.error(f"--max-travel must be above 0 and at most {MAX_TRAVEL[1]:g} degrees")
    if not MAX_RANGE[0] <= args.max <= MAX_RANGE[1]:
        ap.error(f"--max must be {MAX_RANGE[0]} to {MAX_RANGE[1]}")
    if not 1 <= args.speed <= args.max:
        ap.error(f"--speed must be 1 to --max ({args.max})")
    if args.speed2 is not None and (not 1 <= args.speed2 <= args.max or args.speed2 == args.speed):
        ap.error(f"--speed2 must be 1 to --max ({args.max}) and different from --speed")
    if not HOLD_RANGE[0] <= args.hold <= HOLD_RANGE[1]:
        ap.error(f"--hold must be {HOLD_RANGE[0]} to {HOLD_RANGE[1]} seconds")
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
                        say(f"JOINT {args.letter.upper()} ({'+' if args.letter.isupper() else '-'}) IS ABOUT TO MOVE. Keep a hand on the E-stop.")
                        jt.run_reverse() if args.reverse else jt.run_tests()
                    jt.quit_clean()
                else:
                    jt.aborted = "jogd did not reach `jogd ready`" if not jt.rd.of_kind("E") else "E line before `jogd ready`"
                    jt.finish(emergency=False)
            except Abort as err:
                jt.aborted = f"{type(err).__name__}: {err}"
                say("STOPPING: " + jt.aborted)
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
