"""The real C500 controller, driven through its serial console: joint jogging, stop, position and status readback.

What this relies on was measured on the user's controller (CROS 1.16.471, 38400 baud) on 2026-11-23:

  * `ash <application>` gives a shell whose commands answer in about 0.25 s. With `online 1` a
    `joint <axis>,<degrees>` returns at once and the move runs by itself.
  * **Ctrl-Z stops a move in progress** ("^C equivalent - halting robot..."), in both shells.
    Ctrl-C does not, and this version of ash has no `halt` command.
  * `axst &` keeps printing one status word per axis about five times a second while the shell
    stays usable. Bit 0 of a word is that axis's home switch (RAPL-3 `axis_status`). But while it
    runs, every command takes about 1.5 s longer to answer (measured), so it is off unless
    `stream=True`: with it the home switches are watched during a jog, without it jogging responds
    in a fraction of a second and only the position is read after each stop.

A jog is therefore one long relative move (eight seconds of travel, or as far as the soft limit),
followed by another if the stick is still held, and ended with Ctrl-Z the moment it is released,
changed or goes stale. The shell does not answer a second move until the first has finished, so the
next one is typed just before the end; the arm slows to a stop at each join, which is why the moves
are long. If this program dies the arm finishes at most the move already sent (up to eight seconds
of travel, and never past a soft limit when the zero pose is known).

After a Ctrl-Z the controller refuses moves for a moment ("device is being reset"). The next jog is
offered again and again until it is taken, and the position is read back only once the pad is idle,
so the wait between two jogs is the controller's own and nothing is added to it here.

Fine steps. A jog that is stopped with Ctrl-Z costs the controller's two-second lock-out, which makes
small corrections slow. So with L2 pulled at least half way the jog is a run of small relative moves
instead (`joint <axis>,0.5`), one at a time: the next is sent only when the reply to the last has come
back and its travel is over, and not sooner than FINE_GAP_S after it, so a held stick ticks at a steady
rate. A move that runs to its end needs no Ctrl-Z, so letting go of the stick, changing joint or going
stale sends nothing at all and there is no lock-out; at most the one step in flight finishes. Only
`halt()` writes Ctrl-Z. Soft limits cut a step short exactly as they do a continuous move.

The arm is not homed, so there are no software limits: nothing here stops a joint before its hard stop.
"""
import datetime
import os
import re
import threading
import time

import serial

from . import model
from .sim import RobotError

STATUS = re.compile(rb"((?:[0-9a-fA-F]{8} +){7}[0-9a-fA-F]{8})\r?\n")
PROBLEM = re.compile(r"ash:|rror|ail|OFF|busy|reset|not |denied|limit", re.I)
PARTIAL = re.compile(rb"(?:[0-9a-fA-F]{8} +){0,7}[0-9a-fA-F]{1,8} *")
NUMBER = re.compile(r"[-+]?\d+(?:\.\d+)?")
HALT = b"\x1a"                     # Ctrl-Z
CHUNK_S = 8.0                      # travel time of one jog move
TOPUP_S = 0.6                      # send the next move when less than this is left
TOPUP_WAIT_S = 6.0                 # how long the shell may sit on that next move before it counts as no answer
FINE_STEP_DEG = 0.5                # one fine step (L2 held)
FINE_GAP_S = 0.25                  # least time from one fine step being sent to the next
FINE_TRIGGER = -0.5                # aux[0] (R2 minus L2) at or below this means fine steps
BUSY_S = 4.0                       # how long to keep offering a move the controller refuses because it is resetting after a stop
MAX_SPEED_PCT = 25                 # ceiling while the arm is unhomed
STICK_ON = 0.2                     # shaped stick value that starts a jog (about half deflection)
STICK_OFF = 0.08                   # and the value the moving joint's stick must drop below to end it
SETTLE_S = 0.08                    # a joint must be the one asked for this long before it starts
LIMIT_MARGIN_DEG = 3.0             # with true angles known, a jog ends this far inside each joint's range
STATUS_BITS = {0: "home switch", 1: "+limit", 2: "-limit", 3: "limp", 4: "limp (collision)", 5: "armed for zero-cross",
               6: "zero-cross seen", 7: "locked", 8: "error", 9: "servo fault", 10: "motor fault", 11: "homed",
               12: "calibrated", 13: "begin motion", 14: "feedback check", 15: "done"}
QUIET_BITS = (1 << 15) | (1 << 13)  # change with every move; not worth an event


class RealArm:
    """Has the methods the gamepad driver calls on a robot. Everything except joint jog, stop and speed is refused."""

    backend = "c500"

    def __init__(self, port="/dev/ttyUSB0", baud=38400, application="test", log_dir=None, say=print, stream=False,
                 zero_at=None):
        self.say = say
        self.stream = stream                    # run the status display all the time (see the note at the top)
        # The controller's reading of each joint at the pose where every joint is truly at zero (the arm
        # straight up, each joint at the middle of its travel). With it, true angle = reading - zero_at,
        # and a jog is cut short of the joint's range. None = unknown: no limits at all.
        self.zero_at = [float(v) for v in zero_at] if zero_at is not None else None
        self._target = 0.0                      # where the moves sent for the current jog end (controller's numbers)
        self._limit_reached = False             # the current jog has been given its last move
        self._limit_said = None                 # the (joint, direction) last reported as at its limit
        self._fine = False                      # the current jog request is for fine steps
        self._targets = [0.0] * 6               # where each joint ends, counting the fine steps sent since it was last read
        self._step_chain = False                # fine steps have been sent since the position was last read
        self._step_end = 0.0                    # when the fine step in flight has finished travelling
        self._step_next = 0.0                   # when the next fine step may be sent
        self.speed = 10
        self.arm_power = True
        self.notice = ""
        self.joints = [0.0] * 6
        self.words = None                       # latest status word per axis
        self.events = []                        # (seconds since start, axis, old word, new word, joint angle or None)
        self._prompt = f"{application}> ".encode()
        self._application = application
        self._ser = serial.Serial(port, baud, timeout=0, exclusive=True)
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        log_dir = log_dir or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sessions")
        self._raw = open(os.path.join(log_dir, stamp + "-arm.log"), "wb")
        self._table = open(os.path.join(log_dir, stamp + "-arm-events.csv"), "w")
        self._table.write("t,axis,old,new,changed,joint_deg,exact\n")
        self._lock = threading.Lock()           # guards the jog request and writes to the port
        self._want, self._want_until, self._source = None, 0.0, None
        self._pick, self._pick_since = None, 0.0  # the joint the stick points at, and since when
        self._moving = None                     # (axis, sign, from angle, started, expected end)
        self._last_move = None
        self._halted = False                    # Ctrl-Z has been written and the worker has not tidied up yet
        self._speed_sent = None
        self._buf, self._text = b"", b""
        self._halt_at = 0.0                     # when the pending stop was sent
        self._move_acked = True                 # the shell has answered the last move line typed
        self._retry_at = 0.0                    # the controller was busy: when to offer the move again
        self._busy_since = None                 # since when it has been refusing moves
        self._need_position = False             # stopped, and the position has not been read back yet
        self._desync = False                    # a reply was cut short: get a clean prompt before the next command
        self._streaming = False                 # the status display is running
        self._status_at = 0.0                   # when the last status line arrived
        self._pending = []                      # status changes seen during the current move
        self._t0 = time.monotonic()
        self._closing = False
        self._ready = threading.Event()
        self._failed = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not self._ready.wait(200.0) or self._failed:      # long enough for someone to walk over and press Pause/Continue
            self._closing = True
            raise RobotError(self._failed or "the controller did not answer")

    # ---- what the gamepad driver calls -------------------------------------------------------------

    def jog(self, frame, vec, ttl=0.3, source="rest", aux=(0.0, 0.0)):
        vec = [float(v) for v in vec]
        fine = bool(aux) and float(aux[0]) <= FINE_TRIGGER
        if frame != "joint" and any(vec):
            self._stop()
            raise RobotError("the real arm is not homed: joint mode only (Options changes mode)")
        with self._lock:
            current = self._want
        want = None
        if current and abs(vec[current[0]]) >= STICK_OFF and (vec[current[0]] > 0) == (current[1] > 0):
            want = current                      # keep the joint that is moving until its stick lets go
        else:
            axis = max(range(6), key=lambda i: abs(vec[i]))
            pick = (axis, 1 if vec[axis] > 0 else -1) if abs(vec[axis]) >= STICK_ON else None
            # A stick on its way out from the centre points at other joints for a moment, so a joint
            # has to stay the one asked for a short while before it starts.
            now = time.monotonic()
            if pick != self._pick:
                self._pick, self._pick_since = pick, now
            if pick is not None and now - self._pick_since >= SETTLE_S:
                want = pick
            elif pick is not None:
                if current is not None:
                    self._stop()                # the old joint is let go while the new one settles
                return
        if want is None:
            self._stop()
            return
        with self._lock:
            self._want, self._want_until, self._source = want, time.monotonic() + min(float(ttl), 1.0), source
            self._fine = fine
        if self._moving and (self._moving[:2] != want or fine):
            self._write_halt()                  # a continuous move is stopped for a new joint, or to start fine steps

    def halt(self):
        with self._lock:
            self._want = None
        self._write_halt(force=True)            # always: a stop request goes to the controller even if we think it is still

    def set_speed(self, percent):
        if isinstance(percent, bool) or not isinstance(percent, (int, float)) or not 1 <= percent <= 100:
            raise RobotError("speed must be 1 to 100")
        self.speed = int(min(percent, MAX_SPEED_PCT))
        if percent > MAX_SPEED_PCT:
            raise RobotError(f"speed is held to {MAX_SPEED_PCT}% while the arm is not homed")

    def set_arm_power(self, on):
        raise RobotError("arm power is switched at the controller's front panel")

    def set_gripper(self, is_open):
        raise RobotError("the gripper is not connected to this program yet")

    def goto(self, pose, source="rest"):
        raise RobotError("go-to moves need a homed arm: not available yet")

    def home(self, source="rest"):
        raise RobotError("homing is not started from the controller pad yet")

    def cycle_start(self, source="rest"):
        raise RobotError("the cycle runs only on the mock so far")

    def state(self):
        with self._lock:
            wanted = self._want is not None and time.monotonic() < self._want_until
            source = self._source
            fine = self._fine
        stepping = time.monotonic() < self._step_end
        command = {"kind": "jog", "source": source} if wanted or self._moving or stepping else None
        mode = None if command is None else "fine" if fine else "continuous"
        return {"backend": self.backend, "joints": list(self.joints), "true_joints": self.true_joints(),
                "speed": self.speed, "command": command,
                "moving": self._moving is not None, "notice": self.notice, "mode": mode,
                "settling": self._halted or self._last_move is not None or self._step_chain,      # stopped, position not read back yet
                "home_switches": [w & 1 for w in self.words[:6]] if self.words else None}

    def true_joints(self):
        """Joint angles measured from the true zero pose, or None if that pose has not been given."""
        if self.zero_at is None:
            return None
        return [round(a - z, 3) for a, z in zip(self.joints, self.zero_at)]

    def _room(self, axis, sign):
        """Degrees this joint may still be sent in this direction, from where the moves already sent end."""
        lo, hi = model.LIMITS_DEG[axis]
        true_target = self._target - self.zero_at[axis]
        return (hi - LIMIT_MARGIN_DEG) - true_target if sign > 0 else true_target - (lo + LIMIT_MARGIN_DEG)

    def close(self):
        self._stop()
        self._closing = True
        self._thread.join(8.0)

    # ---- stopping ------------------------------------------------------------------------------------

    def _stop(self):
        with self._lock:
            self._want = None
        if self._moving:                        # includes a move that is being sent right now
            self._write_halt()

    def _write_halt(self, force=False):
        """Ctrl-Z to the controller. `force` sends it even if one is already waiting to be tidied up."""
        with self._lock:
            if self._halted and not force:
                return
            if not self._halted:
                self._halt_at = time.monotonic()
            self._halted = True
            self._ser.write(HALT)

    def _stale(self):
        """Called all the time by the worker: stop if the jog request has stopped being refreshed."""
        with self._lock:
            if self._want is not None and time.monotonic() >= self._want_until:
                self._want = None
        if self._moving and (self._want != self._moving[:2] or self._fine):
            self._write_halt()

    # ---- the worker thread: the only reader of the port ---------------------------------------------

    def _pump(self):
        chunk = self._ser.read(4096)
        if not chunk:
            return
        self._raw.write(chunk)
        self._raw.flush()
        self._buf += chunk
        while True:
            found = STATUS.search(self._buf)
            if not found:
                break
            self._text += self._buf[:found.start()]
            self._buf = self._buf[found.end():]
            self._status([int(w, 16) for w in found.group(1).split()])
        tail = self._buf.rsplit(b"\n", 1)[-1]
        if 0 < len(tail) < 72 and PARTIAL.fullmatch(tail):                   # the start of a status line, not yet complete
            self._text += self._buf[:len(self._buf) - len(tail)]
            self._buf = tail
        else:
            self._text, self._buf = self._text + self._buf, b""

    def _status(self, words):
        now = self._status_at = time.monotonic()
        if not any(words[:6]):
            return
        if self.words is not None:
            for axis in range(6):
                old, new = self.words[axis], words[axis]
                if (old ^ new) & ~QUIET_BITS:
                    self._pending.append((now, axis, old, new))
        self.words = words
        self.arm_power = not any(w >> 3 & 1 for w in words[:6])

    def _wait(self, done, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._pump()
            self._stale()
            if done():
                return True
            time.sleep(0.003)
        return False

    def _command(self, line, timeout=3.0, guard=None):
        """Send one line and return what the shell printed up to its next prompt, or None if no prompt came.

        Typed characters are echoed straight into whatever the status display is printing, so the
        line is sent in the quiet gap just after a status line, and the echo is not relied on.
        `guard` is asked at the last moment before sending; if it says no, nothing is sent.
        """
        if self._desync and not self._settle():
            return None
        start = time.monotonic()
        while self._streaming and time.monotonic() - self._status_at > 0.03 and time.monotonic() - start < 0.5:
            self._pump()
            self._stale()
            time.sleep(0.002)
        if guard is not None and not guard():
            return None
        if line.startswith("joint"):
            self._move_acked = False
        self._text = b""
        was_halted = self._halted
        with self._lock:
            self._ser.write(line.encode() + b"\r")
        self._wait(lambda: self._prompt in self._text or (self._halted and not was_halted), timeout)
        end = self._text.find(self._prompt)
        if end < 0:
            self._desync = True                 # the shell may still be answering: tidy up before the next command
            return None
        return self._text[:end].decode("latin-1").replace("\r", "")

    def _settle(self):
        """Wait for the shell to go quiet, then ask it for a fresh prompt."""
        quiet = time.monotonic()
        end = quiet + 3.0
        seen = len(self._text)
        while time.monotonic() < end and time.monotonic() - quiet < 0.3:
            self._pump()
            if len(self._text) != seen:
                seen, quiet = len(self._text), time.monotonic()
            time.sleep(0.003)
        self._text = b""
        with self._lock:
            self._ser.write(b"\r")
        self._desync = not self._wait(lambda: self._prompt in self._text, 2.0)
        self._wait(lambda: False, 0.05)
        return not self._desync

    def _tidy(self):
        """After a Ctrl-Z: get the shell back to a known state, quickly, so the next jog can start.

        If the shell had already answered the last move line, the stop has dealt with it and the shell
        prints a fresh prompt by itself. If a move line was still on its way, the shell may act on it
        after the stop has been and gone, so wait for it to go idle and stop again.
        """
        self._last_move, self._moving = self._moving or self._last_move, None
        mark = self._text.rfind(b"halting robot")
        clean = self._move_acked and not self._desync and self._wait(
            lambda: (lambda at: at >= 0 and self._prompt in self._text[at:])(self._text.find(b"halting robot", max(mark, 0))), 0.6)
        if not clean:
            self._settle()
            with self._lock:
                self._ser.write(HALT)
            self._settle()
        self._move_acked = True
        self._halted = False
        self._need_position = True
        self._step_chain, self._step_end = False, 0.0       # the stop may have cut a step short: its travel is unknown

    def _read_position(self):
        reply = self._command("w2")
        lines = (reply or "").split("\n")
        for i, line in enumerate(lines):
            if line.startswith("JOINTS"):
                values = [float(n) for n in NUMBER.findall(line[6:])]
                if len(values) >= 6:
                    self.joints = values[:6]
                    self._targets, self._step_chain = list(self.joints), False
                    self._report()
                    self._last_move = None
                    return True
        return False

    def _report(self):
        """Write out the status changes seen since the last position reading, with the joint angle at each."""
        moving, pending, self._pending = self._last_move or self._moving, self._pending, []
        for when, axis, old, new in pending:
            angle, exact = self.joints[axis], 1
            if moving and moving[0] == axis and moving[3] < when:
                start, stop = moving[2], self.joints[axis]
                took = max(self._stopped_at - moving[3], 1e-3)
                angle, exact = start + (stop - start) * min((when - moving[3]) / took, 1.0), 0
            changed = "+".join(("" if new >> b & 1 else "not ") + STATUS_BITS[b] for b in range(16) if (old ^ new) >> b & 1 and not QUIET_BITS >> b & 1)
            self.events.append((when - self._t0, axis + 1, old, new, angle))
            self._table.write(f"{when - self._t0:.2f},{axis + 1},{old:04x},{new:04x},{changed},{angle:.2f},{exact}\n")
            self._table.flush()
            self.say(f"  J{axis + 1} status {old:04x} -> {new:04x} ({changed}) at about {angle:.1f} deg")

    def _run(self):
        try:
            self._enter()
        except Exception as err:                # noqa: BLE001 - reported to whoever is constructing us
            self._failed = f"could not start a session on the controller: {err}"
            self._ready.set()
            self._abandon()
            return
        self._ready.set()
        try:
            self._loop()
        except Exception as err:                # noqa: BLE001 - never leave the arm moving or the shell half-used
            self.notice = f"stopped after an internal error: {err!r}"
            self.say("  " + self.notice)
            self._moving = self._moving or (0, 0, 0.0, 0.0, 0.0)      # makes _leave send a stop
        finally:
            with self._lock:
                self._want = None
            try:
                self._leave()
            except Exception as err:            # noqa: BLE001
                self.say(f"  could not leave the controller's shell cleanly: {err!r}")

    def _abandon(self):
        """Start-up failed part-way: interrupt whatever the shell is waiting on and close the port."""
        try:
            self._ser.write(HALT)
            self._wait(lambda: False, 0.5)
            self._ser.close()
        except Exception:                       # noqa: BLE001
            pass

    def _loop(self):
        self._stopped_at = time.monotonic()
        last_position = time.monotonic()
        while not self._closing:
            self._pump()
            self._stale()
            now = time.monotonic()
            if self._halted:
                self._stopped_at = now
                self._tidy()
                continue
            with self._lock:
                want, fine = self._want, self._fine
            if self._moving is None and want is not None and self._speed_sent == self.speed:
                if fine:
                    if now >= self._retry_at and now >= self._step_next:
                        self._step(want)
                elif now >= self._retry_at and now >= self._step_end:     # a step in flight finishes first
                    if self.zero_at is not None and self._need_position:
                        # The limit arithmetic needs to know where the last jog actually stopped.
                        if self._read_position():
                            self._need_position = False
                        else:
                            self._retry_at = time.monotonic() + 0.08
                            continue
                    self._start(want)
            elif self._moving is not None and self._limit_reached:
                if now > self._moving[4] + 0.4:      # its last move should have run out at the limit by now
                    before = list(self.joints)
                    if self._read_position() and max(abs(a - b) for a, b in zip(self.joints, before)) < 0.02:
                        self._last_move, self._moving = None, None      # two readings agree: it has stopped by itself
                        self._stopped_at, self._need_position = time.monotonic(), False
                        last_position = time.monotonic()
                    else:
                        self._wait(lambda: False, 0.25)                 # still moving, or no answer: look again shortly
            elif self._moving is not None and now > self._moving[4] - TOPUP_S:
                self._send_move(*self._moving[:2], top_up=True)
            elif self._moving is None and self._speed_sent != self.speed:
                if now >= self._retry_at and not self._set_speed():
                    self._retry_at = time.monotonic() + 0.15    # refused while it resets after a stop: offer it again
            elif self._moving is None and want is None and self._need_position and now - self._stopped_at > 0.4:
                if self._read_position():       # only once the pad is idle, so it never delays a jog
                    self._need_position = False
                    last_position = time.monotonic()
                    self.say("  stopped at  " + "  ".join(f"J{i + 1} {a:8.2f}" for i, a in enumerate(self.true_joints() or self.joints))
                             + ("   home switches " + "".join(str(w & 1) for w in self.words[:6]) if self.words else ""))
                else:
                    self._stopped_at = time.monotonic() - 0.15
            elif self._moving is None and want is None and now - last_position > 20.0 and now >= self._step_end:
                self._read_position()
                last_position = time.monotonic()
            if not self.arm_power and not self.notice:
                self.notice = "arm power is off"
                self.say("  arm power is off (limp bit set): press the Arm Power button on the controller")
            elif self.arm_power and self.notice == "arm power is off":
                self.notice = ""
            time.sleep(0.003)

    def _start(self, want):
        self._moving = (want[0], want[1], self.joints[want[0]], time.monotonic(), time.monotonic())
        self._target, self._limit_reached, self._step_chain = self.joints[want[0]], False, False
        if not self._send_move(*want) and not self._halted:
            self._moving = None

    def _send_move(self, axis, sign, top_up=False):
        """One jog move. Never sent unless that jog is still wanted; stopped at once if it is not wanted afterwards."""
        def wanted():
            with self._lock:
                return (self._want == (axis, sign) and time.monotonic() < self._want_until and not self._halted
                        and not self._fine)
        rate = model.MAX_SPEED_DEG_S[axis] * self.speed / 100.0
        degrees = sign * rate * CHUNK_S
        if self.zero_at is not None:
            room = self._room(axis, sign)
            if room < 0.05:
                self._limit_reached = True          # nothing more is sent this way; what is under way ends inside the range
                if not top_up and self._limit_said != (axis, sign):
                    self._limit_said = (axis, sign)
                    self.notice = f"J{axis + 1} ({model.JOINT_NAMES[axis]}) is at its limit"
                    self.say("  " + self.notice)
                return False
            degrees = sign * min(abs(degrees), room)
        line = f"joint {axis + 1},{degrees:.2f}"
        sent = time.monotonic()
        # A move typed while another is under way is not answered until that one has finished (seen on
        # the arm), so a top-up is given time to sit there. Letting go of the stick still ends the wait.
        reply = self._command(line, timeout=TOPUP_WAIT_S if top_up else 2.0, guard=wanted)
        if top_up and reply is not None and time.monotonic() - sent > 1.0:
            self.say(f"  (the next move was held for {time.monotonic() - sent:.1f} s before it was taken)")
        if reply is None and not self._desync and not self._halted:
            return False                        # the jog was let go before anything was sent
        if reply is not None:
            self._move_acked = True
        problem = "no answer from the controller" if reply is None else " ".join(reply.replace(line, "").split())
        if reply is not None and not PROBLEM.search(problem):
            problem = ""
        if "being reset" in problem and not top_up and wanted():
            # Still recovering from the last stop: nothing was started. Offer the move again shortly.
            self._busy_since = self._busy_since or sent
            if time.monotonic() - self._busy_since < BUSY_S:
                self._retry_at = time.monotonic() + 0.08
                return False
        if problem or not wanted():
            self._write_halt(force=True)        # it may have started moving: stop it whatever else is going on
            self._busy_since = None
            if problem and not self._halted_by_release():
                self.notice = problem
                self.say(f"  the controller did not accept the move: {problem}")
            return False
        if self._busy_since is not None:
            self.say(f"  (the controller refused moves for {time.monotonic() - self._busy_since:.2f} s after the stop)")
            self._busy_since = None
        self.notice, self._limit_said = "", None
        self._target += degrees
        axis_, sign_, start, started, ends = self._moving
        self._moving = (axis_, sign_, start, started, max(ends, time.monotonic()) + abs(degrees) / rate)
        return True

    def _step(self, want):
        """One fine step. No stop is ever sent for it unless the controller does not answer; see the note at the top."""
        axis, sign = want

        def wanted():
            with self._lock:
                return (self._want == want and time.monotonic() < self._want_until and not self._halted
                        and self._fine)
        if self.zero_at is not None and self._need_position and not self._step_chain:
            if not self._read_position():       # the limit arithmetic needs to know where the last move really stopped
                self._retry_at = time.monotonic() + 0.08
                return
            self._need_position = False
        degrees = sign * FINE_STEP_DEG
        if self.zero_at is not None:
            self._target = self._targets[axis]
            room = self._room(axis, sign)
            if room < 0.05:
                if self._limit_said != (axis, sign):
                    self._limit_said = (axis, sign)
                    self.notice = f"J{axis + 1} ({model.JOINT_NAMES[axis]}) is at its limit"
                    self.say("  " + self.notice)
                self._retry_at = time.monotonic() + 0.05
                return
            degrees = sign * min(FINE_STEP_DEG, room)
        rate = model.MAX_SPEED_DEG_S[axis] * self.speed / 100.0
        line = f"joint {axis + 1},{degrees:.2f}"
        sent = time.monotonic()
        reply = self._command(line, timeout=2.0, guard=wanted)
        if reply is None and not self._desync and not self._halted:
            return                              # let go before anything was sent
        if reply is not None:
            self._move_acked = True
        problem = "no answer from the controller" if reply is None else " ".join(reply.replace(line, "").split())
        if reply is not None and not PROBLEM.search(problem):
            problem = ""
        if "being reset" in problem and wanted():
            self._busy_since = self._busy_since or sent
            if time.monotonic() - self._busy_since < BUSY_S:
                self._retry_at = time.monotonic() + 0.08
                return
        if problem:
            self._busy_since = None
            if reply is None:
                self._write_halt(force=True)    # it may have started moving
            self._retry_at = time.monotonic() + 0.5
            if not self._halted_by_release():
                self.notice = problem
                self.say(f"  the controller did not accept the move: {problem}")
            return
        if self._busy_since is not None:
            self.say(f"  (the controller refused moves for {time.monotonic() - self._busy_since:.2f} s after the stop)")
            self._busy_since = None
        self.notice, self._limit_said = "", None
        self._targets[axis] += degrees
        self._step_chain, self._need_position = True, True
        self._step_end = time.monotonic() + abs(degrees) / rate
        self._step_next = max(self._step_end, sent + FINE_GAP_S)
        self._stopped_at = max(self._stopped_at, self._step_end)     # the position is read 0.4 s after the last step ends

    def _halted_by_release(self):
        """True when a move went unanswered only because the jog was let go while it was being typed."""
        with self._lock:
            return self._want is None

    def _enter(self):
        def text():
            return self._text.rstrip(b" ")
        self._ser.reset_input_buffer()
        self._ser.write(b"\r")
        self._wait(lambda: text().endswith((b"$", b">")), 3.0)
        if not text().endswith(self._prompt.rstrip()):
            if not text().endswith(b"$"):
                raise RobotError("no shell prompt from the controller")
            self._text = b""
            self._ser.write(f"ash {self._application}\r".encode())
            if not self._wait(lambda: self._text.endswith(self._prompt), 8.0):
                raise RobotError("the application shell did not start")
        # Taking control of the arm. After a program that had control ended badly, the controller
        # hands it over only when someone presses Pause/Continue on its front panel.
        if "ON" in (self._command("online", timeout=4.0) or ""):
            self._text = self._prompt           # already on from an earlier session: nothing to ask for
        else:
            self._text = b""
            self._ser.write(b"online 1\r")
        asked = False
        end = time.monotonic() + 180.0
        while not self._text.endswith(self._prompt):
            self._pump()
            if not asked and b"PAUSE/CONTINUE" in self._text:
                asked = True
                self.say("  WAITING: press the PAUSE/CONTINUE button on the controller's front panel to give this program control of the arm")
            if time.monotonic() > end or (not asked and time.monotonic() > end - 170.0):
                raise RobotError("the controller did not hand over control of the arm" + (" (Pause/Continue was not pressed)" if asked else ""))
            time.sleep(0.01)
        if "ON" not in (self._command("online", timeout=4.0) or ""):
            raise RobotError("online mode did not switch on, so moves would block")
        if "Arm power is ON" not in (self._command("arm_status") or ""):
            self.say("  note: the controller reports arm power is not on")
        if self.stream:
            self._command("axst &")
            self._streaming = True
            if not self._wait(lambda: self.words is not None, 5.0):
                raise RobotError("the status display did not start")
        self._set_speed()
        if not self._read_position():
            raise RobotError("could not read the arm's position")

    def _set_speed(self):
        """Tell the controller the speed. True only if it took it: while it is resetting after a stop it
        answers "device is being reset" and keeps the old speed, and every move sized for the new one
        would then be wrong (seen on the arm, 2026-11-23)."""
        speed = self.speed
        line = f"speed {speed}"
        reply = self._command(line)
        if reply is None or PROBLEM.search(reply.replace(line, "")):
            return False
        self._speed_sent = speed
        return True

    def _leave(self):
        """Leave the controller at its system prompt (which also ends the status display).

        A stop is sent only if something may be moving, and the shell is not exited until the
        controller answers normally again: leaving while it is still resetting after a stop makes
        it hold control of the arm until Pause/Continue is pressed.
        """
        if self._moving or self._halted:
            self._ser.write(HALT)
            self._wait(lambda: False, 0.5)
        self._halted, self._moving = False, None
        self._settle()
        for _ in range(12):
            if self._read_position():
                break
            self._wait(lambda: False, 0.25)
        self._text = b""
        self._ser.write(b"exit\r")
        self._wait(lambda: b"sure" in self._text, 2.0)
        self._ser.write(b"yes\r")
        self._wait(lambda: self._text.rstrip(b" ").endswith(b"$"), 3.0)
        self._wait(lambda: False, 0.5)
        self._raw.close()
        self._table.close()
        self._ser.close()
