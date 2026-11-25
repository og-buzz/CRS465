"""JogdArm: a drop-in for crs.RealArm that jogs through `jogd`, a program running ON the controller.

RealArm types whole `joint` commands at the console, so a jog starts late and runs at one speed. jogd
(controller/programs/jogd) takes one-letter commands and moves the joint at a given speed, so this class maps
how far the stick is pushed to a speed (picked when the jog starts, see below).

Protocol (bytes the PC writes while jogd is the foreground program; measured on the arm):
  A..F / a..f + two digits   joint 1..6 positive / negative at that percent (jogd clamps to its maxspeed)
  .                          stop the move            q   stop, release control, exit to the `$` prompt
  p                          print the position       0x1a (Ctrl-Z) stops the arm at once; 0x05 kills jogd
The command in force must be repeated at least every 300 ms or jogd stops the move itself (prints `T`);
it is repeated every 100 ms here. jogd prints one line per event: `+A07` (move started / speed changed),
`-` (stopped), `T` (its dead-man fired), `E <number> <text>` (a controller library call failed),
`P c1..c6` (encoder counts, answer to `p`; the program may not have it yet), `jogd ready`.
Ctrl-Z stops the arm AND kills jogd (the controller prints `^C equivalent - halting robot...` and
`[301 killed by SIGABRT.]` and shows `$`; control stays held, so the next start asks for the front-panel
button). So halt() is only `.`; Ctrl-Z is written only as the emergency stop when `.` goes unanswered, and
then jogd is treated as dead and restarted by itself (see _restart).
Variant `s` (this class starts `jogd 1 <max> s`): the speed of a jog is fixed when it starts. A command with another
speed on the same joint and direction makes jogd stop the move and start it again (`-` then `+L nn`, about half a
second of standstill); the same speed is ignored. A start answers with `+` after about 250 to 300 ms, a `.` with `-`
after about 0.2 s. Hence the speed is picked once, from the largest deflection in the first SPEED_PICK_S of the stick
pointing at a joint, and changed later only for a big, sustained difference (RESPEED_*) or never (respeed=False).

One background thread owns the port (nothing else writes to it): it reads lines, runs the jog state and
writes the commands. jog(), halt() and close() only leave requests for it. `step()` is one pass of that
thread; tests construct with threaded=False and call step() themselves with a fake clock.

Not done here: soft limits (jogd knows none; zero_at only gives true_joints()), fine steps (L2: the stick
simply gives low speeds), the status/home-switch display (state()["home_switches"] is None).
"""
import datetime
import os
import re
import threading
import time

from . import aci, calfile, console, crs, homezc
from .sim import RobotError

REMOTE = "/tmp/jogd"
LOCAL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "controller", "programs", "jogd")
DEFAULT_MAX_SPEED = 25
TOP_SPEED = 99                    # jogd's start command carries two digits (a465/jogprog.py)
READY_WAIT_S = 10.0               # `jogd ready` is printed about 1.1 s after the start command
BUTTON_WAIT_S = homezc.BUTTON_WAIT_S
KEEPALIVE_S = 0.100               # repeat the command in force this often (jogd's dead-man is 300 ms)
SPEED_PICK_S = 0.12               # a stick must point at a joint this long before it starts; the largest deflection in that window is the jog's speed
RESPEED_FACTOR = 1.6              # the wanted speed must differ from the commanded one by this factor (either way) ...
RESPEED_AFTER_S = 0.4             # ... continuously for this long ...
RESPEED_GAP_S = 1.0               # ... and not within this long of the previous start, before a new speed is sent
RESPEED_WAIT_S = 1.0              # the `+` of a speed change (jogd stops, then starts again: `-` then `+`) must come this soon
ANSWER_WAIT_S = 1.0               # a start's `+` can take 0.3 s and more; a speed change waits for it, but no longer than this
STOP_WAIT_S = 0.6                 # after `.` with a move in force `-` (or `T`, or `E`) must come this soon, else Ctrl-Z and jogd is dead
RESTART_WAIT_S = 2.5              # after a Ctrl-Z the controller refuses moves for about 2 s: nothing is started sooner
RESTART_PROMPT_S = 3.0            # the `$` prompt must show this soon after the single `\r` that starts a restart
MAX_RESTARTS = 3                  # per failure streak
POSITION_EVERY_S = 0.250          # `p` this often while idle ...
POSITION_MOVING_EVERY_S = 0.500   # ... and this often while a move is in force (each `p` holds jogd's loop for about 0.1 s)
DEAD_SIGNS = ("killed by SIG", "^C equivalent")    # lines the controller prints when the program is killed (seen on the arm)
REPORT_DELAY_S = 0.3              # after a move has stopped, read the position this much later for the "stopped at" line
NO_POSITION_S = 1.0               # no `P` line this long after the first `p`: say live position is unavailable
QUIT_WAIT_S = 5.0                 # `$` prompt expected this soon after `q`
CTRL_Z_WAIT_S = 2.5               # after Ctrl-Z in the shutdown escalation, before Ctrl-E
POLL_S = 0.002
CTRL_Z, CTRL_E = console.CTRL_Z, console.CTRL_E

# Stick deflection to speed. The pad's value for the picked joint (already shaped by gamepad.shape) goes from
# crs.STICK_ON (where a jog starts) to 1.0. That span is rescaled to 0..1 and raised to this power, so the
# bottom of the travel has fine control (speeds 1, 2, 3 ... percent) and the top is steep. 1 = linear.
SPEED_CURVE_EXPONENT = 1.5
MIN_SPEED_PCT = 1

COUNTS_PER_DEGREE = calfile.COUNTS_PER_DEGREE

LINE_PATTERNS = (                 # searched at the END of a line, so an echoed command in front of a line does no harm
    ("E", re.compile(r"E (-?\d+) (.*)$")),
    ("P", re.compile(r"(?<![A-Za-z])P((?:\s+[-+]?\d+){6})\s*$")),
    ("ready", re.compile(r"jogd ready$")),
    ("+", re.compile(r"\+([A-Fa-f]\d\d)$")),
    ("T", re.compile(r"(?<![A-Za-z])T$")),
    ("-", re.compile(r"-$")),
)


def speed_for(deflection, full):
    """Percent for a stick deflection (the shaped value, STICK_ON..1) with `full` percent at full deflection."""
    span = (min(1.0, max(crs.STICK_ON, abs(deflection))) - crs.STICK_ON) / (1.0 - crs.STICK_ON)
    full = max(MIN_SPEED_PCT, int(full))
    return max(MIN_SPEED_PCT, min(full, int(round(MIN_SPEED_PCT + (full - MIN_SPEED_PCT) * span ** SPEED_CURVE_EXPONENT))))


def counts_to_angles(counts):
    return [c / k for c, k in zip(counts, COUNTS_PER_DEGREE)]


def command_bytes(axis, sign, speed):
    return f"{'ABCDEF'[axis] if sign > 0 else 'abcdef'[axis]}{int(speed):02d}".encode()


def classify(text):
    text = text.strip()
    for kind, pattern in LINE_PATTERNS:
        found = pattern.search(text)
        if found:
            return kind, found
    return "other", text


class JogdArm:
    """Has the methods the gamepad driver calls on a robot (see crs.RealArm). Everything but joint jog, stop and speed is refused."""

    backend = "c500+jogd"

    def __init__(self, port="/dev/ttyUSB0", baud=38400, say=print, zero_at=None, max_speed=DEFAULT_MAX_SPEED,
                 log_dir=None, serial_port=None, clock=time.monotonic, sleep=time.sleep, threaded=True,
                 local=LOCAL, remote=REMOTE, upload=None, respeed=True, stick_speed=True):
        if not 1 <= max_speed <= TOP_SPEED:
            raise RobotError(f"max_speed must be 1 to {TOP_SPEED}")
        self._say = say
        self._clock, self._sleep = clock, sleep
        self.max_speed = int(max_speed)
        self.zero_at = [float(v) for v in zero_at] if zero_at is not None else None
        self.speed = 10
        self.arm_power = True
        self.notice = ""
        self.joints = [0.0] * 6
        self._lock = threading.Lock()
        self._want, self._want_mag, self._want_until, self._source = None, 0.0, 0.0, None
        self._pick, self._pick_since, self._pick_peak = None, 0.0, 0.0
        self._want_start = 0.0              # the deflection that sets the speed of a new jog (peak of the pick window)
        self.respeed = bool(respeed)
        self.stick_speed = bool(stick_speed)    # False: the stick gives the direction only; every jog runs at the speed setting
        self._active_want = None
        self._fresh = True                  # the next start of this wanted jog is its first: it uses the picked speed
        self._diff_since = None             # since when the wanted speed has differed from the commanded one by RESPEED_FACTOR
        self._respeed_pending = None        # when a new speed was sent: its `-` and `+` are not the end of the jog
        self._halt_req = False
        self._closing = self._shut = False
        self._crashed = None
        # jogd state, touched by the thread only
        self._cmd = None                    # (axis, sign, speed) written and not yet stopped
        self._last_write = -1e9
        self._last_key = None               # (axis, sign) of the last move command written
        self._last_move_write = -1e9
        self._last_change = -1e9            # when the last start / speed change was written (keep-alives do not count)
        self._await_since = None            # a start / speed change was written and its `+` has not come
        self._moving = False                # a `+` has come and no `-`
        self._stopping_since = None         # `.` written, `-` not seen yet
        self._blocked = None                # (axis, sign) that came back with E: not sent again until the stick is released
        self._last_p = -1e9
        self._dead = False                  # jogd is not running (seen, or presumed after Ctrl-Z): nothing is written
        self._last_ctrlz = -1e9
        self._auto = False                  # a restart is due now
        self._released = False              # after a failed restart: the stick has been let go
        self._attempts = 0
        self._gave_up = False
        self._remote = remote
        self._first_p = None
        self._p_seen = False
        self._p_said = False
        self._report_at = None              # a move stopped: when to read the position for the "stopped at" line
        self._report_wait = False           # that `p` is written and its answer is awaited
        self._partial, self._since, self._lines = b"", b"", []
        self._button = False
        self._thread = None
        self._ser = None
        self._log = None
        self._open(port, baud, log_dir, serial_port)
        try:
            self._start(local, remote, upload)
        except BaseException:
            self._close_port()
            raise
        if threaded:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    # ---- starting ----------------------------------------------------------------------------------

    def _open(self, port, baud, log_dir, serial_port):
        log_dir = log_dir or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sessions")
        path = os.path.join(log_dir, datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S") + "-jogd.log")
        self._log = open(path, "w")
        self.log_path = path
        if serial_port is None:
            import serial
            try:
                serial_port = serial.Serial(port, baud, timeout=0, exclusive=True)
            except Exception as err:            # noqa: BLE001
                self._log.close()
                raise RobotError(f"cannot open {port}: {err}")
        self._ser = serial_port
        self.link = aci.Link(homezc.LoggedPort(serial_port, self._logline, self._clock), self.say, self._clock, self._sleep)

    def _logline(self, text):
        if self._log and not self._log.closed:
            self._log.write(text + "\n")
            self._log.flush()

    def say(self, text):
        self._logline(f"{'':8}   {text}")
        self._say(text)

    def _start(self, local, remote, upload):
        link = self.link
        if not aci.at_system_prompt(link):
            raise RobotError("the controller is not at its `$` prompt (is another program using the console?)")
        if not os.path.isfile(local):
            raise RobotError(f"{local} does not exist")
        try:
            homezc.ensure_program(link, remote, local, (lambda: upload(link, self.say)) if upload
                                  else (lambda: aci.send_file(link, local, remote, report=self.say)), self.say)
        except (homezc.HomeError, aci.TransferError) as err:
            raise RobotError(f"could not put {remote} on the controller: {err}")
        joints = console.read_joints(link)
        if joints is None:
            raise RobotError("the controller did not answer `w2`")
        self.joints = joints
        link.drain(quiet=0.1, limit=1.0)
        why = self._launch()
        if why:
            raise RobotError(why)

    def _launch(self):
        """Type the start command and wait for `jogd ready` (with the button handling). None if it came up, else why not
        (the controller has then been left at `$` as far as possible)."""
        link = self.link
        command = f"{self._remote} 1 {self.max_speed} s"
        self.say(f"starting: {command}")
        self._reset_since()
        self._button = False
        started = self._clock()
        link.port.write(command.encode() + b"\r")
        end = started + READY_WAIT_S
        while True:
            self._feed()
            if any(kind == "ready" for _, kind, _ in self._lines):
                break
            if b"\n" in self._since and self._at_prompt():
                self.say("jogd returned to the prompt without printing `jogd ready`")
                return "jogd did not start (it returned to the prompt)"
            if homezc.BUTTON.encode() in self._since and not self._button:
                self._button = True
                end = self._clock() + BUTTON_WAIT_S
                self.say("WAITING: PRESS THE PAUSE/CONTINUE BUTTON ON THE CONTROLLER'S FRONT PANEL NOW "
                         f"(up to {BUTTON_WAIT_S:.0f} s). Nothing moves, and nothing is sent, until `jogd ready` is seen.")
            if self._clock() >= end or self._closing:
                self.say("no `jogd ready` within " + (f"{BUTTON_WAIT_S:.0f} s of the button request" if self._button else f"{READY_WAIT_S:.0f} s"))
                self._abort_start()
                return "jogd did not come up" + (": the PAUSE/CONTINUE button was not pressed" if self._button else "")
            self._sleep(POLL_S)
        self._lines = []
        self._reset_since()
        self.say(f"jogd ready after {(self._clock() - started) * 1000:.0f} ms; joint speed clamp {self.max_speed}%")
        self._last_p = self._clock()
        return None

    def _abort_start(self):
        """jogd never said it was ready. Leave the controller at `$` if it is not already there; nothing here ever moves the arm."""
        if self._at_prompt() and b"\n" in self._since:
            return
        if self._button:
            self._write(CTRL_E)             # waiting for the button: nothing can be moving
            self.say("KILLED while waiting for the button (nothing moved): Ctrl-E sent")
            self._wait_prompt(3.0)
            return
        self._leave(stop_first=False)

    # ---- reading -----------------------------------------------------------------------------------

    def _reset_since(self):
        self._since = b""

    def _at_prompt(self):
        return self._since.rsplit(b"\n", 1)[-1].rstrip(b" ").endswith(b"$")

    def _feed(self):
        """Read what the port has. Complete lines are classified into self._lines as (time, kind, match)."""
        chunk = self.link.port.read(4096)      # through the LoggedPort: every byte goes to the log file
        if not chunk:
            return False
        now = self._clock()
        self._since = (self._since + chunk)[-4000:]
        self._partial += chunk
        while b"\n" in self._partial:
            raw, self._partial = self._partial.split(b"\n", 1)
            kind, found = classify(raw.decode("latin-1").replace("\r", ""))
            if kind == "other" and not found:
                continue
            self._lines.append((now, kind, found))
        return True

    def _wait_prompt(self, seconds):
        end = self._clock() + seconds
        while True:
            self._feed()
            self._lines = []
            if self._at_prompt() and (b"\n" in self._since):
                return True
            if self._clock() >= end:
                return False
            self._sleep(POLL_S)

    def _write(self, data):
        self.link.port.write(data)              # through the LoggedPort: every byte goes to the log file
        self.link.port.flush()

    # ---- what the gamepad driver calls ---------------------------------------------------------------

    def jog(self, frame, vec, ttl=0.3, source="rest", aux=(0.0, 0.0)):
        vec = [float(v) for v in vec]
        if frame != "joint" and any(vec):
            self._release()
            raise RobotError("the real arm is not homed: joint mode only (Options changes mode)")
        with self._lock:
            current = self._want
        want, start = None, None
        if current and abs(vec[current[0]]) >= crs.STICK_OFF and (vec[current[0]] > 0) == (current[1] > 0):
            want = current                      # keep the joint that is moving until its stick lets go
        else:
            axis = max(range(6), key=lambda i: abs(vec[i]))
            pick = (axis, 1 if vec[axis] > 0 else -1) if abs(vec[axis]) >= crs.STICK_ON else None
            # A stick on its way out from the centre points at other joints for a moment, so a joint
            # has to stay the one asked for a short while before it starts.
            now = self._clock()
            if pick != self._pick:
                self._pick, self._pick_since = pick, now
                self._pick_peak = abs(vec[pick[0]]) if pick else 0.0
            elif pick is not None:
                self._pick_peak = max(self._pick_peak, abs(vec[pick[0]]))
            if pick is not None and now - self._pick_since >= SPEED_PICK_S:
                want = pick
                start = self._pick_peak
            elif pick is not None:
                if current is not None:
                    self._release()             # the old joint is let go while the new one settles
                return
        if want is None:
            self._release()
            return
        with self._lock:
            self._want, self._want_mag = want, abs(vec[want[0]])
            if start is not None:
                self._want_start = start
            self._want_until, self._source = self._clock() + min(float(ttl), 1.0), source

    def _release(self):
        with self._lock:
            self._want = None

    def halt(self):
        with self._lock:
            self._want, self._pick = None, None
            self._halt_req = True               # the thread writes `.` (never Ctrl-Z: that kills jogd)

    def set_speed(self, percent):
        if isinstance(percent, bool) or not isinstance(percent, (int, float)) or not 1 <= percent <= 100:
            raise RobotError("speed must be 1 to 100")
        self.speed = int(min(percent, self.max_speed))
        if percent > self.max_speed:
            raise RobotError(f"speed is held to {self.max_speed}% (--max-speed)")

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
            wanted = self._want is not None and self._clock() < self._want_until
            source = self._source
        command = ({"kind": "jog", "source": source} if (wanted or self._cmd or self._moving or self._stopping_since is not None)
                   and not self._dead else None)
        return {"backend": self.backend, "joints": list(self.joints), "true_joints": self.true_joints(),
                "speed": self.speed, "command": command,
                "moving": self._moving, "notice": self.notice, "mode": None if command is None else "continuous",
                "settling": self._stopping_since is not None or self._report_at is not None or self._report_wait,
                "home_switches": None}

    def true_joints(self):
        if self.zero_at is None:
            return None
        return [round(a - z, 3) for a, z in zip(self.joints, self.zero_at)]

    def close(self):
        self._closing = True
        with self._lock:
            self._want = None
        if self._thread is not None:
            self._thread.join(30.0)
        self._shutdown()

    # ---- the thread ----------------------------------------------------------------------------------

    def _run(self):
        while self.step():
            self._sleep(POLL_S)

    def step(self):
        """One pass of the thread. False when it is over (closed, or crashed and the arm has been left stopped)."""
        if self._shut:
            return False
        try:
            if self._closing:
                self._shutdown()
                return False
            self._pass()
            return True
        except Exception as err:                # noqa: BLE001 - the arm must never be left moving
            self._crashed = err
            self.notice = f"stopped after an internal error: {err!r}"
            self.say("  " + self.notice)
            try:
                self._write(CTRL_Z)             # in doubt: stop the arm (this also kills jogd)
                self._last_ctrlz, self._dead = self._clock(), True
            except Exception:                   # noqa: BLE001
                pass
            self._shutdown()
            return False

    def _pass(self):
        self._feed()
        now = self._clock()
        lines, self._lines = self._lines, []
        for _, kind, found in lines:
            self._on_line(kind, found, now)
        if not self._dead and b"\n" in self._since and self._at_prompt():
            self._mark_dead(now, "the `$` prompt is showing: " + self._since.decode("latin-1").replace("\r", "").strip()[-80:])
        with self._lock:
            halt, self._halt_req = self._halt_req, False
            want, mag = self._want, self._want_mag
            if want is not None and now >= self._want_until:
                want = self._want = self._pick = None
            start = self._want_start
        if self._dead:
            self._dead_step(want)
            return
        if halt and self._stopping_since is None and (self._cmd is not None or self._moving or now - self._last_move_write < 1.0):
            self._write(b".")                   # the ordinary stop. Never Ctrl-Z here: that kills jogd.
            self._after_write(now)
            if self._cmd is not None or self._moving:
                self._stopping_since = now
            self._cmd, self._await_since = None, None
        if want is None:
            self._blocked = None
        if want != self._active_want:
            self._active_want, self._fresh, self._diff_since = want, True, None
        target = wanted = None
        if want is not None and want != self._blocked:
            full = min(self.speed, self.max_speed)
            if self.stick_speed:
                wanted = speed_for(mag, full)
                target = (want[0], want[1], speed_for(start, full) if self._fresh else wanted)
            else:                               # direction only: the speed is the pad's speed setting
                wanted = max(MIN_SPEED_PCT, int(full))
                target = (want[0], want[1], wanted)
        self._drive(target, now, wanted)
        if self._respeed_pending is not None and now - self._respeed_pending >= RESPEED_WAIT_S:
            self.say(f"  no `+` within {RESPEED_WAIT_S:.0f} s of the speed change: the move is taken as stopped (it is started again if the stick is held)")
            self._respeed_pending, self._cmd, self._moving, self._await_since = None, None, False, None
            self._report_at = now + REPORT_DELAY_S
        if self._stopping_since is not None and now - self._stopping_since >= STOP_WAIT_S:
            self._write(CTRL_Z)
            self._last_ctrlz = now
            self.say(f"  !!! NO `-`, `T` OR `E` WITHIN {STOP_WAIT_S} s OF `.`: Ctrl-Z WRITTEN (emergency stop; it kills jogd)")
            self._mark_dead(now, f"no answer to `.` within {STOP_WAIT_S} s; Ctrl-Z written", loud=False)
            return
        self._positions(now)

    def _mark_dead(self, now, text, loud=True):
        """jogd is not running (or must be treated so). From here nothing is written until it has been restarted."""
        self._dead = True
        self._cmd, self._moving, self._await_since, self._stopping_since = None, False, None, None
        self._respeed_pending = None
        self._report_at, self._report_wait = None, False
        self._auto, self._released = True, False
        self.notice = "jogd is not running: " + text.strip()
        if loud:
            self.say(f"  !!! JOGD IS NOT RUNNING ({text.strip()}): nothing is written until it has been restarted")
        self.say("  the arm is stopped; jogd will be restarted by itself")

    def _dead_step(self, want):
        if self._gave_up:
            return
        if want is None and not self._auto:
            self._released = True
        if self._attempts >= MAX_RESTARTS:
            self._gave_up = True
            self.notice = "jogd is not running and could not be restarted"
            self.say(f"  !!! {MAX_RESTARTS} RESTARTS OF jogd FAILED: giving up. Stop this program; the controller may need its button or a power check")
            return
        if self._auto or (self._released and want is not None):
            self._auto = self._released = False
            self._restart()

    def _restart(self):
        """Restart jogd after it was killed: wait out the controller's 2 s lock-out after a Ctrl-Z, one `\\r` to clear the
        shell's line, require `$`, then start it as at the beginning (the button is asked for: a killed program holds control).
        Blocks the thread (nothing can move meanwhile); close() ends the wait."""
        self._attempts += 1
        self.say(f"  restarting jogd (try {self._attempts} of {MAX_RESTARTS})")
        end = self._last_ctrlz + RESTART_WAIT_S
        while self._clock() < end and not self._closing:
            self._feed()
            self._lines = []
            self._sleep(POLL_S)
        if self._closing:
            return
        self._reset_since()
        self._write(b"\r")
        if not self._wait_prompt(RESTART_PROMPT_S):
            self.say(f"  !!! RESTART FAILED: no `$` prompt within {RESTART_PROMPT_S:.0f} s of the `\\r`. The arm stays stopped; "
                     "push the stick again to try once more")
            return
        self.link.ahead = b""
        why = self._launch()
        if why:
            self.say(f"  !!! RESTART FAILED: {why}. The arm stays stopped; let go of the stick and push it again to try once more")
            return
        self._dead, self._attempts, self._gave_up = False, 0, False
        self._cmd, self._moving, self._stopping_since, self._blocked = None, False, None, None
        self.notice = ""
        self.say("  jogd is running again")

    def _after_write(self, now, move=False):
        self._last_write = now
        if move:
            self._last_move_write = now

    def _drive(self, target, now, wanted=None):
        cmd = self._cmd
        if cmd is not None and (target is None or target[:2] != cmd[:2]):
            self._write(b".")                   # release, another joint, stale input, or a blocked move
            self._after_write(now)
            self._cmd, self._await_since, self._respeed_pending, self._diff_since = None, None, None, None
            self._stopping_since = now
            return
        if target is None or self._stopping_since is not None:
            return
        if cmd is None:
            self._send(target, now)
            self._fresh = False
            return
        if self.respeed and wanted is not None:
            big = max(wanted / cmd[2], cmd[2] / wanted) >= RESPEED_FACTOR
            if not big:
                self._diff_since = None
            elif self._diff_since is None:
                self._diff_since = now
            if (big and now - self._diff_since >= RESPEED_AFTER_S and now - self._last_change >= RESPEED_GAP_S
                    and self._respeed_pending is None and (self._await_since is None or now - self._await_since >= ANSWER_WAIT_S)):
                self._send((cmd[0], cmd[1], wanted), now)
                self._respeed_pending, self._diff_since = now, None
                self.say(f"  speed {cmd[2]}% -> {wanted}%: jogd stops the move and starts it again")
                return
        if now - self._last_write >= KEEPALIVE_S:
            self._write(command_bytes(*cmd))
            self._after_write(now, move=True)

    def _send(self, target, now):
        self._write(command_bytes(*target))
        self._after_write(now, move=True)
        self._await_since, self._last_change = now, now
        self._cmd, self._last_key = target, target[:2]

    def _on_line(self, kind, found, now):
        if kind == "other":
            if any(sign in found for sign in DEAD_SIGNS):
                self._mark_dead(now, found)
            return                              # echo, or text jogd does not define: it is in the log
        if self._dead:
            return
        if kind == "+":
            self._moving, self._await_since, self._respeed_pending = True, None, None
            if self._stopping_since is not None:
                self._stopping_since = now      # the answer to a command written just before the `.`: its `-` follows
        elif kind == "-":
            if self._respeed_pending is not None and self._stopping_since is None:
                self._moving = False            # the stop half of a speed change: its `+` follows, this is not the end of the jog
                return
            if self._moving or self._stopping_since is not None:
                self._report_at = now + REPORT_DELAY_S
            self._moving, self._stopping_since = False, None
        elif kind == "T":
            self._moving, self._cmd, self._await_since, self._stopping_since = False, None, None, None
            self._respeed_pending = None
            self.say("  jogd's dead-man stopped the move (a keep-alive was late); the next command restarts it")
        elif kind == "E":
            if self._cmd is not None or now - self._last_move_write < 1.0:      # not an answer to `p` alone
                self._blocked = self._cmd[:2] if self._cmd else self._last_key
            was_moving = self._moving
            self._write(b".")
            self._after_write(now)
            self._cmd, self._moving, self._await_since, self._respeed_pending = None, False, None, None
            self._stopping_since = now if was_moving else None
            self.notice = f"controller refused: E {found.group(1)} {found.group(2)}"
            self.say("  " + self.notice + "  (stopped; the stick must be released before this move is tried again)")
        elif kind == "P":
            counts = [int(n) for n in found.group(1).split()]
            self._p_seen = True
            self.joints = counts_to_angles(counts)
            if self._report_wait:
                self._report_wait = False
                self._say_position("stopped at")

    def _say_position(self, prefix):
        self.say(f"  {prefix}  " + "  ".join(f"J{i + 1} {a:8.2f}" for i, a in enumerate(self.true_joints() or self.joints)))

    def _write_p(self, now):
        self._write(b"p")
        self._last_p = now
        if self._first_p is None:
            self._first_p = now

    def _positions(self, now):
        if self._report_at is not None and now >= self._report_at:
            self._report_at = None
            if self._p_said:
                self.say("  stopped (live position is not available: the angles shown are the ones read at start)")
            else:
                self._write_p(now)
                self._report_wait = True
        elif now - self._last_p >= (POSITION_MOVING_EVERY_S if self._cmd is not None else POSITION_EVERY_S):
            self._write_p(now)
        if not self._p_seen and not self._p_said and self._first_p is not None and now - self._first_p >= NO_POSITION_S:
            self._p_said = True
            if self._report_wait:
                self._report_wait = False
                self.say("  stopped (live position is not available: the angles shown are the ones read at start)")
            self.say("  jogd has sent no `P` line: live position is not available; the angles shown are the ones read at start")

    # ---- leaving ---------------------------------------------------------------------------------------

    def _shutdown(self):
        if self._shut:
            return
        self._shut = True
        try:
            self._leave(stop_first=not self._dead)
        except Exception as err:                # noqa: BLE001
            self.say(f"  COULD NOT LEAVE THE CONTROLLER CLEANLY: {err!r}")
            try:
                self._write(CTRL_Z)
            except Exception:                   # noqa: BLE001
                pass
        finally:
            self._close_port()

    def _leave(self, stop_first):
        """`.`, `q`, expect the prompt; else Ctrl-Z, wait, Ctrl-E. Then read the joints and report them.
        If jogd is known not to be running (stop_first False after a death) `.` and `q` are skipped: only the prompt is checked."""
        ok = False
        if self._dead:
            ok = self._wait_prompt(RESTART_WAIT_S)
            if not ok:
                self._reset_since()
                self._write(b"\r")
                ok = self._wait_prompt(RESTART_PROMPT_S)
            if ok:
                end = self._last_ctrlz + RESTART_WAIT_S         # w2 is refused for about 2 s after a Ctrl-Z
                while self._clock() < end:
                    self._feed()
                    self._lines = []
                    self._sleep(POLL_S)
        if not ok:
            if stop_first or self._dead:
                self._write(b".")
            self._reset_since()
            self._write(b"q")
            ok = self._wait_prompt(QUIT_WAIT_S)
        if not ok:
            self.say(f"  NO `$` PROMPT WITHIN {QUIT_WAIT_S:.0f} s OF `q`: Ctrl-Z to stop the arm, then Ctrl-E to kill jogd")
            self._write(CTRL_Z)
            self._last_ctrlz = self._clock()
            ok = self._wait_prompt(CTRL_Z_WAIT_S)
            if not ok:
                self._write(CTRL_E)
                self.say("  !!! jogd KILLED with Ctrl-E: control of the arm is still held, so the next start needs the "
                         "PAUSE/CONTINUE button on the front panel")
                ok = self._wait_prompt(3.0)
        if not ok:
            self.say("  !!! THE CONTROLLER IS NOT AT ITS `$` PROMPT. Check it before anything else uses the arm.")
            return
        self.link.ahead = b""
        joints = console.read_joints(self.link)
        if joints is None:
            self.say("  the controller did not answer `w2` at the end")
        else:
            self.joints = joints
            self._say_position("left at")

    def _close_port(self):
        try:
            if self._ser is not None and hasattr(self._ser, "close"):
                self._ser.close()
        except Exception:                       # noqa: BLE001
            pass
        if self._log and not self._log.closed:
            self._log.close()
