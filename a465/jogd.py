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

jogd-c (controller/programs/jogd-c, built by a465/jogprog.py; used for every frame when `frames` includes world or tool, jogd
otherwise): same console protocol, and in modes 3 (world) and 4 (tool) a letter starts ONE straight-line move of a bounded
length (150 mm, or 45 degrees for rotations), `+Xnn`; `.`, the dead-man, another letter or `q` halt it (`-` / `T`). The speed is
fixed per move (the same letter at another speed is only a keep-alive), so there is no respeed in those frames. A move that
reaches its length, or that the controller ends, prints `-` BY ITSELF: that is told from the `-` answering a `.` by there being
no `.` in flight (`_stopping_since` is None). It is then taken as stopped ("move ended by itself"), and if the stick still asks
for the same jog it is started again (150 mm at a time). jogd-c ignores the command it last started once the move has ended, so
that restart is `.` (which prints nothing when nothing moves, and forgets the command) then the start. Axis letters there: A/a
X, B/b Y, C/c Z, D/d yaw (about Z), E/e pitch (Y), F/f roll (X).
`@` vectors (jogd-c, world and tool): `@` + six components (sign and one digit each) + two speed digits, 15 bytes, e.g.
`@+9+6+0+0+0+002`, acknowledged `+@+9+6+0+0+0+002`; one straight line of 150 mm along (x, y, z), any vector keeps it alive,
`.` stops it, a different direction needs a stop and a new start. Only X, Y, Z (the rotations are always 0). See _ask() for how
the sticks make one, and DIR_TOLERANCE_DEG / DIR_CHANGE_S for when a running vector is stopped and replaced.

A controller abort (jogd-c, built 6120 bytes and later): a "Path error" at a joint limit, a collision or a Ctrl-Z no longer kills jogd-c.
It ends the move, re-arms itself, prints ONE line `A <code> <text>` (or `A abort`; up to about 1.1 s later in the controller's 1000 ms
lock-out) and reads on, ignoring repeats of the command that was in force until a `.` or another command. `_on_abort`: the move is over
(no `.` is owed, no emergency timer, no dead state, no restart), the notice says "the controller stopped the move: <text>" (plus the
front-panel advice when a recent controller line said `Arm power is OFF`), the direction is blocked until the sticks are let go (as for `E`),
and a single `.` goes out before the next start (`_rearm`). After its own Ctrl-Z the driver waits CTRL_Z_WAIT_S for the old death signs
or an `A` line (jogd-c only: the plain jogd has no handler); `A` means jogd-c is alive and is not restarted.

Not done here: soft limits (jogd knows none; zero_at only gives true_joints()), fine steps (L2: the stick
simply gives low speeds), the status/home-switch display (state()["home_switches"] is None).
"""
import collections
import datetime
import math
import os
import re
import threading
import time

import numpy as np

from . import aci, calfile, console, crs, homezc, model
from .sim import RobotError

_PROGRAMS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "controller", "programs")
REMOTE = "/tmp/jogd"                  # (local path, remote path) of jogd, used when only the joint frame is switched on ...
LOCAL = os.path.join(_PROGRAMS, "jogd")
REMOTE_C = "/tmp/jogd-c"              # ... and of jogd-c, used for every frame when world or tool is switched on
LOCAL_C = os.path.join(_PROGRAMS, "jogd-c")
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
STOP_WAIT_CART_S = 1.5            # the same in world and tool frames: jogd-c's `-` came 0.53 to 0.65 s after `.` on the arm (2026-11-25)
RESTART_WAIT_S = 2.5              # after a Ctrl-Z the controller refuses moves for about 2 s: nothing is started sooner
RESTART_PROMPT_S = 3.0            # the `$` prompt must show this soon after the single `\r` that starts a restart
MAX_RESTARTS = 3                  # per failure streak
POSITION_EVERY_S = 0.250          # `p` this often while idle ...
POSITION_MOVING_EVERY_S = 0.500   # ... and this often while a move is in force (each `p` holds jogd's loop for about 0.1 s)
DEAD_SIGNS = ("killed by SIG", "^C equivalent")    # lines the controller prints when the program is killed (seen on the arm)
REPORT_DELAY_S = 0.3              # after a move has stopped, read the position this much later for the "stopped at" line
NO_POSITION_S = 1.0               # no `P` line this long after the first `p`: say live position is unavailable
QUIT_WAIT_S = 5.0                 # `$` prompt expected this soon after `q`
CTRL_Z_WAIT_S = 2.5               # after Ctrl-Z in the shutdown escalation, before Ctrl-E; and in jogd-c, for the death signs or an `A` line
DIR_TOLERANCE_DEG = 25.0          # a running vector move: the sticks must differ from the direction sent by more than this ...
DIR_CHANGE_S = 0.3                # ... continuously for this long before it is stopped and started again in the new direction
SELF_END_MIN_S = 0.25             # a move that ends by itself sooner than this after its `+` is a refusal (a limit): not started again until the stick is let go
VEC = 6                           # the "axis" of a command tuple that is an `@` vector; its "sign" is then the (x, y, z) digits
POLL_S = 0.002

# Joint-limit guard (world and tool frames only; in joint frame a joint just stops at its limit). The controller aborts a
# straight-line move that needs a joint beyond its travel ("Error 0x4000006 / Path error") and KILLS the jog program, so the
# PC stops the move first. This table is the guard's own copy (model.LIMITS_DEG is the model's): correct it here. J3's real
# limit is about +/-114 on this arm (seen 2026-11-25); 112 until the controller's value is looked up.
GUARD_LIMITS_DEG = ((-175.0, 175.0), (-90.0, 90.0), (-112.0, 112.0), (-180.0, 180.0), (-105.0, 105.0), (-180.0, 180.0))
GUARD_MARGIN_DEG = 4.0            # a joint is "near" a limit when it is within this of it, or would be GUARD_LOOKAHEAD_S from now ...
GUARD_LOOKAHEAD_S = 0.8           # ... (a stop takes about 0.6 s on this controller)
GUARD_MIN_SPEED_DPS = 0.5         # a joint counts as moving toward a limit only above this speed (position quantisation is ~0.004 degree)
GUARD_SHARE = 0.05                # at a start, a joint takes part in the move when its share of the largest joint motion is at least this
GUARD_MIN_SV_RATIO = 0.01         # Jacobian (translation in metres) with a smaller smallest/largest singular value is "near singular": no start check
CTRL_Z, CTRL_E = console.CTRL_Z, console.CTRL_E

# Stick deflection to speed. The pad's value for the picked joint (already shaped by gamepad.shape) goes from
# crs.STICK_ON (where a jog starts) to 1.0. That span is rescaled to 0..1 and raised to this power, so the
# bottom of the travel has fine control (speeds 1, 2, 3 ... percent) and the top is steep. 1 = linear.
SPEED_CURVE_EXPONENT = 1.5
MIN_SPEED_PCT = 1

COUNTS_PER_DEGREE = calfile.COUNTS_PER_DEGREE

# The controller's manual mode, which jogd takes as its first argument (only when it starts): 1 joint, 3 world, 4 tool.
# Changing frame therefore quits jogd and starts it again (JogdArm.set_frame).
FRAME_MODE = {"joint": 1, "world": 3, "tool": 4}

# In world and tool frames the command letter is still the axis number 1..6. FRAME_AXES says which jogd axis (1..6) and
# which sign a component of the pad's six-vector (gamepad.jog_vector order) is sent as: vec index -> (jogd axis, sign).
# World and tool take the pad's [x, y, z, roll, pitch, yaw] to jogd-c's letters A X, B Y, C Z, D yaw, E pitch, F roll.
# THE SIGNS ARE TO BE CONFIRMED BY A BENCH RUN on the arm; correct them here. (The first three must stay jogd axes 1 to 3: `@` vectors are x, y, z.)
FRAME_AXES = {
    "joint": [(1, +1), (2, +1), (3, +1), (4, +1), (5, +1), (6, +1)],
    "world": [(1, +1), (2, +1), (3, +1), (6, +1), (5, +1), (4, +1)],
    "tool": [(1, +1), (2, +1), (3, +1), (6, +1), (5, +1), (4, +1)],
}

DEFAULT_JOINT_CAPS = {2: 25, 3: 25}        # percent, per joint 1..6; a joint not listed is capped only by max_speed
DEFAULT_FRAME_CAPS = {"world": 15, "tool": 15}    # percent; until the real speed in these frames has been measured
MESSAGE_WINDOW_S = 1.0                     # controller lines this close before a kill sign are reported with it
MESSAGES_KEPT = 40
SPEED_STALE_S = 1.5                        # joint_speeds are zero when the newest position sample is older than this
POWER_OFF_ADVICE = "arm power is off: turn it on at the controller's front panel"
_ECHO = re.compile(r"^(?:[A-Fa-f]\d\d|@[+\d-]+|[.pq]|w2|PULSES.*|JOINTS.*|[\s\d.+-]*)$")


def _check_cap(value, what):
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= TOP_SPEED:
        raise RobotError(f"{what}: cap must be a whole number 1 to {TOP_SPEED}, not {value!r}")
    return value


def parse_joint_caps(text):
    """`2=25,3=25` -> {2: 25, 3: 25}; `none` -> {}. ValueError (with a message) when wrong."""
    text = text.strip()
    if text.lower() == "none":
        return {}
    caps = {}
    for part in text.split(","):
        key, eq, value = part.partition("=")
        if not eq or not key.strip().isdigit() or not value.strip().isdigit():
            raise ValueError(f"{part!r}: expected JOINT=PERCENT, like 2=25")
        joint, percent = int(key), int(value)
        if not 1 <= joint <= 6:
            raise ValueError(f"joint {joint}: joints are 1 to 6")
        if not 1 <= percent <= TOP_SPEED:
            raise ValueError(f"cap {percent}: must be 1 to {TOP_SPEED}")
        caps[joint] = percent
    return caps


def parse_frame_caps(text):
    """`world=15,tool=15` -> {"world": 15, "tool": 15} (a frame left out keeps its default). ValueError when wrong."""
    caps = dict(DEFAULT_FRAME_CAPS)
    for part in text.strip().split(","):
        key, eq, value = part.partition("=")
        key = key.strip().lower()
        if not eq or not value.strip().isdigit():
            raise ValueError(f"{part!r}: expected FRAME=PERCENT, like world=15")
        if key not in DEFAULT_FRAME_CAPS:
            raise ValueError(f"frame {key!r}: use one of {', '.join(DEFAULT_FRAME_CAPS)}")
        if not 1 <= int(value) <= TOP_SPEED:
            raise ValueError(f"cap {value}: must be 1 to {TOP_SPEED}")
        caps[key] = int(value)
    return caps

LINE_PATTERNS = (                 # searched at the END of a line, so an echoed command in front of a line does no harm
    ("E", re.compile(r"E (-?\d+) (.*)$")),
    ("A", re.compile(r"(?<![A-Za-z0-9+@])A (?:(-?\d+) (.*)|abort)$")),
    ("P", re.compile(r"(?<![A-Za-z])P((?:\s+[-+]?\d+){6})\s*$")),
    ("ready", re.compile(r"jogd ready$")),
    ("+", re.compile(r"\+@(?:[+-]\d){6}\d\d$")),
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
    if axis == VEC:                          # sign is the (x, y, z) digits: `@+9+6+0+0+0+002`
        return ("@" + "".join(f"{'-' if c < 0 else '+'}{abs(c)}" for c in (*sign, 0, 0, 0)) + f"{int(speed):02d}").encode()
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
                 local=LOCAL, remote=REMOTE, upload=None, local_c=LOCAL_C, remote_c=REMOTE_C, respeed=True, stick_speed=True,
                 joint_caps=None, frame_caps=None, wall=time.time, frames=None, limit_guard=True, guard_limits=None):
        if not 1 <= max_speed <= TOP_SPEED:
            raise RobotError(f"max_speed must be 1 to {TOP_SPEED}")
        self.joint_caps = {int(j): _check_cap(c, f"joint {j}") for j, c in (DEFAULT_JOINT_CAPS if joint_caps is None else joint_caps).items()}
        if not all(1 <= j <= 6 for j in self.joint_caps):
            raise RobotError("joint caps are for joints 1 to 6")
        self.frame_caps = {f: _check_cap(c, f"{f} frame") for f, c in (DEFAULT_FRAME_CAPS if frame_caps is None else frame_caps).items()}
        for f in ("world", "tool"):
            self.frame_caps.setdefault(f, DEFAULT_FRAME_CAPS[f])
        self.limit_guard = bool(limit_guard)
        self.guard_limits = tuple((float(lo), float(hi)) for lo, hi in (GUARD_LIMITS_DEG if guard_limits is None else guard_limits))
        if len(self.guard_limits) != 6 or any(lo >= hi for lo, hi in self.guard_limits):
            raise RobotError("guard_limits needs six (low, high) pairs")
        self._guard_notice = None           # the notice text the guard set (cleared when a move starts)
        self._guard_joint = None            # 0-based joint the guard has stopped / refused (until the stick is let go)
        self._wall = wall                   # time.time() by default: the stamps in state() ("position_time", controller_message["at"])
        self.frames = tuple(FRAME_MODE) if frames is None else tuple(frames)   # the frames the pad may use
        if "joint" not in self.frames or any(f not in FRAME_MODE for f in self.frames):
            raise RobotError(f"frames must include joint and come from {', '.join(FRAME_MODE)}")
        self._cart = any(f in ("world", "tool") for f in self.frames)
        if self._cart:                      # jogd-c does every frame (mode 1 is jogd's)
            local, remote = local_c, remote_c
        self.frame = "joint"
        self._frame_req = None              # a frame change asked for and not done yet
        self._need_release = False          # after a frame change: no jog starts until the pad has reported no deflection
        self._death_extra = ""
        self._launch_state = None           # "starting" / "waiting for the front-panel button" while _launch runs
        self._msgs = collections.deque(maxlen=MESSAGES_KEPT)   # (clock, text): controller lines that are not jogd protocol
        self.controller_message = None
        self._samples = []                  # [(wall time, joints)] of the newest two `P` lines
        self._last_target = None
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
        self._dir_since = None              # since when the sticks have differed from the vector in force by more than DIR_TOLERANCE_DEG
        self._rearm = False                 # jogd-c ended a move: the next start is preceded by `.` (it ignores the command it last started)
        self._plus_at = -1e9                # when the `+` of the move in force came
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
        self._blocked = None                # (axis, sign) that came back with E (or A): not sent again until the stick is released
        self._abort_notice = None           # the notice text the last `A` line set (the power advice may be added to it)
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

    def _launch(self, frame=None):
        """Type the start command (in the mode of `frame`, default the current one) and wait for `jogd ready` (with the
        button handling). None if it came up, else why not (the controller has then been left at `$` as far as possible)."""
        self._launch_state = "starting"
        try:
            return self._launch_inner(frame or self.frame)
        finally:
            self._launch_state = None

    def _launch_inner(self, frame):
        link = self.link
        command = f"{self._remote} {FRAME_MODE[frame]} {self.max_speed} s"
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
                self._launch_state = "waiting for the front-panel button"
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

    def _reset_since(self, line_start=False):
        """Forget what has been received. line_start: what comes next begins a line, so a bare `$ ` counts as the prompt.
        Measured on the controller 2026-11-25: after `q` with nothing moving, jogd prints nothing and the shell answers
        with `$ ` alone, no line end before it."""
        self._since = b"\n" if line_start else b""

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
            text = raw.decode("latin-1").replace("\r", "")
            kind, found = classify(text)
            if kind == "other" and not found:
                continue
            self._note_message(now, kind, found, text)
            self._lines.append((now, kind, found))
        return True

    def _note_message(self, now, kind, found, text):
        """Keep a complete line from the controller that is not jogd protocol (`E` lines count) for the death report."""
        if kind in ("E", "A"):
            line = text.strip()
        elif kind == "other" and not any(sign in found for sign in DEAD_SIGNS):
            line = re.sub(r"^\$\s*", "", found.strip())
            if (not line or line.endswith("$") or _ECHO.match(line) or line.startswith((self._remote, "cksum"))
                    or homezc.BUTTON in line):          # the button request has its own say() line
                return
        else:
            return
        self._msgs.append((now, line))
        self.controller_message = {"text": line, "at": self._wall()}

    def _recent_messages(self, now):
        return [t for at, t in self._msgs if now - MESSAGE_WINDOW_S <= at <= now + 0.5]

    def _wait_prompt(self, seconds, abort_ok=False):
        """True when the `$` prompt shows within `seconds`; with abort_ok, "abort" when an `A` line comes first."""
        end = self._clock() + seconds
        while True:
            self._feed()
            if abort_ok and any(kind == "A" for _, kind, _ in self._lines):
                self._lines = []
                return "abort"
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

    def set_frame(self, frame):
        """Change the controller's manual mode (joint, world, tool). jogd takes the mode only when it starts, so the thread
        waits until no move is in force, writes `q`, and starts jogd again. jog() calls are ignored meanwhile, and after it
        none starts until the pad has reported no deflection."""
        if frame not in FRAME_MODE:
            raise RobotError(f"unknown frame {frame!r}: use one of {', '.join(FRAME_MODE)}")
        if frame not in self.frames:
            self._refuse_frame(frame)
            return
        with self._lock:
            if self._frame_req is None and frame == self.frame:
                return
            self._frame_req = frame
            self._want, self._pick = None, None
            self._need_release = True
        self.notice = f"changing to {frame.upper()} frame"

    def _refuse_frame(self, frame):
        """The pad is in a frame that is switched off here (--frames): nothing moves, and the notice says why."""
        with self._lock:
            self._want, self._pick = None, None
            self._need_release = True
        self.notice = (f"{frame.upper()} frame is switched off: nothing moves until the pad is back in "
                       f"{' or '.join(f.upper() for f in self.frames)} (Options)")

    def jog(self, frame, vec, ttl=0.3, source="rest", aux=(0.0, 0.0)):
        vec = [float(v) for v in vec]
        if self._frame_req is not None:
            return                              # a frame change is under way: the pad is not listened to
        if frame not in FRAME_MODE:
            if any(vec):
                self._release()
                raise RobotError(f"frame {frame!r} is not available here: use one of {', '.join(FRAME_MODE)}")
            return
        if frame not in self.frames:
            self._refuse_frame(frame)           # nothing moves while the pad is in a frame that is switched off
            return
        if self.notice.endswith("(Options)"):
            self.notice = ""                    # the pad is back in a frame that is switched on
        if frame != self.frame:
            self.set_frame(frame)               # this jog is dropped
            return
        if self._need_release:
            if max(abs(v) for v in vec) < crs.STICK_OFF:
                self._need_release = False      # the stick has been let go once since the change
            return
        eff = [0.0] * 6                         # the pad's components, put where jogd's axes are
        for value, (axis, sign) in zip(vec, FRAME_AXES[frame]):
            eff[axis - 1] = value * sign
        vec = eff
        if frame != "joint":
            self._jog_cart(eff, ttl, source)
            return
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

    # ---- world and tool frames: one axis as a letter, or a straight line along several as an `@` vector ----------------

    @staticmethod
    def _ask(eff):
        """What the sticks ask for to START a move: (want, deflection, pick key) or None. Components below crs.STICK_ON count
        as zero. If the largest deflection is a translation and at least two of x, y, z are past the threshold: a vector,
        the three scaled so the largest is 9 and rounded to digits (`want` = (VEC, digits), deflection = the largest, pick key =
        the signs); else one axis, the largest (a rotation is never part of a vector): `want` = (axis index, sign)."""
        big = max(range(6), key=lambda i: abs(eff[i]))
        if abs(eff[big]) < crs.STICK_ON:
            return None
        if big < 3:
            comps = [v if abs(v) >= crs.STICK_ON else 0.0 for v in eff[:3]]
            if sum(1 for v in comps if v) >= 2:
                peak = max(abs(v) for v in comps)
                digits = tuple(int(round(v / peak * 9)) for v in comps)
                return (VEC, digits), peak, ("V", tuple((v > 0) - (v < 0) for v in comps))
        sign = 1 if eff[big] > 0 else -1
        return (big, sign), abs(eff[big]), ("L", big, sign)

    @staticmethod
    def _held(current, eff):
        """The move in force is still wanted: its stick is past crs.STICK_OFF (a vector: any of x, y, z)."""
        if current[0] == VEC:
            return max(abs(v) for v in eff[:3]) >= crs.STICK_OFF
        return abs(eff[current[0]]) >= crs.STICK_OFF and (eff[current[0]] > 0) == (current[1] > 0)

    @staticmethod
    def _deflection(current, eff):
        if current[0] == VEC:
            return max([abs(eff[i]) for i, d in enumerate(current[1]) if d] or [0.0])
        return abs(eff[current[0]])

    @staticmethod
    def _direction(want):
        """The (x, y, z) direction of a want, or None for a rotation."""
        if want[0] == VEC:
            return tuple(float(d) for d in want[1])
        if want[0] < 3:
            return tuple(float(want[1]) if i == want[0] else 0.0 for i in range(3))
        return None

    def _differs(self, current, asked):
        """The sticks ask for something else than the move in force: for a letter only a vector (several axes) counts; for a vector
        another direction by more than DIR_TOLERANCE_DEG, or a rotation."""
        if current[0] != VEC and asked[0] != VEC:
            return False                        # one axis to one axis: the moving one is kept until its stick lets go
        a, b = self._direction(current), self._direction(asked)
        if a is None or b is None:
            return b is None and current[0] == VEC
        dot = sum(x * y for x, y in zip(a, b))
        norm = math.sqrt(sum(x * x for x in a)) * math.sqrt(sum(y * y for y in b))
        return math.degrees(math.acos(max(-1.0, min(1.0, dot / norm)))) > DIR_TOLERANCE_DEG

    def _jog_cart(self, eff, ttl, source):
        ask, now = self._ask(eff), self._clock()
        with self._lock:
            current = self._want
        want, mag, start = None, 0.0, None
        if current is not None and self._held(current, eff):
            want, mag = current, self._deflection(current, eff)
            if ask is not None and self._differs(current, ask[0]):
                if self._dir_since is None:
                    self._dir_since = now
                elif now - self._dir_since >= DIR_CHANGE_S:
                    self._dir_since = None
                    want, mag = ask[0], ask[1]
                    start = mag
                    self.say(f"  the sticks have moved away from the direction in force: stopping, then starting {self._describe(want)}")
            else:
                self._dir_since = None
        else:
            self._dir_since = None
            key = ask[2] if ask else None
            if key != self._pick:
                self._pick, self._pick_since = key, now
                self._pick_peak = ask[1] if ask else 0.0
            elif ask is not None:
                self._pick_peak = max(self._pick_peak, ask[1])
            if ask is not None and now - self._pick_since >= SPEED_PICK_S:
                want, mag, start = ask[0], ask[1], self._pick_peak
            elif ask is not None:
                if current is not None:
                    self._release()             # the old move is let go while the new one settles
                return
        if want is None:
            self._release()
            return
        with self._lock:
            self._want, self._want_mag = want, mag
            if start is not None:
                self._want_start = start
            self._want_until, self._source = self._clock() + min(float(ttl), 1.0), source

    @staticmethod
    def _describe(want):
        return ("vector " + " ".join(f"{d:+d}" for d in want[1])) if want[0] == VEC else f"axis {want[0] + 1} {'+' if want[1] > 0 else '-'}"

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
        cmd, target = self._cmd, self._last_target
        command = ({"kind": "jog", "source": source} if (wanted or cmd or self._moving or self._stopping_since is not None)
                   and not self._dead else None)
        if command is not None:
            shown = cmd or target
            vector = shown is not None and shown[0] == VEC
            command.update(axis=shown[0] + 1 if shown and not vector else None,
                           sign=shown[1] if shown and not vector else None,
                           vector=list(shown[1]) if vector else None,
                           speed=shown[2] if shown else None, frame=self.frame)
        samples = list(self._samples)
        speeds = [round(v, 3) for v in self._joint_speeds()]
        if self._launch_state == "waiting for the front-panel button":
            jogd_state = self._launch_state
        elif self._frame_req is not None and not self._dead:
            jogd_state = "changing frame"
        elif self._launch_state:
            jogd_state = self._launch_state
        else:
            jogd_state = "dead" if self._dead else "running"
        return {"backend": self.backend, "joints": list(self.joints), "true_joints": self.true_joints(),
                "speed": self.speed, "command": command,
                "moving": self._moving, "notice": self.notice, "mode": None if command is None else "continuous",
                "settling": self._stopping_since is not None or self._report_at is not None or self._report_wait,
                "home_switches": None, "frame": self.frame, "max_speed": self.max_speed,
                "joint_caps": {str(j): c for j, c in sorted(self.joint_caps.items())}, "frame_caps": dict(self.frame_caps),
                "joint_speeds": speeds, "position_time": samples[-1][0] if samples else None,
                "jogd": jogd_state, "controller_message": self.controller_message,
                "limit": self._limit_state(), "guard_limits": [list(l) for l in self.guard_limits] if self.limit_guard else None}

    def _joint_speeds(self):
        """Degrees per second from the last two position samples (zeros when there are not two, or the newest is stale)."""
        samples, now = list(self._samples), self._wall()
        if len(samples) == 2 and now - samples[-1][0] <= SPEED_STALE_S and samples[-1][0] > samples[0][0]:
            dt = samples[-1][0] - samples[0][0]
            return [(b - a) / dt for a, b in zip(samples[0][1], samples[-1][1])]
        return [0.0] * 6

    # ---- joint-limit guard (world and tool frames) ------------------------------------------------------

    def _guard_active(self):
        return self.limit_guard and self.frame in ("world", "tool")

    def _limit_state(self):
        """The joint with the least room to a limit among those inside GUARD_MARGIN_DEG of it (or past it), else None."""
        if not self.limit_guard:
            return None
        best = None
        for i, (angle, (lo, hi)) in enumerate(zip(self.joints, self.guard_limits)):
            for limit, room in ((lo, angle - lo), (hi, hi - angle)):
                if room < GUARD_MARGIN_DEG and (best is None or room < best[0]):
                    best = (room, i, limit, angle)
        if best is None:
            return None
        _, i, limit, angle = best
        return {"joint": i + 1, "limit": limit, "angle": round(angle, 2), "blocked": self._guard_joint == i}

    def _guard_message(self, joint, limit):
        text = f"J{joint + 1} is at its limit ({limit:g}): that direction is blocked; move another way"
        self._guard_joint, self._guard_notice, self.notice = joint, text, text
        self.say("  " + text)

    def _guard_watch(self, now):
        """A new position sample while a move is in force: stop it (`.`, the normal stop) when a joint that is moving toward a
        limit will be inside the margin of it GUARD_LOOKAHEAD_S from now."""
        cmd = self._cmd
        if not self._guard_active() or cmd is None or self._stopping_since is not None:
            return
        speeds = self._joint_speeds()
        for i, (angle, (lo, hi), v) in enumerate(zip(self.joints, self.guard_limits, speeds)):
            ahead = angle + v * GUARD_LOOKAHEAD_S
            if v > GUARD_MIN_SPEED_DPS and ahead > hi - GUARD_MARGIN_DEG:
                limit = hi
            elif v < -GUARD_MIN_SPEED_DPS and ahead < lo + GUARD_MARGIN_DEG:
                limit = lo
            else:
                continue
            was_moving = self._moving
            self._write(b".")
            self._after_write(now)
            self._blocked = cmd[:2]
            self._cmd, self._await_since, self._respeed_pending = None, None, None
            self._stopping_since = now if was_moving else None
            self._guard_message(i, limit)
            return

    def _guard_direction(self, want):
        """The 6-vector (x, y, z in mm, then angular, per unit) a want asks of the tool, in world coordinates, at the current pose."""
        _, rot, _, _ = model.forward(self.joints)
        v = np.zeros(6)
        if want[0] == VEC:
            d = np.array([float(c) for c in want[1]])
            d = d / np.linalg.norm(d)
            v[:3] = rot @ d if self.frame == "tool" else d
            return v
        axis, sign = want
        unit = np.zeros(3)
        if axis < 3:                                    # A, B, C: x, y, z
            unit[axis] = sign
            v[:3] = rot @ unit if self.frame == "tool" else unit
        else:                                           # D yaw (z), E pitch (y), F roll (x)
            unit[{3: 2, 4: 1, 5: 0}[axis]] = sign
            v[3:] = rot @ unit if self.frame == "tool" else unit
        return v

    def _guard_joint_motion(self, want):
        """Joint motion (degrees per unit of the move, six numbers) for a want at the current pose, or None when the Jacobian is
        near singular or anything fails (then only the look-ahead stop applies)."""
        try:
            jac = np.array(model.jacobian(self.joints), dtype=float)
            scaled = jac.copy()
            scaled[:3] /= 1000.0                        # metres, so that translation and rotation are comparable
            sv = np.linalg.svd(scaled, compute_uv=False)
            if not np.all(np.isfinite(sv)) or sv[0] <= 0 or sv[-1] / sv[0] < GUARD_MIN_SV_RATIO:
                return None
            dq = np.linalg.lstsq(jac, self._guard_direction(want), rcond=None)[0]
            return np.degrees(dq) if np.all(np.isfinite(dq)) else None
        except Exception:                               # noqa: BLE001 - never block or crash on the guard's account
            return None

    def _guard_refuses_start(self, want):
        """True (and the notice set) when starting `want` would push a joint that is already inside the margin further toward its limit."""
        dq = self._guard_joint_motion(want)
        if dq is None:
            return False
        biggest = float(np.max(np.abs(dq)))
        if biggest <= 0:
            return False
        for i, (angle, (lo, hi), d) in enumerate(zip(self.joints, self.guard_limits, dq)):
            if abs(d) < GUARD_SHARE * biggest:
                continue
            limit = hi if d > 0 else lo
            if (d > 0 and angle >= hi - GUARD_MARGIN_DEG) or (d < 0 and angle <= lo + GUARD_MARGIN_DEG):
                self._guard_message(i, limit)
                return True
        return False

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
            with self._lock:
                req, self._frame_req = self._frame_req, None
            if req is not None:
                self.frame = req                # the restart that is due starts jogd in the mode wanted
                if self.notice.startswith("changing to"):
                    self.notice = ""
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
            self._guard_joint = None
            if self._frame_req is not None and self._idle():
                self._change_frame(now)
                return
        if want != self._active_want:
            self._active_want, self._fresh, self._diff_since = want, True, None
        target = wanted = None
        if (want is not None and want != self._blocked and self._cmd is None and self._stopping_since is None
                and self._guard_active() and self._guard_refuses_start(want)):
            self._blocked = want                # the same direction stays refused until the sticks are let go
        if want is not None and want != self._blocked:
            full = max(MIN_SPEED_PCT, min(self.speed, self.max_speed, self._cap_for(want[0])))
            if self.stick_speed:
                wanted = speed_for(mag, full)
                target = (want[0], want[1], speed_for(start, full) if self._fresh else wanted)
            else:                               # direction only: the speed is the pad's speed setting
                wanted = max(MIN_SPEED_PCT, int(full))
                target = (want[0], want[1], wanted)
        self._last_target = target
        self._drive(target, now, wanted)
        if self._respeed_pending is not None and now - self._respeed_pending >= RESPEED_WAIT_S:
            self.say(f"  no `+` within {RESPEED_WAIT_S:.0f} s of the speed change: the move is taken as stopped (it is started again if the stick is held)")
            self._respeed_pending, self._cmd, self._moving, self._await_since = None, None, False, None
            self._report_at = now + REPORT_DELAY_S
        stop_wait = STOP_WAIT_S if self.frame == "joint" else STOP_WAIT_CART_S
        if self._stopping_since is not None and now - self._stopping_since >= stop_wait:
            self._write(CTRL_Z)
            self._last_ctrlz = now
            self.say(f"  !!! NO `-`, `T` OR `E` WITHIN {stop_wait} s OF `.`: Ctrl-Z WRITTEN (emergency stop; it kills jogd)")
            if self._cart and self._await_abort_line():
                return                          # jogd-c survived it: its `A` line has been handled, nothing is restarted
            self._mark_dead(now, f"no answer to `.` within {stop_wait} s; Ctrl-Z written", loud=False)
            return
        self._positions(now)

    def _cap_for(self, axis):
        """The speed cap (percent) for a jogd axis index 0..5 in the current frame."""
        if self.frame == "joint":
            return self.joint_caps.get(axis + 1, TOP_SPEED)
        return self.frame_caps.get(self.frame, TOP_SPEED)

    def _await_abort_line(self):
        """After a Ctrl-Z to jogd-c (which no longer dies of it): read for up to CTRL_Z_WAIT_S. True when an `A` line came (jogd-c is
        alive, handled as any abort); False on the old death signs, the `$` prompt, or nothing (the caller carries on as for a dead jogd)."""
        end = self._clock() + CTRL_Z_WAIT_S
        while not self._closing:
            self._feed()
            lines, self._lines = self._lines, []
            for _, kind, found in lines:
                if kind == "A":
                    self._on_abort(found, self._clock(), after_ctrlz=True)
                    return True
                if kind == "other" and any(sign in found for sign in DEAD_SIGNS):
                    return False
            if b"\n" in self._since and self._at_prompt():
                return False
            if self._clock() >= end:
                return False
            self._sleep(POLL_S)
        return False

    def _idle(self):
        return (self._cmd is None and not self._moving and self._stopping_since is None and self._await_since is None
                and self._respeed_pending is None)

    def _change_frame(self, now):
        """Nothing is moving or wanted: `q`, wait for `$`, start jogd in the new mode. Blocks the thread (nothing is written
        in between); a failure goes to the dead / restart handling."""
        with self._lock:
            target = self._frame_req
        if target == self.frame:
            with self._lock:
                self._frame_req = None
            self.notice = ""
            return
        self.say(f"  changing to the {target.upper()} frame: `q`, then jogd {FRAME_MODE[target]}")
        self._report_at, self._report_wait = None, False
        self._reset_since(line_start=True)
        self._write(b"q")
        if not self._wait_prompt(QUIT_WAIT_S):
            with self._lock:
                self._frame_req = None
            self.say(f"  !!! FRAME CHANGE FAILED: no `$` prompt within {QUIT_WAIT_S:.0f} s of `q`")
            self._mark_dead(self._clock(), f"frame change to {target} failed: no `$` prompt after `q`")
            return
        self.link.ahead = b""
        why = self._launch(target)
        with self._lock:
            if self._frame_req == target:
                self._frame_req = None
        if why:
            self.frame = target                 # the restart tries the mode that was asked for
            self.say(f"  !!! FRAME CHANGE FAILED: {why}")
            self._mark_dead(self._clock(), f"frame change to {target} failed: {why}")
            return
        self.frame = target
        self._cmd, self._moving, self._stopping_since, self._blocked = None, False, None, None
        self._await_since, self._respeed_pending = None, None
        self.notice = ""
        self.say(f"  jogd is running in the {target.upper()} frame (mode {FRAME_MODE[target]}); no jog starts until the stick has been let go")

    def _mark_dead(self, now, text, loud=True):
        """jogd is not running (or must be treated so). From here nothing is written until it has been restarted."""
        said_by = self._recent_messages(now)
        self._msgs.clear()
        self._dead = True
        self._death_extra = ""
        self._cmd, self._moving, self._await_since, self._stopping_since = None, False, None, None
        self._respeed_pending = None
        self._report_at, self._report_wait = None, False
        self._auto, self._released = True, False
        extra = ""
        if said_by:
            extra = "controller said: " + " | ".join(said_by)
            if any("Arm power is OFF" in line for line in said_by):
                extra += ". " + POWER_OFF_ADVICE
            self.controller_message = {"text": " | ".join(said_by), "at": self._wall()}
        self._death_extra = extra
        self.notice = "jogd is not running: " + text.strip() + (". " + extra if extra else "")
        if loud:
            self.say(f"  !!! JOGD IS NOT RUNNING ({text.strip()}): nothing is written until it has been restarted")
        if extra:
            self.say("  " + extra)
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
        self.notice = ("jogd was restarted. " + self._death_extra) if self._death_extra else ""    # until the next move starts
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
        if self.respeed and wanted is not None and self.frame == "joint":      # jogd-c fixes the speed per move
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
        if self._rearm:                         # jogd-c ignores the command of a move that has ended: `.` forgets it (prints nothing)
            self._rearm = False
            self._write(b".")
            self._after_write(now)
        if self._guard_notice is not None and self.notice == self._guard_notice:
            self.notice = ""
        if self._abort_notice is not None and self.notice.startswith(self._abort_notice):
            self.notice = ""
        self._abort_notice = self._guard_notice = None
        self._write(command_bytes(*target))
        self._after_write(now, move=True)
        self._await_since, self._last_change = now, now
        self._cmd, self._last_key = target, target[:2]

    def _on_line(self, kind, found, now):
        if kind == "other":
            if any(sign in found for sign in DEAD_SIGNS):
                self._mark_dead(now, found)
            elif ("Arm power is OFF" in found and self._abort_notice is not None and self.notice == self._abort_notice
                  and POWER_OFF_ADVICE not in self.notice):
                self._abort_notice = self.notice = self.notice + ". " + POWER_OFF_ADVICE      # its line came after the `A` line
                self.say("  " + POWER_OFF_ADVICE)
            return                              # echo, or text jogd does not define: it is in the log
        if self._dead:
            return
        if kind == "+":
            self.controller_message = None
            if self.notice.startswith("jogd was restarted"):
                self.notice = ""
            self._moving, self._await_since, self._respeed_pending = True, None, None
            self._plus_at = now
            if self._stopping_since is not None:
                self._stopping_since = now      # the answer to a command written just before the `.`: its `-` follows
        elif kind == "-":
            if self._respeed_pending is not None and self._stopping_since is None:
                self._moving = False            # the stop half of a speed change: its `+` follows, this is not the end of the jog
                return
            if self._moving and self._stopping_since is None and self._cmd is not None and self.frame != "joint":
                self._self_ended(now)           # no `.` in flight: jogd-c's move reached its length, or the controller ended it
                return
            if self._moving or self._stopping_since is not None:
                self._report_at = now + REPORT_DELAY_S
            self._moving, self._stopping_since = False, None
        elif kind == "A":
            self._on_abort(found, now)
        elif kind == "T":
            self._moving, self._cmd, self._await_since, self._stopping_since = False, None, None, None
            self._respeed_pending = None
            self._rearm = self.frame != "joint"
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
            self._samples = (self._samples + [(self._wall(), list(self.joints))])[-2:]
            self._guard_watch(now)
            if self._report_wait:
                self._report_wait = False
                self._say_position("stopped at")

    def _on_abort(self, found, now, after_ctrlz=False):
        """jogd-c's `A <code> <text>` / `A abort`: the controller aborted the move; jogd-c has ended it, re-armed itself and reads on."""
        line = found.group(0)
        text = (found.group(2) or "").strip() or "abort"
        in_force = self._cmd is not None or self._moving or self._await_since is not None or self._stopping_since is not None
        self.controller_message = {"text": line, "at": self._wall()}
        if after_ctrlz:
            self.say("  jogd-c is alive after Ctrl-Z (it printed an `A` line): it is not restarted")
        if not in_force:
            self.notice = f"the controller reported: {line} (no move was in force)"
            self.say("  " + self.notice)
            return
        said_by = [t for t in self._recent_messages(now) if t != line]
        notice = f"the controller stopped the move: {text}"
        if any("Arm power is OFF" in t for t in said_by):
            notice += ". " + POWER_OFF_ADVICE
        self._blocked = self._cmd[:2] if self._cmd else self._last_key
        self._cmd, self._moving, self._await_since, self._respeed_pending = None, False, None, None
        self._stopping_since = None             # a `.` in flight is answered by this: the stop is complete
        self._rearm = True                      # one `.` before the next start: jogd-c ignores repeats of the command that was in force
        self._report_at = now + REPORT_DELAY_S
        self.notice = self._abort_notice = notice
        self.say(f"  {notice}  (`{line}`; the stick must be released before this direction is tried again)")

    def _self_ended(self, now):
        """A `-` nobody asked for while a move was in force. Not an error: nothing is stopping, no dead state. If the stick still
        asks for the same jog the next pass starts it again (after a `.`, see _send), unless it ended almost at once."""
        self.say("  move ended by itself")
        self._moving, self._cmd, self._await_since = False, None, None
        self._rearm = True
        self._report_at = now + REPORT_DELAY_S
        if now - self._plus_at < SELF_END_MIN_S:
            self._blocked = self._last_key      # a refusal (a limit?) and not a finished move: not retried in a loop
            self.notice = "the controller ended the move at once (a limit?); release the stick to try again"
            self.say("  " + self.notice)

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
            self._reset_since(line_start=True)
            self._write(b"q")
            ok = self._wait_prompt(QUIT_WAIT_S)
        if not ok:
            self.say(f"  NO `$` PROMPT WITHIN {QUIT_WAIT_S:.0f} s OF `q`: Ctrl-Z to stop the arm, then Ctrl-E to kill jogd")
            self._write(CTRL_Z)
            self._last_ctrlz = self._clock()
            ok = self._wait_prompt(CTRL_Z_WAIT_S, abort_ok=self._cart)
            if ok == "abort":                   # jogd-c survived Ctrl-Z (it printed `A`): leave it properly
                self.say("  jogd-c is alive after Ctrl-Z (it printed an `A` line): `q`")
                self._reset_since(line_start=True)
                self._write(b"q")
                ok = self._wait_prompt(QUIT_WAIT_S)
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
