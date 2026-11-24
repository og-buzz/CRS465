"""PlayStation controller on this machine -> jog commands for the robot.

Nothing starts unless L1 is held (the enable switch, like a teach pendant's
live-man switch). Jogging lasts only while L1 stays held: each jog command lives
for 0.25 s and is refreshed 50 times a second, so releasing L1, unplugging the
controller or stopping this process all end it. A move started with L1 and a
button (straight up, home, cycle start) carries on by itself after the buttons are
let go; Square stops everything, with or without L1. Holding R1 hands both sticks
to the web page's 3D view.

A wireless controller can go silent (out of range, flat battery) while the kernel
still shows its last stick position, for up to 20 s over Bluetooth. So the controller's
motion-sensor node, which reports about 200 times a second, is watched as a heartbeat:
0.3 s of silence stops whatever the controller started, and L1 must then be let go
and held again.

Button and axis codes are the ones the kernel's hid-playstation driver reports
(read from a DualShock 4 over USB and over Bluetooth).
"""
import asyncio
import glob
import os
import time

import evdev
from evdev import ecodes as e

from .sim import FRAMES, RobotError

STICKS = {e.ABS_X: "lx", e.ABS_Y: "ly", e.ABS_RX: "rx", e.ABS_RY: "ry"}
TRIGGERS = {e.ABS_Z: "l2", e.ABS_RZ: "r2"}
HATS = {e.ABS_HAT0X: ("dpad_left", "dpad_right"), e.ABS_HAT0Y: ("dpad_up", "dpad_down")}   # (negative, positive)
BUTTONS = {
    e.BTN_SOUTH: "cross", e.BTN_EAST: "circle", e.BTN_NORTH: "triangle", e.BTN_WEST: "square",
    e.BTN_TL: "l1", e.BTN_TR: "r1", e.BTN_TL2: "l2", e.BTN_TR2: "r2",
    e.BTN_SELECT: "share", e.BTN_START: "options", e.BTN_MODE: "ps",
    e.BTN_THUMBL: "l3", e.BTN_THUMBR: "r3",
}
DEADZONE = 0.12
TRIGGER_DEADZONE = 0.05
JOG_TTL_S = 0.25
TICK_S = 0.02
SPEED_RAMP_PCT_S = 30.0    # speed change per second with a trigger fully pulled
SAFE_SPEED = 10            # percent, set before "point straight up"
MESSAGE_TICKS = 100        # a refusal stays on the page for two seconds
STALE_S = 0.3              # controller silent this long = signal lost (longest gap measured over Bluetooth: 20 ms)

# The legend the web page shows: rows before and after the frame-specific ones.
HELP = {
    "before": [["L1", "hold to jog; with a button it starts a move that then runs by itself"],
               ["Square", "stop everything (no L1 needed)"],
               ["R1", "hold: right stick orbits the 3D view, left stick up / down zooms"],
               ["Options", "next frame (joint / world / tool)"]],
    "joint": [["Left stick", "J1 waist (left/right), J2 shoulder (up/down)"],
              ["Right stick", "J4 wrist rotate (left/right), J3 elbow (up/down)"],
              ["D-pad", "J5 wrist pitch (up/down), J6 tool roll (left/right)"]],
    "world": [["Left stick", "X forward/back (up/down), Y (left/right)"],
              ["Right stick", "Z (up/down), yaw (left/right)"],
              ["D-pad", "pitch (up/down), roll (left/right)"]],
    "after": [["L1 + L2 / R2", "track back / forward"],
              ["L1 + R1 + L2 / R2", "speed down / up"],
              ["L1 + R1 + D-pad left / right", "carousel"],
              ["L1 + R1 + Circle", "arm power on / off"],
              ["L1 + Circle / Cross", "gripper open / close"],
              ["L1 + Triangle", f"point straight up at {SAFE_SPEED}% speed"],
              ["L1 + L3 / R3 (stick click)", "home / run the pick-and-place cycle"]],
}


def shape(value):
    """Dead zone, then a squared curve so small stick movements give fine control."""
    if abs(value) < DEADZONE:
        return 0.0
    scaled = (abs(value) - DEADZONE) / (1.0 - DEADZONE)
    return scaled * scaled * (1.0 if value > 0 else -1.0)


def camera_vector(axes, buttons):
    """While R1 is held: [orbit right, orbit down, zoom in] for the 3D view. Otherwise zeros."""
    if "r1" not in buttons:
        return [0.0, 0.0, 0.0]
    return [shape(axes.get("rx", 0.0)), shape(axes.get("ry", 0.0)), -shape(axes.get("ly", 0.0))]


def jog_vector(frame, axes, buttons=frozenset()):
    """Six jog fractions for the frame. Stick up and stick left read negative."""
    # While R1 is held the sticks belong to the 3D view, not the arm.
    lx, ly, rx, ry = (0.0 if "r1" in buttons else shape(axes.get(k, 0.0)) for k in ("lx", "ly", "rx", "ry"))
    pitch_up = float(("dpad_up" in buttons) - ("dpad_down" in buttons))
    # With R1 held, D-pad left / right belongs to the carousel (see aux_vector).
    roll = 0.0 if "r1" in buttons else float(("dpad_right" in buttons) - ("dpad_left" in buttons))
    if frame == "joint":
        return [-lx, -ly, -ry, rx, pitch_up, roll]
    # world / tool: x, y, z, then rotation about x (roll), y (pitch, up is negative), z (yaw)
    return [-ly, -lx, -ry, roll, -pitch_up, -rx]


def trigger_pull(axes):
    """R2 minus L2, each 0..1 with a small dead zone: positive means R2."""
    l2, r2 = (v if v >= TRIGGER_DEADZONE else 0.0 for v in (axes.get("l2", 0.0), axes.get("r2", 0.0)))
    return r2 - l2


def aux_vector(axes, buttons):
    """[track, carousel] jog fractions. Triggers drive the track; with R1 held the D-pad drives the carousel."""
    if "r1" in buttons:
        return [0.0, float(("dpad_right" in buttons) - ("dpad_left" in buttons))]
    return [trigger_pull(axes), 0.0]


class GamepadJog:
    """Turns controller state into robot commands. No device access, so it is testable."""

    def __init__(self, robot):
        self.robot = robot
        self.frame = "joint"
        self.enabled = False
        self.message = ""
        self._message_ticks = 0
        self._held = set()
        self._parked = set()         # sticks that were steering the view and have not re-centred yet
        self._speed_change = 0.0     # fraction of a percent carried between ticks
        self._rearm = False          # after a lost signal or a reconnect: L1 must be let go before it enables again

    def update(self, axes, buttons):
        """One 20 ms tick: act on what is pressed and held right now."""
        pressed = buttons - self._held
        self._held = set(buttons)
        if self._message_ticks:
            self._message_ticks -= 1
            if not self._message_ticks:
                self.message = ""
        l1, r1 = "l1" in buttons, "r1" in buttons
        if self._rearm:
            if l1:
                l1 = False
                self._say("release L1, then hold it again")
            else:
                self._rearm = False
        robot = self.robot
        try:
            if "square" in pressed:                           # stop: never needs the enable, stops anyone's move
                robot.halt()
            if "options" in pressed:
                self.frame = FRAMES[(FRAMES.index(self.frame) + 1) % len(FRAMES)]
            acts = pressed & {"circle", "cross", "triangle", "l3", "r3"}   # everything here needs the enable
            if acts and not l1:
                if not self._rearm:
                    self._say("hold L1 for that")
                acts = set()
            if "circle" in acts:                              # L1 + R1: arm power. L1 alone: open the gripper.
                if r1:
                    robot.set_arm_power(not robot.arm_power)
                else:
                    robot.set_gripper(True)
            if "cross" in acts and not r1:
                robot.set_gripper(False)
            starts = acts & {"triangle", "l3", "r3"}
            if "triangle" in starts:
                robot.set_speed(SAFE_SPEED)
                robot.goto("calrdy", source="gamepad")
            elif "l3" in starts:
                robot.home(source="gamepad")
            elif "r3" in starts:
                robot.cycle_start(source="gamepad")

            # Letting go of R1 with a stick still pushed must not start moving the arm, and neither
            # must the nudge a stick gets when it is clicked.
            for stick in ("l", "r"):
                if r1 or stick + "3" in buttons:
                    self._parked.add(stick)
                elif not (shape(axes.get(stick + "x", 0.0)) or shape(axes.get(stick + "y", 0.0))):
                    self._parked.discard(stick)

            if l1 and r1:                                     # triggers set the speed
                self._speed_change += trigger_pull(axes) * SPEED_RAMP_PCT_S * TICK_S
                whole = int(self._speed_change)
                if whole:
                    self._speed_change -= whole
                    robot.set_speed(min(100, max(1, robot.speed + whole)))
            else:
                self._speed_change = 0.0

            was_enabled, self.enabled = self.enabled, l1
            if self.enabled:
                live = {k: (0.0 if k[0] in self._parked and k[1:] in ("x", "y") else v) for k, v in axes.items()}
                vec = jog_vector(self.frame, live, buttons)
                aux = aux_vector(live, buttons)
                if any(vec) or any(aux) or self._jogging():
                    robot.jog(self.frame, vec, ttl=JOG_TTL_S, source="gamepad", aux=aux)
            elif was_enabled and self._jogging():
                robot.halt()                                  # a jog ends with L1; a started move carries on
        except RobotError as err:
            self._say(str(err))

    def release(self):
        """Controller gone or silent: stop anything it started, and ask for a fresh press of L1."""
        self.enabled = False
        self._rearm = True
        self._held = set()
        self._parked = set()
        if self._ours():
            self.robot.halt()

    def _say(self, text):
        self.message, self._message_ticks = text, MESSAGE_TICKS

    def _ours(self):
        command = self.robot.state()["command"]
        return bool(command and command["source"] == "gamepad")

    def _jogging(self):
        command = self.robot.state()["command"]
        return bool(command and command["source"] == "gamepad" and command["kind"] == "jog")


def find_controller(only=None):
    """Return (device, None), or (None, why not). `only` restricts the search to one device path."""
    blocked = None
    for path in [only] if only else _event_nodes():
        try:
            dev = evdev.InputDevice(path)
        except PermissionError:
            name = _sysfs_name(path)
            if "ontroller" in name and "Motion" not in name and "Touchpad" not in name:
                blocked = f"cannot read {path} ({name}): permission denied"
            continue
        except OSError:
            continue
        caps = dev.capabilities()
        keys = caps.get(e.EV_KEY, [])
        absolute = [code for code, _ in caps.get(e.EV_ABS, [])]
        if e.BTN_SOUTH in keys and e.BTN_TL in keys and e.ABS_X in absolute and e.ABS_RX in absolute:
            return dev, None
        dev.close()
    return None, blocked or "no controller connected"


def _event_nodes():
    """Every input event node, readable or not (evdev.list_devices() leaves out the ones we cannot open)."""
    return sorted(glob.glob("/dev/input/event*"), key=lambda path: int(path.rsplit("event", 1)[1]))


def _sysfs(path, attribute):
    try:
        with open(f"/sys/class/input/{os.path.basename(path)}/device/{attribute}") as f:
            return f.read().strip()
    except OSError:
        return ""


def _sysfs_name(path):
    return _sysfs(path, "name")


def find_heartbeat(dev):
    """The motion-sensor node of the same controller: (device, None), or (None, why there is none).

    It sends a report whether or not anything is touched, which the buttons-and-sticks node does not.
    """
    for path in _event_nodes():
        if path == dev.path or not _sysfs_name(path).endswith("Motion Sensors"):
            continue
        uniq, phys = _sysfs(path, "uniq"), _sysfs(path, "phys")
        if not ((dev.uniq and uniq == dev.uniq) or (dev.phys and phys == dev.phys)):
            continue
        try:
            return evdev.InputDevice(path), None
        except PermissionError:
            return None, f"cannot read {path}: install udev/70-a465-gamepad.rules"
        except OSError:
            continue
    return None, "this controller has no motion-sensor node"


class Gamepad:
    """Finds the controller, follows it, and survives it being unplugged and replugged."""

    def __init__(self, robot, device=None):
        self.logic = GamepadJog(robot)
        self.device = device
        self.name = None
        self.why = "not started"
        self.axes = {}
        self.buttons = set()
        self._taps = set()     # presses since the last tick, so a tap shorter than a tick still counts
        self.watchdog = None   # why there is no signal watchdog, or "" when there is one
        self.lost = False      # the watchdog has seen the controller go silent
        self._heard = 0.0      # when the controller last reported

    def status(self):
        message = "signal lost" if self.lost else self.logic.message
        return {"connected": self.name is not None, "name": self.name,
                "message": message if self.name else self.why,
                "watchdog": self.watchdog == "", "watchdog_note": self.watchdog or "", "signal_lost": self.lost,
                "enabled": self.logic.enabled, "frame": self.logic.frame,
                "axes": {k: round(v, 3) for k, v in self.axes.items()},
                "buttons": sorted(self.buttons),
                "camera": camera_vector(self.axes, self.buttons)}

    async def run(self):
        while True:
            dev, self.why = find_controller(self.device)
            if dev is None:
                await asyncio.sleep(1.0)
                continue
            self.name = dev.name
            self._load_current(dev)
            self.logic.release()         # connecting with L1 already held must not enable anything
            pulse, why = find_heartbeat(dev)
            self.watchdog = "" if pulse else why
            self._heard = time.monotonic()
            readers = [asyncio.ensure_future(self._read(dev))]
            if pulse:
                readers.append(asyncio.ensure_future(self._listen(pulse)))
            try:
                while not any(reader.done() for reader in readers):
                    silent = pulse is not None and time.monotonic() - self._heard > STALE_S
                    if silent and not self.lost:
                        self.logic.release()
                    elif not silent:
                        self.logic.update(self.axes, self.buttons | self._taps)
                    self.lost = silent
                    self._taps.clear()
                    await asyncio.sleep(TICK_S)
            finally:
                for reader in readers:
                    reader.cancel()
                self.logic.release()
                self.name, self.axes, self.buttons, self.lost, self.watchdog = None, {}, set(), False, None
                dev.close()
                if pulse:
                    pulse.close()

    def _load_current(self, dev):
        self.buttons = {BUTTONS[c] for c in dev.active_keys() if c in BUTTONS}
        for code in (*STICKS, *TRIGGERS, *HATS):
            self._set_axis(dev, code, dev.absinfo(code).value)

    def _set_axis(self, dev, code, value):
        if code in HATS:
            self.buttons.difference_update(HATS[code])
            if value:
                name = HATS[code][value > 0]
                self.buttons.add(name)
                self._taps.add(name)
            return
        info = dev.absinfo(code)
        if code in STICKS:
            mid, half = (info.max + info.min) / 2.0, (info.max - info.min) / 2.0
            self.axes[STICKS[code]] = max(-1.0, min(1.0, (value - mid) / half))
        elif code in TRIGGERS:
            self.axes[TRIGGERS[code]] = (value - info.min) / (info.max - info.min)

    async def _listen(self, pulse):
        """Note every report from the motion-sensor node; what it says does not matter."""
        try:
            async for _ in pulse.async_read_loop():
                self._heard = time.monotonic()
        except OSError:
            pass    # gone

    async def _read(self, dev):
        try:
            async for event in dev.async_read_loop():
                self._heard = time.monotonic()
                if event.type == e.EV_ABS:
                    self._set_axis(dev, event.code, event.value)
                elif event.type == e.EV_KEY and event.code in BUTTONS:
                    name = BUTTONS[event.code]
                    if event.value:
                        self.buttons.add(name)
                        self._taps.add(name)
                    else:
                        self.buttons.discard(name)
        except OSError:
            pass    # unplugged
