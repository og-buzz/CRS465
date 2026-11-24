"""A stand-in for the C500 console, for testing a465/crs.py without the arm.

It imitates only what was seen on the real controller on 2026-11-23: the `$` and `test>` prompts,
echo, `ash`, `online`, `speed`, `joint`, `w2`, `arm_status`, `axst &` printing status lines about
five times a second between everything else, Ctrl-Z stopping a move with the same text, and the
"device is being reset" answer for a moment afterwards.
"""
import threading
import time

RATE = (180.0, 180.0, 180.0, 171.0, 173.0, 171.0)      # degrees a second at 100%, as a465/model.py has them


class FakeC500:
    def __init__(self, *args, **kwargs):
        self.out = b""
        self.lock = threading.Lock()
        self.shell = "$ "
        self.line = b""
        self._typed = b""
        self.joints = [0.0] * 6
        self.queue = []                 # [axis, degrees left]
        self.speed = 8
        self.online = False
        self.status_on = False
        self.reset_until = 0.0
        self.asking_exit = False
        self.sent = []                  # every complete line received, and "^Z" for each Ctrl-Z
        self.limp = False
        self.switch_at = {}             # axis -> angle above which its home switch reads 0
        self.closed = False
        self.think = 0.0                # seconds before a typed line is acted on and answered
        self.reset_s = 0.3              # how long moves are refused after a stop
        self.needs_button = False       # control of the arm is handed over only after press()
        self._waiting_for_button = False
        self._later = []                # [when, line] typed but not acted on yet
        self.pace = 3840.0              # bytes a second the "line" delivers (38400 baud); None = all at once
        self._credit, self._paced = 0.0, time.monotonic()
        self._last = time.monotonic()
        self._next_status = 0.0
        self.most_pending = 0           # most joint moves ever outstanding at once: typed but not acted on, or still travelling

    # --- the pyserial methods crs.py uses ---
    def reset_input_buffer(self):
        with self.lock:
            self.out = b""

    def write(self, data):
        with self.lock:
            self._advance()
            for byte in data:
                if byte == 0x1a:
                    self.sent.append("^Z")
                    self.queue.clear()
                    self.reset_until = time.monotonic() + self.reset_s
                    self.out = b"^C equivalent - halting robot..." + (b"[ash: Caught SIGABRT]" if self.shell != "$ " else b"") + b"\r\n" + self.shell.encode() + self.out
                elif byte == 0x0d:
                    self.out = self._typed + b"\r\n" + self.out      # the echo cuts into a line that is part-way out
                    self._typed = b""
                    line, self.line = self.line.decode(), b""
                    self.sent.append(line)
                    self._later.append([time.monotonic() + self.think, line.strip()])
                    self._count_pending()
                else:
                    self.line += bytes([byte])
                    self._typed += bytes([byte])
        return len(data)

    def read(self, count=1):
        with self.lock:
            self._advance()
            if self.pace:
                now = time.monotonic()
                self._credit = min(self._credit + (now - self._paced) * self.pace, 64.0) if self.out else 0.0
                self._paced = now
                count = min(count, int(self._credit))
                self._credit -= count
            data, self.out = self.out[:count], self.out[count:]
        return data

    def close(self):
        self.closed = True

    # --- behaviour ---
    def press(self):
        """Someone presses Pause/Continue on the front panel."""
        with self.lock:
            if self._waiting_for_button:
                self._waiting_for_button, self.needs_button, self.online = False, False, True
                self.out += b"ash: Not homed or not calibrated\r\n" + self.shell.encode()

    def _advance(self):
        now = time.monotonic()
        while self._later and self._later[0][0] <= now:
            if self.queue and self._later[0][1].startswith("joint") and self.shell != "$ ":
                break                   # the shell sits on a move until the one under way has finished
            self._run(self._later.pop(0)[1])
            self.out += b"" if self.asking_exit or self._waiting_for_button else self.shell.encode()
        dt, self._last = now - self._last, now
        while dt > 0 and self.queue:
            axis, left = self.queue[0]
            step = RATE[axis] * self.speed / 100.0 * dt
            if abs(left) <= step:
                self.joints[axis] += left
                dt -= abs(left) / (RATE[axis] * self.speed / 100.0)
                self.queue.pop(0)
            else:
                self.joints[axis] += step if left > 0 else -step
                self.queue[0][1] = left - (step if left > 0 else -step)
                dt = 0
        if self.status_on and now >= self._next_status:
            self._next_status = now + 0.18
            words = []
            for axis in range(6):
                word = 0x0406 | (0 if self.queue else 0x8000) | (8 if self.limp else 0)
                if not (axis in self.switch_at and self.joints[axis] > self.switch_at[axis]):
                    word |= 1
                words.append(0 if now < self.reset_until - self.reset_s / 2 else word)
            self.out += (" ".join(f"{w:08x}" for w in words) + " 00008008 00008008\r\n").encode()

    def _count_pending(self):
        waiting = sum(1 for _, line in self._later if line.startswith("joint "))
        self.most_pending = max(self.most_pending, waiting + len(self.queue))

    def _run(self, line):
        if self.asking_exit:
            self.asking_exit = False
            if line == "yes":
                self.shell, self.status_on = "$ ", False
            return
        if not line:
            return
        if self.shell == "$ ":
            if line.startswith("ash "):
                self.shell = line.split()[1] + "> "
                self.out += b"Loading  v3 file 'test.v3'...done.\r\n"
            return
        if time.monotonic() < self.reset_until and line.split()[0] in ("w2", "joint", "speed"):
            self.out += b"ash: device is being reset\r\n"
        elif line == "exit":
            self.asking_exit = True
            self.out += b"Are you sure you want to exit? "
        elif line == "online 1" and self.needs_button:
            self._waiting_for_button = True
            self.out += b"Press the PAUSE/CONTINUE button to gain robot point of control.\r\n"
        elif line == "online 1":
            self.online = True
            self.out += b"ash: Not homed or not calibrated\r\n"
        elif line == "online":
            self.out += b"Online mode is ON\r\n" if self.online else b"Online mode is OFF\r\n"
        elif line == "arm_status":
            self.out += b"Robot arm is of type 465\r\nArm power is ON     Robot is not homed\r\n"
        elif line == "axst &":
            self.status_on = True
            self.out += b"[Started process 271]\r\n"
        elif line.startswith("speed "):
            self.speed = int(line.split()[1])
        elif line.startswith("joint "):
            axis, degrees = line[6:].split(",")
            self.queue.append([int(axis) - 1, float(degrees)])
            self._count_pending()
        elif line == "w2":
            j = self.joints
            self.out += ("Actual Position :\r\n\r\n          Axis 1/7   Axis 2/8     Axis 3     Axis 4     Axis 5     Axis 6\r\n"
                         "PULSES          +0         +0         +0         +0         +0         +0\r\n                +0         +0\r\n\r\n"
                         "          Axis 1/7   Axis 2/8     Axis 3     Axis 4     Axis 5     Axis 6\r\n"
                         f"JOINTS  {j[0]:10.3f} {j[1]:10.3f} {j[2]:10.3f} {j[3]:10.3f} {j[4]:10.3f} {j[5]:10.3f}\r\n             0.000      0.000\r\n\r\n"
                         "          X/TrackX   Y/TrackY          Z      Z-Rot      Y-Rot      X-Rot\r\nWORLD         (mm)       (mm)       (mm)      (deg)      (deg)      (deg)\r\n"
                         "             0.000      0.000   1041.400      0.000    -90.000      0.000\r\n             0.000      0.000\r\n").encode()
        else:
            self.out += f"ash: {line.split()[0]}: not found\r\n".encode()
