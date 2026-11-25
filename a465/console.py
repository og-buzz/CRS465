"""Reading the controller's `$` shell: the per-axis status words (`axst`) and the joint angles (`w2`).

Everything here takes an `aci.Link` (a serial-like port with a read-ahead buffer and a replaceable
clock) and works at the `$` system prompt. Measured on the real controller on 2026-11-23/24:

  * `axst` prints a line of eight 8-hex-digit status words (one per axis) over and over until Ctrl-E.
  * `w2` prints the joint angles once, in a block whose JOINTS line carries axes 1 to 6; a second
    row of numbers (axes 7 and 8) may follow. For about 2 s after Ctrl-Z the shell answers
    "device is being reset" instead.
"""
import re

from . import aci
from .crs import NUMBER, PROBLEM, STATUS, STATUS_BITS      # noqa: F401  (re-exported for callers)

CTRL_E = b"\x05"                   # kills the foreground program
CTRL_Z = b"\x1a"                   # the software stop: halts the robot


def parse_status(data):
    """The eight status words of the first complete `axst` line in `data` (bytes or str), or None."""
    if isinstance(data, str):
        data = data.encode("latin-1")
    found = STATUS.search(data)
    return [int(w, 16) for w in found.group(1).split()] if found else None


def parse_joints(text):
    """The six joint angles from a `w2` answer's JOINTS line, or None."""
    for line in text.replace("\r", "").split("\n"):
        if line.startswith("JOINTS"):
            values = [float(n) for n in NUMBER.findall(line[6:])]
            if len(values) >= 6:
                return values[:6]
    return None


def parse_pulses(text):
    """The six encoder counts from a `w2` answer's PULSES line (signed integers, e.g. `+173`, `-24830`), or None.
    The line after it (axes 7 and 8) is not read."""
    for line in text.replace("\r", "").split("\n"):
        if line.startswith("PULSES"):
            values = [int(n) for n in re.findall(r"[+-]?\d+", line[6:])]
            if len(values) >= 6:
                return values[:6]
    return None


def bit(word, number):
    return bool(word >> number & 1)


def describe_word(word):
    """The set bits of one status word, by name: 'armed for zero-cross, locked, homed'."""
    return ", ".join(name for number, name in STATUS_BITS.items() if bit(word, number)) or "none set"


def read_status(link, wait=4.0):
    """Eight status words from `axst` (stopped again with Ctrl-E), or None. Leaves the `$` prompt showing if the controller gives it back."""
    link.drain(quiet=0.1, limit=1.0)
    link.port.write(b"axst\r")
    end = link.clock() + wait
    words = None
    while words is None and link.clock() < end:
        link._more()
        words = parse_status(link.ahead)
    link.port.write(CTRL_E)
    end = link.clock() + 3.0
    while link.clock() < end and not link.ahead.endswith(b"$ "):
        link._more()
    link.ahead = b""
    return words


def read_joints(link, tries=4):
    """The six joint angles from `w2`, retrying while the controller is still resetting after a stop; None if it never answered."""
    for _ in range(tries):
        link.drain(quiet=0.1, limit=1.0)
        answer = link.shell("w2", lambda g: g.endswith(b"$ ") and b"\n" in g, 10.0).decode("latin-1")
        joints = parse_joints(answer)
        if joints is not None:
            return joints
        link.sleep(1.0)
    return None


def read_pulses(link, tries=4):
    """The six encoder counts from `w2`, retrying as read_joints does; None if never answered."""
    for _ in range(tries):
        link.drain(quiet=0.1, limit=1.0)
        answer = link.shell("w2", lambda g: g.endswith(b"$ ") and b"\n" in g, 10.0).decode("latin-1")
        pulses = parse_pulses(answer)
        if pulses is not None:
            return pulses
        link.sleep(1.0)
    return None


def at_prompt(link):
    return aci.at_system_prompt(link)
