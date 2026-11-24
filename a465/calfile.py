"""The controller's calibration file, /conf/robot.cal.

Worked out from this unit's own file on 2026-11-23 (72 bytes, copied with scripts/a465-getfile) and
from the text inside its robot server, which reads the file in two parts, each checked separately.

The file is two 36-byte records. Each record is a four-byte header and eight little-endian
32-bit signed values, one per axis (six arm joints, then two spare axes):

    byte 0   0x02 in both records of this file (meaning not known)
    byte 1   machine type; 9 on this arm (the same code its stored locations carry)
    byte 2   record type: 2 = home points, 4 = switch-to-index offsets
    byte 3   a check byte (how it is computed is not known; this module never writes the file)

Record type 2 holds each joint's home point in encoder counts from true zero: where the index
pulse that homing stops on sits. Record type 4 holds, per joint, the distance in counts from the
home switch to that index pulse.

COUNTS_PER_DEGREE was measured on this arm by reading counts and degrees together from the
controller's `wact`.
"""
import struct
from dataclasses import dataclass

RECORD_BYTES = 36
HOME_POINTS, SWITCH_OFFSETS = 2, 4
COUNTS_PER_DEGREE = (277.78, 277.78, -276.48, -140.28, -138.89, -140.28)


class CalFileError(ValueError):
    """The bytes are not a calibration file this module understands."""


@dataclass
class Calibration:
    machine_type: int
    home_counts: tuple          # eight values, counts from true zero
    switch_offsets: tuple       # eight values, counts from home switch to index pulse
    check_bytes: tuple          # the two header check bytes, kept as read

    def home_degrees(self):
        """Home point of each arm joint in degrees from true zero, in the controller's sign."""
        return tuple(round(c / per, 2) for c, per in zip(self.home_counts, COUNTS_PER_DEGREE))


def parse(data):
    if len(data) != 2 * RECORD_BYTES:
        raise CalFileError(f"expected {2 * RECORD_BYTES} bytes, got {len(data)}")
    found, checks, machine = {}, [], None
    for start in (0, RECORD_BYTES):
        _, kind_machine, kind, check = data[start:start + 4]
        if kind not in (HOME_POINTS, SWITCH_OFFSETS) or kind in found:
            raise CalFileError(f"record at byte {start} has type {kind}; expected one each of {HOME_POINTS} and {SWITCH_OFFSETS}")
        if machine not in (None, kind_machine):
            raise CalFileError("the two records name different machine types")
        machine = kind_machine
        found[kind] = struct.unpack_from("<8i", data, start + 4)
        checks.append(check)
    return Calibration(machine, found[HOME_POINTS], found[SWITCH_OFFSETS], tuple(checks))


def describe(cal):
    """Lines of text for a person: one row per arm joint."""
    names = ("waist", "shoulder", "elbow", "wrist rotate", "wrist pitch", "tool roll")
    lines = [f"machine type {cal.machine_type}",
             f"{'joint':<16}{'home point (counts)':>20}{'(degrees)':>12}{'switch to index (counts)':>28}"]
    for i, name in enumerate(names):
        lines.append(f"J{i + 1} {name:<13}{cal.home_counts[i]:>20}{cal.home_degrees()[i]:>12.2f}{cal.switch_offsets[i]:>28}")
    spare = [*cal.home_counts[6:], *cal.switch_offsets[6:]]
    lines.append("axes 7 and 8: " + ("all zero (not calibrated)" if not any(spare) else f"home {cal.home_counts[6:]}, offsets {cal.switch_offsets[6:]}"))
    return lines
