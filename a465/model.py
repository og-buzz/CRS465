"""Kinematic model of the CRS A465 arm.

Numbers come from the A465 Robot Arm User Guide (UMI-33-465): joint ranges
(Table 2), section lengths (Table 3), joint speeds (Table 7), accelerations
(Table 8), positive joint directions (Table 20). World frame: origin at the
centre of the base mounting flange, Z up, X forward (Application Environment
guide, "World Coordinate System").

Two things are read off the manual's drawings rather than stated in a table, and
still need confirming on the real arm with `ready` then `wact`:
  * Joint zero is the middle of each range, which puts the all-zero pose
    straight up (the calrdy position) and the ready position at elbow = -90.
  * Yaw/pitch/roll are reported as Rz(yaw) Ry(pitch) Rx(roll) of a tool frame
    whose X axis points out of the flange, so they read 0/0/0 at ready.

Units: millimetres and degrees.
"""
import math

import numpy as np

JOINT_NAMES = ("waist", "shoulder", "elbow", "wrist rotate", "wrist pitch", "tool roll")
LIMITS_DEG = ((-175, 175), (-90, 90), (-110, 110), (-180, 180), (-105, 105), (-180, 180))
MAX_SPEED_DEG_S = (180.0, 180.0, 180.0, 171.0, 173.0, 171.0)
ACCEL_DEG_S2 = (720.0, 720.0, 720.0, 1430.0, 1430.0, 1430.0)
MAX_LINEAR_MM_S = 1020.0  # Table 6, linear and path motions

BASE_TO_SHOULDER, SHOULDER_TO_ELBOW, ELBOW_TO_WRIST, WRIST_TO_FLANGE = 330.0, 305.0, 330.0, 76.0

# Axis 7, the track: the whole arm rides along world X. Speed and acceleration are the
# T475 figures from the Track User Guide. Zero is at the connector end (the guide puts
# it 12.7 mm from that hard stop). The travel is this machine's, "about 7 ft" per its
# owner; the real limits live in the controller's configuration.
TRACK_TRAVEL_MM = 2134.0
TRACK_MAX_SPEED_MM_S = 800.0
TRACK_ACCEL_MM_S2 = 3000.0
# Axis 8, the carousel. Nothing is known about it yet; these numbers are placeholders.
CAROUSEL_MAX_SPEED_DEG_S = 90.0
CAROUSEL_ACCEL_DEG_S2 = 360.0

# Each joint: where it sits in its parent's frame (at zero pose everything stacks
# along Z), the local axis it turns about, and the sign that makes a positive joint
# angle match Table 20 ("raises arm", "raises outer link", "pitches wrist up").
CHAIN = (
    {"offset": (0.0, 0.0, 0.0), "axis": "z", "sign": 1},
    {"offset": (0.0, 0.0, BASE_TO_SHOULDER), "axis": "y", "sign": -1},
    {"offset": (0.0, 0.0, SHOULDER_TO_ELBOW), "axis": "y", "sign": -1},
    {"offset": (0.0, 0.0, 0.0), "axis": "z", "sign": 1},
    {"offset": (0.0, 0.0, ELBOW_TO_WRIST), "axis": "y", "sign": -1},
    {"offset": (0.0, 0.0, 0.0), "axis": "z", "sign": 1},
)
FLANGE_OFFSET = (0.0, 0.0, WRIST_TO_FLANGE)
# Flange face to fingertips of the gripper the web page draws. A stand-in until the real
# tool is measured; the reported pose (x, y, z) is still the flange face.
GRIPPER_LENGTH_MM = 84.0


def tool_down(x, y, z, jaw_deg=0.0):
    """Joint angles that put the fingertips at (x, y, z) from the base with the tool pointing straight down.

    Elbow up, wrist rotate at 0, and the jaws opening along the direction `jaw_deg` from world X
    (they look the same turned half a turn, so tool roll stays within +/-90). Returns None when
    the point is further than the arm reaches. Joint limits are not checked here.
    """
    rho = math.hypot(x, y)
    up = z + GRIPPER_LENGTH_MM + WRIST_TO_FLANGE - BASE_TO_SHOULDER      # wrist centre above the shoulder
    cos_bend = (rho * rho + up * up - SHOULDER_TO_ELBOW ** 2 - ELBOW_TO_WRIST ** 2) / (2 * SHOULDER_TO_ELBOW * ELBOW_TO_WRIST)
    if not -1.0 <= cos_bend <= 1.0:
        return None
    bend = math.acos(cos_bend)                                           # elbow: 0 is a straight arm
    lean = math.atan2(rho, up) - math.atan2(ELBOW_TO_WRIST * math.sin(bend), SHOULDER_TO_ELBOW + ELBOW_TO_WRIST * math.cos(bend))
    wrist = math.pi - lean - bend                                        # the three add up to straight down
    waist = math.degrees(math.atan2(y, x)) if rho > 1e-9 else 0.0
    roll = (waist - jaw_deg + 180.0) % 180.0 - 90.0
    return (waist, -math.degrees(lean), -math.degrees(bend), 0.0, -math.degrees(wrist), roll)


POSES_DEG = {
    "ready": (0.0, 0.0, -90.0, 0.0, 0.0, 0.0),
    "calrdy": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # Tool pointing down in front of the arm, every joint well away from a limit and from the
    # shapes where straight-line motion breaks down. A place to start world and tool jogging.
    "work": tuple(round(a, 3) for a in tool_down(480.0, 0.0, 250.0, jaw_deg=90.0)),
}

_AXES = {"x": np.array([1.0, 0.0, 0.0]), "y": np.array([0.0, 1.0, 0.0]), "z": np.array([0.0, 0.0, 1.0])}
# Tool frame relative to the flange: tool X points out of the flange.
_TOOL = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])


def _rot(axis, angle_rad):
    """Rotation matrix about a unit axis (Rodrigues)."""
    x, y, z = axis
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def forward(joints_deg, track_mm=0.0):
    """Return (tcp position mm, tool rotation 3x3, joint origins, joint axes in world)."""
    r, p = np.eye(3), np.array([track_mm, 0.0, 0.0])
    origins, axes = [], []
    for joint, angle in zip(CHAIN, joints_deg):
        p = p + r @ np.array(joint["offset"])
        axis = joint["sign"] * _AXES[joint["axis"]]
        origins.append(p)
        axes.append(r @ axis)
        r = r @ _rot(axis, math.radians(angle))
    p = p + r @ np.array(FLANGE_OFFSET)
    return p, r @ _TOOL, origins, axes


def jacobian(joints_deg):
    """6x6 geometric Jacobian: rows are mm/rad (linear) then rad/rad (angular)."""
    tcp, _, origins, axes = forward(joints_deg)
    return np.vstack([np.column_stack([np.cross(a, tcp - o) for o, a in zip(origins, axes)]),
                      np.column_stack(axes)])


def fingertip(joints_deg, track_mm=0.0):
    """World position of the gripper's fingertips."""
    tcp, rot, _, _ = forward(joints_deg, track_mm)
    return tcp + GRIPPER_LENGTH_MM * rot[:, 0]


def arm_points(joints_deg, track_mm=0.0):
    """The arm's key points in world coordinates: the shoulder axis, the elbow, the wrist centre, the
    flange face and the fingertips, as a 5 x 3 array. Consistent with forward() and fingertip().

    Also takes a table of poses (n x 6 joints, n tracks) and then returns n x 5 x 3, all at once.
    """
    joints = np.asarray(joints_deg, float)
    single = joints.ndim == 1
    joints = np.atleast_2d(joints)
    track = np.broadcast_to(np.asarray(track_mm, float), (len(joints),))
    a1, p2, a4, p5 = np.radians(joints[:, 0]), -np.radians(joints[:, 1]), np.radians(joints[:, 3]), -np.radians(joints[:, 4])
    theta = p2 - np.radians(joints[:, 2])                     # upper arm and forearm pitch add up
    zero = np.zeros(len(joints))
    c1, s1 = np.cos(a1), np.sin(a1)

    def turn(v):                                              # rotate about the world Z by the waist angle
        return np.stack([c1 * v[0] - s1 * v[1], s1 * v[0] + c1 * v[1], v[2]], axis=-1)

    shoulder = np.stack([track, zero, zero + BASE_TO_SHOULDER], axis=-1)
    elbow = shoulder + SHOULDER_TO_ELBOW * turn((np.sin(p2), zero, np.cos(p2)))
    along = turn((np.sin(theta), zero, np.cos(theta)))        # the forearm's direction
    wrist = elbow + ELBOW_TO_WRIST * along
    across = turn((np.cos(a4) * np.cos(theta), np.sin(a4), -np.cos(a4) * np.sin(theta)))
    hand = np.cos(p5)[:, None] * along + np.sin(p5)[:, None] * across    # the way the flange points
    flange = wrist + WRIST_TO_FLANGE * hand
    points = np.stack([shoulder, elbow, wrist, flange, flange + GRIPPER_LENGTH_MM * hand], axis=1)
    return points[0] if single else points


def sample_path(points, spacing_mm=30.0):
    """Points along the straight segments joining `points` (5 x 3, or n x 5 x 3 for n poses), no further
    apart than `spacing_mm`; the key points themselves are included. Returns (m x 3) or (n x m x 3)."""
    points = np.asarray(points, float)
    single = points.ndim == 2
    points = points[None] if single else points
    out = [points[:, :1]]
    for a, b in zip(points[:, :-1].transpose(1, 0, 2), points[:, 1:].transpose(1, 0, 2)):
        count = max(1, math.ceil(float(np.max(np.linalg.norm(b - a, axis=-1))) / spacing_mm))
        out.append(a[:, None] + (np.arange(1, count + 1) / count)[None, :, None] * (b - a)[:, None])
    out = np.concatenate(out, axis=1)
    return out[0] if single else out


# The three shapes in which this arm loses a direction of straight-line motion.
SINGULARITIES = {
    "wrist": "the wrist is straight (J5 near 0)",
    "elbow": "the arm is at full stretch (J3 near 0)",
    "shoulder": "the wrist is over the waist axis",
}


def conditioning(joints_deg):
    """How far the arm is from each singular shape: 0 means in it, 1 means as far as it gets."""
    _, _, origins, _ = forward(joints_deg)
    wrist_centre = origins[4]
    return {
        "wrist": abs(math.sin(math.radians(joints_deg[4]))),
        "elbow": abs(math.sin(math.radians(joints_deg[2]))),
        "shoulder": min(1.0, math.hypot(wrist_centre[0], wrist_centre[1]) / SHOULDER_TO_ELBOW),
    }


def nearest_singularity(joints_deg):
    """(description, closeness) of the singular shape the arm is nearest to."""
    measures = conditioning(joints_deg)
    name = min(measures, key=measures.get)
    return SINGULARITIES[name], measures[name]


def yaw_pitch_roll(rot):
    """Degrees, for rot = Rz(yaw) Ry(pitch) Rx(roll). Roll is 0 at the pitch = +/-90 singularity."""
    sp = max(-1.0, min(1.0, -rot[2, 0]))
    pitch = math.asin(sp)
    if abs(sp) > 0.999999:
        return math.degrees(math.atan2(-rot[0, 1], rot[1, 1])), math.degrees(pitch), 0.0
    return (math.degrees(math.atan2(rot[1, 0], rot[0, 0])), math.degrees(pitch),
            math.degrees(math.atan2(rot[2, 1], rot[2, 2])))


def describe():
    """The model as plain data, for the web page to draw the same arm."""
    return {
        "joints": [{"name": n, "offset": j["offset"], "axis": j["axis"], "sign": j["sign"],
                    "min": lo, "max": hi, "max_speed": v}
                   for n, j, (lo, hi), v in zip(JOINT_NAMES, CHAIN, LIMITS_DEG, MAX_SPEED_DEG_S)],
        "flange_offset": FLANGE_OFFSET,
        "gripper_length": GRIPPER_LENGTH_MM,
        "track": {"axis": "x", "min": 0.0, "max": TRACK_TRAVEL_MM, "max_speed": TRACK_MAX_SPEED_MM_S},
        "carousel": {"max_speed": CAROUSEL_MAX_SPEED_DEG_S},
        "poses": POSES_DEG,
        "units": {"length": "mm", "angle": "deg"},
    }
