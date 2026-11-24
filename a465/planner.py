"""Pick and place on a grid of cells, with the whole motion worked out before anything moves.

The work surface is a horizontal plane divided into 100 mm cells, lettered by row (A is the
most negative Y) and numbered by column (1 is the most negative X). A job is an ordered list
of pick cells and place cells; pick 1 goes to place 1, and so on.

`Job.plan()` turns that into steps, each one a table of joint angles and track position every
20 ms, and checks every row of every table before it reports the plan as good:

  * Long moves are made joint by joint (every axis starts and stops together). That kind of
    move has no singularities; it is checked for joint limits and for dipping towards the plane.
  * Only the short strokes down to a cell and back up are straight lines. The tool points
    straight down for those, and each sample must be inside the joint limits with the wrist
    bent and the arm short of full stretch, which keeps the arm clear of the three shapes
    where straight-line motion breaks down.
  * The track position for each cell is the one the arm can get to soonest from where it
    was, among those that put the cell at a comfortable reach. Sliding the track and
    swinging the waist happen together, so the quickest spot balances the two.

Keep-out zones are the one kind of collision it knows about: a rectangle on the plane with a top
height. Every sample of every step is checked against them (see `zone_hit`), and a transit that
would pass through one is raised until it clears. Nothing else is modelled beyond staying above
the plane.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from . import model

CELL_MM = 100.0
X_MIN, Y_MIN = -600.0, -700.0
COLS, ROWS = 34, 14                # X from -600 to 2800, Y from -700 to 700
TRACK_HALF_WIDTH_MM = 160.0        # cells whose centre is this close to the track's centre line are not usable
APPROACH_MM = 120.0                # height above the plane for moves between cells
GRIP_HEIGHT_MM = 10.0              # fingertips stop this far above the plane
CLEARANCE_MM = 40.0                # joint moves must keep the fingertips at least this far above the plane
PART_MM = 40.0                     # the cube the mock puts on each pick cell
GRIP_SECONDS = 0.4
LINE_ACCEL_MM_S2 = 1500.0
SAMPLE_S = 0.02
LIMIT_MARGIN_DEG = 3.0
WRIST_MIN_DEG = 15.0               # straight strokes keep the wrist at least this bent
ELBOW_MIN_DEG = 20.0               # and the elbow at least this far from straight
MAX_ZONES = 20
ZONE_KINDS = ("pick", "place", "keepout")
ZONE_LIMIT_MM = 5000.0             # zone corners must be within this of the world origin
ZONE_MIN_MM = 10.0                 # and each side at least this long
ZONE_TOP_MAX_MM = 1500.0
# How close any part of the arm may come to a keep-out zone, horizontally, and how far above its top.
# A STAND-IN, not a measured value: the real arm's body, cables and gripper have not been measured.
ARM_MARGIN_MM = 60.0
ZONE_SAMPLE_MM = 30.0              # spacing of the points checked along the arm
STROKE_SLACK_MM = 5.0              # extra room asked for when choosing the track position for a cell
RAISE_STEP_MM = 50.0               # travel heights tried when a transit has to be raised
JAW_DEG = 0.0                      # jaws open along world X, whatever the waist angle
_SEARCH_MM = 10.0                  # spacing of the track positions tried for a cell
_AXIS_SPEED = np.array([*model.MAX_SPEED_DEG_S, model.TRACK_MAX_SPEED_MM_S])
_AXIS_ACCEL = np.array([*model.ACCEL_DEG_S2, model.TRACK_ACCEL_MM_S2])


class JobError(ValueError):
    """A job that cannot be accepted, with the reason."""


def cell_name(col, row):
    return f"{chr(ord('A') + row)}{col + 1}"


def parse_cell(name):
    """'F12' -> (column index, row index). Raises JobError for anything that is not a cell of the grid."""
    if not isinstance(name, str) or len(name) < 2 or not name[1:].isdigit():
        raise JobError(f"'{name}' is not a cell name (a row letter and a column number, like F12)")
    row, col = ord(name[0].upper()) - ord("A"), int(name[1:]) - 1
    if not (0 <= row < ROWS and 0 <= col < COLS):
        raise JobError(f"'{name}' is outside the grid (rows A to {chr(ord('A') + ROWS - 1)}, columns 1 to {COLS})")
    return col, row


def cell_centre(name):
    col, row = parse_cell(name)
    return X_MIN + (col + 0.5) * CELL_MM, Y_MIN + (row + 0.5) * CELL_MM


def over_track(y):
    return abs(y) < TRACK_HALF_WIDTH_MM


# ---- keep-out zones

def _inside(points, keepouts, slack):
    """For points (... x 3): the zone numbers (in `keepouts`) whose forbidden volume holds any of them, per pose."""
    flat = points.reshape(points.shape[0], -1, 3)
    found = []
    for number, zone in keepouts:
        gap_x = np.maximum(np.maximum(zone["x0"] - flat[..., 0], flat[..., 0] - zone["x1"]), 0.0)
        gap_y = np.maximum(np.maximum(zone["y0"] - flat[..., 1], flat[..., 1] - zone["y1"]), 0.0)
        found.append(np.any((np.hypot(gap_x, gap_y) <= ARM_MARGIN_MM + slack) & (flat[..., 2] < zone["top"] + ARM_MARGIN_MM + slack), axis=1))
    return found


def zones_hit(joints, track, keepouts, slack=0.0):
    """The numbers of the keep-out zones the arm is inside in this pose.

    `keepouts` is a list of (number, zone). The arm is inside a zone when any point sampled
    along its key segments is within ARM_MARGIN_MM of the rectangle (horizontally) and below
    the zone's top plus ARM_MARGIN_MM. `slack` widens both a little, for choosing spots with room to spare.
    """
    if not keepouts:
        return []
    points = model.sample_path(model.arm_points(np.asarray(joints, float)[None], [track]), ZONE_SAMPLE_MM)
    return [number for (number, _), hit in zip(keepouts, _inside(points, keepouts, slack)) if hit[0]]


def zone_hit(joints, track, keepouts, slack=0.0):
    """The number of the first keep-out zone the arm is inside in this pose, or None."""
    found = zones_hit(joints, track, keepouts, slack)
    return found[0] if found else None


def samples_hit(samples, keepouts):
    """The number of the first keep-out zone any row of a sample table (six joints then track) is inside, or None."""
    if not keepouts or not len(samples):
        return None
    samples = np.asarray(samples, float)
    points = model.sample_path(model.arm_points(samples[:, :6], samples[:, 6]), ZONE_SAMPLE_MM)
    hits = _inside(points, keepouts, 0.0)
    first = None
    for (number, _), hit in zip(keepouts, hits):
        if hit.any():
            row = int(np.argmax(hit))
            if first is None or row < first[0]:
                first = (row, number)
    return first[1] if first else None


def _cells_inside(zone):
    """Names of the grid cells whose centre is inside the zone's rectangle, row A first, then by column."""
    return [cell_name(col, row) for row in range(ROWS) for col in range(COLS)
            if zone["x0"] <= X_MIN + (col + 0.5) * CELL_MM <= zone["x1"] and zone["y0"] <= Y_MIN + (row + 0.5) * CELL_MM <= zone["y1"]]


def _overlaps_cell(zone, col, row):
    """True if the cell's square overlaps the zone's rectangle at all (touching edges do not count)."""
    x, y = X_MIN + col * CELL_MM, Y_MIN + row * CELL_MM
    return zone["x0"] < x + CELL_MM and zone["x1"] > x and zone["y0"] < y + CELL_MM and zone["y1"] > y


def _clean_zone(raw, number, plane_z):
    """A zone as the planner keeps it, or JobError."""
    if not isinstance(raw, dict):
        raise JobError(f"zone {number} must be an object with kind, x0, y0, x1, y1 and top")
    kind = raw.get("kind")
    if kind not in ZONE_KINDS:
        raise JobError(f"zone {number}: kind must be one of {', '.join(ZONE_KINDS)}")
    values = {}
    for key in ("x0", "y0", "x1", "y1"):
        v = raw.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not abs(v) <= ZONE_LIMIT_MM:
            raise JobError(f"zone {number}: '{key}' must be a number from -{ZONE_LIMIT_MM:g} to {ZONE_LIMIT_MM:g} mm")
        values[key] = round(float(v), 1)
    x0, x1 = sorted((values["x0"], values["x1"]))
    y0, y1 = sorted((values["y0"], values["y1"]))
    if x1 - x0 < ZONE_MIN_MM or y1 - y0 < ZONE_MIN_MM:
        raise JobError(f"zone {number}: each side must be at least {ZONE_MIN_MM:g} mm long")
    top = None
    if kind == "keepout":
        top = raw.get("top")
        if isinstance(top, bool) or not isinstance(top, (int, float)) or not plane_z < top <= ZONE_TOP_MAX_MM:
            raise JobError(f"zone {number}: 'top' must be a height above the plane ({plane_z:g} mm) and at most {ZONE_TOP_MAX_MM:g} mm")
        top = round(float(top), 1)
    return {"kind": kind, "x0": x0, "y0": y0, "x1": x1, "y1": y1, "top": top}


def _name_cells(names):
    shown = ", ".join(names[:6])
    return shown + (f" and {len(names) - 6} more" if len(names) > 6 else "")


def pose_problem(joints):
    """Why this pose may not be used on a straight stroke, or None if it may."""
    if joints is None:
        return "out of reach"
    for i, (angle, (lo, hi)) in enumerate(zip(joints, model.LIMITS_DEG)):
        if not lo + LIMIT_MARGIN_DEG <= angle <= hi - LIMIT_MARGIN_DEG:
            return f"J{i + 1} ({model.JOINT_NAMES[i]}) would be at its limit"
    if abs(joints[4]) < WRIST_MIN_DEG:
        return "the wrist would be nearly straight"
    if abs(joints[2]) < ELBOW_MIN_DEG:
        return "the arm would be at full stretch"
    return None


def stroke_problem(dx, y, plane_z):
    """Why the arm cannot do the straight stroke over a cell `dx` ahead and `y` to the side of its base, or None."""
    for height in (APPROACH_MM, APPROACH_MM * 0.75, APPROACH_MM * 0.5, APPROACH_MM * 0.25, GRIP_HEIGHT_MM):
        problem = pose_problem(model.tool_down(dx, y, plane_z + height, JAW_DEG))
        if problem:
            return problem
    return None


def above(x, y, plane_z, track, height=APPROACH_MM):
    """Joints and track (seven numbers) with the fingertips `height` above the plane over the cell at world (x, y)."""
    return np.array([*model.tool_down(x - track, y, plane_z + height, JAW_DEG), track])


def _stroke_zone(dx, y, track, plane_z, keepouts):
    """The keep-out zone the arm would be inside somewhere on the stroke over a cell, or None."""
    for height in (GRIP_HEIGHT_MM, GRIP_HEIGHT_MM + 5.0, APPROACH_MM * 0.25, APPROACH_MM * 0.5, APPROACH_MM):
        number = zone_hit(model.tool_down(dx, y, plane_z + height, JAW_DEG), track, keepouts, slack=STROKE_SLACK_MM)
        if number is not None:
            return number
    return None


def choose_track(x, y, plane_z, start, share, spots=None, keepouts=(), fitted=True):
    """Track position for the cell at world (x, y): comfortable reach, and the quickest to get to from `start`.

    `start` is where the arm is before the move (six joints then track). `spots` limits the track
    positions that may be used (default: the whole track, every 10 mm). Spots whose stroke at the
    cell would be inside a keep-out zone are dropped. Returns (track mm, None) or (None, why not).
    """
    if fitted and over_track(y):
        return None, "over the track"
    if spots is None:
        count = int(model.TRACK_TRAVEL_MM // _SEARCH_MM)
        spots = [i * _SEARCH_MM for i in range(count + 1)]
    good = [stroke_problem(x - t, y, plane_z) is None for t in spots]
    if not any(good):
        nearest = min(max(x, 0.0), model.TRACK_TRAVEL_MM) if len(spots) > 1 else spots[0]
        return None, stroke_problem(x - nearest, y, plane_z) or "out of reach"
    # Prefer a spot with room either side, so the pose is not right at the edge of what works.
    roomy = [all(good[max(i - 2, 0):i + 3]) for i in range(len(spots))]
    blocked = None
    for wanted in (roomy, good) if any(roomy) else (good,):
        choice = [t for t, fine in zip(spots, wanted) if fine]
        for t in sorted(choice, key=lambda t: move_seconds(start, above(x, y, plane_z, t), share)):
            blocked = _stroke_zone(x - t, y, t, plane_z, keepouts) if keepouts else None
            if blocked is None:
                return t, None
    return None, f"the arm would be inside keep-out zone {blocked} at this cell"


def _ease(count):
    """Positions 0..1 along a move that starts and ends at rest (minimum jerk), at count + 1 instants."""
    t = np.linspace(0.0, 1.0, count + 1)
    return t ** 3 * (10.0 - 15.0 * t + 6.0 * t * t)


_EASE_PEAK_SPEED, _EASE_PEAK_ACCEL = 1.875, 5.7735     # of distance / time and distance / time^2


def move_seconds(start, end, share):
    """How long a joint-by-joint move takes: the slowest axis sets the time for all of them."""
    distance = np.abs(np.asarray(end, float) - np.asarray(start, float))
    return float(np.max(np.maximum(_EASE_PEAK_SPEED * distance / (_AXIS_SPEED * share),
                                   np.sqrt(_EASE_PEAK_ACCEL * distance / _AXIS_ACCEL))))


def joint_move(start, end, share):
    """Samples (n x 7: six joints then track) of a move in which every axis starts and stops together."""
    start, end = np.asarray(start, float), np.asarray(end, float)
    count = max(1, math.ceil(move_seconds(start, end, share) / SAMPLE_S))
    return start + _ease(count)[:, None] * (end - start)


def stroke(dx, y, z_from, z_to, track, share, roll=None):
    """Samples of a straight vertical line with the tool pointing down, slowed until no joint is over its speed.

    `roll` keeps tool roll at that angle instead of lining the jaws up with the grid.
    """
    length = abs(z_to - z_from)
    seconds = max(_EASE_PEAK_SPEED * length / (model.MAX_LINEAR_MM_S * share), math.sqrt(_EASE_PEAK_ACCEL * length / LINE_ACCEL_MM_S2))
    for _ in range(6):
        count = max(1, math.ceil(seconds / SAMPLE_S))
        samples = np.array([(*model.tool_down(dx, y, z_from + s * (z_to - z_from), JAW_DEG), track) for s in _ease(count)])
        if roll is not None:
            samples[:, 5] = roll
        over = float(np.max(np.abs(np.diff(samples, axis=0)) / SAMPLE_S / (_AXIS_SPEED * share)))
        if over <= 1.0:
            break
        seconds *= over * 1.05
    return samples


def _thin(samples, about):
    """Roughly `about` evenly spaced rows of a sample table, always including the last."""
    stride = max(1, (len(samples) - 1) // about)
    return [*samples[:-1:stride], samples[-1]]


def rise(here, top, share):
    """Samples of a straight lift to height `top` from where the arm is, or None if it is not hanging tool-down.

    For starting a cycle from low over the plane, as after one was stopped at the bottom of a stroke.
    """
    joints, track = here[:6], here[6]
    x, y, z = model.fingertip(joints)
    same = model.tool_down(x, y, z, JAW_DEG)
    if same is None or z >= top or np.max(np.abs(np.array(same[:5]) - joints[:5])) > 0.05:
        return None
    samples = stroke(x, y, z, top, track, share, roll=joints[5])
    return samples if all(pose_problem(row[:6]) is None for row in samples) else None


@dataclass
class Step:
    kind: str                      # "move" (joint by joint), "line" (straight stroke) or "grip"
    label: str
    cell: str = ""
    samples: np.ndarray = None     # for move and line
    open: bool = None              # for grip

    @property
    def seconds(self):
        return GRIP_SECONDS if self.kind == "grip" else (len(self.samples) - 1) * SAMPLE_S


@dataclass
class Plan:
    steps: list = field(default_factory=list)
    problems: list = field(default_factory=list)       # anything here means the plan must not run
    notes: list = field(default_factory=list)
    pairs: list = field(default_factory=list)          # one dict per pick/place pair
    speed: int = 10

    @property
    def ok(self):
        return bool(self.steps) and not self.problems

    @property
    def seconds(self):
        return sum(step.seconds for step in self.steps)

    def path(self):
        """Fingertip positions along the whole plan, for drawing it."""
        points = []
        for step in self.steps:
            if step.kind == "grip":
                continue
            rows = _thin(step.samples, 24 if step.kind == "move" else 2)
            points.extend([round(float(c), 1) for c in model.fingertip(row[:6], row[6])] for row in rows)
        return points

    def summary(self):
        return {
            "ok": self.ok, "problems": self.problems, "notes": self.notes, "pairs": self.pairs,
            "seconds": round(self.seconds, 1), "speed": self.speed,
            "steps": [{"kind": s.kind, "label": s.label, "cell": s.cell, "seconds": round(s.seconds, 2)} for s in self.steps],
            "path": self.path(),
        }


class Job:
    """Which cells to pick from and place into, the zones, the plane they lie on, and the track settings."""

    def __init__(self):
        self.picks, self.places, self.plane_z = [], [], 0.0
        self.zones = []                    # {"kind", "x0", "y0", "x1", "y1", "top"}, in the order they were made
        self.track_fitted = True           # False: the arm is bolted down with its base at X = 0
        self.track_moves = True            # False: the track stays where it is when the cycle starts
        self._grids = {}

    def set(self, picks, places, plane_z=None, zones=None):
        """Change the cells, the plane height and the zones; None leaves `plane_z` or `zones` as they were."""
        if not isinstance(picks, list) or not isinstance(places, list):
            raise JobError("'picks' and 'places' must be lists of cell names")
        names = [cell_name(*parse_cell(name)) for name in [*picks, *places]]
        if len(set(names)) != len(names):
            raise JobError("a cell can be used only once")
        if plane_z is not None:
            if isinstance(plane_z, bool) or not isinstance(plane_z, (int, float)) or not -200 <= plane_z <= 400:
                raise JobError("'plane_z' must be a height from -200 to 400 mm")
        plane = self.plane_z if plane_z is None else float(plane_z)
        if zones is None:
            zones = self.zones
        if not isinstance(zones, list):
            raise JobError("'zones' must be a list of zones")
        if len(zones) > MAX_ZONES:
            raise JobError(f"at most {MAX_ZONES} zones are allowed")
        clean = [_clean_zone(raw, number, plane) for number, raw in enumerate(zones, 1)]
        # A cell must not be both a pick and a place. This looks at every cell inside the areas, usable or not,
        # so that the answer does not change when the track moves.
        as_pick, as_place = set(names[:len(picks)]), set(names[len(picks):])
        for zone in clean:
            if zone["kind"] != "keepout":
                (as_pick if zone["kind"] == "pick" else as_place).update(_cells_inside(zone))
        both = sorted(as_pick & as_place, key=lambda name: (parse_cell(name)[1], parse_cell(name)[0]))
        if both:
            raise JobError(f"{_name_cells(both)} would be both a pick and a place")
        self.plane_z, self.zones = plane, clean
        self.picks, self.places = names[:len(picks)], names[len(picks):]

    def keepouts(self):
        """The keep-out zones as (number, zone), numbered by their place in the zone list."""
        return [(number, zone) for number, zone in enumerate(self.zones, 1) if zone["kind"] == "keepout"]

    def track_spots(self, track):
        """The track positions the planner may use: every one, or just the one the track is held at."""
        if not self.track_fitted:
            return [0.0]
        return None if self.track_moves else [float(track)]

    def grid(self, track=0.0):
        """The grid for the page: its size, and for every cell whether a stroke over it is possible at all.

        `track` matters only when the track is held where it is.
        """
        held = self.track_fitted and not self.track_moves
        key = (self.plane_z, self.track_fitted, self.track_moves, round(float(track), 2) if held else None,
               tuple((n, z["x0"], z["y0"], z["x1"], z["y1"], z["top"]) for n, z in self.keepouts()))
        if key not in self._grids:
            keepouts = [z for _, z in self.keepouts()]
            spots = self.track_spots(track)
            reach = []
            for row in range(ROWS):
                y = Y_MIN + (row + 0.5) * CELL_MM
                if self.track_fitted and over_track(y):
                    cells = "t" * COLS
                elif spots is None:
                    ahead = [d * _SEARCH_MM for d in range(-80, 81)]
                    fine = [d for d in ahead if stroke_problem(d, y, self.plane_z) is None]
                    cells = "".join("y" if any(0.0 <= X_MIN + (col + 0.5) * CELL_MM - d <= model.TRACK_TRAVEL_MM for d in fine) else "n"
                                    for col in range(COLS))
                else:
                    cells = "".join("y" if any(stroke_problem(X_MIN + (col + 0.5) * CELL_MM - t, y, self.plane_z) is None for t in spots) else "n"
                                    for col in range(COLS))
                cells = "".join("k" if any(_overlaps_cell(z, col, row) for z in keepouts) else code for col, code in enumerate(cells))
                reach.append(cells)
            self._grids = {key: reach}
        return {"cell": CELL_MM, "x_min": X_MIN, "y_min": Y_MIN, "cols": COLS, "rows": ROWS, "plane_z": self.plane_z,
                "approach": APPROACH_MM, "part": PART_MM, "grip_height": GRIP_HEIGHT_MM,
                # per row, per column: y usable, n out of reach, t over the track, k overlaps a keep-out zone
                "reach": self._grids[key]}

    def resolve(self, track=0.0):
        """(pick list, place list, zone report): the clicked cells first, then each area's usable cells in zone order.

        The report has one entry per zone: its values, plus for an area the `cells` it contributes and how
        many cells inside it were `left_out` (out of reach, over the track or inside a keep-out zone).
        """
        reach = self.grid(track)["reach"]
        picks, places, report = list(self.picks), list(self.places), []
        for zone in self.zones:
            entry = {**zone, "cells": [], "left_out": 0}
            if zone["kind"] != "keepout":
                target = picks if zone["kind"] == "pick" else places
                for name in _cells_inside(zone):
                    col, row = parse_cell(name)
                    if reach[row][col] != "y":
                        entry["left_out"] += 1
                        continue
                    entry["cells"].append(name)
                    if name not in target:
                        target.append(name)
            report.append(entry)
        return picks, places, report

    def describe(self, plan=None, track=0.0):
        picks, places, report = self.resolve(track)
        return {"grid": self.grid(track), "picks": self.picks, "places": self.places, "zones": report,
                "effective_picks": picks, "effective_places": places,
                "track_fitted": self.track_fitted, "track_moves": self.track_moves,
                "plan": plan.summary() if plan else None}

    def plan(self, joints, track, speed, gripper_open=True):
        """Work out the whole cycle from where the arm is now."""
        plan = Plan(speed=int(speed))
        share = speed / 100.0
        picks, places, report = self.resolve(track)
        for number, entry in enumerate(report, 1):
            if entry["left_out"]:
                plural = "cell" if entry["left_out"] == 1 else "cells"
                plan.notes.append(f"{entry['kind']} area (zone {number}): {entry['left_out']} {plural} left out "
                                  "(out of reach, over the track or inside a keep-out zone)")
        for kind, cells, others in (("pick", picks, places), ("place", places, picks)):
            for index in range(len(others), len(cells)):
                plan.notes.append(f"{kind} {index + 1} ({cells[index]}) has no {'place' if kind == 'pick' else 'pick'} cell yet")
        pairs = list(zip(picks, places))
        if not pairs:
            return plan

        keepouts = self.keepouts()
        spots_allowed = self.track_spots(track)
        spots, before = [], np.array([*joints, track], float)
        for number, (pick, place) in enumerate(pairs, 1):
            pair = {"pick": pick, "place": place}
            for kind, name in (("pick", pick), ("place", place)):
                x, y = cell_centre(name)
                spot, why = choose_track(x, y, self.plane_z, before, share, spots_allowed, keepouts, self.track_fitted)
                if spot is None:
                    plan.problems.append(f"{kind} {number} ({name}): {why}")
                else:
                    before = above(x, y, self.plane_z, spot)
                pair[f"{kind}_track"] = spot
            spots.append(pair)
        plan.pairs = spots
        if plan.problems:
            return plan
        inside = zone_hit(joints, track, keepouts)
        if inside is not None:
            plan.problems.append(f"the arm is inside keep-out zone {inside}: jog it out of the zone first")
            return plan

        here = np.array([*joints, track], float)
        low, high = self.plane_z + GRIP_HEIGHT_MM, self.plane_z + APPROACH_MM
        floor = self.plane_z + CLEARANCE_MM                # joint moves keep the fingertips above this

        def stroke_to(spot_cell, z_from, z_to):
            x, y = cell_centre(spot_cell[0])
            if model.tool_down(x - spot_cell[1], y, max(z_from, z_to), JAW_DEG) is None:
                return None                                 # too high to reach
            return stroke(x - spot_cell[1], y, z_from, z_to, spot_cell[1], share)

        def attempt(depart, arrive, height, check_poses):
            """The lift, move and lowering of one transit at travel height `height`, and the zone they hit (or None).

            Returns None instead of a result if a raised stroke is not valid for the arm.
            """
            parts, at = {}, here
            if depart:
                parts["lift"] = stroke_to(depart, low, self.plane_z + height)
                if parts["lift"] is None:
                    return None
                at = parts["lift"][-1]
            if arrive:
                x, y = cell_centre(arrive[0])
                if model.tool_down(x - arrive[1], y, self.plane_z + height, JAW_DEG) is None:
                    return None
                target = above(x, y, self.plane_z, arrive[1], height)
            else:
                target = np.array([*model.POSES_DEG["work"], at[6]])
            parts["move"] = joint_move(at, target, share)
            if arrive:
                parts["lower"] = stroke_to(arrive, self.plane_z + height, low)
                if parts["lower"] is None:
                    return None
            if check_poses and any(pose_problem(row[:6]) for key in ("lift", "lower") if key in parts for row in parts[key]):
                return None
            hit = None
            for samples in parts.values():
                hit = samples_hit(samples, keepouts)
                if hit is not None:
                    break
            return parts, hit

        def transit(depart, arrive, label, cell, arrive_label):
            """Emit the lift from `depart`, the move and the lowering into `arrive` (either may be None)."""
            nonlocal here, floor
            result, height = attempt(depart, arrive, APPROACH_MM, False), APPROACH_MM
            if result[1] is not None:
                direct_hit = result[1]
                tallest = max(zone["top"] for _, zone in keepouts) - self.plane_z + CLEARANCE_MM
                raised, trial = None, max(APPROACH_MM, tallest)
                while trial <= ZONE_TOP_MAX_MM + 400.0:
                    if trial != APPROACH_MM:
                        found = attempt(depart, arrive, trial, True)
                        if found is None:
                            break
                        if found[1] is None:
                            raised = (found, trial)
                            break
                    trial += RAISE_STEP_MM
                if raised:
                    result, height = raised
                    plan.notes.append(f"{label}{cell and f' ({cell})'} is raised to a travel height of {height:g} mm "
                                      f"to clear keep-out zone {direct_hit}")
                else:
                    plan.problems.append(f"{label}{cell and f' ({cell})'}: keep-out zone {direct_hit} is in the way "
                                         "and no travel height clears it")
            parts = result[0]
            if "lift" in parts:
                plan.steps.append(Step("line", depart[2], depart[0], parts["lift"]))
            samples = parts["move"]
            lowest = min(float(model.fingertip(row[:6], row[6])[2]) for row in _thin(samples, 40))
            if lowest < floor:
                plan.problems.append(f"{label}: the move would bring the gripper within {lowest - self.plane_z:.0f} mm of the plane")
            plan.steps.append(Step("move", label, cell, samples))
            floor = self.plane_z + CLEARANCE_MM
            if "lower" in parts:
                plan.steps.append(Step("line", arrive_label, arrive[0], parts["lower"]))
            here = plan.steps[-1].samples[-1]

        if not gripper_open:
            plan.steps.append(Step("grip", "open the gripper", open=True))
        start_height = float(model.fingertip(joints, track)[2])
        if start_height < floor:                           # starting low: straight up first if it can, and never lower
            samples = rise(here, high, share)
            if samples is None:
                floor = start_height - 0.5
            else:
                plan.steps.append(Step("line", "lift clear of the plane", "", samples))
                here = samples[-1]
                inside = samples_hit(samples, keepouts)
                if inside is not None:
                    plan.problems.append(f"lift clear of the plane: the arm would enter keep-out zone {inside}")
        previous = None
        for number, pair in enumerate(spots, 1):
            for kind, closes in (("pick", True), ("place", False)):
                name, spot = pair[kind], pair[f"{kind}_track"]
                here_stop = (name, spot, f"lift from {kind} {number}")
                transit(previous, (name, spot), f"move above {kind} {number}", name, f"lower to {kind} {number}")
                plan.steps.append(Step("grip", "close the gripper" if closes else "open the gripper", name, open=not closes))
                previous = here_stop
        transit(previous, None, "move to the work pose", "", "")
        return plan
