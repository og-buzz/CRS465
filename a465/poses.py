"""Named arm poses ("teach and replay"): a small JSON file, and the actions object the status page uses.

A pose holds the six joint angles in degrees and the six motor counts as the controller reported them (the counts are what a
joint go-to sends), and the tool point (x, y, z in mm, yaw, pitch, roll in degrees, world frame) computed with a465/model.py at
save time (what a straight-line go-to sends). File layout: {"selected": name or null, "poses": [pose, ...]} in saved order.

Writes are atomic (temp file in the same directory, then rename). A missing file is an empty store; a file that cannot be read
as a pose file is moved aside to `<file>.bad-<time>` (never overwritten) and the store starts empty. In both cases `message`
says so. Every problem the operator can cause (bad name, name taken, no such pose, ...) is a PoseError, which is a ValueError.
"""
import datetime
import json
import os
import re
import tempfile
import threading

from . import model

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "poses.json")
NAME_RE = re.compile(r"[A-Za-z0-9 _-]{1,24}")
NUMBER_KEYS = ("x", "y", "z", "yaw", "pitch", "roll")


class PoseError(ValueError):
    """A refusal with a message meant for the operator."""


def check_name(name):
    name = name.strip() if isinstance(name, str) else name
    if not isinstance(name, str) or not NAME_RE.fullmatch(name):
        raise PoseError("a pose name is 1 to 24 characters: letters, digits, space, - and _")
    return name


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v == v and abs(v) != float("inf")


def _valid_pose(p):
    try:
        return (isinstance(p, dict) and NAME_RE.fullmatch(p["name"]) is not None
                and len(p["joints"]) == 6 and all(_is_number(v) for v in p["joints"])
                and len(p["counts"]) == 6 and all(isinstance(v, int) and not isinstance(v, bool) for v in p["counts"])
                and all(_is_number(p[k]) for k in NUMBER_KEYS) and isinstance(p["saved"], str)
                and (p.get("world_hundredths") is None
                     or (len(p["world_hundredths"]) == 6 and all(isinstance(v, int) for v in p["world_hundredths"]))))
    except (KeyError, TypeError):
        return False


def _copy(pose):
    out = dict(pose)
    out["joints"], out["counts"] = list(pose["joints"]), list(pose["counts"])
    return out


class PoseStore:
    def __init__(self, path=None, clock=datetime.datetime.now):
        self.path = os.path.abspath(path or DEFAULT_PATH)
        self._clock = clock
        self._lock = threading.RLock()
        self._poses = []
        self._selected = None
        self.message = ""
        self._load()

    # ---- file ----------------------------------------------------------------------------------------

    def _load(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                data = json.load(f)
            poses, selected = data["poses"], data.get("selected")
            if not isinstance(poses, list) or not all(_valid_pose(p) for p in poses):
                raise ValueError("not a list of valid poses")
            names = [p["name"] for p in poses]
            if len(set(names)) != len(names) or (selected is not None and selected not in names):
                raise ValueError("duplicate names or a selected pose that does not exist")
        except FileNotFoundError:
            self.message = f"no pose file yet ({self.path}): it is created by the first save"
            return
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as err:
            self._set_aside(err)
            return
        self._poses, self._selected = [_copy(p) for p in poses], selected

    def _set_aside(self, err):
        bad = f"{self.path}.bad-{self._clock().strftime('%Y%m%d-%H%M%S')}"
        n = 1
        while os.path.exists(bad):
            n += 1
            bad = f"{self.path}.bad-{self._clock().strftime('%Y%m%d-%H%M%S')}-{n}"
        try:
            os.replace(self.path, bad)
            self.message = f"the pose file {self.path} could not be read ({err}); it was kept as {bad} and the list starts empty"
        except OSError as err2:
            self.message = (f"the pose file {self.path} could not be read ({err}) and could not be moved aside ({err2}); "
                            "the list is empty and saving is refused until that is fixed")
            self._locked = True

    _locked = False

    def _write(self):
        if self._locked:
            raise PoseError(f"the unreadable pose file {self.path} could not be moved aside: fix that first")
        data = json.dumps({"selected": self._selected, "poses": self._poses}, indent=1)
        directory = os.path.dirname(self.path) or "."
        tmp = None
        try:
            fd, tmp = tempfile.mkstemp(prefix=".poses-", suffix=".tmp", dir=directory)
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(data + "\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
            tmp = None
        except OSError as err:
            raise PoseError(f"could not write {self.path}: {err}")
        finally:
            if tmp is not None:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass

    def _commit(self, poses, selected):
        """Write the new state; only when that worked does it replace the one in memory."""
        old = self._poses, self._selected
        self._poses, self._selected = poses, selected
        try:
            self._write()
        except PoseError:
            self._poses, self._selected = old
            raise

    # ---- reading -------------------------------------------------------------------------------------

    def __len__(self):
        with self._lock:
            return len(self._poses)

    def list(self):
        with self._lock:
            return [_copy(p) for p in self._poses]

    def names(self):
        with self._lock:
            return [p["name"] for p in self._poses]

    def get(self, name):
        with self._lock:
            for p in self._poses:
                if p["name"] == name:
                    return _copy(p)
        raise PoseError(f"no pose named {name!r}")

    def selected(self):
        with self._lock:
            return self._selected

    # ---- changing ------------------------------------------------------------------------------------

    def next_free_name(self):
        with self._lock:
            used = {p["name"] for p in self._poses}
            n = 1
            while f"P{n}" in used:
                n += 1
            return f"P{n}"

    def save(self, name=None, joints=None, counts=None, overwrite=False, world=None):
        """Store a pose and return it. name None: the lowest free P1, P2, ... An existing name is refused unless overwrite
        (the pose then keeps its place in the list).
        world: the controller's own pose, six integers (x, y, z, yaw, pitch, roll in hundredths) from its `W` line, or None; the pose
        then says which source its straight-line numbers have: "world_source" "controller" or "model"."""
        if joints is None or counts is None or len(joints) != 6 or len(counts) != 6:
            raise PoseError("a pose needs six joint angles and six motor counts")
        with self._lock:
            name = self.next_free_name() if name is None or not str(name).strip() else check_name(name)
            point, rot, _, _ = model.forward([float(a) for a in joints])
            yaw, pitch, roll = model.yaw_pitch_roll(rot)
            pose = {"name": name, "joints": [round(float(a), 4) for a in joints], "counts": [int(c) for c in counts],
                    "x": round(float(point[0]), 2), "y": round(float(point[1]), 2), "z": round(float(point[2]), 2),
                    "yaw": round(float(yaw), 2), "pitch": round(float(pitch), 2), "roll": round(float(roll), 2),
                    "saved": self._clock().isoformat(timespec="seconds"),
                    "world_hundredths": [int(v) for v in world] if world is not None else None,
                    "world_source": "controller" if world is not None else "model"}
            poses = [_copy(p) for p in self._poses]
            for i, p in enumerate(poses):
                if p["name"] == name:
                    if not overwrite:
                        raise PoseError(f"a pose named {name!r} already exists")
                    poses[i] = pose
                    break
            else:
                poses.append(pose)
            self._commit(poses, self._selected)
            return _copy(pose)

    def rename(self, old, new):
        with self._lock:
            new = check_name(new)
            self.get(old)
            if new != old and new in self.names():
                raise PoseError(f"a pose named {new!r} already exists")
            poses = [_copy(p) for p in self._poses]
            for p in poses:
                if p["name"] == old:
                    p["name"] = new
            self._commit(poses, new if self._selected == old else self._selected)

    def delete(self, name):
        with self._lock:
            self.get(name)
            self._commit([_copy(p) for p in self._poses if p["name"] != name], None if self._selected == name else self._selected)

    def select(self, name):
        """A name, or None for no selection."""
        with self._lock:
            if name is not None:
                self.get(name)
            if name != self._selected:
                self._commit([_copy(p) for p in self._poses], name)

    def select_next(self):
        """Select the pose after the selected one (wrapping; the first when none is selected). Returns its name."""
        with self._lock:
            names = self.names()
            if not names:
                raise PoseError("no poses saved yet")
            nxt = names[(names.index(self._selected) + 1) % len(names)] if self._selected in names else names[0]
            self.select(nxt)
            return nxt


class PoseActions:
    """What the status page's pose buttons call. Each method is quick and thread-safe; a refusal is a ValueError whose text is
    for the operator (RobotError from the arm is turned into one)."""

    def __init__(self, arm, store):
        self._arm, self._store = arm, store

    def _arm_call(self, fn, *args, **kw):
        from .sim import RobotError
        try:
            return fn(*args, **kw)
        except RobotError as err:
            raise ValueError(str(err))

    def list_poses(self):
        return self._store.list()

    def selected(self):
        return self._store.selected()

    def select(self, name):
        self._store.select(name)

    def save(self, name=None):
        return self._arm_call(self._arm.save_pose, name)

    def rename(self, old, new):
        self._store.rename(old, new)

    def delete(self, name):
        self._store.delete(name)

    def goto_hold(self, name, straight=False):
        self._arm_call(self._arm.goto_hold, name, straight=straight, source="web")

    def stop(self):
        self._arm.halt()
