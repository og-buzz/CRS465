"""The pose store (a465/poses.py) and the actions object the status page calls."""
import datetime
import json
import os

import pytest

from a465 import model, poses
from a465.poses import PoseActions, PoseError, PoseStore
from a465.sim import RobotError

J = [10.0, -20.0, 30.0, 40.0, -50.0, 60.0]
C = [2778, 5556, -8333, -5611, 6944, -8417]


@pytest.fixture
def path(tmp_path):
    return str(tmp_path / "poses.json")


def test_default_path_is_the_repo_top_directory():
    assert poses.DEFAULT_PATH == os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(poses.__file__))), "poses.json")


def test_missing_file_is_an_empty_store_with_a_message(path):
    store = PoseStore(path)
    assert store.list() == [] and len(store) == 0 and store.selected() is None
    assert "no pose file yet" in store.message and not os.path.exists(path)


def test_save_computes_the_tool_point_and_writes_the_file(path):
    store = PoseStore(path, clock=lambda: datetime.datetime(2026, 11, 25, 10, 30, 5))
    pose = store.save(None, J, C)
    point, rot, _, _ = model.forward(J)
    yaw, pitch, roll = model.yaw_pitch_roll(rot)
    assert pose["name"] == "P1" and pose["joints"] == J and pose["counts"] == C and pose["saved"] == "2026-11-25T10:30:05"
    assert (pose["x"], pose["y"], pose["z"]) == (round(point[0], 2), round(point[1], 2), round(point[2], 2))
    assert (pose["yaw"], pose["pitch"], pose["roll"]) == (round(yaw, 2), round(pitch, 2), round(roll, 2))
    assert set(pose) == {"name", "joints", "counts", "x", "y", "z", "yaw", "pitch", "roll", "saved",
                        "world_hundredths", "world_source"}
    assert pose["world_hundredths"] is None and pose["world_source"] == "model"
    assert store.save("W", J, C, world=[1, 2, 3, 4, 5, 6])["world_source"] == "controller"
    store.delete("W")
    again = PoseStore(path)
    assert again.list() == [pose] and again.message == ""
    assert [f for f in os.listdir(os.path.dirname(path)) if f != "poses.json"] == []     # no temp file left


def test_automatic_names_use_the_lowest_free_number(path):
    store = PoseStore(path)
    assert [store.save(None, J, C)["name"] for _ in range(3)] == ["P1", "P2", "P3"]
    store.delete("P2")
    assert store.save(None, J, C)["name"] == "P2"
    store.save("P5", J, C)
    store.rename("P1", "Home")
    assert store.save(None, J, C)["name"] == "P1"
    assert store.names() == ["Home", "P3", "P2", "P5", "P1"]            # saved order


def test_names_are_checked(path):
    store = PoseStore(path)
    for bad in ("", "   ", "x" * 25, "a/b", "tab\there", "naïve", "a.b"):
        with pytest.raises(PoseError):
            store.save(bad, J, C) if bad.strip() else store.rename("nope", bad)
    assert store.save("A b_c-1", J, C)["name"] == "A b_c-1" and store.save("x" * 24, J, C)
    assert store.save("  padded ", J, C)["name"] == "padded"


def test_overwrite_only_when_asked_and_keeps_the_place(path):
    store = PoseStore(path)
    store.save("A", J, C)
    store.save("B", J, C)
    with pytest.raises(PoseError, match="already exists"):
        store.save("A", J, C)
    new = store.save("A", [0] * 6, [0] * 6, overwrite=True)
    assert store.names() == ["A", "B"] and store.get("A") == new and new["joints"] == [0.0] * 6


def test_rename_delete_get_and_selected(path):
    store = PoseStore(path)
    store.save("A", J, C)
    store.save("B", J, C)
    store.select("A")
    assert store.selected() == "A" and PoseStore(path).selected() == "A"
    store.rename("A", "Z")
    assert store.selected() == "Z" and store.names() == ["Z", "B"]
    with pytest.raises(PoseError):
        store.rename("Z", "B")
    with pytest.raises(PoseError):
        store.rename("nope", "Q")
    with pytest.raises(PoseError, match="no pose named"):
        store.get("nope")
    with pytest.raises(PoseError):
        store.select("nope")
    store.delete("Z")
    assert store.selected() is None and PoseStore(path).selected() is None
    with pytest.raises(PoseError):
        store.delete("Z")
    store.select("B")
    store.select(None)
    assert store.selected() is None


def test_select_next_wraps(path):
    store = PoseStore(path)
    with pytest.raises(PoseError, match="no poses"):
        store.select_next()
    for n in "ABC":
        store.save(n, J, C)
    assert [store.select_next() for _ in range(4)] == ["A", "B", "C", "A"]


def test_returned_poses_are_copies(path):
    store = PoseStore(path)
    store.save("A", J, C)
    got = store.get("A")
    got["joints"][0] = 99
    store.list()[0]["counts"][0] = 99
    assert store.get("A")["joints"][0] == J[0] and store.get("A")["counts"][0] == C[0]


@pytest.mark.parametrize("content", ["{not json", "[]", '{"poses": 5}', '{"poses": [{"name": "A"}]}', "",
                                     json.dumps({"poses": [{"name": "A", "joints": [1] * 6, "counts": [1.5] * 6, "x": 0, "y": 0, "z": 0,
                                                            "yaw": 0, "pitch": 0, "roll": 0, "saved": "t"}]})])
def test_a_corrupt_file_is_empty_and_kept_aside(path, content):
    with open(path, "w") as f:
        f.write(content)
    store = PoseStore(path, clock=lambda: datetime.datetime(2026, 11, 25, 11, 0, 0))
    assert store.list() == [] and "could not be read" in store.message and "poses.json.bad-20261125-110000" in store.message
    assert not os.path.exists(path)
    assert open(path + ".bad-20261125-110000").read() == content          # kept as it was
    store.save(None, J, C)                                                 # the store works, and writes a fresh file
    assert PoseStore(path).names() == ["P1"] and os.path.exists(path + ".bad-20261125-110000")


def test_two_corrupt_files_in_one_second_do_not_overwrite_each_other(path):
    clock = lambda: datetime.datetime(2026, 11, 25, 11, 0, 0)       # noqa: E731
    for text in ("one{", "two{"):
        with open(path, "w") as f:
            f.write(text)
        PoseStore(path, clock=clock)
    assert sorted(open(os.path.join(os.path.dirname(path), n)).read() for n in os.listdir(os.path.dirname(path))) == ["one{", "two{"]


def test_a_failed_write_is_an_error_and_changes_nothing(path, monkeypatch):
    store = PoseStore(path)
    store.save("A", J, C)

    def boom(*a, **k):
        raise OSError("disk full")
    monkeypatch.setattr(os, "replace", boom)
    with pytest.raises(PoseError, match="disk full"):
        store.save("B", J, C)
    assert store.names() == ["A"] and [f for f in os.listdir(os.path.dirname(path))] == ["poses.json"]
    with pytest.raises(PoseError):
        store.select("A")
    assert store.selected() is None


# ---- the actions object -------------------------------------------------------------------------------------------

class StubArm:
    def __init__(self, store):
        self.store, self.calls = store, []

    def save_pose(self, name=None):
        self.calls.append(("save", name))
        if name == "busy":
            raise RobotError("cannot save a pose while the arm is moving")
        pose = self.store.save(name, J, C)
        self.store.select(pose["name"])
        return pose

    def goto_hold(self, name, straight=False, source="rest"):
        self.calls.append(("hold", name, straight, source))
        if name == "far":
            raise RobotError("outside the joint limits")

    def halt(self):
        self.calls.append(("halt",))


def test_actions_have_exactly_the_agreed_methods_and_turn_refusals_into_value_errors(path):
    store = PoseStore(path)
    arm = StubArm(store)
    actions = PoseActions(arm, store)
    assert {m for m in dir(actions) if not m.startswith("_")} == {"list_poses", "selected", "select", "save", "rename", "delete",
                                                                  "goto_hold", "stop"}
    assert actions.save()["name"] == "P1" and actions.selected() == "P1"
    assert actions.save("B")["name"] == "B"
    assert [p["name"] for p in actions.list_poses()] == ["P1", "B"]
    actions.select("P1")
    actions.rename("P1", "Home")
    assert actions.selected() == "Home"
    actions.goto_hold("Home", straight=True)
    actions.stop()
    assert arm.calls[-2:] == [("hold", "Home", True, "web"), ("halt",)]
    for call in (lambda: actions.save("busy"), lambda: actions.goto_hold("far"), lambda: actions.select("nope"),
                 lambda: actions.rename("nope", "x"), lambda: actions.delete("nope"), lambda: actions.save("B")):
        with pytest.raises(ValueError):
            call()
    actions.delete("B")
    assert actions.list_poses()[0]["name"] == "Home" and len(actions.list_poses()) == 1
