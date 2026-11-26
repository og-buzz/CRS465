import asyncio
import json
import math
import socket

import aiohttp

from a465 import indicator, model

JOINTS = [10.0, -20.0, 30.0, 5.0, 40.0, -15.0]


def test_tool_point_matches_model():
    out, _ = indicator.augment({"joints": JOINTS, "position_time": 100.0}, None, now=100.1)
    p, rot, _, _ = model.forward(JOINTS)
    yaw, pitch, roll = model.yaw_pitch_roll(rot)
    assert out["tcp"] == {"x": p[0], "y": p[1], "z": p[2], "yaw": yaw, "pitch": pitch, "roll": roll}
    assert out["limits"] == [[lo, hi] for lo, hi in model.LIMITS_DEG]
    assert out["now"] == 100.1 and out["tcp_speed"] == 0.0


def test_tcp_speed_from_two_samples():
    a, b = JOINTS, [11.0] + JOINTS[1:]
    _, prev = indicator.augment({"joints": a, "position_time": 100.0}, None, now=100.0)
    out, prev = indicator.augment({"joints": b, "position_time": 100.2}, prev, now=100.3)
    d = math.dist(model.forward(a)[0], model.forward(b)[0])
    assert abs(out["tcp_speed"] - d / 0.2) < 1e-9 and d > 0
    # same sample polled again keeps the same speed
    out2, prev = indicator.augment({"joints": b, "position_time": 100.2}, prev, now=100.4)
    assert abs(out2["tcp_speed"] - d / 0.2) < 1e-9


def test_tcp_speed_zero_when_stale():
    _, prev = indicator.augment({"joints": JOINTS, "position_time": 100.0}, None, now=100.0)
    out, _ = indicator.augment({"joints": [11.0] + JOINTS[1:], "position_time": 100.2}, prev, now=102.0)
    assert out["tcp_speed"] == 0.0


def test_missing_keys_tolerated():
    for st in ({}, {"joints": None}, {"joints": [1, 2]}, {"position_time": None, "joints": JOINTS}):
        out, _ = indicator.augment(st, None)
        assert out["tcp_speed"] == 0.0 and "now" in out and len(out["limits"]) == 6
    assert indicator.augment({}, None)[0]["tcp"] is None


def test_limit_key_and_guard_limits_reach_the_page(tmp_path):
    limit = {"joint": 3, "limit": -112.0, "angle": -110.4, "blocked": True}
    guard = [[-175.0, 175.0], [-90.0, 90.0], [-112.0, 112.0], [-180.0, 180.0], [-105.0, 105.0], [-180.0, 180.0]]
    out, _ = indicator.augment({"joints": JOINTS, "limit": limit, "guard_limits": guard}, None, now=1.0)
    assert out["limit"] == limit and out["limits"] == guard           # the bars use the guard's table when the state has one
    out, _ = indicator.augment({"joints": JOINTS, "guard_limits": None}, None, now=1.0)
    assert out["limits"] == [[lo, hi] for lo, hi in model.LIMITS_DEG] and "limit" not in out
    html = indicator.PAGE.read_text()
    assert "s.limit" in html and "glim" in html                        # the amber bar is drawn from state["limit"]
    shown = [indicator.demo_state(t)["limit"] for t in (1040.0, 1000.0)]
    assert shown[0]["joint"] == 3 and shown[1] is None


def test_demo_state_has_every_key():
    for t in (1000.0, 1010.0, 1062.0, 1025.0):
        st = indicator.demo_state(t)
        assert set(indicator.KEYS) <= set(st)
        out, _ = indicator.augment(st, None, now=t)
        for (lo, hi), a in zip(model.LIMITS_DEG, st["joints"]):
            assert lo <= a <= hi
    assert indicator.demo_state(1062.0)["controller_message"]["text"]
    assert indicator.demo_state(1062.0)["jogd"] == "dead"


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_server_routes_and_error_survival():
    async def go():
        port = _free_port()
        calls = {"n": 0}

        def get_state():
            calls["n"] += 1
            if calls["n"] == 2:
                raise RuntimeError("boom")
            return {"joints": JOINTS, "position_time": 1.0, "frame": "joint"}

        task = asyncio.create_task(indicator.serve(get_state, "127.0.0.1", port))
        await asyncio.sleep(0.3)
        base = f"http://127.0.0.1:{port}"
        async with aiohttp.ClientSession() as s:
            async with s.get(base + "/") as r:
                assert r.status == 200 and "text/html" in r.headers["Content-Type"]
                assert "/state" in await r.text()
            async with s.get(base + "/state") as r:
                d = await r.json()
                assert d["frame"] == "joint" and d["tcp"]["x"] and len(d["limits"]) == 6
            async with s.get(base + "/state") as r:
                assert "boom" in (await r.json())["error"]
            async with s.get(base + "/state") as r:
                assert "tcp" in await r.json()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        with socket.socket() as sk:  # port released
            sk.bind(("127.0.0.1", port))

    asyncio.run(go())


# ---- saved poses ----
import threading
import time

JSON = {"Content-Type": "application/json"}


class FakeActions:
    def __init__(self):
        self.calls = []
        self.poses = {"A": {"name": "A", "joints": [0] * 6}}
        self.sel = None
        self.slow = 0.0

    def list_poses(self):
        return list(self.poses.values())

    def selected(self):
        return self.sel

    def select(self, name):
        self.calls.append(("select", name))
        if name not in self.poses:
            raise ValueError("no such pose")
        self.sel = name

    def save(self, name=None):
        self.calls.append(("save", name))
        if name == "dup":
            raise ValueError("already exists")
        if name == "crash":
            raise KeyError("x")
        name = name or "P1"
        self.poses[name] = {"name": name, "joints": [1] * 6}
        return self.poses[name]

    def rename(self, old, new):
        self.calls.append(("rename", old, new))
        if old not in self.poses:
            raise ValueError("no such pose")
        self.poses[new] = dict(self.poses.pop(old), name=new)

    def delete(self, name):
        self.calls.append(("delete", name))
        if name not in self.poses:
            raise ValueError("no such pose")
        del self.poses[name]

    def goto_hold(self, name, straight=False):
        self.calls.append(("goto", name, straight))
        time.sleep(self.slow)
        if name not in self.poses:
            raise ValueError("no such pose")

    def stop(self):
        self.calls.append(("stop",))


def _with_server(actions, body):
    async def go():
        port = _free_port()
        task = asyncio.create_task(indicator.serve(lambda: {"joints": JOINTS, "position_time": 1.0}, "127.0.0.1", port, actions=actions))
        await asyncio.sleep(0.3)
        try:
            async with aiohttp.ClientSession() as s:
                await body(s, f"http://127.0.0.1:{port}")
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
    asyncio.run(go())


def test_pose_endpoints_and_errors():
    fa = FakeActions()

    async def body(s, base):
        async def post(path, data=None, raw=None, headers=JSON):
            async with s.post(base + path, data=raw if raw is not None else json.dumps(data), headers=headers) as r:
                return r.status, await r.json()
        async with s.get(base + "/poses") as r:
            assert r.status == 200 and (await r.json()) == {"poses": fa.list_poses(), "selected": None}
        st, d = await post("/poses", {})
        assert st == 200 and d["name"] == "P1"
        st, d = await post("/poses", {"name": "Z"})
        assert st == 200 and d["name"] == "Z"
        assert (await post("/poses", {"name": "dup"})) == (409, {"error": "already exists"})
        st, d = await post("/poses", {"name": "crash"})
        assert st == 500 and "KeyError" in d["error"]
        assert (await post("/poses/select", {"name": "A"}))[0] == 200 and fa.sel == "A"
        assert (await post("/poses/select", {"name": "nope"})) == (409, {"error": "no such pose"})
        assert (await post("/poses/rename", {"old": "Z", "new": "Y"}))[0] == 200 and "Y" in fa.poses
        assert (await post("/poses/rename", {"old": "Z", "new": "Q"}))[0] == 409
        assert (await post("/poses/rename", {"old": "Y"}))[0] == 400
        assert (await post("/poses/delete", {"name": "Y"}))[0] == 200 and "Y" not in fa.poses
        assert (await post("/poses/delete", {"name": "Y"}))[0] == 409
        assert (await post("/goto", {"name": "A"}))[0] == 200 and ("goto", "A", False) in fa.calls
        assert (await post("/goto", {"name": "A", "straight": True}))[0] == 200 and ("goto", "A", True) in fa.calls
        assert (await post("/goto", {"name": "nope"}))[0] == 409
        assert (await post("/goto", {"name": "A", "straight": "yes"}))[0] == 400
        assert (await post("/goto", {}))[0] == 400
        assert (await post("/goto", {"name": 5}))[0] == 400
        assert (await post("/goto", raw="{not json"))[0] == 400
        assert (await post("/goto", raw="[1]"))[0] == 400
        assert (await post("/stop", {}))[0] == 200 and ("stop",) in fa.calls
        # still serving after a 500
        async with s.get(base + "/state") as r:
            assert "tcp" in await r.json()
    _with_server(fa, body)


def test_post_requires_json_content_type():
    fa = FakeActions()

    async def body(s, base):
        for path in ("/goto", "/stop", "/poses", "/poses/select", "/poses/delete"):
            for headers in ({"Content-Type": "application/x-www-form-urlencoded"}, {"Content-Type": "text/plain"}, {}):
                async with s.post(base + path, data="name=A", headers=headers) as r:
                    assert r.status == 415
        assert fa.calls == []
    _with_server(fa, body)


def test_slow_action_does_not_stall_state():
    fa = FakeActions()
    fa.slow = 0.8

    async def body(s, base):
        slow = asyncio.create_task(s.post(base + "/goto", data=json.dumps({"name": "A"}), headers=JSON))
        await asyncio.sleep(0.1)
        t = time.time()
        async with s.get(base + "/state") as r:
            assert r.status == 200
        assert time.time() - t < 0.4
        async with await slow as r:
            assert r.status == 200
    _with_server(fa, body)


def test_read_only_without_actions():
    async def body(s, base):
        for path in ("/poses",):
            async with s.get(base + path) as r:
                assert r.status == 404
        for path in ("/poses", "/poses/select", "/poses/rename", "/poses/delete", "/goto", "/stop"):
            async with s.post(base + path, data="{}", headers=JSON) as r:
                assert r.status == 404
        async with s.get(base + "/") as r:
            assert r.status == 200 and "/state" in await r.text()
        async with s.get(base + "/state") as r:
            assert "tcp" in await r.json()
    _with_server(None, body)


def test_demo_arm_has_poses_and_goto_stops_when_calls_stop():
    arm = indicator.DemoArm()
    assert len(arm.list_poses()) == 3
    st = arm.state()
    assert set(indicator.KEYS) <= set(st) and len(st["counts"]) == 6 and st["goto"] is None
    start = list(arm.joints)
    name = arm.list_poses()[1]["name"]
    d0 = arm._diff(arm.poses[1]["joints"])
    for _ in range(4):
        arm.goto_hold(name)
        time.sleep(0.15)
    g = arm.state()["goto"]
    assert g["name"] == name and g["running"] and 0 < g["remaining_deg"] < d0 and arm.selected() == name
    time.sleep(0.7)                       # calls stopped: the arm stops by itself
    g = arm.state()["goto"]
    held = list(arm.joints)
    assert not g["running"] and held != start
    time.sleep(0.3)
    arm.state()
    assert arm.joints == held
    new = arm.save()
    assert new["name"] == "P1" and new["joints"] == held and len(arm.list_poses()) == 4
    with __import__("pytest").raises(ValueError):
        arm.save("P1")
    with __import__("pytest").raises(ValueError):
        arm.goto_hold("P1")               # already there
    arm.rename("P1", "Mine"); arm.select("Mine"); arm.delete("Mine")
    assert arm.selected() is None


def test_page_has_pose_panel_and_hold_safety():
    html = indicator.PAGE.read_text()
    for needle in ("Saved poses", "Hold to go", "touch-action:none", "pointercancel", "lostpointercapture",
                   "visibilitychange", "/goto", "/stop", "contextmenu"):
        assert needle in html
    assert "confirm(" not in html.replace("P.confirm", "").replace("function confirm", "")
    assert "http://" not in html and "https://" not in html
