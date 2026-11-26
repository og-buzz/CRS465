import asyncio
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
