"""Live position and speed indicator page for the real arm.

    python3 -m a465.indicator --demo [--port 8466]     # made-up moving state, no arm needed

From the pad program:  await indicator.serve(get_state)   (runs until cancelled)
GET / is the page, GET /state is get_state()'s dict plus now, tcp, tcp_speed and limits.
"""
import argparse
import asyncio
import math
import pathlib
import time

from aiohttp import web

from . import model

PAGE = pathlib.Path(__file__).parent / "web" / "indicator.html"
STALE_S = 1.5
KEYS = ("backend", "frame", "speed", "max_speed", "joint_caps", "frame_caps", "joints", "joint_speeds",
        "position_time", "moving", "command", "jogd", "notice", "controller_message", "pad", "limit")


def _joints(state):
    j = state.get("joints")
    if isinstance(j, (list, tuple)) and len(j) == 6 and all(isinstance(v, (int, float)) for v in j):
        return [float(v) for v in j]
    return None


def augment(state, prev=None, now=None):
    """Return (augmented dict, new prev). prev is None or {"new": (t, point), "old": (t, point) or None}."""
    now = time.time() if now is None else now
    out = dict(state or {})
    out["now"] = now
    out["limits"] = [[lo, hi] for lo, hi in model.LIMITS_DEG]
    gl = out.get("guard_limits")        # the jog guard's own table (it may differ from the model's): the bars use it
    if isinstance(gl, (list, tuple)) and len(gl) == 6 and all(isinstance(p, (list, tuple)) and len(p) == 2 for p in gl):
        out["limits"] = [[float(lo), float(hi)] for lo, hi in gl]
    out["tcp"] = None
    out["tcp_speed"] = 0.0
    prev = dict(prev) if prev else {"new": None, "old": None}
    joints = _joints(out)
    if joints is None:
        return out, prev
    p, rot, _, _ = model.forward(joints)
    yaw, pitch, roll = model.yaw_pitch_roll(rot)
    out["tcp"] = {"x": float(p[0]), "y": float(p[1]), "z": float(p[2]), "yaw": yaw, "pitch": pitch, "roll": roll}
    t = out.get("position_time")
    if isinstance(t, (int, float)):
        pt = (float(p[0]), float(p[1]), float(p[2]))
        if prev["new"] is None or t != prev["new"][0]:
            prev = {"new": (float(t), pt), "old": prev["new"]}
        (t1, p1), old = prev["new"], prev["old"]
        if old is not None and now - t1 <= STALE_S and t1 > old[0]:
            out["tcp_speed"] = math.dist(p1, old[1]) / (t1 - old[0])
    return out, prev


async def serve(get_state, host="0.0.0.0", port=8466):
    holder = {"prev": None}

    async def page(request):
        return web.Response(text=PAGE.read_text(), content_type="text/html", headers={"Cache-Control": "no-store"})

    async def state(request):
        try:
            out, holder["prev"] = augment(get_state() or {}, holder["prev"])
            return web.json_response(out, headers={"Cache-Control": "no-store"})
        except Exception as e:  # keep serving
            return web.json_response({"error": f"{type(e).__name__}: {e}"})

    app = web.Application()
    app.router.add_get("/", page)
    app.router.add_get("/state", state)
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    try:
        await web.TCPSite(runner, host, port).start()
        await asyncio.Event().wait()
    finally:
        await runner.cleanup()


def demo_state(t=None):
    """A made-up state that changes with time, exercising every part of the page."""
    t = time.time() if t is None else t
    lim = model.LIMITS_DEG

    def pose(tt):
        return [0.5 * (lo + hi) + 0.6 * (hi - lo) / 2 * math.sin(tt * (0.2 + 0.07 * i) + i)
                for i, (lo, hi) in enumerate(lim)]

    pos_t = t - 0.05
    a, b = pose(pos_t), pose(pos_t - 0.1)
    speeds = [(x - y) / 0.1 for x, y in zip(a, b)]
    phase = int(t // 20) % 3
    frame = ("joint", "world", "tool")[phase]
    moving = int(t) % 12 < 8
    sec = t % 60
    speed = 40 + int(30 * math.sin(t / 30))
    cap = 25 if frame == "joint" else 15
    axis = int(t // 4) % 6 + 1
    return {
        "backend": "c500+jogd", "frame": frame, "speed": speed, "max_speed": 100,
        "joint_caps": {"2": 25, "3": 25}, "frame_caps": {"world": 15, "tool": 15},
        "joints": a, "joint_speeds": speeds if moving else [0.0] * 6, "position_time": pos_t,
        "moving": moving,
        "command": {"kind": "jog", "source": "gamepad", "axis": axis, "sign": 1 if int(t // 8) % 2 else -1,
                    "speed": min(speed, cap), "frame": frame} if moving else None,
        "jogd": "dead" if 40 <= sec < 46 else "running",
        "notice": "Demo mode: made-up numbers" if 20 <= sec < 30 else "",
        "controller_message": {"text": "S: Arm power is OFF | Error 0x7000204 | Collision error on axis 2",
                               "at": t} if 40 <= sec < 46 else None,
        "limit": ({"joint": 3, "limit": -112.0, "angle": -110.4, "blocked": sec % 4 < 2} if 20 <= sec < 30 else None),
        "pad": {"connected": True, "name": "Wireless Controller", "l1": moving},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--demo", action="store_true", help="serve a made-up moving state")
    ap.add_argument("--port", type=int, default=8466)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()
    if not args.demo:
        ap.error("standalone use needs --demo; the pad program calls serve() with the real state")
    print(f"demo indicator on http://{args.host}:{args.port}/")
    try:
        asyncio.run(serve(demo_state, args.host, args.port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
