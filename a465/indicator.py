"""Live position and speed indicator page for the real arm.

    python3 -m a465.indicator --demo [--port 8466]     # made-up moving state, no arm needed

From the pad program:  await indicator.serve(get_state)   (runs until cancelled)
GET / is the page, GET /state is get_state()'s dict plus now, tcp, tcp_speed and limits.
"""
import argparse
import asyncio
import math
import pathlib
import threading
import time
import datetime

from aiohttp import web

from . import model

PAGE = pathlib.Path(__file__).parent / "web" / "indicator.html"
STALE_S = 1.5
KEYS = ("backend", "frame", "speed", "max_speed", "joint_caps", "frame_caps", "joints", "joint_speeds",
        "position_time", "moving", "command", "jogd", "notice", "controller_message", "pad", "limit",
        "selected", "goto", "counts")


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


def _json(data, status=200):
    return web.json_response(data, status=status, headers={"Cache-Control": "no-store"})


def page_version(text=None):
    """A short hash of the page file: the page reloads itself when the server's copy is newer than the one it runs."""
    import hashlib
    return hashlib.sha1((PAGE.read_text() if text is None else text).encode()).hexdigest()[:10]


async def serve(get_state, host="0.0.0.0", port=8466, actions=None):
    """actions None = read-only page. Otherwise see the pose routes below (all POSTs need Content-Type: application/json)."""
    holder = {"prev": None}

    async def page(request):
        text = PAGE.read_text()
        return web.Response(text=text.replace("__PAGE_VERSION__", page_version(text)), content_type="text/html",
                            headers={"Cache-Control": "no-store"})

    async def state(request):
        try:
            out, holder["prev"] = augment(get_state() or {}, holder["prev"])
            out["page_version"] = page_version()
            return web.json_response(out, headers={"Cache-Control": "no-store"})
        except Exception as e:  # keep serving
            return web.json_response({"error": f"{type(e).__name__}: {e}"})

    async def run(fn, *args, **kw):
        """Run an action method off the event loop (so a slow one cannot stall /state); map errors to HTTP."""
        try:
            return _json(await asyncio.to_thread(fn, *args, **kw))
        except ValueError as e:
            return _json({"error": str(e)}, 409)
        except Exception as e:
            return _json({"error": f"{type(e).__name__}: {e}"}, 500)

    async def body(request, *fields):
        """The JSON object body, or a 400/415 response; fields are the required string keys."""
        if request.content_type != "application/json":
            return None, _json({"error": "Content-Type must be application/json"}, 415)
        try:
            d = await request.json()
        except Exception:
            return None, _json({"error": "bad JSON"}, 400)
        if not isinstance(d, dict):
            return None, _json({"error": "JSON body must be an object"}, 400)
        for f in fields:
            if not isinstance(d.get(f), str) or not d[f].strip():
                return None, _json({"error": f"missing field: {f}"}, 400)
        return d, None

    async def poses_get(request):
        def f():
            return {"poses": actions.list_poses(), "selected": actions.selected()}
        return await run(f)

    async def poses_save(request):
        d, err = await body(request)
        if err:
            return err
        name = d.get("name")
        if name is not None and not isinstance(name, str):
            return _json({"error": "name must be text"}, 400)
        return await run(actions.save, name or None)

    async def select(request):
        d, err = await body(request, "name")
        if err:
            return err
        return await run(lambda: (actions.select(d["name"]), {"ok": True})[1])

    async def rename(request):
        d, err = await body(request, "old", "new")
        if err:
            return err
        return await run(lambda: (actions.rename(d["old"], d["new"]), {"ok": True})[1])

    async def delete(request):
        d, err = await body(request, "name")
        if err:
            return err
        return await run(lambda: (actions.delete(d["name"]), {"ok": True})[1])

    async def goto(request):
        d, err = await body(request, "name")
        if err:
            return err
        straight = d.get("straight", False)
        if not isinstance(straight, bool):
            return _json({"error": "straight must be true or false"}, 400)
        return await run(lambda: (actions.goto_hold(d["name"], straight=straight), {"ok": True})[1])

    async def stop(request):
        d, err = await body(request)
        if err:
            return err
        why = d.get("why") if isinstance(d, dict) else None
        why = why[:60] if isinstance(why, str) and why else "no reason sent: the browser is running an older copy of the page, reload it"
        print(f"{'':10}page: hold ended ({why})", flush=True)      # into the pad program's log
        return await run(lambda: (actions.stop(), {"ok": True})[1])

    app = web.Application()
    app.router.add_get("/", page)
    app.router.add_get("/state", state)
    if actions is not None:
        app.router.add_get("/poses", poses_get)
        app.router.add_post("/poses", poses_save)
        app.router.add_post("/poses/select", select)
        app.router.add_post("/poses/rename", rename)
        app.router.add_post("/poses/delete", delete)
        app.router.add_post("/goto", goto)
        app.router.add_post("/stop", stop)
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
        "selected": None, "goto": None, "counts": None,
    }


class DemoArm:
    """Fake state + fake pose actions for --demo: three poses; goto_hold moves the demo joints while it keeps being called."""
    HOLD_S = 0.5            # the arm stops this long after the last goto_hold call
    SPEED_DPS = 20.0

    def __init__(self):
        self.lock = threading.RLock()
        self.joints = [0.0, -30.0, 30.0, 0.0, 20.0, 0.0]
        self.poses = []
        self.sel = None
        self.goto = None            # {"name","straight","target","last","start_diff"}
        self.last_t = time.time()
        for n, j in (("Home", [0, 0, 0, 0, 0, 0]), ("Left", [40, -40, 40, 10, 30, 0]),
                     ("Right", [-45, -20, 60, -10, 45, 20])):
            self._add(n, j)

    def _add(self, name, joints):
        p, rot, _, _ = model.forward(joints)
        yaw, pitch, roll = model.yaw_pitch_roll(rot)
        d = {"name": name, "joints": [float(v) for v in joints], "counts": [int(round(v * 100)) for v in joints],
             "x": float(p[0]), "y": float(p[1]), "z": float(p[2]), "yaw": yaw, "pitch": pitch, "roll": roll,
             "saved": datetime.datetime.now().isoformat(timespec="seconds")}
        self.poses.append(d)
        return dict(d)

    def _find(self, name):
        for p in self.poses:
            if p["name"] == name:
                return p
        raise ValueError(f"no pose named {name}")

    def _diff(self, target):
        return max(abs(a - b) for a, b in zip(self.joints, target))

    def _step(self, now):
        g = self.goto
        dt = min(now - self.last_t, 0.5)
        self.last_t = now
        if not g:
            return
        if now - g["last"] > self.HOLD_S:
            g["running"] = False
            return
        if g["arrived"]:
            return
        step = self.SPEED_DPS * dt
        d = self._diff(g["target"])
        if d <= step:
            self.joints = list(g["target"])
            g["arrived"], g["running"] = True, False
        else:
            f = step / d
            self.joints = [a + (b - a) * f for a, b in zip(self.joints, g["target"])]
            g["running"] = True

    def list_poses(self):
        with self.lock:
            return [dict(p) for p in self.poses]

    def selected(self):
        with self.lock:
            return self.sel

    def select(self, name):
        with self.lock:
            self.sel = self._find(name)["name"]

    def save(self, name=None):
        with self.lock:
            if name is None:
                n = 1
                while any(p["name"] == f"P{n}" for p in self.poses):
                    n += 1
                name = f"P{n}"
            name = name.strip()
            if any(p["name"] == name for p in self.poses):
                raise ValueError(f"a pose named {name} already exists")
            return self._add(name, self.joints)

    def rename(self, old, new):
        with self.lock:
            p = self._find(old)
            new = new.strip()
            if new != old and any(q["name"] == new for q in self.poses):
                raise ValueError(f"a pose named {new} already exists")
            p["name"] = new
            if self.sel == old:
                self.sel = new

    def delete(self, name):
        with self.lock:
            self.poses.remove(self._find(name))
            if self.sel == name:
                self.sel = None

    def goto_hold(self, name, straight=False):
        with self.lock:
            now = time.time()
            self._step(now)
            p = self._find(name)
            g = self.goto
            if not g or g["name"] != name or g["straight"] != straight or now - g["last"] > self.HOLD_S:
                if self._diff(p["joints"]) < 0.05:
                    raise ValueError(f"already at {name}")
                self.goto = g = {"name": name, "straight": straight, "target": list(p["joints"]),
                                 "arrived": False, "running": True}
            g["last"] = now
            self.sel = name

    def stop(self):
        with self.lock:
            self._step(time.time())
            if self.goto and not self.goto["arrived"]:
                self.goto = None

    def state(self, t=None):
        with self.lock:
            now = time.time() if t is None else t
            before = list(self.joints)
            self._step(now)
            st = demo_state(now)
            st["joints"] = list(self.joints)
            st["joint_speeds"] = [(a - b) / 0.2 for a, b in zip(self.joints, before)] if self.goto and self.goto["running"] else [0.0] * 6
            st["moving"] = bool(self.goto and self.goto["running"])
            st["command"] = None
            st["selected"] = self.sel
            st["counts"] = [int(round(v * 100)) for v in self.joints]
            g = self.goto
            st["goto"] = None if not g else {"name": g["name"], "straight": g["straight"], "speed": 20,
                                             "running": bool(g["running"]), "arrived": bool(g["arrived"]),
                                             "remaining_deg": self._diff(g["target"])}
            return st


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
        arm = DemoArm()
        asyncio.run(serve(arm.state, args.host, args.port, actions=arm))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
