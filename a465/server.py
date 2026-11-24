"""Web server for the A465: REST commands, a WebSocket state stream, and the web page.

    python3 -m a465.server            # http://<this machine>:8465

REST: GET /api/state, GET /api/model, GET /api/job, POST /api/<command> with a JSON body.
WebSocket /api/ws: pushes state 20 times a second; accepts {"op": <command>, ...}.
Commands: jog, halt, speed, armpower, home, goto, cycle, job, track, gripper, light, tool (see README).
"""
import argparse
import asyncio
import json
import pathlib

from aiohttp import WSMsgType, web

from . import model
from .gamepad import HELP, Gamepad
from .sim import FRAMES, RobotError, SimRobot

WEB = pathlib.Path(__file__).parent / "web"
SIM_HZ, PUSH_HZ = 50, 20
COMMANDS = ("jog", "halt", "speed", "armpower", "home", "goto", "cycle", "job", "track", "gripper", "light", "tool")
ROBOT = web.AppKey("robot", SimRobot)
GAMEPAD = web.AppKey("gamepad", Gamepad)
USE_GAMEPAD = web.AppKey("use_gamepad", bool)
SOCKETS = web.AppKey("sockets", set)


def _flag(body, key):
    if not isinstance(body.get(key), bool):
        raise RobotError(f"'{key}' must be true or false")
    return body[key]


def apply(robot, op, body, source):
    """Run one command against the robot. Raises RobotError with the reason if refused."""
    if not isinstance(body, dict):
        raise RobotError("command body must be a JSON object")
    if op == "jog":
        robot.jog(body.get("frame"), body.get("vec"), body.get("ttl", 0.3), source, body.get("aux", (0.0, 0.0)))
    elif op == "halt":
        robot.halt()
    elif op == "speed":
        robot.set_speed(body.get("percent"))
    elif op == "armpower":
        robot.set_arm_power(_flag(body, "on"))
    elif op == "goto":
        if "pose" not in body and "joints" not in body:
            raise RobotError("goto needs 'pose' or 'joints'")
        robot.goto(body["pose"] if "pose" in body else body["joints"], source)
    elif op == "home":
        robot.home(source)
    elif op == "cycle":
        robot.cycle_start(source)
    elif op == "job":
        robot.set_job(body.get("picks"), body.get("places"), body.get("plane_z"), body.get("zones"))
    elif op == "track":
        robot.set_track(body.get("fitted"), body.get("moves"), body.get("carousel"))
    elif op == "gripper":
        robot.set_gripper(_flag(body, "open"))
    elif op == "light":
        robot.set_light(_flag(body, "on"))
    elif op == "tool":
        robot.set_tool(_flag(body, "on"))
    else:
        raise RobotError(f"unknown command '{op}'; commands are {', '.join(COMMANDS)}")


def snapshot(app):
    return {"robot": app[ROBOT].state(), "gamepad": app[GAMEPAD].status()}


async def get_state(request):
    return web.json_response(snapshot(request.app))


async def get_model(request):
    return web.json_response({**model.describe(), "frames": FRAMES, "commands": COMMANDS, "gamepad_help": HELP})


def job_report(robot):
    """The grid, the chosen cells, and the cycle planned from where the arm is now."""
    return robot.job.describe(robot.plan(), robot.track)


async def get_job(request):
    return web.json_response(job_report(request.app[ROBOT]))


async def post_command(request):
    robot, op = request.app[ROBOT], request.match_info["op"]
    try:
        body = await request.json() if request.can_read_body else {}
        apply(robot, op, body, "api")
    except json.JSONDecodeError:
        return web.json_response({"error": "body is not valid JSON"}, status=400)
    except RobotError as err:
        return web.json_response({"error": str(err)}, status=400)
    reply = {"ok": True, "robot": robot.state()}
    if op in ("job", "track"):
        reply["job"] = job_report(robot)
    return web.json_response(reply)


async def websocket(request):
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    request.app[SOCKETS].add(ws)
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            op = None
            try:
                body = json.loads(msg.data)
                op = body.get("op") if isinstance(body, dict) else None
                apply(request.app[ROBOT], op, body, "web")
            except json.JSONDecodeError:
                await ws.send_json({"error": "message is not valid JSON"})
            except RobotError as err:
                await ws.send_json({"error": str(err), "op": op})
    finally:
        request.app[SOCKETS].discard(ws)
    return ws


async def index(request):
    return web.FileResponse(WEB / "index.html")


async def _simulate(app):
    loop = asyncio.get_running_loop()
    last = loop.time()
    while True:
        await asyncio.sleep(1 / SIM_HZ)
        now = loop.time()
        app[ROBOT].step(min(now - last, 0.1))
        last = now


async def _push(app):
    while True:
        await asyncio.sleep(1 / PUSH_HZ)
        if not app[SOCKETS]:
            continue
        text = json.dumps(snapshot(app))
        for ws in list(app[SOCKETS]):
            try:
                await ws.send_str(text)
            except (ConnectionError, RuntimeError):
                app[SOCKETS].discard(ws)


async def _background(app):
    jobs = [_simulate(app), _push(app)]
    if app[USE_GAMEPAD]:
        jobs.append(app[GAMEPAD].run())
    tasks = [asyncio.ensure_future(j) for j in jobs]
    yield
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


@web.middleware
async def _no_cache(request, handler):
    response = await handler(request)
    response.headers.setdefault("Cache-Control", "no-cache")
    return response


def build_app(use_gamepad=True, gamepad_device=None, track=True, carousel=True):
    """`track` and `carousel` say whether those axes start fitted (the command line starts without them)."""
    app = web.Application(middlewares=[_no_cache])
    app[ROBOT] = SimRobot()
    app[ROBOT].job.track_fitted = track
    app[ROBOT].carousel_fitted = carousel
    app[GAMEPAD] = Gamepad(app[ROBOT], gamepad_device)
    app[USE_GAMEPAD] = use_gamepad
    app[SOCKETS] = set()
    app.add_routes([
        web.get("/", index),
        web.get("/api/state", get_state),
        web.get("/api/model", get_model),
        web.get("/api/job", get_job),
        web.get("/api/ws", websocket),
        web.post("/api/{op}", post_command),
        web.static("/static", WEB),
    ])
    app.cleanup_ctx.append(_background)
    return app


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0", help="address to listen on (default: all interfaces)")
    ap.add_argument("--port", type=int, default=8465)
    ap.add_argument("--no-gamepad", action="store_true", help="do not look for a game controller")
    ap.add_argument("--gamepad", metavar="DEVICE", help="use this input device (e.g. /dev/input/event17) instead of the first controller found")
    ap.add_argument("--track", action="store_true", help="start with the track fitted (default: not fitted; the page can change it)")
    ap.add_argument("--carousel", action="store_true", help="start with the carousel fitted (default: not fitted; the page can change it)")
    return ap.parse_args(argv)


def main():
    args = parse_args()
    web.run_app(build_app(not args.no_gamepad, args.gamepad, args.track, args.carousel), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
