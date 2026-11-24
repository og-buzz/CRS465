import asyncio

from aiohttp.test_utils import TestClient, TestServer

from a465.server import ROBOT, build_app


def with_client(scenario):
    async def go():
        async with TestClient(TestServer(build_app(use_gamepad=False))) as client:
            await scenario(client)
    asyncio.run(go())


async def post(client, op, body=None):
    response = await client.post(f"/api/{op}", json=body or {})
    return response.status, await response.json()


def test_state_and_model():
    async def scenario(client):
        state = await (await client.get("/api/state")).json()
        assert state["robot"]["backend"] == "mock" and state["robot"]["pose"]["x"] == 406.0
        assert state["gamepad"]["connected"] is False
        described = await (await client.get("/api/model")).json()
        assert [j["name"] for j in described["joints"]][0] == "waist" and len(described["joints"]) == 6
    with_client(scenario)


def test_rest_jog_moves_the_robot_then_it_stops_on_its_own():
    async def scenario(client):
        assert (await post(client, "jog", {"frame": "joint", "vec": [1, 0, 0, 0, 0, 0]}))[0] == 400  # power off
        assert (await post(client, "armpower", {"on": True}))[0] == 200
        status, body = await post(client, "jog", {"frame": "joint", "vec": [1, 0, 0, 0, 0, 0], "ttl": 0.2})
        assert status == 200 and body["robot"]["command"] == {"kind": "jog", "frame": "joint", "source": "api"}
        await asyncio.sleep(0.15)
        assert (await (await client.get("/api/state")).json())["robot"]["moving"]
        await asyncio.sleep(0.8)
        robot = (await (await client.get("/api/state")).json())["robot"]
        assert not robot["moving"] and robot["command"] is None and robot["joints"][0] > 0
    with_client(scenario)


def test_bad_requests_get_400_with_a_reason():
    async def scenario(client):
        for op, body, text in [
            ("fly", {}, "unknown command"),
            ("speed", {"percent": 500}, "1 to 100"),
            ("armpower", {"on": "yes"}, "true or false"),
            ("goto", {}, "needs 'pose' or 'joints'"),
            ("jog", [1, 2], "JSON object"),
        ]:
            status, reply = await post(client, op, body)
            assert status == 400 and text in reply["error"], (op, reply)
        response = await client.post("/api/halt", data="{not json", headers={"Content-Type": "application/json"})
        assert response.status == 400 and "not valid JSON" in (await response.json())["error"]
        assert (await post(client, "halt"))[0] == 200
    with_client(scenario)


def test_websocket_streams_state_and_takes_commands():
    async def scenario(client):
        ws = await client.ws_connect("/api/ws")
        first = await ws.receive_json(timeout=2)
        assert first["robot"]["speed"] == 10
        await ws.send_json({"op": "speed", "percent": 40})
        await ws.send_json({"op": "jog", "frame": "world", "vec": [1, 0, 0, 0, 0, 0]})
        seen_error = seen_speed = False
        for _ in range(20):
            msg = await ws.receive_json(timeout=2)
            seen_error |= msg.get("error") == "arm power is off" and msg.get("op") == "jog"
            seen_speed |= msg.get("robot", {}).get("speed") == 40
        assert seen_error and seen_speed
        await ws.close()
    with_client(scenario)


def test_page_and_assets_are_served():
    async def scenario(client):
        page = await client.get("/")
        assert page.status == 200 and "text/html" in page.headers["Content-Type"]
        for path in ("/static/app.js", "/static/arm.js", "/static/style.css", "/static/vendor/three.module.min.js"):
            assert (await client.get(path)).status == 200, path
    with_client(scenario)


def test_home_cycle_outputs_and_track_over_rest():
    async def scenario(client):
        assert (await post(client, "cycle"))[1]["error"] == "arm power is off"
        await post(client, "armpower", {"on": True})
        assert (await post(client, "cycle"))[1]["error"] == "not homed"
        status, body = await post(client, "home")
        assert status == 200 and body["robot"]["command"] == {"kind": "home", "source": "api"}
        status, body = await post(client, "light", {"on": True})
        assert status == 200 and body["robot"]["light"] is True and body["robot"]["tool"] is False
        assert (await post(client, "tool", {"on": 1}))[1]["error"] == "'on' must be true or false"
        status, body = await post(client, "jog", {"frame": "joint", "vec": [0] * 6, "aux": [1, 0], "ttl": 0.3})
        assert status == 200                           # a jog replaces the homing that was running
        await asyncio.sleep(0.25)
        robot = (await (await client.get("/api/state")).json())["robot"]
        assert robot["track"] > 0 and robot["homed"] is False
        assert (await post(client, "jog", {"frame": "joint", "vec": [0] * 6, "aux": [1]}))[1]["error"] == "aux must be two numbers"
    with_client(scenario)


def test_job_over_rest_plans_checks_and_runs():
    async def scenario(client):
        job = await (await client.get("/api/job")).json()
        assert job["picks"] == [] and job["plan"]["ok"] is False and len(job["grid"]["reach"]) == job["grid"]["rows"]
        status, body = await post(client, "job", {"picks": ["C10"], "places": ["L20"]})
        assert status == 200 and body["robot"]["job_version"] == 1
        plan = body["job"]["plan"]
        assert plan["ok"] and len(plan["steps"]) == 9 and plan["pairs"][0]["place"] == "L20" and plan["path"]
        assert body["robot"]["parts"] == [{"x": 350.0, "y": -450.0, "z": 0.0, "held": False}]
        assert (await post(client, "job", {"picks": ["Q1"], "places": []}))[1]["error"].startswith("'Q1' is outside the grid")
        assert (await post(client, "job", {"picks": "C10"}))[1]["error"] == "'picks' and 'places' must be lists of cell names"
        status, body = await post(client, "job", {"picks": ["A1"], "places": ["L20"]})
        assert status == 200 and body["job"]["plan"]["problems"] == ["pick 1 (A1): out of reach"]
        await post(client, "armpower", {"on": True})
        client.app[ROBOT].homed = True
        assert (await post(client, "cycle"))[1]["error"] == "cycle not started: pick 1 (A1): out of reach"
        await post(client, "job", {"picks": ["C10"], "places": ["L20"]})
        await post(client, "speed", {"percent": 100})
        status, body = await post(client, "cycle")
        assert status == 200 and body["robot"]["command"]["kind"] == "cycle" and body["robot"]["command"]["label"] == "move above pick 1"
        assert (await post(client, "job", {"picks": [], "places": []}))[1]["error"] == "the cycle is running: halt it before changing cells"
        await asyncio.sleep(0.4)
        robot = (await (await client.get("/api/state")).json())["robot"]
        assert robot["moving"] and robot["notice"] == ""
    with_client(scenario)
