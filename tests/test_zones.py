"""Zones (pick areas, place areas, keep-outs) and the two track settings."""
import asyncio

import numpy as np
import pytest
from aiohttp.test_utils import TestClient, TestServer

from a465 import model, planner
from a465.server import ROBOT, build_app
from a465.sim import RobotError, SimRobot

READY = model.POSES_DEG["ready"]
DT = 0.02


def keepout(x0, y0, x1, y1, top):
    return {"kind": "keepout", "x0": x0, "y0": y0, "x1": x1, "y1": y1, "top": top}


def area(kind, x0, y0, x1, y1):
    return {"kind": kind, "x0": x0, "y0": y0, "x1": x1, "y1": y1}


# A keep-out beside the track that the direct move from C10 to L20 swings through, and that a higher transit clears.
BESIDE_RAIL = keepout(700, -420, 800, -200, 300)
# One that no travel height can clear: the place cell itself has to be reached over it.
IN_THE_WAY = keepout(900, 200, 1200, 400, 600)


def inside_zone(plan, keepouts):
    """The first (step label, zone number) where any checked sample of the plan is inside a keep-out zone."""
    for step in plan.steps:
        if step.kind != "grip":
            number = planner.samples_hit(step.samples, keepouts)
            if number is not None:
                return step.label, number
    return None


def run(robot, seconds, refresh=None):
    for _ in range(round(seconds / DT)):
        if refresh:
            refresh()
        robot.step(DT)


@pytest.fixture
def robot():
    r = SimRobot()
    r.set_arm_power(True)
    r.homed = True
    return r


# ---- validation

def test_zones_are_normalised_and_keep_their_order():
    job = planner.Job()
    job.set([], [], zones=[keepout(800, 300, 700, -100, 250), area("pick", 0, 0, 300, 200)])
    assert job.zones[0] == {"kind": "keepout", "x0": 700.0, "y0": -100.0, "x1": 800.0, "y1": 300.0, "top": 250.0}
    assert job.zones[1]["kind"] == "pick" and job.zones[1]["top"] is None


@pytest.mark.parametrize("zone, text", [
    ({"kind": "wall", "x0": 0, "y0": 0, "x1": 100, "y1": 100, "top": 100}, "kind must be one of"),
    (keepout(0, 0, 100, "far", 100), "'y1' must be a number"),
    (keepout(0, 0, 100, True, 100), "'y1' must be a number"),
    (keepout(0, 0, 100, float("nan"), 100), "'y1' must be a number"),
    (keepout(0, 0, 99999, 100, 100), "'x1' must be a number"),
    (keepout(0, 0, 0, 100, 100), "at least 10 mm"),
    (keepout(0, 0, 100, 5, 100), "at least 10 mm"),
    (keepout(0, 0, 100, 100, 0), "'top' must be a height above the plane"),
    (keepout(0, 0, 100, 100, -50), "'top' must be a height above the plane"),
    (keepout(0, 0, 100, 100, 1500.5), "at most 1500"),
    (keepout(0, 0, 100, 100, None), "'top' must be a height above the plane"),
    ("a zone", "must be an object"),
])
def test_bad_zones_are_refused_with_a_reason_and_leave_the_job_alone(zone, text):
    job = planner.Job()
    job.set(["C10"], ["L20"], zones=[keepout(700, -420, 800, -200, 300)])
    with pytest.raises(planner.JobError, match=text):
        job.set(["D12"], [], zones=[zone])
    assert job.picks == ["C10"] and len(job.zones) == 1 and job.zones[0]["top"] == 300.0


def test_at_most_twenty_zones_and_the_list_must_be_a_list():
    job = planner.Job()
    job.set([], [], zones=[area("pick", 0, 0, 100, 100)] * 20)
    with pytest.raises(planner.JobError, match="at most 20"):
        job.set([], [], zones=[area("pick", 0, 0, 100, 100)] * 21)
    with pytest.raises(planner.JobError, match="list of zones"):
        job.set([], [], zones="none")
    assert len(job.zones) == 20


def test_omitting_zones_keeps_them_and_an_empty_list_clears_them():
    job = planner.Job()
    job.set([], [], zones=[keepout(700, -420, 800, -200, 300)])
    job.set(["C10"], ["L20"])
    assert len(job.zones) == 1
    job.set(["C10"], ["L20"], zones=[])
    assert job.zones == []


def test_a_keep_out_must_stay_above_the_plane_when_the_plane_is_raised():
    job = planner.Job()
    job.set([], [], zones=[keepout(700, -420, 800, -200, 300)])
    with pytest.raises(planner.JobError, match="above the plane"):
        job.set([], [], plane_z=350)
    assert job.plane_z == 0.0
    job.set([], [], plane_z=100)
    assert job.plane_z == 100.0


# ---- areas

def test_an_area_contributes_the_cells_whose_centre_is_inside_row_a_first_then_by_column():
    job = planner.Job()
    job.set([], [], zones=[area("pick", 200, -460, 460, -340)])       # centres 250, 350, 450 by -450, -350: columns 9-11, rows C-D
    picks, places, report = job.resolve()
    assert picks == ["C9", "C10", "C11", "D9", "D10", "D11"] == report[0]["cells"]
    assert places == [] and report[0]["left_out"] == 0


def test_clicked_cells_come_first_then_areas_in_zone_order_without_duplicates():
    job = planner.Job()
    job.set(["D10"], ["L20"], zones=[area("pick", 200, -460, 460, -340), area("pick", 300, -460, 560, -340)])
    picks, places, report = job.resolve()
    assert picks == ["D10", "C9", "C10", "C11", "D9", "D11", "C12", "D12"]
    assert report[0]["cells"] == ["C9", "C10", "C11", "D9", "D10", "D11"]     # D10 still counts for the area that holds it
    assert report[1]["cells"] == ["C10", "C11", "C12", "D10", "D11", "D12"]
    assert places == ["L20"]


def test_cells_that_cannot_be_used_are_left_out_and_counted_per_zone_with_one_note_each():
    job = planner.Job()
    job.set(["C10"], [], zones=[
        area("place", 1200, 300, 1500, 600),                            # nine cells, all fine
        area("pick", -600, -700, -300, -300),                           # the far corner: some out of reach
        area("pick", -100, -160, 100, 160),                             # over the track
    ])
    picks, places, report = job.resolve()
    assert len(report[0]["cells"]) == 9 and report[0]["left_out"] == 0 and places == report[0]["cells"]
    assert report[1]["left_out"] == 9 and len(report[1]["cells"]) == 3
    assert report[2]["left_out"] == 8 and report[2]["cells"] == []
    plan = job.plan(READY, 0.0, 100)
    left = " left out (out of reach, over the track or inside a keep-out zone)"
    assert [n for n in plan.notes if "left out" in n] == [f"pick area (zone 2): 9 cells{left}", f"pick area (zone 3): 8 cells{left}"]
    one = planner.Job()
    one.set([], [], zones=[area("pick", -100, -160, 100, -40)])
    assert one.plan(READY, 0.0, 100).notes == ["pick area (zone 1): 4 cells left out (out of reach, over the track or inside a keep-out zone)"]


def test_a_cell_that_is_both_a_pick_and_a_place_is_refused_naming_the_cells():
    job = planner.Job()
    job.set(["C10"], ["L20"])
    with pytest.raises(planner.JobError, match="C10 would be both a pick and a place"):
        job.set(["C10"], [], zones=[area("place", 300, -500, 400, -400)])
    with pytest.raises(planner.JobError, match="C9, C10, C11 would be both"):
        job.set([], [], zones=[area("pick", 200, -500, 500, -400), area("place", 200, -500, 500, -400)])
    with pytest.raises(planner.JobError, match=r"L1, L2, L3, L4, L5, L6 and 4 more would be both"):
        job.set([], [], zones=[area("pick", -600, 400, 400, 500), area("place", -600, 400, 400, 500)])
    assert job.picks == ["C10"] and job.zones == []                      # nothing changed


def test_pick_n_goes_to_place_n_across_clicked_cells_and_areas():
    job = planner.Job()
    job.set(["C10"], ["L20"], zones=[area("pick", 500, -460, 600, -440), area("place", 1500, 340, 1600, 360)])
    plan = job.plan(READY, 0.0, 100)
    assert plan.ok and [(p["pick"], p["place"]) for p in plan.pairs] == [("C10", "L20"), ("C12", "K22")]
    assert [s.cell for s in plan.steps if s.kind == "grip"] == ["C10", "L20", "C12", "K22"]


# ---- keep-out cells and the reach map

def test_a_cell_overlapping_a_keep_out_is_unusable_and_has_its_own_letter():
    job = planner.Job()
    before = job.grid()["reach"]
    job.set([], [], zones=[keepout(250, -450, 300, -420, 200)])          # a sliver inside C9 (x 200..300, y -500..-400)
    reach = job.grid()["reach"]
    assert reach[2][8] == "k" and before[2][8] == "y"
    assert reach[2][9] == before[2][9] and reach[1] == before[1]       # the cell that only touches it is untouched
    job.set([], [], zones=[keepout(300, -500, 400, -400, 200)])          # exactly C10's square: C9 and C11 only touch it
    reach = job.grid()["reach"]
    assert reach[2][9] == "k" and reach[2][8] == before[2][8] and reach[2][10] == before[2][10]


def test_the_reach_map_depends_on_the_zones_and_the_track_settings():
    job = planner.Job()
    plain = job.grid()["reach"]
    job.set([], [], zones=[keepout(300, -450, 400, -350, 200)])
    assert job.grid()["reach"] != plain
    job.set([], [], zones=[])
    assert job.grid()["reach"] == plain
    job.track_moves = False
    assert job.grid(0.0)["reach"] != plain and job.grid(1000.0)["reach"] != job.grid(0.0)["reach"]
    job.track_fitted = False
    assert job.grid(0.0)["reach"][6] != "t" * planner.COLS


def test_keep_out_cells_are_left_out_of_an_area():
    job = planner.Job()
    job.set(["C10"], [], zones=[area("place", 1200, 300, 1500, 600), keepout(1300, 400, 1400, 450, 100)])
    picks, places, report = job.resolve()
    assert "L20" not in places and len(places) == 8 and report[0]["left_out"] == 1 and len(report[0]["cells"]) == 8


# ---- the planner against keep-outs

def test_a_transit_the_direct_move_would_make_through_a_zone_is_raised_to_clear_it():
    job = planner.Job()
    job.set(["C10"], ["L20"])
    direct = job.plan(READY, 0.0, 100)
    job.set(["C10"], ["L20"], zones=[BESIDE_RAIL])
    keepouts = job.keepouts()
    assert inside_zone(direct, keepouts) is not None                     # without the raise, the arm goes through it
    plan = job.plan(READY, 0.0, 100)
    assert plan.ok and plan.problems == []
    assert inside_zone(plan, keepouts) is None                           # every checked sample of every step is clear
    assert [n for n in plan.notes if "raised" in n] == ["move above place 1 (L20) is raised to a travel height of 390 mm to clear keep-out zone 1"]
    assert plan.steps[3].label == "lift from pick 1"
    assert float(model.fingertip(plan.steps[3].samples[-1][:6], plan.steps[3].samples[-1][6])[2]) == pytest.approx(390.0)
    assert plan.seconds > direct.seconds


def test_a_zone_that_nothing_clears_makes_the_plan_not_ok_and_names_the_step_and_zone():
    job = planner.Job()
    job.set(["C10"], ["L20"], zones=[area("pick", 0, 0, 100, 100), IN_THE_WAY])
    plan = job.plan(READY, 0.0, 100)
    assert not plan.ok
    assert plan.problems == ["move above place 1 (L20): keep-out zone 2 is in the way and no travel height clears it"]





def test_starting_inside_a_zone_tells_the_user_to_jog_out_first():
    job = planner.Job()
    job.set(["C10"], ["L20"], zones=[keepout(-100, -100, 200, 100, 600)])
    plan = job.plan(READY, 0.0, 100)
    assert not plan.ok and plan.steps == []
    assert plan.problems == ["the arm is inside keep-out zone 1: jog it out of the zone first"]


def test_a_track_position_whose_stroke_would_be_inside_a_zone_is_dropped():
    x, y = planner.cell_centre("C10")
    free, _ = planner.choose_track(x, y, 0.0, np.array([*model.POSES_DEG["work"], 0.0]), 1.0)
    # a zone the arm would be inside at that track position, but not at others
    zone = {"x0": free - 100.0, "x1": free + 100.0, "y0": y + 100.0, "y1": y + 300.0, "top": 500.0}
    spot, why = planner.choose_track(x, y, 0.0, np.array([*model.POSES_DEG["work"], 0.0]), 1.0, keepouts=[(1, zone)])
    assert why is None and spot != free
    assert planner.zone_hit(model.tool_down(x - free, y, planner.GRIP_HEIGHT_MM, 0.0), free, [(1, zone)]) == 1
    assert planner.zone_hit(model.tool_down(x - spot, y, planner.GRIP_HEIGHT_MM, 0.0), spot, [(1, zone)]) is None


def test_arm_points_agree_with_forward_and_fingertip():
    joints = (20.0, -30.0, -70.0, 15.0, 40.0, 10.0)
    points = model.arm_points(joints, 250.0)
    tcp, _, origins, _ = model.forward(joints, 250.0)
    assert points.shape == (5, 3)
    assert points[0] == pytest.approx(origins[1]) and points[0][2] == pytest.approx(model.BASE_TO_SHOULDER) and points[0][0] == pytest.approx(250.0)
    assert points[3] == pytest.approx(tcp) and points[4] == pytest.approx(model.fingertip(joints, 250.0))
    dense = model.sample_path(points, 30.0)
    assert np.max(np.linalg.norm(np.diff(dense, axis=0), axis=1)) <= 30.0 + 1e-9 and dense[-1] == pytest.approx(points[-1])


def test_a_pose_is_inside_a_zone_within_the_margin_and_below_its_top():
    joints, track = model.POSES_DEG["work"], 0.0
    tip = model.fingertip(joints, track)
    near = {"x0": tip[0] + planner.ARM_MARGIN_MM - 1.0, "x1": tip[0] + 400.0, "y0": -50.0, "y1": 50.0, "top": 1000.0}
    far = {**near, "x0": tip[0] + planner.ARM_MARGIN_MM + 5.0}
    low = {**near, "top": tip[2] - planner.ARM_MARGIN_MM - 5.0}
    assert planner.zone_hit(joints, track, [(1, near)]) == 1
    assert planner.zone_hit(joints, track, [(1, far)]) is None and planner.zone_hit(joints, track, [(1, low)]) is None


# ---- the track settings

def test_without_a_track_the_base_is_at_x_zero_for_the_reach_map_and_the_plan():
    job = planner.Job()
    job.track_fitted = False
    reach = job.grid()["reach"]
    assert reach[6] != "t" * planner.COLS and reach[2][9] == "y" and reach[2][19] == "n"
    job.set(["D10"], ["J10"])
    plan = job.plan(READY, 0.0, 100)
    assert plan.ok and [(p["pick_track"], p["place_track"]) for p in plan.pairs] == [(0.0, 0.0)]
    assert all(np.all(step.samples[:, 6] == 0.0) for step in plan.steps if step.kind != "grip")
    job.set(["C10"], ["L20"])
    assert job.plan(READY, 0.0, 100).problems[:1] == ["place 1 (L20): out of reach"]


def test_with_the_track_held_the_plan_never_moves_it_and_reach_is_from_where_it_stands():
    job = planner.Job()
    job.track_moves = False
    job.set(["C14"], ["L16"])
    assert job.plan(READY, 0.0, 100).problems[0].startswith("pick 1 (C14)")       # from the track's end that is out of reach
    plan = job.plan(READY, 1000.0, 100)
    assert plan.ok and [(p["pick_track"], p["place_track"]) for p in plan.pairs] == [(1000.0, 1000.0)]
    assert all(np.all(step.samples[:, 6] == 1000.0) for step in plan.steps if step.kind != "grip")
    assert job.describe(plan, 1000.0)["grid"]["reach"][2][13] == "y" and job.describe(None, 0.0)["grid"]["reach"][2][13] == "n"


def test_the_track_settings_can_only_change_when_nothing_is_moving(robot):
    robot.jog("joint", [1, 0, 0, 0, 0, 0], ttl=0.5)
    with pytest.raises(RobotError, match="a command is active: halt it before changing the track settings"):
        robot.set_track(moves=False)
    robot.halt()
    run(robot, 0.5)
    robot.set_job(["C10"], ["L20"])
    robot.cycle_start()
    with pytest.raises(RobotError, match="a command is active"):
        robot.set_track(fitted=False)
    robot.halt()
    robot.qd[0] = 5.0
    with pytest.raises(RobotError, match="still moving"):
        robot.set_track(fitted=False)
    robot.qd[:] = 0.0
    for bad in ({"fitted": "no"}, {"moves": 1}, {}):
        with pytest.raises(RobotError):
            robot.set_track(**bad)
    robot.set_track(moves=False)
    assert robot.state()["track_moves"] is False and robot.state()["track_fitted"] is True


def test_turning_the_track_off_snaps_it_to_zero_and_track_jogs_do_nothing(robot):
    robot.track = 700.0
    version = robot.job_version
    robot.set_track(fitted=False)
    assert robot.track == 0.0 and robot.state()["track"] == 0.0 and robot.state()["track_fitted"] is False
    assert robot.job_version == version + 1
    run(robot, 1.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[1.0, 0.0]))
    assert robot.track == 0.0 and robot.track_v == 0.0
    run(robot, 1.0, refresh=lambda: robot.jog("joint", [0, 0, 0, 0, 0, 0], aux=[-1.0, 0.0]))
    assert robot.track == 0.0
    robot.halt()
    run(robot, 0.5)
    robot.set_track(fitted=True)
    run(robot, 1.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[1.0, 0.0]))
    assert robot.track > 50.0


def test_a_cycle_without_a_track_runs_with_the_base_at_x_zero(robot):
    robot.set_track(fitted=False)
    robot.set_speed(100)
    robot.set_job(["D10"], ["J10"])
    robot.cycle_start()
    run(robot, 30.0)
    assert robot.state()["command"] is None and robot.track == 0.0
    assert [p["held"] for p in robot.state()["parts"]] == [False]


def test_a_cycle_with_the_track_held_keeps_it_where_it_was(robot):
    robot.track = 1000.0
    robot.set_track(moves=False)
    robot.set_speed(100)
    robot.set_job(["C14"], ["L16"])
    robot.cycle_start()
    seen = set()
    for _ in range(round(40 / DT)):
        robot.step(DT)
        seen.add(round(robot.track, 6))
        if not robot.state()["command"]:
            break
    assert seen == {1000.0} and robot.state()["command"] is None


# ---- jogs and go-tos near a zone

WORK_ZONE = keepout(200, 250, 700, 600, 400)       # off to the left of the work pose, which stands 480 mm ahead


def tick_until_still(robot, seconds, refresh=None):
    """Run, and return the worst (smallest) clearance state: whether any tick had the arm inside the zone."""
    worst = []
    robot.seen = set()
    for _ in range(round(seconds / DT)):
        if refresh:
            refresh()
        robot.step(DT)
        robot.seen.add(robot.state()["notice"])
        worst.append(planner.zone_hit(robot.q, robot.track, robot.job.keepouts()))
    return worst


def at_work(robot):
    robot.q = np.array(model.POSES_DEG["work"], dtype=float)
    robot.set_speed(100)
    robot.set_job([], [], zones=[WORK_ZONE])


def test_a_joint_jog_at_full_speed_towards_a_zone_stops_short_of_it_and_says_why(robot):
    at_work(robot)
    inside = tick_until_still(robot, 3.0, refresh=lambda: robot.jog("joint", [1, 0, 0, 0, 0, 0]))
    assert set(inside) == {None}                                         # never inside, not even by a tick's slip
    assert abs(float(np.max(np.abs(robot.qd)))) < 1e-6 and robot.q[0] > 5.0
    assert robot.state()["notice"] == "joint jog stopped: the arm would enter keep-out zone 1"
    robot.jog("joint", [-1, 0, 0, 0, 0, 0])                                # and it can back away
    run(robot, 0.5)
    assert robot.q[0] < 20.0 and robot.qd[0] < 0


def test_a_world_jog_and_a_track_jog_are_stopped_too(robot):
    at_work(robot)
    inside = tick_until_still(robot, 3.0, refresh=lambda: robot.jog("world", [0, 1, 0, 0, 0, 0]))
    assert set(inside) == {None} and "keep-out zone 1" in robot.state()["notice"]
    robot.set_job([], [], zones=[keepout(900, -200, 1200, 200, 500)])
    robot.q = np.array(model.POSES_DEG["calrdy"], dtype=float)
    robot.track = 0.0
    inside = tick_until_still(robot, 5.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[1.0, 0.0]))
    assert set(inside) == {None} and 0.0 < robot.track < 900.0 and robot.track_v == 0.0
    assert "keep-out zone 1" in robot.state()["notice"]


def test_a_go_to_into_a_zone_is_halted_and_the_notice_stays_up_for_a_moment(robot):
    at_work(robot)
    robot.goto([90.0, *model.POSES_DEG["work"][1:]])
    inside = tick_until_still(robot, 3.0)
    assert set(inside) == {None} and robot.state()["command"] is None and 0.0 < robot.q[0] < 80.0
    assert "go-to stopped: the arm would enter keep-out zone 1" in robot.seen
    run(robot, 3.0)
    assert robot.state()["notice"] == ""


def test_the_notice_names_the_zone_by_its_number(robot):
    robot.set_job([], [], zones=[area("pick", 0, 0, 100, 100), keepout(300, 150, 700, 600, 400)])
    robot.q = np.array(model.POSES_DEG["work"], dtype=float)
    robot.set_speed(100)
    tick_until_still(robot, 3.0, refresh=lambda: robot.jog("joint", [1, 0, 0, 0, 0, 0]))
    assert robot.state()["notice"] == "joint jog stopped: the arm would enter keep-out zone 2"


def test_an_arm_already_inside_a_zone_is_not_stopped_by_it_and_the_notice_says_so(robot):
    robot.q = np.array(model.POSES_DEG["work"], dtype=float)
    robot.set_speed(100)
    tip = model.fingertip(robot.q, 0.0)
    robot.set_job([], [], zones=[keepout(tip[0] - 150, tip[1] - 150, tip[0] + 150, tip[1] + 150, 600)])
    assert planner.zone_hit(robot.q, 0.0, robot.job.keepouts()) == 1
    run(robot, 0.3, refresh=lambda: robot.jog("joint", [1, 0, 0, 0, 0, 0]))
    assert robot.qd[0] > 10.0 and robot.state()["notice"] == "the arm is inside keep-out zone 1; it can be jogged out"
    run(robot, 3.0, refresh=lambda: robot.jog("joint", [1, 0, 0, 0, 0, 0]))
    assert planner.zone_hit(robot.q, 0.0, robot.job.keepouts()) is None and robot.q[0] > 20.0     # out the other side, and free


def test_with_no_keep_outs_the_motion_is_exactly_what_it_was(robot):
    other = SimRobot()
    other.set_arm_power(True)
    other.homed = True
    other.set_job([], [], zones=[area("pick", 0, 0, 100, 100)])          # areas are not keep-outs
    for r in (robot, other):
        r.q = np.array(model.POSES_DEG["work"], dtype=float)
        r.set_speed(100)
        run(r, 3.0, refresh=lambda r=r: r.jog("joint", [1, 0.3, 0, 0, 0, 0]))
    assert np.array_equal(robot.q, other.q) and robot.state()["notice"] == ""


# ---- the server

def with_client(scenario):
    async def go():
        async with TestClient(TestServer(build_app(use_gamepad=False))) as client:
            await scenario(client)
    asyncio.run(go())


async def post(client, op, body=None):
    response = await client.post(f"/api/{op}", json=body or {})
    return response.status, await response.json()


def test_the_job_op_takes_zones_and_get_job_reports_them_with_the_effective_lists():
    async def scenario(client):
        zones = [area("pick", 200, -460, 460, -340), area("place", 1250, 350, 1450, 550), BESIDE_RAIL]
        status, body = await post(client, "job", {"picks": ["D12"], "places": ["L20"], "zones": zones})
        assert status == 200
        job = await (await client.get("/api/job")).json()
        assert body["job"]["zones"] == job["zones"] and [z["kind"] for z in job["zones"]] == ["pick", "place", "keepout"]
        assert job["zones"][0]["cells"] == ["C9", "C10", "C11", "D9", "D10", "D11"] and job["zones"][0]["left_out"] == 0
        assert job["zones"][2]["top"] == 300.0 and job["zones"][2]["cells"] == []
        assert job["picks"] == ["D12"] and job["effective_picks"][:2] == ["D12", "C9"] and len(job["effective_picks"]) == 7
        assert job["effective_places"][0] == "L20" and job["track_fitted"] is True and job["track_moves"] is True
        assert "k" in "".join(job["grid"]["reach"]) and job["plan"]["ok"]
        status, body = await post(client, "job", {"picks": ["D12"], "places": ["L20"]})        # zones omitted: unchanged
        assert status == 200 and len(body["job"]["zones"]) == 3
        status, body = await post(client, "job", {"picks": [], "places": [], "zones": [area("pick", 0, 0, 5, 5)]})
        assert status == 400 and "at least 10 mm" in body["error"]
        status, body = await post(client, "job", {"picks": ["C10"], "places": [], "zones": [area("place", 300, -500, 400, -400)]})
        assert status == 400 and "C10 would be both a pick and a place" in body["error"]
    with_client(scenario)


def test_the_track_op_sets_either_setting_and_refuses_while_busy():
    async def scenario(client):
        status, body = await post(client, "track", {"moves": False})
        assert status == 200 and body["robot"]["track_moves"] is False and body["robot"]["track_fitted"] is True
        assert body["job"]["track_moves"] is False
        status, body = await post(client, "track", {"fitted": False})
        assert status == 200 and body["robot"]["track_fitted"] is False and body["robot"]["track_moves"] is False
        assert (await post(client, "track", {}))[1]["error"] == "track needs 'fitted', 'moves' or 'carousel'"
        assert (await post(client, "track", {"fitted": "off"}))[1]["error"] == "'fitted' must be true or false"
        await post(client, "armpower", {"on": True})
        await post(client, "jog", {"frame": "joint", "vec": [1, 0, 0, 0, 0, 0], "ttl": 1.0})
        status, body = await post(client, "track", {"fitted": True})
        assert status == 400 and body["error"] == "a command is active: halt it before changing the track settings"
        await post(client, "halt")
        ws = await client.ws_connect("/api/ws")
        await ws.send_json({"op": "track", "moves": True})
        seen = False
        for _ in range(40):
            msg = await ws.receive_json(timeout=2)
            seen |= msg.get("robot", {}).get("track_moves") is True
            if seen:
                break
        assert seen
        await ws.close()
        assert "track" in (await (await client.get("/api/model")).json())["commands"]
    with_client(scenario)


# ---- the carousel, and how the server starts

def test_without_a_carousel_its_angle_is_zero_and_jogs_do_nothing(robot):
    robot.set_speed(100)
    run(robot, 1.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[0.0, 1.0]))
    assert robot.carousel > 10.0
    robot.halt()
    run(robot, 2.0)
    robot.set_track(carousel=False)
    assert robot.carousel == 0.0 and robot.carousel_v == 0.0 and robot.state()["carousel_fitted"] is False
    assert robot.state()["track_fitted"] is True                       # the track is a separate setting
    run(robot, 1.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[0.5, 1.0]))
    assert robot.carousel == 0.0 and robot.track > 10.0                # the track still moves
    with pytest.raises(RobotError, match="a command is active"):
        robot.set_track(carousel=True)
    robot.halt()
    run(robot, 1.0)
    robot.set_track(carousel=True)
    run(robot, 1.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[0.0, 1.0]))
    assert robot.carousel > 10.0


def test_the_server_starts_without_track_or_carousel_unless_the_flags_are_given():
    from a465.server import parse_args

    async def fitted(args):
        async with TestClient(TestServer(build_app(False, None, args.track, args.carousel))) as client:
            robot = (await (await client.get("/api/state")).json())["robot"]
            return robot["track_fitted"], robot["carousel_fitted"]

    def started(argv):
        return asyncio.run(fitted(parse_args(argv)))

    assert started([]) == (False, False)
    assert started(["--track"]) == (True, False)
    assert started(["--carousel"]) == (False, True)
    assert started(["--track", "--carousel"]) == (True, True)
    assert SimRobot().state()["track_fitted"] and SimRobot().state()["carousel_fitted"]     # the class itself defaults to fitted


def test_the_carousel_can_be_switched_over_rest():
    async def scenario(client):
        status, body = await post(client, "track", {"carousel": False})
        assert status == 200 and body["robot"]["carousel_fitted"] is False and body["robot"]["track_fitted"] is True
        assert (await post(client, "track", {"carousel": "no"}))[1]["error"] == "'carousel' must be true or false"
    with_client(scenario)
