import numpy as np
import pytest

from a465 import model, planner
from a465.sim import RobotError, SimRobot

DT = 0.02


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


def test_starts_at_ready_at_ten_percent_with_power_off():
    r = SimRobot()
    assert r.state()["joints"] == list(model.POSES_DEG["ready"])
    assert r.speed == 10 and r.arm_power is False and r.homed is False


def test_motion_is_refused_without_arm_power():
    r = SimRobot()
    with pytest.raises(RobotError, match="arm power is off"):
        r.jog("joint", [1, 0, 0, 0, 0, 0])
    with pytest.raises(RobotError, match="arm power is off"):
        r.goto("calrdy")


def test_jog_stops_by_itself_when_not_refreshed(robot):
    robot.jog("joint", [1, 0, 0, 0, 0, 0], ttl=0.2)
    run(robot, 0.2)
    assert robot.state()["moving"]
    run(robot, 1.0)                       # nobody refreshes
    stopped_at = robot.q[0]
    assert not robot.state()["moving"] and robot.state()["command"] is None
    run(robot, 0.5)
    assert robot.q[0] == stopped_at and 0 < stopped_at < 10


def test_refreshed_joint_jog_runs_at_speed_percent_of_table_speed(robot):
    robot.set_speed(50)
    run(robot, 2.0, refresh=lambda: robot.jog("joint", [0, 0, 0, 1, 0, 0]))
    assert robot.qd[3] == pytest.approx(171 * 0.5)


def test_joint_stops_at_its_limit(robot):
    robot.set_speed(100)
    run(robot, 3.0, refresh=lambda: robot.jog("joint", [0, 1, 0, 0, 0, 0]))
    assert robot.q[1] == 90 and robot.at_limit[1] and robot.qd[1] == 0


def test_world_jog_moves_the_tool_point_straight_along_x(robot):
    start = model.forward(robot.q)[0]
    run(robot, 1.0, refresh=lambda: robot.jog("world", [1, 0, 0, 0, 0, 0]))
    moved = model.forward(robot.q)[0] - start
    assert moved[0] > 50                                   # ~102 mm/s at 10 %
    assert abs(moved[1]) < 1 and abs(moved[2]) < 1
    assert model.yaw_pitch_roll(model.forward(robot.q)[1]) == pytest.approx((0, 0, 0), abs=0.5)


def test_tool_jog_follows_the_tool_axis(robot):
    run(robot, 2.0, refresh=lambda: robot.jog("joint", [1, 0, 0, 0, 0, 0]))   # turn the waist
    start, rot = model.forward(robot.q)[:2]
    run(robot, 0.6, refresh=lambda: robot.jog("tool", [1, 0, 0, 0, 0, 0]))
    moved = model.forward(robot.q)[0] - start
    along = moved @ rot[:, 0]
    assert along > 20 and np.linalg.norm(moved - along * rot[:, 0]) < 1


def test_goto_calrdy_arrives_and_finishes(robot):
    robot.set_speed(100)
    robot.goto("calrdy")
    run(robot, 4.0)
    assert robot.q == pytest.approx(model.POSES_DEG["calrdy"], abs=1e-6)
    assert robot.state()["command"] is None and not robot.state()["moving"]


def test_halt_and_power_off_cancel_motion(robot):
    robot.goto("calrdy")
    run(robot, 0.3)
    robot.halt()
    run(robot, 1.0)
    assert not robot.state()["moving"] and robot.q[2] < 0
    robot.goto("calrdy")
    run(robot, 0.3)
    robot.set_arm_power(False)
    assert robot.state()["command"] is None and not robot.state()["moving"]


@pytest.mark.parametrize("call, message", [
    (lambda r: r.jog("sideways", [0] * 6), "frame must be"),
    (lambda r: r.jog("joint", [1, 2, 3]), "six numbers"),
    (lambda r: r.jog("joint", ["a"] * 6), "six numbers"),
    (lambda r: r.jog("joint", [0] * 6, ttl=0), "ttl"),
    (lambda r: r.goto("nowhere"), "unknown pose"),
    (lambda r: r.goto([0, 120, 0, 0, 0, 0]), "outside"),
    (lambda r: r.set_speed(0), "1 to 100"),
    (lambda r: r.set_speed("fast"), "1 to 100"),
])
def test_bad_commands_are_rejected_with_a_reason(robot, call, message):
    with pytest.raises(RobotError, match=message):
        call(robot)


# ---- homing

def test_before_homing_only_joint_jogging_works():
    r = SimRobot()
    r.set_arm_power(True)
    r.jog("joint", [1, 0, 0, 0, 0, 0])                 # allowed
    for call in (lambda: r.jog("world", [1, 0, 0, 0, 0, 0]), lambda: r.goto("calrdy"), r.cycle_start):
        with pytest.raises(RobotError, match="not homed"):
            call()


def test_home_takes_a_while_and_halting_it_leaves_the_arm_unhomed():
    r = SimRobot()
    with pytest.raises(RobotError, match="arm power is off"):
        r.home()
    r.set_arm_power(True)
    r.home()
    run(r, 1.0)
    assert r.state()["command"] == {"kind": "home", "source": "api"} and not r.homed
    r.halt()
    run(r, 5.0)
    assert not r.homed
    r.home()
    run(r, 5.0)
    assert r.homed and r.state()["command"] is None
    r.set_arm_power(False)                             # homing survives arm power off
    assert r.homed


# ---- track and carousel

def test_track_runs_at_speed_percent_of_its_rated_speed_and_moves_the_tool_point(robot):
    robot.set_speed(50)
    x0 = robot.state()["pose"]["x"]
    run(robot, 1.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[1, 0]))
    assert robot.track_v == pytest.approx(400)         # half of 0.8 m/s
    state = robot.state()
    assert state["track"] > 300 and state["pose"]["x"] - x0 == pytest.approx(state["track"], abs=0.01)
    assert state["joints"] == list(model.POSES_DEG["ready"]) and state["moving"]


def test_track_stops_at_both_ends_of_its_travel(robot):
    robot.set_speed(100)
    run(robot, 4.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[1, 0]))
    assert robot.track == model.TRACK_TRAVEL_MM and robot.state()["track_at_limit"]
    run(robot, 4.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[-1, 0]))
    assert robot.track == 0


def test_carousel_turns_and_wraps(robot):
    robot.set_speed(100)
    run(robot, 5.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[0, 1]))
    assert 0 <= robot.carousel < 360 and robot.carousel_v == pytest.approx(90)
    run(robot, 1.0, refresh=lambda: robot.jog("joint", [0] * 6, aux=[0, -1]))
    assert 0 <= robot.state()["carousel"] < 360


def test_aux_must_be_two_numbers(robot):
    with pytest.raises(RobotError, match="aux must be two numbers"):
        robot.jog("joint", [0] * 6, aux=[1])


# ---- pick-and-place cycle

def finish(robot, limit=60.0):
    seconds = 0.0
    while robot.state()["command"] and seconds < limit:
        robot.step(DT)
        seconds += DT
    return seconds


def test_cycle_start_needs_cells_and_a_plan_that_checks_out(robot):
    with pytest.raises(RobotError, match="choose pick and place cells"):
        robot.cycle_start()
    robot.set_job(["A1"], ["L20"])
    with pytest.raises(RobotError, match=r"cycle not started: pick 1 \(A1\): out of reach"):
        robot.cycle_start()
    assert robot.state()["command"] is None
    with pytest.raises(RobotError, match="outside the grid"):
        robot.set_job(["Z9"], ["L20"])


def test_cycle_carries_each_cube_from_its_pick_cell_to_its_place_cell(robot):
    robot.set_speed(100)
    robot.set_job(["C10", "D12"], ["L20", "K4"])
    assert [(p["x"], p["y"], p["held"]) for p in robot.state()["parts"]] == [(350.0, -450.0, False), (550.0, -350.0, False)]
    expected = robot.plan().seconds
    robot.cycle_start()
    seen, carried = [], False
    seconds = 0.0
    while robot.state()["command"] and seconds < 60:
        robot.step(DT)
        seconds += DT
        command = robot.state()["command"]
        if command and (not seen or seen[-1] != command["step"]):
            seen.append(command["step"])
            assert command["kind"] == "cycle" and command["of"] == 17 and command["label"]
        carried |= any(p["held"] for p in robot.state()["parts"])
    assert seen == list(range(1, 18)) and carried
    assert seconds == pytest.approx(expected, abs=0.5)
    assert [(p["x"], p["y"], p["z"], p["held"]) for p in robot.state()["parts"]] == [(1350.0, 450.0, 0.0, False), (-250.0, 350.0, 0.0, False)]
    assert robot.q == pytest.approx(model.POSES_DEG["work"], abs=1e-6) and robot.gripper_open
    assert not robot.state()["moving"]


def test_starting_again_puts_the_cubes_back_on_the_pick_cells(robot):
    robot.set_speed(100)
    robot.set_job(["C10"], ["L20"])
    robot.cycle_start()
    finish(robot)
    assert robot.state()["parts"][0]["x"] == 1350.0
    robot.cycle_start()
    assert robot.state()["parts"][0]["x"] == 350.0


def test_halting_mid_move_coasts_to_rest_and_keeps_hold_of_the_cube(robot):
    robot.set_speed(100)
    robot.set_job(["C10"], ["L20"])
    robot.cycle_start()
    while not (robot.state()["command"]["label"] == "move above place 1" and abs(robot.track_v) > 400):
        robot.step(DT)
    robot.halt()
    before, steps = robot.q.copy(), 0
    while robot.state()["moving"]:
        robot.step(DT)
        steps += 1
        assert np.all(np.abs(robot.q - before) <= np.array(model.MAX_SPEED_DEG_S) * DT * 1.01)   # no jump
        before = robot.q.copy()
    assert 3 < steps < 60 and robot.state()["command"] is None
    part = robot.state()["parts"][0]
    assert part["held"] and part["z"] > 50
    robot.set_gripper(True)                            # let go: it lands on the plane under the fingers
    tip = model.fingertip(robot.q, robot.track)
    assert robot.state()["parts"][0] == {"x": round(tip[0], 1), "y": round(tip[1], 1), "z": 0.0, "held": False}


def test_a_cycle_stopped_at_the_bottom_of_a_stroke_can_be_started_again(robot):
    robot.set_speed(100)
    robot.set_job(["C10"], ["L20"])
    robot.cycle_start()
    while robot.state()["command"]["label"] != "close the gripper":
        robot.step(DT)
    robot.step(DT)
    robot.halt()                                       # gripper shut around the cube, fingertips 10 mm off the plane
    run(robot, 0.5)
    assert model.fingertip(robot.q, robot.track)[2] == pytest.approx(planner.GRIP_HEIGHT_MM, abs=1e-6)
    robot.cycle_start()
    assert robot.state()["command"]["label"] == "open the gripper"
    finish(robot)
    assert robot.state()["parts"] == [{"x": 1350.0, "y": 450.0, "z": 0.0, "held": False}]


def test_cells_cannot_be_changed_while_the_cycle_runs_and_a_jog_ends_it(robot):
    robot.set_job(["C10"], ["L20"])
    robot.cycle_start()
    run(robot, 0.5)
    with pytest.raises(RobotError, match="halt it before changing cells"):
        robot.set_job([], [])
    robot.jog("joint", [1, 0, 0, 0, 0, 0], ttl=0.1)
    run(robot, 1.0)
    assert robot.state()["command"] is None            # the cycle did not resume


def test_speed_changes_take_effect_during_the_cycle(robot):
    robot.set_job(["C10"], ["L20"])
    robot.set_speed(20)
    slow = robot.plan().seconds
    robot.cycle_start()
    run(robot, 1.0)
    robot.set_speed(100)
    assert 2.0 < 1.0 + finish(robot) < slow * 0.6


def test_closing_the_gripper_by_hand_over_a_cube_picks_it_up(robot):
    robot.set_job(["C10"], ["L20"])
    robot.q = np.array(model.tool_down(350, -450, planner.GRIP_HEIGHT_MM))
    robot.set_gripper(False)
    assert robot.state()["parts"][0]["held"]
    robot.set_gripper(True)
    robot.q = np.array(model.tool_down(350, -350, planner.GRIP_HEIGHT_MM))      # the next cell over: nothing there
    robot.set_gripper(False)
    assert not robot.state()["parts"][0]["held"]


# ---- world and tool jogging: a point moving in space

def jog_for(robot, frame, vec, seconds):
    """Jog and report (how far the point strayed from its line or spot, how far the tool tilted, the last notice)."""
    start, facing = model.forward(robot.q)[:2]
    direction = np.array(vec[:3], float)
    if frame == "tool":
        direction = facing @ direction
    strayed = tilted = 0.0
    notice = ""
    for _ in range(round(seconds / DT)):
        robot.jog(frame, vec, ttl=0.3)
        robot.step(DT)
        notice = robot.state()["notice"] or notice
        point, rot = model.forward(robot.q)[:2]
        moved = point - start
        if direction.any():
            along = direction / np.linalg.norm(direction)
            strayed = max(strayed, float(np.linalg.norm(moved - (moved @ along) * along)))
            tilted = max(tilted, float(np.degrees(np.arccos(np.clip((np.trace(facing.T @ rot) - 1) / 2, -1, 1)))))
        else:
            strayed = max(strayed, float(np.linalg.norm(moved)))
    return strayed, tilted, notice


@pytest.fixture
def at_work(robot):
    robot.q = np.array(model.POSES_DEG["work"])
    robot.set_speed(30)
    return robot


@pytest.mark.parametrize("frame, vec", [("world", [1, 0, 0, 0, 0, 0]), ("world", [0, -1, 0, 0, 0, 0]), ("world", [0, 0, 1, 0, 0, 0]),
                                        ("tool", [1, 0, 0, 0, 0, 0]), ("tool", [0, 0, -1, 0, 0, 0])])
def test_a_linear_jog_is_a_straight_line_with_the_tool_angle_held(at_work, frame, vec):
    start = model.forward(at_work.q)[0]
    strayed, tilted, notice = jog_for(at_work, frame, vec, 0.6)
    assert np.linalg.norm(model.forward(at_work.q)[0] - start) > 80
    assert strayed < 0.5 and tilted < 0.1 and notice == ""


@pytest.mark.parametrize("frame, axis", [("world", 3), ("world", 5), ("tool", 3), ("tool", 4)])
def test_a_rotation_jog_turns_the_tool_about_a_point_that_stays_put(at_work, frame, axis):
    vec = [0.0] * 6
    vec[axis] = 1.0
    facing = model.forward(at_work.q)[1]
    strayed, _, _ = jog_for(at_work, frame, vec, 0.6)
    turned = np.degrees(np.arccos(np.clip((np.trace(facing.T @ model.forward(at_work.q)[1]) - 1) / 2, -1, 1)))
    assert turned > 15 and strayed < 0.5


def test_at_a_joint_limit_the_point_stops_on_its_line_and_the_notice_names_the_joint(at_work):
    strayed, tilted, notice = jog_for(at_work, "world", [-1, 0, 0, 0, 0, 0], 4.0)
    assert notice == "world jog stopped: J3 (elbow) is at its limit"
    assert strayed < 0.5 and tilted < 0.1 and not at_work.state()["moving"]
    assert at_work.q[2] == pytest.approx(-110, abs=0.2) and at_work.state()["at_limit"][2]
    jog_for(at_work, "world", [1, 0, 0, 0, 0, 0], 0.5)                 # backing away from the limit is allowed
    assert at_work.q[2] > -105 and at_work.state()["notice"] == ""


def test_at_full_stretch_the_point_stops_instead_of_sliding(at_work):
    strayed, tilted, notice = jog_for(at_work, "world", [1, 0, 0, 0, 0, 0], 5.0)
    assert notice == "world jog stopped: the arm is at full stretch (J3 near 0)"
    assert strayed < 5 and tilted < 1 and not at_work.state()["moving"]


def test_from_ready_only_the_directions_the_straight_wrist_allows_will_move(robot):
    robot.set_speed(30)
    start = model.forward(robot.q)[0]
    strayed, _, notice = jog_for(robot, "world", [0, 1, 0, 0, 0, 0], 1.0)
    assert notice == "world jog stopped: the wrist is straight (J5 near 0)"
    assert model.forward(robot.q)[0] == pytest.approx(start, abs=1e-6)
    strayed, tilted, _ = jog_for(robot, "world", [1, 0, 0, 0, 0, 0], 0.6)
    assert model.forward(robot.q)[0][0] - start[0] > 80 and strayed < 0.5 and tilted < 0.1


def test_the_notice_clears_when_the_jog_ends(robot):
    jog_for(robot, "world", [0, 1, 0, 0, 0, 0], 0.3)
    assert robot.state()["notice"]
    run(robot, 0.5)                                    # nobody refreshes the jog
    assert robot.state()["notice"] == ""


def test_light_and_tool_are_plain_outputs():
    r = SimRobot()                                     # they work with arm power off
    r.set_light(True)
    r.set_tool(True)
    assert r.state()["light"] and r.state()["tool"]
    r.set_tool(False)
    assert r.state()["light"] and not r.state()["tool"]
