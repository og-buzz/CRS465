import numpy as np
import pytest

from a465 import model, planner

READY = model.POSES_DEG["ready"]
SPEEDS = np.array([*model.MAX_SPEED_DEG_S, model.TRACK_MAX_SPEED_MM_S])
ACCELS = np.array([*model.ACCEL_DEG_S2, model.TRACK_ACCEL_MM_S2])


@pytest.fixture
def job():
    j = planner.Job()
    j.set(["C10", "D12"], ["L20", "K4"])
    return j


def tip(row):
    return model.fingertip(row[:6], row[6])


# ---- cells

def test_cells_are_named_by_row_letter_and_column_number():
    assert planner.cell_centre("A1") == (-550.0, -650.0)
    assert planner.cell_centre("c10") == (350.0, -450.0)
    assert planner.cell_name(*planner.parse_cell("N34")) == "N34"


@pytest.mark.parametrize("name", ["", "C", "10", "O1", "A35", "A0", "C1.5", None, 7])
def test_names_that_are_not_cells_are_refused(name):
    with pytest.raises(planner.JobError):
        planner.parse_cell(name)


def test_a_job_refuses_reused_cells_and_silly_plane_heights(job):
    with pytest.raises(planner.JobError, match="only once"):
        job.set(["C10"], ["C10"])
    with pytest.raises(planner.JobError, match="plane_z"):
        job.set(["C10"], ["L20"], plane_z=5000)
    with pytest.raises(planner.JobError, match="lists"):
        job.set("C10", [])
    assert job.picks == ["C10", "D12"]                 # a refused change leaves the job alone


def test_reach_map_marks_the_track_and_the_edge_of_reach(job):
    reach = job.grid()["reach"]
    assert len(reach) == planner.ROWS and all(len(row) == planner.COLS for row in reach)
    assert reach[6] == reach[7] == "t" * planner.COLS                  # rows G and H lie over the track
    assert set(reach[0]) == set(reach[13]) == {"n"}                    # rows A and N are too far to the side
    assert reach[2][9] == "y" and reach[2][0] == "n"                   # C10 can be served, C1 is past the end of the track
    assert reach[2] == reach[11]                                       # the two sides of the track mirror each other


# ---- the plan

def test_plan_is_pick_lower_grip_lift_for_each_cell_then_the_work_pose(job):
    plan = job.plan(READY, 0.0, 100)
    assert plan.ok and plan.problems == [] and plan.notes == []
    assert [s.kind for s in plan.steps] == ["move", "line", "grip", "line"] * 4 + ["move"]
    assert [s.cell for s in plan.steps if s.kind == "grip"] == ["C10", "L20", "D12", "K4"]
    assert [s.open for s in plan.steps if s.kind == "grip"] == [False, True, False, True]
    assert plan.steps[-1].samples[-1][:6] == pytest.approx(model.POSES_DEG["work"])
    assert plan.seconds == pytest.approx(sum(s.seconds for s in plan.steps)) and 8 < plan.seconds < 30


def test_every_step_starts_where_the_one_before_ended(job):
    plan = job.plan(READY, 250.0, 100)
    here = np.array([*READY, 250.0])
    for step in plan.steps:
        if step.kind == "grip":
            continue
        assert step.samples[0] == pytest.approx(here, abs=1e-9)
        here = step.samples[-1]


@pytest.mark.parametrize("speed", [10, 55, 100])
def test_no_axis_is_asked_to_go_faster_or_accelerate_harder_than_it_can(job, speed):
    plan = job.plan(READY, 0.0, speed)
    for step in plan.steps:
        if step.kind == "grip":
            continue
        velocity = np.diff(step.samples, axis=0) / planner.SAMPLE_S
        assert np.all(np.abs(velocity) <= SPEEDS * speed / 100 * 1.001), step.label
        assert np.all(np.abs(np.diff(velocity, axis=0)) / planner.SAMPLE_S <= ACCELS * 1.05), step.label
        assert np.all(step.samples[:, 6] >= 0) and np.all(step.samples[:, 6] <= model.TRACK_TRAVEL_MM)
        for row in step.samples:
            assert all(lo <= a <= hi for a, (lo, hi) in zip(row[:6], model.LIMITS_DEG))


def test_strokes_are_straight_vertical_lines_clear_of_every_singular_shape(job):
    plan = job.plan(READY, 0.0, 100)
    strokes = [s for s in plan.steps if s.kind == "line"]
    assert len(strokes) == 8
    for step in strokes:
        x, y = planner.cell_centre(step.cell)
        heights = []
        for row in step.samples:
            point = tip(row)
            assert point[:2] == pytest.approx([x, y], abs=1e-6)                        # dead over the cell centre
            assert model.forward(row[:6])[1][:, 0] == pytest.approx([0, 0, -1], abs=1e-9)   # tool straight down
            assert planner.pose_problem(row[:6]) is None                               # limits, wrist, elbow margins
            heights.append(point[2])
        assert {round(min(heights), 6), round(max(heights), 6)} == {planner.GRIP_HEIGHT_MM, planner.APPROACH_MM}
        assert step.samples[0][6] == step.samples[-1][6]                               # the track stands still


def test_moves_between_cells_stay_above_the_plane(job):
    plan = job.plan(READY, 0.0, 100)
    for step in plan.steps:
        if step.kind == "move":
            assert min(tip(row)[2] for row in step.samples) >= planner.CLEARANCE_MM


def test_a_raised_plane_raises_the_whole_plan(job):
    job.set(job.picks, job.places, plane_z=100)
    plan = job.plan(READY, 0.0, 100)
    assert plan.ok
    lowest = min(tip(row)[2] for step in plan.steps if step.kind == "line" for row in step.samples)
    assert lowest == pytest.approx(100 + planner.GRIP_HEIGHT_MM, abs=1e-6)


def test_the_track_goes_where_the_arm_gets_to_soonest(job):
    x, y = planner.cell_centre("C10")
    already_there = planner.above(x, y, 0.0, 300.0)
    assert planner.choose_track(x, y, 0.0, already_there, 1.0) == (300.0, None)       # no reason to move at all
    far = np.array([*model.POSES_DEG["work"], 2000.0])
    spot, why = planner.choose_track(x, y, 0.0, far, 1.0)
    assert why is None and 300.0 < spot < 2000.0                                       # slide part of the way and reach back for the rest


def test_cells_that_cannot_be_served_stop_the_plan_and_say_why():
    job = planner.Job()
    job.set(["A1", "C10"], ["G5", "L20"])
    plan = job.plan(READY, 0.0, 100)
    assert not plan.ok and plan.steps == []
    assert plan.problems == ["pick 1 (A1): out of reach", "place 1 (G5): over the track"]


def test_a_cell_without_a_partner_is_noted_and_left_out():
    job = planner.Job()
    job.set(["C10", "D12"], ["L20"])
    plan = job.plan(READY, 0.0, 100)
    assert plan.ok and plan.notes == ["pick 2 (D12) has no place cell yet"]
    assert [s.cell for s in plan.steps if s.kind == "grip"] == ["C10", "L20"]
    assert planner.Job().plan(READY, 0.0, 100).ok is False                             # nothing chosen: nothing to run


def test_a_closed_gripper_is_opened_first(job):
    plan = job.plan(READY, 0.0, 100, gripper_open=False)
    assert plan.steps[0].kind == "grip" and plan.steps[0].open is True


def test_summary_is_plain_data_with_a_path_to_draw(job):
    summary = job.describe(job.plan(READY, 0.0, 100))
    plan = summary["plan"]
    assert summary["picks"] == ["C10", "D12"] and plan["ok"] and len(plan["steps"]) == 17
    assert plan["pairs"][0]["pick"] == "C10" and 0 <= plan["pairs"][0]["pick_track"] <= model.TRACK_TRAVEL_MM
    assert 50 < len(plan["path"]) < 400 and all(len(point) == 3 for point in plan["path"])


# ---- starting from low over the plane

def test_from_the_bottom_of_a_stroke_the_plan_lifts_straight_up_first(job):
    x, y = planner.cell_centre("C10")
    bottom = model.tool_down(x - 90.0, y, planner.GRIP_HEIGHT_MM, planner.JAW_DEG)     # where a stopped cycle leaves the arm
    plan = job.plan(bottom, 90.0, 100, gripper_open=False)
    assert plan.ok and plan.problems == []
    assert [s.label for s in plan.steps[:3]] == ["open the gripper", "lift clear of the plane", "move above pick 1"]
    lift = plan.steps[1]
    assert lift.kind == "line" and lift.samples[0][:6] == pytest.approx(bottom)
    for row in lift.samples:
        assert tip(row)[:2] == pytest.approx([x, y], abs=1e-6)
    assert tip(lift.samples[-1])[2] == pytest.approx(planner.APPROACH_MM)


def test_a_lift_keeps_the_roll_the_tool_has(job):
    x, y = planner.cell_centre("C10")
    turned = list(model.tool_down(x, y, 15.0, planner.JAW_DEG))
    turned[5] += 25.0
    lift = job.plan(turned, 0.0, 100).steps[0]
    assert lift.label == "lift clear of the plane" and set(lift.samples[:, 5]) == {turned[5]}


def test_from_a_low_pose_that_is_not_tool_down_the_first_move_may_not_go_lower(job):
    x, y = planner.cell_centre("C10")
    tilted = list(model.tool_down(x, y, 25.0, planner.JAW_DEG))
    tilted[3] = 30.0                                   # wrist rotated: no longer hanging straight down
    start = model.fingertip(tilted)[2]
    assert start < planner.CLEARANCE_MM
    plan = job.plan(tilted, 0.0, 100)
    assert plan.ok and plan.steps[0].label == "move above pick 1"
    assert min(tip(row)[2] for row in plan.steps[0].samples) >= start - 0.5
