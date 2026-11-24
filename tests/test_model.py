import numpy as np
import pytest

from a465 import model


def tcp(q):
    return model.forward(q)[0]


def test_ready_position_matches_manual_dimensions():
    p, rot, _, _ = model.forward(model.POSES_DEG["ready"])
    assert p == pytest.approx([330 + 76, 0, 330 + 305], abs=1e-9)
    assert model.yaw_pitch_roll(rot) == pytest.approx((0, 0, 0), abs=1e-9)


def test_calrdy_is_straight_up():
    assert tcp(model.POSES_DEG["calrdy"]) == pytest.approx([0, 0, 330 + 305 + 330 + 76], abs=1e-9)


def test_horizontal_reach_is_711mm_from_shoulder_axis():
    p = tcp((0, -90, 0, 0, 0, 0))
    assert p == pytest.approx([711, 0, 330], abs=1e-9)  # datasheet: 711 mm reach


def test_positive_directions_match_table_20():
    ready = np.array(model.POSES_DEG["ready"])
    def moved(axis, start=ready):
        q = np.array(start, dtype=float); q[axis] += 5
        return tcp(q) - tcp(start)
    assert moved(0)[1] > 0                              # waist: right-hand rule about Z up
    assert moved(1, start=(0, -45, -45, 0, 0, 0))[2] > 0  # shoulder: raises arm
    assert moved(2)[2] > 0                              # elbow: raises outer link
    assert moved(4)[2] > 0                              # wrist pitch: pitches up
    for axis in (3, 5):                                 # rotations about the tool axis leave the TCP in place
        assert np.linalg.norm(moved(axis)) < 1e-9


def test_jacobian_matches_finite_differences():
    q = np.array([20.0, -30.0, -60.0, 40.0, 25.0, -10.0])
    jac = model.jacobian(q)
    eps = 1e-5
    for i in range(6):
        dq = np.zeros(6); dq[i] = eps
        numeric = (tcp(q + dq) - tcp(q - dq)) / (2 * np.radians(eps))
        assert jac[:3, i] == pytest.approx(numeric, abs=1e-3)


# ---- tool pointing down, and the shapes where straight lines fail

@pytest.mark.parametrize("x, y, z, jaw", [(450, 0, 10, 0), (300, -400, 130, 0), (-350, 350, 60, 90), (0, 500, 250, 30)])
def test_tool_down_puts_the_fingertips_on_the_point_with_the_tool_vertical(x, y, z, jaw):
    q = model.tool_down(x, y, z, jaw)
    rot = model.forward(q)[1]
    assert model.fingertip(q) == pytest.approx([x, y, z], abs=1e-6)
    assert rot[:, 0] == pytest.approx([0, 0, -1], abs=1e-9)            # tool X, out of the flange, points down
    jaws = np.degrees(np.arctan2(rot[1, 1], rot[0, 1]))                # the fingers open along tool Y
    assert (jaws - jaw + 90) % 180 - 90 == pytest.approx(0, abs=1e-6)
    assert q[2] < 0 and q[3] == 0 and -90 <= q[5] <= 90                # elbow up, wrist rotate unused


def test_tool_down_says_when_a_point_is_too_far():
    assert model.tool_down(900, 0, 0) is None
    assert model.tool_down(450, 0, 0) is not None


def test_fingertip_rides_the_track():
    q = model.POSES_DEG["work"]
    assert model.fingertip(q, 700.0) == pytest.approx(model.fingertip(q) + [700, 0, 0])


def test_ready_and_straight_up_are_singular_and_the_work_pose_is_not():
    assert model.nearest_singularity(model.POSES_DEG["ready"]) == (model.SINGULARITIES["wrist"], 0.0)
    assert model.conditioning(model.POSES_DEG["calrdy"]) == {"wrist": 0.0, "elbow": 0.0, "shoulder": 0.0}
    assert min(model.conditioning(model.POSES_DEG["work"]).values()) > 0.8
    for angle, (lo, hi) in zip(model.POSES_DEG["work"], model.LIMITS_DEG):
        assert lo + 25 < angle < hi - 25
