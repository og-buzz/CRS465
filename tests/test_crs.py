"""a465/crs.py against a stand-in for the controller's console (tests/fake_c500.py). No arm is involved."""
import threading
import time

import pytest

from a465 import crs
from a465.sim import RobotError
from fake_c500 import FakeC500

J1_PLUS, J1_MINUS, J3_PLUS, STILL = [1, 0, 0, 0, 0, 0], [-1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0] * 6
MOVE_J1 = f"joint 1,{18.0 * crs.CHUNK_S:.2f}"        # one jog move at 10%: 18 degrees a second for CHUNK_S
MOVE_J3 = f"joint 3,{18.0 * crs.CHUNK_S:.2f}"


@pytest.fixture
def rig(monkeypatch, tmp_path):
    """Makes (fake controller, RealArm) pairs and closes them afterwards."""
    made = []

    def make(stream=False, zero_at=None, **settings):
        fake = FakeC500()
        for key, value in settings.items():
            setattr(fake, key, value)
        monkeypatch.setattr(crs.serial, "Serial", lambda *a, **k: fake)
        said = []
        arm = crs.RealArm(log_dir=str(tmp_path), say=said.append, stream=stream, zero_at=zero_at)
        arm.said = said
        made.append(arm)
        return fake, arm

    yield make
    for arm in made:
        arm.close()


def hold(arm, vec, seconds, frame="joint", aux=(0.0, 0.0)):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        arm.jog(frame, vec, ttl=0.25, source="test", aux=aux)
        time.sleep(0.02)


def rest(arm, seconds=3.0):
    """Wait until the arm module has finished tidying up after a stop."""
    end = time.monotonic() + seconds
    while time.monotonic() < end and (arm.state()["moving"] or arm.state()["command"] or arm.state()["settling"]):
        time.sleep(0.02)
    time.sleep(0.1)


def moves_after_last_stop(fake):
    last = max((i for i, line in enumerate(fake.sent) if line == "^Z"), default=-1)
    return [line for line in fake.sent[last + 1:] if line.startswith("joint")]


@pytest.mark.parametrize("stream", [False])     # the always-on status display is not used with jogging (see STREAM_OFF)
def test_holding_a_jog_moves_the_joint_and_letting_go_stops_it(rig, stream):
    fake, arm = rig(stream=stream)
    assert fake.status_on == stream
    hold(arm, J1_PLUS, 2.0)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    moved = fake.joints[0]
    assert 25.0 < moved < 45.0                       # 18 deg/s at 10% for about two seconds
    assert abs(arm.joints[0] - moved) < 0.01         # and it read the position back after stopping
    time.sleep(0.6)
    assert fake.joints[0] == moved and not fake.queue
    assert not moves_after_last_stop(fake)
    assert fake.sent.count(MOVE_J1) >= 1


def test_a_jog_that_stops_being_refreshed_is_stopped(rig):
    fake, arm = rig()
    hold(arm, J1_MINUS, 0.8)                         # then nothing: as if the pad or this program's caller died
    time.sleep(0.6)
    assert "^Z" in fake.sent and not fake.queue
    assert not moves_after_last_stop(fake)
    rest(arm)
    assert arm.state()["command"] is None


@pytest.mark.parametrize("stream", [False])     # the always-on status display is not used with jogging (see STREAM_OFF)
@pytest.mark.parametrize("tap", [0.03, 0.1, 0.2, 0.35])
def test_no_move_is_left_running_after_a_short_tap_on_a_slow_controller(rig, tap, stream):
    fake, arm = rig(stream=stream, think=0.3)
    hold(arm, J1_PLUS, tap)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    time.sleep(1.5)
    assert not fake.queue                            # nothing still moving or queued
    assert not moves_after_last_stop(fake)           # any move that was sent was followed by a stop
    assert abs(fake.joints[0]) < 18.0 * (tap + 0.9)


@pytest.mark.parametrize("tap", [0.03, 0.15])
def test_a_move_the_controller_acts_on_after_the_stop_is_stopped_too(rig, tap):
    """The shell is slow to act on a line, and here it is no longer refusing moves by the time it does."""
    fake, arm = rig(think=0.3, reset_s=0.05)
    hold(arm, J1_PLUS, tap)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    time.sleep(1.2)
    assert not fake.queue
    assert not moves_after_last_stop(fake)
    assert abs(fake.joints[0]) < 18.0 * 0.9          # it may start, but is stopped well short of the move typed


def test_halt_always_reaches_the_controller(rig):
    fake, arm = rig()
    arm.halt()
    time.sleep(0.2)
    assert fake.sent.count("^Z") == 1
    hold(arm, J1_PLUS, 0.6)
    arm.halt()
    time.sleep(0.3)
    assert not fake.queue and not moves_after_last_stop(fake)


def test_changing_joint_stops_the_first_before_the_second_starts(rig):
    fake, arm = rig()
    hold(arm, J1_PLUS, 0.6)
    hold(arm, J3_PLUS, 2.2)                          # the change-over (stop, settle, read position) takes about a second
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    first, second = fake.sent.index(MOVE_J1), fake.sent.index(MOVE_J3)
    assert "^Z" in fake.sent[first:second]
    assert fake.joints[2] > 5.0 and not moves_after_last_stop(fake)


def test_a_joint_the_stick_only_passes_through_is_not_started(rig):
    """Pushing a stick out from the centre points at other joints for a few ticks on the way."""
    fake, arm = rig()
    hold(arm, [0, 0.5, 0, 0, 0, 0], 0.06)            # two or three ticks of J2 while the stick travels
    hold(arm, [1.0, 0.1, 0, 0, 0, 0], 1.0)           # then it settles on J1
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert not any(line.startswith("joint 2") for line in fake.sent)
    assert fake.joints[0] > 5.0 and fake.joints[1] == 0.0
    assert fake.sent.index("^Z") > fake.sent.index(MOVE_J1)      # no stop from a false start


def test_only_joint_mode_jog_stop_and_speed_are_offered(rig):
    fake, arm = rig()
    with pytest.raises(RobotError, match="joint mode only"):
        arm.jog("world", [1, 0, 0, 0, 0, 0], ttl=0.25, source="test")
    for refused in (lambda: arm.home("test"), lambda: arm.goto("calrdy", "test"), lambda: arm.cycle_start("test"),
                    lambda: arm.set_gripper(True), lambda: arm.set_arm_power(False)):
        with pytest.raises(RobotError):
            refused()
    with pytest.raises(RobotError, match="25%"):
        arm.set_speed(60)
    assert arm.speed == crs.MAX_SPEED_PCT
    time.sleep(1.0)
    assert f"speed {crs.MAX_SPEED_PCT}" in fake.sent and not any(line.startswith("joint") for line in fake.sent)


STREAM_OFF = ("the always-on status display hung the real controller's robot server after a stop on 2026-11-23 and "
              "starts unreliably against the stand-in; it is not used for jogging and needs rework before it is")


@pytest.mark.skip(reason=STREAM_OFF)
def test_a_home_switch_change_is_reported_with_the_angle(rig):
    fake, arm = rig(stream=True, switch_at={0: 20.0})
    hold(arm, J1_PLUS, 2.2)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    hits = [event for event in arm.events if event[1] == 1 and (event[2] ^ event[3]) & 1]
    assert len(hits) == 1
    assert abs(hits[0][4] - 20.0) < 8.0              # timing-based estimate while moving; not exact
    assert arm.state()["home_switches"][0] == 0


def test_it_waits_for_pause_continue_when_the_controller_asks(rig, monkeypatch, tmp_path):
    fake = FakeC500()
    fake.needs_button = True
    monkeypatch.setattr(crs.serial, "Serial", lambda *a, **k: fake)
    said = []
    threading.Timer(1.0, fake.press).start()
    started = time.monotonic()
    arm = crs.RealArm(log_dir=str(tmp_path), say=said.append)
    try:
        assert arm.state()["home_switches"] is None      # not watched unless asked for
        assert time.monotonic() - started > 0.9
        assert any("PAUSE/CONTINUE" in line for line in said)
        assert fake.online
    finally:
        arm.close()


def test_close_leaves_the_controller_at_its_system_prompt(rig):
    fake, arm = rig()
    hold(arm, J1_PLUS, 0.5)
    arm.close()
    assert fake.shell == "$ " and not fake.queue
    last_stop = max(i for i, line in enumerate(fake.sent) if line == "^Z")
    assert "w2" in fake.sent[last_stop:fake.sent.index("exit")]      # it waited for the controller to answer normally before leaving


# ---- true angles and soft limits (zero_at given) ------------------------------------------------------

J2_PLUS, J2_MINUS = [0, 1, 0, 0, 0, 0], [0, -1, 0, 0, 0, 0]
EDGE = 90.0 - crs.LIMIT_MARGIN_DEG                   # where a shoulder jog must end


def test_without_a_zero_pose_there_are_no_limits_and_no_true_angles(rig):
    fake, arm = rig()
    assert arm.true_joints() is None and arm.state()["true_joints"] is None


def test_a_jog_held_into_a_limit_ends_inside_the_range_by_itself(rig):
    fake, arm = rig(zero_at=[0.0] * 6)
    arm.speed = 25                                   # 45 degrees a second on the shoulder
    time.sleep(0.6)
    hold(arm, J2_PLUS, 4.0)                          # far longer than the 87 degrees takes
    assert fake.joints[1] <= EDGE + 0.05             # still holding: it has stopped at the edge
    assert fake.joints[1] > EDGE - 1.0 and not fake.queue
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert abs(arm.true_joints()[1] - fake.joints[1]) < 0.01
    hold(arm, J2_PLUS, 0.6)                          # pushing on into the limit does nothing but say so
    assert fake.joints[1] <= EDGE + 0.05
    assert any("J2 (shoulder) is at its limit" in line for line in arm.said)
    hold(arm, J2_MINUS, 1.0)                         # and it can be jogged back out
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert fake.joints[1] < EDGE - 10.0


def test_quick_repeated_jogs_cannot_creep_past_a_limit(rig):
    """Each jog must start from where the last one really stopped, not from an old reading."""
    fake, arm = rig(zero_at=[0.0] * 6)
    arm.speed = 25
    time.sleep(0.6)
    for _ in range(9):
        hold(arm, J2_PLUS, 0.9)
        arm.jog("joint", STILL, ttl=0.25, source="test")
        time.sleep(0.15)                             # straight back on the stick, before the position has been read
        assert fake.joints[1] <= EDGE + 0.05
    rest(arm)
    assert fake.joints[1] <= EDGE + 0.05 and fake.joints[1] > 60.0


def test_true_angles_are_measured_from_the_zero_pose_given(rig):
    fake, arm = rig(zero_at=[0.0, 10.0, 0.0, 0.0, 0.0, 0.0])      # the controller read 10 where the shoulder is truly at 0
    assert arm.true_joints()[1] == -10.0
    arm.speed = 25
    time.sleep(0.6)
    hold(arm, J2_PLUS, 4.0)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert abs(fake.joints[1] - (EDGE + 10.0)) < 0.6             # the limit sits 10 further on in the controller's numbers
    assert abs(arm.true_joints()[1] - EDGE) < 0.6


def test_a_joint_already_past_its_range_can_only_be_jogged_back(rig):
    fake, arm = rig(zero_at=[0.0] * 6, joints=[0.0, 0.0, 0.0, 0.0, 0.0, -183.0])    # tool roll resting on its hard stop
    hold(arm, [0, 0, 0, 0, 0, -1], 0.6)
    assert fake.joints[5] == -183.0 and not any(line.startswith("joint 6") for line in fake.sent)
    assert any("J6 (tool roll) is at its limit" in line for line in arm.said)
    hold(arm, [0, 0, 0, 0, 0, 1], 1.0)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert fake.joints[5] > -175.0


# ---- fine steps (L2 pulled at least half way) ---------------------------------------------------------

FINE = (-1.0, 0.0)
STEP_J1 = f"joint 1,{crs.FINE_STEP_DEG:.2f}"


def step_lines(fake):
    return [line for line in fake.sent if line.startswith("joint")]


def test_a_held_fine_jog_sends_one_small_step_at_a_time(rig):
    fake, arm = rig()
    hold(arm, J1_PLUS, 2.0, aux=FINE)
    assert arm.state()["mode"] == "fine"
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    lines = step_lines(fake)
    assert lines and set(lines) == {STEP_J1}
    assert 5 <= len(lines) <= 9                      # one every FINE_GAP_S for about two seconds
    assert abs(fake.joints[0] - 0.5 * len(lines)) < 0.01      # every step sent was made, and nothing else
    assert abs(arm.joints[0] - fake.joints[0]) < 0.01
    assert any(line.startswith("  stopped at") for line in arm.said)
    assert arm.state()["mode"] is None


def test_releasing_a_fine_jog_sends_no_stop(rig):
    fake, arm = rig()
    hold(arm, J1_MINUS, 1.0, aux=FINE)
    arm.jog("joint", STILL, ttl=0.25, source="test", aux=FINE)
    rest(arm)
    assert "^Z" not in fake.sent and not fake.queue
    assert fake.joints[0] < -1.0
    assert abs(fake.joints[0] / 0.5 - round(fake.joints[0] / 0.5)) < 0.01
    sent_at_release = len(fake.sent)
    time.sleep(0.5)
    assert len(fake.sent) == sent_at_release


def test_changing_joint_in_fine_mode_needs_no_stop(rig):
    fake, arm = rig()
    hold(arm, J1_PLUS, 0.8, aux=FINE)
    hold(arm, J3_PLUS, 0.8, aux=FINE)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert "^Z" not in fake.sent
    assert fake.joints[0] > 0.9 and fake.joints[2] > 0.9


def test_fine_steps_never_pile_up_on_a_slow_controller(rig):
    fake, arm = rig(think=0.3)
    hold(arm, J1_PLUS, 2.5, aux=FINE)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert len(step_lines(fake)) >= 3
    assert fake.most_pending == 1
    assert abs(fake.joints[0] - 0.5 * len(step_lines(fake))) < 0.01


def test_fine_steps_stop_at_the_soft_limit(rig):
    fake, arm = rig(zero_at=[0.0] * 6, joints=[0.0, EDGE - 1.2, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.6)
    hold(arm, J2_PLUS, 2.5, aux=FINE)
    assert step_lines(fake) == ["joint 2,0.50", "joint 2,0.50", "joint 2,0.20"]
    assert abs(fake.joints[1] - EDGE) < 0.01
    assert any("J2 (shoulder) is at its limit" in line for line in arm.said)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert "^Z" not in fake.sent


def test_fine_steps_on_a_run_account_for_every_step_before_the_limit(rig):
    """The limit arithmetic must count steps sent but not yet read back: two runs in a row, no position read between."""
    fake, arm = rig(zero_at=[0.0] * 6, joints=[0.0, EDGE - 2.2, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.6)
    hold(arm, J2_PLUS, 0.9, aux=FINE)
    hold(arm, J2_PLUS, 2.0, aux=FINE)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert fake.joints[1] <= EDGE + 0.01 and fake.joints[1] > EDGE - 0.3


def test_continuous_then_fine_on_the_same_joint(rig):
    fake, arm = rig(zero_at=[0.0] * 6)
    time.sleep(0.6)
    hold(arm, J1_PLUS, 1.0)
    hold(arm, J1_PLUS, 2.5, aux=FINE)                # L2 pulled mid-jog: the move is stopped, then it steps
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert fake.sent.index("^Z") < fake.sent.index(STEP_J1) and not fake.queue
    after = fake.sent[max(i for i, line in enumerate(fake.sent) if line == "^Z"):]
    assert after.count(STEP_J1) >= 2
    assert abs(arm.true_joints()[0] - fake.joints[0]) < 0.01


def test_fine_then_continuous_on_the_same_joint(rig):
    fake, arm = rig(zero_at=[0.0] * 6)
    time.sleep(0.6)
    hold(arm, J1_PLUS, 1.0, aux=FINE)
    stepped = fake.joints[0]
    assert stepped > 0.9 and "^Z" not in fake.sent
    hold(arm, J1_PLUS, 1.5)
    assert fake.sent.index(MOVE_J1) > max(i for i, line in enumerate(fake.sent) if line == STEP_J1)
    arm.jog("joint", STILL, ttl=0.25, source="test")
    rest(arm)
    assert fake.joints[0] > stepped + 10.0
    assert not fake.queue and not moves_after_last_stop(fake)
    assert abs(arm.true_joints()[0] - fake.joints[0]) < 0.01


def test_halt_during_fine_steps_writes_a_stop(rig):
    fake, arm = rig()
    hold(arm, J1_PLUS, 0.6, aux=FINE)
    assert "^Z" not in fake.sent
    arm.halt()
    time.sleep(0.3)
    assert fake.sent.count("^Z") == 1
    count = len(step_lines(fake))
    time.sleep(0.6)
    assert len(step_lines(fake)) == count


def test_a_fine_jog_that_stops_being_refreshed_sends_nothing_more(rig):
    fake, arm = rig()
    hold(arm, J1_PLUS, 0.8, aux=FINE)                # then nothing
    time.sleep(0.5)
    count = len(fake.sent)
    time.sleep(1.0)
    assert len(fake.sent) == count and "^Z" not in fake.sent
    rest(arm)
    assert arm.state()["command"] is None


def test_a_long_hold_is_followed_by_another_move_and_a_refused_speed_change_is_offered_again(rig, monkeypatch):
    monkeypatch.setattr(crs, "CHUNK_S", 1.2)         # short moves so the join is reached quickly
    monkeypatch.setattr(crs, "TOPUP_S", 0.5)
    fake, arm = rig()
    line = f"joint 1,{18.0 * 1.2:.2f}"
    hold(arm, J1_PLUS, 3.2)
    assert fake.sent.count(line) >= 2 and not any("did not accept" in said for said in arm.said)
    assert 18.0 * 2.2 < fake.joints[0] < 18.0 * 3.4  # it kept going across the join
    arm.jog("joint", STILL, ttl=0.25, source="test")
    arm.speed = 20                                   # asked for while the controller is still resetting after that stop
    rest(arm)
    time.sleep(1.0)
    assert fake.speed == 20 and fake.sent.count("speed 20") >= 1
