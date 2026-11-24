import asyncio

import evdev
import pytest
from evdev import ecodes

from a465 import gamepad
from a465.gamepad import GamepadJog, aux_vector, camera_vector, jog_vector, shape, trigger_pull
from a465.sim import SimRobot

NEUTRAL = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "l2": 0.0, "r2": 0.0}
STICK_CODES = set(gamepad.STICKS)


@pytest.fixture
def pad():
    robot = SimRobot()
    robot.set_arm_power(True)
    robot.homed = True
    return GamepadJog(robot)


def command(pad):
    return pad.robot.state()["command"]


# ---- pure mapping

def test_stick_curve_has_a_dead_zone_and_reaches_full_scale():
    assert shape(0.05) == 0 and shape(-0.11) == 0
    assert shape(1.0) == 1.0 and shape(-1.0) == -1.0
    assert 0 < shape(0.5) < 0.5                       # gentle near the centre


def test_joint_frame_mapping():
    up_left = dict(NEUTRAL, lx=-1.0, ly=-1.0)         # left stick pushed up and to the left
    assert jog_vector("joint", up_left) == [1, 1, 0, 0, 0, 0]       # waist +, shoulder raises
    assert jog_vector("joint", dict(NEUTRAL, ry=-1.0), {"dpad_up"}) == [0, 0, 1, 0, 1, 0]    # elbow raises, wrist pitches up
    assert jog_vector("joint", NEUTRAL, {"dpad_down", "dpad_right"}) == [0, 0, 0, 0, -1, 1]  # wrist pitches down, tool rolls


def test_world_frame_mapping():
    assert jog_vector("world", dict(NEUTRAL, ly=-1.0)) == [1, 0, 0, 0, 0, 0]    # stick up = +X forward
    assert jog_vector("world", dict(NEUTRAL, lx=-1.0)) == [0, 1, 0, 0, 0, 0]    # stick left = +Y
    assert jog_vector("world", dict(NEUTRAL, ry=-1.0)) == [0, 0, 1, 0, 0, 0]    # right stick up = +Z
    assert jog_vector("world", NEUTRAL, {"dpad_up", "dpad_left"}) == [0, 0, 0, -1, -1, 0]    # roll -, pitch up


def test_r1_hands_both_sticks_to_the_camera():
    sticks = dict(NEUTRAL, lx=-1.0, ly=-1.0, rx=1.0, ry=-1.0)
    assert camera_vector(sticks, set()) == [0, 0, 0]
    assert camera_vector(sticks, {"r1"}) == [1, -1, 1]                    # orbit right, orbit up, left stick up = zoom in
    assert camera_vector(dict(NEUTRAL, ly=1.0), {"r1"}) == [0, 0, -1]     # left stick down = zoom out
    assert jog_vector("joint", sticks, {"r1"}) == [0, 0, 0, 0, 0, 0]      # neither stick moves the arm
    assert jog_vector("world", sticks, {"r1", "dpad_up"}) == [0, 0, 0, 0, -1, 0]    # D-pad up / down still pitches


def test_triggers_drive_the_track_and_the_dpad_drives_the_carousel_with_r1():
    assert trigger_pull(dict(NEUTRAL, r2=1.0)) == 1 and trigger_pull(dict(NEUTRAL, l2=0.5)) == -0.5
    assert trigger_pull(dict(NEUTRAL, r2=0.03)) == 0                       # resting finger
    assert aux_vector(dict(NEUTRAL, r2=1.0), set()) == [1, 0]              # R2: track forward
    assert aux_vector(dict(NEUTRAL, l2=1.0), {"dpad_right"}) == [-1, 0]    # without R1 the D-pad is not the carousel
    assert aux_vector(dict(NEUTRAL, r2=1.0), {"r1", "dpad_right"}) == [0, 1]   # with R1: carousel, and no track
    assert jog_vector("joint", NEUTRAL, {"r1", "dpad_right"}) == [0, 0, 0, 0, 0, 0]   # and no tool roll


# ---- enable switch

def test_nothing_moves_unless_l1_is_held(pad):
    pad.update(dict(NEUTRAL, lx=1.0, r2=1.0), set())
    assert command(pad) is None
    pad.update(dict(NEUTRAL, lx=1.0), {"l1"})
    assert command(pad) == {"kind": "jog", "frame": "joint", "source": "gamepad"}


def test_releasing_l1_halts_only_what_the_gamepad_started(pad):
    pad.update(dict(NEUTRAL, lx=1.0), {"l1"})
    pad.update(dict(NEUTRAL, lx=1.0), set())
    assert command(pad) is None
    pad.update(NEUTRAL, {"l1"})                        # enabled, sticks centred: no jog issued
    pad.robot.goto("calrdy", source="web")
    pad.update(NEUTRAL, set())                         # releasing L1 leaves the web's move alone
    assert command(pad) == {"kind": "goto", "source": "web"}


def test_arm_power_off_is_reported_not_raised(pad):
    pad.robot.set_arm_power(False)
    pad.update(dict(NEUTRAL, lx=1.0), {"l1"})
    assert pad.message == "arm power is off"
    assert command(pad) is None
    for _ in range(120):                               # the note clears a couple of seconds after the cause stops
        pad.update(NEUTRAL, set())
    assert pad.message == ""


@pytest.mark.parametrize("pushed", [dict(NEUTRAL, ry=-1.0), dict(NEUTRAL, ly=-1.0)])   # orbiting, zooming
def test_releasing_r1_with_a_stick_pushed_does_not_move_the_arm(pad, pushed):
    pad.update(pushed, {"l1", "r1"})                   # steering the view with motion enabled
    assert command(pad) is None
    pad.update(pushed, {"l1"})                         # R1 released, stick still pushed
    assert command(pad) is None
    pad.update(NEUTRAL, {"l1"})                        # stick back to centre
    pad.update(pushed, {"l1"})                         # now it drives the arm again
    assert command(pad) == {"kind": "jog", "frame": "joint", "source": "gamepad"}


def test_a_stick_that_was_not_used_for_the_view_is_released_separately(pad):
    pad.update(dict(NEUTRAL, ry=-1.0), {"l1", "r1"})   # only the right stick is pushed
    pad.update(dict(NEUTRAL, ry=-1.0), {"l1"})         # R1 released; the left stick is seen centred
    pad.update(dict(NEUTRAL, ry=-1.0, lx=1.0), {"l1"}) # so it works straight away, while the right one waits
    pad.robot.step(0.02)
    assert pad.robot.qd[0] < 0 and pad.robot.qd[2] == 0   # waist turns, elbow (right stick) stays put


# ---- buttons

def test_l1_with_circle_opens_and_cross_closes_the_gripper(pad):
    pad.update(NEUTRAL, {"l1", "cross"})
    assert pad.robot.gripper_open is False
    pad.update(NEUTRAL, {"l1"})
    pad.update(NEUTRAL, {"l1", "cross"})               # closing again leaves it closed
    assert pad.robot.gripper_open is False
    pad.update(NEUTRAL, {"l1", "circle"})
    assert pad.robot.gripper_open is True
    assert pad.robot.arm_power is True                 # L1 + Circle alone never touches arm power


def test_gripper_buttons_need_l1_and_square_leaves_the_gripper_alone(pad):
    pad.update(NEUTRAL, {"cross"})
    assert pad.robot.gripper_open is True and "hold L1" in pad.message
    pad.robot.set_gripper(False)
    pad.update(NEUTRAL, set())
    pad.update(NEUTRAL, {"circle"})
    assert pad.robot.gripper_open is False and pad.robot.arm_power is True
    pad.update(NEUTRAL, {"l1", "square"})
    assert pad.robot.gripper_open is False


def test_arm_power_toggles_only_with_l1_r1_and_circle(pad):
    for held in ({"circle"}, {"r1", "circle"}, {"l1", "circle"}):
        pad.update(NEUTRAL, held)
        pad.update(NEUTRAL, set())
        assert pad.robot.arm_power is True, held
    for _ in range(5):                                 # held across several ticks: one toggle
        pad.update(NEUTRAL, {"l1", "r1", "circle"})
    assert pad.robot.arm_power is False
    pad.update(NEUTRAL, {"l1", "r1"})
    pad.update(NEUTRAL, {"l1", "r1", "circle"})
    assert pad.robot.arm_power is True


def test_in_combined_mode_circle_and_cross_leave_the_gripper_alone(pad):
    pad.update(NEUTRAL, {"l1", "r1", "cross"})
    assert pad.robot.gripper_open is True
    pad.robot.set_gripper(False)
    pad.update(NEUTRAL, {"l1", "r1"})
    pad.update(NEUTRAL, {"l1", "r1", "circle"})        # toggles power, does not open the gripper
    assert pad.robot.gripper_open is False and pad.robot.arm_power is False


def test_options_cycles_frames(pad):
    for expected in ("world", "tool", "joint"):
        pad.update(NEUTRAL, {"options"})
        pad.update(NEUTRAL, set())
        assert pad.frame == expected


def tick(pad, axes, buttons, seconds):
    for _ in range(round(seconds / 0.02)):
        pad.update(axes, buttons)
        pad.robot.step(0.02)


def test_triangle_points_straight_up_at_safe_speed_and_carries_on_after_release(pad):
    pad.robot.set_speed(60)
    pad.update(NEUTRAL, {"triangle"})                  # without L1: refused
    assert command(pad) is None and pad.robot.speed == 60 and "hold L1" in pad.message
    pad.update(NEUTRAL, {"l1"})
    pad.update(NEUTRAL, {"l1", "triangle"})
    assert pad.robot.speed == 10 and command(pad) == {"kind": "goto", "source": "gamepad"}
    tick(pad, NEUTRAL, {"l1"}, 0.2)
    tick(pad, NEUTRAL, set(), 1.0)                     # a tap is enough: it keeps going with nothing held
    assert command(pad) == {"kind": "goto", "source": "gamepad"} and pad.robot.q[2] > -90
    tick(pad, NEUTRAL, set(), 12.0)
    assert command(pad) is None and pad.robot.q == pytest.approx([0] * 6, abs=0.05)


def test_square_stops_whatever_is_running_without_l1(pad):
    pad.update(NEUTRAL, {"l1", "triangle"})
    tick(pad, NEUTRAL, set(), 0.5)
    pad.update(NEUTRAL, {"square"})
    assert command(pad) is None
    pad.robot.goto("ready", source="web")              # a move the page started stops too
    pad.update(NEUTRAL, set())
    pad.update(NEUTRAL, {"square"})
    assert command(pad) is None


def test_a_tapped_cycle_start_runs_the_whole_cycle(pad):
    pad.robot.set_speed(100)
    pad.robot.set_job(["C10"], ["L20"])
    pad.update(NEUTRAL, {"l1", "r3"})
    tick(pad, NEUTRAL, {"l1"}, 0.1)
    tick(pad, NEUTRAL, set(), 0.5)                     # both buttons let go
    assert command(pad)["kind"] == "cycle"
    tick(pad, NEUTRAL, {"l1"}, 0.5)                    # holding L1 again with the sticks centred changes nothing
    assert command(pad)["kind"] == "cycle"
    tick(pad, NEUTRAL, set(), 15.0)
    assert command(pad) is None and pad.robot.state()["parts"][0]["x"] == 1350.0


def test_a_stick_nudged_by_its_own_click_does_not_cancel_what_the_click_started(pad):
    pad.robot.set_job(["C10"], ["L20"])
    nudged = dict(NEUTRAL, rx=0.4, ry=0.3)
    pad.update(nudged, {"l1", "r3"})
    tick(pad, nudged, {"l1", "r3"}, 0.2)
    tick(pad, nudged, {"l1"}, 0.2)                     # click released, stick not yet back at centre
    assert command(pad)["kind"] == "cycle"
    tick(pad, NEUTRAL, {"l1"}, 0.1)
    tick(pad, dict(NEUTRAL, rx=1.0), {"l1"}, 0.1)      # a deliberate push after re-centring takes over
    assert command(pad)["kind"] == "jog"


def test_a_controller_that_goes_away_stops_the_cycle_it_started(pad):
    pad.robot.set_job(["C10"], ["L20"])
    pad.update(NEUTRAL, {"l1", "r3"})
    tick(pad, NEUTRAL, set(), 0.3)
    pad.release()
    assert command(pad) is None


def test_stick_clicks_start_home_and_cycle(pad):
    pad.robot.homed = False
    pad.update(NEUTRAL, {"l1", "r3"})                  # cycle start before homing is refused
    assert command(pad) is None and pad.message == "not homed"
    pad.update(NEUTRAL, {"l1"})
    pad.update(NEUTRAL, {"l1", "l3"})
    assert command(pad) == {"kind": "home", "source": "gamepad"}
    tick(pad, NEUTRAL, {"l1"}, 0.1)
    tick(pad, NEUTRAL, set(), 5.0)                     # a tap: homing finishes with nothing held
    assert pad.robot.homed and command(pad) is None
    pad.update(NEUTRAL, {"l1", "r3"})                  # nothing chosen to pick or place yet
    assert command(pad) is None and "choose pick and place cells" in pad.message
    pad.update(NEUTRAL, {"l1"})
    pad.robot.set_job(["C10"], ["L20"])
    pad.update(NEUTRAL, {"l1", "r3"})
    assert command(pad)["kind"] == "cycle" and command(pad)["source"] == "gamepad"
    pad.update(NEUTRAL, set())                         # letting go does not stop it
    assert command(pad)["kind"] == "cycle"


# ---- triggers

def test_l1_and_trigger_move_the_track(pad):
    for _ in range(50):
        pad.update(dict(NEUTRAL, r2=1.0), {"l1"})
        pad.robot.step(0.02)
    assert pad.robot.track > 20 and pad.robot.speed == 10
    forward = pad.robot.track
    for _ in range(100):
        pad.update(dict(NEUTRAL, l2=1.0), {"l1"})
        pad.robot.step(0.02)
    assert pad.robot.track < forward


def test_l1_r1_and_trigger_change_speed_and_leave_the_track_alone(pad):
    for _ in range(50):                                # one second of full R2
        pad.update(dict(NEUTRAL, r2=1.0), {"l1", "r1"})
        pad.robot.step(0.02)
    assert pad.robot.speed == pytest.approx(40, abs=1) and pad.robot.track == 0
    for _ in range(25):                                # half a second of half L2
        pad.update(dict(NEUTRAL, l2=0.5), {"l1", "r1"})
    assert pad.robot.speed == pytest.approx(32, abs=2)
    for _ in range(500):
        pad.update(dict(NEUTRAL, l2=1.0), {"l1", "r1"})
    assert pad.robot.speed == 1                        # stops at 1 %


def test_l1_r1_and_dpad_turn_the_carousel(pad):
    for _ in range(50):
        pad.update(NEUTRAL, {"l1", "r1", "dpad_right"})
        pad.robot.step(0.02)
    assert 0 < pad.robot.carousel < 180 and pad.robot.q[5] == 0     # carousel turned, tool did not roll


# ---- lost signal and reconnect

def test_after_a_lost_signal_l1_must_be_let_go_before_it_enables_again(pad):
    push = dict(NEUTRAL, lx=1.0)
    pad.update(push, {"l1"})
    assert command(pad)["kind"] == "jog"
    pad.release()                                     # what the watchdog and a disconnect both call
    assert command(pad) is None
    pad.update(push, {"l1"})                          # signal back, L1 and stick still held
    assert command(pad) is None and not pad.enabled
    assert "release L1" in pad.message
    pad.update(push, set())
    pad.update(push, {"l1"})
    assert command(pad)["kind"] == "jog" and pad.enabled


def test_buttons_do_nothing_until_l1_is_rearmed(pad):
    pad.release()
    pad.update(NEUTRAL, {"l1", "l3"})
    assert command(pad) is None
    pad.update(NEUTRAL, {"l1", "triangle"})
    assert command(pad) is None


class FakeNode:
    """Stands in for an evdev device: events are pushed into it by the test."""

    def __init__(self, name, keys=()):
        self.name, self.path, self.uniq, self.phys = name, "/fake/" + name, "aa:bb", ""
        self.keys, self.queue, self.closed = set(keys), asyncio.Queue(), False

    def active_keys(self):
        return list(self.keys)

    def absinfo(self, code):
        return evdev.AbsInfo(128 if code in STICK_CODES else 0, 0, 255, 0, 0, 0)

    async def async_read_loop(self):
        while True:
            event = await self.queue.get()
            if event is None:
                raise OSError("gone")
            yield event

    def close(self):
        self.closed = True


def run_pad(monkeypatch, scenario, heartbeat=True):
    robot = SimRobot()
    robot.set_arm_power(True)
    robot.homed = True
    main, pulse = FakeNode("pad"), FakeNode("pad Motion Sensors")
    monkeypatch.setattr(gamepad, "find_controller", lambda only=None: (main, None))
    monkeypatch.setattr(gamepad, "find_heartbeat",
                        lambda dev: (pulse, None) if heartbeat else (None, "this controller has no motion-sensor node"))

    async def go():
        pad = gamepad.Gamepad(robot)
        task = asyncio.ensure_future(pad.run())

        async def beat(seconds):                       # the controller reporting normally
            for _ in range(int(seconds / 0.01)):
                pulse.queue.put_nowait(evdev.InputEvent(0, 0, ecodes.EV_MSC, ecodes.MSC_TIMESTAMP, 1))
                robot.step(0.01)
                await asyncio.sleep(0.01)

        async def silence(seconds):                    # radio link dead: the kernel state stays frozen
            for _ in range(int(seconds / 0.01)):
                robot.step(0.01)
                await asyncio.sleep(0.01)

        def press(code, value):
            main.queue.put_nowait(evdev.InputEvent(0, 0, ecodes.EV_KEY if code in gamepad.BUTTONS else ecodes.EV_ABS, code, value))

        try:
            await scenario(pad, robot, beat, silence, press)
        finally:
            task.cancel()
    asyncio.run(go())


def test_watchdog_stops_the_arm_when_the_controller_goes_silent(monkeypatch):
    async def scenario(pad, robot, beat, silence, press):
        await beat(0.1)
        press(ecodes.BTN_TL, 1)
        press(ecodes.ABS_X, 255)
        await beat(0.3)
        assert robot.state()["command"]["kind"] == "jog" and pad.status()["watchdog"]
        await silence(gamepad.STALE_S + 0.1)           # L1 and the stick are still "held" as far as the kernel knows
        assert robot.state()["command"] is None
        assert pad.status()["signal_lost"] and pad.status()["message"] == "signal lost"
        await beat(0.3)                                # signal back with L1 still held: stays stopped
        assert robot.state()["command"] is None and not pad.status()["signal_lost"]
        press(ecodes.BTN_TL, 0)
        await beat(0.1)
        press(ecodes.BTN_TL, 1)
        await beat(0.1)
        assert robot.state()["command"]["kind"] == "jog"
    run_pad(monkeypatch, scenario)


def test_without_a_heartbeat_node_the_pad_still_works_and_says_so(monkeypatch):
    async def scenario(pad, robot, beat, silence, press):
        await silence(0.1)
        press(ecodes.BTN_TL, 1)
        press(ecodes.ABS_X, 255)
        await silence(gamepad.STALE_S + 0.2)
        status = pad.status()
        assert robot.state()["command"]["kind"] == "jog"
        assert not status["watchdog"] and "no motion-sensor node" in status["watchdog_note"]
    run_pad(monkeypatch, scenario, heartbeat=False)


def test_a_controller_that_connects_with_l1_held_does_not_enable(monkeypatch):
    async def scenario(pad, robot, beat, silence, press):
        press(ecodes.ABS_X, 255)
        await beat(0.2)
        assert robot.state()["command"] is None and not pad.status()["enabled"]
    robot_keys = {ecodes.BTN_TL}
    original = FakeNode.active_keys
    monkeypatch.setattr(FakeNode, "active_keys", lambda self: list(robot_keys) if self.name == "pad" else original(self))
    run_pad(monkeypatch, scenario)
