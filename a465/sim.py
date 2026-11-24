"""Mock A465 on its track: a kinematic simulation with the real arm's limits, speeds and accelerations.

It stands in for the controller until the serial link exists. Commands follow the
controller's own vocabulary (speed percent, halt, home, ready, calrdy, arm power,
outputs), plus velocity jogging that stops by itself unless it keeps being refreshed.
"""
import math

import numpy as np

from . import model, planner

FRAMES = ("joint", "world", "tool")
MAX_TTL_S = 1.0
ROT_JOG_DEG_S = 180.0      # tool rotation rate at 100% speed
HOME_SECONDS = 4.0         # how long the mock takes to "home"; it does not imitate the real search motion
LINEAR_ACCEL_MM_S2 = 1500.0     # how quickly a world or tool jog picks up and sheds speed
ANGULAR_ACCEL_DEG_S2 = 600.0
_LENGTH_SCALE = 300.0      # mm treated as comparable to one radian when solving cartesian jogs
_DAMPING = 0.02
_HOLD_GAIN = 10.0          # 1/s: how firmly an axis that is not being jogged is held where it was
_UNREACHABLE = 0.15        # refuse a world or tool jog when this fraction of it cannot be produced
_LIMIT_BAND_DEG = 0.1      # a joint this close to the end of its range counts as at its limit
_STRETCH_DEG = 10.0        # a world or tool jog will not straighten the elbow past this (2.4 mm short of full reach)

def _rotation_vector(rot):
    """The rotation matrix as an axis times an angle in radians."""
    axis = 0.5 * np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    sine = float(np.linalg.norm(axis))
    if sine < 1e-12:
        return np.zeros(3)
    return axis * (math.atan2(sine, (float(np.trace(rot)) - 1.0) / 2.0) / sine)


class RobotError(ValueError):
    """A command the robot refuses, with the reason."""


def _numbers(values, count, what):
    try:
        out = np.array([float(x) for x in values])
    except (TypeError, ValueError):
        out = None
    if out is None or out.shape != (count,) or not np.all(np.isfinite(out)):
        raise RobotError(f"{what} must be {'six' if count == 6 else 'two'} numbers")
    return out


class SimRobot:
    backend = "mock"

    def __init__(self):
        self.q = np.array(model.POSES_DEG["ready"], dtype=float)
        self.qd = np.zeros(6)
        self.track, self.track_v = 0.0, 0.0            # mm, mm/s
        self.carousel, self.carousel_v = 0.0, 0.0      # degrees (0..360), deg/s
        self.carousel_fitted = True    # False: no carousel, its angle is 0 and stays 0
        self.speed = 10            # percent; the C500C always starts at 10
        self.arm_power = False
        self.homed = False
        self.gripper_open = True
        self.light = False
        self.tool = False
        self.at_limit = [False] * 6
        self.track_at_limit = False
        self.t = 0.0               # simulated seconds
        self._jog = None           # (frame, vec, expires, source, aux)
        self._goto = None          # (target joints, source)
        self._home = None          # (finish time, source)
        self._run = None           # the pick-and-place plan being carried out
        self.job = planner.Job()   # which cells to pick from and place into
        self.parts = []            # the mock's cubes: {"x", "y", "z" (base centre, world mm), "held"}
        self.job_version = 0       # goes up whenever the cells change, so pages know to fetch the job again
        self._cart = None          # world / tool jog in progress: its ramped speed and the pose it holds
        self.notice = ""           # why a world or tool jog is stopped or slowed
        self._zone_notice = ("", 0.0)   # the last keep-out notice and when it stops being shown

    # ---- commands -------------------------------------------------------

    def jog(self, frame, vec, ttl=0.3, source="api", aux=(0.0, 0.0)):
        """Move at a fraction (-1..1 per axis) of the current speed until `ttl` seconds pass.

        `vec` is the six arm axes in the given frame; `aux` is (track, carousel).
        """
        if frame not in FRAMES:
            raise RobotError(f"frame must be one of {', '.join(FRAMES)}")
        v = _numbers(vec, 6, "vec")
        a = _numbers(aux, 2, "aux")
        if not self.job.track_fitted:
            a[0] = 0.0             # no track: a request to move it does nothing
        if not self.carousel_fitted:
            a[1] = 0.0
        if not isinstance(ttl, (int, float)) or not math.isfinite(ttl) or ttl <= 0:
            raise RobotError("ttl must be a positive number of seconds")
        self._require_power()
        if frame != "joint" and not self.homed:
            raise RobotError("not homed: only joint jogging works before homing")
        self.halt()
        self._jog = (frame, np.clip(v, -1.0, 1.0), self.t + min(ttl, MAX_TTL_S), source, np.clip(a, -1.0, 1.0))

    def goto(self, joints, source="api"):
        """Joint-interpolated move to a named pose ('ready', 'calrdy') or six joint angles."""
        target = self._target(joints)
        self._require_power()
        self._require_homed()
        self.halt()
        self._goto = (target, source)

    def home(self, source="api"):
        """Start homing. The arm counts as homed when it finishes without being halted."""
        self._require_power()
        self.halt()
        self._home = (self.t + HOME_SECONDS, source)

    def set_job(self, picks, places, plane_z=None, zones=None):
        """Choose the pick and place cells (lists of names like 'C10'), and optionally the plane height and the zones."""
        if self._run:
            raise RobotError("the cycle is running: halt it before changing cells")
        try:
            self.job.set(picks, places, plane_z, zones)
        except planner.JobError as err:
            raise RobotError(str(err)) from None
        self.job_version += 1
        self._reset_parts()

    def set_track(self, fitted=None, moves=None, carousel=None):
        """Say whether the track is fitted, whether the cycle may move it, and whether the carousel is fitted.

        Without a track the arm is bolted down with its base at X = 0, so the track snaps to 0 (a
        reconfiguration of the mock), and likewise the carousel angle without a carousel. Only allowed when nothing is moving and no command is active.
        """
        for name, value in (("fitted", fitted), ("moves", moves), ("carousel", carousel)):
            if value is not None and not isinstance(value, bool):
                raise RobotError(f"'{name}' must be true or false")
        if fitted is None and moves is None and carousel is None:
            raise RobotError("track needs 'fitted', 'moves' or 'carousel'")
        if self._command():
            raise RobotError("a command is active: halt it before changing the track settings")
        if self.state()["moving"]:
            raise RobotError("the arm is still moving: wait for it to stop before changing the track settings")
        if fitted is not None:
            self.job.track_fitted = fitted
            if not fitted:
                self.track, self.track_v = 0.0, 0.0
        if moves is not None:
            self.job.track_moves = moves
        if carousel is not None:
            self.carousel_fitted = carousel
            if not carousel:
                self.carousel, self.carousel_v = 0.0, 0.0
        self.job_version += 1
        self._reset_parts()

    def plan(self):
        """The cycle as it would run from where the arm is now."""
        return self.job.plan(self.q, self.track, self.speed, self.gripper_open)

    def cycle_start(self, source="api"):
        """Work out the pick-and-place cycle from here, and run it if every step of it checks out."""
        self._require_power()
        self._require_homed()
        plan = self.plan()
        if plan.problems:
            raise RobotError(f"cycle not started: {plan.problems[0]}")
        if not plan.steps:
            raise RobotError("cycle not started: choose pick and place cells on the page first")
        self.halt()
        self._reset_parts()
        self._run = {"plan": plan, "source": source, "step": 0, "at": 0.0, "until": None, "rate": 1.0}

    def halt(self):
        self._jog = self._goto = self._home = self._run = None

    def _reset_parts(self):
        """One cube on each pick cell that has a place to go."""
        picks, places, _ = self.job.resolve(self.track)
        self.parts = [{"x": x, "y": y, "z": self.job.plane_z, "held": False}
                      for x, y in map(planner.cell_centre, picks[:len(places)])]

    def set_speed(self, percent):
        if isinstance(percent, bool) or not isinstance(percent, (int, float)) or not 1 <= percent <= 100:
            raise RobotError("speed must be a percentage from 1 to 100")
        self.speed = int(percent)

    def set_arm_power(self, on):
        self.arm_power = bool(on)
        if not self.arm_power:
            self.halt()
            self.qd[:] = 0.0       # brakes engage
            self.track_v = self.carousel_v = 0.0

    def set_gripper(self, open_):
        """Open or close the jaws. Closing them around a cube picks it up; opening them drops it onto the plane."""
        open_ = bool(open_)
        tip = model.fingertip(self.q, self.track)
        if open_:
            for part in self.parts:
                if part["held"]:
                    part.update(x=float(tip[0]), y=float(tip[1]), z=self.job.plane_z, held=False)
        elif self.gripper_open and not any(part["held"] for part in self.parts):
            for part in self.parts:
                if math.hypot(part["x"] - tip[0], part["y"] - tip[1]) <= planner.PART_MM / 2 and 0.0 <= tip[2] - part["z"] <= planner.PART_MM:
                    part["held"] = True
                    break
        self.gripper_open = open_

    def set_light(self, on):
        self.light = bool(on)

    def set_tool(self, on):
        self.tool = bool(on)

    @staticmethod
    def _target(joints):
        if isinstance(joints, str):
            if joints not in model.POSES_DEG:
                raise RobotError(f"unknown pose '{joints}'")
            joints = model.POSES_DEG[joints]
        target = _numbers(joints, 6, "joints")
        for name, angle, (lo, hi) in zip(model.JOINT_NAMES, target, model.LIMITS_DEG):
            if not lo <= angle <= hi:
                raise RobotError(f"{name} target {angle:g} is outside {lo}..{hi}")
        return target

    def _require_power(self):
        if not self.arm_power:
            raise RobotError("arm power is off")

    def _require_homed(self):
        if not self.homed:
            raise RobotError("not homed")

    # ---- simulation -----------------------------------------------------

    def step(self, dt):
        self.t += dt
        if self._jog and self.t >= self._jog[2]:
            self._jog = None       # nobody refreshed the jog: stop
        if self._home and self.t >= self._home[0]:
            self.homed, self._home = True, None
        if self._run:
            self._follow_plan(dt)
            self._cart, self.notice = None, ""
            self._step_carousel(0.0, dt)
            return

        share = self.speed / 100.0
        vmax = np.array(model.MAX_SPEED_DEG_S) * share
        want = np.zeros(6)
        track_want = carousel_want = 0.0
        if self._goto:
            err = self._goto[0] - self.q
            if np.max(np.abs(err)) < 0.01 and np.max(np.abs(self.qd)) < 0.5:
                self.q, self._goto = self._goto[0].copy(), None
                self.qd[:] = 0.0
            else:
                want = self._fit(err * 4.0, vmax)
        elif self._jog:
            frame, vec, _, _, aux = self._jog
            want = vec * vmax if frame == "joint" else self._cartesian(frame, vec, vmax, dt)
            track_want = float(aux[0]) * model.TRACK_MAX_SPEED_MM_S * share
            carousel_want = float(aux[1]) * model.CAROUSEL_MAX_SPEED_DEG_S * share

        if not (self._jog and self._jog[0] != "joint"):
            self._cart, self.notice = None, ""

        keepouts = self.job.keepouts()
        if keepouts and (self._jog or self._goto):
            want, track_want = self._zone_guard(keepouts, want, track_want, dt)

        self.qd = self._velocity_after(want, dt)
        self.q = self.q + self.qd * dt
        for i, (lo, hi) in enumerate(model.LIMITS_DEG):
            self.at_limit[i] = not lo + _LIMIT_BAND_DEG < self.q[i] < hi - _LIMIT_BAND_DEG
            if not lo < self.q[i] < hi:
                self.q[i] = min(max(self.q[i], lo), hi)
                self.qd[i] = 0.0

        step = model.TRACK_ACCEL_MM_S2 * dt
        self.track_v += min(max(track_want - self.track_v, -step), step)
        self.track += self.track_v * dt
        self.track_at_limit = not 0.0 < self.track < model.TRACK_TRAVEL_MM
        if self.track_at_limit:
            self.track = min(max(self.track, 0.0), model.TRACK_TRAVEL_MM)
            self.track_v = 0.0
        self._step_carousel(carousel_want, dt)
        if not self.notice and self.t < self._zone_notice[1]:
            self.notice = self._zone_notice[0]             # a keep-out notice stays up for a moment after the jog ends

    def _velocity_after(self, want, dt):
        """Joint speeds one tick from now if the joints are asked for `want`."""
        dv = np.array(model.ACCEL_DEG_S2) * dt
        change = want - self.qd
        if self._cart is not None or not (self._jog or self._goto):
            # A point moving in space, or coasting to rest: every joint changes speed in step, so the
            # point keeps its direction while it speeds up and slows down.
            change /= max(1.0, float(np.max(np.abs(change) / dv)))
        else:
            change = np.clip(change, -dv, dv)
        return self.qd + change

    def _zone_guard(self, keepouts, want, track_want, dt):
        """Stop a jog or go-to that would carry the arm into a keep-out zone.

        Looks one tick ahead, then asks where the arm would come to rest if it braked from there. If that
        pose is inside a zone that the arm is not inside now, the motion is stopped (a jog is treated as
        zero so the arm brakes; a go-to is halted) and `notice` names the zone. A zone the arm is already
        inside never blocks it, so it can be jogged out.
        """
        qd = self._velocity_after(want, dt)
        step = model.TRACK_ACCEL_MM_S2 * dt
        track_v = self.track_v + min(max(track_want - self.track_v, -step), step)
        # Braking every joint in step (the gentler way to stop is shorter, so this is the cautious estimate).
        seconds = float(np.max(np.abs(qd) / np.array(model.ACCEL_DEG_S2)))
        rest = np.clip(self.q + qd * dt + qd * seconds / 2.0, [lo for lo, _ in model.LIMITS_DEG], [hi for _, hi in model.LIMITS_DEG])
        rest_track = min(max(self.track + track_v * dt + track_v * abs(track_v) / (2.0 * model.TRACK_ACCEL_MM_S2), 0.0), model.TRACK_TRAVEL_MM)
        inside = planner.zones_hit(self.q, self.track, keepouts)
        entering = [n for n in planner.zones_hit(rest, rest_track, keepouts) if n not in inside]
        if not entering:
            if inside:
                self.notice = f"the arm is inside keep-out zone {inside[0]}; it can be jogged out"
            return want, track_want
        what = "go-to" if self._goto else f"{self._jog[0]} jog"
        self._zone_notice = (f"{what} stopped: the arm would enter keep-out zone {entering[0]}", self.t + 2.0)
        self.notice = self._zone_notice[0]
        if self._goto:
            self._goto = None
        else:
            frame, _, expires, source, _ = self._jog
            self._jog = (frame, np.zeros(6), expires, source, np.zeros(2))
        if self._cart is not None:
            self._cart["twist"][:] = 0.0
        return np.zeros(6), 0.0

    def _step_carousel(self, want, dt):
        step = model.CAROUSEL_ACCEL_DEG_S2 * dt
        self.carousel_v += min(max(want - self.carousel_v, -step), step)
        self.carousel = (self.carousel + self.carousel_v * dt) % 360.0

    def _follow_plan(self, dt):
        """Play the planned samples back, faster or slower if the speed has changed since planning."""
        run = self._run
        plan = run["plan"]
        before, track_before = self.q.copy(), self.track
        step = plan.steps[run["step"]]
        if step.kind == "grip":
            if run["until"] is None:
                self.set_gripper(step.open)
                run["until"] = self.t + planner.GRIP_SECONDS
            done = self.t >= run["until"]
        else:
            last = len(step.samples) - 1
            run["rate"] += min(max(self.speed / plan.speed - run["rate"], -2.0 * dt), 2.0 * dt)   # a speed change eases in
            run["at"] = min(run["at"] + dt / planner.SAMPLE_S * run["rate"], last)
            lower = min(int(run["at"]), last - 1) if last else 0
            mix = run["at"] - lower
            row = step.samples[lower] * (1.0 - mix) + step.samples[min(lower + 1, last)] * mix
            self.q, self.track = row[:6].copy(), float(row[6])
            done = run["at"] >= last
        self.qd = (self.q - before) / dt
        self.track_v = (self.track - track_before) / dt
        self.at_limit = [False] * 6
        self.track_at_limit = False
        if done:
            run.update(step=run["step"] + 1, at=0.0, until=None)
            if run["step"] >= len(plan.steps):
                self._run = None
                self.qd[:] = 0.0                       # the plan ends at rest
                self.track_v = 0.0

    @staticmethod
    def _fit(qd, vmax):
        """Slow every joint by the same factor so none exceeds its limit (keeps the direction)."""
        worst = np.max(np.abs(qd) / vmax)
        return qd / worst if worst > 1.0 else qd

    def _cartesian(self, frame, vec, vmax, dt):
        """Joint velocities (deg/s) that move the tool point as a world or tool jog asks.

        The stick sets the speed of the point along the frame's axes. Every axis that is not being
        pushed is held where it was, so the point travels in a straight line and the tool keeps
        its angle. When the arm cannot do that (a joint at its limit, or a shape where the motion
        does not exist) it stops rather than drift, and `notice` says why.
        """
        tcp, rot, _, _ = model.forward(self.q)
        cart = self._cart
        if cart is None or cart["frame"] != frame:
            cart = self._cart = {"frame": frame, "twist": np.zeros(6), "at": tcp.copy(), "facing": rot.copy()}
        share = self.speed / 100.0
        goal = np.concatenate([vec[:3] * model.MAX_LINEAR_MM_S * share, np.radians(vec[3:] * ROT_JOG_DEG_S * share)])
        ramp = np.repeat([LINEAR_ACCEL_MM_S2 * dt, math.radians(ANGULAR_ACCEL_DEG_S2) * dt], 3)
        cart["twist"] += np.clip(goal - cart["twist"], -ramp, ramp)
        speed = cart["twist"]

        axes = rot if frame == "tool" else np.eye(3)
        drift = axes.T @ (cart["at"] - tcp)              # how far each frame axis is from where it is held
        drift[np.abs(speed[:3]) > 1e-9] = 0.0            # an axis being jogged is free
        cart["at"] = tcp + axes @ drift
        if np.any(np.abs(speed[3:]) > 1e-12):
            cart["facing"] = rot.copy()                  # turning: the angle is free
            turn_back = np.zeros(3)
        else:
            turn_back = _rotation_vector(cart["facing"] @ rot.T)
        twist = np.concatenate([(axes @ (speed[:3] + _HOLD_GAIN * drift)) / _LENGTH_SCALE,
                                axes @ speed[3:] + _HOLD_GAIN * turn_back])
        size = float(np.linalg.norm(twist))
        if size < 1e-9:
            self.notice = ""
            return np.zeros(6)

        jac = model.jacobian(self.q)
        jac[:3] /= _LENGTH_SCALE
        qd = jac.T @ np.linalg.solve(jac @ jac.T + _DAMPING ** 2 * np.eye(6), twist)
        if float(np.linalg.norm(jac @ qd - twist)) > _UNREACHABLE * size:
            return self._refuse(frame, model.nearest_singularity(self.q)[0])
        qd = np.degrees(qd)

        worst = float(np.max(np.abs(qd) / vmax))         # joints capped at their speed: the point slows, on its line
        if worst > 1.0:
            qd /= worst
        brake = 1.0                                      # slow down in time to stop at a joint limit
        stopped_by = None
        for i, (lo, hi) in enumerate(model.LIMITS_DEG):
            if abs(qd[i]) < 1e-9:
                continue
            room = (hi - self.q[i] if qd[i] > 0 else self.q[i] - lo) - _LIMIT_BAND_DEG / 2
            why = f"J{i + 1} ({model.JOINT_NAMES[i]}) is at its limit"
            if i == 2 and (qd[i] > 0) == (self.q[i] < 0):                # the elbow is straightening: full stretch is a limit too
                stretch = abs(self.q[i]) - _STRETCH_DEG
                if stretch < room:
                    room, why = stretch, model.SINGULARITIES["elbow"]
            allowed = 0.7 * math.sqrt(2.0 * model.ACCEL_DEG_S2[i] * max(room, 0.0))
            if abs(qd[i]) > allowed and allowed / abs(qd[i]) < brake:
                brake, stopped_by = allowed / abs(qd[i]), why
        if brake < 1e-3:
            return self._refuse(frame, stopped_by)
        near, closeness = model.nearest_singularity(self.q)
        self.notice = f"{frame} jog slowed: {near}" if worst > 3.0 and closeness < 0.35 else ""
        return qd * brake

    def _refuse(self, frame, why):
        self.notice = f"{frame} jog stopped: {why}"
        self._cart["twist"][:] = 0.0
        return np.zeros(6)

    # ---- reporting ------------------------------------------------------

    def _command(self):
        if self._jog:
            return {"kind": "jog", "frame": self._jog[0], "source": self._jog[3]}
        if self._run:
            step = self._run["plan"].steps[self._run["step"]]
            return {"kind": "cycle", "source": self._run["source"], "step": self._run["step"] + 1,
                    "of": len(self._run["plan"].steps), "label": step.label, "cell": step.cell}
        if self._home:
            return {"kind": "home", "source": self._home[1]}
        if self._goto:
            return {"kind": "goto", "source": self._goto[1]}
        return None

    def _parts(self):
        tip = model.fingertip(self.q, self.track)
        held = [float(tip[0]), float(tip[1]), float(tip[2]) - planner.GRIP_HEIGHT_MM]
        return [{"x": round(x, 1), "y": round(y, 1), "z": round(z, 1), "held": part["held"]}
                for part in self.parts
                for x, y, z in [held if part["held"] else (part["x"], part["y"], part["z"])]]

    def state(self):
        tcp, rot, _, _ = model.forward(self.q, self.track)
        yaw, pitch, roll = model.yaw_pitch_roll(rot)
        moving = bool(max(float(np.max(np.abs(self.qd))), abs(self.track_v), abs(self.carousel_v)) > 0.01)
        return {
            "backend": self.backend,
            "t": round(self.t, 3),
            "joints": [round(float(a), 3) for a in self.q],
            "track": round(self.track, 2),
            "carousel_fitted": self.carousel_fitted,
            "track_fitted": self.job.track_fitted,
            "track_moves": self.job.track_moves,
            "carousel": round(self.carousel, 2),
            "pose": {"x": round(float(tcp[0]), 2), "y": round(float(tcp[1]), 2), "z": round(float(tcp[2]), 2),
                     "yaw": round(yaw, 2) + 0.0, "pitch": round(pitch, 2) + 0.0, "roll": round(roll, 2) + 0.0},
            "speed": self.speed,
            "arm_power": self.arm_power,
            "homed": self.homed,
            "moving": moving,
            "at_limit": list(self.at_limit),
            "track_at_limit": bool(self.track_at_limit),
            "gripper_open": self.gripper_open,
            "light": self.light,
            "tool": self.tool,
            "notice": self.notice,
            "parts": self._parts(),
            "job_version": self.job_version,
            "command": self._command(),
        }
