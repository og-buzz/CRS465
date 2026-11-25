# Roadmap

Things asked for or found, not built yet. Newest first.

## Digital twin

Asked for 2026-11-24. The web page's 3D arm should mirror the real arm live, and the page should be
where the joint positions are seen while jogging. Today the page shows only the simulated arm, and the
real arm is driven by `scripts/a465-padjog` with nothing on screen.

What there is to build it from:

- The arm homes properly (`scripts/a465-home`), so the controller's joint readings are true angles.
- `jogd`, our program on the controller, answers `p` with all six joints' encoder counts in about
  0.1 s. `a465/jogd.py` (`JogdArm`) already polls it and has the same methods and `state()` keys as
  the simulated arm the server uses.

Steps:

1. A switch for `a465.server` that uses `JogdArm` as its robot, so the 3D view, the joint readouts and
   the gamepad all work on the real arm. One program owns the serial port, so in this mode the pad has
   to go through the server.
2. Show what the simulation cannot know: homed or not, the controller's last error line, the speed
   setting, whether `jogd` is running.
3. Position while `jogd` is not running (during homing, or while a program runs): read `w2`.
4. Later: the page's moves (go to a pose, pick and place) sent to the real arm, tried on the simulated
   arm first.

## Coordinated motion on the real arm

Asked for 2026-11-23. Today the real arm moves one joint at a time: a jog is a single-axis
`joint <axis>,<degrees>` command, and commands queued behind it run one after another, not together.
Coordinated motion means several joints moving at once and arriving together, which is what the
mock already does (world and tool jog, go-to poses, and the planned pick-and-place cycle).

What it needs, in order:

1. **A homed arm.** The controller allows only single-joint moves until it is homed, and homing is
   blocked because no home switch signal reaches the controller (see README).
2. **Moves that involve every joint.** This controller's application shell lists `move` and `moves`
   (to a taught location, joint-interpolated or in a straight line), `appro` / `depart`, and
   relative straight-line commands (`wx`, `wy`, `wz`, `tx`, `ty`, `tz`, `yaw`, `pitch`, `roll`).
   None of these has been tried here yet. To check first: how a location is created from numbers
   over the console (rather than taught with `here`), and whether queued moves blend or stop
   between each.
3. **Jogging a point in space from the pad.** With the commands above, world and tool jog become
   short relative straight-line moves, topped up while the stick is held, the same way joint jog
   works now. Two joints at once from two sticks comes for free in that mode. In joint mode it
   would need a move to a location that differs in two joints.
4. **Running a planned cycle.** The planner already produces checked joint-space moves and straight
   strokes. They would be sent as `move` / `moves` to locations, one step at a time, with the stop
   key and the keep-out checks unchanged.

The response-time limits in the next item apply here too.

## Jog speed from how far the stick is pushed

Status 2026-11-24: way 3 below is built. `jogd` (made with our own assembler, `a465/r3asm.py`; no
vendor compiler was needed) runs on the controller, and `scripts/a465-padjog --jogd` sends it the
stick's deflection as a speed. First run on the arm: smooth, about 0.25 s from stick to motion.
Still open: the driver's handling of a stop (being fixed), shortening the delay, travel limits in
this mode, and straight-line jogging in X, Y, Z (jogd's modes 3 and 4).

Asked for 2026-11-23. On the real arm a stick is on/off today: a jog runs at the set speed
(L1 + R1 + triggers) however far the stick is pushed, and one joint moves at a time.

Why it is not simple: the console protocol has no velocity command. A jog is a relative
`joint <axis>,<degrees>` move sent to the controller's application shell, and the speed is a
separate `speed <percent>` setting that applies to moves typed after it. Each line takes about
0.25 s to be acted on (measured), so the speed can be changed at most a few times a second, and
only at the join between two moves.

Ways to do it, cheapest first:

1. **Speed bands.** Three or four stick ranges mapped to fixed speeds (for example 3, 8, 15, 25%).
   When the stick moves to another band, send `speed` and then the next top-up move. Cost: a visible
   step in speed, about a quarter of a second late. No new protocol.
2. **Shorter moves.** Cut a jog into moves of about 0.4 s and set the speed before each one, so the
   speed follows the stick more closely. Needs a check that back-to-back moves blend instead of
   stopping between each (not measured yet).
3. **A program on the controller.** A small RAPL-3 program that reads a speed and direction from the
   serial line and runs the joint itself would respond in milliseconds. Needs the RAPL-3 compiler,
   which is part of Robcomm3 (Windows only, and no copy found yet).
4. **Another channel.** The controller runs an ACI daemon (`fastacid`) for Robcomm3's binary protocol
   on the same port. If that protocol has a jog or a faster position stream, it would beat typing at
   a shell. Not investigated.

## Other open items

- Homing works since 2026-11-24 (`scripts/a465-home`; see `docs/CONTROLLER.md`, "Homing that works").
  Open around it: the controller's proper shutdown command (it complains at boot that it "was not
  properly shut down"), and a straightedge check of joint 5's zero.
- The web page still shows only the simulated arm: see "Digital twin" above.
- Gripper air fittings: see `docs/PNEUMATICS.md`.
- Watching the home switches during a jog: the always-on status display (`axst &`) slowed every
  command by about 1.5 s and hung the controller's robot server after a stop. Needs another way.
- Planner: a cell right beside a keep-out zone shows as usable but can fail planning.
- Pick-and-place cells, zones and the track toggles are forgotten when the web service restarts.
