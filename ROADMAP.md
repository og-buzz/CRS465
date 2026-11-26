# Roadmap

Things asked for or found, not built yet. Newest first.

## Goal: the functions of Robcomm3

Set 2026-11-25. The owner: "Robcomm3 equivalent functions is what we really need to set as a /goal".
Robcomm3 is the vendor's Windows program for this controller and we have no copy, so the goal is
to have, on Linux, each function the vendor's manuals attribute to it. What the manuals say it
does, with citations, and how each function compares with this repo is in
[docs/ROBCOMM3.md](docs/ROBCOMM3.md). Status below is from its table, 2026-11-25.

- [x] Terminal window to the controller (`scripts/a465-term`)
- [x] Serial port and baud settings
- [~] Send and receive files (`a465-getfile` is verified up to 43 KB; `a465-putfile` is verified by the controller's `cksum` up to 3600 bytes, 2026-11-25)
- [~] Back up and restore controller files (a verified backup exists; no one-step command)
- [ ] Make a directory on the controller as part of sending
- [ ] Editor for RAPL-3 source
- [ ] Compile RAPL-3 source to an object file, with errors, warnings and options (we have only an assembler, disassembler and test interpreter for the bytecode)
- [ ] Multi-file programs and libraries (`.include`, `-L`)
- [ ] Application setups: saved source, controller directory, object name and options; compile and send in one step
- [~] Version check (`crosver` by hand; no check of a program's library stamps before sending)
- [~] Application variable (`.v3`) files: copied, not read or edited
- [ ] Teach, list and edit locations from the PC
- [~] Run and stop programs (specific supervised programs only; no general run)
- [~] Configuration and calibration files (`robot.cal` is read; the rest only copied)
- [ ] Firmware install (a separate vendor utility; not attempted)

The largest gap is the compiler. The rest is mostly wrapping what already works.

## ROS: export the arm model; what exists already

Asked 2026-11-25. Not built.

**Export.** Write the arm model we already have (`a465/model.py`: the joint chain, link offsets,
joint limits and speeds) out as a URDF file, so that the ROS 3D viewer and the MoveIt planner can
be tried later without redoing the kinematics. The model's tool position matched the controller's
own `w2` reading at one pose on 2026-11-25 (X and Y exact, Z 0.4 mm low, angles within 0.01
degrees). ROS itself is not planned for jogging: its joystick control streams targets many times
a second and this controller takes one move at a time (see docs/CONTROLLER.md, "Motion: what was
measured"). It would earn its place for planned moves around obstacles, sent as a continuous
path, and if the drives are ever replaced.

**Does anyone have this arm in ROS already?** Searched 2026-11-25: no ROS package, URDF or MoveIt
configuration for the A465 was found. The nearest thing is
[PetteriAimonen/crs-c500c-utils](https://github.com/PetteriAimonen/crs-c500c-utils) (BSD licence,
last commit 2025-05-16): tools for the C500C controller, written for a CRS F3 arm; its readme
names the A465 and A255 as arms the same controller runs. The author is also the source of the
manual set we use (jpa.kapsi.fi). What is in it, read 2026-11-25, none of it tried here:

- ROS 1 packages for the F3: a MoveIt configuration (F3 link lengths, so not usable for the A465
  as it stands) and a bridge from ROS-Industrial's simple_message protocol to the controller's
  serial port.
- The controller side of that bridge as RAPL-3 **source** (`simple_message/*.r3`): a status
  server and a control server that takes joint trajectory points and runs each with `online(ON)`
  and `move()`. On our controller that kind of queued move stops at every point (measured
  2026-11-25); whether it runs through on a C500C is not stated.
- Their controller (CROS 3.1.1249 on a C500C, 486 processor) has the RAPL-3 compiler on board:
  sources are sent over and compiled there with `r3c`. Our `/bin` (CROS 1.16.471 on a C500) has
  no `r3c`; other directories have not been checked for it.
- Copies of that controller's system programs, among them `rc1`, the compiler itself (157 KB,
  x86, FreeBSD a.out format) and `r3interp`, the bytecode interpreter. A lead for the missing
  compiler in the Robcomm3 goal: whether `rc1` can be run on a PC, and whether what it produces
  suits our older firmware, is not known.
- Things worth trying on our unit: their control server installs a handler for SIGABRT so an
  emergency stop does not kill it (our jog program is killed by SIGABRT on a controller fault);
  it talks over a second serial port, `/dev/sio1`, at 115200 baud, leaving the console free;
  `/bin/rdb`, a debugger that lists a program's symbols (their notes give the variable names in
  `robotsrv`, such as `robot_homed`, `robot_calibrated`, `poc_check_disable`); a file transfer
  that needs only a small `uudecode` program on the controller.

## Camera calibration

Asked for 2026-11-25. After discussing lens calibration and tying the bench camera to the arm, the
owner said "have an agent start this". Now: `a465/camcal.py`, `scripts/a465-camcal` and
`tests/test_camcal.py` (synthetic data only), `docs/CAMERA.md` (procedure), and printable targets in
`docs/camera/` (ChArUco board for the lens, one ArUco marker for the gripper). The lens step
(`grab`, `lens`, `undistort`) and the camera-to-robot step (`robot`, `project`) are built and tested on
synthetic data. Remaining: the lens capture (board held by a person), the robot capture (marker on the
gripper, arm taken to 12 or more stopped poses), then using both to locate parts on the bench.

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
