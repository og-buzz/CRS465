# crs-a465

Control stack, notes, manuals and terminal sessions for a CRS A465 six-axis arm on
a C500C controller (T475 configuration: extra axis outputs for a track and a
carousel), worked from a Linux dev box.

## Status (2026-11-23)

- **Web interface + REST API + PS4 controller: working against a mock robot.**
  The mock uses the real arm's dimensions, joint ranges, speeds and accelerations.
- **The real controller answers over serial (since 2026-11-23): 38400 baud, 8N1,**
  `/dev/ttyUSB0`. Reading its state works, and so does joint jogging with a proven stop.
  The built-in home search runs the waist into its end stop; homing on the encoder index pulse
  works (all six joints, 2026-11-24). See [Homing that works](#homing-that-works) and
  [The real controller: what has been measured](#the-real-controller-what-has-been-measured).
- Manuals gathered and searchable (see [docs/SOURCES.md](docs/SOURCES.md)).

## The interface

```sh
sudo apt install python3-aiohttp python3-evdev python3-numpy python3-serial python3-unicorn
python3 -m a465.server              # http://<this machine>:8465
python3 -m a465.server --track --carousel   # start with the track and the carousel fitted
```

`python3-unicorn` is only for the RAPL-3 compiler: `scripts/a465-r3c prog.r3 -o prog -s` compiles on this
computer with CRS's own compiler run inside an emulator (files in `vendor/crs/`, which are CRS's, not ours;
see [docs/RAPL3-COMPILER.md](docs/RAPL3-COMPILER.md)). It sends nothing to the controller.

Neither the track nor the carousel is assembled on the real machine, so the server **starts
with both not fitted** unless `--track` and `--carousel` are given. The page's toggles change
them at run time (and `POST /api/track`); the starting state is the only thing the flags set.
The user unit in `systemd/` runs without flags, so the live service starts without them.

To keep it running across logins, install the user service:

```sh
cp systemd/a465.service ~/.config/systemd/user/ && systemctl --user enable --now a465
```

The page shows a live 3D view of the arm, tool position, joint angles against
their limits, hold-to-move jog buttons (joint / world / tool frames), speed, arm
power, home, cycle start, light and tool outputs, HALT, the track and carousel,
the pick-and-place planner, and the controller's live state.

**Everything that moves the arm is a jog that expires.** A jog command carries a
lifetime (default 0.3 s, max 1 s) and must keep being re-sent. If the sender
stops, for any reason, the arm stops.

There is no login and no single-operator lock yet. That is acceptable for the
mock; both are needed before this drives the real arm.

### Joint mode versus world and tool modes

In **joint** mode each control turns one motor, so the tool swings in arcs.

In **world** and **tool** modes the controls move the tool point itself. Push X and the
point travels in a straight line with the tool holding its angle; all six joints do
whatever that takes. Turn it and the tool rotates about that point while the point stays
where it is. World directions are fixed to the room (X along the track), tool directions
to the gripper. Every axis that is not being pushed is held where it was.

A straight line is not always possible, and the mock does not pretend otherwise:

- At a **joint limit**, or at **full stretch**, the point slows down along its line and
  stops there. It does not slide sideways.
- With the **wrist straight** (J5 at 0, as in the ready pose), with the **arm straight**
  (straight up), or with the wrist over the waist axis, some directions do not exist.
  A jog in one of those is refused; the directions that do exist still work. Near such
  a shape the joints have to race, so the point slows down.
- The header shows why, for example `world jog stopped: the wrist is straight (J5 near 0)`.
  Get out of it in joint mode, or press **Work pose**: tool pointing down in front of
  the arm, every joint mid-range, which is a good place to start world and tool jogging.

Measured on the mock at 20% speed from the work pose: under 0.5 mm off the line and
under 0.3 degrees of tilt in every direction, including runs into a limit. At 100%
speed (1 m/s) the worst seen was a few millimetres while accelerating.

### Pick and place

The work surface is a horizontal plane (default: level with the base) in 100 mm cells,
rows A to N across the track and columns 1 to 34 along it. Cells the arm can serve are
drawn as tiles. Press **Choose cells**, then click a cell to pick from and the cell to
place into; repeat for more pairs. Clicking a chosen cell removes it. **Top view** shows
the whole grid as a map.

Every change is planned at once, before anything moves (`a465/planner.py`):

- Moves between cells are joint by joint, all axes starting and stopping together, with
  the track sliding at the same time. That kind of move has no singularities.
- Only the 110 mm strokes down to a cell and back are straight lines. The tool points
  straight down for those, and every 20 ms sample of each stroke must be inside the
  joint limits with the wrist bent and the arm short of full stretch.
- For each cell the planner picks the track position the arm reaches soonest among those
  that put the cell at a comfortable reach.
- Every sample is checked against each axis's speed and acceleration, and the gripper
  must stay 40 mm above the plane between cells.

The result is drawn as a green line (the fingertips' route) with the time it will take
at the current speed. If a cell cannot be served the plan says which and why, and cycle
start is refused. **Cycle start** (or L1 + R3 on the controller) plans again from where
the arm is and runs it. The mock puts a cube on each pick cell; closing the gripper
around a cube picks it up and opening it drops the cube on the plane, by hand as well
as in the cycle.

**Track and carousel settings.** The page has toggles for three things; each can be changed
only when nothing is moving and no command is active (otherwise it is refused, like changing
cells during a cycle):

- **Track fitted.** Off means the arm is bolted down with its base at world X = 0: the track
  position is 0 and stays 0, track jogs from the page or the controller do nothing, the page
  hides the track's jog row and readout and the 3D scene hides the rail, and the reach map and
  the planner use a base fixed at X = 0 (without the rail, the cells over its centre line are
  ordinary cells). Turning it off **snaps the track to 0**; that is a reconfiguration of the mock.
- **Move track in cycle.** Off means the planner keeps the track where it is when the cycle
  starts, for the whole cycle. The reach map then shows what is reachable from the current track
  position and is drawn again after the track has been jogged somewhere else and has come to rest.
- **Carousel fitted.** Off means the carousel angle is 0 and stays 0, its jog requests do
  nothing and the page hides its jog row and readout. (The 3D scene draws no carousel.)

**Zones.** The card has a 2D map of the whole grid seen from above (X right, Y up, as in
**Top view**) showing reach shading, the rail, the arm's base and fingertip live, numbered pick
and place cells, and the zones. Clicking a cell on it does what clicking in the 3D view does
(with **Choose cells** on); dragging a rectangle fills the zone form's X and Y fields, snapped to
50 mm, without adding anything. **Enlarge map** blows it up over the 3D view. A zone is one of:

- **Pick area / Place area**: every usable cell whose centre is inside the rectangle (edges
  included), row A first, then by column, is added to the pick (or place) list after the clicked
  cells, in zone order, duplicates dropped. Cells that are out of reach, over the track or inside
  a keep-out are left out; each zone shows "N cells (M left out)" and the plan gets one note per
  zone that left any out. Pick N still goes to place N. A change that would make a cell both a
  pick and a place is refused, naming the cells; that test looks at every cell inside the areas
  (usable or not) so the answer does not depend on where the track is.
- **Keep-out zone** (`top` in world Z, at most 1500 mm and above the plane): the volume below
  `top` inside the rectangle is forbidden. A cell whose square overlaps one at all is unusable
  (letter `k` in the reach map, red on the page). The arm is "inside" a keep-out when any point
  sampled every 30 mm or closer along its shoulder-elbow-wrist-flange-fingertip line is within
  `ARM_MARGIN_MM` (60 mm) of the rectangle horizontally and below `top` plus that margin.
  The planner checks every sample of every step. When a direct move between cells would enter
  one it tries a higher travel height at both ends (from the tallest top plus 40 mm, in 50 mm
  steps, as long as the longer strokes stay valid for the arm), takes the first that clears and
  adds a note; if none does, the plan has a problem naming the step and the zone. The same check
  covers the first move and the last move to the work pose. If the arm starts inside a keep-out
  the plan says to jog it out first. Jogs and go-to moves are guarded too: each tick the sim asks
  where the arm would come to rest if it braked now, and if that is inside a keep-out the arm is
  not already inside, the motion is stopped and `notice` names the zone. A keep-out the arm is
  already inside never blocks it, so it can be jogged out.

The planner knows nothing else about collisions except staying above the plane. The arm's body
is only the five key points and the segments between them, so the base column and the gripper
jaws are not tested.

### PS4 controller (on this machine, USB or Bluetooth)

Either connection works and the server picks the controller up by itself. Over Bluetooth it is
paired and trusted, so pressing **PS** reconnects it; hold PS about 10 seconds to switch it off.
To pair a new one, run `scripts/pair-gamepad`, unplug its USB cable and hold **Share + PS** until
the light bar double-blinks.

**Signal-loss watchdog.** When a wireless controller goes out of range or its battery dies, Linux
keeps showing the last stick position until the link times out (measured here: 20 s). So the server
also listens to the controller's motion-sensor node, which reports about 200 times a second whether
or not anything is touched. After 0.3 s of silence it stops whatever the controller started and the
page shows SIGNAL LOST; when the signal returns, L1 must be let go and held again. The same re-arm
applies after a reconnect. Reading that node needs `udev/70-a465-gamepad.rules` installed (it is on
this machine); without it the page says "no signal watchdog" and the controller works as before.

Nothing starts unless **L1 is held**. **Square stops everything**, with or without L1.

| Control | Joint frame | World / tool frame |
|---|---|---|
| Left stick | J1 waist (left/right), J2 shoulder (up/down) | X (left/right), Y (up/down) |
| Right stick | J4 wrist rotate (left/right), J3 elbow (up/down) | yaw (left/right), Z (up/down) |
| D-pad | J5 wrist pitch (up/down), J6 tool roll (left/right) | pitch (up/down), roll (left/right) |

In every frame:

| Control | Does |
|---|---|
| L1 + L2 / R2 | Track back / forward, proportional to the trigger |
| L1 + R1 + L2 / R2 | Speed down / up (up to 30% per second) |
| L1 + R1 + D-pad left / right | Carousel |
| R1 (hold) | Sticks steer the 3D view: right stick orbits, left stick up / down zooms |
| L1 + R1 + Circle | Arm power on / off |
| L1 + Circle / Cross | Gripper open / close |
| L1 + Triangle | Point straight up (calrdy) at 10% speed |
| L1 + L3 click | Home |
| L1 + R3 click | Cycle start (the pick-and-place plan) |
| Square | Stop everything, whoever started it |
| Options | Next frame |

- Jogging lasts only while L1 is held. A move started with L1 and a button (straight
  up, home, cycle start) carries on by itself after a tap; stop it with Square, with
  HALT on the page, or by taking over with a stick while holding L1.
- If the controller that started a move drops off (signal lost, unplugged), the move stops.
- Clicking a stick ignores that stick's own nudge until it is back at centre.
- After R1 is released a stick stays inert until it returns to centre.
- Sticks have a dead zone and a squared response for fine control.

Button codes were read from a DualShock 4 (054c:05c4) through the kernel's
`hid-playstation` driver. A DualSense should report the same codes; untested.

### REST and WebSocket

```sh
curl http://HOST:8465/api/state
curl http://HOST:8465/api/model                       # geometry, limits, command list
curl -X POST http://HOST:8465/api/armpower -d '{"on": true}'
curl -X POST http://HOST:8465/api/speed    -d '{"percent": 25}'
curl -X POST http://HOST:8465/api/jog      -d '{"frame": "joint", "vec": [1,0,0,0,0,0], "ttl": 0.3}'
curl -X POST http://HOST:8465/api/jog      -d '{"frame": "joint", "vec": [0,0,0,0,0,0], "aux": [1, 0]}'   # track forward
curl -X POST http://HOST:8465/api/home
curl -X POST http://HOST:8465/api/goto     -d '{"pose": "ready"}'     # or "calrdy", "work", or {"joints": [..6 angles..]}
curl http://HOST:8465/api/job                                         # grid, chosen cells, and the plan from here
curl -X POST http://HOST:8465/api/job      -d '{"picks": ["C10", "D12"], "places": ["L20", "K4"]}'   # optional "plane_z": mm
curl -X POST http://HOST:8465/api/job      -d '{"picks": ["C10"], "places": ["L20"], "zones": [
    {"kind": "pick", "x0": 200, "y0": -460, "x1": 460, "y1": -340},
    {"kind": "keepout", "x0": 700, "y0": -420, "x1": 800, "y1": -200, "top": 300}]}'   # zones omitted: unchanged; [] clears
curl -X POST http://HOST:8465/api/track    -d '{"fitted": false, "moves": false, "carousel": false}'   # any of the three
curl -X POST http://HOST:8465/api/cycle                               # cycle start: run the plan
curl -X POST http://HOST:8465/api/gripper  -d '{"open": false}'
curl -X POST http://HOST:8465/api/light    -d '{"on": true}'
curl -X POST http://HOST:8465/api/tool     -d '{"on": true}'
curl -X POST http://HOST:8465/api/halt
```

- `jog`: `frame` is `joint`, `world` or `tool`. `vec` is six numbers from -1 to 1:
  fractions of the current speed for J1..J6, or for X, Y, Z, roll, pitch, yaw.
  Optional `aux` is `[track, carousel]`, also -1 to 1.
- Until the arm is homed only joint-frame jogs are accepted; `goto`, `cycle` and
  world / tool jogs answer "not homed".
- `job`: pick 1 goes to place 1, and so on. The reply carries the plan: `ok`, `problems`,
  `seconds`, the `steps`, the track position chosen for each cell, and the `path` to draw.
  `cycle` answers 400 with the first problem if the plan does not check out.
- `job` also takes `zones`, a list (at most 20) of `{kind, x0, y0, x1, y1, top}` with `kind` one
  of `pick`, `place`, `keepout`, in millimetres, corners in any order (`top` is used only by
  keep-outs). `GET /api/job` returns `zones` (each area with its `cells` and `left_out` count),
  the clicked `picks` / `places`, and the `effective_picks` / `effective_places` the plan uses.
  `track` takes `fitted`, `moves` and `carousel` (each optional, true or false); it is refused
  while anything is moving or a command is active. State carries `track_fitted`, `track_moves`
  and `carousel_fitted`.
- State carries `notice` (why a world or tool jog is stopped or slowed), `parts` (the
  mock's cubes) and, during a cycle, the step being run.
- Refused commands return HTTP 400 with `{"error": "<reason>"}`.
- `ws://HOST:8465/api/ws` pushes the same state 20 times a second and accepts
  the same commands as `{"op": "jog", ...}`.

### What the mock gets from the manual, and what is provisional

From the A465 guide: joint ranges (Table 2), section lengths (Table 3), joint
speeds (Table 7), accelerations (Table 8), positive joint directions (Table 20).
World frame per the Application Environment guide: origin at the base mounting
flange, Z up, X forward. Start-up speed 10% (C500C guide p.40).

Provisional until checked on the real arm with `ready` then `wact`:

- Joint zero is taken as the middle of each range, so all-zero is straight up
  (calrdy) and ready is elbow = -90. Read off the range-of-motion drawing (Fig. 2)
  and the `calrdy` description.
- Yaw / pitch / roll sign and order.
- The gripper, light and tool are plain on/off flags. On the controller they will
  be digital outputs (`output <channel>, <state>`); the channel numbers are not known yet.
- Track (axis 7): speed 0.8 m/s and acceleration 3 m/s² are the T475 figures from
  the Track User Guide. Travel of 2134 mm ("about 7 ft") and its direction along
  +X are from the owner; the real limits are in the controller's configuration.
- Track and carousel are **not assembled on the real machine** (owner), so the server starts
  with both not fitted (see the `--track` / `--carousel` flags above); the robot class itself
  defaults to fitted. With no track the base is taken to be at X = 0.
- Keep-out clearance: `ARM_MARGIN_MM` = 60 mm is a **stand-in, not a measured value**. The arm is
  reduced to its five key points joined by straight lines; the real arm's thickness, cables and
  gripper have not been measured.
- Carousel (axis 8): nothing is known about it. It is a free-turning axis at a
  made-up 90 deg/s, shown as a number only.
- Homing is a 4-second wait that sets the homed flag; it does not imitate the real
  search motion. The mock starts un-homed, like the controller after power-up.
- "Cycle start" runs the pick-and-place plan. The gripper is taken to be 84 mm from
  flange face to fingertips (the one the page draws) and to open along world X; the
  reported pose is still the flange face. The plane height, cell size and the cubes
  are inventions of the mock.
- World and tool jogging is the mock's own arithmetic. The real controller does its
  own, and how it behaves at limits and singular shapes has to be observed on it.
- The 3D shape (`a465/web/arm.js`) is modelled by hand from the A465 guide's
  drawings (Fig. 2 and Fig. 27). Link lengths and the 514 mm shoulder width are
  the manual's dimensions; other proportions and the colours are by eye. No CAD
  or mesh of the arm was found to use instead.

### How this will map onto the real controller (jogging built, the rest not)

The application shell has the commands needed (ASH guide, ch.3): `joint <axis>, <deg>`
and `wx`/`wy`/`wz`/`xrot`/`yrot`/`zrot` (and `tx`.. for the tool frame) move by an
increment, `speed <percent>`, `online 1` queues up to 8 motions, `halt` stops,
`ready`/`calrdy`, and `wact` prints pulses, joint angles and world coordinates.
`joint 7, <distance>` moves the track, `home` homes the arm and then the track,
and `output <channel>, <state>` switches a digital output.
So a jog becomes a stream of small increments followed by `halt` on release.
How smooth that is depends on how fast the shell accepts commands, which has to
be measured. `armpower` can only switch arm power off from software; switching it
on needs the front-panel button.

The manual's own warning applies: "Motion using ASH can be dangerous... Use
small increments and slow speed. Be prepared to hit an e-stop."

## Connecting to the real controller

**This unit's serial port is a 25-pin D connector behind the front panel door**
(owner-observed). The C500C guide instead describes a 9-pin "Console" port on the
front; that does not match this unit. The Robcomm3 guide (ch.7, "Installing the
Cable") does describe this port, for C500 / C500B controllers: a DB-25
"Communication" port inside the front panel door, full RS-232 with control
signals, needing a **straight-through** cable with a DB-25 **male** end.

**Never connect to the rear-panel SYSIO DB-25.** It is 24 V I/O, not serial, and
the manual warns that a serial device there damages both ends.

Settings: 8 data bits, no parity, 1 stop bit, no handshake. Baud is not certain
for this unit: the manuals give 38400 for the DB-25 port generation and 57600 for
the C500C. `scripts/serial-probe scan` tries both.

World and tool jogging on the real arm is on the pad: `scripts/a465-padjog --jogd --frames
joint,world,tool`. The status page is on port 8466, and poses can be saved and replayed with
`--poses`. What was measured, and the limits found, are in
[docs/CONTROLLER.md](docs/CONTROLLER.md) ("Jogging in world and tool frames").

### The real controller: what has been measured

Measured on this unit on 2026-11-23, through a StarTech ICUSB232DB25 adapter (a Prolific
PL2303, `/dev/ttyUSB0`). Raw logs of every exchange are in `sessions/`.

| | |
|---|---|
| Line | 38400 baud, 8N1, no handshake. One Enter returns the `$` system prompt. |
| System | `crosver`: "CROS on a C500", version 1.16.471. |
| Configuration | `/conf/robot.cfg`: no track, 6 axes, metric, air gripper, no tool or base offset. |
| Geometry | With every joint at 0 the tool point is at Z = 1041.4 mm, so the links are 330.2 / 304.8 / 330.2 / 76.2 mm. |
| **Stop** | **Ctrl-Z** ("^C equivalent - halting robot...") stops a move in progress, in both shells. Ctrl-C does not, and this `ash` has no `halt` command. |
| Application shell | `ash test` answers in about 0.25 s. With `online 1`, `joint <axis>,<degrees>` returns at once and the move runs by itself. Do not send `online 1` when it is already on: the shell hangs. |
| System shell | Every command is a program loaded from flash, about 1.2 s each. `joint 1 40 &` runs a move in the background. |
| Status words | `axst` prints one word per axis (stop it with Ctrl-E). The bits are those of RAPL-3 `axis_status`: bit 0 home switch, bit 3 limp, bit 11 homed, bit 12 calibrated, bit 15 done. Running it in the background makes every other command about 1.5 s slower. |
| Control of the arm | Only one program has it. If the program holding it ends badly (for example the shell is left while the controller is still resetting after a stop), the next one is refused until someone presses **Pause/Continue** on the front panel. |
| Serial ports | The console is `/dev/sio1` (38400, software flow control off: `siocfg -c 1` prints the settings and changes nothing when given no other option). `/dev/sio0` is a second serial port set to 19200; its connector on the chassis has not been identified. |
| Shell quirks | No `type`; `more` pages with "Hit <space> or <q>" and drops unprintable bytes; `exit` in `ash` asks for `yes`; long listings take over 3 s. |

### Homing that works

Measured on the real arm on 2026-11-24, with the owner at the cell. All six joints were homed with
the index step alone (the `homezc` program, below), one at a time, and each was then returned to
zero. The owner checked the marks and said "looks spot on". Joint 6, whose mark the owner calls
spot on, came in at an offset of 251 against an ideal of 250.

**What the built-in `home` does.** Two steps per joint. The first prints "Seeking axis N": a move
that stops when status bit 0 changes. The library's table of seek distances for this arm is +100000,
+50000, +75000, -50000, +30000 and +2000000 counts (joints 1 to 6). The second step is the index
step. On this arm bit 0 reads 1 on all six joints at every position tried, so the seek step runs
its whole distance into the hard stops. Whether the seek step runs at all is decided by the product
code the robot server reads from the hardware at start-up, not by any file. The owner's position is
that this arm has no home switches; the code and the measurements are described here without arguing
that point.

**The index step alone** is the library call 0x4a (`homezc`), robot server message 29 with mode 1.
It moves the joint in the + counts direction to the NEXT encoder index pulse and sets the joint's
counter to that joint's home point from record 1 of `/conf/robot.cal`. It writes no file. On success
the controller prints `Homing axis N...Ok.`; the offset-printing version of the program then prints
`zero-cross offset: K`, where K is the number of encoder counts the joint travelled to reach the
index pulse. (`a465/crosprog.py` `homezc_offset_program` makes that version from the plain
re-stamped program; it is the 10-word, 22-byte in-place patch first made by hand that day.)

Because it takes the NEXT index pulse, the joint has to be parked half a motor turn (in counts)
before its home point first. Facts per joint:

| Joint | Home point (counts) | Counts per degree (signed) | Counts per motor turn | Parking point = home - half a turn (counts) | Offset measured 2026-11-24 |
|---|---|---|---|---|---|
| 1 | 47145 | +277.78 | 1000 | 46645 | 381 |
| 2 | 676 | +277.78 | 1000 | 176 | 502 |
| 3 | 16701 | -277.78 | 1000 | 16201 | 258 |
| 4 | -24395 | -140.28 | 500 | -24645 | 132 |
| 5 | -3106 | -138.89 | 500 | -3356 | 437 |
| 6 | 25133 | -140.28 | 500 | 24883 | 251 |

Degrees = counts / counts per degree, so joint 6's home point is -179.16 degrees and joint 3's is
-60.12. The ideal offset is half a turn (500 or 250); the difference from the ideal is how far the
joint's reading was from the truth before homing (derived from what the offset is, not measured
separately). After the index step the joint coasts 12 to 25 counts past the home point.

Counts per motor turn are 1000 on joints 1 to 3 and 500 on joints 4 to 6 (measured: a second index
step on joint 6 started 18 counts past an index and reported 482).

**Procedure.** The precondition is that each joint's present reading is already within less than
half a motor turn (1.8 degrees) of the truth: the arm was put on its zero marks and zeroed
(`scripts/a465-xzero N` runs `/diag/xzero N`), or it is already homed. Nothing can check this; the
offsets tell afterwards.

```sh
scripts/a465-home --plan     # no motion: checks /conf/robot.cal against the local copy, reads w2, prints the moves
scripts/a465-home            # joints 2 4 5 3 1 6, each returned to zero before the next starts
scripts/a465-home 6 --stay   # one joint, left where the index step put it
```

Done by hand it is, per joint: `scripts/a465-joint N <degrees>` to the parking point (the controller's
`joint <n> <degrees>` program, relative move), then `scripts/a465-homezc N --remote /tmp/homezo
--local sessions/controller-files/patched/homezc-offset`, then `scripts/a465-joint N <degrees>` back
to 0.000. `scripts/a465-home` does exactly that and stops everything if the joint is not within 40
counts of the parking point, if the homing program does not print `Ok.` and an offset, or if a
button request shows up at a motion step. An offset within a quarter turn of the ideal is
"centred"; otherwise it is "near the edge": the starting reading was off by (offset - ideal) counts
and the neighbouring index is one motor turn (3.6 degrees) away, so that joint should be checked by
eye. Exit codes: 0 all centred, 3 all ran but one or more near the edge, 1 a failure, 2 nothing started.

**Joint 5.** Its offset was 437 against an ideal of 250, near the edge: its hand-set zero was either
1.3 degrees off (the right index) or 2.3 degrees off the other way (one index out). The owner's look
at the arm afterwards found nothing wrong.

**The calibration file was not changed.** `/conf/robot.cal` is intact: `cksum` printed
`0x00000e00 72 /conf/robot.cal` before and after. Record 1 holds the home points. `calzc N` (mode 0)
overwrites joint N's home point in `/conf/robot.cal` immediately when it succeeds; `/tmp/calzc` is on
the controller and must not be run unless the owner decides to recalibrate. `homezc` writes no file.

**Other programs at the `$` prompt.** `/diag/xzero N` zeros one joint's counter (no motion);
`joint <n> <deg>`, `motor`, `ready`, `speed` and `w2` exist as programs. `w2` prints a PULSES row
(counts), a second line for axes 7 and 8, then a JOINTS row in degrees. `robotsrv` and `robotlib.r`
are RAPL-3 bytecode; `python3 -m a465.r3dis` lists them.

**Not yet known:** whether the readings survive a controller power cycle well enough to re-home
without putting the arm on its marks first (the counters restart at zero wherever the arm is), and
whether `/tmp` on the controller survives a power cycle.

**Homing by the built-in `home` does not work.** A home from the pendant drives the waist positive at about
11 degrees a second until it stalls on its end stop, and the controller reports "Runaway error on
axis 1" and drops arm power.

**The owner's position, which the project now works on: this arm has no home switches. Homing is
by encoder index pulse.** No home-switch signal has ever been seen on any joint at any position
(bit 0 of the status word reads 1 on all six joints at every position tried, including slow sweeps
across the stored points of the waist, shoulder and wrist pitch). The built-in `home` and
`/diag/calseek` wait for that switch, so on this arm they run past the stored points into the hard
stops (measured 2026-11-23: a home from the pendant, waist towards +170, "Runaway error on axis 1";
`/diag/calseek 6`, tool roll towards -179, "Collision error on axis 6"). The calibration file is
intact, though: copied byte for byte and decoded, it holds a home point and a second number for all
six joints (see [docs/CONTROLLER.md](docs/CONTROLLER.md)). Nothing was erased, and it has not been
written since a bulk load on a Dec 13 of an unrecorded year (see [docs/TIMELINE.md](docs/TIMELINE.md)).

**The old cell configuration does not matter for homing.** `/diag/setup` was run again on
2026-11-24 with the old cell's answers (2 extra axes, track, servo gripper) but cut off by our
script before it saved. The settings were applied live (the controller listed 8 axes), and
`/conf/robot.cfg` was not rewritten. The six joints' status words did not change at all (8c07;
axes 7 and 8 read 8808), so that configuration has nothing to do with homing and the idea of
restoring it was dropped. `setup` applies settings as it goes and saves only at the end; killing
it leaves the robot's point of control held, so the next program that takes control asks for
PAUSE/CONTINUE. The live 8-axis state lasts until power-off.

**Homing on the index pulse: `homezc`.** The vendor reference describes `homezc(int axis, var int
offset)` as "Homes the axis specified, and returns the offset in pulses" (its example prints 519)
and `calzc(int axis, var int offset)` as "Calibrates at the next zero pulse of the encoder". The six
numbers in the second record of `/conf/robot.cal` (700, 422, 530, 411, 500, 455) are of that size;
that they are these offsets is likely but not proven. The controller's own help says `/diag/cal`
"will overwrite the existing calibration file (/conf/robot.cal)" and that `/diag/zero` and
`/diag/xzero` are "used in preparation for calibrating the robot".

The library on this controller has `homezc` (call 0x4a) and `calzc` (0x49) but no program that
calls them. `/bin/home` and `/diag/cal` are the same 732-byte program apart from the routine they
call. The 2001 firmware's `home`, re-stamped with this controller's library stamps and build time
(file format: the docstring of `a465/crosprog.py`), is byte-identical to this controller's `home`.
Its `homezc` and `calzc`, re-stamped the same way, were uploaded to `/tmp/homezc` and `/tmp/calzc`
and read back identical. `homezc` has been run since (see Homing that works); `calzc` has not and must not be. This firmware refuses gzip-compressed programs
("not an executable"): `/app/test/fib` and `sieve` on the controller are gzip files of 2001 builds
and do not run.

**File upload works.** `/sbin/fastacid` sits between the serial line and the shell (it opens
`/dev/console` and the pipes `/dev/cin` and `/dev/cout`) and carries the vendor's file-transfer
protocol; the protocol is in the docstring of `a465/aci.py`. First live transfers, 2026-11-24: a
5-byte text file and the two 640-byte programs, each acknowledged packet by packet and verified with
the controller's `cksum`; the programs were also read back byte for byte. Typing a file in with
`cp /dev/console <file>` does not work: fastacid takes about three quarters of the typed bytes and
`cp` writes nothing when it is stopped.

**Answered by the runs of 2026-11-24:** the robot server does act on `homezc`, and the joint moves in
the + counts direction to the next index pulse (see Homing that works). `scripts/a465-homezc` is the
supervised single-joint run; `scripts/a465-home` does all six.

**`/diag/zero` makes the controller treat the arm as homed.** Measured 2026-11-23: before it,
`arm_status` said "Robot is not homed" and every status word was 8407; after `/diag/zero` at the
straight-up pose it says "Robot is calibrated and homed", every joint shows homed, the status
words are 8c07 (bit 11 set), and the controller refuses moves past a joint's range ("Joint 4
limit exceeded"). So zeroing at the true zero pose is a working way to home this arm today:
jog to the pose (waist, shoulder and tool roll on their arrows, the rest by eye), run
`/diag/zero`, and the controller's straight-line moves, `ready` and stored locations should be
open. It lasts until the controller is power-cycled, and it is only as accurate as the pose.

It also explains the owner's account of the fault. A `zero` run with the arm anywhere else makes
the controller believe it is homed with the wrong angles: the pendant shows HOMED, limits and
moves are wrong, and nothing is written to any file. A power cycle clears it. That is not lasting
damage. What does not work is the built-in search (the seek step); the index step does.

The firmware's text names two variants, A465 and A465B; this unit runs as A465B. Earlier notes
here read that as a mismatch. With the history above it is just this arm's name in the firmware.

Tools:

```sh
scripts/a465-record                  # log position and status while someone else moves the arm (display commands only)
scripts/a465-padjog                  # jog the REAL arm with the PlayStation controller, one joint at a time
scripts/a465-getfile /conf/robot.cal # copy files off the controller byte for byte, checked against its checksum
scripts/a465-calinfo                 # print what the calibration file says
scripts/a465-putfile local /tmp/x    # put a file on the controller (fastacid's packets), checked with the controller's cksum
scripts/a465-homezc --check          # no motion: is /tmp/homezc there, and does the controller load it?
scripts/a465-homezc 6                # supervised run: index-home joint 6 (stand at the E-stop)
scripts/a465-home --plan             # no motion: what a six-joint homing would do
scripts/a465-home                    # home joints 2 4 5 3 1 6 and return each to zero (see Homing that works)
scripts/a465-joint 6 -10             # move one joint by degrees with the controller's `joint` program
scripts/a465-xzero 6                 # zero one joint's counter (/diag/xzero), no motion
python3 -m a465.r3dis --data --syslib <library> <program>   # disassemble one of the controller's compiled programs
```

`a465-padjog` uses `a465/crs.py`. A jog is a relative move of about a second and a half of travel,
topped up while the stick is held and ended with Ctrl-Z on release, on a change of joint, or when the
controller pad goes silent. It is joint mode only, speed is held to 25%, and nothing limits a joint
short of its hard stop while the arm is not homed. `tests/test_crs.py` runs the module against a
stand-in for the console (`tests/fake_c500.py`), including the case of a move typed just before a stop.

With L2 pulled at least half way the jog is a fine step jog instead: one `joint <axis>,0.5` move at a
time, the next sent only once the last has answered and finished and not sooner than 0.25 s after it.
Letting go of the stick or changing joint sends no Ctrl-Z, so the controller's two-second lock-out never
starts and small corrections can follow one another at once; only the Square stop button writes Ctrl-Z.

### Motion: what was measured

Jogging with the controller's manual-move calls (the pendant's routine) fixes the speed when a jog
starts, and manual mode must be armed before every start or a reversal sweeps the joint fast through
its starting point (measured 2026-11-24). Moves issued back to back by a program do not run through:
each ends before the next begins, so six 2 degree joint moves averaged 4.4 deg/s against a cruise of
8.6 deg/s. Only `cpath` runs through its points, at one speed set before it starts (measured
2026-11-25, J6, with `scripts/a465-segtest`). So none of the controller's motion calls gives a speed
that changes while the joint keeps moving. The numbers and the call sequences are in
`docs/CONTROLLER.md`, "Motion: what was measured".

### Making a cable from a Cisco USB console cable and an RJ45-to-DB25 adapter

A straight-through cable to a PC means the controller port expects the PC's own
pinout on the plug. Standard RS-232 positions on a DB-25:

| Signal (PC side) | DB-25 pin | Needed |
|---|---|---|
| TXD (PC sends) | 2 | yes |
| RXD (PC receives) | 3 | yes |
| Ground | 7 | yes |
| RTS / CTS | 4 / 5 | optional |
| DSR / DTR | 6 / 20 | optional |

Only the first three matter with no handshake. Which RJ45 wire carries which
signal varies by cable and by adapter wire colour, so identify them rather than
trust a table:

1. Plug the USB cable into the dev box and the RJ45 into the adapter, pins still loose.
2. `scripts/serial-probe wires`. All outputs start low.
3. Ground: on Cisco console cables RJ45 pins 4 and 5 are both ground, so the two
   loose pins that show continuity to each other are the ground pair.
4. Press `t`. The one wire that swings positive against ground (a few volts up
   to +12) is **TXD**. Press `t` again to lower it.
5. Touch the TXD wire to each remaining wire. When the screen shows
   `RXD:hears TXD`, that wire is **RXD**.
6. Insert TXD into DB-25 position 2, RXD into 3, a ground into 7.

Before plugging into the controller, meter its port with the controller on:
against pin 7, pin 3 should sit at a steady negative voltage (the controller's
idle transmit line). Anything near 24 V means it is not the serial port.

Then:

```sh
scripts/serial-probe scan            # one Enter at 38400, 57600, 19200, 9600; shows which answers
scripts/a465-term --baud <rate>      # terminal; exit with Ctrl-]
```

Each terminal run writes what appeared on screen to `sessions/<timestamp>.log`.
The Enter key sends CR and local echo is off; if typing is invisible use `--echo`,
if Enter does nothing try `--eol crlf`.

### What to expect (from the manuals, for a C500C)

- After power-up the terminal shows a `$` prompt (system shell) or `>>`.
  `ash <name>` starts the application shell.
- If a teach pendant is attached it takes control at start-up; hand control to
  the terminal from the pendant.
- Before applying arm power, check encoder feedback with arm power **off**
  (A465 guide p.55): type `w1`, push each joint by hand, confirm the counts go
  both up and down, then `Ctrl+E`. An axis without feedback can run away.

### What a plain terminal cannot do

Per the Robcomm3 guide: file transfer to the controller is only possible with
Robcomm3 (Windows software), and RAPL-3 programs are compiled on the PC. The
controller does have an on-board editor (UMI-R3-166).

## Layout

| Path | Contents |
|---|---|
| `a465/model.py` | Arm geometry, limits, forward kinematics, tool-down inverse kinematics |
| `a465/planner.py` | Pick-and-place cells, reach map, and the pre-computed, checked cycle |
| `a465/sim.py` | The mock robot |
| `a465/gamepad.py` | PS4 controller to jog commands |
| `a465/crs.py` | The real controller over its serial console: joint jog, stop, position and status |
| `a465/server.py`, `a465/web/` | REST + WebSocket server and the web page |
| `tests/` | `python3 -m pytest` |
| `scripts/a465-term` | Serial terminal with session logging |
| `scripts/a465-record` | Log the controller's position and status while the arm is moved by someone else |
| `scripts/a465-padjog` | Jog the real arm with the PS4 controller |
| `scripts/a465-getfile` | Copy files off the controller, verified |
| `scripts/a465-calinfo`, `a465/calfile.py` | Read the calibration file |
| `scripts/a465-putfile`, `a465/aci.py` | Put a file on the controller (fastacid's file-transfer protocol) |
| `scripts/a465-homezc`, `a465/homezc.py`, `a465/console.py` | Supervised run of the index-pulse homing program; reading status words and joint angles from the `$` shell |
| `scripts/a465-home`, `scripts/a465-joint`, `scripts/a465-xzero`, `a465/home.py` | Six-joint index homing; single-joint move; zero one counter |
| `a465/crosprog.py`, `a465/r3dis.py` | The controller's compiled-program file format; a disassembler for them |
| `controller/` | Verified copies of the controller's own files (calibration, configuration, taught locations) |
| `docs/CONTROLLER.md`, `docs/TIMELINE.md` | File copy, the calibration file's format, and a dated record of what changed on the controller |
| `scripts/serial-probe` | Identify adapter wires; find the baud rate |
| `scripts/pair-gamepad` | Pair a PS4/PS5 controller over Bluetooth |
| `udev/70-a465-gamepad.rules` | Lets the server read the controller's motion-sensor node (signal watchdog) |
| `scripts/fetch-manuals.sh` | Re-download and verify the manuals, extract text |
| `docs/` | Manual list and sources. PDFs and text are not committed |
| `sessions/`, `programs/` | Terminal logs; robot programs |

Search the manuals with e.g. `grep -n -i 'siocfg' docs/text/*.txt`.
