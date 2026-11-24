# crs-a465

Control stack, notes, manuals and terminal sessions for a CRS A465 six-axis arm on
a C500C controller (T475 configuration: extra axis outputs for a track and a
carousel), worked from a Linux dev box.

## Status (2026-11-23)

- **Web interface + REST API + PS4 controller: working against a mock robot.**
  The mock uses the real arm's dimensions, joint ranges, speeds and accelerations.
- **The real controller answers over serial (since 2026-11-23): 38400 baud, 8N1,**
  `/dev/ttyUSB0`. Reading its state works, and so does joint jogging with a proven stop.
  It is not homed: its home search runs the waist into its end stop. See
  [The real controller: what has been measured](#the-real-controller-what-has-been-measured).
- Manuals gathered and searchable (see [docs/SOURCES.md](docs/SOURCES.md)).

## The interface

```sh
sudo apt install python3-aiohttp python3-evdev python3-numpy python3-serial
python3 -m a465.server              # http://<this machine>:8465
python3 -m a465.server --track --carousel   # start with the track and the carousel fitted
```

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
| Left stick | J1 waist (left/right), J2 shoulder (up/down) | Y (left/right), X forward/back (up/down) |
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
| Shell quirks | No `type`; `more` pages with "Hit <space> or <q>" and drops unprintable bytes; `exit` in `ash` asks for `yes`; long listings take over 3 s. |

**Homing does not work yet.** A home from the pendant drives the waist positive at about
11 degrees a second until it stalls on its end stop, and the controller reports "Runaway error on
axis 1" and drops arm power.

**Why it does not home: the home switch signals never reach the controller.** The owner homed
this arm with the built-in `home` (pendant and terminal) until about 2024. That routine drives each
joint towards a stored home point, waits for the joint's home switch, then takes the next encoder
index pulse. Measured on 2026-11-23:

- The calibration file is intact. Copied byte for byte and decoded, it holds a home point and a
  switch-to-index offset for all six joints, within a few degrees of the switch positions the
  manual gives (see [docs/CONTROLLER.md](docs/CONTROLLER.md)). Nothing was erased, and it has not
  been written since a bulk load on a Dec 13 of an unrecorded year
  (see [docs/TIMELINE.md](docs/TIMELINE.md)).
- Both attempts headed for those stored points and ran past them into the hard stops: a home
  from the pendant (waist towards +170, "Runaway error on axis 1") and `/diag/calseek 6` (tool
  roll towards -179, "Collision error on axis 6").
- Bit 0 of the status word (home switch) reads 1 on all six joints at every position tried,
  including slow sweeps across the stored points of the waist, shoulder and wrist pitch.

Six signals absent together points at something they share (the sensors' supply or their common
return), not at six wires. Not yet checked. The manual puts the six home lines in the motor power
cable (round 24-pin connector, pins 18, 19, 20, 22, 23, 24, 12 V) and says each arm cable has a
braided ground strap that must be connected; this unit's arm cables are hand-made harnesses and
the controller's ground posts were bare in the owner's photos. The owner's own view is that the
arm has no home switches and that a `cal` or `zero` command erased a stored reference; the
decoded file does not support the second part, and the first is not something that can be seen
from the console.

Meanwhile the arm is used with a manual zero: jogged to the straight-up pose (waist, shoulder and
tool roll on their arrows, the rest by eye) and `/diag/zero` run there, which makes the
controller's angles true until it is power-cycled. The controller still reports "not homed", so
its straight-line moves and stored locations stay locked.

The firmware's text names two variants, A465 and A465B; this unit runs as A465B. Earlier notes
here read that as a mismatch. With the history above it is just this arm's name in the firmware.

Tools:

```sh
scripts/a465-record                  # log position and status while someone else moves the arm (display commands only)
scripts/a465-padjog                  # jog the REAL arm with the PlayStation controller, one joint at a time
scripts/a465-getfile /conf/robot.cal # copy files off the controller byte for byte, checked against its checksum
scripts/a465-calinfo                 # print what the calibration file says
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
| `controller/` | Verified copies of the controller's own files (calibration, configuration, taught locations) |
| `docs/CONTROLLER.md`, `docs/TIMELINE.md` | File copy, the calibration file's format, and a dated record of what changed on the controller |
| `scripts/serial-probe` | Identify adapter wires; find the baud rate |
| `scripts/pair-gamepad` | Pair a PS4/PS5 controller over Bluetooth |
| `udev/70-a465-gamepad.rules` | Lets the server read the controller's motion-sensor node (signal watchdog) |
| `scripts/fetch-manuals.sh` | Re-download and verify the manuals, extract text |
| `docs/` | Manual list and sources. PDFs and text are not committed |
| `sessions/`, `programs/` | Terminal logs; robot programs |

Search the manuals with e.g. `grep -n -i 'siocfg' docs/text/*.txt`.
