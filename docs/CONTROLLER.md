# Working with this controller's files

Measured on this unit (C500, serial RC6044, CROS 1.16.471) on 2026-11-23.

## Copying files off, byte for byte

The shell cannot send files anywhere, and proper file transfer needs Robcomm3. But

    cp <file> /dev/console

prints a file's raw bytes on the console line, and `cksum <file>` prints its size and a plain sum
of its bytes. `scripts/a465-getfile` does both and saves a file only when size and sum match:

    scripts/a465-getfile /conf/robot.cal /conf/robot.cfg
    scripts/a465-getfile --out controller/backup-2026-11-23 /app/caco/caco2.v3

- Nothing is altered on the way: about 30 files, from 8 bytes to 43 KB, all matched.
- About 3800 bytes a second. `cksum` on a 30 KB file takes several seconds before it answers.
- It must be run at the `$` prompt with nothing else on the port (stop `scripts/a465-padjog`).
- It is read-only. Putting files on is a separate tool, `scripts/a465-putfile` (below).
- Command lines longer than about 80 characters are cut off by the shell.
- `more <binary>` shows only a binary's readable text; useful for a quick look.

`controller/backup-2026-11-23/` holds verified copies of everything in `/conf`, the application
data in `/app`, the logs' odometer file and a full dated listing. The firmware's own programs
that were copied for study are under `sessions/controller-files/` and are not meant for the repo.

## The calibration file, `/conf/robot.cal`

72 bytes: two 36-byte records, each a four-byte header and eight little-endian 32-bit signed
values, one per axis. `a465/calfile.py` reads it; `scripts/a465-calinfo` prints it.

| Header byte | Value here | Meaning |
|---|---|---|
| 0 | 02 | not known |
| 1 | 09 | machine type (the code this arm's stored locations also carry) |
| 2 | 02 or 04 | record type: 2 = home points, 4 = seek-to-index distances |
| 3 | 3d, 1f | a check byte; how it is computed is not known |

This arm's values:

| Joint | Home point (counts) | Home point (degrees) | Counts per motor turn | Second record (counts) |
|---|---|---|---|---|
| 1 waist | 47145 | +169.72 | 1000 | 700 |
| 2 shoulder | 676 | +2.43 | 1000 | 422 |
| 3 elbow | 16701 | -60.12 | 1000 | 530 |
| 4 wrist rotate | -24395 | +173.90 | 500 | 411 |
| 5 wrist pitch | -3106 | +22.36 | 500 | 500 |
| 6 tool roll | 25133 | -179.16 | 500 | 455 |
| 7, 8 | 0 | | | 0 |

Degrees use the counts per degree the controller itself shows in `w2` (waist and shoulder 277.78,
elbow -277.78, wrist rotate -140.28, wrist pitch -138.89, tool roll -140.28). The home point is the
value the index step writes into the joint's counter at the index pulse (measured on all six joints
on 2026-11-24). The second record is used only by the built-in `home`: after its seek step it moves
the joint by this number minus half a motor turn, and then runs the index step (read from
`robotlib.r`). The half turn it uses comes from the robot server at run time; the 500 counts per
turn on joints 4 to 6 was measured (joint 6), the 1000 on joints 1 to 3 follows from joint 2's
offset of 502 from a 503-count park.

What this says about homing: the built-in `home` cannot be used on this arm, because its seek step
never ends (it moves until status bit 0 changes, and that bit reads 1 on every joint at every
position; both attempts on 2026-11-23 ran into the hard stops). The index step on its own works:
see "Homing that works". The owner's position is that this arm has no home switches and is homed by
its encoder index pulse. The file is intact and was not changed by the homing on 2026-11-24.

The vendor reference describes `homezc(int axis, var int offset)` as "Homes the axis specified, and
returns the offset in pulses" (its example prints 519) and `calzc(int axis, var int offset)` as
"Calibrates at the next zero pulse of the encoder". Measured: the offset `homezc` returns is the
distance the joint travelled to reach the index pulse. `calzc` rewrites this file when it succeeds.
The controller's own help says `/diag/cal` "will overwrite the existing calibration file
(/conf/robot.cal)" and that `/diag/zero` and `/diag/xzero` are "used in preparation for calibrating
the robot".

The file was last written on a Dec 13 (controller clock, year not recorded), in the same minute
as three startup files. See [TIMELINE.md](TIMELINE.md).

## Times stored inside files

- **Compiled programs** (first four bytes `1f ab ad 10`): bytes 8 to 11 are the build time,
  seconds since 1970, little-endian. This firmware: 1999-06-30 21:18.
- **Variable files `.v3`** (first four bytes `ae 10 1a da`): bytes 8 to 11 are a time from the
  controller's clock, apparently when the file was created; it survives later saves. The name
  of the application follows a little further on.

## The controller's clock

On 2026-11-23 it read Apr 22 2022: 1676 days 11 h 47 min behind, at the right rate. `date` with
arguments sets it (`date yyyy mo dd hh mi ss` per the manual). It has been left alone so far so
that old file dates stay comparable; set it only after noting the error at that moment.

## Other things worth knowing

- Opening an application's variable file in `ash` and leaving re-saves it, which changes its
  date. Copy files with `a465-getfile` first if their dates matter.
- `/diag/zero` changes no file, but it does more than clear the counters: it marks every joint
  homed and calibrated at the pose the arm is in (status bit 11 set, `arm_status` says "Robot is
  calibrated and homed", joint limits enforced). Run it only at the true zero pose. A power cycle
  undoes it. `/diag/cal` overwrites `robot.cal` if it completes (it is the same 732-byte program as `/bin/home`, apart from the routine it calls);
  `/conf/robot.cal.bak` is a copy made on 2026-11-23.
- `odometer` counts seconds of running, kept in `/log/robot.odo` as the same 32-bit number twice.

## Putting files on the controller

`/sbin/fastacid` sits between the serial line and the shell (it opens `/dev/console` and the pipes
`/dev/cin` and `/dev/cout`) and carries the vendor's file-transfer protocol. The protocol is in the
docstring of `a465/aci.py`; `scripts/a465-putfile local /tmp/name` uses it and then checks the copy
with the controller's `cksum`.

- First live transfers, 2026-11-24: a 5-byte text file and two 640-byte programs, each acknowledged
  packet by packet and verified with `cksum`; the programs were also read back byte for byte.
- Typing a file in with `cp /dev/console <file>` does not work: fastacid takes about three quarters
  of the typed bytes, and `cp` writes nothing when it is stopped.
- Not known: how larger files behave; only files up to 640 bytes have been sent.

## Homing that works

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

Joint 2's 502 is from its second run, when it was already homed and parked 503 counts before its
index; its first run was made before the offset was printed. Degrees = counts / counts per degree,
so joint 6's home point is -179.16 degrees and joint 3's is -60.12. The ideal offset is half a turn (500 or 250); the difference from the ideal is how far the
joint's reading was from the truth before homing (derived from what the offset is, not measured
separately). After the index step the joint coasts 12 to 25 counts past the home point.

Counts per motor turn are 1000 on joints 1 to 3 and 500 on joints 4 to 6 (measured: a second index
step on joint 6 started 18 counts past an index and reported 482).

**After a power cycle (measured 2026-11-24, evening).** The arm was left at zero, the controller was
switched off and on, and `scripts/a465-home --time-limit 30` homed all six joints in one run:

| Joint | Offset after the power cycle | Ideal | Reading was off by |
|---|---|---|---|
| 1 | 511 | 500 | 0.04 degrees |
| 2 | 499 | 500 | 0.00 |
| 3 | 505 | 500 | 0.02 |
| 4 | 253 | 250 | 0.02 |
| 5 | 230 | 250 | 0.14 |
| 6 | 222 | 250 | 0.20 |

What the power cycle showed:

- The counters restart at zero where the arm is when the controller boots, and the arm held its
  position with the power off to within 0.2 degrees. Joint 5 read +3.78 degrees and joint 6 +0.20 by
  the time arm power was on; that movement happened after the controller had started counting and
  was counted correctly (joint 5's offset came back at 230).
- The boot messages report 6 axes, "Robot calibration data read" and "Robot is not homed"; the
  status words read 8407 (bit 11 clear). `/tmp` keeps its files through a power cycle. The boot also
  prints "WARNING: system was not properly shut down -- repairing." and checks its file system.
- The teach pendant's program starts at boot and holds control of the arm: `speed 10` answered
  "speed: resource busy" until the owner released control on the pendant.
- In the "not homed" state the index step is about six times slower: 7 to 10 s per joint, against
  about 1.5 s when the controller already counted the arm as homed (after `/diag/zero`). The first
  attempt was cut off by the 8 s time limit with joint 2 at 534 of the 676 counts; hence 30 s.

So the routine after switching on is: release the pendant, then `scripts/a465-home`. Leave the arm
at zero before switching off.

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

## Compiled programs and the homing programs

The program file format (header, check value, library stamps) is in the docstring of
`a465/crosprog.py`; `python3 -m a465.r3dis` disassembles a program.

- The library on this controller has `homezc` (call 0x4a) and `calzc` (0x49) but no program that
  calls them.
- The 2001 firmware's `home`, re-stamped with this controller's library stamps and build time, is
  byte-identical to this controller's `home`. Its `homezc` and `calzc`, re-stamped the same way, were
  uploaded to `/tmp/homezc` and `/tmp/calzc` and read back identical. `homezc` has been run since
  (see "Homing that works"); `calzc` has not and must not be, as it overwrites the calibration file.
  Local copies are in `sessions/controller-files/patched/` (git-ignored).
- `homezc <axis>` is run with `scripts/a465-homezc` (`--check` first: no motion; then one joint
  with a time limit, the operator at the E-stop). With anything other than one argument it prints
  `<name>: invalid command line arguments.` and `Usage: <name> [newspeed]` (the usage text is a
  leftover); with an axis outside 1 to 8, `<name>: invalid axis '<arg>'`; otherwise it calls the
  library's `homezc(axis, offset)`, prints `<name>: <error text>` on failure, and prints nothing
  itself on success (not even the offset); the controller prints `Homing axis N...Ok.`. The patched
  `homezc-offset` also prints `zero-cross offset: K`.
- Answered by the runs of 2026-11-24: the robot server does act on `homezc`; the joint moves in the
  + counts direction to the next index pulse; see "Homing that works". Not recorded here: whether it
  honours `speed` beyond the speed it was run at.
- This firmware refuses gzip-compressed programs ("not an executable"): `/app/test/fib` and `sieve`
  on the controller are gzip files of 2001 builds and do not run.

## Motion: what was measured

Measured on this arm (CRS A465, C500, CROS 1.16.471).

### Jogging with the manual-move calls

Measured 2026-11-24. These are the calls the teach pendant's routine uses.

| Library call | Number | Meaning |
|---|---|---|
| `man_mode_set(mode)` | robotlib 0x52 | 0 off, 1 joint, 2 cylindrical, 3 world, 4 tool |
| `man_move_start(joint, speed)` | 0x53 | signed joint number, speed in percent |
| `man_move_stop(joint)` | 0x54 | |
| `halt` | 0x21 | |
| `speed_set` | 0x44 | |

- Each library call takes about 0.1 s.
- The speed is fixed when a jog starts. A second `man_move_start` or a `speed_set` on a joint that is
  already moving returns no error and changes nothing (3% then 8% stayed at 2.2 deg/s on J6). "No error"
  is not evidence that a call did anything; only a position measurement is.
- Manual mode must be armed before every start, as the pendant does: [`speed_set` if the speed changed]
  -> `man_mode_set(mode)` -> `man_move_start`. Stop: `man_move_stop` -> `halt` if that failed ->
  `man_mode_set(0)`. With manual mode armed only once, reversing direction swept the joint fast through
  its starting point to the mirror-image angle (J6 at 5%: peaks of 24 and 47 deg/s). With the per-jog
  sequence, all four moves of the reversal test peaked at 3.65 to 4.19 deg/s.
- A start is acknowledged 250-300 ms after the command; a stop takes about 200 ms. J6 runs about
  0.73 deg/s per percent in manual mode.
- Ctrl-Z (0x1a) on the console stops the arm and also kills the program that holds control: the
  controller prints `^C equivalent - halting robot...` and `[NNN killed by SIGABRT.]`. Control stays
  held; the next program that takes control prints "Press the PAUSE/CONTINUE button to gain robot point
  of control." So a PC-side driver must not use Ctrl-Z as its normal stop.
- From reading the controller's own programs (`robotsrv`, `robotlib`): no message to the servo board
  updates the speed or target of a move that is already running. Each call costs one servo-board reply
  plus the server's polling (a 20 ms delay and a 21 ms poll).
- `controller/programs/jogd` is our own jog program for the controller; its console protocol is
  described in `a465/jogprog.py`.

### Back-to-back moves and paths

Measured 2026-11-25 on J6 with `scripts/a465-segtest`, online mode on.

| What was run | Result |
|---|---|
| One joint move, 12 deg at 5% | cruises at 8.6 deg/s, about 0.1 s ramps, 1.5 s in all |
| One joint move, 6 deg at 15% | reached at least 23 deg/s |
| Six joint moves of 2 deg (5% then 15%) | 2.6 s for the 12 deg, mean 4.4 deg/s, half of cruise: the joint slows or stops at every join. The first call returns in 65 ms; each later call blocks 277-353 ms, until the move before it has ended |
| Six `move()` calls to locations 2 deg apart | the same: mean 4.4 deg/s, calls block 260-352 ms |
| Six motor moves of 281 pulses (2 deg) | stop and go; each call blocks about 1.75 s at 5% and 0.53 s at 15%; a `speed_set` between moves applies to the next move issued |
| `cpath` through six points 2 deg apart at 5% | runs through the points without slowing (steady 2.8 deg/s across four joins, sampled every 0.1-0.2 s) |
| The same with `speed_set(15)` sent right after `cpath` returned | no change: still 2.8 deg/s to the end |
| `cpath` at 15% | 8.7 deg/s: the path takes its speed from the speed setting when it is sent |
| `cpath` through points 1 deg apart at 5% | 2.8 deg/s again: spacing of the points does not change the speed |
| How long `cpath` holds the program | 2343 ms (5%, 2 deg spacing), 1802 ms (5%, 1 deg), 1113 ms (15%, 2 deg); the joint was about at the first point when it returned |

What follows from these measurements: moves issued one after another by a program do not run through;
each ends before the next begins. Only `cpath` runs through its points, and it holds one speed, set
before it starts, and needs 1-2 s to load six points. So none of the controller's motion calls gives a
speed that changes while the joint keeps moving. In no run did any joint other than J6 move.

## `/diag/setup` and the old cell configuration

`/diag/setup` was run again on 2026-11-24 with the old cell's answers (2 extra axes, track, servo
gripper) but cut off by our script before it saved. The settings were applied live (the controller
listed 8 axes); `/conf/robot.cfg` was not rewritten. The six joints' status words did not change at
all (8c07; axes 7 and 8 read 8808), so that configuration has nothing to do with homing, and the plan
to restore it was dropped. `setup` applies settings as it goes and saves only at the end. Killing it
leaves the robot's point of control held, so the next program that takes control asks for
PAUSE/CONTINUE. The live 8-axis state lasts until power-off.

Measured the same day: with control held like that, `speed 10` at the `$` prompt printed "Press the
PAUSE/CONTINUE button to gain robot point of control." and waited (`speed` alone only prints "Current
Speed: 10% of max."). The controller prints nothing when the button is then pressed. So
`scripts/a465-homezc` does its waiting for the button at its `speed` step, where nothing can move, and
kills the homing program at once if the request appears at the motion step, so that a move never runs
without its time limit.

## Serial ports

The console is `/dev/sio1` (38400, software flow control off: `siocfg -c 1` prints the settings and
changes nothing when given no other option). `/dev/sio0` is a second serial port set to 19200; its
connector on the chassis has not been identified.
