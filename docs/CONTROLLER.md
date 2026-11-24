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
- It is read-only. **There is still no way to put a file onto the controller.**
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
| 2 | 02 or 04 | record type: 2 = home points, 4 = switch-to-index offsets |
| 3 | 3d, 1f | a check byte; how it is computed is not known |

This arm's values:

| Joint | Home point (counts) | Home point (degrees) | Manual's switch position | Switch to index (counts) |
|---|---|---|---|---|
| 1 waist | 47145 | +169.72 | +175 | 700 |
| 2 shoulder | 676 | +2.43 | +10 | 422 |
| 3 elbow | 16701 | -60.41 | -60 | 530 |
| 4 wrist rotate | -24395 | +173.90 | +180 | 411 |
| 5 wrist pitch | -3106 | +22.36 | +25 | 500 |
| 6 tool roll | 25133 | -179.16 | 180 | 455 |
| 7, 8 | 0 | | | 0 |

Degrees use the counts per degree measured on this arm (waist and shoulder 277.78, elbow
-276.48, wrist rotate -140.28, wrist pitch -138.89, tool roll -140.28) and the controller's
signs. The home point is where the index pulse that homing stops on sits, measured from true
zero. The second record is how far that index pulse is from the joint's home switch.

What this says about homing: the controller drives each joint towards its home point, waits for
the home switch, then takes the next index pulse and sets the joint's position from the first
record. Both failed attempts on 2026-11-23 headed for these points (waist positive towards
+170, tool roll negative towards -179) and ran past them into the hard stops, because the
switch signal never changed. The file is intact; the missing piece is that signal.

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
- `/diag/zero` changes no file. `/diag/cal` overwrites `robot.cal` if it completes;
  `/conf/robot.cal.bak` is a copy made on 2026-11-23.
- `odometer` counts seconds of running, kept in `/log/robot.odo` as the same 32-bit number twice.
