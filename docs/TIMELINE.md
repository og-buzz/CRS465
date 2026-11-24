# Forensic timeline of the controller

Built 2026-11-23 from what the controller itself records. The aim was to find out when the
calibration was last written and what changed before homing stopped working.

## What the evidence is, and how far to trust it

| Source | What it gives | Trust |
|---|---|---|
| `ls -l` on the controller | Month, day and time each file was last written. **No year**, and `ls` has no option for one. Folder dates are the folder's creation: adding files does not change them. | Good for order within a year; the year has to come from elsewhere. |
| `date` | The controller's clock. | Read twice on 2026-11-23, four hours and one power cycle apart: **1676 days 11 h 47 min behind** real time both times. It runs at the right rate now. Whether that error was the same years ago is not known. |
| Build time inside each program | Compiled programs carry the time they were built (bytes 8 to 11). | Set by the maker's build machine: reliable. |
| Stamp inside each `.v3` variable file | Bytes 8 to 11 hold a time from the controller's clock. It is not rewritten on every save (see `caco2.v3` below), so it reads as when the file was created. | Reliable as a controller-clock reading. |
| `/conf/robot.cfg` | Contains its own "Written" line with the full date. | Reliable as a controller-clock reading. |
| `/log/acid.log*` | The last four power-ups, with full dates. | Reliable as controller-clock readings. |
| `odometer` | Seconds the robot has been running, kept in `/log/robot.odo`. | Reliable as a count; when it started from zero is inferred. |

Everything was read with display commands or copied with `scripts/a465-getfile` (each copy
checked against the controller's own checksum). The raw listing is in
`controller/backup-2026-11-23/listing.txt`.

Real times below are the controller time plus 1676 days 11 h 47 min. They are only given where
the controller time is recent enough for that error to be the measured one.

## Timeline

| Controller clock | Real time | What happened | Evidence |
|---|---|---|---|
| 1999-06-30 21:18 | the same | Firmware built (CROS 1.16.471). | Build time inside `/bin/home`, `/diag/cal`, `/sbin/robotsrv`, both libraries; `ls` shows every firmware file as Jun 30 21:18 or 21:19. |
| 2003-11-15 00:38 | unknown | The lab cell's locations file was created: 17 variables (carousel, incubator, Tecan, sealer, scanner, safe positions). Inside it calls itself `caco`. | Stamp inside `/app/caco/caco2.v3`. |
| Dec 13, 02:20 to 02:22, year not recorded | unknown | **The controller's file store was set up and loaded in one go**: the root folder, `rc`, `rc.sim`, `rc.simsockd`, **`robot.cal`** (02:20); `servogrp.cal`, `simsockd.cfg`, folders `/app/2` and `/app/caco` (02:21); `caco2.v3` (02:22). | `ls` dates. One minute for the calibration file and three startup files is a bulk load, not a calibration run, which takes minutes of arm motion and writes one file. |
| Dec 13, 03:50, same day | unknown | The two stock example programs were written to `/app/test`. | `ls` dates. |
| (nothing) | | **No file on the controller was written between that Dec 13 and the Apr 20 below.** | Every file is dated either Dec 13 or Apr 20 to 22. |
| 2022-04-20 08:24 | Sat Nov 21 2026, 20:11 | Variable file `2.v3` created, empty. | Stamp inside the file. |
| 2022-04-20 11:29 | Sat Nov 21, 23:15 | Variable file `caco.v3` created, empty. The old data is not in it and never was: it is in `caco2.v3`. | Stamp inside the file. |
| 2022-04-20 22:31 to 23:02 | Sun Nov 22, 10:17 to 10:48 | Application `SampleProgram` created; `ash` help used for the first time (its index file was built). | `ls` dates; stamp inside `SampleProgram.v3` (22:43). |
| 2022-04-21 04:05 to 04:08 | Sun Nov 22, 15:51 to 15:54 | Application `new _p0` created. | `ls` dates. |
| 2022-04-21 05:51:22 | Sun Nov 22, 17:38 | **`robot.cfg` rewritten**: no track, 6 axes, metric, air gripper, no tool or base offset. This is what `/diag/setup` writes. | The "Written" line inside the file. |
| 2022-04-21 08:54 | Sun Nov 22, 20:40 | System shell help used for the first time (its index file was built). | `ls` date of `/lib/shell.ndx`. |
| 2022-04-22 01:59 | Mon Nov 23, 13:45 | `test.v3` written. | `ls` date as first seen. |
| 2022-04-22 03:28, 03:31, 06:04 | Mon Nov 23, 15:14, 15:18, 17:50 | Power-ups. | `/log/acid.log.*` as first seen and now. |
| 2022-04-22 06:18 | Mon Nov 23, 18:04 | `2.v3` and `caco.v3` saved again, still empty (24 bytes each). | `ls` dates. |
| | Mon Nov 23, 18:16 | Owner's photos: pendant shows "HOMED" on its homing menu. | Photo timestamps. |
| 2022-04-22 06:41:56 | Mon Nov 23, 18:28 | Power-up. First serial contact from this project at 18:31. Odometer then: 9.13 hours. | `acid.log.0`; session logs. |
| | Mon Nov 23, 18:50 | Home from the pendant, recorded: waist driven 110 degrees into its stop, "Runaway error on axis 1". | `sessions/2026-11-23_184916-record.*`. |
| 2022-04-22 07:47:43 | Mon Nov 23, 19:34 | Power-up (after a test of mine hung the controller). | `acid.log`. |
| 2022-04-22 08:21 | Mon Nov 23, 20:07 | **Changes made by this project**: `robot.cal.bak` created (a copy of `robot.cal`); `caco2.v3` re-saved when I opened it in `ash` to list it, which moved its `ls` date from Dec 13 02:22 to this time. Its size and the 2003 stamp inside are unchanged. | Session logs; `ls`. |
| | Mon Nov 23, 20:21 | `/diag/calseek 6` test: tool roll driven 183 degrees into its stop, "Collision error on axis 6". | `sessions/2026-11-23_202131-calseek-j6.log`. |
| | Mon Nov 23, about 21:35 | `/diag/zero` at the pose the owner set as zero. Writes no file. | `sessions/zero-reference.txt`. |
| 2022-04-22 10:40 | Mon Nov 23, 22:26 | Listing and copies taken. Odometer: 12.13 hours. | `controller/backup-2026-11-23/`. |

## What it shows

1. **The calibration file was last written on a Dec 13, as part of a bulk load, and nothing has
   written it since.** Its contents are specific and plausible for all six joints (see
   [CONTROLLER.md](CONTROLLER.md)). A calibration run after that date would have re-dated it.
2. **No trace of the accident.** The owner recalls last homing it in about 2024 and then running
   "some cal or zero command". A `zero` writes nothing, so it leaves no trace and cannot break
   homing. A `cal` that completed would have re-dated `robot.cal`; one that failed part-way
   writes nothing.
3. **Which year that Dec 13 was cannot be read off the controller.** It is after 2003-11-15 on
   the controller's clock. If the clock error has always been what it is now, it was about July 16
   in real time, of 2024, 2025 or 2026 for controller years 2019, 2020 or 2021.
4. **The variable file that was "emptied" was created empty two days ago.** The lab's taught
   locations are intact in `caco2.v3`.
5. **The odometer is young.** 9.1 hours at first contact and 12.1 now. Either it was started from
   zero within the last few days of use, or the robot has run very little since the file store
   was set up.
6. **One thing did change in the last two days that matters to the machine's setup**: `robot.cfg`
   was rewritten on Nov 22 at 17:38. It does not hold anything about homing, as far as the robot
   server's own text shows (its keys are track, number of axes, tool and base offset, track
   lengths, gripper type and units).

## What it does not show

- When and how the home switch signals stopped reaching the controller. No file records them.
- Who did the Dec 13 load, and with what. Loading files needs Robcomm3 or its transfer protocol.
