# Robcomm3: what it does, and how much of it this repo has

Set as the project's goal 2026-11-25. The owner's words: "Robcomm3 equivalent functions is what we
really need to set as a /goal". Robcomm3 is the vendor's Windows program for the C500 / C500C
controller. We have no copy of it, so this file lists what the vendor's own manuals say it does,
then says where this repo stands against each function.

Sources, all local under `docs/text/`. Line numbers are lines of the `.txt` file.

- `Robocomm3.txt` (spelled with the extra "o" in the file name): the Robcomm3 part of the
  Application Development Guide, May 2000. Cited below as RC.
- `Application-Environment.txt` (AE), `Application-Development.txt` (AD),
  `ApplicationShellASH.txt` (ASH), `CROSandSystemShell.txt` (SYS),
  `C500CControllerUserGuide.txt` (UG), `A465-User-Guide-for-C500C.txt` (A465),
  `Firmware-Download-Utility-for-the-C500C-Controller.txt` (FW), `RAPL-3LanguageReferenceGuide.txt`
  (R3REF), `Track-User-Guide.txt`, `Force-Sensor-C500C.txt`.
- `sessions/controller-files/lib/ash.help`: the controller's own help for its application shell.

Where the manuals say nothing or little, this file says "not described in the manuals we have".

## 1. What Robcomm3 does

RC lines 182-190 give the one-paragraph summary: a Windows-based application for RAPL-3 whose
integrated environment lets you create and edit robot programs as source files, compile source
files into object files, transfer files between the computer and the controller, simplify compiling
and transferring with the application setup feature, and communicate interactively with the
controller using the terminal window. The table at RC lines 195-220 maps each to a menu entry.
AE lines 286-330 and the glossary in UG lines 2625-2629 say the same in fewer words.

### A. Terminal to the controller

1. **Terminal window.** Opens a window (black background, white letters) that is a command line to
   the controller's system shell (`$`) and application shell (`ash`); from it the teach pendant
   can also be started. One terminal window at a time; open it before powering the controller to
   capture the start-up text. RC lines 2297-2370 (opening, "One Window at a Time", "Capturing
   Start-up Information", "No Prompt"), 2380-2460 (what the shells and pendant are used for);
   AE lines 326-330.
2. **Serial settings.** C500 menu, COM Settings: choose the computer's serial port and the baud
   rate (57600 for a C500C, 38400 for a C500 or C500B). RC lines 565-590. UG line 1505 adds that
   Robcomm3 is limited to 57,600 baud or lower. "Junk characters" in the terminal mean a baud mismatch
   (RC lines 2362-2366).
3. **Other terminals.** The manuals say a plain terminal emulator or serial terminal also reaches
   the shells, but file transfer, editing and compiling are possible only with Robcomm3 (RC lines
   2468-2496).

### B. Sending and receiving files

4. **File Transfer dialog.** C500 menu, File Transfer: two panes, computer and controller; select
   one or several files (CTRL-click) and send them in either direction with the arrow buttons.
   Works for any file: programs, variable files, configuration files. RC lines 1754-1800
   (steps), 1802-1806 ("Transferring Other Files"). AE lines 316-322 say the receiving side on the
   controller is `fastacid`, a CROS process.
5. **Backing up controller files.** The same dialog, controller to computer, into a directory made
   for the purpose. Needed because installing new CROS erases all files on the controller. RC lines
   1807-1832; FW lines 52 and 124 (back up `/app` and `/conf`, restore afterwards); SYS line 829.
6. **Making a directory on the controller.** Done by hand at the system shell (`cd app`,
   `mkdir name`, `dir`) before sending; App Setup (item 11) does it automatically. RC lines
   1728-1752.

### C. Editing, compiling and sending RAPL-3 programs

7. **Editor windows.** File New / Open / Save As, several windows at once; cut, copy, paste,
   delete, find, find next, replace, undo; syntax highlighting of RAPL-3 elements with changeable
   colours; Comment / Uncomment (adds or removes `;;`); per-window font. Source files must end in
   `.r3`. RC lines 607-1112 (highlighting 701-735, comments 743-765, standard features 770-916,
   font 927-950, saving and closing 953-1070). The manuals say you can use any other editor
   instead (RC lines 1076-1112).
8. **Compile.** Application menu, Compile (or the "R3" toolbar button) turns a `.r3` source file
   into an object file in the same directory, saving the edits first. The window must have a file
   name. RC lines 1122-1210. The compiler is the command-line program `r3c`; it can be run without
   Robcomm3 (RC lines 1362-1560; R3REF line 3660, which says it is typically in
   `C:\Program Files\CRS Robotics\RAPL-3\bin`). AE lines 307-310: no controller connection is
   needed to compile.
9. **Errors and warnings.** An error window lists each message with program name, line and
   description; errors stop the build, warnings do not. RC lines 1215-1262.
10. **Compiler options.** Set in App Setup: `-E` preprocess only, `-Dname[=def]` define a macro,
    `-nolibs`, `-fnimp` (no implicit declarations), `-fnat` (no automatic teachables),
    `-fstack=N`, `-l` line numbers, `-Lmodule` link a library, `-O0` / `-O1`, `-s` strip, `-x`
    keep only essential symbols, `-Wall` / `-Wmax` / `-Wnone`. RC lines 1280-1357. What the
    compiler does inside, and the object-file format, are not described in the manuals we have
    (the format was worked out here: `a465/crosprog.py`).
11. **Application setups (`.app` files).** Application menu, New App / Open App / Setup / Close
    App. An app file stores four settings: source file, controller directory (under `\app` if
    `ash` is to teach locations), object name, compile options. With an app open, Compile and
    Send work on those settings regardless of which editor windows are open, and Send also makes
    the controller's sub-directory if it is missing. With no app open the defaults are the active
    editor's file, `-Wall`, object name = source name without extension, directory `\app\test`.
    RC lines 1840-2290 (settings 1907-1930, defaults 2275-2290).
12. **Send App.** Sends the compiled object file to the controller directory named in the app
    setup. RC lines 1995-2005 (the Send App button and menu); AD lines 1076-1100 and 1334-1345; AE lines 316-322.
13. **Multi-file programs and libraries.** An app can compile a main file that `.include`s other
    `.r3` files; a library is compiled with `-x`, given a `.r` extension and sent to `\lib`, and a
    program links it with `-Lname.r`. RC lines 2020-2260.
14. **Version check.** C500 menu, Version shows the CROS version on the computer; `crosver` at
    the shell shows the controller's. The minor numbers must match, or a program fails with
    "syslib: signature incompatible". RC lines 1657-1705.

### D. Application files, teachable variables (`.v3`) and teaching locations

15. **Application variable files.** An application is a directory under `\app` holding a program
    file and a variable file of the same name, for example `\app\test\test` and
    `\app\test\test.v3`. The manuals attribute to Robcomm3 only the sending of the program file
    to the right directory and, through File Transfer, copying of variable files (ASH lines
    405-415; RC lines 1802-1806; AD lines 730-775, which also gives the file-name conventions).
    Reading or editing the contents of a `.v3` file from the PC is not described in the manuals
    we have.
16. **Teaching locations.** Done in `ash` or on the teach pendant, not in Robcomm3: the task table
    at AD lines 150-190 leaves the Robcomm3 column empty for open, transform, teach and set, and
    gives Robcomm3 only "write program, compile program" and "send program". The Robcomm3
    terminal is how `ash` is reached (items 1 and 17). After a newer program file is sent, `ash`'s
    `refresh` updates the variable database (AD lines 1334-1345; `ash.help`, `@@refresh`).

### E. Running and stopping programs

17. **Run a program.** Not a Robcomm3 menu function in the manuals: you type `run` at the `ash`
    prompt (runs the program and variable file named like the open application), or type the
    program's name at the shell (Force-Sensor guide, lines 454-460: `ash test`, then `fon.r`,
    `fon`). AD lines 839-856. The glossary (UG lines 2625-2629) nevertheless says Robcomm3 is used
    to "load, retrieve, edit, and run robot applications"; no run button or menu is described in
    the manuals we have. How a running program is stopped from Robcomm3 is not described in the
    manuals we have; the controller's stop keys are in this repo's README (Ctrl-Z).

### F. Controller configuration, calibration files and firmware

18. **Configuration and calibration files.** Robcomm3 has no editor for them in the manuals. They
    are ordinary files in `/conf` (`robot.cal`, `robot.cfg`, `rc`, `simsockd.cfg`, `startup.sh`)
    that File Transfer copies out and back (FW lines 52 and 124). Configuring and calibrating
    themselves are system-shell commands (`setup`, `cal`, `home`), listed under the terminal at RC
    lines 2395-2410.
19. **Firmware.** Installing CROS is a separate program, the CROS-500C Firmware Download Utility;
    Robcomm3's part is the file backup and restore around it (items 4 and 5). FW lines 40-130.

### G. Mentioned in passing, not pinned down

- UG lines 2627-2629 say Robcomm3 "provides utilities such as terminal emulation and C500C memory
  checks". No memory-check function is described in the Robcomm3 chapters we have.
- `Track-User-Guide.txt` line 1528 says to enter HOME "using the teach pendant, Robcomm terminal
  emulator utility, or Robcomm Dashboard". No "Dashboard" is described anywhere else.
- `Force-Sensor-C500C.txt` line 1323 says "choose terminal mode from the utilities menu"; the
  Robcomm3 chapters name no "utilities menu" (they use the C500 menu). It may be another version.
- The Robcomm3 User Guide (UMI-UM-RC1) and Robcomm3 for Windows Reference Manual (UMI-23-504)
  are named at `Force-Sensor-C500C.txt` lines 94-95 and `Servo-Gripper-User-Guide.txt` line 92.
  They are not among our manuals, so anything only they would say is unknown to us.
- POLARA (CROS on Windows NT): RC lines 2600-2660 say Robcomm3 there can edit and compile but
  not send, and its terminal reaches only the C500. Not relevant to this arm.

## 2. Where this repo stands

Size is a rough guess of work: small is an evening, medium a few sessions, large is the bulk of the
project. Status as of 2026-11-25.

| # | Function | What we have today | What is missing to call it equivalent | Size |
|---|----------|--------------------|---------------------------------------|------|
| 1 | Terminal window | `scripts/a465-term`: serial terminal to the Console port, raw keys, everything also logged to `sessions/` (exit Ctrl-]). The `$` shell and `ash` both work through it. | A terminal in the web page, so no second program needs the port. | small: the serial and prompt handling exist (`a465/console.py`) |
| 2 | Serial settings | `--port` / `--baud` options on every script; 57600 is the factory console rate, our tools run the line at 38400. | Nothing the manuals ask for. | small |
| 3 | Other terminals | Any terminal works; ours is not special. | Nothing. | small |
| 4 | File transfer, both directions | Controller to PC: `scripts/a465-getfile`, checked against the controller's `cksum`, about 30 files up to 43 KB (2026-11-23). PC to controller: `scripts/a465-putfile` over `/sbin/fastacid`'s packets (`a465/aci.py`), checked with `cksum`; used since for programs of 2444 and 3600 bytes (2026-11-25), each matching the controller's `cksum`. | Uploads beyond a few kilobytes tried; several files in one go; a directory browser (listing is only typed `dir` or `ls`); one tool or page that does both. | medium: protocol done, needs testing at size and a front end |
| 5 | Backing up controller files | `controller/backup-2026-11-23/` is a verified copy of `/conf`, `/app` data and a dated listing, made with `a465-getfile` one file at a time. | A single command that copies all of `/app` and `/conf`; a restore command (putfile can put each file back). | small |
| 6 | Make a directory on the controller | Typed `mkdir` in the terminal. | Done for you when sending. | small |
| 7 | Editor for RAPL-3 source | Nothing. The tests use `.r3s` assembly files (below), not RAPL-3. | Any text editor handles `.r3` files; Robcomm3's highlighting and comment keys are conveniences. Only worth doing together with the compiler. | small as a plain page, medium with highlighting |
| 8 | Compile `.r3` to an object file | **Done, proven on the controller 2026-11-26** (`scripts/a465-r3c`, CRS's compiler under an emulator; `r3c-hello` and `r3c-fib` ran: docs/RAPL3-COMPILER.md). Besides the compiler we have an assembler for the controller's bytecode (`a465/r3asm.py`, source `.r3s`, proved byte-for-byte on vendor programs), a disassembler (`a465/r3dis.py`), the object-file header writer (`a465/crosprog.py`) and an interpreter for tests (`a465/r3vm.py`). Our own programs on the controller (`jogd`, `segtest`) were made this way; `homezc` is the 2001 firmware's program re-stamped. The compiler pass `rc1` and the preprocessor `rpp` are kept under `vendor/crs/` and run through `a465/crosemu.py`; the vendor's `r3c` driver is kept for reference and never run (`a465/r3c.py` does its job). | A compiler from RAPL-3 source (language described in `docs/text/RAPL-3LanguageReferenceGuide.txt`) to that bytecode, linking `syslib.r` and `robotlib.r` (builtin numbers are in `a465/r3dis.py`, library routines in `controller/robotlib-functions.txt`). | large: a full language front end plus code generation; the object format and instruction set are already known |
| 9 | Compile errors and warnings | Assembler errors only. | Messages with program name, line and text, errors against warnings. Comes with item 8. | medium |
| 10 | Compiler options | None. | At least `-L`, `-x`, `-s`, `-W*`, `-D`, `-E`. Comes with item 8. | medium |
| 11 | Application setups (`.app`) | Nothing: each tool takes its paths on the command line. | A saved setup per application: source, controller directory, object name, options; compile and send in one step. | small once 8 exists |
| 12 | Send App | `a465-putfile` sends any file, but you give the controller path and make the directory yourself. | Make the directory, then send; one command from the saved setup. | small |
| 13 | Multi-file programs and libraries | None for source. `r3dis` reads our unit's two libraries. | `.include`, library compile (`-x`, `.r`), link (`-L`). Part of item 8. | medium |
| 14 | Version check | `crosver` typed in the terminal; the controller is CROS 1.16.471 (`docs/CONTROLLER.md`). The "signature incompatible" stamps are understood (`a465/crosprog.py`). | A check that a compiled program's library stamps match the controller before sending. | small |
| 15 | Variable (`.v3`) files | Copied off and on with item 4 (`controller/backup-2026-11-23/app/caco/caco2.v3` is the lab cell's locations). The file's first bytes (magic `ae 10 1a da`, a time stamp, the application name) are noted in `docs/CONTROLLER.md`. | A reader and writer for the records (locations, numbers, strings) so locations can be listed and edited on the PC; the record format is not worked out. | medium: format to be worked out from `caco2.v3` and the `v3_vars_*` builtins |
| 16 | Teaching locations | Partly: poses can be saved from the pad and the status page and replayed with go-to moves (built 2026-11-26, not yet run on the arm); the controller's own `.v3` variable files are still not read. The `ash` commands (`here`, `move`, `moves`, `appro`, `depart`) are available by typing; none of them has been tried here yet apart from the relative moves in section 3. | Create, list and edit locations from the PC; teach by `here`; move to them; tied to item 15. | medium |
| 17 | Run and stop a program | Specific programs only, each under supervision: `a465-homezc`, `a465-joint`, `a465-segtest`, `a465-jogtest`, `a465-padjog --jogd`, `a465-home`; all stop with Ctrl-Z and report. Typing `ash <app>` then `run` is possible by hand; not tried by us. | A general "run this program, show its output, stop it on request" from the PC; arm-power and E-stop reminders. | small to medium: the run-and-watch pattern exists in `a465/home.py` and `a465/homezc.py` |
| 18 | Configuration and calibration files | `a465/calfile.py` and `scripts/a465-calinfo` read `/conf/robot.cal`; items 4 and 5 copy the files; `scripts/a465-xzero` zeroes one joint. `/diag/setup` has been driven once (`a465-restore-cell-config`, dropped). | An editor or viewer for `robot.cfg` and the other `/conf` files; calibration steps from the PC. `calzc` / `cal` have not been run and must not be without the owner. | medium |
| 19 | Firmware install | Nothing. | The firmware download utility is a separate vendor program; its protocol is not described in the manuals we have. Not attempted. | large, and risky: a failed install erases the controller's files |
| 20 | Terminal-level arm control | Joint jog and homing from the pad or scripts (`a465-padjog`, `a465-home`, `a465-joint`); the web page drives only the simulated arm (see ROADMAP, "Digital twin"). | The `ash` motion and display commands (section 3) wrapped as PC functions. | medium |

Honest summary: we can talk to the controller, copy files both ways, run our own small programs on
it, and move and home the arm. We can compile RAPL-3 source (proven on the controller 2026-11-26), so a program
written as a Robcomm3 user would write it can be produced. Teachable-variable files
are copied but not read.

## 3. The controller's command set (ash)

From `sessions/controller-files/lib/ash.help` of this unit (`@@commands`, then one `@@name` entry
per command). Type `help <command>` in `ash` for the full text. Distances are in the current units
(`units` 1 is millimetres); angles are in degrees; `speed` is a percent of maximum.

| Purpose | Commands | What they do (from the help text) |
|---------|----------|-----------------------------------|
| Joint moves | `joint <n>,<units>` | Rotate a rotational joint by degrees, or move a linear joint (track). |
| | `motor <n>,<pulses>` | Rotate a motor by encoder pulses. |
| | `ready`, `calrdy` | Move to the ready position / the cal ready position. |
| World frame, joint-interpolated | `wx wy wz <dist>` | Jog the tool centre point along world X, Y or Z. |
| | `xrot yrot zrot <angle>` | Jog the tool centre point around a world axis. |
| World frame, straight line | `wxs wys wzs <dist>`, `xrots yrots zrots <angle>` | The same in straight-line mode. |
| Tool frame, joint-interpolated | `tx ty tz <dist>` | Jog along the tool's X, Y or Z. |
| Tool frame, straight line | `txs tys tzs <dist>` | The same in straight-line mode. |
| Rotations of the tool | `yaw`, `pitch`, `roll <angle>` | Alter the tool yaw / pitch / roll. |
| | `yaws`, `pitchs`, `rolls <angle>` | The same keeping the tool tip in place. |
| | `align <n\|x\|-x\|y\|-y\|z\|-z>` | Align the tool to the nearest or a given cartesian axis. |
| Locations | `here` / `here <loc>` | Show the position / store it in a location variable. |
| | `move <loc>`, `moves <loc>` | Move the tool tip to a location, joint-interpolated / straight line. |
| | `appro <loc>,<dist>`, `appros <loc>,<dist>` | Move to an approach position, joint-interpolated / straight line. |
| | `depart <dist>`, `departs <dist>` | Move to a depart position, joint-interpolated / straight line. |
| | `wshift <loc>,...`, `tshift <loc>,...` | Shift a location in the world / tool coordinate system. |
| Variables and application | `new`, `set` (`!`), `print` (`?`), `list`, `erase`, `eraseall` | Create, set, show, list and delete variables (types int, float, string, cloc, ploc, gloc). |
| | `save [name]`, `merge <file>`, `refresh`, `run` | Save variables to the `.v3` file, merge another file, re-read the program file, run the application's program. |
| Tool and base transforms | `tool_set` (`tool`) | Display or set the tool transform (x,y,z,yaw,pitch,roll or a cloc). |
| | `base_set` (`base`) | Display or set the base offset. |
| | `stance_set` (`stance`) | Display or set the arm stance (shoulder, elbow, wrist). |
| Speeds and accelerations | `speed` | Display or set the speed in percent of maximum. |
| | `linspd`, `linacc` | Linear speed and acceleration. |
| | `rotspd_set`, `rotacc_set` | Maximum rotational speed and acceleration. |
| | `accel <j1>,...,<j6>` | Display or set the joint accelerations. |
| Gripper | `grip_open` (`go`), `grip_close` (`gc`), `grip <dist>` (`gripdist_set`), `gtype`, `wgrip` | Open and close with a force; open to a distance; set air, servo or magnetic type; report the jaw distance. |
| Status and display | `wcmd` (`w0`), `wact` (`w2`), `wend` (`w4`) | Report commanded, actual and endpoint position. |
| | `w1`, `w3`, `w5` | Continuously report actual position, commanded position and position error in pulses; Ctrl-E ends. |
| | `arm_status`, `servoerr`, `robotver`, `ampstat`, `input <ch>[,<var>]`, `output <ch>,<0\|1>` | Arm status; servo error parameters; kinematics version; F3 amplifiers; read a GPIO input; set a GPIO output (channels 1-16). |
| Arm power, limp, errors | `armpower <on\|off>` (`arm`), `limp`, `nolimp`, `lock`, `unlock`, `clrerror`, `online`, `finish` | Arm power switch; limp or unlimp axes; lock or unlock joints; clear persistent F3 error bits; online mode; wait for a command to finish before the next. |
| Homing and calibration | `home [axis,...]` | Home one or more axes, or all. |
| | `/diag`: `cal calgrip calseek encres f3pack setup srvstat xzero zero` | Calibration and diagnostic programs, listed under "Robot setup & diagnostic" in the help. |
| Configuration | `cfg_save`, `units`, `gtype`, `use` | Save number of axes, base and tool offsets, track lengths, gripper type and units; set units; choose the robot server. |
| Shell and files | `cd`, `echo`, `help`, `exit`, `pendant`, `ver` | Change directory; echo; help; end the session; hand control to the pendant; ash version. |
| | `/bin`: `ls dir cp copy mv rm del mkdir cksum df ps more v3make v3update ...` | System commands listed under "System (/bin) Commands". `v3make` and `v3update` have no help text in this file. |

Measured on the arm 2026-11-25:

- `wzs 10` moved the tool 10.009 mm straight up in world Z (X +0.002 mm, Y +0.019 mm, rotations
  unchanged). `wzs -10` returned it to within 0.06 mm.
- The pendant-style manual mode in world and tool frames (`man_mode_set 3` / `4` followed by
  `man_move_start`) is accepted by the controller but produced no motion in five trials the same
  day. The cause is not yet known.
