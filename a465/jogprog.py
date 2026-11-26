"""jogd: a program that runs ON the controller and jogs the arm from short commands on its console.

    python3 -m a465.jogprog            build controller/programs/jogd and jogd-nospeedset
                                       (each with its .r3s source and .lst listing)

On the controller (after `scripts/a465-putfile controller/programs/jogd /tmp/jogd`):

    /tmp/jogd [mode [maxspeed [s|r]]]          defaults: 1 (joint), 25, s

Why: the PC's gamepad jog types a whole command per move, and each is a separate start-stop
move. jogd keeps one continuous move going with the routines the teach pendant uses and changes
or stops it on a 3-byte command, so the PC only has to send keep-alives.

Protocol (standard input, printable ASCII only: fastacid removes 0x01, 0x05, 0x07, 0x16, 0x1a)

    A..F nn   joint 1..6 positive at nn percent (01..99, clamped to maxspeed); a..f negative
    .         stop the move               q   stop, release control, exit 0
    p         print the six joints' encoder counts (does not touch the move, the dead-man or a
              half-received command)
    anything else is ignored; a letter always starts a new command, so a lost byte resyncs.

    The command in force again is a keep-alive. Another joint, another direction, or the same
    joint and direction at another speed: stop, then start. (`r` as the third argument keeps the
    old experiment: a speed change calls man_move_start again without stopping. On this
    controller that returns no error and does NOT change the speed.) Dead-man, while a move is in
    force: when there is nothing to read and no command or keep-alive has come for DEADMAN_MS
    (300), or after any byte when none has come for DEADMAN_LONG_MS (1000, so that a stream of
    other bytes cannot hold the move), the move is stopped and `T` printed; the next command
    starts afresh. The clock restarts when a start sequence has finished, so the time the
    controller's calls take (about 0.1 s each on the arm) does not count against the PC, and
    bytes that queued meanwhile are read before the 300 ms check.

    One jog is the pendant's sequence, in its order (bin/pendant: key down 0x0d40 -> 0x0474 ->
    0x06d7, key up 0x07d9):
        start   speed_set(s) if s is not the speed in force (0x0474; not in jogd-nospeedset),
                man_mode_set(mode) (0x05b2(1) at 0x06f2), man_move_start(joint, s) (0x0701)
        stop    man_move_stop(|joint|) (0x07e5), halt() only if that failed (0x07eb),
                man_mode_set(0) (0x05b2(0) at 0x07f3)
    Every stop uses that sequence: `.`, the dead-man, a change of command, a failed start (after
    its E line), a failed input, `q`. Like the pendant's 0x05b2, man_mode_set is not called when
    the wanted state is in force already, and after a refusal robot_mode_get (0x55) says whether
    manual mode is on (2 = MODE_MANUAL), as the pendant asks at 0x0625.

    Output, one line each: `jogd ready`; `+A07` a move started (the command as used, after the
    clamp); `-` a move stopped; `T` stopped by the dead-man; `P 4225 0 0 -24830 0 2` the counts
    of joints 1..6 (printf "{}": a minus sign when negative, no plus sign); `E <r> <text>` a library call
    returned r < 0 (text from syslib str_error(-r), as homezc words its errors). A failed start
    leaves no move in force; repeats of that same command are then treated as keep-alives (no
    retry storm) until a different command or `.`.

Seen on the real arm (2026-11-24, CROS 1.16.471), with the earlier jogd that set manual mode once
per run: it loads and runs (`jogd ready` 1.13 s after the command line); the second non-blocking
open of /dev/cin works and printf output is not held back; `+F02` 211 ms after the command with a
speed_set first, 116-118 ms without; `-` 90-102 ms after `.`; `T` 425 ms after the last keep-alive;
`p` answers in about 95 ms. Under the gamepad: a second man_move_start at another speed kept the
first speed, and a reversal ran fast through zero to about the mirror position before creeping on.
This version copies the pendant's per-jog man_mode_set because of that; that the per-jog mode set
cures it is not shown by any file (see below).

What the listings say (bin/pendant, sbin/robotsrv, lib/robotlib.r; addresses are word addresses)

- The pendant sets its keyboard to report key releases and not to repeat (0x01a4, 0x01a8; the two
  routines named by the order of the pendant's exported names), and its event loop (0x113c) hands
  each key to 0x0dd6: a jog key starts a jog once (0x0d40); event 0 (taken to be the release) and
  the stop events (0x3000000.., from its signal flags) stop it (0x07d9). So it never calls man_move_start on a moving joint in normal use; nothing in
  0x0d40/0x06d7 checks for that, though. A speed key during a jog only calls speed_set (0x1042).
- Before man_mode_set the pendant also turns its live-man output on (0x0145(1) at 0x06de: an
  ioctl on /dev/status), and turns it off when robotisdone (robotlib 0x58, name by the order of
  robotlib's names) reports the move done (0x03b2). That is the pendant's own device, not a robot
  library call, and jogd does not do it.
- robotsrv: message 41 (man_mode_set, handler 0x1256) sends the mode to the servo board
  (/dev/xputer1, message 1145 via 0x0013) and on success sets its state (0 off, 2 manual) and all
  eight 2-bit direction fields to 3 (stop). Message 42 (man_move_start, 0x12e6) refuses unless
  manual (-13) and 1 <= |joint| <= 8 (-22), stores the speed in one word for all joints (G544),
  sets the joint's direction field (1 +, 2 -) and sends the 13-word block [speed, 8 words of 128,
  direction fields, 12, 0, 0] to the servo board (message 1934). Message 43 (man_move_stop,
  0x1386) sets the field to 3 and sends the same block. The server keeps no position for manual
  moves; what the servo board does with the block (a reference position, a ramp, when it takes a
  new speed) is not in any file here.
- ctl_get (robotlib 0x02) and ctl_rel (0x04): diag/xzero; the pendant takes control in 0x04ff.
  Modes: the pendant's display (0x0bfd) shows its setting as JOINT, CYL, WORLD, TOOL; robotlib's
  constants: MMODE_OFF 0, MMODE_JOINT 1, MMODE_CYLIND 2, MMODE_WORLD 3, MMODE_TOOL 4.
- speed_get (0x43) when control is taken, speed_set (0x44) only for a speed other than the one in
  force, the first speed put back at exit: the pendant's 0x03d4, 0x0474 and 0x04ff. speed_set goes
  to the servo board as its own message (robotsrv 0x0bd2, message 1016); whether manual moves use
  it is not in any file. jogd-nospeedset leaves speed_get and speed_set out.
- `p`: pos_get (robotlib 0x0e) with (POSITION_ACTUAL = 1, &loc), then the builtin loc_pdata_get(&loc,
  &counts) (no result). bin/ash's "Actual Position :" display, which is the block `w2` prints, does
  exactly this (ash 0x22ab, then 0x13e8 prints the counts as its PULSES row with "{+10}"), and ash's
  continuous readout of the same call (0x228a) is headed "Actual Position (motor pulses)"; its
  JOINTS row is motor_to_joint (0x0b) of the same location. diag/xzero reads with type 2
  (POSITION_COMMANDED; robotlib's constants: ACTUAL 1, COMMANDED 2). The robot server's handler
  of pos_get (message 5, robotsrv 0x023e) checks neither control nor manual mode and does not touch
  the manual move: it asks the motion task and builds a precision location from its 8 axis words.
  The 3.1 interpreter's loc_pdata_get (0x7020) copies location words 1..8 to the 8-word array. The
  w2 program itself (/bin/w2, 1260 bytes) was never copied off, so that it reads type 1 is inferred
  from its output, which matches ash's line for line. Option pos_type changes the type.
- The input is opened again as "/dev/cin" with flags 5 = O_RDONLY|O_NONBLOCK (syslib's constants)
  and read one byte at a time with reads(); -35 (-EAGAIN) means nothing has arrived, and the
  program then waits with syslib delay(10). That is how sbin/fastacid reads /dev/cout (0x0195:
  open flags 5, reads, -35, delay(10)); /dev/cin is the shell's standard input (bin/shell 0x0095
  opens it and dup()s it to fd 0). The dead-man clock is mtime(&t), whose first word syslib's own
  delay() and TimeStampEnd use as milliseconds.


jogd-v (controller/programs/jogd-v, option maxvel; the other builds are byte for byte unchanged): a bench
build to try whether a RUNNING manual jog follows the motor speed limit table. At start it reads the table
with maxvels_get (robotlib 0x15 -> 0x04ba: request 9 op 0, 8 floats; the manual gives rpm) and prints
`MV v1 .. v8`. `vNN` (two digits, 01..99) while a move is in force sets the moving joint's entry to NN
percent of the value read at start, every other entry as read (maxvels_set, 0x16 -> 0x04c3, request 9 op 1;
robotsrv 0x04b0 sends it to the servo board at once as message 1167 and keeps nothing; no file is written
for it), and prints `V NN <value>`; it can never set an entry above the value read at start. Every stop
(`.`, the dead-man, a change of command, a failed start, `q`, an input error) puts the table read at start
back right after man_move_stop, and finish() tries once more. A kill (Ctrl-Z, Ctrl-E) cannot restore it.
`v` also ends a half-received jog command; a letter ends a half-received `v`.


jogd-c (controller/programs/jogd-c, option cart; the other builds are byte for byte unchanged). Mode 1 (and 2)
letters are jogd's manual moves, unchanged. Modes 3 (world) and 4 (tool): online(ON 1) after speed_get, and
online(OFF 0) at every exit that runs finish() (q, an input error); a letter is one straight-line move of CART_MM
(150) mm, or CART_DEG (45) degrees for D E F, with jog_ws (mode 3) or jog_ts (mode 4) at speed_set(NN):
    jog_ws(axis, distance) robotlib 0x34 -> 0x07f5: request 19 [axis, distance, 0]; jog_w 0x07df sends 1 there
        (joint-interpolated). robotsrv 0x08e0 puts the distance (negated for a negative axis) in element |axis| of
        eight floats and sends them with that flag as message 1005; jog_ts 0x36 -> 0x0821 is request 58, robotsrv
        0x0964, message 1198. ash's wxs .. zrots / txs .. rolls are these with a fixed axis (robotlib 0x14c9 ..
        0x1598): 1 X, 2 Y, 3 Z, 4 Z rotation (zrots, yaws), 5 Y rotation (yrots, pitchs), 6 X rotation (xrots,
        rolls); ash passes the distance as a float in current units (ash 0x0487: an int argument is itof'd).
    Letters A..F / a..f: + / - axis 1..6 in that numbering. `+Xnn` when the call has returned 0.
    Stop (`.`, the dead-man, another letter, q, an input error): halt() first (robotlib 0x21, request 23: robotsrv
        0x0b2b sends message 1999 and answers at once; its wait loop 0x00bf returns 0 at 0x00c1), then `-` / `T` as
        soon as it returns, then settle(): robotisfinished until it says 1, at most SETTLE_MS (1500), delay(10) between
        (the shape of robotsrv's disabled post-halt loop 0x00c3-0x00f4, which polls message 1010; 1010 is the one
        message the -42 lock-out lets through, 0x003a-0x0041), and only then the next start. halt() does not start the
        1000 ms -42 lock-out ("device is being reset"): only Ctrl-Z / Ctrl-A at the console (fastacid 0x0137 / 0x013c
        signal the server with 6 / 8; robotsrv 0x202c, which also sends 6 to the controlling program, 0x2044) and arm
        power going off (0x1fb2) do.
    Input first: during an online move a `p` is only noted and answered when reads() finds nothing waiting, and the
        end-by-itself poll runs only then too (main loop, idle), so a `.` waits at most for the one call under way.
    A move that ends by itself: with no input waiting, at most every FIN_MS (1000) ms and not sooner after the start,
        robotisfinished (robotlib 0x3e -> 0x09d4, request 20 op 1; the manual: 1 finished, 0 not); 1 prints `-` and the
        command's repeats are then ignored. The same letter at another speed while moving: a keep-alive (the speed is
        fixed per move). An error from the move call: `E`, then the stop (halt, `-`).
    SIGABRT (6): robotsrv sends it to its controlling process on a board error (`Error 0x4000006` / `Path error`), arm
        power off (0x1f9b-0x2017: unless the process registered a signal with sig_arm_set) and on Ctrl-Z (0x2044); its
        default action ends the program. jogd-c installs a handler (signal(6, on_sigabrt, 0), as ash does at
        0x27fb-0x2800) that only notes it; the main loop then ends the move, reads robot_error_get (robotlib 0x7a,
        request 55: robotsrv's G654, the code it prints as `Error 0x...`; after -42, i.e. the lock-out, it waits
        ABORT_WAIT_MS (1100) and asks again), prints `A <code> <text>` (or `A abort`), puts manual mode off (mode 1) or
        online(ON) again (modes 3, 4) and reads on. The command in force stays, so its repeats are ignored. Control is
        kept: robotsrv's error path does not touch G557, online (G563) or manual mode. Ctrl-E (fastacid 0x012e:
        sigfifo SIGINT 7 to the console's readers) still ends jogd-c.
Go-to lines (jogd-c, every mode): `G c1 c2 c3 c4 c5 c6 NN` then CR (or LF; a following LF or CR is ignored):
absolute motor counts for axes 1..6 as `p` prints them; one move() (robotlib 0x2c, joint-interpolated) to pos_get(2)
with axes 1..6 replaced (7 and 8 as they are). `L x y z yaw pitch roll NN` then CR: hundredths of a mm / of a degree,
the cloc words 1..6 (ash's WORLD row prints loc_cdata words 1..6 as X Y Z Z-Rot Y-Rot X-Rot, ash 0x142a-0x1436); one
moves() (straight) to here() as a cloc with those words replaced (words 7, 8 as here() gave them). Both at
speed_set(NN) clamped to maxspeed, online (switched on for the move and off at its end in modes 1, 2), answered `+G` /
`+L` when the call returned 0; a refusal prints `E n text`, then the stop (halt, `-`). Grammar: the letter, then six
times one space and a number (an optional '-' and 1..9 digits), then one space, two digits, the end of line. A byte
that does not fit drops the line: `.`, `q`, `@`, `G`, `L` and A..F / a..f silently (and are then taken as usual),
`p` is answered and keeps the line, anything else prints `E -22 bad go-to line` (speed 00 too). `G` or `L` ends a
half-received jog command or vector. While a go-to is in force `g` is its keep-alive (the jog dead-man rules: `T`
after a halt); any jog letter, vector, G / L line, `.`, `q` stops it first (halt first, as for a jog); its arrival is
robotisfinished from the idle loop at most every GOTO_FIN_MS (250) ms, which prints `-`; `g` after that is ignored.
`w` (jogd-c): `W x y z yaw pitch roll`, here() as a cloc in the L line's units (rounded), so a pose can be noted and
replayed; during an online move it is answered once no input is waiting, as `p`.
jogd-c input: one reads() takes every waiting byte (up to CBUF, 120) and they are worked through in order with no
clock or library call per byte (the dead-man clock is read once per reads()); measured in r3vm on a 106-byte burst of
vector keep-alives: 56 instructions, 0.01 reads() and 0.08 mtime() per byte, against 60, 1 and 1.07 before. `g` is
the keep-alive of any move in force (jog, vector, go-to); repeats of the letter or vector still are too.
Interrupted calls: syslib's delay retries msleep on -4 (syslib 0x0140), but a robot call or reads() interrupted by
SIGABRT returns -4 (EINTR). In jogd-c report() turns -4, or any error while an abort is pending, into the abort
(no `E` line), stop() and the starts do nothing while one is pending, and an interrupted reads() is the abort too, so
the one `A` line comes whatever call the signal hit.
`@` vectors (jogd-c, all modes but 2): `@` then six components, each a sign and a digit (+0..+9, -0..-9), then
two speed digits: `@+3+4+0+0+0+002` (15 bytes). A letter, `.`, `q`, `@` or any character that does not fit its
place drops a half-received vector (then it is taken as usual); `p` does not. `+@...` echoes it as used (signs and
digits as received, the speed clamped) when the move call has returned 0. While a vector move is in force any
vector is its keep-alive; the vector last started is ignored when nothing is moving (it ended or failed); a vector
while a letter's move is in force stops that first. Modes 3, 4: components 4..6 must be 0 (else `E -22`); the
target is the commanded pose shifted by CART_MM x (x, y, z) / |(x, y, z)|, one moves() (robotlib 0x2d -> 0x0714:
straight; 0x0319 refuses a precision location for it, -16777217): here() into a cloc (class 1: robotlib 0x0f88
-> motor_to_world, request 36), mode 3 adds to loc_cdata words 1..3 (x, y, z, then yaw, pitch, roll: manual), mode 4
calls shift_t(&pose, 1, displacement cloc tag 5) as ash's tshift does (ash 0x1cc2-0x1cd3; robotlib 0x13c7, request
57). Mode 1: one move() (robotlib 0x2c) to pos_get(2) with joint k advanced by c_k x VEC_DEG (30) / max|c| degrees
(counts per degree from a465/calfile.py), in online(ON) set just before it and put back to OFF at its stop or end.
"""
import argparse
import math
import struct
from dataclasses import dataclass, replace
from pathlib import Path

from a465 import calfile, r3asm, r3dis

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "controller" / "programs"
BUILD_TIME = 0x6B057C40          # 2026-11-24 12:00 UTC; the firmware is not known to check it

# robotlib routine numbers (controller/robotlib-functions.txt)
CTL_GET, CTL_REL, POS_GET, HALT, SPEED_GET, SPEED_SET = 0x02, 0x04, 0x0E, 0x21, 0x43, 0x44
MAN_MODE_SET, MAN_MOVE_START, MAN_MOVE_STOP, ROBOT_MODE_GET = 0x52, 0x53, 0x54, 0x55
MOVE, MOVES, JOG_WS, JOG_TS, ROBOTISFINISHED, ONLINE, HERE, SHIFT_T = 0x2C, 0x2D, 0x34, 0x36, 0x3E, 0x3F, 0x62, 0x80
STR_ERROR, DELAY = 0x07, 0x14    # syslib
VEC_AXIS = 99                    # jogd-c: `axis` while an `@` vector is the command in force
GOTO_AXIS = 98                   # jogd-c: `axis` while a G / L go-to is the command in force


@dataclass(frozen=True)
class Options:
    mode: int = 1                # MMODE_JOINT
    max_speed: int = 25
    restart: bool = False        # default speed-change variant: False = s (stop, start); True = r (experiment)
    deadman_ms: int = 300        # silence with nothing to read
    deadman_long_ms: int = 1000  # since the last command, checked after every byte too
    poll_ms: int = 10
    input_path: str = "/dev/cin"
    open_flags: int = 5          # O_RDONLY | O_NONBLOCK
    speed_set: bool = True       # False: never speed_get/speed_set (the jogd-nospeedset build)
    pos_type: int = 1            # pos_get type for `p`: POSITION_ACTUAL, as w2's PULSES row
    maxvel: bool = False         # True: the `vNN` bench command (the jogd-v build); see the docstring
    cart: bool = False           # True: modes 3, 4 as straight-line online moves, and `@` vectors (jogd-c)
    cart_mm: int = 150           # jogd-c: a letter's (and a vector's) distance in modes 3, 4, mm ...
    cart_deg: int = 45           # ... and a rotation letter's, degrees
    vec_deg: int = 30            # jogd-c mode 1 `@`: the largest joint change, degrees
    fin_ms: int = 1000           # jogd-c: robotisfinished (end by itself) at most this often, only with no input waiting
    settle_ms: int = 1500        # jogd-c: after halt, robotisfinished until 1 for at most this long
    abort_wait_ms: int = 1100    # jogd-c: after SIGABRT, when the server answers -42 (its 1000 ms lock-out)
    goto_fin_ms: int = 250       # jogd-c: G / L go-to: robotisfinished (arrival) at most this often
    build_time: int = BUILD_TIME


def _float_word(x):
    return struct.unpack("<I", struct.pack("<f", x))[0]


def _cart(opts):
    """jogd-c's pieces of source, keyed by the place in source() they go; all empty for other builds."""
    keys = ("equ", "data", "setup", "loop", "vbyte", "keep", "start", "stop", "tprint", "idle", "finish", "gk", "report",
            "code")
    if not opts.cart:
        return dict.fromkeys(keys, "")
    if not opts.speed_set or opts.maxvel:
        raise ValueError("the cart option needs speed_set and not maxvel")
    if not 1 <= opts.goto_fin_ms <= 10000:
        raise ValueError("goto_fin_ms 1..10000")
    if not (1 <= opts.cart_mm <= 1000 and 1 <= opts.cart_deg <= 180 and 1 <= opts.vec_deg <= 90
            and 1 <= opts.fin_ms <= 10000 and 1 <= opts.settle_ms <= 10000):
        raise ValueError("cart_mm 1..1000, cart_deg 1..180, vec_deg 1..90, fin_ms and settle_ms 1..10000")
    ktab = [0] + [_float_word(opts.cart_mm / math.sqrt(n)) for n in range(1, 244)]
    rows = "\n".join("            word " + ", ".join(str(w) for w in ktab[i:i + 8]) for i in range(0, len(ktab), 8))
    cpd = ", ".join(str(round(c * 100)) for c in calfile.COUNTS_PER_DEGREE)
    lib = "\n    libcall robotlib:"
    p = {}
    p["equ"] = f"""
.equ CART_MM {opts.cart_mm}
.equ CART_DEG {opts.cart_deg}
.equ VEC_DEG {opts.vec_deg}
.equ FIN_MS {opts.fin_ms}
.equ SETTLE_MS {opts.settle_ms}
.equ ABORT_WAIT_MS {opts.abort_wait_ms}
.equ SIGABRT 6
.equ VEC_AXIS {VEC_AXIS}
.equ GOTO_AXIS {GOTO_AXIS}
.equ GOTO_FIN_MS {opts.goto_fin_ms}
.equ CBUF 120
.equ EINTR_R -4"""
    p["data"] = f"""
cart:       word 0                  ; jogd-c: 1 in modes 3, 4 (online ON from start to exit)
mkind:      word 0                  ; the move in force is an online move (jog_ws, jog_ts, moves, move): halt stops it
lastfin:    word 0, 0               ; mtime() of the last robotisfinished (or of the start)
ppend:      word 0                  ; a `p` read during an online move, answered once no input is waiting
cbuf:       string 124 ""           ; jogd-c reads up to CBUF waiting bytes at once ...
ci:         word 0                  ; ... and works through them: the next one
cn:         word 0                  ; ... and how many there are
settle0:    word 0, 0               ; mtime() when settle() began
aborted:    word 0                  ; set by the SIGABRT handler, acted on at the top of the main loop
errw:       zeros 2                 ; robot_error_get's word: the server's last board error (robotsrv G654)
s_abort:    string "A abort\\n"
finms:      word {opts.fin_ms}               ; finpoll's interval for the move in force: FIN_MS, or GOTO_FIN_MS for a go-to
gk:         word 0                  ; a G / L line being received: 0 none, 1 G, 2 L ...
gsub:       word 0                  ; ... where in it: 0 the space after the letter, 1 a number's first character,
gfield:     word 0                  ;     2 after its '-', 3 in its digits, 4 / 5 the speed digits, 6 the end of line;
gneg:       word 0                  ;     the number being read (field, sign, digit count, value)
gdig:       word 0
gval:       word 0
gtens:      word 0
gkind:      word 0                  ; the go-to in force or last started: 1 G, 2 L
gnums:      zeros 6                 ; its six numbers
wpend:      word 0                  ; a `w` read during an online move, answered once no input is waiting
wloc:       zeros 9                 ; `w`: here() as a cloc ...
wf:         zeros 8                 ; ... and its eight floats
s_pg:       string "+G\\n"
s_pl:       string "+L\\n"
s_gbad:     string "E -22 bad go-to line\\n"
s_w:        string "W {{}} {{}} {{}} {{}} {{}} {{}}\\n"
s_aborte:   string "A {{}} {{}}\\n"
vpos:       word 0                  ; `@` parser: 0 none, else the position (1..14) of the next character
vtens:      word 0
vsg:        zeros 6                 ; the received signs (1 = '-') ...
vdg:        zeros 6                 ; ... and digits of the six components
vc:         zeros 6                 ; their values
vl:         zeros 6                 ; the values of the vector last started
vloc:       zeros 9                 ; here() / pos_get(2), then the target
vdisp:      zeros 9                 ; mode 4: the displacement for shift_t
vf:         zeros 8                 ; loc_cdata_get's eight floats, or loc_pdata_get's eight counts
vd:         zeros 8                 ; the displacement (floats): x, y, z, then zeros
s_plus:     string "+"
s_minus:    string "-"
s_vline:    string "+@{{}}{{}}{{}}{{}}{{}}{{}}{{}}{{}}{{}}{{}}{{}}{{}}{{}}{{}}\\n"
cpd:        word {cpd}             ; counts per degree x 100, joints 1..6 (a465/calfile.py)
ktab:                               ; CART_MM / sqrt(n) as floats, n = 0..243 (n = x*x + y*y + z*z)
{rows}"""
    p["setup"] = """
    pushi 0
    pushaddr text:on_sigabrt
    pushi SIGABRT
    pushi 3
    builtinx signal                 ; survive SIGABRT, as ash does (ash 0x27fb-0x2800: signal(6, 0x000d, 0))
    getg L6, mode
    gtc L5, L6, 2
    putg L5, cart                   ; modes 3, 4: straight-line online moves
    ifnot L5, c_setup_done
    pushi 1
    pushi 1
    call setonline                  ; online(ON): a move call returns at once (segtest, robotsrv 0x0b66)
    pop L5
    ltc L4, L5, 0
    if L4, setup_failed
c_setup_done:"""
    p["loop"] = """
    getg L6, aborted
    ifnot L6, l_buf
    pushi 0
    call onabort                    ; jogd-c: a SIGABRT came (or a call was interrupted)
l_buf:
    getg L6, ci
    getg L5, cn
    lt L4, L6, L5
    if L4, l_next                   ; bytes left from the last reads()
    pushi CBUF
    pushleag cbuf
    getg L6, fd
    push L6
    pushi 3
    builtin reads                   ; every byte that is waiting, up to CBUF, in one call
    pop L5
    gtc L4, L5, 0
    ifnot L4, l_none
    putg L5, cn
    ldi L6, 0
    putg L6, ci
l_next:
    getg L6, ci
    push L6
    pushleag cbuf
    pushi 2
    builtin str_chr_get
    pop L7
    addc L6, L6, 1
    putg L6, ci
    push L7
    pushi 1
    call byte
    pop L4
    if L4, quit
    getg L6, ci
    getg L5, cn
    lt L4, L6, L5
    if L4, loop                     ; the rest of them, in order (the abort check runs between bytes)
    ldi L4, DEADMAN_LONG_MS
    goto deadman                    ; one clock read per reads(), not per byte
l_none:
    eqc L4, L5, EINTR_R
    ifnot L4, no_byte               ; -35 or 0: idle; anything else: the input failed, as in jogd
    ldi L6, 1
    putg L6, aborted                ; an interrupted reads(): the abort is handled next
    goto loop"""
    p["vbyte"] = """
    getg L6, gk                     ; ---- jogd-c: a G / L line being received
    ifnot L6, gl_none
    getg L6, gsub
    eqc L5, L6, 0
    if L5, gl_space1
    eqc L5, L6, 1
    if L5, gl_start
    eqc L5, L6, 2
    if L5, gl_first
    eqc L5, L6, 3
    if L5, gl_digits
    eqc L5, L6, 4
    if L5, gl_tens
    eqc L5, L6, 5
    if L5, gl_units
    goto gl_term
gl_space1:                          ; right after the letter: one space
    eqc L5, A3, 32
    ifnot L5, gl_bad
    ldi L5, 1
    putg L5, gsub
    goto gl_ok
gl_start:                           ; a number's first character: '-' or a digit
    ldi L5, 0
    putg L5, gneg
    eqc L5, A3, '-'
    ifnot L5, gl_first
    ldi L5, 1
    putg L5, gneg
    ldi L5, 2
    putg L5, gsub
    goto gl_ok
gl_first:                           ; its first digit
    gec L5, A3, '0'
    lec L4, A3, '9'
    land L5, L5, L4
    ifnot L5, gl_bad
    subc L4, A3, '0'
    putg L4, gval
    ldi L5, 1
    putg L5, gdig
    ldi L5, 3
    putg L5, gsub
    goto gl_ok
gl_digits:                          ; more digits (9 at most), or the space that ends the number
    eqc L5, A3, 32
    if L5, gl_endnum
    gec L5, A3, '0'
    lec L4, A3, '9'
    land L5, L5, L4
    ifnot L5, gl_bad
    getg L6, gdig
    gtc L5, L6, 8
    if L5, gl_bad
    addc L6, L6, 1
    putg L6, gdig
    getg L6, gval
    mulc L6, L6, 10
    subc L4, A3, '0'
    add L6, L6, L4
    putg L6, gval
    goto gl_ok
gl_endnum:
    getg L6, gval
    getg L5, gneg
    ifnot L5, gl_signed
    neg L6, L6
gl_signed:
    getg L5, gfield
    leag L4, gnums
    stx L6, L4, L5
    addc L5, L5, 1
    putg L5, gfield
    ldi L6, 1
    ltc L4, L5, 6
    if L4, gl_sub
    ldi L6, 4                       ; after the sixth number and its space: the two speed digits
gl_sub:
    putg L6, gsub
    goto gl_ok
gl_tens:
    gec L5, A3, '0'
    lec L4, A3, '9'
    land L5, L5, L4
    ifnot L5, gl_bad
    subc L4, A3, '0'
    putg L4, gtens
    ldi L5, 5
    putg L5, gsub
    goto gl_ok
gl_units:
    gec L5, A3, '0'
    lec L4, A3, '9'
    land L5, L5, L4
    ifnot L5, gl_bad
    subc L4, A3, '0'
    getg L5, gtens
    mulc L5, L5, 10
    add L4, L5, L4
    putg L4, gval                   ; the speed
    ldi L5, 6
    putg L5, gsub
    goto gl_ok
gl_term:                            ; the end of the line: CR (or LF)
    eqc L5, A3, 13
    eqc L4, A3, 10
    lor L5, L5, L4
    ifnot L5, gl_bad
    getg L6, gk
    ldi L5, 0
    putg L5, gk
    getg L5, gval
    push L5
    push L6
    pushi 2
    call gocommand
gl_ok:
    ldi L5, 0
    retv L5
gl_bad:                             ; a byte that does not fit
    eqc L5, A3, 'p'
    if L5, gl_none                  ; `p`: answered, the line kept
    ldi L5, 0
    putg L5, gk                     ; the line is dropped: silently for a command byte, taken as usual next ...
    eqc L5, A3, '.'
    if L5, gl_none
    eqc L5, A3, 'q'
    if L5, gl_none
    eqc L5, A3, '@'
    if L5, gl_none
    eqc L5, A3, 'G'
    if L5, gl_none
    eqc L5, A3, 'L'
    if L5, gl_none
    gec L5, A3, 'A'
    lec L4, A3, 'F'
    land L5, L5, L4
    if L5, gl_none
    gec L5, A3, 'a'
    lec L4, A3, 'f'
    land L5, L5, L4
    if L5, gl_none
    pushi 0                         ; ... with E for anything else
    pushleag s_gbad
    pushi 2
    builtinx printf
gl_none:
    eqc L5, A3, 'G'
    ifnot L5, gl_notg
    ldi L6, 1
    goto gl_begin
gl_notg:
    eqc L5, A3, 'L'
    ifnot L5, gl_end
    ldi L6, 2
gl_begin:                           ; G or L: a new line; it ends a half-received jog command or vector
    putg L6, gk
    ldi L5, 0
    putg L5, gsub
    putg L5, gfield
    putg L5, pstate
    putg L5, vpos
    retv L5
gl_end:
    eqc L5, A3, 'p'                 ; ---- jogd-c: `p` during an online move is answered later (main loop, idle)
    ifnot L5, vb_notp
    getg L6, mkind
    ifnot L6, vb_notp               ; no online move: at once, as jogd
    ldi L6, 1
    putg L6, ppend
    ldi L5, 0
    retv L5
vb_notp:
    getg L6, vpos                   ; ---- `@` and the 14 characters after it
    ifnot L6, vb_none
    ltc L5, L6, 13
    ifnot L5, vb_speed
    modc L5, L6, 2
    ifnot L5, vb_digit
    ldi L4, 0                       ; odd positions: the sign
    eqc L5, A3, '+'
    if L5, vb_sign
    ldi L4, 1
    eqc L5, A3, '-'
    if L5, vb_sign
    goto vb_drop
vb_sign:
    subc L5, L6, 1
    divc L5, L5, 2
    leag L3, vsg
    stx L4, L3, L5
    goto vb_next
vb_digit:                           ; even positions: the digit
    gec L5, A3, '0'
    lec L4, A3, '9'
    land L5, L5, L4
    ifnot L5, vb_drop
    subc L4, A3, '0'
    subc L5, L6, 2
    divc L5, L5, 2
    leag L3, vdg
    stx L4, L3, L5
    goto vb_next
vb_speed:                           ; positions 13, 14: the speed
    gec L5, A3, '0'
    lec L4, A3, '9'
    land L5, L5, L4
    ifnot L5, vb_drop
    subc L4, A3, '0'
    eqc L5, L6, 13
    ifnot L5, vb_last
    putg L4, vtens
    goto vb_next
vb_last:
    ldi L5, 0
    putg L5, vpos
    getg L5, vtens
    mulc L5, L5, 10
    add L4, L5, L4
    push L4
    pushi 1
    call vcommand
    goto vb_ret
vb_next:
    addc L6, L6, 1
    putg L6, vpos
vb_ret:
    ldi L5, 0
    retv L5
vb_drop:
    eqc L5, A3, 'p'
    if L5, vb_none                  ; `p` leaves a half-received vector alone, as it does a jog command
    ldi L5, 0
    putg L5, vpos                   ; anything else that does not fit drops it and is then taken as usual
vb_none:
    eqc L5, A3, '@'
    ifnot L5, vb_end
    ldi L5, 1
    putg L5, vpos
    ldi L5, 0
    putg L5, pstate                 ; `@` ends a half-received jog command, as a letter does
    goto vb_ret
vb_end:"""
    p["keep"] = """
    getg L6, mkind
    ifnot L6, c_kj                  ; jogd-c: an online move in force ...
    getg L6, moving
    ifnot L6, c_kj
    ifnot L4, c_kj
    goto c_done                     ; ... and its letter again, at any speed: a keep-alive (the speed is fixed per move)
c_kj:"""
    p["start"] = f"""
    getg L6, aborted
    ifnot L6, s_live
    ret                             ; an abort is pending: no new move before onabort
s_live:
    getg L6, cart
    ifnot L6, s_manual              ; jogd-c modes 3, 4: one straight-line move, online
    ldi L6, 1
    putg L6, moving
    putg L6, mkind
    ldi L6, FIN_MS
    putg L6, finms
    put A3, L7                      ; put SRC, DST: L7 = A3
    ltc L5, A3, 0
    ifnot L5, s_cabs
    neg L7, L7
s_cabs:
    ldi L6, CART_MM
    gtc L5, L7, 3
    ifnot L5, s_cdist
    ldi L6, CART_DEG                ; axes 4, 5, 6 are rotations (degrees)
s_cdist:
    ltc L5, A3, 0
    ifnot L5, s_cpos
    neg L6, L6
s_cpos:
    itof L6, L6
    getg L5, mode
    eqc L4, L5, 4
    if L4, s_ctool
    push L6
    push L7
    pushi 2{lib}jog_ws             ; wxs .. zrots: robotlib 0x07f5, request 19 [axis, distance, 0 = straight]
    goto s_ccalled
s_ctool:
    push L6
    push L7
    pushi 2{lib}jog_ts             ; txs .. rolls: robotlib 0x0821, request 58
s_ccalled:
    pop L5
    ltc L4, L5, 0
    if L4, s_failed
    pushleag lastfin
    pushi 1
    builtinx mtime
    goto s_moving
s_manual:"""
    p["stop"] = f"""
    getg L6, aborted
    ifnot L6, t_live
    ret                             ; an abort is pending: onabort ends the move and says A
t_live:
    getg L6, mkind
    ifnot L6, t_manual              ; jogd-c: an online move stops with halt (robotsrv 0x0b2b: message 1999)
    ldi L6, 0
    putg L6, mkind
    pushi 0{lib}halt               ; the first call after the stop cause
    pop L5
    ltc L4, L5, 0
    ifnot L4, t_halted
    push L5
    pushi 1
    call report
t_halted:
    getg L6, aborted
    if L6, t_done                   ; halt was interrupted by the abort: onabort says A
    ifnot A3, t_cdash               ; the line as soon as halt has returned
    pushi 0
    pushleag s_dead
    pushi 2
    builtinx printf
    goto t_settle
t_cdash:
    pushi 0
    pushleag s_stop
    pushi 2
    builtinx printf
t_settle:
    pushi 0
    call settle                     ; then wait until the board reports the move finished
    getg L6, cart
    if L6, t_done
    pushi 0
    pushi 1
    call setonline                  ; mode 1: a vector's move is over: online(OFF) before the next jog
    pop L5
    ltc L4, L5, 0
    ifnot L4, t_done
    push L5
    pushi 1
    call report
    ret
t_manual:"""
    p["tprint"] = ""
    p["idle"] = """
    getg L6, ppend                  ; jogd-c: nothing to read, so now the deferred work, one piece, then read again
    ifnot L6, i_poll
    ldi L6, 0
    putg L6, ppend
    pushi 0
    call position
    goto loop
i_poll:
    getg L6, wpend
    ifnot L6, i_fin
    ldi L6, 0
    putg L6, wpend
    pushi 0
    call wpose
    goto loop
i_fin:
    pushi 0
    call finpoll
    pop L5
    if L5, loop"""
    p["gk"] = """
    goto b_none
b_cg:
    eqc L5, A3, 'g'
    ifnot L5, b_cw
    getg L6, moving                 ; jogd-c: `g` is the keep-alive of whatever move is in force
    ifnot L6, b_none
    pushleag last
    pushi 1
    builtinx mtime
    goto b_none
b_cw:
    eqc L5, A3, 'w'
    ifnot L5, b_none
    getg L6, mkind
    ifnot L6, b_cw_now
    ldi L6, 1
    putg L6, wpend                  ; during an online move: answered once no input is waiting
    goto b_none
b_cw_now:
    pushi 0
    call wpose"""
    p["report"] = """
    eqc L5, A3, EINTR_R
    getg L6, aborted
    lor L5, L5, L6
    ifnot L5, rp_print
    ldi L6, 1
    putg L6, aborted                ; jogd-c: an interrupted call (-4) or an abort pending: no E; onabort says A
    ret
rp_print:"""
    p["finish"] = """
    getg L6, cart
    ifnot L6, f_online
    pushi 0
    pushi 1
    call setonline                  ; online(OFF) again
    pop L5
    ltc L4, L5, 0
    ifnot L4, f_online
    push L5
    pushi 1
    call report
f_online:"""
    p["code"] = f"""

setonline:                          ; setonline(on): online(ON 1 / OFF 0) (robotlib 0x09e6, request 24); its result
    enter 10
    push A3
    pushi 1{lib}online
    pop L5
    retv L5
    retv 0

finpoll:                            ; finpoll(): robotisfinished at most every FIN_MS while an online move is in force;
    enter 10                        ; 1 when it made the call
    getg L6, mkind
    ifnot L6, fp_none
    getg L6, moving
    ifnot L6, fp_none
    pushleag now
    pushi 1
    builtinx mtime
    getg L5, now
    getg L6, lastfin
    sub L4, L5, L6
    getg L3, finms
    lt L3, L4, L3
    if L3, fp_none
    putg L5, lastfin
    pushi 0{lib}0x3e              ; robotisfinished: robotlib 0x09d4, request 20 FINISH_TEST: 1 finished, 0 not
    pop L5
    ltc L4, L5, 0
    ifnot L4, fp_result
    push L5
    pushi 1
    call report
    pushi 0
    pushi 1
    call stop                       ; an error: the whole stop
    goto fp_done
fp_result:
    eqc L4, L5, 1
    ifnot L4, fp_done
    ldi L6, 0                       ; the move has ended by itself: the line first
    putg L6, moving
    putg L6, mkind
    pushi 0
    pushleag s_stop
    pushi 2
    builtinx printf
    getg L6, cart
    if L6, fp_done
    pushi 0
    pushi 1
    call setonline
    pop L5
    ltc L4, L5, 0
    ifnot L4, fp_done
    push L5
    pushi 1
    call report
fp_done:
    ldi L5, 1
    retv L5
fp_none:
    ldi L5, 0
    retv L5
    retv 0

settle:                             ; settle(): after halt, robotisfinished until 1, at most SETTLE_MS (as robotsrv's
    enter 10                        ; disabled post-halt loop 0x00c3-0x00f4 polls message 1010 with delay(10))
    pushleag settle0
    pushi 1
    builtinx mtime
st_poll:
    pushi 0{lib}0x3e
    pop L5
    eqc L4, L5, 1
    if L4, st_done
    ltc L4, L5, 0
    ifnot L4, st_wait
    push L5
    pushi 1
    call report
    goto st_done
st_wait:
    pushleag now
    pushi 1
    builtinx mtime
    getg L6, now
    getg L5, settle0
    sub L6, L6, L5
    ldi L5, SETTLE_MS
    lt L4, L6, L5
    ifnot L4, st_done
    pushi 10
    pushi 1
    libcall syslib:delay
    chkpop
    goto st_poll
st_done:
    ret

gocommand:                          ; gocommand(kind, speed): a whole G (1) or L (2) line has arrived (gnums)
    enter 10
    ltc L5, A4, 1
    ifnot L5, go_speed
    pushi 0                         ; 00 is not a speed
    pushleag s_gbad
    pushi 2
    builtinx printf
    ret
go_speed:
    getg L6, maxspeed
    gt L5, A4, L6
    ifnot L5, go_clamped
    put L6, A4
go_clamped:
    pushleag last
    pushi 1
    builtinx mtime
    getg L6, moving
    ifnot L6, go_idle
    pushi 0
    pushi 1
    call stop                       ; whatever is moving: the whole stop first (halt first), as for a jog
go_idle:
    ldi L6, GOTO_AXIS
    putg L6, axis
    putg A4, speed
    putg A3, gkind
    push A4
    pushi 1
    call gostart
    ret

gostart:                            ; gostart(speed): one move() (G) or moves() (L) to the target in gnums, online
    enter 30
    getg L6, aborted
    if L6, go_none                  ; an abort is pending: no new move before onabort
    push A3
    pushi 1
    call vspeed
    pop L5
    ltc L4, L5, 0
    if L4, go_report
    getg L6, gkind
    eqc L5, L6, 2
    if L5, go_cart
    pushleag vloc                   ; ---- G: the commanded location (pos_get 2), axes 1..6 replaced
    pushi 2
    pushi 2{lib}pos_get
    pop L5
    ltc L4, L5, 0
    if L4, go_report
    pushleag vf
    pushleag vloc
    pushi 2
    builtin loc_pdata_get
    leag L7, vf
    leag L6, gnums
    ldi L5, 6
    memcopy L7, L6, L5              ; axes 7 and 8 as they are
    pushleag vf
    pushleag vloc
    pushi 2
    builtin loc_pdata_set
    pushleag vloc
    pushi 1
    builtin loc_re_check
    goto go_online
go_cart:                            ; ---- L: here() as a cloc (world), words 1..6 replaced by the numbers / 100
    pushi 1
    pushleag vloc
    pushi 2
    builtin loc_class_set
    pushleag vloc
    pushi 1{lib}here
    pop L5
    ltc L4, L5, 0
    if L4, go_report
    pushleag vf
    pushleag vloc
    pushi 2
    builtin loc_cdata_get
    ldi L8, 100
    itof L8, L8
    ldi L9, 0
go_conv:
    leag L7, gnums
    ldx L6, L7, L9
    itof L6, L6
    fdiv L6, L6, L8
    leag L7, vf
    stx L6, L7, L9
    addc L9, L9, 1
    ltc L5, L9, 6
    if L5, go_conv
    pushleag vf
    pushleag vloc
    pushi 2
    builtin loc_cdata_set
    pushleag vloc
    pushi 1
    builtin loc_re_check
go_online:
    getg L6, cart
    if L6, go_move
    pushi 1
    pushi 1
    call setonline                  ; modes 1, 2: online(ON) for this move only
    pop L5
    ltc L4, L5, 0
    if L4, go_report
go_move:
    ldi L6, 1
    putg L6, moving
    putg L6, mkind
    ldi L6, GOTO_FIN_MS
    putg L6, finms
    getg9 L20, vloc
    getg L6, gkind
    eqc L5, L6, 2
    if L5, go_moves
    push9 L20
    pushi 9{lib}move               ; joint-interpolated (robotlib 0x0702)
    goto go_called
go_moves:
    push9 L20
    pushi 9{lib}moves              ; straight line (robotlib 0x0714)
go_called:
    pop L5
    ltc L4, L5, 0
    if L4, go_failed
    pushleag last
    pushi 1
    builtinx mtime
    pushleag lastfin
    pushi 1
    builtinx mtime
    leag L5, s_pg
    getg L6, gkind
    eqc L4, L6, 2
    ifnot L4, go_said
    leag L5, s_pl
go_said:
    pushi 0
    push L5
    pushi 2
    builtinx printf
    ret
go_failed:
    push L5
    pushi 1
    call report
    pushi 0
    pushi 1
    call stop                       ; halt, `-`, and online OFF again in modes 1, 2
    ret
go_report:
    push L5
    pushi 1
    call report
go_none:
    ret

wpose:                              ; wpose(): W x y z yaw pitch roll, hundredths, from here() as a cloc (the L units)
    enter 20
    pushi 1
    pushleag wloc
    pushi 2
    builtin loc_class_set
    pushleag wloc
    pushi 1{lib}here
    pop L5
    ltc L4, L5, 0
    ifnot L4, w_got
    push L5
    pushi 1
    call report
    ret
w_got:
    pushleag wf
    pushleag wloc
    pushi 2
    builtin loc_cdata_get
    ldi L8, 100
    itof L8, L8
    ldlit L7, {_float_word(0.5)}
    ldi L9, 5
w_value:
    leag L6, wf
    ldx L5, L6, L9
    fmul L5, L5, L8
    ltc L3, L5, 0                   ; the float's sign bit
    if L3, w_neg
    fadd L5, L5, L7
    goto w_round
w_neg:
    fsub L5, L5, L7
w_round:
    ftoi L5, L5                     ; rounded half away from zero
    push L5 tag 1
    subc L9, L9, 1
    gec L6, L9, 0
    if L6, w_value
    pushi 6
    pushleag s_w
    pushi 14
    builtinx printf
    ret

on_sigabrt:                         ; on_sigabrt(sig): the handler. robotsrv sends 6 to its controlling process on a
    enter 5                         ; board error or arm power off (0x2013-0x2017) and on Ctrl-Z (0x2040-0x2044)
    ldi L4, 1
    putg L4, aborted
    ret

errget:                             ; errget(): robot_error_get(&errw) (robotlib 0x133c, request 55); its result
    enter 10
    pushleag errw
    pushi 1{lib}robot_error_get
    pop L5
    retv L5
    retv 0

onabort:                            ; onabort(): the move is over (the board stopped it, or robotsrv halted it);
    enter 10                        ; say so, then be ready again
    getg L7, mkind
    ldi L6, 0
    putg L6, aborted
    putg L6, moving
    putg L6, mkind
    pushi 0
    call errget
    pop L5
    eqc L4, L5, -42
    ifnot L4, ab_said               ; -42: the server's lock-out after Ctrl-Z or arm power off (robotsrv 0x22e3)
    pushi ABORT_WAIT_MS
    pushi 1
    libcall syslib:delay
    chkpop
    pushi 0
    call errget
    pop L5
ab_said:
    ltc L4, L5, 0
    if L4, ab_plain
    getg L6, errw
    ifnot L6, ab_plain
    neg L5, L6
    push L5
    pushi 1
    libcall syslib:str_error
    pop L5
    push L5 tag 19
    push L6 tag 1
    pushi 2
    pushleag s_aborte
    pushi 6
    builtinx printf                 ; A <the board's error> <its text>, as robotsrv prints them (0x1fcb-0x1fdd)
    goto ab_fix
ab_plain:
    pushi 0
    pushleag s_abort
    pushi 2
    builtinx printf
ab_fix:
    getg L6, inmanual
    ifnot L6, ab_online
    pushi 0
    pushi 1
    call setman                     ; manual mode off, as after a jog
    pop L5
ab_online:
    getg L6, cart
    if L6, ab_on
    ifnot L7, ab_done               ; mode 1: online was on only for a vector's move
    pushi 0
    pushi 1
    call setonline
    goto ab_result
ab_on:
    pushi 1
    pushi 1
    call setonline                  ; modes 3, 4: online(ON) again
ab_result:
    pop L5
    ltc L4, L5, 0
    ifnot L4, ab_done
    push L5
    pushi 1
    call report
ab_done:
    ret

vspeed:                             ; vspeed(s): speed_set(s) unless it is the speed in force; 0, or r < 0
    enter 10
    getg L6, speedset
    eq L5, L6, A3
    if L5, vs_same
    push A3
    pushi 1{lib}speed_set
    pop L5
    ltc L4, L5, 0
    if L4, vs_ret
    putg A3, speedset
vs_same:
    ldi L5, 0
vs_ret:
    retv L5
    retv 0

vcommand:                           ; vcommand(speed): a whole `@` command has arrived (vsg, vdg)
    enter 10
    ltc L5, A3, 1
    if L5, vc_done                  ; 00 is not a speed: ignored
    getg L6, maxspeed
    gt L5, A3, L6
    ifnot L5, vc_clamped
    put L6, A3
vc_clamped:
    ldi L7, 0
vc_value:
    leag L6, vdg
    ldx L5, L6, L7
    leag L6, vsg
    ldx L4, L6, L7
    ifnot L4, vc_signed
    neg L5, L5
vc_signed:
    leag L6, vc
    stx L5, L6, L7
    addc L7, L7, 1
    ltc L5, L7, 6
    if L5, vc_value
    pushleag last
    pushi 1
    builtinx mtime                  ; a vector feeds the dead-man as a jog command does
    getg L6, moving
    ifnot L6, vc_idle
    getg L6, axis
    eqc L5, L6, VEC_AXIS
    ifnot L5, vc_stop
    goto vc_done                    ; a vector move in force: any vector is its keep-alive
vc_stop:
    pushi 0
    pushi 1
    call stop                       ; a letter's move in force: stop it first
    goto vc_go
vc_idle:
    getg L6, axis
    eqc L5, L6, VEC_AXIS
    ifnot L5, vc_go
    getg L6, speed
    eq L5, L6, A3
    ifnot L5, vc_go
    ldi L7, 0
vc_same:
    leag L6, vc
    ldx L5, L6, L7
    leag L6, vl
    ldx L4, L6, L7
    eq L3, L5, L4
    ifnot L3, vc_go
    addc L7, L7, 1
    ltc L5, L7, 6
    if L5, vc_same
    goto vc_done                    ; the vector last started (it has ended or failed): ignored
vc_go:
    leag L7, vl
    leag L6, vc
    ldi L5, 6
    memcopy L7, L6, L5
    ldi L6, VEC_AXIS
    putg L6, axis
    putg A3, speed
    push A3
    pushi 1
    call vstart
vc_done:
    ret

vstart:                             ; vstart(speed): one move toward the vector vc
    enter 30
    getg L6, aborted
    if L6, v_done                   ; an abort is pending: no new move before onabort
    getg L6, mode
    eqc L5, L6, 2
    if L5, v_bad                    ; no vector in cylindrical mode
    push A3
    pushi 1
    call vspeed
    pop L5
    ltc L4, L5, 0
    if L4, v_report
    getg L6, mode
    eqc L5, L6, 1
    if L5, v_joint
    leag L7, vc                     ; ---- modes 3, 4: rotations must be 0 in this version
    ldp L6, L7, 3
    ldp L5, L7, 4
    or L6, L6, L5
    ldp L5, L7, 5
    or L6, L6, L5
    if L6, v_bad
    ldp L6, L7, 0
    mul L5, L6, L6
    ldp L6, L7, 1
    mul L6, L6, L6
    add L5, L5, L6
    ldp L6, L7, 2
    mul L6, L6, L6
    add L5, L5, L6
    ifnot L5, v_done                ; all zero: nothing
    leag L6, ktab
    ldx L8, L6, L5                  ; CART_MM / |v|
    ldi L9, 0
v_scale:
    leag L7, vc
    ldx L6, L7, L9
    itof L6, L6
    fmul L6, L6, L8
    leag L7, vd
    stx L6, L7, L9
    addc L9, L9, 1
    ltc L5, L9, 3
    if L5, v_scale
    pushi 1
    pushleag vloc
    pushi 2
    builtin loc_class_set           ; loc_cartesian (robotlib's here 0x0f88: class 1 -> motor_to_world)
    pushleag vloc
    pushi 1{lib}here               ; the commanded pose as a cloc (robotlib 0x0f79)
    pop L5
    ltc L4, L5, 0
    if L4, v_report
    getg L6, mode
    eqc L5, L6, 4
    if L5, v_tool
    pushleag vf                     ; ---- world: x, y, z (loc_cdata words 1..3) + the displacement
    pushleag vloc
    pushi 2
    builtin loc_cdata_get
    ldi L9, 0
v_add:
    leag L7, vf
    ldx L6, L7, L9
    leag L7, vd
    ldx L5, L7, L9
    fadd L6, L6, L5
    leag L7, vf
    stx L6, L7, L9
    addc L9, L9, 1
    ltc L5, L9, 3
    if L5, v_add
    pushleag vf
    pushleag vloc
    pushi 2
    builtin loc_cdata_set
    pushleag vloc
    pushi 1
    builtin loc_re_check
    goto v_move
v_tool:                             ; ---- tool: shift_t(&pose, 1, displacement cloc), as ash's tshift (0x1cc2-0x1cd3)
    leag L7, vdisp
    leag L6, vloc
    ldi L5, 9
    memcopy L7, L6, L5              ; the displacement gets the pose's header word
    pushleag vd
    pushleag vdisp
    pushi 2
    builtin loc_cdata_set
    pushleag vdisp
    pushi 1
    builtin loc_re_check
    getg9 L20, vdisp
    push9 L20 tag 5
    pushi 1
    pushleag vloc
    pushi 12{lib}shift_t            ; robotlib 0x13c7, request 57 (message 1967); the shifted pose back in vloc
    pop L5
    ltc L4, L5, 0
    if L4, v_report
v_move:
    ldi L6, 1
    putg L6, moving
    putg L6, mkind
    ldi L6, FIN_MS
    putg L6, finms
    getg9 L20, vloc
    push9 L20
    pushi 9{lib}moves              ; straight line (robotlib 0x0714, request 18, message 1002); a cloc only
    pop L5
    ltc L4, L5, 0
    if L4, v_failed
    goto v_started
v_joint:                            ; ---- mode 1: one move() with joints 1..6 advanced in proportion
    ldi L9, 0
    ldi L8, 0
v_max:
    leag L7, vc
    ldx L6, L7, L9
    ltc L5, L6, 0
    ifnot L5, v_abs
    neg L6, L6
v_abs:
    gt L5, L6, L8
    ifnot L5, v_maxed
    put L6, L8
v_maxed:
    addc L9, L9, 1
    ltc L5, L9, 6
    if L5, v_max
    ifnot L8, v_done                ; all zero: nothing
    mulc L8, L8, 100                ; the largest component gets VEC_DEG degrees
    pushleag vloc
    pushi 2
    pushi 2{lib}pos_get            ; POSITION_COMMANDED, as segtest's kind l
    pop L5
    ltc L4, L5, 0
    if L4, v_report
    pushleag vf
    pushleag vloc
    pushi 2
    builtin loc_pdata_get
    ldi L9, 0
v_joints:
    leag L7, vc
    ldx L6, L7, L9
    mulc L6, L6, VEC_DEG
    leag L7, cpd
    ldx L5, L7, L9
    mul L6, L6, L5
    div L6, L6, L8                  ; counts: c x VEC_DEG x cpd100 / (100 x max|c|)
    leag L7, vf
    ldx L5, L7, L9
    add L5, L5, L6
    stx L5, L7, L9
    addc L9, L9, 1
    ltc L5, L9, 6
    if L5, v_joints
    pushleag vf
    pushleag vloc
    pushi 2
    builtin loc_pdata_set
    pushleag vloc
    pushi 1
    builtin loc_re_check
    pushi 1
    pushi 1
    call setonline                  ; online(ON) for this move only (manual jogs need it OFF)
    pop L5
    ltc L4, L5, 0
    if L4, v_report
    ldi L6, 1
    putg L6, moving
    putg L6, mkind
    ldi L6, FIN_MS
    putg L6, finms
    getg9 L20, vloc
    push9 L20
    pushi 9{lib}move               ; joint-interpolated (robotlib 0x0702)
    pop L5
    ltc L4, L5, 0
    if L4, v_failed
v_started:
    pushleag last
    pushi 1
    builtinx mtime
    pushleag lastfin
    pushi 1
    builtinx mtime
    getg L6, speed                  ; +@ and the command as used: signs and digits as received, the speed clamped
    modc L5, L6, 10
    push L5 tag 1
    divc L5, L6, 10
    push L5 tag 1
    ldi L9, 5
v_echo:
    leag L7, vdg
    ldx L6, L7, L9
    push L6 tag 1
    leag L7, vsg
    ldx L6, L7, L9
    leag L5, s_plus
    ifnot L6, v_sign
    leag L5, s_minus
v_sign:
    push L5 tag 19
    subc L9, L9, 1
    gec L6, L9, 0
    if L6, v_echo
    pushi 14
    pushleag s_vline
    pushi 30
    builtinx printf
    ret
v_failed:
    push L5
    pushi 1
    call report
    pushi 0
    pushi 1
    call stop                       ; halt (and online OFF in mode 1), then -
    ret
v_bad:
    ldi L5, -22
v_report:
    push L5
    pushi 1
    call report
v_done:
    ret"""
    return p


def source(opts=Options()):
    """The jogd program as r3asm source."""
    if not 1 <= opts.mode <= 4 or not 1 <= opts.max_speed <= 99:
        raise ValueError("mode must be 1..4 and max_speed 1..99")
    speed_set = """
    getg L6, speedset
    eq L5, L6, A4
    if L5, s_go
    push A4
    pushi 1
    libcall robotlib:speed_set      ; the pendant sets the speed before man_move_start
    pop L5
    ltc L4, L5, 0
    if L4, s_failed
    putg A4, speedset
s_go:""" if opts.speed_set else ""
    restore = """
    getg L6, speedset
    getg L5, saved
    eq L4, L6, L5
    if L4, f_release                ; the speed is still the one found at start
    ltc L4, L5, 1
    if L4, f_release
    push L5
    pushi 1
    libcall robotlib:speed_set      ; the speed found at start, as the pendant puts it back
    pop L5
    ltc L4, L5, 0
    ifnot L4, f_release
    push L5
    pushi 1
    call report""" if opts.speed_set else ""
    speed_get = """
    pushleag saved
    pushi 1
    libcall robotlib:speed_get      ; the pendant reads the speed when it takes control (0x04ff)
    pop L5
    ltc L4, L5, 0
    if L4, setup_failed
    getg L6, saved
    putg L6, speedset               ; its G84: speed_set is called only for another speed""" if opts.speed_set else ""
    mv = opts.maxvel
    mv_data = """
s_mv:       string "MV {} {} {} {} {} {} {} {}\\n"
s_v:        string "V {} {}\\n"
vstate:     word 0                  ; `v`: 0 none, 1 `v`, 2 `v` and one digit
vdigit:     word 0
mvchanged:  word 0                  ; maxvels differ from the values read at start (or a set was tried)
mvsaved:    zeros 8                 ; maxvels_get() at start: floats, never written again
mvwork:     zeros 8""" if mv else ""
    mv_setup = """
    pushleag mvsaved
    pushi 1
    libcall robotlib:maxvels_get    ; request 9, op 0 (robotlib 0x04ba)
    pop L5
    ltc L4, L5, 0
    if L4, setup_failed
    leag L6, mvsaved""" + "".join(f"""
    ldp L5, L6, {k}
    push L5 tag 2""" for k in range(7, -1, -1)) + """
    pushi 8
    pushleag s_mv
    pushi 18
    builtinx printf""" if mv else ""
    mv_letter = """
    ldi L5, 0
    putg L5, vstate""" if mv else ""
    mv_digit = """
    getg L5, vstate
    ifnot L5, b_vno
    eqc L4, L5, 1
    ifnot L4, b_v2
    putg L6, vdigit
    ldi L5, 2
    putg L5, vstate
    goto b_none
b_v2:
    ldi L5, 0
    putg L5, vstate
    getg L5, vdigit
    mulc L5, L5, 10
    add L6, L5, L6
    push L6
    pushi 1
    call vel
    goto b_none
b_vno:""" if mv else ""
    after_p = "b_vel" if mv else "b_cg" if opts.cart else "b_none"
    c = _cart(opts)
    mv_byte = """
    goto b_none
b_vel:
    eqc L5, A3, 'v'
    ifnot L5, b_none
    ldi L5, 1
    putg L5, vstate                 ; `v` starts the two digits, and ends any half-received jog command
    ldi L5, 0
    putg L5, pstate""" if mv else ""
    mv_stop = """
    pushi 0
    call mvrestore                  ; the values read at start back, once the joint is stopped""" if mv else ""
    mv_finish = """
    pushi 0
    call mvrestore""" if mv else ""
    mv_code = """

vel:                                ; vel(pct): the moving joint's maxvels entry to pct % of the value read at start
    enter 10
    getg L6, moving
    ifnot L6, v_done                ; only while a move is in force
    ltc L5, A3, 1
    if L5, v_done                   ; 00: nothing
    leag L7, mvwork
    leag L6, mvsaved
    ldi L5, 8
    memcopy L7, L6, L5              ; every other entry as read at start
    getg L6, axis
    ltc L5, L6, 0
    ifnot L5, v_abs
    neg L6, L6
v_abs:
    subc L6, L6, 1
    leag L5, mvsaved
    ldx L4, L5, L6
    itof L3, A3
    fmul L4, L4, L3
    ldi L3, 100
    itof L3, L3
    fdiv L4, L4, L3                 ; start value x pct / 100: pct is 1..99, so never above it
    leag L5, mvwork
    stx L4, L5, L6
    ldi L5, 1
    putg L5, mvchanged              ; before the call: any attempt is followed by a restore
    pushleag mvwork
    pushi 1
    libcall robotlib:maxvels_set    ; request 9, op 1, forwarded at once to the servo board (robotsrv 0x04c6)
    pop L5
    ltc L2, L5, 0
    ifnot L2, v_ok
    push L5
    pushi 1
    call report
    ret
v_ok:
    push L4 tag 2
    push A3 tag 1
    pushi 2
    pushleag s_v
    pushi 6
    builtinx printf
v_done:
    ret

mvrestore:                          ; mvrestore(): maxvels back to the values read at start, if changed
    enter 10
    getg L6, mvchanged
    ifnot L6, r_done
    ldi L6, 0
    putg L6, mvchanged
    leag L7, mvwork
    leag L6, mvsaved
    ldi L5, 8
    memcopy L7, L6, L5              ; a copy: maxvels_set writes the board's answer back into its array
    pushleag mvwork
    pushi 1
    libcall robotlib:maxvels_set
    pop L5
    ltc L4, L5, 0
    ifnot L4, r_done
    push L5
    pushi 1
    call report
r_done:
    ret""" if mv else ""
    return f"""; jogd: continuous jog from 3-byte console commands. Built by a465/jogprog.py; see its docstring.
.build {opts.build_time:#010x}
.entry main
.lib syslib {r3asm.SYSLIB.slot} {r3asm.SYSLIB.build_time:#010x} {r3asm.SYSLIB.signature:#010x} {r3asm.SYSLIB.word1}
.lib robotlib {r3asm.ROBOTLIB.slot} {r3asm.ROBOTLIB.build_time:#010x} {r3asm.ROBOTLIB.signature:#010x} {r3asm.ROBOTLIB.word1}
.equ DEADMAN_MS {opts.deadman_ms}
.equ DEADMAN_LONG_MS {opts.deadman_long_ms}
.equ POLL_MS {opts.poll_ms}
.equ OPEN_FLAGS {opts.open_flags}
.equ POS_TYPE {opts.pos_type}
.equ NO_DATA -35                    ; -EAGAIN: reads() on a non-blocking fd with nothing to read{c['equ']}

.data
s_cin:      string {r3asm.quote_bytes(opts.input_path.encode())}
s_ready:    string "jogd ready\\n"
s_line:     string 8 "+A00\\n"     ; word 1 (the 4 bytes after the header word) set per start
s_stop:     string "-\\n"
s_dead:     string "T\\n"
s_err:      string "E {{}} {{}}\\n"
s_pos:      string "P {{}} {{}} {{}} {{}} {{}} {{}}\\n"
s_badargs:  string "{{}}: invalid command line arguments.\\n"
s_usage:    string "Usage: {{}} [mode [maxspeed [s|r]]]\\n  r: speed change without a stop (does not change the speed here)\\n"
s_r:        string "r"
s_s:        string "s"
inbuf:      string 4 ""            ; reads() puts the byte here
fd:         word 0
mode:       word {opts.mode}
maxspeed:   word {opts.max_speed}
restart:    word {int(opts.restart)}            ; 1: variant r, 0: variant s
moving:     word 0                  ; a move is in force (the pendant's G88)
inmanual:   word 0                  ; manual mode is on (G144) ...
curmode:    word 0                  ; ... in this mode (G145)
rmode:      word 0                  ; robot_mode_get()'s answer
axis:       word 0                  ; the command in force: signed joint (0: none) ...
speed:      word 0                  ; ... and its speed
pstate:     word 0                  ; parser: 0 no letter, 1 letter, 2 letter and one digit
paxis:      word 0
pdigit:     word 0
speedset:   word -1                 ; the last speed given to speed_set, -1 none
saved:      word 0                  ; speed_get() at start
last:       word 0, 0               ; mtime() of the last command or keep-alive
now:        word 0, 0
loc:        zeros 9                 ; pos_get's location
pul1:       word 0                  ; loc_pdata_get's 8 counts, joint 1 first
pul2:       word 0
pul3:       word 0
pul4:       word 0
pul5:       word 0
pul6:       word 0
            zeros 2                 ; axes 7 and 8{mv_data}{c['data']}

.code
main:
    enter 10
    pushi 0                         ; ---- jogd [mode [maxspeed [r|s]]]
    builtin argc
    pop L6
    gtc L5, L6, 4
    if L5, usage
    ltc L5, L6, 2
    if L5, args_done
    pushi 1
    pushi 1
    builtin argv
    pop L7
    push L7
    pushlea L8
    pushi 2
    builtin str_to_int              ; as homezc reads its argument
    pop L5
    ltc L4, L5, 0
    ltc L3, L8, 1
    lor L4, L4, L3
    gtc L3, L8, 4
    lor L4, L4, L3
    if L4, usage
    putg L8, mode
    ltc L5, L6, 3
    if L5, args_done
    pushi 2
    pushi 1
    builtin argv
    pop L7
    push L7
    pushlea L8
    pushi 2
    builtin str_to_int
    pop L5
    ltc L4, L5, 0
    ltc L3, L8, 1
    lor L4, L4, L3
    gtc L3, L8, 99
    lor L4, L4, L3
    if L4, usage
    putg L8, maxspeed
    ltc L5, L6, 4
    if L5, args_done
    pushi 3
    pushi 1
    builtin argv
    pop L7
    leag L5, s_r
    streq L4, L7, L5                ; as xzero compares its -v
    ifnot L4, arg_s
    ldi L5, 1
    putg L5, restart
    goto args_done
arg_s:
    leag L5, s_s
    streq L4, L7, L5
    ifnot L4, usage
    ldi L5, 0
    putg L5, restart
    goto args_done
usage:
    pushi 0
    pushi 1
    builtin argv
    pop L6
    push L6 tag 19
    pushi 1
    pushleag s_badargs
    pushi 2
    pushi 5
    builtinx fprintf
    pushi 0
    pushi 1
    builtin argv
    pop L6
    push L6 tag 19
    pushi 1
    pushleag s_usage
    pushi 2
    pushi 5
    builtinx fprintf
    pushi 1
    pushi 1
    builtinx exit
args_done:
    pushi 0                         ; ---- open the input without blocking, as fastacid does
    pushi OPEN_FLAGS
    pushleag s_cin
    pushleag fd
    pushi 4
    builtin open
    pop L5
    ltc L4, L5, 0
    ifnot L4, opened
    push L5
    pushi 1
    call report
    pushi 1
    pushi 1
    builtinx exit
opened:
    pushi 0                         ; ---- take control
    libcall robotlib:ctl_get
    pop L5
    ltc L4, L5, 0
    ifnot L4, have_control
    push L5
    pushi 1
    call report
    pushi 1
    pushi 1
    builtinx exit
have_control:{speed_get}{mv_setup}{c['setup']}
    goto ready
setup_failed:
    push L5
    pushi 1
    call report
    pushi 0
    libcall robotlib:ctl_rel
    pop L5
    pushi 1
    pushi 1
    builtinx exit
ready:
    pushi 0
    pushleag s_ready
    pushi 2
    builtinx printf
loop:                               ; ---- one byte, or a short wait{c['loop']}
    pushi 1
    pushleag inbuf
    getg L6, fd
    push L6
    pushi 3
    builtin reads
    pop L5
    gtc L4, L5, 0
    ifnot L4, no_byte
    pushi 0
    pushleag inbuf
    pushi 2
    builtin str_chr_get
    pop L6
    push L6
    pushi 1
    call byte
    pop L4
    if L4, quit
    ldi L4, DEADMAN_LONG_MS         ; after a byte: read on (bytes may be waiting); only the long limit
    goto deadman
no_byte:
    eqc L4, L5, NO_DATA
    if L4, idle
    eqc L4, L5, 0
    if L4, idle
    push L5                         ; the input failed: stop, release, exit 1
    pushi 1
    call report
    pushi 0
    pushi 1
    call stop
    pushi 1
    pushi 1
    call finish
idle:{c['idle']}
    pushi POLL_MS
    pushi 1
    libcall syslib:delay
    chkpop
    ldi L4, DEADMAN_MS              ; nothing to read: the dead-man proper
deadman:                            ; L4: the limit in ms
    getg L5, moving
    ifnot L5, loop
    pushleag now
    pushi 1
    builtinx mtime
    getg L5, now
    getg L6, last
    sub L3, L5, L6
    ge L2, L3, L4
    ifnot L2, loop
    pushi 1
    pushi 1
    call stop
    ldi L4, 0
    putg L4, axis                   ; no command in force after the dead-man
    goto loop
quit:
    pushi 0
    pushi 1
    call stop
    pushi 0
    pushi 1
    call finish
    retv 0

byte:                               ; byte(c): one input byte; returns 1 for q
    enter 10{c['vbyte']}
    gec L5, A3, 'A'
    lec L4, A3, 'F'
    land L5, L5, L4
    ifnot L5, b_lower
    subc L6, A3, 64                 ; 'A' is joint 1
    goto b_letter
b_lower:
    gec L5, A3, 'a'
    lec L4, A3, 'f'
    land L5, L5, L4
    ifnot L5, b_digit
    subc L6, A3, 96
    neg L6, L6                      ; 'a' is joint 1, negative
b_letter:
    putg L6, paxis
    ldi L5, 1
    putg L5, pstate{mv_letter}
    goto b_none
b_digit:
    gec L5, A3, '0'
    lec L4, A3, '9'
    land L5, L5, L4
    ifnot L5, b_dot
    subc L6, A3, '0'{mv_digit}
    getg L5, pstate
    eqc L4, L5, 1
    ifnot L4, b_second
    putg L6, pdigit
    ldi L5, 2
    putg L5, pstate
    goto b_none
b_second:
    eqc L4, L5, 2
    ifnot L4, b_none                ; a digit with no letter before it
    ldi L5, 0
    putg L5, pstate
    getg L5, pdigit
    mulc L5, L5, 10
    add L6, L5, L6
    push L6
    getg L5, paxis
    push L5
    pushi 2
    call command
    goto b_none
b_dot:
    eqc L5, A3, '.'
    ifnot L5, b_quit
    ldi L5, 0
    putg L5, pstate
    pushi 0
    pushi 1
    call stop
    ldi L5, 0
    putg L5, axis
    goto b_none
b_quit:
    eqc L5, A3, 'q'
    ifnot L5, b_pos
    ldi L5, 1
    retv L5
b_pos:
    eqc L5, A3, 'p'
    ifnot L5, {after_p}
    pushi 0
    call position                   ; no pstate change, no dead-man feed{mv_byte}{c['gk']}
b_none:
    ldi L5, 0
    retv L5
    retv 0

command:                            ; command(axis, speed): a whole command
    enter 10
    ltc L5, A4, 1
    if L5, c_done                   ; 00 is not a speed: ignored
    getg L6, maxspeed
    gt L5, A4, L6
    ifnot L5, c_clamped
    put L6, A4
c_clamped:
    pushleag last
    pushi 1
    builtinx mtime                  ; a command or keep-alive feeds the dead-man
    getg L6, axis
    eq L4, L6, A3                   ; same joint and direction as the command in force
    getg L6, speed
    eq L3, L6, A4
    land L2, L4, L3{c['keep']}
    if L2, c_done                   ; the command in force: keep-alive
    getg L6, moving
    ifnot L6, c_start
    ifnot L4, c_stop                ; another joint or direction: stop first
    getg L6, restart
    if L6, c_start                  ; variant r: start again without stopping
c_stop:
    pushi 0
    pushi 1
    call stop
c_start:
    push A4
    push A3
    pushi 2
    call start
c_done:
    ret

start:                              ; start(axis, speed): one jog as the pendant starts it (0x0d40, 0x06d7)
    enter 10
    putg A3, axis
    putg A4, speed{speed_set}{c['start']}
    getg L6, moving
    if L6, s_move                   ; variant r: manual mode is on already
    pushi 1
    pushi 1
    call setman                     ; man_mode_set(mode) before each jog (pendant 0x06f2)
    pop L5
    if L5, s_done                   ; refused (reported): no move
s_move:
    ldi L6, 1
    putg L6, moving                 ; set before the start, as the pendant sets G88 (0x06f9)
    push A4
    push A3
    pushi 2
    libcall robotlib:man_move_start
    pop L5
    ltc L4, L5, 0
    ifnot L4, s_moving
s_failed:
    push L5
    pushi 1
    call report
    pushi 0
    pushi 1
    call stop                       ; the whole stop sequence (it acts only if a move was in force)
s_done:
    ret
s_moving:
    pushleag last
    pushi 1
    builtinx mtime                  ; the time spent in the controller's calls is not the PC's
    ltc L5, A3, 0                   ; the line +Xnn: '+', the letter, two digits
    if L5, s_neg
    addc L6, A3, 64
    goto s_letter
s_neg:
    neg L6, A3
    addc L6, L6, 96
s_letter:
    shlc L6, L6, 8
    ldi L5, '+'
    or L6, L6, L5
    divc L5, A4, 10
    addc L5, L5, '0'
    shlc L5, L5, 16
    or L6, L6, L5
    modc L5, A4, 10
    addc L5, L5, '0'
    shlc L5, L5, 24
    or L6, L6, L5
    leag L5, s_line
    stp L6, L5, 1                   ; the string's first four bytes
    pushi 0
    pushleag s_line
    pushi 2
    builtinx printf
    ret

stop:                               ; stop(deadman): as the pendant's 0x07d9; prints E..., then - or T
    enter 10
    getg L6, moving
    ifnot L6, t_done
    ldi L6, 0
    putg L6, moving{c['stop']}
    getg L6, axis
    ltc L5, L6, 0
    ifnot L5, t_abs
    neg L6, L6
t_abs:
    push L6
    pushi 1
    libcall robotlib:man_move_stop
    pop L5
    ltc L4, L5, 0
    ifnot L4, t_said
    push L5
    pushi 1
    call report
    pushi 0
    libcall robotlib:halt           ; as the pendant does when man_move_stop fails
    pop L5
    ltc L4, L5, 0
    ifnot L4, t_said
    push L5
    pushi 1
    call report
t_said:{mv_stop}
    pushi 0
    pushi 1
    call setman                     ; man_mode_set(0) after each jog (pendant 0x07f3)
    pop L5{c['tprint']}
    ifnot A3, t_dash
    pushi 0
    pushleag s_dead
    pushi 2
    builtinx printf
    ret
t_dash:
    pushi 0
    pushleag s_stop
    pushi 2
    builtinx printf
t_done:
    ret

setman:                             ; setman(on): the pendant's 0x05b2. man_mode_set(mode) or (0)
    enter 10                        ; unless that is in force already; 0, or the result r < 0
    getg L6, inmanual
    eq L5, L6, A3
    ifnot L5, m_set
    ifnot A3, m_same                ; off already
    getg L6, curmode
    getg L5, mode
    eq L4, L6, L5
    if L4, m_same                   ; on already, in this mode
m_set:
    ldi L6, 0
    ifnot A3, m_call
    getg L6, mode
m_call:
    push L6
    pushi 1
    libcall robotlib:man_mode_set
    pop L5
    ltc L4, L5, 0
    if L4, m_failed
    putg A3, inmanual
    putg L6, curmode
m_same:
    ldi L5, 0
    retv L5
m_failed:
    push L5
    pushi 1
    call report
    pushleag rmode
    pushi 1
    libcall robotlib:robot_mode_get ; as the pendant asks after a refusal (0x0625)
    pop L4
    ltc L3, L4, 0
    if L3, m_ret
    getg L4, rmode
    eqc L3, L4, 2                   ; MODE_MANUAL
    putg L3, inmanual
m_ret:
    retv L5
    retv 0

position:                           ; position(): print P and joints 1..6's encoder counts
    enter 10
    pushleag loc
    pushi POS_TYPE
    pushi 2
    libcall robotlib:pos_get        ; as ash's Actual Position display and diag/xzero read it
    pop L5
    ltc L4, L5, 0
    ifnot L4, p_got
    push L5
    pushi 1
    call report
    ret
p_got:
    pushleag pul1
    pushleag loc
    pushi 2
    builtin loc_pdata_get           ; no result: nothing to pop (as in ash and xzero)
    getg L6, pul6
    push L6 tag 1
    getg L6, pul5
    push L6 tag 1
    getg L6, pul4
    push L6 tag 1
    getg L6, pul3
    push L6 tag 1
    getg L6, pul2
    push L6 tag 1
    getg L6, pul1
    push L6 tag 1
    pushi 6
    pushleag s_pos
    pushi 14
    builtinx printf
    ret

report:                             ; report(r): print "E <r> <text>" for a result r < 0
    enter 10{c['report']}
    neg L5, A3
    push L5
    pushi 1
    libcall syslib:str_error
    pop L6
    push L6 tag 19
    push A3 tag 1
    pushi 2
    pushleag s_err
    pushi 6
    builtinx printf
    ret

finish:                             ; finish(code): manual mode off if still on, the speed back, release, exit
    enter 10
    pushi 0
    pushi 1
    call setman
    pop L5{mv_finish}{c['finish']}{restore}
f_release:
    pushi 0
    libcall robotlib:ctl_rel
    pop L5
    ltc L4, L5, 0
    ifnot L4, f_exit
    push L5
    pushi 1
    call report
f_exit:
    push A3
    pushi 1
    builtinx exit
    retv 0{mv_code}{c['code']}
"""


def build(opts=Options()):
    """The program file's bytes."""
    return r3asm.assemble(source(opts), "jogd.r3s")


def listing(image, name="jogd"):
    """The r3dis listing of a built program, with syslib and robotlib routine names."""
    robot = {n: name for name, n in r3asm.robotlib_names().items()}
    mod = r3dis.Module(image, name)
    lines = r3dis.listing(mod, {"syslib": dict(r3dis.SYSLIB), "robotlib": robot})
    return "\n".join(lines + ["; data segment"] + r3dis.data_listing(mod)) + "\n"


BUILDS = {"jogd": Options(), "jogd-nospeedset": replace(Options(), speed_set=False),
          "jogd-v": replace(Options(), maxvel=True), "jogd-c": replace(Options(), cart=True)}


def write(out_dir=OUT, builds=None):
    """Write each build NAME (default BUILDS) as NAME, NAME.r3s and NAME.lst; {name: bytes}."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = {}
    for name, opts in (builds or BUILDS).items():
        image = build(opts)
        (out_dir / name).write_bytes(image)
        (out_dir / f"{name}.r3s").write_text(source(opts))
        (out_dir / f"{name}.lst").write_text(listing(image, name))
        out[name] = image
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(OUT), help="directory for the builds (each with .r3s and .lst)")
    args = ap.parse_args(argv)
    for name, image in write(args.out).items():
        print(f"{args.out}/{name}: {len(image)} bytes")


if __name__ == "__main__":
    main()
