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

"""
import argparse
from dataclasses import dataclass, replace
from pathlib import Path

from a465 import r3asm, r3dis

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "controller" / "programs"
BUILD_TIME = 0x6B057C40          # 2026-11-24 12:00 UTC; the firmware is not known to check it

# robotlib routine numbers (controller/robotlib-functions.txt)
CTL_GET, CTL_REL, POS_GET, HALT, SPEED_GET, SPEED_SET = 0x02, 0x04, 0x0E, 0x21, 0x43, 0x44
MAN_MODE_SET, MAN_MOVE_START, MAN_MOVE_STOP, ROBOT_MODE_GET = 0x52, 0x53, 0x54, 0x55
STR_ERROR, DELAY = 0x07, 0x14    # syslib


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
    build_time: int = BUILD_TIME


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
.equ NO_DATA -35                    ; -EAGAIN: reads() on a non-blocking fd with nothing to read

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
            zeros 2                 ; axes 7 and 8

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
have_control:{speed_get}
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
loop:                               ; ---- one byte, or a short wait
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
idle:
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
    enter 10
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
    putg L5, pstate
    goto b_none
b_digit:
    gec L5, A3, '0'
    lec L4, A3, '9'
    land L5, L5, L4
    ifnot L5, b_dot
    subc L6, A3, '0'
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
    ifnot L5, b_none
    pushi 0
    call position                   ; no pstate change, no dead-man feed
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
    land L2, L4, L3
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
    putg A4, speed{speed_set}
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
    putg L6, moving
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
t_said:
    pushi 0
    pushi 1
    call setman                     ; man_mode_set(0) after each jog (pendant 0x07f3)
    pop L5
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
    enter 10
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
    pop L5{restore}
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
    retv 0
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


BUILDS = {"jogd": Options(), "jogd-nospeedset": replace(Options(), speed_set=False)}


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
