"""jogd: a program that runs ON the controller and jogs the arm from short commands on its console.

    python3 -m a465.jogprog            build controller/programs/jogd (+ .r3s source, .lst listing)

On the controller (after `scripts/a465-putfile controller/programs/jogd /tmp/jogd`):

    /tmp/jogd [mode [maxspeed [r|s]]]          defaults: 1 (joint), 25, s

Why: the PC's gamepad jog types a whole command per move, and each is a separate start-stop
move. jogd keeps one continuous move going with the routines the teach pendant uses and changes
or stops it on a 3-byte command, so the PC only has to send keep-alives.

Protocol (standard input, printable ASCII only: fastacid removes 0x01, 0x05, 0x07, 0x16, 0x1a)

    A..F nn   joint 1..6 positive at nn percent (01..99, clamped to maxspeed); a..f negative
    .         stop the move               q   stop, leave manual mode, release control, exit 0
    anything else is ignored; a letter always starts a new command, so a lost byte resyncs.

    The command in force again is a keep-alive. Another joint or direction: stop, then start. The
    same joint and direction at another speed: with `r`, man_move_start again without stopping;
    with `s` (default), stop then start. No command or keep-alive for DEADMAN_MS (300) while a
    move is in force: the move is stopped and `T` printed; the next command starts afresh.

    Output, one line each: `jogd ready`; `+A07` a move started (the command as used, after the
    clamp); `-` a move stopped; `T` stopped by the dead-man; `E <r> <text>` a library call
    returned r < 0 (text from syslib str_error(-r), as homezc words its errors). A failed start
    leaves no move in force; repeats of that same command are then treated as keep-alives (no
    retry storm) until a different command or `.`. A failed man_move_stop is followed by halt().

What it calls, and where each piece comes from (listings by a465/r3dis.py)

- ctl_get (robotlib 0x02) to take control and ctl_rel (0x04) to give it back: diag/xzero.
- man_mode_set (0x52) once at start with the mode; man_mode_set(0) at exit: the pendant
  (bin/pendant 0x05b2) calls it with 1..4 from its frame setting before it starts a jog and with 0
  when the jog stops; its display routine (0x0bfd) shows that setting as JOINT, CYL, WORLD, TOOL.
  robotlib's own constants (type man_mode_t) agree: MMODE_OFF 0, MMODE_JOINT 1, MMODE_CYLIND 2,
  MMODE_WORLD 3, MMODE_TOOL 4 (and its strings 'Off' 'Joint' 'Cylindrical' 'World' 'Tool'). Only
  1..4 are accepted as the argument.
- man_move_start (0x53) with (signed joint, speed) and man_move_stop (0x54) with the joint: the
  pendant's 0x06d7 and 0x07d9; halt (0x21) when the stop fails: pendant 0x07e8. The robot server
  (sbin/robotsrv, handler of message 42 at 0x12e6) refuses a start unless in manual mode (-13)
  or unless 1 <= |joint| <= 8 (-22), takes the sign as the direction and copies the speed.
- speed_get (0x43) at start and speed_set (0x44) before a start at a new speed, the old speed put
  back at exit: the pendant does exactly this (0x03d4, 0x0474, 0x04ff) so that the robot's speed
  setting equals the speed man_move_start is given. Whether the motion code uses the setting, the
  argument or both is not in any file here (the server passes the move to the servo board,
  /dev/xputer1); the build option speed_set=False leaves speed_set out.
- The input is opened again as "/dev/cin" with flags 5 = O_RDONLY|O_NONBLOCK (syslib's constants)
  and read one byte at a time with reads(); -35 (-EAGAIN) means nothing has arrived, and the
  program then waits with syslib delay(10). That is how sbin/fastacid reads /dev/cout (0x0195:
  open flags 5, reads, -35, delay(10)); /dev/cin is the shell's standard input (bin/shell 0x0095
  opens it and dup()s it to fd 0). The dead-man clock is mtime(&t), whose first word syslib's own
  delay() and TimeStampEnd use as milliseconds.

Not known, so made build options: whether the controller's printf output appears without delay
(no flush call exists in syslib's names), how long the library calls take, and the two speed
change variants (a runtime argument).
"""
import argparse
from dataclasses import dataclass, replace
from pathlib import Path

from a465 import r3asm, r3dis

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "controller" / "programs"
BUILD_TIME = 0x6B057C40          # 2026-11-24 12:00 UTC; the firmware is not known to check it

# robotlib routine numbers (controller/robotlib-functions.txt)
CTL_GET, CTL_REL, HALT, SPEED_GET, SPEED_SET = 0x02, 0x04, 0x21, 0x43, 0x44
MAN_MODE_SET, MAN_MOVE_START, MAN_MOVE_STOP = 0x52, 0x53, 0x54
STR_ERROR, DELAY = 0x07, 0x14    # syslib


@dataclass(frozen=True)
class Options:
    mode: int = 1                # MMODE_JOINT
    max_speed: int = 25
    restart: bool = False        # default speed-change variant: False = s (stop, start), True = r
    deadman_ms: int = 300
    poll_ms: int = 10
    input_path: str = "/dev/cin"
    open_flags: int = 5          # O_RDONLY | O_NONBLOCK
    speed_set: bool = True
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
    speed_get = """
    pushleag saved
    pushi 1
    libcall robotlib:speed_get      ; the pendant reads the speed when it takes control
    pop L5
    ltc L4, L5, 0
    if L4, setup_failed""" if opts.speed_set else ""
    return f"""; jogd: continuous jog from 3-byte console commands. Built by a465/jogprog.py; see its docstring.
.build {opts.build_time:#010x}
.entry main
.lib syslib {r3asm.SYSLIB.slot} {r3asm.SYSLIB.build_time:#010x} {r3asm.SYSLIB.signature:#010x} {r3asm.SYSLIB.word1}
.lib robotlib {r3asm.ROBOTLIB.slot} {r3asm.ROBOTLIB.build_time:#010x} {r3asm.ROBOTLIB.signature:#010x} {r3asm.ROBOTLIB.word1}
.equ DEADMAN_MS {opts.deadman_ms}
.equ POLL_MS {opts.poll_ms}
.equ OPEN_FLAGS {opts.open_flags}
.equ NO_DATA -35                    ; -EAGAIN: reads() on a non-blocking fd with nothing to read

.data
s_cin:      string {r3asm.quote_bytes(opts.input_path.encode())}
s_ready:    string "jogd ready\\n"
s_line:     string 8 "+A00\\n"     ; word 1 (the 4 bytes after the header word) set per start
s_stop:     string "-\\n"
s_dead:     string "T\\n"
s_err:      string "E {{}} {{}}\\n"
s_badargs:  string "{{}}: invalid command line arguments.\\n"
s_usage:    string "Usage: {{}} [mode [maxspeed [r|s]]]\\n"
s_r:        string "r"
s_s:        string "s"
inbuf:      string 4 ""            ; reads() puts the byte here
fd:         word 0
mode:       word {opts.mode}
maxspeed:   word {opts.max_speed}
restart:    word {int(opts.restart)}            ; 1: variant r, 0: variant s
moving:     word 0                  ; a move is in force
axis:       word 0                  ; the command in force: signed joint (0: none) ...
speed:      word 0                  ; ... and its speed
pstate:     word 0                  ; parser: 0 no letter, 1 letter, 2 letter and one digit
paxis:      word 0
pdigit:     word 0
speedset:   word -1                 ; the last speed given to speed_set, -1 none
saved:      word 0                  ; speed_get() at start
last:       word 0, 0               ; mtime() of the last command or keep-alive
now:        word 0, 0

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
    getg L6, mode
    push L6
    pushi 1
    libcall robotlib:man_mode_set
    pop L5
    ltc L4, L5, 0
    ifnot L4, ready
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
deadman:
    getg L4, moving
    ifnot L4, loop
    pushleag now
    pushi 1
    builtinx mtime
    getg L5, now
    getg L6, last
    sub L4, L5, L6
    ldi L5, DEADMAN_MS
    ge L3, L4, L5
    ifnot L3, loop
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
    ifnot L5, b_none
    ldi L5, 1
    retv L5
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

start:                              ; start(axis, speed): start a move; prints +Xnn or E
    enter 10
    putg A3, axis
    putg A4, speed{speed_set}
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
    call stop                       ; acts only if a move was in force (a failed restart)
    ret
s_moving:
    ldi L6, 1
    putg L6, moving
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

stop:                               ; stop(deadman): stop the move in force; prints E..., then - or T
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

finish:                             ; finish(code): leave manual mode, put the speed back, release, exit
    enter 10
    pushi 0                         ; MMODE_OFF, as the pendant does when a jog ends
    pushi 1
    libcall robotlib:man_mode_set
    pop L5
    ltc L4, L5, 0
    ifnot L4, f_speed
    push L5
    pushi 1
    call report
f_speed:
    getg L6, speedset
    ltc L4, L6, 0
    if L4, f_release                ; speed_set never called
    getg L6, saved
    ltc L4, L6, 1
    if L4, f_release
    push L6
    pushi 1
    libcall robotlib:speed_set
    pop L5
    ltc L4, L5, 0
    ifnot L4, f_release
    push L5
    pushi 1
    call report
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


def listing(image):
    """The r3dis listing of a built program, with syslib and robotlib routine names."""
    robot = {n: name for name, n in r3asm.robotlib_names().items()}
    mod = r3dis.Module(image, "jogd")
    lines = r3dis.listing(mod, {"syslib": dict(r3dis.SYSLIB), "robotlib": robot})
    return "\n".join(lines + ["; data segment"] + r3dis.data_listing(mod)) + "\n"


def write(out_dir=OUT, opts=Options()):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    image = build(opts)
    (out_dir / "jogd").write_bytes(image)
    (out_dir / "jogd.r3s").write_text(source(opts))
    (out_dir / "jogd.lst").write_text(listing(image))
    return image


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(OUT), help="directory for jogd, jogd.r3s and jogd.lst")
    ap.add_argument("--no-speed-set", action="store_true", help="leave speed_set/speed_get out")
    ap.add_argument("--deadman-ms", type=int, default=Options.deadman_ms)
    args = ap.parse_args(argv)
    opts = replace(Options(), speed_set=not args.no_speed_set, deadman_ms=args.deadman_ms)
    image = write(args.out, opts)
    print(f"{args.out}/jogd: {len(image)} bytes")


if __name__ == "__main__":
    main()
