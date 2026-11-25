"""segtest: a bench program that runs ON the controller and times short moves issued back to back.

    python3 -m a465.segprog            build controller/programs/segtest (+ .r3s source, .lst listing)

On the controller (after `scripts/a465-putfile controller/programs/segtest /tmp/segtest`):

    /tmp/segtest <kind> <axis> <step> <count> <speed1> <speed2>

    kind  j  joint(axis, step degrees), relative            (robotlib 0x1a)
          m  motor(axis, step pulses, MSTOP_NONE), relative  (robotlib 0x1b)
          l  move() to precision locations: the commanded position (pos_get type 2) with that one
             axis advanced by step degrees per move          (robotlib 0x2c)
          c  the same locations as one continuous path: cpath() (robotlib 0x39)

The question it answers: do consecutive short moves issued by a program run through without
stopping, and does a speed change between two queued moves take effect.

Sequence: take control (ctl_get); speed_get (noted); speed_set(speed1); [l, c: pos_get(2) and the
locations]; online(ON); `count` moves back to back, speed_set(speed2) after the first half
((count + 1) // 2 moves; for c: right after cpath returns); then, until robotisfinished() says
done or POLL_LIMIT_MS (6 s) have passed, position lines; finish() (waits for the motion to end);
online(OFF); the noted speed back; ctl_rel; `done`; exit 0.

Output lines (times are ms since the program started, from syslib's mtime clock):
    I <kind> <axis> <step> <count> <speed1> <speed2> <pulses per step, l and c only, else 0>
    M <index> <name> <ms at call> <ms at return> <result>     every library call but pos_get for P
    P <ms> c1 c2 c3 c4 c5 c6     at most every P_MS (50); ms is the middle of the pos_get call;
                                 counts are pos_get type 1 (POSITION_ACTUAL), as `w2` shows them
    E <result> <text>            a library call returned < 0: then halt, online(OFF), the speed
                                 back, ctl_rel, exit 1
    done
Limits, checked before anything is called: axis 1..6, count 1..8, step not 0, |step| x count at
most 20 degrees (m: 3000 pulses), speeds 1..25. Nothing is read from the console.

Where each piece comes from (listings by a465/r3dis.py of this unit's files)
- joint(axis, distance): robotlib 0x0527 sends request 10 [distance, axis]; distance is a float
  (the pendant's incremental jog passes its float increment, bin/pendant 0x071e); ash.help:
  "joint <joint #>,<units>: rotates a rotational joint by a specified number of degrees".
- motor(axis, pulses, cond): robotlib 0x0537, request 11; bin/ash 0x190f calls it with cond 0;
  robotlib's constants MSTOP_NONE 0, MSTOP_ONHOME 32000, MSTOP_OFFHOME -32000.
- move(location): the location is passed by value, 9 words (ash 0x192f: push9, count 9); robotlib
  0x0702 sends request 18 [location, a second location with flags 8 (LOC_NULL), 1 = INTERP_JOINT].
- A location is 9 words. Word 0: byte 0 class (2 = loc_precision), byte 1 machine type, byte 2
  flags, byte 3 a check byte that makes the 36 bytes sum to 0; words 1..8 the per-axis data (3.1
  interpreter 0x6e94..0x70a0). The server builds pos_get's location from the 8 axis words of the
  motion task with loc_pdata_set, flags 1, machine type, class 2 and loc_re_check (robotsrv
  0x0264..0x0277); ash prints those words as its PULSES row (motor pulses). So a target for one
  axis is the commanded location with that axis's word changed (loc_pdata_get, loc_pdata_set) and
  loc_re_check called, exactly as the server makes its own. Degrees to pulses: the counts per
  degree measured on this arm (a465/calfile.py COUNTS_PER_DEGREE), x100 as integers.
- cpath(&locations[0], start, finish, &triggers): robotlib 0x097e -> 0x087a: header (request 63:
  path 0, class, count, speed -1 = the robot's speed setting), points in batches of 5 (request
  64; each must pass loc_check), the 16 x 2 trigger table (request 66), then ctpath_go(0) (request
  65). The manual (RAPL-3 reference, cpath/ctpath) gives the trigger table as rows {location index,
  +/- output channel}; this program passes all zeros, which is also what every unused row
  becomes inside 0x087a (relative index 0, setting 0).
- online(mode): robotlib 0x09e6, request 24; ONLINE_OFF 0, ONLINE_ON 1. robotsrv 0x0b66 passes the
  mode to the servo board (message 1197) and sets its own mode word to 1 (0 for OFF), which makes
  it refuse manual moves.
- robotisfinished(): robotlib 0x09d4, request 20 with FINISH_TEST 1, returns the reply's third
  word; finish(): 0x0996, request 20 with FINISH_WAIT 0. halt(): 0x0615, request 23.
- Every move request goes to the servo board through robotsrv (move: 0x087c, message 1002) and is
  answered when the board answers; what the board does with a queue of moves is what this
  program measures. A request with flag 1 in robotlib's 0x0282 (all motion) first takes control
  itself (0x0288 calls ctl_get's routine).

Not known, so the PC side only reports: whether robotisfinished's result is 1 for finished (it is
taken so; if not, the 6 s limit ends the polling and finish() still waits for the end).
"""
import argparse
from dataclasses import dataclass
from pathlib import Path

from a465 import calfile, r3asm, r3dis

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "controller" / "programs"
BUILD_TIME = 0x6B05F9D0          # 2026-11-24 21:00 UTC; the firmware is not known to check it

KINDS = {"j": 1, "m": 2, "l": 3, "c": 4}
# robotlib routine numbers (controller/robotlib-functions.txt; 0x3e robotisfinished by its symbol)
ROUTINES = {"ctl_get": 0x02, "ctl_rel": 0x04, "pos_get": 0x0E, "joint": 0x1A, "motor": 0x1B, "halt": 0x21,
            "move": 0x2C, "cpath": 0x39, "finish": 0x3A, "robotisfinished": 0x3E, "online": 0x3F,
            "speed_get": 0x43, "speed_set": 0x44}
ONLINE_OFF, ONLINE_ON = 0, 1
POSITION_ACTUAL, POSITION_COMMANDED = 1, 2
MSTOP_NONE = 0


@dataclass(frozen=True)
class Options:
    max_degrees: int = 20        # |step| x count for j, l, c
    max_pulses: int = 3000       # |step| x count for m
    max_count: int = 8
    max_speed: int = 25
    p_ms: int = 50               # position lines at most this often
    check_ms: int = 200          # robotisfinished at most this often while polling
    poll_limit_ms: int = 6000
    poll_wait_ms: int = 10       # delay() between polling rounds
    build_time: int = BUILD_TIME


class _Code:
    """Source lines with a counter for the labels of generated call sites."""

    def __init__(self):
        self.lines, self.n = [], 0

    def __call__(self, text):
        self.lines += [ln for ln in text.strip("\n").split("\n")]

    def lib(self, name, pushes=(), count=None, check=True):
        """One library call: pre(), the pushes (last argument first), the call, the M line, and
        (check) fail(r) if r < 0. Leaves r in L5."""
        self.n += 1
        words = count if count is not None else sum(
            (9 if p.startswith("push9") else 1) + (1 if " tag " in p else 0) for p in pushes if p.startswith("push"))
        self("    pushi 0\n    call pre")
        for p in pushes:
            self("    " + p)
        self(f"    pushi {words}\n    libcall robotlib:{ROUTINES[name]:#04x}      ; {name}\n    pop L5")
        self(f"    push L5\n    pushleag n_{name}\n    pushi 2\n    call mline\n    pop L5")
        if check:
            self(f"    ltc L4, L5, 0\n    ifnot L4, ok{self.n}\n    push L5\n    pushi 1\n    call fail\nok{self.n}:")


def source(opts=Options()):
    """The segtest program as r3asm source."""
    cpd = [round(c * 100) for c in calfile.COUNTS_PER_DEGREE]
    k = _Code()
    names = "\n".join(f'n_{n}:{" " * (16 - len(n))}string "{n}"' for n in ROUTINES)
    k(f"""; segtest: back-to-back moves on one axis, timed. Built by a465/segprog.py; see its docstring.
.build {opts.build_time:#010x}
.entry main
.lib syslib {r3asm.SYSLIB.slot} {r3asm.SYSLIB.build_time:#010x} {r3asm.SYSLIB.signature:#010x} {r3asm.SYSLIB.word1}
.lib robotlib {r3asm.ROBOTLIB.slot} {r3asm.ROBOTLIB.build_time:#010x} {r3asm.ROBOTLIB.signature:#010x} {r3asm.ROBOTLIB.word1}
.equ P_MS {opts.p_ms}
.equ CHECK_MS {opts.check_ms}
.equ POLL_LIMIT_MS {opts.poll_limit_ms}
.equ POLL_WAIT_MS {opts.poll_wait_ms}

.data
s_bad:      string "{{}}: invalid command line arguments.\\n"
s_usage:    string "Usage: {{}} j|m|l|c axis step count speed1 speed2\\n"
s_j:        string "j"
s_m:        string "m"
s_l:        string "l"
s_c:        string "c"
f_m:        string "M {{}} {{}} {{}} {{}} {{}}\\n"
f_p:        string "P {{}} {{}} {{}} {{}} {{}} {{}} {{}}\\n"
f_e:        string "E {{}} {{}}\\n"
f_i:        string "I {{}} {{}} {{}} {{}} {{}} {{}} {{}}\\n"
s_done:     string "done\\n"
{names}
cpd:        word {", ".join(str(c) for c in cpd)}    ; counts per degree x 100, joints 1..6
kind:       word 0              ; 1 j, 2 m, 3 l, 4 c
axis:       word 0
step:       word 0
count:      word 0
speed1:     word 0
speed2:     word 0
half:       word 0              ; speed_set(speed2) after this many moves
dpulses:    word 0              ; l, c: pulses per step (for the I line)
v0:         word 0              ; l, c: the axis's commanded counts at the start
c100:       word 0
tmpint:     word 0
t0:         word 0, 0           ; mtime() at the start
tm:         word 0, 0
tcall:      word 0
tlastp:     word -100000
tlastc:     word -100000
tpoll:      word 0
mindex:     word 0
saved:      word 0              ; speed_get()
havectl:    word 0
haveonline: word 0
havespeed:  word 0
base:       zeros 9             ; pos_get(2)
ints:       zeros 8
locs:       zeros {9 * opts.max_count}            ; the targets, 9 words each
trig:       zeros 32            ; cpath's trigger table: 16 rows of {{index, setting}}, all 0
ploc:       zeros 9             ; pos_get(1) for P lines
pint1:      word 0
pint2:      word 0
pint3:      word 0
pint4:      word 0
pint5:      word 0
pint6:      word 0
            zeros 2

.code
main:
    enter 24
    pushleag t0
    pushi 1
    builtinx mtime
    pushi 0                         ; ---- segtest kind axis step count speed1 speed2
    builtin argc
    pop L6
    eqc L5, L6, 7
    if L5, a_kind
    pushi 0
    call usage
a_kind:
    pushi 1
    pushi 1
    builtin argv
    pop L7
    ldi L8, 0""")
    for letter, code in KINDS.items():
        k(f"""    leag L5, s_{letter}
    streq L4, L7, L5
    ifnot L4, a_not_{letter}
    ldi L8, {code}
a_not_{letter}:""")
    k("""    if L8, a_kind_ok
    pushi 0
    call usage
a_kind_ok:
    putg L8, kind""")
    for n, var in ((2, "axis"), (3, "step"), (4, "count"), (5, "speed1"), (6, "speed2")):
        k(f"""    pushi {n}
    pushi 1
    call iarg
    pop L6
    putg L6, {var}""")
    k(f"""    getg L6, axis                   ; ---- limits, before anything is called
    ltc L5, L6, 1
    gtc L4, L6, 6
    lor L5, L5, L4
    getg L6, count
    ltc L4, L6, 1
    lor L5, L5, L4
    gtc L4, L6, {opts.max_count}
    lor L5, L5, L4
    getg L6, speed1
    ltc L4, L6, 1
    lor L5, L5, L4
    gtc L4, L6, {opts.max_speed}
    lor L5, L5, L4
    getg L6, speed2
    ltc L4, L6, 1
    lor L5, L5, L4
    gtc L4, L6, {opts.max_speed}
    lor L5, L5, L4
    getg L6, step
    eqc L4, L6, 0
    lor L5, L5, L4
    ifnot L5, a_travel
    pushi 0
    call usage
a_travel:
    getg L6, step
    ltc L5, L6, 0
    ifnot L5, a_abs
    neg L6, L6
a_abs:
    getg L5, count
    mul L6, L6, L5                  ; |step| x count
    ldi L5, {opts.max_degrees}
    getg L4, kind
    eqc L4, L4, 2
    ifnot L4, a_cmp
    ldi L5, {opts.max_pulses}
a_cmp:
    gt L4, L6, L5
    ifnot L4, a_ok
    pushi 0
    call usage
a_ok:
    getg L6, count                  ; half = (count + 1) / 2
    addc L6, L6, 1
    divc L6, L6, 2
    putg L6, half
    getg L6, kind                   ; l, c: pulses per step, from the counts per degree
    ltc L5, L6, 3
    if L5, a_info
    getg L6, axis
    subc L6, L6, 1
    leag L5, cpd
    ldx L4, L5, L6
    putg L4, c100
    getg L6, step
    mul L6, L6, L4
    ldi L5, 100
    div L6, L6, L5
    putg L6, dpulses
a_info:
    getg L6, dpulses
    push L6 tag 1
    getg L6, speed2
    push L6 tag 1
    getg L6, speed1
    push L6 tag 1
    getg L6, count
    push L6 tag 1
    getg L6, step
    push L6 tag 1
    getg L6, axis
    push L6 tag 1
    push L7 tag 19
    pushi 7
    pushleag f_i
    pushi 16
    builtinx printf""")
    k.lib("ctl_get")                                                   # ---- control, speed
    k("""    ldi L6, 1
    putg L6, havectl""")
    k.lib("speed_get", ["pushleag saved"])
    k("""    ldi L6, 1
    putg L6, havespeed""")
    k.lib("speed_set", ["getg L6, speed1", "push L6"])
    k("""    getg L6, kind                   ; ---- l, c: the target locations
    ltc L5, L6, 3
    if L5, no_locs""")
    k.lib("pos_get", ["pushleag base", f"pushi {POSITION_COMMANDED}"])
    k("""    pushleag ints
    pushleag base
    pushi 2
    builtin loc_pdata_get
    getg L6, axis
    subc L6, L6, 1
    leag L5, ints
    ldx L7, L5, L6
    putg L7, v0
    ldi L8, 1
bl_loop:
    getg L6, count
    gt L5, L8, L6
    if L5, no_locs
    getg L6, step                   ; target = v0 + k * step * c100 / 100
    mul L6, L6, L8
    getg L5, c100
    mul L6, L6, L5
    ldi L5, 100
    div L6, L6, L5
    getg L5, v0
    add L6, L6, L5
    getg L5, axis
    subc L5, L5, 1
    leag L4, ints
    stx L6, L4, L5
    leag L7, locs                   ; &locs[k - 1]
    subc L5, L8, 1
    mulc L5, L5, 9
    add L7, L7, L5
    leag L6, base
    ldi L5, 9
    memcopy L7, L6, L5              ; the commanded location, header and all ...
    pushleag ints
    push L7
    pushi 2
    builtin loc_pdata_set           ; ... with the one axis changed ...
    push L7
    pushi 1
    builtin loc_re_check            ; ... and its check byte made good, as the server does
    addc L8, L8, 1
    goto bl_loop
no_locs:""")
    k.lib("online", [f"pushi {ONLINE_ON}"])
    k("""    ldi L6, 1
    putg L6, haveonline
    pushi 1
    pushi 1
    call psample
    pop L5
    ltc L4, L5, 0
    ifnot L4, mv_begin
    push L5
    pushi 1
    call fail
mv_begin:
    getg L6, kind
    eqc L5, L6, 4
    if L5, mv_path
    ldi L8, 1                       ; ---- j, m, l: count moves back to back
mv_loop:
    getg L6, count
    gt L5, L8, L6
    if L5, poll_begin
    getg L6, kind
    eqc L5, L6, 1
    if L5, mv_j
    eqc L5, L6, 2
    if L5, mv_m
    leag L6, locs                   ; l: move(locs[k - 1]), by value
    subc L5, L8, 1
    mulc L5, L5, 9
    add L6, L6, L5
    ldp9 L20, L6, 0""")
    k.lib("move", ["push9 L20"], count=9)
    k("""    goto mv_next
mv_j:""")
    k.lib("joint", ["getg L6, step", "itof L5, L6", "push L5", "getg L6, axis", "push L6"])
    k("""    goto mv_next
mv_m:""")
    k.lib("motor", [f"pushi {MSTOP_NONE}", "getg L6, step", "push L6", "getg L6, axis", "push L6"])
    k("""mv_next:
    pushi 0
    pushi 1
    call psample
    pop L5
    ltc L4, L5, 0
    ifnot L4, mv_half
    push L5
    pushi 1
    call fail
mv_half:
    getg L6, half
    eq L5, L8, L6
    ifnot L5, mv_inc""")
    k.lib("speed_set", ["getg L6, speed2", "push L6"])
    k("""mv_inc:
    addc L8, L8, 1
    goto mv_loop
mv_path:                            ; ---- c: one continuous path, then the second speed""")
    k.lib("cpath", ["pushleag trig", "getg L6, count", "subc L6, L6, 1", "push L6", "pushi 0", "pushleag locs"])
    k.lib("speed_set", ["getg L6, speed2", "push L6"])
    k("""poll_begin:                         ; ---- until robotisfinished says done, or the limit
    pushi 0
    call now
    pop L6
    putg L6, tpoll
poll:
    pushi 0
    pushi 1
    call psample
    pop L5
    ltc L4, L5, 0
    ifnot L4, poll_check
    push L5
    pushi 1
    call fail
poll_check:
    pushi 0
    call now
    pop L6
    getg L5, tlastc
    sub L4, L6, L5
    ldi L5, CHECK_MS
    lt L3, L4, L5
    if L3, poll_time
    putg L6, tlastc""")
    k.lib("robotisfinished")
    k("""    gtc L4, L5, 0
    if L4, poll_done
poll_time:
    pushi 0
    call now
    pop L6
    getg L5, tpoll
    sub L4, L6, L5
    ldi L5, POLL_LIMIT_MS
    ge L3, L4, L5
    if L3, poll_done
    pushi POLL_WAIT_MS
    pushi 1
    libcall syslib:delay
    chkpop
    goto poll
poll_done:""")
    k.lib("finish")
    k("""    pushi 1
    pushi 1
    call psample
    pop L5
    ltc L4, L5, 0
    ifnot L4, end_seq
    push L5
    pushi 1
    call fail
end_seq:""")
    k.lib("online", [f"pushi {ONLINE_OFF}"])
    k("""    ldi L6, 0
    putg L6, haveonline""")
    k.lib("speed_set", ["getg L6, saved", "push L6"])
    k.lib("ctl_rel")
    k("""    ldi L6, 0
    putg L6, havectl
    pushi 0
    pushleag s_done
    pushi 2
    builtinx printf
    pushi 0
    pushi 1
    builtinx exit
    retv 0

usage:                              ; usage(): print it, exit 1
    enter 6
    pushi 0
    pushi 1
    builtin argv
    pop L6
    push L6 tag 19
    pushi 1
    pushleag s_bad
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
    ret

iarg:                               ; iarg(n): argv(n) as an int, or usage()
    enter 8
    push A3
    pushi 1
    builtin argv
    pop L6
    push L6
    pushleag tmpint
    pushi 2
    builtin str_to_int              ; as homezc reads its argument
    pop L5
    ltc L4, L5, 0
    ifnot L4, i_ok
    pushi 0
    call usage
i_ok:
    getg L6, tmpint
    retv L6
    retv 0

now:                                ; now(): ms since the start
    enter 6
    pushleag tm
    pushi 1
    builtinx mtime
    getg L6, tm
    getg L5, t0
    sub L4, L6, L5
    retv L4
    retv 0

pre:                                ; pre(): the time at a call
    enter 6
    pushi 0
    call now
    pop L6
    putg L6, tcall
    ret

mline:                              ; mline(name, r): "M index name call return r"; returns r
    enter 8
    pushi 0
    call now
    pop L7
    getg L6, mindex
    push A4 tag 1
    push L7 tag 1
    getg L5, tcall
    push L5 tag 1
    push A3 tag 19
    push L6 tag 1
    pushi 5
    pushleag f_m
    pushi 12
    builtinx printf
    addc L6, L6, 1
    putg L6, mindex
    get L5, A4
    retv L5
    retv 0

psample:                            ; psample(force): a P line unless one was printed less than P_MS ago
    enter 10
    pushi 0
    call now
    pop L7
    if A3, ps_go
    getg L6, tlastp
    sub L5, L7, L6
    ldi L4, P_MS
    lt L3, L5, L4
    if L3, ps_skip
ps_go:
    putg L7, tlastp
    pushleag ploc
    pushi 1                         ; POSITION_ACTUAL
    pushi 2
    libcall robotlib:0x0e           ; pos_get
    pop L5
    pushi 0
    call now
    pop L6
    ltc L4, L5, 0
    ifnot L4, ps_ok
    putg L7, tcall                  ; an error: its M line, and the caller fails
    push L5
    pushleag n_pos_get
    pushi 2
    call mline
    pop L5
    retv L5
ps_ok:
    add L4, L7, L6
    divc L4, L4, 2                  ; the middle of the call
    pushleag pint1
    pushleag ploc
    pushi 2
    builtin loc_pdata_get
    getg L3, pint6
    push L3 tag 1
    getg L3, pint5
    push L3 tag 1
    getg L3, pint4
    push L3 tag 1
    getg L3, pint3
    push L3 tag 1
    getg L3, pint2
    push L3 tag 1
    getg L3, pint1
    push L3 tag 1
    push L4 tag 1
    pushi 7
    pushleag f_p
    pushi 16
    builtinx printf
ps_skip:
    ldi L5, 0
    retv L5
    retv 0

fail:                               ; fail(r): E line; halt, online(OFF), the speed back, ctl_rel; exit 1
    enter 10
    neg L5, A3
    push L5
    pushi 1
    libcall syslib:str_error
    pop L6
    push L6 tag 19
    push A3 tag 1
    pushi 2
    pushleag f_e
    pushi 6
    builtinx printf
    getg L6, havectl
    ifnot L6, f_exit
    ldi L6, 0
    putg L6, havectl""")
    k.lib("halt", check=False)
    k("""    getg L6, haveonline
    ifnot L6, f_speed""")
    k.lib("online", [f"pushi {ONLINE_OFF}"], check=False)
    k("""f_speed:
    getg L6, havespeed
    ifnot L6, f_rel""")
    k.lib("speed_set", ["getg L6, saved", "push L6"], check=False)
    k("f_rel:")
    k.lib("ctl_rel", check=False)
    k("""f_exit:
    pushi 1
    pushi 1
    builtinx exit
    ret""")
    return "\n".join(k.lines) + "\n"


def build(opts=Options()):
    return r3asm.assemble(source(opts), "segtest.r3s")


def listing(image, name="segtest"):
    robot = {n: name for name, n in r3asm.robotlib_names().items()}
    robot.update({v: k for k, v in ROUTINES.items()})
    mod = r3dis.Module(image, name)
    lines = r3dis.listing(mod, {"syslib": dict(r3dis.SYSLIB), "robotlib": robot})
    return "\n".join(lines + ["; data segment"] + r3dis.data_listing(mod)) + "\n"


def write(out_dir=OUT, opts=Options()):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    image = build(opts)
    (out_dir / "segtest").write_bytes(image)
    (out_dir / "segtest.r3s").write_text(source(opts))
    (out_dir / "segtest.lst").write_text(listing(image))
    return image


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args(argv)
    image = write(args.out)
    print(f"{args.out}/segtest: {len(image)} bytes")


if __name__ == "__main__":
    main()
