"""jointlim: a read-only program that runs ON the controller and prints the joint travel limits it enforces.

    python3 -m a465.jlimprog           build controller/programs/jointlim (+ .r3s source, .lst listing)

On the controller (after `scripts/a465-putfile controller/programs/jointlim /tmp/jointlim`): `/tmp/jointlim`.
It moves nothing and does not take control. Output, one line per axis 1..NAXES:

    J <axis> <positive limit> <negative limit>      floats as the controller's printf "{}" prints them
    E <axis> <r> <text>                             jointlim_get returned r < 0 (text: syslib str_error(-r))

then `done`, exit 0.

Where the numbers come from (listings by a465/r3dis.py of this unit's files):
- jointlim_get(axis, var float pos, var float neg): robotlib 0x6e -> 0x1178 (axis 1..8, else -22) -> 0x1156 with op 0:
  request 51 [0, axis - 1] sent with flag 0 (no control needed); the reply's words 2 and 3 are stored at the two
  pointers (0x1174, 0x1175). robotsrv 0x173d serves op 0 to any client (0x174a-0x174d: the controller, or op 0) by
  sending message 1166 [op, axis - 1, pos, neg] to the servo board and returning its answer: the limits are the
  board's. robotsrv sends 1166 itself only at start-up, for a track axis (0x2149, from robot.cfg's trackposlength /
  trackneglength); /conf/robot.cfg on this unit has no joint limits. jointlim_set (0x6f -> 0x118a) calls finish()
  first and sends the same request with op 1; /diag/setup uses it only for the track.
- The manual (jointlim_get) gives no units; RAPL-II's @XLIMITS took radians. The A465 datasheet lists J1 +/-175,
  J2 +/-90, J3 +/-110, J4 +/-180, J5 +/-105, J6 +/-180 degrees; this program prints what the board holds.
"""
import argparse
from pathlib import Path

from a465 import r3asm, r3dis

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "controller" / "programs"
BUILD_TIME = 0x6B057C40
NAXES = 6


def source(naxes=NAXES, build_time=BUILD_TIME):
    return f"""; jointlim: print the controller's joint travel limits. Built by a465/jlimprog.py; see its docstring.
.build {build_time:#010x}
.entry main
.lib syslib {r3asm.SYSLIB.slot} {r3asm.SYSLIB.build_time:#010x} {r3asm.SYSLIB.signature:#010x} {r3asm.SYSLIB.word1}
.lib robotlib {r3asm.ROBOTLIB.slot} {r3asm.ROBOTLIB.build_time:#010x} {r3asm.ROBOTLIB.signature:#010x} {r3asm.ROBOTLIB.word1}
.equ NAXES {naxes}

.data
s_j:        string "J {{}} {{}} {{}}\\n"
s_e:        string "E {{}} {{}} {{}}\\n"
s_done:     string "done\\n"
pos:        word 0
neg:        word 0

.code
main:
    enter 10
    ldi L7, 1
next:
    pushleag neg
    pushleag pos
    push L7
    pushi 3
    libcall robotlib:jointlim_get   ; robotlib 0x1178: request 51 op 0, message 1166
    pop L5
    ltc L4, L5, 0
    ifnot L4, got
    neg L6, L5
    push L6
    pushi 1
    libcall syslib:str_error
    pop L6
    push L6 tag 19
    push L5 tag 1
    push L7 tag 1
    pushi 3
    pushleag s_e
    pushi 8
    builtinx printf
    goto after
got:
    getg L6, neg
    push L6 tag 2
    getg L6, pos
    push L6 tag 2
    push L7 tag 1
    pushi 3
    pushleag s_j
    pushi 8
    builtinx printf
after:
    addc L7, L7, 1
    gtc L4, L7, NAXES
    ifnot L4, next
    pushi 0
    pushleag s_done
    pushi 2
    builtinx printf
    retv 0
"""


def build(naxes=NAXES):
    return r3asm.assemble(source(naxes), "jointlim.r3s")


def listing(image, name="jointlim"):
    robot = {n: name for name, n in r3asm.robotlib_names().items()}
    mod = r3dis.Module(image, name)
    lines = r3dis.listing(mod, {"syslib": dict(r3dis.SYSLIB), "robotlib": robot})
    return "\n".join(lines + ["; data segment"] + r3dis.data_listing(mod)) + "\n"


def write(out_dir=OUT):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    image = build()
    (out_dir / "jointlim").write_bytes(image)
    (out_dir / "jointlim.r3s").write_text(source())
    (out_dir / "jointlim.lst").write_text(listing(image))
    return image


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(OUT), help="directory for the build (with .r3s and .lst)")
    args = ap.parse_args(argv)
    image = write(args.out)
    print(f"{args.out}/jointlim: {len(image)} bytes")


if __name__ == "__main__":
    main()
