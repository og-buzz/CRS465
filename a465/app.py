"""build / send / run / go: compile a RAPL-3 program on the PC, put it on the controller, run it under supervision.

The command line is scripts/a465-app; the steps live here so that tests can drive them with a fake console.

  build SOURCE.r3 [-o OUT] [rc1 options]   a465/r3c.py, nothing touches the controller
  send FILE [--to /tmp/NAME]               df check (8 K to spare), put, verify with the controller's cksum
  run /tmp/NAME [ARGS] [--time-limit S]    start at `$`, print timestamped lines, Ctrl-E at the limit
  go SOURCE.r3 [ARGS]                      build, send, run

Exit codes: 0 done; 3 (run, go) the program was stopped at the time limit or by the operator; 1 any failure.

Safety. A program that links robotlib can move the arm. The decision is made from the compiled file's library
table (a465/crosprog.py). `run` of a program known from this PC (send, go) that links robotlib needs --robot;
`run` of a path whose content is not known needs --robot or --no-robot. Ctrl-Z (robot stop) is never sent
unless --robot was given and Ctrl-E did not bring the prompt back within 3 s.
"""
import argparse
import os
import re
import sys

from . import aci, console, crosprog, home, homezc

ROOM_K = 8                     # kilobytes that must stay free after a send
OK_DIRS = ("/tmp/", "/app/")
DEFAULT_LIMIT_S = 60.0
CTRL_E_WAIT_S = 3.0
ROBOTLIB_SIGNATURES = (0xAE952686, 0x51B97334)        # this unit's and the C500C's (docs/RAPL3-COMPILER.md)
DF_LINE = re.compile(r"\(mfs\)\s*:\s*(\d+)K/\s*(\d+)K bytes")
EXIT_OK, EXIT_FAIL, EXIT_STOPPED = 0, 1, 3


class AppError(RuntimeError):
    pass


# ------------------------------------------------------------------------------------------------ pieces

def links_robotlib(data):
    """True/False from the library table; None if the bytes are not a compiled program."""
    try:
        prog = crosprog.parse(data)
    except (crosprog.ProgramError, Exception):          # noqa: BLE001 -- struct errors on short files too
        return None
    return any(lib.name.lower().endswith("robotlib") or lib.signature in ROBOTLIB_SIGNATURES for lib in prog.libs)


def parse_df(text):
    """(free_bytes, total_bytes) of the RAM file system from `df` output, or None."""
    found = DF_LINE.search(text)
    return (int(found.group(1)) * 1024, int(found.group(2)) * 1024) if found else None


def read_df(link):
    link.drain(quiet=0.1, limit=1.0)
    answer = link.shell("df", lambda g: g.endswith(b"$ ") and b"\n" in g, 10.0)
    return parse_df(answer.decode("latin-1"))


def check_dest(dest, anywhere=False):
    if not dest.startswith("/") or any(ch.isspace() or not 33 <= ord(ch) < 127 for ch in dest):
        raise AppError(f"{dest!r}: the destination must be an absolute path of printable ASCII without spaces")
    if not anywhere and not dest.startswith(OK_DIRS):
        raise AppError(f"{dest}: refusing a destination outside /tmp and /app (--anywhere overrides)")


def build(source, out=None, options=(), say=print, **kw):
    """Compile with a465/r3c.py. Returns the output path. Raises AppError."""
    try:
        import unicorn  # noqa: F401
    except ImportError:
        raise AppError("the compiler needs the unicorn module: sudo apt install python3-unicorn") from None
    from . import r3c
    try:
        res = r3c.compile(source, out=out, options=list(options), **kw)
    except (FileNotFoundError, ValueError) as err:
        raise AppError(str(err)) from None
    if res.messages:
        say(res.messages.rstrip("\n"))
    if res.output is None:
        raise AppError(f"compile failed (status {res.status})")
    say(f"{res.output}: {os.path.getsize(res.output)} bytes")
    return res.output


def send(link, local, dest, say=print, anywhere=False, upload=None):
    """Put `local` (a compiled program) at `dest`. Returns the bytes sent (0 if already there). Raises AppError."""
    check_dest(dest, anywhere)
    with open(local, "rb") as f:
        data = f.read()
    want = (sum(data) & 0xFFFFFFFF, len(data))
    if not aci.at_system_prompt(link):
        raise AppError("the controller is not at its `$` system prompt")
    there = aci.remote_cksum(link, dest)
    if there == want:
        say(f"{dest} is already on the controller with the same bytes ({want[1]} bytes, sum {want[0]:#x}); nothing sent")
        return 0
    df = read_df(link)
    if df is None:
        raise AppError("`df` gave no figure for the RAM file system; refusing to send blind")
    free, total = df
    if len(data) + ROOM_K * 1024 > free:
        raise AppError(f"{local} is {len(data)} bytes; the controller has {free} bytes free of {total}, and {ROOM_K} K must be "
                       f"left over (needs {len(data) + ROOM_K * 1024}): not sending")
    say(f"{local}: {len(data)} bytes, sum {want[0]:#x}; controller has {free} bytes free of {total}"
        + (f"; replacing {dest} ({there[1]} bytes, sum {there[0]:#x})" if there else ""))
    try:
        (upload or (lambda: aci.send_file(link, local, dest, report=say)))()
    except aci.TransferError as err:
        raise AppError(f"transfer stopped: {err} (reason in /log/acid.log on the controller)") from None
    got = aci.remote_cksum(link, dest)
    if got != want:
        raise AppError(f"CHECK FAILED: the controller's cksum of {dest} gives {got}, the file here is {want}")
    say(f"{dest}: {want[1]} bytes, sum {want[0]:#x}, verified by the controller's cksum")
    return len(data)


class _Watch:
    """Splits the controller's output into stamped lines as it arrives."""

    def __init__(self, link, command, say):
        self.link, self.command, self.say = link, command, say
        self.t0, self.first, self.button = link.clock(), True, False

    def pump(self):
        """Read once; print complete lines. True when the prompt is showing."""
        self.link._more()
        text = self.link.ahead.decode("latin-1")
        done, _, rest = text.rpartition("\n")
        for line in (done.replace("\r", "").split("\n") if done else []):
            if self.first:
                self.first = False
                if line.strip().endswith(self.command.strip()):
                    continue
            if line.strip():
                self.say(f"[{self.link.clock() - self.t0:7.2f}] {line.rstrip()}")
            if homezc.BUTTON in line and not self.button:
                self.button = True
                self.say("NOTE: the controller asks for the PAUSE/CONTINUE button on its front panel")
        if done:
            self.link.ahead = rest.encode("latin-1")
        return rest.strip(" \r") == "$" and not self.first

    def wait_prompt(self, seconds):
        end = self.link.clock() + seconds
        while True:
            if self.pump():
                return True
            if self.link.clock() >= end:
                return False


def run(link, command, limit=DEFAULT_LIMIT_S, robot=False, say=print):
    """Start `command` at `$`. Returns EXIT_OK, EXIT_STOPPED or EXIT_FAIL."""
    if not aci.at_system_prompt(link):
        say("the controller is not at its `$` prompt; not starting anything")
        return EXIT_FAIL
    link.drain(quiet=0.1, limit=1.0)
    watch = _Watch(link, command, say)
    link.port.write(command.encode() + b"\r")
    reason = None
    try:
        while not watch.pump():
            if link.clock() - watch.t0 > limit:
                reason = f"the time limit of {limit:g} s"
                break
    except KeyboardInterrupt:
        reason = "the operator (Ctrl-C)"
    if reason is None:
        say(f"the program ended by itself after {link.clock() - watch.t0:.1f} s")
        return EXIT_OK
    say(f"STOPPING the program ({reason}): sending Ctrl-E")
    link.port.write(console.CTRL_E)
    if watch.wait_prompt(CTRL_E_WAIT_S):
        say("stopped by Ctrl-E; the controller is back at its `$` prompt")
        return EXIT_STOPPED
    if not robot:
        say(f"NO PROMPT within {CTRL_E_WAIT_S:g} s of Ctrl-E. Ctrl-Z is not sent without --robot. Look at the controller.")
        return EXIT_FAIL
    say("!!! Ctrl-E DID NOT BRING THE PROMPT BACK: SENDING Ctrl-Z (ROBOT STOP, SIGABRT) !!!")
    link.port.write(console.CTRL_Z)
    watch.wait_prompt(homezc.RESET_WAIT_S + 1.0)        # the controller resets for about 2 s after Ctrl-Z
    link.ahead = b""
    link.port.write(console.CTRL_E)                     # then Ctrl-E again, for whatever is still running
    back = watch.wait_prompt(CTRL_E_WAIT_S)
    say("the controller is back at its `$` prompt after Ctrl-Z" if back else "!!! THE CONTROLLER IS NOT AT ITS `$` PROMPT !!!")
    return EXIT_STOPPED if back else EXIT_FAIL


def robot_gate(known, robot, no_robot, say=print):
    """known: True (links robotlib), False (does not), None (content not known). Returns (ok, robot): `robot` allows Ctrl-Z."""
    if known:
        if not robot:
            say("REFUSED: this program links robotlib and can move the arm. Run it with --robot (stand at the E-stop).")
            return False, False
        say("!!! WARNING: THIS PROGRAM LINKS ROBOTLIB AND CAN MOVE THE ARM. Keep a hand on the E-stop. !!!")
        return True, True
    if known is None and not robot and not no_robot:
        say("REFUSED: the program on the controller is not known from this PC. State --robot (it may move the arm) "
            "or --no-robot (it cannot).")
        return False, False
    if robot:
        say("!!! WARNING: --robot given: the program may move the arm. Keep a hand on the E-stop. !!!")
    return True, robot


# ------------------------------------------------------------------------------------------------ command line

def parse_args(argv):
    ap = argparse.ArgumentParser(prog="a465-app", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def port(p):
        p.add_argument("--port", default=os.environ.get("A465_PORT", "/dev/ttyUSB0"))

    b = sub.add_parser("build", help="compile SOURCE.r3 (unknown options go to rc1)")
    b.add_argument("source")
    b.add_argument("-o", dest="out")
    s = sub.add_parser("send", help="put a compiled program (or a .r3, compiled first) on the controller")
    s.add_argument("file")
    s.add_argument("--to", dest="dest")
    s.add_argument("--anywhere", action="store_true", help="allow a destination outside /tmp and /app")
    s.add_argument("-o", dest="out", help="where to put the compiled output of a .r3 (default: a temporary file)")
    port(s)
    for name, help_ in (("run", "run /tmp/NAME at the controller's prompt"), ("go", "build, send, run")):
        r = sub.add_parser(name, help=help_)
        r.add_argument("target", help="the program path on the controller" if name == "run" else "SOURCE.r3")
        r.add_argument("args", nargs="*", help="arguments for the program")
        r.add_argument("--time-limit", type=float, default=DEFAULT_LIMIT_S, help="seconds before Ctrl-E (default %(default)g)")
        r.add_argument("--robot", action="store_true", help="the program may move the arm; allows Ctrl-Z as a last resort")
        r.add_argument("--no-robot", action="store_true", help="state that the program cannot move the arm")
        port(r)
        if name == "go":
            r.add_argument("--to", dest="dest")
            r.add_argument("--anywhere", action="store_true")
    args, extra = ap.parse_known_args(argv)
    if args.cmd != "build" and extra:
        ap.error(f"unrecognized arguments: {' '.join(extra)}")
    args.extra = extra
    if args.cmd in ("run", "go"):
        if args.robot and args.no_robot:
            ap.error("--robot and --no-robot together make no sense")
        if args.time_limit <= 0:
            ap.error("--time-limit must be above 0")
    return args


def main(argv=None, repo=None, open_port=None, clock=None, sleep=None, log_dir=None):
    """Returns the exit code. `open_port(args)` replaces the serial port in tests."""
    import tempfile
    repo = repo or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    args = parse_args(argv)
    local = dest = None
    known, robot = None, False
    try:
        if args.cmd == "build":
            build(args.source, args.out, args.extra)
            return EXIT_OK
        if args.cmd == "run":
            if not args.target.startswith("/") or re.search(r"\s", args.target):
                raise AppError("the program path must be absolute and without spaces")
            command = " ".join([args.target] + args.args)
        else:
            src = args.file if args.cmd == "send" else args.target
            if args.cmd == "go" and not src.endswith(".r3"):
                raise AppError("go takes a .r3 source file")
            local = src
            if src.endswith(".r3"):
                out = getattr(args, "out", None) or os.path.join(tempfile.mkdtemp(prefix="a465-app-"),
                                                                os.path.splitext(os.path.basename(src))[0])
                local = build(src, out)
            with open(local, "rb") as f:
                known = links_robotlib(f.read())
            if known is None:
                raise AppError(f"{local} is not a compiled RAPL-3 program (no readable library table)")
            dest = args.dest or "/tmp/" + os.path.splitext(os.path.basename(src))[0]
            check_dest(dest, args.anywhere)
            command = " ".join([dest] + getattr(args, "args", []))
        if args.cmd in ("run", "go"):
            ok, robot = robot_gate(known, args.robot, args.no_robot)
            if not ok:
                return EXIT_FAIL
    except AppError as err:
        print(f"a465-app: {err}", file=sys.stderr)
        return EXIT_FAIL
    with home.session(args.cmd, args, repo, open_port, clock, sleep, log_dir) as (link, say):
        try:
            if args.cmd in ("send", "go"):
                send(link, local, dest, say, args.anywhere)
            if args.cmd == "send":
                return EXIT_OK
            return run(link, command, args.time_limit, robot, say)
        except AppError as err:
            say(f"STOPPED: {err}")
            return EXIT_FAIL
