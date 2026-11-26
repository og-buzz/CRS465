"""a465/app.py (scripts/a465-app) against fake controllers: no serial port, no waiting (a FakeClock)."""
import os

import pytest

from a465 import app, console, crosprog
from fake_acid import FakeAcid, FakeClock
from test_crosprog import PROG_01, make_program

HERE = os.path.dirname(os.path.abspath(__file__))
HELLO = os.path.join(HERE, "r3", "hello.r3")
ROBOT_PROG = PROG_01                                    # links syslib and robotlib
PLAIN_PROG = make_program(0x3B2A12B8, [("syslib", 0x3B2A12B0, 0x0A0D4875)])

try:
    import unicorn  # noqa: F401
    HAVE_UNICORN = True
except ImportError:
    HAVE_UNICORN = False
needs_unicorn = pytest.mark.skipif(not HAVE_UNICORN, reason="python3-unicorn missing")


# ---------------------------------------------------------------------------------------- fake controllers

class DfAcid(FakeAcid):
    """fastacid's fake plus `df`."""

    def __init__(self, clock=None, free_k=77):
        super().__init__(clock)
        self.free_k = free_k

    def _shell(self, b):
        if b == 0x0D and bytes(self._line).decode().strip() == "df":
            self._line = bytearray()
            self.typed.append(b)
            self.out += (b"\r\nFilesystem             type     :  free/  total\r\n"
                         + f".            (mfs)    :     {self.free_k}K/    213K bytes ( 36%) free\r\n".encode() + b"$ ")
            return
        super()._shell(b)


class Shell:
    """Echo, `$ `, and /tmp/prog in several moods: 'ends' (prints two lines after 2 s), 'hangs' (until Ctrl-E),
    'stubborn' (ignores Ctrl-E; Ctrl-Z frees it)."""

    def __init__(self, clock, mode="ends", at_prompt=True):
        self.clock, self.mode, self.at_prompt = clock, mode, at_prompt
        self.events, self.line, self.sent, self.running, self.closed = [], b"", [], False, False
        if at_prompt:
            self.events.append([0.0, b"$ "])

    def _put(self, data, delay=0.0):
        self.events.append([self.clock() + delay, data])

    def close(self):
        self.closed = True

    def write(self, data):
        for b in data:
            if b == 0x05:
                self.sent.append("^E")
                if self.running and self.mode != "stubborn":
                    self.running = False
                    self._put(b"[12 killed by SIGINT.]\r\n$ ", 0.2)
                elif not self.running:
                    self._put(b"$ ")
            elif b == 0x1a:
                self.sent.append("^Z")
                self.running = False
                self._put(b"[12 killed by SIGABRT.]\r\n$ ", 0.5)
            elif b == 0x0d:
                self._put(b"\r\n")
                line, self.line = self.line.decode(), b""
                self.sent.append(line)
                if line.startswith("/tmp/prog"):
                    if self.mode == "ends":
                        self._put(b"hello\r\nworld\r\n$ ", 2.0)
                    else:
                        self.running = True
                        self._put(b"working\r\n", 1.0)
                elif not line.strip() and self.at_prompt:
                    self._put(b"$ ")
            else:
                self.line += bytes([b])
                self._put(bytes([b]))
        return len(data)

    def read(self, count=1):
        now, out = self.clock(), b""
        self.events.sort(key=lambda e: e[0])
        while self.events and self.events[0][0] <= now and len(out) < count:
            out += self.events.pop(0)[1]
        return out

    def flush(self):
        pass


@pytest.fixture
def rig(tmp_path):
    clock = FakeClock()

    def go(*argv, port=None):
        port = port or Shell(clock)
        code = app.main(list(argv), repo=str(tmp_path), open_port=lambda a: port, clock=clock, sleep=clock.sleep,
                        log_dir=str(tmp_path))
        return code, port, sorted(tmp_path.glob("*.log"))
    go.clock, go.tmp = clock, tmp_path
    return go


def prog_file(tmp_path, data=PLAIN_PROG, name="prog"):
    path = tmp_path / name
    path.write_bytes(data)
    return str(path)


# ---------------------------------------------------------------------------------------------- build

@needs_unicorn
def test_build_wraps_the_compiler(tmp_path, capsys):
    out = tmp_path / "hello"
    assert app.main(["build", HELLO, "-o", str(out), "-s"]) == 0
    assert out.is_file() and crosprog.check_ok(out.read_bytes())
    assert f"{out}: " in capsys.readouterr().out


@needs_unicorn
def test_build_failure_is_nonzero(tmp_path, capsys):
    assert app.main(["build", os.path.join(HERE, "r3", "bad.r3"), "-o", str(tmp_path / "bad")]) == 1
    assert "Error" in capsys.readouterr().out and not (tmp_path / "bad").exists()


def test_build_missing_source(tmp_path, capsys):
    pytest.importorskip("unicorn")
    assert app.main(["build", str(tmp_path / "nope.r3")]) == 1


# ----------------------------------------------------------------------------------------------- send

def send_rig(tmp_path, free_k=77, files=None, **kw):
    clock = FakeClock()
    fake = DfAcid(clock, free_k)
    fake.files.update(files or {})
    code = app.main(["send", *kw.pop("argv")], repo=str(tmp_path), open_port=lambda a: fake, clock=clock,
                    sleep=clock.sleep, log_dir=str(tmp_path))
    return code, fake


def test_send_verifies_with_cksum(tmp_path, capsys):
    local = prog_file(tmp_path)
    code, fake = send_rig(tmp_path, argv=[local])
    out = capsys.readouterr().out
    assert code == 0 and fake.files["/tmp/prog"] == PLAIN_PROG
    assert "verified by the controller's cksum" in out and "77" not in out.split("verified")[0].split("sum")[-1]
    assert list(tmp_path.glob("*-send.log"))


def test_send_refuses_when_too_full(tmp_path, capsys):
    local = prog_file(tmp_path, PLAIN_PROG + bytes(2000))
    code, fake = send_rig(tmp_path, free_k=9, argv=[local])          # 9 K free, 2.2 K file + 8 K to spare = too much
    out = capsys.readouterr().out
    assert code == 1 and "/tmp/prog" not in fake.files
    assert "9216 bytes free of 218112" in out and "8 K must be left over" in out


def test_send_fits_with_exactly_the_room(tmp_path):
    local = prog_file(tmp_path)
    code, fake = send_rig(tmp_path, free_k=8 + 1, argv=[local])
    assert code == 0 and "/tmp/prog" in fake.files


def test_send_skips_identical_file(tmp_path, capsys):
    local = prog_file(tmp_path)
    code, fake = send_rig(tmp_path, files={"/tmp/prog": PLAIN_PROG}, argv=[local])
    assert code == 0 and "already on the controller with the same bytes" in capsys.readouterr().out
    assert fake.packets == [] and b"df" not in bytes(fake.typed)


def test_send_replaces_a_different_file(tmp_path, capsys):
    local = prog_file(tmp_path)
    code, fake = send_rig(tmp_path, files={"/tmp/prog": b"old"}, argv=[local])
    assert code == 0 and fake.files["/tmp/prog"] == PLAIN_PROG and "replacing /tmp/prog" in capsys.readouterr().out


@pytest.mark.parametrize("dest", ["/conf/robot.cal", "/lib/x", "relative", "/tmp/a b"])
def test_send_rejects_bad_destination(tmp_path, capsys, dest):
    local = prog_file(tmp_path)
    code, fake = send_rig(tmp_path, argv=[local, "--to", dest])
    assert code == 1 and fake.packets == [] and not fake.typed


def test_send_anywhere_allows_other_directory(tmp_path):
    local = prog_file(tmp_path)
    code, fake = send_rig(tmp_path, argv=[local, "--to", "/conf/prog", "--anywhere"])
    assert code == 0 and fake.files["/conf/prog"] == PLAIN_PROG


def test_send_app_dir_and_not_a_program(tmp_path, capsys):
    code, fake = send_rig(tmp_path, argv=[prog_file(tmp_path), "--to", "/app/p"])
    assert code == 0 and "/app/p" in fake.files
    junk = tmp_path / "junk"
    junk.write_bytes(b"not a program")
    code, fake = send_rig(tmp_path, argv=[str(junk)])
    assert code == 1 and not fake.typed


def test_send_detects_a_failed_check(tmp_path, capsys):
    clock = FakeClock()
    fake = DfAcid(clock)
    local = prog_file(tmp_path)
    with pytest.raises(app.AppError, match="CHECK FAILED"):
        link = app.homezc.make_link(fake, lambda t: None, clock, clock.sleep)
        app.send(link, local, "/tmp/prog", lambda t: None, upload=lambda: fake.files.__setitem__("/tmp/prog", b"damaged"))


@needs_unicorn
def test_send_compiles_a_r3_first(tmp_path):
    code, fake = send_rig(tmp_path, argv=[HELLO])
    assert code == 0 and crosprog.check_ok(fake.files["/tmp/hello"])


# ------------------------------------------------------------------------------------------------- run

def test_run_prints_stamped_lines_and_returns_0(rig, capsys):
    code, shell, logs = rig("run", "/tmp/prog", "a", "b", "--no-robot")
    out = capsys.readouterr().out
    assert code == 0 and "/tmp/prog a b" in shell.sent and "^E" not in shell.sent and "^Z" not in shell.sent
    assert "[   2.0" in out and "hello" in out and "world" in out and "ended by itself" in out
    assert len(list(rig.tmp.glob("*-run.log"))) == 1 and shell.closed


def test_run_time_limit_sends_ctrl_e_and_returns_3(rig, capsys):
    code, shell, _ = rig("run", "/tmp/prog", "--no-robot", "--time-limit", "5", port=Shell(FakeClock(), "hangs"))
    # the Shell above has its own clock: rebuild on the rig's
    clock = rig.clock
    code, shell, _ = rig("run", "/tmp/prog", "--no-robot", "--time-limit", "5", port=Shell(clock, "hangs"))
    out = capsys.readouterr().out
    assert code == 3 and shell.sent.count("^E") == 1 and "^Z" not in shell.sent
    assert "working" in out and "stopped by Ctrl-E" in out and "killed by SIGINT" in out


def test_run_ctrl_c_stops_with_ctrl_e(rig, capsys):
    shell = Shell(rig.clock, "hangs")
    real = rig.clock.sleep
    n = []

    def sleep(sec):
        n.append(1)
        if len(n) == 400:
            raise KeyboardInterrupt
        real(sec)
    rig.clock.sleep = sleep
    try:
        code = app.main(["run", "/tmp/prog", "--no-robot"], repo=str(rig.tmp), open_port=lambda a: shell,
                        clock=rig.clock, sleep=sleep, log_dir=str(rig.tmp))
    finally:
        rig.clock.sleep = real
    assert code == 3 and shell.sent.count("^E") == 1 and "^Z" not in shell.sent
    assert "the operator (Ctrl-C)" in capsys.readouterr().out


def test_run_refuses_when_not_at_prompt(rig, capsys):
    code, shell, _ = rig("run", "/tmp/prog", "--no-robot", port=Shell(rig.clock, at_prompt=False))
    assert code == 1 and "not at its `$` prompt" in capsys.readouterr().out
    assert "/tmp/prog" not in " ".join(shell.sent)


def test_stubborn_program_without_robot_never_gets_ctrl_z(rig, capsys):
    code, shell, _ = rig("run", "/tmp/prog", "--no-robot", "--time-limit", "3", port=Shell(rig.clock, "stubborn"))
    assert code == 1 and shell.sent.count("^E") == 1 and "^Z" not in shell.sent
    assert "Ctrl-Z is not sent without --robot" in capsys.readouterr().out


def test_stubborn_program_with_robot_gets_ctrl_z_then_ctrl_e(rig, capsys):
    code, shell, _ = rig("run", "/tmp/prog", "--robot", "--time-limit", "3", port=Shell(rig.clock, "stubborn"))
    out = capsys.readouterr().out
    assert shell.sent.count("^Z") == 1 and shell.sent.count("^E") == 2
    assert shell.sent.index("^E") < shell.sent.index("^Z") < len(shell.sent) - 1 and shell.sent[-1] == "^E"
    assert code == 3 and "SENDING Ctrl-Z" in out and "!!!" in out


def test_ctrl_z_not_used_when_ctrl_e_works_even_with_robot(rig):
    code, shell, _ = rig("run", "/tmp/prog", "--robot", "--time-limit", "3", port=Shell(rig.clock, "hangs"))
    assert code == 3 and "^Z" not in shell.sent


# ------------------------------------------------------------------------------------------ robotlib rule

def test_links_robotlib_from_the_library_table(tmp_path):
    assert app.links_robotlib(ROBOT_PROG) is True
    assert app.links_robotlib(PLAIN_PROG) is False
    assert app.links_robotlib(b"junk") is None


@needs_unicorn
def test_compiled_hello_does_not_link_robotlib(tmp_path):
    out = tmp_path / "hello"
    assert app.main(["build", HELLO, "-o", str(out)]) == 0
    assert app.links_robotlib(out.read_bytes()) is False


def test_run_unknown_path_needs_a_robot_flag(rig, capsys):
    code, shell, _ = rig("run", "/tmp/prog")
    assert code == 1 and shell.sent == [] and "--robot" in capsys.readouterr().out and not list(rig.tmp.glob("*.log"))


def test_run_robot_flag_prints_warning(rig, capsys):
    code, shell, _ = rig("run", "/tmp/prog", "--robot")
    assert code == 0 and "WARNING" in capsys.readouterr().out


def go_rig(rig, data, *flags):
    src = rig.tmp / "prog.r3"
    src.write_text("main\nend main\n")
    fake = DfAcid(rig.clock)
    from unittest import mock
    with mock.patch.object(app, "build", lambda s, out=None, *a, **k: (open(out, "wb").write(data), out)[1]):
        code = app.main(["go", str(src), *flags], repo=str(rig.tmp), open_port=lambda a: fake, clock=rig.clock,
                        sleep=rig.clock.sleep, log_dir=str(rig.tmp))
    return code, fake


def test_go_refuses_robotlib_program_without_flag(rig, capsys):
    code, fake = go_rig(rig, ROBOT_PROG)
    assert code == 1 and fake.typed == b"" and fake.files == {} and "links robotlib" in capsys.readouterr().out


def test_go_with_robot_warns_and_sends_then_runs(rig, capsys):
    code, fake = go_rig(rig, ROBOT_PROG, "--robot")
    out = capsys.readouterr().out
    assert fake.files["/tmp/prog"] == ROBOT_PROG and "ARM" in out
    assert b"/tmp/prog\r" in bytes(fake.typed)
    assert code in (0, 1, 3)           # DfAcid has no program to run: only that the order build, send, run held


def test_go_plain_program_runs_without_flags(rig, capsys):
    code, fake = go_rig(rig, PLAIN_PROG)
    assert fake.files["/tmp/prog"] == PLAIN_PROG and b"/tmp/prog\r" in bytes(fake.typed)


def test_go_stops_at_failed_build(rig, capsys):
    src = rig.tmp / "prog.r3"
    src.write_text("x")
    fake = DfAcid(rig.clock)
    from unittest import mock
    with mock.patch.object(app, "build", side_effect=app.AppError("compile failed (status 1)")):
        code = app.main(["go", str(src)], repo=str(rig.tmp), open_port=lambda a: fake, clock=rig.clock,
                        sleep=rig.clock.sleep, log_dir=str(rig.tmp))
    assert code == 1 and fake.typed == b""
