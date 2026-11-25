"""a465/aci.py against tests/fake_acid.py, a stand-in for fastacid written from its disassembly."""
import pytest

from a465 import aci
from fake_acid import FakeAcid, FakeClock

# What the PC sends to put the 5 bytes "hello" at /tmp/t, worked out by hand from the packet layout
# (see the aci.py docstring); each packet goes after BEL SYN and the controller's RS.
HELLO = [
    ("start", "01 01 00 00 04 00 01 00 00 00 00 00 07 00 00 00"),
    ("name", "01 02 00 00 00 00 00 03 06 00 fa ff 2f 74 6d 70 2f 74 00 00 28 04 00 00"),
    ("data 1", "01 02 00 00 01 00 00 04 05 00 fb ff 68 65 6c 6c 6f 00 00 00 1b 04 00 00"),
]


class Wire:
    """The fake controller, with a record of every byte the PC wrote."""

    def __init__(self, fake):
        self.fake, self.sent = fake, bytearray()

    def write(self, data):
        self.sent += data
        return self.fake.write(data)

    def read(self, count=1):
        return self.fake.read(count)


@pytest.fixture
def rig(tmp_path):
    def make(content=b"hello", remote="/tmp/t", **kwargs):
        clock = FakeClock()
        fake = FakeAcid(clock)
        wire = Wire(fake)
        local = tmp_path / "local.bin"
        local.write_bytes(content)
        lines = []

        def run(**more):
            return aci.send_file(wire, str(local), remote, report=lines.append, clock=clock, sleep=clock.sleep,
                                 **{**kwargs, **more})
        return fake, wire, run, lines
    return make


def test_the_hello_packets_are_these_bytes():
    got = [(label, aci.hexs(packet)) for label, packet in aci.plan(b"hello", "/tmp/t")]
    assert got == HELLO


def test_hello_reaches_the_controller_and_nothing_reaches_its_shell(rig):
    fake, wire, run, lines = rig()
    assert run() == 5
    assert fake.files["/tmp/t"] == b"hello"
    expected = b"".join(b"\x07\x16" + bytes.fromhex(h) for _, h in HELLO)
    assert bytes(wire.sent) == expected
    assert fake.typed == b"" and fake.signals == [] and not fake.out_sem_held
    assert fake.fd is None and not fake.mode            # fastacid closed the file and left transfer mode
    assert sum(1 for line in lines if line.startswith("<- 06")) == 3


def test_every_byte_value_arrives_and_none_of_them_acts_on_the_console(rig):
    content = bytes(range(256)) * 2 + bytes(range(128))      # 640 bytes, BEL, SYN, ^A, ^E and ^Z included
    fake, wire, run, _ = rig(content, "/conf/new.bin")
    run()
    assert fake.files["/conf/new.bin"] == content
    assert fake.writes == [("/conf/new.bin", 256), ("/conf/new.bin", 256), ("/conf/new.bin", 128)]
    assert fake.typed == b"" and fake.signals == []


@pytest.mark.parametrize("size, chunk", [(1, 256), (3, 1), (4, 4), (5, 4), (1032, 1032), (2049, 1024), (640, 7)])
def test_any_size_and_chunk_arrive_whole(rig, size, chunk):
    content = bytes((i * 37 + 11) & 0xFF for i in range(size))
    fake, _, run, _ = rig(content, "/tmp/sizes", chunk=chunk)
    fake.free_words = 10 ** 6                           # 92 packets of 7 bytes would need 106 KB free
    run()
    assert fake.files["/tmp/sizes"] == content
    assert len(fake.packets) == 2 + -(-size // chunk)


def test_missing_directories_on_the_path_are_made(rig):
    fake, _, run, _ = rig(b"x", "/app/newapp/sub/f")
    run()
    assert {"/app/newapp", "/app/newapp/sub"} <= fake.dirs


def test_sizes_fastacid_cannot_take_are_refused_before_anything_is_sent():
    with pytest.raises(ValueError, match="at most 1032"):
        aci.data_packet(1, bytes(1033), last=True)
    with pytest.raises(ValueError, match="at most 255"):
        aci.plan(bytes(256 * 10), "/tmp/t", chunk=10)
    with pytest.raises(ValueError, match="absolute"):
        aci.plan(b"x", "tmp/t")
    with pytest.raises(ValueError, match="empty file"):
        aci.plan(b"", "/tmp/t")


def test_without_fastacid_only_bel_syn_is_sent(rig):
    fake, wire, run, _ = rig()
    fake.running = False
    with pytest.raises(aci.TransferError, match="no RS"):
        run()
    assert bytes(wire.sent) == b"\x07\x16"


def test_shell_output_on_its_way_before_the_rs_is_passed_over(rig):
    fake, _, run, lines = rig()
    fake.chatter = b"\r\nsome output\r\n$ "
    run()
    assert fake.files["/tmp/t"] == b"hello"
    assert any("some output" in line for line in lines)


def damage_packet(n, how):
    seen = []

    def damage(packet):
        seen.append(packet)
        return how(packet) if len(seen) == n else packet
    return damage


def flip(at):
    return lambda p: p[:at] + bytes([p[at] ^ 0x40]) + p[at + 1:]


def test_a_damaged_data_packet_is_nakked_and_the_partial_file_is_deleted(rig):
    content = bytes(600)
    fake, wire, run, lines = rig(content, "/tmp/big")
    fake.damage = damage_packet(4, flip(20))            # the second data packet
    with pytest.raises(aci.TransferError, match="NAK"):
        run()
    assert "/tmp/big" not in fake.files and "deleted transfer file" in fake.log
    assert fake.packets[-1][:5] == bytes((1, 1, 0, 0, aci.ABORT))
    assert not fake.dead and fake.typed == b""


def test_a_nak_is_answered_by_sending_the_same_packet_again_when_asked(rig):
    fake, _, run, _ = rig(bytes(600), "/tmp/big", retries=1)
    fake.damage = damage_packet(4, flip(20))
    run()
    assert fake.files["/tmp/big"] == bytes(600)


def test_a_damaged_start_command_stops_without_an_abort(rig):
    fake, _, run, _ = rig()
    fake.damage = damage_packet(1, flip(6))
    with pytest.raises(aci.TransferError, match="start: NAK"):
        run()
    assert len(fake.packets) == 1 and not fake.dead     # command 1 now would close descriptor 0


def test_a_name_the_controller_cannot_open_stops_without_an_abort(rig):
    fake, _, run, lines = rig()
    fake.unwritable.add("/tmp/t")
    with pytest.raises(aci.TransferError, match="name: CAN"):
        run()
    assert len(fake.packets) == 2 and not fake.dead
    assert any("aborted" in line for line in lines)      # fastacid's own message is shown


def test_a_failed_write_is_a_failure_even_though_an_ack_follows_the_can(rig):
    fake, _, run, _ = rig()
    fake.fail_writes = True
    with pytest.raises(aci.TransferError, match="data 1: CAN"):
        run()
    assert "/tmp/t" not in fake.files                    # the abort deleted what was opened


def test_after_a_silent_failure_nothing_more_is_sent(rig):
    fake, wire, run, _ = rig(bytes(600), "/tmp/big")
    fake.damage = damage_packet(3, lambda p: bytes([2]) + p[1:])     # first data packet, wrong address
    with pytest.raises(aci.TransferError) as err:
        run()
    assert err.value.silent
    first_data = aci.plan(bytes(600), "/tmp/big")[2][1]
    assert wire.sent.endswith(b"\x07\x16" + first_data) and wire.sent.count(b"\x07\x16") == 3
    assert fake.out_sem_held                            # what fastacid is left with; see the aci.py docstring


def test_a_lost_reply_is_a_silent_failure_too(rig):
    fake, wire, run, _ = rig()
    fake.drop_reply = True
    with pytest.raises(aci.TransferError) as err:
        run()
    assert err.value.silent and len(fake.packets) == 1


def test_a_packet_cut_short_times_out_on_the_controller_before_the_pc_gives_up():
    clock = FakeClock()
    fake = FakeAcid(clock)
    link = aci.Link(fake, report=lambda line: None, clock=clock, sleep=clock.sleep)
    packet = aci.command_packet(aci.RECEIVE, 1)
    with pytest.raises(aci.TransferError) as err:
        aci.exchange(link, packet[:10], "start")
    assert err.value.silent and fake.log == ["header rcv failed (10)"]
    assert aci.REPLY_TIMEOUT > 5.0 and aci.RS_TIMEOUT > 5.0


def test_the_shell_prompt_and_cksum_are_read_like_a465_getfile_does(rig):
    fake, wire, run, _ = rig()
    run()
    link = aci.Link(wire, report=lambda line: None, clock=fake.clock, sleep=fake.clock.sleep)
    assert aci.at_system_prompt(link)
    assert aci.remote_cksum(link, "/tmp/t") == (sum(b"hello"), 5)
    assert aci.remote_cksum(link, "/tmp/none") is None


def test_xon_xoff_bytes_are_counted():
    assert aci.risky_bytes(b"\x11ab\x13\x13") == 3 and aci.risky_bytes(b"hello") == 0


def load_script(name):
    import importlib.machinery
    import importlib.util
    from pathlib import Path
    path = str(Path(__file__).resolve().parent.parent / "scripts" / name)
    loader = importlib.machinery.SourceFileLoader(name.replace("-", "_"), path)
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name, loader))
    loader.exec_module(module)
    return module


@pytest.fixture
def putfile(monkeypatch, tmp_path):
    """scripts/a465-putfile run in-process against a fake controller in place of the serial port."""
    import sys
    import types
    fake = FakeAcid()

    class Port:
        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return fake

        def __exit__(self, *exc):
            return False

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=Port))
    monkeypatch.setattr(aci, "SETTLE", 0.0)
    script = load_script("a465-putfile")

    def run(content, remote, *flags):
        local = tmp_path / "f"
        local.write_bytes(content)
        monkeypatch.setattr(sys, "argv", ["a465-putfile", *flags, str(local), remote])
        script.main()
    return fake, run


def test_the_script_sends_checks_and_verifies(putfile, capsys):
    fake, run = putfile
    run(b"hello", "/tmp/t")
    out = capsys.readouterr().out
    assert fake.files["/tmp/t"] == b"hello"
    assert "verified by the controller's cksum" in out
    assert "-> 01 01 00 00 04 00 01 00 00 00 00 00 07 00 00 00" in out and "<- 06   ACK" in out


def test_the_script_will_not_replace_a_file_unless_told(putfile, capsys):
    fake, run = putfile
    fake.files["/conf/robot.cal"] = b"old"
    with pytest.raises(SystemExit, match="already there"):
        run(b"new", "/conf/robot.cal")
    assert fake.files["/conf/robot.cal"] == b"old" and not fake.packets
    run(b"new", "/conf/robot.cal", "--replace")
    assert fake.files["/conf/robot.cal"] == b"new"


def test_the_script_stops_and_says_what_to_do_after_silence(putfile, capsys, monkeypatch):
    fake, run = putfile
    fake.drop_reply = True
    monkeypatch.setattr(aci, "REPLY_TIMEOUT", 0.2)
    with pytest.raises(SystemExit):
        run(b"hello", "/tmp/t")
    out = capsys.readouterr().out
    assert "STOPPED" in out and "power-cycle" in out and len(fake.packets) == 1


def test_fastacid_wants_12k_and_1k_per_packet_free_and_the_script_checks_first(putfile, rig):
    assert aci.memory_needed(3) == 4 * (3072 + 3 * 256)
    fake, _, run, _ = rig(bytes(600), "/tmp/big")
    fake.free_words = 3072 + 2 * 256                    # enough for two packets, not three
    with pytest.raises(aci.TransferError, match="name: CAN"):
        run()
    script_fake, script_run = putfile
    script_fake.free_words = 3072 + 2 * 256
    with pytest.raises(SystemExit, match="bytes free"):
        script_run(bytes(600), "/tmp/big")
    assert not script_fake.packets
    script_run(bytes(600), "/tmp/big", "--chunk", "300")
    assert script_fake.files["/tmp/big"] == bytes(600)
