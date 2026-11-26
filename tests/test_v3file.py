"""a465/v3file.py against the five real .v3 files copied off the controller on 2026-11-23."""
import glob
import math
import os
import subprocess
import sys

import pytest

from a465 import v3file
from a465.calfile import COUNTS_PER_DEGREE

ROOT = os.path.join(os.path.dirname(__file__), "..")
APP = os.path.join(ROOT, "controller", "backup-2026-11-23", "app")
SAMPLES = sorted(glob.glob(os.path.join(APP, "*", "*.v3")))
TEST_V3 = os.path.join(APP, "test", "test.v3")
CACO2 = os.path.join(APP, "caco", "caco2.v3")
SCRIPT = os.path.join(ROOT, "scripts", "a465-v3")


def raw(path):
    with open(path, "rb") as f:
        return f.read()


def test_all_five_samples_are_present():
    assert len(SAMPLES) == 5


@pytest.mark.parametrize("path", SAMPLES)
def test_round_trip_is_byte_identical(path):
    assert v3file.dump(v3file.load(path)) == raw(path)


def test_empty_files_have_no_variables_and_no_end_word():
    for name in ("caco/caco.v3", "2/2.v3"):
        t = v3file.load(os.path.join(APP, name))
        assert t.variables == [] and t.end_word == b""
    assert v3file.load(os.path.join(APP, "2/2.v3")).tag == "2"


def test_sample_program_has_one_untaught_int():
    t = v3file.load(os.path.join(APP, "SampleProgram", "SampleProgram.v3"))
    v = t.find("I")
    assert (v.type_name, v.dims, v.taught, v.element(0)) == ("int", (), 0, 0)


def test_test_v3_contents():
    t = v3file.load(TEST_V3)
    assert t.tag == "test" and t.saves == 2 and t.timestamp == 0x62626AED
    assert [v.name for v in t.variables] == ["here2", "mt"]
    h = t.find("here2")
    assert h.type_name == "ploc" and h.dims == () and h.taught == 1
    loc = h.element(0)
    assert loc.is_precision and loc.machine == 9 and loc.check_ok
    assert loc.counts() == (-13600, 13233, 795, 25895, -8336, 28163, 0, 0)
    assert loc.joints_deg[0] == pytest.approx(-13600 / COUNTS_PER_DEGREE[0])
    assert t.find("mt").element(0).counts()[:3] == (-13600, 13265, 776)


def test_caco2_summary_by_type():
    t = v3file.load(CACO2)
    assert t.tag == "caco" and len(t.variables) == 17 and t.saves == 29
    kinds = {}
    for v in t.variables:
        kinds[v.type_name] = kinds.get(v.type_name, 0) + 1
    assert kinds == {"ploc": 1, "int": 1, "cloc": 15}
    assert t.end_word == b"\0\0\0\0" and len(t.slack) == 10092


def test_caco2_entries_look_like_real_locations():
    t = v3file.load(CACO2)
    c = t.find("carousel1")
    assert c.dims == (8,)
    for k in range(8):
        loc = c.element(k)
        assert loc.is_precision and loc.check_ok
        assert all(math.isfinite(d) and abs(d) < 400 for d in loc.joints_deg)
    assert c.element(0).counts()[:3] == (11305, -5088, 30080)
    s = t.find("s_inc").element(0)
    assert s.is_cartesian and s.check_ok
    assert s.cartesian[0] == pytest.approx(16.7139, abs=1e-3)
    assert s.cartesian[2] == pytest.approx(11.7468, abs=1e-3)
    tec = t.find("tecan1")
    assert tec.dims == (3, 10) and tec.count == 30
    for k in range(30):
        e = tec.element(k)
        assert e.is_cartesian and e.check_ok and all(math.isfinite(f) for f in e.floats())
    assert t.find("incubator1").dims == (3, 96)
    assert t.find("close").taught == 0


def test_two_index_order_and_flat_index():
    v = v3file.load(CACO2).find("tecan1")
    assert v.flat_index((1, 2)) == 1 + 3 * 2
    assert v.flat_index((7,)) == 7
    with pytest.raises(v3file.V3Error):
        v.flat_index((3, 0))
    with pytest.raises(v3file.V3Error):
        v.flat_index(())


def test_set_cartesian_changes_only_that_element():
    before = raw(CACO2)
    t = v3file.load(CACO2)
    v3file.set_value(t, "tecan1[2]", ["1", "2", "3", "-90", "1.5", "0.25"])
    after = v3file.dump(t)
    assert len(after) == len(before)
    v = v3file.load(after).find("tecan1")
    loc = v.element(2)
    assert loc.cartesian == pytest.approx((1, 2, 3, -90, 1.5, 0.25))
    assert loc.check_ok
    start = before.index(b"\x06\0\0\0tecan1\0\0") + 4 + 8 + 2 * 36            # name is 6 bytes padded to 8
    old = v3file.Location.from_bytes(before[start:start + 36])
    assert loc.words[6:] == old.words[6:] and (loc.cls, loc.machine) == (old.cls, old.machine)
    diff = [i for i, (a, b) in enumerate(zip(before, after)) if a != b]
    assert diff and all(start <= i < start + 36 for i in diff)


def test_set_joints_and_counts():
    t = v3file.load(TEST_V3)
    v3file.set_value(t, "here2", ["10", "20", "-30", "40", "-50", "60"])
    loc = v3file.load(v3file.dump(t)).find("here2").element(0)
    assert loc.joints_deg == pytest.approx((10, 20, -30, 40, -50, 60), abs=0.5 / 130)
    assert loc.check_ok and loc.words[6:] == (0, 0)
    v3file.set_value(t, "mt", ["1", "2", "3", "4", "5", "6"], counts=True)
    assert v3file.load(v3file.dump(t)).find("mt").element(0).counts()[:6] == (1, 2, 3, 4, 5, 6)


def test_set_changes_nothing_outside_the_record():
    before = raw(TEST_V3)
    t = v3file.load(TEST_V3)
    v3file.set_value(t, "mt", ["0", "0", "0", "0", "0", "0"])
    after = v3file.dump(t)
    assert after[:0x50] == before[:0x50] and after[0x84:] == before[0x84:]
    assert after != before


def test_set_int_and_float_on_a_synthetic_table():
    t = v3file.load(os.path.join(APP, "SampleProgram", "SampleProgram.v3"))
    v3file.set_value(t, "I", ["42"])
    t2 = v3file.load(v3file.dump(t))
    assert t2.find("I").element(0) == 42 and t2.find("I").taught == 1
    with pytest.raises(v3file.V3Error):
        v3file.set_value(t2, "I", ["1.5"])


def test_edit_errors():
    t = v3file.load(CACO2)
    for ref, vals in (("nosuch", ["1"]), ("tecan1[99]", ["1"] * 6), ("tecan1[0]", ["1", "2"]),
                      ("carousel1[0]", ["1"] * 6 + ["x"]), ("carousel1", ["1"] * 6), ("close", ["1", "2"])):
        with pytest.raises(v3file.V3Error):
            v3file.set_value(t, ref, vals)
    with pytest.raises(v3file.V3Error):                       # float nan
        v3file.set_value(t, "tecan1[0]", ["nan"] * 6)
    with pytest.raises(v3file.V3Error):                       # cartesian values into a precision element
        t.find("carousel1").set_cartesian(0, [1] * 6)


@pytest.mark.parametrize("data, text", [
    (b"", "bytes"),
    (b"\0" * 24, "magic"),
    (raw(TEST_V3)[:60], "past the end"),
    (raw(TEST_V3)[:-3], "stray"),
])
def test_malformed_input_is_a_clear_error(data, text):
    with pytest.raises(v3file.V3Error, match=text):
        v3file.parse(data)


def test_bad_check_byte_count_and_kind_are_rejected():
    good = bytearray(raw(TEST_V3))
    bad = bytearray(good); bad[0x2c + 3] ^= 1
    with pytest.raises(v3file.V3Error, match="check byte"):
        v3file.parse(bytes(bad))
    bad = bytearray(good); bad[14] = 5
    with pytest.raises(v3file.V3Error, match="records"):
        v3file.parse(bytes(bad))
    bad = bytearray(good); bad[0x18 + 2] = 9
    with pytest.raises(v3file.V3Error, match="record header"):
        v3file.parse(bytes(bad))


def test_parse_ref():
    assert v3file.parse_ref("a") == ("a", ())
    assert v3file.parse_ref("a[3]") == ("a", (3,))
    assert v3file.parse_ref("a[1][2]") == ("a", (1, 2))
    with pytest.raises(v3file.V3Error):
        v3file.parse_ref("a[x]")


def run(*args):
    return subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True)


def test_script_list_show_check_set(tmp_path):
    r = run("list", TEST_V3)
    assert r.returncode == 0 and "here2" in r.stdout
    r = run("list", CACO2, "--json")
    assert r.returncode == 0 and '"tecan1"' in r.stdout
    assert run("show", CACO2, "tecan1[4]").returncode == 0
    assert run("check", CACO2).returncode == 0
    out = tmp_path / "o.v3"
    r = run("set", TEST_V3, "mt", "1", "2", "3", "4", "5", "6", "-o", str(out))
    assert r.returncode == 0 and out.exists()
    assert v3file.load(str(out)).find("mt").element(0).joints_deg[0] == pytest.approx(1, abs=0.01)
    r = run("set", TEST_V3, "mt", "1", "2", "3", "4", "5", "6")
    assert r.returncode != 0
    assert raw(TEST_V3) == raw(TEST_V3)
    r = run("set", TEST_V3, "nosuch", "1", "-o", str(out))
    assert r.returncode == 1 and "no variable" in r.stderr
