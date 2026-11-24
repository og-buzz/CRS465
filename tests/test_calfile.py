"""a465/calfile.py against this unit's real calibration file, copied off the controller on 2026-11-23."""
import pytest

from a465 import calfile

# /conf/robot.cal exactly as the controller printed it (72 bytes; its own cksum gives a byte sum of 0xe00).
ROBOT_CAL = bytes.fromhex(
    "0209023d" "29b80000" "a4020000" "3d410000" "b5a0ffff" "def3ffff" "2d620000" "00000000" "00000000"
    "0209041f" "bc020000" "a6010000" "12020000" "9b010000" "f4010000" "c7010000" "00000000" "00000000")


def test_the_copy_in_this_test_is_the_file_the_controller_has():
    assert len(ROBOT_CAL) == 72 and sum(ROBOT_CAL) == 0xE00


def test_both_records_are_read():
    cal = calfile.parse(ROBOT_CAL)
    assert cal.machine_type == 9
    assert cal.home_counts == (47145, 676, 16701, -24395, -3106, 25133, 0, 0)
    assert cal.switch_offsets == (700, 422, 530, 411, 500, 455, 0, 0)
    assert cal.check_bytes == (0x3D, 0x1F)


def test_home_points_come_out_near_the_manuals_switch_positions():
    degrees = calfile.parse(ROBOT_CAL).home_degrees()
    assert degrees == pytest.approx((169.72, 2.43, -60.41, 173.90, 22.36, -179.16), abs=0.01)
    manual = (175, 10, -60, 180, 25, 180)                       # Table 14 of the A465 guide
    for got, want in zip(degrees, manual):
        assert abs(abs(got) - abs(want)) < 8                  # signs follow the controller, not the manual


def test_the_order_of_the_two_records_does_not_matter():
    swapped = ROBOT_CAL[36:] + ROBOT_CAL[:36]
    assert calfile.parse(swapped).home_counts == calfile.parse(ROBOT_CAL).home_counts


@pytest.mark.parametrize("bad, why", [
    (ROBOT_CAL[:71], "expected 72 bytes"),
    (ROBOT_CAL[:36] + ROBOT_CAL[:36], "expected one each"),
    (bytes([2, 9, 7, 0]) + ROBOT_CAL[4:], "has type 7"),
    (ROBOT_CAL[:37] + bytes([8]) + ROBOT_CAL[38:], "different machine types"),
])
def test_anything_else_is_refused_with_the_reason(bad, why):
    with pytest.raises(calfile.CalFileError, match=why):
        calfile.parse(bad)


def test_the_description_names_every_joint():
    lines = calfile.describe(calfile.parse(ROBOT_CAL))
    assert len(lines) == 9 and "J3 elbow" in lines[4] and "-60.41" in lines[4]
    assert "not calibrated" in lines[-1]
