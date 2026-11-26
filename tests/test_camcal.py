"""a465/camcal.py on synthetic data only: no camera, no network, no arm."""
import json

import cv2
import numpy as np
import pytest

from a465 import camcal, model

W, H = 960, 540
K = np.array([[900.0, 0, 480.0], [0, 900.0, 270.0], [0, 0, 1]])
DIST = np.array([-0.30, 0.10, 0.001, -0.0008, -0.02])
BOARD = dict(camcal.DEFAULT_BOARD)


def render_board(rvec, tvec, k=K, dist=DIST, size=(W, H)):
    """The printed board as the camera sees it: every pixel's ray, undistorted, meets the board plane."""
    px_per_mm = 6.0
    bw, bh = BOARD["squares_x"] * BOARD["square_mm"], BOARD["squares_y"] * BOARD["square_mm"]
    tex = camcal.make_board(BOARD).generateImage((int(bw * px_per_mm), int(bh * px_per_mm)), marginSize=0,
                                                 borderBits=1)
    uu, vv = np.meshgrid(np.arange(size[0], dtype=np.float64), np.arange(size[1], dtype=np.float64))
    n = cv2.undistortPoints(np.column_stack([uu.ravel(), vv.ravel()]).reshape(-1, 1, 2), k, dist).reshape(-1, 2)
    rays = np.column_stack([n, np.ones(len(n))])
    rot = camcal.rot_of(rvec)
    normal, d0 = rot[:, 2], np.asarray(tvec, float)
    s = (normal @ d0) / (rays @ normal)
    pts = (rays * s[:, None] - d0) @ rot          # board coordinates, mm
    mapx = (pts[:, 0] * px_per_mm).reshape(size[1], size[0]).astype(np.float32)
    mapy = (pts[:, 1] * px_per_mm).reshape(size[1], size[0]).astype(np.float32)
    img = cv2.remap(tex, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    return img


def board_poses():
    """Twelve board poses that between them push the board into every part of the frame."""
    rng = np.random.default_rng(3)
    bw, bh = BOARD["squares_x"] * BOARD["square_mm"], BOARD["squares_y"] * BOARD["square_mm"]
    out = []
    for i, (fx, fy) in enumerate([(0.15, 0.2), (0.85, 0.2), (0.15, 0.8), (0.85, 0.8), (0.5, 0.5), (0.5, 0.2),
                                  (0.5, 0.8), (0.15, 0.5), (0.85, 0.5), (0.3, 0.35), (0.7, 0.65), (0.35, 0.7)]):
        z = 430 + 40 * (i % 3)
        centre = np.array([(fx * W - K[0, 2]) / K[0, 0] * z, (fy * H - K[1, 2]) / K[1, 1] * z, z])
        tilt = camcal.rot_of([rng.uniform(-.5, .5), rng.uniform(-.5, .5), rng.uniform(-.3, .3)])
        rot = tilt
        t = centre - rot @ np.array([bw / 2, bh / 2, 0])
        out.append((camcal.rvec_of(rot), t))
    return out


@pytest.fixture(scope="module")
def lens_detections():
    dets = []
    for i, (rv, tv) in enumerate(board_poses()):
        img = render_board(rv, tv)
        img = cv2.GaussianBlur(img, (3, 3), 0.7)
        c, ids = camcal.detect_board(img, BOARD)
        dets.append(("s%02d" % i, c, ids))
    return dets


def test_synthetic_board_is_detected(lens_detections):
    assert all(len(ids) >= 20 for _, _, ids in lens_detections)


def test_lens_recovers_known_camera(lens_detections):
    res = camcal.calibrate_lens(lens_detections, (W, H), BOARD)
    assert res["rms_px"] < 0.5
    k = np.array(res["camera_matrix"])
    assert k[0, 0] == pytest.approx(900, rel=0.015)
    assert k[1, 1] == pytest.approx(900, rel=0.015)
    assert k[0, 2] == pytest.approx(480, abs=6)
    assert k[1, 2] == pytest.approx(270, abs=6)
    assert res["dist_coeffs"][0] == pytest.approx(-0.30, abs=0.03)
    assert res["dist_coeffs"][1] == pytest.approx(0.10, abs=0.08)
    # what matters: where a pixel's ray goes, across the frame, including the corners
    pts = np.array([[20, 20], [940, 20], [20, 520], [940, 520], [480, 270]], np.float64).reshape(-1, 1, 2)
    a = cv2.undistortPoints(pts, K, DIST).reshape(-1, 2)
    b = cv2.undistortPoints(pts, k, np.array(res["dist_coeffs"])).reshape(-1, 2)
    assert np.abs(a - b).max() * 900 < 4.0      # pixels, in a 960 wide frame
    assert res["good"], res["problems"]


def test_lens_result_round_trips_through_json(lens_detections, tmp_path):
    res = camcal.calibrate_lens(lens_detections, (W, H), BOARD)
    camcal.save_json(res, str(tmp_path / "lens.json"))
    k, dist, size, d = camcal.load_lens(str(tmp_path / "lens.json"))
    assert size == (W, H) and k.shape == (3, 3) and len(dist) == 5 and d["good"]


def test_lens_refuses_poor_coverage_and_too_few(lens_detections):
    centre = [d for d in lens_detections][4:5] * 12      # twelve copies of the same centred view
    res = camcal.calibrate_lens(centre, (W, H), BOARD)
    assert not res["good"] and any("coverage" in p for p in res["problems"])
    few = lens_detections[:6]
    res = camcal.calibrate_lens(few, (W, H), BOARD)
    assert not res["good"] and any("usable pictures" in p for p in res["problems"])
    with pytest.raises(camcal.CalibrationError):
        camcal.calibrate_lens(lens_detections[:3], (W, H), BOARD)


def test_lens_flags_a_bad_fit(lens_detections):
    rng = np.random.default_rng(1)
    noisy = [(n, c + rng.normal(0, 3.0, c.shape).astype(np.float32), ids) for n, c, ids in lens_detections]
    assert not camcal.calibrate_lens(noisy, (W, H), BOARD)["good"]


def test_undistort_straightens(lens_detections):
    rv, tv = board_poses()[0]
    img = render_board(rv, tv)
    out = camcal.undistort(img, K, DIST)
    assert out.shape == img.shape
    c0, ids0 = camcal.detect_board(out, BOARD)
    assert len(ids0) > 10


def test_target_pages(tmp_path):
    files = camcal.write_targets(str(tmp_path))
    names = {f.rsplit("/", 1)[1] for f in files}
    assert "charuco_7x9_25mm_DICT_5X5_100.pdf" in names and "aruco_id90_60mm_DICT_5X5_100.png" in names
    img = camcal.read_image(str(tmp_path / "charuco_7x9_25mm_DICT_5X5_100.png"))
    assert img.shape == (3300, 2550)                         # Letter at 300 dpi
    c, ids = camcal.detect_board(img, BOARD)                 # the printed page is itself detectable
    assert len(ids) == (BOARD["squares_x"] - 1) * (BOARD["squares_y"] - 1)
    # square size at true size: neighbouring corners 25 mm = 295.3 px apart
    d = np.linalg.norm(c[np.argsort(ids)][0] - c[np.argsort(ids)][1])
    assert d == pytest.approx(25 / 25.4 * 300, rel=0.01)
    assert json.load(open(tmp_path / "charuco_7x9_25mm_DICT_5X5_100.json"))["square_mm"] == 25.0
    m = camcal.read_image(str(tmp_path / "aruco_id90_60mm_DICT_5X5_100.png"))
    assert camcal.detect_marker(m, 90) is not None
    with pytest.raises(camcal.CalibrationError):
        camcal.target_board_image({"square_mm": 40})


def test_pose_convention_matches_model():
    q = (20, -50, 30, 15, -40, 70)
    p, rot, _, _ = model.forward(q)
    y, pi, r = model.yaw_pitch_roll(rot)
    assert camcal.pose_matrix(*p, y, pi, r)[:3, :3] == pytest.approx(rot, abs=1e-9)


# ---------------------------------------------------------------- camera to robot

MARKER_MM = 60.0
T_WC_TRUE = None


def look_at(eye, target, up=(0, 0, 1)):
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return camcal.matrix(np.column_stack([x, y, z]), eye)


def simulate(n=12, noise=0.3, seed=5, k=K, dist=DIST):
    """A fixed camera, a marker offset from the tool point, n arm poses; returns the truth and the samples."""
    rng = np.random.default_rng(seed)
    t_wc = look_at(np.array([1100.0, -150.0, 450.0]), np.array([350.0, 0.0, 350.0]))
    t_tm = camcal.matrix(camcal.rot_ypr(10, -20, 95), (35.0, -12.0, 48.0))
    obj = camcal.marker_object_points(MARKER_MM)
    tools, corners = [], []
    for _ in range(n):
        centre = np.array([rng.uniform(150, 500), rng.uniform(-250, 250), rng.uniform(150, 550)])
        face = camcal.inv(t_wc)[:3, 3] - centre                 # marker normal roughly toward the camera
        zaxis = face / np.linalg.norm(face)
        tilt = camcal.rot_of(rng.uniform(-0.6, 0.6, 3))
        base = look_at(np.zeros(3), -zaxis)[:3, :3] @ np.diag([1, -1, -1])   # z toward the camera
        rot = tilt @ base @ camcal.rot_of([0, 0, rng.uniform(-1.2, 1.2)])
        t_wm = camcal.matrix(rot, centre)
        t_wt = t_wm @ camcal.inv(t_tm)
        t_cm = camcal.inv(t_wc) @ t_wm
        px, _ = cv2.projectPoints(obj, camcal.rvec_of(t_cm[:3, :3]), t_cm[:3, 3], k, dist)
        corners.append(px.reshape(4, 2) + rng.normal(0, noise, (4, 2)))
        tools.append(t_wt)
    return t_wc, t_tm, tools, corners


def rot_err_deg(a, b):
    return np.degrees(np.linalg.norm(camcal.rvec_of(a[:3, :3].T @ b[:3, :3])))


def test_robot_step_recovers_camera_and_marker_offset():
    t_wc, t_tm, tools, corners = simulate()
    res = camcal.solve_robot(tools, corners, MARKER_MM, K, DIST)
    got_wc, got_tm = np.array(res["T_world_camera"]), np.array(res["T_tool_marker"])
    assert np.linalg.norm(got_wc[:3, 3] - t_wc[:3, 3]) < 6.0          # mm
    assert rot_err_deg(got_wc, t_wc) < 0.5
    assert np.linalg.norm(got_tm[:3, 3] - t_tm[:3, 3]) < 3.0
    assert rot_err_deg(got_tm, t_tm) < 1.0
    assert res["rms_px"] < 0.8 and res["good"], res["problems"]
    assert len(res["per_sample"]) == 12


def test_robot_step_from_pose_file_and_pixels(tmp_path):
    t_wc, t_tm, tools, corners = simulate()
    samples = []
    for t, c in zip(tools, corners):
        y, p, r = model.yaw_pitch_roll(t[:3, :3])
        samples.append({"corners_px": c.tolist(),
                        "pose": dict(zip("x y z yaw pitch roll".split(), [*t[:3, 3], y, p, r]))})
    path = tmp_path / "samples.json"
    path.write_text(json.dumps({"marker_mm": MARKER_MM, "samples": samples}))
    lens = {"kind": "lens", "image_size": [W, H], "camera_matrix": K.tolist(), "dist_coeffs": DIST.tolist()}
    (tmp_path / "lens.json").write_text(json.dumps(lens))
    res = camcal.solve_robot_file(str(path), str(tmp_path / "lens.json"))
    assert np.linalg.norm(np.array(res["camera_position_mm"]) - t_wc[:3, 3]) < 6.0


def test_robot_step_with_noise_degrades_gracefully():
    t_wc, t_tm, tools, corners = simulate(noise=1.0, seed=8)
    res = camcal.solve_robot(tools, corners, MARKER_MM, K, DIST)
    assert np.linalg.norm(np.array(res["camera_position_mm"]) - t_wc[:3, 3]) < 15.0
    assert 0.5 < res["rms_px"] < 2.0


def test_robot_step_refuses_too_few():
    _, _, tools, corners = simulate(n=12)
    with pytest.raises(camcal.CalibrationError, match="need at least"):
        camcal.solve_robot(tools[:6], corners[:6], MARKER_MM, K, DIST)


def test_robot_step_refuses_clustered_positions():
    _, _, tools, corners = simulate(n=12)
    near = []
    for t in tools:
        t = t.copy()
        t[:3, 3] = tools[0][:3, 3] + (t[:3, 3] - tools[0][:3, 3]) * 0.1
        near.append(t)
    with pytest.raises(camcal.CalibrationError, match="positions too close"):
        camcal.solve_robot(near, corners, MARKER_MM, K, DIST)


def test_robot_step_refuses_same_orientation():
    _, _, tools, corners = simulate(n=12)
    same = []
    for t in tools:
        t = t.copy()
        t[:3, :3] = tools[0][:3, :3]
        same.append(t)
    with pytest.raises(camcal.CalibrationError, match="orientations too alike"):
        camcal.solve_robot(same, corners, MARKER_MM, K, DIST)


def test_robot_step_flags_inconsistent_data():
    t_wc, t_tm, tools, corners = simulate()
    tools = list(tools)
    bad = tools[3].copy()
    bad[:3, 3] += [60, -40, 30]                  # the controller "said" the arm was elsewhere
    tools[3] = bad
    res = camcal.solve_robot(tools, corners, MARKER_MM, K, DIST)
    assert not res["good"]


def test_plane_round_trip():
    t_wc, _, _, _ = simulate()
    pts = np.array([[300.0, 0.0, 20.0], [450.0, 180.0, 20.0], [200.0, -200.0, 20.0]])
    px = camcal.world_to_pixel(pts, t_wc, K, DIST)
    back = camcal.pixel_to_world_plane(px, 20.0, t_wc, K, DIST)
    assert back == pytest.approx(pts, abs=0.05)
    # a different plane height gives a different point for the same pixel
    other = camcal.pixel_to_world_plane(px, 120.0, t_wc, K, DIST)
    assert np.linalg.norm(other - back, axis=1).min() > 10


def test_projection_errors():
    t_wc, _, _, _ = simulate()
    with pytest.raises(camcal.CalibrationError, match="behind"):
        camcal.world_to_pixel([[3000.0, 0, 400]], t_wc, K, DIST)
    with pytest.raises(camcal.CalibrationError):
        camcal.pixel_to_world_plane([[480, 270]], 5000.0, t_wc, K, DIST)
