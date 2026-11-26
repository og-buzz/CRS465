"""Calibration of the cell's bench camera: its lens, and where it sits in the robot's world frame.

Two steps (docs/CAMERA.md has the procedure):

  1. Lens: camera matrix and distortion from pictures of a printed ChArUco board (`target_*`,
     `detect_board`, `calibrate_lens`, `undistort`).
  2. Camera to robot: one ArUco marker fixed flat to the gripper, the arm taken to several poses
     and stopped; for each, the tool pose from the controller and the marker corners in the picture.
     `solve_robot` finds the camera's pose in the world frame and the marker's fixed offset from
     the tool point together. It starts from OpenCV's calibrateRobotWorldHandEye and then refines
     both poses by least squares on the pixel error of the marker corners (Levenberg-Marquardt, our
     own, numpy only), because pixels are what was measured.

Frames and units: millimetres, degrees at the edges, radians inside. T_a_b is the 4x4 matrix that
takes a point written in frame b to frame a. W is the robot world frame, T the tool point (the
frame whose pose the controller reports), M the marker, C the camera (OpenCV convention: x right,
y down, z forward). Tool pose x, y, z, yaw, pitch, roll is T_W_T with rotation Rz(yaw) Ry(pitch)
Rx(roll), the same convention as a465.model.yaw_pitch_roll.

Needs OpenCV with the aruco module (tested with 4.10, Debian python3-opencv).
"""
import datetime
import glob
import json
import os

import numpy as np

# ---- thresholds (the reasoning is in docs/CAMERA.md) ----
LENS_MIN_IMAGES = 10          # usable pictures (board seen with enough corners)
LENS_MIN_CORNERS = 12         # per picture
LENS_RMS_MAX_PX = 1.0         # refuse above this
LENS_RMS_WARN_PX = 0.5
LENS_IMAGE_MAX_PX = 2.5       # one picture worse than this is named as an outlier
COVERAGE_MIN = 0.55           # share of the frame inside the board-corner hulls of all pictures
COVERAGE_CORNER_MIN = 0.20    # same, inside each of the four corner boxes (outer 20% x 20%)
ROBOT_MIN_SAMPLES = 8
ROBOT_RECOMMENDED = 12
ROBOT_MIN_POSITION_SPREAD_MM = (200.0, 100.0)   # largest and second principal extent of tool positions
ROBOT_MIN_ROTATION_SPREAD_DEG = (40.0, 20.0)    # same, of the tool orientations (as rotation vectors)
ROBOT_RMS_MAX_PX = 2.0
ROBOT_MAX_PX = 5.0
ROBOT_RMS_MAX_MM = 5.0

DEFAULT_BOARD = {"squares_x": 7, "squares_y": 9, "square_mm": 25.0, "marker_mm": 18.0,
                 "dictionary": "DICT_5X5_100"}
DEFAULT_MARKER = {"marker_id": 90, "marker_mm": 60.0, "dictionary": "DICT_5X5_100"}
LETTER_MM = (215.9, 279.4)
CAMERA_URL = "http://192.168.1.96:8080/snapshot.jpg"
CAMERA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "camera")


class CalibrationError(Exception):
    """A calibration was refused (too little data, bad spread, or poor quality)."""


def _cv2():
    import cv2
    if not hasattr(cv2, "aruco"):
        raise CalibrationError("this OpenCV has no aruco module (apt install python3-opencv)")
    return cv2


def now():
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


# ------------------------------------------------------------------ poses

def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_ypr(yaw, pitch, roll):
    """Rz(yaw) Ry(pitch) Rx(roll), angles in degrees."""
    return _rz(np.radians(yaw)) @ _ry(np.radians(pitch)) @ _rx(np.radians(roll))


def matrix(rot, t):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rot, np.asarray(t, float).reshape(3)
    return m


def inv(m):
    r, t = m[:3, :3], m[:3, 3]
    return matrix(r.T, -r.T @ t)


def pose_matrix(x, y, z, yaw, pitch, roll):
    return matrix(rot_ypr(yaw, pitch, roll), (x, y, z))


def rvec_of(rot):
    cv2 = _cv2()
    return cv2.Rodrigues(np.asarray(rot, float))[0].reshape(3)


def rot_of(rvec):
    cv2 = _cv2()
    return cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))[0]


def matrix_from_params(p):
    return matrix(rot_of(p[:3]), p[3:6])


def params_of_matrix(m):
    return np.concatenate([rvec_of(m[:3, :3]), m[:3, 3]])


# ------------------------------------------------------------------ targets

def _dict(name):
    cv2 = _cv2()
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_board(params=None):
    cv2 = _cv2()
    b = dict(DEFAULT_BOARD, **(params or {}))
    return cv2.aruco.CharucoBoard((b["squares_x"], b["squares_y"]), b["square_mm"], b["marker_mm"],
                                  _dict(b["dictionary"]))


def board_stem(params):
    b = dict(DEFAULT_BOARD, **params)
    return "charuco_%dx%d_%gmm_%s" % (b["squares_x"], b["squares_y"], b["square_mm"], b["dictionary"])


def marker_stem(params):
    m = dict(DEFAULT_MARKER, **params)
    return "aruco_id%d_%gmm_%s" % (m["marker_id"], m["marker_mm"], m["dictionary"])


def _letter_canvas(dpi):
    w, h = (int(round(v / 25.4 * dpi)) for v in LETTER_MM)
    return np.full((h, w), 255, np.uint8), dpi / 25.4


def _text(img, text, x_mm, y_mm, px_per_mm, height_mm=4.0):
    cv2 = _cv2()
    scale = height_mm * px_per_mm / 22.0
    cv2.putText(img, text, (int(x_mm * px_per_mm), int(y_mm * px_per_mm)), cv2.FONT_HERSHEY_SIMPLEX,
                scale, 0, max(1, int(round(scale * 1.6))), cv2.LINE_AA)


def _check_line(img, y_mm, px_per_mm, x0_mm=None):
    """A 100 mm line with end ticks, centred on the page; measure it with a ruler after printing."""
    cv2 = _cv2()
    x0 = x0_mm if x0_mm is not None else (LETTER_MM[0] - 100.0) / 2
    p = lambda x, y: (int(round(x * px_per_mm)), int(round(y * px_per_mm)))   # noqa: E731
    th = max(2, int(round(0.4 * px_per_mm)))
    cv2.line(img, p(x0, y_mm), p(x0 + 100, y_mm), 0, th)
    for x in (x0, x0 + 100):
        cv2.line(img, p(x, y_mm - 3), p(x, y_mm + 3), 0, th)
    _text(img, "This line is 100 mm. If your ruler says otherwise, the print was scaled.",
          x0 - 10, y_mm + 8, px_per_mm, 2.8)


def target_board_image(params=None, dpi=300):
    """The ChArUco board as a US Letter page (portrait), grayscale, at true size for `dpi`."""
    b = dict(DEFAULT_BOARD, **(params or {}))
    img, ppm = _letter_canvas(dpi)
    bw, bh = b["squares_x"] * b["square_mm"], b["squares_y"] * b["square_mm"]
    if bw > LETTER_MM[0] - 20 or bh > LETTER_MM[1] - 50:
        raise CalibrationError("board %.0f x %.0f mm does not fit on Letter with room for the text" % (bw, bh))
    size = (int(round(bw * ppm)), int(round(bh * ppm)))
    x0, y0 = (LETTER_MM[0] - bw) / 2, 22.0
    art = make_board(b).generateImage(size, marginSize=0, borderBits=1)
    px, py = int(round(x0 * ppm)), int(round(y0 * ppm))
    img[py:py + size[1], px:px + size[0]] = art
    _text(img, "ChArUco %dx%d %s square %g mm marker %g mm" %
          (b["squares_x"], b["squares_y"], b["dictionary"], b["square_mm"], b["marker_mm"]), 12, 11, ppm, 3.4)
    _text(img, "Print at 100% (no fit to page). Glue FLAT to a rigid board.",
          12, 17, ppm, 3.0)
    _check_line(img, y0 + bh + 14, ppm)
    return img


def target_marker_image(params=None, dpi=300):
    """One ArUco marker, centred on a Letter page, with a white border, size printed on it."""
    cv2 = _cv2()
    m = dict(DEFAULT_MARKER, **(params or {}))
    img, ppm = _letter_canvas(dpi)
    side = int(round(m["marker_mm"] * ppm))
    art = cv2.aruco.generateImageMarker(_dict(m["dictionary"]), m["marker_id"], side, borderBits=1)
    x0, y0 = (LETTER_MM[0] - m["marker_mm"]) / 2, 50.0
    px, py = int(round(x0 * ppm)), int(round(y0 * ppm))
    img[py:py + side, px:px + side] = art
    _text(img, "ArUco id %d %s black square side %g mm" % (m["marker_id"], m["dictionary"], m["marker_mm"]),
          12, 11, ppm, 3.4)
    _text(img, "Print at 100%. Cut out leaving the white border,", 12, 17, ppm, 3.0)
    _text(img, "glue FLAT and rigid to the gripper; it must not move.", 12, 22, ppm, 3.0)
    _check_line(img, y0 + m["marker_mm"] + 25, ppm)
    return img


def write_page(img, path_stem, dpi=300):
    """Write PNG and PDF (both marked as `dpi`, so they print at true size)."""
    from PIL import Image
    pil = Image.fromarray(img)
    pil.save(path_stem + ".png", dpi=(dpi, dpi))
    pil.save(path_stem + ".pdf", resolution=float(dpi))
    return path_stem + ".png", path_stem + ".pdf"


def write_targets(outdir=CAMERA_DIR, board=None, marker=None, which="both", dpi=300):
    os.makedirs(outdir, exist_ok=True)
    written = []
    if which in ("board", "both"):
        b = dict(DEFAULT_BOARD, **(board or {}))
        stem = os.path.join(outdir, board_stem(b))
        written += list(write_page(target_board_image(b, dpi), stem, dpi))
        with open(stem + ".json", "w") as f:
            json.dump({"kind": "charuco_board", **b, "page": "US Letter portrait", "dpi": dpi,
                       "made": now()}, f, indent=1)
        written.append(stem + ".json")
    if which in ("marker", "both"):
        m = dict(DEFAULT_MARKER, **(marker or {}))
        stem = os.path.join(outdir, marker_stem(m))
        written += list(write_page(target_marker_image(m, dpi), stem, dpi))
        with open(stem + ".json", "w") as f:
            json.dump({"kind": "aruco_marker", **m, "page": "US Letter portrait", "dpi": dpi,
                       "made": now()}, f, indent=1)
        written.append(stem + ".json")
    return written


def load_board(path=None):
    if not path:
        return dict(DEFAULT_BOARD)
    with open(path) as f:
        d = json.load(f)
    return {k: d[k] for k in DEFAULT_BOARD}


# ------------------------------------------------------------------ board detection and coverage

def detect_board(img, params=None):
    """(corners Nx2 float32, ids N) of the ChArUco chessboard corners found; empty arrays if none."""
    cv2 = _cv2()
    det = cv2.aruco.CharucoDetector(make_board(params))
    corners, ids, _, _ = det.detectBoard(img)
    if corners is None or ids is None:
        return np.zeros((0, 2), np.float32), np.zeros(0, int)
    return corners.reshape(-1, 2), ids.reshape(-1)


class Coverage:
    """Which part of the frame the pictures so far have put the board in."""

    GRID = (160, 90)   # cells across, down

    def __init__(self, size):
        self.size = tuple(size)     # (width, height) of the pictures
        self.mask = np.zeros((self.GRID[1], self.GRID[0]), np.uint8)

    def add(self, corners):
        cv2 = _cv2()
        if len(corners) < 3:
            return
        sx, sy = self.GRID[0] / self.size[0], self.GRID[1] / self.size[1]
        pts = (np.asarray(corners, float) * [sx, sy]).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillConvexPoly(self.mask, cv2.convexHull(pts), 1)

    def fraction(self):
        return float(self.mask.mean())

    def corner_fractions(self):
        h, w = self.mask.shape
        ch, cw = int(h * 0.2), int(w * 0.2)
        return {"top-left": float(self.mask[:ch, :cw].mean()), "top-right": float(self.mask[:ch, w - cw:].mean()),
                "bottom-left": float(self.mask[h - ch:, :cw].mean()),
                "bottom-right": float(self.mask[h - ch:, w - cw:].mean())}

    def missing(self):
        return [k for k, v in self.corner_fractions().items() if v < COVERAGE_CORNER_MIN]

    def ok(self):
        return self.fraction() >= COVERAGE_MIN and not self.missing()

    def summary(self):
        s = "covers %d%% of the frame" % round(100 * self.fraction())
        miss = self.missing()
        return s + ("; still empty: " + ", ".join(miss) if miss else "; all four corners reached")


def list_images(directory):
    out = []
    for ext in ("jpg", "jpeg", "png", "JPG", "PNG"):
        out += glob.glob(os.path.join(directory, "*." + ext))
    return sorted(p for p in set(out) if "_undistorted" not in os.path.basename(p))


def read_image(path):
    cv2 = _cv2()
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise CalibrationError("cannot read image " + path)
    return img


# ------------------------------------------------------------------ lens

def calibrate_lens(detections, image_size, params=None):
    """detections: list of (name, corners Nx2, ids N). Returns a dict (see `save_json`); never writes.

    The result has "good": True only when every threshold at the top of this file is met;
    "problems" lists what failed. Raises CalibrationError when there is too little to solve at all.
    """
    cv2 = _cv2()
    board = make_board(params)
    used = [d for d in detections if len(d[2]) >= LENS_MIN_CORNERS]
    if len(used) < 4:
        raise CalibrationError("only %d usable pictures (board seen with at least %d corners); need at least 4 "
                               "to solve and %d to trust it" % (len(used), LENS_MIN_CORNERS, LENS_MIN_IMAGES))
    obj, img = [], []
    for _, corners, ids in used:
        o, i = board.matchImagePoints(corners.reshape(-1, 1, 2), ids.reshape(-1, 1))
        obj.append(o.reshape(-1, 3).astype(np.float32))
        img.append(i.reshape(-1, 2).astype(np.float32))
    k0 = np.array([[1.2 * image_size[0], 0, image_size[0] / 2], [0, 1.2 * image_size[0], image_size[1] / 2],
                   [0, 0, 1]])
    rms, k, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, tuple(image_size), k0, np.zeros(5),
                                                     flags=cv2.CALIB_USE_INTRINSIC_GUESS)
    per = []
    for (name, _, _), o, i, rv, tv in zip(used, obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, rv, tv, k, dist)
        per.append((name, float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - i) ** 2, axis=1)))), len(o)))
    cov = Coverage(image_size)
    for _, c, _ in used:
        cov.add(c)
    problems = []
    if len(used) < LENS_MIN_IMAGES:
        problems.append("only %d usable pictures, need %d" % (len(used), LENS_MIN_IMAGES))
    if rms > LENS_RMS_MAX_PX:
        problems.append("RMS reprojection error %.2f px is above %.1f px" % (rms, LENS_RMS_MAX_PX))
    if not cov.ok():
        problems.append("coverage too poor: %s (need %d%% of the frame and every corner)" %
                        (cov.summary(), round(100 * COVERAGE_MIN)))
    outliers = [n for n, e, _ in per if e > LENS_IMAGE_MAX_PX]
    if outliers:
        problems.append("pictures with error above %.1f px (blurred, bent board?): %s" %
                        (LENS_IMAGE_MAX_PX, ", ".join(outliers)))
    return {
        "kind": "lens", "good": not problems, "problems": problems,
        "image_size": [int(image_size[0]), int(image_size[1])],
        "camera_matrix": k.tolist(), "dist_coeffs": dist.reshape(-1).tolist(),
        "dist_model": "OpenCV k1 k2 p1 p2 k3",
        "rms_px": float(rms), "warn": rms > LENS_RMS_WARN_PX,
        "per_image": [{"image": n, "rms_px": e, "corners": c} for n, e, c in per],
        "coverage": {"fraction": cov.fraction(), "corners": cov.corner_fractions()},
        "board": dict(DEFAULT_BOARD, **(params or {})),
        "date": now(),
    }


def calibrate_lens_dir(directory, params=None, progress=None):
    dets, size = [], None
    for path in list_images(directory):
        img = read_image(path)
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif size != (img.shape[1], img.shape[0]):
            raise CalibrationError("%s is %dx%d, the others %dx%d" % (path, img.shape[1], img.shape[0], *size))
        c, ids = detect_board(img, params)
        dets.append((os.path.basename(path), c, ids))
        if progress:
            progress(path, len(ids))
    if size is None:
        raise CalibrationError("no images in " + directory)
    return calibrate_lens(dets, size, params)


def load_lens(path):
    with open(path) as f:
        d = json.load(f)
    if d.get("kind") != "lens":
        raise CalibrationError(path + " is not a lens calibration file")
    return np.array(d["camera_matrix"], float), np.array(d["dist_coeffs"], float), tuple(d["image_size"]), d


def undistort(img, camera_matrix, dist, alpha=0.0):
    """The undistorted picture; alpha 0 crops to valid pixels only, 1 keeps every source pixel."""
    cv2 = _cv2()
    h, w = img.shape[:2]
    new_k, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist, (w, h), alpha)
    return cv2.undistort(img, camera_matrix, dist, None, new_k)


def save_json(obj, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(obj, f, indent=1)


# ------------------------------------------------------------------ marker

def marker_object_points(marker_mm):
    """Marker corners in the marker frame (centre origin, x right, y up, z out), in ArUco's order."""
    h = marker_mm / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


def detect_marker(img, marker_id, dictionary="DICT_5X5_100"):
    """The 4 corner pixels (4x2, ArUco order) of marker `marker_id`, or None."""
    cv2 = _cv2()
    det = cv2.aruco.ArucoDetector(_dict(dictionary), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(img)
    if ids is None:
        return None
    for c, i in zip(corners, ids.reshape(-1)):
        if int(i) == marker_id:
            return c.reshape(4, 2).astype(np.float64)
    return None


def marker_pose(corners_px, marker_mm, k, dist):
    """T_C_M from the 4 corner pixels (IPPE_SQUARE, then a few iterations)."""
    cv2 = _cv2()
    ok, rv, tv = cv2.solvePnP(marker_object_points(marker_mm), np.asarray(corners_px, np.float64), k, dist,
                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise CalibrationError("marker pose could not be solved")
    return matrix(rot_of(rv), tv)


# ------------------------------------------------------------------ robot step

def read_samples(path):
    """Read SAMPLES.json. Format:

        {"marker_mm": 60, "marker_id": 90, "dictionary": "DICT_5X5_100",
         "samples": [
           {"image": "p01.jpg",                      # relative to this file; or "corners_px": [[u,v] x4]
            "pose": {"x":..,"y":..,"z":..,"yaw":..,"pitch":..,"roll":..}},   # as the controller reports
           {"corners_px": [[..],[..],[..],[..]], "joints_deg": [j1,..,j6]}   # or six joint angles
         ]}

    Returns (marker_mm, [(name, corners_px (4x2) or None, T_W_T)], skipped) where None corners means
    "image given, detect later". The image route needs the lens file.
    """
    with open(path) as f:
        d = json.load(f)
    base = os.path.dirname(os.path.abspath(path))
    marker_mm = float(d.get("marker_mm", DEFAULT_MARKER["marker_mm"]))
    out = []
    for n, s in enumerate(d["samples"]):
        if "pose" in s:
            p = s["pose"]
            t_wt = pose_matrix(*(float(p[k]) for k in ("x", "y", "z", "yaw", "pitch", "roll")))
        elif "joints_deg" in s:
            from a465 import model
            pos, rot, _, _ = model.forward(s["joints_deg"])
            t_wt = matrix(rot, pos)
        else:
            raise CalibrationError("sample %d has neither 'pose' nor 'joints_deg'" % n)
        if "corners_px" in s:
            c = np.array(s["corners_px"], float).reshape(4, 2)
            name = s.get("name", "sample %d" % n)
        elif "image" in s:
            c = None
            name = os.path.join(base, s["image"])
        else:
            raise CalibrationError("sample %d has neither 'image' nor 'corners_px'" % n)
        out.append((name, c, t_wt))
    return {"marker_mm": marker_mm, "marker_id": int(d.get("marker_id", DEFAULT_MARKER["marker_id"])),
            "dictionary": d.get("dictionary", DEFAULT_MARKER["dictionary"])}, out


def _extent(points):
    """Principal extents (max - min along each principal axis), largest first."""
    p = np.asarray(points, float)
    p = p - p.mean(axis=0)
    _, _, vt = np.linalg.svd(p, full_matrices=False)
    proj = p @ vt.T
    return np.sort(proj.max(axis=0) - proj.min(axis=0))[::-1]


def check_spread(tool_poses):
    """Raise CalibrationError unless the tool poses are numerous and varied enough to fix the solution."""
    n = len(tool_poses)
    if n < ROBOT_MIN_SAMPLES:
        raise CalibrationError("%d usable samples; need at least %d (take %d or more)" %
                               (n, ROBOT_MIN_SAMPLES, ROBOT_RECOMMENDED))
    pos = _extent([t[:3, 3] for t in tool_poses])
    rel = [rvec_of(tool_poses[0][:3, :3].T @ t[:3, :3]) for t in tool_poses]
    rot = np.degrees(_extent(rel))
    if pos[0] < ROBOT_MIN_POSITION_SPREAD_MM[0] or pos[1] < ROBOT_MIN_POSITION_SPREAD_MM[1]:
        raise CalibrationError(
            "tool positions too close together: extents %.0f and %.0f mm along the two main directions, "
            "need %.0f and %.0f mm; spread the poses over the bench" % (pos[0], pos[1], *ROBOT_MIN_POSITION_SPREAD_MM))
    if rot[0] < ROBOT_MIN_ROTATION_SPREAD_DEG[0] or rot[1] < ROBOT_MIN_ROTATION_SPREAD_DEG[1]:
        raise CalibrationError(
            "tool orientations too alike: spread %.0f and %.0f deg about the two main axes, need %.0f and %.0f; "
            "tilt and roll the gripper between poses" % (rot[0], rot[1], *ROBOT_MIN_ROTATION_SPREAD_DEG))


def _residuals(x, tool_poses, corners, marker_mm, k, dist):
    cv2 = _cv2()
    t_wc, t_tm = matrix_from_params(x[:6]), matrix_from_params(x[6:])
    obj = marker_object_points(marker_mm)
    out = []
    for t_wt, c in zip(tool_poses, corners):
        t_cm = inv(t_wc) @ t_wt @ t_tm
        proj, _ = cv2.projectPoints(obj, rvec_of(t_cm[:3, :3]), t_cm[:3, 3], k, dist)
        out.append((proj.reshape(4, 2) - c).reshape(-1))
    return np.concatenate(out)


def _levenberg_marquardt(f, x0, iters=100):
    x, lam = np.array(x0, float), 1e-3
    r = f(x)
    cost = r @ r
    for _ in range(iters):
        jac = np.empty((len(r), len(x)))
        for j in range(len(x)):
            h = 1e-6 * max(1.0, abs(x[j]))
            d = np.zeros(len(x))
            d[j] = h
            jac[:, j] = (f(x + d) - f(x - d)) / (2 * h)
        a, g = jac.T @ jac, jac.T @ r
        for _ in range(12):
            step = np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-12), -g)
            r2 = f(x + step)
            c2 = r2 @ r2
            if c2 < cost:
                x, r, lam = x + step, r2, lam * 0.3
                break
            lam *= 10
        else:
            break
        if np.max(np.abs(step)) < 1e-9 and c2 <= cost:
            cost = c2
            break
        cost = c2
    return x


def solve_robot(tool_poses, corners, marker_mm, k, dist, names=None, lens_file=None):
    """Camera pose in the world frame and marker offset from the tool point.

    tool_poses: list of T_W_T (4x4); corners: list of 4x2 raw marker corner pixels; k, dist: the lens.
    Returns a dict with "T_world_camera", "T_tool_marker" (4x4 lists), residuals per sample, "good".
    """
    cv2 = _cv2()
    check_spread(tool_poses)
    k, dist = np.asarray(k, float), np.asarray(dist, float)
    t_cm = [marker_pose(c, marker_mm, k, dist) for c in corners]
    # OpenCV's robot-world/hand-eye routine names things for a camera on the gripper; with the
    # camera fixed we give it: its "world" = our camera, "cam" = our marker, "base" = our world,
    # "gripper" = our tool. Its outputs are then T_C_W (world to camera) and T_M_T (tool to marker).
    r_w2c = [inv(m)[:3, :3] for m in t_cm]
    t_w2c = [inv(m)[:3, 3].reshape(3, 1) for m in t_cm]
    r_b2g = [inv(t)[:3, :3] for t in tool_poses]
    t_b2g = [inv(t)[:3, 3].reshape(3, 1) for t in tool_poses]
    rb2w, tb2w, rg2c, tg2c = cv2.calibrateRobotWorldHandEye(r_w2c, t_w2c, r_b2g, t_b2g)
    t_wc0 = inv(matrix(rb2w, tb2w))
    t_tm0 = inv(matrix(rg2c, tg2c))
    x0 = np.concatenate([params_of_matrix(t_wc0), params_of_matrix(t_tm0)])
    c = [np.asarray(v, float) for v in corners]
    f = lambda x: _residuals(x, tool_poses, c, marker_mm, k, dist)   # noqa: E731
    x = _levenberg_marquardt(f, x0)
    t_wc, t_tm = matrix_from_params(x[:6]), matrix_from_params(x[6:])
    res = f(x).reshape(-1, 4, 2)
    px = np.sqrt(np.mean(np.sum(res ** 2, axis=2), axis=1))
    # millimetres: where the camera's own view puts the marker centre, against where the tool pose
    # and the solved offset put it; the camera is poor along its viewing direction, so this is the
    # pessimistic of the two measures
    mm = []
    for t_wt, m in zip(tool_poses, t_cm):
        seen = (t_wc @ m)[:3, 3]
        predicted = (t_wt @ t_tm)[:3, 3]
        mm.append(float(np.linalg.norm(seen - predicted)))
    rms_px = float(np.sqrt(np.mean(px ** 2)))
    rms_mm = float(np.sqrt(np.mean(np.square(mm))))
    problems = []
    if rms_px > ROBOT_RMS_MAX_PX:
        problems.append("RMS %.2f px is above %.1f px" % (rms_px, ROBOT_RMS_MAX_PX))
    if px.max() > ROBOT_MAX_PX:
        problems.append("worst sample %.1f px is above %.1f px" % (px.max(), ROBOT_MAX_PX))
    if rms_mm > ROBOT_RMS_MAX_MM:
        problems.append("RMS %.1f mm is above %.1f mm" % (rms_mm, ROBOT_RMS_MAX_MM))
    names = names or ["sample %d" % i for i in range(len(tool_poses))]
    return {
        "kind": "robot", "good": not problems, "problems": problems,
        "T_world_camera": t_wc.tolist(), "T_tool_marker": t_tm.tolist(),
        "camera_position_mm": t_wc[:3, 3].tolist(), "marker_offset_mm": t_tm[:3, 3].tolist(),
        "marker_mm": marker_mm, "n_samples": len(tool_poses), "lens_file": lens_file,
        "rms_px": rms_px, "rms_mm": rms_mm,
        "per_sample": [{"sample": n, "px": float(p), "mm": m} for n, p, m in zip(names, px, mm)],
        "date": now(),
    }


def solve_robot_file(samples_path, lens_path):
    k, dist, size, _ = load_lens(lens_path)
    meta, samples = read_samples(samples_path)
    tools, corners, names, skipped = [], [], [], []
    for name, c, t_wt in samples:
        if c is None:
            img = read_image(name)
            if (img.shape[1], img.shape[0]) != tuple(size):
                raise CalibrationError("%s is %dx%d but the lens was calibrated at %dx%d" %
                                       (name, img.shape[1], img.shape[0], *size))
            c = detect_marker(img, meta["marker_id"], meta["dictionary"])
            if c is None:
                skipped.append(name)
                continue
        tools.append(t_wt)
        corners.append(c)
        names.append(os.path.basename(name))
    result = solve_robot(tools, corners, meta["marker_mm"], k, dist, names, os.path.abspath(lens_path))
    result["skipped_no_marker"] = skipped
    return result


# ------------------------------------------------------------------ projection

def load_robot(path):
    with open(path) as f:
        d = json.load(f)
    if d.get("kind") != "robot":
        raise CalibrationError(path + " is not a camera-to-robot calibration file")
    return np.array(d["T_world_camera"], float), np.array(d["T_tool_marker"], float), d


def world_to_pixel(points_w, t_wc, k, dist):
    """Pixels (Nx2, in the raw distorted picture) of world points (Nx3, mm)."""
    cv2 = _cv2()
    p = np.atleast_2d(np.asarray(points_w, float))
    t_cw = inv(t_wc)
    cam = p @ t_cw[:3, :3].T + t_cw[:3, 3]
    if np.any(cam[:, 2] <= 0):
        raise CalibrationError("point is behind the camera")
    px, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), np.asarray(k, float), np.asarray(dist, float))
    return px.reshape(-1, 2)


def pixel_to_world_plane(pixels, z_mm, t_wc, k, dist):
    """World points (Nx3) where the rays through raw pixels (Nx2) meet the plane z = z_mm."""
    cv2 = _cv2()
    uv = np.atleast_2d(np.asarray(pixels, float)).reshape(-1, 1, 2)
    n = cv2.undistortPoints(uv, np.asarray(k, float), np.asarray(dist, float)).reshape(-1, 2)
    rays_c = np.column_stack([n, np.ones(len(n))])
    rays_w = rays_c @ t_wc[:3, :3].T
    origin = t_wc[:3, 3]
    if np.any(np.abs(rays_w[:, 2]) < 1e-9):
        raise CalibrationError("a ray runs parallel to the plane")
    s = (z_mm - origin[2]) / rays_w[:, 2]
    if np.any(s <= 0):
        raise CalibrationError("the plane z = %g is behind the camera along that pixel's ray" % z_mm)
    return origin + rays_w * s[:, None]
