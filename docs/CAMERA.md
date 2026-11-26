# Bench camera calibration

Written 2026-11-25. Tooling is built and tested on synthetic data; nothing has been captured on the
real camera yet (see "Not done yet").

## The camera

A Sony IMX477 on an NVIDIA Jetson (host `manhatten`, 192.168.1.96), fixed in the cell, looking at the
arm and bench through a wide lens with clear barrel distortion. The service on the Jetson serves a
1920x1080 JPEG at `http://192.168.1.96:8080/snapshot.jpg` and an MJPEG stream at `/stream.mjpg`.
A snapshot fetched on 2026-11-25 shows the arm from the front-left at close range, the bench top
and vise below it, and the shelves and wall behind. The code is `a465/camcal.py`; the command is
`scripts/a465-camcal`. OpenCV 4.10.0 with the aruco module and numpy 2.3.5 were already installed
on the dev box (checked 2026-11-25); nothing was installed. The board pages are made with Pillow
(also already installed).

## Step 1: the lens (camera matrix and distortion)

Commands, in order:

    scripts/a465-camcal target            # writes docs/camera/*.pdf, .png and a .json beside each
    scripts/a465-camcal grab              # Enter = one snapshot, q = finish
    scripts/a465-camcal lens docs/camera/lens-shots
    scripts/a465-camcal undistort docs/camera/lens-shots/snap_001.jpg

`grab` also takes `--count N --every SECONDS` (no Enter key), `--url`, and `--out DIR`.
`lens` writes `docs/camera/lens.json` (or `--out PATH`); it writes nothing, and exits with status 1, if
the result is not good (`--force` writes it anyway, marked `"good": false`).

The target is a ChArUco board, 7 x 9 squares of 25 mm, 18 mm markers, dictionary DICT_5X5_100, on one
US Letter page (`docs/camera/charuco_7x9_25mm_DICT_5X5_100.pdf`, 300 dpi; the parameters are in the file
name and the `.json` beside it). The page carries its parameters and a 100 mm check line.

What the operator does:

1. Print the PDF at 100% (no "fit to page"). Measure the 100 mm line with a ruler; if it is not 100 mm,
   fix the print before going on. Glue the page flat to something rigid (a clipboard back, foam
   board, a ceramic tile); a curled page ruins the result. Matte paper, not glossy.
2. Do not change the camera's focus or zoom after this step, and do not move or re-aim the camera.
   Every later picture, and the robot step, assumes this lens setting. Set focus for the bench
   distance first, then capture.
3. Run `grab`. Take 20 to 30 pictures (10 usable is the minimum the tool accepts). Hold the board
   still for each one (rolling shutter and motion blur). The board must be fully visible. Move it so that
   over the set it is placed at the left, right, top and bottom edges and in all four corners
   of the frame, where the distortion is largest, plus the middle; tilt it up to about 30 to 45
   degrees in different directions on some pictures; use two or three distances (the distance the
   gripper works at, nearer and farther). After each picture `grab` says whether the board was found,
   how many corners, and what share of the frame is covered and which corners are still empty.
   Keep going until it says all four corners are reached and the coverage is at least 55%.
4. Run `lens`. Then run `undistort` on one picture and look at it: lines that are straight in the
   room (shelf edges, the bench edge) should be straight in the result.

Thresholds (set in `a465/camcal.py`; they are my choice, not measured on this camera):

- at least 10 usable pictures, each with at least 12 board corners found: fewer leaves the five
  distortion terms poorly determined;
- RMS reprojection error at most 1.0 px (refused above); at most 0.5 px is expected from a good set
  and 0.5 to 1.0 is flagged as acceptable but worth a look; with 1920x1080 pictures and a sharp print,
  well under 0.5 px should be reachable (an expectation, not yet measured);
- any single picture above 2.5 px is named as an outlier (blur, a bent board): delete it and rerun;
- coverage: the board's corner hulls over all pictures must cover at least 55% of the frame and at least
  20% of each of the four corner boxes (each 20% x 20% of the frame). A low RMS from pictures that
  all sit in the middle says nothing about the edges, which is why coverage is a refusal.

The model is OpenCV's standard five terms (k1, k2, p1, p2, k3) with the pinhole matrix, at 1920x1080.
A picture of any other size is refused by the later steps.

## Step 2: camera to robot

Question answered: where the fixed camera sits in the robot's world frame (X forward, Z up, origin at
the centre of the base flange, as `a465/model.py`), and where the marker sits relative to the tool
point. One ArUco marker (id 90, DICT_5X5_100, black square 60 mm, from
`docs/camera/aruco_id90_60mm_DICT_5X5_100.pdf`) is fixed flat to the gripper. The arm is taken to
N poses and stopped; for each pose the controller's tool pose and the marker in a picture are
recorded. The marker's offset from the tool point is not needed in advance: the solver finds the camera
pose and the offset together.

Method: the marker pose in each picture comes from the four corners (solvePnP, IPPE_SQUARE) with the lens
from step 1. OpenCV's `calibrateRobotWorldHandEye` (present in 4.10, used with the roles mapped as the
code comments say) gives a starting answer, then a Levenberg-Marquardt refinement of the 12 unknowns
minimises the pixel distance between the observed marker corners and the corners predicted from
(tool pose, offset, camera pose). The synthetic test starts about 2.5 mm off and ends within a few mm.

Samples file (JSON, image paths relative to the file):

    {"marker_mm": 60, "marker_id": 90,
     "samples": [
       {"image": "p01.jpg", "pose": {"x": 400.1, "y": -20.3, "z": 250.0, "yaw": 5, "pitch": -30, "roll": 12}},
       {"corners_px": [[u,v],[u,v],[u,v],[u,v]], "joints_deg": [10, -40, -50, 0, 20, 0]}
     ]}

Each sample gives either `image` (the marker is looked for in it) or `corners_px` (raw, distorted pixels,
in ArUco order: top-left, top-right, bottom-right, bottom-left of the marker as printed), and either `pose`
(what the controller reports: millimetres and degrees, yaw/pitch/roll as Rz Ry Rx) or `joints_deg` (six
angles, converted with `a465.model.forward`). A picture in which the marker is not found is listed and
skipped.

Command:

    scripts/a465-camcal robot SAMPLES.json --lens docs/camera/lens.json     # writes docs/camera/robot.json

It prints, per sample, the pixel residual (RMS of the four corners) and the millimetre residual (where the
camera's view puts the marker centre against where tool pose plus offset puts it; this one is dominated by
the camera's poor depth sense, so it is a pessimistic measure). It is called good if RMS is at most 2.0 px,
the worst sample at most 5 px and RMS at most 5 mm; otherwise it is not saved.

Minimum it accepts, and what is asked for:

- at least 8 usable samples (12 or more asked for);
- tool positions spread so that their two largest principal extents are at least 200 mm and 100 mm;
- tool orientations spread so that, as rotation vectors relative to the first pose, the two largest
  extents are at least 40 and 20 degrees (so tilt and roll must change, not just move the gripper
  around keeping it pointing the same way).
  Fewer or tighter sets are refused with a message saying which limit failed.

What the operator does (with the arm under its own control, at least two people or the pad):

1. Fix the printed marker flat and rigid to the gripper, facing where the camera can see it in all the poses.
   Measure and record the printed size with a ruler. The marker must not move on the gripper from the
   first pose to the last; if it is removed, start over.
2. Do not touch the camera, its focus or zoom since step 1.
3. Take the arm to 12 to 15 poses over the bench volume it will work in, with the marker facing the camera
   (no more than about 60 degrees off the line of sight), at varied heights, and with the tool tilted and
   rolled between poses. Stop the arm at each pose and wait a second before capturing: the sensor is a
   rolling shutter, so a moving marker is skewed.
4. At each pose take one snapshot and, in the same moment, read the controller's tool pose (X, Y, Z, yaw,
   pitch, roll) or the six joint angles; put them in the samples file. The snapshot and pose must
   belong to the same stopped position.
5. Run `robot`; look at the per-sample residuals. A single sample far above the rest is a mismatched
   pose or a blurred picture: remove it and rerun.

## Using it

    scripts/a465-camcal project --point 300 0 20                 # world mm to pixel
    scripts/a465-camcal project --pose X Y Z YAW PITCH ROLL      # where the tool point and the marker appear
    scripts/a465-camcal project --pixel 1000 700 --plane-z 20    # pixel to the world point on z = 20 mm

`--lens` and `--robot` default to `docs/camera/lens.json` and `docs/camera/robot.json`. The pixel
route is the one for picking parts off the bench: give the height of the bench top or the part's top face.
It refuses a plane behind the camera or a ray parallel to the plane. In code: `camcal.world_to_pixel` and
`camcal.pixel_to_world_plane`.

## Accuracy, and what limits it

- Resolution at the bench, an estimate: judging from the 2026-11-25 snapshot by eye, about 1 mm per pixel
  near the arm, coarser farther away and nearer the corners. This is not measured. After step 1 it can be
  measured: the 175 mm wide board seen at the working distance spans a known number of pixels.
- The pixel-to-bench-plane answer is good to a few pixels at best, so expect a few mm at best, and worse
  toward the picture edges if the lens fit is weaker there. These are expectations, not
  results; the real accuracy is to be found by the check below.
- Measuring it: after the robot step, take the arm to poses that were not used, and compare
  `project --pose` with where the marker actually appears.
- Rolling shutter: capture with the arm stopped. The same applies to the board.
- The arm model gave Z 0.4 mm low at one pose on 2026-11-25; X and Y exact. The robot step takes tool poses as
  true, so errors in the arm's own calibration (and any offset between controller pose and model) go
  into the result as residual.
- A camera moved, bumped, or refocused invalidates both files; redo both steps.
- Marker size error scales the depth; measure the printed marker.
- A single marker's pose is weakest along the viewing direction, hence the mm residual above.

## Not done yet

- No picture of the board has been taken; `lens.json` does not exist. The tests use synthetic images.
- The marker has not been printed or mounted; no robot samples exist; `robot.json` does not exist.
- The synthetic tests do not cover real lens behaviour (focus breathing, a lens that is not well
  described by five terms; for a very wide lens a fisheye model might fit better, to be judged from the
  `undistort` picture and the edge residuals).
- The pixel-error thresholds are chosen from experience and the synthetic runs, not yet from the real camera.
- Locating parts on the bench from the camera is not started. It needs both files.
- `grab` and the whole pipeline have not been run against the real snapshot service beyond one fetch of
  the image used to write this.
