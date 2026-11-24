# Copies of the controller's files

`backup-2026-11-23/` was taken with `scripts/a465-getfile` on 2026-11-23. Every file was checked
against the controller's own checksum and size before it was saved.

- `conf/`: the calibration file `robot.cal` (and the copy made beside it on the controller,
  `robot.cal.bak`), the configuration `robot.cfg`, the startup scripts and the gripper calibration.
- `app/`: the applications' variable files. `app/caco/caco2.v3` holds the old lab cell's taught
  locations; `app/test/` has the two stock example programs and their sources.
- `log/robot.odo`, `lib/*.ndx`: small state files.
- `listing.txt`: the controller's clock, odometer, a full dated listing and its boot log, as read.

There is no way yet to write a file back to the controller, so these are a record, not a restore
image. What the files mean is in `docs/CONTROLLER.md`; what their dates show is in
`docs/TIMELINE.md`. One application folder, `/app/new _p0`, was not copied because of the space
in its name.
