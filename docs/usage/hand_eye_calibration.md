# Hand-eye calibration

Operator workflow for calibrating a camera against an arm with `HandEyeCalibrationModule`: where a wrist camera sits on the flange, or where a fixed camera sits in the arm's base. The run drives the arm through the planner, detects a printed ChArUco board in every frame, solves the camera pose, and writes it where the arm's blueprints read it. See [Camera calibration](/docs/usage/camera_calibration.md) for intrinsics; this page assumes the camera's `camera_info` is right.

The module is composed per arm. The xArm6 wrist camera has `xarm6-handeye-calibrate`; the module config in `dimos/robot/manipulators/xarm/blueprints/calibration.py` is the reference for another arm.

## Print and measure the board

The default board is 8 by 5 squares of 30 mm with AprilTag 36h11 markers of 22 mm, ids from 0. Generate it with OpenCV:

```python
import cv2

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
board = cv2.aruco.CharucoBoard((8, 5), 0.030, 0.022, d)
cv2.imwrite("charuco_8x5_30mm.png", board.generateImage((2400, 1500), marginSize=0))
```

1. Print at 100 percent on matte paper. Turn off "fit to page".
2. Glue it flat on rigid backing. A bent board moves between poses and fails the misclosure check.
3. Measure the pitch of all eight squares along the long edge with a rule and divide by eight. A print 2 percent large gives a 2 mm wrong answer, which the verdict catches; 1 percent passes with a 1 mm error, which it does not.
4. Set `board.square_m` to the measured pitch and `board.marker_m` to 0.7333 times it.

## Set up the bench

- Wrist camera: tape the board flat about 0.45 m in front of the base. Jog the arm with teleop to the home pose: tool pointing down about 0.4 m over the board, board in the middle of the frame. The default `home_joints` are a starting point; what matters is the view.
- Fixed camera: fix the board in the gripper, set `camera_frame` to `base`, and put the camera's tape-measured position in `nominal_translation`. The planner keeps the arm `camera_keepout_m` away from it.
- Clear the table within the arm's reach. Measure two things: the height of the highest surface within reach relative to the base plate, the board on its backing included, plus 15 mm of margin (positive when above the plate), and the distance from the base centre to the table's near edge. The planner keeps the fingertips above that height, so a value below the board's surface lets a tilted pose touch it (it did, on 2026-11-03, with the table top given and the 8 mm board forgotten). They go on the command line as `--handeyecalibrationmodule.table-top-z <m>` and `--handeyecalibrationmodule.table-x-m "[<edge m>, 1.0]"`; the blueprint's defaults are a table level with the base plate from 0.1 m out. The module adds the table to the planner as a box for the run. Nothing else on the table is known to the planner.

## Run `xarm6-handeye-calibrate`

Terminal 1:

```bash
dimos run xarm6-handeye-calibrate --xarmconnection.ip <arm ip> --realsensecamera.serial-number <serial> --handeyecalibrationmodule.table-top-z <m> --handeyecalibrationmodule.table-x-m "[<edge m>, 1.0]"
```

Terminal 2, with the e-stop in hand:

```bash
dimos shell
```

```python
cal = app.get_module("HandEyeCalibrationModule")
```

An operator-approved run can wait on `approve()` longer than an RPC client waits for `calibrate()`, so start it in the background and poll:

```python
cal.start_calibration()
cal.status()            # running, moves, waiting_for_approval; last_result when the run is over
cal.approve()           # from a second shell, once per gated move
```

The solver asks for views from different heights on its own: it servos the mean camera-to-board distance onto 0.4 m from either side and then alternates nearer and farther views, so one run from the default home holds the spread that separates a shift of the camera along its viewing axis from a tilt.
```
cal.calibrate()      # blocks until the verdict
```

The run waits before each of the first `approve_first_moves` moves and logs the target. Release each with `cal.approve()` from another shell, or run unattended with `--handeyecalibrationmodule.approve-first-moves 0`. `cal.abort()` cancels the running trajectory and ends the run. `cal.status()` shows progress.

`--handeyecalibrationmodule.dry-run true` plans every target and executes none; the log then says how many the planner accepted.

## Read the result

`calibrate()` returns the verdict, the confidence numbers, and the transform in two forms: the solved colour optical frame, and the mount (`link6` to `camera_link`) that a blueprint uses. The same is written to `STATE_DIR/calibration/<name>.json`. The xArm grasp blueprints read `xarm6_wrist_camera.json` from there when it exists and has `passed: true`.

`passed` means all of these held:

| check | meaning |
|---|---|
| probe error | the point 0.4 m in front of the camera moves under 2 mm when the poses are resampled |
| worst resample | no subset of poses moves it over 4 mm |
| misclosure | the board lands within 1 mm of one place from every pose |
| deviation from nominal | the solve is within twice `nominal_tolerance_m` of the stated mount |
| poses | at least 4 views solved |

`reason` names the checks that failed. `too few detections` means the camera did not see the board from the draws: fix the home pose or the lighting. A large `deviation_from_nominal_mm` with everything else clean means the mount is not where the config says; raise `nominal_tolerance_m` only after measuring why.

Run it three times. The three mounts should agree within 0.5 mm.

## Test without an arm

`dimos run xarm6-handeye-calibrate-sim` is the same stack over a MuJoCo bench with the board on the table; `calibrate()` from `dimos shell` then runs the same code path. The simulated camera's true pose is on tf as `link6` to `wrist_camera_color_optical_frame`, so the result can be checked against it.

## A fixed camera: the chest run

`xarm6-handeye-calibrate-chest` is the mirror run. The printed gripper plate (5 by 4 ChArUco, 30 mm squares, 22.5 mm markers, ids from 100; `gripper_plate_A4.pdf`, glued flat on a stiff backing) is held in the closed fingers and a RealSense on a post beside the bench watches it. The result is the camera's pose in the arm's base, written as `xarm6_chest_camera`.

Site inputs, on top of the arm address and the camera serial: where the lens is and where it looks, base frame, from a tape measure (base centre to the lens, positive y to the arm's left as seen from behind it), as `--handeyecalibrationmodule.nominal-translation "[x, y, z]"`. The rotation is taken from the default look-at (the middle of the workspace); if the camera looks elsewhere, give `--handeyecalibrationmodule.nominal-rotation-xyzw` from the same convention as the wrist run (OpenCV optical axes: z along the view, x image right, y image down). The verdict allows the solve 0.30 m from the stated position, so a tape measure is enough. The run keeps every part of the arm 0.15 m from the stated lens position (`camera_keepout_m`).

The home pose holds the plate out in front of the arm with its face toward a camera on the arm's right; a camera on the left needs `--handeyecalibrationmodule.home-joints` mirrored (joint 1 and joint 6 negated). Check once on the camera's live view that the whole plate is in frame at the home pose before approving the first move.

`dimos run xarm6-handeye-calibrate-chest-sim` runs the same stack over a bench with the plate in the gripper and the camera on a post, where the camera's true pose is known.
