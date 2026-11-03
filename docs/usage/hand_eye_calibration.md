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
- Clear the table within the arm's reach. The planner's floor obstacle (`floor_z`) and its own collision model are what keep the arm off the table; nothing on the table is known to it.

## Run `xarm6-handeye-calibrate`

Terminal 1:

```bash
dimos run xarm6-handeye-calibrate --xarmconnection.ip 192.168.1.185 --realsensecamera.serial-number <serial>
```

Terminal 2, with the e-stop in hand:

```bash
dimos shell
```

```python
cal = app.get_module("HandEyeCalibrationModule")
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
