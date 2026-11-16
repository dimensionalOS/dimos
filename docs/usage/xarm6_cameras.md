# xArm6 with two calibrated cameras

The bench xArm6 carries two RealSense D435: one on the kit wrist mount and one on a post beside the bench, the chest camera. Both are hand-eye calibrated against the arm, the results ship with the package, and `xarm6-cameras` brings the arm and both cameras up with every cloud and image already in the arm's frame. Build on that blueprint; nothing here needs a tape measure.

## What is in the frame tree

| camera | module instance | tf edge | published by | topics |
|---|---|---|---|---|
| wrist | `realsensecamera` | `link6` to `camera_link` | the planner, from `xarm6_wrist_camera.json` | `color_image`, `depth_image`, `camera_info`, `pointcloud` |
| chest | `chest/realsensecamera` | `world` to `chest/camera_link` | the planner, from `xarm6_chest_camera.json` | `chest/color_image`, `chest/depth_image`, `chest/camera_info`, `chest/pointcloud` |

`world` is the arm's base (`link_base` at the origin). Each camera's own imager frames hang off its link the way the RealSense module always publishes them: `camera_color_optical_frame`, `chest/camera_color_optical_frame`.

The chest camera runs under the `chest` blueprint namespace, which is what gives it separate topics and frames. On the command line its module is `chest_realsensecamera`.

## Run it

```bash
dimos run xarm6-cameras --xarmconnection.ip <arm ip> --realsensecamera.serial-number <wrist serial> --chest_realsensecamera.serial-number <chest serial>
```

The planner's viser view is at http://127.0.0.1:8095. The Rerun viewer opens with the stack (`--viewer none` leaves it out). Both point clouds should lie on the same table top; a box on the table appears once, not twice. If the two clouds are offset, one camera has moved since its calibration: re-run that camera's calibration below.

Nothing moves until something plans a motion. `dimos shell` gives the planner as `app.get_module("ManipulationModule")`.

## Where the calibrations come from, and which one wins

The results are files in the `save_hand_eye` layout: the camera's mount as a transform, the solver's confidence numbers and `passed`. Two copies can exist:

1. A run on this machine, written by the calibration blueprints to `~/.local/state/dimos/calibration/<name>.json`. Wins when present and passed.
2. The measured result shipped with the package, `dimos/robot/manipulators/xarm/calibration/<name>.json`.

A failed or aborted run writes `<name>_failed.json` and never replaces a passed result. The loader is `dimos.robot.manipulators.xarm.cameras`.

Shipped today:

| file | measured | result |
|---|---|---|
| `xarm6_wrist_camera.json` | 3 November 2026, kit wrist mount, run 8 | 13 of 14 views, probe error 1.5 mm, misclosure 2.7 mm; ruler check 1 to 2 mm in the default posture family |
| `xarm6_chest_camera.json` | 7 November 2026, post to the arm's right | 14 of 14 views, probe error 1.1 mm, misclosure 1.6 mm; lens 0.32 m forward, 0.46 m right, 0.71 m up |

The wrist result holds for that mount with the camera seated the same way. The chest result holds for that post where it stood on 7 November; a camera that has been moved needs a new run, which takes about five minutes.

## Re-calibrate

Both procedures are in [Hand-eye calibration](/docs/usage/hand_eye_calibration.md). The short form, for this bench:

Wrist camera, board taped flat on the table about 0.45 m in front of the base:

```bash
dimos run xarm6-handeye-calibrate --xarmconnection.ip <arm ip> --realsensecamera.serial-number <wrist serial> --realsensecamera.enable-depth false --realsensecamera.align-depth-to-color false --handeyecalibrationmodule.table-top-z 0.11 --handeyecalibrationmodule.table-x-m "[0.08, 1.0]"
```

Chest camera, printed plate held in the closed fingers with its pattern toward the camera, camera on the arm's right:

```bash
dimos run xarm6-handeye-calibrate-chest --xarmconnection.ip <arm ip> --realsensecamera.serial-number <chest serial> --realsensecamera.enable-depth false --realsensecamera.align-depth-to-color false --handeyecalibrationmodule.table-top-z 0.11 --handeyecalibrationmodule.table-x-m "[0.08, 1.0]" --handeyecalibrationmodule.home-joints "[-0.212, -0.099, -1.161, -0.773, 1.028, 2.95]" --handeyecalibrationmodule.nominal-tolerance-m 0.3 --handeyecalibrationmodule.nominal-tolerance-deg 20
```

Then from `dimos shell`: `cal = app.get_module("HandEyeCalibrationModule")`, `cal.start_calibration()`, and `cal.approve()` for each of the first three moves after checking the plan in viser. The verdict is `cal.status()["last_result"]`. A pass writes the state file, which the cameras blueprint reads on its next start.

## Known limits of this arm

The bench xArm6 (XI1303) has a laterally soft joint 4. A point commanded in the default wrist posture family lands within 1 to 2 mm of where the cameras put it; the same point with the forearm flipped half a turn lands up to 9 mm away. Both calibrations are correct to the arm; the difference is the arm's. Plan work in the default posture family until the joint is serviced.
