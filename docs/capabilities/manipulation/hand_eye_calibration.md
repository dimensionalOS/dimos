# Eye-in-hand calibration

`dimos handeyecalibrate run` measures the transform from a wrist camera’s optical
frame to the robot link carrying it. The planner moves the arm, you approve one
image per pose, and DimOS fits the transform using **8 training views** and checks
it using **4 separate validation views**. These are the minimum counts. For 15
training and 5 validation images, add `--count 15 --validation-count 5`.

## Before you start

- Rigidly attach the camera to the mounting link. Use a flat **tag36h11, ID 0**
  with a **150 mm outer black-square edge**, excluding the white margin. For a
  different tag in this family, pass `--marker-id` and `--marker-size-m`.
- Start the camera driver. It must publish an 8-bit color or grayscale `Image`
  and `CameraInfo` for the same optical frame and pixels. The supported distortion
  models are `plumb_bob` and `rational_polynomial`. Intrinsics are read once.
  Keep resolution, crop, focus and zoom fixed during capture.
- Start a planner/coordinator with one pose-capable group of rotational arm
  joints, one measured `joint_state` output, and TF from the group base to the
  camera’s mounting link. Timestamp images and arm feedback on the same clock.
- Choose a `--home` that centers the tag, with one angle in radians per joint in
  the planner’s order. Pass the mounting link as `--ee-frame`. Check clearance
  for the arm, camera and cable through the tilts and the return move. The A1Z
  return pose is given below. Other arms default to zero radians at every joint.
  Automatic review requires a graphical display.

| Input | Default topic | Override |
| --- | --- | --- |
| Image | `zenoh:dimos/color_image` | `--image-topic` |
| Intrinsics | `zenoh:dimos/camera_info` | `--camera-info-topic` |
| Measured TF | `zenoh:dimos/tf` | `--tf-topic` |

## Example with a Galaxea A1Z and RealSense D405

The D405 is mounted on `arm_link6`. The A1Z planner moves the gripper tool
center point (TCP), `gripper_eef_link`. The D405 driver supplies factory color
intrinsics on `camera_info`.

In the first terminal, start the stack with the gripper enabled (the default).
The A1Z connection can move at startup and shutdown. See the
[A1Z hardware guide](/docs/capabilities/manipulation/a1z.md).

```sh skip
dimos run a1z-planner-coordinator real-sense-camera --a1zconnection.can-port a1zcan
```

If your Apple Silicon Mac needs administrator access for the camera and uses
Homebrew libraries in `/opt/homebrew/lib`, use this form from the checkout.
Add `--realsensecamera.serial-number DEVICE_SERIAL` to select a specific D405.

```sh skip
sudo env DYLD_FALLBACK_LIBRARY_PATH="/opt/homebrew/lib${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}" \
  "$PWD/.venv/bin/dimos" run a1z-planner-coordinator real-sense-camera \
  --a1zconnection.can-port a1zcan
```

In a second terminal in the same DimOS environment, start with the arm still.
The `--home` angles below center the tag. After calibration, the A1Z returns
to `0 0.05 -0.05 0 0 0` radians (J1 through J6), placing J2 and J3 about
2.9° inside their respective joint limits.

```sh skip
dimos handeyecalibrate run --session ./hand-eye-01 \
  --home 0 1.0 -1.0 1.1 0 0 --ee-frame arm_link6
```

## Capture, solve and validate

1. The arm moves to `--home`. Center the tag on the preview crosshair and secure
   it relative to the base. If you adjust it, press **R** in the image window
   to refresh. Keep the target, base and mount fixed from the first accepted image.
2. Press **Enter** (Return on Mac) to accept the displayed image, or **R** to
   retake it. The command saves an accepted image and its measured mount pose,
   then moves to the next view. The planner targets the same TCP position and
   varies its orientation. The solver uses the measured mount pose at exposure.
   Training starts at home, then tilts 11°
   around a ring. The validation ring uses 7° and starts half a validation
   step around the ring.
   If the tag is rejected at a ring pose, the planner tries half the tilt once.
3. After training, DimOS fits `camera_to_ee`. Each training view now gives an
   estimate of the tag pose in the base frame. DimOS averages those positions
   and rotations to get one reference pose. If training passes, DimOS uses the
   saved transform on each held-out image and compares its tag pose with that
   reference. Both transforms stay fixed throughout validation.
4. A completed run saves `hand_eye.json` and reports **PASS** or **FAIL**. If
   the run moved the arm, it requests the return pose and verifies
   arrival within the A1Z parking tolerance of 0.05 rad (about 2.86°) per joint.
   Other arms use 2° per joint. If that move fails, the error gives the path to
   the saved result. Check the file’s `validated` flag before using the transform.

### Stop or retry

While reviewing an image, press **Q**, **Esc**, or close the window to stop at
that pose. Use **Ctrl-C** in the calibration terminal to interrupt a move. Repeat the same
command and session path to resume at the first unsaved view. Once you have
accepted the first image, the saved home tool pose anchors the remaining views.
Reopening a fully captured session repeats grading without moving the arm.

If a move misses its goal, check the reported joint or TCP error and wrist
clearance. For a failed solve or validation, check tag size and flatness,
mount rigidity, image sharpness, pose diversity and timestamps. Start a fresh
session after moving the target or changing the mount, intrinsics, home or view
schedule. Older schedules can still be graded with `solve` and `validate`.

### Pass criteria

| Check | Requirement |
| --- | --- |
| Timing | Image acquired after the capture request, age ≤1 s. Each TF edge within 150 ms of image time. |
| Stationary capture | At least 0.5 s within 1 mm and 0.5° of the first measured mount pose. |
| Automatic arrival | Home joints within 2° of the request. TCP within 10 mm and 2° of each ring goal. Measured joints within 150 ms of exposure and within 2° of the reached posture. |
| Detection | Exactly one matching tag ID, reprojection RMS ≤2 px. |
| Training geometry | At least 8 views with rotations ≥15° about two axes separated by ≥30°. |
| Target consistency | All training and validation views must agree with the training reference within 10 mm and 2°. Validation requires at least 4 held-out views. |

A failed quality check gives a nonzero exit status. These limits measure
repeatability against a fixed target. Absolute accuracy requires an independent
measurement.

## Using the saved result

`session.json` records the intrinsics, frames and observations paired with the
numbered PNGs. In `hand_eye.json`, `parent_frame` is the mounting link and
`child_frame` is the camera optical frame. `camera_to_ee` is a 4×4 matrix with
translation in metres. For a point expressed as `[x, y, z, 1]` in the optical
frame, `camera_to_ee @ point` gives the point in the mounting link. Installation
into TF is a separate step.

## Manual capture or offline grading

For an arm you position yourself, use a separate session and a stack publishing
measured TF. The example assumes `base_link` and `arm_link6`. Use `--base-frame`
and `--ee-frame` for other links. Hold still and press Enter in the terminal at
each pose. Use `q` to end capture. Tilt about at least two axes for training,
then choose new poses for validation.

```sh skip
dimos handeyecalibrate capture --session ./hand-eye-manual --count 8
dimos handeyecalibrate solve --session ./hand-eye-manual
dimos handeyecalibrate capture --session ./hand-eye-manual --validation --count 4
dimos handeyecalibrate validate --session ./hand-eye-manual
```

`capture` reads live intrinsics by default. To use your own camera calibration,
pass `--intrinsics PATH` with a matching color `CameraInfo` YAML. See the
[camera calibration guide](/docs/usage/camera_calibration.md).

Use one writer per session. `solve` and `validate` also work offline on saved
sessions. After adding training images, repeat both commands. After adding
only validation images, repeat `validate`. Python callers can use
`dimos.calibration.hand_eye_run.run_hand_eye` with motion, capture and review
callbacks.
