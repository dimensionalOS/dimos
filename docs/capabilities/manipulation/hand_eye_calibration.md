# Eye-in-hand calibration

Eye-in-hand calibration measures the transform from a wrist camera’s optical frame into its mounting link. Call the calibration skill for unattended capture, use `dimos handeyecalibrate run` for per-image review, or position the arm yourself. All three use the same solver and pass criteria.

## Before capture

- Attach the camera rigidly. The default target is a flat **tag36h11, ID 0**, measuring **150 mm across the outer black square**. The CLI accepts `--marker-id` and `--marker-size-m` for another tag in that family. Keep the tag fixed relative to the base through validation.
- Publish an 8-bit color or grayscale `Image`, matching `CameraInfo` and measured base-from-mount TF. Timestamp images and arm feedback on the same clock. Keep resolution, crop, focus and zoom fixed. Supported distortion models are `plumb_bob` and `rational_polynomial`.
- Both planner-driven modes need a planner/coordinator with one pose-capable arm group and one measured `joint_state` output. Check arm, camera and cable clearance through both rings and the return move. CLI image review also needs a display.

**Supply `home` and `ee_frame` each time you start a planner-driven calibration.** `home` is the pose that centers the tag, with one angle per joint in planner order, in radians. `ee_frame` is the TF name of the link carrying the camera. Pass `--home` and `--ee-frame` to the CLI, include both fields in the MCP call, or give both values to the chat agent.

| Stream | Default topic | CLI override |
| --- | --- | --- |
| Color or grayscale image | `zenoh:dimos/color_image` | `--image-topic` |
| Camera intrinsics | `zenoh:dimos/camera_info` | `--camera-info-topic` |
| Measured TF | `zenoh:dimos/tf` | `--tf-topic` |

Select the camera at launch by adding its DimOS module or blueprint to `dimos run`. If a camera already publishes the image and matching intrinsics, start just the calibration stack. For MCP or chat, point the skill at nondefault topics with `--handeyecalibrationskills.image-topic` and `--handeyecalibrationskills.camera-info-topic` on the launch command.

## A1Z and D405 example

Mount the RealSense D405 on `arm_link6` of the Galaxea A1Z. The planner moves the gripper tool center point (TCP), `gripper_eef_link`. The D405 supplies factory intrinsics. Choose one stack and leave the gripper enabled (the default). The A1Z can move at startup and shutdown. [A1Z hardware guide](/docs/capabilities/manipulation/a1z.md)

### Unattended calibration via MCP

Start `a1z-hand-eye-calibration-agent` with `real-sense-camera` for the D405. For a direct MCP demo, `--disable agent` disables the chat agent and keeps the calibration skill available through the local MCP server. This demo requires no API key or per-image confirmation.

```sh skip
dimos run a1z-hand-eye-calibration-agent real-sense-camera --disable agent \
  --a1zconnection.can-port a1zcan
```

On an Apple Silicon Mac that needs administrator access for the camera and Homebrew libraries in `/opt/homebrew/lib`, use this command from the checkout. Substitute your D405 device serial.

```sh skip
sudo env DYLD_FALLBACK_LIBRARY_PATH="/opt/homebrew/lib${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}" \
  "$PWD/.venv/bin/dimos" run a1z-hand-eye-calibration-agent real-sense-camera --disable agent \
  --a1zconnection.can-port a1zcan --realsensecamera.serial-number 260522271635
```

**Place and secure the target before starting calibration.** It should be centered at home and visible from both tilt rings. From a second terminal, pass the home and mount for your setup. The values below are from the tested A1Z/D405 setup.

```sh skip
dimos mcp call start_hand_eye_calibration \
  --json-args '{"home":[0,1.0,-1.0,1.1,0,0],"ee_frame":"arm_link6"}'
dimos mcp call hand_eye_calibration_status
```

Each start creates a session under `./hand-eye-sessions/`. Home counts as the first of **8 training views**. At each stationary pose, the first captured image must contain the expected tag and pass the quality checks below. A rejection stops the run at that pose and reports the reason. After solving, the skill takes **4 held-out views**, saves the grade and requests the return move. The status includes the errors and absolute path to `hand_eye.json`.

While calibration holds the arm, MCP rejects another start. Use `dimos mcp call hand_eye_calibration_status` to check the active run. To stop it, call `dimos mcp call stop_hand_eye_calibration`. A move already executing can finish. If the planner is still computing a move when the stop arrives, calibration skips execution. Wait for the final status before starting again. After a failure, correct the setup and start a fresh run.

**Optional chat agent.** Omit `--disable agent` from the same launch command and provide `OPENAI_API_KEY` for the default model. After placing the target, use `dimos agent-send "Start hand-eye calibration with home [0, 1.0, -1.0, 1.1, 0, 0] radians and mount frame arm_link6."` The agent asks for any missing value before calling the skill. [Agent configuration](/docs/capabilities/agents/index.md)

### CLI with operator review

Start the planner and camera in the first terminal.

```sh skip
dimos run a1z-planner-coordinator real-sense-camera --a1zconnection.can-port a1zcan
```

On a Mac that requires privileged camera access, use the same `sudo env` and virtual environment executable as in the MCP example with this planner-and-camera command.

In a second terminal using the same DimOS environment, start with the arm stationary. `--ee-frame` names the camera’s mounting link.

```sh skip
dimos handeyecalibrate run --session ./hand-eye-01 \
  --home 0 1.0 -1.0 1.1 0 0 --ee-frame arm_link6
```

1. At home, center the tag on the crosshair, secure it, and press **R** to refresh if you adjusted it.
2. Press **Enter** in the image window to accept each image and its measured mount pose, or **R** to retake it. The default is **8 training views** (home and an 11° tilt ring), then **4 validation views** on an offset 7° ring. To collect more, add `--count 15 --validation-count 5`.
3. After grading, both the CLI and MCP workflows check the arm’s return pose when the run has moved the arm. The A1Z returns to `0 0.05 -0.05 0 0 0` radians (J1–J6), using its driver’s parking tolerance of 0.05 rad (about 2.86°) per joint. Other arms default to zero on every joint with a 2° tolerance. If the return fails, the error gives the saved result path.

## Operator-positioned capture

Move the arm with your usual controls. `capture` reads the streams at each pose you choose. Pass the camera mounting link with `--ee-frame` on both capture commands. The base frame defaults to `base_link` and can be set with `--base-frame`. Use a separate session and one writer at a time.

```sh skip
dimos handeyecalibrate capture --session ./hand-eye-manual --ee-frame arm_link6 --count 8
dimos handeyecalibrate solve --session ./hand-eye-manual
dimos handeyecalibrate capture --session ./hand-eye-manual --ee-frame arm_link6 --validation --count 4
dimos handeyecalibrate validate --session ./hand-eye-manual
```

Hold still and press **Enter** in the terminal at each pose. Use `q` to stop. Tilt about at least two axes during training and choose fresh poses for validation. `--count` requests additional accepted images in each invocation. For custom intrinsics, `capture` also accepts `--intrinsics PATH` with a matching `CameraInfo` YAML. [Camera calibration guide](/docs/usage/camera_calibration.md)

## What passes

`solve` fits `camera_to_ee` using training views. Each view then predicts the fixed tag pose in the base frame. DimOS averages those positions and rotations into one reference. `validate` holds both the fitted transform and that reference fixed and checks each new prediction against it.

| Check | Requirement |
| --- | --- |
| Timing and stillness | A new image after the capture request, age ≤1 s. Each TF edge within 150 ms of exposure. Measured mount within 1 mm and 0.5° of the first pose over ≥0.5 s. |
| Automatic motion | Home joints within 2° of the request. TCP within 10 mm and 2° of each ring goal. Measured joint feedback within 150 ms of exposure and 2° of the reached posture. |
| Detection | Exactly one matching tag ID. Reprojection RMS ≤2 px per accepted view. |
| Training geometry | ≥8 views, with rotations ≥15° about two axes separated by ≥30°. |
| Target consistency | Every training view and every held-out view within 10 mm and 2° of the training reference. ≥4 held-out views. |

Use `hand_eye.json` when `validated` is `true`. A failed grade gives a nonzero exit status in the CLI. These checks measure repeatability against a fixed target. Absolute accuracy requires an independent measurement.

## Stop or retry

- In `run`, **Q**, **Esc** or closing the review window stops at the current pose. **Ctrl-C** in the calibration terminal interrupts a move. Repeat the same command with the same setup to resume at the first unsaved view. A fully captured run regrades without motion.
- In the operator-reviewed CLI, a rejected tag on a ring triggers one try at half the tilt. For a missed pose, check the reported error and clearance. For a failed solve or validation, check tag size, flatness, sharpness, mounting rigidity, rotation diversity and timestamps. Start a fresh session after changing the target placement, camera or mount, intrinsics, home or view schedule.
- `solve` and `validate` work offline. After adding training images repeat both commands. After adding just validation images repeat `validate`.

## Saved result and hardware acceptance

`session.json` pairs the numbered PNGs with measured poses and intrinsics. In `hand_eye.json`, `parent_frame` is the mounting link and `child_frame` the camera optical frame. For a homogeneous optical-frame point `[x, y, z, 1]`, `camera_to_ee @ point` gives mounting-link coordinates in metres.

Both workflows below passed on the physical A1Z/D405 with 8 training and 4 held-out views. The table gives the maximum held-out errors.

| Workflow | Position | Rotation | Reprojection |
| --- | ---: | ---: | ---: |
| Operator-reviewed CLI | 0.877 mm | 0.410° | 0.509 px |
| Unattended MCP call | 2.306 mm | 0.519° | 0.346 px |

The MCP demonstration used a direct tool call. A chat-initiated hardware run and a blueprint that loads the saved mount are still pending.
