# Eye-in-hand calibration

Eye-in-hand calibration measures the transform from a wrist camera’s optical frame into its mounting link. Call the calibration skill for unattended capture, use `dimos handeyecalibrate run` for per-image review, or position the arm yourself. All three use the same solver and pass criteria.

## Before capture

- Attach the camera rigidly. The default target is a flat **tag36h11, ID 0**, measuring **150 mm across the outer black square**. The CLI accepts `--marker-id` and `--marker-size-m` for another tag in that family. Keep the tag fixed relative to the base through validation.
- Publish an 8-bit color or grayscale `Image`, matching `CameraInfo` and measured base-from-mount TF. Timestamp images and arm feedback on the same clock. Keep resolution, crop, focus and zoom fixed. Supported distortion models are `plumb_bob` and `rational_polynomial`.
- Both planner-driven modes need one pose-capable arm group, measured `joint_state`, and TF between the planner tool and the rigid camera mounting link. Check arm, camera and cable clearance through both sweeps and the return move. CLI image review also needs a display.

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

**Place and secure the target before starting calibration.** Center it at home and keep it visible from both sweeps. From a second terminal, pass the home and mount for your setup. The values below are from the tested A1Z/D405 setup.

```sh skip
dimos mcp call start_hand_eye_calibration \
  --json-args '{"home":[0,1.0,-1.0,1.1,0,0],"ee_frame":"arm_link6"}'
dimos mcp call hand_eye_calibration_status
```

Each start creates a session under `./hand-eye-sessions/`. The arm uses home as its motion reference and takes the first of **8 training views** at an angle. It must read the tag there before proceeding. At any later training or validation stop, if tag detection or pose estimation fails its checks, the skill moves halfway toward home and takes one fresh stationary image. On a sweep this halves the lens offset from home and re-aims at the tag. On a tilt it halves the requested angle. A successful retry fills the same numbered view using its measured pose. A second rejection stops the run at the retry pose and reports the reason.

After solving, the skill takes **4 held-out views**, saves the grade and requests the return move. The status includes the errors and absolute path to `hand_eye.json`.

While calibration holds the arm, MCP rejects another start. Use `dimos mcp call hand_eye_calibration_status` to check the active run. To stop it, call `dimos mcp call stop_hand_eye_calibration`. A move already executing can finish. If the planner is still computing a move when the stop arrives, calibration skips execution. Wait for the final status before starting again. After a failure, correct the setup and start a fresh run.

**Optional chat agent.** Omit `--disable agent` from the same launch command and provide `OPENAI_API_KEY` for the default model. After placing the target, use `dimos agent-send "Start hand-eye calibration with home [0, 1.0, -1.0, 1.1, 0, 0] radians and mount frame arm_link6."` The agent asks for any missing value before calling the skill. [Agent configuration](/docs/capabilities/agents/index.md)

### CLI with operator review

Start the planner and camera in the first terminal. On the example Mac with the D405 connected, run this from the DimOS checkout.

```sh skip
sudo env DYLD_FALLBACK_LIBRARY_PATH="/opt/homebrew/lib${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}" \
  "$PWD/.venv/bin/dimos" run a1z-planner-coordinator real-sense-camera \
  --a1zconnection.can-port a1zcan --realsensecamera.serial-number 260522271635
```

In a second terminal using the same DimOS environment, start with the arm stationary. `--ee-frame` names the camera’s mounting link.

```sh skip
dimos handeyecalibrate run --session ./hand-eye-01 \
  --home 0 1.0 -1.0 1.1 0 0 --ee-frame arm_link6
```

1. Before starting, secure the tag so it is centered when the arm is at home. The first review appears after the arm moves to an angled pose.
2. Press **Enter** in the image window to accept each image and its measured mount pose, or **R** to retake it. The default is **8 training views** and **4 held-out views**. For seven stops across the wide sweep, add `--count 12`.
3. After grading, both the CLI and MCP workflows check the arm’s return pose when the run has moved the arm. The A1Z returns to `0 0.05 -0.05 0 0 0` radians (J1–J6), using its driver’s parking tolerance of 0.05 rad (about 2.86°) per joint. Other arms default to zero on every joint with a 2° tolerance. If the return fails, the error gives the saved result path.

### Wide views around home

The CLI and MCP both plan these views from the current session. From home, the tool moves 3 cm to the right (world −Y) and yaws 8° toward the target for the first sighting. Four 11° tilts about two tool axes estimate where the camera sits on the wrist. The planner can then place the lens relative to its home position and aim it at the measured tag center at each stop. Set home with the arm centered along the planner's X axis, with +X forward, Y left/right and +Z up.

| Captures | Camera positions relative to home |
| --- | --- |
| Training | The lens starts 15.5 cm to one side, crosses the center, and finishes 15.5 cm to the other side. Both ends are 11 cm below and 1.2 cm forward of home. At the raised stop the lens is 2.2 cm above and 6.8 cm behind home, offset to one side. |
| Validation | The lens sweeps back across a 25 cm span. |

The first angled sighting and four tilts count toward `--count`. On both sweeps a stop that would fall on the home X centerline shifts sideways by half the spacing to the preceding stop, up to 4 cm. The default four validation stops are already off-axis. With seven sweep stops, the first stop after center uses half its usual offset toward the left, back and down to improve tag visibility. The lateral limits are fixed offsets chosen just inside the modeled reach of the example A1Z at this home. Check clearance for your setup, especially below home at the two ends. The running planner checks each move.

The final solve uses every training view, and validation measures agreement against that frozen result. If the initial tilts give insufficient rotation or inconsistent target poses, the workflow stops before the wide sweep. Check that the tag stays fixed and clearly visible through the tilts before retrying.

If you explicitly supply a validated result from the same physical mount with `--orbit-calibration PATH` (MCP `orbit_calibration`), the workflow skips the four tilts and uses that mount for aiming. It locates the tag from the angled first sighting and fits a new calibration.

### Optional smaller rings

For the A1Z/D405 mount described above, add `--orbit-axis x` to the CLI run or `"orbit_axis":"x"` to the MCP start arguments. Use a fresh session for the first orbit run. Center the tag at home and leave clearance for the wrist and cable on both sides of that pose.

In this mode, the first image is taken at an 8° in-place tilt and gives the distance to the tag. DimOS places an estimated pivot that far along the selected tool axis, then moves the TCP sideways and turns the camera inward at each capture pose. At the example home distance of 29 cm, the training circle is **11 cm across** and the validation circle is **8 cm across**. At shorter distances the circles shrink to keep the tilts within 11° and 8°. The planner connects discrete capture poses. Tag visibility is checked at each stop.

On another mount, choose the signed TCP axis (`x`, `-x`, `y`, `-y`, `z`, `-z`) aligned with the camera’s viewing direction. The estimate assumes the camera is near the TCP. Centering accuracy depends on that alignment and offset.

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

- In `run`, **Q**, **Esc** or closing the review window stops at the current pose. **Ctrl-C** in the calibration terminal interrupts a move. Repeat the same command with the same setup to resume at the first unsaved view. A fully captured run regrades without motion. Older sessions resume their saved viewpoints. Choose a fresh session directory for the off-axis schedule.
- If the CLI rejects a frame, **R** retakes the same numbered view in place. Where offered, **H** moves halfway toward home for one retry at that position. **Enter** accepts a frame that passed detection.
- For a missed pose, check the reported error and clearance. For a failed solve or validation, check tag size, flatness, sharpness, mounting rigidity, rotation diversity and timestamps. Start a fresh session after changing the target placement, camera or mount, intrinsics, home or view schedule.
- `solve` and `validate` work offline. After adding training images repeat both commands. After adding just validation images repeat `validate`.

## Saved result and hardware acceptance

Open `rejected/` inside the session to inspect failed tag observations and operator-requested retakes. For example, `train-0009-01.png` contains the original image from the first rejection at training view 9. The matching JSON gives the reason, image timestamp, optical frame and measured arm pose. Further rejections at that view use `-02`, `-03` and so on, including after a resume. Only accepted images count toward the solve and validation.

`session.json` pairs the numbered PNGs with measured poses and intrinsics. In `hand_eye.json`, `parent_frame` is the mounting link and `child_frame` the camera optical frame. For a homogeneous optical-frame point `[x, y, z, 1]`, `camera_to_ee @ point` gives mounting-link coordinates in metres.

Both workflows below passed on the physical A1Z/D405 using the original tilt-in-place rings with 8 training and 4 held-out views. The table gives the maximum held-out errors.

| Workflow | Position | Rotation | Reprojection |
| --- | ---: | ---: | ---: |
| Operator-reviewed CLI | 0.877 mm | 0.410° | 0.509 px |
| Unattended MCP call | 2.306 mm | 0.519° | 0.346 px |

The MCP demonstration used a direct tool call. A chat-initiated hardware run and a blueprint that loads the saved mount are still pending.
