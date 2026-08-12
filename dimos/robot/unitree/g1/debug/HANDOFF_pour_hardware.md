# Handoff: G1 pour sequence on hardware

Goal: the G1 sees an AprilTag on a plant pot, walks the last metre to a
stance it can pour from, and pours. Sim does this end to end at ~1 cm.
Branch `manip/mobile-manipulation-demo-aug18`, demo `unitree-g1-water-demo`.

## The blocker, and the best lead

**The head camera stops publishing a few seconds into a run.** Everything
downstream (AprilTag detections, the approach servo) starves, and the
approach aborts on its staleness guard about 2 s later.

The strongest suspect is `dimos/hardware/sensors/camera/realsense/camera.py:336-339`:

```python
try:
    frames = self._pipeline.wait_for_frames(timeout_ms=1000)
except (RuntimeError, AttributeError):
    # Pipeline stopped or None - exit loop
    break
```

If `wait_for_frames` throws once mid-stream — a USB hiccup, a device reset,
a frame that misses the 1 s timeout — the capture thread exits **silently
and permanently**. No log line, no retry, no republish. The camera is dead
for the rest of the process and nothing in the output says so.

The background retry added in b3cefcbf6/267bacace only covers failing to
*open* the device at startup. It does not cover the stream dying later.

This fits every symptom: detections work at first, stop shortly after, and
come back only on a fresh run (a new process). It also explains why the
robot barely moved in five consecutive attempts.

### First thing to do

Confirm it before fixing it. Add a log line in that `except` (and log the
exception), then run the demo and drive the robot around. If that line
prints, the diagnosis is confirmed and the fix is to restart the pipeline
instead of breaking out of the loop — reuse the existing `_retry_open`
path rather than writing a second recovery mechanism.

Watch for the trigger too: if it only fires while the robot is walking,
suspect the USB connector or power under vibration, not software.

## What is done and pushed

Five commits, all on the branch:

- `docs(g1)` — the Jetson's measured spec in the water demo docstring
  (Orin NX, 8 cores, 15 GiB + 7 GiB swap, kernel 5.10.104-tegra).
- `fix(control)` — gravity feedforward is clamped per joint to the model's
  effort limits (G1 wrists are 5 Nm against the shoulders' 25).
- `feat(g1)` — hardware gravity feedforward defaults to **0**, with
  `--gravity-ff-scale` to step it. Sim keeps 0.7. **Note:** hardware had
  been running 0.7 unmeasured before this; pass `--gravity-ff-scale 0.7`
  to get the old behaviour back.
- `feat(g1)` — the pour sequence runs against hardware (`--target hardware`).
- `feat(g1)` — per-tick approach telemetry and `--sighting-timeout`.

## How to run it

Stack, on the robot:

```
.venv/bin/dimos --rerun-open none --rerun-host 0.0.0.0 run unitree-g1-water-demo
```

Sequence, second shell on the robot:

```
.venv/bin/python -m dimos.robot.unitree.g1.debug.demo_pour_sequence --target hardware status
.venv/bin/python -m dimos.robot.unitree.g1.debug.demo_pour_sequence --target hardware approach
```

`status` moves nothing. `approach` walks and stops. Bare `demo` is the full
run. Ctrl-C stops the base.

## Verified facts (don't re-derive these)

- **No odometry exists on this robot.** The G1's DDS link carries `rt/lowstate`
  (motors + pelvis IMU) and nothing else. There is no Unitree estimator
  odometry for G1 in the SDK — `SportModeState` is Go2-only. `/odom` has a
  transport in the water demo but no publisher, ever.
- **The hardware world frame is pelvis-anchored.** The planning model already
  places the base at `_G1_NOMINAL_PELVIS_Z = 0.74` facing +x, and tag
  detections resolve in the `pelvis` frame, so tag poses need no transform
  and `latch_base_pose` is skipped on hardware. This is why the sequence
  works without odometry.
- **Topics:** twist is `/g1/cmd_vel` on hardware, `/cmd_vel` in sim. Live
  detections are `/detections` (`Detection3DArray`, confirmed unique in the
  blueprint so it resolves to that exact name). `/object_pose` is the
  *frozen* latch — do not steer on it, it never gets closer.
- **Teleop and the servo write the same twist topic.** Last message wins.
  Don't drive with WASD while the servo runs. Putting the servo under the
  coordinator as a separate claimant was deliberately not done; it is not
  needed to make this work.
- **Head camera:** 848x480 @ 15 fps, pitched 47.6° down (`d435_joint` rpy in
  the URDF), 0.474 m above the pelvis frame, ~70° horizontal / 43° vertical
  FOV. Intrinsics are committed; detection silently skips frames whose size
  differs from `CameraInfo`.

## Open questions

- **Pelvis height.** A floor-level tag reads 0.574 m below the pelvis, which
  puts the pelvis at ~0.57 m, not the 0.74 m the planner assumes. If that
  holds, every world-frame z target is ~17 cm off, including `POUR_Z = 0.90`.
  `status` prints both numbers. Worth settling before judging a pour that
  lands low.
- **Pour height vs the real plant.** `POUR_Z` is a fixed 0.90 m above the
  planner's floor and ignores the tag's z entirely. Confirm that clears the
  actual pot.
- **Gravity feedforward on hardware is uncalibrated.** Sim's 0.7 was fit to
  URDF masses running ~30% heavy with no joint friction. Step it from 0
  against measured droop (`droop_harness.ignore.py`) before trusting it.

## Wrong turns — don't repeat these

- I claimed the tag leaves the camera's view as the robot closes in
  (camera pitched down, tag low). **Refuted by the logs:** the stance x
  stayed 0.67–0.68 across five runs, so the robot never travelled, so
  distance was never the variable. The geometry is real but it was not
  the failure.
- I claimed the 34.7° stance bearing against the 35° half-FOV was the
  cause. Secondary at best.
- The approach's abort message reports the staleness *timeout*, not when
  the sighting stopped. "2.1 s ago" is just the constant. That is what the
  new per-tick telemetry fixes — read those lines, not the abort message.
- The sideways walk during approach is **correct**, not a bug: the right
  arm pours to the robot's right, so the stance puts the pot 33 cm ahead
  and 22 cm right, and the robot must turn and sidestep to get there.

## Not verified

Nothing in the hardware path has completed a run. The sim path is
unchanged in behaviour but has not been re-run since these commits.
