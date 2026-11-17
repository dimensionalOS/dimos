# Booster K1

22-motor K1 locomotion using Booster's released `k1_walk.pt`. The head is held at
home; the actor controls the other 20 joints. Inference runs at 50 Hz on CPU.

This integration builds on the sim2 PR stack through `pim/feat/sim2-e-g1`
(`4c2cef3a0680903716be8124ddb110c9d22fb788`).

## Simulation

Use Python 3.12 and the native MuJoCo USD importer (macOS ARM64 or Linux x86_64):

```bash
uv sync --python 3.12 --extra booster --extra sim
uv run dimos run booster-k1-sim2
```

The blueprint composes `UsdManager`, `SimModule`, `SimImuModule`,
`SimJointEncoderModule`, `K1PolicyModule`, `K1SimConnection`, two camera devices,
and a shared `CameraRenderModule`.
The policy consumes standard `joint_state`, `imu`, and `cmd_vel` streams;
the sim connection maps its motor targets to sim2's timed `ActionCommand`.
The Controller app can drive the usual `cmd_vel` topic and display the USD scene.

The RGB-D camera mounts at Booster's `head_realsense_rgb_link`; the stereo RGB
view mounts at `head_booster_stereo_rgb_link`. Both frames come from the vendor
model and move with the head. Primary streams are `color_image`, `depth_image`,
and `camera_info`; the second view uses `stereo_color_image` and
`stereo_camera_info`. Both render at 320×240 and 15 Hz with sim2's default pinhole
intrinsics. These are simulated intrinsics, not factory camera calibration; the
stereo view is one RGB image, not a synthesized left/right pair.

From another terminal, run `uv run dimos shell`:

```python
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

app.K1PolicyModule.move(Twist(linear=Vector3(0.3, 0, 0)), duration=2)
app.K1PolicyModule.move(Twist(angular=Vector3(0, 0, 0.4)), duration=2)
app.K1PolicyModule.move(Twist())
app.K1PolicyModule.status()
```

Streaming commands expire after 0.3 seconds; the robot then balances with a zero
velocity target. Limits are forward 0.5 m/s, backward 0.3 m/s, lateral 0.3 m/s,
and yaw 0.6 rad/s. Restart the blueprint for a fresh simulation episode.

## Hardware

The official `booster-robotics-sdk-python==1.6.3` wheels support Linux. On the
robot's computer, or a Linux host on its DDS network:

```bash
uv sync --python 3.12 --extra booster
uv run dimos run booster-k1 --k1connection.network-interface <interface>
```

`K1Connection` uses the SDK's `B1LowStateSubscriber`, `B1LowCmdPublisher`, and
`B1LocoClient`; these APIs also serve K1. It selects **serial** joints and the
K1 count of 22, including both ankle axes. It republishes policy targets at
nominally 500 Hz. It starts with telemetry only.

Use `app.K1Connection.prepare()` to request the vendor prepare posture. Once the
robot is standing near the policy's home posture and telemetry is current, call
`app.K1Connection.activate()` to enter custom control. The same policy `move()`
RPC and `cmd_vel` stream then drive it. `app.K1Connection.deactivate()` releases
custom control into damping. Stale feedback or policy output also enters damping;
velocity-command expiry alone keeps the balancing policy running.

The SDK mapping and lifecycle have automated tests with a mocked SDK boundary.
Physical K1 control and firmware compatibility still require a hardware run.

## Assets and verification

The LFS archive `data/.lfs/booster_k1.tar.gz` contains the USD scene, checkpoint,
upstream licenses, and a provenance manifest with revisions and SHA-256 hashes.
It is unpacked lazily through `LfsPath`.

Sources:

- [booster_assets](https://github.com/BoosterRobotics/booster_assets/tree/12a97516f57469078707afb587a5f84c1cb110d1), BSD-3-Clause; `robots/K1/K1_22dof.xml` and meshes.
- [booster_deploy](https://github.com/BoosterRobotics/booster_deploy/tree/7bb1462e2ac742eba5c01659b815e2e99fbe617f), Apache-2.0; checkpoint, observation layout, motor gains, action filter, and right-elbow correction.

To regenerate with those repositories checked out at those revisions:

```bash
uv run python -m dimos.robot.booster.k1.demo_prepare_assets /path/to/booster_assets /path/to/booster_deploy data/booster_k1
```

The conversion preserves the source collision selection, resolves the exported
ground-material binding, removes native sensor definitions in favor of sim2's
mounted sensors, and authors the policy's initial posture and motor drives.

```bash
uv run pytest dimos/robot/booster/k1 --import-mode=importlib -m ''
```

Tests cover upstream policy output parity, history reset, joint ordering,
timestamp pairing, command expiry, SDK mode failures, and an actual nine-worker
sim2 deployment that balances, walks, turns, stops, and renders both head cameras.
