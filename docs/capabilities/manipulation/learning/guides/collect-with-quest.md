# Collect demonstrations with Quest

Use this guide when your OpenYAM Quest teleoperation setup already works. For
hand-guided collection without Quest, use the
[OpenYAM tutorial](/docs/capabilities/manipulation/learning/tutorials/openyam-hand-teaching.md).

## Record with one arm

Choose new recording and dataset paths. Configure your CAN interface and wrist
camera, then launch the collection stack:

```bash skip
dimos run openyam-quest-collection --daemon \
  --can-port follower_l \
  --recorder.recording recordings/quest-001 \
  --episodes.task "pick up the cube" \
  --wrist.hardware.camera-index /dev/video0
dimos imitation control
```

Use your configured Quest teleoperation connection to control the arm. The panel
controls episode boundaries, not teleoperation:

| Control | Action |
| --- | --- |
| Start episode | Start a take |
| Save episode | Save the active take |
| Discard | Discard the active take |
| Q | Detach; during recording, press Q again to confirm |

The profile records measured joint state and the main coordinator's
`coordinator_joint_command` stream. These actions represent attempted commands;
they do not prove acceptance by the controller or physical arrival at a target.

## Record with two arms

For two arms with Quest, launch the dual collection blueprint and attach the
same episode controls:

```bash skip
dimos run dual-openyam-quest-collection --daemon \
  --recorder.recording recordings/fold-001 \
  --episodes.task "fold the towel" \
  --left-can-port follower_l \
  --right-can-port follower_r \
  --left-wrist.hardware.camera-index /dev/video0 \
  --right-wrist.hardware.camera-index /dev/video2
dimos imitation control
```

The dual profile uses 640x480 RGB images at 30 Hz with a 20 ms alignment
tolerance anchored on the left wrist. Joint order is left arm joints 1-6,
right arm joints 1-6, left gripper, right gripper. Images and measured state
are snapshots; attempted joint commands declare `source_kind="joint_position_updates"`.
Use the [data preparation guide](/docs/capabilities/manipulation/learning/guides/prepare-and-inspect-data.md)
with the dual recording and dataset paths. Dual-arm collection does not imply support for dual-arm rollout.

The collection Blueprint constructs dual teleop with
`ViserVisualizationConfig(host="0.0.0.0")`; ordinary dual teleop retains its
local-only default. Open `http://ROBOT_HOST:8095` on a trusted network, or restrict
access with `--manipulationmodule.visualization.host 127.0.0.1`.

## Collect with the simulated xArm7

`learning-collect-webxr-xarm7` runs the arm and camera in MuJoCo. On Linux
laptops with integrated graphics and an NVIDIA GPU, rendering on the integrated
GPU can miss the requested camera rate. With GLX, try PRIME offload:

```bash skip
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  uv run dimos --simulation run learning-collect-webxr-xarm7 \
  --recording recordings/collect --mujocosimmodule.fps 30
```

Use a new recording directory. Allow at least five seconds after the first
camera frame and check the logs before recording. A "MuJoCo camera rate is below
requested fps" warning reports the measured and requested fps when the rate
stays below 95% of the request. Resolve the rendering slowdown before collecting
demonstrations, since `prepare` can reject low-rate recordings. The PRIME
settings apply to Linux/NVIDIA GLX setups and do not guarantee the target rate.

Open `https://<host-ip>:8443/teleop` in the headset, accept the certificate, and
tap Connect. Hold the right grip to move the arm. Press B to start an episode
and B again to save it, then stop the stack with Ctrl+C.

## Finish and prepare the recording

Save or discard the final take. Select **Stop stack…**, support the arms, and
confirm shutdown. Press **Q** to detach instead if you want the stack to keep
running. Detaching or losing the connection does not stop the robot or recorder.
Reattach with `dimos imitation control` if needed.

Continue with [Prepare and inspect a recording](/docs/capabilities/manipulation/learning/guides/prepare-and-inspect-data.md).
