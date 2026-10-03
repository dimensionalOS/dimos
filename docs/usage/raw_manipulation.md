# Raw manipulation

Run the default-scene cylinder lift through plain robot topics:

```bash skip
dimos evals run dimos.evals.suites.mujoco_xarm_raw \
  --agent dimos.evals.agents.pi --set no_dimos=true
```

The goal is to lift the cylinder at least 5 cm above its initial position, graded
with the existing recorded-body-pose scorer. No robosuite assets are required.

## Architecture

```text
Agent <-> isolated Zenoh <-> RawManipulationBridge
                                  | ee_twist_command / gripper_command streams
                           Existing ControlCoordinator tasks <-> robot adapter

Camera streams ----------> RawManipulationBridge --> JPEG / float32 depth
```

The bridge is wired like keyboard and hosted teleop: it publishes
`ee_twist_command` (`TwistStamped`) and `gripper_command` (normalized `Float32`),
which autoconnect to the coordinator's existing `eef_twist` and gripper tasks.
Those tasks own the robot model, IK and execution; the bridge loads no model or
solver, and needs no changes in `dimos/control`. Robot state comes from the
coordinator's `coordinator_joint_state` stream.

The robot-only `xarm-sim` blueprint configures an `ArmTwistCoordinator` with the
`eef_twist` and gripper tasks, plus cameras. A suite sets
`raw_bridge=True, raw_interface="manipulation"`; the eval launcher appends the
generic bridge and assigns an isolated endpoint. `mcp-server` supplies lifecycle
readiness, but Pi's `no_dimos` mode receives `ROBOT.md` rather than MCP access.
The existing navigation interface remains the default for `raw_bridge=True`.

## Commands and frames

Subscribe before commanding. Publish JSON to `robot/arm/command/json`:

```json
{"kind":"twist","linear":[0,0,0.05],"angular":[0,0,0],"t":1.0}
{"kind":"gripper","opening":1.0}
```

- `twist`: end-effector velocity, linear in m/s and angular in rad/s about fixed
  world axes. The bridge holds it for `t` seconds (capped), republishing to the
  task, then sends one zero twist: a deadman like the navigation bridge's
  `cmd_vel`. A new twist replaces the previous one; a zero twist stops the arm.
  Components are clamped to the limits in `robot/arm/info/json`.
- `gripper`: normalized `opening`, 0 closed and 1 open, independent of the arm.

There are no joint targets, stop command, IDs, acknowledgements or status replies.
The agent observes the continuous state/camera streams to decide what to send next.
Invalid inputs are logged and dropped. The default scene's robot base is at world
z=0.12 m and unrotated, so world and base axes coincide.

## Observations

| Topics (under `robot/`) | Contents |
| --- | --- |
| `camera/jpeg` | Wrist RGB, normally 15 Hz |
| `camera/depth_f32`, `camera/depth_info/json` | Aligned metric optical-Z depth and dimensions |
| `camera_info/json`, `camera_pose/json` | Wrist intrinsics and optical pose (frame named in the message) |
| `overview/jpeg` | Fixed-camera RGB, normally 5 Hz |
| `overview/camera_info/json`, `overview/camera_pose/json` | Overview's own intrinsics and optical pose |
| `arm/info/json`, `arm/state/json` | Limits/units; measured joints and normalized gripper opening |

Depth is little-endian float32, row-major `(height, width)`, decoded with
`np.frombuffer(payload, dtype="<f4").reshape(height, width)`. It is optical-axis Z
in metres, not distance along the pixel ray. Mask nonfinite/nonpositive depths.
RGB/depth binary messages have a `{"t": unix_seconds}` attachment; match pose and
image timestamps within each camera. The overview has no depth and uses different
calibration. Internal camera poses use TF; full TF and object truth are not
forwarded. Wrist depth is live only; the eval recorder does not JPEG-encode it.

Metadata repeats at 1 Hz; robot state is published at 20 Hz by default. Keep the
latest samples and save image/depth bytes rather than printing arrays. Camera poses
carry their TF parent frame; the simulated blueprint publishes them in world.

## Robot context

Suites can select a robot-only bundle with `robot_context=local_robot_context(...)`,
which resolves `$DIMOS_ROBOT_CONTEXT_DIR/<name>`; with the variable unset the case
runs without robot context. The bundle is kept outside the repository for now. The
xArm7 bundle (`xarm7`) contains arm/gripper URDFs, portable meshes, licenses,
joint/frame information, TCP offsets and gripper geometry estimates. Pi and
dimcode stage only checksum-manifest-listed files into the case's `robot/`
directory. Scene XML, object poses and grader state are not included.

The gripper's analytic jaw-gap estimate is nonlinear (approximately 1.6-88.9 mm),
not a hardware specification or dynamic contact calibration. Commands use
normalized opening. Read the short context once and consult URDFs
selectively; use live observations for current state.
