# OpenYAM integration

OpenYAM uses the generic Damiao whole-body adapter backed by
`can-motor-control`. The adapter owns one upstream robot containing the six-joint
arm and its gripper; motor topology is independent of the host operating system.

## CAN transport

The generic Damiao layer selects the native transport provided by
`can-motor-control>=0.0.8`:

| Host | Default | `--can-port` override |
|------|---------|-----------------------|
| Linux | `SocketCanBus("can0")` | SocketCAN interface, such as `can1` |
| macOS | first `1d50:606f` gs_usb adapter | USB serial number |

The macOS transport is implemented in Rust with native IOKit access. It does
not require PyUSB, libusb, `python-can`, or the Python `gs_usb` package.

Logical buses are ordered. Without explicit selectors, a future two-bus Damiao
robot maps to `can0`/`can1` on Linux and gs_usb indices 0/1 on macOS. Production
multi-arm deployments should configure USB serial numbers because enumeration
order is not a stable device identity.

List the selectors accepted on the current host:

```bash
dimos hardware can list
```

On Linux this prints SocketCAN network-interface names. Configure each physical
interface before use:

```bash
dimos hardware can setup can0
dimos hardware can setup can1
```

On macOS, connect a `gs_usb`/candleLight-compatible adapter with USB ID
`1d50:606f` for each arm. No SocketCAN interface or extra USB library is needed.
The motor buses still require CAN-H/CAN-L wiring, motor power, and correct bus
termination. To assign sides reliably, connect one adapter at a time, run the
list command, record and label its USB serial, then repeat for the other arm.
Adapters used together must expose unique serial numbers.

## Run

Use the default CAN device:

```bash
dimos run coordinator-openyam
```

Select another Linux SocketCAN interface:

```bash
dimos --can-port can1 run coordinator-openyam
```

Select a macOS adapter by USB serial number:

```bash
dimos --can-port <USB-SERIAL> run coordinator-openyam
```

The dual-arm WebXR blueprint is identical on both operating systems; only the
selector values differ:

```bash
# Linux
dimos run teleop-webxr-dual-openyam --left-can-port can0 --right-can-port can1

# macOS
dimos run teleop-webxr-dual-openyam \
  --left-can-port <LEFT-USB-SERIAL> \
  --right-can-port <RIGHT-USB-SERIAL>
```

Linux interfaces must already be configured for classical CAN at 1 Mbit/s.
The native macOS transport configures the adapter bitrate directly.

## Hardware topology

| Motor | Send ID | Reply ID | Type |
|-------|---------|----------|------|
| joint1–3 | `0x01`–`0x03` | send ID + `0x10` | DM4340 |
| joint4–6 | `0x04`–`0x06` | send ID + `0x10` | DM4310 |
| gripper | `0x08` | `0x18` | DM4310 |

The bus uses classical CAN at 1 Mbit/s. Opening the gripper decreases motor
position. The arm and gripper are exposed together as one whole-body hardware
component.

## Grasping on the dual rig

`dual-openyam-grasp` is the xArm grasp stack on the dual OpenYAM: coordinator,
planner, pick-and-place, scene registration and a grasp provider, with a fixed
RealSense over the table feeding perception and one RealSense on each wrist.
Each arm is a planning group with its own gripper, so pick-and-place calls take
`left_manipulator` or `right_manipulator`. The three D405 serials of the
benchmark rig are the defaults; override them with
`--realsensecamera.serial-number`, `--left-wrist-camera.serial-number` and
`--right-wrist-camera.serial-number`.

```bash
# robot, heuristic top-down grasps
dimos run dual-openyam-grasp --left-can-port follower_l --right-can-port follower_r

# robot, GraspGenX grasps (up to 100 ranked learned grasps per object)
dimos run dual-openyam-grasp --left-can-port follower_l --right-can-port follower_r --graspgen

# in-memory arms, no CAN or camera needed (removes all three cameras)
dimos run dual-openyam-grasp --disable real-sense-camera --disable object-scene-registration-module
```

### What the planner avoids

Neither the dual OpenYAM URDF nor the upstream i2rt model carries collision
geometry, so the grasp blueprint builds its own planning model
(`dual_openyam_grasp_model_config`):

- every link collides as the convex hull of its visual mesh;
- the wrist camera and its bracket are boxes on each gripper link
  (`DUAL_OPENYAM_WRIST_CAMERA_BOXES`, 1.5 cm margin plus room for the plug);
- the two fingertips and the nested wrist pair are excluded, everything else
  adjacent is filtered;
- the arm bases stand at the measured `DUAL_OPENYAM_BASE_SPACING`;
- the table top and the bin are static obstacles (`DUAL_OPENYAM_STATIC_BOXES`,
  table top 4.5 cm below the base plates, bin at the far edge of the 130 x 80 cm
  table, centred).

Detected objects and, with a voxel map, unknown clutter are added on top by the
world monitor. Viser's "Robot display" switch shows the collision bodies. The
acceptance test lives in `test_grasp_collision.py` (self-hosted, needs Drake):
a plan with the fingertips in the bin is refused, free space plans. Re-measure
the bin and the base spacing whenever the rig changes.

The blueprint carries a Rerun bridge with the three cameras as tiles and the
scene beside them. After `scan_objects` the detected objects appear as labelled
boxes; `pick_object` draws the ranked grasp proposals as jaw glyphs, the one
being attempted in yellow, and every plan draws the tip's path before the arm
moves. It opens no window on the robot; watch it from a laptop:

```bash
uvx dimos-viewer --connect rerun+http://<robot-ip>:9877/proxy --ws-url ws://<robot-ip>:3030/ws
```

Every run records the policy-training data to `recordings/<run-id>/memory.db`
without any flag; `--record=` (an empty value) turns it off and `--record-topics` widens or
narrows the set. What is kept, against the data a learned policy needs:

| Training input | Stream | Source |
|---|---|---|
| joint states | `coordinator_joint_state` | coordinator, 12 arm joints and both grippers, 100 Hz |
| joint trajectory | `planned_joint_trajectory` | ManipulationModule, the plan handed over per `execute()` |
| joint commands | `applied_joint_position_command` | coordinator, the positions the hardware accepted, per tick |
| camera images | `color_image`, `depth_image`, `camera_info` | overhead camera |
| | `left_wrist_color_image`, `left_wrist_depth_image`, `left_wrist_camera_info` | left wrist camera |
| | `right_wrist_color_image`, `right_wrist_depth_image`, `right_wrist_camera_info` | right wrist camera |
| frames | `tf`, `left_wrist_tf`, `right_wrist_tf` | planner and cameras |

`--graspgen` is a global flag read when the blueprint is imported, so it also
works as `dimos --graspgen run dual-openyam-grasp ...`. GraspGenX has the same
requirements as on the xArm: Linux x86_64, a CUDA 12.8-compatible GPU and
`uv >= 0.9.25`; the first launch prepares its isolated environment and downloads
the checkpoints, which takes minutes. The gripper is described to GraspGenX as
a sweep volume measured off the URDF finger meshes
(`DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME`): 9.4 cm opening, pads 2 cm tall, 14.7 cm
from the gripper link to the fingertips.

Then from `dimos shell`:

```python skip
app.ManipulationSkills.go_init()
scan = app.PickAndPlaceModule.scan_objects(["mustard bottle", "soup can"])
app.PickAndPlaceModule.pick_object("<object_id>", planning_group="right_manipulator")
app.PickAndPlaceModule.place_at(0.35, -0.25, 0.20, planning_group="right_manipulator")
```

The camera pose in `blueprints/grasp.py` (`DUAL_OPENYAM_CAMERA_TRANSFORM`) is
the mount on the benchmark rig; re-measure it when the camera moves.

### Driving it from humancli

`dual-openyam-grasp-agent` adds an MCP server and an LLM agent over the same
stack, with a system prompt that names both arms as `left_manipulator` and
`right_manipulator`. The agent needs `OPENAI_API_KEY` in the environment or in
the `.env` of the directory `dimos run` starts in. `--graspgen` works here too.

```bash
# terminal 1, robot
dimos run dual-openyam-grasp-agent --left-can-port follower_l --right-can-port follower_r

# terminal 2, same machine
dimos humancli
```

Then talk to it: "scan for a soup can, a mustard bottle and a banana", then
"pick up the soup can with the right hand and put it in the bin". The agent
calls `stage_pick_and_place`, which plans every leg of the job from the
predicted end of the one before (approach, descend, grasp, lift, carry, lower,
release, retreat, return home) without moving. Viser plays the whole motion as
a ghost and Rerun draws the full tool path. The agent reports the legs, the
seconds of motion and the grasp rank, then waits. Say "proceed" and it calls
`proceed`, which runs the legs in order and stops at the first that fails; say
"discard" and nothing moves. The agent passes the arm as `planning_group` on
every motion skill and asks once when the arm is not stated.


The recording lands under `recordings/<run-id>/`. `applied_joint_position_command`
carries only the targets the hardware accepted, at the control rate, so it is
the executed trajectory; `coordinator_joint_state` includes the two grippers.

## Safety

- Keep the workspace clear and the emergency stop reachable during first
  activation.
- Verify motor IDs and types against the physical unit before commanding large
  motions; an incorrect type scales velocity and torque incorrectly.
- Deactivation removes motor torque, so support the arm before stopping it.
