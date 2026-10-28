# Dual MakerMods Metal with ManipulationModule

This stack adds two independent Metal arm hardware components to one
ControlCoordinator and one bimanual ManipulationModule. Viser offers selectable
`left_arm` and `right_arm` planning groups. Both arms share a collision world and
a single 12-joint trajectory task, allowing synchronized plans and cross-arm
collision checking. Each gripper has its own task and stream. This stack adds no
teleoperation module or separate motor driver.

## Run with mock hardware

```bash
uv sync --extra planning --inexact
uv run dimos run dual-makermods-metal-planner-coordinator
```

Open [Viser](http://127.0.0.1:8095). Select either planning group to manipulate
its target and independent gripper. ManipulationModule's `plan_to_joints` and
`plan_to_poses` accept both group IDs for a synchronized bimanual plan. Planning
speed scale defaults to 0.2. Each arm contributes six angular joints and one
normalized gripper; finger geometry remains fixed in the planning model.

## Geometry and dimensions

The model lazily composes two copies of the same pinned official Metal URDF used
by the [single-arm integration](/docs/capabilities/manipulation/makermods_metal_integration.md).
Links, joints, material names, and references are prefixed `left_` / `right_`.
The root is `dual_metal_base`, with tips `left_gripper_base` and
`right_gripper_base`. Meshes and kinematics come from the official model.

MakerMods publishes a **90 x 90 mm, four M5 screw** base mounting pattern and
612.5 mm working radius on its [Metal product page](https://www.makermods.ai/metal-arm).
These describe one arm, not the spacing between two arm bases. The official
[two-arm control configuration](https://github.com/makermods-robotics/metal-python-ros/blob/bccf3c82a21453a7b1784a94966456e16ff0f008/metal_ros2/src/metal_controller/config/two_arm_control.yaml)
defines separate CAN interfaces and topics, without a shared mounting transform.
No authoritative dual-base transform was found in the reviewed official models
and configuration. The official Lab renders its arms separately rather than
providing a calibrated shared collision model.

The default layout is an **example only**: same orientation, left base at
`[0, 0.3, 0]` m, right at `[0, -0.3, 0]` m (600 mm center distance), both
with `[0, 0, 0]` roll/pitch/yaw. It is not an official station dimension.
Override the following fields to match the actual installation before using
physical bimanual planning:

- `--left-base-xyz`, `--right-base-xyz`: JSON triples, meters in `dual_metal_base`.
- `--left-base-rpy`, `--right-base-rpy`: JSON triples, radians, URDF roll/pitch/yaw.

For example, a measured 500 mm same-orientation spacing can be represented as:

```bash
uv run dimos run dual-makermods-metal-planner-coordinator \
  --left-base-xyz '[0, 0.25, 0]' \
  --right-base-xyz '[0, -0.25, 0]'
```

The combined model is cached by its content and mount transforms. Importing the
blueprint does not download or generate assets.

## Physical CAN setup

Both arms reuse `MakerModsMetalAdapter` with the shared custom firmware profiles
from `can-motor-control>=0.0.10`. Each arm uses its own CAN bus because motor
IDs 1-7 are repeated. Run `dimos hardware can list`, identify and label both
adapter serials, and supply **both distinct** selectors:

```bash
uv run dimos run dual-makermods-metal-planner-coordinator \
  --left-can-port LEFT_USB_SERIAL \
  --right-can-port RIGHT_USB_SERIAL \
  --left-gripper-opening-direction increasing_position \
  --right-gripper-opening-direction increasing_position \
  --left-base-xyz '[0, 0.25, 0]' \
  --right-base-xyz '[0, -0.25, 0]'
```

The direction and geometry in this example must match the commissioned rig.
Linux selectors are configured SocketCAN interfaces such as `can0` and `can1`;
macOS selectors are bare unique gs_usb USB serials. Partial, empty, and duplicate
selectors are rejected rather than selecting one real and one mock arm.
Current ratio defaults to 0.1 on each gripper. Selecting physical hardware
activates both arms and calibrates both grippers; stopping removes torque.
Gravity compensation remains disabled as in the single-arm stack.

Without physical selectors both components are in-memory mocks. Physical dual-arm
commissioning was not performed as part of this integration.
