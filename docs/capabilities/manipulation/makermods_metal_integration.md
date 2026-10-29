# MakerMods Metal Arm integration

Metal Arm uses the shared `DamiaoWholeBodyAdapter` and `can-motor-control`.
`makermods_metal` declares six angular arm joints and a normalized gripper
on one classical CAN bus at 1 Mbit/s. It is different from the RobStride
Maker Arm. No MakerMods SDK, Python CAN loop, or codec is added to dimOS.

## Firmware profiles

Metal firmware uses custom MIT scaling. Stock DM4340 or DM4310 profiles are
incompatible, including position scaling, even if the motor hardware is Damiao.

| Joints | Send / feedback IDs | Shared profile | PMAX / VMAX / TMAX |
| --- | --- | --- | --- |
| joint1, joint3, joint4 | 1/0x11, 3/0x13, 4/0x14 | custom 0x8000 | 6.28 rad / 10 rad/s / 30 Nm |
| joint2 | 2/0x12 | custom 0x8001 | 6.28 rad / 10 rad/s / 120 Nm |
| joint5, joint6, gripper | 5/0x15, 6/0x16, 7/0x17 | custom 0x8002 | 6.28 rad / 30 rad/s / 20 Nm |

These are protocol scaling ranges, not mechanical or safe operating limits.
Source: MakerMods' [native motor table](https://github.com/makermods-robotics/metal-python-ros/blob/bccf3c82a21453a7b1784a94966456e16ff0f008/metal_sdk/native/config/motor_config.cpp),
corroborated by its [LeRobot profiles](https://github.com/makermods-robotics/lerobot/blob/a86385a0f381fa81e227c41d2774fbb94c2610fc/src/lerobot/motors/damiao/tables.py)
and [joint mapping](https://github.com/makermods-robotics/lerobot/blob/a86385a0f381fa81e227c41d2774fbb94c2610fc/src/lerobot/robots/metal_follower/metal_follower.py).
The adapter defines these profiles with the shared driver's custom `MotorProfile`
API and registers them on its own `DamiaoCodec`; no Metal-specific library enum
or profile table is needed. The dependency minimum is `can-motor-control>=0.0.11`. Physical use requires
the custom profile API, per-motor feedback fix, and macOS shutdown TX drain
included in 0.0.11.
Version 0.0.8 lacks the API;
this adapter fails before opening USB rather than substituting stock profiles.

## Run

Install the `control` extra. `dimos run coordinator-makermods-metal` defaults
to in-memory hardware. The `joint_trajectory` task controls `joint1` through
`joint6`; `metal_gripper` controls `arm/gripper`, where 0 is closed and 1 is open.

For physical hardware, finish the vendor's motor ID and zero-pose procedure,
then use `dimos hardware can list` to obtain the device selector. Pass
`--can-port` after the blueprint name: a configured SocketCAN interface on Linux
or the exact USB serial on macOS. Gripper opening direction defaults to `increasing_position`; there is no
gripper-direction CLI flag. The direction is fixed by the Metal topology.
Gripper current defaults to
`0.1` per-unit; there is no gripper-current CLI flag. Python callers may override
`metal_hardware(gripper_current=...)` with a value in `(0, 1]`. This ratio is not
amperes and is not MakerMods Lab's 0.5 A default.
See [CAN setup](/docs/capabilities/manipulation/openyam_integration.md#can-transport).

The model is loaded lazily from the pinned official `metal_with_gripper.urdf`,
with the two finger joints fixed. The shared base validates six ordered arm
joints and their angular limits. Gravity compensation is disabled for initial
commissioning. Metal allows up to 0.06 rad of feedback overshoot at URDF
boundaries, normalizing the reported state to the boundary. This accommodates
measured joint3 zero-stop feedback of -0.05050 rad; it is a dimOS tolerance,
not a vendor limit. Planning limits stay unchanged. Larger deviations latch a
fault and disable motors; initial arm feedback is checked before gripper calibration.
Follow gains come from the pinned LeRobot configuration.

Activation enables torque and calibrates the gripper by moving toward its
endpoints. Support the arm and clear its travel. Deactivation removes torque.
The shared gripper uses normalized opening and position/current control;
MakerMods Lab's raw-angle MIT gripper and holding-torque controller have different
behavior. This integration does not claim teleoperation, holding-force tuning,
or physical commissioning equivalence.

## MakerMods CAN adapter on macOS

There are two upstream paths, depending on adapter firmware:

| Path | MakerMods usage | dimOS alignment |
| --- | --- | --- |
| SLCAN firmware | LeRobot defaults to `can_interface="slcan"`, using `python-can` + `pyserial` on `/dev/cu.usbmodem…` | Not currently supported by the shared native driver |
| candleLight / gs_usb firmware | Lab accepts `gs_usb:<exact USB serial>`, opens channel 0 with PyUSB/libusb and the `gs-usb` frame codec | Same CAN/USB protocol; native Rust + nusb/IOKit replaces Python USB dependencies |

The pinned [LeRobot Metal documentation](https://github.com/makermods-robotics/lerobot/blob/a86385a0f381fa81e227c41d2774fbb94c2610fc/docs/source/metal.mdx)
describes SLCAN on macOS, but explicitly says macOS and Windows have not been
tested end to end. Its Linux CANable timing measurements do not prove macOS
hardware validation. A CANable model name alone does not establish which
firmware is installed; the USB serial device path and native gs_usb selector
are not interchangeable.

The more recent Lab [transport hook](https://github.com/makermods-robotics/makermodslab/blob/f7980524665e3447fb7d07e945afea72800ac290/makermodslab/maker_can.py)
routes Damiao buses with `gs_usb:` ports into its
[bounded USB transport](https://github.com/makermods-robotics/makermodslab/blob/f7980524665e3447fb7d07e945afea72800ac290/makermodslab/gs_usb_transport.py).
Lab installs `makermodslab[gs-usb]` and `brew install libusb`. It selects a
unique exact serial, claims the selected adapter, programs 1 Mbps, sends
classic 8-byte standard frames, ignores TX echoes, bounds USB I/O, and releases
the selected handle on failure/shutdown. Descriptor discovery does not enable motors.

Our native path likewise selects a unique serial and channel 0, configures
1 Mbps classic CAN, separates TX echoes from feedback, bounds queues/I/O, and
resets the CAN channel and releases its handle at shutdown. It uses asynchronous
native USB transfers and echo credits. Lab optionally enables hardware timestamps;
our transport uses untimestamped classic host frames. These are protocol options,
not a need to add a second motor stack. Lab avoids USB-device reset; our
initialization resets the CAN channel rather than re-enumerating the USB device.

Device coverage is narrower in dimOS: the base currently filters USB VID:PID
`1d50:606f`, while Lab uses its Python gs_usb device matcher. Inspect the actual
adapter's descriptors before claiming compatibility with a particular supplied
unit. In dimOS pass the bare serial, without Lab's `gs_usb:` prefix. No libusb,
PyUSB, `python-can`, or `gs-usb` Python package is needed by our macOS driver.
SLCAN-only adapters need a shared-driver transport extension or compatible
firmware; passing a tty path to the existing native backend will not work.

This is source-level alignment, not a hardware certification. Adapter firmware,
USB identity, zero offsets, profile registers, gripper behavior, round-trip
latency, fault handling, and reconnect behavior remain physical bring-up checks.

## Planning and Viser

Run the coordinator together with `ManipulationModule` and its Viser UI:

```bash
uv sync --extra planning --inexact
uv run dimos run makermods-metal-planner-coordinator
```

Open [the local Viser UI](http://127.0.0.1:8095). Without a CAN selector the
blueprint uses mock hardware. The coordinator streams actual joint state to
ManipulationModule; planned trajectories execute through its `joint_trajectory`
task, and the UI controls the independent normalized `metal_gripper` task.
Use the panel to set joint/pose targets, plan, preview, then execute. Initial
planning speed scale is 0.2; the panel can change it.

The pose target is the official `gripper_base` mount frame, not a calibrated
fingertip TCP. The planner fixes the finger joints and does not visualize their
opening. A renamed fixed mount joint avoids a joint/link frame-name collision
in the official URDF. Model checkout and mesh loading occur at startup.

For commissioned physical Metal hardware, the same blueprint accepts
`--can-port` as described above. The gripper
uses the default current ratio of 0.1. Selecting it activates motors and moves the gripper for calibration;
first verify the model, firmware profile, motor zero pose, and gripper settings.

## Dual-arm setup

This stack adds two independent Metal arm hardware components to one
ControlCoordinator and one bimanual ManipulationModule. Viser offers selectable
`left_arm` and `right_arm` planning groups. Both arms share a collision world and
a single 12-joint trajectory task, allowing synchronized plans and cross-arm
collision checking. Each gripper has its own task and stream. This stack adds no
teleoperation module or separate motor driver.

### Run with mock hardware

```bash
uv sync --extra planning --inexact
uv run dimos run dual-makermods-metal-planner-coordinator
```

Open [Viser](http://127.0.0.1:8095). Select either planning group to manipulate
its pose target. ManipulationModule's `plan_to_joints` and
`plan_to_poses` accept both group IDs for a synchronized bimanual plan. Planning
speed scale defaults to 0.2. Each arm contributes six angular joints and one
normalized gripper; finger geometry remains fixed in the planning model.

### Geometry and dimensions

The model creates its base in memory with `RobotModel.from_root_link`, then calls
`with_attached_model` twice to weld copies of the pinned official Metal URDF
described above.
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

The RobotModel attachment API owns namespacing, asset resolution, and fixed
welds to `dual_metal_base`. It uses the existing in-memory load pipeline without
generating URDF files or adding a separate cache. The in-memory base has no source file; each arm resolves assets relative to its
own URDF. Importing the blueprint does not load the arm assets.

### Gripper task bindings

Both normalized gripper tasks and their separate input streams remain available
on the ControlCoordinator. This layer does not bind them to ManipulationModule
planning groups or enable the Viser gripper controls. Per-group bindings belong
to [PR #4747](https://github.com/dimensionalOS/dimos/pull/4747), which introduces
`ManipulationModuleConfig.gripper_tasks`. After that API lands, the bindings are
`left_arm` to `left_arm_gripper` and `right_arm` to `right_arm_gripper`.
The model and planning configuration carry no deployment-specific gripper mapping.

### Physical CAN setup

Both arms reuse `MakerModsMetalAdapter` with the shared custom firmware profiles
from `can-motor-control>=0.0.11`. Each arm uses its own CAN bus because motor
IDs 1-7 are repeated. Run `dimos hardware can list`, identify and label both
adapter serials, and supply **both distinct** selectors:

```bash
uv run dimos run dual-makermods-metal-planner-coordinator \
  --left-can-port LEFT_USB_SERIAL \
  --right-can-port RIGHT_USB_SERIAL \
  --left-base-xyz '[0, 0.25, 0]' \
  --right-base-xyz '[0, -0.25, 0]'
```

Both grippers use the Metal topology's fixed `increasing_position` direction.
The geometry in this example must match the commissioned rig.
Linux selectors are configured SocketCAN interfaces such as `can0` and `can1`;
macOS selectors are bare unique gs_usb USB serials. Partial, empty, and duplicate
selectors are rejected rather than selecting one real and one mock arm.
Current ratio defaults to 0.1 on each gripper. Selecting physical hardware
activates both arms and calibrates both grippers; stopping removes torque.
Gravity compensation remains disabled as in the single-arm stack.

Without physical selectors both components are in-memory mocks. Physical dual-arm
commissioning was not performed as part of this integration.
