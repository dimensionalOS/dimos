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
or profile table is needed. The dependency minimum is `can-motor-control>=0.0.10`. Physical use requires
that release to contain the custom profile API and per-motor feedback fix.
Version 0.0.8 lacks the API;
this adapter fails before opening USB rather than substituting stock profiles.

## Run

Install the `control` extra. `dimos run coordinator-makermods-metal` defaults
to in-memory hardware. The `joint_trajectory` task controls `joint1` through
`joint6`; `metal_gripper` controls `arm/gripper`, where 0 is closed and 1 is open.

For physical hardware, finish the vendor's motor ID and zero-pose procedure,
then use `dimos hardware can list` to obtain the device selector. Pass
`--can-port` after the blueprint name: a configured SocketCAN interface on Linux
or the exact USB serial on macOS. Also pass `--gripper-opening-direction`
(`increasing_position` or `decreasing_position`) and `--gripper-current`
(the commissioned per-unit current in `(0, 1]`). Both settings are required.
See [CAN setup](/docs/capabilities/manipulation/openyam_integration.md#can-transport).

The model is loaded lazily from the pinned official `metal_with_gripper.urdf`,
with the two finger joints fixed. The shared base validates six ordered arm
joints and their angular limits. Gravity compensation is disabled for initial
commissioning. Follow gains come from the pinned LeRobot configuration.

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
`--can-port`, `--gripper-opening-direction`, and `--gripper-current` as described
above. Selecting it activates motors and moves the gripper for calibration;
first verify the model, firmware profile, motor zero pose, and gripper settings.
