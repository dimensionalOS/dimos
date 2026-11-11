# OpenYAM integration

OpenYAM and Dual OpenYAM use `ConnectionModule` implementations that own the
native `can-motor-control` robot. The coordinator receives `JointState` feedback
and sends position commands through streams. It does not construct hardware
adapters. `openyam-teach-collection` uses the same path with
`OpenYamConnection` in teach mode (`teach=True`): zero stiffness, teaching damping, gravity compensation
and passive grippers, declared as zero-stiffness joints. Seeed B601-DM also uses
its own direct connection.

## Run without hardware

Use `--simulation` to compose a separate `MockJointConnection`. Its initial
arm positions come from the planning model's home posture, or zero when none
is configured. Grippers start closed. Native startup requires CAN selectors.

```bash
dimos --simulation run coordinator-openyam
dimos --simulation run openyam-planner-coordinator
dimos --simulation run keyboard-teleop-openyam
dimos --simulation run keyboard-teleop-openyam-planner
dimos --simulation run teleop-webxr-openyam
dimos --simulation run coordinator-dual-openyam
dimos --simulation run dual-openyam-planner-coordinator
dimos --simulation run teleop-webxr-dual-openyam
```

`openyam-connection-control` exposes trajectory and normalized-gripper control.

## CAN transport

List the selectors accepted on the current host:

```bash
dimos hardware can list
```

Linux uses SocketCAN interface names. Configure each interface for classical
CAN at 1 Mbit/s before starting:

```bash
dimos hardware can setup can0
dimos hardware can setup can1
dimos run teleop-webxr-openyam --can-port can0
dimos run teleop-webxr-dual-openyam --left-can-port can0 --right-can-port can1
```

macOS uses native Rust IOKit access to gs_usb/candleLight adapters with USB ID
`1d50:606f`. Use their USB serial numbers as selectors. No PyUSB, libusb,
`python-can`, or Python `gs_usb` package is required. The native transport sets
the bitrate. Connect one adapter at a time, record its serial, and label its
side; adapters used together must have unique serial numbers.

```bash
dimos run teleop-webxr-openyam --can-port <USB-SERIAL>
dimos run teleop-webxr-dual-openyam \
  --left-can-port <LEFT-USB-SERIAL> \
  --right-can-port <RIGHT-USB-SERIAL>
```

Dual YAM requires both selectors, and they must differ. Blank selectors are
rejected before connection. `--simulation` composes a separate
`MockJointConnection` without starting a native CAN connection. Module-qualified overrides are also available,
for example `--openyamconnection.can-port can0` or
`--dualopenyamconnection.left-can-port can0`.

A selector starting with `/dev/` uses `DmSerialBus` on either Linux or macOS,
preserving the native Damiao serial USB-CAN transport.

## Models and joint names

The canonical planning model is the managed OpenYAM single-arm description from
`dimensionalOS/dim_description` (`openyam_single`, pinned by commit). Its arm
joints are mapped onto `yam_joint1` through `yam_joint6`. Shipped deployments
use connection source `arm`, with `arm/yam_joint1` through `arm/yam_joint6` and
`arm/joint7` for a single YAM. `joint7` is the managed model's driven finger
joint, in metres: 0.0594641 is closed and 0.01081 fully open, and `joint8`
copies it. The gripper task maps its 0.0 (closed) to 1.0 (open) command onto
that range. Dual YAM uses
`arm/left_joint1` through `arm/left_joint6`, `arm/right_joint1` through
`arm/right_joint6`, then `arm/left_gripper` and `arm/right_gripper`. Its URDF has
its finger joints fixed, so these two are openings from 0.0 closed to 1.0 open.

The deployed planner model, planning groups, task claims and recorded schema
all use those exact names. Link and target-frame names remain unchanged.
Single YAM retains a separate physical model for gravity compensation. Dual
YAM uses its arm-only combined model for planning and gravity compensation.

`OpenYamConnection()` and `DualOpenYamConnection()` default to sources `yam` and
`dual_openyam` when used directly. For a custom stack, the coordinator's model is
`make_openyam_model_config(robot="arm").model` or
`dual_openyam_control_model(robot="arm")`.

New Quest recordings and policy rollout use the deployed seven-coordinate names.
Hand-guided teaching retains its original bare names, PD gains and passive-jaw
behavior; its CAN override remains `--controlcoordinator.can-port`.

Planner gripper routing is explicit through `ManipulationModuleConfig.gripper_tasks`:
single YAM binds `manipulator` to `arm_gripper`; dual YAM binds
`left_manipulator` and `right_manipulator` to their respective `*_arm_gripper`
tasks for both command dispatch and feedback.

## Teleoperation

Single YAM WebXR uses the right controller for arm pose and a separate gripper
task. Keyboard twist control and gripper control remain independent. Manual
control takes priority over a running planner trajectory.

Dual YAM has one pose task per arm. Each controller's grip engages only its own
arm; releasing it leaves the other arm's task independent. Each trigger controls
its matching gripper. Both arms share one connection, with one CAN bus per arm.

## Motor topology

| Motor | Send ID | Reply ID | Type |
|---|---|---|---|
| joint1-3 | `0x01`-`0x03` | send ID + `0x10` | DM4340 |
| joint4-6 | `0x04`-`0x06` | send ID + `0x10` | DM4310 |
| gripper | `0x08` | `0x18` | DM4310 |

Each arm uses the same IDs on its own bus. Gripper opening decreases motor
position. The connection retains the YAM MIT gains and gravity compensation.
Commands are position-only, and invalid target messages are rejected as a
whole. The connection holds measured positions when its command deadline expires.

Support the arm before stopping because deactivation removes torque. Verify
motor types, IDs, CAN-H/CAN-L wiring, power and termination before physical use.

## Verification

See the [Damiao connection test guide](/docs/capabilities/manipulation/damiao-connections.md)
for focused tests, software-only checks and physical bring-up. The YAM tests
cover deployed joint names, gripper routing, planner priority, Dual YAM hand
isolation, model limits and the Quest profile. The native recorder test verifies
Zenoh serialization and episode extraction with synthetic input streams.
