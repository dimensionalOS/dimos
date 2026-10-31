# OpenYAM integration

OpenYAM and Dual OpenYAM use `ConnectionModule` implementations that own the
native `can-motor-control` robot. The coordinator receives `JointState` feedback
and sends position commands through streams. It does not construct hardware
adapters.

## Run without hardware

All shipped YAM blueprints use an in-memory connection when no CAN selector is
supplied. Single YAM starts at zero; Dual YAM starts at its configured home
posture. These initial positions apply only to the in-memory robot.

```bash
dimos run coordinator-openyam
dimos run openyam-planner-coordinator
dimos run keyboard-teleop-openyam
dimos run keyboard-teleop-openyam-planner
dimos run teleop-webxr-openyam
dimos run coordinator-dual-openyam
dimos run dual-openyam-planner-coordinator
dimos run teleop-webxr-dual-openyam
```

`openyam-connection-control` exposes trajectory and normalized-gripper control.
`openyam-quest-collection` adds a wrist webcam, episode monitor and recorder to
the WebXR stack. Its seven-dimensional state and action schemas use the same
ordered joint names as the deployed connection.

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
rejected before connection. `--simulation` always selects the in-memory robot,
even when selectors are supplied. Module-qualified overrides are also available,
for example `--openyamconnection.can-port can0` or
`--dualopenyamconnection.left-can-port can0`.

## Models and joint names

The canonical planning assets retain their arm joint names. Shipped deployments
use connection source `arm`, with `arm/yam_joint1` through `arm/yam_joint6` and
`arm/gripper` for a single YAM. Dual YAM uses `arm/left_joint1` through
`arm/left_joint6`, `arm/right_joint1` through `arm/right_joint6`, then
`arm/left_gripper` and `arm/right_gripper`. Gripper values are normalized openings,
`0.0` closed and `1.0` open.

The deployed planner model, planning groups, task claims and recorded schema
all use those exact names. Link and target-frame names remain unchanged.
Single YAM fixes the two finger joints in its planning model and retains a
separate physical model for gravity compensation. Dual YAM uses its arm-only
combined model for planning and gravity compensation.

`OpenYamConnection()` and `DualOpenYamConnection()` default to sources `yam` and
`dual_openyam` when used directly. Use `.models(source=...)` for a matching
coordinator/planner model when composing a custom stack.

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
