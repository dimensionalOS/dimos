# OpenArm Mini Teleop

OpenArm Mini teleop uses the Feetech leader arms directly and publishes OpenArm
follower `JointState` commands through the generic teleop runtime. It does not
depend on LeRobot at runtime.

## Install optional dependencies

```bash
uv sync --extra openarm-mini-teleop
# Or, outside a source checkout:
pip install 'dimos[openarm-mini-teleop]'
```

The Feetech package installs as `ftservo-python-sdk` and imports as
`scservo_sdk`.

## One-shot motor ID setup

To write a physical Feetech motor ID, connect exactly one motor to the USB
controller and run the one-shot setup helper. Do not leave multiple motors on
the bus when changing IDs, especially if they may share the same current ID.

```bash
dimos hardware openarm-mini setup-motor-id \
  --port <feetech-port> \
  --baudrate <feetech-baudrate> \
  --new-id 3
```

If the current ID is known, skip scanning:

```bash
dimos hardware openarm-mini setup-motor-id \
  --port <feetech-port> \
  --baudrate <feetech-baudrate> \
  --old-id 1 \
  --new-id 3
```

The helper opens the Feetech port, verifies or scans for one responding motor,
disables torque, unlocks EEPROM, writes the ID, locks EEPROM, verifies the new
ID responds, and exits. Run calibration after motor IDs are assigned.

## Calibration storage

Runtime startup is non-interactive. Create calibration artifacts before running
the teleop blueprint. Defaults are side-specific directories:

- left: `STATE_DIR / "teleop" / "openarm_mini" / "left" / "calibration.json"`
- right: `STATE_DIR / "teleop" / "openarm_mini" / "right" / "calibration.json"`

`STATE_DIR` is DimOS' XDG state directory, typically
`~/.local/state/dimos` on Linux.

## Manual calibration

Run the calibration utility with the OpenArm Mini leader connected. The utility only
opens the leader Feetech serial ports; it never starts `ControlCoordinator` and
never connects follower OpenArm hardware.

Before calibration, place the selected leader side in its natural zero pose: the
pose designed to correspond to the OpenArm follower's all-zero arm-joint
configuration. Calibration reads arm motors `joint_1` through `joint_7` once and
writes those raw positions as `homing_offset` values. Motor 8 / gripper is not
read or stored in v1 because the OpenArm follower gripper is not yet exposed as a
formal coordinator-controllable API.

```bash
dimos hardware openarm-mini calibrate \
  --side both \
  --port-left <left-feetech-port> \
  --port-right <right-feetech-port> \
  --baudrate <feetech-baudrate>
```

The script prints a confirmation table with each semantic arm joint, physical
Feetech motor id, captured raw zero offset, and `flip` value before writing the
artifact. Calibration artifacts are strict arm-only JSON with exactly
`joint_1`...`joint_7`, each containing only:

- `id`: physical Feetech motor id for that semantic leader joint
- `homing_offset`: raw tick value captured in the leader zero pose
- `flip`: whether to negate the calibrated radians for that joint

Default flip sets match the known OpenArm Mini leader orientation. Override them
when needed:

```bash
dimos hardware openarm-mini calibrate \
  --side left \
  --port-left <left-feetech-port> \
  --port-right <right-feetech-port> \
  --baudrate <feetech-baudrate> \
  --left-flips joint_1,joint_3,joint_4,joint_5,joint_6,joint_7
```

Use `--left-flips none` or `--right-flips none` to record no flipped joints.

At runtime, raw Feetech ticks convert to radians around the captured zero using
the full Feetech encoder span, then per-joint `flip` is applied. The teleop
module maps semantic leader joints directly to OpenArm follower arm-joint names
and clamps outgoing positions to OpenArm follower joint limits before publishing. The
operator must still align the follower near the leader-implied command before
enabling teleop authority; automatic startup alignment gating is out of scope for
v1.

To inspect calibrated leader readings without starting robot control:

```bash
dimos hardware openarm-mini calibrate \
  --side left \
  --port-left <left-feetech-port> \
  --port-right <right-feetech-port> \
  --baudrate <feetech-baudrate> \
  --live-readout
```

For a Rich terminal UI that continuously displays raw ticks, calibrated radians,
sender-side clamped follower radians, motor ids, and flip values:

```bash
dimos hardware openarm-mini joint-tui \
  --side right \
  --port <feetech-port>
```

The TUI is also leader-only: it reads OpenArm Mini Feetech ports and existing
calibration files, but does not start `ControlCoordinator` or connect follower
OpenArm hardware.

The TUI visualizes one side at a time. `--side` selects the side-specific default
calibration path. Use `--calibration-path` to select a non-default calibration
artifact. The default baudrate is `1000000`; pass `--baudrate` only if your
leader was configured differently.

The CLI loads serial and visualization dependencies only after a command runs.
Listing OpenArm Mini commands and rendering their help does not discover devices
or open serial ports.

## Direct module invocation

The hardware CLI is the preferred operator interface. Developers and existing
scripts may invoke the same command functions as Python modules:

```bash
python -m dimos.teleop.openarm_mini.cli.calibrate --help
python -m dimos.teleop.openarm_mini.cli.joint_tui --help
python -m dimos.teleop.openarm_mini.cli.setup_motor_id --help
```

## Viser bring-up (mock follower)

Use the left-side blueprint to validate real OpenArm Mini leader motion before
connecting any OpenArm follower hardware:

```bash
dimos run teleop-openarm-mini-left \
  -o openarmminiteleopmodule.port_left=<left-feetech-port>
```

Teleop defaults to the standard Feetech serial baudrate of `1000000`. Override
`openarmminiteleopmodule.baudrate` only if your leader was
configured differently.

The blueprint requires:

- a real OpenArm Mini left leader connected to the configured left Feetech serial
  port
- a valid left calibration artifact
- Viser dependencies from `uv sync --extra manipulation` or `uv sync --extra all`

Without CAN ports the follower is in-memory. The leader-derived `joint_command`
streams through the coordinator's `joint_trajectory` task into mock follower
hardware, and `ManipulationModule` renders the follower-observed
`coordinator_joint_state` in Viser. Leader joint N drives follower joint N with no
reordering; each target is clamped to the OpenArm v2.0 joint limits on the sender
side and rate-limited by the trajectory task on the follower side.

Use `teleop-openarm-mini-right` for the right leader:

```bash
dimos run teleop-openarm-mini-right \
  -o openarmminiteleopmodule.port_right=<right-feetech-port>
```

The blueprints publish the coordinator joint names `openarm_left_joint1` through
`openarm_left_joint7` and `openarm_right_joint1` through `openarm_right_joint7`.

## Dual-arm bring-up

Use `teleop-openarm-mini` for bimanual leader teleop. It runs one bimanual
`OpenArmMiniTeleopModule`, one coordinator, and one `ManipulationModule` with the
bimanual OpenArm model:

```bash
dimos run teleop-openarm-mini \
  -o openarmminiteleopmodule.port_left=<left-feetech-port> \
  -o openarmminiteleopmodule.port_right=<right-feetech-port>
```

## Driving the real OpenArm follower

The same blueprints drive the physical bimanual OpenArm 2.0 when both CAN ports
are supplied. Bring the CAN interfaces up first as described in
[OpenArm Integration](/docs/capabilities/manipulation/openarm_integration.md),
then add the coordinator ports:

```bash
dimos run teleop-openarm-mini \
  -o openarmminiteleopmodule.port_left=<left-feetech-port> \
  -o openarmminiteleopmodule.port_right=<right-feetech-port> \
  --controlcoordinator.left-can-port can0 \
  --controlcoordinator.right-can-port can1
```

Both CAN ports are required for the physical adapter, including the single-side
blueprints; the non-driven arm holds its position. Start with the leader posed
near the follower so the first streamed target is a short move, and keep the
follower workspace clear.
