# OpenArm Mini Teleop

OpenArm Mini teleop uses the Feetech leader arms directly and publishes OpenArm
follower `JointState` commands through the generic teleop runtime. It does not
depend on LeRobot at runtime.

## Install optional dependencies

```bash
uv sync --extra openarm
# or only the leader teleop SDK
uv sync --extra openarm-mini-teleop
```

The Feetech package installs as `ftservo-python-sdk` and imports as
`scservo_sdk`.

## One-shot motor ID setup

To write a physical Feetech motor ID, connect exactly one motor to the USB
controller and run the one-shot setup helper. Do not leave multiple motors on
the bus when changing IDs, especially if they may share the same current ID.

```bash
python -m dimos.teleop.openarm_mini.tools.setup_motor_id \
  --port <feetech-port> \
  --baudrate <feetech-baudrate> \
  --new-id 3
```

If the current ID is known, skip scanning:

```bash
python -m dimos.teleop.openarm_mini.tools.setup_motor_id \
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
python -m dimos.teleop.openarm_mini.tools.calibrate \
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
python -m dimos.teleop.openarm_mini.tools.calibrate \
  --side left \
  --port-left <left-feetech-port> \
  --baudrate <feetech-baudrate> \
  --left-flips joint_1,joint_3,joint_4,joint_5,joint_6,joint_7
```

Use `--left-flips none` or `--right-flips none` to record no flipped joints.

At runtime, raw Feetech ticks convert to radians around the captured zero using
the full Feetech encoder span, then per-joint `flip` is applied. The adapter maps
semantic leader joints directly to OpenArm follower arm-joint names and clamps
outgoing positions to OpenArm follower joint limits before publishing. The
operator must still align the follower near the leader-implied command before
enabling teleop authority; automatic startup alignment gating is out of scope for
v1.

To inspect calibrated leader readings without starting robot control:

```bash
python -m dimos.teleop.openarm_mini.tools.calibrate \
  --side left \
  --port-left <left-feetech-port> \
  --baudrate <feetech-baudrate> \
  --live-readout
```

For a Rich terminal UI that continuously displays raw ticks, calibrated radians,
sender-side clamped follower radians, motor ids, and flip values:

```bash
python -m dimos.teleop.openarm_mini.tools.joint_tui \
  --side both \
  --port-left <left-feetech-port> \
  --port-right <right-feetech-port> \
  --baudrate <feetech-baudrate>
```

The TUI is also leader-only: it reads OpenArm Mini Feetech ports and existing
calibration files, but does not start `ControlCoordinator` or connect follower
OpenArm hardware.

Use `--left-calibration-path` and `--right-calibration-path` to write or read
non-default calibration directories.

## Visualization-only Viser bring-up

Use the left-side Viser blueprint to validate real OpenArm Mini leader motion
before connecting any OpenArm follower hardware:

```bash
dimos run openarm-mini-left-teleop-viser \
  -o teleopmodule.adapter.backend=openarm_mini \
  -o teleopmodule.adapter.port_left=<left-feetech-port> \
  -o teleopmodule.adapter.baudrate=<feetech-baudrate>
```

The blueprint requires:

- a real OpenArm Mini left leader connected to the configured left Feetech serial
  port
- a valid left calibration artifact
- Viser dependencies from `uv sync --extra manipulation` or `uv sync --extra all`

This workflow is visualization-only on the follower side. It routes the
leader-derived `joint_command` through `ControlCoordinator` into mock follower
hardware, then renders `coordinator_joint_state` in `ManipulationModule`'s Viser
backend. It never connects real OpenArm follower hardware.

## Right-arm coordinator + Viser bring-up

Use `openarm-mini-right-teleop-viser` to route a real OpenArm Mini right leader
through `ControlCoordinator` and render the right follower state in
`ManipulationModule`'s Viser backend. The leader is always physical; the follower
is always mock in this blueprint.

Run with the required right leader connection settings:

```bash
uv run dimos run openarm-mini-right-teleop-viser \
  -o teleopmodule.adapter.backend=openarm_mini \
  -o teleopmodule.adapter.port_right=<right-feetech-port> \
  -o teleopmodule.adapter.baudrate=<feetech-baudrate>
```

The blueprint requires:

- a real OpenArm Mini right leader connected to the configured right Feetech
  serial port
- a valid right calibration artifact at the default right calibration path, or a
  configured `right_calibration_path`
- Viser dependencies from `uv sync --extra manipulation` or `uv sync --extra all`

The right blueprint publishes ManipulationModule-compatible global coordinator
joint names (`right_arm/openarm_right_joint1` through
`right_arm/openarm_right_joint7`). Viser renders follower-observed
`coordinator_joint_state`, not the raw sender-side command, so mock mode validates
the same coordinator routing used before real hardware is connected. Real
follower hardware is intentionally out of scope for these Viser demo blueprints.

## Dual-arm coordinator + Viser bring-up

Use `openarm-mini-dual-teleop-viser` for bimanual OpenArm Mini leader teleop with
the same coordinator-observed Viser path. It uses one bimanual generic
`TeleopModule` with the OpenArm Mini adapter backend, one `ControlCoordinator`,
and one `ManipulationModule` with both left and right OpenArm models.

Run with the required leader connection settings:

```bash
uv run dimos run openarm-mini-dual-teleop-viser \
  -o teleopmodule.adapter.backend=openarm_mini \
  -o teleopmodule.adapter.port_left=<left-feetech-port> \
  -o teleopmodule.adapter.port_right=<right-feetech-port> \
  -o teleopmodule.adapter.baudrate=<feetech-baudrate>
```

The dual blueprint publishes ManipulationModule-compatible global coordinator
joint names for both arms:

- `left_arm/openarm_left_joint1` through `left_arm/openarm_left_joint7`
- `right_arm/openarm_right_joint1` through `right_arm/openarm_right_joint7`

Each follower side remains mocked. Real follower hardware is intentionally out of
scope for these Viser demo blueprints.
