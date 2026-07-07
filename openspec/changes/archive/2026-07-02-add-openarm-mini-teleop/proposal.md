## Why

DimOS has Quest and keyboard teleoperation paths, but it does not support using an OpenArm Mini physical leader as a direct teleoperation input for OpenArm manipulators. Adding this path enables low-latency joint mirror teleoperation while establishing a small reusable teleop adapter shell for future devices.

## What Changes

- Add a generic teleoperation module shell that owns DimOS lifecycle, periodic command retrieval, coordinator-facing publishing, and structural safety checks.
- Add a teleop adapter contract where device-specific adapters connect to a human input source and return a command envelope containing exactly one primary coordinator-facing motion command type.
- Add an OpenArm Mini teleop adapter that reads Feetech-based leader arm state and emits OpenArm follower `JointState` commands.
- Add non-interactive OpenArm Mini runtime calibration loading with side-specific default calibration directories under DimOS state storage.
- Add a manual OpenArm Mini calibration/demo script that performs interactive leader calibration outside normal blueprint startup.
- Add an OpenArm Mini teleop blueprint that connects the generic teleop module to the existing OpenArm control coordinator through `joint_command`.
- Keep existing Quest teleop behavior unchanged in v1.

## Capabilities

### New Capabilities
- `teleop-adapter-runtime`: Defines reusable teleoperation adapter/module behavior, command envelopes, safety ownership, and coordinator stream publishing.
- `openarm-mini-teleop`: Supports OpenArm Mini leader-arm teleoperation of OpenArm followers, including calibration storage, direct Feetech integration, and blueprint wiring.

### Modified Capabilities

None.

## Impact

- Affected code areas: `dimos/teleop/`, OpenArm manipulator blueprints, generated blueprint registry, and optional dependency metadata.
- Adds a narrow optional dependency for the Feetech motor communication library used by OpenArm Mini teleop.
- Does not add a LeRobot runtime dependency; LeRobot remains only a reference for OpenArm Mini behavior.
- Does not change `ControlCoordinator` inputs or existing Quest teleop modules for v1.
