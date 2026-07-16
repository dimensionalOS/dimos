## Why

The existing A1Z keyboard teleoperation blueprint drives only a mock-backed arm. It cannot provide the simulated camera, gripper motion, tabletop contact, or repeatable operator workflow needed to validate collection-ready teleoperation.

Stage 1 establishes a stable, standalone MuJoCo teleoperation environment before introducing episode recording and persistence in Stage 2.

## What Changes

- Add an explicit `--simulation` branch to the existing `keyboard-teleop-a1z` blueprint; hardware behavior remains available when simulation is disabled.
- Provide a converted A1Z MuJoCo scene with a heuristic two-finger gripper, explicit visible physical A1Z finger-pad inserts whose render and collision geometry match, a tabletop, a fixed cube, physical finger contact, and a wrist RGB camera. The inserts are non-vendor simulation fixtures.
- Extend keyboard teleoperation with latched `[` open and `]` close gripper commands.
- Compose the existing six-joint Cartesian EEF-twist task with the built-in single-joint servo task for the abstract `arm/gripper` command.
- Use a manual simulator/process restart for reset during Stage 1. Episode controls, recording, and data persistence are out of scope.

## Affected DimOS Surfaces

- Modules/streams: keyboard teleop publishes partial gripper joint commands; the control coordinator arbitrates the existing EEF-twist task with a gripper servo task.
- Blueprints/CLI: the existing `keyboard-teleop-a1z` blueprint explicitly branches on `--simulation`.
- Skills/MCP: none.
- Hardware/simulation/replay: A1Z hardware configuration gains a MuJoCo-backed variant; a deterministic A1Z tabletop scene supplies the gripper, actuator mapping, camera, and collision geometry.
- Docs/generated registries: update operator-facing usage as needed; regenerate the built-in blueprint registry if the blueprint discovery test requires it.

## Capabilities

### New Capabilities
- `a1z-simulated-keyboard-teleoperation`: simulated A1Z Cartesian keyboard teleoperation with latched gripper endpoints and a deterministic visual manipulation scene.

### Modified Capabilities

None.

## Impact

Operators can manually validate arm movement, gripper endpoint control, camera framing, cube contact, and restart-based reset using the existing blueprint name with `--simulation`. The gripper and its visible pad inserts are deliberately heuristic simulation fixtures with raw meter displacement semantics, so they must not be interpreted as a hardware-gripper contract or hardware/data-parity guarantee. Verification includes structural MuJoCo compilation, control-path tests where practical, and manual simulator smoke testing of the full pad opening sweep and cube contact; no new runtime dependency or data-collection format is introduced.
