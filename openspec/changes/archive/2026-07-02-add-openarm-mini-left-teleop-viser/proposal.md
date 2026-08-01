## Why

OpenArm Mini teleop needs a safe bring-up path that validates real leader hardware, calibration, sign/flip behavior, and follower joint naming before any OpenArm follower hardware is connected. A left-arm Viser validation blueprint gives operators immediate visual feedback from the real OpenArm Mini leader while keeping the follower side visualization-only.

## What Changes

- Add a left-side visualization-only OpenArm Mini teleop validation path.
- Require a real OpenArm Mini left leader connected over Feetech serial; fake/replay leader input is out of scope for this change.
- Render the leader-derived left OpenArm follower arm-joint command in Viser.
- Do not wire `ControlCoordinator`, OpenArm follower hardware, or mock OpenArm hardware into this validation path.
- Support left-only OpenArm Mini adapter operation so the validation path does not require right-side leader calibration or connection.
- Add tests and documentation for the left-only Viser validation workflow.

## Capabilities

### New Capabilities
- `openarm-mini-left-teleop-viser`: Visualization-only validation for a real OpenArm Mini left leader driving a left OpenArm follower model in Viser without follower hardware or coordinator execution.

### Modified Capabilities
- None.

## Impact

- Affected code: OpenArm Mini teleop adapter/config, teleop/visualization module or blueprint wiring, OpenArm blueprint registry, and related tests/docs.
- Dependencies: uses existing OpenArm Mini Feetech optional dependency and existing Viser visualization dependency path; no new follower hardware dependency.
- Systems: adds a validation blueprint/tooling path for hardware bring-up, but does not change the production bimanual OpenArm Mini teleop blueprint.
