## Why

DimOS currently has a passive `JointTrajectoryTask` that samples nominal joint-space trajectories and emits final `SERVO_POSITION` commands directly to the coordinator. This is appropriate for rigid motion, but it gives no reusable way to soften trajectory tracking under contact or unexpected resistance. A separate compliance task cannot safely run beside the trajectory task today because the coordinator arbitrates final joint ownership rather than composing command transforms.

This change introduces a narrow compliant joint trajectory capability: trajectory generation remains the nominal reference source, while a compliance transform shapes the final servo-position command using joint feedback. The first implementation targets position-servo compliance that works across existing manipulator adapters and MuJoCo simulation without requiring torque output support.

## What Changes

- Add a coordinator-facing composed control task pattern for one source task plus ordered reference transforms.
- Add a compliant joint trajectory behavior that composes joint trajectory sampling with joint-space compliance output shaping.
- Add a reusable joint compliance task that can consume an upstream joint position reference and produce a bounded `SERVO_POSITION` command.
- Keep existing rigid trajectory behavior available and unchanged.
- Add verification coverage for unit-level compliance math, composed task behavior, and MuJoCo simulation/contact scenarios.
- No **BREAKING** public CLI, skill/MCP, or hardware-safety behavior changes are intended.

## Affected DimOS Surfaces

- Modules/streams: control task abstractions, trajectory task integration, coordinator task outputs, joint command/task tests.
- Blueprints/CLI: manipulator coordinator task configuration may gain a compliant trajectory task option; existing blueprint behavior remains unchanged unless the new task is selected.
- Skills/MCP: none directly.
- Hardware/simulation/replay: manipulator adapters remain position/velocity-command based; MuJoCo simulation is used for smoke/contact verification through existing xArm/Piper-style simulated hardware paths.
- Docs/generated registries: control/task documentation and capability docs may be updated; generated blueprint registry changes are only needed if a new blueprint is added.

## Capabilities

### New Capabilities

- `compliant-joint-trajectory-control`: Joint-space trajectory execution with bounded position-servo compliance and verification expectations.

### Modified Capabilities

- None.

## Impact

Developers gain a supported path for compliant trajectory execution without introducing coordinator-level task graphs or torque-mode requirements. Existing users of `JointTrajectoryTask` should see no behavioral change. Main risks are control tuning, safety-bound defaults, and simulation realism; the implementation should therefore include conservative clamps, explicit effort-feedback configuration, and tests that separate unit-level behavior from MuJoCo contact demos.
