## Why

DimOS manipulation planning currently exposes IK through `KinematicsSpec`, but the selectable planning backends are limited to the existing Jacobian and Drake optimization implementations. A customized Pinocchio IK solver also exists, but it is control-task oriented and is not available through the planning stack used by `ManipulationModule.plan_to_pose()`.

Adding Pink as a planning kinematics backend gives manipulation users a task-based, Pinocchio-backed differential IK option while preserving the existing customized Pinocchio control paths. This creates a clean path to compare Pink against the current Jacobian backend from the same planning API and manual QA flow.

## What Changes

- Add a new Pink-based IK backend selectable through the manipulation planning kinematics factory.
- Preserve the existing `KinematicsSpec.solve(...)` contract and `IKResult` return shape.
- Keep existing standalone customized Pinocchio IK usage in cartesian and teleop control tasks unchanged.
- Add dependency/import handling so environments without Pink fail clearly when the Pink backend is requested, without breaking other kinematics backends.
- Add tests and a simulation-only manual QA command that exercises factory selection and a real planning path using the Pink backend.
- No **BREAKING** public API, CLI, or hardware-safety behavior changes are intended.

## Affected DimOS Surfaces

- Modules/streams: manipulation planning kinematics backend selection; `ManipulationModule.plan_to_pose()` behavior only when configured to use the new backend; no stream schema changes.
- Blueprints/CLI: expose a simulation-safe xArm manipulation QA path by selecting `kinematics_name="pink"` on an existing blueprint through CLI `-o` overrides; do not add a dedicated Pink blueprint unless later needed.
- Skills/MCP: no direct skill or MCP tool changes.
- Hardware/simulation/replay: primary validation should run in simulation or replay first; hardware usage remains gated by existing collision checks and planning flow.
- Docs/generated registries: update manipulation/planning documentation for backend selection and CLI override usage; no generated blueprint registry change is required when using existing blueprints.

## Capabilities

### New Capabilities
- `manipulation-kinematics-backends`: Behavior for selecting and running alternative IK backends through the manipulation planning stack, including Pink.

### Modified Capabilities
- None.

## Impact

Developers gain a new IK backend that can be selected by name and compared against the current Jacobian solver without changing call sites that already depend on `KinematicsSpec`. The implementation introduces an optional Pink/PyPI `pin-pink` dependency path plus QP solver availability considerations, so error messages and dependency documentation are part of the scope.

Testing should cover factory dispatch, missing dependency behavior, successful Pink solve result shaping, joint order/limit handling, and collision-check delegation. Manual QA should provide a clear command sequence for installing the optional dependency, running a simulation-safe xArm manipulation blueprint configured with `kinematics_name="pink"`, planning to a target pose, and comparing the same target against the existing `jacobian` backend.
