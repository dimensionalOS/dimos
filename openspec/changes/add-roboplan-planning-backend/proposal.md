## Why

DimOS manipulation planning currently has protocol abstractions for interchangeable worlds and planners, but the runtime wiring is effectively Drake-only for the world backend and supports only the existing generic RRT planner. This prevents evaluating RoboPlan as an alternate model, collision, and planning backend while preserving the existing `WorldSpec`/`PlannerSpec` swappability goal.

Adding a RoboPlan backend lets DimOS test a second manipulation planning implementation behind the same high-level APIs. The first proof point is making the existing generic `RRTConnectPlanner` plan against a RoboPlan-backed world, then adding RoboPlan-native planning as an explicit planner choice.

## What Changes

- Add an optional RoboPlan manipulation backend dependency set using Git-sourced RoboPlan packages while RoboPlan is still in active development.
- Add explicit manipulation backend configuration for selecting the world backend independently from the planner.
- Add a RoboPlan-backed world implementation that supports the `WorldSpec` methods needed by normal runtime wiring and the generic RRT planner.
- Allow `planner_name="roboplan"` only when `world_backend="roboplan"`; in that mode the RoboPlan world may also serve as the `PlannerSpec` implementation.
- Fail fast for invalid world/planner/kinematics combinations and for unsupported planning-critical robot or obstacle features.
- Leave obscure or high-risk non-critical query methods explicitly unimplemented until their semantics are verified.

## Affected DimOS Surfaces

- Modules/streams: manipulation planning world, planner, kinematics, and world-monitor runtime wiring; no new streams are expected.
- Blueprints/CLI: manipulation module configuration gains explicit `world_backend` selection and validates combinations with `planner_name` and `kinematics_name`; normal module option overrides should be usable from `dimos run`.
- Skills/MCP: no direct skill or MCP surface changes are expected.
- Hardware/simulation/replay: affects manipulation planning stacks for simulated and real manipulators that opt into the RoboPlan backend; unsupported safety-critical collision/model features must fail before planning instead of being ignored.
- Docs/generated registries: update manipulation planning/backend documentation if present; no blueprint registry update is expected unless new blueprints are added.

## Capabilities

### New Capabilities

- `manipulation-planning-backends`: behavior for explicit manipulation world/planner backend selection, RoboPlan world compatibility with generic planning, and validation of backend combinations.

### Modified Capabilities

<!-- None. No existing OpenSpec capability specs are present for this behavior. -->

## Impact

Developers gain a way to install and select RoboPlan as an optional manipulation planning backend without replacing the current Drake default. Existing Drake behavior remains the default path. Compatibility risk is concentrated in optional compiled RoboPlan dependencies, model loading/joint mapping semantics, obstacle geometry support, and collision-query correctness. Testing should cover factory/startup validation, unsupported input failures, RoboPlan world compatibility with the generic RRT planner, and RoboPlan-native planner wiring when implemented.
