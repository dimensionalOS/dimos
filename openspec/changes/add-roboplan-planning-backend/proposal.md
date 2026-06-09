## Why

DimOS manipulation planning now has a single-active-backend abstraction for selecting one executable planning scene at startup, but the available implementation is still centered on Drake. RoboPlan offers a separate native planning stack with scene loading, collision checking, RRT planning, IK, path shortcutting, and optional TOPPRA timing that can complement Drake and MPlib without being forced through Drake-shaped internals.

This change adds RoboPlan as a first-class optional manipulation planning backend so users can choose it through the existing backend-selection surface while preserving current planning skills, trajectory storage, and coordinator execution behavior. RoboPlan should own its native scene representation when selected, report unsupported features honestly, and avoid any parallel Drake/RoboPlan scene synchronization.

## What Changes

- Add RoboPlan as a selectable active manipulation planning backend alongside existing backend choices.
- Add behavior for configuring RoboPlan robot planning assets, including URDF, SRDF, package paths, planning group, joint ordering, base frame, and end-effector frame requirements.
- Add RoboPlan-backed joint-space planning, pose planning where supported, collision/path validation, FK/Jacobian queries where supported, and normalized planning results that can be stored, previewed, and executed through existing DimOS manipulation flows.
- Add RoboPlan scene projection behavior for DimOS obstacles and diagnostics for unsupported or approximated scene features.
- Add clear availability, dependency, and configuration errors that trigger only when RoboPlan is selected.
- Preserve existing Drake/default manipulation behavior and existing ControlCoordinator trajectory execution semantics.
- **BREAKING**: None expected for public CLI, skills, or hardware execution behavior. RoboPlan support is additive and opt-in.

## Affected DimOS Surfaces

- Modules/streams: ManipulationModule backend selection and planning flow, manipulation planning backend facades, robot state synchronization into the active backend, obstacle scene updates, collision/path validation, normalized planned paths and trajectories.
- Blueprints/CLI: Manipulation and arm-planner blueprints may gain documented RoboPlan backend configuration. No new top-level CLI command is expected. Generated blueprint registries are affected only if new user-runnable RoboPlan demo blueprints are added.
- Skills/MCP: Existing manipulation planning and execution skills should keep the same user-facing names and behavior while surfacing RoboPlan-specific capability diagnostics and planning failures when RoboPlan is active.
- Hardware/simulation/replay: Real and simulated manipulator execution must continue through ControlCoordinator trajectory tasks. RoboPlan hardware use requires mock/sim and validation coverage before supervised real-hardware QA. Replay impact is limited to manipulation stacks that select RoboPlan and feed recorded joint/object streams into the active backend.
- Docs/generated registries: Manipulation planning docs, backend selection docs, RoboPlan robot asset configuration docs, optional dependency troubleshooting, and generated registries if new blueprints are exported.

## Capabilities

### New Capabilities

- `roboplan-manipulation-planning`: Behavior for configuring and using RoboPlan as an optional active manipulation planning backend, including backend availability, robot asset validation, joint and pose planning, scene projection, collision/path validation, FK/Jacobian support where available, normalized result/trajectory behavior, capability diagnostics, and safe omission when RoboPlan is not selected.

### Modified Capabilities

None. This change adds a RoboPlan-specific backend capability on top of the existing active manipulation planning backend model without changing the current default backend requirements.

## Impact

Users gain an additional planner choice without changing the manipulation RPC, skill, or coordinator execution surfaces they already use. Developers get a concrete backend target for RoboPlan while keeping RoboPlan imports and dependency failures isolated to configurations that select the backend.

Compatibility risk is primarily around native dependency availability, robot asset completeness, joint/frame ordering, scene projection semantics, and trajectory normalization. Hardware safety risk is controlled by keeping execution routed through existing trajectory tasks and requiring validation before real-hardware use. Documentation and QA should cover RoboPlan install verification, missing dependency/config diagnostics, mock or simulated planning, obstacle handling, pose/frame conventions, and confirmation that Drake/default manipulation stacks remain unaffected.
