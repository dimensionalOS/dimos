## Why

DimOS manipulation planning currently treats the planning world, scene mutation, and planner selection as tightly coupled concerns. That makes it difficult to add new planners such as MPlib without either rebuilding expensive world objects during planning or maintaining brittle synchronization between a canonical scene and backend-specific planning scenes.

This change introduces a single-active-planning-backend model: one backend owns the executable planning scene at a time, while DimOS exposes backend-agnostic scene and planner behavior to higher-level modules, skills, and blueprints. MPlib integration is the first concrete backend addition for this model because it provides a lightweight, ROS-independent manipulation planner that fits DimOS's Python-first manipulation stack.

## What Changes

- Add behavior for selecting one active manipulation planning backend at a time.
- Add unified scene access and mutation behavior for the active backend, covering robot state, obstacles, pointclouds, collision queries, and diagnostic scene summaries.
- Add unified planner behavior for joint-space planning, pose planning where supported, IK where supported, path validation, result normalization, and backend capability reporting.
- Add MPlib as an optional but complete manipulation planning backend with explicit robot model, SRDF, move-group, joint-order, package-path, scene-projection, collision, and result-normalization requirements.
- Add explicit startup backend selection behavior that prepares one active backend from durable DimOS planning configuration and reports unsupported or lossy scene features.
- Preserve existing manipulation execution behavior through normalized DimOS trajectories sent to the control coordinator.
- **BREAKING**: Developer-facing manipulation planning internals may change from direct `WorldSpec`/planner access to active-backend facade access; public robot execution behavior must continue to route through existing trajectory execution paths.

## Affected DimOS Surfaces

- Modules/streams: ManipulationModule planning flow, planning world/scene wrappers, robot state handling, obstacle/perception scene updates, trajectory planning outputs, collision query surfaces.
- Blueprints/CLI: Manipulation and xArm planner blueprints may gain backend selection/configuration for MPlib; generated blueprint registry may need updates if new demo blueprints are added. No new top-level CLI command is expected.
- Skills/MCP: Existing manipulation skills that plan or execute motions should continue to use the same user-facing behavior while receiving clearer backend errors and diagnostics.
- Hardware/simulation/replay: Real and simulated manipulator execution must continue through ControlCoordinator trajectory tasks; MPlib support should be optional and safe to omit from non-manipulation installs. Replay/manual QA should cover mock or simulated xArm planning before any hardware execution.
- Docs/generated registries: Manipulation capability docs, planning backend configuration docs, optional dependency notes, and generated blueprint registries if new blueprints are introduced.

## Capabilities

### New Capabilities

- `manipulation-planning-backends`: Backend-agnostic behavior for selecting one active planning backend at startup, mutating its scene through a unified surface, planning motions, reporting capabilities, and preserving current execution behavior.
- `mplib-motion-planning`: Behavior for configuring and using MPlib as an optional complete manipulation planning backend, including robot asset requirements, joint and pose planning, native primitive/pointcloud scene support, attached-object support where MPlib exposes it, collision diagnostics, planning result normalization, and fallback/error handling when MPlib is unavailable or a specific feature is unsupported.

### Modified Capabilities

None. No existing OpenSpec capability specs are present in this worktree.

## Impact

Users gain a clearer path to choose manipulation planners without paying per-plan world construction cost or maintaining continuous multi-backend scene synchronization. Developers get a narrower integration contract: higher layers talk to a unified active backend facade, while each backend owns its native scene representation.

Compatibility risk is mainly internal API churn around current planning world access and planner construction. Hardware safety risk is controlled by keeping execution routed through existing trajectory tasks and preserving the current execution path. Dependency impact is scoped to the existing manipulation extra: MPlib should not affect non-manipulation installs, and missing dependency/config errors should not break non-MPlib stacks. Documentation and QA should cover backend selection, MPlib robot configuration, mock/sim planning, primitive and pointcloud scene collision, current execution behavior, and diagnostics for unsupported scene features.
