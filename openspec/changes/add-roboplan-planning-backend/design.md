## Context

DimOS manipulation planning now has an active backend boundary under `dimos/manipulation/planning/backends/`. `ManipulationModuleConfig` already exposes `planning_backend: str = "drake"` and `planning_backend_options: dict[str, Any]`, and `ManipulationModule` initializes one backend through `create_planning_backend()`, registers configured `RobotModelConfig` values, finalizes the backend scene, routes `joint_state: In[JointState]` updates into the active `SceneFacade`, and executes normalized trajectories through the existing `ControlCoordinator` task RPC path.

The current registry only accepts `"drake"`. `DrakePlanningBackend` is a compatibility wrapper around `WorldMonitor`, current Drake world behavior, RRT planning, IK, Meshcat visualization, perception obstacle caches, and legacy `WorldSpec`/`PlannerSpec`/`KinematicsSpec` escape hatches. The public backend Protocols already define the intended adapter seam: `PlanningBackend`, `SceneFacade`, `PlannerFacade`, `VisualizationFacade`, `BackendCapabilities`, `BackendDiagnostics`, `SceneUpdateResult`, and `PlannedMotion`.

RoboPlan should be added as a sibling active backend, not as a Drake subcomponent and not as an MPlib-shaped adapter. RoboPlan owns its native `roboplan.core.Scene` plus planner/IK/retiming objects such as `roboplan.rrt.RRT`, `roboplan.simple_ik.SimpleIk`, and optional `roboplan.toppra.PathParameterizerTOPPRA`. The integration must preserve existing manipulation RPCs, skills, trajectory storage, and coordinator execution behavior while making RoboPlan import/config errors visible only when users select `planning_backend="roboplan"`.

Relevant constraints:

- OpenSpec specs describe observable behavior; this design may name implementation modules, adapter classes, and rollout steps.
- RoboPlan native dependencies and Python bindings are optional. `roboplan.rrt` and `roboplan.toppra` may require separate binding packages in some packaging environments, and import checks should target the exact modules DimOS uses.
- RoboPlan Python binding defaults can drift from C++ header defaults, so DimOS should set planner/IK/retiming options explicitly instead of relying on implicit upstream defaults.
- `all_blueprints.py` is generated and must not be edited manually.

## Goals / Non-Goals

**Goals:**

- Register `roboplan` as an opt-in active manipulation planning backend selected through the existing `ManipulationModuleConfig.planning_backend` surface.
- Keep the Drake/default path behavior-preserving for existing blueprints, skills, and execution flows.
- Implement a RoboPlan backend that owns one native RoboPlan scene per active backend instance and does not maintain a synchronized parallel Drake scene.
- Validate RoboPlan robot planning assets up front: URDF, SRDF where required, package paths, planning group or active joint set, joint ordering, base frame, end-effector frame, limits, and name mapping.
- Map DimOS joint state, obstacles, planning goals, and planning results into RoboPlan-native types and back into `JointPath`, `JointTrajectory`, `PlanningResult`, `IKResult`, and backend diagnostics.
- Expose capability flags and diagnostics for unsupported RoboPlan features rather than silently approximating semantics.
- Preserve trajectory execution through `ControlCoordinator.task_invoke(..., "execute", {"trajectory": ...})` and existing coordinator joint-name translation.

**Non-Goals:**

- No runtime backend switching within a running `ManipulationModule`.
- No multi-backend ensemble, A/B planner comparison, or live synchronization between Drake and RoboPlan scenes.
- No requirement that RoboPlan emulate Drake-only internals such as Drake plant access, scratch contexts, Drake optimization IK internals, or Meshcat-native preview.
- No new top-level `dimos` CLI command for RoboPlan backend selection in the first version.
- No hardware execution policy rewrite; safety remains mediated by the existing coordinator execution stack.
- No broad backend facade redesign beyond extensions needed for RoboPlan capability reporting and normalized result handling.

## DimOS Architecture

### Target package shape

Add a RoboPlan backend package next to the current Drake backend:

```text
dimos/manipulation/planning/backends/
├── base.py
├── registry.py
├── drake/
│   └── backend.py
└── roboplan/
    ├── __init__.py
    ├── backend.py          # PlanningBackend, SceneFacade, PlannerFacade, optional VisualizationFacade
    ├── conversion.py       # Joint/Pose/trajectory/result conversion helpers
    └── config.py           # RoboPlan option parsing and validation helpers, if needed
```

`registry.py` should recognize `"roboplan"` and lazily import `RoboPlanPlanningBackend` only for that selection. Unknown backend errors should include both `"drake"` and `"roboplan"` once the backend is registered. RoboPlan import failures should include the missing module name and a concise install hint, but only after `planning_backend="roboplan"` is selected.

### Backend construction and lifecycle

`RoboPlanPlanningBackend` should implement the existing `PlanningBackend` Protocol:

```text
RoboPlanPlanningBackend
  name = "roboplan"
  scene() -> RoboPlanPlanningBackend
  planner() -> RoboPlanPlanningBackend
  visualization() -> VisualizationFacade | None
  capabilities() -> BackendCapabilities
  diagnostics() -> BackendDiagnostics
  stop() -> None
```

The first implementation may use the backend object itself as `SceneFacade` and `PlannerFacade`, matching `DrakePlanningBackend`, if that keeps the adapter small. Split separate scene/planner classes later only if the file becomes difficult to maintain.

`add_robot(config: RobotModelConfig)` validates all RoboPlan-required assets and records a single active robot/group. If RoboPlan cannot support multiple robots in one native scene for the first version, reject a second robot with a clear backend initialization error rather than sharing state incorrectly. `finalize()` builds the native RoboPlan scene, active joint ordering, collision model, RRT planner, IK solver, and optional TOPPRA parameterizer once. Planning calls must reuse these objects rather than reconstructing them per plan.

### Configuration model

Use existing `RobotModelConfig` fields where they already express the required data:

- `model_path`: URDF or xacro-derived URDF path consumed by RoboPlan.
- `package_paths`: package URI resolution for meshes.
- `joint_names`: DimOS/URDF controlled joint order.
- `joint_name_mapping`: coordinator name to URDF name mapping for state ingestion and execution output.
- `base_link`, `end_effector_link`, `base_pose`, explicit joint/velocity/acceleration limits.

Put RoboPlan-specific settings in `ManipulationModuleConfig.planning_backend_options`, with per-robot overrides keyed by robot name when necessary:

```text
planning_backend_options = {
  "roboplan": {
    "srdf_path": "...",
    "planning_group": "xarm7",
    "active_joint_names": [...],
    "base_frame": "link_base",
    "end_effector_frame": "link7",
    "rrt": {...},
    "ik": {...},
    "toppra": {...},
    "retiming": "toppra" | "dimos" | "none",
    "scene": {
      "mesh_obstacles": false,
      "primitive_obstacles": true,
      "pointcloud_layers": false,
    },
  }
}
```

The exact shape can be flattened if implementation conventions prefer `options.get("rrt", {})`, but docs and validation should make each required field explicit. If a value duplicates `RobotModelConfig`, the backend should either verify equality or choose one documented source of truth; it should not silently let frame or joint-order settings diverge.

### SceneFacade behavior

RoboPlan scene behavior should cover the existing `SceneFacade` methods that `ManipulationModule`, pick-and-place code, TF publishing, preview, and validation call:

- `on_joint_state()` maps coordinator joint names to URDF/RoboPlan active joint order and stores the latest `JointState` plus optional velocities.
- `get_current_joint_state()`, `get_current_velocities()`, `wait_for_state()`, and `is_state_stale()` preserve current state-monitor semantics without requiring Drake `WorldMonitor`.
- `get_joint_limits()` returns model/config limits in DimOS joint order and fails fast if limits are incomplete.
- `is_state_valid()` and `is_path_valid()` use RoboPlan collision checking along the active joint order. Path interpolation resolution should be explicit and consistent with DimOS `step_size`.
- `get_ee_pose()`, `get_link_pose()`, and `get_jacobian()` use RoboPlan-native FK/Jacobian APIs where available. If the installed RoboPlan binding lacks a query, return `None` and record a diagnostic instead of fabricating Drake-style output.
- Perception cache methods (`on_detections`, `on_objects`, `refresh_obstacles`, `clear_perception_obstacles`, `get_perception_status`, `get_cached_objects`, `list_cached_detections`, `list_added_obstacles`) should either reuse the existing backend-agnostic obstacle cache pattern or provide equivalent behavior so `PickAndPlaceModule` does not need backend-specific branches.

Obstacle projection should be capability-aware:

- Project `ObstacleType.BOX` to RoboPlan primitive collision geometry when supported.
- Project spheres/cylinders only if RoboPlan exposes matching primitives; otherwise report `unsupported` or an explicitly documented approximation.
- Treat meshes and pointcloud layers as opt-in features gated by runtime capability probing. If unavailable, keep the DimOS obstacle cache and return diagnostics that the object was not applied to the native collision scene.
- `update_obstacle_pose()` should report whether the update was applied live, required remove/re-add, was approximated, or was unsupported. Do not claim collision geometry moved if only a visualization/cache pose changed.

### PlannerFacade behavior

`plan_to_joints()` should:

1. Read the latest current joint state.
2. Map start and goal into RoboPlan active joint order.
3. Construct `RRTOptions` explicitly from backend options and defaults owned by DimOS.
4. Call RoboPlan RRT planning.
5. Shortcut/smooth only when configured and supported.
6. Validate and normalize the returned path into DimOS joint order.
7. Build `PlannedMotion(path, trajectory, planning_result)`.

`plan_to_pose()` should prefer RoboPlan-native pose planning or IK plus joint planning based on what RoboPlan exposes. If pose planning is implemented as IK followed by RRT, return `PlannedMotion` with the `IKResult` attached, matching the Drake wrapper behavior. Pose conversion must centralize frame IDs and quaternion order so frame mistakes are not copied across call sites.

Trajectory timing has two acceptable modes:

- `retiming="toppra"`: use RoboPlan TOPPRA when installed and configured, with explicit grid/discretization options and limits. Normalize resulting time, positions, velocities, and accelerations into `JointTrajectory`.
- `retiming="dimos"`: ignore backend timing and use the existing `JointTrajectoryGenerator`, matching current Drake behavior.

The default should be documented. If TOPPRA is unavailable but selected, initialization should fail clearly; if `retiming="dimos"`, missing TOPPRA should not matter.

### Capabilities, diagnostics, and legacy access

`BackendCapabilities` for RoboPlan should be populated by runtime probing of imported modules and constructed native objects:

- `joint_planning=True` only when `roboplan.rrt` is importable and an RRT object can be constructed.
- `pose_planning` and `inverse_kinematics` depend on available RoboPlan IK/pose APIs.
- `forward_kinematics`, `jacobian`, `distance_query`, `primitive_obstacles`, `mesh_obstacles`, `pointcloud_layers`, and `attached_objects` should reflect actual bound methods/features, not desired parity.
- `visualization` and `path_preview` should be false unless a real backend-neutral or native visualization path is implemented.
- `drake_native_access=False`; `world`, `world_monitor`, `legacy_planner`, and `legacy_kinematics` return `None` for RoboPlan.

Diagnostics should be RPC-friendly through `BackendDiagnostics.as_dict()`. Use diagnostics for missing optional features, unsupported geometry, import failures, invalid robot config, joint-order mismatch, no current state, failed collision validation, no solution, timeout, and retiming failures.

### Streams, transports, DimOS Specs, skills, MCP, and CLI

No new stream types or transports are required. Existing `joint_state: In[JointState]` remains the state input, and existing perception/object streams in subclasses continue to feed the scene facade. No new DimOS RPC `Spec` Protocol is required for RoboPlan itself; the change is internal to the manipulation module and backend adapter Protocols.

Existing `@rpc` and `@skill` methods on `ManipulationModule` and `PickAndPlaceModule` should keep their names, docstrings, and result shape where possible. Skills should surface RoboPlan planning failures through the existing `SkillResult`/error paths with clearer backend messages, not through new RoboPlan-specific skill names.

No new top-level CLI entry point is expected. Backend selection should happen through blueprint/module config and be documented in manipulation planning docs. If new runnable demo blueprints are exported, regenerate the generated registry with:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

## Decisions

### Decision 1: Add RoboPlan at the active backend seam

RoboPlan will be implemented as `RoboPlanPlanningBackend` under `dimos/manipulation/planning/backends/roboplan/` and registered by `create_planning_backend("roboplan", ...)`.

Alternatives considered:

- Add RoboPlan behind `DrakePlanningBackend`. Rejected because it would preserve Drake ownership assumptions and obscure native RoboPlan scene semantics.
- Add a separate RoboPlan manipulation module. Rejected because it would duplicate existing skills, trajectory storage, state handling, and coordinator execution.
- Build a canonical scene synchronized to both Drake and RoboPlan. Rejected because the single-active-backend model intentionally avoids continuous multi-scene sync.

### Decision 2: Keep RoboPlan optional and lazy-imported

RoboPlan imports belong inside the RoboPlan backend package or registry branch. Default Drake imports, non-manipulation installs, and manipulation installs that do not select RoboPlan should not fail because RoboPlan native bindings are absent.

This also lets error messages mention exact missing modules such as `roboplan.core`, `roboplan.rrt`, or `roboplan.toppra` only when relevant.

### Decision 3: Set upstream planner options explicitly

DimOS should construct RoboPlan options with explicit values from backend config plus documented DimOS defaults. This applies especially to RRT collision checking, step sizes, timeouts, shortcutting, IK tolerances, and TOPPRA/grid/discretization options.

Rationale: RoboPlan Python binding defaults may drift from C++ defaults, and hidden upstream defaults would make planning behavior hard to reproduce.

### Decision 4: Use DimOS joint order at the public boundary

All public DimOS inputs and outputs remain in `RobotModelConfig.joint_names` order, while RoboPlan can use its required active joint order internally. The backend owns both direction maps and validates start, goal, path, limit, and trajectory lengths at the boundary.

This protects coordinator execution and existing skills from backend-specific joint ordering.

### Decision 5: Capability diagnostics over fake parity

RoboPlan should implement every stable native feature DimOS can map safely, but it must report unsupported operations instead of pretending to provide Drake parity. This is especially important for dynamic obstacle updates, mesh/pointcloud collision, attached objects, arbitrary link pose, Jacobian, distance query, and visualization.

## Safety / Simulation / Replay

Hardware execution remains routed through existing `ControlCoordinator` trajectory tasks, coordinator task names, joint-name translation, and current execution state machines. RoboPlan must not send commands directly to robot SDKs.

Default blueprints remain Drake-backed unless explicitly configured otherwise. RoboPlan should first be validated with mock or simulated manipulator flows that exercise initialization, state ingestion, joint planning, pose planning or IK+planning, obstacle projection, path validation, trajectory normalization, preview/diagnostic surfaces, and execute-through-coordinator behavior. Real hardware RoboPlan blueprints should not become defaults and should require the same supervised hardware configuration as current xArm/Piper coordinator flows.

Replay behavior is limited to manipulation stacks that select RoboPlan and feed recorded joint/object streams. Recorded joint states should populate the RoboPlan backend state cache exactly as live states do. Unsupported recorded scene features should produce diagnostics rather than silent unsafe planning.

Manual QA surface for implementation:

- `dimos run` a mock/sim manipulation blueprint with `planning_backend="drake"` and confirm current joint planning still works.
- `dimos run` a mock/sim RoboPlan blueprint or driver config and confirm backend initialization reports RoboPlan capabilities.
- Drive `plan_to_joints`, `plan_to_pose` or documented unsupported pose planning, `preview_path` or documented no-preview behavior, `execute`, and one invalid target/blocked path through the public RPC/skill surface.
- Verify missing RoboPlan modules/config fail only when RoboPlan is selected and produce actionable messages.

## Risks / Trade-offs

- **Native dependency fragility**: RoboPlan wheels/bindings may vary by platform and packaging channel. Mitigation: lazy imports, exact module import checks, optional TOPPRA behavior, and clear install diagnostics.
- **Joint-order bugs**: RoboPlan active joint order may differ from DimOS/coordinator names. Mitigation: centralize maps, validate all inputs/outputs, and test non-identity joint mappings.
- **Frame/quaternion bugs**: Pose planning depends on base/world/end-effector frame conventions and quaternion ordering. Mitigation: centralize pose conversion and document accepted frames.
- **Scene projection mismatch**: DimOS obstacle types may not map perfectly to RoboPlan collision geometry. Mitigation: capability probing and `SceneUpdateResult`/diagnostics for unsupported or approximated updates.
- **Timing inconsistency**: RoboPlan TOPPRA and DimOS `JointTrajectoryGenerator` may produce different timing. Mitigation: explicit `retiming` mode and explicit TOPPRA grid/discretization settings when TOPPRA is selected.
- **Facade pressure**: RoboPlan may expose useful APIs not currently represented by `SceneFacade`/`PlannerFacade`. Mitigation: add narrowly scoped facade fields only when required by observable behavior; keep RoboPlan-specific details behind backend options and diagnostics.
- **Visualization parity**: RoboPlan may not provide Meshcat-equivalent preview. Mitigation: preserve existing no-preview-safe RPC behavior and add backend-neutral summaries before claiming visualization support.

## Migration / Rollout

1. Add `dimos/manipulation/planning/backends/roboplan/` with lazy import helpers, configuration validation, conversion utilities, and `RoboPlanPlanningBackend`.
2. Register `"roboplan"` in `create_planning_backend()` while keeping `"drake"` as the default and behavior-preserving path.
3. Add RoboPlan optional dependency instructions to the manipulation dependency/docs surface. If project packaging adds RoboPlan to an extra, guard platform-specific bindings carefully and verify `import roboplan.core, roboplan.rrt` plus optional `roboplan.toppra`.
4. Implement robot asset validation and one-time native scene/planner construction.
5. Implement joint-state ingestion, joint order mapping, joint-limit lookup, state/path collision validation, and joint planning result normalization.
6. Implement pose planning or IK+joint planning, with clear unsupported diagnostics for unavailable native APIs.
7. Implement obstacle projection for supported primitives first, then mesh/pointcloud/attached-object support only where runtime probing confirms usable RoboPlan APIs.
8. Add or update docs for backend selection, RoboPlan config, dependency troubleshooting, capability diagnostics, and safe hardware rollout.
9. Add mock/sim demonstration coverage if appropriate. If new blueprint exports are added or renamed, regenerate `dimos/robot/all_blueprints.py` using `pytest dimos/robot/test_all_blueprints_generation.py` rather than editing it manually.

Rollback is low risk if the default remains Drake: remove RoboPlan-specific config/blueprints and the registry branch, and existing Drake-backed manipulation stacks continue through the same active backend path.

## Open Questions

- Which RoboPlan distribution channel should DimOS document as primary for supported platforms: PyPI/git install, conda-forge/Pixi packages, or a project-specific optional extra?
- Should the default RoboPlan retiming mode be `"toppra"` when available or `"dimos"` for consistency with current controller behavior?
- Which RoboPlan versions expose stable FK, Jacobian, distance, mesh, pointcloud, and attached-object APIs, and should DimOS gate support by version or runtime feature probing only?
- Should RoboPlan-specific robot fields eventually move into typed `RobotModelConfig` fields once the backend stabilizes, or remain in `planning_backend_options` to avoid broad config churn?
- Is first-version multi-robot support required for RoboPlan, or can the backend explicitly support one active robot/group per `ManipulationModule` until a concrete multi-arm RoboPlan use case exists?
