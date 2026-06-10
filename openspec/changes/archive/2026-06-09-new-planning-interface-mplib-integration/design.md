## Context

The current manipulation stack centers on `ManipulationModule`, `WorldMonitor`, `WorldSpec`, `KinematicsSpec`, and `PlannerSpec`. `WorldMonitor` constructs a Drake-backed `WorldSpec`, adds configured robots and obstacles, finalizes the world, syncs incoming joint state, provides collision/FK/Jacobian queries, drives Meshcat preview/visualization, and exposes the underlying `world` for direct planner/IK calls. `ManipulationModule` plans by reading the current joint state, solving IK when needed, calling `_planner.plan_joint_path(world=..., robot_id=..., start=..., goal=...)`, generating a `JointTrajectory`, translating URDF joint names to coordinator names, and executing through `ControlCoordinator.task_invoke(..., "execute", {"trajectory": ...})`.

Pointclouds are not currently streamed directly into `ManipulationModule` as a raw planning collision layer. The base module only sees joint state and explicit `Obstacle` objects. `PickAndPlaceModule` subscribes to perception `Object` detections, caches them through `WorldObstacleMonitor`, and `refresh_obstacles()` converts eligible objects into planning obstacles. That conversion defaults to axis-aligned/pose-aware box obstacles from `Object.center`, `Object.size`, and `Object.pose`; if `WorldObstacleMonitor(use_mesh_obstacles=True)` is used, the object's `PointCloud2` can be converted to a temporary convex-hull OBJ mesh and added as a `MESH` obstacle, with fallback to a box if hull generation fails. Separately, `GraspingModule` uses object and scene `PointCloud2` values for grasp generation and optional grasp collision filtering, but that path does not populate the planning world.

The Drake surface currently used by manipulation is broader than simple path planning. Existing behavior depends on robot lifecycle (`add_robot`, `finalize`, `get_robot_config`, `get_joint_limits`), live state sync, scratch contexts, config/edge collision checks, minimum distance, FK, arbitrary link poses for TF publishing, Jacobian access for Jacobian IK, Drake-specific optimization IK internals, obstacle add/update/remove/list/clear, perception object obstacle refresh, Meshcat visualization URL/publishing, path preview animation, preview hiding, and direct Drake plant access in `DrakeOptimizationIK`.

MPlib is not a drop-in `WorldSpec` implementation. It owns a planner/world representation built from URDF/SRDF, a single move-group target link, explicit active link/joint ordering, optional pointcloud and attached-object collision state, and returns time-parameterized planning dictionaries. Treating MPlib, RoboPlan, or VAMP-like planners as Drake-shaped worlds would force them into the wrong abstraction. At the same time, maintaining a canonical planning scene synchronized into multiple backend scenes is too complex and slow for the intended use.

## Goals / Non-Goals

**Goals:**

- Introduce a single-active-planning-backend architecture where exactly one executable scene/planner backend is selected at module startup or blueprint construction.
- Provide backend-agnostic scene and planner facades so `ManipulationModule`, pick-and-place code, skills, and blueprints do not depend directly on Drake internals.
- Preserve existing user-facing manipulation skills/RPCs and existing default blueprints unless a backend is explicitly selected.
- Support MPlib as an optional but complete active manipulation backend across the native MPlib surface DimOS can represent: joint planning, pose planning, primitive and pointcloud collision scene projection, attached-object handling where exposed, result timing, and capability diagnostics.
- Cover all current Drake use cases through the new interface or explicit backend capability/diagnostic paths.
- Keep trajectory execution routed through existing `ControlCoordinator` trajectory tasks.
- Avoid per-plan construction of expensive world/planner objects.
- Avoid continuous synchronization across multiple simultaneously active planning scenes.

**Non-Goals:**

- No runtime backend switching in the first version; backend selection is fixed for a `ManipulationModule` instance until restart.
- No multi-backend planner ensemble or live A/B planning against the same changing scene.
- No guarantee that every backend supports every Drake query exactly.
- No hardware execution path changes beyond preserving existing trajectory execution behavior.
- No immediate replacement of all `WorldSpec`, `KinematicsSpec`, and `PlannerSpec` internals; compatibility shims are acceptable during migration.
- No promise that MPlib will emulate Drake-only internals such as Drake optimization IK, scratch contexts, Drake-specific distance diagnostics, or Meshcat-native preview when MPlib has no equivalent API.

## DimOS Architecture

### Target package shape

Add a planning backend layer under `dimos/manipulation/planning/backends/`:

```text
dimos/manipulation/planning/backends/
├── base.py          # PlanningBackend, SceneFacade, PlannerFacade, capabilities, diagnostics
├── registry.py      # create_planning_backend(name, config)
├── drake/
│   └── backend.py   # wraps current WorldMonitor/DrakeWorld/RRT/IK behavior
└── mplib/
    └── backend.py   # owns mplib.Planner and MPlib scene/planner state
```

The long-term direction is that `ManipulationModule` depends on an `ActivePlanningBackend` rather than raw `WorldMonitor.world`. The rollout should start with a Drake backend that wraps the current `WorldMonitor`/`DrakeWorld` implementation so existing behavior remains available while the API boundary is introduced.

### Core adapter Protocols

Introduce backend Protocols separate from DimOS RPC `Spec` Protocols:

```text
PlanningBackend
  start(robots, options) -> None
  stop() -> None
  scene() -> SceneFacade
  planner() -> PlannerFacade
  capabilities() -> BackendCapabilities
  diagnostics() -> BackendDiagnostics
```

```text
SceneFacade
  add_robot(config) -> WorldRobotID
  finalize() -> None
  get_robot_ids() -> list[WorldRobotID]
  get_robot_config(robot_id) -> RobotModelConfig
  get_joint_limits(robot_id) -> lower, upper

  sync_joint_state(robot_id, joint_state) -> None
  get_current_joint_state(robot_id) -> JointState | None
  get_current_velocities(robot_id) -> JointState | None
  wait_for_state(robot_id, timeout) -> bool
  is_state_stale(robot_id, max_age) -> bool

  add_obstacle(obstacle) -> str
  remove_obstacle(obstacle_id) -> bool
  update_obstacle_pose(obstacle_id, pose) -> SceneUpdateResult
  clear_obstacles() -> None
  get_obstacles() -> list[Obstacle]
  update_pointcloud_layer(layer_id, frame_id, points, resolution) -> SceneUpdateResult
  clear_pointcloud_layer(layer_id) -> SceneUpdateResult
  attach_object(robot_id, object_id, link_name, geometry, pose) -> SceneUpdateResult
  detach_object(robot_id, object_id) -> SceneUpdateResult

  check_config_collision_free(robot_id, joint_state) -> bool
  check_edge_collision_free(robot_id, start, end, step_size) -> bool
  is_path_valid(robot_id, path, step_size) -> bool
  get_min_distance(robot_id, joint_state | None = None) -> float | None

  get_ee_pose(robot_id, joint_state | None = None) -> PoseStamped
  get_link_pose(robot_id, link_name, joint_state | None = None) -> PoseStamped | None
  get_jacobian(robot_id, joint_state) -> NDArray | None
```

```text
PlannerFacade
  plan_joint_path(robot_id, start, goal, timeout, options) -> PlanningResult
  plan_pose(robot_id, start, target_pose, timeout, options) -> PlanningResult
  solve_ik(robot_id, target_pose, seed, options) -> IKResult
  validate_path(robot_id, path, step_size) -> bool
  normalize_result(result, robot_config) -> JointTrajectory | JointPath
```

The facades are capability-aware. A backend may report unsupported methods through `BackendCapabilities` and return explicit diagnostics instead of silently approximating unsupported behavior. The first version should preserve current call sites by adapting existing methods onto this facade rather than forcing all callers to understand capabilities immediately.

### Current Drake use-case coverage

| Current use case | New facade/API coverage | Notes |
|---|---|---|
| Robot registration/finalization | `SceneFacade.add_robot`, `finalize` | Drake backend wraps current `DrakeWorld`; MPlib backend constructs and owns `mplib.Planner` once per active robot/group after robot config validation. |
| Joint limits | `SceneFacade.get_joint_limits` | MPlib should use configured/model limits where available. |
| Live joint-state sync | `sync_joint_state`, `get_current_joint_state`, `wait_for_state`, `is_state_stale` | Preserve `RobotStateMonitor` behavior for Drake; MPlib stores latest qpos in active backend state. |
| Scratch context planning | Hidden inside backend implementation | Do not expose Drake contexts in the public facade. Drake uses scratch contexts internally; MPlib uses native planner state. |
| Collision checks | `check_config_collision_free`, `check_edge_collision_free`, `is_path_valid` | Required for existing RRT and execution validation. Backend can implement via native collision checks or report unsupported. |
| Minimum distance | `get_min_distance` | Drake supports it. MPlib should implement native distance/collision diagnostics if exposed by the installed API and otherwise report the unsupported query explicitly. |
| FK and EE pose | `get_ee_pose` | Required by current RPCs and pose helpers. MPlib should implement all FK/pose queries exposed by its model/planner and use native pose planning when standalone FK is not available. |
| Arbitrary link pose | `get_link_pose` | Required for TF extra links. MPlib should support configured `mplib_link_names` when exposed by its model/planner and report only genuinely unsupported links. |
| Jacobian IK | `get_jacobian` or backend `solve_ik` | Drake supports Jacobian through current world. MPlib should use native pose planning/IK first and expose Jacobians if the installed MPlib API supports them. |
| Drake optimization IK | Backend-specific planner/IK option | Keep Drake-specific implementation behind `DrakePlanningBackend`; do not require MPlib to emulate Drake internals. |
| Obstacle add/remove/update | `add_obstacle`, `remove_obstacle`, `update_obstacle_pose` | Drake currently has a known caveat: post-finalize pose update changes Meshcat but collision still uses original pose unless remove/re-add. MPlib should project every DimOS obstacle shape it can represent through native primitive APIs such as boxes and report `applied_live`, `requires_readd`, `approximated`, or `unsupported` for the rest. |
| Perception object obstacle refresh | Same scene obstacle APIs plus object cache in monitor | `WorldObstacleMonitor` becomes backend-agnostic over `SceneFacade`. Object cache remains outside backend. |
| Mesh/convex hull obstacles | `add_obstacle` with capability diagnostics | Drake uses convex mesh. MPlib should use all native mesh, primitive, and pointcloud collision APIs available in the installed package and diagnose only unsupported geometry. |
| Pointcloud-derived obstacles | `add_obstacle` with `BOX`/`MESH`, plus optional `update_pointcloud_layer` | Preserve current behavior: perception objects become boxes by default, or convex-hull mesh obstacles when mesh conversion is enabled. MPlib should additionally support native pointcloud collision through `update_point_cloud` when a backend config explicitly enables raw pointcloud layers and the layer frame is compatible. |
| Attached objects | `attach_object`, `detach_object`, attached-object diagnostics | MPlib should support native attached-object APIs when exposed by the installed package. If unavailable, return explicit unsupported diagnostics rather than silently dropping the attachment. |
| Path rendering data | Stored planning result/path APIs | Viser owns preview/review. Planning backends return normalized path data and do not expose native preview or viewer APIs through the backend abstraction. |
| Direct `.world`, `.plant`, `.scene_graph` access | Backend-native debug handle only | Keep as compatibility/internal escape hatch for Drake-specific code during migration, not a general API. |
| Trajectory generation/execution | `PlannerFacade` result normalization + existing `JointTrajectoryGenerator`/MPlib timing | Preserve execution through ControlCoordinator. MPlib time-parameterized outputs can be converted directly when suitable. |

### ManipulationModule integration

`ManipulationModuleConfig` should gain a backend selection field while preserving current defaults:

```text
planning_backend: str = "drake"
planner_name: str = "rrt_connect"
kinematics_name: str = "jacobian"
planning_backend_options: dict = {}
```

Existing blueprints that do not specify a backend keep Drake/RRT behavior. Blueprints that choose MPlib specify `planning_backend="mplib"` and provide MPlib robot asset fields through `RobotModelConfig`/`RobotConfig`.

The `ManipulationModule` planning flow should become:

1. Initialize active backend once in `start()`.
2. Register configured robots and finalize/prepare backend scene once.
3. Route incoming joint states and perception obstacle updates through the active backend scene facade.
4. Plan to joints through `PlannerFacade.plan_joint_path`.
5. Plan to pose through `PlannerFacade.plan_pose` when the backend supports native pose planning; otherwise solve IK then plan joints as today.
6. Normalize the backend result to `JointPath` and `JointTrajectory`.
7. Execute the translated trajectory through the existing coordinator RPC.

### Streams, transports, and module refs

No new stream types are required for the first version. Existing manipulation streams remain:

- `joint_state: In[JointState]` for active robot state sync.
- `objects: In[list[DetObject]]` in `PickAndPlaceModule` for perception obstacle caching.
- Existing coordinator task RPCs for trajectory and gripper execution.

No new MCP server behavior is required. Existing `@skill` methods should keep the same names, docstrings, and success/failure shape where possible.

### Blueprints, CLI, and generated registries

Existing manipulation blueprints should continue to run without changes. New or updated MPlib demo blueprints may be added for mock/sim xArm planning once the backend is implemented. If blueprint module exports are added or renamed, regenerate and validate the registry with:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

No new top-level `dimos` CLI command is required. Backend selection should happen through blueprint/module config rather than a new CLI surface in the first version.

## Decisions

### Decision 1: Option A final shape, staged through a thin wrapper

Use `dimos/manipulation/planning/backends/` as the final home for the active backend abstraction. Do not make `WorldMonitor` the permanent facade. During rollout, implement `DrakePlanningBackend` as a wrapper over current `WorldMonitor`/`DrakeWorld` behavior so the codebase can migrate incrementally.

Alternatives considered:

- Make `WorldMonitor` the facade. This is smaller initially but cements an overloaded object that already owns state monitors, obstacle monitors, Drake world access, visualization, and query helpers.
- Force every planner to implement `WorldSpec`. This makes non-Drake backends pretend to be Drake-shaped worlds and leaks scratch-context assumptions into MPlib/VAMP-like planners.

### Decision 2: Backend fixed at startup

A `ManipulationModule` instance selects one active backend at startup. Runtime backend switching is out of scope for the first version. This avoids the scene sync problem entirely and keeps planner ownership simple.

Alternatives considered:

- Runtime switching with lossy reseed. Useful later, but it still requires portable snapshots and edge-case handling.
- Warm multi-backend sync. Rejected because sync maintenance is the complexity this change removes.

### Decision 3: Unified API covers current Drake behavior, capabilities cover non-parity

The unified facade should include the current Drake use cases, but methods that are not universally available are capability-gated. Backends must report unsupported operations clearly instead of silently returning misleading success.

This means Drake may remain richer for Drake-specific internals, while MPlib must still be complete for its own native affordances. The MPlib backend should implement every stable native operation DimOS can map safely, and reserve unsupported diagnostics for unavailable APIs, unsupported geometry, or semantics that cannot be represented without lying.

### Decision 4: MPlib first version should be a complete native backend

MPlib v1 should support as much of MPlib's native planning and scene API as DimOS can safely represent:

- `plan_to_joints` via `mplib.Planner.plan_qpos`.
- `plan_to_pose` via `mplib.Planner.plan_pose`.
- Native IK/FK/Jacobian/distance queries when exposed by the installed MPlib planner/model API.
- Explicit MPlib robot config: URDF, SRDF, move group, user link names, user joint names, package path or package URI handling, velocity limits, acceleration limits, end-effector link, collision resolution, and pointcloud policy.
- One-time planner construction with stable cache keys based on robot assets, move group, link ordering, and joint ordering; no per-plan planner/world reconstruction.
- Joint-order mapping in both directions so MPlib can use its preferred `user_joint_names` while DimOS receives paths in `RobotModelConfig.joint_names` order.
- Primitive obstacle projection through all native methods available on the installed planner, including boxes at minimum when supported.
- Pointcloud collision layers through native `update_point_cloud` when enabled, frame-compatible, and available.
- Attached-object support through native attach/detach APIs when exposed.
- Result normalization from MPlib status/time/position/velocity/acceleration/duration into DimOS planning results and `JointTrajectory`.
- Clear dependency/config errors during backend initialization.
- Clear per-feature diagnostics for unsupported geometry, skipped pointcloud layers, missing native methods, failed plans, collisions, invalid starts/goals, and timeouts.

MPlib v1 should not stop at joint/pose planning if the installed MPlib package exposes more useful scene or query APIs. The only acceptable gaps are features that MPlib does not expose, DimOS cannot represent safely, or features that would require pretending MPlib has Drake-specific semantics.

### Decision 5: MPlib belongs in the manipulation extra

Add MPlib to the existing manipulation dependency surface instead of a separate extra for this change. Missing or unsupported MPlib environments should still produce clear backend initialization errors, not break non-MPlib default blueprints.

### Decision 6: Preserve current execution safety behavior

Do not introduce a new execution safety gate in this change. Preserve the current behavior: planned paths are generated, converted into `JointTrajectory`, translated to coordinator joint names, and submitted to the coordinator trajectory task. Validation improvements may be added through the backend facade, but the first design should not change hardware execution semantics or add a new execution policy layer.

## Safety / Simulation / Replay

Hardware execution remains mediated by `ControlCoordinator` trajectory tasks and gripper RPCs. The backend change must not bypass coordinator claims, task priorities, trajectory state machines, or joint name translation.

Default blueprints should remain Drake/RRT unless explicitly configured otherwise. MPlib should be validated first through mock or simulated xArm-style planner blueprints. Any real hardware MPlib blueprint should require the same robot IP/hardware configuration as existing xArm coordinator flows and should not be the default.

Manual QA should exercise:

- Existing Drake-backed xArm planner/coordinator flow still plans, exposes stored path data for Viser, and executes mock trajectories.
- Existing manipulation skills continue to expose the same user-facing behavior.
- MPlib backend initializes once and plans to a joint target without reconstructing per plan.
- MPlib backend plans to a pose using its native pose planning path.
- MPlib backend maps joint order correctly between MPlib and DimOS names.
- MPlib backend projects primitive obstacles through native APIs where available.
- MPlib backend applies pointcloud collision layers through native pointcloud APIs when enabled and frame-compatible.
- MPlib backend supports attached objects when the installed MPlib API exposes attach/detach behavior.
- MPlib unavailable/config-invalid cases fail with clear diagnostics.
- Obstacle add/remove/update behavior reports correct support or limitation.
- Perception obstacle refresh remains functional for Drake and reports clear MPlib limitations if not fully supported.

Replay impact is limited to manipulation stacks that use recorded joint/object streams. Replay should continue to feed the same streams into the active backend; backend-specific unsupported scene features should be surfaced in diagnostics rather than causing silent unsafe execution.

## Risks / Trade-offs

- **Internal API churn**: `WorldSpec`, `WorldMonitor.world`, and direct Drake access are used in several places. Mitigation: stage through a `DrakePlanningBackend` wrapper and preserve old call paths until ManipulationModule and tests are migrated.
- **Fake uniformity**: MPlib cannot provide every Drake query, but should provide every useful native MPlib capability. Mitigation: implement native MPlib surfaces first, then use capability-gated facade methods and diagnostics for true gaps.
- **Joint-order bugs**: MPlib requires explicit `user_joint_names` and active joint order. Mitigation: validate that MPlib joint names match `RobotModelConfig.joint_names`, and fail fast on missing/mismatched start/goal joints.
- **Pose/frame convention bugs**: MPlib uses world-frame pose by default and quaternion ordering differs across libraries. Mitigation: centralize pose conversion in the MPlib backend and document frame/quaternion conventions.
- **Scene mutation mismatch**: Drake and MPlib differ in dynamic obstacle semantics. Mitigation: `SceneUpdateResult` reports `applied_live`, `requires_readd`, `approximated`, or `unsupported`.
- **Trajectory timing differences**: MPlib can return time-parameterized paths; current DimOS also has `JointTrajectoryGenerator`. Mitigation: design a normalization path that can either preserve backend timing or fall back to existing trajectory generation consistently.
- **Dependency footprint**: Adding MPlib to the manipulation extra may affect install reliability. Mitigation: keep backend selection explicit and surface import/config errors only when MPlib is selected.
- **Visualization parity**: Drake Meshcat preview is no longer part of the backend abstraction. Mitigation: expose backend-neutral path data for Viser rendering and keep native viewer details out of planning backends.

## Migration / Rollout

1. Add backend Protocols, capability/diagnostic models, and a backend registry under `dimos/manipulation/planning/backends/`.
2. Implement `DrakePlanningBackend` as a compatibility wrapper around current `WorldMonitor`, `DrakeWorld`, current RRT planner, and current IK solvers.
3. Update `ManipulationModule` internally to obtain `scene()` and `planner()` from the active backend while keeping existing RPCs/skills and default config behavior.
4. Move `WorldObstacleMonitor` and robot state monitor usage behind `SceneFacade` without changing the public `PickAndPlaceModule` obstacle/scan methods.
5. Add MPlib fields to robot config conversion if not already present: MPlib URDF, SRDF, package path, move group, link names, joint names, and optional end-effector link override.
6. Implement `MPlibPlanningBackend` with one-time planner construction, joint planning, pose planning, native scene projection for all supported primitives/pointclouds/attached objects, joint-order mapping, native query support where exposed, result normalization, and clear unsupported diagnostics for genuine MPlib gaps.
7. Add or update mock/sim blueprints only after the backend is usable. Regenerate blueprint registry if exports change.
8. Update manipulation docs with backend selection, MPlib configuration, supported/unsupported features, and troubleshooting.
9. Keep old direct world access as a private/compatibility escape hatch during rollout; deprecate direct use in follow-up cleanup once tests cover the facade.

Rollback is straightforward if the Drake wrapper remains behavior-preserving: default blueprints can continue to select Drake/RRT, and MPlib-specific config/blueprints can be removed without changing existing manipulation execution paths.

## Open Questions

- Should MPlib timing output be authoritative for `JointTrajectory`, or should DimOS always retime with `JointTrajectoryGenerator` for consistency with current controllers?
- How much of `DrakeOptimizationIK` should be migrated behind backend-native IK versus left as a Drake-specific compatibility path?
- Should `WorldSpec` be kept as a legacy protocol name during migration, or renamed/split immediately into `SceneFacade` and backend-private Drake types?
- Are MPlib dependencies stable enough to include in the default manipulation extra across supported platforms, or should install failures be guarded by platform markers?
- Which installed MPlib versions expose stable attached-object, FK/Jacobian, and primitive obstacle APIs, and should DimOS gate those by runtime capability probing or by a minimum supported MPlib version?
