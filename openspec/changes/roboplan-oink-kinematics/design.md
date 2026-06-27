## Context

The manipulation planning factory currently supports world backends `drake` and `roboplan`, planners `rrt_connect` and `roboplan`, and kinematics backends `jacobian`, `drake_optimization`, and `pink`. RoboPlan planning is already scene-coupled: `create_planner(name="roboplan", world=..., world_backend="roboplan")` returns the existing `RoboPlanWorld` object as `PlannerSpec`.

`RoboPlanWorld` owns the RoboPlan `Scene`, live/scratch contexts, global/native joint-name maps, planning-group registry, generated composite groups, group FK/Jacobian helpers, selected-state overlays, joint limits, and collision checks. Those are also the exact inputs needed for RoboPlan-native IK.

Robokin's `RoboPlanOinkKinematics` wrapper demonstrates the relevant RoboPlan API shape: construct `Oink(scene, group_name)`, create `FrameTask` objects from `CartesianConfiguration`, add `PositionLimit` and optional `VelocityLimit` constraints, call `oink.solveIk(...)`, integrate `delta_q` through the scene, and repeat toward the goal. Local bindings expose `roboplan.optimal_ik.Oink`, `FrameTask`, `FrameTaskOptions`, `PositionLimit`, and `VelocityLimit`.

## Goals / Non-Goals

**Goals:**

- Add `backend="roboplan"` as a typed manipulation kinematics config.
- Keep `RoboPlanWorld` as the single RoboPlan integration object for world, planner, and kinematics roles.
- Implement `KinematicsSpec` on `RoboPlanWorld` using RoboPlan Oink task IK rather than a hand-written Jacobian pseudoinverse loop.
- Support multiple pose targets when the selected planning groups are non-overlapping and represented by one RoboPlan group or generated composite group.
- Preserve DimOS public global joint names at API boundaries.
- Return selected global joints in the `IKResult`, including auxiliary selected groups that retain seed/current positions.
- Fail clearly for unsupported selections, missing pose target frames, missing Oink bindings, or collision-invalid final states.

**Non-Goals:**

- Do not implement the deferred robokin backend.
- Do not split `RoboPlanWorld` into `RoboPlanPlanner` or `RoboPlanIK` adapter classes.
- Do not add a planner config unless RoboPlan planner tunables are needed separately.
- Do not promise global IK completeness or obstacle-aware IK optimization.
- Do not change existing `planner_name="roboplan"` behavior beyond any shared helper reuse needed by IK.
- Do not change planning-group semantics to allow overlapping selected joints.

## Decisions

### Decision: Use one RoboPlanWorld object for all RoboPlan roles

`RoboPlanWorld` should implement the existing world role, continue serving as `PlannerSpec` for `planner_name="roboplan"`, and also serve as `KinematicsSpec` for `kinematics.backend="roboplan"`.

Rationale: RoboPlan IK needs the same scene, contexts, group registry, native order, and collision state that `RoboPlanWorld` already owns. A separate adapter would either call private internals or require an artificial service surface that adds indirection without reducing coupling.

Alternative considered: create `RoboPlanIK(world, config)` and `RoboPlanPlanner(world)` adapters. Rejected because the user prefers the current single-class pattern, and the existing planner path already established `RoboPlanWorld` as the coupled integration object.

### Decision: Configure role-specific options after world construction

`RoboPlanWorld` remains constructed from world/backend options. `create_kinematics(config=RoboPlanKinematicsConfig, world=..., world_backend="roboplan")` should configure the existing world with Oink IK options and return it as `KinematicsSpec`.

Suggested shape:

```python
class RoboPlanKinematicsConfig(BaseConfig):
    backend: Literal["roboplan"] = "roboplan"
    max_iterations: int = 100
    dt: float = 0.05
    position_cost: float = 1.0
    orientation_cost: float = 1.0
    task_gain: float = 0.5
    lm_damping: float = 1e-6
    regularization: float = 1e-8
    velocity_limit: float | None = None
    collision_check: bool = True
```

Rationale: world options and IK solver options have different lifecycles. Passing all role-specific tuning through the world constructor would make the constructor absorb every future planner/kinematics knob. Configuring the single object after construction preserves one integration object while keeping configs role-specific.

Alternative considered: avoid a dedicated config class and rely on `kinematics_name="roboplan"` only. Rejected because Oink has real solver knobs that need typed defaults and future CLI/config override support.

### Decision: Use RoboPlan Oink, not DimOS JacobianIK math

RoboPlan IK should follow the Oink task-solver pattern: create one `FrameTask` per target, add joint constraints, call `solveIk`, integrate the delta, and iterate until tolerances are met or iteration budget is exhausted.

Rationale: Oink is RoboPlan's task-based IK surface and supports multiple tasks in one solve call. It also aligns with robokin's RoboPlan wrapper while allowing DimOS to generalize from one target frame to multiple DimOS planning groups.

Alternative considered: implement a local stacked-Jacobian damped least-squares loop with `get_group_jacobian(...)`. Rejected because it duplicates solver logic already available in RoboPlan and would likely handle task priorities, damping, and constraints less consistently than Oink.

### Decision: Model multitarget IK as a selected planning-group problem

`solve_pose_targets(...)` should build a `PlanningGroupSelection` from pose target groups plus auxiliary groups. Existing selection validation rejects overlapping selected joints. RoboPlan IK should resolve that selection to a RoboPlan group or generated composite group, then create one `FrameTask` for each pose target group.

Rationale: this preserves the current DimOS rule that selected groups cannot overlap. It naturally supports multi-robot targets and same-robot disjoint targets through existing composite group generation, while avoiding ambiguous ownership of shared joints.

Alternative considered: allow overlapping groups and solve with shared variables. Rejected for v1 because it changes planning-group semantics and would require new conflict-resolution rules for returned selected joints.

### Decision: Collision validation, not collision-aware IK, in v1

RoboPlan IK should optionally check collision freedom of the final selected candidate using existing RoboPlanWorld collision helpers. If the converged candidate is colliding, return an IK failure. It should not add obstacle-avoidance tasks or try to optimize around collisions in this slice.

Rationale: Oink task IK and path planning serve different purposes. The caller can plan to a valid IK result afterward. Adding collision avoidance inside IK increases scope and tuning complexity before basic RoboPlan-native IK is validated.

Alternative considered: integrate collision avoidance constraints into Oink immediately. Rejected as broader and less certain than the current need.

## Risks / Trade-offs

- `roboplan.optimal_ik` pybind signatures are opaque. → Mitigation: mirror the call pattern validated by robokin and cover the binding surface with faked tests plus at least one import/constructor smoke path when dependencies are available.
- Multi-target IK depends on generated composite group availability. → Mitigation: reuse existing selection-to-group validation and fail with `IKStatus.NO_SOLUTION` when a selection cannot be represented by RoboPlan.
- Oink IK may converge locally but not globally. → Mitigation: document local IK semantics and keep path planning as the follow-up for moving to the IK result.
- Collision-free final states may still be unreachable by a path planner. → Mitigation: treat IK as endpoint generation only; do not weaken planner validation.
- Returning the same object for world, planner, and kinematics increases class responsibility. → Mitigation: keep implementation helpers private and cohesive around RoboPlan scene/group state; revisit splitting only if the class becomes unmanageable.

## Migration Plan

1. Add `RoboPlanKinematicsConfig` and include it in the discriminated `ManipulationKinematicsConfig` union and legacy name lookup.
2. Extend supported kinematics and backend-combination validation with `roboplan` requiring `world_backend="roboplan"`.
3. Change `create_kinematics(...)` and `create_planning_specs(...)` so RoboPlan kinematics receives and returns the existing `RoboPlanWorld` object.
4. Add RoboPlanWorld kinematics configuration storage with default values.
5. Implement `solve(...)` as a one-target convenience wrapper over `solve_pose_targets(...)` where compatible with group semantics.
6. Implement `solve_pose_targets(...)` with Oink tasks, selected/composite group resolution, seed/current-state initialization, iterative integration, tolerance checks, optional final collision validation, and selected global joint-state return.
7. Add factory/config tests and RoboPlanWorld unit tests using fakes for `roboplan.optimal_ik`.
8. Run targeted manipulation planning tests and OpenSpec validation.

Rollback is isolated to the kinematics config/factory additions and `RoboPlanWorld` `KinematicsSpec` methods. Existing RoboPlan planner behavior should remain intact.

## Open Questions

- Should Oink `FrameTaskOptions.priority` be exposed in config immediately, or kept at binding/default behavior until a concrete need appears?
- Should velocity limits be configured as one scalar, per-selected-joint values, or initially omitted unless the binding requires them?
