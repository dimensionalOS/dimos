## Why

`GeneratedPlan` currently stores only discrete joint waypoints, so preview and execution independently project and time the same plan when they consume it. Planning should instead finish in one operation with a single synchronized trajectory that both consumers use unchanged.

## What Changes

- **BREAKING** Collapse `PlanningResult` into `GeneratedPlan` as the sole planner result and generated-plan model.
- **BREAKING** Rename the discrete `path` field to `waypoints` and remove the unused optional planner `timestamps` field.
- Add an optional global `JointTrajectory` to `GeneratedPlan`; planner backends return it unpopulated and the manipulation module populates it before a successful plan is cached or exposed.
- Parameterize all planned joints together on one relative clock using their globally qualified names and configured limits.
- Make preview and execution consume the stored trajectory instead of lazily projecting or parameterizing waypoints.
- Split the stored global trajectory into robot/task-specific commands only at the execution boundary; joints absent from the plan are not commanded.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `group-aware-ik-rrt`: Planner backends return `GeneratedPlan` waypoints directly, without planner-owned timing.
- `manipulation-module-group-api`: A successful public planning operation materializes one synchronized trajectory before exposing or caching its generated plan, and preview/execution reuse that trajectory.

## Impact

- Affects planning result models and `PlannerSpec`, all planner implementations, manipulation-module planning/preview/execution flow, world-monitor visualization protocols, Drake and Viser preview backends, coordinator trajectory splitting, tests, and manipulation planning documentation.
- Removes the public `PlanningResult` type and renames `GeneratedPlan.path`, requiring downstream callers and tests to migrate.
- Reuses the existing `JointTrajectory` and `JointTrajectoryGenerator`; no new parameterization framework or dependency is introduced.
- Multi-task coordinator submission remains non-atomic, and concurrent ownership semantics for joints omitted from a plan remain outside this change.
