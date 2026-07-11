## Why

The manipulation visualization boundary currently lets Viser pull from both `ManipulationModule` and `WorldMonitor`, so UI code performs planning-group lookup, state normalization, FK/collision composition, preview projection, and execution freshness checks. Preview and execution also derive different representations from an untimed stored path, making the boundary difficult to reason about and easy to drift.

## What Changes

- **BREAKING** Extend `GeneratedPlan` to contain both its geometric path and one synchronized, globally named `JointTrajectory`; parameterize the path exactly once immediately after `PlannerSpec.plan` succeeds and before the plan is cached or exposed.
- Make the stored trajectory the sole motion/timing authority for preview and execution. Preserve the geometric path for inspection and diagnostics.
- **BREAKING** Replace the pull-oriented visualization API with one-shot static initialization, pushed current-state frames, raw synchronized-trajectory animation, and cancellation. Remove `show_preview`, `hide_preview`, and `animate_plan(GeneratedPlan)`.
- Include resolved planning groups in immutable visualization initialization metadata. Visualization topology does not change during a session.
- Add one concrete `ManipulationOperator` facade that composes `ManipulationModule` and `WorldMonitor` for typed target evaluation and manipulation actions. Viser receives this facade through visualization initialization instead of receiving the raw module and monitor.
- Keep target drafts and asynchronous request sequencing frontend-owned. Make target evaluation advisory and require planning to consume the original complete target request.
- Move plan freshness enforcement into `ManipulationModule.execute`, comparing current planned joints with the stored trajectory start. Remove Viser-side robot snapshots, stale-state policy, and execution safety decisions.
- Push only current joint states to visualizations at visualization rate; freshness remains an authoritative manipulation concern and does not cross the rendering API.
- Update the existing `JointTrajectoryTask` to accept a non-empty configured joint subset while retaining its full configured resource claim. It emits commands only for trajectory joints, so omitted joints are not commanded.
- Preserve the existing Viser controls and interaction contract while making preview ghosts transient: completion or cancellation hides them immediately.

## Capabilities

### New Capabilities
- `manipulation-visualization-boundary`: Static visualization sessions, pushed joint-state frames, raw synchronized-trajectory preview, cancellation, and backend isolation from manipulation internals.
- `manipulation-operator`: The concrete UI-neutral facade, canonical target requests/evaluations, compact status, and action semantics used by interactive visualization panels.
- `partial-joint-trajectory-execution`: Validation and execution of configured joint subsets through the existing `JointTrajectoryTask` while preserving full resource claims.

### Modified Capabilities
- `manipulation-module-group-api`: Generated plans become materialized path-plus-trajectory artifacts; preview and execution consume the stored trajectory and module-owned execution freshness.

## Impact

- Affects manipulation planning models and module materialization/execution, visualization protocols and scene metadata, `WorldMonitor`, Drake and Viser backends, Viser panel/backend/state code, trajectory control task validation/output, tests, and manipulation documentation.
- Removes direct `ManipulationModule` and `WorldMonitor` dependencies from Viser GUI/renderer construction.
- Replaces existing visualization method signatures and `GeneratedPlan` construction sites; this is an intentional internal breaking migration without compatibility shims.
- Does not change planner algorithms, robot/world collision ownership, coordinator batching, actual hardware execution cancellation, or the visible Viser interaction design.
