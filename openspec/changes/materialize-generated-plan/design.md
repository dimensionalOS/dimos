## Context

`PlannerSpec` currently returns `PlanningResult`, after which `ManipulationModule` copies most fields into `GeneratedPlan`, drops optional planner timestamps, and caches only selected-global `JointState` waypoints. Preview backends project those waypoints when rendering, while execution separately projects and parameterizes them into robot-local trajectories. Consequently, one stored plan has no timing authority and can be interpreted differently by preview and execution.

The current planners produce geometric waypoints only. The existing `JointTrajectoryGenerator` can accept per-joint velocity and acceleration vectors and generate trapezoidal profiles, so it can serve as the provisional global parameterizer until a formal parameterization procedure replaces it.

## Goals / Non-Goals

**Goals:**

- Make one public planning operation produce a complete `GeneratedPlan` containing geometric waypoints and one synchronized trajectory.
- Use `GeneratedPlan` as the sole planner result model and preserve planner status and metrics without a copy-only intermediate type.
- Give preview and execution the same authoritative timing and planned-joint positions.
- Command only joints explicitly present in the plan.
- Preserve a replaceable boundary for future formal trajectory parameterization.

**Non-Goals:**

- Introduce TOPPRA or another formal parameterization framework.
- Guarantee atomic multi-task coordinator submission.
- Define concurrent ownership or locking semantics for joints omitted from a plan.
- Change collision-checking algorithms, planning-group selection, or planner search behavior.
- Make planner backends responsible for timing.

## Decisions

### Collapse planner results into `GeneratedPlan`

Remove `PlanningResult`. `PlannerSpec` implementations return `GeneratedPlan` directly with planner status, metrics, selected planning-group IDs, geometric `waypoints`, and `trajectory=None`. The field formerly named `path` becomes `waypoints`; the unused planner `timestamps` field is removed.

An incomplete or failed instance is an internal planner result. `ManipulationModule` MUST cache or expose a successful plan only after its trajectory has been populated. This reuses one model without making preview or execution support partially generated plans.

Alternative considered: retain separate geometric and executable result types. This gives stronger static state distinctions but preserves nearly duplicate models and contradicts the desired single-result workflow.

### Parameterize once within the manipulation planning operation

After a planner returns successful waypoints, `ManipulationModule` validates the selected global joint names and waypoint dimensions, resolves each planned joint's configured velocity and acceleration limits, and calls one `JointTrajectoryGenerator` across the entire planned-joint vector. The resulting `JointTrajectory` uses globally qualified joint names and one relative clock.

All planned joints participate in each segment duration calculation, so the slowest required joint determines that segment's duration. This preserves synchronized waypoint progress across planning groups and robots. Parameterization failure leaves `_last_plan` empty and makes the public planning operation fail.

Alternative considered: generate one trajectory per robot and synchronize afterward. This creates competing clocks and requires resampling or stretching, so it is rejected.

### Store only planned joints in the trajectory

The synchronized trajectory contains exactly the joints represented by the selected planning groups, in the same canonical global order as the waypoints. Joints outside the selection are omitted and receive no command from this plan. Preview may combine planned positions with renderer state for omitted joints, but omitted state is not part of the trajectory contract.

Alternative considered: expand each affected robot to its complete controllable state. This would command joints the caller did not select and would retain the projection policy being removed.

### Make the stored trajectory authoritative

Preview and execution MUST reject plans whose status is unsuccessful, whose trajectory is absent, or whose trajectory is inconsistent with the waypoint joint set. Neither consumer may generate new timing from waypoints.

Preview backends animate `trajectory.points` according to `time_from_start`. An explicit preview-duration override acts only as playback scaling and does not mutate or replace stored timing. Renderer frame sampling remains a visualization concern.

Execution partitions the global trajectory by robot/task while preserving every point's relative timestamp and planned-joint ordering. Robot-local and coordinator joint-name translation occurs only at dispatch. The complete multi-robot plan remains the unit of execution even though coordinator task submissions are currently sequential.

### Support selected-joint commands at the control boundary

Trajectory tasks MUST validate that incoming joint names are unique members of the task's configured joint set and that every trajectory point has matching dimensions. Task output MUST use the active trajectory's joint names rather than expanding to all configured task joints. Existing per-joint coordinator and hardware routing then leaves omitted joints uncommanded.

Task claim/locking behavior and concurrent commands to omitted joints are unchanged and explicitly deferred.

## Risks / Trade-offs

- **[Breaking result-model migration]** Planner implementations and callers currently construct or inspect `PlanningResult` and `GeneratedPlan.path`. → Update all implementations, tests, exports, and documentation together; do not retain aliases that prolong ambiguous terminology.
- **[Mutable transient plan]** `GeneratedPlan.trajectory` is temporarily `None` after planner return. → Keep this state inside the planning operation and validate before caching, preview, or execution.
- **[Limit resolution errors]** A selected global joint may not resolve to configured motion limits. → Fail parameterization clearly and do not publish a partial plan.
- **[Preview compatibility]** Existing preview APIs accept arbitrary display duration and currently animate waypoint indices. → Treat duration as playback scaling over stored trajectory time and test that the plan is unchanged.
- **[Partial control assumptions]** Trajectory tasks currently emit configured task joint names even for arbitrary incoming trajectories. → Validate subset commands and emit only active trajectory names before relying on omitted-joint behavior.
- **[Future parameterizer migration]** The provisional trapezoidal generator is not a formal globally optimized parameterizer. → Keep parameterization behind the manipulation-module generation step so a later implementation can replace it without changing planner, preview, execution, or `GeneratedPlan` contracts.
