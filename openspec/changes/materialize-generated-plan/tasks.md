## 1. Consolidate the Plan Model and Planner Contract

- [ ] 1.1 Replace `PlanningResult` with `GeneratedPlan`, rename `path` to `waypoints`, add `trajectory: JointTrajectory | None`, and remove planner timestamps from the planning models and exports.
- [ ] 1.2 Update `PlannerSpec` and every planner backend to return `GeneratedPlan` with canonical selected-global waypoints, planner status/metrics, and no trajectory.
- [ ] 1.3 Migrate planner and model tests to assert waypoint naming/order, failure statuses, unsupported selection behavior, and the unpopulated internal trajectory state.

## 2. Materialize the Synchronized Trajectory During Planning

- [ ] 2.1 Add manipulation-module validation and limit resolution that maps every planned global joint to configured velocity and acceleration limits in canonical waypoint order.
- [ ] 2.2 Parameterize all planned joints in one `JointTrajectoryGenerator` call and populate one global `JointTrajectory` with matching global names and a shared relative clock.
- [ ] 2.3 Make every public group and compatibility planning operation cache and report success only after trajectory materialization succeeds; clear or preserve no partial `_last_plan` on validation or parameterization failure.
- [ ] 2.4 Add tests for multi-group and multi-robot shared timing, slowest-joint segment duration, malformed waypoint names/dimensions, unresolved limits, and parameterization failure.

## 3. Consume the Stored Trajectory for Preview and Execution

- [ ] 3.1 Update manipulation preview routing, WorldMonitor, Drake, and Viser to validate and animate `GeneratedPlan.trajectory` rather than projecting or timing waypoints.
- [ ] 3.2 Preserve stored `time_from_start` values during normal preview and implement explicit duration as non-mutating playback scaling; test default timing, scaling, cancellation, and plan immutability.
- [ ] 3.3 Replace execution-time path projection and parameterization with partitioning of the stored global trajectory by robot/task while preserving relative timestamps and planned-joint order.
- [ ] 3.4 Add tests proving preview and execution reject failed or unpopulated plans and never invoke trajectory generation after a plan has been cached.

## 4. Preserve Planned-Joint Subsets at the Control Boundary

- [ ] 4.1 Validate incoming trajectory joint names as a unique configured subset and validate every point's position/velocity dimensions before a trajectory task accepts execution.
- [ ] 4.2 Make trajectory-task command output use the active trajectory's joint names and values so omitted configured joints receive no command from the plan.
- [ ] 4.3 Add coordinator and trajectory-task tests for valid subsets, unknown and duplicate joints, malformed points, name translation, and omitted-joint command behavior.

## 5. Complete Migration and Validation

- [ ] 5.1 Remove obsolete `PlanningResult`, `.path`, planner timestamp, lazy plan-projection, and execution-time parameterization references across production code and tests.
- [ ] 5.2 Update manipulation planning documentation and glossary language to distinguish planner waypoints from the synchronized generated-plan trajectory.
- [ ] 5.3 Run focused planner, manipulation-module, world-monitor, Drake, Viser, coordinator, and trajectory-task pytest suites with no relevant skips.
- [ ] 5.4 Run targeted mypy and Ruff over all changed production files, `git diff --check`, and `openspec validate materialize-generated-plan --strict`.
