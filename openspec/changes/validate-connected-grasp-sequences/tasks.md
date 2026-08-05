## 1. Align RoboPlan Explicit-Start Planning

- [x] 1.1 Update robot-scoped and selected-group RoboPlan planning to pass any valid normalized explicit start to native RRT while retaining live unselected scene state.
- [x] 1.2 Add backend contract tests proving hypothetical starts are honored, paths begin at the requested start, and authoritative live robot state is not mutated.
- [x] 1.3 Extend multi-robot/auxiliary-joint coverage to prove hypothetical selected-group planning preserves other live scene state.

## 2. Add Connected Dry-Run Planning

- [x] 2.1 Add a side-effect-free manipulation helper that checks an ordered pose sequence from current state or an explicit selected-joint start and returns the first failed index plus the final endpoint.
- [x] 2.2 Chain each successful path endpoint into both the next IK seed and the next planner start, using collision-aware IK and the existing planning timeout.
- [x] 2.3 Log stage-specific IK and planner failure details without storing a generated execution plan or changing manipulation state.
- [x] 2.4 Add unit tests for complete success, IK failure, path failure, malformed/empty planner paths, explicit sequence starts, endpoint chaining, and no state or plan mutation.

## 3. Integrate Shared Preparation and Candidate Validation

- [x] 3.1 Factor the existing low-height safety-lift target calculation from its physical execution so it can be dry-run planned.
- [x] 3.2 Plan a required safety lift once in `PREPARE`, abort with `PLANNING_FAILED` on failure, and pass its endpoint as the shared start for all candidates.
- [x] 3.3 Replace independent pre-grasp/grasp/retreat IK screening with connected sequence checks in ranked candidate order.
- [x] 3.4 Preserve `pre_grasp_infeasible`, `grasp_infeasible`, and `retreat_infeasible` counters based on the first failed segment while keeping backend details diagnostic-only.
- [x] 3.5 Verify candidate screening issues no motion or gripper commands and that physical execution still replans every segment from fresh measured state.
- [x] 3.6 Add pick-pipeline tests for no-lift and shared-lift starts, lift failure, each failed candidate segment, lower-ranked fallback, exhausted candidates, and execution-time replan failure.

## 4. Validate MVP Scene and Integration Boundaries

- [x] 4.1 Add compact open-scene and parameterized-blocker coverage for successful sequences and blocked pre-grasp, grasp, and retreat paths without creating separate scene fixtures per case.
- [x] 4.2 Verify connected validation keeps the target suppressed, retains non-target obstacles, and allows later segments to observe live scene updates.
- [x] 4.3 Document and test the MVP boundary that retreat excludes attached-object geometry and uses the gripper state currently represented in the planning scene.
- [x] 4.4 Expose connected dry-run segment paths without storing an execution plan or changing manipulation state.
- [x] 4.5 Run focused manipulation tests, RoboPlan tests, static typing, formatting, and lint checks.
