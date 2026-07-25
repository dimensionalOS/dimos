## 1. Execution Interface and Coordinator Adapter

- [x] 1.1 Add typed dispatch and cancellation outcomes, structured result models,
  immutable execution targets, execution policy, and the coordinator execution
  adapter Protocol.
- [x] 1.2 Implement the production coordinator adapter over `RPCClient.task_invoke`
  with explicit handling for `True`, `False`, `None`, and RPC exceptions.
- [x] 1.3 Add focused adapter tests for accepted, rejected, already-stopped,
  unknown, and exception outcomes.
- [x] 1.4 Validate and precompute model-to-coordinator joint mappings when execution
  targets are constructed, including identity fallback and ambiguous-mapping
  rejection.

## 2. Plan Execution Manager

- [x] 2.1 Implement successful-plan and structural validation for non-empty paths,
  timed trajectories, finite values, canonical global joint names, and complete
  robot coverage.
- [x] 2.2 Implement per-robot trajectory splitting and name alignment while
  preserving positions, velocities, point times, trajectory timestamp, and shared
  multi-robot timing.
- [x] 2.3 Implement live plan-start freshness validation with the existing `1e-6`
  tolerance and safe rejection for missing, duplicate, reordered, stale,
  non-finite, or mismatched joints.
- [x] 2.4 Implement replacement execution by cancelling every previously tracked
  task before dispatch and blocking replacement when task safety is unknown.
- [x] 2.5 Implement complete-plan dispatch, pre-RPC possibly-active tracking,
  partial-dispatch compensation, and unresolved-task fault results.
- [x] 2.6 Implement fail-fast concurrent execution, cancellation priority,
  generation checks, and idempotent manager cancellation.
- [x] 2.7 Add manager-interface tests for successful single- and multi-robot
  dispatch, invalid and stale plans, partial-plan rejection, replacement,
  rollback, uncertain cancellation, and execute/cancel races.

## 3. Manipulation Module Integration

- [x] 3.1 Construct execution targets, the coordinator adapter, and
  `PlanExecutionManager` from existing robot configuration and current-state
  access during manipulation initialization.
- [x] 3.2 Delegate `execute()` and `execute_plan()` to the manager while retaining
  existing signatures, cached-plan selection, boolean returns, and public
  `ManipulationState` projection.
- [x] 3.3 Reject the legacy `robot_name` filter when the stored or explicit plan
  contains multiple robots; retain compatibility for a matching single-robot plan.
- [x] 3.4 Keep planning cancellation in `ManipulationModule`, delegate planned
  execution cancellation to the manager, and preserve legacy idle-cancel boolean
  behavior.
- [x] 3.5 Cancel manager-tracked tasks before closing the module-owned coordinator
  client during shutdown.
- [x] 3.6 Remove superseded module-local execution preparation, translation,
  dispatch, rollback, and transaction fields without moving direct gripper
  actuation.
- [x] 3.7 Update inconsistent `GeneratedPlan` fixtures to set successful planning
  status explicitly and retain module-level integration tests for delegation and
  compatibility behavior.

## 4. Documentation

- [x] 4.1 Add `docs/capabilities/manipulation/plan_execution.md` covering
  whole-plan atomicity, successful and fresh plan requirements, execution
  acceptance, replacement, cancellation, failure uncertainty, and the
  single-writer convention.
- [x] 4.2 Update `docs/capabilities/manipulation/planning_groups.md` to link the
  execution guide and remove guidance that permits filtering a multi-robot plan at
  execution time.

## 5. Verification

- [x] 5.1 Run `openspec validate extract-plan-execution-manager`.
- [x] 5.2 Run focused execution and manipulation tests with
  `uv run pytest dimos/manipulation/test_execution_manager.py dimos/manipulation/test_plan_execution_reservation.py dimos/manipulation/test_manipulation_unit.py dimos/manipulation/test_manipulation_module.py -v`.
- [x] 5.3 Run the planning-group end-to-end coordinator test with
  `uv run pytest dimos/e2e_tests/test_manipulation_planning_groups.py -v`.
- [x] 5.4 Run type and style checks for touched Python files with the repository's
  mypy and Ruff commands.
- [x] 5.5 Run `bin/doclinks` and `bin/run-doc-codeblocks` for
  `docs/capabilities/manipulation/plan_execution.md` and
  `docs/capabilities/manipulation/planning_groups.md`.
- [x] 5.6 Manually exercise the manipulation client against a mock or simulated
  coordinator: dispatch a valid plan, replace it, cancel it, and confirm that a
  multi-robot plan cannot be filtered to one robot.
- [x] 5.7 Confirm `dimos/robot/all_blueprints.py` remains unchanged; if blueprint
  inputs changed unexpectedly, run
  `pytest dimos/robot/test_all_blueprints_generation.py`.
