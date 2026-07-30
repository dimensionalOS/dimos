## 1. Configuration and Adapter Boundary

- [x] 1.1 Pin `roboplan==0.5.1` in manipulation and lint dependencies and regenerate `uv.lock`.
- [ ] 1.2 Add a focused RoboPlan 0.5.1 binding contract test covering TOPP-RA construction, fitting-mode names, options, native trajectory fields, and missing-limit behavior.
- [ ] 1.3 Add typed startup configuration for `simple_trapezoid` and `roboplan_toppra`, including validated common scales/output period and backend-specific fitting controls.
- [ ] 1.4 Add the internal trajectory-parametrizer adapter Protocol and typed request/failure boundary without introducing a separate public generated-trajectory lifecycle.
- [ ] 1.5 Extend planning factory validation so exactly one parametrizer is constructed at startup and `roboplan_toppra` with a non-RoboPlan world fails before planning.

## 2. Parametrization Backends

- [ ] 2.1 Wrap the existing `JointTrajectoryGenerator` as the `simple_trapezoid` adapter while preserving current limit resolution, waypoint, and timing behavior.
- [ ] 2.2 Implement the RoboPlan TOPP-RA adapter using the finalized `RoboPlanWorld` model, selected-group lookup, and exact global-to-native joint mapping.
- [ ] 2.3 Validate that every selected RoboPlan joint has finite positive URDF-backed velocity and acceleration limits, with no fallback to generic DimOS motion-limit fields.
- [ ] 2.4 Map configured TOPP-RA fitting mode, output period, velocity/acceleration reduction scales, and adaptive/blend options into the pinned 0.5.1 API.
- [ ] 2.5 Convert RoboPlan native trajectory output back to exact selected global joint order and retain positions, velocities, timestamps, and native acceleration data long enough for limit validation.
- [ ] 2.6 Ensure a selected backend failure returns one actionable materialization error and never invokes the other backend; retain documented RoboPlan internal safe fitting-mode behavior.

## 3. Plan Materialization and Validation

- [ ] 3.1 Construct and retain the selected trajectory parametrizer during manipulation planning initialization.
- [ ] 3.2 Route `_materialize_generated_plan()` through the selected adapter while preserving the source `JointState` path unchanged in `GeneratedPlan`.
- [ ] 3.3 Preserve planning-epoch atomicity so parametrization or output-validation failure leaves no cached executable plan.
- [ ] 3.4 Extend canonical timed-trajectory validation for exact global joint ordering, dimensions, finite values, zero start time, strictly increasing times, positive non-noop duration, and start/goal preservation.
- [ ] 3.5 Validate returned velocity and acceleration against the backend's applicable limits with documented numerical tolerances, preferring native acceleration samples when available.
- [ ] 3.6 Verify preview and execution reuse the accepted stored trajectory without regeneration or retiming.

## 4. Automated Tests

- [ ] 4.1 Add adapter tests for valid simple and RoboPlan trajectories, multi-waypoint continuity, global/native reordering, composite planning groups, and configurable fitting modes.
- [ ] 4.2 Add startup/configuration tests for each backend, unknown backends, invalid scales/options, incompatible world selection, and startup-only backend lifetime.
- [ ] 4.3 Add RoboPlan limit tests for valid URDF velocity/acceleration limits, missing limits, non-finite or non-positive limits, reduction scales, and proof that generic DimOS defaults are not substituted.
- [ ] 4.4 Add materialization tests for backend failure, no cross-backend fallback, malformed/native output rejection, motion-limit rejection, and no plan caching after failure.
- [ ] 4.5 Update preview/execution tests to prove the exact accepted timed trajectory reaches visualization and the coordinator without regeneration.
- [ ] 4.6 Run focused test targets including `dimos/manipulation/test_generated_plan_materialization.py`, `dimos/manipulation/test_planning_factory.py`, `dimos/manipulation/test_roboplan_world.py`, `dimos/manipulation/test_plan_execution.py`, and new parametrizer tests.

## 5. Documentation

- [ ] 5.1 Update `dimos/manipulation/planning/README.md` with the path-to-trajectory lifecycle, backend configuration examples, RoboPlan fitting modes, `RoboPlanWorld` compatibility, no cross-backend fallback, and URDF limit requirements.
- [ ] 5.2 Update `docs/capabilities/manipulation/index.md` to explain that a plan is accepted only after parametrization and that preview and execution share the stored trajectory.
- [ ] 5.3 Update `docs/capabilities/manipulation/adding_a_custom_arm.md` with RoboPlan 0.5.1 URDF velocity and extended acceleration-limit requirements and missing-limit failure behavior.
- [ ] 5.4 Reconcile `CONTEXT.md` and `docs/adr/0001` through `docs/adr/0006` with the implemented names and behavior; update `AGENTS.md` only if stable extension guidance is added.

## 6. Verification and Manual QA

- [ ] 6.1 Run `OPENSPEC_TELEMETRY=0 openspec validate add-trajectory-parametrization`.
- [ ] 6.2 Run `uv lock --check` and verify RoboPlan resolves to exactly `0.5.1` on supported Python/platform markers.
- [ ] 6.3 Run `uv run mypy dimos/manipulation` and the repository's Ruff/pre-commit checks for changed Python files.
- [ ] 6.4 Run the focused tests from task 4.6 and the broader fast manipulation test suite.
- [ ] 6.5 Run `doclinks` and applicable `md-babel-py run` commands for changed documentation examples; run `bin/gen-diagrams` only if generated diagram sources changed.
- [ ] 6.6 Manually plan, preview, and execute a nontrivial multi-waypoint path in a manipulation simulation with `simple_trapezoid`, confirming compatibility behavior and identical preview/execution timing.
- [ ] 6.7 Manually repeat the simulation with `RoboPlanWorld` and `roboplan_toppra`, confirming smooth interior traversal, URDF-limit compliance, and identical preview/execution timing.
- [ ] 6.8 Manually verify actionable pre-motion failures for an incompatible world/backend combination, a missing URDF acceleration limit, and a TOPP-RA parametrization failure.
