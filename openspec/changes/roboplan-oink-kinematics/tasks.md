## 1. Configuration and Factory Wiring

- [ ] 1.1 Add `RoboPlanKinematicsConfig(backend="roboplan")` with Oink IK tuning fields and include it in `ManipulationKinematicsConfig`.
- [ ] 1.2 Update `kinematics_config_from_name("roboplan")`, `SUPPORTED_KINEMATICS`, and error messages to include RoboPlan kinematics.
- [ ] 1.3 Extend `validate_backend_combination(...)` so `kinematics_name="roboplan"` requires `world_backend="roboplan"`.
- [ ] 1.4 Update `create_kinematics(...)` to accept the existing `world` and `world_backend`, configure `RoboPlanWorld` for IK, and return it as `KinematicsSpec` for RoboPlan kinematics.
- [ ] 1.5 Update `create_planning_specs(...)` to pass the world into `create_kinematics(...)` while preserving existing non-RoboPlan kinematics behavior.

## 2. RoboPlanWorld Kinematics Implementation

- [ ] 2.1 Add role-specific IK config storage and a `configure_kinematics(...)` method on `RoboPlanWorld`.
- [ ] 2.2 Import or lazy-load `roboplan.optimal_ik` with clear actionable errors when the Oink binding is unavailable.
- [ ] 2.3 Implement `solve(...)` as a single-target convenience path or a clear unsupported path when no planning group can be identified.
- [ ] 2.4 Implement `solve_pose_targets(...)` using `PlanningGroupSelection` for pose and auxiliary groups.
- [ ] 2.5 Resolve selected groups to the RoboPlan native or generated composite group required by `Oink(scene, group_name)`.
- [ ] 2.6 Create one `FrameTask` per pose target using the target group's base/tip frame metadata and requested `PoseStamped` target.
- [ ] 2.7 Add `PositionLimit` and optional `VelocityLimit` constraints from the kinematics config.
- [ ] 2.8 Seed from the provided `JointState` when complete, otherwise fall back to the current RoboPlanWorld belief state.
- [ ] 2.9 Iterate `solveIk`, integrate the candidate, clamp/validate limits, check pose tolerances, and return selected global joints in selection order.
- [ ] 2.10 Optionally validate final collision freedom and fail clearly if the converged IK state is colliding.

## 3. Tests

- [ ] 3.1 Add config tests for parsing/defaults of `RoboPlanKinematicsConfig`.
- [ ] 3.2 Add factory tests that RoboPlan kinematics requires RoboPlan world backend and returns the same world object.
- [ ] 3.3 Add RoboPlanWorld unit tests for Oink task creation, multi-target task list construction, seed/current-state fallback, selected global joint return order, and auxiliary group retention.
- [ ] 3.4 Add failure tests for empty targets, missing pose target frame, overlapping selected groups, unsupported composite selection, missing `roboplan.optimal_ik`, non-convergence, and colliding final candidate.
- [ ] 3.5 Keep tests simulator-free with faked RoboPlan Oink bindings where possible.

## 4. Validation and Documentation

- [ ] 4.1 Run `uv run pytest dimos/manipulation/test_planning_factory.py -v`.
- [ ] 4.2 Run `uv run pytest dimos/manipulation/test_roboplan_world.py -v`.
- [ ] 4.3 Run targeted kinematics/config tests added for this change.
- [ ] 4.4 Run OpenSpec validation for `roboplan-oink-kinematics` and fix artifact issues.
- [ ] 4.5 Document any remaining Oink binding limitations discovered during implementation.
