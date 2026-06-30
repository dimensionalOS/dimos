## 1. RoboPlan Group Registration and SRDF Policy

- [x] 1.1 Add `PlanningGroupRegistry`, robot-name lookup, and native joint-order metadata fields to `RoboPlanWorld` without introducing a new custom group dataclass.
- [x] 1.2 Register `RobotModelConfig.planning_groups` during `add_robot(...)` and map each group to its owning robot ID.
- [x] 1.3 Update SRDF path selection so `RobotModelConfig.srdf_path` is passed directly when provided.
- [x] 1.4 Update generated SRDF behavior to allow exactly one configured planning group and use that group's name and joints.
- [x] 1.5 Reject no-SRDF multi-group configurations with a clear `RobotModelConfig.srdf_path` error.
- [x] 1.6 Validate each configured group against `scene.getJointGroupInfo(group.group_name)` and store RoboPlan native joint order.

## 2. Joint-Order Conversion Helpers

- [x] 2.1 Add helper to resolve a planning group ID to its public `PlanningGroup`, owning robot ID, robot config, and native joint order.
- [x] 2.2 Add helper to project full robot-local `q` into RoboPlan native group order by local joint name.
- [x] 2.3 Add helper to convert public selected-group `JointState` values into RoboPlan native group `q` values.
- [x] 2.4 Add helper to convert RoboPlan native group path waypoints back into public global selected-group `JointState` order.
- [x] 2.5 Add helper to overlay group positions onto full robot-local `q` only if a current group-oriented query requires preserving non-group joints.

## 3. Group World Queries

- [x] 3.1 Implement `get_group_ee_pose(ctx, group_id)` using the group's target frame and native order projection.
- [x] 3.2 Implement `get_group_jacobian(ctx, group_id)` and reorder columns into public group local joint order.
- [x] 3.3 Remove RoboPlan legacy FK/Jacobian wrapper expectations from tests and adapter design; RoboPlan should expose only the group-oriented world query methods.

## 4. Native Selected-Group Planning

- [x] 4.1 Implement `plan_selected_joint_path(...)` on `RoboPlanWorld` for one selected planning group on one robot.
- [x] 4.2 Validate selected start and goal states exactly match the configured planning group by joint set.
- [x] 4.3 Convert selected start and goal states into `roboplan_core.JointConfiguration(native_joint_names, native_q)`.
- [x] 4.4 Set `RRTOptions.group_name` to the selected group's RoboPlan group name and keep explicit option values such as `collision_check_use_bisection`.
- [x] 4.5 Extract RoboPlan `JointPath` waypoints, validate waypoint lengths and names when available, and return public global group-order waypoints.
- [x] 4.6 Return `PlanningStatus.UNSUPPORTED` with explanatory messages for multi-group, multi-robot, or non-matching selections.
- [x] 4.7 Remove RoboPlan legacy `plan_joint_path(...)` wrapper expectations and make selected-group planning the native RoboPlan planner entry point.

## 5. Factory and Compatibility Checks

- [x] 5.1 Update `create_planner("roboplan", world=...)` validation to require the current group-native `plan_selected_joint_path(...)` planner contract.
- [x] 5.2 Update `PlannerSpec`/`WorldSpec` validation assumptions if they still force RoboPlan to keep deprecated robot-scoped planner/FK/Jacobian wrappers.
- [x] 5.3 Ensure error messages distinguish unsupported planning selections from RoboPlan planner failures.
- [x] 5.4 Preserve existing robot-scoped collision APIs and joint-limit behavior in robot-local `config.joint_names` order because those remain part of world state/collision management.

## 6. Tests and Verification

- [x] 6.1 Extend RoboPlan fake scene/planner helpers to expose group info, native group order, and selected-group path behavior.
- [x] 6.2 Add tests for provided SRDF passthrough and single-group generated SRDF naming.
- [x] 6.3 Add tests for no-SRDF multi-group rejection.
- [x] 6.4 Add tests for native group order differing from public group order across planning input and output.
- [x] 6.5 Add tests for `plan_selected_joint_path(...)` success and unsupported selection failures.
- [x] 6.6 Add tests for `get_group_ee_pose(...)` and `get_group_jacobian(...)` group behavior without legacy wrapper coverage.
- [x] 6.7 Run `uv run pytest dimos/manipulation/test_roboplan_world.py dimos/manipulation/test_planning_factory.py -q`.
