## Why

`RoboPlanWorld` is wired into a manipulation stack that now plans through public planning groups, but its native planner path still assumes robot-local joint order and uses the robot name as the RoboPlan group. This makes RoboPlan planning fragile when public group order, RoboPlan native group order, and full robot state order diverge.

This change aligns RoboPlan planning, FK, and Jacobian queries with the existing planning-group API while keeping the custom backend interface lean and minimal.

## What Changes

- Add RoboPlan support for `PlannerSpec.plan_selected_joint_path(...)` over a single selected planning group.
- Preserve full robot-local context state as the world belief model; project between full robot state, public group order, and RoboPlan native group order by name.
- Use existing public `PlanningGroup` data instead of introducing a new custom group interface; store only RoboPlan-specific native joint order as adapter metadata.
- Keep auto-generated SRDF support intentionally simple: one configured planning group only. Require `RobotModelConfig.srdf_path` for multi-group or more complex RoboPlan SRDF semantics.
- Implement only the up-to-date `WorldSpec` group APIs (`get_group_ee_pose(...)`, `get_group_jacobian(...)`) and `PlannerSpec.plan_selected_joint_path(...)` for RoboPlan.
- Remove RoboPlan-specific legacy planning/world wrapper expectations instead of preserving `plan_joint_path(...)`, `get_ee_pose(...)`, or `get_jacobian(...)` compatibility behavior.
- Return explicit unsupported or validation failures for multi-group, multi-robot, missing SRDF, mismatched group names, or mismatched joint sets instead of silently assuming order compatibility.

## Capabilities

### New Capabilities
- `roboplan-planning-group-native-order`: RoboPlan world/planner behavior uses planning-group-native joint ordering while preserving full robot belief state and keeping the RoboPlan interface minimal.

### Modified Capabilities

None.

## Impact

- Affected code:
  - `dimos/manipulation/planning/world/roboplan_world.py`
  - `dimos/manipulation/planning/spec/protocols.py` if the current protocol definitions still expose deprecated robot-scoped planner/world methods
  - `dimos/manipulation/planning/factory.py`
  - `dimos/manipulation/test_roboplan_world.py`
  - related planning-group tests if integration behavior needs coverage
- Public planning API impact:
  - Enables the existing `plan_selected_joint_path(...)` contract for RoboPlan-native planning.
  - Implements only current `PlannerSpec`/`WorldSpec` group-oriented APIs for RoboPlan.
- Dependency impact:
  - No new dependencies.
  - RoboPlan still consumes URDF/SRDF through existing bindings.
- Behavior impact:
  - Simple generated SRDF remains supported for one planning group.
  - Complex/multi-group RoboPlan setups must provide `RobotModelConfig.srdf_path`.
