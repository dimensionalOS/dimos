# Manipulation Planning Groups

Planning groups are named, selectable kinematic chains used by manipulation
planning. They separate the hardware robot identity from the part of the robot
being planned.

## Concepts

| Concept | Meaning |
|---------|---------|
| Planning group | A named serial chain of controllable robot joints. |
| Planning group ID | Stable API ID in the form `{robot_name}/{group_name}`. |
| Global joint name | Boundary-level joint name in the form `{robot_name}/{local_joint_name}`. |
| Local joint name | The joint name as it appears in the robot model. |
| Generated plan | Minimal planning artifact containing selected group IDs and one synchronized global-joint path. |
| Auxiliary group | A group selected for a pose request without receiving its own pose target. |

Local URDF/SRDF joint names stay inside robot-scoped APIs, model parsing, and
backend internals. Group-scoped APIs, generated plans, preview, execution, and
coordinator boundaries use global joint names so two robots can safely have the
same local joint names.

`PlanningGroup` descriptors returned by `list_planning_groups()` include both
namespaces:

- `id`: public `{robot_name}/{group_name}` selector;
- `joint_names`: selected global joint names in group order;
- `local_joint_names`: selected local model joint names in group order;
- `base_link` / `tip_link`: model links for group kinematics. `tip_link=None`
  means the group can participate as an auxiliary group but cannot receive a
  pose target.

## Planning group sources

DimOS discovers planning groups in this order:

1. Explicit `srdf_path` on `RobotConfig` / `RobotModelConfig`.
2. Conservative SRDF auto-discovery near the model path, with a visible warning.
3. Fallback generation of one `{robot_name}/manipulator` group if the configured
   controllable joints form exactly one unambiguous serial chain.
4. Error if no SRDF exists and fallback cannot infer a single chain.

Supported SRDF group forms:

```xml
<group name="arm">
  <chain base_link="base_link" tip_link="tool0" />
</group>
```

```xml
<group name="arm">
  <joint name="joint1" />
  <joint name="joint2" />
  <joint name="joint3" />
</group>
```

Unsupported SRDF forms are skipped with warnings: link groups, nested group
references, mixed group declarations, branching/non-serial groups, and SRDF
`<end_effector>` metadata. A chain group's `tip_link` is the pose target frame.
An ordered joint-list group may be pose-targeted only when DimOS can validate a
unique serial target frame.

## Fallback behavior

When no SRDF is available, fallback uses `RobotModelConfig.joint_names` as the
candidate controllable set. This field is the robot's ordered local model joint
set, not an implicit planning group.

Fallback succeeds only when those joints form one unambiguous serial chain. It
allows prismatic joints in the middle of the chain and strips only terminal/tip
prismatic joints, which usually represent gripper fingers. The generated group
name is always `manipulator`.

## Planning APIs

Planning APIs select groups explicitly. Descriptors returned by
`ManipulationModule.list_planning_groups()` can be passed anywhere a group ID is
accepted; the module normalizes descriptors back to IDs and re-resolves current
world state.

```python skip
# Discover groups. Each item is a PlanningGroup dataclass.
groups = manip.list_planning_groups()
arm = groups[0]

# Joint-space planning for one group. Named targets may use global names...
manip.plan_to_joint_targets({
    arm.id: JointState(
        name=["left_arm/joint1", "left_arm/joint2"],
        position=[0.2, -0.1],
    )
})

# ...or local model names, but not a mix of both namespaces.
manip.plan_to_joint_targets({
    arm: JointState(
        name=["joint1", "joint2"],
        position=[0.2, -0.1],
    )
})

# Pose planning for an arm while a torso/waist group participates as free DOFs.
manip.plan_to_pose_targets(
    {"robot/arm": target_pose},
    auxiliary_groups=["robot/torso"],
)

plan = manip._last_plan
manip.preview_plan(plan)
manip.execute_plan(plan)
```

### Pose targets

`plan_to_pose_targets()` accepts `Mapping[PlanningGroupID | PlanningGroup,
Pose]`. It wraps each `Pose` in the world frame before calling the group-scoped
IK path. Every key in `pose_targets` must refer to a pose-targetable group
(`tip_link` is not `None`). `auxiliary_groups` may include non-pose-targeted
groups whose joints should stay in the solve as free DOFs.

`inverse_kinematics()` is the lower-level RPC. It accepts stamped poses keyed by
group ID plus optional auxiliary group IDs and returns an `IKResult` without
running collision filtering or planning.

The compatibility wrapper `plan_to_pose(pose, robot_name=None)` still exists. It
selects the default pose-targetable group for the robot, then delegates to
`plan_to_pose_targets()`.

### Joint targets

`plan_to_joint_targets()` accepts `Mapping[PlanningGroupID | PlanningGroup,
JointState]`.

For a group-scoped joint target:

- an unnamed vector is interpreted in that group's joint order;
- named targets may use all-global names or all-local names;
- global and local names must not be mixed in one target;
- named targets must provide exactly the group's selected joints, with no
  missing or extra joints.

The compatibility wrapper `plan_to_joints(joints, robot_name=None)` still exists.
It selects the robot's default group, then delegates to
`plan_to_joint_targets()`.

Robot-scoped state helpers such as `set_init_joints()` still use local model
joint names. When unnamed, those vectors are interpreted in full robot model
joint order.

## Generated plans and execution

A `GeneratedPlan` stores:

- selected planning group IDs;
- a single synchronized path of `JointState` waypoints keyed by selected global
  joint names;
- status, timing, path length, iteration count, and message metadata.

Preview and execution project this path lazily. `preview_plan(plan=None,
duration=None, robot_name=None)` defaults to `_last_plan`; `robot_name` is only a
filter that rejects plans which do not affect the requested robot. Preview sends
the generated global-joint path to the world monitor for animation.

`execute_plan(plan=None)` also defaults to `_last_plan`. It infers affected
robots from the selected groups, projects each waypoint back into each robot's
full local model joint order, fills unselected robot joints from current state,
then writes a coordinator `JointTrajectory` using global joint names. The
trajectory is dispatched to each robot's configured `coordinator_task_name`.
Controllers remain planning-group agnostic.

Multi-task dispatch is not atomic in this change: if one trajectory task accepts
and a later task rejects, DimOS reports the rejection but does not roll back the
accepted task.

## Compatibility planning config fields

`RobotConfig.base_link`, `RobotConfig.base_pose`,
`RobotModelConfig.base_link`, `RobotModelConfig.base_pose`, and
`RobotModelConfig.end_effector_link` remain as compatibility fields for the
current Drake weld/placement behavior and robot-scoped compatibility helpers.
New planning logic should use model/SRDF structure and planning group base/tip
links instead.

Robot placement should be encoded in URDF/xacro/MJCF. `joint_names` remains
supported and should describe the ordered controllable local model joint set, not
a planning group. `joint_name_mapping` can map external/coordinator joint names
back to local model joint names for adapters that publish scoped hardware names.
`coordinator_task_name` identifies the trajectory task used by `execute_plan()`.
