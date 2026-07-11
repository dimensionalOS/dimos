# Manipulation Planning Groups

Planning groups are named, selectable kinematic chains used by manipulation
planning. They let APIs target a specific part of a robot, such as an arm or
torso, without confusing that group with the robot's hardware identity.

## Concepts

| Concept | Meaning |
|---------|---------|
| Robot name | The configured robot ID in `RobotModelConfig.name`. |
| Planning group | A named serial chain of controllable joints on one robot. |
| Planning group ID | Stable API ID in the form `{robot_name}/{group_name}`. |
| Local joint name | Joint name inside a robot model, such as `joint1`. |
| Global joint name | Boundary-level joint name in the form `{robot_name}/{local_joint_name}`. |
| Generated plan | Planning artifact containing selected group IDs, geometric waypoints, and one synchronized global-joint trajectory. |
| Auxiliary group | A selected group that contributes free DOFs to a pose plan without receiving its own pose target. |

Local URDF/SRDF joint names stay inside robot-scoped configuration, model
parsing, and backend internals. Flat planning states and generated plan paths
use global joint names so multiple robots can safely share local names such as
`joint1`.

## Discovering planning groups

Planning-group config helpers can discover groups in this order:

1. Explicit `planning_groups` on the robot model config.
2. Explicit `srdf_path` on the robot model config.
3. Conservative SRDF auto-discovery near the model path, with a warning.
4. Fallback generation of one `{robot_name}/manipulator` group when the
   configured controllable joints form exactly one unambiguous serial chain.
5. Error if no SRDF or fallback chain can provide a single valid group.

Direct `RobotModelConfig(...)` construction does not run discovery or synthesize
groups in `model_post_init`; callers must pass explicit `planning_groups` there.

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
references, mixed group declarations, branching or non-serial groups, and SRDF
`<end_effector>` metadata. A chain group's `tip_link` is its pose target frame.
An ordered joint-list group can be pose-targeted only when DimOS can validate a
unique serial target frame.

## Fallback behavior

When no SRDF or explicit group config is available,
fallback uses `RobotModelConfig.joint_names` as the candidate controllable set.
This field is the robot's ordered local model joint set, not an implicit
planning group.

Fallback succeeds only when those joints form one unambiguous serial chain. It
allows prismatic joints in the middle of the chain and strips only terminal tip
prismatic joints, which usually represent gripper fingers. The generated group
name is always `manipulator`.

## Current APIs

Use `list_planning_groups()` to discover group IDs and capabilities before
planning:

```python skip
groups = manip.list_planning_groups()
pose_groups = [group for group in groups if group.has_pose_target]
group_id = pose_groups[0].id
```

Joint-space planning targets group IDs and uses local joint names inside each
target `JointState`:

```python skip
ok = manip.plan_to_joint_targets(
    {
        "left_arm/manipulator": JointState(
            name=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            position=[0.0, -0.4, 0.2, 0.0, 0.3, 0.0],
        )
    }
)
```

Pose planning targets pose-capable group IDs. Add auxiliary groups when another
chain should participate as free DOFs but does not have its own pose target:

```python skip
ok = manip.plan_to_pose_targets(
    {"left_arm/manipulator": target_pose},
    auxiliary_groups=["torso/manipulator"],
)
```

After a successful planning call, preview and execution use the module's current
stored plan:

```python skip
manip.preview_plan()
manip.execute_plan()
```

Callers that already hold a `GeneratedPlan` may pass it explicitly:

```python skip
manip.preview_plan(plan)
manip.execute_plan(plan)
```

For robot-scoped compatibility APIs, unnamed joint vectors are interpreted in
the robot model's configured joint order. If names are provided, they must be
local model joint names: no global names, missing joints, extra joints, or
partial joint sets.

## Generated plans and execution

A `GeneratedPlan` stores:

- selected planning group IDs;
- a geometric path of `JointState` waypoints keyed by global joint names;
- one materialized synchronized `JointTrajectory` over the same selected global
  joint names;
- status, timing, path length, iteration count, and message metadata.

Preview and execution consume the stored trajectory; they do not lazily
parameterize the geometric path. Preview forwards the raw globally named
trajectory through the visualization boundary, where renderers project it to
their robot-local visuals while preserving stored timestamps. Execution splits
the stored trajectory by affected trajectory task, translates selected joint
names at the coordinator boundary, and invokes each trajectory controller
without filling or commanding omitted joints. Controllers remain planning-group
agnostic, and trajectory tasks still claim their full configured joint set while
executing only the active planned subset.

Multi-task dispatch is not atomic: if one trajectory task accepts and a later
task rejects, DimOS reports the rejection but does not roll back the accepted
task.

## Robot placement config

`RobotModelConfig.base_pose` and `RobotModelConfig.base_link` describe robot
placement: `base_pose` places `base_link` in the world and current backends
use that link for weld/placement and optional model-authored world-joint
stripping. This is robot placement metadata, not planning-chain metadata.

Planning-group `base_link` and `tip_link` values are the only source for chain
bases and pose target frames. Robot-scoped end-effector config is no longer
supported; robot-level EE helper APIs are wrappers over a unique pose-targetable
planning group and should use explicit group APIs when multiple pose groups
exist.

Robot placement can be encoded either in model assets or in `base_pose`,
depending on the blueprint. `joint_names` remains supported and should describe
the ordered controllable local model joint set.
