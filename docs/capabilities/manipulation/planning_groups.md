# Planning Groups

Planning groups are named, selectable kinematic chains used by manipulation
planning. They let APIs target a specific part of a robot, such as an arm or
torso, without confusing that group with the robot's hardware identity.

## Concepts

| Concept | Meaning |
|---------|---------|
| Model | The single configured `RobotModelConfig`. |
| Planning group | A named subset of the model's controllable joints. |
| Planning group ID | Stable declared group name, such as `left_arm`. |
| Canonical joint name | Joint name used unchanged by the model, planner, coordinator, and visualization. |
| Generated plan | Planning artifact containing selected group IDs, geometric waypoints, and one synchronized canonical trajectory. |
| Auxiliary group | A selected group that contributes free DOFs to a pose plan without receiving its own pose target. |

The configured model owns one canonical joint namespace. Planning groups select
subsets of that namespace; they do not rename or prefix joints.

## Discovering planning groups

Robot configs can provide planning groups explicitly with
`RobotModelConfig.planning_groups`. Direct `RobotModelConfig(...)` construction
does not run discovery or synthesize groups in `model_post_init`; callers must
pass explicit `planning_groups` there.

When code uses the discovery helper instead of explicit config, dimOS discovers
groups in this order:

1. Explicit `srdf_path` provided to the helper.
2. Conservative SRDF auto-discovery near the model path, with a warning.
3. Fallback generation of one `manipulator` group when the
   configured controllable joints form exactly one unambiguous serial chain.
4. Error if no SRDF or fallback chain can provide a single valid group.

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
An ordered joint-list group can be pose-targeted only when dimOS can validate a
unique serial target frame.

## Fallback behavior

When discovery runs without an SRDF, fallback uses
`RobotModelConfig.joint_names` as the candidate controllable set.
This field is the model's ordered canonical joint set, not an implicit
planning group.

Fallback removes terminal prismatic leaves first, including branched finger
joints, and then requires the remaining joints to form one unambiguous serial
chain. Internal prismatic axes remain part of the arm. The generated group name
is always `manipulator`.


## Current APIs

Use `list_planning_groups()` to discover group IDs and capabilities before
planning:

```python skip
groups = manip.list_planning_groups()
pose_groups = [group for group in groups if group.has_pose_target]
group_id = pose_groups[0].id
```

Joint-space planning targets group IDs. Each target `JointState` may be
unnamed in the group's joint order or named with the group's canonical joints.

```python skip
result = manip.plan_to_joints(
    {
        "left_arm": JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.0, -0.4, 0.2],
        )
    }
)
```

Pose planning targets pose-capable group IDs with `PoseStamped` values:

```python skip
result = manip.plan_to_poses({"left_arm": target_pose})
```

After a successful planning call, preview and execution use the module's current
stored plan:

```python skip
if result.succeeded:
    manip.preview_plan(result.plan)
    manip.execute()
```

Callers that already hold a `GeneratedPlan` may preview it explicitly; execution
always consumes the module's current stored plan:

```python skip
manip.preview_plan(plan)
manip.execute()
```

A generated plan is the execution boundary. The canonical trajectory is sent
unchanged to the coordinator. To execute a subset, first plan only that subset's
planning group.

## Generated plans and execution

A `GeneratedPlan` stores:

- selected planning group IDs;
- a geometric path of `JointState` waypoints keyed by canonical joint names;
- one materialized synchronized `JointTrajectory` over the same selected canonical
  joint names;
- status, timing, path length, iteration count, and message metadata.

Preview and execution consume the stored trajectory; they do not lazily
parameterize the geometric path. Preview and execution forward the canonical
trajectory without renaming, splitting, or merging it. The coordinator's
trajectory task remains planning-group agnostic and holds omitted joints while
executing the selected subset.

## Robot placement config

`RobotModelConfig.base_pose` and `RobotModelConfig.base_link` describe robot
placement: `base_pose` places `base_link` in the world and current backends
use that link for weld/placement and optional model-authored world-joint
stripping. This is model placement metadata, not planning-chain metadata.

Planning-group `base_link` and `tip_link` values are the source for chain bases
and pose target frames. Convenience pose APIs require an explicit group when
the model has multiple pose-targetable groups.

Robot placement can be encoded either in model assets or in `base_pose`,
depending on the blueprint. `joint_names` describes the ordered controllable
canonical model joint set.
