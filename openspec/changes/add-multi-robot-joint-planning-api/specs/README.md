# Multi-robot joint planning specs

This folder contains the OpenSpec capability delta for adding coordinated multi-robot joint planning to the manipulation stack.

## What changed conceptually

The key internal representation change is that the manipulation planner should no longer treat a successful plan as independent per-robot cached artifacts:

```text
_planned_paths[robot_name]         -> JointPath
_planned_trajectories[robot_name]  -> JointTrajectory
```

Instead, a successful single- or multi-robot plan should produce one authoritative active timed motion plan:

```python
@dataclass
class MotionPlan:
    robot_names: list[RobotName]
    paths: dict[RobotName, JointPath]
    trajectories: dict[RobotName, JointTrajectory]
    duration: float
    created_from: Literal["joints", "pose"]
```

The old per-robot dictionaries may remain as compatibility mirrors, but they should be derived atomically from the active `MotionPlan`.

## Key implementation invariant

For coordinated multi-robot planning, timing must be generated on the combined joint vector before trajectories are split by robot:

```text
ordered robot_names
  -> composite start / goal / limits
  -> combined geometric path
  -> combined timed trajectory
  -> split synchronized per-robot trajectories
  -> store one MotionPlan
```

Do not time-parameterize each robot independently for a coordinated plan.

## API direction

Reuse the existing planning APIs with scalar-or-list inputs instead of adding a family of `multiple` methods:

```python
plan_to_joints(joints: JointState | list[JointState], robot_name: str | list[str] | None = None)
plan_to_pose(pose: Pose | list[Pose], robot_name: str | list[str] | None = None)
```

List inputs are ordered. That order defines the composite joint-vector layout.

`plan_to_pose` keeps current semantics: solve IK per requested target pose, then perform coordinated joint-space planning. It is not Cartesian path planning.

## Spec files

- `manipulation-stack/spec.md`: normative behavior requirements and scenarios.
- `manipulation-stack/README.md`: technical summary of the internal representation and design decisions.
