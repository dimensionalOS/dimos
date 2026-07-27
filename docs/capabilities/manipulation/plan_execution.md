# Manipulation Plan Execution

DimOS executes a `GeneratedPlan` as one synchronized unit through the control
coordinator's sole trajectory task. A plan may command all configured joints or
any subset, but execution never filters a plan by robot.

## Execute a plan

Planning stores the latest generated plan in the manipulation module:

```python skip
if manip.plan_to_pose_targets({"left_arm/manipulator": target_pose}):
    accepted = manip.execute_plan()
```

Callers that already hold a plan may pass it directly:

```python skip
accepted = manip.execute_plan(plan)
```

To execute one arm, create a plan for that arm's planning group. A coordinated
two-arm plan is always dispatched as a whole.

`True` means the coordinator trajectory task accepted the trajectory. It does
not mean the robot has finished moving. The module projects acceptance as
`ManipulationState.COMPLETED` for compatibility with the current state model.

## Validation and mapping

The planner owns `GeneratedPlan` correctness. The execution manager validates
only the plan status and the global-to-coordinator joint mapping, preserves the
trajectory's joint order and shared timing, and makes one coordinator RPC.

The coordinator's trajectory task is the executable-trajectory authority. It
validates trajectory structure and configured joints. Before acceptance, it
also compares every submitted joint's first trajectory position with the
coordinator's current hardware position. Missing state or a difference above
the task's `start_position_tolerance` rejects the trajectory.

Subset trajectories retain subset output behavior: the trajectory task emits
only the submitted joints. Unmentioned joints are not synthesized into the
trajectory.

## Replacement, cancellation, and failures

A new accepted trajectory replaces the trajectory task's current trajectory.
The execution manager does not pre-cancel or perform multi-task rollback because
there is only one coordinator trajectory task.

Call `cancel()` to stop planned execution:

```python skip
cancelled = manip.cancel()
```

Execute and cancel operations are serialized. If cancellation arrives while an
execute RPC is in flight, it waits for that RPC and then cancels the accepted
trajectory.

Known coordinator outcomes use semantic result types. Invalid trajectories,
start-state mismatches, missing state, and missing trajectory-task
configuration are deterministic rejections. RPC exceptions and timeouts are
uncertain because the remote operation may still have occurred; manipulation
enters `FAULT` for those outcomes.

Direct gripper commands remain separate from planned execution.
