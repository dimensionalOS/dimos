# Manipulation Plan Execution

DimOS executes a `GeneratedPlan` as one synchronized unit. A plan that spans
multiple robots commands every affected trajectory task; callers cannot execute
only one robot from that plan.

## Execute a plan

Planning stores the latest generated plan in the manipulation module. Execute it
after the planning call succeeds:

```python skip
if manip.plan_to_pose_targets({"left_arm/manipulator": target_pose}):
    accepted = manip.execute_plan()
```

Callers that already hold a plan may pass it directly:

```python skip
accepted = manip.execute_plan(plan)
```

The compatibility form `execute_plan(robot_name="left_arm")` accepts only a
single-robot plan for `left_arm`. It rejects a multi-robot plan instead of
filtering it.

`True` means every affected coordinator task accepted its trajectory. It does
not mean the robot has finished moving. The module projects acceptance as
`ManipulationState.COMPLETED` for compatibility with the current state model.

## Validation

Execution accepts only a successful `GeneratedPlan` with a non-empty path and a
complete, timed trajectory. Before the first coordinator call, it checks:

- every trajectory value is finite and every point has the expected width;
- timestamps start at zero and increase;
- global joint names use `{robot_name}/{local_joint_name}`;
- planning groups and trajectory joints cover the same robots;
- the path and trajectory use the same ordered joints; and
- live joint positions match the first trajectory waypoint within `1e-6`.

Missing, duplicate, reordered, stale, or mismatched joints reject the whole plan.
No new trajectory task receives a partial plan.

## Replacement and cancellation

DimOS remembers every trajectory task that may have accepted a plan. A later
`execute_plan()` first cancels those tasks, then dispatches the replacement. If
the coordinator cannot confirm that an earlier task stopped, DimOS rejects the
replacement because overlapping motion may still be active.

Call `cancel()` to stop all tracked planned-execution tasks:

```python skip
cancelled = manip.cancel()
```

Repeated cancellation is safe. The existing boolean API returns `False` when
there is no planning or tracked execution to cancel. Planning cancellation
remains separate: it invalidates the in-progress planning result before asking
the execution manager to cancel tracked tasks.

## Dispatch failures

Multi-task coordinator calls happen sequentially. If one task rejects a
trajectory after another accepts, DimOS cancels every task that may have
accepted. A confirmed rollback is a safe rejection. An RPC exception, unknown
dispatch result, or unconfirmed cancellation leaves execution safety uncertain
and produces a fault.

Direct gripper commands do not use planned execution and remain owned by the
manipulation module.

## Coordinator ownership

Treat the manipulation execution manager as the single writer for its configured
trajectory tasks. The coordinator RPC has no caller identity or lease, so it
cannot enforce ownership. Diagnostic clients must not invoke `execute` on the
same coordinator tasks while manipulation plan execution is active.
