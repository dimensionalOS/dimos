# Policy teleoperation as a coordinator task

`PolicyTeleopTask` blends joint targets inside `ControlCoordinator`, using the
same measured joints and clock as its source tasks. For each joint:

```text
out = (1 - w) * U + w * P
```

`Q` is measured joint position, `U` is the operator task's target, and `P` is
the policy task's target. `w` is the policy's share: zero selects the operator
and one selects the policy. With both sides present, `w` moves towards the
configured `alpha` or the result of `TakeoverFn`.

The policy runs without operator input. Operator commands can join and leave at
any time; their share ramps in and out while the policy continues computing.
PTM has no mode switch or deadman. A device may gate its own publication with a
deadman; holding it admits operator commands into the blend.

## Sources and joint ownership

Ordinary source tasks remain registered before the PTM in the coordinator's
`tasks` list. Existing routing and conversion remain in place:

| Source | Coordinator entry point | Source task |
| --- | --- | --- |
| Keyboard twist | `ee_twist_command` | `eef_twist` |
| Headset pose | Cartesian pose input | `teleop_ik` |
| Gripper intent | `gripper_command` | `gripper` |
| Policy trajectory | `execute_trajectory(JointTrajectory)` | `trajectory` |
| Joint target device | `joint_command` | `trajectory` |

A registered source belongs to exactly one PTM and one side of that PTM. Task
names refer to instances, not registered type names. Each source computes once
per tick, called by PTM, and its output bypasses normal arbitration. PTM alone
arbitrates for the source joints. This prevents twist double integration and
preemption of a lower-priority policy trajectory by an operator source.

A joint device used as the operator source needs its own task instance, distinct
from the policy source. The coordinator currently permits one canonical
`JointTrajectoryTask`; listing that instance on both sides is rejected.

`ControlTaskRegistry.create(..., tasks=...)` injects the coordinator's already
created tasks only into factories declaring a `tasks` parameter. PTM declares
no `TASK_CONSUMES`; its card exposes only `get_status`. Source routing, trajectory
RPCs, TF publication and coordinator emergency stops continue through the
registered source tasks. Actual preemption of PTM is forwarded to every source.

## Configuration

Use `TaskConfig(type="policy_teleop", name=..., joint_names=..., priority=...,
params=...)`, or the `policy_teleop_task` blueprint helper in
`dimos/robot/manipulators/common/blueprints.py`. Its joints must include every
joint claimed by its sources. Unknown, repeated, shared or nested source names
fail at creation.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `operator_tasks` | required | Names of tasks producing U |
| `policy_tasks` | required | Names of tasks producing P |
| `alpha` | `0.5` | Policy share in [0, 1] while both sides are present |
| `takeover_fn` | `None` | Callable or `"package.module:function"`, usable from JSON/CLI |
| `switch_joints` | `[]` | Select a source at weight 0.5 instead of interpolating |
| `weight_rate` | `2.0` | Positive maximum change of weight per second |
| `source_timeout_s` | `0.2` | Positive lifetime of the last output on each joint |

Sources must produce position outputs. PTM returns `SERVO_POSITION`, including
when `alpha=1`; policy targets never pass through Cartesian conversion or IK.

## Per-tick computation

1. Call `compute(state)` on each active source. Cache each returned joint
   position with `state.t_now`, separately for operator and policy.
2. A side is present on a joint until its last output is older than
   `source_timeout_s`. Inactive trajectory tasks can therefore hold their latest
   target between policy submissions. Freshness is per joint, not per task.
3. Choose the target share: `TakeoverFn`/`alpha` for both sides, one for policy
   only, zero for operator only. Emit nothing when neither side is present.
4. Move `w`, initially one, towards the target by at most
   `weight_rate * state.dt`. If either side has never produced a value for the
   joint, set `w` to the target immediately.
5. Blend the latest values, retaining expired endpoints while the weight ramps.
   A switch joint selects P at `w >= 0.5`, otherwise U.
6. Return one position output for joints with a present source, or `None`.

The task uses only `state.t_now` and `state.dt` for time. It is always active and
claims its configured joints and priority. Emergency stop and runtime reset
forget cached values and presence, and restore all weights to one. The
coordinator stops the source tasks itself. No cached pre-stop command is
restored by PTM.

Weight ramping bounds the change in source authority. Source targets can still
change, and switch joints intentionally select whole targets at the threshold.

## Takeover function

```python
@dataclass(frozen=True)
class TakeoverContext:
    joint_names: tuple[str, ...]
    measured: Mapping[str, float]
    operator: Mapping[str, float]
    policy: Mapping[str, float]
    t_now: float
    dt: float

TakeoverFn = Callable[[TakeoverContext], float | Mapping[str, float]]
```

`joint_names` contains the joints on which both sides are present. The mappings
are read-only snapshots of Q, U and P. The callable is invoked once per tick,
only when at least one joint has both sides present. A scalar applies to every
such joint; a mapping overrides named joints and uses `alpha` for missing names.
Finite weights are clipped to [0, 1]. An exception or non-finite result falls
back to `alpha` for that tick, with the failure logged once per task instance.
Without a callable, `alpha` is the constant arbitration function.

Linear, quadratic, power and logarithmic progress curves remain optional helpers
in `dimos/teleop/policy/takeover.py`. The offline Cartesian plotting examples do
not participate in the coordinator's joint control path.

## Status and verification

`ControlCoordinator.task_invoke("policy_teleop", "get_status")` returns a plain
mapping with `weights` per joint, `operator_present` joint names and
`policy_present` joint names, from the latest tick.

The `policy-teleop-sim` blueprint contains `eef_twist`, `joint_trajectory` and
`policy_teleop`. Keyboard commands reach `ee_twist_command`. `JointPolicyDemo`
submits an out-and-back joint trajectory through `execute_trajectory` after the
first measured feedback, using the same RPC as `LeRobotPolicyModule`. There is
no TCP feedback adapter or twist-publishing policy. `alpha` is 0.5; the two demo
variants set it to zero or one while both sources are present.

Tests cover source absence and expiry, opposing targets, per-joint weights,
weight ramps, switch joints, callable failures and import paths, emergency stop,
single source computation, source routing, status and uninterrupted trajectory
execution. Self-hosted tests exercise actual Pink IK and MuJoCo.
