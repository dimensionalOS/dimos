# Policy teleoperation

Shared control runs as `PolicyTeleopTask` inside the coordinator. It mixes
operator and policy **joint targets** on each coordinator tick:

```text
output = (1 - weight) * operator + weight * policy
```

The policy runs alone until an operator source starts commanding. With both
sources present, the policy share approaches `alpha` (default 0.5). When the
operator's output expires, the share approaches one again. The policy keeps
running throughout. Device deadman keys gate the device's publication.

## Blueprint configuration

```python
from dimos.robot.manipulators.common.blueprints import (
    eef_twist_task,
    policy_teleop_task,
    trajectory_task,
)

operator = eef_twist_task(hardware, robot_model=model, target_frame="link_tcp")
policy = trajectory_task(hardware)
tasks = [
    operator,
    policy,
    policy_teleop_task(
        hardware.joints,
        operator_tasks=[operator.name],
        policy_tasks=[policy.name],
        alpha=0.5,
        weight_rate=2.0,
        source_timeout_s=0.2,
    ),
]
```

Pass `tasks` to the coordinator. Sources stay registered for stream routing,
RPCs and emergency stops. The blender computes them once each tick and owns
their joints during arbitration. Keep every source before the blender in the
configuration. Add the gripper task to `operator_tasks` and its joint names to
`switch_joints` to select gripper targets instead of interpolating them.

`LeRobotPolicyModule` already uses `execute_trajectory`; it requires no adapter
or new output stream. Operator devices retain their existing command streams.

## Custom weights and status

A takeover function receives measured positions, latest source targets and tick
time. Return a scalar policy share or a mapping keyed by joint name:

```python
from dimos.control.tasks.policy_teleop_task.policy_teleop_task import TakeoverContext


def takeover(context: TakeoverContext) -> float:
    error = max(abs(context.operator[j] - context.policy[j]) for j in context.joint_names)
    return min(1.0, error / 0.5)
```

Pass `takeover_fn=takeover` in Python, or `"your_package.weights:takeover"` in
JSON/CLI configuration. Missing mapping keys use `alpha`. Finite weights are
clipped to [0, 1]; callable failures or non-finite weights use `alpha` for that
tick and log the failure once. Weight changes are limited by `weight_rate`.

Read `coordinator.task_invoke("policy_teleop", "get_status")` to inspect
`weights`, `operator_present` and `policy_present`. Source presence is tracked
per joint from the time the source last returned a position. Emergency stop
clears cached targets and weights; sources follow their existing stop behavior.

## Simulation

```bash
uv run dimos run policy-teleop-sim
uv run dimos run demo-policy-teleop-human-sim
uv run dimos run demo-policy-teleop-policy-sim
uv run dimos run policy-teleop-sim --mujocosimmodule.headless=true
```

After initial feedback, the synthetic policy sweeps the base joint left and
right by 0.6 rad (about 34 degrees) around its starting position. One cycle takes
12 seconds; 20 cycles give four minutes to try the controls. Other arm joints
keep their starting targets. The sampled sinusoidal path returns to its center
each cycle without drifting. It is submitted as one trajectory, so e-stop or
cancellation does not automatically restart the demo.

Click the separate **Keyboard Teleop** window to give it keyboard focus. The
MuJoCo viewer and the terminal do not forward keys to teleoperation. The keyboard
window shows a focus hint; while holding Q its green linear-twist display should
read `Z=0.120 m/s`. If it stays at zero, the device has not received the key.

Use the keyboard window to supply EEF twists. Hold Q/E for a second or two to
raise/lower the tip while the policy sweeps sideways, or A/D to push along/against
the lateral motion. Release the key to fade back to the policy's path. W/S moves
along X; R/F, T/G and Y/H rotate. This demo sets keyboard speeds to 0.12 m/s and
0.8 rad/s to make the intervention visible. The variants set policy share to 0
or 1 when both sides are present. With only one source present that source still
commands the arm. Gripper teleoperation is not configured in this minimal example.
Stop a blueprint before starting another.

Tune the sweep with module configuration, for example:

```bash
uv run dimos run policy-teleop-sim --jointpolicydemo.amplitude_rad=0.8 --jointpolicydemo.duration_s=16 --jointpolicydemo.cycles=10
```

The curve helpers and Cartesian examples in this directory remain available
for offline illustration:

```bash
uv run python -m dimos.teleop.policy.demo_takeover --output /tmp/policy-teleop.png
uv run pytest dimos/control/tasks/policy_teleop_task dimos/control/test_policy_teleop.py
uv run pytest dimos/teleop/policy/test_simulation.py -m self_hosted
```

See the [task specification](/docs/capabilities/teleoperation/policy_teleop_spec.md)
for the exact per-tick rules and configuration contract.
