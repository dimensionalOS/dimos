# Policy teleoperation

`PolicyTeleopModule` blends velocities for **one TCP in one world frame**.
Linear components use m/s; angular components use rad/s. Both sources must
already refer to that TCP. Pose and trajectory commands need upstream adapters.

| Port | Type | Meaning |
| --- | --- | --- |
| `current_pose` | `In[Transform]` | Measured world-to-TCP transform |
| `teleop_command` | `In[TwistStamped]` | Human velocity |
| `policy_command` | `In[TwistStamped]` | Policy velocity |
| `ee_twist_command` | `Out[TwistStamped]` | Blended velocity |

Pass `takeover_fn` at construction or in `.blueprint(takeover_fn=...)`.
It returns final policy authority: a scalar or six weights ordered
`[vx, vy, vz, wx, wy, wz]`. For each axis:

```text
output = (1 - alpha) * human + alpha * policy
```

Zero selects human control; one selects policy control. Rotation weights blend
angular velocities, never quaternion components. Values outside `[0, 1]` are
faults, rather than silently clamped weights.

`PolicyTeleopConfig` provides `control_hz`, `timeout_s`, `world_frame` and
`tcp_frame`. `timeout_s` applies to all three input timestamps (Unix seconds);
producers must use a synchronized clock. A missing, stale, future-dated,
nonfinite or wrong-frame required input, invalid pose quaternion, invalid
weights or a failing callable produces a fresh zero twist each tick.
An entirely unused command source may be absent. Pose feedback is always required.
Fault transitions are logged, and `get_fault()` returns the current reason or
`None` after recovery. Shutdown cancels the periodic task, publishes zero and
releases subscriptions through `Module.stop()`.

## Pose-dependent weights

Define importable functions at module scope so blueprints can send them to workers:

```python
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.teleop.policy.module import PolicyTeleopModule
from dimos.teleop.policy.takeover import (
    distance_to_target,
    logarithmic,
    quadratic,
    reference_path_progress,
)


def distance_takeover(pose: Transform) -> float:
    progress = distance_to_target(pose, (0.4, 0.0, 0.4), outer_radius=0.3)
    return logarithmic(progress, k=9.0)


def path_takeover(pose: Transform) -> float:
    path = [(0.25, -0.1, 0.25), (0.4, -0.1, 0.35), (0.4, 0.0, 0.4)]
    return quadratic(reference_path_progress(pose, path))


blender = PolicyTeleopModule.blueprint(takeover_fn=distance_takeover)
```

`distance_to_target` maps the outer radius to zero and the inner radius to one,
clamping points beyond those radii. `reference_path_progress` projects onto the
nearest polyline segment and divides traveled arc length by total length.
Repeated points are ignored; a zero-length path is invalid. Self-intersections
select the first closest segment. Available curves are `linear`, `quadratic`,
`power(progress, q)` with `q > 0`, and `logarithmic(progress, k)` with `k > 0`.

## Simulation and plots

```bash
# Synthetic bottle-cap example: human lifts along world Z, policy rotates about Z.
uv run dimos run policy-teleop-sim

# Same simulation and sources, with only one source selected.
uv run dimos run demo-policy-teleop-human-sim
uv run dimos run demo-policy-teleop-policy-sim

# Headless operation (the simulation module's normal config override).
uv run dimos run policy-teleop-sim --mujocosimmodule.headless=true

# Four curves, a spatial path colored by weight, and three motion comparisons.
uv run python -m dimos.teleop.policy.demo_takeover --output /tmp/policy-teleop.png
```

Stop a running blueprint before starting another. Synthetic sources wait for TCP
feedback, perform one lift/rotate-and-return cycle, then publish zeros. Replace
`SyntheticVelocitySources` with real teleop and policy velocity producers when
integrating an application. The example only illustrates bottle-cap motion; it
does not grasp a physical cap or control the gripper.

The simulation blueprint connects coordinator TF through `TcpPoseAdapter`, with
`publish_frame_poses=True`, and sends only the blender's output to
`ArmTwistCoordinator.ee_twist_command`. `EEFTwistTask` performs integration and
Pink IK. It is the only motion task, so stopping cannot resume a parallel
trajectory. Its command timeout stops motion even if the blender disappears;
the coordinator's emergency stop remains authoritative. Adding parallel motion
tasks requires an explicit arbitration/hold design.

`LeRobotPolicyModule` currently sends joint trajectories directly to the
coordinator. It cannot be wired into this blender as-is; a separate Cartesian
velocity output and adapter belong upstream. Training and direct trajectory
blending are outside this module.

```bash
uv run pytest dimos/teleop/policy
# Requires xArm assets, Pink and MuJoCo; runs the actual coordinator and IK.
uv run pytest dimos/teleop/policy/test_simulation.py -m self_hosted
```
