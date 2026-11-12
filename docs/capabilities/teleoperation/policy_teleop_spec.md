# PolicyTeleopModule

## Goal

Continuously blend human teleop and policy commands for one end effector, using
pose-dependent weights. The MVP blends linear and angular velocities; pose and
trajectory sources require upstream adapters. Policy training, gripper control,
and direct whole-trajectory blending are outside scope.

## Interface and blending

All commands refer to the same TCP in the world frame, using m/s and rad/s.

| Port               | Type                | Meaning                    |
| ------------------ | ------------------- | -------------------------- |
| `current_pose`     | `In[Transform]`     | Measured world-to-TCP pose |
| `teleop_command`   | `In[TwistStamped]`  | Human velocity command     |
| `policy_command`   | `In[TwistStamped]`  | Policy velocity command    |
| `ee_twist_command` | `Out[TwistStamped]` | Blended velocity command   |

Pass `TakeoverFn(current_pose: Transform) -> float | Weights6` at construction.
It returns final policy weights in `[0, 1]`: `0` is human, `1` is policy.
A scalar applies to all axes; six weights correspond to `[vx, vy, vz, wx, wy, wz]`.
Rotation weights act on angular velocity, not quaternion components.

Each control tick computes `alpha = takeover_fn(current_pose)` and:

**`u_out[i] = (1 - alpha[i]) * u_teleop[i] + alpha[i] * u_policy[i]`**

Provide distance-to-target and reference-path progress examples that map the pose
to `lambda` in `[0, 1]`, then return `alpha = g(lambda)`:

| Curve       | `g(lambda)`                             |
| ----------- | --------------------------------------- |
| Linear      | `lambda`                                |
| Quadratic   | `lambda ** 2`                           |
| Power       | `lambda ** q`, `q > 0`                  |
| Logarithmic | `log1p(k * lambda) / log1p(k)`, `k > 0` |

`PolicyTeleopConfig` sets the loop frequency, input timeouts, world frame, and TCP.
Missing/stale required inputs, invalid values or frames, and callable failures
produce zero velocity and a reported fault. Entirely unused command sources may
be absent. Shutdown cleans up the loop and subscriptions.

## File organization

Proposed package: `dimos/teleop/policy/`.

| File                                                     | Responsibility                                                           |
| -------------------------------------------------------- | ------------------------------------------------------------------------ |
| `module.py`                                              | Config, streams, lifecycle, periodic blending, timeouts, and publication |
| `takeover.py`                                            | `TakeoverFn`, `Weights6`, curves, and spatial mapping helpers            |
| `blending.py`                                            | Pure `blend_twist(teleop, policy, weights)` and numerical validation     |
| `test_takeover.py`, `test_blending.py`, `test_module.py` | Numerical behavior and module lifecycle tests                            |
| `demo_takeover.py`                                       | Curve plots and physical-space trajectory visualization                  |
| `README.md`                                              | Interface conventions and usage                                          |

Robot-specific wiring belongs in
`dimos/robot/manipulators/xarm/blueprints/policy_teleop_sim.py`, following the
[existing simulation blueprint](/dimos/robot/manipulators/xarm/blueprints/hybrid_teleop_sim.py).
Keep robot configuration and policy-specific adapters outside the blender.

## Repository integration

- Feed the output to `ArmTwistCoordinator.ee_twist_command`; reuse
  [EEFTwistTask](/dimos/control/tasks/eef_twist_task/eef_twist_task.py) for integration and IK.
- Adapt world-to-TCP TF data into `current_pose`. Coordinator TF publication
  requires `publish_frame_poses=True` and a `tf` output.
- [LeRobotPolicyModule](/dimos/imitation/policy/lerobot/module.py) currently submits
  joint trajectories directly. It needs a separate output path and Cartesian
  adapter; begin the demo with synthetic velocity sources.
- Both sources must pass through the blender. Preserve coordinator emergency-stop
  behavior and prevent parallel motion tasks from resuming when blending stops.
- When implementing the module/blueprint, regenerate the registry with
  `uv run pytest dimos/robot/test_all_blueprints_generation.py`.

## Demo and acceptance

Show all four curves, a spatial path colored by policy weight, and human-only,
policy-only, and blended motion. Use a bottle-cap example: human-dominant lifting
along world Z and policy-dominant rotation about Z.

Verify exact outputs at weights `0` and `1`, component-wise averaging at `0.5`,
independent axis weights, curve endpoints, timeout/fault handling, and lifecycle
cleanup. Run the blended output through the actual coordinator and IK in simulation.
