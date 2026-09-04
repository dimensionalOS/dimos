# MicroDuck MuJoCo simulation

`microduck-sim` is the experimental, simulation-only MicroDuck stack. It runs
Pollen Robotics' official walking-mode MJCF and seven non-roller ONNX policies
through the existing `MujocoSimModule` and `ControlCoordinator`. Rerun provides
the interactive view; no physical MicroDuck is supported yet.

## Features

| Capability | Behavior |
|---|---|
| Locomotion | Walk, reverse, strafe, turn, stand, and recover |
| Head control | Joint-space head pose or gaze toward a trunk-frame point |
| Body and posture | Standing height/roll/pitch offsets, sit, and rise |
| One-shot motions | Ground pick, left kick, right kick, and forward roll |
| Simulation | Head RGB/depth camera, joint state, IMU, odometry, and reset |
| Visualization | Animated official CAD and optional cooked scene in Rerun |
| Teleoperation | Rerun WASD, standalone keyboard, or `Twist` stream |

Roller policies, mouth/audio features, physical sensors, policy reload, and real
hardware control are out of scope. The policies control 14 joints; the fifteenth
physical servo, the mouth, is deliberately absent.

## Run

Install the required extras:

```sh skip
uv sync --extra cpu --extra sim --extra visualization
```

Start the basic simulation:

```sh skip
uv run dimos --simulation mujoco --viewer rerun run microduck-sim
```

Add the office scene:

```sh skip
uv run dimos --simulation mujoco --viewer rerun \
  --scene-package office run microduck-sim
```

Run without Rerun:

```sh skip
uv run dimos --simulation mujoco --viewer none run microduck-sim --daemon
uv run dimos status
uv run dimos log -f
```

Stop a foreground run with Ctrl-C, or stop any registered run with:

```sh skip
uv run dimos stop
```

MuJoCo is always headless in this blueprint. `--viewer` controls DimOS
visualization, not MuJoCo's native window. A selected scene package is composed
into the same physics model and its cooked GLB is logged to Rerun. The first
office launch can be slower while its assets are extracted and compiled.

## Drive and inspect

Focus the DimOS Rerun viewer and use W/S for forward/reverse, Q/E for strafe,
A/D for yaw, and Space to stop. Viewer commands and other velocity producers
share `/microduck/cmd_vel`:

| Axis | Range |
|---|---:|
| Forward `vx` | `[-0.4, 0.4] m/s` |
| Left `vy` | `[-0.3, 0.3] m/s` |
| Left yaw `yaw_rate` | `[-1.0, 1.0] rad/s` |

Commands expire after 500 ms. Continuous publishers should refresh them at
10-50 Hz. `stop_motion` clears velocity immediately but does not cancel an
active one-shot motion.

Open a second terminal and attach the shell:

```sh skip
uv run dimos shell
```

Use the policy task through `ControlCoordinator`:

```python skip
cc = app.get_module("ControlCoordinator")

cc.describe_task("microduck_policy")
cc.task_invoke("microduck_policy", "get_status")
cc.task_invoke("microduck_policy", "list_skills")

cc.task_invoke(
    "microduck_policy",
    "set_head_pose",
    {"neck_pitch": 0.0, "head_pitch": -0.25, "head_yaw": 0.35, "head_roll": 0.0},
)
cc.task_invoke(
    "microduck_policy",
    "look_at",
    {"x": 1.0, "y": 0.3, "z": 0.0, "neck_pitch": 0.0},
)
cc.task_invoke(
    "microduck_policy",
    "set_body_pose",
    {"z": -0.01, "roll": 0.1, "pitch": 0.0, "active": True},
)
cc.task_invoke("microduck_policy", "set_posture", {"posture": "sit"})
cc.task_invoke("microduck_policy", "set_posture", {"posture": "stand"})
cc.task_invoke("microduck_policy", "run_skill", {"name": "kick_left"})
cc.task_invoke("microduck_policy", "stop_motion")
```

Available one-shots are `ground_pick`, `kick_left`, `kick_right`, and `roulade`.
Sit/stand uses `set_posture` so repeated requests are safe no-ops rather than
toggles.

The global E-stop remains a coordinator operation. Clearing it does not re-arm:

```python skip
cc.set_estop(True)
cc.set_estop(False)
cc.set_activated(True)
```

Reset physics and policy history together:

```python skip
cc.set_activated(False)
app.get_module("MujocoSimModule").reset()
cc.reset_runtime_state(reactivate=True)
```

## Public contract

Stable deployment names and streams:

| Item | Name |
|---|---|
| Blueprint | `microduck-sim` |
| Coordinator | `ControlCoordinator` |
| Task type and instance | `microduck_policy` |
| Hardware ID | `microduck` |
| Velocity input | `/microduck/cmd_vel` (`Twist`) |
| Joint output | `/microduck/joints` (`JointState`) |
| IMU output | `/microduck/imu` (`Imu`) |
| Root pose output | `/microduck/odom` (`PoseStamped`) |

The task consumes `twist_command` through `on_twist_command` and exposes:

| Command | Contract |
|---|---|
| `start` | Activate and auto-arm for this simulation |
| `arm`, `disarm` | Idempotently enable or suppress policy output |
| `stop_motion` | Clear requested and smoothed velocity |
| `set_head_pose` | Latch four finite policy-space head offsets |
| `look_at` | Solve head offsets for a trunk-frame point |
| `set_body_pose` | Enable or clear standing Z/roll/pitch offsets |
| `set_posture` | Request `sit` or `stand` |
| `run_skill` | Start an available one-shot policy |
| `list_skills` | Return accepted one-shots and manifest metadata |
| `get_status` | Return a lock-consistent JSON snapshot |
| `reset_runtime_state` | Clear recurrent state and optionally re-arm |

Intent commands return `{"accepted": bool, "reason": str | null}`. `look_at`
also reports whether the target was clamped and the applied head values.
`get_status` reports activation, arming, E-stop, busy state, current policy,
posture, active skill, applied velocity, command age, and last error.

There is intentionally no task-local E-stop, `move`, policy reload, or drive-mode
RPC. Use `ControlCoordinator.set_estop()` for global safety and the velocity
stream for locomotion. Policies are pinned and loaded at startup; changing them
requires a new asset revision and process restart. Roller mode requires a
different physical model and therefore a separate blueprint.

## Build contract

- `ControlCoordinator` owns the passive `MicroDuckPolicyTask`, ticks it at
  50 Hz, and arbitrates all 14 joints at priority 50 in servo-position mode.
- `MujocoSimModule` owns headless physics, sensors, scene composition, and the
  0.005 s MuJoCo timestep. The adapter only translates shared-memory state and
  native position targets.
- The policy task owns all ONNX sessions and shared action/filter history. RPCs
  only update locked intent state; inference runs only from `compute()`.
- Rerun consumes odometry and joint state to animate the official MJCF meshes.
  Its native and web keyboard outputs are remapped to the same velocity input.

Scheduler priority on every tick is: active one-shot, ground pick, sit/rise,
stand/recover, then walk. Stale or near-zero velocity selects stand. Ordinary
policy switches retain previous raw action and filtered targets; disarm, E-stop,
and runtime reset clear them.

Every ONNX policy has one float32 `[1, 61]` input and one `[1, 14]` output:

| Observation slice | Value |
|---|---|
| `0:3` | Trunk-frame gyroscope |
| `3:6` | Projected gravity |
| `6:20` | Joint position minus HOME |
| `20:34` | Joint velocity |
| `34:48` | Previous raw action |
| `48:61` | Velocity, head, and body command block |

Exact joint order, HOME values, `RobotSimSpec`, and asset paths live in
[`dimos/robot/pollen/microduck/config.py`](/dimos/robot/pollen/microduck/config.py).
They are policy ABI and must not be reordered or tuned independently. Model
names, action scales, durations, and one-shot encodings come from the bundled
manifest rather than a second hardcoded table.

Startup must fail on missing assets, invalid manifests, wrong tensor shapes,
missing MJCF bindings, or failed warm-up. Missing initial state emits no target.
A runtime inference error or non-finite observation/action disarms the task and
records `last_error`. E-stop clears all pending motion within one coordinator
tick and requires explicit re-arming.

## Sources and implementation

Authoritative upstream sources:

| Source | Responsibility |
|---|---|
| [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) | Runtime intent and safety semantics |
| [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) | MJCF, training setup, and policy ABI |
| [pollen-robotics/microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies) | Official ONNX policies and manifest |
| [mujocolab/mjlab](https://github.com/mujocolab/mjlab) | Upstream training framework; not a runtime dependency |

The LFS bundle contains only the pinned walking MJCF, required meshes, seven
non-roller policies, manifest, licenses, and provenance. Runtime operation must
not depend on developer clones or network downloads.

Key implementation files:

| File | Responsibility |
|---|---|
| [`dimos/control/tasks/microduck_policy_task/microduck_policy_task.py`](/dimos/control/tasks/microduck_policy_task/microduck_policy_task.py) | Policy validation, scheduler, inference, RPCs, and safety state |
| [`dimos/robot/pollen/microduck/blueprints/simulation.py`](/dimos/robot/pollen/microduck/blueprints/simulation.py) | Blueprint, streams, viewer, and scene composition |
| [`dimos/simulation/adapters/whole_body/microduck.py`](/dimos/simulation/adapters/whole_body/microduck.py) | Shared-memory whole-body adapter |
| [`dimos/robot/pollen/microduck/rerun.py`](/dimos/robot/pollen/microduck/rerun.py) | MicroDuck-specific Rerun bindings |
| [`dimos/visualization/rerun/mjcf_robot.py`](/dimos/visualization/rerun/mjcf_robot.py) | Generic MJCF-to-Rerun visualization |

## Verify

```sh skip
uv run pytest dimos/control/tasks/microduck_policy_task \
  dimos/simulation/adapters/whole_body \
  dimos/robot/pollen/microduck -v
uv run pytest -m mujoco dimos/robot/pollen/microduck -v
uv run pytest dimos/robot/test_all_blueprints_generation.py
```

This is a functional native-actuator simulation baseline, not a sim-to-real
fidelity or physical-hardware safety claim.
