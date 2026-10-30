# robosuite in DimOS

DimOS runs a robosuite environment and connects it to typed streams. There are
two entry paths: an existing robosuite task, or an existing DimOS hardware
application whose motor device is replaced with a simulated device.

| Path | Reused from robosuite | Supplied by DimOS |
|---|---|---|
| Stock task, such as Panda Lift | Robot, task, controller, observations, reward and reset | Worker lifecycle, action/result streams and snapshots |
| Hardware-style robot control | Model composition and environment stepping/reset | Stock velocity policy or external controller, shared connection and client |
| MuJoCo camera or standalone IMU | `Observable` sampling, corruption, filtering and delay | Sensor attachment and typed message conversion |

MuJoCo computes physics in both entry paths. Hardware-style locomotion policies are independent of robosuite task controllers.

## Use an existing robosuite task

This is the shipped `robosuite-lift` example:

```python
from dimos.sim2.module import SimulationModule
from dimos.sim2.rendering import make_environment


def panda_lift():
    return make_environment(
        "Lift",
        robots="Panda",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        control_freq=20,
        horizon=1000,
    )


robosuite_lift = SimulationModule.blueprint(environment=panda_lift)
```

```bash
dimos run robosuite-lift
```

`make_environment()` creates the upstream task and selects a headless rendering
context that works inside a worker, including native CGL on macOS. It leaves the
original task class untouched. The factory executes inside the worker. The module
receives native actions on
`actions: In[list[float]]` (or the `set_action()` RPC) and publishes
`environment_result: Out[EnvironmentStep]` containing the upstream observation
dictionary, reward, done and info. `reset()` calls the upstream reset. The
original task's control frequency and horizon are preserved; a terminated task
waits for reset. No new robot definition or task implementation is required.

Source: [robosuite_lift.py](/dimos/robot/robosuite/blueprints/robosuite_lift.py).

Use robosuite's own domain-randomization wrapper in that factory:

```python
from robosuite.wrappers import DomainRandomizationWrapper

from dimos.robot.robosuite.blueprints.robosuite_lift import panda_lift
from dimos.sim2.module import SimulationModule


def randomized_panda_lift():
    return DomainRandomizationWrapper(
        panda_lift(),
        randomize_camera=True,
        randomize_lighting=True,
        randomize_dynamics=True,
        randomize_color=False,  # upstream texture randomizer requires MuJoCo 3.1.1
        randomize_on_reset=True,
        randomize_every_n_steps=0,
    )


randomized_lift = SimulationModule.blueprint(environment=randomized_panda_lift)
```

The same wrapper can wrap `MotorScene`. DimOS calls the wrapper's `step()` and
`reset()`; robosuite supplies the randomization. Snapshot workers receive new
models on reset; model changes during an episode are not propagated. These
modders do not control Unreal's native materials or lighting. Camera changes
can reach Unreal through named model cameras; its fixed observer mount is
configured separately.

## Attach robot connections

After resolving configuration, `ModuleCoordinator.build()` calls
`attach_simulation()` for `--simulation mujoco`. It collects every
`RobotConnectionSpec.describe_simulated_robot(resolved_config)` result into one
world. Descriptions contain model assets, instance placement and either a motor
command interface or a velocity interface with a locomotion controller.

The retained connections receive non-module `SimRobotClient` instances using
the selected transport (Zenoh by default). Namespaces separate robot identities,
private command topics and public feedback. A manually supplied empty
`SimulationModule.blueprint()` is populated instead of adding another world.
Explicit sensor workers are collected without duplication; replaced hardware
sensor workers retain their original names and remaps.

The shared MJCF importer preserves authored dynamics and appearance. Robot
packages supply data; no robot-specific simulator or robosuite subclass is
required. Existing stock robosuite tasks still use their own native actions.

## Sensors and reset

Ordinary cameras and IMUs use robosuite `Observable`: sampling, corruption,
filtering and delay. Thin DimOS modules convert the latest sampled observation
into images, calibration, IMU and TF messages. Held observations retain their
acquisition timestamps and are not repeatedly republished. Rendering occurs on
the environment's owner thread, so camera rendering can slow environment stepping.
Configure upstream observable modifiers on the environment.

Lidar workers restore authoritative world snapshots without advancing physics.
Their ray models can retain acquisition history independently of Observables.

Reset runs on the environment owner thread, drops old commands, advances the
sensor epoch and refreshes worker model files. This handles upstream hard resets
and model randomization without restoring snapshots into an obsolete model.
Resetting application maps, planner goals or policy memory remains the
application's responsibility.

## Dependencies and checks

```bash
uv sync --extra sim
uv run pytest dimos/sim2
uv run pytest -m self_hosted dimos/sim2  # native renderer checks
```

Published robosuite 1.5.1 is pinned: 1.5.2's Mink pin conflicts with DimOS's NumPy 2
and Rerun dependencies. MuJoCo is constrained to `>=3.5,<3.10` for ray normals and
the controller API supported by this robosuite release. Normal dependency
resolution is used, without Git dependencies or dependency overrides. Texture
randomization is unavailable with this version pair; camera, lighting and dynamics
randomization are tested. macOS workers use `spawn` because native MuJoCo graphics
can abort in forked processes; Linux workers retain `forkserver`.

The stock task example publishes its observation dictionary through
`environment_result`; it does not automatically expose every observation as a
Desktop camera topic. RoboCasa, demonstration collection and Gym/RL workflows
are not configured or validated by this stack. Their embodiment, action-space
and task compatibility still need integration; wrapping `MotorScene` does not
supply benchmark task rewards or success conditions.
