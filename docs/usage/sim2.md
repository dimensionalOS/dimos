# robosuite in DimOS

DimOS runs a **robosuite environment**. Robosuite owns model composition, physics,
controllers, observations, reset and task behavior. DimOS connects that environment
to the existing robot application and its typed streams.

For a native Unreal RGB-D camera attached to the same world, see
[Unreal cameras for sim2](/docs/usage/sim2-unreal.md).

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
`reset()`; robosuite supplies the randomization. Independent lidar workers receive
the new model on reset, so use per-reset randomization with those workers;
model changes during an episode are not propagated to their compiled models.

## Add another robot

For a robot already supplied by robosuite, use its existing model/task directly.
For an existing DimOS hardware application:

1. Supply a robosuite `RobotModel` in the robot package, reusing the physical
   model asset. Existing upstream models need no new anatomy definition. When
   importing a raw MJCF, preserve and check its authored physics: robosuite
   changes some joint defaults unless the model restores them.
2. Add the connection's `simulated_robot()` binding. Its public motor order must
   match the model's actuator order; optional scales/offsets express hardware
   units. Declare native position-only joints so halt preserves their positions.
3. Reuse the same application blueprint. No robot-specific world class is needed.

`MotorController` derives joint/actuator indices and native servo behavior from
the compiled model. It preserves full PD plus feedforward commands; it does not
replace the external locomotion policy with an upstream position or IK controller.
Startup holds the model's initial configuration. After control begins, halt and expired commands use the binding's configured
damping until a new command or reset.

## Sensors and reset

Ordinary cameras and IMUs use robosuite `Observable`: sampling, corruption,
filtering and delay. Thin DimOS modules convert the latest sampled observation
into images, calibration, IMU and TF messages. Held observations retain their
acquisition timestamps and are not repeatedly republished. Rendering occurs on
the environment's owner thread, so camera rendering can slow environment stepping.
Configure upstream observable modifiers on the environment.

Rolling Mid360 acquisition remains a specialized worker: per-ray timing, motion
distortion, return modeling and the device's own IMU feed the unchanged Point-LIO
inputs. It restores authoritative snapshots and never advances a second world.
Its history/ray model is not replaced by a latest-value observation dictionary.

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

A stock robosuite task works through its native action interface. This does not
claim every RoboCasa task accepts the G1 motor binding: embodiment, action-space
and task compatibility still have to be established for each integration.
