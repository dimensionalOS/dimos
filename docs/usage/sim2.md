# robosuite in DimOS

DimOS runs a robosuite environment and connects it to typed streams. There are
two entry paths: an existing robosuite task, or an existing DimOS hardware
application whose motor device is replaced with a simulated device.

| Path | Reused from robosuite | Supplied by DimOS |
|---|---|---|
| Stock task, such as Panda Lift | Robot, task, controller, observations, reward and reset | Worker lifecycle, action/result streams and snapshots |
| Existing G1 application | `Task`/`RobotModel` composition and `MujocoEnv` stepping/reset | Existing G1 MJCF, GR00T policy, hardware PD semantics and connection binding |
| MuJoCo camera or standalone IMU | `Observable` sampling, corruption, filtering and delay | Sensor attachment and typed message conversion |
| Unreal camera and rolling Mid360 | Shared environment state | Independent capture, ray timing and message workers |

MuJoCo computes physics in both entry paths. The live G1/Unreal composition does
not use robosuite's stock task controllers or ordinary camera/IMU Observables.

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

## Run the existing G1 application

```bash
# Same application and connection, different motor device.
dimos run unitree-g1-groot-wbc --network-interface=enp2s0
dimos --simulation mujoco run unitree-g1-groot-wbc
```

```text
ControlCoordinator / existing GR00T policy
    -> G1WholeBodyConnection
        -> real mode: G1MotorDevice -> Unitree DDS
        -> simulation: SimulatedMotorDevice -> MotorScene (robosuite MujocoEnv)

MotorScene
    Task(Arena, G1Model)       # upstream composition
    MotorController          # exact q/dq/kp/kd/tau motor behavior
    Observable               # ordinary sensor acquisition
    -> feedback -> retained connection -> existing GR00T policy
    -> state snapshots -> rolling Mid360 -> Point-LIO -> mapping/planning
```

There is one robot-local simulation file,
[G1Model](/dimos/robot/unitree/g1/simulation.py). It subclasses robosuite's
`RobotModel`, loads the existing 29-DOF MJCF and uses upstream placement methods.
Motor names and the GR00T home pose/gains come from the existing hardware/control
definitions. The importer preserves the MJCF's physical parameters; robosuite's
default joint tuning is not allowed to alter the G1 model.

The connection attaches that model to DimOS using this actual binding:

```python
from functools import partial
from pathlib import Path

from dimos.robot.unitree.g1.simulation import G1Model, g1_model
from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnectionConfig
from dimos.sim2.spec import SimulatedRobot


def simulated_robot(config: G1WholeBodyConnectionConfig, scene: Path) -> SimulatedRobot:
    return SimulatedRobot(
        robot_id="g1",
        model=partial(g1_model, scene),
        joint_names=G1Model.public_motor_names,
        frame_id=config.frame_id,
        control_imu=G1Model.control_imu,
        startup_kp=G1Model.startup_kp,
        startup_kd=G1Model.startup_kd,
        wait_for_first_command=True,
    )
```

`SimulatedRobot` describes the **connection binding**: public motor names, units,
frames and startup/watchdog behavior. Geometry, joint anatomy and actuator behavior come from the existing MJCF;
robosuite supplies model composition and placement methods. There is no parallel `Joint`,
`RobotConfig`, `RobotInstance` or `WorldConfig` simulation model schema.

### Where simulation attaches

1. [ModuleCoordinator.build()](/dimos/core/coordination/module_coordinator.py)
   sees `--simulation mujoco` after configuration is resolved and calls
   [attach_simulation()](/dimos/sim2/composition.py).
2. The connection's
   [simulated_robot()](/dimos/robot/unitree/g1/wholebody_connection.py) hook
   supplies the model binding. Attachment adds `SimulationModule` with
   `MotorScene` and injects a `SimulatedDeviceFactory` into that connection's
   `simulation_device` config.
3. [ConnectionModule.connect()](/dimos/control/connection/connection_module.py)
   starts that injected device. Without simulation, it starts the hardware device.
   The connection keeps the same command validation, halt behavior and public ports.

Supported standalone sensor drivers are replaced under their existing instance
names/remaps. Unreal is attached separately with `UnrealCameraModule.blueprint()`;
it reads snapshots from `SimulationModule` and never creates or advances a world.
Automatic attachment currently supports one PD motor connection; explicit
`MotorScene` composition supports multiple models.

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

This shares the application, connection and control configuration across real
and sim. It does **not** yet make every robot description a single source:
G1's URDF and simulation MJCF remain separate existing assets. Adding a robot
without a suitable MJCF still requires creating and validating that physics
model; the binding does not generate it from the hardware connection.

`MotorScene` owns its controllers. The runtime configures public motor units on
those same controllers. `MotorController` derives joint/actuator indices and
native servo behavior from the compiled model. It preserves full PD plus feedforward commands; it does not
replace the external locomotion policy with an upstream position or IK controller.
Startup holds the model's initial configuration. After control begins, G1 halt
and expired commands damp the motors until a new command or reset.

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

The stock task example publishes its observation dictionary through
`environment_result`; it does not automatically expose every observation as a
Desktop camera topic. RoboCasa, demonstration collection and Gym/RL workflows
are not configured or validated by this stack. Their embodiment, action-space
and task compatibility still need integration; wrapping `MotorScene` does not
supply benchmark task rewards or success conditions.
