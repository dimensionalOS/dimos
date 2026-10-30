# robosuite in DimOS

DimOS runs a robosuite environment and connects it to typed streams. There are
two entry paths: an existing robosuite task, or an existing DimOS hardware
application whose motor device is replaced with a simulated device.

| Path | Reused from robosuite | Supplied by DimOS |
|---|---|---|
| Stock task, such as Panda Lift | Robot, task, controller, observations, reward and reset | Worker lifecycle, action/result streams and snapshots |
| G1 stock or external motor control | Model composition and environment stepping/reset | Stock velocity policy or external controller, shared connection and client |
| MuJoCo camera or standalone IMU | `Observable` sampling, corruption, filtering and delay | Sensor attachment and typed message conversion |
| Unreal camera and rolling Mid360 | Shared environment state | Independent capture, ray timing and message workers |

MuJoCo computes physics in both entry paths. The G1 locomotion policies are separate from robosuite task controllers; Unreal
and rolling Mid360 acquisition do not use ordinary camera/IMU Observables.

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

## One G1 connection, two control modes

```python
from dimos.robot.unitree.g1.connection import G1Connection

stock = G1Connection.blueprint()                        # cmd_vel + vendor RPCs
stock_dds = G1Connection.blueprint(stock_transport="dds")
motors = G1Connection.blueprint(control_mode="motors")  # external PD commands
```

The class is the same on hardware and in simulation. Stock mode uses the Unitree
WebRTC or DDS locomotion API. Motor mode uses DDS motor IO. GR00T is an external
controller in the application, not a connection mode or a default robot policy.
Its blueprint explicitly supplies its initial pose/gains.

```bash
# Stock firmware interface, hardware:
dimos --viewer none run unitree-g1-basic --g1connection.stock-transport dds

# Same connection and stock-style velocity interface, shared simulation:
dimos --simulation mujoco --viewer none run unitree-g1-basic-sim

# External GR00T motor controller, hardware or simulation:
dimos --viewer none run unitree-g1-groot-wbc
dimos --simulation mujoco --viewer none run unitree-g1-groot-wbc
```

```text
stock application -> G1Connection(stock) -> hardware WebRTC/DDS
                                        -> SimRobotClient.set_velocity
                                             -> shared world + stock velocity policy

external controller -> G1Connection(motors) -> hardware motor DDS
                                          -> SimRobotClient.set_motor_command
                                               -> shared world
```

Stock simulation reuses the existing G1 ONNX locomotion policy. This is a learned
surrogate for body-velocity commands, not execution of Unitree firmware. Vendor
arm gestures, stand/lie transitions and firmware mode requests are unsupported
in simulation and return an explicit error. They do not report simulated success.
The stock policy does not load GR00T. Each instance owns its policy state; no
process-global MuJoCo control callback is used.

### Where simulation attaches

1. [ModuleCoordinator.build()](/dimos/core/coordination/module_coordinator.py)
   resolves configuration and calls
   [attach_simulation()](/dimos/sim2/composition.py) for `--simulation mujoco`.
2. The collector calls `describe_simulated_robot(resolved_config)` on every
   connection satisfying `RobotConnectionSpec`. The returned `RobotInstanceSpec`
   describes the asset, instance pose, command interface and optional stock
   locomotion controller. It contains no robosuite model object.
3. All instances are assembled into one world. Each retained connection receives
   a `SimRobotClient` factory for its own command/feedback topics. The client uses
   the selected transport, Zenoh by default, without becoming another module.
4. Camera/lidar workers attach to that world. Standalone sensor-driver bindings
   retain their names/remaps. Explicit simulated sensor workers are collected
   without duplication. Unreal independently reads snapshots.

Source: [G1 connection](/dimos/robot/unitree/g1/connection.py),
[G1 model declarations](/dimos/robot/unitree/g1/model.py),
[shared model importer](/dimos/sim2/model.py).

### Multiple instances

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.connection import G1Connection


two_g1s = autoconnect(
    G1Connection.blueprint(sim_position=(0.0, 0.0, 0.0)).namespace("left"),
    G1Connection.blueprint(sim_position=(2.0, 0.0, 0.0)).namespace("right"),
).global_config(simulation="mujoco", viewer="none")
```

Namespaces separate public streams, private client topics and model identities.
Sensor ownership must be unambiguous; two robots do not implicitly share every
sensor. The simulator buffers each robot's latest command and applies it on its
next world tick. Command expiry stops stock velocity commands or damps externally
controlled effort motors.

### Robot descriptions and adding a robot

A new robot provides its model asset and connection description; the shared
importer handles robosuite naming, mesh resolution and preservation of authored
joint/material settings. No robot-specific simulator or robosuite subclass is
required. A velocity interface additionally needs a locomotion controller; a
motor interface receives commands from the application's controller.

The G1 description reuses public motor names and existing assets. Its two existing
policy plants remain explicit: the stock velocity policy uses its position-servo
MJCF, while external motor control uses the torque-actuated MJCF. Neither is
regenerated from the planning URDF yet. These models have differing revisions,
masses and joint offsets; consolidating them requires choosing authoritative
anatomy and validating both policies. This integration does not claim that the
physical model has become a single source of truth.

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
