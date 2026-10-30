# G1 with replaceable physics

One `G1Connection` works with hardware, MuJoCo or Genesis. Stock mode accepts
velocity commands; motor mode accepts `MotorCommandArray` from GR00T or another
controller. The connection keeps its streams, validation and deadman handling.

```text
G1Connection -> SimRobotClient -> SimulationModule -> PhysicsBackend
     |                                |                 |
     |                                +-- WorldState <--+
     |
     +-- robot topics --> Desktop
```

## Run G1

```bash
uv sync --extra sim --extra unitree

# Stock locomotion; same connection as stock hardware.
dimos --simulation mujoco run unitree-g1-control

# The real GR00T controller, motor connection and simulator.
dimos --simulation mujoco run unitree-g1-groot-control
```

Genesis requires PyTorch 2.8, while the repository's legacy ML test group pins
an older version. Its locked install uses a separate environment:

```bash
UV_PROJECT_ENVIRONMENT=.venv-genesis uv sync --no-default-groups --extra sim-genesis --extra unitree

.venv-genesis/bin/dimos --simulation genesis run unitree-g1-control
.venv-genesis/bin/dimos --simulation genesis run unitree-g1-groot-control
```

Both control blueprints publish the existing robot topics for Desktop and omit
Rerun. Native camera/lidar acquisition and the full mapping/navigation profile
currently require MuJoCo; selecting Genesis for those profiles reports the
unsupported capability. Renderer cameras can consume neutral body poses independently of the physics engine.

Source: [stock connection](/dimos/robot/unitree/g1/connection.py),
[GR00T composition](/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_groot_wbc.py).

## Description, state and commands

| Type | Meaning |
|---|---|
| `WorldDescription` | Stable body/joint IDs, relationships, motion types, drives and capabilities |
| `WorldState` | One tick's body poses/velocities and joint configuration/velocity/actuator effort |
| `CommandBatch` | Named joint drives, body wrenches or kinematic targets, with an epoch and duration |
| `CommandReport` | Accepted, rejected, applied or expired; includes the relevant physics tick |
| `VisualScene` | Static meshes, materials and body-local visual/camera attachments |

Body poses and velocities use world coordinates at the body origin. Quaternions
are xyzw; distances are metres and angles radians. A joint relates bodies: a
cabinet hinge and a robot elbow have the same physical meaning. A joint need not
have an actuator. Free-root motion is represented by the root body's state.

`JointDrive` targets a declared scalar actuator. `BodyWrench` applies force and
torque to a dynamic body, optionally at an offset point. `KinematicTarget`
prescribes an authored kinematic body's world pose; it never teleports a dynamic
robot. Genesis currently rejects kinematic targets. Unsupported operations fail
explicitly. Initialization/reset is separate from physical actuation.

```python
from contextlib import ExitStack

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.sim2.module import SimulationModule
from dimos.sim2.physics import BodyWrench, CommandBatch

with ExitStack() as cleanup:
    stack = autoconnect(G1Connection.blueprint()).global_config(
        simulation="mujoco", viewer="none"
    )
    app = ModuleCoordinator.build(stack)
    cleanup.callback(app.stop)
    sim = app.get_instance(SimulationModule)
    state = sim.read_state()
    pelvis = state.bodies["g1/pelvis"]
    knee = state.joints["g1/left_knee_joint"]

    receipt = sim.submit(CommandBatch(
        command_id="pelvis-push",
        epoch=state.metadata.episode_id,
        commands=(BodyWrench("g1/pelvis", force=(20.0, 0.0, 0.0)),),
        duration=0.1,
    ))
    assert receipt.status == "accepted"
    app.loop()  # Run until Ctrl-C; the force lasts 0.1 simulated seconds.
```

Acceptance queues a command; the `command_reports` stream reports application
at a physics tick. Zero duration means one step; positive duration uses simulation
time. Reset and pause/resume clear commands and invalidate their epoch. Multiple
body wrenches add.
An explicit scene joint drive overrides the robot's drive for that joint while
active. Actual movement is read from `world_state`; actuator effort excludes
contact/reaction forces and represents the last integration interval. A backend
that cannot report actuator effort returns `None`, including native robosuite tasks.

[`physics.py`](/dimos/sim2/physics.py) defines the contract.
[`SimulationRuntime`](/dimos/sim2/world/runtime.py) owns timing, command expiry,
stock policy execution and publications. Backends own model handles, physics
integration and native command translation. Engine-specific sensor checkpoints
remain private; they are not portable world state or cross-engine save files.

## Where the simulator attaches

After resolving configuration, the coordinator calls
[`attach_simulation()`](/dimos/sim2/composition.py). It collects every connection's
`describe_simulated_robot()` before constructing one shared world, then injects
one `SimRobotClient` into each retained connection. Namespaces distinguish robot
instances. A manually supplied empty `SimulationModule` is populated in place.

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.connection import G1Connection

two_g1s = autoconnect(
    G1Connection.blueprint(sim_position=(0, 0, 0)).namespace("left"),
    G1Connection.blueprint(sim_position=(2, 0, 0)).namespace("right"),
).global_config(simulation="genesis", viewer="none")
```

The stock ONNX policy consumes neutral `RobotState`. It never receives native
engine handles. GR00T remains an external controller using the same motor stream
on hardware and both engines. Stock simulation models locomotion; Unitree firmware
gestures and stand/lie transitions remain unsupported.

## Assets and current limits

Both adapters consume the existing G1 MJCF assets through one source-format
importer. Genesis uses MuJoCo for MJCF asset parsing only; Genesis owns and steps
the running physics state. Authored masses/inertias and robot placement are
preserved. Explicit stock foot/floor contact eligibility is lowered for Genesis,
whose importer otherwise ignores MJCF contact-pair declarations. Pair-specific
solver/friction overrides are not equivalent across engines.

Stock position-servo and external torque-control G1 plants remain distinct
existing assets. Consolidating them with the planning URDF is still unfinished.
The neutral interface does not claim that a different engine produces identical
physics, that every source format is supported, or that trained policies transfer
without validation. The current asset compiler accepts MJCF; URDF conversion is
not implemented here.

Native robosuite tasks remain a MuJoCo integration, with their own action vectors,
observations, rewards and episode semantics:

```bash
dimos run robosuite-lift
```

Their task API does not define the generic physics interface. Existing robosuite
camera/IMU Observables and rolling lidar workers remain available on MuJoCo.
RoboCasa and RL training workflows are not validated by this change.

## Reproduce the G1 checks

```bash
# Two stock G1s, velocity policy, body force, reset and external motor commands.
python -m dimos.robot.unitree.g1.demo_physics_conformance --engine mujoco
.venv-genesis/bin/python -m dimos.robot.unitree.g1.demo_physics_conformance --engine genesis

# Actual GR00T blueprint, workers, retained connection and simulation client.
python -m dimos.robot.unitree.g1.demo_physics_conformance --engine mujoco --scenario groot
.venv-genesis/bin/python -m dimos.robot.unitree.g1.demo_physics_conformance --engine genesis --scenario groot
```

These checks use the actual G1 assets and controllers. Reports contain measured
movement, upright height, command/feedback status and reset results; failed
checks return a nonzero exit code. Physical hardware was not exercised.
