# sim2 core

This PR contains world descriptions, sampled world state, joint/body actions,
a physics backend interface, and sensor acquisition. G1, walking policies,
robot connections and ControlCoordinator integration are outside this PR.
The previous integrated prototype is preserved at `b91c563637`.

## What lives where

| File | Responsibility / public interface |
|---|---|
| `types.py` | `World`, `Instance`, `Drive`, sensor profiles; `Layout`, `WorldState`, `Frame`; `JointTargets`, `JointEfforts`, `Wrench`; `PhysicsBackend`, `SensorReader` protocols. |
| `runtime.py` | `Runtime(backend)`: `submit`, `advance`, `prepare`, `snapshot`, `capture`, `reset`, `place`. Serializes commands, expires them in simulation time and rejects stale epochs. No modules or transport. |
| `mujoco.py` | `MujocoBackend(world)` implements `PhysicsBackend`. Imports the shared URDF models and owns native physics data. Only implemented physics backend. |
| `module.py` | `SimModule`: DimOS lifecycle, physics pacing, stream inputs/outputs and scene RPCs. |
| `sensors.py` | `MujocoSensors(world, layout)` implements `SensorReader`. Reads copied state for RGB/depth and lidar; never advances dynamics. |
| `sensor_module.py` | `SimSensorModule`: schedules acquisition and publishes ordinary DimOS sensor messages. |

`robot/assets/model.py` adds model prefixing and fixed-mount resolution. The same
`RobotModel` can be supplied to existing `UrdfManager`; sim2 does not create a
second robot description or change UrdfManager's implementation.

## Description → array names → sampled values

```python
# World: what exists, its initial placement, drives and sensors.
world = World(instances=(...), sensors=(...), dt=0.002)
backend = MujocoBackend(world)
sim = Runtime(backend)

layout = backend.layout                 # stable names / array order
state = sim.snapshot()                  # values at one simulation instant
index = layout.bodies.index("box")
box_pose = state.poses[index]           # x,y,z,qx,qy,qz,qw, in world
joint_index = layout.joints.index("arm/elbow")
elbow_angle = state.position[joint_index]
```

`Instance` places a `RobotModel` in the world. Robots, furniture and loose objects
use the same representation: a box has one link; a door has links and a hinge;
a robot has links and driven joints. `floating=True` gives the root six free
degrees of freedom. `Drive` enables servo/effort control on a scalar joint.

`Layout` supplies `bodies`, `joints`, `drives`, `roots`, `imu_frames` and `revision`.
`WorldState` supplies:

```python
epoch, tick, sim_time, wall_time, revision
poses       # (bodies, 7): world-from-link xyz + quaternion xyzw
twists      # (bodies, 6): world-axis linear + angular velocity at link origin
position    # (joints,): m or rad
velocity    # (joints,): m/s or rad/s
effort      # (joints,): actuator N or Nm; not contact/constraint reaction
```

`Frame(state, imu)` carries one coherent sample plus mounted IMU specific force
and angular velocity. It is a data packet; a sensor's named coordinate frame is
a link/fixed mount in `RobotModel`. Neither is another scene description.
`WorldState` is observable truth, not a restorable native solver checkpoint.

## Apply actions and edit the scene

With `world` containing the named objects/joints:

```python
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.runtime import Runtime
from dimos.sim2.types import JointEfforts, JointTargets, Wrench

sim = Runtime(MujocoBackend(world))
sim.submit(JointTargets(sim.epoch, ("arm/elbow",), (0.5,)))
sim.advance(100)                        # 100 fixed physics steps
sim.submit(JointEfforts(sim.epoch, ("arm/elbow",), (1.0,)))
sim.submit(Wrench(sim.epoch, "box", force=(5.0, 0.0, 0.0), duration=0.1))
sim.advance(50)
state = sim.snapshot()

# Privileged scene operations, not robot actions:
sim.place("box", (1.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0))
sim.reset()
```

Joint commands drive a degree of freedom; a wrench applies world-axis force and
torque to a link, optionally at a world point. The physics engine resolves both
through constraints and contacts. Targets support position, velocity, kp, kd
and feedforward effort. Direct effort replaces the position servo until another
command or expiration. Expiration holds measured position. Reset/place change
the epoch, invalidate pending actions and sensor history, and restart sim time.
`place` accepts only a floating root and clears that root's velocity.

The backend interface is deliberately synchronous and in-process:

```python
layout: Layout
dt: float
targets(command: JointTargets) -> None
efforts(command: JointEfforts) -> None
hold(joints: tuple[str, ...]) -> None
validate_wrench(wrench: Wrench) -> None
step(wrenches: tuple[Wrench, ...]) -> None
snapshot(epoch: str, tick: int) -> Frame
reset() -> None
place(body: str, pose: tuple[float, ...]) -> None
```

## DimOS modules and sensors

Use the ordinary blueprint system; supply your `World` and sensor mount names:

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.sim2.module import SimModule
from dimos.sim2.sensor_module import SimSensorModule
from dimos.sim2.sensors import MujocoSensors

blueprint = autoconnect(
    SimModule.blueprint(world=world, backend=MujocoBackend, publish_hz=50),
    SimSensorModule.blueprint(
        world=world, reader=MujocoSensors,
        camera="arm/camera", lidar="arm/lidar", imu="arm/imu",
    ),
)
# blueprint.build().loop() in your application's entry point.
```

| Module | Inputs | Outputs | RPCs |
|---|---|---|---|
| `SimModule` | `targets: JointTargets`, `efforts: JointEfforts`, `wrench: Wrench` | `world_frame: Frame`, `world_layout: Layout`, `joint_state: JointState`, world→root TF | `describe()`, `state()`, `reset()`, `place(body, pose)`, `apply_wrench(wrench)`, `timing()` |
| `SimSensorModule` | `world_frame: Frame`; injected `SimulationSpec` for layout | `color_image`, `depth_image`, `camera_info`, `lidar`, `imu` | `statistics()` |

Sensor selection is explicit and optional: a lidar-only module needs only
`lidar="arm/lidar"`. Profiles in `World.sensors` configure rate/range/resolution;
the shared model defines each mount. One module publishes one bundle per type;
use ordinary module instances/remappings for additional bundles.

`SensorReader` exposes `observe(state)`, `camera(frame, state)`,
`lidar(frame, duration)`, `close()` and a `gaps` counter. IMU values come from the
physics sample. Lidar uses timed ray packets and state history, rejecting scans
with missing history. Its current `PointCloud2` output is in **world coordinates**.
Automatic real/sim sensor replacement and raw MID360/PointLIO parity are **not
implemented** by this PR.

Physics and sensors run in separate workers. MuJoCo sensors use a second native
model/data copy for queries, not another dynamics simulation. State arrays are
copied and transported; these interfaces are not zero-copy. Physics never waits
for rendering or a controller. Under load sim time slows rather than skipping
steps/resetting. Publication can drop old snapshots; acquisition reports gaps.

## Verification and limits

```bash
uv run pytest dimos/sim2 dimos/robot/assets/test_model.py
uv run pytest dimos/sim2/test_module.py -m native_e2e
```

Fixtures are temporary URDFs: two independent pushers, a loose box and a hinged
door. Checks cover joint servos/efforts, force-at-point, contacts, expiry/reset,
shared URDF mounts, sensor acquisition, existing EvalRunner scoring, and real
DimOS workers progressing with sensors through force application and reset.
No downloaded robot assets or new demo/lab framework.

Only fixed and independent scalar URDF joints are supported (no mimic joints).
A replacement physics backend must meet the same behavioral contract; no second
backend or cross-engine equivalence is verified. Sensor realism, controller
scheduling, physical hardware interchange and Desktop acceptance remain separate
work. This core is not a runnable G1 stack.
