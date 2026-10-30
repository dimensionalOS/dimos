# sim2: G1 at a desk

Prototype branch `pim/feat/sim2-minimal`, built from `main@64b2d8aa6b`.
The main example is a **drivable G1, a desk/computer/chair from the existing office
assets, and two loose boxes**. Small synthetic worlds exist only in pytest fixtures.
The previous sim2 PR stack and running deployments are unchanged.

## Run

```bash
cd /Users/pimvandenbosch/Desktop/dimos-sim2-minimal
source .venv/bin/activate
# Fresh checkout: uv sync --all-groups --extra cpu

dimos run unitree-g1-sim2
```

Use the normal lifecycle (`--daemon`, `dimos status`, `dimos shell`, `dimos stop`).
No run.sh. If another blueprint already owns the default bus, stop it yourself
before starting this one. This change did not stop the existing G1 run.
The `g1_urdf` and `g1_desk` LFS packs provide meshes/textures;
`groot/balance.onnx` and `groot/walk.onnx` provide the GR00T WBC weights.
They resolve through `LfsPath` on a fresh checkout.

Scene content is one `data/.lfs/g1_desk.tar.gz` archive (three derived meshes and
three unchanged office textures). Small `desk.urdf` and `box.urdf` descriptions
stay reviewable under `assets/g1_desk/`, beside their preparation sources.
The G1 runtime has no dependency on test fixtures or the full office asset pack.

Open **Controller in DimOS Desktop** on the same bus. Use **world** as fixed frame,
**g1/pelvis** as robot/follow frame, and the **humanoid** drive profile. WASD/arrows
use `/cmd_vel`; Q/E strafe. Release the key or press Space to stop the requested
velocity; the balancing policy continues. Velocity commands expire after .3 s.
Prototype limits are .5 m/s forward/backward, .3 m/s lateral and .6 rad/s yaw.
There is no navigation planner or collision avoidance in this example: driving
into furniture can still make the robot fall.

Camera, depth, lidar and IMU are on `/color_image`, `/depth_image`, `/lidar_raw`
and `/imu`. RGB/depth are scheduled at **15 FPS**, 320×240, in simulation time;
Desktop's wall-time FPS also depends on simulation speed. `/camera_info` supplies
calibration. The camera and lidar are mounted
on the torso/head; the policy IMU is mounted on the pelvis. All frames share the
**world** root. There is no separate `odom` TF tree; `/odom` is expressed in world.
The G1 model uses the existing `/render_file_request` → `/file_info` protocol.
Furniture has separate URDF request channels; the current Controller's single
robot-model loader does not automatically draw those meshes. Camera and lidar see it.

Desktop's existing session remains pointed at the previous checkout. Its gateway
can consume normal streams, but choose this checkout in Desktop's DimOS settings
if launching the blueprint from Desktop rather than the terminal. Desktop UI
acceptance is **not claimed**: tests used a private domain/discovery bus, which
this installed Controller does not support. Do not use those test isolation flags
for the ordinary Desktop run. Native camera renders and real stream/control
integration have been verified.

In `dimos shell`:

```python
app.ControlCoordinator.control_status()  # feedback clock, computations, timing/error
app.ControlCoordinator.task_invoke("groot_wbc", "state_snapshot")  # balance/walk, arming
app.SimModule.timing()
app.SimSensorModule.statistics()
app.SimModule.reset()  # restore G1 and objects; reset policy history via a fresh epoch

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
app.G1Connection.move(Twist(linear=Vector3(.15, 0, 0)), duration=2)

# Move an arm while GR00T keeps balancing/walking. Trajectory motion is rate limited.
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
app.ControlCoordinator.execute_trajectory(JointTrajectory(
    joint_names=["g1/left_elbow_joint"],
    points=[TrajectoryPoint(positions=[0.8])],
))

from dimos.sim2.types import Wrench
state = app.SimModule.state()
app.SimModule.apply_wrench(Wrench(state.epoch, "parcel/box", (1, 0, 0), duration=.1))
```

## What the coordinator does

```text
Desktop /cmd_vel → G1Connection.move → ControlCoordinator.set_velocity_command
                                             │
                            GR00T legs/waist + arm trajectory task
                            existing per-joint arbitration + holds
                                             │ MotorControl
                                             ▼
                                       G1Connection
                                       SimRobotClient
                                             │ JointTargets (q/dq/kp/kd/tau)
                                             ▼
                                          SimModule
                                          PhysicsBackend
                                          (MuJoCo today)
                                             │ Frame + Layout
                         G1Connection ◀───────┘
                              │ ControlFeedback (joint state + pelvis IMU)
                              └──────────► ControlCoordinator
```

**ControlCoordinator runs the existing GR00T task and arm trajectory task.**
GR00T claims the 15 leg/waist joints; the arm task can claim the other 14.
Unclaimed joints hold their last commanded or initial measured position.
There is no G1SimDevice and no policy inside the connection or physics.

**ModuleCoordinator** is the existing deployment machinery: it builds workers,
wires streams and manages lifecycle. Physics, control, connection and sensor
acquisition have separate dedicated workers. Four UrdfManagers serve scene models.

The control configuration adds `feedback_robot="g1"` and a named `motor_gains`
map in motor order. It runs once per coherent `ControlFeedback(epoch, tick, state)`
using `CoordinatorState.t_now/dt` from simulation. `MotorControl` carries that same
epoch/tick, joint names and the existing `MotorCommandArray`; a late response is
never relabelled with newer feedback. Duplicate samples resend the same reply,
without another inference. Reset clears policy history, trajectories, holds and
latched velocity. Velocity expiry uses **wall time**, including while sim is slow.
A control-task failure stops producing replies and is visible in `control_status()`.

One feedback-clock coordinator currently controls one robot; multiple robot
coordinators can share the same world's control barrier. This is an explicit
prototype limit. The existing wall-clock/adaptor/per-robot-port mode still works.
Motor ports now accept configured position servos and hold unclaimed joints;
unsupported task control modes are rejected. A separately authored controller can
send all five motor fields through MotorControl; physics evaluates the PD law
every integration step and clamps resulting effort at the URDF actuator limits.

`G1Connection` selects I/O independently of GR00T:

| connection_type | Motor authority / interface |
|---|---|
| `webrtc` | Stock Unitree firmware; `move` and vendor RPCs work as before |
| `dds` | External hardware controller via `motor_command`; existing DDS driver, soft start and sport-mode handoff |
| `sim2` | External controller via `motor_control`; `move` forwards to the injected velocity controller |

The old second `G1WholeBodyConnection` module is removed; its existing DDS logic
is now `G1DDSDevice`, owned by G1Connection. Existing hardware blueprints select
`connection_type="dds"`; SDK settings are nested under `dds`. DDS mode does not
also subscribe to firmware velocity requests. Physical hardware is **not tested**
by this change. GR00T is not an emulation of Unitree's stock firmware.
The legacy `G1SimConnection` belongs to the earlier simulator and remains outside
this minimal blueprint; this change does not migrate those older simulation stacks.

## Where simulation attaches

Read [the blueprint](/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_sim2.py):

```python
unitree_g1_sim2 = autoconnect(
    SimModule.blueprint(world=G1_DESK_WORLD, backend=MujocoBackend,
                        control_joints=all_g1_motor_names),
    G1Connection.blueprint(connection_type="sim2"),
    ControlCoordinator.blueprint(
        feedback_robot="g1", model=g1_model, motor_gains=g1_gains,
        tasks=[groot_task, arm_trajectory_task],
    ),
    SimSensorModule.blueprint(world=G1_DESK_WORLD, reader=MujocoSensors, prefix="g1"),
    # Existing UrdfManager for each instance's exact same model.
    ...,
)
```

`G1_DESK_WORLD` explicitly places one `G1_SENSOR_MODEL`, the desk and two boxes.
There is no hidden blueprint replacement or automatic coordinator discovery.
`G1_MODEL` wraps the existing **g1.urdf** and is also used by the planning model.
Sensor mounts are shared `RobotModel.with_fixed_frame()` additions. The same
materialized URDF feeds physics and UrdfManager; no parallel robot MJCF is loaded.

The walking drive profile adds gains, rotor inertia, friction and initial angles;
it does not duplicate geometry, kinematics, inertia tensors or joint limits.
Gains and initial angles come from the task's shared `profile.py`; URDF limits
prevail. The policy observes the pelvis, as in the
[upstream GR00T deployment](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/main/decoupled_wbc/sim2mujoco/scripts/run_mujoco_gear_wbc.py).
Torso IMU feedback is not interchangeable: the waist moves relative to the base.
The URDF's mass, contact geometry and some effort limits differ from the GR00T
reference MJCF. The tests establish prototype usability with this model, not
training-model equivalence or sim-to-real transfer.

## Core ownership and data

| File | Owns |
|---|---|
| `sim2/types.py` | Portable world/instance configuration, named layout, sampled physical state and commands |
| `sim2/runtime.py` | Single physics owner, queued actions, expiry, reset/epoch rejection |
| `sim2/mujoco.py` | URDF compilation and MuJoCo dynamics only |
| `sim2/module.py` | Normal module lifecycle, pacing and stream publication |
| `sim2/sensors.py` | Read-only MuJoCo query/render replica; no dynamics stepping |
| `sim2/sensor_module.py` | Camera/lidar acquisition worker, standard DimOS outputs |
| `control/feedback.py` | Coherent feedback/reply envelopes around existing control state and MotorCommandArray |
| `control/coordinator.py`, `tick_loop.py`, `connection_source.py` | Feedback clock, existing arbitration, holds and motor output |
| `sim2/client.py` | Map simulator arrays and forward motor commands with their original epoch/tick |
| `robot/unitree/g1/connection.py` | One public robot connection; select WebRTC, DDS or sim I/O |
| `robot/unitree/g1/dds_device.py` | Existing low-level Unitree driver, extracted without policy logic |
| Existing `control/tasks/g1_groot_wbc_task/` | GR00T observation/action convention, history, balance/walk inference and shared motor profile |
| `robot/unitree/g1/model.py` | Canonical G1 description, shared mounts and walking drive profile |
| Existing `robot/urdf_manager.py` | Internal TF edges and model/mesh serving |

`WorldState` contains epoch, tick, sim/wall time, model revision, world link poses
(xyz/xyzw), world twists at link origins, scalar joint position/velocity and applied
actuator effort. `Layout` names the array order. IMU measurements accompany it in
`Frame`; images and scans remain separate sensor messages.

JointTargets includes optional desired velocity, kp/kd and feedforward effort.
Its servo law is `kp*(target_position-q) + kd*(target_velocity-dq) + feedforward`.
Omitted fields use zero velocity/feedforward and World Drive gains.
Joint position targets/efforts actuate joints; `Wrench` applies a force/torque to a
body for a finite interval. A point is in world coordinates; omitting it uses the
link origin. `place()` is a privileged free-root teleport, not a force command.
Reset/place invalidate queued commands and sensor histories using a new epoch.

For this G1 blueprint, physics advances ten 2 ms steps per policy response. At
the next boundary it waits for targets bearing that exact epoch and tick. This
also gates startup until the policy is ready. Missing/slow inference **pauses sim
time**, rather than stepping indefinitely with old actions or resetting the robot.
The gate is joint-generic and optional; it contains no policy callback. It costs
latency and synchronizes the whole world, so it is not a real-time hardware loop
or a claimed batched training API. Wall-time scheduling is best effort.

Only MuJoCo is implemented. Changing engine means implementing `PhysicsBackend`
and an acquisition provider, including identical frame/action/reset semantics;
changing an import is insufficient evidence of portability. Sensors have one
read-only native copy of the world. This prototype does not include Unreal or a
native interactive viewer process; the previous stack's viewer remains intact.

## Verification (2026-10-30)

Passed: **203 control/connection regression tests** and **29 simulation tests**,
including ten simulated minutes standing, every walking direction, concurrent arm
motion, stop/resume, reset and real worker transport/sensors. Tests use the actual G1 URDF and
GR00T weights; the worker blueprint runs on an isolated Zenoh bus. Generic checks cover coherent clock/duplicates, delayed
inference, stale replies after reset, servo field preservation and atomic invalid
batch rejection. Stock WebRTC routing and DDS soft-start behavior use mocked I/O;
there is no physical-hardware, low-power-mode or Desktop UI acceptance claim.

The worker test measured **14.34 camera FPS per simulated second**, **0.83× real
time** and **0.84 ms maximum coordinator computation** over roughly five simulated
seconds while driving and moving an arm. Camera scheduling remains 15 Hz. These
short samples are not a low-power or cross-platform performance guarantee.

The integration adds one control worker and a small feedback/reply round trip per
control sample. It adds no physics/render model copy and does not run duplicate
inference on repeated feedback. The ten-minute standing test retains
its 30 cm drift threshold; contact with furniture is not a recovery guarantee.
