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
app.G1Connection.control_status()  # controller: GR00T WBC; model: balance or walk
app.SimModule.timing()
app.SimSensorModule.statistics()
app.SimModule.reset()  # restore G1 and objects; reset policy history via a fresh epoch

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
app.G1Connection.move(Twist(linear=Vector3(.15, 0, 0)), duration=2)

from dimos.sim2.types import Wrench
state = app.SimModule.state()
app.SimModule.apply_wrench(Wrench(state.epoch, "parcel/box", (1, 0, 0), duration=.1))
```

## What the coordinator does

```text
Desktop /cmd_vel: Twist
          ↓
G1Connection → G1SimDevice → existing G1GrootWBCTask
          ↑ joint + IMU feedback          ↓ 29 named joint targets
          └────────── SimModule ──────────┘
                        ↓
                  PhysicsBackend
                  (MuJoCo today)
```

**ModuleCoordinator starts processes, wires streams and stops them. It does not
run the walking policy.** This example has seven modules in six worker processes:
physics, connection/control and sensor acquisition each get a dedicated worker;
four existing UrdfManagers share the other workers.

**ControlCoordinator is a different component.** It schedules control tasks,
resolves which task owns each joint and combines their motor commands. This
example calls its existing **G1GrootWBCTask** directly, with complete joint and
pelvis IMU feedback. The task drives the 15 leg/waist joints; the facade holds
the 14 arm joints at the reference pose. There is one command owner, so this
example does not instantiate ControlCoordinator. Concurrent arm tasks still need
that coordinator's arbitration; this example does not implement it again.

The existing task owns balance/walk selection, six-frame observation history,
action scaling and reset. The facade supplies feedback and sends named targets;
it does not copy those algorithms. Zero velocity selects balance; movement selects
walk. There is no empirical velocity bias or fallback to the old walking model.
Two small ONNX sessions each use one inference thread.

The simulated G1's walking policy is outside physics, in its connection worker.
The real `G1Connection` still delegates velocity commands to Unitree WebRTC/firmware.
The ONNX emulator is **not a copy of stock Unitree firmware**.

## Where simulation attaches

Read [the blueprint](/dimos/robot/unitree/g1/blueprints/basic/unitree_g1_sim2.py):

```python
unitree_g1_sim2 = autoconnect(
    SimModule.blueprint(world=G1_DESK_WORLD, backend=MujocoBackend,
                        control_joints=all_g1_motor_names),
    G1Connection.blueprint(connection_type="sim2"),
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
| `sim2/client.py` | Ordinary device-side named target sender; no process/module |
| `robot/unitree/g1/sim_device.py` | Adapt shared feedback to the existing GR00T task; deadman, target sending and epoch reset |
| Existing `control/tasks/g1_groot_wbc_task/` | GR00T observation/action convention, history, balance/walk inference and shared motor profile |
| `robot/unitree/g1/model.py` | Canonical G1 description, shared mounts and walking drive profile |
| Existing `robot/urdf_manager.py` | Internal TF edges and model/mesh serving |

`WorldState` contains epoch, tick, sim/wall time, model revision, world link poses
(xyz/xyzw), world twists at link origins, scalar joint position/velocity and applied
actuator effort. `Layout` names the array order. IMU measurements accompany it in
`Frame`; images and scans remain separate sensor messages.

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

## Verification and measured limits (2026-10-30)

- 86 checks cover the core, shared descriptions, task evaluation, existing GR00T
  task behavior, actual GR00T control, real workers/sensors and blueprint registry.
- G1 stands for ten simulated minutes: 8.5 cm total displacement from startup,
  minimum pelvis height .743 m. Forward/backward and both strafe directions run
  for 15 s each, then return to balance; both turn directions, deadman expiry,
  duplicate feedback and reset are checked. No falls in these trials. This does
  not prove the cause of the earlier reported fall or guarantee obstacle recovery.
- Normal CLI daemon startup, status, attached state/reset and graceful stop passed
  on a separate test bus. Startup was about 1.6 s after build began.
- Before the camera rate increase, a CLI sample at 19.7 simulated seconds showed
  maximum task compute .69 ms, zero rejected commands/publication drops, 94 images
  and 183 scans. Two incomplete
  scan intervals were discarded; there were no acquisition errors. These are
  measurements on this Mac, not platform guarantees or a low-power acceptance run.
- At the requested 15 FPS setting, a subsequent CLI sample delivered 180 RGB/depth
  pairs over 12.04 sim seconds: **14.95 FPS in sim time, 12.95 FPS in wall time**.
  No acquisition errors, queue drops or rejected control commands; G1 stayed upright.
- Native RGB/depth rendering uses MuJoCo 3.10.0. macOS reports limited depth
  precision without ARB_clip_control. The scanner is an ideal ray model, not a
  calibrated Mid360 noise/intensity simulator. No Desktop UI acceptance yet.
- Desk visuals derive from the existing `mujoco_sim/scene_office1` pack, preserving
  UVs and diffuse textures. Desk/chair collisions are measured box approximations;
  computer monitors are decorative. No new asset ingestion framework is involved.

```bash
pytest dimos/sim2 dimos/robot/assets/test_model.py dimos/control/tasks/g1_groot_wbc_task -m '' -q
pytest dimos/robot/test_all_blueprints_generation.py
```

`conftest.py` creates temporary primitive URDFs for the joint/force, two-robot,
mount, sensor and reset tests. `test_eval.py` runs the existing EvalRunner against
physical push/no-op outcomes (scores 1 and 0). There is no public lab robot,
connection module, blueprint or eval CLI. Real worker integration is tested with
the G1 example. No new G1 task benchmark or automatic world collection is claimed.
