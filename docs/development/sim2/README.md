# sim2: minimal working architecture

Prototype branch `pim/feat/sim2-minimal`, built afresh from `main@64b2d8aa6b`.
The previous sim2 stack is unchanged. This example is a small physical workbench:
**two instances of one robot, one free box, one passive hinged door, mounted
RGB/depth + lidar + IMU, robot commands, forces, reset and a scored task.**

## Run it

From this checkout (the tiny generated URDF assets live in `assets/sim2_lab`):

```bash
source .venv/bin/activate
# On a fresh checkout: uv sync --all-groups

dimos run sim2-lab
```

This uses the normal blueprint lifecycle. There is no `run.sh`, separate
simulation launcher or Rerun bridge. Another running blueprint on the same bus
must be stopped by its operator first. The verification below used a separate
Zenoh domain, discovery address and run registry; it did not stop the existing G1.

In another terminal in this checkout:

```bash
source .venv/bin/activate
dimos shell
```

In the shell:

```python
modules()
app.SimModule.describe()        # names corresponding to the state arrays
app.SimModule.state()           # physical truth, not a sensor observation
app.SimSensorModule.statistics()

# Slide +X slowly enough to push the box. Refresh the .5 s deadman.
import time
for i in range(40):
    app.left.move(min(.65, i * .02), 0.)
    time.sleep(.1)

app.SimModule.reset()           # both robots + box + door; new episode ID
app.SimModule.place("box", (.2, .2, .2, 0., 0., 0., 1.))
app.SimModule.timing()

from dimos.sim2.types import Wrench
app.SimModule.apply_wrench(Wrench(app.SimModule.state().epoch,
    "door/panel", force=(0., 0., 0.), torque=(0., 0., .2), duration=.3))
```

The fixture has prismatic slides, not a mobile base: its controls are joint
positions, not WASD velocity commands. `left/position_command` and
`right/position_command` accept ordinary `JointState` messages with local joint
names `slide_x`, `slide_y`.

Desktop is the application viewer. On the same DimOS bus, select **world** as
the fixed frame, `left/base_link` as the robot frame and, if desired,
`left/tool` as the follow frame. Sensor topics are `/color_image`,
`/depth_image`, `/camera_info`, `/lidar`, `/imu`. The lidar cloud is already in
world coordinates and preserves per-point acquisition offsets. URDF managers
serve the four models on `<instance>/render_file_request` → `<instance>/file_info`.
There is no `odom` tree in this example. Transport/TF and native rendering are
tested; this branch does not change Desktop or claim that its current UI has
been verified against this scene. The existing Desktop page was disconnected
during inspection. Native viewer UI and Unreal are not included in this slice.

Run the scored physical baseline without a GUI, daemon or API key:

```bash
python -m dimos.evals.suites.sim2_lab --out /tmp/sim2-lab-eval
# push: score=1; do_nothing: score=0
```

## Where simulation attaches

Read [the blueprint](/dimos/robot/sim2_lab/blueprints.py) first. It is the
complete composition, not an implicit replacement elsewhere in the stack:

```python
world = lab_world()  # collect every instance before compilation

autoconnect(
    SimModule.blueprint(world=world, backend=MujocoBackend),
    PusherConnection.blueprint(source="left", model=PUSHER),
    PusherConnection.blueprint(source="right", model=PUSHER),
    SimSensorModule.blueprint(world=world, reader=MujocoSensors),
    # one existing UrdfManager per instance, using that instance's model
)
```

The real file includes instance names and stream remappings. The connection's
`SimRobotClient` is an ordinary object using existing typed Zenoh streams; it
is not another module/process. Policies do not live in the simulator. This
fixture has no physical hardware driver; it demonstrates the sim device side
of a single connection class, not verified real/sim parity for a real robot.

The world list is explicit at composition time. A coordinator hook that discovers
arbitrary robot connections and resolves `--simulation` is deliberately still
an integration task. Editing a connection's model through CLI overrides alone
does not rebuild the world description: change the common definition/composition.

## What Jeff's shared model owns

[model.py](/dimos/robot/sim2_lab/model.py) is the robot's one declaration:

```python
PUSHER = (
    RobotModel.from_file(ASSETS / "pusher.urdf")
    .with_fixed_frame("camera", "tool", xyz=(-.25, 0, .6), rpy=(...))
    .with_fixed_frame("lidar", "tool", xyz=(0, 0, .12))
    .with_fixed_frame("imu", "tool")
)

left_model = PUSHER.with_prefix("left/")
```

The generated URDF defines anatomy, visual/collision geometry, inertia, joint
limits and axes. The model variants add mounts and instance namespaces. Both
MuJoCo and UrdfManager consume the same materialized XML; there is no authored
MJCF copy. **The shared-model additions are `with_prefix()` and
`resolve_frame()`**. Fixed-frame inputs are also normalized so CLI parsing preserves model identity.
UrdfManager itself is unchanged: it serves assets and
publishes internal joint/mount TF. SimModule publishes only `world → root` TF.

Add another instance: one entry in `lab_world()`; no new files. Add another robot
type: its canonical model plus its connection/device binding and composition.
Existing geometry assets remain assets. Actuator gains and sensor acquisition
profiles live beside the world declaration; they are not new URDF-manager duties.
No transforms, limits or inertias are copied into a simulator-specific robot file.

## The actual shared boundary

[types.py](/dimos/sim2/types.py) contains the complete public contract.

| Value | Exact meaning |
|---|---|
| `World` | Instances of shared models, placements, drive gains, sensor profiles, timestep |
| `Layout` | Stable body/joint/drive/root/frame names and a description revision hash |
| `WorldState` | Episode ID, completed tick, sim time, wall delivery timestamp, model revision; body pose/twist and scalar joint position/velocity/actual actuator effort arrays |
| `Frame` | One coherent `WorldState` plus native IMU measurements from that same tick |
| `JointTargets` | Position servo targets; validated as a whole; finite lifetime |
| `JointEfforts` | Direct N/Nm commands on declared drives; same joint effort limits |
| `Wrench` | Force + torque in world axes, applied to a body at a world point (default: link origin), for a finite simulation duration |
| `reset` / `place` | Privileged scene operations; invalidate old commands/history; never used as robot actions |

Pose = `[x,y,z,qx,qy,qz,qw]`, world-from-link. Twist = world-expressed
`[vx,vy,vz,wx,wy,wz]` at the link origin. Joints use m/rad, m/s or rad/s and N/Nm.
IMU = local specific force (stationary upright reads +9.81 Z), then local angular
velocity. Its orientation estimate is explicitly unavailable. Actual effort
excludes passive contact/constraint reactions. No contact array or task reward
is fabricated. Floating-root coordinates are body state; v1 supports independent
scalar articulated joints and rejects mimic/other joint types at compilation.

```python
runtime.submit(JointTargets(epoch, ("left/slide_x",), (.4,)))
runtime.submit(JointEfforts(epoch, ("left/slide_x",), (5.,)))
runtime.submit(Wrench(epoch, "box", force=(5., 0., 0.), duration=.3))
runtime.submit(Wrench(epoch, "door/panel", force=(0., 0., 0.),
                      torque=(0., 0., .2), duration=.3))
```

The box moves freely; the panel's joint constrains its motion. A passive joint
is not a motor. These all use the same solver and contacts. Expired drive
commands hold measured joint position; expired wrenches disappear. Reset creates
a fresh UUID, including across process restarts, so an old command is rejected.

The physics and sensor provider are injected separately. The generic module,
runtime and public types import no MuJoCo. MuJoCo is the **only implemented and
validated backend** here. Another engine must implement the physical behavior
and either supply a sensor reader or satisfy this reader's model/state binding;
a Protocol does not establish equivalent contact dynamics or sensor fidelity.

## What survives, what becomes unnecessary

Comparison target: old implementation `460f5685a8`, and notebook
`SIM2_SYSTEM_ARCHITECTURE.html` / `SIM2_ARCHITECTURE.html`.

| Decision | Result in this branch |
|---|---|
| Keep one shared world | Both robot instances, box and door compile before stepping; no per-namespace sim |
| Keep sensor workers and capture history | RGB-D, packet-timed lidar, solver IMU; physics never waits for rendering |
| Keep existing ownership | ConnectionModule, UrdfManager, streams/Zenoh, EvalRunner and lifecycle reused |
| Remove repeated robot facts | URDF imported directly; instance prefixes are model variants; no G1 simulation file |
| Remove policy from physics | The robot connection sends commands; no locomotion inference in the backend |
| Remove duplicated mutable truth | Feedback/TF/query views derive from a captured sample; no separate robot-state store |
| Replace broad engine API | Physics owns physical operations; scorer owns reward; sensor reader owns acquisition |
| Avoid new infrastructure | No cache service, plugin registry, eval runner, special launcher or new CLI command |
| Preserve useful native copies | One physics model/data + one read-only native query replica; no second dynamics owner |
| Defer broad integration | Automatic coordinator discovery, physical drivers, G1 stock/GR00T, Genesis, Unreal, contact evidence, full noise/randomization/recording policies |

## Performance and duplication evidence

Measured on this Mac, MuJoCo 3.10.0, this 15-link/5-scalar-joint scene, 2026-10-30.
These are small-scene timings, not a humanoid or low-power stability claim.

| Operation | Median | p95 |
|---|---:|---:|
| Owner step including coherent native forward refresh | 17.5 µs | 19.4 µs |
| Capture owned arrays, 50 Hz | 102.6 µs | 133.1 µs |
| RGB + depth, 320×240 | 2.34 ms | 3.19 ms |
| Lidar: 192 rays, 6 time packets | 0.63 ms | 0.71 ms |

5,000 steps, 500 captures, 49 RGB-D/lidar acquisitions; warm-path measurements.
A captured frame encoded to 2,123 CBOR bytes (~106 kB/s at 50 Hz before transport
framing; three recipients in this blueprint). Actual wire traffic, peak RSS and
high-load control latency have not been measured. A real CLI startup took ~3.5 s,
with ~2.8 s spent warming sensor dependencies. The blueprint deploys eight
existing-style module workers; only two hold native scene copies.

Physics ticks at 500 Hz; snapshots at 50 Hz; RGB-D at 5 Hz; lidar at 10 Hz.
Publishing runs outside the owner loop through a one-slot queue. Native
`mj_forward` after each integration makes joint/pose/IMU samples coherent; that
extra solve is included in the step timing and deserves scrutiny on G1.
Lidar uses preceding snapshots with a maximum 30 ms pose age, not continuous
motion reconstruction. Missing intervals invalidate scans. This ideal scanner
is not a Mid360/Point-LIO fidelity claim. Expensive frames can be dropped;
ticks are neither skipped nor reset when wall time falls behind.

## Verification

```bash
pytest dimos/sim2 dimos/robot/assets/test_model.py -m ''
mypy dimos/sim2 dimos/robot/sim2_lab dimos/robot/assets/model.py \
  dimos/evals/environments/sim2_lab.py dimos/evals/suites/sim2_lab.py \
  --follow-imports=silent
```

**61 tests passed**, including the native worker integration test; targeted mypy
and the repository pre-commit checks passed. Tests exercise physical pushes, force/torque, direct effort, robot isolation,
limits, expiry, reset/stale commands, owned snapshots, model/TF/native mount
agreement, RGB-D changes, actual floor hits, lidar history gaps, CLI config
roundtripping, real worker startup/shutdown and both scored outcomes. Generated
URDFs pass the URDF skill validator. The normal `dimos run sim2-lab --daemon`
startup, status, attached RPC inspection/reset and graceful stop were also exercised on an isolated bus.

The production sim2 core is seven Python files (~1,200 lines including headers,
docstrings and whitespace). The complete branch also includes the robot example,
generators/assets, tests, eval adapter, documentation and shared-model/stub edits.

The assets here are tiny primitive fixtures with readable generators. They are
checkout-local, not a completed PyPI asset-distribution design. Before migrating
the existing stack, the decisive next acceptance is a real robot definition
(G1 stock plus its existing low-level controller), then one second backend
against the same behavior tests. Neither requires replacing this core with a
framework registry.
