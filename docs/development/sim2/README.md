# sim2 Option G: body state, actions and independent sensors

SimModule owns time, stepping and command validity. PhysicsBackend implements
engine operations. There is no separate Runtime/Simulation execution layer.

```text
camera / lidar / IMU -- get_usd() --> UsdManager mounts
UsdManager -- get_scene() + get_asset() --> physics / rendering / raycasting

ActionCommand --> SimModule --> observations: WorldState --> sensor modules
                                                        --> UsdManager TF
```

## Directory and ownership

```text
sim2/
  module.py                  execution, complete samples, commands and RPCs
  description.py             selects scene_manager or a local USD scene
  cbor.py                    CBOR observation/command envelope validation
  physics/interface.py       PhysicsBackend protocol
  physics/mujoco.py          native dynamics and physical queries
  physics/drives.py          existing USD drive read/write helpers for MuJoCo
  geometry/mujoco.py          shared model compilation
  geometry/query.py          geometry-only pose application; no physics stepping
  geometry/poses.py          complete pose validation and lookup
  sensors/sampling.py        acquisition cadence in simulation time
  sensors/imu/               mounted motion -> Imu
  sensors/encoder/           joint truth -> measured JointState
  sensors/camera/             poses + renderer -> Image / CameraInfo
  sensors/lidar/              pose history + raycaster -> PointCloud2
  examples/                  normal blueprints, deployed sensor and outcome tests
```

## WorldState

The generated `dimos_lcm.sim_msgs.WorldState` is one complete acquisition:

```text
header: SampleHeader
model_revision: string
bodies: BodyState[]              # every described rigid body/frame
joint_truth: JointState          # configured scalar joints; empty selection exports all
cbor_observations: CborObservation[] # all currently reported optional properties
```

`BodyState` and the existing `sensor_msgs.JointState` are nested in this
message. `joint_truth.header` is the standard `std_msgs.Header` with the same acquisition
time as the outer header; its effort is actuator-applied
joint torque/force, excluding gravity and contact contributions. Each BodyState contains `body_id`, `pose: Pose`, `twist: Twist` and
`acceleration: Accel`. Pose maps body to world. Linear/angular velocity and
linear/angular acceleration are at the body-frame origin, expressed in world axes. There is one record
per body, including fixed sensor mounts. No overlapping pose/motion lists. Their values share the outer header. A snapshot replaces its predecessor;
omission never means to retain an old extension value. The sample is physical
truth, not a sensor measurement or native solver checkpoint. Every sensor consumes
`observations: In[WorldState]` and produces its ordinary measurement message.

`CborObservation` contains `entity_id: string`, `key: string`, and
`value: byte[]`. IDs are resolved body/joint names or `world`. The key identifies
the property and its agreed payload format. Each value contains one CBOR item; `(entity_id,key)`
is unique. Unknown optional extensions may be ignored. Required consumers must
check their key, payload and availability. Typed physics and control fields remain outside CBOR.

See the companion dimos-lcm draft for exact array-count fields and generated
bindings. Sensor and TF messages retain their existing std_msgs.Header wire formats.
SampleHeader is used only by the Sim2 WorldState and ActionCommand contracts.

## Calls and ports

Every mounted camera, lidar and IMU implements `get_usd() -> [asset_root, usd]`.
A scene `UsdManager` mounts those device descriptions at named frames in its
prepared USD asset. The G1 example's asset contains the robot, desk and loose box;
the blueprint supplies the camera, Mid360 and IMU at the robot's attachment frames.
There is no Python `Instance`/`Drive` ledger or `robot_models` shared-config delivery.

Physics, camera and lidar use `scene_manager=...`, call `get_scene()`, then fetch
referenced files through `get_asset(name)`. The common client checks declared content
hashes and stores relative, content-addressed filenames so readers get the same
scene revision in different cache directories. `scene=UsdModel(...)` selects a local
USD instead. An encoder observes existing joints and adds no physical subtree.

MuJoCo 4.0's native USD importer reads the composed USD directly. Install the
`sim` extra on Python 3.11-3.13, on macOS ARM64 or Linux x86_64.
DimOS defaults to Python 3.14, but MuJoCo 4.0's USD importer does not support it;
use Python 3.12 for this stack. CI runs the native Sim2 tests in a separate 3.12
environment on both self-hosted platforms. There is no
runtime USD-to-URDF-to-MJCF path. Convert and validate assets during authoring;
the running stack loads the prepared files. The camera and lidar create private
geometry copies with independent bodies driven by `WorldState` poses; neither
sensor steps physics. This prototype tests rigid bodies and scalar joints.

`SimModule.simulation_config()` supplies the run ID and configured gravity through
an ordinary RPC during build. The policy receives the ID and the IMU receives
gravity before start. Gravity is fixed for the run and is absent from WorldState.

UsdManager uses Jeff's `JointTf` helper when its input is joint state. When connected
to simulation observations, it publishes the complete TF tree from body poses at
their acquisition time instead; the joint-state publisher stays off for that run.

`start()` opens the physics owner thread and one publisher thread. The owner calls
`_apply_commands()`, `_sample_truth()` and `_step()`; `_inspect()` constructs a
complete snapshot between physics steps. Only the owner accesses native dynamics.

```text
# PhysicsBackend; all reads return detached records owned by the caller.
load(scene: UsdModel, dt_ns: int, gravity_world: tuple[float, float, float]) -> None
read_bodies() -> list[BodyState]
read_joints(joint_ids: tuple[str, ...]) -> JointState
read_cbor_observations() -> list[CborObservation]
apply_action(joint_commands: list[JointCommand], cbor_commands: list[CborCommand]) -> None
step() -> None
hold(joint_ids: tuple[str, ...]) -> None
reset() -> None
place(body_id: str, pose: tuple[float, ...]) -> None
close() -> None
```

MuJoCo's read_bodies() computes shared constraint-aware accelerations once per
acquisition, then exports pose, velocity and acceleration for every body. This
is a denser motion export than F's selected-frame list; no speedup is claimed.

MuJoCo exports no optional observation properties by default. A backend can return
current properties from `read_cbor_observations()` without changing the module or sensors.
The deployed extension test exercises this path over real dimOS transport.

Input: `actions: ActionCommand`.
Output: `observations: WorldState`.
RPCs: `inspect()` and `place(body_id, pose)` each return WorldState;
`submit(command)` and `status()` handle commands/diagnostics. RPC requests execute
on the owner thread between steps. No manual-step API or standalone executor.

## Actions

`ActionCommand` contains one `SampleHeader`, `based_on_time_ns`, `apply_at_ns`,
`valid_until_ns`, `joint_commands: JointCommand[]` and `cbor_commands: CborCommand[]`
(with matching array counts). One or both arrays must be nonempty. The existing ROS
`control_msgs.JointCommand` is unchanged: `header`, `joint_names[]`,
`interface_name`, `values[]`. Sim2's motor profile sends five records in one batch:
`position`, `velocity`, `kp`, `kd`, and `effort`, each addressing the same named
joint set. The backend applies `kp*(position-q) + kd*(velocity-dq) + effort`.
Names resolve values independently of array order. Missing/duplicate interfaces,
mismatched lengths, nonfinite values or negative gains reject the complete action.
This five-interface motor law is a Sim2 convention, not a generic ROS guarantee;
ordinary ROS position controllers can consume a single position record.

The module checks run identity, timing, counts and CBOR envelopes,
then calls `backend.apply_action(...)` once. The backend validates all entries
before mutation; unsupported operations reject the entire action. Accepted source/
sequence pairs are not executed twice. Joint targets persist until expiry, then
hold the current position. Due commands arriving after `apply_at_ns` still apply
if unexpired; physics never rewinds or waits for them. Multiple sources targeting
the same joint overwrite in acceptance order; arbitration belongs upstream.

`CborCommand` contains `entity_id`, `key`, `value_length`, `value`.
IDs and byte rules match CborObservation, but commands request operations: unknown
keys and invalid payloads **reject**, rather than being ignored. The operation specifies effects
and lifetime; execution is once at acceptance. Expiry does not undo CBOR effects.
MuJoCo implements joint targets and rejects all CBOR commands. A test backend
extends real MuJoCo with a small counter operation to verify transport, atomic
rejection and duplicate handling. No registry or wrench input is introduced.

## Timing and sensors

SampleHeader identifies the run clock, acquisition time in nanoseconds, source,
sequence and spatial frame; epoch is zero for this restart-based implementation.
A fresh episode uses `dimos restart`: a new clock ID, time zero and fresh sensor,
controller and mapper state. There is no in-place SimModule.reset() RPC. Placement
is an in-run state edit: it does not rewind time, clear maps or start an episode.
Sensor and TF stamps use seconds/nanoseconds in this simulation clock. The G1
demonstration joins IMU and encoder readings by timestamp; its run identity arrives
once during build through the ordinary `simulation_config()` RPC. The same RPC
provides gravity to the IMU. Each consumer selects its simulation by RPC name.

Physics defaults to fixed 2 ms steps and complete observations at 500 Hz. Sensors
retain their own acquisition rates in simulation time. A rate not integral in
physics steps samples at the next actual step (200 Hz with 2 ms steps: 0, 6, 10,
16, 20 ms). Physics advances independently of sensors and controllers. A late
sensor sample keeps its acquisition timestamp; it is not retimestamped on delivery.

`pace_to_wall_time=False` removes real-time pacing; it does not make downstream
modules deterministic or lossless. Camera can skip output when busy; lidar drops
scans when its bounded pose history has gaps. A full publication queue drops its
oldest observation rather than blocking physics. Physics keeps its fixed timestep
when overloaded and reports slower-than-real-time execution, never a larger step.

`status()` reports real-time factor, late steps, accepted/late/rejected commands,
expired joint targets and dropped observations. Sensor `statistics()` reports
input drops, history gaps and render overruns. Warnings are throttled. Expired
motor targets hold their current position; that is not a balance controller and
cannot guarantee a humanoid remains standing without timely policy output.

IMU uses world motion at its mount, rotates into sensor axes, subtracts gravity and
applies bias/noise. Ideal orientation is opt-in. Encoders select named joints and
apply calibration/quantization/noise; effort is unavailable. Both cache repeated
acquisitions. Camera and lidar maintain geometry-only scenes from the same models;
they never step a second dynamics trajectory. Lidar checks pose-history coverage.
Its `lidar: Out[PointCloud2]` contains raw sensor-axis points, scan-start time,
per-point firing offsets and laser-channel IDs. Its optional
`pointcloud: Out[PointCloud2]` contains the same hits registered in world using
simulation truth; it is for inspection, not an estimator input. The raycaster
returns distances and surface normals. `Mid360Scan` defines directions/times;
`Mid360Returns` applies the reused approximate range-noise and incidence-dropout
model. Device configuration and optical exclusions belong to the sensor, not
the physics backend. See the G1 guide for the truth-registered preview map.

## Run and test

```sh skip
uv sync --python 3.12 --extra sim
source .venv/bin/activate
dimos run sim2-basic --daemon
dimos shell
# app.SimModule.inspect()
dimos stop

pytest dimos/sim2 -m 'not native_e2e' --import-mode=importlib -o addopts='' -q
pytest dimos/sim2/test_module.py dimos/sim2/examples --import-mode=importlib -o addopts='' -q
```

The final integration slice provides [G1 GR00T beside a desk](/docs/development/sim2/g1.md), with camera,
lidar, IMU and encoders. MuJoCo is the validated backend. Native renderer/raycaster
adapters are selectable; another physics backend and cross-renderer fidelity have
not been validated. The core imports no robot-specific policy or coordinator code.

## Review stack

The stack implements Option G: one `BodyState` per body, `JointCommand` records,
and CBOR commands/observations without a schema field. The LCM dependency is pinned
to the matching draft commit until a package release is available. JointCommand
retains its published ROS wire layout. The earlier `sensor_msgs.JointCommand` was
a separate dimOS-only type; its two manipulation publishers now emit named
`control_msgs.JointCommand` position records. Trajectories must provide joint
names and Cartesian feedback must name each joint in the IK result's order.

Pim's nine PRs build directly on Jeff's asset-serving layers:

```text
main
  -> Jeff: #5102 USD conversion and file serving
  -> Jeff: #5105 namespaces and mounts
  -> Jeff: #5109 empty bridge (Mid360 moved to the lidar layer)
  -> Jeff: #5111 UsdManager rename
  -> Pim: #5033 messages -> #5034 models -> #5035 physics
  -> Pim: #5036 IMU -> #5037 encoders -> #5038 camera -> #5039 lidar
  -> Pim: #5040 examples -> #5041 G1 GR00T
```

No shared-configuration hook or coordinator change is required. Modules use the
existing build/start lifecycle and explicit RPC dependencies. Jeff's Sim2 branches
#4908-#4915 and #5126 are not dependencies.

#5039 carries Jeff's unchanged `mid360_usd()` provider and `Mid360.get_usd()` RPC,
which he moved from #5109 to his lidar PR. It shares that device description with
the real driver without importing Jeff's Sim2 implementation.

The integration keeps Pim's `SimModule`, `PhysicsBackend`, `WorldState`,
`ActionCommand` and independent sensor modules. Jeff's asset serving and composition
RPCs supply the same composed USD to physics and geometric sensors. The old
`robot_models()` bypass, `Instance` and `Drive` descriptions are removed.
The companion [dimos-lcm #31](https://github.com/dimensionalOS/dimos-lcm/pull/31)
owns the schemas and generated codecs.

| PR | Review scope |
| --- | --- |
| [#5033](https://github.com/dimensionalOS/dimos/pull/5033) | Message integration |
| [#5034](https://github.com/dimensionalOS/dimos/pull/5034) | Shared models and scenes |
| [#5035](https://github.com/dimensionalOS/dimos/pull/5035) | SimModule and physics backend |
| [#5036](https://github.com/dimensionalOS/dimos/pull/5036) | IMU |
| [#5037](https://github.com/dimensionalOS/dimos/pull/5037) | Joint encoders |
| [#5038](https://github.com/dimensionalOS/dimos/pull/5038) | Camera |
| [#5039](https://github.com/dimensionalOS/dimos/pull/5039) | Lidar |
| [#5040](https://github.com/dimensionalOS/dimos/pull/5040) | Blueprints and evaluation |
| [#5041](https://github.com/dimensionalOS/dimos/pull/5041) | G1 GR00T integration |

### Camera ownership

`SimCameraModule(sensor=CameraConfig(...))` owns a camera's `get_usd()` description
and declares its ordinary `color_image`, `depth_image` and `camera_info` outputs.
Add one `CameraRenderModule(scene_manager=scene, renderer=MujocoRenderer,
cameras=("camera",))` for cameras viewing that scene. `cameras` contains the device
RPC names (`instance_name` when explicitly set, otherwise the class name). During
build the owner reads each device's `camera_config(renderer_name)` and existing
`get_stream_descriptors()` RPC. No global discovery or core configuration exchange.
Images are published directly to those output topics, without an intermediate
image stream or image RPC. Existing namespaces and stream remappings still apply.

The owner receives `WorldState` once and calls `render(sample, camera_ids)` for
the cameras due at that simulation time. The renderer loads one scene for all
cameras. MuJoCo shares its model/data across every view and its OpenGL context
between views with equal resolution; distinct resolutions need separate native
contexts. Returned arrays retain their pixels after subsequent renders. RGB is
`uint8[H,W,3]`; depth is optical-axis distance in metres, `float32[H,W]`.

If a blueprint deliberately uses several rendering owners, give each an explicit
`instance_name` and set each camera's `renderer_name` to that name. Each owner
lists its camera devices explicitly; conflicting ownership or omitted devices
fail during build/start. Camera acquisition remains latest-state
preview: this change does not add a required-sensor barrier or exposure model.

Each PR targets the preceding branch so its diff shows only its capability.
The stack publishes complete observations and accepts one action input. The previous broad drafts #4982/#4983 are
closed with their review history preserved.

## Shared-code boundary

The stack adds no mapping or localization implementation. `dimos/mapping`,
Point-LIO, `UsdManagerPointClearing` and `RoboPlanBodyFilter` match Jeff's USD-manager prerequisite base.
The G1 demo may compose the existing mapper as a world-frame preview consumer;
it does not include the earlier estimated-map or robot-filtering integration.

The shared files that still change have these specific roles:

| File or group | Reason within Option G |
| --- | --- |
| `robot/assets/model.py` | Resolve fixed mounts and namespace one model shared by physics, sensors and TF; normalize frame values after CLI config parsing. |
| `robot/usd_manager.py` | Compose mounted device USD through RPC and publish the ordinary TF tree from WorldState. |
| `msgs/sensor_msgs/Image.py`, `PointCloud2.py` | Preserve valid subsecond acquisition stamps in existing decoders; schemas are unchanged. An all-zero cloud stamp still means unspecified. |
| G1 URDF and its existing mount publisher | Define the native Mid360 orientation once in the model instead of adding its roll separately in hardware TF. |
| Existing GR00T task | Accept an explicit IMU reading without requiring a hardware adapter in the demonstration host. |
| Blueprint registry and gateway annotations | Expose the normal `dimos run` / Desktop launch entries. |
| MuJoCo stubs and dependency metadata | Type the backend calls and install the Option G message and CBOR dependencies. |

The first PR's remaining sensor edits are decoder behavior, not a new header or
transport. The standard Image decoder preserves zero; PointCloud2 retains its
existing unspecified-zero convention. Consumers must not assume every timestamp
has Unix-epoch meaning.

Optional observation payloads use the existing CBOR array, with namespaced,
versioned keys. Defining an unused extension costs zero additional bytes; old
compiled readers can ignore unknown entries. Changing WorldState or its nested
core fields still changes the LCM fingerprint and requires a new wire version.
The companion LCM PR contains a compiled C++ compatibility test and a frozen
JointCommand fixture from published 0.1.6.
