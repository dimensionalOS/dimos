# sim2: complete physical state and independent sensors

SimModule owns time, stepping, reset and command validity. PhysicsBackend implements
engine operations. There is no separate Runtime/Simulation execution layer.

```text
UrdfManager -- shared model configuration --> SimModule / geometry consumers

ActionCommand --> SimModule --> observations: WorldState --> sensor modules
                      |                                 --> UrdfManager TF
                      +--> clock: SimulationClock
```

## Directory and ownership

```text
sim2/
  module.py                  execution, complete samples, commands and RPCs
  description.py             model instances, placement, gravity and drives
  extensions.py              CBOR property/command envelope validation
  physics/interface.py       PhysicsBackend protocol
  physics/mujoco.py          native dynamics and physical queries
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
body_poses: BodyPose[]       # all described rigid bodies, including stationary ones
body_motion: BodyMotion[]   # configured frame origins, world-axis motion
joint_truth: JointTruth     # configured scalar joints; empty selection exports all
extensions: CborExtension[] # all currently reported optional properties
```

`BodyMotion` and `JointTruth` are nested records, not independently published
messages. Their values share the outer header. A snapshot replaces its predecessor;
omission never means to retain an old extension value. The sample is physical
truth, not a sensor measurement or native solver checkpoint. Every sensor consumes
`observations: In[WorldState]` and produces its ordinary measurement message.

`CborExtension` contains `entity_id: string`, `key: string`, `schema: string`, and
`value: byte[]`. IDs are resolved body/joint names or `world`. The schema identifies
a versioned payload contract. Each value contains one CBOR item; `(entity_id,key)`
is unique. Unknown optional extensions may be ignored. Required consumers must
check their schema. Typed physics and control fields remain outside CBOR.

See the companion dimos-lcm draft for exact array-count fields and generated
bindings. Sensor/TF SampleHeader encoding is unchanged by Option F.

## Calls and ports

`config_receive()` resolves SceneConfig against models delivered by UrdfManager's
`@shared_config(SharedConfigModule)` provider from #4937, before build/start. Models
are materialized once; simulation and geometry consumers use the same descriptions.
Sensor settings stay in their sensor modules.

`start()` opens the physics owner thread and one publisher thread. The owner calls
`_apply_commands()`, `_sample_truth()` and `_step()`; `_inspect()` constructs a
complete snapshot between physics steps. Only the owner accesses native dynamics.

```python
# PhysicsBackend; all reads return detached records owned by the caller.
load(scene: SceneConfig, dt_ns: int) -> None
read_body_poses() -> list[BodyPose]
read_motion(frame_id: str) -> BodyMotion
read_joints(joint_ids: tuple[str, ...]) -> JointTruth
read_extensions() -> list[CborExtension]
apply_action(joint_targets: list[JointTarget], cbor_commands: list[CborCommand]) -> None
step() -> None
hold(joint_ids: tuple[str, ...]) -> None
reset() -> None
place(body_id: str, pose: tuple[float, ...]) -> None
close() -> None
```

MuJoCo exports no optional extension properties by default. A backend can return
current properties from `read_extensions()` without changing the module or sensors.
The deployed extension test exercises this path over real DimOS transport.

Input: `actions: ActionCommand`.
Outputs: `observations: WorldState`, `clock: SimulationClock` (execution status).
RPCs: `inspect()`, `reset()` and `place(body_id, pose)` each return WorldState;
`submit(command)` and `status()` handle commands/diagnostics. RPC requests execute
on the owner thread between steps. No manual-step API or standalone executor.

## Actions

`ActionCommand` contains one `SampleHeader`, `based_on_time_ns`, `apply_at_ns`,
`valid_until_ns`, `joint_targets: JointTarget[]` and `cbor_commands: CborCommand[]`
(with matching array counts). One or both arrays must be nonempty. Each joint
target carries `joint_id`, `position`, `velocity`, `kp`, `kd` and `feedforward`.

The module checks identity, timing, counts, CBOR envelopes and controller ownership,
then calls `backend.apply_action(...)` once. The backend validates all entries
before mutation; unsupported operations reject the entire action. Accepted source/
sequence pairs are not executed twice. Joint targets persist until expiry, then
hold the current position. A CBOR-only operation never releases a motor boundary.

`CborCommand` contains `entity_id`, `key`, `schema`, `value_length`, `value`.
IDs and byte rules match CborExtension, but commands request operations: unknown
keys/schemas **reject**, rather than being ignored. The schema specifies effects
and lifetime; execution is once at acceptance. Expiry does not undo CBOR effects.
MuJoCo implements joint targets and rejects all CBOR commands. A test backend
extends real MuJoCo with a small counter operation to verify transport, atomic
rejection and duplicate handling. No registry or wrench input is introduced.

## Timing and sensors

SampleHeader identifies the clock, reset epoch, acquisition time in nanoseconds,
source, sequence and spatial frame. Reset/placement increments epoch and returns
simulation time to zero. Receivers must not join samples across clocks/epochs.

Physics defaults to 2 ms, complete observations to 500 Hz. Sensors retain their own
acquisition rates in simulation time. A rate not integral in physics steps samples
at the next actual step (200 Hz with 2 ms steps: 0, 6, 10, 16, 20 ms). Configured
controller boundaries must align with the observation schedule. Physics waits for
the matching command, with a finite host-time timeout; it never resets automatically.

`pace_to_wall_time=False` removes real-time pacing. Required controller replies
still gate progress. Camera/lidar currently allow skipped samples/output when busy;
lossless accelerated sensing/backpressure is not implemented. Observation queue
overflow faults explicitly. This is not a measured speedup claim.

IMU uses world motion at its mount, rotates into sensor axes, subtracts gravity and
applies bias/noise. Ideal orientation is opt-in. Encoders select named joints and
apply calibration/quantization/noise; effort is unavailable. Both cache repeated
acquisitions. Camera and lidar maintain geometry-only scenes from the same models;
they never step a second dynamics trajectory. Lidar checks pose-history coverage.

## Run and test

```sh
uv sync --extra sim
source .venv/bin/activate
dimos run sim2-basic --daemon
dimos shell
# app.SimModule.inspect()
dimos stop

pytest dimos/sim2 -m 'not native_e2e' --import-mode=importlib -o addopts='' -q
pytest dimos/sim2/test_module.py dimos/sim2/examples --import-mode=importlib -o addopts='' -q
```

The final integration slice provides [G1 GR00T beside a desk](https://github.com/dimensionalOS/dimos/blob/pim/feat/sim2-e-g1/docs/development/sim2/g1.md), with camera,
lidar, IMU and encoders. MuJoCo is the validated backend. Native renderer/raycaster
adapters are selectable; another physics backend and cross-renderer fidelity have
not been validated. The core imports no robot-specific policy or coordinator code.

## Review stack

[The nine draft PRs](https://github.com/dimensionalOS/dimos/stack/5042) build on
[shared configuration #4937](https://github.com/dimensionalOS/dimos/pull/4937).
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

Each PR targets the preceding branch so its diff shows only its capability.
The final tree implements Option F: complete observations and one action input. The previous broad drafts #4982/#4983 are
closed with their review history preserved.
