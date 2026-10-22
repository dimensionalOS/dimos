# sim2 core

sim2 owns a MuJoCo world and independently scheduled camera/lidar workers.
Robot connections and policy hosts use `SimRobotClient` for internal motor IO.
The core does not choose a locomotion policy or create public robot connections.

This branch extracts the existing emulator onto current main. Robot blueprint
migrations are subsequent work; existing MuJoCo blueprints have not been
switched by this core change.

## Ownership

### Source layout

```text
dimos/sim2/
  spec.py                RobotConfig, RobotInstance, WorldConfig
  blueprint.py           World/device blueprint composition
  discovery.py           Collect robot descriptions before deployment
  module.py              SimulationModule and scene RPCs
  client.py              Robot-side command/feedback client
  world/                 MuJoCo assembly, stepping, scene state, viewer, run evidence
  messages.py            Robot endpoint description, timed commands and samples
  streams.py             DimOS Zenoh pub/sub and bounded local sample history
  sensors/
    spec.py              Shared mounts and acquisition requirements
    composition.py       Supported sensor families and integrated attachments
    device.py            Acquisition clock and delivery envelope
    reader.py            Read-only world snapshots/history
    module.py            Shared worker lifecycle and publication
    camera/              spec.py, device.py, module.py, renderers/
    lidar/               spec.py, device.py, module.py, raycast.py, models/
    imu/                 spec.py, device.py, module.py
```

Put a sensor's configuration beside its measurement implementation and publisher.
Ray patterns belong to `lidar/models/`; image renderers belong to
`camera/renderers/`. Neither belongs in a robot package. A robot package owns
the choice of devices, their mounts and its actuator bindings. Tests live with
their owner; cross-sensor acquisition tests stay in `sensors/test_sensors.py`.

### Robot-owned code

G1's package keeps the public entry at `connection.py`. The implementation is
organized by responsibility, not by which blueprint first needed a file:

```text
dimos/robot/unitree/g1/
  connection.py, connection_spec.py   Public robot interface
  model/                             Joint groups, physical metadata, sim bindings
  assets/                            Robot MJCF and URDF
  devices/                           DDS motors, fault checks, stock SDK/WebRTC
  controllers/groot/                 Gains, coordinator host, control composition
  manipulation/                      Planning groups, arm IK, ready pose
  sensors/                           Real sensor transforms and recording
  visualization/                     G1 Rerun configuration
  skills/                            Robot skills and agent prompt
  blueprints/                        Runnable stacks and navigation composition
```

The GR00T coordinator task remains in `control/tasks/g1_groot_wbc_task/`, where
the coordinator discovers tasks. SONIC is maintained in the independent G1
PRs #4304, #4305 and #3557; this stack does not include its implementation or
sim2 integration. Changing a policy does not require moving or duplicating
the robot description.

Go2 follows the same ownership: `controllers/locomotion/` contains policy
inference, while `simulation/` contains the simulated device, physical model
bindings, and measured physics fit. Its synchronous `simulation/reference.py`
is retained for calibration/regression rollouts, not used by sim2's live loop.
Small xArm/Piper packages keep `connection.py`, `config.py`, and `simulation.py`
alongside `blueprints/`; they do not need empty policy/device directories.

| Component | Owns |
| --- | --- |
| `SimulationModule` | World lifecycle, paced physics thread, privileged scene RPCs |
| `SimulationRuntime` | One mutable MuJoCo model/data, robot bindings, command application |
| `SimRobotClient` | One robot's internal command/observation channel; not a Module |
| `CameraDevice`, `LidarDevice`, `ImuDevice` | Measurement acquisition, simulation-time scheduling, delayed delivery |
| Sensor modules | Worker lifecycle, a local `WorldReader`, publication of device measurements |
| Robot connection or policy host | Controller selection, command refresh, public robot interface |

`simulation(world=WorldConfig(...))` returns a normal `Blueprint` containing one
world and the declared sensors. It does not return `HardwareComponent` entries,
register coordinator adapters, or add a second public connection for a robot.
Multiple robots are composed into that world before MuJoCo compilation.

### Connection-owned robot definitions

An opted-in connection declares `default_sim_scene` and the static method
`describe_simulated_robot(resolved_config, scene: Path) -> RobotInstance`.
The method must only describe the model, devices and placement; it must not
instantiate a driver, open hardware, compile physics or start a policy.

With `--simulation mujoco`, `ModuleCoordinator.build` collects these descriptions
after configuration resolution and before deploying any workers. It adds one
world and the declared sensor workers. Robot IDs are the connection's instance
names, including namespaces. Two copies named `left/arm` and `right/arm` share
one world and have separate device channels. Different default scenes require
an explicit shared scene choice; there is no arbitrary winning robot.

An explicitly supplied `SimulationModule` can set the scene, timestep, viewer
and scene-mounted devices. Its settings are preserved while the robot map is
completed. These are ordinary module configuration options when that module is
declared in the input blueprint. Automatically generated devices take settings
from the robot description, not additional CLI flags for undeclared modules.

Collected sensor outputs are always qualified: `left/arm/wrist/color_image`,
for example. The robot blueprint maps its consumers to those names; `.namespace()`
then keeps the robot stack isolated. TF messages still share the `tf` stream.
No real or DimSim connection is changed by collection. Adding another robot to
a running world requires rebuilding the complete blueprint; cross-host shared
memory clients are rejected.

`RobotConfig` binds an MJCF model, joints/actuators and named sensor attachments.
`RobotInstance` adds placement, initial joint positions, the explicitly selected
control IMU, startup/watchdog settings and controller provenance. A robot has no
whole-body/manipulator packet category. Every joint states whether it is an
effort motor or a native position servo; one robot can contain both. Fixed servo
gains belong to the device; policy PD gains arrive with each motor command.

Scene-mounted devices use the same definitions and workers. A mount names a
body in the scene; `world` is a fixed frame. A named MJCF camera keeps its
authored intrinsics. No robot is required for a sensor-only world.

```python skip
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.spec import WorldConfig

world = WorldConfig(
    scene=scene_path,
    robots=robot_instances,
    sensors=(Camera("overhead", Mount("world", xyz=(1.5, 0, 3))),),
)
blueprint = simulation(world=world)
```

The external camera publishes on `world/overhead/color_image` with frame
`world/overhead_optical`. Robot sensor topics are always qualified by robot and
device name, even with one sensor. Application blueprints explicitly select the
device they consume; adding a device cannot rename existing streams.

## Internal device interface

The world's `describe()` RPC supplies the exact channel descriptor for each
robot. It includes joint order, physical timestep and world-generation identity.
Do not reconstruct it from a robot name or reuse it after restarting the world.

```python skip
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.sim2.blueprint import simulation
from dimos.sim2.client import SimRobotClient
from dimos.sim2.messages import StreamDescriptor
from dimos.sim2.module import SimulationModule
from dimos.sim2.spec import WorldConfig

app = ModuleCoordinator.build(simulation(world=WorldConfig(scene_path, robot_instances)))
try:
    world = app.get_instance(SimulationModule)
    descriptor = StreamDescriptor.from_dict(world.describe()["robots"]["arm"])
    with SimRobotClient(descriptor, writable=True) as device:
        observed = device.read()
        device.write_positions(target_positions, observed=observed)
finally:
    app.stop()
```

This demonstrates the internal IO boundary, not a trajectory controller. A real
connection/controller refreshes commands in its existing control loop.

| Client call | Contract |
| --- | --- |
| `connect()` / `close()` | Attach/release resources; context-manager equivalent |
| `read()` | One copied, coherent joint/IMU/base frame with episode/tick/time metadata |
| `write_motors(MotorCommandArray, observed=...)` | Complete finite q/dq/kp/kd/tau arrays in descriptor order |
| `write_positions(positions, observed=...)` | Native position targets, including declared gripper units |
| `disable(observed=...)` | Zero effort outputs; native position servos target current position |

Readers are read-only by default. Exactly one writable client may own a robot's
actuator set; public feedback readers do not acquire that ownership. Vendor
sentinels must be normalized by the vendor integration before writing commands.

Feedback and command timeouts default to 250 ms and are explicit client settings.
The runtime rejects expired commands independently of the writer. On expiry it
holds the measured joint position using the instance's holding gains; it
does not replay the old target. Disabled effort outputs remain disabled until a
new enabling command or reset. None of these operations guarantees balance for
a floating robot, replaces a physical estop, or implements vendor firmware.

## Scene and reset boundary

The world exposes `describe_scene`, `scene_state`, `set_scene_state`, `reset`,
`set_spawn`, `set_paused`, `set_truth_enabled`, and `status`. These are privileged
operator/evaluator APIs, not robot perception or ordinary agent skills.

Scene edits validate the complete update before changing existing named bodies
or joints and preserve each robot's motor-enable state, including teleports.
Reset restores the authored baseline. Both invalidate old motor
commands and publish the new generation immediately. A policy result computed
before a reset cannot command the new generation, including a reset racing with
command publication. Application-level reset must also stop/reset controllers,
maps and policy history; physics reset alone does not do that.

`SceneState.episode` counts physical resets. `SceneState.generation` also
advances for an edit or resume, invalidating pending control and sensor work
without starting a new task episode. Graders must retain both, plus simulation
time, rather than treating every scene edit as another trial. Internal transport
`FrameMetadata.episode_id` carries that invalidation generation.

## Rolling Mid360

Use `Mid360` as a `Lidar.model` and attach its IMU to the same named site:

```python skip
from dimos.sim2.sensors.lidar.models.mid360.model import Mid360
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.spec import Lidar

lidar = Lidar(
    "lidar", "mid360_link", Mid360,
    imu=Imu("lidar_imu", "mid360_link"),
    self_occlusion=True,
    truth_outputs=False,
)
```

`raw_pointcloud` carries sensor-frame returns, nanosecond offsets and channel
IDs. `imu_raw` is in SI units, at 200 Hz by default. Wire these to current
`PointLio.lidar_raw` and `PointLio.imu_raw`; use zero extrinsic translation when
the simulated IMU and laser are colocated. Do not connect ground-truth pose or
the corrected `pointcloud` to the estimator. Point-LIO does not yet expose a
filter reset: restart it after resetting, teleporting or pausing its robot.
Resume preserves the physical state and motor-enable state but begins a new
observation generation, discarding pre-pause commands and acquisition history.

The scanner fires 200,000 points/s continuously across scan boundaries.
`downsample` retains complete four-channel groups and their original timestamps.
Range noise and grazing-angle dropout can be disabled independently without
changing the firing pattern. Those returns are an approximation, not a measured
reflectivity/material model. Ray casting uses moving scene/robot geometry at
acquisition time; raw scans retain motion distortion. Missing snapshot intervals
drop a scan rather than replacing it with an instantaneous one.

The Fourier implementation is shared with main's Mid360 model. The small
`mid360_pattern` LFS asset retains the earlier G1-capture frequencies, which
differ from the Go2 fit in `go2_sim`. The Go2 empirical mount-occlusion mask is
not applied to arbitrary robots.

## Transport and tests

`device.poll()` returns typed `Measurement` values without requiring a Module,
transport or dedicated process. Each value records acquisition start/end,
modeled availability, and epoch. `device.published(value)` separately records
actual host publication and lateness. Device rates follow simulation time, not
worker polling. Instantaneous devices acquire the newest due sample and count
missed samples; rolling lidar requires complete history and reports dropped
scans/IMU samples. Pending output is discarded on an epoch change. Pause stops
acquisition; resume reanchors the clock and invalidates old work.

The publication timestamp is producer-side, not downstream receipt time. The
measurement envelope stays inside the device/module implementation; public
streams still carry Image, PointCloud2 and Imu messages. A configured delay is
an explicit model parameter, not evidence that the hardware delay was measured.

Each sensor definition supplies its required sample rate/history duration.
Physics sizes the snapshot history from these requirements without recognizing
particular lidar models. The selected coherent control IMU travels with joint
feedback; additional IMUs can be independent streams. Delayed coherent control
IMU feedback is not implemented and is rejected explicitly.

The shipped modules still use dedicated workers. That is a deployment default,
not a requirement of the measurement implementations; this change makes no new
claim that processes outperform threads.

### Run evidence

CLI runs retain a small JSON record under the existing run log's `sim2/`
directory. Direct Python users can set `SimulationModule.evidence_dir` or call
`describe_run()`. It includes resolved device parameters, controller provenance,
initial MuJoCo integration state, model/source hashes and MuJoCo version.
Controller provenance contains the values supplied by the robot instance. It
does not inspect a policy loaded in another process: custom controller weights
and configuration must be supplied explicitly and checked against that run.
`WorldConfig.calibration_evidence` records parameter-to-evidence references;
`sampled_variation` records caller-sampled values, distributions and seeds. These
are separate: randomizing a parameter does not calibrate it. Unlisted parameters
are unvalidated, not implicitly measured. Sensor-model RNG seeds are also
preserved in their resolved model arguments.

No assets or compiled models are duplicated. This record identifies the initial
conditions; it is not a portable bundle or deterministic replay of asynchronous
commands. Record those command streams through the existing DimOS recorder.
The initial record does not include later scene mutations or controller resets.

Motor commands, robot feedback and world snapshots use DimOS `pZenohTransport`.
Startup uses the existing `ZenohRPC` to obtain a first sample and acquire the
robot's command ownership. It is not called for each command or physics tick.
The world accepts one writer per robot; commands and ownership expire if it
stops sending. A newly attached writer has a 30-second startup allowance.
Closing the writer releases it immediately. A killed writer cannot leave a
shared-memory segment or file lock behind.

Every command names its observation epoch and retains its original issue time.
Reset discards held commands and rejects delayed pre-reset messages. A client
reads cached feedback without waiting; stale feedback raises rather than being
restamped. Source timestamps are Unix seconds; expiry uses the same host's
monotonic clock. Remote physics/clients are not yet supported.

Snapshots carry the complete MuJoCo integration state. Each sensor keeps a
bounded local history, clears it on reset, and refuses to interpolate over a
missing interval. Only the world steps physics; sensing workers restore the
snapshots into their own model/data. `mj_forward` refreshes derived poses and
sensor readings after integration, before publishing the post-step sample.
Zenoh's native SHM optimization remains at the repository default. This code
does not require or claim zero-copy delivery.

```bash
uv run pytest dimos/sim2
uv run pytest -m self_hosted dimos/sim2  # Requires a working native OpenGL renderer.
uv run mypy --follow-imports=silent dimos/sim2
```

Default tests use small generated MJCFs without asset downloads. Coverage includes two
robots, complete PD commands, gripper units, exclusive writers, spawned and
independent-process readers, stale feedback/commands, reset races, scene edits,
RGB-D/raycast output, and a real headless DimOS deployment using Zenoh.
These are core integration checks, not robot-policy or loaded-performance results.

The explicit native canary builds current main's Point-LIO binary and drives a
0.6 m linear stage from the ordinary device client. It checks the estimator
against independently read truth, with and without a 640x480 RGB-D worker:

```bash
cargo build --release --locked --bin pointlio_native \
  --manifest-path dimos/hardware/sensors/lidar/pointlio/rust/Cargo.toml
uv run pytest dimos/sim2/sensors/lidar/test_mid360.py -m native_e2e \
  -o junit_family=legacy --junitxml=/tmp/sim2-mid360-native.xml
```

This requires the calibration asset and native OpenGL for the camera case.
Set `SIM2_POINTLIO_BINARY` for an externally built executable. The JUnit
`sim2_mid360_evidence` property records rates, frame age and pose errors.
This synthetic-room check is not validation of G1/Go2 locomotion, hardware
sensor equivalence, or a complex-scene performance claim.
