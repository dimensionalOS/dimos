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

## G1 and Go2 task workflows

The current task delivery is G1 GR00T and Go2. Arm migrations and the rolling
Mid360 are separate review branches; legacy arm engines/evals remain available.

`dimos.evals.suites.sim2_mobile` defines three cases for each robot using the
existing `EvalCase`, `Sim` environment and `EvalRunner`. It adds no task compiler
or benchmark population system. The authored room is robot-free MJCF with a
semantic sidecar. Both robot stacks use the same room and their existing
navigation configuration. The crate starts in storage behind the spawn and is
moved into the route only in the replanning case. These are workflow checks,
not a general obstacle-avoidance benchmark.

| Case | Setup / intervention | Grading |
| --- | --- | --- |
| Reach | Start at the authored spawn, request world goal (1.7, 0) | Reach within 0.35 m and remain there for at least one simulated second |
| Replan | Move an existing crate into the route after the robot advances | Same goal/dwell, after the recorded intervention |
| Search | Move a green box after the first approach | Reacquire it from RGB-D and remain within 1.25 m of its new position |

All cases reject a fallen robot or a sampled robot contact with the walls,
crate or target. Contacts are sampled at 10 Hz, not an every-physics-tick
collision audit. The scripted search reference is a colour/depth detector,
not a general object-perception model. Navigation still runs through the public
planner and connection APIs; the reference never reads scene truth.

```bash
# All six cases, no model API needed; run only one live eval at a time.
uv run dimos evals run dimos.evals.suites.sim2_mobile --agent dimos.evals.agents.mobile_scripted
# Same environments and graders, with an installed/configured Pi model agent:
uv run dimos evals run dimos.evals.suites.sim2_mobile --agent dimos.evals.agents.pi
# Add --tags g1 or --tags go2 to select a robot.
```

`Sim2Environment` starts a fresh application for every case. This resets the
controller, maps and perception by restarting their owners. It deliberately
rejects `attach=True`; bare `SimulationModule.reset()` cannot provide that
application-reset contract. Runtime object edits stay within the physical
episode and advance only the command/sensing invalidation `generation`.

The environment produces three artifacts beside the existing recording:

- `memory.db`: public camera/depth, odometry, lidar, TF, commands, path and costmap.
- `truth.db`: complete `SceneState` samples; intervention requests, before/after
  states and applied ticks/times; a completion marker after clean recording.
- `setup.json`: environment configuration, scene hash/description, sim2 run
  provenance and the first recorded state.

Only the public recording is handed to the evaluated agent. The grader receives
all artifacts and rejects missing completion, unrecorded mutations, mixed
episodes or simulation-time gaps over 0.5 s. A private artifact is **not a
sandbox**: a shell with access to these files or simulator RPCs can still read
truth. Model-agent isolation and measured model task success remain separate
acceptance work. The suite does not claim deterministic replay or sim-to-real
calibration.

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

Tests use small generated MJCFs without asset downloads. Coverage includes two
robots, complete PD commands, gripper units, exclusive writers, spawned and
independent-process readers, stale feedback/commands, reset races, scene edits,
RGB-D/raycast output, and a real headless DimOS deployment using Zenoh.
These are core integration checks, not robot-policy or loaded-performance results.
