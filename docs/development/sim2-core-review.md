# sim2: the interface a caller uses and a backend implements

**Proposal for review with Jeff, not a new implemented runtime API.** The current
PR already has `describe_world`, `read_state`, `submit`, `set_paused` and `reset`.
This proposal gives those calls precise, smaller data types and behavior.
The complete typed contract is [sim2_contract.py](/docs/development/sim2_contract.py).
The Python examples below import that proposal and define functions; they do not
command the running simulator. Do not send these new types to the current PR.

## 1. What a user does

Run the ordinary robot blueprint. The existing coordinator collects robot
instances from their connections, adds one simulation module, and supplies the
selected backend with the robot models and scene before it builds the world.

```bash skip
dimos --simulation mujoco --transport zenoh run unitree-g1-groot-control
```

The store/Unreal blueprint remains `unitree-g1-groot-wbc-unreal`, with the existing
scene-package and renderer options. Desktop consumes its normal robot, TF, image
and point-cloud streams. This proposal changes the physics boundary, not that
launch workflow. A compatible backend selection changes the engine; it must
validate the requested model/control profile before starting.
Today the Genesis G1 path supports control-only; the Mid360/Point-LIO navigation
blueprint still requires MuJoCo. A shared physics contract does not automatically
port sensor implementations or make native asset formats interchangeable.

An application obtains the running module through the existing DimOS client.
Inside another module, the same `Simulation` Spec can be injected normally.
This connection example is for **after** the proposed types are implemented:

```python
from dimos.core.global_config import global_config
from dimos.porcelain.dimos import Dimos
from docs.development.sim2_contract import Simulation


def connect_to_sim() -> tuple[Dimos, Simulation]:
    global_config.update(transport="zenoh")
    app = Dimos.connect(timeout=15)
    return app, app.find_module_by_spec(Simulation)
```

The caller gets five operations; it never runs the physics loop itself:

| Call | What it does |
| --- | --- |
| `sim.describe_world()` | Lists valid body/joint IDs, body mobility, supported joint drive types, model revision and physics step duration. |
| `sim.read_state()` | Returns the latest completed, coherent physical state. It does not advance time. |
| `sim.submit(batch)` | Queues a partial scene command for the next physics boundary; returns accepted/rejected and its reason. |
| `sim.set_paused(True / False)` | Stops/resumes physics time. No pose change or reset. |
| `sim.reset()` | Restores the configured initial state, cancels pending actions and starts a new epoch. |

### Push G1 while its controller keeps it standing

```python
from uuid import uuid4
from docs.development.sim2_contract import (
    BodyWrench, CommandBatch, CommandReport, Simulation,
)


def push_g1(sim: Simulation) -> CommandReport:
    state = sim.read_state()
    return sim.submit(CommandBatch(
        command_id=uuid4().hex,
        epoch=state.id.epoch,
        commands=(BodyWrench("g1/pelvis", force_world=(20.0, 0.0, 0.0)),),
        duration=0.1,
    ))
```

This asks for **20 N along world +X at the pelvis center of mass for 0.1 simulated
seconds**. G1's controller continues supplying its 29 motor targets. The caller
supplies only the push. `SimulationModule` combines it with those targets.

For a world with `step_dt=0.005`, accepted after tick 100: the force first affects
100→101; `applied` names state 101. It affects 20 intervals through 119→120, then
`expired` names state 120. It is absent from 120→121. Acceptance does not promise
that G1 stays upright or reaches a target position.

Subscribe to `app.SimulationModule.get_stream("world_state")` for resulting motion
and `get_stream("command_reports")` for this command's lifecycle. Both use the
existing `.subscribe(callback)` interface. Subscribe before submitting; the
returned unsubscribe functions detach listeners. `app.stop()` disconnects this
remote client without stopping the running blueprint.
Publication may be slower than physics; gaps between published tick numbers do
not imply skipped physics steps. `read_state()` returns the latest completed tick.

### Read what actually happened

```python
from dimos.msgs.geometry_msgs.Pose import Pose
from docs.development.sim2_contract import ScalarJointState, Simulation


def read_g1(sim: Simulation, knee_joint: str) -> tuple[Pose, ScalarJointState]:
    state = sim.read_state()
    knee = state.joints[knee_joint]  # canonical ID from describe_world().joints
    if not isinstance(knee, ScalarJointState):
        raise ValueError("Expected a scalar G1 knee joint")
    return state.bodies["g1/pelvis"].pose, knee
```

The knee contains its measured angle, speed and actual actuator effort. The
pelvis contains its world pose and twist. A command report records that an input
was applied; state records the physical response. Actual motor effort is not an
echo of the target. A complete log of requested inputs belongs to runtime
tracing, correlated with the resulting state ID.

### A box, a door and a robot link use the same body-force call

| Target | Request | Meaning |
| --- | --- | --- |
| Free box body | `BodyWrench("box", force_world=(20, 0, 0))` | Push at its center of mass; collisions and inertia determine motion. |
| Passive door leaf | `BodyWrench("door_leaf", torque_world=(0, 0, 3))` | Apply a 3 Nm couple; its hinge constrains the resulting motion. |
| Motorized door hinge | `JointPosition("door_hinge", position=1.0)` | Ask its declared native position servo for 1 rad. |
| PD-driven hinge | `JointPD("door_hinge", 1.0, 0.0, 30.0, 2.0, 0.0)` | Ask for a 1 rad target with explicit gains, desired speed and feedforward. |
| Static store fixture | Any `BodyWrench` | Reject: the body is static. |
| Passive hinge | Either joint-drive command | Reject: no supported actuator. |
| G1 knee owned by its controller | Generic scene joint-drive command | Reject: use the existing robot control path or deliberately select a different controller. |

A **body** has pose, velocity and inertia. A **joint** constrains relative motion
between bodies. An **actuator** supplies effort to an allowed joint coordinate.
G1 is a collection of these bodies/joints, not another physics entity type.
There is no generic action that teleports an object or makes a joint hit its
requested position instantly.

The current Konbini store fixtures are static. The free box and hinged door above
are proposed additions to the G1 conformance scene, not claims about those assets.
Making them movable requires a physical body/inertia/collision and, for the door,
a hinge in the authored scene. Unreal visual meshes alone do not provide that.

## 2. Exact state and action types

[The contract file](/docs/development/sim2_contract.py) is authoritative for field
names and Python types. These are the contents, not extra messages to implement:

| Value | Exact contents |
| --- | --- |
| `StateId` | `epoch: UUID`, `tick: int` |
| `WorldState` | `id: StateId`, `sim_time: float`, `model_revision: str`, `bodies: Mapping[str, BodyState]`, `joints: Mapping[str, ScalarJointState \| BallJointState]` |
| `BodyState` | `pose: Pose`, `twist: Twist` |
| `ScalarJointState` | `position: float`, `velocity: float`, `actuator_effort: float \| None` |
| `BallJointState` | `orientation: Quat`, `angular_velocity: Vec3`, `actuator_torque: Vec3 \| None` |
| `WorldDescription` | `model_revision: str`, `step_dt: float`, `bodies: Mapping[str, dynamic/static/kinematic]`, `joints: Mapping[str, JointInfo]` |
| `JointInfo` | `kind: revolute/prismatic/spherical/fixed`, `drive: pd_effort/native_position/None` |
| `JointPD` | `joint: str`, `position, velocity, kp, kd, feedforward: float` |
| `JointPosition` | `joint: str`, `position: float` |
| `BodyWrench` | `body: str`, `force_world: Vec3`, `torque_world: Vec3`, `point_world: Vec3 \| None` |
| `CommandBatch` | `command_id: str`, `epoch: UUID`, `commands: tuple[JointPD \| JointPosition \| BodyWrench, ...]`, `duration: float` |
| `CommandReport` | `command_id: str`, `state: StateId`, `status: accepted/applied/rejected/expired/cancelled`, `reason: str` |

Coordinates and completeness are part of the contract:

- SI units, right-handed Z-up world; quaternion order **xyzw**. Pose and both
  twist vectors use world axes. Linear velocity is at the authored body origin,
  not at its center of mass. Joint values use shared-model signs/zeros/axes.
- Scalar joints use rad/rad·s⁻¹/Nm or m/m·s⁻¹/N. Ball orientation maps child joint
  coordinates into parent joint coordinates; relative angular velocity and torque
  use child joint axes. Ball-joint drives are outside the first core contract.
- `bodies` includes every described physical body. `joints` includes every
  described non-fixed joint. Free roots appear only as body state; fixed joints
  are topology only. Sensor/optical frames resolve through the shared model.
- Reported effort is the actuator contribution used in the final internal
  integration step, after transmission/limits. It excludes contact reactions,
  passive forces and external wrenches. `None` on initial/reset state or if
  unavailable. It is not a whole-step average.
- All state values come from one completed step and own their data; no live engine
  buffer aliases. `model_revision` identifies the compiled model/profile; a
  topology/profile change requires rebuild. This state is not an engine checkpoint.

### Submission, ownership and lifetime

- Validate the whole batch before enqueueing: current epoch, nonempty unique
  command ID for that epoch, nonempty commands, finite values, nonnegative
  duration/gains, valid IDs and supported drive types. Reject the whole batch on
  error, with a reason; no partial application.
- Queue in owner acceptance order and apply at the next available physics
  boundary. Wrenches add. Duplicate/overlapping joint writers are rejected,
  including robot-owned joints. A batch reserves its scene joints until expiry.
- Duration starts at the first affected interval. `0` means one public step;
  positive durations use `ceil(duration / step_dt)` intervals. Time is simulated
  time; pausing or slow rendering does not consume the duration.
- A world-space force remains fixed in world axes. `point_world=None` follows the
  body's instantaneous center of mass. Otherwise the point is fixed in world
  coordinates and contributes `(point_world - com_world) × force_world` in
  addition to `torque_world`, recomputed at every internal integration substep.
  Only dynamic bodies accept wrenches, including bodies constrained by hinges.
- Reports: accepted/rejected names the latest state at admission; applied names
  the first completed affected tick; expired names the final affected tick. Reset
  cancels queued/active commands against the last old-epoch state, then publishes
  the new initial state. A backend failure stops stepping and cancels active work
  with a reason; it must not emit an applied report for a failed step.
- A scene PD drive returns to zero gains/feedforward on expiry. A native position
  servo holds the position measured at expiry; removing its target does not mean
  removing its authored servo. The runtime supplies these fallback inputs
  explicitly. Robot fallback belongs to the selected control profile.
- Pause preserves epoch and active commands; submissions while paused wait for
  resume. Reset clears commands, restores initial state, creates a new UUID epoch
  and returns tick/time to zero. The model stays loaded. If a separate wall-clock
  connection-loss rule expired while paused, resume waits for fresh robot control
  instead of silently replacing the held controller targets with damping.

## 3. What happens inside the module

The caller submits a **partial batch**. The runtime alone constructs a **complete
input for one physics step**. That separation is why pushing a box does not
require the caller to send 29 G1 motor targets.

| Runtime input | Translation before stepping |
| --- | --- |
| GR00T → existing G1 connection → `MotorCommandArray` | All 29 `q/dq/kp/kd/tau` entries become `JointPD`, using the connection's motor-order-to-model-ID mapping. |
| Stock G1 velocity command → its locomotion policy | All 29 position targets become `JointPosition`; native servo gains/dynamics stay in the model profile. |
| Application `CommandBatch` | Expand active durations into this step's scene drives and body wrenches. |
| Missing/expired control | Supply the declared control-profile fallback and record why; no backend-specific implicit hold. |

The actual G1 conversion functions are in
[sim2_contract.py](/docs/development/sim2_contract.py). One G1 connection remains.
Stock and GR00T are different control/physics profiles of that connection.
Hardware still receives `MotorCommandArray`; this does not replace its message.

The engine implementer implements precisely:

```python
from typing import Protocol
from docs.development.sim2_contract import StepInput, WorldDescription, WorldState


class PhysicsBackend(Protocol):
    def build(self) -> WorldDescription: ...
    def read_state(self) -> WorldState: ...
    def step(self, inputs: StepInput) -> WorldState: ...
    def reset(self) -> WorldState: ...
    def close(self) -> None: ...
```

`StepInput` contains `from_state: StateId`, `drives: tuple[JointPD | JointPosition, ...]`
and `wrenches: tuple[BodyWrench, ...]`. The existing composition factory configures
models/instances/scene before `build()`; this proposal does not add a scene format.

- Build returns the compiled description and establishes tick/time zero with a
  new epoch. Read returns that state or the latest completed state without stepping.
- Each step validates everything before mutation. `from_state` must exactly match
  current epoch/tick. Every declared driven joint has exactly one drive of its
  declared type. Unsupported actuation/transmissions fail at build.
- Success advances exactly `step_dt`, increments tick once and returns the new
  state. Invalid input changes nothing. Internal solver substeps are backend-owned.
- For PD effort, calculate `kp*(q_target-q) + kd*(dq_target-dq) + feedforward` at
  each integration step, then apply through the declared actuator/transmission,
  retaining limits and dynamics. Native position drives retain authored servos.
- Wrenches affect this public step only. Reset creates the initial state of a new
  epoch. Close releases engine resources and is safe to repeat. Calls after close
  fail; build is called once per backend instance.

Physics, command admission and reset have one owner. Sensor work, rendering and
network publication run outside its critical section, consuming completed state.
Control observations use simulation time. The runtime records input source,
accepted age, effective drives, fallback decisions and resulting tick. A separate
wall-clock connection-loss rule must not disguise scheduler stalls as new motor
commands. See the recorded [idle-jolt evidence](/docs/development/sim2-idle-jolts.md).

Reset must also synchronize controller history and reject late old-epoch motor
samples. `MotorCommandArray` currently has no epoch: the sim control bridge needs
an epoch-bearing envelope or a proven reset barrier. Merely stamping an old motor
packet with the current epoch at receipt is insufficient. This remains required
implementation work, not a guarantee already provided by the proposal file.

## 4. The specific handoff to Jeff

**Shared model owns names/topology/geometry and authored frames. Simulation owns
changing physical state. UrdfManager publishes the resulting TF and model assets.**

| Consumer asks the shared model for | Example/result |
| --- | --- |
| Instance-scoped physical body and joint IDs | `g1/pelvis`, plus canonical G1 joint IDs used in state/commands. |
| Joint topology, origins, axes, limits, geometry and inertia | Compile a backend or serve the same robot model to planning/Desktop. |
| Resolve a named fixed frame to a physical ancestor | `g1/mid360` → physical body ID plus fixed body-to-lidar transform. |

A sensor resolves that mount once. At acquisition time it composes the physical
body pose from **that acquisition's WorldState** with the fixed mount transform.
No live TF query inside physics; no repeated camera/lidar offsets in sim2.
Measurement settings, noise and acquisition rates remain sensor configuration.
Root TF comes from the simulated root body; joint TF comes from the manager.
Each TF edge has one publisher. No repeated robot pose/joint arrays in WorldState.

[RobotModel](/dimos/robot/assets/model.py) already handles URDF/Xacro, variants,
renamed joints and `with_fixed_frame()`. Typed link/inertia/collision catalogs and
instance-aware frame resolution need extension. [UrdfManager](/dimos/robot/urdf_manager.py)
already serves assets and publishes fixed/scalar joint TF; floating-root TF and
partial-state validity need an explicit ownership contract. It currently defaults
missing joint positions to zero, which must not become simulated physical truth.

For a robot **with an existing DimOS connection**, the intended addition is one
shared model registration/configuration, zero new sim connection modules and zero
repeated mount transforms. A brand-new hardware robot still needs its connection
and possibly a controller. Backend import/generated artifacts are outputs, not
second authored robot definitions.

G1 still has distinct stock/GR00T native assets today. One canonical authored
model plus named physics/control profiles is the target; their trained dynamics
must be reconciled and validated before deleting those assets. This handoff does
not claim that moving mounts into RobotModel already solves model duplication.

## 5. What changes from this PR

| Current implementation | Proposed change |
| --- | --- |
| `WorldState.robots` repeats root/joints and embeds IMU | Keep body/joint state; robot adapters project standard feedback, sensor adapters acquire IMU. |
| Backend requires task action/reward, visual and checkpoint APIs | Require only the five physics lifecycle methods. Keep robosuite tasks, sensors, Unreal and native viewer as optional adapters/capabilities. |
| One overloaded `JointDrive`; different omitted-target behavior across engines | Explicit PD/native-position types and complete inputs assembled by the runtime. |
| Generic joint batches overwrite robot controller drives | Reject conflicting ownership; external body pushes remain independent. |
| Pause/resume increments epoch and clears commands | Preserve state/epoch/commands on pause; reset is the explicit invalidation boundary. |
| Sim mounts, visual attachments and G1 TF constants repeat authored frames | Resolve the same shared model frames once; retain engine/renderer compilation in adapters. |
| Wall-clock watchdog and publications share the physics lock | Make effective control and its clock explicit; publish outside the owner critical section. |

Implement and validate against full G1 **stock and GR00T** first: all 29 joints,
root state, pelvis push, a movable box/passive door, command expiry, reset with
late inputs, and CPU overload. Then run the same contract cases on MuJoCo and
Genesis. Common units/action semantics are required; identical trajectories are
not. Engines must reject unsupported assets/actuators explicitly.

Sources: [physics.py](/dimos/sim2/physics.py),
[runtime.py](/dimos/sim2/world/runtime.py), [module.py](/dimos/sim2/module.py),
[G1 model profiles](/dimos/robot/unitree/g1/model.py),
[MuJoCo backend](/dimos/sim2/backends/mujoco.py),
[Genesis backend](/dimos/sim2/backends/genesis.py).
