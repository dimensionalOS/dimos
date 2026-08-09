# Control Coordinator Refactor: Design Review Report

| Field | Value |
|---|---|
| Status | Proposed for discussion |
| Change | `refactor-control-coordinator-connections` |
| Prerequisite | Gripper API refactor, PR #3381 |
| Delivery | Seven-review stacked PR sequence |
| Compatibility policy | Intentional breaking cutover; no compatibility layer |
| Core boundary | Connections own device truth and safety; the coordinator arbitrates declared interfaces; manipulation controls one robot |

## Executive Summary

The current control coordinator is both an arbitration engine and a hardware framework. It selects adapter registries, constructs devices, owns connection lifecycle, polls state, performs conversion, and dispatches commands. Manipulation then carries hardware and robot identity through planning and execution. The result is a wide coordinator, shallow connections, duplicate naming systems, and safety behavior without one clear owner.

This design draws three firm boundaries:

```text
 semantic intent                arbitration and safety domain              device truth

 Manipulation / tasks          ControlCoordinator                         Connection modules
 --------------------          ------------------                         ------------------
 one prepared robot            one logical robot                          SDK/native lifecycle
 planning groups               exact interface claims                     ordering + conversion
 trajectories / goals    --->  winner selection + heartbeat       --->    limits + watchdog
 canonical names         <---  state cache + public status        <---    state + status
```

The coordinator no longer knows xArm, G1, CAN, DDS, gripper ranges, adapter registries, or driver-native joint order. A connection instance declares a resolved set of scalar control interfaces such as `left/j1/position`, publishes complete state snapshots, accepts sparse commands for its declared command profile, and independently enforces its watchdog and safe-stop policy. Multiple connection instances share fixed class-level streams; their configured module names provide source identity and namespace.

Manipulation controls exactly one logical robot from one statically prepared URDF/SRDF. A dual-arm model is one prepared model with `left/...` and `right/...` joints. Planning groups select subsets. Dynamic robot-part composition is a valid future concept, but it is not needed for current hardware and is deliberately absent from this change.

The design is large, but the rollout is layered. The first two PRs simplify the model and manipulation independently. The next PR establishes contracts. Three parallel connection-family PRs adopt them. One atomic coordinator cutover removes hardware ownership, and the final integration and verification PRs finish the system. Each merge point is a coherent product, not scaffolding for unfinished architecture.

## Review Map

This report is meant to be presented in the following order:

1. Agree on the three ownership boundaries in the executive summary.
2. Review the message and naming contracts under Decisions 2–5.
3. Review lifecycle and failure behavior under Safety / Simulation / Replay.
4. Confirm the deletion and deferral lists.
5. Review the stacked PR graph and merge gates.

The OpenSpec capability files define observable requirements. This report explains architecture and trade-offs. `tasks.md` provides the implementation checklist.

## Context

### Current shape

The coordinator currently constructs adapters from hardware configuration and registries, connects and activates them, polls state during its tick, groups winning commands by hardware ID, and writes driver-oriented arrays. The gripper refactor in PR #3381 strengthens task ownership and vector-valued gripper behavior, but its current branch still routes limits and adapter construction through coordinator-owned hardware.

Manipulation independently models several `RobotModelConfig` entries and exposes robot selectors. Backends then maintain maps from robot IDs to native model instances, planning groups translate between local and global joint names, execution splits trajectories by robot, and visualization mirrors those maps. The shipped dual-xArm blueprint is a mock use case, not a requirement to coordinate two independently modeled robots.

Existing low-level state has two patterns:

- Rich sensor data already uses typed streams, including `JointState`, `Imu`, images, poses, and twists.
- Coordinator control state bypasses that model: its tick polls adapters and constructs an internal state cache keyed by hardware identity.

`PoseStamped` is not a required device control-state type. Pose streams generally come from upstream estimation, kinematics, or task input. Their `frame_id` is a coordinate frame and must not become a connection or task-routing key. Full IMU, image, and lidar streams remain ordinary typed streams for localization, perception, logging, and replay.

### Module-system constraints

Module ports are class-annotation driven. `Module.__init_subclass__` installs ports, module construction instantiates them from type hints, and blueprint atoms snapshot the resulting stream references. Autoconnect joins every port with the same effective stream name and exact Python type. It supports several outputs on a shared bus, but it does not synthesize ports from resolved instance configuration.

Runtime-generated coordinator subclasses would therefore be fragile: worker processes serialize classes by importable module and qualified name, blueprint and RPC introspection depend on stable class identity, static typing cannot see generated attributes, and blueprint registry generation only sees source-level declarations. A fixed family of shared typed buses fits the framework without extending it.

Configuration is fully resolved after blueprint import. A blueprint helper cannot reliably derive the final interface manifest from raw constructor arguments because CLI, environment, and JSON overrides apply later. Connections must announce their resolved descriptions after startup, before the coordinator becomes ready.

### Transport constraints

LCM and Zenoh can carry shared multi-publisher state buses. Current SHM/pSHM channels are latest-wins and not multiwriter-safe: several connection processes can race the same segment metadata. One coordinator publishing commands to several subscribers is compatible with that topology; several connections publishing state are not. This design keeps message semantics transport-independent, uses LCM or Zenoh for all multiwriter control planes, and defers a safe typed multiwriter SHM implementation.

### Constraints accepted for this change

- PR #3381 lands first; this refactor preserves its useful task semantics and changes its ownership boundary.
- No backward-compatible adapter, robot-ID, or hardware-management facade is retained.
- The task registry and `TaskConfig(type=...)` remain unchanged.
- Manipulation scope is joint control. Non-joint planning is not generalized here.
- Canonical control values are `float64` in the first contract.

## Goals / Non-Goals

**Goals:**

- Make each connection module the sole owner of its device connection, native protocol, lifecycle, conversion, hard limits, watchdog, and safe stop.
- Make the coordinator a hardware-agnostic assembler and exact-interface arbitration engine for one logical robot.
- Allow several instances of the same connection class, including two identically implemented arms, without coordinator subclasses or hardware IDs.
- Use one canonical interface namespace from prepared model through execution and visualization.
- Represent joint position, velocity, effort, gains, gripper joints, and control-relevant sensor scalars through one small control-value contract.
- Keep rich sensor and semantic task streams typed and independent.
- Define startup readiness, transactional arming, stale-state faults, command heartbeat, emergency stop, and explicit recovery.
- Deliver the refactor as reviewable stacked PRs with tests and hardware validation gates.

**Non-Goals:**

- Dynamically composing robot parts or generating URDF/SRDF at runtime.
- Coordinating multiple logical robots from one manipulation module or coordinator.
- Redesigning the task registry or making task configuration a discriminated type hierarchy.
- Adding multiwriter-safe typed SHM.
- Changing pose/twist task routing or cleaning up current `PoseStamped.frame_id` use.
- Introducing a generic estimator-provider abstraction for control state.
- Supporting non-joint manipulation planning.
- Switching incompatible device command modes while armed.
- Generalizing scalar values beyond `float64` without a concrete device need.
- Preserving old APIs, configuration fields, names, or data paths.

## DimOS Architecture

### Target module graph

```text
                         semantic streams
                 PoseStamped / TwistStamped / trajectories
                                      |
                                      v
                              +----------------+
                              | Manipulation   |
                              | one model      |
                              | N groups       |
                              +-------+--------+
                                      |
                              tasks / trajectories
                                      v
 +----------------+    +--------------+---------------+    +----------------+
 | left Connection|--->|                              |--->| left Connection|
 | description    |    |      ControlCoordinator      |    | control_command|
 | status         |    |                              |    +----------------+
 | control_state  |    | descriptions -> ready        |
 +----------------+    | states -> cache -> tasks      |    +----------------+
                       | claims -> exact-key winners   |--->| right Connection|
 +----------------+    | heartbeat -> command frames  |    | control_command |
 | right Connection|-->| lifecycle -> transaction     |    +----------------+
 | description    |    +------------------------------+
 | status         |
 | control_state  |             public robot RPCs
 +----------------+                     |
                                      skills / shell / modules

 Rich sensors: Connection ----------------------> localization/perception/logging
               (Image, Imu, point clouds, etc.)    (coordinator is not connected)
```

### Static streams

| Stream | Direction | Type | Transport rule | Purpose |
|---|---|---|---|---|
| `control_description` | connections → coordinator | `ConnectionDescription` | LCM or Zenoh | Immutable resolved capabilities and policies |
| `control_status` | connections → coordinator | `ConnectionStatus` | LCM or Zenoh | Operational state and lifecycle acknowledgements |
| `control_state` | connections → coordinator | `ControlValues` | LCM or Zenoh initially | Complete scalar control-state snapshots |
| `control_command` | coordinator → connections | `ControlValues` | Typed SHM allowed; LCM/Zenoh valid | Sparse commands plus heartbeat epoch |
| `connection_control` | coordinator → connections | `ConnectionControl` | LCM or Zenoh | Arm, disarm, emergency stop, and clear-estop transactions |

Every connection class declares the same applicable class-level ports. The coordinator declares inverse ports. `instance_name="left"` and `instance_name="right"` distinguish module/RPC/config identity; each description and state frame also carries its source. No blueprint namespace remapping is needed for these global buses.

A connection may expose unrelated typed streams from the same physical device. For example, an IMU connection can publish a full `Imu` to localization and publish only acceleration or angular-rate scalars needed by a controller on `control_state`. An RGB stream does not connect to the coordinator merely because the camera belongs to the same robot.

### Blueprint and configuration shape

```python
autoconnect(
    XArmConnection.blueprint(instance_name="left", ...),
    XArmConnection.blueprint(instance_name="right", ...),
    ControlCoordinator.blueprint(connections=["left", "right"], ...),
    ManipulationModule.blueprint(model=prepared_dual_arm_model, ...),
)
```

The illustrative `connections` list is an allow-list of required source names, not a hardware registry. Resolved connection config remains on each connection instance. Existing per-instance configuration provides CLI and environment forms such as:

```text
--left.address ...             LEFT__ADDRESS=...
--right.address ...            RIGHT__ADDRESS=...
```

Relevant hardware-specific fields are removed from `GlobalConfig` when their connection is migrated. A connection publishes its final description only after all overrides are resolved. The coordinator validates descriptions and initial snapshots before reporting ready.

### Public module interfaces

The coordinator keeps a robot-level public surface:

| Keep or add | Remove |
|---|---|
| `list_interfaces` | `add_hardware` |
| `describe_interface` | `remove_hardware` |
| `get_state` | `list_hardware` |
| `list_tasks` / `describe_task` | adapter construction/registry APIs |
| trajectory execute/cancel | adapter-specific gripper RPCs |
| `arm` / `disarm` | dynamic task add/remove if no shipped caller remains |
| `estop` / `clear_estop` | robot-ID selectors |
| lifecycle and fault status | hardware-ID/task-name range plumbing |

DimOS `Spec` Protocols for coordinator callers are updated to this surface. Connection control uses typed streams rather than normal public lifecycle RPCs; narrowly scoped private diagnostics may remain. Adapter Protocols may remain inside a connection package as a driver seam, but no adapter Protocol or registry crosses into the coordinator.

Agent skills and MCP tools continue to expose semantic robot operations. Their schemas no longer accept `robot_name`, `robot_id`, or `hardware_id`. Low-level arm/disarm/estop tools, if exposed, call the coordinator’s robot-level lifecycle API.

Blueprint module-level variables remain source-declared. Changed built-in blueprint names or imports require regeneration with:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

## Decisions

### Decision 1: One coordinator and one manipulation module describe one logical robot

“Robot” is the caller-visible controllable unit. A dual-arm setup is one robot, not two robots coordinated by a hidden multi-robot layer. Manipulation receives one prepared model and planning groups select its joints.

The static model configuration contains one prepared URDF path, an optional SRDF path, a logical root frame, an optional whole-model placement, and planning groups. A group contains a stable name, canonical joint names, and group-specific base and tip links. There is no singular model-wide end-effector link because a bimanual robot has several tips.

```text
prepared robot model
├── joint: left/j1 ... left/gripper
├── joint: right/j1 ... right/gripper
├── group: left_arm  -> [left/j1, ...]
├── group: right_arm -> [right/j1, ...]
└── group: both_arms -> [left/j1, ..., right/j1, ...]
```

The dual-xArm mock uses a pre-generated combined URDF/SRDF fixture. Authored bimanual models use the same path. Runtime composition is deferred until real hardware and requirements justify a first-class robot-assembly capability.

Backend-native numeric handles may remain private implementation details. They are not domain robot IDs and must not leak into manipulation APIs, trajectories, state, or visualization.

**Alternatives rejected:** preserving a list of robots, keeping a singular end-effector compatibility property, or extracting a dynamic composition framework now.

### Decision 2: Canonical names are declared once

The connection instance namespace is reflected directly in model and interface names: `left/j1`, `right/j1`, `base/wheel_left`. These exact canonical names flow through model loading, planning groups, state, tasks, trajectories, coordinator commands, gripper handling, and visualization.

Control interface keys append the control field:

```text
left/j1/position
left/j1/velocity
left/j1/effort
left/j1/kp
left/j1/kd
left/gripper/position
imu/angular_velocity_x
```

The namespace is not inferred from random modules at runtime, and the coordinator does not semantically remap it. Configuration declares final names. A backend parser compatibility test decides whether `/` can be used natively. If a backend rejects it, that backend loader performs one private, reversible encoding; the canonical public name remains unchanged.

**Alternatives rejected:** `robot_id`, hardware-ID prefixes, local/global joint-name pairs, semantic mapping tables, and collision detection across unnamespaced modules.

### Decision 3: Control state and commands share one scalar payload shape

`ControlValues` is a concrete encodable message used on two separate streams. Its conceptual fields are:

| Field | Meaning |
|---|---|
| `source` | Connection instance for state; coordinator identity for commands |
| `source_ts` | Producer sample time |
| `seq` | Monotonic sequence within the producer epoch |
| `interface_names[]` | Fully qualified exact keys |
| `values[]` | Parallel `float64` values |

State frames are complete snapshots of every state interface declared by that source. A missing, duplicated, unknown, non-finite, or mismatched entry invalidates the entire frame. The coordinator retains the last valid snapshot until its staleness deadline, then faults if the source is required.

Command frames may be sparse. They contain only winning values for a connection plus the heartbeat epoch. Empty commands are meaningful: the coordinator is alive but has no active winner. A connection rejects a batch containing any undeclared or unsupported key; it never applies the valid subset of an invalid batch.

`source_ts` describes sampling. Coordinator-local monotonic receipt time drives staleness because device clocks are not assumed synchronized. `seq` rejects repeats and out-of-order frames and supports diagnostics.

Canonical units are SI: radians or meters for position, radians/second or meters/second for velocity, and Nm or N for effort. Vendor-native values and ordering stop at the connection boundary.

**Alternatives rejected:** `JointState` as both state and command, ordered unnamed `MotorCommandArray` as a robot-wide contract, one union envelope for all sensors and commands, and separate dynamically generated ports per connection.

### Decision 4: Exact scalar interfaces are the arbitration resource

The coordinator claims and selects winners for exact keys. Internal names become `InterfaceClaim`, `InterfaceCommand`, and interface arbitration. The coordinator does not understand joints, modes, atomic devices, arms, grippers, or end-effectors.

A connection exposes one statically configured compatible command profile. Examples:

| Connection profile | Command interfaces |
|---|---|
| xArm position | joint `/position` keys |
| xArm velocity | joint `/velocity` keys |
| G1 impedance | joint `/position`, `/velocity`, `/kp`, `/kd`, `/effort` keys |
| Gripper | one or more joint `/position` keys |

The profile is chosen in module configuration and recorded in the immutable description. Runtime profile switching while active is outside scope. There is no global “atomic interface” concept. A task claims whatever exact set must be coordinated for its behavior; a dual-arm task may claim both end-effector groups without inventing a robot boundary.

### Decision 5: Connections announce immutable resolved descriptions

`ConnectionDescription` is the source of truth after configuration resolution. It includes:

- source name and description epoch/version;
- declared state and command interface keys;
- joint/group metadata needed by callers;
- canonical units and hard/soft limits;
- command profile and supported fields;
- expected state rate and stale timeout;
- command watchdog timeout;
- per-interface omission behavior;
- connection-specific safe-stop behavior.

The coordinator expects a configured set of source names. It rejects duplicate command ownership, duplicate keys within a source, inconsistent metadata, unsupported units/profiles, and descriptions that do not match initial state. A description is immutable for a running epoch. A changed description forces a fault and explicit re-arm rather than silently changing the control surface.

This is runtime description exchange, not dynamic port construction. It preserves type-safe fixed buses while allowing config-resolved resources. It also supports two xArm modules because `left` and `right` declare disjoint final names.

**Alternatives rejected:** building a coordinator subclass in a blueprint helper, reading unresolved blueprint kwargs, a fixed maximum number of ports, and bypassing module streams to construct transports manually.

### Decision 6: Rich sensors remain domain-native streams

The scalar control bus contains only values required in the coordinator’s feedback/arbitration loop. Rich sensor messages retain their existing types and consumers:

| Data | Primary stream | Coordinator involvement |
|---|---|---|
| Complete joint feedback needed for control | `control_state` scalars | Yes |
| Full IMU sample | `Imu` | Only selected scalars if a controller needs them |
| RGB/depth image | `Image` | None |
| Point cloud/lidar | existing typed stream | None |
| Estimated pose | `PoseStamped` | Semantic/task input, not device identity |
| Velocity intent | `TwistStamped` or existing command type | Upstream task input |

This lets one connection module handle every interface of one device or robot controller without forcing the coordinator to subscribe to irrelevant bandwidth.

### Decision 7: Lifecycle is a separate reliable control plane

High-rate scalar values do not encode lifecycle. Three typed contracts define it:

- `ConnectionControl`: target source names, `operation_id`, and `ARM`, `DISARM`, `ESTOP`, or `CLEAR_ESTOP`.
- `ConnectionStatus`: source, state, active description epoch, current fault, last accepted operation, and acknowledgement result.
- `ConnectionDescription`: capabilities and safety policy, as above.

Lifecycle messages use a multi-publisher-safe transport such as LCM or Zenoh. Arming is a transaction: all required connections acknowledge activation before the coordinator reports armed. Any rejection or timeout triggers disarm rollback for every connection that activated. Emergency stop is fanout and remains latched until every required connection has cleared it; clearing does not arm the robot.

Normal users call robot-level coordinator RPCs. They do not arm connections one by one.

### Decision 8: The gripper is a joint at the control boundary

After PR #3381, `GripperControlTask` remains the sole task owner of gripper joints and keeps vector native-position, normalized-position, sweep, reference-pose, hold, and multi-DOF semantics. This refactor removes its dependency on coordinator hardware or adapter access.

Gripper state, limits, and commands use the same description and scalar contracts as every other joint. The connection owns vendor conversion. Manipulation selects a gripper through a planning/control group and does not cache ranges or construct a task name from hardware identity. Integrated and standalone grippers differ only in which connection owns their interfaces.

### Decision 9: Obsolete concepts are deleted at their cutover

No aliases or compatibility translations survive. The following domain concepts are removed where their replacements land:

- `RobotName`, `WorldRobotID`, robot selector parameters, and robot registries;
- local/global model-joint distinctions and robot-prefix parsing helpers;
- trajectory splitting and execution targets by robot;
- coordinator hardware objects, hardware add/remove/list RPCs, and adapter registries;
- hardware-ID command grouping and gripper task-name/range plumbing;
- hardware-specific global config fields superseded by connection instance config.

Private backend handles and private adapter helpers may remain when they improve locality, but their types and identities do not cross the new boundary.

## Safety / Simulation / Replay

### Safety state machine

```text
       descriptions + initial complete states
 BOOT --------------------------------------------> READY
  |                                                   |
  | invalid manifest/state                            | arm transaction succeeds
  v                                                   v
 FAULT <------------------------------------------- ARMED
  ^       stale state / watchdog / connection fault   |
  |                                                   | disarm
  | clear fault + fresh state                         v
  +----------------------------------------------- DISARMED

 Any state --ESTOP--> ESTOPPED --clear_estop--> DISARMED
                                      (explicit arm still required)
```

The default is not auto-enable. Readiness requires one immutable description and one complete valid state snapshot from every configured source. A required connection that becomes stale or faults causes robot-wide fault handling and safe stop. State recovery does not automatically resume motion; callers must clear the fault where applicable and issue a new arm and command.

Every connection implements an independent command watchdog. The coordinator publishes a command heartbeat at its control rate even when no task owns an interface. Missing command frames invoke the connection’s declared safe stop. Task timeout and coordinator watchdog are useful but never substitute for the connection-local watchdog.

Default omission behavior is explicit and discoverable:

| Interface | Command omitted while heartbeat continues |
|---|---|
| position | Retain last accepted target |
| velocity | Command zero |
| effort | Command zero |
| `kp` / `kd` | Retain configured or last accepted safe value |

A connection may declare a stricter policy. No policy may weaken hard limits or watchdog safe stop. Hard limits are enforced at the connection after canonical-to-native conversion; tasks and the coordinator may also use published limits for validation and planning.

### Simulation and replay

- Every connection contract first receives a deterministic fake that can publish descriptions/states, acknowledge lifecycle operations, inject stale frames and faults, and record commands.
- Simulation connections implement the same descriptions, command profiles, lifecycle acknowledgements, watchdog behavior, names, and units as hardware connections.
- Replay can publish historical state and status for consumers, but must never arm or command physical hardware. Any mixed replay/hardware blueprint must fail validation unless explicitly designed and tested later.
- Recorded old coordinator-owned adapter traffic is not supported through a compatibility decoder.
- The static dual-xArm fixture validates bimanual naming and planning without claiming hardware availability.

### Verification matrix

| Surface | Automated gate | Manual/hardware gate |
|---|---|---|
| Contract | codec, length, duplicate, unknown, stale, sequence tests | topic inspection |
| Coordinator | arbitration, readiness, transaction rollback, estop, heartbeat | shell/RPC lifecycle exercise |
| xArm + gripper | fake/sim loop and limits | single xArm smoke test including gripper |
| G1 | whole-body simulation, impedance profile, watchdog | platform-owner checklist for real G1 |
| Piper/OpenArm | mock contract and blueprint build | platform-owner bring-up checklist |
| Mobile bases | fake/sim velocity profile and safe zero | platform-owner smoke test where available |
| Manipulation | backend tests on one model and several groups | visual trajectory sanity check |
| Blueprints | build/import and generated registry test | representative sim/replay launch |

No real dual-arm test blocks the change because no such hardware use case currently exists.

## Risks / Trade-offs

| Risk or trade-off | Consequence | Mitigation |
|---|---|---|
| A broad breaking change touches many platforms | Long review and temporary branch divergence | Stack by coherent boundaries; merge prerequisite contracts before family migrations |
| Shared scalar keys lose domain structure | Debugging can become string-heavy | Immutable typed descriptions, strict validators, introspection RPCs, canonical naming |
| `float64` cannot carry future discrete modes | A later device may need another value kind | Add a concrete typed contract only when a real profile requires it |
| LCM/Zenoh state adds serialization/network cost | High-rate whole-body loops may expose latency | Benchmark G1; keep payload compact; pursue multiwriter-safe typed SHM separately |
| Runtime descriptions delay readiness | Startup becomes asynchronous | Explicit readiness state, timeout diagnostics, deterministic fake tests |
| Slash names may fail in a model backend | A backend could reject canonical URDF names | Make parser compatibility an early PR1 spike; encode once inside only that loader if needed |
| Sparse commands plus retention can hide old targets | Unexpected motion after ownership changes | Explicit omission policy, task cancellation tests, heartbeat epochs, zero velocity/effort defaults |
| Atomic coordinator cutover has a large diff | Review and rollback cost | Land contracts and connection implementations first; keep cutover behavior-focused and delete old path in the same PR |
| PR #3381 changes while this stack develops | Gripper integration assumptions drift | Start implementation only after merge; rebase the stack and validate task API before PR6 |

The chief intentional trade-off is using a shallow scalar wire format with a rich immutable description. That keeps the high-rate message small and generic while moving semantics to configuration and introspection. Strict startup validation is therefore essential, not optional polish.

## Migration / Rollout

### Stack graph

```text
PR #3381  Gripper API refactor (prerequisite)
    |
    v
PR 1  Static single-robot model
    |
    v
PR 2  Single-robot manipulation cutover
    |
    v
PR 3  Scalar control + lifecycle contracts
    |
    +-------------------+--------------------+
    v                   v                    v
PR 4a Manipulator/sim  PR 4b Mobile base    PR 4c G1 whole-body
    +-------------------+--------------------+
                        |
                        v
PR 5  Atomic coordinator cutover + blueprint migration
                        |
                        v
PR 6  Manipulation/control/gripper integration
                        |
                        v
PR 7  Cross-platform verification + docs
```

### Merge-unit intent

| PR | Product state after merge | Required deletion |
|---|---|---|
| 1 | All manipulation model consumers can load one prepared model with canonical groups | multi-model config surface where isolated |
| 2 | Manipulation plans and executes for one robot across one or several groups | robot IDs, registries, selector APIs, trajectory splitting |
| 3 | Stable messages, validators, fake connection, and safety semantics exist without changing production ownership | obsolete experimental control-message types if superseded |
| 4a | Manipulator and sim connections can speak the new contracts | coordinator-facing adapter assumptions in migrated packages |
| 4b | Mobile-base connections can speak the new contracts | migrated global hardware config |
| 4c | G1 exposes its configured whole-body profile and state | migrated special coordinator plumbing |
| 5 | Production coordinator consumes shared buses and owns no hardware | old hardware ownership, polling, registries, management RPCs |
| 6 | Manipulation, trajectories, and gripper tasks use canonical interfaces end to end | hardware/task/range mapping remnants |
| 7 | Supported blueprints, docs, and platform checklists describe only the new system | stale docs, demos, generated registry entries |

The stack may branch at PR4, but PR5 depends on every production connection family required by migrated blueprints. No PR introduces a compatibility shim or leaves both command paths active. If a merge unit cannot preserve a working product, its boundary must move rather than adding a temporary abstraction.

### Deployment and rollback

This is a source-level migration, not an in-place protocol upgrade. Deploy coordinator and all participating connections from the same release. The description version/epoch causes mismatched contracts to fail closed during readiness. Rollback means redeploying the previous complete release and its blueprints, not mixing old and new modules.

Blueprint registry changes require `pytest dimos/robot/test_all_blueprints_generation.py`. Normal formatting, mypy, targeted unit tests, blueprint build tests, sim/replay smoke tests, and platform checklists gate their corresponding PRs. Documentation lands alongside behavior and is completed in PR7.

## Open Questions

The architecture has no unresolved product decision from the design grill. The following are validation questions to answer during implementation, not invitations to broaden scope:

1. Which model backends accept `/` in native joint and link names? PR1 must record the result and isolate any required private encoding.
2. What measured control-state rate and payload size does G1 require, and does LCM or Zenoh meet it with margin? If not, that evidence seeds the separate multiwriter-SHM project.
3. Which existing dynamic task add/remove callers, if any, are shipped? PR5 removes the API if repository search confirms none.
4. Which connection families are required to keep every currently registered blueprint runnable at PR5? The migration inventory must make that dependency explicit.

### Deferred follow-up register

| Follow-up | Trigger to revisit |
|---|---|
| Dynamic robot-part assembly | A real robot must be assembled from independently shipped model components |
| Type-safe task configuration | A separate task API proposal owns the registry/config problem |
| Multiwriter-safe typed SHM | Benchmarks show LCM/Zenoh misses a real control-rate budget |
| Generic estimator control-state provider | A controller needs derived state not owned by a connection |
| Pose routing cleanup | Semantic task routing is redesigned across manipulation callers |
| Runtime command-profile switching | A real device workflow requires safe in-operation switching |
| Upper-level multi-robot coordination | A caller genuinely coordinates separate logical robots |

These follow-ups must not add hooks, compatibility fields, or abstractions to this refactor before their triggers occur.
