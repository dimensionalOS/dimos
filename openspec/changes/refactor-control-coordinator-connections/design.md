# Control Coordinator Refactor: Design Review Report

| Field | Value |
|---|---|
| Status | Proposed for discussion |
| Change | `refactor-control-coordinator-connections` |
| Prerequisite | Gripper API refactor, PR #3381, merged 2026-08-25 |
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
 planning groups               whole-joint claims                         ordering + conversion
 trajectories / goals    --->  winner selection + heartbeat       --->    limits + watchdog
 canonical names         <---  state cache + public status        <---    state + status
```

The coordinator no longer knows xArm, G1, CAN, DDS, gripper ranges, adapter registries, or driver-native joint order. A connection instance declares canonical joint resources and their supported scalar command interfaces, publishes complete state snapshots, performs safe command-interface transitions, and independently enforces its watchdog and safe-stop policy. Tasks contend for whole joints; each winning task alone selects the command interfaces used for its joints.

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

The coordinator currently constructs adapters from hardware configuration and registries, connects and activates them, polls state during its tick, groups winning commands by hardware ID, and writes driver-oriented arrays. The merged gripper refactor in PR #3381 strengthens task ownership and vector-valued gripper behavior; this design preserves those semantics while moving limits and device ownership to connections.

Current command switching has no readiness state or acknowledgement path. `ConnectedHardware.write_command()` stores the newest targets, calls `adapter.set_control_mode()` when the requested mode changes, and immediately writes if that call returns `True`. The tick loop ignores the method's boolean result; tasks receive no failure or readiness signal, and a trajectory clock advances from its first compute tick whether or not hardware accepted the mode. Mode-conflicting winners on different joints of one hardware component are dropped during routing. The refactor must move this synchronous assumption into connection-owned behavior rather than reproduce the silent failure behavior.

The module framework also has no operational-readiness contract. Blueprint construction waits for synchronous `start()` RPCs to return, while `ModuleCoordinator.health_check()` only checks whether worker processes still have PIDs. A connection can log an initialization failure and return normally, leaving the blueprint apparently started although the hardware is unusable. This refactor introduces universal, live module readiness rather than adding a coordinator-specific command-ready signal.

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

- PR #3381 is merged; this refactor preserves its task semantics and changes its ownership boundary.
- No backward-compatible adapter, robot-ID, or hardware-management facade is retained.
- The task registry and `TaskConfig(type=...)` remain unchanged.
- Manipulation scope is joint control. Non-joint planning is not generalized here.
- Canonical control values are `float64` in the first contract.

## Goals / Non-Goals

**Goals:**

- Make each connection module the sole owner of its device connection, native protocol, lifecycle, conversion, hard limits, watchdog, and safe stop.
- Make the coordinator a hardware-agnostic assembler and per-joint arbitration engine for one logical robot.
- Allow several instances of the same connection class, including two identically implemented arms, without coordinator subclasses or hardware IDs.
- Use one canonical interface namespace from prepared model through execution and visualization.
- Represent joint position, velocity, effort, gains, gripper joints, and control-relevant sensor scalars through one small control-value contract.
- Keep rich sensor and semantic task streams typed and independent.
- Define universal live module readiness, transactional arming, layered safe stop, stale-state faults, command epochs and watchdogs, emergency stop, connection-specific shutdown, and explicit recovery.
- Deliver the refactor as reviewable stacked PRs with tests and hardware validation gates.

**Non-Goals:**

- Dynamically composing robot parts or generating URDF/SRDF at runtime.
- Coordinating multiple logical robots from one manipulation module or coordinator.
- Redesigning the task registry or making task configuration a discriminated type hierarchy.
- Adding multiwriter-safe typed SHM.
- Changing pose/twist task routing or cleaning up current `PoseStamped.frame_id` use.
- Introducing a generic estimator-provider abstraction for control state.
- Supporting non-joint manipulation planning.
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
                       | claims -> joint winners       |--->| right Connection|
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
| `control_status` | connections → coordinator | `ConnectionStatus` | LCM or Zenoh | Robot lifecycle state, faults, and correlated acknowledgements |
| `control_state` | connections → coordinator | `ControlValues` | LCM or Zenoh initially | Complete scalar control-state snapshots |
| `control_command` | coordinator → connections | `ControlValues` | Typed SHM allowed; LCM/Zenoh valid | Sparse commands plus heartbeat epoch |
| `connection_control` | coordinator → connections | `ConnectionControl` | LCM or Zenoh | Arm, safe-stop, emergency-stop, and clearance transactions |
| `coordinator_joint_state` | coordinator → robot-state consumers | `JointState` | Existing compatible transport | Aggregated canonical position, velocity, and effort feedback |

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

Relevant hardware-specific fields are removed from `GlobalConfig` when their connection is migrated. A connection publishes its final description only after all overrides are resolved. The coordinator combines the framework readiness of each required connection with description and initial-snapshot validation before reporting the robot ready.

### Public module interfaces

The coordinator keeps a robot-level public surface:

| Keep or add | Remove |
|---|---|
| `list_interfaces` | `add_hardware` |
| `describe_interface` | `remove_hardware` |
| `get_state` | `list_hardware` |
| `list_tasks` / `describe_task` | adapter construction/registry APIs |
| trajectory execute/cancel | adapter-specific gripper RPCs |
| `arm` / `safe_stop` / clear operations | dynamic task add/remove if no shipped caller remains |
| `estop` / `clear_estop` | robot-ID selectors |
| lifecycle and fault status | hardware-ID/task-name range plumbing |

DimOS `Spec` Protocols for coordinator callers are updated to this surface. Connection control uses typed streams rather than normal public lifecycle RPCs; narrowly scoped private diagnostics may remain. Adapter Protocols may remain inside a connection package as a driver seam, but no adapter Protocol or registry crosses into the coordinator.

Agent skills and MCP tools continue to expose semantic robot operations. Their schemas no longer accept `robot_name`, `robot_id`, or `hardware_id`. Low-level arm, safe-stop, and emergency-stop tools, if exposed, call the coordinator's robot-level lifecycle API.

The coordinator continues to publish `coordinator_joint_state` for manipulation, visualization, and other robot-state consumers. It projects `left/j1/position`, `left/j1/velocity`, and `left/j1/effort` from its validated scalar cache into one `JointState` entry named `left/j1`. The prepared model uses that same joint name, so consumers perform no semantic mapping. Control-only fields such as gains remain available through coordinator introspection and task state but do not masquerade as `JointState` fields.

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

### Joint-Name Mapping Deep Dive: One Name from Model to Wire

The canonical namespace removes a chain of identity conversions, not merely one prefix operation. Today, a joint can acquire a model-local name, robot-qualified name, world-robot association, execution mapping, hardware association, adapter position, and special gripper name before a command reaches the driver. Each layer must know how to translate to the next and, for state feedback, invert the translation correctly.

Conceptually, the current path resembles this:

```text
caller
  (robot_name, local_joint_name)
          |
          v
planning group
  (robot_name, local_joint_names[])
          |
          v
manipulation global name  <---- RobotModelConfig.joint_name_mapping
          |
          v
world/backend
  (WorldRobotID, backend-local joint)
          |
          v
execution target
  split by robot + invert name mapping
          |
          v
coordinator
  group by hardware_id
          |
          v
adapter
  name -> ordered array index -> driver-native joint
```

The proposed path carries one public name through every layer:

```text
prepared model       planning group      trajectory
  left/j1       --->   left/j1      --->   left/j1
                                               |
                                               v
task claim          winning command       connection description
  left/j1      ---> left/j1/position  ---> owner: left
                                               |
                                               v
                                      private driver boundary
                                      left/j1 -> index 0
```

State follows the same path in reverse. The connection labels driver index `0` as `left/j1/position`; the coordinator caches that exact key; manipulation and visualization consume `left/j1` without reconstructing a robot prefix or consulting another mapping table.

| Concern | Current design | Proposed design |
|---|---|---|
| Distinguish two arms | Robot/model IDs plus name prefixes | Prepared model and connection config both declare `left/...` and `right/...` |
| Select planner joints | Robot name plus model-local joint names | Planning group lists canonical joint names |
| Load a planning backend | Robot ID mapped to a backend model instance and native names | One model; optional private name encoding inside the loader |
| Execute a trajectory | Split by robot, map names, then invert mappings for results | Submit one trajectory with canonical names |
| Route a command | Derive hardware ID, group values, then order adapter arrays | Description identifies the connection that owns each canonical joint |
| Read state | Poll hardware-keyed adapters and rebuild global names | Connection publishes canonical keys directly |
| Address a gripper | Special hardware suffix, task name, and cached range | Ordinary canonical joint such as `left/gripper` |
| Visualize state | Per-robot joint-state maps | One canonical joint-state set for one model |

This change removes several classes of error:

- forward and inverse mappings that disagree;
- state and command paths that spell the same joint differently;
- joint lookup that requires both robot identity and hardware identity;
- trajectories that lose or duplicate joints while splitting and merging;
- gripper behavior that depends on parsing `/gripper` or deriving a task name;
- configuration changes that update a model mapping but not an execution or adapter mapping.

The design does not claim that all translation disappears. Two translations may remain, each at a narrow private boundary:

1. A model loader may reversibly encode canonical names if its backend rejects `/`.
2. A connection maps canonical names to fixed driver indices and converts canonical SI values to native units.

Neither translation creates a second domain identity. The backend returns canonical names before data leaves the loader, and the connection publishes canonical names before state leaves the device boundary. No intermediate module can observe or depend on the encoded name or native index.

Startup validation makes the simplification enforceable. Prepared-model joints, planning-group references, and connection-described joint resources must agree exactly. Missing joints, duplicate ownership, and undeclared interfaces fail before arming. This turns name alignment from a runtime convention into a checked system invariant.

### Decision 3: Control state and commands share one scalar payload shape

`ControlValues` is a concrete encodable message used on two separate streams. Its conceptual fields are:

| Field | Meaning |
|---|---|
| `source` | Connection instance for state; coordinator identity for commands |
| `source_ts` | Producer sample time |
| `epoch` | Producer epoch for state; current arm/control epoch for commands |
| `seq` | Monotonic sequence within the producer epoch |
| `interface_names[]` | Fully qualified exact keys carried in this frame |
| `values[]` | Parallel `float64` values |

Every frame is self-describing: `interface_names[i]` identifies `values[i]`. Consumers match exact names rather than interpreting a slot through description-defined ordering. `ConnectionDescription` declares which names are legal and complete; it does not remove names from high-rate frames. This modest repetition preserves sparse messages, readable traces, and direct validation without a presence bitmap or an implicit schema-index coupling.

State frames are complete snapshots of every state interface declared by that source. A missing, duplicated, unknown, non-finite, or mismatched entry invalidates the entire frame. The coordinator retains the last valid snapshot until its staleness deadline, then faults if the source is required.

Command frames may be sparse. One coordinator frame contains the current winning interface values across the robot plus the heartbeat epoch. Every connection receives it and consumes only names declared in its description. Empty commands are meaningful: the coordinator is alive but has no active winner. A connection rejects the batch for its interfaces if any targeted name is undeclared, unsupported, or malformed; it never applies the valid subset of an invalid batch.

`source_ts` describes sampling. Coordinator-local monotonic receipt time drives staleness because device clocks are not assumed synchronized. `seq` rejects repeats and out-of-order frames and supports diagnostics.

Canonical units are SI: radians or meters for position, radians/second or meters/second for velocity, and Nm or N for effort. Vendor-native values and ordering stop at the connection boundary.

**Alternatives rejected:** `JointState` as both connection state and command, description-indexed unnamed values with presence bitmaps, ordered unnamed `MotorCommandArray` as a robot-wide contract, one union envelope for all sensors and commands, and separate dynamically generated ports per connection. `JointState` remains the named robot-facing projection for position, velocity, and effort.

### Decision 4: Arbitration owns joints; winners select command interfaces

The coordinator selects one winning task for each claimed joint. A task claim contains canonical joint names and priority, not position-, velocity-, or effort-specific resource keys. The winning task alone supplies every command field for that joint. Two tasks may contend for the same joint while proposing different command interfaces, but they cannot independently own writable lanes on that joint after arbitration.

```text
trajectory task                    velocity task
claim: left/j1, priority 10        claim: left/j1, priority 50
command: left/j1/position          command: left/j1/velocity
              \                    /
               +--- arbitration --+
                         |
                         v
winner: velocity task owns left/j1
active command interface: left/j1/velocity
```

This preserves the coordinator's central property: priority and preemption apply to joints. It also supports runtime changes from position to velocity, effort, or a multi-field impedance command. Switching active command interfaces does not require restarting either the task or connection.

After arbitration, the coordinator publishes the winning command interfaces and values. It does not track native modes, request a transition, wait for transition acknowledgement, or roll back arbitration. A command-interface change is ordinary command input to the connection.

The connection compares each valid winning command with its active native behavior. If a transition is required, the connection withholds the new values from the old native mode, performs the vendor-specific operation, retains the newest compatible command received during the transition, and applies only a fresh command after success. This normal internal transition does not withdraw module readiness because the connection remains able to accept commands and preserve safety. A transition failure safe-stops the device and withdraws module readiness, so it uses the same robot safety path as any other required-module failure.

Connections declare which command interfaces exist, then validate each winning command combination against device constraints. A conventional arm may reject mixed position and velocity interfaces across its joints. A hybrid controller may accept them. An impedance controller may require position, velocity, `kp`, `kd`, and effort from the same joint owner. The coordinator neither encodes these combinations nor reruns arbitration to invent a lower-priority fallback.

No generic `ControlMode`, public `set_mode()` API, or command-interface transition protocol crosses the coordinator boundary. A connection privately maps winning command interfaces to vendor SDK modes and transition calls. This keeps arbitration deterministic and makes native state transitions part of device ownership.

**Alternatives rejected:** exact-interface arbitration, coordinator-orchestrated mode switching, whole-connection static command profiles, inferring vendor modes in the coordinator, and applying new command values before a native transition completes.

### Decision 5: Connections announce immutable resolved descriptions

`ConnectionDescription` is the source of truth after configuration resolution. It includes:

- source name and description epoch/version;
- declared state and command interface keys;
- joint/group metadata needed by callers;
- canonical units and hard/soft limits;
- supported command interfaces and any static combination constraints useful for early validation;
- expected state rate and stale timeout;
- command watchdog timeout;
- per-interface omission behavior;
- connection-specific safe-stop behavior.

The coordinator expects a configured set of source names. It rejects duplicate joint ownership across connections, duplicate keys within a source, inconsistent metadata, unsupported units or interface declarations, and descriptions that do not match initial state. A description is immutable for a running epoch. A changed description forces a fault and explicit re-arm rather than silently changing the control surface.

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

High-rate scalar values do not encode lifecycle. Universal module readiness reports whether a module can currently perform its declared responsibility. Three connection-specific typed contracts define the robot control plane without duplicating readiness:

- `ConnectionControl`: target source names, `operation_id`, and `PREPARE_ARM`, `COMMIT_ARM`, `ABORT_ARM`, `SAFE_STOP`, `CLEAR_SAFE_STOP`, `ESTOP`, or `CLEAR_ESTOP`.
- `ConnectionStatus`: source, `STANDBY`, `PREPARING`, `PREPARED`, `COMMITTING`, `ARMED`, `SAFE_STOPPED`, or `ESTOPPED` state, active description epoch, current control fault, last accepted lifecycle operation, and acknowledgement result.
- `ConnectionDescription`: capabilities and safety policy, as above.

Lifecycle messages use a multi-publisher-safe transport such as LCM or Zenoh. The coordinator repeats operations with the same `operation_id` until every target acknowledges or the transaction times out; connections handle duplicates idempotently. Arming is a two-phase robot-wide transaction. `PREPARE_ARM` lets each connection establish its device-specific operating preconditions while the ordinary task-command gate remains closed. Preparation may include bounded connection-owned motion. `COMMIT_ARM` creates one new control epoch and activates every prepared connection; the coordinator opens its task-command gate only after every required connection acknowledges. `ABORT_ARM` returns an uncommitted preparation to `STANDBY`. A preparation or commit rejection, timeout, readiness loss, or stale state safe-stops every connection because physical preparation may already have begun.

Each connection description declares a `DIRECT` or `OPERATOR_CONFIRMED` activation policy. A robot is operator-confirmed if any required connection requires confirmation. The public `arm()` convenience performs both phases for a direct robot, but stops in `PREPARED` for an operator-confirmed robot; only `confirm_arm(operation_id)` may commit that preparation. The requirement is declared by the connection rather than enforced only by a platform CLI, so another caller cannot bypass it accidentally. Clearing safe stop is also robot-wide and transactional: partial success causes the coordinator to restore or confirm `SAFE_STOPPED` on every required connection. Emergency stop is fanout and remains latched until every required connection has cleared it; clearing does not arm the robot.

Normal users call robot-level coordinator RPCs. They do not arm connections one by one.

### Decision 8: Readiness is universal and live

Readiness belongs to the module framework, not to the control protocol. Every module starts not ready, becomes ready after successful initialization, and may withdraw readiness later with a diagnostic reason. Returning normally from `start()` means initialization succeeded; a failed synchronous initialization raises instead of logging and returning. Modules with a later operational failure publish the readiness loss through the same framework surface. Worker-process health remains a separate liveness check.

The control coordinator observes that standard readiness for each configured connection source. Robot readiness additionally requires a valid immutable description and a complete fresh initial state from every source. Arm is refused unless all three conditions hold. If a required connection loses module readiness during preparation, commit, or armed operation, the coordinator performs robot-wide safe stop and becomes faulted. Recovery returns only to standby; it never resumes old task output.

There is no `command_ready`, per-interface readiness, or coordinator-visible transition state. A connection stays ready during an ordinary native transition because it can safely accept and retain the newest compatible command. If it can no longer uphold that contract, it withdraws readiness.

### Decision 9: The gripper is a joint at the control boundary

After PR #3381, `GripperControlTask` remains the sole task owner of gripper joints and keeps vector native-position, normalized-position, sweep, reference-pose, hold, and multi-DOF semantics. This refactor removes its dependency on coordinator hardware or adapter access.

Gripper state, limits, and commands use the same description and scalar contracts as every other joint. The connection owns vendor conversion. Manipulation selects a gripper through a planning/control group and does not cache ranges or construct a task name from hardware identity. Integrated and standalone grippers differ only in which connection owns their interfaces.

### Decision 10: Obsolete concepts are deleted at their cutover

No aliases or compatibility translations survive. The following domain concepts are removed where their replacements land:

- `RobotName`, `WorldRobotID`, robot selector parameters, and robot registries;
- local/global model-joint distinctions and robot-prefix parsing helpers;
- trajectory splitting and execution targets by robot;
- coordinator hardware objects, hardware add/remove/list RPCs, and adapter registries;
- hardware-ID command grouping and gripper task-name/range plumbing;
- hardware-specific global config fields superseded by connection instance config.

Private backend handles and private adapter helpers may remain when they improve locality, but their types and identities do not cross the new boundary.

## Safety Mechanism Design

Safety is layered. The coordinator owns the robot-wide command gate and detects cross-connection failures. Each connection owns the physical response for its device. A native controller or external safety layer must cover failures that also kill the connection process.

```text
 task outputs
      |
      v
 coordinator gate ---- fault ----> repeat SAFE_STOP to every connection
      |                                  |
      | valid current-epoch commands     | correlated acknowledgements
      v                                  v
 connection gate -----------------> connection-specific safe policy
      |                                  |
      | validated native commands        | hold / brake / damp / balance
      v                                  v
 native controller ----------------> physical robot
      |
      +---- native timeout or external supervisor survives process death
```

### Safety responsibilities

| Layer | Responsibility | It must not assume |
|---|---|---|
| Task | Produce commands only for claimed joints and stop on preemption | That its output reaches hardware |
| Coordinator | Gate commands, detect robot-wide faults, initiate and track safe stop | That silence or zero values are universally safe |
| Connection | Validate commands, latch lifecycle state, execute device-specific safe behavior, enforce watchdog | That the coordinator remains alive |
| Native/external safety | Stop persistent unsafe output after connection-process or link failure | That a Python watchdog survives its own process |

### Safe stop, shutdown, and emergency stop are distinct

`SAFE_STOP` ends task-directed motion while preserving whatever support the device needs. An arm may capture measured position and hold it, a base may ramp to zero and brake, and a humanoid may retain balance or damping control. The operation is latched and recoverable, but it does not imply actuator power-off.

`ESTOP` invokes the strongest available emergency behavior and has a separate latch. Neither `CLEAR_SAFE_STOP` nor `CLEAR_ESTOP` arms the robot; successful clearance returns every connection to `STANDBY`, with its command gate closed.

Physical deactivation is not a generic runtime lifecycle operation. `Module.stop()` asks each connection to execute its device-specific shutdown sequence: establish a stable state, park or lower when supported and configured, disable actuators only when safe, then disconnect its native transport. Domain-specific concepts such as MAVLink motor disarm remain on their owning connection API and do not define the joint-control protocol.

No generic command vector implements these operations. In particular, an empty frame, zero effort, zero gains, command silence, and motor disable are not interchangeable safe-stop policies.

Task cancellation is not a lifecycle operation. It ends one task's output and releases its joint claims while the coordinator continues fresh current-epoch heartbeats. The owning connection applies its declared omission or armed-idle policy, and the robot may remain `ARMED`. Cancellation never clears or creates a safety latch. A platform that cannot remain stable under its declared armed-idle policy must use `SAFE_STOP` instead of treating cancellation as sufficient.

### Optional supervised activation

Some robots can enter normal control directly after validation. Others require a supervised physical preparation before arbitrary task commands are safe. This difference does not create platform-specific coordinator states or make every robot use a manual ceremony. Connections declare either `DIRECT` or `OPERATOR_CONFIRMED`; the coordinator aggregates the strictest policy across the required connection set.

```text
                              operator confirmation
                                     |
                                     v
STANDBY --begin_arm()--> PREPARING --> PREPARED --confirm_arm(id)--> COMMITTING --> ARMED
   ^                           |            |
   +------- abort_arm(id) -----+------------+
                               |
                               +-- failure, timeout, stale state,
                                   or readiness loss --> SAFE_STOPPED
```

Ordinary task commands are blocked in `STANDBY`, `PREPARING`, `PREPARED`, and `COMMITTING`. During `PREPARING`, only bounded connection-owned activation behavior is permitted. In `PREPARED`, each connection maintains its declared stable prepared behavior while upstream tasks may compute shadow diagnostics without reaching hardware. The new control epoch becomes usable only after every required connection acknowledges `COMMIT_ARM` and the coordinator enters `ARMED`.

For a direct robot, the public `arm()` call runs preparation and commit without an operator pause. For an operator-confirmed robot, `arm()` returns a receipt in `PREPARED`; a matching `confirm_arm(operation_id)` is required. Cancelling before commit calls `ABORT_ARM` and returns to `STANDBY` only after every connection acknowledges its stable standby behavior. Failure after physical preparation has started uses robot-wide `SAFE_STOP`, not a best-effort abort.

G1 is the motivating operator-confirmed connection. Its current hardware CLI first enables policy dry-run, starts a real position ramp from measured joints toward the policy default, waits for the ramp, asks the operator to inspect the robot, remote, and E-stop, then enables live policy output. In the target design, the G1 connection owns the preparation ramp and prepared-state hold. GR00T may evaluate in shadow while the coordinator gate is closed, but task output cannot reach hardware until commit. The optional bimanual ready-pose trajectory remains an ordinary manipulation action after arming; it is not a lifecycle state. Stopping only GR00T remains task cancellation, robot-wide stopping uses `SAFE_STOP`, and physical shutdown remains connection-specific `Module.stop()` behavior.

### Coordinator-to-connection stop protocol

When a required source becomes stale, not ready, or faulted, the coordinator closes its command gate before sending lifecycle traffic:

```text
detect fault
    |
    v
close command gate atomically
    |
    v
enter FAULT_STOPPING
    |
    +---- stop ordinary control-command heartbeats
    |
    +---- repeatedly publish
    |     ConnectionControl(
    |       operation=SAFE_STOP,
    |       operation_id=N,
    |       targets=[every required connection],
    |       reason=...)
    |
    v
collect ConnectionStatus(operation_id=N, state=SAFE_STOPPED)
    |
    +---- all acknowledge ------> SAFE_STOPPED
    |
    +---- timeout/missing ------> remain faulted; report unknown source state
```

The explicit operation is the normal path. Withdrawing ordinary heartbeats is the fallback: a connection that misses `SAFE_STOP` invokes the same local safe policy when its command watchdog expires. Continuing empty heartbeats after a fault is forbidden because it could mask a lost stop request.

A connection that detects its own SDK error, native-transition failure, invalid unsafe command, device-link loss, or heartbeat timeout safe-stops immediately without waiting for the coordinator. It then publishes status and withdraws module readiness if it can no longer perform its declared function. The coordinator safe-stops the remaining connections. A healthy peer that safe-stops only because of the robot-wide operation remains module-ready but reports `SAFE_STOPPED` lifecycle state.

### Command epochs and lifecycle latches

A successful arm transaction creates a new control epoch. Every command frame carries it. A connection accepts a command only while `ARMED`, for the current epoch, with a newer sequence. `SAFE_STOP` and `ESTOP` invalidate the epoch before executing their physical behavior, so a late or reordered command cannot restart motion.

```text
SAFE_STOPPED --clear safe stop--> STANDBY --prepare--> PREPARED --commit(epoch=16)--> ARMED
                                                                                           |
late command(epoch=15) --------------------------------------------------------------------+--> reject
```

Every `SAFE_STOP`, whether automatic or explicitly requested, is latched. Recovery requires the fault cause to clear where applicable, fresh complete state, live module readiness, a successful robot-wide `CLEAR_SAFE_STOP` transaction, and a new arm transaction. No public API clears one connection independently. If any required connection rejects or times out, the coordinator keeps its command gate closed and restores or confirms `SAFE_STOPPED` across the robot. Recovery never restores old winners, replays cached commands, or resumes a trajectory.

### Watchdogs and process death

The connection software watchdog covers coordinator failure while the connection process remains alive. It cannot cover its own process death. Each connection description therefore declares one process-loss classification:

| Classification | Meaning | Production physical arming |
|---|---|---|
| `NATIVE_WATCHDOG` | A configured device/controller timeout enters the declared safe state | Allowed after platform verification |
| `EXTERNAL_SUPERVISOR` | An independent safety controller detects loss and enters the declared safe state | Allowed after integration verification |
| `INTRINSICALLY_SAFE` | Command loss cannot sustain hazardous output for this device and command path | Allowed after documented verification |
| `SIMULATION` | No physical actuator is reachable | Allowed only for simulation |
| `UNPROTECTED` | No independent process-loss mechanism is established | Refused |

The coordinator refuses a production arm transaction if any required physical connection declares `UNPROTECTED`, `SIMULATION`, an unverified classification, or no classification. This is an arm gate, not module readiness: an unprotected connection may still be operational and inspectable in standby. Hardware validation must exercise the declared process/link-loss path rather than merely inspect configuration.

### State publication during faults

When any required source becomes stale, the coordinator retains its last valid snapshot for diagnostics but stops publishing normal aggregated `coordinator_joint_state`. It must not present a mixture of fresh and cached values as live robot state. Fresh state and restored readiness may return the robot only to `STANDBY` after the safety latch is cleared.

### Safety state machine

```text
BOOT --ready + description + fresh state--> STANDBY
 |                                             |
 | invalid manifest/state                      | begin_arm()
 v                                             v
FAULT                                      PREPARING --> PREPARED --> COMMITTING --> ARMED
                                               |           |              |            |
                                               +-- abort ---+              |            |
                                               |                          |            |
                                               v                          |            |
                                            STANDBY                       |            |
                                                                          |            |
fault / timeout / stale state / readiness loss ---------------------------+------------+
                                      |
                                      v
                               FAULT_STOPPING
                                |           |
              all stop acks     |           | stop timeout
                                v           v
                          SAFE_STOPPED     FAULT
                                |
                                | cause cleared + clear_safe_stop()
                                v
                             STANDBY

 Any state --ESTOP--> ESTOPPED --clear_estop--> STANDBY
                                           (explicit arm still required)
```

The default is not auto-enable. Robot readiness requires live module readiness, one immutable description, and one complete valid state snapshot from every configured source. A required connection that withdraws readiness, becomes stale, or faults during preparation, commit, or armed operation causes robot-wide safe stop. Recovery does not automatically resume motion; callers must clear the safety latch and begin a new arm operation. Delayed confirmation from an older preparation cannot satisfy the new operation ID.

Every connection implements an independent command watchdog. While healthy and armed, the coordinator publishes a command heartbeat at its control rate even when no task owns an interface. Missing command frames invoke the connection's declared safe stop. Task timeout and coordinator watchdog are useful but never substitute for connection-local and lower-level watchdogs.

Default omission behavior is explicit and discoverable:

| Interface | Command omitted while heartbeat continues |
|---|---|
| position | Retain last accepted target |
| velocity | Command zero |
| effort | Command zero |
| `kp` / `kd` | Retain configured or last accepted safe value |

A connection may declare a stricter policy. Omission applies only to fresh, current-epoch frames while armed; it never defines fault behavior. No policy may weaken hard limits or watchdog safe stop. Hard limits are enforced at the connection after canonical-to-native conversion; tasks and the coordinator may also use published limits for validation and planning.

## Simulation and Replay

- Every connection contract first receives a deterministic fake that can publish descriptions/states, acknowledge lifecycle operations, inject stale frames and faults, and record commands.
- Simulation connections implement the same descriptions, command-interface transitions, lifecycle acknowledgements, watchdog behavior, names, and units as hardware connections.
- Replay can publish historical state and status for consumers, but must never arm or command physical hardware. Any mixed replay/hardware blueprint must fail validation unless explicitly designed and tested later.
- Recorded old coordinator-owned adapter traffic is not supported through a compatibility decoder.
- The static dual-xArm fixture validates bimanual naming and planning without claiming hardware availability.

### Verification matrix

| Surface | Automated gate | Manual/hardware gate |
|---|---|---|
| Contract | codec, length, duplicate, unknown, stale, sequence tests | topic inspection |
| Coordinator | arbitration, readiness, command gate, stop retry, epochs, estop, heartbeat withdrawal | shell/RPC lifecycle exercise |
| xArm + gripper | fake/sim loop, limits, powered safe stop, stale epochs | single xArm smoke test including gripper and process-loss behavior |
| G1 | whole-body simulation, impedance combination, balance/damping safe policy, watchdog | platform-owner checklist for real G1 safe-state behavior |
| Piper/OpenArm | mock contract and blueprint build | platform-owner bring-up checklist |
| Mobile bases | fake/sim velocity interfaces and safe zero | platform-owner smoke test where available |
| Manipulation | backend tests on one model and several groups | visual trajectory sanity check |
| Blueprints | build/import and generated registry test | representative sim/replay launch |

No real dual-arm test blocks the change because no such hardware use case currently exists.

## Risks / Trade-offs

| Risk or trade-off | Consequence | Mitigation |
|---|---|---|
| A broad breaking change touches many platforms | Long review and temporary branch divergence | Stack by coherent boundaries; merge prerequisite contracts before family migrations |
| Shared scalar keys lose domain structure | Debugging can become string-heavy | Immutable typed descriptions, strict validators, introspection RPCs, canonical naming |
| `float64` cannot carry future discrete command data | A later device may need another value kind | Add a concrete typed contract only when a real interface requires it |
| LCM/Zenoh state adds serialization/network cost | High-rate whole-body loops may expose latency | Benchmark G1; keep payload compact; pursue multiwriter-safe typed SHM separately |
| Runtime descriptions delay robot readiness | Startup becomes asynchronous | Universal module readiness, timeout diagnostics, deterministic fake tests |
| Slash names may fail in a model backend | A backend could reject canonical URDF names | Make parser compatibility an early PR1 spike; encode once inside only that loader if needed |
| Sparse commands plus retention can hide old targets | Unexpected motion after ownership changes | Explicit omission policy, task cancellation tests, heartbeat epochs, zero velocity/effort defaults |
| A connection process dies with persistent native output | Its software watchdog dies with it | Require declared and tested native timeout or external supervisor before claiming process-crash safety |
| One generic physical shutdown action is unsafe across platforms | Gravity-loaded or balancing robots may fall | Connection-owned safe-stop and shutdown policies; keep safe stop and estop distinct |
| Atomic coordinator cutover has a large diff | Review and rollback cost | Land contracts and connection implementations first; keep cutover behavior-focused and delete old path in the same PR |

The chief intentional trade-off is using a shallow scalar wire format with a rich immutable description. That keeps the high-rate message small and generic while moving semantics to configuration and introspection. Strict startup validation is therefore essential, not optional polish.

## Migration / Rollout

### Current implementation baseline

PR1 and PR2 are no longer proposals: #3420 introduced static single-robot models and #3431 cut manipulation over to one model. The remaining stack starts by rebasing on their merged implementation and removing residue against this final design. This reconciliation is a separate small review unit so PR3 does not mix model/manipulation cleanup with new control-wire contracts.

The initial current-`main` audit already identifies the singular `end_effector_link` model-info projection and local/global wording that survived in helpers or tests. PR2a removes these rather than treating them as compatibility promises. It also repeats the forbidden-concept search after newer manipulation work. Gripper hardware IDs, coordinator-derived gripper task names, and cached gripper ranges remain assigned to PR6 because deleting them earlier would break the working product before the connection-described replacement exists.

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
PR 2a Post-merge model/manipulation cleanup
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
| 1 (merged #3420) | All manipulation model consumers can load one prepared model with canonical groups | multi-model config surface where isolated |
| 2 (merged #3431) | Manipulation plans and executes for one robot across one or several groups | robot IDs, registries, selector APIs, trajectory splitting |
| 2a | The merged model/manipulation implementation matches the final reviewed vocabulary and surface before control contracts build on it | singular model-wide end-effector projection, leftover transition aliases, obsolete local/global or multi-robot terminology |
| 3 | Stable messages, validators, fake connection, and safety semantics exist without changing production ownership | obsolete experimental control-message types if superseded |
| 4a | Manipulator and sim connections can speak the new contracts | coordinator-facing adapter assumptions in migrated packages |
| 4b | Mobile-base connections can speak the new contracts | migrated global hardware config |
| 4c | G1 exposes its whole-body interface combinations and state | migrated special coordinator plumbing |
| 5 | Production coordinator consumes shared buses and owns no hardware | old hardware ownership, polling, registries, management RPCs |
| 6 | Manipulation, trajectories, and gripper tasks use canonical interfaces end to end | hardware/task/range mapping remnants |
| 7 | Supported blueprints, docs, and platform checklists describe only the new system | stale docs, demos, generated registry entries |

The stack may branch at PR4, but PR5 depends on every production connection family required by migrated blueprints. No PR introduces a compatibility shim or leaves both command paths active. If a merge unit cannot preserve a working product, its boundary must move rather than adding a temporary abstraction.

### Deployment and rollback

This is a source-level migration, not an in-place protocol upgrade. Deploy coordinator and all participating connections from the same release. The description version/epoch causes mismatched contracts to fail closed during readiness. Rollback means redeploying the previous complete release and its blueprints, not mixing old and new modules.

Blueprint registry changes require `pytest dimos/robot/test_all_blueprints_generation.py`. Normal formatting, mypy, targeted unit tests, blueprint build tests, sim/replay smoke tests, and platform checklists gate their corresponding PRs. Documentation lands alongside behavior and is completed in PR7.

## Open Questions

The core ownership boundary is settled. The following review questions still need explicit resolution:

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
| Upper-level multi-robot coordination | A caller genuinely coordinates separate logical robots |

These follow-ups must not add hooks, compatibility fields, or abstractions to this refactor before their triggers occur.
