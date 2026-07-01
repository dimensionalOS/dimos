# DimOS Robotics Context

DimOS describes robots, actuators, control surfaces, and manipulation planning using precise robotics terminology. This glossary records stable domain language only, not implementation details.

## Language

**Planning group**:
A named subset of a robot model's joints and frames that can be selected as a planning unit.
_Avoid_: move group, joint group

**Composite planning group**:
A planning group that represents coordinated motion across multiple selected planning groups.
_Avoid_: group combination, combined groups, multi-group plan

**Auxiliary planning group**:
A planning group included in a planning request that may move as part of the plan but does not have its own task-space target in that plan.
_Avoid_: extra group, passive target group, unconstrained target

**Linear TCP path**:
A motion recipe where the tool center point follows a straight Cartesian segment from its start pose to its target pose within configured Cartesian tolerance.
_Avoid_: linear joint motion, linear motion

**Linear TCP trajectory smoothing**:
A manipulation-planning capability that makes a Linear TCP path executable without treating every intermediate Cartesian sample as a stop, while preserving Cartesian-line tolerance.
_Avoid_: waypoint skipping, making linear motion faster

**Adaptive-conservative smoothing**:
A trajectory smoothing policy that starts with an aggressive simplification and, on validation failure, preserves more of the original path rather than relaxing correctness tolerances.
_Avoid_: tolerance loosening, unsafe smoothing retry

**Trajectory post-processing pipeline**:
A staged manipulation-planning capability that may refine a geometric path, validate the refinement, assign timing, and apply execution-oriented smoothing while preserving the path's declared constraints.
_Avoid_: hidden waypoint hack, one-off retiming step

**Path-constraint metadata**:
Optional planning metadata attached to a geometric path that declares the path constraints any post-processing must preserve.
_Avoid_: planner debug data, visualization-only metadata

**Non-blocking smoothing fallback**:
A trajectory post-processing policy where smoothing failures fall back to the original geometric path rather than failing parametrization; the worst expected outcome is a slower valid trajectory.
_Avoid_: strict smoothing gate, smoothing-required parametrization

**Composite RoboPlan model**:
A RoboPlan-facing robot model that represents multiple registered robot models as one planning scene.
_Avoid_: combined URDF, merged robot scene

**Planning world**:
The authoritative belief state for manipulation planning, including robot state and scene state used by planners.
_Avoid_: planner context, backend instance

**Trajectory parametrization**:
The manipulation-planning capability that assigns time to a geometric joint path under motion constraints.
_Avoid_: trajectory generation, retiming when referring to the broader capability

**Generated trajectory**:
A manipulation-planning artifact that represents a geometric path after trajectory parametrization, ready for preview, validation, benchmarking, or execution planning.
_Avoid_: timed generated plan, generated plan when referring to the time-parametrized artifact, joint trajectory when referring to the manipulation-level artifact

**Shared trajectory time domain**:
The single timing basis a generated trajectory uses for all selected joints and robot-local projections in a composite or multi-robot motion.
_Avoid_: independent per-robot timing, per-arm retiming when referring to coordinated composite motion

**Trajectory dispatch**:
An execution-preparation artifact that derives control-task-specific joint trajectory messages from a generated trajectory without changing the generated trajectory's canonical global timing.
_Avoid_: generated trajectory projection, execution-time parametrization, per-task generated trajectory

**Policy trajectory dispatch**:
An execution-preparation artifact that derives control-task-specific joint trajectory messages from policy action chunks while leaving control authority with the configured control task.
_Avoid_: direct policy control, policy motor write, learned controller bypass

**Joint-trajectory policy chunk**:
A v1 robot-learning action chunk shaped to become the `JointTrajectory` argument accepted by a joint trajectory control task.
_Avoid_: generic policy action, direct motor command, backend-native tensor

**Contract-assigned trajectory timing**:
A v1 policy rollout convention where the robot policy contract assigns simple relative timing to backend action rows when constructing a joint trajectory for dispatch, starting the first action row at `time_from_start = 0`.
_Avoid_: controller-side chunk timing inference, untimed policy positions, trajectory parametrization

**Dispatch-owned trajectory timestamp**:
A policy rollout convention where the rollout module assigns the absolute intended start time for the whole contract-built trajectory chunk when submitting it to the control system.
_Avoid_: contract-owned dispatch time, backend inference timestamp as control timestamp, trajectory creation time

**Subchunk trajectory dispatch**:
A policy rollout convention where the rollout module generates a larger policy action chunk but dispatches only a configured leading subset of action rows to the joint trajectory control task before replanning.
_Avoid_: executing the entire policy horizon by default, deep policy command queue, hidden action-horizon truncation

**Completion-gated policy rollout**:
A simple rollout cadence where the policy module waits for the dispatched trajectory subset to finish before fetching the latest observations and generating the next backend action chunk.
_Avoid_: asynchronous continuous inference, speculative trajectory queueing, wall-clock-only rollout loop

**Sleep-gated policy rollout**:
A v1 completion-gated policy rollout implementation where the policy module waits for the expected row coverage duration of the dispatched trajectory subset using the rollout clock before fetching observations and replanning.
_Avoid_: task-completion dependency, signal-based rollout synchronization, speculative inference

**Policy action representation**:
The robot-learning action form a policy is trained or configured to produce before it is adapted for a DimOS control task.
_Avoid_: assuming all policy actions are joint positions, opaque policy output

**Robot policy action**:
A runtime-independent robot-learning action emitted by a robot policy module after backend inference and contract conversion, before adaptation to benchmark runtime frames or real robot control commands.
_Avoid_: runtime action frame, motor command, backend tensor

**Native benchmark action surface**:
A benchmark runtime action interface whose command values are defined by the benchmark environment itself rather than by a DimOS motor or joint surface.
_Avoid_: motor command alias, hidden joint target, controller-specific shortcut

**LIBERO action mode**:
The simulator-side interpretation of `env.step(action)` in LIBERO, defining whether the action vector represents joint positions, relative end-effector deltas, or another environment-supported action form.
_Avoid_: DimOS controller, ControlCoordinator task, policy output type

**Native LIBERO action mode**:
A LIBERO runtime mode, matching the official LeRobot LIBERO environment setup, that accepts the environment's own relative end-effector delta plus gripper action vector instead of a DimOS whole-body motor-position command.
_Avoid_: joint-position LIBERO profile, fake motor command, end-effector action as motor q

**Runtime action frame**:
A runtime protocol command envelope for a named non-motor action surface, carrying action values and semantic identity without pretending they are joint positions or motor commands.
_Avoid_: overloaded motor frame, unnamed action vector, backend tensor leak

**Action-surface control task**:
A control-coordinator task that accepts and validates commands for a semantic action surface while leaving the concrete runtime or robot mapping to the task implementation.
_Avoid_: joint trajectory task, motor adapter, policy bypass

**Robot policy contract**:
A robot-learning boundary that declares a specific robot/runtime and policy-backend input-output convention, including how aligned robot-native samples become backend-ready batches and how backend outputs become robot-native policy action chunks.
_Avoid_: execution contract, control-task adapter, universal robot contract

**Robot learning sample**:
A runtime-independent policy observation artifact that carries semantically named observation roles, task context, timestamps, and metadata for robot-learning inference or training.
_Avoid_: benchmark sidecar response, runtime observation frame, backend-ready batch

**Backend-ready batch**:
The policy-backend-specific inference or training input produced from an aligned robot-native sample by a robot policy contract.
_Avoid_: universal DimOS batch, raw observation bundle, synchronized sample

**Backend output**:
The policy-backend-specific action result that a robot policy contract converts into a robot-native policy action chunk.
_Avoid_: control command, joint trajectory, final actuator target

**Backend output envelope**:
A small rollout artifact that carries a backend-native action result together with inference metadata needed for validation, tracing, and contract conversion.
_Avoid_: policy action chunk, control command, backend internals leak

**Temporal sample readiness**:
The rollout-time check that the observation roles needed for a policy sample are available with acceptable freshness for the current inference tick, without requiring perfect cross-sensor timestamp equality.
_Avoid_: exact timestamp match, strict synchronization gate, semantic policy validation

**Contract conversion failure**:
A fail-fast result when a robot policy contract receives a supposedly ready sample or backend output that violates the contract's declared semantic input-output convention.
_Avoid_: not-ready sample, silent coercion, best-effort backend batch

**Robokin kinematics backend**:
A DimOS kinematics backend that presents multiple robokin-supported inverse-kinematics engines through one robotics-facing capability.
_Avoid_: Oink backend, RoboKin world backend, single-engine Oink solver

**Robokin engine**:
A specific solver implementation selected inside the Robokin kinematics backend, such as Placo, Pyroki, or Oink.
_Avoid_: Robokin backend, world backend, planner backend

**RoboPlan kinematics backend**:
A DimOS kinematics backend implemented by RoboPlanWorld using RoboPlan-backed model state, planning groups, Jacobians, and collision state.
_Avoid_: Robokin backend, separate RoboPlan IK world, planner-only RoboPlan integration

**RoboPlan Oink IK solver**:
RoboPlan's task-based inverse-kinematics capability for solving one or more frame pose targets under joint constraints.
_Avoid_: hand-written Jacobian IK, Robokin-only Oink wrapper

**Coordinated simulation clock**:
A simulation benchmark clock policy where simulator time advances in lockstep with the DimOS control coordinator clock.
_Avoid_: autonomous simulator loop, write-triggered stepping, settle step

**Runtime sidecar**:
A separate process or environment that owns a benchmark simulator backend while DimOS owns orchestration, control integration, skills, and artifacts.
_Avoid_: plugin, embedded simulator

**HTTP runtime removal gate**:
A migration success condition requiring existing HTTP runtime sidecar servers, clients, payload fetch endpoints, and HTTP-first demos to be removed once their behavior is covered by Simulator Runtime Modules.
_Avoid_: HTTP fallback, target simulator architecture, long-term transport boundary

**Simulator Runtime Module**:
A first-class DimOS Module that represents a simulator runtime at the blueprint boundary while preserving simulator ownership of benchmark reset, stepping, observations, and scoring.
_Avoid_: HTTP runtime sidecar, benchmark script launcher, embedded simulator

**Simulator runtime blueprint helper**:
A package-local blueprint factory that registers a simulator runtime's named Python project environment and places its Simulator Runtime Module into that environment using the standard blueprint placement API.
_Avoid_: module-local deployment flag, global sidecar registry, caller-written placement boilerplate

**Remote module worker**:
A separate Python environment that hosts first-class DimOS Modules while the main DimOS process owns blueprint orchestration and module coordination.
_Avoid_: arbitrary sidecar service, embedded optional dependency

**Venv module worker**:
A same-machine Python virtual environment that hosts first-class DimOS Modules separately from the main DimOS Python environment.
_Avoid_: remote deployment, sidecar service, optional dependency import guard

**Module import descriptor**:
A portable identity for a DimOS Module class that a venv module worker can import in its own Python environment.
_Avoid_: pickled module class, shared interpreter object, source checkout assumption

**Module placement**:
A blueprint-level decision that chooses where a DimOS Module instance runs, such as the default worker pool or a named venv module worker.
_Avoid_: intrinsic module type, stream transport, permanent class identity

**Local worker control channel**:
A same-machine private channel used by the coordinator process to send lifecycle and wiring commands to a worker process.
_Avoid_: stream data plane, remote public API, transport topic

**Multiprocessing connection control channel**:
A local worker control channel implemented with Python's multiprocessing connection Listener/Client machinery so separately launched Python interpreters can exchange DimOS worker protocol messages.
_Avoid_: stream transport, stdio protocol, remote deployment API

**Named venv**:
A runtime-resolved label for a Python virtual environment that can host venv module workers.
_Avoid_: hardcoded interpreter path in blueprint, deployment target, stream transport

**Worker protocol runtime**:
The DimOS runtime surface that a worker environment must provide to receive coordinator lifecycle commands and host first-class Modules.
_Avoid_: full optional dependency set, application module package, public remote API

**Module contract**:
A lightweight coordinator-visible DimOS Module class that declares the streams, module references, RPC surface, and config expectations used for blueprint wiring.
_Avoid_: heavy implementation module, connection-only shim, runtime sidecar

**Module IO contract**:
The coordinator-visible set of typed stream inputs and outputs a DimOS Module exposes for blueprint wiring.
_Avoid_: stream schema, port list, dynamic ports

**Configuration-resolved module IO**:
A module IO contract whose streams are determined from the module's final configuration before blueprint wiring.
_Avoid_: runtime dynamic IO, late-bound ports, generated subclass IO

**Module implementation descriptor**:
A portable identity for the concrete Module implementation class that a venv module worker imports and instantiates for a module contract.
_Avoid_: coordinator-imported heavy class, pickled module class, alternate stream contract

**Contract compatibility responsibility**:
The phase-1 expectation that a module implementation descriptor names a concrete Module compatible with its coordinator-visible module contract, without an explicit verifier.
_Avoid_: schema validation requirement, subclass proof, remote API compatibility guarantee

**Venv worker failure semantics**:
The phase-1 rule that an incompatible or failing venv module implementation fails the normal blueprint build lifecycle instead of degrading into a partial system.
_Avoid_: optional module fallback, background retry policy, partial blueprint success

**Venv module placement API**:
A blueprint-level mapping that runs selected import-safe Python Module classes in named runtime environments.
_Avoid_: new Module base class, permanent class deployment attribute, global transport setting

**Python venv placement**:
A mapping entry keyed by an import-safe Python Module class and valued by the name of a Python runtime environment.
_Avoid_: deployment type, stream transport key, hardcoded worker process

**Reserved deploy kwargs**:
Internal coordinator-to-worker-manager metadata carried through module deploy kwargs during early architecture spikes and stripped before Module instantiation.
_Avoid_: user module config, public constructor argument, permanent ModuleSpec schema

**Venv worker environment config**:
A runtime mapping from a named venv to an existing Python executable that can launch venv module workers.
_Avoid_: automatic venv creation, package installation plan, blueprint-embedded filesystem path

**Runtime environment registry**:
A runtime configuration map from stable environment names to environment backends that resolve interpreters, executables, command environment variables, and optional preparation steps for DimOS-managed processes.
_Avoid_: venv-only config, blueprint-embedded machine paths, per-module ad hoc build commands

**Runtime environment preparation**:
An explicit pre-run action that prepares only the runtime environments used by active module placements in a loaded blueprint configuration.
_Avoid_: implicit install during blueprint run, global environment-name command, preparing unused registry entries

**Python project runtime environment**:
A convention-driven runtime environment rooted at a Python project directory, where standard files such as `pyproject.toml`, `uv.lock`, and optionally `pixi.toml` determine how the worker Python environment is prepared and launched.
_Avoid_: per-tool runtime backend taxonomy, blueprint-embedded setup command, manually enumerated manifest paths

**Pixi-backed uv runtime**:
A Python project runtime environment where Pixi prepares the native/toolchain layer and provides the Python interpreter used to create the project-local uv `.venv`; worker launch uses the `.venv` Python with Pixi activation environment applied.
_Avoid_: Pixi-only Python environment, coordinator Python venv, launching without native activation environment

**Toolchain-mediated worker launch**:
A worker launch policy for convention-based Python project runtimes where DimOS invokes the project toolchain command, such as `pixi run uv run --no-sync python`, instead of reconstructing activation variables or launching the venv interpreter path directly.
_Avoid_: hand-built Pixi activation env, mutating sync during blueprint run, bypassing project runtime conventions

**Named runtime environment**:
A stable label in the runtime environment registry that modules and worker placements reference when they need a non-default execution environment.
_Avoid_: hardcoded venv path, Nix command string as identity, deployment type

**Named venv worker pool**:
A set of worker processes launched from the same named venv that may host multiple compatible placed Modules without mixing Modules assigned to other Python environments.
_Avoid_: one process per Module, shared cross-venv pool, global worker pool replacement

**Worker launch strategy**:
The mechanism used to start and connect a worker process while preserving the shared worker pool and runtime protocol architecture.
_Avoid_: duplicated worker scheduler, separate venv-only runtime, stream transport selection

**Worker process handle**:
A coordinator-side abstraction for a running worker process that supports module deployment, lifecycle requests, capacity accounting, and shutdown regardless of how the process was launched.
_Avoid_: launch mechanism, worker scheduler, module implementation

**Worker launcher**:
A strategy object that creates worker process handles for a specific Python launch environment, such as the coordinator venv or a named venv.
_Avoid_: pool manager, module deployer, transport factory

**Import-safe module file**:
A Python file defining a venv-deployable DimOS Module that can be imported by the coordinator environment without importing worker-only optional dependencies at module import time.
_Avoid_: split package requirement, top-level heavy dependency import, hidden sidecar boundary

**Depth observation**:
A depth image interpreted with its camera calibration and the pose of its camera frame at the image timestamp.
_Avoid_: depth point cloud, raw 3D points

**Grasp target**:
The intended physical object or bounded scene region that grasp generation should produce grasp candidates for.
_Avoid_: correct object, target object, grasp object

**Object id**:
A stable identifier for a registered perceived object, used when a robot command must refer to one non-ambiguous physical object.
_Avoid_: object-ish argument, object name when identity matters

**Registered object**:
A perceived object that has been assigned an Object id and has enough spatial metadata to be used as a Grasp target.
_Avoid_: detection when referring to a persistent object reference

**Target-masked TSDF**:
A grasp-generation workspace representation where observations outside the selected Grasp target are suppressed with a deliberate cushion so the target remains intact.
_Avoid_: censored TSDF, object-only scene

**Target bounds**:
A world-frame axis-aligned bounding region used as a rough attention area for a Grasp target before grasp generation.
_Avoid_: perfect object geometry, grasp geometry

**Grasp candidate**:
A proposed end-effector grasp pose for a Grasp target, optionally carrying ranking metadata such as score, width, or approach information.
_Avoid_: executed grasp, final pick action, object pose

**Pointcloud grasp generator**:
A grasp-generation component that consumes point cloud observations of a Grasp target, optionally with scene context, and proposes Grasp candidates.
_Avoid_: TSDF grasp generator, robot execution controller, perception registration module

**SHM runtime data plane**:
A local shared-memory command/state channel between a ControlCoordinator-facing hardware adapter and a DimOS simulator client module, used when high-rate motor control must cross local process boundaries without RPC.
_Avoid_: remote sidecar protocol, public simulator API, benchmark control plane, module object sharing

**Motor state projection**:
The hardware-facing subset of simulator state that resembles what a raw robot driver exposes: actuator positions, velocities, efforts, commands, enable state, and errors.
_Avoid_: task observation, scene observation, evaluator state

**Whole-body motor surface**:
A hardware control surface that treats a robot as an ordered set of motors with per-motor state and commands, independent of whether the robot is a manipulator, mobile base, or humanoid.
_Avoid_: manipulator-only adapter, end-effector API, task action API

**Runtime motor action frame**:
An ordered command frame addressed to a simulator runtime's declared whole-body motor surface for one benchmark step.
_Avoid_: backend-native action vector, opaque simulator action, stream transport payload

**Benchmark episode config**:
A backend-facing declaration of benchmark intent that names the task, robot, runtime constraints, and evaluation setup before any DimOS blueprint is launched.
_Avoid_: hardware config, simulator config, blueprint config

**Benchmark reset authority**:
The control-plane responsibility for establishing a new benchmark episode state before simulation time advances.
_Avoid_: reset topic ownership, observation stream side effect, simulator auto-reset

**Semantic skill benchmark episode**:
A benchmark episode where the agent acts through named, task-level DimOS skills while simulator backends provide reset, observation, and external scoring.
_Avoid_: motor-control benchmark episode, raw simulator action episode, code-as-policy episode

**Agentic manipulation module**:
A universal DimOS skill-facing module that exposes agent-appropriate manipulation capabilities by coordinating existing manipulation and control modules.
_Avoid_: benchmark skill container, Robosuite skill module, simulator-specific manipulation API

**Resolved runtime plan**:
The concrete DimOS launch material derived from a benchmark episode config, including hardware components, simulator connection config, observation streams, evaluator setup, and artifact routing.
_Avoid_: benchmark intent, user-authored task config

**Runtime prelaunch orchestration**:
The phase that starts and coordinates the simulator sidecar environment and the DimOS blueprint environment before a benchmark episode begins.
_Avoid_: config parsing, blueprint launch, single-process startup

**Runtime asset bootstrap**:
A deliberate preparation phase that retrieves, stages, or validates external benchmark assets before a runtime sidecar starts an episode.
_Avoid_: implicit sidecar download, startup mutation, hidden dataset setup

**Remote runtime boundary**:
The network-facing protocol boundary between a DimOS simulator client and a benchmark backend process that may run in another environment or on another machine.
_Avoid_: shared memory boundary, hardware adapter boundary, in-process simulator object

**Runtime protocol schema**:
The shared, backend-neutral message contract used on the remote runtime boundary to describe episodes, robot motor surfaces, actions, observations, scores, and artifacts.
_Avoid_: backend SDK type, DimOS hardware adapter type, simulator object

**Runtime protocol package**:
A lightweight installable package in the monorepo that contains only remote runtime protocol schemas, codecs, and compatibility tests so sidecars can depend on it without installing DimOS.
_Avoid_: DimOS submodule, simulator backend package, hardware adapter package

**Runtime observation stream**:
A simulator-derived observation exposed through DimOS's normal typed stream contracts so visualization, agents, and evaluators can consume the same observation path.
_Avoid_: artifact-only observation, sidecar metadata, viewer shortcut

**Runtime payload reference**:
A protocol observation field that names retrievable binary observation data, allowing step responses to carry metadata while clients fetch image, depth, or segmentation payloads separately.
_Avoid_: inline base64 image, local file path contract, remote shared memory

**Array-native observation payload**:
A runtime observation payload that preserves array shape, dtype, and values across the remote runtime boundary without image compression semantics.
_Avoid_: JPEG-first image transport, display-only frame, encoded screenshot

**Runtime observation module**:
A DimOS module that turns remote runtime observation metadata and payload references into normal typed DimOS streams.
_Avoid_: demo script publisher, sidecar-owned DimOS stream, artifact replay

**Step-synchronized observation**:
A runtime observation publication policy where images and camera metadata are emitted from the same episode step that produced the motor state, score metadata, and protocol trace.
_Avoid_: independent polling frame, unsynchronized viewer feed, latest-only observation

**Script-hosted runtime demo**:
A plain Python demo that orchestrates sidecar startup, runtime stepping, local control plumbing, and visualization without becoming a product CLI or benchmark runner.
_Avoid_: production runner, DimOS CLI command, artifact-only smoke test

**Rerun runtime demo**:
A script-hosted runtime demo mode that publishes simulator observations into Rerun through DimOS observation streams for visual inspection.
_Avoid_: simulator viewer shortcut, saved-image artifact, direct Rerun-only bypass

**Stream-backed runtime visualization**:
A visualization path where runtime observations are rendered by consumers of DimOS streams, not by calling a visualization SDK directly at the runtime boundary.
_Avoid_: direct Rerun log call, viewer-only proof, stream bypass

**Canonical demo camera topics**:
The first runtime camera visualization demo publishes a single RGB image and camera model on the conventional `color_image` and `camera_info` topics.
_Avoid_: demo-only topic namespace, artifact image, direct viewer entity

**Damiao-based Robot**:
A robot whose joints are actuated by one or more Damiao motors, possibly spread across multiple CAN buses and physical limbs.
_Avoid_: Damiao arm when the robot may contain multiple motor groups

**Damiao Joint Group**:
An ordered set of Damiao-driven joints that forms a meaningful physical group such as an arm, torso, or other controllable body section.
_Avoid_: Arm when the group is not necessarily an arm

**Damiao Bus**:
A named communication channel used by a Damiao-based Robot to reach one or more Damiao motors.
_Avoid_: Treating a bus as owned by a single joint group when multiple groups may share a channel

**OpenArm**:
An OpenArm robot configuration built from Damiao motors, with OpenArm-specific joints, side naming, limits, and robot description.
_Avoid_: Damiao robot when referring to OpenArm-specific geometry or naming

**Teleop adapter**:
A device-specific bridge from a human teleoperation source, such as a headset controller, phone, keyboard, or physical leader arm, into DimOS coordinator-facing command streams.
_Avoid_: teleop backend when the component emits coordinator command types, teleop module when referring only to the device adapter, controller when the device is not a robot controller

**Teleop profile**:
A configured teleoperation behavior that selects one primary way human intent drives robot motion while allowing secondary engagement, status, and diagnostic signals.
_Avoid_: backend when referring to behavior selection, mode when the distinction affects routing and safety semantics

**Primary motion output**:
The single motion-control path a teleoperation behavior uses to drive robot movement, so one human input source does not unintentionally command the same robot through multiple motion abstractions at once.
_Avoid_: all active outputs, debug stream, secondary status output

**Teleop command envelope**:
A small wrapper around a coordinator-facing teleoperation command that distinguishes an active command from no command and from an explicit stop command.
_Avoid_: using a missing command as a stop signal, overloading raw motion-message contents with teleop authority state
