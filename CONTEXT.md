# DimOS Robotics Context

DimOS describes robots, actuators, control surfaces, and manipulation planning using precise robotics terminology. This glossary records stable domain language only, not implementation details.

## Language

**Planning group**:
A named subset of a robot model's joints and frames that can be selected as a planning unit.
_Avoid_: move group, joint group

**Composite planning group**:
A planning group that represents coordinated motion across multiple selected planning groups.
_Avoid_: group combination, combined groups, multi-group plan

**Composite RoboPlan model**:
A RoboPlan-facing robot model that represents multiple registered robot models as one planning scene.
_Avoid_: combined URDF, merged robot scene

**Planning world**:
The authoritative belief state for manipulation planning, including robot state and scene state used by planners.
_Avoid_: planner context, backend instance

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
A blueprint-level mapping that runs selected Python module contracts in named venvs using concrete module implementation descriptors.
_Avoid_: new Module base class, permanent class deployment attribute, global transport setting

**Python venv placement**:
A mapping entry keyed by the coordinator-visible module contract class and valued by a named venv plus implementation descriptor.
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

**SHM runtime data plane**:
A local shared-memory command/state channel between a ControlCoordinator-facing hardware adapter and a DimOS simulator client module, used when high-rate motor control must cross local process boundaries without RPC.
_Avoid_: remote sidecar protocol, public simulator API, benchmark control plane, module object sharing

**Motor state projection**:
The hardware-facing subset of simulator state that resembles what a raw robot driver exposes: actuator positions, velocities, efforts, commands, enable state, and errors.
_Avoid_: task observation, scene observation, evaluator state

**Whole-body motor surface**:
A hardware control surface that treats a robot as an ordered set of motors with per-motor state and commands, independent of whether the robot is a manipulator, mobile base, or humanoid.
_Avoid_: manipulator-only adapter, end-effector API, task action API

**Benchmark episode config**:
A backend-facing declaration of benchmark intent that names the task, robot, runtime constraints, and evaluation setup before any DimOS blueprint is launched.
_Avoid_: hardware config, simulator config, blueprint config

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
