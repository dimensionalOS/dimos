## Purpose

Define blueprint-level placement of import-safe Python Modules into named local Python runtime environments while preserving normal DimOS worker protocol behavior.

## Requirements

### Requirement: Blueprints place Modules into named Python runtime environments
The system SHALL allow blueprints to place selected import-safe Python Module classes into named Python runtime environments without making the placement an intrinsic Module class property.

#### Scenario: Blueprint places one Module into a venv environment
- **WHEN** a blueprint places `CameraModule` into named runtime environment `sensors`
- **THEN** the coordinator deploys that Module to a worker process launched from the `sensors` Python environment

#### Scenario: Same Module can run without venv placement
- **WHEN** the same Module class appears in a blueprint without venv placement
- **THEN** the coordinator deploys it through the default Python worker pool

### Requirement: Venv workers preserve DimOS worker protocol semantics
The system SHALL preserve normal DimOS worker request/response semantics for Modules deployed into venv worker processes.

#### Scenario: Venv Module receives lifecycle RPCs
- **WHEN** the coordinator builds, starts, stops, or undeploys a Module placed into a venv worker
- **THEN** the Module receives the same lifecycle calls it would receive in the default Python worker pool

#### Scenario: Venv Module participates in stream wiring
- **WHEN** a Module placed into a venv worker has typed input or output streams
- **THEN** the coordinator wires those streams using the normal DimOS stream transport mechanism

#### Scenario: Venv Module participates in Module refs and RPC calls
- **WHEN** another Module calls an RPC or uses a ref exposed by a Module placed into a venv worker
- **THEN** the call uses the normal DimOS proxy behavior and returns or fails according to the existing worker protocol semantics

### Requirement: Venv workers use local multiprocessing connection control channel
The system SHALL launch venv worker processes as separate Python interpreters and use Python `multiprocessing.connection.Listener/Client` for the same-machine worker control channel.

#### Scenario: Venv worker connects to coordinator control channel
- **WHEN** the coordinator launches a venv worker process
- **THEN** the worker connects to the coordinator's local multiprocessing connection endpoint before receiving deploy requests

#### Scenario: Worker logs do not corrupt control messages
- **WHEN** a venv worker or hosted Module writes to stdout or stderr
- **THEN** those logs do not corrupt the worker request/response control channel

### Requirement: Named venv worker pools isolate Python environments
The system SHALL maintain separate worker pools for distinct named Python runtime environments and SHALL NOT mix Modules assigned to different Python environments in the same worker process.

#### Scenario: Multiple Modules share one named venv pool
- **WHEN** two Modules are placed into the same named Python runtime environment and capacity allows sharing
- **THEN** they may be hosted by the same venv worker pool according to existing worker capacity rules

#### Scenario: Modules in different named venvs are isolated
- **WHEN** two Modules are placed into different named Python runtime environments
- **THEN** they run in separate worker pools launched from their respective Python interpreters

### Requirement: Venv deployment fails during normal build lifecycle on incompatible environments
The system SHALL fail the normal blueprint build/deploy lifecycle when a named venv environment is missing, incompatible, or unable to import required DimOS worker runtime code.

#### Scenario: Missing venv Python fails build
- **WHEN** a placement references a Python runtime environment whose configured Python executable does not exist
- **THEN** blueprint build fails with an actionable error identifying the runtime environment and missing executable

#### Scenario: Worker runtime import failure fails build
- **WHEN** a venv worker starts but cannot import compatible DimOS worker runtime modules
- **THEN** deployment fails instead of silently degrading into a partial blueprint

### Requirement: Project runtime workers preserve worker lifecycle semantics
The system SHALL deploy Modules placed into Python project runtimes through the existing DimOS worker lifecycle and control-channel semantics.

#### Scenario: Project runtime worker connects to coordinator
- **WHEN** the coordinator launches a worker through a uv-only or Pixi-backed project runtime command
- **THEN** the worker connects to the coordinator's local multiprocessing connection endpoint before receiving deploy requests

#### Scenario: Project runtime worker shuts down normally
- **WHEN** a project-runtime worker receives the normal DimOS shutdown request
- **THEN** the worker exits through the existing worker shutdown path without requiring a separate shutdown protocol for uv or Pixi wrappers

#### Scenario: Project runtime worker termination fallback
- **WHEN** a project-runtime worker does not exit after the normal shutdown timeout
- **THEN** DimOS applies the existing worker process-handle termination fallback to the launched process and tests verify whether additional cleanup is necessary

### Requirement: Project runtime placement remains active-module scoped
The system SHALL keep Python project runtime placement scoped to active module placements in the loaded blueprint configuration.

#### Scenario: Unused project runtime is not launched
- **WHEN** a blueprint registers a Python project runtime environment that is not referenced by any active module placement
- **THEN** blueprint build does not prepare, launch, or allocate a worker pool for that runtime

#### Scenario: Same Module can run without project runtime placement
- **WHEN** a Module class appears in a blueprint without project runtime placement after previously being used with a project runtime
- **THEN** the coordinator deploys it through the default Python worker pool

### Requirement: Project runtime placement supports real GPD grasp detector Modules
The system SHALL support placing a real GPD grasp detector Module into a Python project runtime while preserving normal DimOS blueprint wiring and worker lifecycle behavior.

#### Scenario: Blueprint places GPD generator into project runtime
- **WHEN** the GPD MuJoCo demo blueprint is built
- **THEN** the GPD grasp detector Module is assigned to the named GPD Python project runtime and other demo Modules remain in their normal runtime placements

#### Scenario: GPD placement uses existing worker protocol
- **WHEN** the placed GPD generator receives lifecycle calls, stream wiring, or RPC calls
- **THEN** those operations use the same DimOS worker protocol semantics as other project-runtime Modules

#### Scenario: GPD demo remains opt-in
- **WHEN** existing xArm perception or VGN demo blueprints are loaded
- **THEN** they do not implicitly include the GPD grasp generator or prepare/launch its project runtime
