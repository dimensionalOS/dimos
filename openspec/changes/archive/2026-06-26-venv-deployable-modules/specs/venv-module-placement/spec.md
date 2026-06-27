## ADDED Requirements

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
