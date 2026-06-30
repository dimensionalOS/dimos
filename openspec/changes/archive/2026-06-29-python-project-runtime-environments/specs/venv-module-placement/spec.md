## ADDED Requirements

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
