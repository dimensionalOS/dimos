## ADDED Requirements

### Requirement: Runtime environments are named and Python-configurable
The system SHALL provide a Python API for registering named runtime environments that DimOS-managed processes can reference at blueprint build and module runtime.

#### Scenario: Register runtime environments through Python API
- **WHEN** a blueprint or runtime configuration registers named runtime environments using typed Python objects
- **THEN** the coordinator resolves those environment names without requiring a YAML or TOML configuration file

#### Scenario: Missing runtime environment name fails clearly
- **WHEN** a Module placement or NativeModule config references a runtime environment name that is not registered
- **THEN** blueprint build or module build fails with an error identifying the missing runtime environment name

### Requirement: Runtime environments expose capability-specific resolution
The system SHALL let runtime environment consumers request only the runtime capability they need, such as Python interpreter resolution for worker launch or native executable resolution for `NativeModule` launch.

#### Scenario: Python venv worker requests Python launch material
- **WHEN** a venv worker placement references a Python venv runtime environment
- **THEN** the worker launcher receives a Python executable path and environment variables needed to start the worker process

#### Scenario: Native module requests native executable material
- **WHEN** a NativeModule references a Nix-backed runtime environment
- **THEN** the NativeModule receives executable, cwd, environment variables, and optional preparation/build material without transferring native process lifecycle ownership to the registry

### Requirement: Runtime environments support current and Nix-backed execution material
The system SHALL support at least a current-process environment backend and a Nix-backed environment backend sufficient to model existing native module build/executable configuration.

#### Scenario: Current environment backend is used
- **WHEN** a Module or process uses the current runtime environment
- **THEN** DimOS launches it with the coordinator's current Python/runtime environment semantics

#### Scenario: Nix-backed environment resolves existing native configuration
- **WHEN** a Nix-backed runtime environment names a flake/package/executable equivalent to an existing native module `build_command` and `executable`
- **THEN** NativeModule can launch the same native process behavior through named environment resolution

### Requirement: NativeModule legacy configuration remains supported
The system SHALL keep existing `NativeModuleConfig` executable, build command, cwd, and extra environment fields usable while adding named runtime environment resolution.

#### Scenario: Legacy native configuration continues to work
- **WHEN** a NativeModule is configured only with existing executable/build/cwd/extra_env fields
- **THEN** it builds and starts using the same behavior as before runtime environment registry support

#### Scenario: Runtime environment and legacy overrides are combined deterministically
- **WHEN** a NativeModule references a runtime environment and also supplies supported legacy override fields
- **THEN** DimOS applies a documented precedence order and launches with the resulting executable, cwd, env, and build behavior
