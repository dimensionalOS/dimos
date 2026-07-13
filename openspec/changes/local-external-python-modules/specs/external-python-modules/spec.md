## ADDED Requirements

### Requirement: Declare a local external Python implementation
DimOS SHALL allow a module author to declare a local external Python implementation with a single implementation import reference while retaining the declaration's normal module configuration, streams, RPC methods, skills, and module references.

#### Scenario: Compose an external declaration with ordinary modules
- **GIVEN** an external module declaration and a normal DimOS module with compatible typed streams or a compatible module reference
- **WHEN** an author includes both declarations in an ordinary Blueprint
- **THEN** DimOS SHALL compose and deploy them through the normal Blueprint and coordinator lifecycle
- **AND** consumers SHALL use the declaration's RPC and skill contract rather than a runtime-specific public API

### Requirement: Resolve the runtime project deterministically
DimOS SHALL resolve an external module's runtime project from the declaration directory's sibling `python/` directory. The runtime project SHALL contain `pyproject.toml` and MAY contain `pixi.toml`.

#### Scenario: Resolve a uv runtime project
- **GIVEN** an external declaration whose sibling `python/` directory contains `pyproject.toml` and no `pixi.toml`
- **WHEN** DimOS deploys the declaration
- **THEN** DimOS SHALL prepare and run the implementation as that uv Python project

#### Scenario: Resolve a Pixi-layered runtime project
- **GIVEN** an external declaration whose sibling `python/` directory contains both `pyproject.toml` and `pixi.toml`
- **WHEN** DimOS deploys the declaration
- **THEN** DimOS SHALL prepare and run the uv Python project through the Pixi environment

#### Scenario: Reject an incomplete runtime project
- **GIVEN** an external declaration whose sibling `python/` directory is absent or lacks `pyproject.toml`
- **WHEN** DimOS deploys the declaration
- **THEN** deployment SHALL fail before the module starts
- **AND** the error SHALL identify the missing runtime-project requirement

### Requirement: Preserve the declaration contract at runtime
DimOS SHALL verify that the external implementation fulfills the declaration contract and SHALL expose RPC and skill calls under the declaration identity.

#### Scenario: Start a valid implementation
- **GIVEN** a runtime implementation that fulfills its declaration contract
- **WHEN** the runtime starts successfully
- **THEN** DimOS SHALL wire its declared streams and module references through the coordinator
- **AND** an RPC client or skill consumer SHALL call the declaration contract successfully

#### Scenario: Reject an invalid implementation
- **GIVEN** an implementation import reference that cannot be imported or an implementation that does not fulfill its declaration contract
- **WHEN** DimOS starts the external runtime
- **THEN** deployment SHALL fail with diagnostics identifying the implementation failure
- **AND** DimOS SHALL clean up the failed runtime process

### Requirement: Manage external runtime lifecycle locally
DimOS SHALL prepare, start, stop, and restart an external runtime as a local process without requiring remote-target configuration or a separate deployment command.

#### Scenario: Restart an external module
- **GIVEN** a running external module in a Blueprint with connected streams or module references
- **WHEN** the module is restarted through the existing coordinator lifecycle
- **THEN** DimOS SHALL launch a fresh external runtime process
- **AND** preserve the module's configured stream and module-reference connections

#### Scenario: Report unexpected runtime termination
- **GIVEN** a successfully started external runtime
- **WHEN** its process exits unexpectedly
- **THEN** DimOS SHALL report the module as failed with bounded runtime diagnostics
- **AND** shutdown of the remaining system SHALL remain safe
