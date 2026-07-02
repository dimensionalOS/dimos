## ADDED Requirements

### Requirement: Place Module Contracts into runtime environments
Blueprint authors SHALL be able to place a dependency-light Module Contract into a named runtime environment and select a Runtime Implementation for execution.

#### Scenario: Contract is placed with implementation path
- **GIVEN** a Module Contract used in a blueprint
- **AND** a runtime placement binds the contract to runtime `detector-runtime`
- **AND** the placement names a Runtime Implementation import path
- **WHEN** the blueprint deploys
- **THEN** the coordinator keeps the Module Contract as the module identity
- **AND** the runtime worker instantiates the Runtime Implementation.

### Requirement: Runtime placement is keyed by module class
Runtime placement SHALL target module classes rather than module names, instance identifiers, or arbitrary strings.

#### Scenario: Placement uses a contract class
- **GIVEN** a runtime placement for `DetectorContract`
- **WHEN** the blueprint deploys
- **THEN** all DimOS coordinator identity for that module uses `DetectorContract`
- **AND** `get_instance(DetectorContract)` returns the actor proxy for the runtime implementation.

### Requirement: Runtime Implementation subclasses Module Contract
For this change, a Runtime Implementation MUST subclass the Module Contract named by its runtime placement.

#### Scenario: Implementation is compatible by subclassing
- **GIVEN** a placement from `DetectorContract` to implementation `detector_runtime.Detector`
- **AND** `detector_runtime.Detector` subclasses `DetectorContract`
- **WHEN** the runtime worker deploys the placed module
- **THEN** deployment succeeds if normal module construction succeeds.

#### Scenario: Implementation is not a subclass
- **GIVEN** a placement from `DetectorContract` to implementation `detector_runtime.Detector`
- **AND** `detector_runtime.Detector` does not subclass `DetectorContract`
- **WHEN** the runtime worker validates the placement
- **THEN** deployment fails before starting the module
- **AND** the error names the contract and implementation.
