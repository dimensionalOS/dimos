## ADDED Requirements

### Requirement: Route placed modules by runtime placement
Python module deployment SHALL route placed modules to worker pools associated with their runtime placement.

#### Scenario: Placed and unplaced modules deploy together
- **GIVEN** one module placed into `detector-runtime`
- **AND** one regular unplaced Python module
- **WHEN** deployment runs
- **THEN** the placed module is deployed through the `detector-runtime` worker pool
- **AND** the unplaced module is deployed through the default Python worker pool.

### Requirement: Deployment identity remains contract-based
The coordinator SHALL expose placed modules by their Module Contract identity even when the runtime worker instantiates a Runtime Implementation.

#### Scenario: Get instance for placed module
- **GIVEN** a placed Module Contract `DetectorContract`
- **AND** a Runtime Implementation `DetectorImplementation`
- **WHEN** deployment succeeds
- **THEN** coordinator lookup by `DetectorContract` returns the actor proxy
- **AND** callers do not need to know the implementation class to call contract-declared behavior.

### Requirement: Deployment failures roll back new runtime managers
If deploying a runtime-aware deployment slice fails, deployment SHALL not leave partially registered worker managers or placement state that affects later deployments.

#### Scenario: Runtime placement deployment fails
- **GIVEN** deployment creates a runtime worker manager for a placed module
- **AND** the placed module fails to deploy
- **WHEN** deployment reports failure
- **THEN** newly created runtime managers and placement state for the failed deployment slice are cleaned up or made inert
- **AND** existing workers from earlier successful deployments remain unaffected.
