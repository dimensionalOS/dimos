## ADDED Requirements

### Requirement: Reconcile active runtime projects during deployment
Deployment SHALL reconcile every active placed Runtime Project before launching workers for the deployment slice.

#### Scenario: Initial build has active placed runtime
- **GIVEN** a blueprint with an active module placed into a Runtime Project
- **WHEN** the blueprint is built
- **THEN** Runtime Reconciliation runs for that Runtime Project before module workers launch
- **AND** deployment proceeds only after reconciliation succeeds.

#### Scenario: Disabled placed module
- **GIVEN** a blueprint with a placed module that is disabled for deployment
- **WHEN** the blueprint is built
- **THEN** the runtime project used only by that disabled module is not reconciled
- **AND** deployment does not fail because of that inactive runtime project.

### Requirement: Reconcile dynamic deployment slices
Dynamic blueprint loading SHALL reconcile runtime projects required by the newly loaded deployment slice before launching workers for that slice.

#### Scenario: Later blueprint load introduces runtime placement
- **GIVEN** an existing coordinator with running modules
- **WHEN** a blueprint load introduces a new placed module requiring `detector-runtime`
- **THEN** `detector-runtime` is reconciled before the new module worker launches
- **AND** unrelated existing workers are not restarted solely because of this reconciliation.

### Requirement: Enforce deployment barrier
If Runtime Reconciliation fails for any runtime project in a deployment slice, deployment SHALL abort before launching workers for that slice.

#### Scenario: One of multiple runtimes fails
- **GIVEN** a deployment slice requiring multiple runtime projects
- **AND** one runtime project fails reconciliation
- **WHEN** deployment runs
- **THEN** no workers for that deployment slice launch
- **AND** the failure report includes the failing runtime project.

### Requirement: Reconcile independent runtime projects in parallel
Deployment SHALL be able to reconcile independent Runtime Projects in parallel while preserving the deployment barrier.

#### Scenario: Multiple independent runtime projects
- **GIVEN** a deployment slice requiring two independent runtime projects
- **WHEN** Runtime Reconciliation starts
- **THEN** both projects may reconcile concurrently
- **AND** worker launch waits until all required projects have succeeded.
