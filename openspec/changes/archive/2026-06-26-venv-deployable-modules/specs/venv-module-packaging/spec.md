## ADDED Requirements

### Requirement: Venv-deployable Modules are import-safe in coordinator environments
The system SHALL define venv-deployable Python Modules as import-safe Module classes whose defining files can be imported by the coordinator environment without importing worker-only optional dependencies at module import time.

#### Scenario: Coordinator imports Module class without worker-only dependency
- **WHEN** the coordinator imports a venv-deployable Module class for blueprint wiring
- **THEN** the import succeeds even if the coordinator environment does not have the Module's worker-only runtime dependency installed

#### Scenario: Worker-only dependency is imported at runtime
- **WHEN** the Module starts inside its assigned venv worker environment
- **THEN** it may import and use dependencies declared by its venv module package

### Requirement: Venv Module packages declare dependency closure with pyproject
The system SHALL support separately packaged Python distributions for venv-deployable Modules where each package's `pyproject.toml` declares the dependency closure required by those Modules.

#### Scenario: Venv module package installs into isolated venv
- **WHEN** a named Python runtime environment is prepared for a venv Module package
- **THEN** that environment installs the package according to its own `pyproject.toml` dependency declaration

#### Scenario: Package dependency conflicts are isolated per venv package
- **WHEN** two venv Module packages require incompatible Python dependencies
- **THEN** each package can be installed in a separate named Python runtime environment without forcing those dependencies to resolve together in the coordinator environment

### Requirement: Venv Module packages may depend on current dimos in phase 1
The system SHALL allow phase-1 venv Module packages to depend on the current root `dimos` package while preserving a path to depend on a smaller worker runtime package later.

#### Scenario: Phase-1 package depends on root dimos
- **WHEN** a demo or early venv Module package declares a dependency on the current `dimos` package plus module-specific dependencies
- **THEN** DimOS can launch the package in an isolated venv worker environment without requiring a prior core package split

#### Scenario: Future package depends on worker runtime subset
- **WHEN** a smaller DimOS worker runtime package becomes available
- **THEN** venv Module packages can depend on that runtime package instead of the full root `dimos` package

### Requirement: Demo proves venv worker placement with a lightweight package
The system SHALL include a demo package and blueprint proving that a Module can be declared in one import-safe package and run in a named venv worker environment while the coordinator imports, builds, and wires the blueprint.

#### Scenario: Coordinator imports demo package
- **WHEN** the coordinator imports the demo Module class and builds the demo blueprint
- **THEN** the import and build succeed without running the demo's runtime helper at module import time

#### Scenario: Venv worker uses demo runtime helper
- **WHEN** the demo blueprint runs with the demo Module placed into its named Python runtime environment
- **THEN** the Module uses its package-local runtime helper inside the venv and publishes or responds through normal DimOS Module behavior
