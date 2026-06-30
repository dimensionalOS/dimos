## MODIFIED Requirements

### Requirement: LIBERO-PRO sidecar package
The system SHALL evolve the first-class LIBERO-PRO runtime package into an import-safe package that provides a Simulator Runtime Module for LIBERO-PRO execution and removes the existing HTTP server boundary as part of migration success.

#### Scenario: LIBERO-PRO module imports without backend dependencies
- **WHEN** a developer imports the LIBERO-PRO runtime module class or blueprint helper in a normal DimOS development environment without LIBERO-PRO installed
- **THEN** the import succeeds without importing `libero`, `robosuite`, `torch`, or backend asset libraries

#### Scenario: LIBERO-PRO runtime installs in isolated environment
- **WHEN** a developer prepares the LIBERO-PRO runtime package in a LIBERO-PRO-compatible Python project runtime environment
- **THEN** the placed DimOS worker can import DimOS worker runtime support, runtime protocol models, LIBERO-PRO, and required backend dependencies

#### Scenario: HTTP entrypoint is removed
- **WHEN** the LIBERO-PRO Simulator Runtime Module covers runtime description, reset, step, observation streams, score, and demo execution
- **THEN** the package no longer exposes an HTTP runtime server entrypoint for benchmark execution

### Requirement: LIBERO-PRO asset validation and bootstrap
The system SHALL support explicit opt-in LIBERO-PRO runtime asset bootstrap while requiring module reset/setup validation to report prepared asset failures without downloading or mutating local asset layout by default.

#### Scenario: Prepared assets validate successfully
- **WHEN** required BDDL and init-state assets exist for the selected registered suite task
- **THEN** module reset or setup validation reports the assets as usable without modifying them

#### Scenario: Missing assets fail clearly
- **WHEN** required BDDL or init-state assets are missing for the selected registered suite task
- **THEN** module `reset()` fails synchronously with a clear validation error that identifies the missing asset category

#### Scenario: Asset bootstrap is explicit
- **WHEN** a developer requests asset preparation through an explicit bootstrap command or demo flag
- **THEN** the system may retrieve and stage supported external assets and then validates the resulting layout before module reset uses the assets

### Requirement: LIBERO-PRO step ownership and observation export
The LIBERO-PRO runtime package SHALL own backend-native environment reset and step calls inside its Simulator Runtime Module and SHALL translate runtime motor action frames into LIBERO-PRO actions while exporting motor state, reward, done, success, and observations through module RPC metadata and DimOS-native typed streams.

#### Scenario: LIBERO-PRO reset applies init state through RPC
- **WHEN** DimOS calls module `reset()` for a configured LIBERO-PRO registered task
- **THEN** the module resets the environment, applies the selected init state on the simulator owner thread, and returns initial task and motor metadata

#### Scenario: Motor step advances LIBERO-PRO through RPC
- **WHEN** DimOS calls module `step()` with a motor position action frame for the described Panda motor surface
- **THEN** the module maps the action to the LIBERO-PRO environment step on the simulator owner thread and returns reward, done, success if available, and lightweight step metadata

#### Scenario: Camera observation publishes as Image and CameraInfo streams
- **WHEN** a LIBERO-PRO step produces a configured camera observation
- **THEN** the module publishes the camera output as `Image` and camera metadata as `CameraInfo` through DimOS streams without requiring DimOS to fetch a `.npy` payload from an HTTP endpoint or receive a raw NumPy array in the step response

### Requirement: LIBERO-PRO verification split
The system SHALL verify LIBERO-PRO module-runtime behavior with always-on contract tests that do not require real LIBERO-PRO dependencies or data, and SHALL keep real LIBERO-PRO execution behind optional/manual integration coverage.

#### Scenario: Normal CI runs without LIBERO-PRO data
- **WHEN** normal test suites run in the main DimOS development environment
- **THEN** they verify import boundaries, blueprint helper placement, runtime environment selection, stubbed module RPCs, action-surface failures, and score shape without requiring LIBERO-PRO assets or dependencies

#### Scenario: Manual integration exercises real LIBERO-PRO module
- **WHEN** a developer runs the optional real LIBERO-PRO integration with prepared dependencies and assets
- **THEN** it launches the placed Simulator Runtime Module, runs reset and synchronous step RPCs for one registered task, observes camera stream publication, and writes score and artifacts
