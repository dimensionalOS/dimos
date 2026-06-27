## ADDED Requirements

### Requirement: LIBERO-PRO sidecar package
The system SHALL provide a first-class LIBERO-PRO runtime sidecar package in the monorepo that depends on the runtime protocol package and isolates LIBERO-PRO-specific dependencies from the main DimOS package.

#### Scenario: LIBERO-PRO sidecar imports without DimOS
- **WHEN** a developer imports the LIBERO-PRO sidecar package module in a normal DimOS development environment without LIBERO-PRO installed
- **THEN** the import succeeds without importing `dimos`, `libero`, `robosuite`, or `torch`

#### Scenario: LIBERO-PRO sidecar installs in isolated environment
- **WHEN** a developer installs the LIBERO-PRO sidecar package in a LIBERO-PRO-compatible environment
- **THEN** the sidecar can start and import runtime protocol models without installing the main DimOS package

### Requirement: Registered LIBERO-PRO task selection
The LIBERO-PRO sidecar SHALL support registered LIBERO-PRO benchmark suites in v1 using backend options for benchmark name, task order index, task index, init-state index, controller, cameras, horizon, and asset roots.

#### Scenario: Registered task is described
- **WHEN** the sidecar is configured with a registered LIBERO-PRO benchmark name, task order index, task index, and init-state index
- **THEN** the runtime description includes task metadata such as benchmark name, task name, language, BDDL path, init-state index, controller, horizon, and camera configuration

#### Scenario: Dynamic perturbation request is rejected
- **WHEN** v1 configuration requests dynamic perturbation generation instead of a registered prepared suite task
- **THEN** the sidecar rejects the setup with a clear error before starting an episode

### Requirement: LIBERO-PRO asset validation and bootstrap
The system SHALL support explicit opt-in LIBERO-PRO runtime asset bootstrap while requiring sidecar startup and health checks to validate prepared assets without downloading or mutating local asset layout.

#### Scenario: Prepared assets validate successfully
- **WHEN** required BDDL and init-state assets exist for the selected registered suite task
- **THEN** the sidecar health or setup validation reports the assets as usable without modifying them

#### Scenario: Missing assets fail clearly
- **WHEN** required BDDL or init-state assets are missing for the selected registered suite task
- **THEN** the sidecar reports a clear validation failure that identifies the missing asset category before episode reset

#### Scenario: Asset bootstrap is explicit
- **WHEN** a developer requests asset preparation through an explicit bootstrap command or demo flag
- **THEN** the system may retrieve and stage supported external assets and then validates the resulting layout before sidecar use

### Requirement: LIBERO-PRO motor surface validation
The LIBERO-PRO sidecar SHALL expose the full-control v1 path only when the selected task and controller provide a Panda joint-position plus gripper whole-body motor surface compatible with DimOS motor action frames.

#### Scenario: Compatible motor surface is described
- **WHEN** the selected LIBERO-PRO environment exposes the expected Panda joint-position plus gripper action surface
- **THEN** the runtime description reports a stable ordered motor surface with supported position command mode and the expected motor count

#### Scenario: Incompatible controller fails fast
- **WHEN** the selected LIBERO-PRO environment exposes only OSC pose control or an action dimension that cannot be mapped to Panda joint-position plus gripper commands
- **THEN** the sidecar rejects the episode setup with a clear protocol error before accepting step requests

### Requirement: LIBERO-PRO step ownership and observation export
The LIBERO-PRO sidecar SHALL own backend-native environment reset and step calls and SHALL translate runtime protocol action frames into LIBERO-PRO actions while exporting motor state, reward, done, success, and observation metadata.

#### Scenario: LIBERO-PRO reset applies init state
- **WHEN** DimOS requests episode reset for a configured LIBERO-PRO registered task
- **THEN** the sidecar resets the environment, applies the selected init state, and returns initial motor state and task metadata

#### Scenario: Motor step advances LIBERO-PRO
- **WHEN** DimOS sends a motor position action frame for the described Panda motor surface
- **THEN** the sidecar maps the action to the LIBERO-PRO environment step and returns motor state, reward, done, success if available, and observation frames

#### Scenario: Camera payload can be fetched
- **WHEN** a LIBERO-PRO step response includes a camera observation with a payload reference
- **THEN** the DimOS runtime client can fetch the referenced `.npy` payload for stream publication and artifacts

### Requirement: Sidecar-owned LIBERO-PRO score
The LIBERO-PRO sidecar SHALL provide normalized episode score output that includes backend-owned success extraction, reward or score, step count, and task metadata.

#### Scenario: Score is collected after episode
- **WHEN** the LIBERO-PRO demo completes, times out, or reaches done
- **THEN** the runner can request score output from the sidecar and write success, reward or score, steps, benchmark name, task name, language, and init-state index with episode artifacts

### Requirement: LIBERO-PRO verification split
The system SHALL verify LIBERO-PRO sidecar behavior with always-on contract tests that do not require real LIBERO-PRO dependencies or data, and SHALL keep real LIBERO-PRO execution behind optional/manual integration coverage.

#### Scenario: Normal CI runs without LIBERO-PRO data
- **WHEN** normal test suites run in the main DimOS development environment
- **THEN** they verify import boundaries, backend option validation, stubbed sidecar endpoints, action-surface failures, and score shape without requiring LIBERO-PRO assets or dependencies

#### Scenario: Manual integration exercises real LIBERO-PRO
- **WHEN** a developer runs the optional real LIBERO-PRO integration with prepared dependencies and assets
- **THEN** it launches the real sidecar, runs the full ControlCoordinator and SHM demo path for one registered task, fetches camera payloads, and writes score and artifacts
