## Purpose

Define the LIBERO-PRO runtime package, registered-suite task selection, asset preparation boundary, motor-control contract, observation export, scoring, and verification split for LIBERO-PRO benchmark demos through a Simulator Runtime Module.
## Requirements
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

### Requirement: Registered LIBERO-PRO task selection
The LIBERO-PRO runtime package SHALL support registered LIBERO-PRO benchmark suites in v1 using backend options for benchmark name, task order index, task index, init-state index, controller, cameras, horizon, and asset roots.

#### Scenario: Registered task is described
- **WHEN** the runtime module is configured with a registered LIBERO-PRO benchmark name, task order index, task index, and init-state index
- **THEN** the runtime description includes task metadata such as benchmark name, task name, language, BDDL path, init-state index, controller, horizon, and camera configuration

#### Scenario: Dynamic perturbation request is rejected
- **WHEN** v1 configuration requests dynamic perturbation generation instead of a registered prepared suite task
- **THEN** the sidecar rejects the setup with a clear error before starting an episode

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

### Requirement: LIBERO-PRO motor surface validation
The LIBERO-PRO runtime package SHALL expose the full-control v1 path only when the selected task and controller provide a Panda joint-position plus gripper whole-body motor surface compatible with DimOS motor action frames.

#### Scenario: Compatible motor surface is described
- **WHEN** the selected LIBERO-PRO environment exposes the expected Panda joint-position plus gripper action surface for motor-frame mode
- **THEN** the runtime description reports a stable ordered motor surface with supported position command mode and the expected motor count

#### Scenario: Incompatible motor mode fails fast
- **WHEN** motor-frame mode is selected but the LIBERO environment exposes only a native end-effector action surface or an action dimension that cannot be mapped to Panda joint-position plus gripper commands
- **THEN** the runtime module rejects the episode setup with a clear protocol error before accepting step requests

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

### Requirement: Sidecar-owned LIBERO-PRO score
The LIBERO-PRO runtime package SHALL provide normalized episode score output that includes backend-owned success extraction, reward or score, step count, and task metadata.

#### Scenario: Score is collected after episode
- **WHEN** the LIBERO-PRO demo completes, times out, or reaches done
- **THEN** the runner can request score output from the runtime module and write success, reward or score, steps, benchmark name, task name, language, and init-state index with episode artifacts

### Requirement: LIBERO-PRO verification split
The system SHALL verify LIBERO-PRO module-runtime behavior with always-on contract tests that do not require real LIBERO-PRO dependencies or data, and SHALL keep real LIBERO-PRO execution behind optional/manual integration coverage.

#### Scenario: Normal CI runs without LIBERO-PRO data
- **WHEN** normal test suites run in the main DimOS development environment
- **THEN** they verify import boundaries, blueprint helper placement, runtime environment selection, stubbed module RPCs, action-surface failures, and score shape without requiring LIBERO-PRO assets or dependencies

#### Scenario: Manual integration exercises real LIBERO-PRO
- **WHEN** a developer runs the optional real LIBERO-PRO integration with prepared dependencies and assets
- **THEN** it launches the placed Simulator Runtime Module, runs reset and synchronous step RPCs for one registered task, observes camera stream publication, and writes score and artifacts

### Requirement: Native LIBERO action mode
The LIBERO runtime module SHALL support a native LIBERO action mode that follows the official LeRobot LIBERO action setup for relative end-effector delta plus gripper actions.

#### Scenario: Native action mode validates environment action spec
- **WHEN** the runtime module starts in native LIBERO action mode
- **THEN** it inspects the LIBERO environment action spec and requires action dimension `(7,)` with bounds compatible with `[-1, 1]`

#### Scenario: Native action mode is described
- **WHEN** the runtime module description is requested in native LIBERO action mode
- **THEN** it reports the native action surface identifier, action shape, action bounds, action mode metadata, task metadata, language, horizon, and camera configuration

#### Scenario: Native action mode accepts runtime action frame
- **WHEN** DimOS sends a runtime action frame with `space_id` `libero.ee_delta_6d_gripper.normalized.v1` and valid `float32[7]` values
- **THEN** the runtime module maps the values directly to the LIBERO environment step action and returns observations, reward, done, and success metadata

#### Scenario: Native action mode rejects motor frame
- **WHEN** the runtime module is running in native LIBERO action mode and receives a motor action frame
- **THEN** it rejects the step request with a clear protocol error

### Requirement: Native LIBERO observation export for policy rollout
The LIBERO runtime module SHALL export the observations needed by the VLA-JEPA LIBERO policy contract when running native LIBERO action mode through DimOS streams and runtime metadata.

#### Scenario: Policy observations are available after reset
- **WHEN** the runtime module resets a registered task in native LIBERO action mode
- **THEN** DimOS streams and runtime metadata include agent-view camera observation metadata, wrist or eye-in-hand camera observation metadata when available, robot state observation metadata, and task language metadata for contract conversion

#### Scenario: Policy observations are available after step
- **WHEN** the runtime module completes a native runtime action step
- **THEN** DimOS streams and runtime metadata include updated camera and robot state observations needed for the next policy inference tick
