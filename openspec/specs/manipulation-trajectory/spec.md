## Purpose

Define first-class manipulation trajectory parametrization as the boundary between geometric planning and control-task dispatch, including generated trajectory artifacts, backend policy, speed scaling, and status semantics.

## Requirements

### Requirement: First-class trajectory parametrization spec
The system SHALL provide a `TrajectoryParametrizerSpec` manipulation-planning role that converts a successful geometric `GeneratedPlan` into a time-parametrized `GeneratedTrajectory`.

#### Scenario: Parametrizer consumes geometric plan
- **WHEN** geometric planning succeeds and produces a `GeneratedPlan`
- **THEN** trajectory parametrization MUST consume the geometric path without overwriting the planning result

#### Scenario: Parametrizer remains independent of coordinator wiring
- **WHEN** a trajectory parametrization backend is implemented
- **THEN** it MUST NOT depend on `ControlCoordinator` task names, task instances, or task-specific dispatch wiring

#### Scenario: Parametrizer accepts runtime speed policy without storing it
- **WHEN** a caller invokes `TrajectoryParametrizerSpec.parametrize`
- **THEN** the caller MAY provide a `speed_scale` argument for that invocation and the backend MUST NOT require persistent mutable speed state on the parametrizer spec

### Requirement: Generated trajectory artifact
The system SHALL define `GeneratedTrajectory` as the canonical global time-parametrized manipulation artifact produced from a `GeneratedPlan`.

#### Scenario: Generated trajectory is global
- **WHEN** a `GeneratedTrajectory` is produced
- **THEN** it MUST represent the global selected joint trajectory rather than task-specific or robot-local command messages

#### Scenario: Generated trajectory has explicit status
- **WHEN** trajectory parametrization succeeds or fails
- **THEN** `GeneratedTrajectory` MUST report an explicit parametrization status and message without changing `GeneratedPlan.status`

### Requirement: Shared trajectory time domain
A `GeneratedTrajectory` produced from a composite or multi-robot `GeneratedPlan` SHALL preserve one shared time domain across all selected joints.

#### Scenario: Composite trajectory is parametrized
- **WHEN** a composite or multi-robot `GeneratedPlan` is parametrized
- **THEN** all selected joints in the resulting `GeneratedTrajectory` MUST use the same timing basis

#### Scenario: Dispatch derives local messages
- **WHEN** task-specific or robot-local command messages are derived from a composite `GeneratedTrajectory`
- **THEN** those messages MUST preserve the generated trajectory's shared timing basis

### Requirement: Trajectory dispatch boundary
The system SHALL define a `TrajectoryDispatch` execution-preparation artifact that derives control-task-specific `JointTrajectory` messages from a `GeneratedTrajectory`.

#### Scenario: Dispatch prepares coordinator messages
- **WHEN** execution is requested for a successful `GeneratedTrajectory`
- **THEN** manipulation orchestration MUST derive the required task-specific `JointTrajectory` messages through dispatch before invoking control tasks

#### Scenario: Dispatch is separate from parametrization
- **WHEN** a `GeneratedTrajectory` is produced
- **THEN** execution-specific projections MUST be produced by a separate dispatch/preparation step rather than stored as canonical generated trajectory data

### Requirement: Shared generated trajectory for preview, validation, benchmarking, and execution
Preview, validation, benchmarking, and execution dispatch SHALL consume the same `GeneratedTrajectory` artifact for a planned manipulation motion where those paths exist.

#### Scenario: Preview uses generated trajectory timing
- **WHEN** a planned motion is previewed
- **THEN** preview MUST use the timing from `GeneratedTrajectory` rather than assigning an independent preview duration to the geometric `GeneratedPlan`

#### Scenario: Execution dispatch uses generated trajectory timing
- **WHEN** a planned motion is executed
- **THEN** execution dispatch MUST consume the same `GeneratedTrajectory` artifact used by preview, validation, or benchmarking for that motion

### Requirement: Parametrization backend policy
The system SHALL support backend-configurable trajectory parametrization policy with a default `simple_trapezoid` backend and an explicit opt-in RoboPlan backend.

#### Scenario: Default backend is available
- **WHEN** no trajectory parametrization backend is explicitly configured
- **THEN** the system MUST use a `simple_trapezoid` backend that preserves current baseline timing behavior behind the new spec boundary

#### Scenario: RoboPlan backend is configured with RoboPlan world
- **WHEN** `backend="roboplan"` is configured with `world_backend="roboplan"`
- **THEN** the system MUST use RoboPlan-owned scene, group, native-joint mapping, and bundled TOPP-RA wrapper for trajectory parametrization

#### Scenario: RoboPlan backend is configured with a non-RoboPlan world
- **WHEN** `backend="roboplan"` is configured without `world_backend="roboplan"`
- **THEN** initialization MUST fail clearly with guidance to select the RoboPlan world backend

#### Scenario: RoboPlan TOPP-RA wrapper is unavailable
- **WHEN** `backend="roboplan"` is configured and RoboPlan's trajectory parametrization wrapper cannot be imported
- **THEN** parametrization MUST report `BACKEND_UNAVAILABLE` with guidance to install RoboPlan-backed TOPP-RA support

#### Scenario: RoboPlan backend policy is reproducible
- **WHEN** RoboPlan parametrization is configured for tests or benchmarks
- **THEN** the RoboPlan wrapper options (`dt`, spline mode, adaptive controls, and velocity/acceleration scales) MUST be explicit enough for reproducible comparisons

### Requirement: Runtime motion speed tuning
The system SHALL expose a runtime motion speed scale API that affects future plan-generated trajectory parametrizations without changing geometric plans, static backend config, or already generated trajectories.

#### Scenario: Runtime speed scale is updated
- **WHEN** an operator or agent sets a valid motion speed scale in `(0, 1]`
- **THEN** the module MUST store the new runtime scale for the next generated trajectory without clearing an existing cached `GeneratedTrajectory` or mutating the cached `GeneratedPlan`

#### Scenario: Runtime speed scale is applied during future parametrization
- **WHEN** a new generated plan is parametrized after the runtime speed scale is changed
- **THEN** the effective velocity and acceleration scales passed to the backend MUST multiply configured scales by the runtime speed scale

#### Scenario: Existing generated trajectory remains frozen
- **WHEN** the runtime speed scale changes after a `GeneratedTrajectory` has already been produced
- **THEN** the existing generated trajectory MUST remain available for preview and execution with the speed scale captured when it was generated

#### Scenario: Invalid runtime speed scale is rejected
- **WHEN** an operator or agent sets a speed scale that is not greater than zero or is greater than one
- **THEN** the module MUST reject the update and avoid changing the current runtime speed scale

### Requirement: RoboPlan TOPP-RA packaging and repository tests
The system SHALL expose RoboPlan-backed TOPP-RA support through the existing `manipulation-toppra` optional extra, include that extra in `all`, and run RoboPlan trajectory parametrization tests unconditionally in the repository test environment.

#### Scenario: Runtime user omits RoboPlan backend
- **WHEN** a runtime installation does not configure `backend="roboplan"`
- **THEN** baseline manipulation trajectory parametrization MUST remain available through `simple_trapezoid`

#### Scenario: Repository tests run
- **WHEN** the repository test environment runs trajectory parametrization tests
- **THEN** RoboPlan trajectory parametrization tests MUST run without import-based skipping

### Requirement: Separate planning, parametrization, dispatch, and execution statuses
The system SHALL distinguish geometric planning status, trajectory parametrization status, dispatch preparation status, and runtime execution status.

#### Scenario: Parametrization is infeasible after planning succeeds
- **WHEN** geometric planning succeeds but trajectory constraints are infeasible
- **THEN** `GeneratedPlan.status` MUST remain successful and `GeneratedTrajectory.status` MUST report parametrization infeasibility

#### Scenario: Dispatch fails after parametrization succeeds
- **WHEN** trajectory parametrization succeeds but coordinator task dispatch fails
- **THEN** `GeneratedTrajectory.status` MUST remain successful and dispatch or execution status MUST report the dispatch failure

### Requirement: RoboPlan trajectory parametrization uses a validated post-processing pipeline
When RoboPlan trajectory parametrization is configured, the system SHALL run geometric path post-processing through explicit internal stages before TOPP-RA retiming.

#### Scenario: RoboPlan post-processing runs before TOPP-RA
- **WHEN** a successful `GeneratedPlan` is parametrized with the RoboPlan backend
- **THEN** the backend MUST interpret the input path, optionally preprocess the geometric path, validate any refined path, and only then pass the accepted path to RoboPlan TOPP-RA

#### Scenario: Pipeline stages remain internally separable
- **WHEN** RoboPlan post-processing is implemented under the trajectory parametrization facade
- **THEN** input interpretation, preprocessing, validation, TOPP-RA retiming, and fallback handling MUST be represented as separable internal responsibilities

### Requirement: RoboPlan smoothing is conservative and enabled by default
RoboPlan trajectory parametrization SHALL enable conservative geometric smoothing by default for eligible paths.

#### Scenario: Default smoothing is enabled
- **WHEN** `backend="roboplan"` is configured and no smoothing override is provided
- **THEN** the RoboPlan backend MUST attempt conservative smoothing for eligible generated plans before TOPP-RA retiming

#### Scenario: Smoothing can be disabled
- **WHEN** configuration disables RoboPlan smoothing
- **THEN** RoboPlan trajectory parametrization MUST parametrize the original geometric path without running smoothing preprocessing

#### Scenario: Smoothing skips ineligible short paths
- **WHEN** a generated plan has fewer waypoints than the configured smoothing minimum
- **THEN** RoboPlan trajectory parametrization MUST skip smoothing and parametrize the original geometric path

### Requirement: Smoothing validation preserves geometric correctness
The system SHALL accept a smoothed or simplified path only after validation confirms it preserves the source plan's applicable constraints.

#### Scenario: Generic validation succeeds
- **WHEN** a path without explicit path-constraint metadata is smoothed
- **THEN** validation MUST confirm endpoint preservation, selected-joint compatibility, joint limits, collision acceptability, and configured maximum joint-space deviation before the smoothed path is used

#### Scenario: Path-constraint metadata is enforced
- **WHEN** a generated plan includes path-constraint metadata
- **THEN** validation MUST enforce those declared constraints before accepting the smoothed path

#### Scenario: Invalid smoothing candidate is rejected
- **WHEN** a smoothing candidate violates validation
- **THEN** the backend MUST NOT pass that candidate to TOPP-RA

### Requirement: RoboPlan smoothing failure is non-blocking
RoboPlan smoothing failures SHALL fall back to the original geometric path rather than failing trajectory parametrization by themselves.

#### Scenario: Smoothing validation fails
- **WHEN** every smoothing attempt fails validation
- **THEN** RoboPlan trajectory parametrization MUST parametrize the original geometric path
- **AND** smoothing failure alone MUST NOT produce a failed `GeneratedTrajectory`

#### Scenario: Conservative retry preserves more waypoints
- **WHEN** an aggressive smoothing attempt fails validation
- **THEN** the backend MUST retry with a more conservative candidate when additional smoothing attempts are configured

#### Scenario: Original parametrization can still fail
- **WHEN** smoothing falls back to the original path and RoboPlan TOPP-RA cannot parametrize the original path
- **THEN** trajectory parametrization MUST report the TOPP-RA failure through `GeneratedTrajectory.status` and message

### Requirement: Path-constraint metadata is preserved through generated plans
The system SHALL allow geometric generated plans to carry optional path-constraint metadata for downstream trajectory post-processing.

#### Scenario: GeneratedPlan carries optional metadata
- **WHEN** a `GeneratedPlan` is constructed without path-constraint metadata
- **THEN** the plan MUST remain valid and behave as an unconstrained geometric path for post-processing purposes

#### Scenario: Planning metadata reaches parametrization
- **WHEN** a successful planning result contains path-constraint metadata and `ManipulationModule` stores it as a `GeneratedPlan`
- **THEN** the resulting `GeneratedPlan` MUST preserve that metadata for trajectory parametrization
