## ADDED Requirements

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
