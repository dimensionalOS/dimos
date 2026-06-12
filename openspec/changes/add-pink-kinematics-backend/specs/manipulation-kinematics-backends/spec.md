## ADDED Requirements

### Requirement: Selectable Pink kinematics backend
DimOS SHALL allow manipulation planning users to select a Pink-based kinematics backend by name without changing existing Cartesian planning call sites.

#### Scenario: Pink backend selected for Cartesian planning
- **GIVEN** a manipulation planning stack configured with a supported robot model and `kinematics_name` set to `pink`
- **WHEN** a caller requests planning to a reachable Cartesian end-effector pose
- **THEN** the system SHALL attempt IK using the Pink backend through the same public planning behavior used by other kinematics backends
- **AND** the successful result SHALL be usable by the existing joint-path planning flow.

#### Scenario: Pink backend selected for direct IK
- **GIVEN** a manipulation planning stack configured with a supported robot model and `kinematics_name` set to `pink`
- **WHEN** a caller requests an IK-only solve for a reachable Cartesian end-effector pose
- **THEN** the system SHALL return an `IKResult` from the Pink backend without invoking joint-path planning
- **AND** the result SHALL include the solved joint state when IK succeeds
- **AND** callers MAY provide an explicit seed joint state to initialize local IK, otherwise the current robot joint state SHALL be used.

#### Scenario: Existing backends remain available
- **GIVEN** a manipulation planning stack configured with `kinematics_name` set to an existing backend such as `jacobian`
- **WHEN** the stack starts and plans to a Cartesian target
- **THEN** the existing backend behavior SHALL remain available without requiring Pink-specific dependencies.

### Requirement: Clear Pink dependency failures
DimOS SHALL report actionable errors when the Pink backend is requested but required Pink or QP solver dependencies are unavailable.

#### Scenario: Pink package missing
- **GIVEN** an environment where Pink is not installed
- **WHEN** a manipulation planning stack is configured with `kinematics_name` set to `pink`
- **THEN** startup or backend creation SHALL fail with a message that identifies the missing Pink dependency
- **AND** other kinematics backends SHALL remain importable and selectable in the same environment.

#### Scenario: QP solver unavailable
- **GIVEN** an environment where Pink is installed but no supported QP solver is available
- **WHEN** the Pink backend attempts to solve IK
- **THEN** the system SHALL return or raise an actionable solver-availability failure rather than silently falling back to an unrelated backend.

### Requirement: Planning safety compatibility
DimOS SHALL preserve existing planning safety expectations when using the Pink backend.

#### Scenario: Collision checking enabled
- **GIVEN** a Pink IK candidate for a target pose and collision checking enabled
- **WHEN** the candidate violates the planning world's collision checks
- **THEN** the IK result SHALL be reported as unsuccessful
- **AND** the candidate SHALL NOT be passed onward as a successful joint target for path planning.

#### Scenario: IK fails to converge
- **GIVEN** a target pose outside the supported convergence behavior of the Pink backend
- **WHEN** the configured iteration or attempt budget is exhausted
- **THEN** the system SHALL report an unsuccessful IK result with convergence failure details
- **AND** the manipulation planning flow SHALL NOT execute a trajectory for that failed IK result.

### Requirement: Simulation manual QA path
DimOS SHALL provide a simulation-safe manual QA path for validating the Pink kinematics backend without commanding physical hardware.

#### Scenario: Run Pink QA path in simulation
- **GIVEN** Pink dependencies are installed in the development environment
- **WHEN** the operator starts an existing simulation-safe manipulation blueprint with `kinematics_name` overridden to `pink` and plans to a conservative reachable Cartesian target
- **THEN** the system SHALL run the manipulation stack with the Pink backend selected
- **AND** the operator SHALL be able to preview the planned path before any execution step.

#### Scenario: Compare against baseline backend
- **GIVEN** the same simulated robot and Cartesian target
- **WHEN** the operator runs the baseline Jacobian simulation blueprint and repeats the planning request
- **THEN** the operator SHALL be able to compare Pink planning behavior against the existing backend from the same manual client workflow.
