## ADDED Requirements

### Requirement: A single trajectory parametrization backend is selected at startup

The manipulation stack SHALL select exactly one trajectory parametrization backend during startup and SHALL use that backend for every untimed geometric path materialized during that run.

#### Scenario: Simple backend is selected
- **GIVEN** a manipulation stack configured with the simple trajectory parametrization backend
- **WHEN** the stack starts successfully
- **THEN** every accepted geometric path MUST be converted by the simple backend
- **AND** the backend MUST remain unchanged for the lifetime of the running stack

#### Scenario: RoboPlan TOPP-RA backend is selected
- **GIVEN** a manipulation stack configured with the RoboPlan TOPP-RA backend and `RoboPlanWorld`
- **WHEN** the stack starts successfully
- **THEN** every accepted geometric path MUST be converted by RoboPlan TOPP-RA
- **AND** the planner that produced the path MUST NOT be required to be RoboPlan's planner

#### Scenario: Backend and world are incompatible
- **GIVEN** a manipulation stack configured with RoboPlan TOPP-RA and a non-RoboPlan world
- **WHEN** the stack is initialized
- **THEN** initialization MUST fail with an actionable configuration error
- **AND** planning MUST NOT begin with a backend that cannot operate against the configured world

#### Scenario: Planner returns a timed trajectory
- **GIVEN** a planner returns a trajectory with authoritative timestamps and velocities
- **WHEN** the generated plan is materialized
- **THEN** the system MUST preserve and canonically validate the planner-provided timing
- **AND** it MUST NOT invoke a trajectory parametrization backend
- **AND** this bypass MUST NOT be treated as backend fallback

### Requirement: Parametrization completes before a generated plan is accepted

The manipulation stack SHALL convert an accepted geometric path into a timed trajectory before exposing or caching the corresponding generated plan.

#### Scenario: Parametrization succeeds
- **GIVEN** a planner has returned a successful geometric path
- **WHEN** the selected backend produces a valid timed trajectory
- **THEN** the system SHALL construct and cache one generated plan containing the source path and timed trajectory
- **AND** preview and execution MUST consume that same accepted trajectory

#### Scenario: Parametrization fails
- **GIVEN** a planner has returned a successful geometric path
- **WHEN** the selected backend cannot produce a valid timed trajectory
- **THEN** the system MUST report plan materialization failure
- **AND** it MUST NOT cache or expose that path as an executable generated plan

#### Scenario: Existing planner timing is accepted
- **GIVEN** a planner has returned a valid timed trajectory
- **WHEN** canonical timed-output validation succeeds
- **THEN** the system SHALL construct and cache one generated plan containing the source path and planner-native trajectory
- **AND** preview and execution MUST consume that same trajectory without retiming

### Requirement: Parametrization converts a path into continuous timed motion

The selected backend SHALL convert the source geometric path into one trajectory with a shared time domain across every selected joint.

#### Scenario: Multi-waypoint path is converted
- **GIVEN** a valid path containing two or more consistently ordered joint configurations
- **WHEN** the path is parametrized
- **THEN** the result MUST contain timed positions and velocities for every selected joint
- **AND** its time values MUST start at zero and increase strictly through the final trajectory duration

#### Scenario: Backend performs bounded curve fitting
- **GIVEN** a backend mode that fits continuous motion between supplied waypoints
- **WHEN** the path is parametrized
- **THEN** the backend MAY produce samples that do not coincide with every interior waypoint
- **AND** it MUST preserve the source start and goal within the configured numerical tolerance
- **AND** it MUST honor its configured geometric-deviation and collision-preservation contract

#### Scenario: Source geometric path remains available
- **GIVEN** a path is successfully converted with interpolation or bounded curve fitting
- **WHEN** the generated plan is inspected
- **THEN** its source geometric path MUST remain unchanged
- **AND** its timed trajectory MUST be stored as a distinct representation within the generated plan

### Requirement: Timed trajectories satisfy canonical invariants

The manipulation stack MUST reject timed trajectories that are malformed, non-finite, incorrectly ordered, or inconsistent with the selected joints and motion limits.

#### Scenario: Valid trajectory is accepted
- **GIVEN** a backend returns a trajectory with the expected global joint ordering
- **WHEN** all samples have finite positions and velocities, strictly increasing finite times, preserved endpoints, and motion within applicable limits
- **THEN** the trajectory SHALL be accepted for preview and execution

#### Scenario: Malformed trajectory is rejected
- **GIVEN** a backend returns missing samples, inconsistent dimensions, duplicate or decreasing times, non-finite values, unexpected joint names, or a mismatched endpoint
- **WHEN** the result is validated
- **THEN** plan materialization MUST fail with a diagnostic identifying the violated invariant
- **AND** the invalid trajectory MUST NOT reach execution

#### Scenario: Motion limits are exceeded
- **GIVEN** a backend returns motion exceeding an applicable joint velocity or acceleration limit beyond numerical tolerance
- **WHEN** the result is validated
- **THEN** plan materialization MUST fail
- **AND** the generated motion MUST NOT be exposed as executable

### Requirement: RoboPlan TOPP-RA uses authoritative URDF limits

The RoboPlan TOPP-RA backend SHALL use the RoboPlan scene's joint velocity and acceleration limits sourced from the robot URDF.

#### Scenario: Required URDF limits are present
- **GIVEN** every selected joint has usable velocity and acceleration limits in the URDF-backed RoboPlan scene
- **WHEN** RoboPlan TOPP-RA parametrizes a path
- **THEN** it MUST constrain the trajectory using those limits and the configured reduction scales

#### Scenario: A required URDF limit is missing
- **GIVEN** at least one selected joint lacks a usable URDF velocity or acceleration limit
- **WHEN** RoboPlan TOPP-RA is initialized for or applied to that planning group
- **THEN** the operation MUST fail with a diagnostic naming the missing limit and affected joint
- **AND** the system MUST NOT substitute DimOS's generic motion-limit defaults

### Requirement: Backend failures do not trigger cross-backend fallback

The manipulation stack MUST NOT silently switch to another trajectory parametrization backend when the startup-selected backend fails.

#### Scenario: Selected backend rejects a path
- **GIVEN** exactly one parametrization backend was selected at startup
- **WHEN** that backend rejects or fails to parametrize a path
- **THEN** plan materialization MUST fail using that backend's diagnostic
- **AND** no other parametrization backend may be invoked for the path

#### Scenario: RoboPlan uses an internal safety fitting mode
- **GIVEN** RoboPlan TOPP-RA remains the selected backend
- **WHEN** RoboPlan applies its documented internal safety behavior between curve-fitting modes
- **THEN** the result MAY be accepted if the backend reports a valid trajectory
- **AND** this MUST NOT be treated as switching to a different parametrization backend

### Requirement: Existing manipulation control surfaces remain compatible

Trajectory parametrization SHALL integrate without changing the public plan, preview, execute, skill, MCP, or stream signatures.

#### Scenario: Existing preview and execution flow
- **GIVEN** a generated plan was successfully materialized from either an untimed path or a planner-native timed trajectory
- **WHEN** a caller invokes the existing preview or execute surface
- **THEN** the caller MUST use the same public operation and argument shape as before
- **AND** execution MAY project the accepted stored trajectory into robot-local joint order
- **AND** the accepted timestamps and velocities MUST be previewed or dispatched without regeneration or retiming

### Requirement: Viser controls the speed of future plans

The manipulation Viser panel SHALL expose a bounded runtime speed scale for
plans generated after the setting changes.

#### Scenario: Operator reduces next-plan speed
- **GIVEN** the Viser panel is idle and displays a fresh accepted plan
- **WHEN** the operator moves `Next plan speed` to a value in `(0, 1]`
- **THEN** the module MUST retain that value for future planning
- **AND** the existing accepted plan MUST remain unchanged and executable

#### Scenario: A new untimed path is planned
- **GIVEN** a runtime next-plan speed below `1.0`
- **WHEN** an untimed geometric path is materialized
- **THEN** the selected parametrizer MUST multiply its configured velocity and acceleration reduction scales by the runtime scale

#### Scenario: A new Cartesian path is planned from Viser
- **GIVEN** a runtime next-plan speed below `1.0`
- **WHEN** Viser requests planner-native Cartesian planning
- **THEN** the request MUST carry that velocity and acceleration scale
- **AND** the returned planner-native timing MUST still bypass path parametrization

#### Scenario: Speed changes during an active panel operation
- **GIVEN** the Viser panel is planning, previewing, executing, cancelling, or clearing
- **WHEN** the speed control is rendered
- **THEN** it MUST be disabled until the operation becomes idle
