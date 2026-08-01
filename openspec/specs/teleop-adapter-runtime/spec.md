## Purpose

Define the generic teleoperation adapter runtime contract and coordinator-facing publishing behavior shared by device-specific teleop bridges.

## Requirements

### Requirement: Teleop adapter contract
The system SHALL provide a teleop adapter contract for device-specific bridges that connect to a human input source, disconnect from it, and report the current teleoperation command without publishing directly to DimOS streams.

#### Scenario: Module retrieves command from adapter
- **WHEN** a teleop adapter is connected and the teleop module control tick runs
- **THEN** the teleop module calls the adapter's current-command method and remains responsible for publishing any returned command

#### Scenario: Adapter has no authority
- **WHEN** the adapter returns no command for the current tick
- **THEN** the teleop module publishes no motion command for that tick

### Requirement: Teleop command envelope
The system SHALL represent adapter output with a command envelope that distinguishes an active coordinator-facing command from no command and from an explicit stop command.

#### Scenario: Active command is returned
- **WHEN** an adapter returns a command envelope containing a `JointState`, `PoseStamped`, or `Twist` with no stop flag
- **THEN** the teleop module treats the envelope as an active motion command

#### Scenario: Explicit stop command is returned
- **WHEN** an adapter returns a command envelope with the stop flag set
- **THEN** the teleop module treats the envelope as an explicit stop rather than as missing authority

### Requirement: Single primary motion output
The system SHALL require each teleop adapter instance to declare exactly one primary motion output type among joint, cartesian, and twist commands.

#### Scenario: Adapter declares one primary output
- **WHEN** a teleop adapter declares one primary motion output type
- **THEN** the teleop module publishes active commands only to the matching coordinator-facing output stream

#### Scenario: Adapter attempts conflicting primary outputs
- **WHEN** a teleop adapter configuration attempts to use more than one primary motion output type
- **THEN** the teleop module rejects the adapter configuration before publishing motion commands

### Requirement: Coordinator-facing stream publishing
The system SHALL expose coordinator-facing teleop outputs compatible with existing `ControlCoordinator` inputs without requiring changes to `ControlCoordinator` for v1.

#### Scenario: Joint command adapter emits command
- **WHEN** a joint-primary adapter returns an active `JointState` command envelope
- **THEN** the teleop module publishes the `JointState` on `joint_command`

#### Scenario: Cartesian command adapter emits command
- **WHEN** a cartesian-primary adapter returns an active `PoseStamped` command envelope
- **THEN** the teleop module publishes the `PoseStamped` on `coordinator_cartesian_command`

#### Scenario: Twist command adapter emits command
- **WHEN** a twist-primary adapter returns an active `Twist` command envelope
- **THEN** the teleop module publishes the `Twist` on `twist_command`

### Requirement: Generic structural teleop safety
The teleop module SHALL enforce structural safety checks that do not require device-specific or robot-specific interpretation.

#### Scenario: Command stream becomes stale
- **WHEN** the adapter has not produced a valid command within the configured stale-command timeout
- **THEN** the teleop module stops publishing active motion commands until a valid command is available again

#### Scenario: Publish rate is configured
- **WHEN** the teleop module runs with a configured maximum publish rate
- **THEN** the teleop module does not publish active motion commands faster than that rate

#### Scenario: Module stops
- **WHEN** the teleop module is stopped
- **THEN** it disconnects the adapter and performs any configured structural stop handling

### Requirement: Existing Quest teleop remains unchanged in v1
The system SHALL keep existing Quest teleop modules and Quest teleop blueprints functionally unchanged while adding the v1 teleop adapter runtime.

#### Scenario: Existing Quest blueprint is run
- **WHEN** an existing Quest teleop blueprint is used after this change
- **THEN** it continues to use its existing Quest module implementation rather than the new generic teleop module
