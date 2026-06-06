## ADDED Requirements

### Requirement: DMMotor adapter selection

DimOS SHALL provide an opt-in DMMotor manipulator adapter path for DMMotor/Damiao arms that uses the `can_motor_control` Python binding as its hardware API surface.

#### Scenario: Selecting the DMMotor adapter
- **GIVEN** a manipulator hardware configuration selects the DMMotor adapter type
- **AND** the `can_motor_control` Python binding is available in the runtime environment
- **WHEN** the hardware is initialized
- **THEN** DimOS SHALL create the DMMotor adapter through the manipulator adapter registry
- **AND** the adapter SHALL use the Python binding rather than direct Rust crate calls.

#### Scenario: Binding unavailable
- **GIVEN** a manipulator hardware configuration selects the DMMotor adapter type
- **AND** the `can_motor_control` Python binding is not importable
- **WHEN** the hardware is initialized
- **THEN** DimOS SHALL fail with an explicit error indicating that the Python binding is unavailable
- **AND** DimOS SHALL indicate that the binding is provided by the `dimos[manipulation]` extra on supported platforms.

### Requirement: DMMotor adapter lifecycle safety

The DMMotor adapter SHALL expose safe lifecycle behavior for connection, enablement, state reads, command writes, and shutdown.

#### Scenario: Clean lifecycle
- **GIVEN** a configured DMMotor arm with available hardware or mock transport
- **WHEN** DimOS connects, enables, reads state, writes commands, and stops the adapter
- **THEN** the adapter SHALL perform those lifecycle steps through the Python binding
- **AND** the adapter SHALL disable the motors during stop or disconnect.

#### Scenario: Interrupted operation
- **GIVEN** a DMMotor arm is enabled through DimOS
- **WHEN** the owning module or blueprint is stopped or interrupted
- **THEN** DimOS SHALL attempt to disable the motors before releasing the binding object
- **AND** shutdown failures SHALL be surfaced or logged without masking the need for safe operator intervention.

### Requirement: DMMotor state and command compatibility

The DMMotor adapter SHALL present manipulator state and position/effort command behavior compatible with DimOS manipulator control surfaces.

#### Scenario: Reading coherent joint state
- **GIVEN** DimOS requests joint positions, velocities, and efforts for a DMMotor arm during one control cycle
- **WHEN** the adapter reads from the Python binding
- **THEN** the returned values SHALL represent one coherent state snapshot in DimOS joint order
- **AND** repeated per-field reads SHALL NOT independently advance the hardware loop multiple times for the same cycle.

#### Scenario: Commanding joints
- **GIVEN** DimOS sends a supported position or effort command to a connected DMMotor arm
- **WHEN** the adapter forwards the command through the Python binding
- **THEN** the command SHALL be applied using SI units expected by DimOS
- **AND** the adapter SHALL preserve joint ordering between the DimOS hardware configuration and the binding arm group.

#### Scenario: Rejecting velocity commands
- **GIVEN** DimOS sends a velocity command to a connected DMMotor arm
- **WHEN** the adapter evaluates the command
- **THEN** the adapter SHALL reject the command rather than emulate velocity with nonzero MIT position gains.
