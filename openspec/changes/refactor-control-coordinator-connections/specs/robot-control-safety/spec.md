## ADDED Requirements

### Requirement: Readiness requires every configured connection

The coordinator SHALL report ready only after every required source supplies a valid immutable description, a complete fresh state snapshot, and non-faulted status.

#### Scenario: All sources become ready
- **GIVEN** a coordinator requiring `left` and `right`
- **WHEN** both provide valid description, status, and initial state
- **THEN** the coordinator reports ready but not armed

#### Scenario: A source is missing
- **GIVEN** two required sources
- **WHEN** only one initializes before timeout
- **THEN** readiness identifies the missing source
- **AND** arm requests are refused

### Requirement: Transactional robot arming

Arming SHALL activate every required connection as one transaction. The coordinator SHALL report armed only after every source acknowledges the same operation successfully.

#### Scenario: Every connection arms
- **GIVEN** a ready disarmed robot
- **WHEN** a caller requests arm
- **THEN** one identified operation targets all required sources
- **AND** armed is reported only after every success

#### Scenario: One connection rejects arm
- **GIVEN** some connections acknowledged arm
- **WHEN** another rejects or times out
- **THEN** the coordinator rolls back every activated connection
- **AND** reports unarmed with the failed source

### Requirement: Independent command watchdog

Every connection SHALL enforce a command watchdog independently of coordinator and task timeouts. Missing heartbeat frames beyond the declared timeout SHALL invoke safe stop.

#### Scenario: Coordinator stops publishing
- **GIVEN** an armed connection receiving heartbeats
- **WHEN** none arrives before its deadline
- **THEN** the connection safe-stops without an RPC
- **AND** publishes fault status

#### Scenario: Empty heartbeats continue
- **GIVEN** no active task winner
- **WHEN** fresh empty command frames continue
- **THEN** the connection applies omission policy
- **AND** does not classify the coordinator as dead

### Requirement: Required-state staleness causes robot-wide fault

If any required source lacks complete valid state within its stale timeout, the coordinator SHALL fault and stop the logical robot as a whole.

#### Scenario: One arm becomes stale
- **GIVEN** an armed two-arm robot
- **WHEN** the right source exceeds its timeout
- **THEN** the coordinator initiates robot-wide safe stop or disarm
- **AND** reports the right source as cause

#### Scenario: Invalid frames keep arriving
- **GIVEN** malformed snapshots continue arriving
- **WHEN** the last valid snapshot becomes stale
- **THEN** the robot faults
- **AND** malformed traffic does not preserve freshness

### Requirement: Explicit omission behavior

Every command interface SHALL advertise an omission policy. Unless stricter, omitted position retains its last accepted target, velocity and effort become zero, and gains retain configured or last accepted safe values.

#### Scenario: Velocity task releases ownership
- **GIVEN** an active velocity winner
- **WHEN** a later heartbeat omits it
- **THEN** the connection commands zero velocity

#### Scenario: Position task releases ownership
- **GIVEN** an accepted position target
- **WHEN** a later heartbeat omits it
- **THEN** the connection retains it unless a stricter policy is declared

### Requirement: Emergency stop is latched and robot-wide

Emergency stop SHALL stop every required connection and latch the logical robot. Clearing it SHALL NOT arm or resume commands.

#### Scenario: User invokes emergency stop
- **GIVEN** an armed robot
- **WHEN** a caller invokes robot-level emergency stop
- **THEN** the operation reaches every required source
- **AND** none remains command-active

#### Scenario: Clear emergency stop
- **GIVEN** all sources acknowledged emergency stop
- **WHEN** the condition is cleared successfully
- **THEN** the robot becomes disarmed
- **AND** requires a new arm and commands before motion

### Requirement: Fault recovery never resumes motion automatically

Recovery of transport, freshness, or connection status SHALL NOT re-arm the robot, restore a prior winner, or resume a trajectory automatically.

#### Scenario: Stale connection reconnects
- **GIVEN** staleness faulted an armed robot
- **WHEN** the source reconnects with valid description, status, and state
- **THEN** the robot may return to ready and disarmed after clearance
- **AND** old task output is not replayed

### Requirement: Lifecycle acknowledgements are correlated

Lifecycle controls and statuses SHALL carry an operation identifier so delayed or duplicate acknowledgements cannot satisfy a different transaction.

#### Scenario: Delayed arm acknowledgement arrives
- **GIVEN** an arm timed out and rollback began
- **WHEN** its delayed acknowledgement arrives
- **THEN** it is not counted toward a newer operation
- **AND** current safe state remains authoritative

### Requirement: Replay cannot command physical hardware

Replay SHALL preserve observable state for consumers but SHALL NOT arm or command physical connections.

#### Scenario: Run replay
- **GIVEN** recorded control state and status
- **WHEN** replay publishes them
- **THEN** consumers observe recorded robot state
- **AND** no physical connection receives control operations

#### Scenario: Mix replay and hardware unintentionally
- **GIVEN** replay state and a physical command connection are combined without explicit support
- **WHEN** the blueprint validates or starts
- **THEN** it fails closed
- **AND** hardware remains disarmed
