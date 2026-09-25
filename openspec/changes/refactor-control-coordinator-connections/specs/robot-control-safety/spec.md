## ADDED Requirements

### Requirement: Readiness is universal and live

The module framework SHALL expose live readiness separately from worker-process liveness. A module SHALL become ready only after successful initialization and SHALL be able to withdraw readiness after startup with a diagnostic reason. The control coordinator SHALL report the robot ready only after every required connection module is ready and supplies a valid immutable description and a complete fresh state snapshot.

#### Scenario: All sources become ready
- **GIVEN** a coordinator requiring `left` and `right`
- **WHEN** both modules report ready and provide valid description and initial state
- **THEN** the coordinator reports `STANDBY`

#### Scenario: A source is missing
- **GIVEN** two required sources
- **WHEN** only one initializes before timeout
- **THEN** readiness identifies the missing source
- **AND** arm requests are refused

#### Scenario: A ready connection becomes unavailable
- **GIVEN** an armed robot whose required connections are ready
- **WHEN** one connection withdraws module readiness with a reason
- **THEN** the coordinator performs robot-wide safe stop and reports that source and reason
- **AND** restored readiness does not re-arm the robot or replay old task output

### Requirement: Transactional robot arming

Arming SHALL be a two-phase robot-wide transaction. `PREPARE_ARM` SHALL let every required connection establish its device-specific operating preconditions while the coordinator blocks ordinary task commands. `COMMIT_ARM` SHALL activate the prepared connections with one new control epoch, and the coordinator SHALL report `ARMED` and open its command gate only after every source acknowledges that commit successfully. A connection description SHALL declare `DIRECT` or `OPERATOR_CONFIRMED` activation. If any required connection requires confirmation, the coordinator SHALL remain `PREPARED` until a caller confirms the matching operation ID.

#### Scenario: Direct robot arms
- **GIVEN** a robot in `STANDBY` whose required connections all declare `DIRECT`
- **WHEN** a caller requests arm
- **THEN** one identified operation prepares every required source and commits one new control epoch
- **AND** `ARMED` is reported only after every commit succeeds

#### Scenario: Operator-confirmed robot prepares
- **GIVEN** a robot in `STANDBY` with a required connection declaring `OPERATOR_CONFIRMED`
- **WHEN** a caller begins arming and every connection finishes preparation
- **THEN** the coordinator reports `PREPARED` with the operation ID
- **AND** ordinary task commands remain blocked
- **AND** no control epoch is active

#### Scenario: Operator confirms preparation
- **GIVEN** every required connection is `PREPARED` for one operation ID
- **WHEN** a caller confirms that operation ID
- **THEN** the coordinator commits every required connection with one new control epoch
- **AND** opens its command gate only after every connection acknowledges `ARMED`

#### Scenario: Caller aborts preparation
- **GIVEN** a robot is `PREPARING` or `PREPARED` and no commit has begun
- **WHEN** a caller aborts the matching operation ID
- **THEN** every connection returns to its declared stable standby behavior
- **AND** the coordinator reports `STANDBY` only after every acknowledgement

#### Scenario: One connection rejects preparation or commit
- **GIVEN** physical preparation has started or some connections acknowledged commit
- **WHEN** another required connection rejects, times out, becomes stale, or loses readiness
- **THEN** the coordinator safe-stops every required connection and invalidates the proposed control epoch
- **AND** reports the latched failure with the failed source

#### Scenario: Stale confirmation arrives
- **GIVEN** an earlier preparation was aborted or safe-stopped
- **WHEN** a caller confirms its old operation ID
- **THEN** the coordinator rejects the confirmation
- **AND** the robot remains in its current safe state

### Requirement: Activation preparation is connection-owned

Only connection-owned, bounded activation behavior SHALL be permitted while the robot is `PREPARING`. Ordinary task commands SHALL remain blocked in `STANDBY`, `PREPARING`, `PREPARED`, and `COMMITTING`. A prepared connection SHALL maintain its declared stable behavior until commit, abort, safe stop, or emergency stop. Task or policy computation MAY run in shadow while prepared, but its output SHALL NOT reach hardware.

#### Scenario: G1 performs supervised preparation
- **GIVEN** a G1 connection requiring operator confirmation
- **WHEN** it receives `PREPARE_ARM`
- **THEN** the connection may ramp from measured joints toward its configured operating pose
- **AND** reports `PREPARED` only after it can maintain the declared stable prepared behavior
- **AND** GR00T task output remains blocked from hardware until commit

#### Scenario: Ready-pose trajectory runs after arming
- **GIVEN** G1 completed its arm commit and the coordinator reports `ARMED`
- **WHEN** manipulation requests the optional bimanual ready pose
- **THEN** it executes as an ordinary arbitrated trajectory
- **AND** it does not add or bypass a lifecycle state

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

### Requirement: Safe stop is explicit, latched, and connection-owned

The coordinator SHALL close its command gate and request `SAFE_STOP` from every required connection when the robot faults. Each connection SHALL latch `SAFE_STOPPED` before executing its device-specific stable behavior. Every automatic or requested safe stop SHALL require explicit `CLEAR_SAFE_STOP` followed by a new arm transaction. Safe stop SHALL NOT imply zero commands, command silence, physical deactivation, motor disable, or emergency stop.

#### Scenario: One required source faults
- **GIVEN** an armed robot with several required connections
- **WHEN** one source becomes stale, not ready, or faulted
- **THEN** the coordinator prevents further ordinary command publication before requesting robot-wide safe stop
- **AND** every connection independently executes its declared safe-stop policy

#### Scenario: A gravity-loaded arm safe-stops
- **GIVEN** disabling the arm would let it fall
- **WHEN** the connection receives `SAFE_STOP`
- **THEN** it retains the power and control needed for its declared stable behavior
- **AND** it does not interpret safe stop as physical deactivation

#### Scenario: User requests safe stop without a fault
- **GIVEN** an armed robot with no current fault
- **WHEN** a caller requests robot-level safe stop
- **THEN** every required connection latches `SAFE_STOPPED`
- **AND** later commands remain rejected until explicit clear and re-arm

### Requirement: Task cancellation is not a safety transition

Cancelling a task SHALL release that task's claims and output without changing the robot lifecycle state or clearing any safety latch. While the robot remains armed, fresh heartbeats SHALL cause each connection to apply its declared omission or armed-idle policy. A caller SHALL use robot-level safe stop when continued armed-idle behavior is insufficient.

#### Scenario: Cancel an active trajectory
- **GIVEN** a trajectory task owns arm joints on an armed robot
- **WHEN** the trajectory is cancelled
- **THEN** the task releases its claims and the coordinator continues current-epoch heartbeats
- **AND** the connection applies its declared omission or armed-idle behavior without entering `SAFE_STOPPED`

#### Scenario: Cancel while robot is safe-stopped
- **GIVEN** the robot already latched `SAFE_STOPPED`
- **WHEN** a task is cancelled
- **THEN** the safety latch remains set
- **AND** cancellation cannot substitute for `CLEAR_SAFE_STOP` and re-arm

### Requirement: Safe-stop delivery is retryable and watchdog-backed

The coordinator SHALL repeat a safe-stop operation with one operation identifier until every target acknowledges or the operation times out. It SHALL stop ordinary command heartbeats after closing the gate, so a connection that misses the operation invokes the same safe policy through its watchdog.

#### Scenario: A safe-stop message is lost
- **GIVEN** the coordinator has closed its command gate
- **WHEN** one connection misses the first `SAFE_STOP` message
- **THEN** repeated messages may satisfy the same idempotent operation
- **AND** absent command heartbeats still trigger the connection's local safe stop

#### Scenario: A connection does not acknowledge
- **GIVEN** a robot-wide safe-stop operation is in progress
- **WHEN** one required connection does not acknowledge before timeout
- **THEN** the coordinator remains faulted and reports that source as unknown
- **AND** no command gate reopens

### Requirement: Control epochs prevent stale command resumption

A successful arm transaction SHALL establish a new control epoch. Connections SHALL accept commands only while armed and only when the frame carries the current epoch and a newer sequence. Safe stop and emergency stop SHALL invalidate the current epoch before device action.

#### Scenario: A command arrives after safe stop
- **GIVEN** epoch 15 was invalidated by safe stop
- **WHEN** a delayed command from epoch 15 arrives
- **THEN** the connection rejects it
- **AND** the safe-stop latch remains authoritative

#### Scenario: Robot is re-armed
- **GIVEN** the fault and safe-stop latch were explicitly cleared
- **WHEN** every required connection arms successfully for epoch 16
- **THEN** only epoch-16 commands may be accepted
- **AND** no epoch-15 task output is replayed

### Requirement: Physical safety survives connection-process failure where claimed

Each connection SHALL declare exactly one process-loss classification: verified native watchdog, verified external supervisor, verified intrinsically safe behavior, simulation, or unprotected. A production arm transaction SHALL reject every physical connection that is unprotected, classified as simulation, missing a classification, or lacks verification evidence.

#### Scenario: Connection process dies
- **GIVEN** a physical platform declares native process-loss protection
- **WHEN** its connection process stops sending native commands
- **THEN** the native controller or external supervisor invokes the declared safe behavior
- **AND** the protection does not depend on code in the dead process

#### Scenario: No lower-level watchdog exists
- **GIVEN** a platform cannot safe-stop after connection-process loss
- **WHEN** its production arming policy is evaluated
- **THEN** the connection is classified as unprotected
- **AND** the production arm transaction is refused

#### Scenario: Simulation declares process-loss behavior
- **GIVEN** a connection is classified as simulation
- **WHEN** a physical production blueprint attempts to use it as a required connection
- **THEN** production arming is refused
- **AND** simulation status cannot satisfy a physical safety claim

#### Scenario: Device is intrinsically safe on command loss
- **GIVEN** a physical connection declares intrinsically safe command-loss behavior
- **WHEN** platform validation verifies that behavior for its actual command path
- **THEN** production arming may proceed
- **AND** the evidence is associated with that connection implementation and configuration

### Requirement: Required-state staleness causes robot-wide fault

If any required source lacks complete valid state within its stale timeout, the coordinator SHALL fault and stop the logical robot as a whole.

#### Scenario: One arm becomes stale
- **GIVEN** an armed two-arm robot
- **WHEN** the right source exceeds its timeout
- **THEN** the coordinator initiates robot-wide safe stop
- **AND** reports the right source as cause

#### Scenario: Aggregated state is no longer complete and fresh
- **GIVEN** the coordinator retained the last valid snapshot for diagnostics
- **WHEN** any required source exceeds its stale timeout
- **THEN** normal aggregated `JointState` publication stops
- **AND** downstream consumers are not shown mixed fresh and cached values as live state

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
- **THEN** the robot enters `STANDBY`
- **AND** requires a new arm and commands before motion

### Requirement: Fault recovery never resumes motion automatically

Recovery of transport, freshness, or connection status SHALL NOT re-arm the robot, restore a prior winner, or resume a trajectory automatically.

#### Scenario: Stale connection reconnects
- **GIVEN** staleness faulted an armed robot
- **WHEN** the source reconnects with valid description, status, and state
- **THEN** the robot may return to `STANDBY` after clearance
- **AND** old task output is not replayed

### Requirement: Safe-stop clearance is robot-wide and transactional

The coordinator SHALL expose safe-stop clearance only as a robot-level transaction targeting every required connection. It SHALL report `STANDBY` only after every target acknowledges the same `CLEAR_SAFE_STOP` operation. A rejection or timeout SHALL keep the command gate closed and restore or confirm `SAFE_STOPPED` for every required connection.

#### Scenario: Every connection clears safe stop
- **GIVEN** all fault causes are cleared and every required connection is ready with fresh state
- **WHEN** all targets acknowledge one `CLEAR_SAFE_STOP` operation
- **THEN** the coordinator reports `STANDBY`
- **AND** a separate arm transaction is still required

#### Scenario: One connection rejects clearance
- **GIVEN** some connections acknowledged `CLEAR_SAFE_STOP`
- **WHEN** another required connection rejects or times out
- **THEN** the coordinator keeps its command gate closed
- **AND** restores or confirms `SAFE_STOPPED` on every required connection

#### Scenario: Caller attempts to clear one connection
- **GIVEN** the logical robot is safe-stopped
- **WHEN** a normal public caller requests recovery
- **THEN** the operation targets the complete required connection set
- **AND** no public coordinator API permits partial clearance

### Requirement: Physical deactivation is connection-specific shutdown

The generic runtime control protocol SHALL NOT expose disarm or actuator-disable operations. `Module.stop()` SHALL delegate teardown to each connection, which SHALL establish its device-specific stable state before parking, disabling actuators, or disconnecting as appropriate. Domain-specific motor-disarm APIs MAY remain outside this joint-control protocol.

#### Scenario: Stop a gravity-loaded connection module
- **GIVEN** disabling the device before support or parking would make it fall
- **WHEN** its module shutdown begins
- **THEN** the connection first executes its declared stable shutdown sequence
- **AND** disables actuators only when its device-specific preconditions permit it

### Requirement: Lifecycle acknowledgements are correlated

Lifecycle controls and statuses SHALL carry an operation identifier so delayed or duplicate acknowledgements cannot satisfy a different transaction.

#### Scenario: Delayed arm acknowledgement arrives
- **GIVEN** an arm timed out and rollback began
- **WHEN** its delayed acknowledgement arrives
- **THEN** it is not counted toward a newer operation
- **AND** current safe state remains authoritative

#### Scenario: Safe stop is retried
- **GIVEN** a connection already handled safe-stop operation 72
- **WHEN** it receives operation 72 again
- **THEN** it returns the same authoritative acknowledgement idempotently
- **AND** does not leave `SAFE_STOPPED`

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
- **AND** every physical command gate remains closed
