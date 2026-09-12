## ADDED Requirements

### Requirement: Connection-owned device boundary

Each connection module SHALL own its device or simulator connection, native lifecycle, ordering and unit conversion, hard-limit enforcement, command watchdog, and safe stop. The coordinator SHALL NOT construct, connect, poll, or invoke device adapters.

#### Scenario: Start a hardware connection
- **GIVEN** a connection instance configured for a physical device
- **WHEN** the module starts
- **THEN** that module establishes and monitors the native connection
- **AND** reports universal module readiness only while it can safely accept responsibility for the device
- **AND** the coordinator interacts only through declared typed streams and robot-level lifecycle operations

#### Scenario: Use a private adapter helper
- **GIVEN** a connection package uses an adapter internally to isolate an SDK
- **WHEN** coordinator and manipulation modules are configured
- **THEN** neither receives an adapter object, registry key, nor native handle

### Requirement: Resolved immutable connection description

Each connection SHALL publish a description after all instance configuration is resolved and before it may be armed. The description SHALL identify its source, joint resources, interfaces, units, limits, supported interface combinations, rates, timeouts, omission rules, safe-stop policy, and verified process-loss classification, and SHALL remain immutable for that running epoch.

#### Scenario: Announce a configured connection
- **GIVEN** a connection whose instance overrides are resolved
- **WHEN** it initializes its device-facing contract
- **THEN** it publishes the final canonical interfaces and policies
- **AND** the coordinator validates them without inspecting blueprint constructor arguments

#### Scenario: Description changes during a run
- **GIVEN** the coordinator accepted a description
- **WHEN** that source announces a different interface set or policy in the same epoch
- **THEN** the coordinator faults the robot and prevents continued commands
- **AND** explicit reinitialization and re-arming are required

### Requirement: Multiple instances of one connection class

The system SHALL support multiple instances of the same connection implementation through distinct module/source names and disjoint canonical interface namespaces.

#### Scenario: Configure two xArm instances
- **GIVEN** xArm connection modules named `left` and `right`
- **WHEN** they publish on the shared control streams
- **THEN** the coordinator accepts `left/...` and `right/...` interfaces
- **AND** no hardware ID or generated coordinator type is required

#### Scenario: Duplicate joint ownership
- **GIVEN** two descriptions claiming command ownership of the same canonical joint
- **WHEN** the coordinator validates readiness
- **THEN** readiness fails with both sources and the conflicting joint
- **AND** neither connection is armed

### Requirement: Per-instance configuration ownership

Device addresses, bus identifiers, credentials, native options, command-interface behavior, and safety settings SHALL be configured on each connection instance rather than in hardware-specific global coordinator configuration.

#### Scenario: Override one identical device
- **GIVEN** `left` and `right` instances of one connection class
- **WHEN** a user supplies an override for `left`
- **THEN** only `left` receives that setting
- **AND** both descriptions reflect independently resolved configuration

### Requirement: Connection-validated command-interface combinations

A connection SHALL advertise its available command interfaces and autonomously perform any native transition required by a change in winning command semantics. It SHALL accept only combinations supported by the device and SHALL keep native mode identifiers private.

#### Scenario: Switch an arm from position to velocity
- **GIVEN** position interfaces are active and the connection supports velocity control
- **WHEN** it receives fresh winning velocity commands
- **THEN** it changes native device behavior without restarting
- **AND** applies velocity values only after the native transition succeeds

#### Scenario: Transition is in progress
- **GIVEN** a connection is changing its native behavior for a new command interface
- **WHEN** command frames continue to arrive
- **THEN** it remains module-ready, retains the newest compatible command, and never interprets new values through the old native mode
- **AND** it applies only a fresh compatible command after success without exposing transition state to the coordinator

#### Scenario: Native transition fails
- **GIVEN** a winning command requires a supported interface transition
- **WHEN** the native device rejects or fails that transition
- **THEN** the connection enters its safe state and withdraws module readiness with the fault reason
- **AND** the coordinator handles the fault through its normal required-connection safety policy

#### Scenario: Whole-body impedance combination
- **GIVEN** the connection supports position, velocity, proportional gain, derivative gain, and effort together
- **WHEN** one winning task activates that combination for its joints
- **THEN** the connection accepts and dispatches those fields in native order
- **AND** no other task supplies a field for those owned joints

### Requirement: Atomic command-batch validation

A connection SHALL validate armed lifecycle state, current control epoch, target, sequence, lengths, duplicate keys, supported keys, finite values, and hard limits before applying any value in a command batch.

#### Scenario: Command arrives while safe-stopped
- **GIVEN** a connection latched `SAFE_STOPPED` and invalidated its control epoch
- **WHEN** any ordinary command frame arrives
- **THEN** the connection rejects it before invoking the driver
- **AND** only explicit recovery followed by a new arm transaction can reopen its command gate

#### Scenario: One key is unknown
- **GIVEN** a batch with valid keys and one undeclared key
- **WHEN** the owning connection receives it
- **THEN** it rejects the entire batch with a diagnostic
- **AND** applies none of its values

#### Scenario: Command exceeds a hard limit
- **GIVEN** a canonical command outside a declared hard limit
- **WHEN** the connection validates it
- **THEN** it refuses the command before invoking the driver
- **AND** reports the fault according to policy

### Requirement: Connections own physical safe behavior

Each connection SHALL implement `SAFE_STOP` and `ESTOP` according to the mechanics and native controller of its device. It SHALL latch lifecycle state before device action and SHALL acknowledge repeated operation identifiers idempotently. It SHALL NOT treat command silence or one generic zero vector as a substitute for these operations. Physical deactivation SHALL remain part of the connection's device-specific `Module.stop()` teardown rather than the generic runtime control protocol.

#### Scenario: Connection detects its own fault
- **GIVEN** an armed connection detects an SDK failure, device-link loss, unsafe command, or command-watchdog timeout
- **WHEN** the failure occurs
- **THEN** it latches and executes its safe-stop policy without waiting for the coordinator
- **AND** publishes status so the coordinator safe-stops the rest of the robot

#### Scenario: Healthy peer receives robot-wide safe stop
- **GIVEN** another required connection caused the robot fault
- **WHEN** this healthy connection receives `SAFE_STOP`
- **THEN** it enters `SAFE_STOPPED` while retaining module readiness
- **AND** rejects ordinary commands until explicit recovery and re-arming

### Requirement: Rich device streams remain independently routable

A connection MAY expose typed sensor streams beyond scalar control. Consumers SHALL connect only the streams they need, and the coordinator SHALL NOT require unrelated high-bandwidth data.

#### Scenario: Camera and joint data share a connection module
- **GIVEN** one module owns camera and joint interfaces
- **WHEN** a blueprint connects control and perception
- **THEN** the coordinator receives only control streams
- **AND** perception receives typed images without coordinator routing

#### Scenario: IMU feeds control and localization
- **GIVEN** a full typed IMU stream and a controller needing selected IMU scalars
- **WHEN** the blueprint runs
- **THEN** localization receives the full IMU
- **AND** only declared scalar values enter coordinator state
