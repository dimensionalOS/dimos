## ADDED Requirements

### Requirement: Connection-owned device boundary

Each connection module SHALL own its device or simulator connection, native lifecycle, ordering and unit conversion, hard-limit enforcement, command watchdog, and safe stop. The coordinator SHALL NOT construct, connect, poll, or invoke device adapters.

#### Scenario: Start a hardware connection
- **GIVEN** a connection instance configured for a physical device
- **WHEN** the module starts
- **THEN** that module establishes and monitors the native connection
- **AND** the coordinator interacts only through declared typed streams and robot-level lifecycle operations

#### Scenario: Use a private adapter helper
- **GIVEN** a connection package uses an adapter internally to isolate an SDK
- **WHEN** coordinator and manipulation modules are configured
- **THEN** neither receives an adapter object, registry key, nor native handle

### Requirement: Resolved immutable connection description

Each connection SHALL publish a description after all instance configuration is resolved and before it may be armed. The description SHALL identify its source, interfaces, units, limits, profile, rates, timeouts, omission rules, and safe-stop policy, and SHALL remain immutable for that running epoch.

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

#### Scenario: Duplicate command ownership
- **GIVEN** two descriptions claiming the same command interface
- **WHEN** the coordinator validates readiness
- **THEN** readiness fails with both sources and the conflicting key
- **AND** neither connection is armed

### Requirement: Per-instance configuration ownership

Device addresses, bus identifiers, credentials, native options, command profiles, and safety settings SHALL be configured on each connection instance rather than in hardware-specific global coordinator configuration.

#### Scenario: Override one identical device
- **GIVEN** `left` and `right` instances of one connection class
- **WHEN** a user supplies an override for `left`
- **THEN** only `left` receives that setting
- **AND** both descriptions reflect independently resolved configuration

### Requirement: Static compatible command profile

A connection SHALL advertise one configured command profile containing mutually compatible fields and SHALL reject commands outside that profile. Changing to an incompatible profile while active is unsupported.

#### Scenario: Position profile receives velocity
- **GIVEN** an armed connection declaring position commands only
- **WHEN** it receives a velocity command key
- **THEN** it rejects the complete batch
- **AND** no value from that batch reaches the device

#### Scenario: Whole-body impedance profile
- **GIVEN** a profile declaring position, velocity, proportional gain, derivative gain, and effort
- **WHEN** it receives a valid batch
- **THEN** it dispatches the fields in native order
- **AND** public commands and state remain in canonical SI units

### Requirement: Atomic command-batch validation

A connection SHALL validate target, epoch, sequence, lengths, duplicate keys, supported keys, finite values, and hard limits before applying any value in a command batch.

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
