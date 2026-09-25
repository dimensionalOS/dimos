## ADDED Requirements

### Requirement: Named scalar control-value contract

Control-state and command frames SHALL carry a source, producer timestamp, producer/control epoch, per-source sequence, parallel interface-name and `float64` value arrays, and fully qualified canonical keys. Every frame SHALL identify each value by the name in the same frame rather than by a description-defined index.

#### Scenario: Publish joint feedback
- **GIVEN** a connection owns joint position and velocity
- **WHEN** it publishes state
- **THEN** the frame names `left/j1/position` and `left/j1/velocity` explicitly
- **AND** values align one-to-one with names

#### Scenario: Interface description defines capabilities
- **GIVEN** an accepted description lists supported interfaces in some order
- **WHEN** a valid state or command frame arrives
- **THEN** the consumer matches its explicitly carried names against the description
- **AND** does not interpret values through the description's list positions

#### Scenario: Frame arrays do not align
- **GIVEN** a frame with different name and value counts
- **WHEN** a consumer validates it
- **THEN** the complete frame is rejected
- **AND** no partial update occurs

### Requirement: Whole-joint arbitration

The coordinator SHALL arbitrate canonical joints and select at most one winning task for each joint. The winner SHALL exclusively supply all command-interface values for that joint.

#### Scenario: Tasks claim disjoint joints
- **GIVEN** tasks claiming `left/j1` and `right/j1`
- **WHEN** both produce valid commands
- **THEN** both tasks may win independently and route to their owning connections

#### Scenario: Tasks contend with different command interfaces
- **GIVEN** a lower-priority task claiming `left/j1` with a position command and a higher-priority task claiming `left/j1` with a velocity command
- **WHEN** the coordinator arbitrates
- **THEN** the higher-priority task alone owns `left/j1`
- **AND** no position value from the losing task is combined with the winner's velocity value

### Requirement: Runtime command-interface transitions

The system SHALL support changing the command interfaces emitted by winning tasks without restarting tasks or connections. The coordinator SHALL preserve ordinary joint arbitration and SHALL NOT orchestrate native device transitions.

#### Scenario: Winner changes from position to velocity
- **GIVEN** a position task owns an arm's joints and a higher-priority velocity task contends for them
- **WHEN** the velocity task wins
- **THEN** the coordinator selects the velocity task as joint owner and publishes its velocity commands
- **AND** the connection alone performs any required native transition before applying those values

#### Scenario: Hardware rejects a proposed combination
- **GIVEN** winning commands require an interface combination unsupported by one connection
- **WHEN** that connection validates the command
- **THEN** it applies no value through an incompatible native device mode
- **AND** it reports a fault through normal connection status

### Requirement: Native modes remain connection-private

The generic control contract SHALL express active command interfaces rather than vendor-specific control modes. Each connection SHALL map accepted command-interface sets to its native device behavior.

#### Scenario: Device has a vendor-specific velocity mode
- **GIVEN** tasks and the coordinator request joint velocity interfaces
- **WHEN** the connection performs the transition
- **THEN** it selects the required vendor-native mode internally
- **AND** the vendor mode identifier does not appear in task or coordinator APIs

### Requirement: Complete connection state snapshots

Each state frame SHALL contain exactly one value for every state interface in its accepted description. The coordinator SHALL atomically accept a valid snapshot or reject the complete frame.

#### Scenario: Receive a complete snapshot
- **GIVEN** an accepted description and a newer sequence
- **WHEN** the source publishes every state key once
- **THEN** the coordinator replaces its cached snapshot atomically
- **AND** tasks observe coherent source state

#### Scenario: Snapshot omits an interface
- **GIVEN** a description declaring several state keys
- **WHEN** a frame omits one or adds an unknown key
- **THEN** the frame is rejected
- **AND** the last valid snapshot remains until its stale deadline

### Requirement: Named robot joint-state projection

The coordinator SHALL publish aggregated position, velocity, and effort feedback as `JointState` with canonical joint names carried in every message. It SHALL derive each joint entry from the validated scalar interfaces with the same canonical joint prefix.

#### Scenario: Publish aggregated arm state
- **GIVEN** valid state interfaces for `left/j1` and `right/j1`
- **WHEN** the coordinator publishes robot state
- **THEN** `JointState.name` contains `left/j1` and `right/j1` directly
- **AND** manipulation consumes those names without connection, robot-ID, or local/global name mapping

### Requirement: Sparse commands and explicit heartbeat

Command frames MAY contain only interfaces with current winners. The coordinator SHALL publish at its heartbeat rate even when no task supplies a value.

#### Scenario: No active winner
- **GIVEN** an armed robot with no active task claim
- **WHEN** the coordinator ticks
- **THEN** it publishes an empty frame with fresh sequence and epoch
- **AND** connections distinguish liveness from ownership

#### Scenario: Coordinator enters safe stop
- **GIVEN** the coordinator has closed its command gate because of a fault
- **WHEN** it initiates robot-wide safe stop
- **THEN** it stops publishing ordinary command heartbeats
- **AND** it does not represent safe stop as an empty command frame

#### Scenario: Route sparse commands
- **GIVEN** winners owned by different connections
- **WHEN** the coordinator publishes them
- **THEN** each connection consumes its declared keys
- **AND** omitted keys follow advertised policies

### Requirement: Canonical units

Control interfaces SHALL use radians or meters for position, radians per second or meters per second for velocity, and Nm or N for effort. Native units SHALL not cross the connection boundary.

#### Scenario: Command a vendor-native gripper
- **GIVEN** a gripper with a non-SI native range
- **WHEN** a task emits a canonical joint command
- **THEN** the control frame contains the canonical value
- **AND** only the connection converts it

### Requirement: Sequence and freshness semantics

Consumers SHALL reject repeated or out-of-order sequences. Producer timestamps SHALL describe sampling, while coordinator-local monotonic receipt time SHALL determine safety freshness.

#### Scenario: Device clocks differ
- **GIVEN** an unsynchronized device clock
- **WHEN** valid frames arrive within the expected interval
- **THEN** the source remains fresh by monotonic receipt time
- **AND** source time remains available for analysis

#### Scenario: An older frame arrives late
- **GIVEN** sequence 42 is accepted
- **WHEN** sequence 41 arrives afterward
- **THEN** it is rejected
- **AND** cached state does not roll back

### Requirement: Static shared typed streams

Connection instances SHALL exchange concrete control message types over shared class-declared streams. Connection count SHALL NOT require generated subclasses or a maximum port count.

#### Scenario: Add another connection
- **GIVEN** a connection class already in a blueprint
- **WHEN** a second named instance with disjoint interfaces is added
- **THEN** both use the existing shared streams
- **AND** coordinator port declarations remain unchanged

### Requirement: Transport-safe topology

Streams with multiple connection publishers SHALL use a multiwriter-safe transport. The single-writer coordinator command stream MAY use a compatible latest-wins transport.

#### Scenario: Build a multi-connection blueprint
- **GIVEN** several connections publish description, status, and state
- **WHEN** transports are assigned
- **THEN** those streams use a multiwriter-safe transport
- **AND** startup rejects unsafe shared-memory assignment
