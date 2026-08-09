## ADDED Requirements

### Requirement: Named scalar control-value contract

Control-state and command frames SHALL carry a source, producer timestamp, per-source sequence, parallel interface-name and `float64` value arrays, and fully qualified canonical keys.

#### Scenario: Publish joint feedback
- **GIVEN** a connection owns joint position and velocity
- **WHEN** it publishes state
- **THEN** the frame names `left/j1/position` and `left/j1/velocity` explicitly
- **AND** values align one-to-one with names

#### Scenario: Frame arrays do not align
- **GIVEN** a frame with different name and value counts
- **WHEN** a consumer validates it
- **THEN** the complete frame is rejected
- **AND** no partial update occurs

### Requirement: Exact interface-key arbitration

The coordinator SHALL arbitrate exact scalar interface keys and SHALL NOT require joint, hardware, robot, atomic-device, or mode semantics to select a winner.

#### Scenario: Tasks claim disjoint fields
- **GIVEN** tasks claiming `left/j1/position` and `right/j1/position`
- **WHEN** both produce valid commands
- **THEN** both may win independently and route to their owners

#### Scenario: Tasks contend for one field
- **GIVEN** tasks claiming the same key at different priorities
- **WHEN** the coordinator ticks
- **THEN** only the higher-priority valid value wins
- **AND** unrelated keys are unaffected

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

### Requirement: Sparse commands and explicit heartbeat

Command frames MAY contain only interfaces with current winners. The coordinator SHALL publish at its heartbeat rate even when no task supplies a value.

#### Scenario: No active winner
- **GIVEN** an armed robot with no active task claim
- **WHEN** the coordinator ticks
- **THEN** it publishes an empty frame with fresh sequence and epoch
- **AND** connections distinguish liveness from ownership

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
