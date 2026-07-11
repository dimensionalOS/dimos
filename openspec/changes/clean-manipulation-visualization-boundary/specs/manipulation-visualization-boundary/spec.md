## ADDED Requirements

### Requirement: Visualization initialization must receive immutable topology
The visualization protocol MUST initialize each backend once with robot configurations, resolved planning groups, and an optional concrete manipulation operator. Robot and planning-group topology MUST remain unchanged for the initialized session.

#### Scenario: Interactive visualization initializes
- **WHEN** manipulation world setup completes with an interactive visualization backend
- **THEN** the backend receives all robot IDs, robot configs, resolved planning groups, and the concrete manipulation operator in one initialization session

#### Scenario: Non-interactive visualization initializes
- **WHEN** a rendering backend does not provide an operator panel
- **THEN** it initializes from the same static topology and ignores the optional operator

### Requirement: Visualization state must be pushed without world access
WorldMonitor MUST push current joint-state frames to visualization backends at visualization update cadence. A visualization backend MUST NOT pull current state, freshness, FK, collision, planning, or execution data from `WorldMonitor` or `ManipulationModule`.

#### Scenario: Current robot state changes
- **WHEN** WorldMonitor publishes a visualization update
- **THEN** the backend receives current joint states keyed by initialized world robot IDs

#### Scenario: State freshness changes
- **WHEN** telemetry becomes stale
- **THEN** no freshness policy is added to the visualization frame and authoritative manipulation actions validate state independently

### Requirement: Visualization preview must consume raw synchronized trajectories
The visualization protocol MUST animate the stored globally named `JointTrajectory` directly and MUST NOT receive `GeneratedPlan` or regenerate timing from a geometric path.

#### Scenario: Stored trajectory is previewed
- **WHEN** manipulation previews a generated plan without a display-duration override
- **THEN** the backend plays the raw trajectory using its stored timestamps

#### Scenario: Display duration is overridden
- **WHEN** preview specifies a display duration
- **THEN** the backend scales playback delays without mutating or re-parameterizing the stored trajectory

#### Scenario: Multiple robots are planned
- **WHEN** trajectory joint names span multiple initialized robots
- **THEN** the backend projects those joints through static topology and advances every affected preview ghost on the same stored clock

### Requirement: Visualization preview must be transient
The visualization protocol MUST expose only trajectory animation and cancellation for preview lifecycle. Preview ghosts MUST disappear after completion, cancellation, error, replacement, or backend close.

#### Scenario: Preview completes
- **WHEN** the final trajectory time is displayed
- **THEN** all preview ghosts are hidden immediately

#### Scenario: Preview is cancelled
- **WHEN** cancellation occurs during playback
- **THEN** stale frames stop mutating the scene and all preview ghosts are hidden immediately

#### Scenario: Preview is replaced
- **WHEN** a new trajectory begins while another preview is active
- **THEN** the old generation is cancelled and cannot mutate the new preview

### Requirement: Visualization backends must remain domain-isolated
Viser GUI and renderer construction MUST NOT receive raw `ManipulationModule` or `WorldMonitor` dependencies. Runtime render methods MUST NOT perform planning-result validation, target evaluation, execution freshness checks, or action dispatch outside the concrete manipulation operator.

#### Scenario: Viser panel starts
- **WHEN** the visualization session includes an operator
- **THEN** the panel uses only that operator for domain reads and actions

#### Scenario: Preview renders
- **WHEN** Viser receives a valid raw trajectory
- **THEN** it performs only topology/name projection, visual-state overlay, playback, and ghost cleanup
