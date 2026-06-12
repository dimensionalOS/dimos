## ADDED Requirements

### Requirement: Planning scene metadata is synchronized to visualization at startup
The system SHALL synchronize initialized planning-scene metadata to an attached manipulation visualization backend after robots are added and before periodic visualization publishing starts.

#### Scenario: External visualization receives robot metadata once at startup
- **WHEN** a manipulation module initializes with an external visualization backend and one or more configured robots
- **THEN** the visualization backend receives a planning-scene snapshot containing each world robot ID and its corresponding robot model configuration before the visualization thread starts

#### Scenario: Embedded visualization can ignore scene sync
- **WHEN** the active visualization is embedded in the world backend and already observes world robot registration
- **THEN** startup scene synchronization succeeds without requiring additional backend-specific robot registration behavior

### Requirement: Robot addition remains implementation agnostic
The system SHALL keep robot addition in the world monitor independent of visualization backend-specific capabilities.

#### Scenario: Robot is added without probing visualization-specific methods
- **WHEN** `WorldMonitor.add_robot()` adds a robot to the world
- **THEN** it records the robot metadata needed for later scene synchronization
- **AND** it does not call a backend-specific `register_robot` hook on the visualization

### Requirement: Planning scene snapshot is reusable collaborator metadata
The planning-scene synchronization model SHALL be defined as neutral metadata that can be reused by future external collaborators without exposing visualization widgets, backend handles, or mutable world contexts.

#### Scenario: Snapshot contains stable robot metadata
- **WHEN** a planning-scene snapshot is created
- **THEN** it contains robot IDs mapped to robot model configurations
- **AND** it does not contain Viser-specific handles, GUI state, Drake contexts, planner instances, or execution state

### Requirement: Viser initializes robot visuals from scene sync
The Viser manipulation visualizer SHALL initialize its robot scene objects from startup planning-scene synchronization instead of per-robot registration during `WorldMonitor.add_robot()`.

#### Scenario: Viser receives startup scene snapshot
- **WHEN** Viser receives a planning-scene snapshot with a robot configuration
- **THEN** it prepares current robot, target ghost, preview ghost, and joint-name mapping state for that robot

#### Scenario: Viser target and preview behavior remains unchanged after sync
- **WHEN** target updates, preview animations, or current-state publishes occur after startup sync
- **THEN** Viser uses the synced robot metadata to update the same target/current/preview visuals as before
