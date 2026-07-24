## ADDED Requirements

### Requirement: Finalized obstacle lifecycle
The planning world SHALL reject every obstacle add, remove, complete update, pose update, clear, and retrieval operation until world finalization is complete.

#### Scenario: Obstacle operation before finalization
- **WHEN** a caller invokes any planning-world obstacle operation before finalization
- **THEN** the operation raises `RuntimeError` without mutating native or stored obstacle state

#### Scenario: Obstacle operation after finalization
- **WHEN** a caller invokes a valid obstacle operation after finalization
- **THEN** the planning world evaluates the operation according to its normal result contract

### Requirement: Complete obstacle replacement
The planning world SHALL provide a complete replacement operation that identifies the target solely by the non-empty immutable `Obstacle.name`, replaces all obstacle properties without merging stored values, returns `True` on success, and returns `False` when the named obstacle does not exist.

#### Scenario: Replace an existing obstacle
- **WHEN** a caller supplies a valid complete obstacle whose name identifies an existing obstacle
- **THEN** the world replaces the obstacle's native geometry and stored snapshot and returns `True`

#### Scenario: Replace an unknown obstacle
- **WHEN** a caller supplies a valid complete obstacle whose name is not present
- **THEN** the world returns `False` without adding or mutating an obstacle

#### Scenario: Invalid complete replacement
- **WHEN** a caller supplies an obstacle with an empty name, invalid geometry dimensions, unsupported type, invalid pose, or unusable mesh information
- **THEN** the world raises `ValueError` before changing native or stored obstacle state

#### Scenario: Obstacle renaming
- **WHEN** a caller needs to change an obstacle's name
- **THEN** the caller must remove the old obstacle and add a complete obstacle with the new name

### Requirement: Pose-only obstacle update
The planning world SHALL provide a pose-only update that identifies an obstacle by name, atomically changes its pose, preserves every non-pose property, returns `True` on success, and returns `False` when the name does not exist.

#### Scenario: Move an existing obstacle
- **WHEN** a caller supplies a valid pose for an existing obstacle name
- **THEN** the native collision geometry and stored obstacle snapshot use the new pose while type, dimensions, color, and mesh information remain unchanged

#### Scenario: Move an unknown obstacle
- **WHEN** a caller supplies a pose for an obstacle name that is not present
- **THEN** the world returns `False` without creating an obstacle

### Requirement: Atomic native scene operations
Each world backend SHALL use one backend-owned reentrant lock so an obstacle mutation and another native scene operation cannot execute concurrently. A native scene operation SHALL observe either the complete obstacle state before one update or the complete state after it.

#### Scenario: Collision query races with replacement
- **WHEN** a complete replacement has removed old native geometry but has not installed new geometry
- **THEN** a concurrent collision query remains blocked until the replacement commits or fails

#### Scenario: Replacement races with collision query
- **WHEN** a collision or distance query is already using the native scene
- **THEN** an obstacle update remains blocked until that query completes

#### Scenario: Generic planner spans an update
- **WHEN** an obstacle update occurs between two separate collision checks made by a generic planner
- **THEN** each collision check is individually atomic and the planner is permitted to observe different complete obstacle revisions

#### Scenario: RoboPlan native planning
- **WHEN** RoboPlan-native planning is executing its opaque native call
- **THEN** obstacle mutations remain blocked for the duration of that call

### Requirement: Fail-closed native update failure
The planning world SHALL propagate an unexpected native obstacle-update exception, mark itself invalid, and reject later native scene operations until reconstruction.

#### Scenario: Native replacement fails after mutation begins
- **WHEN** a native backend raises unexpectedly after an obstacle update starts
- **THEN** the original exception propagates and the planning world becomes invalid without attempting rollback

#### Scenario: Operation on an invalid world
- **WHEN** a caller invokes a native scene operation after the world becomes invalid
- **THEN** the operation raises `RuntimeError` without querying or mutating the native scene

### Requirement: Obstacle snapshot ownership
The planning world SHALL own private obstacle snapshots and SHALL NOT expose mutable aliases to accepted or stored obstacle values.

#### Scenario: Caller mutates an accepted input
- **WHEN** a caller mutates an obstacle object after a successful add or complete replacement
- **THEN** the planning world's stored snapshot and native geometry remain unchanged

#### Scenario: Caller mutates a retrieved obstacle
- **WHEN** a caller mutates an obstacle returned by obstacle retrieval
- **THEN** the planning world's stored snapshot and native geometry remain unchanged

### Requirement: Visualization obstacle updates
`VisualizationSpec` SHALL expose complete and pose-only obstacle update commands matching the planning-world semantics, and each visualization implementation SHALL make its own renderer mutation atomic.

#### Scenario: Complete visualization replacement
- **WHEN** visualization receives a complete obstacle replacement
- **THEN** the renderer exposes either the complete old representation or the complete new representation and never a partial replacement

#### Scenario: Visualization pose update
- **WHEN** visualization receives a pose-only update
- **THEN** the representation moves to the new pose while preserving its non-pose appearance and geometry

### Requirement: Visualization failure isolation
Visualization implementations SHALL handle renderer update failures internally, log the failure, expose a persistent frontend warning when a frontend exists, and return without invalidating or rolling back the authoritative planning world.

#### Scenario: Renderer update fails
- **WHEN** the planning-world update succeeds and the corresponding renderer mutation fails
- **THEN** the renderer records a log warning and frontend warning, the visualization command returns normally, and the planning world remains valid with the accepted update

#### Scenario: Visualization remains stale
- **WHEN** a renderer update fails in version one
- **THEN** the system does not automatically retry, replay, or resynchronize obstacle visualization

### Requirement: Incoming obstacle update routing
The obstacle monitor SHALL route a pose-only message to pose update and SHALL require a complete obstacle description for any structural or appearance replacement without merging missing fields.

#### Scenario: Pose-only message for an existing obstacle
- **WHEN** an update message for an existing obstacle contains only a pose change
- **THEN** the monitor invokes pose-only update

#### Scenario: Complete replacement message
- **WHEN** an update message contains a structural or appearance change and all complete obstacle fields
- **THEN** the monitor constructs one complete obstacle and invokes full replacement

#### Scenario: Incomplete replacement message
- **WHEN** an update message contains a structural or appearance change but omits required complete obstacle information
- **THEN** the monitor rejects the message with a warning and does not merge stored fields

#### Scenario: Pose-only message for an unknown obstacle
- **WHEN** an update message contains only a pose and names an unknown obstacle
- **THEN** the monitor does not create an obstacle

#### Scenario: Complete update for an unknown obstacle
- **WHEN** an update message names an unknown obstacle and contains a complete obstacle description
- **THEN** the monitor may preserve existing update-as-add ingestion behavior

### Requirement: Per-obstacle transaction scope
The system SHALL make each individual obstacle operation atomic and SHALL NOT imply one transaction across multiple obstacles from the same perception frame.

#### Scenario: Collision query between frame updates
- **WHEN** a perception frame causes updates to multiple obstacles
- **THEN** a collision query may run after one obstacle update commits and before another begins

### Requirement: Manipulation client access
`ManipulationModule` SHALL expose complete and pose-only obstacle updates over
RPC, and the interactive manipulation client SHALL provide callable helpers for
both operations.

#### Scenario: Complete client replacement
- **WHEN** a client supplies an obstacle name, pose, shape, dimensions, mesh information, and appearance
- **THEN** the module constructs a complete replacement without retrieving or merging the existing obstacle

#### Scenario: Pose-only client update
- **WHEN** a client supplies an obstacle name and new pose through the pose-only helper
- **THEN** the module invokes the authoritative pose-only update path

### Requirement: Stored plans remain outside update semantics
Successful obstacle updates SHALL NOT invalidate stored generated plans or add execution-time collision guarantees as part of this capability.

#### Scenario: Obstacle changes after planning
- **WHEN** an obstacle changes after a plan is generated
- **THEN** this capability does not alter stored-plan validity or execution behavior
