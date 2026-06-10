# manipulation-planning-backends Specification

## Purpose
Define the active manipulation planning backend abstraction used by manipulation modules to keep planning, scene state, diagnostics, stored paths, and coordinator execution backend-neutral.

## Requirements
### Requirement: Startup active backend selection
DimOS manipulation planning SHALL select exactly one active planning backend when a manipulation planning module starts.

#### Scenario: Default backend preserves existing behavior
- **GIVEN** an existing manipulation or xArm planner blueprint with no explicit backend selection
- **WHEN** the blueprint starts
- **THEN** the planning stack uses the current Drake-backed behavior by default
- **AND** existing user-facing planning, Viser path review, obstacle, and execution commands remain available.

#### Scenario: Runtime backend switching is not supported
- **GIVEN** a manipulation planning module has already started with an active backend
- **WHEN** a caller attempts to switch to another backend without restarting/reconstructing the module
- **THEN** DimOS reports that runtime backend switching is unsupported for this change
- **AND** the active backend scene is not partially replaced or synchronized with another backend.

### Requirement: Backend scene behavior covers current manipulation use cases
The active backend scene SHALL expose the current manipulation scene behavior needed by planning, perception obstacle refresh, state sync, and diagnostics.

#### Scenario: Robot state is synchronized into the active backend
- **GIVEN** a configured robot and incoming joint state messages
- **WHEN** the manipulation planning module receives joint state for that robot
- **THEN** the active backend records the current joint position in the robot model's planning joint order
- **AND** planning requests can use the latest available state as their start state.

#### Scenario: Current obstacle behavior is preserved
- **GIVEN** a user or module adds, removes, updates, or clears planning obstacles
- **WHEN** the active backend receives the scene mutation
- **THEN** the observable obstacle list and subsequent planning collision behavior reflect the mutation according to the backend's reported capability
- **AND** unsupported or approximated scene updates are reported through diagnostics rather than silently ignored.

#### Scenario: Perception objects remain planning obstacles
- **GIVEN** pick-and-place perception has cached detected objects
- **WHEN** the user refreshes perception obstacles
- **THEN** eligible objects are added to the active planning scene as box obstacles by default
- **AND** existing scan, list, clear, and pick/place flows continue to observe stable object IDs and obstacle IDs.

### Requirement: Current Drake features remain fully supported after abstraction
The first implementation milestone SHALL deliver a running refactored structure with full support for all current Drake-backed manipulation features before any future backend is added.

#### Scenario: Drake-backed planning still supports current planning surfaces
- **GIVEN** the active backend is Drake
- **WHEN** a caller plans to joints, plans to a pose, reviews stored path data through Viser, queries current joints, queries end-effector pose, or executes a stored trajectory
- **THEN** the behavior matches the pre-refactor Drake-backed manipulation behavior
- **AND** trajectories still execute through existing coordinator trajectory tasks.

#### Scenario: Drake-backed scene queries remain available
- **GIVEN** the active backend is Drake
- **WHEN** a caller or skill needs collision checks, path validation, FK, link poses for TF publishing, Jacobian IK, Drake optimization IK, or obstacle inspection
- **THEN** the refactored backend surface supports those current behaviors
- **AND** no current Drake feature is dropped as part of the abstraction milestone.

### Requirement: Backend planning results are normalized for manipulation callers
The active backend planner SHALL return planning outcomes in a DimOS-observable form that existing callers can store, render through Viser, and execute.

#### Scenario: Successful plan returns a storable path and executable trajectory
- **GIVEN** a backend can find a path for a valid target
- **WHEN** a caller requests a plan through a manipulation planning RPC or skill path
- **THEN** DimOS stores the planned joint path for Viser rendering and execution
- **AND** DimOS can convert or preserve timing into a `JointTrajectory` suitable for the existing coordinator execution path.

#### Scenario: Failed plan reports actionable status
- **GIVEN** a backend cannot produce a valid plan
- **WHEN** planning fails because of timeout, invalid start, invalid goal, collision, unsupported target, stale state, missing configuration, or no solution
- **THEN** the caller receives a status or message that identifies the failure category
- **AND** the failure does not trigger execution of stale or partial trajectory data.

### Requirement: Backend capability diagnostics are visible
DimOS SHALL expose backend capability and diagnostic information for behavior that differs between planning backends.

#### Scenario: Unsupported feature is requested
- **GIVEN** the active backend does not support a requested scene, query, or planning feature
- **WHEN** a caller requests that feature
- **THEN** DimOS reports that the feature is unsupported by the active backend
- **AND** the report names the relevant backend and feature rather than pretending the operation succeeded.

#### Scenario: Backend-specific approximation is used
- **GIVEN** an obstacle, pointcloud, or attached object request must be approximated for the active backend
- **WHEN** DimOS applies the approximation
- **THEN** the resulting diagnostics report that the operation was approximated
- **AND** planning continues only against the scene representation that was actually applied.
