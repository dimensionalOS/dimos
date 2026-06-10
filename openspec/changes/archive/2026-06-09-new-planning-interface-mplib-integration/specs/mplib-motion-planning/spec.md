## ADDED Requirements

### Requirement: MPlib backend availability and configuration
DimOS SHALL allow users to select MPlib as the active manipulation planning backend when the manipulation extra and required robot planning assets are available.

#### Scenario: MPlib starts with complete robot assets
- **GIVEN** a manipulation stack selects the MPlib backend at startup
- **AND** the robot configuration provides URDF, SRDF, package path, move group, link ordering, joint ordering, and end-effector information required by MPlib
- **WHEN** the stack starts
- **THEN** DimOS initializes the MPlib planning backend once for that robot and move group
- **AND** subsequent plans reuse the initialized backend rather than rebuilding the planner for every request.

#### Scenario: MPlib configuration is incomplete
- **GIVEN** a manipulation stack selects the MPlib backend
- **AND** required MPlib robot assets or ordering fields are missing
- **WHEN** the stack starts or the first MPlib planning request is prepared
- **THEN** DimOS reports which MPlib configuration fields are missing
- **AND** non-MPlib default manipulation stacks remain unaffected.

### Requirement: MPlib joint planning
The MPlib backend SHALL support joint-space planning through MPlib's native joint planning surface when it is selected as the active backend.

#### Scenario: Plan to joints succeeds
- **GIVEN** MPlib is the active backend with a current start state and a valid target joint state
- **WHEN** a caller requests a plan to joints
- **THEN** DimOS calls the native MPlib joint planner and returns a successful planning result when MPlib succeeds
- **AND** the returned path is expressed in the DimOS robot joint order expected by Viser path review and execution callers.

#### Scenario: MPlib joint ordering differs from DimOS ordering
- **GIVEN** MPlib requires a configured joint order that differs from the DimOS robot joint order
- **WHEN** a plan request is sent to MPlib and a path is returned
- **THEN** DimOS maps start and goal values into MPlib order before planning
- **AND** maps returned waypoints back into DimOS robot joint order before storing, rendering through Viser, or executing the path.

### Requirement: MPlib pose planning
The MPlib backend SHALL support pose planning through MPlib's native pose planning surface when the installed MPlib package exposes it.

#### Scenario: Plan to pose succeeds
- **GIVEN** MPlib is the active backend with a current start state and a target end-effector pose in the planning frame
- **WHEN** a caller requests a plan to pose
- **THEN** DimOS uses MPlib's native pose planning behavior when available
- **AND** returns a normalized DimOS planning result that can be rendered through Viser and executed through the existing trajectory path.

#### Scenario: Native pose planning is unavailable
- **GIVEN** MPlib is the active backend but the installed MPlib package does not expose usable native pose planning for the configured robot
- **WHEN** a caller requests a plan to pose
- **THEN** DimOS reports pose planning as unsupported or unavailable for that backend configuration
- **AND** does not silently fall back to an invalid or unvalidated target.

### Requirement: MPlib scene projection
The MPlib backend SHALL project the active DimOS planning scene into MPlib using native MPlib scene APIs wherever DimOS can represent the scene safely.

#### Scenario: Primitive obstacles are projected
- **GIVEN** MPlib is active and DimOS has box, sphere, cylinder, or mesh obstacles in the planning scene
- **WHEN** the backend prepares or updates the MPlib scene
- **THEN** every obstacle shape supported by the installed MPlib API is applied to the MPlib collision scene
- **AND** unsupported shapes are reported in backend diagnostics with the obstacle identifier.

#### Scenario: Perception objects use current obstacle semantics
- **GIVEN** pick-and-place perception has cached object detections
- **WHEN** perception obstacles are refreshed while MPlib is active
- **THEN** objects are represented as box obstacles by default, matching current manipulation behavior
- **AND** object pointclouds are used for convex-hull mesh obstacles only when that option is explicitly enabled and hull generation succeeds.

#### Scenario: Raw pointcloud collision is enabled
- **GIVEN** MPlib is active and raw pointcloud collision support is enabled in backend configuration
- **AND** a pointcloud layer is in the active planning frame and the installed MPlib API supports pointcloud updates
- **WHEN** the scene is prepared for planning
- **THEN** DimOS applies the pointcloud to MPlib's native pointcloud collision scene with the configured resolution
- **AND** skipped pointcloud layers are reported with a reason such as disabled policy, frame mismatch, missing data, or unavailable native API.

#### Scenario: Attached objects are supported when available
- **GIVEN** MPlib is active and the installed MPlib package exposes native attached-object behavior
- **WHEN** an object is attached to or detached from a robot link
- **THEN** DimOS updates the MPlib scene using the native attached-object operation
- **AND** if native attached-object support is unavailable, DimOS reports the feature as unsupported instead of dropping the attachment silently.

### Requirement: MPlib result and diagnostic normalization
The MPlib backend SHALL normalize native MPlib planning outcomes into DimOS planning statuses, timing, paths, trajectories, and diagnostics.

#### Scenario: Successful MPlib result includes timing
- **GIVEN** MPlib returns a successful path with timing, velocity, acceleration, duration, or status information
- **WHEN** DimOS converts the result
- **THEN** the planning result preserves available timing information where it is compatible with existing trajectory execution
- **AND** reports the native backend status in diagnostics.

#### Scenario: MPlib reports failure
- **GIVEN** MPlib returns timeout, collision, invalid start, invalid goal, no solution, missing method, or another failed status
- **WHEN** DimOS converts the result
- **THEN** the caller receives a DimOS planning failure status and message that reflects the MPlib failure category
- **AND** no stale successful plan is executed as a result of the failed MPlib request.

### Requirement: MPlib does not break non-MPlib stacks
Adding MPlib support SHALL NOT break existing non-MPlib manipulation, navigation, perception, or agentic stacks.

#### Scenario: Default manipulation stack omits MPlib configuration
- **GIVEN** an existing Drake-backed manipulation blueprint without MPlib-specific robot fields
- **WHEN** the blueprint starts after this change
- **THEN** it continues to use the default Drake-backed planning behavior
- **AND** missing MPlib assets or bindings are not reported unless MPlib is selected.

#### Scenario: Environment lacks MPlib bindings
- **GIVEN** MPlib Python bindings are not installed in the active environment
- **WHEN** a non-MPlib blueprint starts
- **THEN** the blueprint starts normally if it does not select MPlib
- **AND** selecting MPlib reports a clear dependency error with an installation or verification hint.
