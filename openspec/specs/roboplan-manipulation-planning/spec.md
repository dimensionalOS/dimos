# roboplan-manipulation-planning Specification

## Purpose
Define the optional RoboPlan manipulation planning backend, including selection, robot asset validation, planning behavior, diagnostics, stored path rendering, and coordinator-mediated execution compatibility.

## Requirements
### Requirement: RoboPlan backend selection
DimOS SHALL allow manipulation stacks to select RoboPlan as an optional active planning backend through the existing manipulation backend-selection configuration surface.

#### Scenario: Select RoboPlan backend
- **GIVEN** a manipulation stack configured with valid robot planning assets and `planning_backend="roboplan"`
- **WHEN** the manipulation module starts
- **THEN** the active planning backend is RoboPlan
- **AND** existing manipulation RPC and skill names remain available to callers.

#### Scenario: Preserve default backend behavior
- **GIVEN** a manipulation stack that does not configure RoboPlan
- **WHEN** the manipulation module starts
- **THEN** the existing default backend behavior is preserved
- **AND** missing RoboPlan packages or native bindings do not prevent the stack from starting.

#### Scenario: Unknown or unavailable backend reports clearly
- **GIVEN** a manipulation stack configured to use RoboPlan
- **WHEN** RoboPlan is not installed or a required RoboPlan binding cannot be imported
- **THEN** startup fails with an actionable backend availability error
- **AND** the error identifies RoboPlan as the selected backend and names the missing required capability or module where possible.

### Requirement: RoboPlan robot asset validation
DimOS SHALL validate RoboPlan robot planning assets before accepting RoboPlan planning requests.

#### Scenario: Valid RoboPlan robot configuration
- **GIVEN** a RoboPlan-selected manipulation stack with a robot model, required package paths, joint names, base frame, end-effector frame, limits, and any required SRDF or planning-group data
- **WHEN** the manipulation module starts
- **THEN** the robot is accepted for RoboPlan planning
- **AND** subsequent planning requests use the validated joint and frame configuration.

#### Scenario: Missing required robot asset
- **GIVEN** a RoboPlan-selected manipulation stack with a missing required robot asset or frame setting
- **WHEN** the manipulation module starts
- **THEN** startup fails before any robot motion request is accepted
- **AND** the failure message identifies the missing or invalid asset.

#### Scenario: Joint order is observable and stable
- **GIVEN** a RoboPlan-selected manipulation stack whose robot has a configured DimOS joint order and coordinator joint-name mapping
- **WHEN** callers provide joint states or joint goals through existing DimOS surfaces
- **THEN** callers continue to use the configured DimOS joint order
- **AND** returned paths and trajectories use that same public joint order regardless of RoboPlan's internal active joint ordering.

### Requirement: RoboPlan joint-space planning
DimOS SHALL support RoboPlan-backed joint-space planning where RoboPlan planning is available for the selected robot.

#### Scenario: Plan to joint target succeeds
- **GIVEN** a RoboPlan-selected manipulation stack with current robot joint state available
- **WHEN** a caller requests planning to a collision-free joint target through the existing manipulation planning surface
- **THEN** DimOS returns a successful planning result
- **AND** stores a normalized planned path and trajectory that can be rendered by Viser or executed through existing manipulation flows.

#### Scenario: Current state is unavailable
- **GIVEN** a RoboPlan-selected manipulation stack with no current joint state for the requested robot
- **WHEN** a caller requests joint-space planning
- **THEN** DimOS rejects the request with a planning failure that explains the missing state
- **AND** no trajectory is stored for execution.

#### Scenario: No path found
- **GIVEN** a RoboPlan-selected manipulation stack and a joint target RoboPlan cannot solve within the requested planning constraints
- **WHEN** a caller requests joint-space planning
- **THEN** DimOS returns a failed planning result without executing motion
- **AND** the result includes a backend diagnostic or message suitable for user-facing skill output.

### Requirement: RoboPlan pose planning and IK behavior
DimOS SHALL support RoboPlan-backed pose planning where available and SHALL report unsupported pose or IK capability clearly when unavailable.

#### Scenario: Plan to pose succeeds
- **GIVEN** a RoboPlan-selected manipulation stack whose selected RoboPlan configuration supports pose planning or IK for the end effector
- **WHEN** a caller requests planning to a reachable target pose through the existing manipulation surface
- **THEN** DimOS returns a successful planning result
- **AND** the resulting path and trajectory are normalized for Viser rendering and existing execution flows.

#### Scenario: Pose planning unsupported
- **GIVEN** a RoboPlan-selected manipulation stack whose RoboPlan capability report does not support pose planning or IK
- **WHEN** a caller requests planning to a pose
- **THEN** DimOS rejects the request with an unsupported-capability failure
- **AND** the failure does not claim that a plan or IK solution was attempted successfully.

#### Scenario: Frame mismatch is reported
- **GIVEN** a RoboPlan-selected manipulation stack with a target pose in a frame that cannot be interpreted for the configured robot
- **WHEN** a caller requests pose planning
- **THEN** DimOS rejects the request with a frame or pose-conversion diagnostic
- **AND** no trajectory is stored for execution.

### Requirement: RoboPlan scene projection and collision validation
DimOS SHALL project supported DimOS planning obstacles into the RoboPlan scene and SHALL report unsupported or approximated scene features.

#### Scenario: Supported primitive obstacle is applied
- **GIVEN** a RoboPlan-selected manipulation stack and a supported primitive obstacle
- **WHEN** the obstacle is added through existing manipulation scene or perception-obstacle flows
- **THEN** the obstacle affects RoboPlan collision validation and planning
- **AND** the scene update result or diagnostics indicate that the obstacle was applied.

#### Scenario: Unsupported obstacle is reported
- **GIVEN** a RoboPlan-selected manipulation stack and an obstacle type not supported by the active RoboPlan configuration
- **WHEN** the obstacle is added or refreshed
- **THEN** DimOS reports the obstacle as unsupported or approximated
- **AND** DimOS does not silently claim full collision support for that obstacle.

#### Scenario: Path collision validation uses RoboPlan state
- **GIVEN** a RoboPlan-selected manipulation stack with a planned path and active scene obstacles
- **WHEN** DimOS validates the path before storing or executing it
- **THEN** validation uses the RoboPlan-backed active scene state
- **AND** invalid paths are rejected without robot execution.

### Requirement: RoboPlan capability diagnostics
DimOS SHALL expose RoboPlan planning capabilities and diagnostics through existing manipulation status, RPC, or skill-result surfaces where backend details are user or developer visible.

#### Scenario: Capability report identifies supported features
- **GIVEN** a RoboPlan-selected manipulation stack after startup
- **WHEN** a caller inspects backend capabilities or receives planning diagnostics
- **THEN** the report identifies RoboPlan as the active backend
- **AND** indicates support status for joint planning, pose planning or IK, collision checking, FK/Jacobian/distance queries where exposed, obstacle classes, and retiming.

#### Scenario: Unsupported optional feature remains non-fatal
- **GIVEN** a RoboPlan-selected manipulation stack where an optional RoboPlan feature such as TOPPRA retiming, mesh obstacles, pointcloud layers, attached objects, FK, Jacobian, or distance query is unavailable
- **WHEN** the unavailable feature is not required by the selected request or configuration
- **THEN** the stack can still start and use supported RoboPlan features
- **AND** diagnostics identify the unavailable feature without failing unrelated planning behavior.

#### Scenario: Required optional feature fails fast
- **GIVEN** a RoboPlan-selected manipulation stack configured to require an optional RoboPlan feature
- **WHEN** that required feature is unavailable
- **THEN** startup or the relevant request fails with an actionable diagnostic
- **AND** DimOS does not fall back silently to a different backend or different feature semantics.

### Requirement: RoboPlan trajectory execution compatibility
DimOS SHALL preserve existing manipulation trajectory storage and execution behavior for RoboPlan-planned motions.

#### Scenario: Execute RoboPlan-planned trajectory
- **GIVEN** a successful RoboPlan planning result stored by the manipulation module
- **WHEN** a caller requests execution through the existing manipulation execution surface
- **THEN** DimOS sends a normalized trajectory through the existing coordinator execution path
- **AND** RoboPlan does not bypass coordinator task ownership, joint-name translation, or execution state handling.

#### Scenario: Viser renders stored path data
- **GIVEN** a successful RoboPlan planning result stored by the manipulation module
- **WHEN** an operator opens the manipulation Viser panel
- **THEN** the panel can render the stored path through manipulation path-query APIs
- **AND** the RoboPlan backend is not required to provide native preview or visualization APIs.

#### Scenario: Invalid or failed plan is not executable
- **GIVEN** a RoboPlan planning request that failed or produced an invalid path
- **WHEN** a caller requests execution
- **THEN** DimOS does not execute a stale or failed RoboPlan result
- **AND** the caller receives a failure indicating that no valid current plan is available.

### Requirement: Replay and simulation safety for RoboPlan
DimOS SHALL support RoboPlan validation in mock, simulation, and replay manipulation flows before real hardware use.

#### Scenario: Replay feeds RoboPlan state
- **GIVEN** a replay or simulated manipulation stack configured to use RoboPlan and providing recorded or simulated joint states
- **WHEN** the stack runs planning requests
- **THEN** RoboPlan uses the replayed or simulated joint state as the current planning state
- **AND** unsupported replayed scene features are reported as diagnostics rather than silently ignored as fully supported.

#### Scenario: Real hardware execution remains supervised by existing execution path
- **GIVEN** a real manipulator stack configured to use RoboPlan
- **WHEN** a planned motion is executed
- **THEN** execution uses the same coordinator-mediated trajectory path as other manipulation backends
- **AND** RoboPlan does not introduce a direct hardware command path.
