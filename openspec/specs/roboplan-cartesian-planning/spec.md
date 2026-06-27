## Purpose

Define planner-level Cartesian path planning for RoboPlan-backed planners, including absolute Cartesian goals, relative Cartesian deltas, and linear TCP path mode.

## Requirements

### Requirement: PlannerSpec exposes Cartesian path planning
`PlannerSpec` SHALL expose `plan_cartesian_path(world: WorldSpec, request: CartesianPlanningRequest) -> PlanningResult` for planner-level Cartesian TCP requests.

#### Scenario: Planner receives Cartesian request object
- **WHEN** a caller has a selected planning group, start joint state, and Cartesian target
- **THEN** the caller SHALL be able to invoke `plan_cartesian_path(...)` with one `CartesianPlanningRequest`
- **AND** the planner SHALL return a `PlanningResult`

#### Scenario: Cartesian planning does not replace joint planning
- **WHEN** a caller already has a joint-space goal
- **THEN** the caller SHALL continue to use `plan_selected_joint_path(...)`
- **AND** `plan_cartesian_path(...)` SHALL be reserved for TCP-space targets or deltas

### Requirement: CartesianPlanningRequest defines the public API
The system SHALL define `CartesianPlanningRequest` with fields `selection: PlanningGroupSelection`, `group_id: PlanningGroupID`, `start: JointState`, `target: PoseStamped | CartesianDelta`, `target_mode: Literal["absolute", "relative"]`, `path_mode: Literal["free", "linear"] = "free"`, `reference_frame: str = "world"`, `max_translation_step: float = 0.01`, `max_rotation_step: float = 0.05`, and `timeout: float = 10.0`.

#### Scenario: Request names the planned joints and active TCP group
- **WHEN** a Cartesian request is constructed
- **THEN** `selection` SHALL define the selected global joints that are planned and returned
- **AND** `group_id` SHALL identify the planning group's TCP/target frame that receives the Cartesian target

#### Scenario: Request controls path mode
- **WHEN** a Cartesian request sets `path_mode` to `"free"` or `"linear"`
- **THEN** the planner SHALL interpret that value as the requested Cartesian path semantics

### Requirement: CartesianDelta defines relative Cartesian targets
The system SHALL define `CartesianDelta` with fields `translation: tuple[float, float, float] = (0.0, 0.0, 0.0)`, `rotation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)`, and `frame_id: str = "world"`.

#### Scenario: Relative target uses delta model
- **WHEN** `CartesianPlanningRequest.target_mode` is `"relative"`
- **THEN** `target` SHALL be a `CartesianDelta`
- **AND** translation SHALL be interpreted in meters
- **AND** rotation SHALL be interpreted as roll, pitch, yaw in radians

#### Scenario: Absolute target uses stamped pose
- **WHEN** `CartesianPlanningRequest.target_mode` is `"absolute"`
- **THEN** `target` SHALL be a `PoseStamped`

### Requirement: Free Cartesian mode plans to the target pose
For `path_mode="free"`, the planner SHALL plan a collision-free selected-joint path whose final TCP pose satisfies the Cartesian target without requiring straight-line TCP motion between start and target.

#### Scenario: Absolute free Cartesian target
- **WHEN** a RoboPlan-backed planner receives an absolute Cartesian request with `path_mode="free"`
- **THEN** it SHALL plan to the requested final TCP pose if supported and feasible
- **AND** it SHALL return selected global-joint waypoints in `PlanningResult.path`

#### Scenario: Relative free Cartesian target
- **WHEN** a RoboPlan-backed planner receives a relative Cartesian request with `path_mode="free"`
- **THEN** it SHALL compute the start TCP pose from `request.start` and `request.group_id`
- **AND** it SHALL apply `CartesianDelta` to derive the final target pose before planning

### Requirement: Linear Cartesian mode preserves straight-line TCP intent
For `path_mode="linear"`, the planner SHALL require or request straight-line TCP motion from the start TCP pose to the target TCP pose according to the request interpolation limits.

#### Scenario: Linear mode succeeds only for linear TCP path
- **WHEN** a RoboPlan-backed planner returns success for `path_mode="linear"`
- **THEN** the returned joint waypoints SHALL represent a TCP path that honors the requested straight-line Cartesian segment within the request interpolation limits

#### Scenario: Linear mode is not silently downgraded
- **WHEN** the planner cannot support or verify linear TCP path semantics
- **THEN** it SHALL return `PlanningStatus.UNSUPPORTED` or a non-success planning status
- **AND** it SHALL NOT return a successful free-space joint path as if it satisfied linear mode

### Requirement: Unsupported planners fail explicitly
Planners that do not support Cartesian planning SHALL return `PlanningStatus.UNSUPPORTED` from `plan_cartesian_path(...)` with an explanatory message.

#### Scenario: Generic joint planner receives Cartesian request
- **WHEN** a non-Cartesian planner receives `plan_cartesian_path(...)`
- **THEN** it SHALL return `PlanningStatus.UNSUPPORTED`
- **AND** it SHALL NOT perform an implicit IK-to-joint-plan fallback

### Requirement: RoboPlanWorld implements Cartesian planning as PlannerSpec
`RoboPlanWorld` SHALL implement `plan_cartesian_path(...)` because it is the RoboPlan-backed object that implements both `WorldSpec` and `PlannerSpec`.

#### Scenario: RoboPlanWorld returns public joint-state path
- **WHEN** RoboPlan-backed Cartesian planning succeeds
- **THEN** `RoboPlanWorld` SHALL convert the native path into `PlanningResult.path` using public global joint names in the caller's selected order

#### Scenario: RoboPlan native details stay internal
- **WHEN** callers use `plan_cartesian_path(...)`
- **THEN** callers SHALL NOT need to import or pass `roboplan.*` objects
- **AND** RoboPlan-specific options SHALL remain inside the RoboPlan adapter/configuration

#### Scenario: RoboPlan request validation fails clearly
- **WHEN** `target_mode`, `target`, `group_id`, selected joints, or interpolation limits are invalid or unsupported
- **THEN** `RoboPlanWorld` SHALL return an appropriate non-success `PlanningResult` with an explanatory message
