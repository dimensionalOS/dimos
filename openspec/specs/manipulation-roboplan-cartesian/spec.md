## Purpose

Define planner-level Cartesian path planning for RoboPlan-backed planners, including absolute Cartesian goals, relative Cartesian deltas, explicit auxiliary planning groups, and linear TCP path mode.

## Requirements

### Requirement: Cartesian planner validates target and auxiliary coverage
The system SHALL require Cartesian planner calls to partition the selected planning groups into targeted groups and explicit auxiliary planning groups.

#### Scenario: Target and auxiliary groups exactly cover selection
- **WHEN** a Cartesian planner call provides a `PlanningGroupSelection`, target map, and auxiliary group list
- **THEN** the target group IDs and auxiliary group IDs SHALL be disjoint
- **AND** their union SHALL equal `set(selection.group_ids)`

#### Scenario: Selected group is neither targeted nor auxiliary
- **WHEN** a selected planning group is absent from both the target map and `auxiliary_groups`
- **THEN** the planner SHALL return a non-success `PlanningResult` with an explanatory validation message

#### Scenario: Target group is not selected
- **WHEN** a target map contains a planning group ID that is not in `selection.group_ids`
- **THEN** the planner SHALL return a non-success `PlanningResult` with an explanatory validation message

#### Scenario: Auxiliary group has a target
- **WHEN** a planning group ID appears in both the target map and `auxiliary_groups`
- **THEN** the planner SHALL return a non-success `PlanningResult` with an explanatory validation message

### Requirement: PlannerSpec exposes Cartesian path planning
`PlannerSpec` SHALL expose direct Cartesian planning methods for absolute and relative task-space goals: `plan_cartesian_path(world: WorldSpec, selection: PlanningGroupSelection, start: JointState, pose_targets: Mapping[PlanningGroupID, PoseStamped], *, auxiliary_groups: Sequence[PlanningGroupID] = (), path_mode: CartesianPathMode = "free", timeout: float = 10.0) -> PlanningResult` and `plan_relative_cartesian_path(world: WorldSpec, selection: PlanningGroupSelection, start: JointState, delta_targets: Mapping[PlanningGroupID, CartesianDelta], *, auxiliary_groups: Sequence[PlanningGroupID] = (), path_mode: CartesianPathMode = "free", timeout: float = 10.0) -> PlanningResult`.

#### Scenario: Planner receives direct absolute Cartesian parameters
- **WHEN** a caller has a selected planning group set, selected start joint state, and one or more absolute Cartesian pose targets
- **THEN** the caller SHALL be able to invoke `plan_cartesian_path(...)` without constructing a request dataclass
- **AND** the planner SHALL return a `PlanningResult`

#### Scenario: Planner receives direct relative Cartesian parameters
- **WHEN** a caller has a selected planning group set, selected start joint state, and one or more relative Cartesian delta targets
- **THEN** the caller SHALL be able to invoke `plan_relative_cartesian_path(...)` without constructing a request dataclass
- **AND** the planner SHALL return a `PlanningResult`

#### Scenario: Cartesian planning does not replace joint planning
- **WHEN** a caller already has a joint-space goal
- **THEN** the caller SHALL continue to use `plan_selected_joint_path(...)`
- **AND** the existing joint-space planner method signatures SHALL remain unchanged

#### Scenario: Selection defines planned joint order
- **WHEN** a Cartesian planner call provides a `PlanningGroupSelection`
- **THEN** `selection` SHALL define the planned groups, selected global joints, and returned joint ordering
- **AND** target map insertion order SHALL NOT define selected joint ordering

### Requirement: CartesianDelta defines relative Cartesian targets
The system SHALL define `CartesianDelta` with fields `translation: tuple[float, float, float] = (0.0, 0.0, 0.0)`, `rotation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)`, and `frame_id: str = "world"`.

#### Scenario: Relative target uses delta model
- **WHEN** a caller invokes `plan_relative_cartesian_path(...)`
- **THEN** each value in `delta_targets` SHALL be a `CartesianDelta`
- **AND** translation SHALL be interpreted in meters
- **AND** rotation SHALL be interpreted as roll, pitch, yaw in radians

#### Scenario: Absolute target uses stamped pose
- **WHEN** a caller invokes `plan_cartesian_path(...)`
- **THEN** each value in `pose_targets` SHALL be a `PoseStamped`

#### Scenario: Relative delta frame is unsupported outside world
- **WHEN** a relative Cartesian planner call includes a `CartesianDelta` whose `frame_id` is not `"world"`
- **THEN** the planner SHALL return `PlanningStatus.UNSUPPORTED` or another non-success `PlanningResult` explaining that non-world relative frames are unsupported

#### Scenario: Absolute pose frame is unsupported outside world
- **WHEN** an absolute Cartesian planner call includes a `PoseStamped` whose `frame_id` is not `""` or `"world"`
- **THEN** the planner SHALL return `PlanningStatus.UNSUPPORTED` or another non-success `PlanningResult` explaining that non-world absolute frames are unsupported

### Requirement: Free Cartesian mode plans to the target pose
For `path_mode="free"`, the planner SHALL plan a collision-free selected-joint path whose final targeted TCP poses satisfy the Cartesian targets without requiring straight-line TCP motion between start and target.

#### Scenario: Absolute free Cartesian targets
- **WHEN** a RoboPlan-backed planner receives an absolute Cartesian request with `path_mode="free"`
- **THEN** it SHALL plan to all requested final TCP poses if supported and feasible
- **AND** it SHALL return selected global-joint waypoints in `PlanningResult.path`

#### Scenario: Relative free Cartesian targets
- **WHEN** a RoboPlan-backed planner receives a relative Cartesian request with `path_mode="free"`
- **THEN** it SHALL compute each targeted start TCP pose from `start`
- **AND** it SHALL apply each `CartesianDelta` to derive final absolute target poses before planning

#### Scenario: Free mode has no path-shape guarantee
- **WHEN** a planner returns success for `path_mode="free"`
- **THEN** the final targeted TCP poses SHALL satisfy the requested Cartesian targets
- **AND** intermediate waypoints SHALL NOT be required to preserve Cartesian target constraints or straight-line TCP motion

### Requirement: Linear Cartesian mode preserves straight-line TCP intent
For `path_mode="linear"`, the planner SHALL require or request straight-line TCP motion from the start TCP pose to the target TCP pose. In v1, RoboPlanWorld SHALL support linear mode for exactly one targeted TCP group and SHALL reject multi-target linear requests.

RoboPlanWorld SHALL implement single-target linear mode through RoboPlan Oink task-space IK over sampled straight-line TCP waypoints. It SHALL NOT use RoboPlan SimpleIk for linear mode, and it SHALL NOT fall back to free-space joint planning when Oink cannot satisfy the sampled path.

#### Scenario: Single-target linear mode is supported
- **WHEN** a RoboPlan-backed Cartesian planner call has `path_mode="linear"` and exactly one targeted TCP group
- **THEN** the planner SHALL attempt a straight-line Cartesian TCP path for that target
- **AND** RoboPlanWorld SHALL use Oink task-space IK to solve the sampled Cartesian waypoints
- **AND** a successful result SHALL contain selected global-joint waypoints whose TCP samples follow the requested Cartesian segment

#### Scenario: Linear mode succeeds only for linear TCP path
- **WHEN** a RoboPlan-backed planner returns success for `path_mode="linear"`
- **THEN** the returned joint waypoints SHALL represent a TCP path that honors the requested straight-line Cartesian segment

#### Scenario: Linear mode is not silently downgraded
- **WHEN** the planner cannot support or verify linear TCP path semantics
- **THEN** it SHALL return `PlanningStatus.UNSUPPORTED` or a non-success planning status
- **AND** it SHALL NOT return a successful free-space joint path as if it satisfied linear mode

#### Scenario: Multi-target linear mode is unsupported in v1
- **WHEN** a Cartesian planner call has `path_mode="linear"` and more than one targeted TCP group
- **THEN** the planner SHALL return `PlanningStatus.UNSUPPORTED` or another non-success status
- **AND** it SHALL NOT select an implicit primary target group

### Requirement: Unsupported planners fail explicitly
Planners that do not support Cartesian planning SHALL return `PlanningStatus.UNSUPPORTED` from both `plan_cartesian_path(...)` and `plan_relative_cartesian_path(...)` with explanatory messages.

#### Scenario: Generic joint planner receives absolute Cartesian request
- **WHEN** a non-Cartesian planner receives `plan_cartesian_path(...)`
- **THEN** it SHALL return `PlanningStatus.UNSUPPORTED`
- **AND** it SHALL NOT perform an implicit IK-to-joint-plan fallback

#### Scenario: Generic joint planner receives relative Cartesian request
- **WHEN** a non-Cartesian planner receives `plan_relative_cartesian_path(...)`
- **THEN** it SHALL return `PlanningStatus.UNSUPPORTED`
- **AND** it SHALL NOT perform an implicit IK-to-joint-plan fallback

### Requirement: RoboPlanWorld implements Cartesian planning as PlannerSpec
`RoboPlanWorld` SHALL implement the revised absolute and relative Cartesian planning methods because it is the RoboPlan-backed object that implements both `WorldSpec` and `PlannerSpec`.

#### Scenario: RoboPlanWorld returns public joint-state path
- **WHEN** RoboPlan-backed Cartesian planning succeeds
- **THEN** `RoboPlanWorld` SHALL convert the native path into `PlanningResult.path` using public global joint names in the caller's selected order

#### Scenario: RoboPlan native details stay internal
- **WHEN** callers use `plan_cartesian_path(...)` or `plan_relative_cartesian_path(...)`
- **THEN** callers SHALL NOT need to import or pass `roboplan.*` objects
- **AND** RoboPlan-specific options SHALL remain inside the RoboPlan adapter/configuration

#### Scenario: RoboPlan request validation fails clearly
- **WHEN** target maps, auxiliary groups, selected joints, target frames, start state, or path mode are invalid or unsupported
- **THEN** `RoboPlanWorld` SHALL return an appropriate non-success `PlanningResult` with an explanatory message

#### Scenario: RoboPlan free mode uses terminal IK then joint planning
- **WHEN** RoboPlan-backed free Cartesian planning is requested
- **THEN** `RoboPlanWorld` MAY solve a final joint goal satisfying all Cartesian targets, run RoboPlan RRT from `start` to that goal, and verify final TCP poses before returning success

### Requirement: RoboPlan linear Cartesian planning returns constraint metadata
RoboPlan linear Cartesian planning SHALL return enough path-constraint metadata for downstream trajectory post-processing to validate straight-line TCP preservation.

#### Scenario: Linear Cartesian result includes TCP segment metadata
- **WHEN** RoboPlan-backed Cartesian planning succeeds with `path_mode="linear"`
- **THEN** the returned planning result MUST include metadata for the constrained planning group, constrained TCP frame, start TCP pose, target TCP pose, and Cartesian deviation tolerances

#### Scenario: Linear Cartesian metadata matches validated path
- **WHEN** RoboPlanWorld validates a linear Cartesian path before returning success
- **THEN** the returned path-constraint metadata MUST describe the same start and target TCP segment that was validated

#### Scenario: Free Cartesian result omits linear constraint metadata
- **WHEN** RoboPlan-backed Cartesian planning succeeds with `path_mode="free"`
- **THEN** the returned planning result MUST NOT include linear TCP path-constraint metadata

### Requirement: RoboPlan post-processing can validate linear TCP constraints
RoboPlanWorld SHALL provide the backend operations needed for trajectory post-processing to validate linear TCP constraints against candidate geometric paths.

#### Scenario: Candidate path is checked against TCP line
- **WHEN** RoboPlan trajectory post-processing validates a candidate path with linear TCP constraint metadata
- **THEN** RoboPlanWorld MUST evaluate the constrained TCP poses along the candidate path and compare them with the declared Cartesian line tolerance

#### Scenario: Candidate path cannot be validated
- **WHEN** RoboPlanWorld cannot evaluate the constrained TCP poses or constraint metadata is inconsistent with the selected planning groups
- **THEN** the candidate path MUST be rejected and non-blocking smoothing fallback MUST apply
