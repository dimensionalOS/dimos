## ADDED Requirements

### Requirement: RoboPlan registers configured planning groups
`RoboPlanWorld` SHALL register planning groups from `RobotModelConfig.planning_groups` using the existing public planning-group registry model.

#### Scenario: Planning group metadata is validated against RoboPlan
- **WHEN** a robot is added to `RoboPlanWorld`
- **THEN** the world SHALL query RoboPlan group metadata for each configured planning group
- **AND** the world SHALL validate that the RoboPlan native joint-name set matches the configured planning group's local joint-name set

#### Scenario: Native joint order is preserved as adapter metadata
- **WHEN** RoboPlan returns a planning group's joint names in a different order than the configured public planning group
- **THEN** the world SHALL preserve the RoboPlan native order separately from the public planning group order
- **AND** the world SHALL use name-based conversion at RoboPlan boundaries

### Requirement: RoboPlan SRDF handling is explicit and narrow
`RoboPlanWorld` SHALL either use a caller-provided SRDF or generate only a simple SRDF for a single configured planning group.

#### Scenario: Provided SRDF is used directly
- **WHEN** `RobotModelConfig.srdf_path` is set
- **THEN** `RoboPlanWorld` SHALL pass that SRDF path to RoboPlan scene construction
- **AND** `RoboPlanWorld` SHALL validate configured planning groups against the RoboPlan scene

#### Scenario: Generated SRDF supports one configured group
- **WHEN** `RobotModelConfig.srdf_path` is not set and exactly one planning group is configured
- **THEN** `RoboPlanWorld` SHALL generate an SRDF containing that planning group's name and joints
- **AND** the generated SRDF SHALL retain configured and adjacent-link collision exclusions

#### Scenario: Multi-group generated SRDF is rejected
- **WHEN** `RobotModelConfig.srdf_path` is not set and more than one planning group is configured
- **THEN** `RoboPlanWorld` SHALL reject the robot configuration with a clear error instructing the caller to provide `RobotModelConfig.srdf_path`

### Requirement: RoboPlan world belief remains full robot local state
`RoboPlanContext` SHALL keep robot state in full robot-local joint order while planning-group operations project from that state by name.

#### Scenario: Live joint state sync stores full robot state
- **WHEN** a named driver `JointState` is synced into `RoboPlanWorld`
- **THEN** the world SHALL store the robot state ordered by `RobotModelConfig.joint_names`

#### Scenario: Group operations project from full robot state
- **WHEN** a RoboPlan group operation needs a group vector
- **THEN** the world SHALL project the full robot-local vector into RoboPlan native group order by joint name

### Requirement: RoboPlan native planner supports selected planning groups
`RoboPlanWorld` SHALL implement `PlannerSpec.plan_selected_joint_path(...)` for one selected planning group using RoboPlan native group order.

#### Scenario: Selected group plan uses RoboPlan native order
- **WHEN** `plan_selected_joint_path(...)` is called with one supported planning group and valid global start and goal joint states
- **THEN** `RoboPlanWorld` SHALL convert start and goal into RoboPlan native group order
- **AND** it SHALL call RoboPlan RRT with `RRTOptions.group_name` set to the selected group name
- **AND** it SHALL construct RoboPlan `JointConfiguration` values with native joint names and native-order positions

#### Scenario: Selected group plan returns public order
- **WHEN** RoboPlan returns a successful native path
- **THEN** `RoboPlanWorld` SHALL return `PlanningResult.path` as `JointState` waypoints named with the selected group's public global joint names
- **AND** waypoint positions SHALL be ordered by the public selection order

#### Scenario: Unsupported selections fail explicitly
- **WHEN** `plan_selected_joint_path(...)` is called with multiple planning groups, multiple robots, or a selection that does not exactly match one configured group
- **THEN** `RoboPlanWorld` SHALL return `PlanningStatus.UNSUPPORTED` with an explanatory message

### Requirement: RoboPlan group FK and Jacobian use selected group semantics
`RoboPlanWorld` SHALL implement group-scoped FK and Jacobian methods using the selected planning group's target frame and native joint order.

#### Scenario: Group FK uses target frame
- **WHEN** `get_group_ee_pose(ctx, group_id)` is called for a group with a target frame
- **THEN** `RoboPlanWorld` SHALL project context state into the group's native order
- **AND** it SHALL query RoboPlan forward kinematics for the group's target frame

#### Scenario: Group Jacobian uses target frame and group columns
- **WHEN** `get_group_jacobian(ctx, group_id)` is called for a group with a target frame
- **THEN** `RoboPlanWorld` SHALL return a 6xN Jacobian whose columns correspond to the public group's local joint order

### Requirement: RoboPlan exposes only current group-oriented planning contracts
`RoboPlanWorld` SHALL implement the current group-oriented `PlannerSpec` and `WorldSpec` surface without adding RoboPlan-specific legacy compatibility wrappers.

#### Scenario: Factory validates selected-group planner contract
- **WHEN** RoboPlan is selected as a planner backend
- **THEN** planner creation SHALL validate support for `plan_selected_joint_path(...)`
- **AND** it SHALL not rely on RoboPlan-specific `plan_joint_path(...)` compatibility behavior

#### Scenario: Protocol validation does not force RoboPlan legacy wrappers
- **WHEN** protocol or factory validation is updated for RoboPlan
- **THEN** RoboPlan SHALL be validated through the group-oriented planner/world methods used by the current codebase
- **AND** RoboPlan SHALL not be required to keep robot-scoped compatibility methods solely to satisfy stale validation

#### Scenario: Group world queries are the RoboPlan FK and Jacobian interface
- **WHEN** callers need RoboPlan FK or Jacobian data
- **THEN** callers SHALL use `get_group_ee_pose(...)` or `get_group_jacobian(...)`
- **AND** the RoboPlan adapter SHALL not add robot-scoped FK/Jacobian compatibility requirements beyond the current `WorldSpec`
