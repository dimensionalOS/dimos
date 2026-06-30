## Purpose
Define RoboPlan-backed composite multi-robot planning behavior as part of the manipulation backend surface.

## Requirements

### Requirement: Multi-robot RoboPlan finalization
`RoboPlanWorld` SHALL support registering multiple robot models before constructing the RoboPlan `Scene`, and SHALL construct one Composite RoboPlan model when two or more robots are registered.

#### Scenario: Finalize two registered robots
- **WHEN** two robot configs are registered with distinct robot names
- **THEN** RoboPlan world finalization creates one RoboPlan `Scene` from a generated composite URDF and generated composite SRDF

#### Scenario: Single robot keeps direct SRDF pass-through
- **WHEN** exactly one robot config is registered with `srdf_path` set
- **THEN** RoboPlan world finalization passes that SRDF path directly to RoboPlan instead of forcing composite SRDF generation

### Requirement: Composite RoboPlan native naming
The system SHALL rewrite RoboPlan-facing link, joint, frame, and planning-group names so every robot-local name is unique in the Composite RoboPlan model, while preserving DimOS public global names at API boundaries.

#### Scenario: Duplicate local joint names across robots
- **WHEN** two registered robots both contain a local joint named `joint1`
- **THEN** the generated RoboPlan model uses distinct native joint names for each robot and the public names remain `robot_name/joint1`

#### Scenario: Path returned from native RoboPlan order
- **WHEN** RoboPlan returns a path in native Composite planning-group order
- **THEN** `PlanningResult.path` contains global `JointState` waypoints in the caller's selected joint order

### Requirement: Composite URDF base placement
The generated composite URDF SHALL attach each registered robot under a synthetic world root using its `RobotModelConfig.base_pose` exactly once.

#### Scenario: Robot has non-identity base pose
- **WHEN** a robot config has a non-identity `base_pose`
- **THEN** the generated composite URDF contains one fixed transform from the synthetic world root to that robot's prefixed base frame

#### Scenario: Model-authored world joint stripping is enabled
- **WHEN** `strip_model_world_joint` is enabled for a robot model
- **THEN** the model-authored world/base fixed joint is removed before applying `RobotModelConfig.base_pose`

### Requirement: Composite SRDF group generation
The generated composite SRDF SHALL include each configured Planning group and every non-overlapping Planning-group combination of size two or greater, ordered by canonical `PlanningGroupRegistry` order.

#### Scenario: Dual-arm manipulator groups
- **WHEN** the registry contains `left_arm/manipulator` and `right_arm/manipulator`
- **THEN** the composite SRDF contains a deterministic Composite planning group containing both groups' native joints

#### Scenario: Caller selection uses a different order
- **WHEN** a caller selects the same groups in a different order than registry order
- **THEN** RoboPlan uses the same generated Composite planning group and DimOS remaps start, goal, and path waypoints at the boundary

### Requirement: Composite group safety cap
`RoboPlanWorld` SHALL enforce a configurable maximum number of generated Composite planning groups and fail finalization clearly when the cap would be exceeded.

#### Scenario: Composite group count exceeds cap
- **WHEN** registered Planning groups would generate more Composite planning groups than `max_generated_composite_groups`
- **THEN** finalization fails with an error explaining the cap and the number of groups that would be generated

### Requirement: Collision disable preservation
The generated composite SRDF SHALL preserve configured per-robot collision exclusions using RoboPlan-native prefixed link names and SHALL leave inter-robot collisions enabled unless explicitly configured otherwise.

#### Scenario: Configured self-collision exclusion
- **WHEN** a robot config contains a collision exclusion pair for two local links
- **THEN** the generated composite SRDF contains the equivalent disable-collisions entry using that robot's prefixed native link names

#### Scenario: No inter-robot exclusion configured
- **WHEN** two robots are registered without explicit inter-robot collision exclusions
- **THEN** the generated composite SRDF does not disable collisions between their links

### Requirement: Planning world state drives RoboPlan current state
Before invoking RoboPlan-native group RRT, `RoboPlanWorld` SHALL set the RoboPlan `Scene` full current joint positions from the Planning world's authoritative belief state.

#### Scenario: Planning selected group while other robot is fixed
- **WHEN** a selected group is planned while another registered robot is not selected
- **THEN** RoboPlan receives full current joint positions containing the non-selected robot's current Planning world state before RRT planning starts

### Requirement: Start state consistency validation
`plan_selected_joint_path` SHALL reject a selected `start` state that disagrees with the Planning world's current selected state beyond the configured tolerance.

#### Scenario: Start differs from Planning world
- **WHEN** the selected `start` joint positions differ from the Planning world's selected positions beyond tolerance
- **THEN** `plan_selected_joint_path` returns `PlanningStatus.INVALID_START` without invoking RoboPlan RRT

#### Scenario: Start matches Planning world
- **WHEN** the selected `start` joint positions match the Planning world's selected positions within tolerance
- **THEN** `plan_selected_joint_path` may invoke RoboPlan RRT using the selected start and goal

### Requirement: RoboPlan-native selected planning inputs
`plan_selected_joint_path` SHALL call RoboPlan RRT with `JointConfiguration` start and goal values in the selected RoboPlan group's native joint order and SHALL set RoboPlan planner options explicitly.

#### Scenario: Composite selected planning request
- **WHEN** a selected planning request maps to a generated Composite planning group
- **THEN** RoboPlan RRT receives `RRTOptions.group_name` for that generated group and `JointConfiguration` start and goal values in native group order

#### Scenario: Explicit planner options
- **WHEN** RoboPlan RRT options are created
- **THEN** DimOS sets behavior-affecting options such as planning timeout and collision checking mode explicitly instead of relying on binding defaults

### Requirement: Joint limits use group native order
RoboPlan joint-limit extraction SHALL validate native group joint names by name and reorder limits into the DimOS public or robot-local order required by the caller.

#### Scenario: RoboPlan returns limits in native order
- **WHEN** RoboPlan reports position limits for a group in native joint order
- **THEN** DimOS validates the joint-name set and reorders limits by name before exposing them through DimOS APIs

### Requirement: Unsupported selections fail without partial planning
`plan_selected_joint_path` SHALL return `PlanningStatus.UNSUPPORTED` for selections that cannot be represented by a single generated RoboPlan group.

#### Scenario: Overlapping groups are selected
- **WHEN** a selection contains overlapping joints or groups that were not generated as one Composite planning group
- **THEN** `plan_selected_joint_path` returns `PlanningStatus.UNSUPPORTED` and does not invoke RoboPlan RRT

### Requirement: Dynamic obstacles remain scene state
Dynamic obstacles SHALL remain Planning world and RoboPlan scene state, not baked into the generated composite URDF or SRDF.

#### Scenario: Obstacle changes after finalization
- **WHEN** obstacle geometry changes after RoboPlan world finalization
- **THEN** the change is represented through RoboPlan scene/world state updates rather than by regenerating the composite URDF/SRDF
