## Purpose

RoboPlan Oink kinematics lets RoboPlan-backed manipulation stacks use the existing RoboPlanWorld scene, planning-group metadata, joint ordering, and collision checks through the DimOS KinematicsSpec surface.

## Requirements

### Requirement: RoboPlan kinematics configuration
The manipulation kinematics configuration SHALL support a `roboplan` backend with typed Oink solver tuning fields.

#### Scenario: Default RoboPlan kinematics config
- **WHEN** a caller requests kinematics config from the legacy name `roboplan`
- **THEN** the system returns a config whose discriminator is `backend="roboplan"` and whose Oink solver fields have deterministic defaults

#### Scenario: RoboPlan kinematics requires RoboPlan world
- **WHEN** backend validation receives `kinematics_name="roboplan"` with any `world_backend` other than `roboplan`
- **THEN** validation fails with a clear error that RoboPlan kinematics requires `world_backend="roboplan"`

### Requirement: RoboPlanWorld acts as KinematicsSpec
When RoboPlan kinematics is selected, the planning factory SHALL return the existing `RoboPlanWorld` instance as the `KinematicsSpec`.

#### Scenario: Factory returns shared RoboPlanWorld for kinematics
- **WHEN** `create_planning_specs` is called with `world_backend="roboplan"` and `kinematics.backend="roboplan"`
- **THEN** the returned kinematics object is the same `RoboPlanWorld` instance passed to the factory

#### Scenario: Planner wiring remains unchanged
- **WHEN** `planner_name="roboplan"` is selected with `world_backend="roboplan"`
- **THEN** the returned planner object remains the same `RoboPlanWorld` instance as before

### Requirement: Oink-backed pose target solving
`RoboPlanWorld.solve_pose_targets` SHALL solve pose targets using RoboPlan Oink tasks and constraints instead of a hand-written DimOS Jacobian pseudoinverse loop.

#### Scenario: Single pose target creates one frame task
- **WHEN** one pose-targeted planning group is provided
- **THEN** RoboPlan IK creates one Oink frame task for that group's target frame and solves through `Oink.solveIk`

#### Scenario: Multiple pose targets create multiple frame tasks
- **WHEN** multiple non-overlapping pose-targeted planning groups are provided and their selection maps to a RoboPlan group or generated composite group
- **THEN** RoboPlan IK creates one Oink frame task per pose target and solves them in one selected-group IK problem

#### Scenario: Missing Oink binding
- **WHEN** the RoboPlan optimal IK binding is unavailable
- **THEN** RoboPlan IK returns or raises an actionable error indicating that `roboplan.optimal_ik` is required for `kinematics.backend="roboplan"`

### Requirement: Planning-group selection semantics for IK
RoboPlan IK SHALL use existing DimOS planning-group selection semantics for pose target and auxiliary groups.

#### Scenario: Overlapping selected groups
- **WHEN** pose target groups or auxiliary groups select overlapping global joints
- **THEN** RoboPlan IK fails clearly without invoking Oink

#### Scenario: Unsupported composite selection
- **WHEN** the non-overlapping selection cannot be represented by a RoboPlan native group or generated composite group
- **THEN** RoboPlan IK returns `IKStatus.NO_SOLUTION` with a message describing the unsupported selection

#### Scenario: Auxiliary group retained
- **WHEN** an auxiliary group is selected without a pose target
- **THEN** RoboPlan IK includes that group's joints in the returned selected joint state using seed or current belief positions

### Requirement: Seed, state, and result ordering
RoboPlan IK SHALL preserve DimOS public global joint ordering at API boundaries while converting to RoboPlan native group order internally.

#### Scenario: Complete seed provided
- **WHEN** the caller provides a complete seed for the selected joints
- **THEN** RoboPlan IK initializes the Oink candidate from the provided seed

#### Scenario: Incomplete or absent seed
- **WHEN** the caller omits seed positions required for selected joints
- **THEN** RoboPlan IK initializes missing selected joints from the RoboPlanWorld current belief state

#### Scenario: Successful IK result order
- **WHEN** RoboPlan IK succeeds
- **THEN** `IKResult.joint_state.name` lists selected global joint names in `PlanningGroupSelection` order and positions correspond to those names

### Requirement: Local IK semantics and final validation
RoboPlan IK SHALL behave as local endpoint inverse kinematics and SHALL not replace motion planning.

#### Scenario: Tolerances reached and final state valid
- **WHEN** Oink iterations reach requested position and orientation tolerances and the final candidate passes configured final validation
- **THEN** RoboPlan IK returns `IKStatus.SUCCESS`

#### Scenario: Iteration budget exhausted
- **WHEN** Oink iterations do not reach requested tolerances within the configured iteration budget
- **THEN** RoboPlan IK returns `IKStatus.NO_SOLUTION` with final error information when available

#### Scenario: Final collision validation fails
- **WHEN** collision checking is enabled and the converged final candidate is colliding
- **THEN** RoboPlan IK returns a failure rather than a colliding endpoint

#### Scenario: IK does not perform path planning
- **WHEN** RoboPlan IK returns a successful endpoint
- **THEN** the system does not imply that a collision-free path to the endpoint exists; callers that need motion SHALL still use a planner
