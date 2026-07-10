## MODIFIED Requirements

### Requirement: Manipulation module must expose explicit group planning APIs
The manipulation module MUST allow callers to target planning groups explicitly for joint and pose planning operations. A successful public planning operation MUST expose and cache only a fully generated plan containing the selected geometric waypoints and a synchronized trajectory.

#### Scenario: Caller plans to a group pose
- **WHEN** a caller submits a pose target for a valid group ID and geometric planning and parameterization succeed
- **THEN** the module stores a successful `GeneratedPlan` with populated `waypoints` and `trajectory`

#### Scenario: Parameterization fails after geometric planning
- **WHEN** a planner returns successful waypoints but the module cannot parameterize them
- **THEN** the public planning operation fails and no generated plan is cached or exposed

## ADDED Requirements

### Requirement: Generated plans must contain one synchronized planned-joint trajectory
The manipulation module MUST parameterize all selected planning-group joints together into one `JointTrajectory` with globally qualified joint names and one relative clock. The trajectory MUST contain exactly the planned joints in the same canonical order as every geometric waypoint, and MUST omit joints outside the selected planning groups.

#### Scenario: Plan affects groups on two robots
- **WHEN** geometric planning returns waypoints for selected groups on two robots
- **THEN** the generated plan contains one trajectory whose points represent both groups on the same time axis

#### Scenario: Robot has an unselected controllable joint
- **WHEN** a selected planning group omits another controllable joint on the same robot
- **THEN** the generated trajectory omits that joint and execution does not command it

#### Scenario: Planned joint limits differ
- **WHEN** planned joints require different segment durations under their configured limits
- **THEN** each shared segment duration accommodates the slowest required planned joint

### Requirement: Preview and execution must consume stored generated-plan timing
Preview and execution MUST use the populated synchronized trajectory stored in `GeneratedPlan` and MUST NOT project or parameterize `GeneratedPlan.waypoints` into a replacement motion.

#### Scenario: Generated plan is previewed
- **WHEN** preview is requested for a successful generated plan
- **THEN** the visualizer animates the stored trajectory according to its relative point timing

#### Scenario: Preview duration is overridden
- **WHEN** preview requests a display duration different from the stored trajectory duration
- **THEN** playback is scaled without mutating the generated plan or replacing its trajectory timing

#### Scenario: Generated plan is executed
- **WHEN** execution is requested for a successful generated plan
- **THEN** the module partitions its stored global trajectory by affected robot and task while preserving relative timestamps

#### Scenario: Incomplete plan reaches a consumer
- **WHEN** preview or execution receives an unsuccessful generated plan or a plan without a trajectory
- **THEN** the operation rejects it without deriving timing from waypoints

### Requirement: Trajectory tasks must preserve planned-joint subsets
Trajectory-task execution MUST validate incoming joint names and point dimensions against the task configuration and MUST emit commands only for the joints named by the active trajectory.

#### Scenario: Task receives a valid joint subset
- **WHEN** a trajectory names a unique subset of the task's configured joints and every point has matching dimensions
- **THEN** the task emits commands for exactly that subset

#### Scenario: Task receives malformed joint names
- **WHEN** a trajectory contains unknown or duplicate joint names or mismatched point dimensions
- **THEN** the task rejects the trajectory before execution
