## ADDED Requirements

### Requirement: Successful planning must materialize one complete generated plan
After `PlannerSpec.plan` returns a successful geometric path, ManipulationModule MUST validate the selected global waypoints and parameterize them exactly once into one synchronized globally named `JointTrajectory`. It MUST cache or expose a `GeneratedPlan` only when both path and trajectory are complete and mutually consistent.

#### Scenario: Selected-group planning succeeds
- **WHEN** a planner returns valid waypoints for the requested planning groups
- **THEN** ManipulationModule stores one generated plan containing the original path and one trajectory over the same selected global joints

#### Scenario: Parameterization fails
- **WHEN** joint limits, waypoint data, or generated trajectory validation fails
- **THEN** planning fails and no generated plan is cached or exposed

#### Scenario: Multiple robots are selected
- **WHEN** selected group joints span multiple robots
- **THEN** one generator call creates one shared relative trajectory clock across all planned joints

### Requirement: Stored trajectory must be preview and execution authority
Preview, execution, completion timing, and status logic MUST consume the stored generated-plan trajectory without re-projecting or re-parameterizing the geometric path.

#### Scenario: Plan is previewed and executed
- **WHEN** a stored generated plan is previewed and later executed
- **THEN** both operations consume the same trajectory points, joint order, and timestamps

#### Scenario: Preview duration is overridden
- **WHEN** a caller requests a different display duration
- **THEN** only visualization playback rate changes and the stored trajectory remains unchanged

### Requirement: Generated trajectories must contain only planned joints
The generated path and trajectory MUST contain exactly the selected planning groups' global joint names in selection order. Joints outside the selected groups MUST NOT be added to the generated plan or execution command.

#### Scenario: Planning group omits a robot joint
- **WHEN** a selected group excludes another configured robot joint
- **THEN** the excluded joint appears in neither the generated trajectory nor the trajectory task command

### Requirement: Manipulation module must own execution freshness
Immediately before dispatch, ManipulationModule MUST compare current planned-joint positions with the stored trajectory's first point within configured tolerance. Missing, malformed, duplicate, reordered, stale, or mismatched planned-joint state MUST reject execution before task dispatch.

#### Scenario: Planned joints still match
- **WHEN** all current planned joints match the trajectory start within tolerance
- **THEN** execution may translate and dispatch the stored trajectory

#### Scenario: Planned joint moved after planning
- **WHEN** any current planned joint differs from the trajectory start beyond tolerance
- **THEN** execution is rejected without relying on frontend snapshots

### Requirement: Execution must translate only at the coordinator boundary
Execution MUST split the stored global trajectory by affected robot without changing timing, translate selected local names to coordinator names at dispatch, and submit the resulting partial trajectory to the existing configured trajectory task.

#### Scenario: Selected joints use coordinator aliases
- **WHEN** a robot config maps selected local joint names to coordinator names
- **THEN** dispatch preserves stored points and timestamps while translating only those selected names

#### Scenario: Plan spans multiple robots
- **WHEN** planned global joints span multiple configured trajectory tasks
- **THEN** each task receives its selected local subset on the same stored relative clock using the existing sequential submission behavior
