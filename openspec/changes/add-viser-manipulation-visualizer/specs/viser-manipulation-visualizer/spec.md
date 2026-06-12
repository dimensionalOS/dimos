## ADDED Requirements

### Requirement: Viser manipulation visualization can be selected explicitly

The manipulation planning stack SHALL support a Viser-backed visualization backend when Viser visualization is explicitly selected and its optional dependencies are available.

#### Scenario: Select Viser backend
- **GIVEN** a manipulation planning stack configured with the Viser visualization backend
- **WHEN** the stack starts with valid robot visualization metadata
- **THEN** the stack exposes a Viser visualization URL when the Viser server is running
- **AND** planning behavior remains available through the configured manipulation planner and world backend

#### Scenario: Missing optional Viser dependency
- **GIVEN** a manipulation planning stack configured with the Viser visualization backend
- **AND** the optional Viser dependencies are not installed
- **WHEN** the stack starts visualization
- **THEN** startup fails with a clear message describing the missing Viser dependency and installation extra
- **AND** Meshcat and no-visualization configurations do not require the Viser dependency

### Requirement: Viser renders robot state, preview state, and planned paths

The Viser backend SHALL visualize configured manipulation robots, their current joint state when available, a preview representation, and planned path animation through the manipulation visualization behavior.

#### Scenario: Current robot state is published
- **GIVEN** a Viser visualization backend with a configured robot
- **AND** the manipulation stack has received a current joint state for that robot
- **WHEN** visualization publishing occurs
- **THEN** the Viser scene reflects the robot's current joint configuration
- **AND** publishing does not change the stored planned path or robot command state

#### Scenario: Preview representation visibility is controlled
- **GIVEN** a Viser visualization backend with a configured robot
- **WHEN** a caller requests preview visibility for that robot
- **THEN** the Viser scene shows the preview representation for that robot
- **WHEN** a caller hides preview visibility for that robot
- **THEN** the Viser scene hides the preview representation without disrupting planning behavior

#### Scenario: Planned path is animated
- **GIVEN** a planned manipulation joint path and an active Viser visualization backend
- **WHEN** a caller previews the planned path
- **THEN** Viser animates the preview representation along the path
- **AND** the planned path remains available for later execution through existing manipulation behavior

### Requirement: Viser panel preserves manipulation operator controls

The Viser backend SHALL optionally provide the manipulation panel behavior from the earlier Viser prototype, including status display, target controls, preview planning, plan preview, cancellation, clearing, and guarded execution.

#### Scenario: Panel displays manipulation state
- **GIVEN** the Viser panel is enabled
- **WHEN** the manipulation state, robot selection, current joints, planned path, or error state changes
- **THEN** the panel presents the latest available snapshot of that state
- **AND** stale or unavailable robot state is visible to the operator rather than silently treated as valid

#### Scenario: Panel requests planning through existing manipulation behavior
- **GIVEN** the Viser panel is enabled and the operator requests a target preview or plan
- **WHEN** the request is submitted
- **THEN** the request uses existing manipulation planning and validation behavior
- **AND** it reports success, infeasibility, timeout, or failure through the panel state

#### Scenario: Panel execution requires explicit opt-in
- **GIVEN** the Viser panel is enabled
- **AND** panel execution is not explicitly allowed
- **WHEN** the operator attempts to execute a plan from the panel
- **THEN** the panel refuses to execute the robot command
- **AND** the existing planned path remains available for preview or clearing

#### Scenario: Panel execution uses existing safety gates
- **GIVEN** the Viser panel is enabled
- **AND** panel execution is explicitly allowed
- **AND** a valid planned path is available
- **WHEN** the operator requests execution from the panel
- **THEN** execution uses the existing manipulation execution path
- **AND** visualization state does not bypass planning, collision, or module state checks

### Requirement: Viser visualization reads do not create operator-visible planning side effects

The Viser backend SHALL obtain display data in a way that does not change planned paths, generated trajectories, execution state, collision world contents, or robot commands merely because visualization is publishing or the panel is refreshing.

#### Scenario: Periodic Viser refresh while idle
- **GIVEN** the Viser backend is active and the manipulation module is idle
- **WHEN** the Viser scene or panel refreshes periodically
- **THEN** the refresh updates displayed robot and panel state from the latest available data
- **AND** it does not start planning, run execution, clear stored plans, or modify robot command outputs

#### Scenario: Viser refresh during planning or execution
- **GIVEN** the Viser backend is active
- **AND** manipulation planning or execution is already in progress
- **WHEN** the Viser scene or panel refreshes
- **THEN** the refresh either displays the last safe snapshot or reports that the current state is busy or stale
- **AND** it does not interrupt or re-enter the active planning or execution operation

### Requirement: Viser lifecycle is cleanly managed

The Viser backend SHALL release visualization resources when the manipulation stack stops or when visualization is closed.

#### Scenario: Close Viser visualization
- **GIVEN** a running Viser visualization backend
- **WHEN** the manipulation stack closes visualization
- **THEN** Viser scene updates, panel workers, background refresh loops, and server resources are stopped or released
- **AND** repeated close calls complete without requiring robot hardware motion
