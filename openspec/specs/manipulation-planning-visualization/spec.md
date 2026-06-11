## Purpose

Define optional manipulation planning visualization behavior separately from world planning behavior, including Meshcat-style URL lookup, preview animation, preview dismissal, and safe no-op behavior when visualization is unavailable.

## Requirements

### Requirement: Optional planning visualization remains available independently from world planning behavior

The manipulation planning stack SHALL expose visualization behavior through a dedicated planning visualization capability without requiring backend-agnostic planning code to depend on visualization methods.

#### Scenario: Visualization enabled
- **GIVEN** a manipulation planning stack with visualization enabled
- **WHEN** a caller asks for the visualization URL
- **THEN** the stack returns the active visualization URL when one is available
- **AND** planning, collision checking, and kinematics callers can continue to depend on world planning behavior separately from visualization behavior

#### Scenario: Visualization disabled
- **GIVEN** a manipulation planning stack with visualization disabled
- **WHEN** visualization publishing or URL lookup is requested
- **THEN** the request completes without disrupting planning behavior
- **AND** URL lookup returns no URL

### Requirement: Planned paths can be previewed through the visualization capability

The manipulation planning stack SHALL support previewing a planned joint path through the visualization capability when visualization is available.

#### Scenario: Preview planned path
- **GIVEN** a planned manipulation path and visualization enabled
- **WHEN** a caller previews the path
- **THEN** the visualization animates the planned path using the preview representation
- **AND** the planned path remains available for later execution

#### Scenario: Dismiss stale preview before replanning
- **GIVEN** a preview representation is visible for a robot
- **WHEN** a new plan starts for that robot
- **THEN** the stack can hide the stale preview through visualization behavior before planning the new path
- **AND** the hide operation does not require callers to depend on a concrete world backend

### Requirement: Preview playback follows trajectory timing by default

The manipulation planning stack SHALL use generated trajectory timing for preview playback by default and SHALL allow callers to override playback duration for visual inspection.

#### Scenario: Default preview duration uses generated trajectory duration
- **GIVEN** a planned manipulation path and a generated timed trajectory for the same robot
- **WHEN** a caller previews the path without specifying a duration
- **THEN** the visualization animates the preview over the generated trajectory duration
- **AND** preview playback does not change the stored path or timed trajectory used for execution

#### Scenario: Explicit preview duration overrides playback timing
- **GIVEN** a planned manipulation path and a generated timed trajectory for the same robot
- **WHEN** a caller previews the path with an explicit duration
- **THEN** the visualization uses the explicit duration for preview playback
- **AND** the explicit preview duration does not alter the generated trajectory duration used for execution

#### Scenario: Target FPS controls preview frame density
- **GIVEN** a planned manipulation path whose waypoint count is sparse for the preview duration
- **WHEN** a caller previews the path with a target FPS
- **THEN** the visualization densifies displayed preview waypoints to meet the nominal target frame count
- **AND** preview frame density does not require a separate public joint-space interval setting

### Requirement: Visualization routing preserves existing robot safety behavior

Visualization behavior SHALL NOT change robot command execution, collision checking, or planned trajectory generation.

#### Scenario: Execute after preview
- **GIVEN** a planned path has been previewed
- **WHEN** the caller executes the path
- **THEN** execution uses the existing planned trajectory behavior
- **AND** visualization state does not authorize, alter, or bypass robot motion checks

#### Scenario: Planning without visualization
- **GIVEN** visualization is unavailable
- **WHEN** a caller plans to a pose or joint target
- **THEN** planning behavior remains available according to the configured world, kinematics, and planner behavior
- **AND** missing visualization does not cause planning failure by itself
