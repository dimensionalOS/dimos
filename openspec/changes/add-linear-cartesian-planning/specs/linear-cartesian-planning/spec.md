## ADDED Requirements

### Requirement: Internal linear Cartesian planning operation

The manipulation planner interface SHALL accept a linear Cartesian planning request
containing an ordered planning-group selection, an explicit combined start state,
Cartesian targets keyed by planning-group ID, and optional auxiliary planning-group
IDs. The operation SHALL return the standard planning result type and SHALL NOT require
a timeout argument.

#### Scenario: Complete target and auxiliary coverage

- **GIVEN** a non-empty planning-group selection
- **AND** each selected group appears exactly once as either a target group or an
  auxiliary group
- **WHEN** a caller requests a linear Cartesian path
- **THEN** the planner accepts the request for backend processing

#### Scenario: Invalid group coverage

- **GIVEN** target and auxiliary groups that overlap, omit a selected group, or include
  a group outside the selection
- **WHEN** a caller requests a linear Cartesian path
- **THEN** the result has `INVALID_GOAL` status
- **AND** no path is returned

#### Scenario: No Cartesian target

- **GIVEN** a selection containing only auxiliary groups
- **WHEN** a caller requests a linear Cartesian path
- **THEN** the result has `INVALID_GOAL` status

### Requirement: Absolute and relative world-frame targets

The planner SHALL accept each targeted group's endpoint as either an absolute
world-frame TCP pose or a world-frame Cartesian delta. The planner SHALL resolve every
delta from that group's TCP pose at the explicit planning start state. Delta translation
SHALL use world axes, and delta rotation SHALL be pre-multiplied onto the start
orientation.

#### Scenario: Absolute world target

- **GIVEN** an absolute TCP target whose frame is `world`
- **WHEN** the request is planned
- **THEN** the target is used as that group's Cartesian endpoint

#### Scenario: Relative translation preserves orientation

- **GIVEN** a world-frame Cartesian delta with nonzero translation and zero rotation
- **WHEN** the delta is resolved
- **THEN** its translation is added along world axes
- **AND** the target TCP orientation equals the orientation at the explicit start state

#### Scenario: Relative rotation composition

- **GIVEN** a world-frame Cartesian delta with an XYZ roll-pitch-yaw rotation
- **WHEN** the delta is resolved
- **THEN** the target rotation equals the delta rotation pre-multiplied by the start
  rotation

#### Scenario: Mixed target kinds

- **GIVEN** a multi-group request containing an absolute target for one group and a
  relative target for another
- **WHEN** the request is planned
- **THEN** both endpoints are resolved independently in the same request

#### Scenario: Unsupported target frame

- **GIVEN** an absolute pose or Cartesian delta expressed outside the `world` frame
- **WHEN** the request is planned
- **THEN** the result has `UNSUPPORTED` status
- **AND** the frame is not silently reinterpreted

### Requirement: Linear TCP path semantics

For every targeted group, the planner SHALL produce a TCP path whose position follows
the segment between the start and target positions and whose orientation interpolates
between the start and target rotations within configured tracking tolerances.

#### Scenario: Fixed end-effector orientation

- **GIVEN** a target whose orientation equals the TCP orientation at the start state
- **WHEN** the linear path is planned successfully
- **THEN** the TCP orientation remains within the configured orientation tolerance of
  that orientation throughout the path

#### Scenario: Requested orientation change

- **GIVEN** a target orientation different from the start orientation
- **WHEN** the linear path is planned successfully
- **THEN** the TCP orientation follows spherical interpolation from start to target
  within the configured orientation tolerance

### Requirement: Synchronized multi-group result

The planner SHALL plan all targeted and auxiliary groups as one combined joint system
and SHALL return one synchronized sequence of combined joint states. Multiple targeted
TCP tracks SHALL share one timeline; a track that finishes before another SHALL hold
its target through the remaining samples.

#### Scenario: Tracks with different lengths

- **GIVEN** two targeted groups whose Cartesian paths require different durations
- **WHEN** the planner succeeds
- **THEN** both groups are represented in every combined waypoint on one shared timeline
- **AND** the shorter track holds its target until the longer track completes

#### Scenario: Auxiliary group participation

- **GIVEN** a selected auxiliary group without a Cartesian target
- **WHEN** the planner succeeds
- **THEN** the auxiliary group's joints are included in every combined waypoint
- **AND** no TCP track is required for that group

### Requirement: Timed universal planning result

A successful result SHALL contain globally named combined joint-state waypoints in
selection order, including positions and available velocities, plus a timestamp for
every waypoint. Acceleration values SHALL NOT be stored in the joint-state effort
field.

#### Scenario: Successful timed conversion

- **GIVEN** a backend trajectory containing positions, velocities, accelerations, and
  waypoint times
- **WHEN** it is converted to the standard planning result
- **THEN** every waypoint contains the selected global joint names, positions, and
  velocities
- **AND** the result timestamps preserve the backend waypoint times
- **AND** accelerations are omitted rather than represented as effort

### Requirement: Start-state and collision validation

The planner SHALL reject a start state that is invalid or does not match the
authoritative planning scene. Before reporting success, the planner SHALL validate
every combined waypoint and synchronized interpolated samples along every adjacent
edge against the DimOS planning world.

#### Scenario: Start state differs from the scene

- **GIVEN** an explicit start state that differs from the authoritative scene state
- **WHEN** the request is planned
- **THEN** the result has `INVALID_START` status
- **AND** no path is returned

#### Scenario: Collision-free trajectory

- **GIVEN** a generated trajectory whose combined waypoints and interpolated edges are
  collision-free
- **WHEN** post-validation completes
- **THEN** the result may be reported as `SUCCESS`

#### Scenario: Synchronized multi-robot collision

- **GIVEN** a generated multi-robot trajectory with a collision at a combined waypoint
  or synchronized interpolated edge sample
- **WHEN** post-validation runs
- **THEN** all robots are checked at the positions belonging to the same trajectory
  instant
- **AND** the result has `NO_SOLUTION` status with no path

#### Scenario: No free-space fallback

- **GIVEN** a generated linear Cartesian trajectory that fails collision
  post-validation
- **WHEN** planning finishes
- **THEN** the planner does not substitute a free-space joint path

### Requirement: Explicit unsupported behavior

A planner backend that cannot generate linear Cartesian paths SHALL implement the
operation by returning `UNSUPPORTED` without attempting joint-space approximation.

#### Scenario: Unsupported planner backend

- **GIVEN** a valid linear Cartesian request sent to a planner without Cartesian support
- **WHEN** the operation is called
- **THEN** the result has `UNSUPPORTED` status
- **AND** no path is returned

### Requirement: Typed planner configuration compatibility

Manipulation configuration SHALL accept a backend-discriminated planner configuration.
The RoboPlan variant SHALL contain a typed linear Cartesian subconfiguration for step
time, Cartesian speed and acceleration bounds, tracking errors, joint-limit scaling,
position-limit behavior, and per-step attempts. The existing planner-name setting SHALL
remain accepted as a deprecated compatibility input during this change.

#### Scenario: Typed RoboPlan configuration

- **GIVEN** a RoboPlan planner configuration with linear Cartesian overrides
- **WHEN** the planning stack is created
- **THEN** those values configure the official RoboPlan Cartesian planner
- **AND** bounded-speed mode is used regardless of caller configuration

#### Scenario: Legacy planner name

- **GIVEN** an existing configuration that explicitly sets only the legacy planner name
- **WHEN** the planning stack is created
- **THEN** the name is converted to the corresponding default typed planner
  configuration
- **AND** existing joint planning remains available
