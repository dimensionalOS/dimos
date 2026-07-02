## Purpose

Specify world and monitor support for planning-group queries.

## Requirements

### Requirement: World backends must answer group pose queries
World backends MUST provide end-effector pose queries for pose-targetable planning groups using the group's configured target frame.

#### Scenario: Group pose is requested
- **WHEN** a caller requests FK for a valid pose-targetable group ID
- **THEN** the backend returns the pose of that group's `tip_link`

### Requirement: World backends must return group-ordered Jacobians
World backends MUST return Jacobian columns ordered according to the requested group's local joint names.

#### Scenario: Backend returns a full robot Jacobian
- **WHEN** a group contains a subset of robot joints in a custom order
- **THEN** the backend projects and reorders Jacobian columns to match the group

### Requirement: Monitors must expose group-scoped state
World monitors MUST support planning-group state queries without requiring callers to infer robot-local joint mappings.

#### Scenario: Group state is requested for a multi-group robot
- **WHEN** a caller requests state for a specific group ID
- **THEN** the monitor returns joint data scoped and ordered for that group
