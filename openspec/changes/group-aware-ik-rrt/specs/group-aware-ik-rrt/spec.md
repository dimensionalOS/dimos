## ADDED Requirements

### Requirement: IK solvers must use planning-group target frames
IK solvers MUST resolve pose target frames from the requested planning group's `tip_link` rather than from robot-scoped end-effector metadata.

#### Scenario: Pose IK targets a manipulator group
- **WHEN** a pose target is submitted for a group with `tip_link="tcp"`
- **THEN** the IK solver constrains the `tcp` frame

### Requirement: Planners must preserve group-local joint ordering
Planners MUST accept and return joint targets in the requested group's local joint order while projecting through full robot state for collision checks.

#### Scenario: Group joint target uses subset order
- **WHEN** a group target names a subset of robot joints
- **THEN** planning uses the correct full robot state and returns a path scoped to the group

### Requirement: Algorithms must fail clearly for non-pose-targetable groups
Algorithms that require a pose target frame MUST return an explicit failure when the requested group has no `tip_link`.

#### Scenario: Pose IK targets a joint-only group
- **WHEN** a pose target is submitted for a group without `tip_link`
- **THEN** the solver returns a no-solution or unsupported result with an explanatory message
