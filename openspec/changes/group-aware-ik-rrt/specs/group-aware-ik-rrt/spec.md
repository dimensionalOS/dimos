## ADDED Requirements

### Requirement: IK solvers must use planning-group target frames
IK solvers MUST resolve pose target frames from the requested planning group's `tip_link` rather than from robot-scoped end-effector metadata.

`solve_pose_targets` is a pose-target set IK API: callers MAY provide one or more pose targets keyed by planning group. Backends MUST either solve the requested target set or return `IKStatus.UNSUPPORTED` when the requested target-set shape is outside that backend's capability.

#### Scenario: Pose IK targets a manipulator group
- **WHEN** a pose target is submitted for a group with `tip_link="tcp"`
- **THEN** the IK solver constrains the `tcp` frame

#### Scenario: Pink IK receives multiple pose targets
- **WHEN** Pink IK receives multiple pose targets for pose-targetable planning groups
- **THEN** it solves the target frames as a target set and returns selected joints in planning-group selection order

#### Scenario: Single-target IK backend receives multiple pose targets
- **WHEN** Drake optimization IK or Jacobian IK receives more than one pose target or auxiliary planning groups
- **THEN** it returns `IKStatus.UNSUPPORTED` with an explanatory message

### Requirement: Planners must preserve group-local joint ordering
Planners MUST accept and return joint targets in the requested group's local joint order while projecting through full robot state for collision checks.

Selected joint planning MUST be part of the planner protocol. Planner backends that do not support selected/group planning MUST return `PlanningStatus.UNSUPPORTED` with an explanatory message rather than treating unsupported capability as a failed search.

#### Scenario: Group joint target uses subset order
- **WHEN** a group target names a subset of robot joints
- **THEN** planning uses the correct full robot state and returns a path scoped to the group

### Requirement: Algorithms must fail clearly for non-pose-targetable groups
Algorithms that require a pose target frame MUST return an explicit unsupported failure when the requested group has no `tip_link`.

#### Scenario: Pose IK targets a joint-only group
- **WHEN** a pose target is submitted for a group without `tip_link`
- **THEN** the solver returns `IKStatus.UNSUPPORTED` with an explanatory message
