## MODIFIED Requirements

### Requirement: Planners must preserve group-local joint ordering
Planners MUST accept joint targets in the requested groups' local joint order and MUST return a `GeneratedPlan` whose `waypoints` use the selected planning groups' canonical global joint order while projecting through full robot state for collision checks.

Selected joint planning MUST be part of the planner protocol. Planner backends that do not support selected/group planning MUST return a `GeneratedPlan` with `PlanningStatus.UNSUPPORTED` and an explanatory message rather than treating unsupported capability as a failed search. Planner backends MUST leave `GeneratedPlan.trajectory` unpopulated and MUST NOT assign waypoint timing.

#### Scenario: Group joint target uses subset order
- **WHEN** a group target names a subset of robot joints
- **THEN** planning uses the correct full robot state and returns `GeneratedPlan.waypoints` scoped and ordered to the selected groups

#### Scenario: Planner returns successful geometric waypoints
- **WHEN** a planner backend finds a selected-joint path
- **THEN** it returns a successful `GeneratedPlan` with geometric `waypoints` and `trajectory=None`

#### Scenario: Planner does not support selected groups
- **WHEN** a planner backend cannot plan the requested group selection
- **THEN** it returns an unsupported `GeneratedPlan` without a trajectory and with an explanatory message
