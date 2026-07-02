# manipulation-planning-config Specification

## Requirements

### Requirement: Robot model configuration must separate placement from planning chains
Robot model configuration MUST treat robot-scoped `base_link` as the link placed by `base_pose` and used by backend weld/strip behavior. Planning-chain base links and pose target frames MUST come from planning groups.

#### Scenario: Robot has one pose-targetable group
- **WHEN** a robot model has `base_link="base_link"` and a planning group with `base_link="link1"` and `tip_link="tcp"`
- **THEN** placement uses the robot-scoped `base_link` while pose planning targets `tcp`
