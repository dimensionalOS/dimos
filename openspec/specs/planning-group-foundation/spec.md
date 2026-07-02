# planning-group-foundation Specification

## Requirements

### Requirement: Planning group definitions must identify kinematic chains
The planning configuration MUST represent kinematic chain metadata with planning-group definitions containing group-local joint names, a chain base link, and an optional pose-target tip link.

#### Scenario: Explicit manipulator group is configured
- **WHEN** a manipulator config declares a group with local joints, `base_link`, and `tip_link`
- **THEN** the resolved robot model exposes that group without relying on a robot-scoped end-effector field

### Requirement: Robot configuration must reject removed end-effector metadata
Robot configuration MUST NOT accept robot-scoped `end_effector_link` as the source of pose-target metadata. It MUST direct callers to planning-group `tip_link` values or SRDF discovery.

#### Scenario: Legacy end-effector field is provided
- **WHEN** a robot config is constructed with `end_effector_link`
- **THEN** validation fails with migration guidance

### Requirement: Group identifiers must be globally scoped
Planning-group APIs MUST use stable group identifiers scoped by robot name so multi-robot stacks can distinguish groups with the same local name.

#### Scenario: Two robots use manipulator groups
- **WHEN** two robot configs each define a `manipulator` group
- **THEN** their resolved group IDs are distinct
