## ADDED Requirements

### Requirement: Manipulation module must expose explicit group planning APIs
The manipulation module MUST allow callers to target planning groups explicitly for joint and pose planning operations.

#### Scenario: Caller plans to a group pose
- **WHEN** a caller submits a pose target for a valid group ID
- **THEN** the module plans using that group and returns the planning result through the existing public contract

### Requirement: Robot-scoped pose wrappers must fail safely without a unique pose group
Robot-scoped pose wrapper APIs MUST fail safely when a selected robot has zero or multiple pose-targetable groups.

#### Scenario: End-effector pose is requested with no unique pose group
- **WHEN** `get_ee_pose` cannot resolve exactly one pose-targetable group
- **THEN** it returns `None`

#### Scenario: Robot-scoped pose planning is requested with no unique pose group
- **WHEN** `plan_to_pose` cannot resolve exactly one pose-targetable group
- **THEN** it returns `False`

#### Scenario: Robot-scoped IK is requested with no unique pose group
- **WHEN** `inverse_kinematics_single` cannot resolve exactly one pose-targetable group
- **THEN** it returns an `IKResult` with `NO_SOLUTION`
