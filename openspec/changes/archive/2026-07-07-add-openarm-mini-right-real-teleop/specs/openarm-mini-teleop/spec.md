## MODIFIED Requirements

### Requirement: OpenArm Mini to OpenArm joint command mapping
The OpenArm Mini teleop adapter SHALL convert calibrated OpenArm Mini leader arm-joint readings into OpenArm follower arm-joint `JointState` commands in radians using configurable OpenArm follower joint names.

#### Scenario: Leader joint readings are available
- **WHEN** calibrated OpenArm Mini leader joint readings are available and teleop authority is active
- **THEN** the adapter returns a `JointState` command envelope whose joint names match the target OpenArm follower joints

#### Scenario: OpenArm Mini transform rules are applied
- **WHEN** the adapter converts leader readings into follower commands
- **THEN** it converts raw Feetech encoder ticks to radians around each calibrated homing offset
- **AND** it applies the per-joint `flip` value from the calibration artifact
- **AND** it clamps each outgoing command to the OpenArm follower joint limits before returning the command

#### Scenario: Leader joint assignment is calibrated
- **WHEN** the calibration artifact maps a semantic leader joint name to a physical Feetech motor id
- **THEN** the adapter reads that motor id for the semantic joint instead of applying a hardcoded runtime joint remap

#### Scenario: Configured follower joint namespace is applied
- **WHEN** the adapter is configured for a blueprint that targets ManipulationModule-compatible global joint names
- **THEN** the returned `JointState` command uses those configured target joint names instead of only the default local OpenArm follower joint names

#### Scenario: Gripper is out of scope for v1
- **WHEN** the adapter returns an OpenArm follower command
- **THEN** the command contains only OpenArm arm-joint names
- **AND** it does not include gripper names or gripper command values
