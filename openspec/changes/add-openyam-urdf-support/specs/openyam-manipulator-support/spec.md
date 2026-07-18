## ADDED Requirements

### Requirement: OpenYAM model variants
DimOS SHALL provide bare-arm and gripper-equipped OpenYAM manipulator models that can be resolved by its planning and visualization workflows, including their mesh resources and six arm joints.

#### Scenario: Load the bare-arm model
- **GIVEN** DimOS is installed with the OpenYAM robot-description asset available
- **WHEN** a developer constructs the bare-arm OpenYAM manipulation model configuration
- **THEN** the configuration resolves the generated bare-arm URDF and its package-relative mesh resources
- **AND** the configuration exposes `joint1` through `joint6` as the arm joints without a gripper hardware command

#### Scenario: Load the gripper-equipped model
- **GIVEN** the OpenYAM model is loaded for visualization or planning
- **WHEN** a developer selects the gripper-equipped OpenYAM model configuration
- **THEN** the configuration resolves the gripper-enabled Xacro model and its package-relative mesh resources
- **AND** it exposes `joint1` through `joint6` as the arm joints and a direct mock gripper command channel without modeling finger-state synchronization

#### Scenario: Command the mock gripper
- **GIVEN** the gripper-equipped OpenYAM model is used with mock hardware
- **WHEN** a caller commands the gripper through the configured gripper hardware ID
- **THEN** the mock hardware accepts the single gripper command
- **AND** the arm planning group remains limited to the six arm joints without promising finger-state visualization or synchronization

### Requirement: Runnable OpenYAM manipulation workflows
DimOS SHALL make bare-arm and gripper-equipped OpenYAM models available through built-in basic/planning and keyboard-teleoperation blueprint workflows using mock hardware.

#### Scenario: Discover OpenYAM blueprints
- **GIVEN** the built-in blueprint registry is generated from the repository sources
- **WHEN** a developer lists runnable DimOS blueprints
- **THEN** bare-arm and gripper-equipped OpenYAM basic/planning and keyboard-teleoperation workflows are discoverable

#### Scenario: Use the mock workflow
- **GIVEN** a developer launches an OpenYAM workflow without a physical hardware adapter
- **WHEN** the blueprint is built
- **THEN** it configures mock manipulation hardware with the selected OpenYAM model variant
- **AND** it does not require ROS control, CAN, or physical robot connectivity
