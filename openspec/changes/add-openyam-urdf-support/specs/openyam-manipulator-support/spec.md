## ADDED Requirements

### Requirement: Gripper-equipped OpenYAM model
DimOS SHALL provide exactly one gripper-equipped OpenYAM manipulator model, resolved through the DimOS-owned `yam_gripper.urdf.xacro` wrapper by its planning and visualization workflows, including its mesh resources and six arm joints.

#### Scenario: Load the gripper-equipped model
- **GIVEN** DimOS is installed with the OpenYAM robot-description asset available
- **WHEN** a developer constructs the OpenYAM manipulation model configuration
- **THEN** the configuration resolves `yam_gripper.urdf.xacro` and its package-relative mesh resources
- **AND** it exposes `joint1` through `joint6` as the arm joints and a direct mock gripper command channel without modeling finger-state synchronization
- **AND** any corrected mesh orientation applies to both visual and collision mesh presentation while preserving mesh bytes, XYZ origins, joints/axes, and inertials, with no custom collision exclusions

#### Scenario: Command the mock gripper
- **GIVEN** the gripper-equipped OpenYAM model is used with mock hardware
- **WHEN** a caller commands the gripper through the configured gripper hardware ID
- **THEN** the mock hardware accepts the single gripper command
- **AND** the arm planning group remains limited to the six arm joints without promising finger-state visualization or synchronization

### Requirement: Runnable OpenYAM manipulation workflows
DimOS SHALL make the gripper-equipped OpenYAM model available through built-in basic/planning and keyboard-teleoperation blueprint workflows using mock hardware.

#### Scenario: Discover OpenYAM blueprints
- **GIVEN** the built-in blueprint registry is generated from the repository sources
- **WHEN** a developer lists runnable DimOS blueprints
- **THEN** the OpenYAM basic/planning and keyboard-teleoperation workflows are discoverable

#### Scenario: Use the mock workflow
- **GIVEN** a developer launches an OpenYAM workflow without a physical hardware adapter
- **WHEN** the blueprint is built
- **THEN** it configures mock manipulation hardware with the OpenYAM gripper-equipped model
- **AND** it does not require ROS control, CAN, or physical robot connectivity
