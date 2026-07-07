## Purpose

Define direct OpenArm Mini leader teleoperation into OpenArm follower joint commands, including calibration, Feetech integration, runtime mapping, and blueprint wiring.

## Requirements

### Requirement: OpenArm Mini direct Feetech integration
The system SHALL implement OpenArm Mini teleoperation using the lower-level Feetech motor communication library rather than a LeRobot runtime dependency.

#### Scenario: OpenArm Mini adapter is imported without LeRobot
- **WHEN** the OpenArm Mini teleop adapter is imported in an environment without LeRobot installed
- **THEN** the import does not fail because of a missing LeRobot package

#### Scenario: Feetech dependency is missing
- **WHEN** OpenArm Mini teleop is started without the required Feetech dependency installed
- **THEN** the system fails with a clear message identifying the OpenArm Mini optional dependency needed to use the adapter

### Requirement: OpenArm Mini non-interactive runtime startup
The OpenArm Mini teleop adapter SHALL load calibration artifacts non-interactively during normal blueprint startup and fail fast when required calibration is missing or invalid.

#### Scenario: Calibration artifacts exist
- **WHEN** both side-specific OpenArm Mini calibration directories contain valid calibration artifacts
- **THEN** the OpenArm Mini teleop adapter loads them without prompting the user and proceeds with connection

#### Scenario: Calibration artifact is missing
- **WHEN** a required OpenArm Mini calibration artifact is missing during normal blueprint startup
- **THEN** the adapter fails fast with a clear error explaining how to run the manual calibration/demo script

### Requirement: OpenArm Mini side-specific calibration paths
The OpenArm Mini teleop adapter SHALL support side-specific calibration path configuration with DimOS state-directory defaults.

#### Scenario: Calibration paths are omitted
- **WHEN** `left_calibration_path` and `right_calibration_path` are not configured
- **THEN** the adapter resolves them to `STATE_DIR / "teleop" / "openarm_mini" / "left"` and `STATE_DIR / "teleop" / "openarm_mini" / "right"`

#### Scenario: Calibration paths are configured
- **WHEN** `left_calibration_path` or `right_calibration_path` is configured explicitly
- **THEN** the adapter uses the configured path for that side instead of the default DimOS state-directory path

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

### Requirement: Manual OpenArm Mini calibration/demo script
The system SHALL provide a manual OpenArm Mini calibration/demo script for interactive leader setup outside normal teleop runtime.

#### Scenario: Calibration script is run
- **WHEN** the OpenArm Mini calibration/demo script is run by an operator
- **THEN** it connects only to the OpenArm Mini leader arm-joint hardware, captures the current leader zero pose, and writes side-specific calibration artifacts

#### Scenario: Calibration script is run near follower hardware
- **WHEN** the calibration/demo script is running
- **THEN** it does not connect to follower OpenArm hardware and does not start `ControlCoordinator`

#### Scenario: Leader zero pose is captured
- **WHEN** the operator places an OpenArm Mini side in its designed natural pose and runs calibration
- **THEN** the script reads each arm-joint Feetech motor's raw position
- **AND** it writes those raw positions as homing offsets for the semantic `joint_1` through `joint_7` entries

#### Scenario: Calibration artifact is arm-only and minimal
- **WHEN** the calibration script writes a side-specific calibration artifact
- **THEN** the artifact contains exactly `joint_1` through `joint_7`
- **AND** each motor entry contains only the physical motor id, homing offset, and flip value
- **AND** it does not contain gripper entries, drive-mode fields, observed ranges, or gripper placeholders

#### Scenario: Calibration avoids follower startup
- **WHEN** the calibration/demo script runs
- **THEN** it does not touch gripper motor 8, does not connect to follower OpenArm hardware, and does not start `ControlCoordinator`

#### Scenario: Startup alignment is required
- **WHEN** an operator enables v1 OpenArm Mini teleop
- **THEN** the operator is responsible for placing the follower near the leader-implied command before enabling authority
- **AND** v1 does not claim automatic first-command follower-state gating

### Requirement: OpenArm Mini teleop blueprint
The system SHALL provide an OpenArm Mini teleop blueprint that wires the generic teleop module to the existing OpenArm control coordinator through `joint_command`.

#### Scenario: Blueprint is listed
- **WHEN** DimOS blueprints are listed after the OpenArm Mini teleop blueprint is added and the generated registry is refreshed
- **THEN** the OpenArm Mini teleop blueprint appears with the expected blueprint name

#### Scenario: Blueprint is built
- **WHEN** the OpenArm Mini teleop blueprint is built
- **THEN** the teleop module's `joint_command` output is connected to the coordinator's `joint_command` input
