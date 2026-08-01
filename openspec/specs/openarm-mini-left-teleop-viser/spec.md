## Purpose

Define the visualization-only OpenArm Mini left-leader validation path that renders leader-derived OpenArm follower commands in Viser without connecting follower hardware.

## Requirements

### Requirement: Real left leader input
The validation path SHALL require a real OpenArm Mini left leader connected through the Feetech SDK and a valid left-side calibration artifact. It SHALL NOT provide fake, replay, or synthetic leader input modes.

#### Scenario: Left leader starts with calibration
- **WHEN** the visualization blueprint starts with the OpenArm Mini left leader connected and a valid left calibration artifact available
- **THEN** the system connects only to the left leader Feetech bus and begins producing left OpenArm follower arm-joint commands

#### Scenario: Required leader input is unavailable
- **WHEN** the left leader serial connection, Feetech dependency, or left calibration artifact is unavailable
- **THEN** startup or connection reports a clear error instead of falling back to fake or replay input

### Requirement: Visualization-only follower path
The validation path SHALL render the leader-derived OpenArm left follower pose in Viser without instantiating `ControlCoordinator`, OpenArm follower hardware, mock OpenArm follower hardware, or any physical execution path.

#### Scenario: Follower hardware remains disconnected
- **WHEN** the visualization blueprint is built or run
- **THEN** it contains no OpenArm follower `HardwareComponent`, no mock follower hardware adapter, and no `ControlCoordinator`

#### Scenario: Commands are visualized only
- **WHEN** the left leader produces a `joint_command`
- **THEN** the command updates the Viser-rendered OpenArm left follower model and does not command follower hardware

### Requirement: Left-only OpenArm Mini adapter operation
The OpenArm Mini adapter SHALL support explicit left-only operation that loads, connects, and reads only the left side while preserving the existing bimanual default behavior for production teleop.

#### Scenario: Left-only operation omits right side
- **WHEN** the adapter is configured for left-only visualization
- **THEN** it requires only the left port and left calibration, opens only the left Feetech bus, and emits only `openarm_left_joint1` through `openarm_left_joint7`

#### Scenario: Production default remains bimanual
- **WHEN** the existing production OpenArm Mini teleop blueprint constructs the adapter without visualization-specific side selection
- **THEN** the adapter still requires and emits both left and right arm-joint commands as before

### Requirement: Viser rendering from named JointState commands
The visualization module SHALL render an OpenArm left follower model from incoming `JointState` commands by matching joint names, not by assuming positional vector order. It SHALL render arm joints only and SHALL NOT require or render gripper commands.

#### Scenario: JointState names are normalized to model order
- **WHEN** the visualizer receives a `JointState` containing all OpenArm left arm-joint names in any order
- **THEN** it updates Viser using the OpenArm left model's expected joint order

#### Scenario: Required joints are missing
- **WHEN** the visualizer receives a `JointState` missing one or more required OpenArm left arm joints
- **THEN** it reports the missing joints and skips rendering that incomplete command instead of rendering a misleading pose

#### Scenario: Gripper data is absent
- **WHEN** the validation path processes left leader arm-joint commands
- **THEN** it does not require, emit, or render OpenArm Mini gripper values

### Requirement: Registered left-only Viser blueprint
The system SHALL expose a runnable static blueprint named `openarm-mini-left-teleop-viser` that wires real left OpenArm Mini teleop output to the visualization-only Viser renderer through `joint_command`.

#### Scenario: Blueprint is listed
- **WHEN** users list available DimOS blueprints
- **THEN** `openarm-mini-left-teleop-viser` appears as a runnable blueprint

#### Scenario: Blueprint wiring is visualization-only
- **WHEN** the blueprint is inspected in tests
- **THEN** it wires `TeleopModule` with a left-only `OpenArmMiniTeleopAdapter` to the Viser visualizer via `joint_command` and does not include coordinator or follower hardware atoms
