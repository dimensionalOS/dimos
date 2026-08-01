## ADDED Requirements

### Requirement: Right OpenArm Mini leader input
The right real-teleop blueprint SHALL use a real OpenArm Mini right leader connected through the Feetech SDK and a valid right-side calibration artifact.

#### Scenario: Right leader starts with calibration
- **WHEN** the right real-teleop blueprint starts with the OpenArm Mini right leader connected and a valid right calibration artifact available
- **THEN** the system connects only to the right leader Feetech bus
- **AND** it begins producing right OpenArm follower arm-joint commands

#### Scenario: Right calibration is missing
- **WHEN** the right calibration artifact is unavailable or invalid
- **THEN** startup fails with a clear error instead of using default calibration

#### Scenario: Right leader default port is used
- **WHEN** the blueprint is run without a right leader port override
- **THEN** it uses `/dev/ttyACM0` for the OpenArm Mini right leader

### Requirement: Real follower opt-in through can-port
The right real-teleop blueprint SHALL use mock right OpenArm follower hardware unless the operator provides global `--can-port`, and SHALL use real right OpenArm follower hardware when `--can-port` is provided.

#### Scenario: can-port is absent
- **WHEN** the blueprint starts without `--can-port`
- **THEN** the follower hardware component uses the mock adapter
- **AND** no real OpenArm follower CAN connection is attempted

#### Scenario: can-port is provided
- **WHEN** the blueprint starts with `--can-port can0`
- **THEN** the follower hardware component uses the real OpenArm adapter
- **AND** it connects to the right OpenArm follower through `can0`

### Requirement: Right follower coordinator servo routing
The right real-teleop blueprint SHALL route OpenArm Mini right leader joint commands through `ControlCoordinator` to a right OpenArm servo task.

#### Scenario: Leader command is published
- **WHEN** the OpenArm Mini right leader produces a valid arm-joint command
- **THEN** the command is published on `joint_command`
- **AND** `ControlCoordinator` receives it for the right OpenArm servo task

#### Scenario: Follower hardware is configured
- **WHEN** the blueprint is inspected
- **THEN** it contains one right OpenArm follower hardware component
- **AND** one servo task covering the right OpenArm arm joints

### Requirement: ManipulationModule Viser visualization
The right real-teleop blueprint SHALL render follower-observed coordinator state through `ManipulationModule` with the Viser visualization backend.

#### Scenario: Coordinator state is published
- **WHEN** `ControlCoordinator` publishes `coordinator_joint_state`
- **THEN** `ManipulationModule` consumes that state and updates the right OpenArm model in Viser

#### Scenario: Custom command-only visualizer is not used
- **WHEN** the right real-teleop blueprint is inspected
- **THEN** it uses `ManipulationModule` with `visualization={"backend": "viser"}`
- **AND** it does not include the custom OpenArm Mini Viser renderer module

### Requirement: Registered right real-teleop blueprint
The system SHALL expose a runnable static blueprint named `openarm-mini-right-teleop-viser` for right-side OpenArm Mini teleop with mock-or-real right OpenArm follower and ManipulationModule Viser.

#### Scenario: Blueprint is listed
- **WHEN** users list available DimOS blueprints after the generated registry is refreshed
- **THEN** `openarm-mini-right-teleop-viser` appears as a runnable blueprint

#### Scenario: Blueprint config can override right leader port
- **WHEN** the blueprint is run with `-o openarmminiteleopmodule.openarm_mini.port_right=/dev/ttyUSB0`
- **THEN** the OpenArm Mini right leader uses `/dev/ttyUSB0` instead of the blueprint default
