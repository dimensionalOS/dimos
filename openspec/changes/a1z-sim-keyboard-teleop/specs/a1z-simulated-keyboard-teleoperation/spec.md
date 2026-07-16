## ADDED Requirements

### Requirement: Simulation-enabled A1Z keyboard teleoperation
The existing `keyboard-teleop-a1z` blueprint SHALL provide a simulated A1Z teleoperation environment when invoked with `--simulation`, while preserving its existing non-simulation behavior when that flag is not enabled.

#### Scenario: Start the simulated blueprint
- **GIVEN** the A1Z simulation assets are available
- **WHEN** an operator runs `dimos --simulation run keyboard-teleop-a1z`
- **THEN** the process SHALL start an A1Z MuJoCo-backed teleoperation stack
- **AND** the stack SHALL expose a wrist RGB image stream at 640×480 and 30 FPS.

#### Scenario: Start without simulation
- **GIVEN** no simulation flag is enabled
- **WHEN** an operator runs `dimos run keyboard-teleop-a1z`
- **THEN** the blueprint SHALL retain its existing non-simulation hardware selection behavior
- **AND** it SHALL NOT require the MuJoCo scene to start.

### Requirement: Latched keyboard gripper endpoints
The simulated keyboard teleoperation experience SHALL provide latched gripper endpoint controls in addition to the existing Cartesian arm controls.

#### Scenario: Open the gripper
- **GIVEN** simulated A1Z keyboard teleoperation is running
- **WHEN** the operator presses `[` 
- **THEN** the gripper SHALL target its configured open endpoint of 0.015 m driver displacement
- **AND** it SHALL continue holding that endpoint after the key is released.

#### Scenario: Close the gripper
- **GIVEN** simulated A1Z keyboard teleoperation is running
- **WHEN** the operator presses `]`
- **THEN** the gripper SHALL target its configured closed endpoint of 0.0 m driver displacement
- **AND** it SHALL continue holding that endpoint after the key is released.

#### Scenario: Move the arm while holding a gripper target
- **GIVEN** the gripper has a latched open or closed target
- **WHEN** the operator commands Cartesian arm motion with an existing keyboard control
- **THEN** the arm SHALL remain responsive to the Cartesian command
- **AND** the gripper SHALL retain its previously selected target.

### Requirement: Deterministic visual contact scene
The simulation SHALL provide a deterministic tabletop environment for manual teleoperation validation, including explicit visible physical A1Z finger-pad inserts whose render and collision geometry match. The inserts SHALL be identified as non-vendor simulation fixtures and SHALL NOT imply hardware or data parity.

#### Scenario: Inspect the initial scene
- **GIVEN** simulated A1Z keyboard teleoperation has started
- **WHEN** the operator views the wrist RGB stream
- **THEN** the view SHALL include the deterministic tabletop scene and its fixed cube when the configured pose is in frame
- **AND** the cube, table, lighting, and camera placement SHALL not be randomized between process starts.

#### Scenario: Sweep the gripper
- **GIVEN** the simulated gripper is clear of obstacles
- **WHEN** an operator alternates between the open and closed endpoint controls
- **THEN** the two visible fingers SHALL move in opposing directions over the configured range
- **AND** the simulated finger contact geometry SHALL be present for subsequent manual cube-contact testing.
- **AND** the visible pad inserts SHALL be the same geometry used for rendered appearance and collision contact.
- **AND** manual validation SHALL include a full opening sweep, unintended-contact checks, and a cube-contact check.

### Requirement: Stage 1 manual reset boundary
Stage 1 SHALL leave episode lifecycle and persistence out of the teleoperation interface.

#### Scenario: Reset after a manual trial
- **GIVEN** an operator has completed a simulated teleoperation trial
- **WHEN** the operator needs a known initial scene state
- **THEN** restarting the simulated process SHALL be the supported Stage 1 reset mechanism
- **AND** the keyboard interface SHALL NOT expose episode save, discard, or automatic-reset controls.
