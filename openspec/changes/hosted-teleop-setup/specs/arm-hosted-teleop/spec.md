# arm-hosted-teleop

## ADDED Requirements

### Requirement: Engage-gated delta-pose control
Controller poses SHALL drive the arm only while the hand is engaged (primary button held); engagement captures the initial pose and the arm follows the delta from it at the control-loop rate (default 50 Hz). Release disengages. Output poses are stamped with the coordinator task name (`frame_id`) so the ControlCoordinator routes them to the right TeleopIK task; analog triggers ride the Buttons stream for gripper control.

#### Scenario: Engage and move
- **WHEN** the operator holds the primary button and moves the controller 10cm
- **THEN** delta poses stamped `teleop_xarm` stream to `/coordinator/cartesian_command` and the arm tracks the motion

#### Scenario: Not engaged
- **WHEN** the controller moves with the primary button released
- **THEN** no poses are published and the arm holds position

### Requirement: Arm E-STOP freezes in place
E-STOP SHALL latch, disengage both hands, and stop pose publishing — the coordinator's TeleopIK task keeps its last target so the arm freezes rather than dropping. While latched, engagement is refused. After `estop_clear`, motion resumes only via fresh engagement (which recaptures the initial pose, so the delta restarts from zero — no jump).

#### Scenario: E-STOP mid-motion
- **WHEN** `estop` arrives while engaged and moving
- **THEN** the arm freezes at its current target and further poses are suppressed until clear + re-engage

### Requirement: Operator loss disengages
On `operator_lost` the arm connection SHALL disengage both hands so a stale engagement cannot stream the last delta into the coordinator when the operator reconnects. Loss is not an E-STOP: re-engagement is allowed immediately.

#### Scenario: Link drop while engaged
- **WHEN** the operator connection drops mid-engagement
- **THEN** the hand disengages and the arm holds; on reconnect the operator must re-engage

### Requirement: Dual RealSense as distinct modules
Two RealSense cameras SHALL run as distinct module classes (`FrontCamera`, `WristCamera` subclassing `RealSenseCamera`) so each has its own blueprint identity, config namespace (`frontcamera.*` / `wristcamera.*`), and serial-number pinning. Front feeds cam1 (boot-default view), wrist feeds cam2, both over LCM into the connection's mux.

#### Scenario: Two cameras, one blueprint
- **WHEN** `teleop-hosted-xarm7-multicam` runs with both serials configured
- **THEN** both cameras stream concurrently and the operator can select front, wrist, or both

### Requirement: Command decode robustness
Operator bytes on `cmd_unreliable` SHALL be fingerprint-dispatched (PoseStamped/Joy); foreign or undecodable frames and unexpected frame_ids MUST be dropped without raising into the transport callback. Pose arrivals feed the command-plane stats pushed to the operator HUD.

#### Scenario: Foreign frame
- **WHEN** an unrecognized binary frame arrives on the command channel
- **THEN** it is skipped silently and pose/joy processing continues

### Requirement: Arm operator UI (pending)
The broker-side operator page for arms SHALL send WebXR controller poses and Joy state on `cmd_unreliable`, provide camera-select and E-STOP controls speaking the shared control-plane protocol, and seed its state from robot telemetry (engaged hands, cams, estop latch).

#### Scenario: Arm cockpit connects
- **WHEN** an operator opens the arm page against a registered arm robot
- **THEN** video renders, camera buttons reflect `state.cams`, and engaging streams poses to the robot
