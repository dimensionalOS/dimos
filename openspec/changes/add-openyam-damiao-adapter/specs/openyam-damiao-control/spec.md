## ADDED Requirements

### Requirement: Supported Linux architecture
The physical OpenYAM Damiao integration SHALL support Linux on x86_64 in
addition to any other explicitly supported Linux architecture.

#### Scenario: Deploying on Linux x86_64
- **WHEN** the physical OpenYAM integration is deployed on a Linux x86_64 host
- **THEN** the platform is supported by this capability

### Requirement: Six-DOF OpenYAM Damiao arm control
The system SHALL provide a hardware-backed OpenYAM manipulator adapter using
the shared Damiao motor runtime. The adapter SHALL expose exactly six arm
degrees of freedom in `yam_joint1` through `yam_joint6` order, corresponding to
CAN IDs 1 through 6. It SHALL configure CAN IDs 1 through 3 as DM4340 and IDs 4
through 6 as DM4310. The separate DM4310 gripper at CAN ID 7 SHALL NOT be
counted as an arm degree of freedom.

#### Scenario: Adapter reports OpenYAM arm topology
- **WHEN** the OpenYAM hardware adapter is created
- **THEN** it reports six arm degrees of freedom with the specified motor types
  and CAN ordering, while retaining CAN ID 7 as a separate gripper actuator

### Requirement: Default-deny operator approval for physical enable
Physical OpenYAM enable SHALL require the explicit operator configuration
`openyam_operator_approved=true`, exposed through the CLI as
`--openyam-operator-approved`. The default SHALL be false. The adapter SHALL
reject both normal and error-recovery enable before motor enable when approval
is absent or false. Approval SHALL NOT bypass gravity preflight, direction
commissioning, or limit validation.

#### Scenario: Normal enable lacks operator approval
- **WHEN** normal physical enable is requested without
  `openyam_operator_approved=true`
- **THEN** the adapter rejects enable before motor enable

#### Scenario: Error recovery lacks operator approval
- **WHEN** error-recovery physical enable is requested without
  `openyam_operator_approved=true`
- **THEN** the adapter rejects recovery enable before motor enable

#### Scenario: Operator approval is supplied by CLI
- **WHEN** the operator explicitly supplies `--openyam-operator-approved`
- **THEN** the resolved configuration sets `openyam_operator_approved=true`,
  while all other safety gates still apply

### Requirement: Gripper capability is upstream-gated
The system SHALL leave OpenYAM gripper position and command operations
unavailable until a released upstream `can-motor-control` API provides a
normalized, calibrated opening getter. It SHALL not fabricate gripper state,
infer it from motor angle, or invent a replacement getter.

#### Scenario: Gripper support is not released
- **WHEN** the upstream normalized calibrated-opening getter is unavailable
- **THEN** gripper position and command operations report unavailable and issue
  no fabricated state or gripper command

#### Scenario: Reading a gripper aperture after upstream support
- **WHEN** the released upstream getter is available and the gripper is
  calibrated
- **THEN** the adapter may convert normalized opening to aperture metres, with
  `0` closed and `1` open, without exposing normalized units publicly

### Requirement: Gravity-compensation activation
The system SHALL configure OpenYAM activation and recovery to use zero position
and velocity gains with feed-forward gravity torque computed from a valid,
expanded, fixed-finger six-DOF gravity-only URDF. Every enable or recovery path
SHALL load the model and preflight finite `G(q)` before sending any zero-gain
command. The active gripper Xacro SHALL remain the sole source of planning and
hardware command limits. It SHALL NOT represent zero gains without a valid
model and finite `G(q)` as gravity compensation.

#### Scenario: Activating a configured OpenYAM arm
- **WHEN** the coordinator activates an OpenYAM adapter
- **THEN** it first loads the fixed-finger model and verifies finite `G(q)` for
  the current state, then enables the arm and sends `Kp=0`, `Kd=0`, and gravity
  feed-forward torque

#### Scenario: Gravity preflight fails on enable or recovery
- **WHEN** the model is missing, unloadable, or produces non-finite `G(q)`
- **THEN** the enable or recovery fails before zero-gain control, performs a
  no-motion disable, and sends neither zero torque nor any other motion command

#### Scenario: Disable versus park
- **WHEN** control is disabled
- **THEN** the adapter disables without issuing a motion command
- **WHEN** an operator explicitly requests park
- **THEN** parking is performed only as that separate, opt-in operation

#### Scenario: No-motion disable fails
- **WHEN** a no-motion disable fails or its resulting actuator state cannot be
  confirmed
- **THEN** the system enters an unresolved energized/unknown state, does not
  report disabled, and escalates to the operator and approved e-stop procedure

### Requirement: OpenYAM physical hardware selection
The system SHALL configure OpenYAM's physical manipulator hardware to use the
Damiao-backed adapter instead of the mock adapter. It SHALL preserve physical
encoder-zero home and SHALL parse the active gripper Xacro fail-closed as the
authority for arm command limits. The physical factory SHALL reject mock-only
initial-position configuration; initial positions SHALL be mock-only and
unavailable to physical construction.

#### Scenario: Building the physical OpenYAM configuration
- **WHEN** DimOS builds an OpenYAM blueprint for physical hardware
- **THEN** it instantiates the Damiao-backed adapter only with six unique,
  finite, valid limits parsed from the active gripper Xacro, uses encoder-zero
  home, rejects mock initial positions, and does not add a homing or
  joint-offset procedure

### Requirement: External direction commissioning precondition
The system SHALL require approved external vendor or bench-tool direction
commissioning for all six arm motors before physical enable. The adapter SHALL
not perform direction discovery or commissioning by commanding the arm, because
the driver activates gravity mode with zero position and velocity gains.

#### Scenario: Direction commissioning is absent
- **WHEN** physical enable is requested without an approved six-joint direction
  commissioning result
- **THEN** enable is rejected before any arm command is sent
