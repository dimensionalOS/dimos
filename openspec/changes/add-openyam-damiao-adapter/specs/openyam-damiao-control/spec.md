## ADDED Requirements

### Requirement: Six-DOF OpenYAM Damiao arm control
The system SHALL provide a hardware-backed OpenYAM manipulator adapter using
the shared Damiao motor runtime. The adapter SHALL expose exactly six arm
degrees of freedom in `yam_joint1` through `yam_joint6` order, corresponding
to CAN IDs 1 through 6. It SHALL configure CAN IDs 1 through 3 as DM4340 and
IDs 4 through 6 as DM4310. The separate DM4310 gripper at CAN ID 7 SHALL NOT
be counted as an arm degree of freedom.

#### Scenario: Adapter reports OpenYAM arm topology
- **WHEN** the OpenYAM hardware adapter is created
- **THEN** it reports six arm degrees of freedom with the specified motor types
  and CAN ordering, while retaining CAN ID 7 as a separate gripper actuator

### Requirement: Gripper aperture interface
The system SHALL expose OpenYAM gripper position through the manipulator API
as an aperture in metres. It SHALL convert the supported aperture range
linearly to the driver's calibrated normalized opening, where `0` is closed
and `1` is open, without exposing normalized opening or motor angle through
the public API.

#### Scenario: Commanding a gripper aperture
- **WHEN** a caller writes a supported gripper aperture in metres
- **THEN** the adapter commands the corresponding calibrated normalized driver
  opening for the separate CAN-ID-7 gripper

#### Scenario: Reading a gripper aperture
- **WHEN** the separate gripper state is refreshed after calibration
- **THEN** the adapter returns its opening as an aperture in metres

### Requirement: Gravity-compensation activation
The system SHALL configure OpenYAM activation to use zero position and velocity
gains with feed-forward gravity torque computed from a valid, expanded,
gripper-equipped URDF gravity model. It SHALL NOT represent zero gains without
a valid gravity model as gravity compensation.

#### Scenario: Activating a configured OpenYAM arm
- **WHEN** the coordinator activates an OpenYAM adapter with a loadable gravity
  model
- **THEN** the adapter enables the arm and sends commands with `Kp=0`, `Kd=0`,
  and gravity feed-forward torque for the current arm configuration

#### Scenario: Gravity model is unavailable
- **WHEN** an OpenYAM hardware configuration lacks a loadable expanded gravity
  model
- **THEN** activation fails before enabling zero-gain arm control

### Requirement: OpenYAM physical hardware selection
The system SHALL configure OpenYAM's physical manipulator hardware to use the
Damiao-backed adapter instead of the mock adapter. It SHALL preserve existing
encoder zeroes as home and SHALL use the active gripper Xacro as the authority
for arm command limits.

#### Scenario: Building the physical OpenYAM configuration
- **WHEN** DimOS builds an OpenYAM blueprint for physical hardware
- **THEN** it instantiates the Damiao-backed adapter with six-joint limits from
  the active gripper Xacro and does not add a homing or joint-offset procedure
