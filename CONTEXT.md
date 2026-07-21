# OpenYAM Manipulator

The OpenYAM manipulator context describes the physical arm, its actuator groups,
and its kinematic-model vocabulary.

## Language

**OpenYAM Arm**:
The six-joint physical OpenYAM manipulator controlled through one CAN bus.

**Supported Linux architecture**:
Linux x86_64 is a supported deployment architecture for the physical Damiao
integration.

**Shoulder group**:
The first three arm actuators, corresponding in order to `yam_joint1` through
`yam_joint3`; each uses a DM4340 motor.

**Distal arm group**:
The final three arm actuators, corresponding in order to `yam_joint4` through
`yam_joint6`; each uses a DM4310 motor.

**CAN motor order**:
The arm's planning joints `yam_joint1` through `yam_joint6` correspond in order
to CAN motor IDs `1` through `6`; the gripper actuator is CAN motor ID `7`.

**Gripper actuator**:
The separate physical gripper actuator, using a DM4310 motor.
_Avoid_: seventh arm joint

**Gripper position**:
The gripper aperture expressed in metres, but unavailable until released
upstream normalized calibrated-opening getter support exists.
_Avoid_: gripper motor angle, fabricated or inferred gripper state

**Driver opening**:
The driver-internal normalized representation: `0` is closed and `1` is open.
It is not the manipulator API's public unit and must come from released upstream
calibration support.

**Read-only startup**:
The initial connected state in which actuator state may be observed but no hold
or motion command has been issued.

**Arm activation**:
The transition from read-only startup to active control after gravity-model
loadability and finite-`G(q)` preflight. Every enable and recovery path repeats
this preflight before motor enable. It does not imply gripper availability.

**Gravity-compensation mode**:
The active arm state with zero position and velocity gains and feed-forward
gravity torques from a valid fixed-finger six-DOF inertial model.
_Avoid_: zero-gain limp mode

**Gravity-model asset**:
The stable, pre-expanded, fixed-finger six-DOF LFS URDF used only to calculate
OpenYAM gravity torques. It is not the planning or limit source.
_Avoid_: runtime Xacro expansion or using it for command limits

**Direction commissioning**:
An approved external vendor or bench-tool no-load validation that establishes
whether each motor's positive motion matches its planning joint's positive
direction. It is a physical precondition, not an adapter-side routine, because
the driver activates gravity mode with zero gains.

**Encoder-zero home**:
The OpenYAM home pose defined by the existing zero position of each arm motor
encoder; it requires neither a homing routine nor a separate joint offset.

**Initial positions**:
Mock-only test configuration. Physical hardware starts from encoder-zero home;
the physical factory rejects initial-position configuration.

**Arm-limit authority**:
The active OpenYAM gripper Xacro is parsed fail-closed and defines the six arm
joints' command limits. Duplicate joints, missing/nonfinite values, and invalid
ranges are rejected.
_Avoid_: merging limits from the generated gravity URDF

**Disable**:
A no-motion operation that disables control without issuing a motion command. If
it fails or the resulting actuator state is unconfirmed, the state is
unresolved energized/unknown, never disabled, and must escalate to the operator
and approved e-stop procedure.

**Park**:
An optional, explicit motion operation; disabling never implicitly parks.

**Planning joint**:
A kinematic-model joint named `yam_joint1` through `yam_joint6`.
_Avoid_: CAN motor identifier
