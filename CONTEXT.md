# OpenYAM Manipulator

The OpenYAM manipulator context describes the physical arm, its actuator groups,
and its kinematic-model vocabulary.

## Language

**OpenYAM Arm**:
The six-joint physical OpenYAM manipulator controlled through one CAN bus.

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
The gripper aperture expressed in metres, as required by the manipulator API.
_Avoid_: gripper motor angle, normalized gripper opening

**Driver opening**:
The driver-internal normalized representation of the gripper aperture: `0` is
closed and `1` is open. It is not the manipulator API's public unit.

**Read-only startup**:
The initial connected state in which actuator state may be observed but no hold
or motion command has been issued.

**Arm activation**:
The transition from read-only startup to active control. It enables the arm and
calibrates the gripper's normalized opening endpoints.

**Gravity-compensation mode**:
The active arm state with zero position and velocity gains and feed-forward
gravity torques from a valid OpenYAM inertial model.
_Avoid_: zero-gain limp mode

**Gravity-model asset**:
The stable, pre-expanded, gripper-equipped LFS URDF used to calculate OpenYAM
gravity torques.
_Avoid_: runtime Xacro expansion

**Direction commissioning**:
The no-load validation that establishes whether each motor's positive motion
matches its planning joint's positive direction.

**Encoder-zero home**:
The OpenYAM home pose defined by the existing zero position of each arm motor
encoder; it requires neither a homing routine nor a separate joint offset.

**Arm-limit authority**:
The active OpenYAM gripper Xacro defines the six arm joints' command limits.
_Avoid_: merging limits from the generated bare URDF

**Planning joint**:
A kinematic-model joint named `yam_joint1` through `yam_joint6`.
_Avoid_: CAN motor identifier
