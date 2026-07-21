## Context

OpenYAM's current robot configuration selects a mock manipulator adapter. The
shared Damiao runtime already manages `can-motor-control` connection,
refresh, MIT commands, and Pinocchio gravity feed-forward; `DamiaoArmAdapter`
provides the six-or-more joint `ManipulatorAdapter` facade used by the control
coordinator. Its gripper methods are placeholders because it models one joint
group whose motor count defines the arm DOF.

OpenYAM is one CAN-bus arm with six planning joints and a separate gripper:
CAN IDs 1–3 are DM4340, 4–6 are DM4310, and ID 7 is a DM4310 gripper. The
gripper must retain the manipulator API's metre aperture contract while the
driver uses its calibrated normalized opening representation. The arm must
enter gravity compensation only with a materialized, gripper-equipped URDF
that Pinocchio can load directly; the runtime cannot expand Xacro.

## Goals / Non-Goals

**Goals:**

- Reuse the Damiao runtime, specs, and `DamiaoArmAdapter` pattern without
  inheriting OpenArm-specific kinematics or hardware metadata.
- Present exactly six arm DOFs in `yam_joint1` through `yam_joint6` order.
- Control the gripper as an internal second motor group and expose aperture in
  metres through the existing manipulator interface.
- Activate the arm with zero position/velocity gains and valid `G(q)`
  feed-forward torque from a stable expanded URDF.
- Preserve encoder-zero home and make motor direction a hardware commissioning
  check rather than an implicit software correction.

**Non-Goals:**

- Redesigning the generic manipulator API, Damiao runtime, or OpenYAM blueprint
  structure.
- Using the whole-body Damiao adapter.
- Adding a homing routine, stored encoder offsets, or a seventh arm DOF.
- Generating Xacro at runtime or merging limits from the generated bare URDF.

## Decisions

### Create a narrow OpenYAM adapter over `DamiaoArmAdapter`

The adapter SHALL construct a six-motor arm `DamiaoJointGroupSpec` and retain
the inherited arm command/state behavior. It SHALL add a second internal
single-motor group for the gripper and override only gripper state/command
methods. This preserves the established adapter lifecycle while ensuring
`get_dof()` remains six.

Using `DamiaoWholeBodyAdapter` was rejected because OpenYAM is one
manipulator, not a multi-limb robot, and its raw motor command path is not the
needed contract. Copying the OpenArm adapter wholesale was rejected because
its seven-joint topology, geometry, and calibration are OpenArm-specific.

### Encode OpenYAM hardware metadata locally

The adapter's arm group SHALL specify DM4340 motors at CAN IDs 1–3 and DM4310
motors at IDs 4–6 in planning-joint order. The gripper group SHALL specify its
DM4310 at CAN ID 7. OpenYAM gains and limits SHALL derive from its established
motor configuration; planning limits SHALL come only from the active gripper
Xacro. Existing encoder zeroes remain the home reference, so no offset or
homing configuration is added.

### Keep gripper calibration and normalization adapter-local

The public gripper interface SHALL use aperture metres. The adapter SHALL map
the supported aperture range linearly to the driver opening range, where zero
is closed and one is open, using calibrated endpoints. The nominal OpenYAM
aperture span is 0.096 m; endpoint calibration follows the
`can-motor-control` gripper opening lifecycle and remains in memory for the
active connection. This avoids leaking motor-angle or normalized units into
the generic API.

### Require an expanded gravity-model asset for activation

The OpenYAM hardware configuration SHALL provide an LFS-backed, expanded,
gripper-equipped URDF through `gravity_model_path`. On activation, the
inherited gravity-command path sends `Kp=Kd=0` and `tau=G(q)`. Passing a Xacro
was rejected because the runtime uses `pinocchio.buildModelFromUrdf()` directly
and has no Xacro/package-resolution support. Zero gains without a loaded model
are explicitly not gravity compensation.

## Risks / Trade-offs

- [Incorrect motor direction can move a joint opposite its planning command]
  → Validate every motor under no-load direction commissioning before normal
  operation; do not silently assume the configured sign is physically correct.
- [Gripper endpoint calibration actuates into physical end stops]
  → Use the driver's bounded calibration procedure and validate aperture
  travel without copying example CAN IDs or exposing an uncalibrated command.
- [An unavailable or invalid LFS gravity asset would produce limp zero-gain
  behavior] → Validate the asset can be materialized and loaded by Pinocchio
  before enabling gravity-compensation mode.
- [URDF/Xacro sources disagree on limits or inertials] → Treat the active
  gripper Xacro as the sole arm-limit authority and the expanded gripper URDF
  as the gravity-model authority.
