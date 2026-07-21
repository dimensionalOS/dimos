## Context

OpenYAM is one CAN-bus arm with six planning joints and a separate gripper:
CAN IDs 1–3 are DM4340, 4–6 are DM4310, and ID 7 is a DM4310 gripper. The
gravity model is a fixed-finger, six-DOF, gravity-only URDF. It is deliberately
separate from the active Xacro, which remains the planning and command-limit
authority; the runtime cannot expand Xacro. Supported Linux deployment
architectures SHALL include x86_64.

## Goals / Non-Goals

**Goals:**

- Reuse the Damiao runtime, specs, and `DamiaoArmAdapter` pattern without
  inheriting OpenArm-specific kinematics or hardware metadata.
- Present exactly six arm DOFs in `yam_joint1` through `yam_joint6` order.
- Keep the gripper unavailable until a released upstream `can-motor-control`
  normalized, calibrated opening getter exists; do not fabricate gripper state.
- Require every enable and recovery path to load the gravity model and complete
  finite-`G(q)` preflight before zero-gain gravity operation; failures must
  disable with no motion command and must not send zero torque.
- Preserve physical encoder-zero home; initial positions are mock-only.

**Non-Goals:**

- Redesigning the generic manipulator API, Damiao runtime, or OpenYAM blueprint
  structure.
- Using the whole-body Damiao adapter.
- Adding a homing routine, stored encoder offsets, or a seventh arm DOF.
- Generating Xacro at runtime or using the gravity URDF for planning limits.
- Providing fabricated gripper state or an invented upstream getter.

## Decisions

### Create a narrow OpenYAM adapter over `DamiaoArmAdapter`

The adapter SHALL construct a six-motor arm `DamiaoJointGroupSpec` and retain
the inherited arm command/state behavior. It SHALL add a second internal
single-motor group for the gripper, without counting it in `get_dof()`. Gripper
position and command operations SHALL remain unavailable until released
upstream normalized calibrated-opening getter support exists.

Using `DamiaoWholeBodyAdapter` was rejected because OpenYAM is one
manipulator, not a multi-limb robot. Copying the OpenArm adapter wholesale was
rejected because its topology, geometry, and calibration are OpenArm-specific.

### Encode OpenYAM hardware metadata locally

The adapter's arm group SHALL specify DM4340 motors at CAN IDs 1–3 and DM4310
motors at IDs 4–6 in planning-joint order. The gripper group SHALL specify its
DM4310 at CAN ID 7. OpenYAM gains SHALL derive from its motor configuration.
Hardware limits SHALL be parsed fail-closed from the active gripper Xacro and
never from the gravity URDF. The parser SHALL reject duplicate joints, missing
or nonfinite values, and invalid ranges. Physical encoder zeroes remain the
home reference; the physical factory SHALL reject mock-only initial-position
configuration, with no homing or offset behavior.

### Keep gripper support upstream-first

When the upstream getter is released, the public gripper interface SHALL use
aperture metres and convert linearly to the driver's calibrated normalized
opening, where zero is closed and one is open. Until then, the adapter SHALL
report the capability as unavailable and SHALL neither command nor synthesize
its state. The nominal future aperture span is 0.096 m; endpoint calibration
must follow the upstream opening lifecycle.

### Require a fixed-finger gravity-model asset for activation

The hardware configuration SHALL provide an LFS-backed, expanded, fixed-finger
six-DOF URDF through `gravity_model_path`. It is gravity-only and is not a
source of planning joints or command limits. Before enabling or recovering the
arm, activation SHALL load the model, evaluate `G(q)` for the current six-joint
state, and reject non-finite results. Every such path SHALL complete this
preflight before sending any zero-gain command. If model loading, gravity
evaluation, enabling, or recovery fails, the arm SHALL be disabled with no
motion command; it SHALL not send zero torque as a failure response. Only
after successful preflight may the command path send `Kp=Kd=0` and `tau=G(q)`.

Disabling SHALL be a no-motion operation by default. An optional park action
may be requested explicitly as a separate operation; disabling must never
implicitly park the arm. If a no-motion disable fails, the resulting state is
explicitly unresolved energized/unknown: it SHALL NOT be reported as disabled.
The system SHALL escalate to the operator and the approved e-stop procedure.

## Risks / Trade-offs

- [Incorrect motor direction can move a joint opposite its planning command]
  → Require an approved external vendor/bench-tool direction commissioning
  result for all six joints before hardware enable. This is a precondition,
  not an adapter-side motion routine: the driver enters gravity mode with
  zero gains, so the adapter must not attempt to discover direction in control.
- [Gripper support is unavailable or calibration actuates into end stops]
  → Block the gripper API until released upstream getter support exists; then
  use only the driver's bounded calibration procedure.
- [An unavailable, invalid, or numerically unstable gravity asset could enable
  unsafe zero-gain behavior] → Require model load and finite-`G(q)` preflight
  on every enable/recovery path; use no-motion disable, never zero torque, on
  failure.
- [No-motion disable can fail while the driver remains energized or unknown]
  → Preserve an unresolved energized/unknown state, never claim disabled, and
  escalate to the operator and approved e-stop procedure.
- [URDF/Xacro sources disagree on limits or inertials] → Treat the active
  gripper Xacro as the fail-closed planning-limit authority and the fixed-finger
  URDF as gravity-only authority.
