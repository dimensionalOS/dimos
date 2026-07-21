## Why

OpenYAM currently uses a mock manipulator adapter, so DimOS cannot operate the
physical arm. The existing Damiao runtime and OpenArm adapter establish a
reusable integration pattern, while OpenYAM needs explicit separation between
its gravity asset, active planning Xacro, hardware limits, and unavailable
gripper state. The physical integration also targets supported Linux hosts
including x86_64.

## What Changes

- Add a hardware-backed OpenYAM manipulator adapter built on the shared Damiao
  runtime and generic arm adapter.
- Configure the six-joint arm as CAN IDs 1–6 (DM4340 shoulder group and DM4310
  distal group), while keeping the DM4310 CAN-ID-7 gripper separate from the
  arm's six degrees of freedom.
- Keep gripper operations unavailable until a released upstream
  `can-motor-control` normalized calibrated-opening getter exists; do not
  fabricate state. Then expose aperture metres through the existing API.
- Activate the arm in gravity-only compensation using zero position and
  velocity gains plus feed-forward gravity torque from a stable, expanded,
  fixed-finger six-DOF URDF, with loaded-model and finite-`G(q)` preflight on
  every enable/recovery path. Gravity failure performs no-motion disable and
  never sends zero torque.
- Keep that gravity URDF separate from the active Xacro, which is the
  fail-closed planning and hardware-limit authority; reject duplicate,
  nonfinite, missing, or invalid Xacro limit entries.
- Use physical encoder-zero home; the physical factory rejects mock-only
  initial positions. Require approved external vendor/bench-tool direction
  commissioning before enable because the driver enters gravity mode at zero
  gains. Disable is no-motion by default, while park is explicit; a failed
  disable remains unresolved energized/unknown, is never reported disabled,
  and escalates to operator/e-stop procedure.
- Default-deny physical enable behind explicit `openyam_operator_approved`
  configuration, exposed as `--openyam-operator-approved`; reject normal and
  error-recovery enable before motor enable without it.

## Capabilities

### New Capabilities

- `openyam-damiao-control`: Hardware control of the six-joint OpenYAM arm and
  its separate CAN-bus gripper through the Damiao motor runtime.

### Modified Capabilities

- None.

## Impact

- Adds an OpenYAM-specific adapter and physical-hardware registration under
  `dimos/hardware/manipulators/`.
- Updates `dimos/robot/manipulators/openyam/config.py` to select the hardware
  implementation instead of the mock adapter.
- Adds a materialized fixed-finger, six-DOF gravity-only OpenYAM URDF asset to
  LFS-backed robot resources; it does not supply planning limits.
- Reuses the existing `can-motor-control` dependency and shared Damiao runtime;
  it introduces no generic API change beyond the planned hardware integration.
