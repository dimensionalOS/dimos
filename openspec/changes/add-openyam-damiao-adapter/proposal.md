## Why

OpenYAM currently uses a mock manipulator adapter, so DimOS cannot operate the
physical arm. The existing Damiao runtime and OpenArm adapter establish a
tested integration pattern that can be reused while keeping OpenYAM's motor,
gripper, and kinematic-model requirements explicit.

## What Changes

- Add a hardware-backed OpenYAM manipulator adapter built on the shared Damiao
  runtime and generic arm adapter.
- Configure the six-joint arm as CAN IDs 1–6 (DM4340 shoulder group and
  DM4310 distal group), while keeping the DM4310 CAN-ID-7 gripper separate
  from the arm's six degrees of freedom.
- Expose the gripper through the existing manipulator API in aperture metres,
  with adapter-local calibrated conversion to the driver's normalized opening.
- Activate the arm in gravity-compensation mode using zero position and
  velocity gains plus feed-forward gravity torque from a stable, expanded
  gripper-equipped URDF asset.
- Replace the OpenYAM mock adapter configuration with the hardware adapter and
  register the physical hardware implementation.

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
- Adds a materialized OpenYAM gravity-model URDF asset to LFS-backed robot
  resources.
- Reuses the existing `can-motor-control` dependency and shared Damiao runtime;
  it introduces no public API change beyond making the existing OpenYAM
  manipulator and gripper operations functional on hardware.
