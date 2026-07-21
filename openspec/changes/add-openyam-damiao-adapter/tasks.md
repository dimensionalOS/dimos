## 1. Hardware resources and metadata

- [ ] 1.0 Confirm supported Linux deployment includes x86_64.
- [ ] 1.1 Add the materialized, fixed-finger six-DOF gravity-only OpenYAM URDF
  as an LFS-backed resource and make it available to the hardware configuration.
- [ ] 1.2 Define OpenYAM arm and gripper Damiao metadata: IDs 1–3 as DM4340,
  IDs 4–6 as DM4310, and gripper ID 7 as DM4310, with OpenYAM gains.
- [ ] 1.3 Parse the active gripper Xacro's six arm limits fail-closed into the
  hardware metadata, rejecting duplicate joints, missing/nonfinite values, and
  invalid ranges; keep physical encoder-zero home, reject mock-only initial
  positions in the physical factory, and add no homing or encoder offsets.

## 2. OpenYAM Damiao adapter

- [ ] 2.1 Implement a six-DOF OpenYAM adapter that reuses `DamiaoArmAdapter`
  for arm state, MIT commands, and gravity feed-forward behavior.
- [ ] 2.2 Add an internal one-motor gripper group without including it in the
  arm DOF, but keep gripper state and commands unavailable until released
  upstream normalized calibrated-opening getter support exists.
- [ ] 2.3 After upstream support exists, implement its bounded calibration and
  linear conversion between metre aperture and normalized opening over 0.096 m;
  do not fabricate state before then.
- [ ] 2.4 Require loaded-model and finite-`G(q)` preflight on every enable and
  recovery path before `Kp=Kd=0` with `G(q)` feed-forward; on failure perform
  no-motion disable and send no zero torque, keeping optional park separate.

## 3. OpenYAM integration

- [ ] 3.1 Register the OpenYAM hardware adapter and select it from the physical
  OpenYAM configuration in place of the mock adapter.
- [ ] 3.2 Preserve simulation and mock configuration behavior where physical
  Damiao hardware is not selected.

## 4. Verification and commissioning support

- [ ] 4.1 Add focused tests for six-DOF topology, motor metadata, separate
  gripper availability, and (after upstream support) aperture conversion.
- [ ] 4.2 Add focused tests that every enable/recovery path rejects a missing,
  unloadable, or non-finite gravity result, uses zero gains only after
  preflight, and performs no-motion disable without zero torque on failure.
- [ ] 4.3 Document the approved external vendor/bench-tool direction
  commissioning precondition for all six joints; verify the adapter does not
  discover direction under zero-gain gravity mode, disable never implicitly
  parks, failed disable is unresolved energized/unknown rather than disabled
  and escalates to operator/e-stop, and physical factories reject initial
  positions; verify gravity preflight and external direction commissioning remain
  required before normal and recovery enable.
- [ ] 4.4 Run the focused OpenYAM and Damiao test suite plus the blueprint
  registry generation test if registration changes generated blueprint output.
