## 1. Hardware resources and metadata

- [ ] 1.1 Add the materialized, gripper-equipped OpenYAM gravity-model URDF as
  an LFS-backed resource and make it available to the hardware configuration.
- [ ] 1.2 Define OpenYAM arm and gripper Damiao metadata: IDs 1–3 as DM4340,
  IDs 4–6 as DM4310, and gripper ID 7 as DM4310, with OpenYAM gains and limits.
- [ ] 1.3 Load the active gripper Xacro's six arm limits into the OpenYAM
  hardware metadata without introducing homing or encoder-offset behavior.

## 2. OpenYAM Damiao adapter

- [ ] 2.1 Implement a six-DOF OpenYAM adapter that reuses `DamiaoArmAdapter`
  for arm state, MIT commands, and gravity feed-forward behavior.
- [ ] 2.2 Add an internal one-motor gripper group and implement gripper state
  and commands without including it in the adapter's arm DOF.
- [ ] 2.3 Implement calibrated gripper endpoint handling and linear conversion
  between metre aperture and normalized driver opening over the 0.096 m span.
- [ ] 2.4 Configure activation to require a loadable expanded gravity URDF and
  send `Kp=Kd=0` with `G(q)` feed-forward torque.

## 3. OpenYAM integration

- [ ] 3.1 Register the OpenYAM hardware adapter and select it from the physical
  OpenYAM configuration in place of the mock adapter.
- [ ] 3.2 Preserve simulation and mock configuration behavior where physical
  Damiao hardware is not selected.

## 4. Verification and commissioning support

- [ ] 4.1 Add focused tests for six-DOF topology, motor metadata, separate
  gripper behavior, and metre-to-opening conversion.
- [ ] 4.2 Add focused tests that gravity-compensation activation rejects a
  missing or unloadable gravity model and uses zero gains with gravity torque.
- [ ] 4.3 Document the no-load motor direction commissioning procedure and
  verify it against all six planning-joint directions before hardware motion.
- [ ] 4.4 Run the focused OpenYAM and Damiao test suite plus the blueprint
  registry generation test if registration changes generated blueprint output.
