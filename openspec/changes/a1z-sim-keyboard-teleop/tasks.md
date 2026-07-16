## 1. Simulation Assets and Model

- [ ] 1.1 Hydrate and unpack the tracked A1Z description, convert the G1Z fixed finger joints into a 0–0.015 m driver/follower prismatic pair with the accepted opposing outward diagonal axes, preserve source mesh names and fixed closed pose at zero, add the follower URDF mimic declaration, and repack the local LFS asset deliberately.
- [ ] 1.2 Add a repeatable A1Z URDF-to-MJCF generation path that preserves repository-relative mesh resolution, emits seven ordered arm-plus-driver actuators, and emits the equivalent MuJoCo equality for the follower mimic.
- [x] 1.3 Add the deterministic MuJoCo tabletop scene: table-edge arm mount, fixed lighting, a 5 cm cube, explicit visible physical A1Z finger-pad inserts with matching render/collision geometry, and an offset wrist RGB camera configured for 640×480 at 30 FPS. The inserts are non-vendor CAD fixtures.
- [ ] 1.4 Compile the final scene from its repository location and add focused structural validation for arm/gripper joint order, driver range, equality coupling, seven-actuator mapping, camera availability, and asset resolution.
- [ ] 1.5 Establish provisional A1Z home, table/base, cube, camera, and collision-pad transforms; apply the home to simulator startup/planning; configure runtime 30 FPS; then adjust scene transforms only from observed manual simulation results.

## 2. Simulated A1Z Composition

- [ ] 2.1 Add an A1Z MuJoCo hardware configuration that uses the existing shared-memory simulator adapter, shares its address with `MujocoSimModule`, exposes `arm/gripper` as raw driver displacement through an explicit identity mapping, and applies the same initial arm state to simulation and planning configuration.
- [ ] 2.2 Modify `keyboard-teleop-a1z` to explicitly compose the MuJoCo simulator with dedicated worker separation and 30 FPS when resolved `--simulation` is enabled, while retaining the prior non-simulation composition when it is disabled.
- [ ] 2.3 Add focused blueprint/configuration tests covering the explicit simulation branch, non-simulation compatibility, simulation address/model wiring, home/FPS configuration, and simulator/coordinator separation.

## 3. Keyboard Gripper Control

- [x] 3.1 Extend the keyboard teleop module with a partial named joint-command output and latched `[` open / `]` close handling that publishes `arm/gripper` endpoint changes only.
- [ ] 3.2 Configure the built-in `JointServoTask` for only `arm/gripper` in the simulation branch, with a non-conflicting task priority, closed default target, and indefinite target hold; retain `EEFTwistTask` ownership of the six arm joints.
- [ ] 3.3 Wire keyboard joint commands into the simulated coordinator and add focused tests for endpoint mapping, latching, disjoint arm/gripper task claims, concurrent Cartesian arm motion, and identity gripper command round-trip.

## 4. Documentation and Registry

- [x] 4.1 Update the relevant A1Z or keyboard-teleoperation user documentation with the `dimos --simulation run keyboard-teleop-a1z` command, `[`/`]` controls, deterministic-scene limitations, and restart-based Stage 1 reset.
- [ ] 4.2 Run `pytest dimos/robot/test_all_blueprints_generation.py`; commit generated registry changes only if the test updates them.

## 5. Verification and Manual QA

- [ ] 5.1 Run `openspec validate a1z-sim-keyboard-teleop`.
- [ ] 5.2 Run focused pytest targets for the keyboard module, A1Z configuration/blueprint, control-task integration, and MuJoCo scene validation.
- [ ] 5.3 Run the applicable documentation link/Markdown validation for the updated user documentation.
- [ ] 5.4 Manually run `dimos --simulation run keyboard-teleop-a1z` and verify the wrist RGB view, deterministic table/cube scene, arm motion, latched gripper endpoints, visible physical A1Z finger-pad inserts with matching render/collision geometry, full finger opening sweep without unintended collision, and manual cube contact. Treat the inserts as non-vendor CAD fixtures; do not infer hardware/data parity.
- [ ] 5.5 Manually restart the simulated process and verify restoration of the documented initial Stage 1 scene; do not add episode lifecycle or persistence controls.
