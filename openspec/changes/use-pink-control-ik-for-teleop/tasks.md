## 1. Teleop Pink Task

- [x] 1.1 Add focused teleop task tests with a surgical fake Pink backend covering exact engagement-relative translation/rotation composition, first-tick measured FK capture, retained baseline, missing joint state, bounded `dt`, valid output, measured-state holds, and joint-delta rejection.
- [x] 1.2 Refactor `TeleopIKTaskConfig` and `TeleopIKTask` to specialize the Cartesian IK task and remove direct construction or use of legacy `PinocchioIK`, `model_path`, and `ee_joint_id`.
- [x] 1.3 Implement and test lifecycle baseline resets for disengage, timeout, stop, and clear while preserving task activation and preemption behavior.
- [x] 1.4 Implement and test fail-closed E-STOP handling that clears transient state, rejects pose and gripper commands while latched, and prevents command replay after clear.
- [x] 1.5 Preserve and test analog gripper interpolation, combined arm/gripper resource claims, and appending gripper positions to both valid Pink outputs and measured-state holds.
- [x] 1.6 Update teleop parameter validation and factory tests so authoritative Pink configuration is required, joint/model mismatches fail actionably, and missing optional Pink dependencies report the manipulation-extra installation path.

## 2. Atomic Blueprint Migration

- [x] 2.1 Change the shared `teleop_ik_task` blueprint helper to accept `RobotModelConfig`, resolve the nested Pink control configuration, and reject the removed legacy model parameters.
- [x] 2.2 Migrate Piper teleop to its authoritative robot model and named `gripper_base` end-effector frame while preserving hand selection, routing, priority, and gripper units.
- [x] 2.3 Migrate xArm6 and xArm7 teleop tasks to their authoritative no-gripper control models while preserving the lower-priority EEF-twist fallback and gripper behavior.
- [x] 2.4 Migrate mixed xArm/Piper teleop to per-hardware-namespace robot models whose ordered coordinator joints match each hardware task.
- [x] 2.5 Remove teleop-only legacy FK-model constants and imports that become unused after all call sites migrate.

## 3. Routing and Configuration Verification

- [x] 3.1 Extend blueprint tests to verify every shipped `teleop_ik` task has a reconstructable `PinkControlIKConfig`, matching hardware/model joints, the expected named end-effector frame, and no `model_path` or `ee_joint_id`.
- [x] 3.2 Preserve and verify registry-card behavior for task-name-routed Cartesian pose deltas and broadcast teleop buttons.
- [x] 3.3 Verify coordinator arbitration and preemption behavior remains unchanged for teleop versus lower-priority EEF-twist and higher-priority overlapping tasks.

## 4. Documentation and Validation

- [x] 4.1 Update manipulation and custom-arm documentation to describe Pink-based teleop configuration through `RobotModelConfig`, engagement-relative targets, named end-effector frames, and the removal of numeric joint IDs.
- [x] 4.2 Run the focused teleop, Cartesian IK, EEF-twist, coordinator-routing, and manipulator-blueprint pytest suites and resolve failures.
- [x] 4.3 Run repository formatting, lint, type checks, and `git diff --check` for all changed control and blueprint files.
- [x] 4.4 Smoke-test Piper, xArm6, and xArm7 teleop in simulation at conservative settings, verifying fresh-baseline engagement, translation/rotation direction, E-STOP recovery, and gripper operation; record any hardware gain tuning as rollout follow-up rather than changing architecture.

  Piper, xArm6, and xArm7 each passed the daemon startup health check with the
  Quest server, MuJoCo adapter, Pink task, command routing, and gripper channel
  active. Deterministic task tests verified fresh measured-state baselines,
  translation/rotation composition, E-STOP clear without replay, and gripper
  interpolation/output. No gain changes were needed; hardware-specific tuning
  remains a rollout follow-up.
