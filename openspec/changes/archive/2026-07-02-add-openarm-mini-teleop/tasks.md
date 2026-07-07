## 1. Teleop Adapter Runtime

- [x] 1.1 Add teleop runtime package structure under `dimos/teleop/` without changing existing Quest modules.
- [x] 1.2 Define `TeleopPrimaryOutput`, `TeleopCommandMetadata`, `TeleopCommand`, and `TeleopAdapter` types with `connect()`, `disconnect()`, and `get_current_command()`.
- [x] 1.3 Implement a generic `TeleopModule` with stable coordinator-facing outputs for `joint_command`, `coordinator_cartesian_command`, and `twist_command`.
- [x] 1.4 Implement structural safety in `TeleopModule`: no publish on `None`, one primary output, max publish rate, stale-command timeout, explicit stop handling, and adapter disconnect on stop.
- [x] 1.5 Add unit tests for command envelope handling, output routing, conflicting primary output rejection, stale timeout behavior, rate limiting, and stop/disconnect behavior.

## 2. OpenArm Mini Feetech Integration

- [x] 2.1 Verify the lower-level Feetech Python package name/import surface and add it to a narrow OpenArm Mini optional extra in `pyproject.toml`.
- [x] 2.2 Add localized missing-dependency handling so OpenArm Mini teleop fails with a clear install hint when the Feetech extra is not installed.
- [x] 2.3 Implement OpenArm Mini config with `port_left`, `port_right`, `left_calibration_path`, and `right_calibration_path` plus DimOS `STATE_DIR / "teleop" / "openarm_mini" / <side>` defaults.
- [x] 2.4 Update OpenArm Mini calibration artifact load/save models and validation to require strict arm-only `joint_1` through `joint_7` entries with `id`, `homing_offset`, and `flip` only.
- [x] 2.5 Implement Feetech bus connection/configuration for both OpenArm Mini sides using non-interactive calibration loading during runtime startup.
- [x] 2.6 Implement OpenArm Mini arm-joint reading from calibration-defined motor ids, raw tick to radians conversion around homing offsets, per-joint flip handling, OpenArm follower arm-joint naming, and sender-side follower-limit clamping.
- [x] 2.7 Implement `OpenArmMiniTeleopAdapter.get_current_command()` to return `JointState` command envelopes only when teleop authority is active and leader readings are valid.
- [x] 2.8 Update unit tests for calibration path defaults, missing/invalid arm-only calibration errors, leader joint assignment by motor id, radians conversion, flip handling, follower arm-joint names, follower-limit clamping, and gripper omission.

## 3. Manual Calibration Demo

- [x] 3.1 Add `dimos/teleop/openarm_mini/demo_calibrate_openarm_mini.py` as a manual script excluded from pytest collection by name.
- [x] 3.2 Replace the calibration UX with zero-capture: operator places one OpenArm Mini side in the leader zero pose, the script reads arm motors only, and writes homing offsets without connecting to follower OpenArm hardware.
- [x] 3.3 Update optional live-readout mode to inspect calibrated arm-joint radians only, without gripper output and without starting `ControlCoordinator`.
- [x] 3.4 Update OpenArm Mini calibration docs for leader zero pose, strict arm-only artifacts, per-joint flip, startup alignment, sender-side clamp, and default calibration storage paths.
- [x] 3.5 Remove live dashboard observed min/max calibration semantics and remove gripper motor 8 from the main calibration path.
- [x] 3.6 Ensure calibration artifacts do not include drive-mode fields, observed ranges, gripper entries, or gripper placeholders.
- [x] 3.7 Ensure the calibration script displays captured raw zero offsets for operator confirmation and records per-joint flip values without requiring gripper endpoint calibration.
- [x] 3.8 Add tests or testable pure helpers for zero-capture artifact writing, arm-only validation, per-joint flip defaults/overrides, gripper omission, and no follower/coordinator startup from calibration.

## 4. Blueprint Wiring

- [x] 4.1 Add an OpenArm Mini teleop blueprint that connects the generic teleop module to the existing OpenArm control coordinator through `joint_command`.
- [x] 4.2 Keep existing Quest teleop blueprints unchanged and verify they still reference their existing Quest modules.
- [x] 4.3 Regenerate `dimos/robot/all_blueprints.py` with the existing blueprint generation test after adding the new blueprint.
- [x] 4.4 Add blueprint build/list tests or update existing blueprint generation coverage so the new blueprint appears and wires `joint_command` correctly.

## 5. Validation

- [x] 5.1 Run focused unit tests for the teleop adapter runtime and revised OpenArm Mini adapter mapping/calibration logic.
- [x] 5.2 Run the blueprint generation test for `dimos/robot/all_blueprints.py`.
- [x] 5.3 Run the relevant fast pytest subset for teleop/OpenArm modules after the revised calibration/runtime changes.
- [ ] 5.4 Perform hardware validation with calibrated OpenArm Mini and OpenArm follower when hardware is available, including missing-calibration and explicit-stop checks.
