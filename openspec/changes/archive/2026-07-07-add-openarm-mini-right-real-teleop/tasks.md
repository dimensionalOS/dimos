## 1. OpenArm Mini command naming

- [x] 1.1 Add configuration support for OpenArm Mini teleop commands to target blueprint-specific follower joint names, including ManipulationModule-compatible global names.
- [x] 1.2 Preserve existing default local-name behavior for bimanual OpenArm Mini teleop and left visualization paths.
- [x] 1.3 Add tests proving configured right-arm global target names are emitted and gripper values remain omitted.

## 2. Right real-teleop blueprint wiring

- [x] 2.1 Add a right-arm hardware/model setup for the new blueprint using `right_arm` as the robot/hardware identity and `openarm_right_joint1` through `openarm_right_joint7` as local model joints.
- [x] 2.2 Add `openarm-mini-right-teleop-viser` wiring `OpenArmMiniTeleopModule`, `ControlCoordinator`, and `ManipulationModule` with Viser visualization.
- [x] 2.3 Configure the blueprint's OpenArm Mini right leader defaults with `enabled_sides=("right",)` and `port_right="/dev/ttyACM0"`.
- [x] 2.4 Configure follower hardware to use the real OpenArm adapter with `global_config.can_port` when provided and mock hardware when it is absent.
- [x] 2.5 Configure one right-arm servo task covering the right-arm global coordinator joint names.

## 3. ManipulationModule Viser integration

- [x] 3.1 Ensure `ControlCoordinator.coordinator_joint_state` autoconnects to `ManipulationModule.coordinator_joint_state` for the new blueprint.
- [x] 3.2 Ensure the ManipulationModule robot model is the right OpenArm model with robot name `right_arm` and Viser backend enabled.
- [x] 3.3 Ensure the custom OpenArm Mini Viser renderer is not used by the new right real-teleop blueprint.

## 4. Blueprint registry, docs, and tests

- [x] 4.1 Add blueprint tests covering mock fallback, real `--can-port` selection, right leader default port, global joint naming, Viser ManipulationModule usage, and absence of the custom renderer.
- [x] 4.2 Regenerate and verify `dimos/robot/all_blueprints.py` includes `openarm-mini-right-teleop-viser`.
- [x] 4.3 Update OpenArm Mini teleop docs with mock and real-follower run commands, calibration prerequisites, `--can-port` safety semantics, and `/dev/ttyACM0` right leader default.

## 5. Validation

- [x] 5.1 Run focused OpenArm Mini teleop and OpenArm blueprint tests.
- [x] 5.2 Run ruff on touched OpenArm Mini teleop, OpenArm blueprint, and test files.
- [x] 5.3 Validate blueprint registry generation in CI/non-mutating mode after regeneration.
- [x] 5.4 Validate OpenSpec change artifacts.
- [x] 5.5 Manually run mock-follower bring-up with a real OpenArm Mini right leader and no `--can-port`.
- [x] 5.6 Manually run real right-follower bring-up with `--can-port` when hardware is available and startup alignment is verified.
