## 1. Left-only OpenArm Mini adapter support

- [x] 1.1 Add explicit OpenArm Mini adapter side-selection configuration with the existing bimanual behavior preserved as the default.
- [x] 1.2 Ensure left-only operation loads only the left calibration artifact, opens only the left Feetech serial port, and emits only left OpenArm arm-joint names.
- [x] 1.3 Add adapter tests covering left-only startup, right-side omission, emitted joint names, and unchanged bimanual default behavior.

## 2. Visualization-only Viser renderer

- [x] 2.1 Add a visualization-only module that subscribes to `joint_command` and renders an OpenArm arm model in Viser without coordinator or hardware components.
- [x] 2.2 Reuse existing Viser scene/URDF preparation helpers so joint updates use the loaded model's actuated-joint order.
- [x] 2.3 Normalize incoming `JointState` values by joint name, report missing required left arm joints, and skip incomplete commands.
- [x] 2.4 Add renderer tests for joint-name reordering, missing-joint handling, gripper omission, and Viser helper integration seams.

## 3. Blueprint and registry wiring

- [x] 3.1 Add a static `openarm_mini_left_teleop_viser` blueprint wiring `TeleopModule(OpenArmMiniTeleopAdapter(left only))` to the Viser renderer via `joint_command`.
- [x] 3.2 Ensure the blueprint does not include `ControlCoordinator`, OpenArm follower hardware, mock follower hardware, or any physical execution path.
- [x] 3.3 Regenerate `dimos/robot/all_blueprints.py` and verify `openarm-mini-left-teleop-viser` is listed.
- [x] 3.4 Add blueprint tests for left-only adapter selection, `joint_command` wiring, registry naming, and absence of coordinator/hardware atoms.

## 4. Documentation

- [x] 4.1 Document the `openarm-mini-left-teleop-viser` command, required real OpenArm Mini left leader hardware, left calibration requirement, and Viser dependency expectation.
- [x] 4.2 Document that the follower side is visualization-only and that this workflow does not validate coordinator routing or physical OpenArm follower execution.

## 5. Validation

- [x] 5.1 Run focused tests for OpenArm Mini adapter side selection and Viser renderer behavior.
- [x] 5.2 Run OpenArm/manipulator blueprint tests and registry generation validation.
- [x] 5.3 Run relevant lint/format checks for modified teleop, visualization, and OpenArm blueprint files.
- [ ] 5.4 Perform manual validation with a real OpenArm Mini left leader connected and no OpenArm follower hardware connected, confirming Viser updates from leader motion.
