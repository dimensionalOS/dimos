## 1. Description Assets and Configuration

- [x] 1.1 Verify the selected OpenYAM upstream revision and available license metadata; package generated `yam.urdf`, gripper-enabled `yam_arm.xacro`, a DimOS-owned Xacro wrapper that instantiates it with a stable arm ID, and all package-relative mesh resources as `data/.lfs/yam_description.tar.gz`, with archive basename, top-level directory, and `LfsPath` component all equal to `yam_description`.
- [x] 1.2 Parse both source models and record source-defined base links, variant-specific planning TCP/end-effector links, six arm-joint limits, gripper limits, home poses, and collision exclusions; explicitly retain unvalidated physical assumptions rather than merging values between sources.
- [x] 1.3 Add `dimos/robot/manipulators/openyam/config.py` with LFS paths, bare-arm and gripper-equipped `RobotModelConfig` variants, six-joint mapping, conditional one-joint gripper configuration, and a mock-hardware factory patterned after A1Z.
- [x] 1.4 Add focused configuration tests that confirm both variant asset paths resolve from a clean cache, mesh package paths, expected links and six arm joints, and direct mock-only gripper configuration.

## 2. Blueprint Integration

- [x] 2.1 Add bare-arm and gripper-equipped OpenYAM basic/planning blueprints using the existing manipulation coordinator and model factories.
- [x] 2.2 Add bare-arm and gripper-equipped OpenYAM keyboard-teleoperation blueprints using the existing FK, manipulation, and visualization surfaces with the corresponding model configurations.
- [x] 2.3 Extend focused blueprint tests to cover both OpenYAM variants and verify they use mock hardware without requiring ROS control, CAN, or physical robot connectivity.
- [x] 2.4 Run `pytest dimos/robot/test_all_blueprints_generation.py` and include the generated `dimos/robot/all_blueprints.py` updates; do not hand-edit the registry.

## 3. Documentation

- [x] 3.1 Check the runnable-blueprint documentation/quick-reference for manipulator listings; add the bare-arm and gripper-equipped, mock-only OpenYAM workflows if that listing is maintained, or document why no user-facing documentation change is required. (No maintained manipulator runnable-blueprint listing exists; `docs/usage/blueprints.md` is conceptual.)

## 4. Verification

- [x] 4.1 Run `openspec validate add-openyam-urdf-support`.
- [x] 4.2 Run the focused OpenYAM configuration and manipulator-blueprint pytest targets.
- [x] 4.3 Run `pytest dimos/robot/test_all_blueprints_generation.py` after the final blueprint changes.
- [x] 4.4 Run the applicable documentation link validation if Markdown documentation changed. (No Markdown documentation changed.)
- [x] 4.5 Manually QA the user surface by confirming the generated OpenYAM blueprint names appear in `dimos list` and that the mock model can be constructed with its URDF resources resolved.
