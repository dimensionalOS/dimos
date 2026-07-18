## 1. Description Assets and Configuration

- [x] 1.1 Verify the selected OpenYAM upstream revision and available license metadata; package `yam_arm.xacro`, the DimOS-owned `yam_gripper.urdf.xacro` wrapper, and all package-relative mesh resources as `data/.lfs/yam_description.tar.gz`, with archive basename, top-level directory, and `LfsPath` component all equal to `yam_description`.
- [x] 1.2 Parse the wrapped model and record its source-defined base link, planning TCP/end-effector link, six arm-joint limits, gripper limits, home pose, and collision exclusions; retain unvalidated physical assumptions, apply orientation corrections to both visual and collision meshes, and preserve mesh bytes, XYZ origins, joints/axes, and inertials without adding custom collision exclusions.
- [x] 1.3 Add `dimos/robot/manipulators/openyam/config.py` with the wrapper path, six-joint mapping, one-joint direct gripper configuration, and a mock-only hardware factory patterned after A1Z.
- [x] 1.4 Add focused configuration tests that confirm the wrapper asset resolves from a clean cache, mesh package paths, expected links and six arm joints, direct mock-only gripper configuration, and the visual-and-collision mesh-orientation scope.

## 2. Blueprint Integration

- [x] 2.1 Add the gripper-equipped OpenYAM basic/planning blueprint using the existing manipulation coordinator and model factory.
- [x] 2.2 Add the gripper-equipped OpenYAM keyboard-teleoperation blueprint using the existing FK, manipulation, and visualization surfaces.
- [x] 2.3 Add focused blueprint tests to verify the workflow uses mock hardware without requiring ROS control, CAN, or physical robot connectivity.
- [x] 2.4 Run `pytest dimos/robot/test_all_blueprints_generation.py` and include the generated `dimos/robot/all_blueprints.py` updates; do not hand-edit the registry.

## 3. Documentation

- [x] 3.1 Check the runnable-blueprint documentation/quick-reference for manipulator listings; add the exactly one gripper-equipped, mock-only OpenYAM workflow if that listing is maintained, or document why no user-facing documentation change is required.

## 4. Verification

- [x] 4.1 Run `openspec validate add-openyam-urdf-support`.
- [x] 4.2 Run the focused OpenYAM configuration and manipulator-blueprint pytest targets.
- [x] 4.3 Run `pytest dimos/robot/test_all_blueprints_generation.py` after the final blueprint changes.
- [x] 4.4 Run the applicable documentation link validation if Markdown documentation changed. (No separate documentation link checker is configured; strict OpenSpec validation passed.)
- [x] 4.5 Manually QA the user surface by confirming the OpenYAM blueprint names appear in `dimos list` and that the mock model can be constructed with its wrapper resources resolved.
