## Why

DimOS supports A1Z as a runnable manipulator description, but has no equivalent integration for OpenYAM. Adding OpenYAM lets developers use its gripper-equipped arm model with the existing planning, teleoperation, and visualization workflows.

The upstream description provides two complementary sources: generated `yam.urdf` for the bare six-axis arm and `yam_arm.xacro` for the arm with gripper and TCP links. DimOS should make both choices explicit rather than presenting gripper geometry when a user selects the bare-arm robot.

## What Changes

- Add OpenYAM description assets and robot-specific manipulation configurations for bare-arm and gripper-equipped models.
- Add runnable OpenYAM manipulation blueprints that compose the existing planning, teleoperation, and visualization surfaces.
- Expose the six controlled arm joints for both variants and a direct mock-gripper command channel only for the gripper-equipped variant; no finger-state visualization or synchronization is in scope.
- Add focused validation for the model configuration and blueprint wiring.
- Do not add real OpenYAM hardware control in this change.

## Affected DimOS Surfaces

- Modules/streams: Existing manipulation planning, FK, coordinator, and visualization modules configured for the OpenYAM model.
- Blueprints/CLI: New OpenYAM basic/planner and teleoperation blueprints, including generated blueprint-registry entries.
- Skills/MCP: None.
- Hardware/simulation/replay: Mock manipulation hardware and URDF/Xacro-backed planning/visualization only; no physical robot driver or MuJoCo simulation integration.
- Docs/generated registries: Generated `dimos/robot/all_blueprints.py` and any needed user-facing blueprint documentation.

## Capabilities

### New Capabilities
- `openyam-manipulator-support`: Configure and run bare-arm and gripper-equipped OpenYAM variants through DimOS's manipulation planning and teleoperation workflows.

### Modified Capabilities

None.

## Impact

Developers gain bare-arm and gripper-equipped OpenYAM robot options using the established A1Z-style workflows. The change adds description assets and depends on their upstream mesh/license provenance being acceptable for redistribution. It does not alter existing robots or public skills. Validation must cover URDF/Xacro asset resolution, expected links and joints, variant-specific gripper configuration, blueprint generation, and the existing blueprint-level test paths.
