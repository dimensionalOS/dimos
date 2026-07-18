## Why

DimOS supports A1Z as a runnable manipulator description, but has no equivalent integration for OpenYAM. Adding OpenYAM lets developers use exactly one gripper-equipped arm model with the existing planning, teleoperation, and visualization workflows.

The OpenYAM description is exposed through the DimOS-owned `yam_gripper.urdf.xacro` wrapper, which supplies the complete gripper-equipped model and its TCP links.

## What Changes

- Add the OpenYAM description assets and exactly one gripper-equipped robot-specific manipulation configuration, using the DimOS-owned `yam_gripper.urdf.xacro` wrapper.
- Add runnable OpenYAM manipulation blueprints that compose the existing planning, teleoperation, and visualization surfaces for that configuration.
- Expose the six controlled arm joints and a direct mock-gripper command channel; finger-state visualization or synchronization is not in scope.
- Limit corrected mesh orientation to the wrapper's visual and collision mesh presentations; preserve mesh bytes, XYZ origins, joints/axes, and inertials, and add no custom collision exclusions or hardware behavior.
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
- `openyam-manipulator-support`: Configure and run exactly one gripper-equipped OpenYAM model through DimOS's manipulation planning and teleoperation workflows.

### Modified Capabilities

None.

## Impact

Developers gain exactly one gripper-equipped OpenYAM robot configuration using the established A1Z-style workflows. The change adds description assets and depends on their upstream mesh/license provenance being acceptable for redistribution. It does not alter existing robots or public skills. Validation must cover wrapper-based Xacro asset resolution, expected links and joints, direct mock-gripper configuration, blueprint generation, and the existing blueprint-level test paths.
