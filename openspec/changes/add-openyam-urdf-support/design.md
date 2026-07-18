## Context

The manipulation stack accepts a `RobotModelConfig` that supplies a URDF path, package roots, controlled-joint mapping, base and end-effector links, collision exclusions, and home configuration. A1Z demonstrates this convention through an LFS-backed description archive, configuration factories, and basic and teleoperation blueprints. The generic planning and visualization layers consume the configured model path and package paths.

The selected upstream OpenYAM source provides `yam_arm.xacro` with gripper/TCP links, `finger_joint1`, and a mimicked second finger. DimOS's existing model preparation supports Xacro input. The DimOS-owned `yam_gripper.urdf.xacro` wrapper instantiates that macro as the single planning and visualization model.

## Goals / Non-Goals

**Goals:**
- Provide an LFS-backed gripper-equipped OpenYAM description that resolves for planning and visualization.
- Provide OpenYAM model and mock-hardware factories consistent with existing manipulator integrations.
- Make OpenYAM runnable via basic/planner and keyboard teleoperation blueprints.
- Validate the expected model structure and blueprint registration.

**Non-Goals:**
- No physical OpenYAM CAN, ros2_control, or OpenArm driver integration.
- No MuJoCo simulation or replay integration.
- No new skills, MCP tools, streams, or DimOS Python `Spec` Protocols.

## DimOS Architecture

The change adds a sibling OpenYAM robot package under `dimos/robot/manipulators/`, modeled on A1Z. Its configuration module owns an LFS archive whose basename and top-level package directory are both `yam_description`, and exposes `yam_gripper.urdf.xacro`, a DimOS-owned wrapper that instantiates upstream `yam_arm.xacro` with the stable `arm_id="yam"`. The configuration defines the six arm joints as the coordinator-controlled arm group and direct mock hardware gripper control; the gripper's finger joints remain outside the six-joint planning group.

The basic blueprint composes the current planning/coordinator stack with the OpenYAM hardware and model factories. The teleoperation blueprint composes the existing keyboard teleoperation, FK, manipulation, and visualization modules using the same gripper-equipped model configuration. Both are ordinary built-in blueprints and are discovered through the generated `dimos/robot/all_blueprints.py` registry. No new typed stream contracts, RPC references, adapters, or agent-facing skills are introduced.

## Decisions

1. **Use one DimOS-owned wrapper.** `yam_gripper.urdf.xacro` instantiates `yam_arm.xacro` as the gripper-equipped planning model because the upstream Xacro defines only a macro.
2. **Keep gripper control separate from the six-arm-joint planning group.** The source has no moving finger geometry, so this change provides direct mock-gripper control only; it does not promise finger-state visualization or kinematic synchronization.
3. **Scope mesh orientation corrections narrowly.** Corrected visual and collision mesh orientations are authored in the DimOS-owned wrapper only. The mesh bytes, XYZ origins, joints/axes, and inertials remain unchanged; no custom collision exclusions or physical-behavior claims are introduced.
4. **Start with mock hardware.** It enables deterministic planning and teleoperation validation without asserting an unsupported physical-control contract.
5. **Vendor the description as an LFS archive.** This preserves package-relative mesh resolution and follows the repository's existing robot-description delivery mechanism.
6. **Regenerate, never hand-edit, the blueprint registry.** Run `pytest dimos/robot/test_all_blueprints_generation.py` after adding the built-in blueprints.

## Safety / Simulation / Replay

The initial blueprints are mock-only and must not claim to control a physical OpenYAM robot. Planning and visualization may be manually inspected with the robot at its configured home state; no hardware actuation or safety certification is part of this change. There is no simulation or replay behavior to validate beyond the existing URDF-backed visualization path.

## Risks / Trade-offs

- The upstream Xacro's inertial and joint-limit data must be preserved rather than replaced by unvalidated values.
- Mesh redistribution provenance is not fully established by the upstream repository's metadata. Verify the source revision and license before adding the archive.
- The gripper-equipped model's hand TCP is manually authored upstream and must be treated as an unvalidated default frame until physical validation is available.
- Archive basename, archive top-level directory, and the `LfsPath` first component must remain identical for lazy asset extraction to resolve paths.

## Migration / Rollout

This is additive and does not alter existing robot configurations. Add the archive and OpenYAM package, run the blueprint-registry generation test, and include the resulting generated registry update. Document the new blueprint alongside other runnable manipulator blueprints if the project maintains a user-facing list. A rollback removes the OpenYAM package, archive, and generated registry entries without changing shared manipulation interfaces.

## Open Questions

- Which upstream Xacro revision and archive layout have approved mesh redistribution provenance?
- What home pose, self-collision exclusions, and mock gripper limits are appropriate after parsing the wrapped model?
