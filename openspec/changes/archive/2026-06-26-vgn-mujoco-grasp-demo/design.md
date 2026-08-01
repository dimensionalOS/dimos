## Context

The core VGN reconstruction change defines the TSDF scene product and VGN grasp candidate output. This companion change validates that pipeline in a full simulated setting. The existing xArm simulation blueprint already uses `MujocoSimModule` with `camera_name="wrist_camera"` and `base_frame_id="link7"`, and Rerun visualization is already available through `RerunBridgeModule` and `PointCloud2.to_rerun()`.

The first demo should prove geometry and frame correctness, not manipulation execution. A simple scene with one object is enough to show whether wrist depth, TSDF reconstruction, VGN inference, and grasp candidate visualization line up.

## Goals / Non-Goals

**Goals:**
- Provide an opt-in MuJoCo demo that runs xArm7, wrist depth camera, scene reconstruction, VGN grasp generation, and Rerun visualization together.
- Use a minimal scene: robot, gripper, table, and one simple stable graspable object.
- Visualize scene pointcloud, optional TSDF surface/voxel points, and simplified gripper candidates in world coordinates.
- Make the best/top grasps visually distinct from lower-scoring candidates.
- Keep the demo deterministic enough for repeated local debugging.

**Non-Goals:**
- Autonomous physical or simulated pick execution.
- Cluttered object piles or benchmark-quality grasp evaluation.
- Viser as the primary visualization path in v1.
- General multi-object scene generation.

## Decisions

### Use Rerun first

Use Rerun for v1 visualization because the xArm simulation stack already includes `RerunBridgeModule`, and `PointCloud2.to_rerun()` already exists.

Alternatives considered:
- Viser: preferred long-term by the user, but would require more new infrastructure before verifying VGN geometry.
- Open3D-only debug windows: useful for local debugging, but less integrated with the existing DimOS visualization stack.

### Render grasp candidates as simplified gripper wireframes

Add `GraspCandidateArray.to_rerun()` or an equivalent visualization helper that logs simplified grippers with `rr.LineStrips3D`. Each candidate pose transforms local gripper geometry into world; jaw width controls finger separation; score controls color/ranking.

Use fixed v1 visualization config:

```python
GraspVisConfig:
    max_grasps: int = 50
    top_k_highlight: int = 5
    finger_length_m: float = 0.055
    palm_depth_m: float = 0.035
    finger_thickness_m: float = 0.004
    default_width_m: float = 0.08
    min_score: float = 0.0
    top_color: tuple[int, int, int] = (0, 255, 80)
    good_color: tuple[int, int, int] = (255, 220, 0)
    low_color: tuple[int, int, int] = (255, 80, 0)
```

Alternatives considered:
- Pose arrows only: too thin; does not show jaw width or gripper envelope.
- Mesh grippers: prettier but more implementation work than needed for validation.

### Keep the scene simple

Use xArm7 with gripper, table, and one simple object such as a cube or mug-like cylinder with a distinct material. Avoid clutter for v1.

Alternatives considered:
- Multiple cluttered objects: better stress test, but makes first failure diagnosis harder.
- No object, synthetic TSDF only: easier to run, but does not validate MuJoCo camera and scene visualization together.

### Wrist camera uses existing MuJoCo camera configuration

Use the existing wrist camera concept attached to `link7` through `MujocoSimModule` configuration. The demo should ensure depth and camera-info streams are enabled and that TF makes the wrist camera frame resolvable to `world`.

Alternatives considered:
- Static external camera: simpler view, but does not validate the wrist-camera use case needed for real manipulation.

### Demo is opt-in and separate from existing xArm perception sim

Add a new demo blueprint/script rather than changing the default xArm perception simulation behavior.

Alternatives considered:
- Modify `xarm_perception_sim` directly: simpler to find, but risks changing existing workflows and tests.

## Risks / Trade-offs

- MuJoCo asset/keyframe changes can break derived scene paths → keep demo asset additions minimal and verify the selected scene path exists.
- Wrist camera frame or depth intrinsics may not match reconstruction assumptions → add explicit Rerun visualization of pointcloud/TSDF/grasp frames and fail clearly when TF is missing.
- VGN may produce no grasps on a too-simple or badly scaled object → choose object size compatible with parallel-jaw gripper and show reconstruction status/model status.
- Rerun line-strip APIs may differ by version → keep visualization helper small and covered by a smoke test or import test.
- This change depends on the core reconstruction/VGN change → implement after or alongside `vgn-tsdf-scene-reconstruction` and avoid duplicating core types.

## Migration Plan

1. Add demo assets and visualization helpers behind opt-in demo names.
2. Wire a new demo blueprint/script without changing existing simulation blueprints.
3. Validate visually in Rerun and with smoke tests.
4. Rollback is removing the opt-in demo and helper wiring.

## Open Questions

- Exact object shape for the first committed scene: cube versus cylinder/mug-like object.
- Whether the demo should include scripted arm scan poses in v1 or rely on static wrist camera placement first.
- Whether TSDF surface visualization should be produced by the reconstruction module or a demo-only helper.
