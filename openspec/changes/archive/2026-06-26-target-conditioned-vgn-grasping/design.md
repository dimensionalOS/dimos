## Context

The current VGN grasp path is intentionally workspace-level: `SceneReconstructionModule` publishes a full scene TSDF, and `VGNGraspGenModule` generates candidates for whatever graspable geometry exists in that TSDF. That matches upstream VGN clutter-removal semantics, but it does not answer commands like “grasp the cup” when multiple objects or surfaces are present.

DimOS already has a user-facing grasp orchestration layer in `GraspingModule`, plus `ObjectSceneRegistrationModule` for perception-backed object lookup. The existing pointcloud flow can resolve an `object_id` into an object pointcloud, while the new TSDF/VGN flow only knows about a latest TSDF. The new target-conditioned path adds one layer between these concepts: `GraspingModule` resolves a non-ambiguous registered object into world-frame target bounds, and the VGN module masks its latest TSDF to those bounds before inference.

The glossary terms for this design are: Grasp target, Object id, Registered object, Target bounds, and Target-masked TSDF.

## Goals / Non-Goals

**Goals:**

- Add a typed `RegisteredObject` metadata contract for object-id-based grasp target resolution.
- Add an object registration lookup that returns object metadata by object id without forcing callers to reconstruct bounds from pointclouds.
- Add a user-facing `GraspingModule` API for generating grasps for a specific object id.
- Keep low-level grasp generation decoupled from `ObjectDB` and object ids by passing target bounds, not object records.
- Build Target-masked TSDF preprocessing inside `VGNGraspGenModule` so the generic reconstruction module remains full-scene and reusable.
- Preserve existing workspace-level TSDF grasp generation and pointcloud grasping behavior.
- Update the opt-in VGN MuJoCo demo to exercise the object-id-targeted flow with deterministic object selection, target visualization, and a wrist-camera pose that sees the object.

**Non-Goals:**

- Do not make VGN semantically understand object classes or object ids.
- Do not make `SceneReconstructionModule` produce object-specific TSDFs.
- Do not replace object detection, tracking, or selection policy in this change.
- Do not require exact object meshes or segmentation masks for v1 target conditioning.
- Do not remove the target-agnostic workspace VGN path.

## Decisions

### 1. User-facing object API lives in `GraspingModule`

`GraspingModule` will expose a user-facing `generate_grasps_for_object(object_id: str, cushion_m: float = 0.03)`-style API. It will resolve the object id through object scene registration, extract target bounds, and call the TSDF grasp generator with those bounds.

Alternatives considered:

- Put object id handling directly in `VGNGraspGenModule`: rejected because that couples grasp backends to `ObjectDB` semantics and makes non-VGN grasp generators inherit perception-specific concerns.
- Put target selection in `SceneReconstructionModule`: rejected because scene reconstruction should remain a generic full-scene producer.

### 2. Add typed `RegisteredObject` metadata lookup

Object scene registration will provide `get_object_by_object_id(object_id: str) -> RegisteredObject | None`. `RegisteredObject` will live under `dimos/msgs/perception_msgs/RegisteredObject.py` and carry object id, name, center, size, frame id, and timestamp.

The contract intentionally carries rough world-frame spatial metadata rather than full pointclouds. Target-conditioned VGN only needs a rough attention region; exact geometry still comes from the TSDF.

Alternatives considered:

- Use a dict: rejected because this crosses module boundaries and should remain typed/testable.
- Recompute bounds from object pointcloud every time: rejected because a first-class metadata lookup is cheaper, clearer, and avoids duplicating object-bound semantics in grasp orchestration.

### 3. Low-level TSDF grasp API accepts target bounds

`TSDFGraspGenSpec` will gain a target-bounds method such as `generate_grasps_for_target_bounds(target_center: Vector3, target_size: Vector3, cushion_m: float = 0.03) -> GraspCandidateArray | None`. The VGN module owns latest TSDF state, so the user-facing object flow does not need to pass a large TSDF through RPC.

An explicit-TSDF variant may be added for tests and offline callers if useful, but the primary live path should use the latest streamed TSDF.

Alternatives considered:

- Pass `RegisteredObject` to graspgen: rejected because grasp generation should not depend on perception record shape.
- Pass object id to graspgen: rejected because object id is a user/perception identity concern, not a low-level grasp generation contract.

### 4. Target-masked TSDF is internal to `VGNGraspGenModule`

The VGN module will construct a Target-masked TSDF by suppressing voxels outside target bounds expanded by `cushion_m`. Suppressed voxels should be set to free/unknown values that prevent VGN from selecting grasps on outside clutter while preserving target geometry inside the cushion.

The original full TSDF is not mutated; masking is a per-request preprocessing step before VGN inference.

Alternatives considered:

- Reconstruct a target-only TSDF upstream: rejected because it fragments reconstruction and makes target selection a reconstruction concern.
- Crop from object pointcloud and reconstruct a new TSDF: rejected for v1 because bbox conditioning is faster and rough bounds are sufficient for VGN to find actual surfaces.

### 5. Demo selection may be automatic only for convenience

The real user-facing API should use explicit object ids after perception has created them. The demo cannot take an object id as a normal startup parameter because the object id is generated only after the blueprint starts, perception observes the scene, and object registration creates runtime records. Therefore the demo is configured by target description, not object id.

Demo behavior should be deterministic:

1. Move or initialize the xArm end effector/wrist camera to a known pre-grasp observation pose that points at the table object before collecting the reconstruction window.
2. Run perception with a configured target prompt/name for the demo object, such as `cup` when using the existing xArm scene.
3. Wait for `ObjectSceneRegistrationModule` to register one or more objects matching the configured target prompt/name.
4. Select a matching runtime registered object deterministically, then use that generated object id for the user-facing `GraspingModule.generate_grasps_for_object(...)` call.
5. If no matching registered object appears before timeout, stop with a clear no-target outcome instead of silently falling back to workspace-level VGN.
6. Resolve that registered object to Target bounds and call the target-conditioned VGN path.

### 6. Demo visualization must show why the grasp is target-conditioned

The demo should visualize enough context to answer “what object did it choose?” It should keep the existing scene products and grasp wireframes, and add target-specific context:

- full scene pointcloud under `world/scene_pointcloud`;
- TSDF surface or voxel points under `world/tsdf_surface`;
- selected Target bounds under `world/grasp_target_bounds`;
- grasp candidates under `world/grasp_candidates`;
- optionally a Target-masked TSDF debug view under `world/target_masked_tsdf` if useful during validation.

The Target bounds visualization is required for v1. The Target-masked TSDF visualization is optional because it can be heavier and the bounds plus candidates are enough to validate target association.

## Risks / Trade-offs

- Target bounds too tight → target surface may be suppressed. Mitigation: default cushion and tests around mask inclusion at bounds edges.
- Target bounds too loose → nearby clutter may remain graspable. Mitigation: make cushion explicit/configurable and document that target bounds are rough attention, not exact segmentation.
- Object frame mismatch → grasps may be generated around the wrong region. Mitigation: require target bounds in a resolvable frame and transform to TSDF frame before masking.
- No latest TSDF available → user-facing object grasp request cannot run. Mitigation: return a clear no-result/status and do not publish stale candidates.
- Ambiguous demo target prompt/name → wrong runtime object selected. Mitigation: select deterministically from registered matches using documented tie-breakers such as highest confidence, then nearest to workspace center; log and visualize selected object id/name and Target bounds.
- Wrist camera default pose does not see the object → reconstruction contains no useful target geometry. Mitigation: add a demo observation pose or keyframe that points the end effector/wrist camera at the table object before integration.
- Demo convenience auto-selection could hide targeting errors → wrong object appears targeted. Mitigation: configure by target prompt/name, wait for runtime registration, select only among matching registered objects, log/visualize selected object id/name and Target bounds, and do not fall back to workspace-level VGN when no registered target is available.
- Active previous OpenSpec changes are not archived yet → this proposal is expressed as additive capabilities rather than modifying main specs until archive time.

## Migration Plan

1. Add typed `RegisteredObject` and object registration lookup while preserving existing pointcloud lookup methods.
2. Extend TSDF grasp spec and VGN module with target-bounds generation and Target-masked TSDF preprocessing.
3. Add `GraspingModule.generate_grasps_for_object` orchestration using object id → registered object → target bounds.
4. Update the opt-in demo to move/initialize the wrist camera to an observation pose, include object registration/selection, visualize selected Target bounds, and call the targeted API.
5. Keep existing workspace-level TSDF and pointcloud grasp APIs as compatibility paths.

Rollback is straightforward because all APIs are additive; existing workspace-level behavior remains available.

## Open Questions

- Which exact xArm joint pose/keyframe should be used as the default wrist-camera observation pose for the existing table object?
- Should the default demo target prompt/name be `cup`, or should it target the easiest visible object in the current scene after the observation pose is chosen?
