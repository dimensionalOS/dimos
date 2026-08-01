## 1. Registered object targeting contracts

- [x] 1.1 Add `RegisteredObject` message type under `dimos/msgs/perception_msgs/` with object id, name, center, size, frame id, and timestamp.
- [x] 1.2 Add serialization and type tests for `RegisteredObject` metadata round-trip and target bounds fields.
- [x] 1.3 Extend `ObjectSceneRegistrationSpec` with `get_object_by_object_id(object_id: str) -> RegisteredObject | None`.
- [x] 1.4 Implement the typed object-id metadata lookup in `ObjectSceneRegistrationModule` without removing existing pointcloud lookup methods.
- [x] 1.5 Add object registration tests for known object id, unknown object id, and preservation of existing pointcloud lookup behavior.

## 2. Target-conditioned TSDF grasp generation

- [x] 2.1 Extend `TSDFGraspGenSpec` with a target-bounds grasp generation method that accepts target center, target size, and cushion without object ids.
- [x] 2.2 Implement latest-TSDF target-bounds generation in `VGNGraspGenModule` with clear no-TSDF and no-candidate outcomes.
- [x] 2.3 Implement Target-masked TSDF preprocessing inside `VGNGraspGenModule` using target bounds expanded by cushion.
- [x] 2.4 Ensure target masking does not mutate the stored full-scene latest TSDF.
- [x] 2.5 Transform Target bounds into the TSDF grid frame before masking when target and TSDF frames differ.
- [x] 2.6 Preserve existing workspace-level `generate_grasps_from_tsdf(tsdf)` behavior without target masking.
- [x] 2.7 Add unit tests for mask inclusion/exclusion, cushion behavior, no-latest-TSDF handling, transform failure, transformed bounds, and workspace-generation compatibility.

## 3. User-facing grasp orchestration

- [x] 3.1 Add `GraspingModule.generate_grasps_for_object(object_id: str, cushion_m: float = 0.03)` or equivalent user-facing API.
- [x] 3.2 Wire `GraspingModule` to resolve `RegisteredObject` metadata by object id before calling target-bounds TSDF grasp generation.
- [x] 3.3 Return clear user-facing messages for unknown object id, missing latest TSDF, transform failure, and no candidates.
- [x] 3.4 Keep existing `generate_grasps(object_name=..., object_id=...)` pointcloud flow unchanged.
- [x] 3.5 Add unit tests for object-id orchestration success, unknown object id, target-bounds forwarding, and compatibility with the existing pointcloud path.

## 4. Target-conditioned demo update

- [x] 4.1 Add or configure a deterministic xArm observation pose/keyframe so the wrist camera sees the table object before reconstruction starts.
- [x] 4.2 Update the opt-in VGN MuJoCo demo wiring or helper flow to include object registration and target-prompt/name selection.
- [x] 4.3 Add demo selection logic that waits for runtime registered objects and selects only a registered object matching the configured demo target prompt/name.
- [x] 4.4 Add demo logic that uses the selected runtime-generated object id to call `GraspingModule.generate_grasps_for_object(...)` after reconstruction has a latest TSDF.
- [x] 4.5 Add Target bounds visualization under a stable Rerun path such as `world/grasp_target_bounds` alongside grasp candidates.
- [x] 4.6 Ensure no-target and configured-target-not-visible outcomes are reported clearly and do not fall back to workspace-level VGN.
- [x] 4.7 Ensure existing xArm simulation blueprints remain unchanged unless the opt-in demo is selected.
- [x] 4.8 Add smoke or integration tests for demo construction, observation pose configuration, target resolution path, visualization paths, and no-target reporting.

## 5. Validation and documentation

- [x] 5.1 Update documentation with the object-id-targeted demo command, default target selection behavior, observation pose behavior, and expected target-conditioned visual acceptance criteria.
- [x] 5.2 Run targeted pytest for registered object contracts, object registration lookup, target-conditioned VGN module behavior, grasp orchestration, and demo smoke tests.
- [x] 5.3 Run targeted ruff checks for modified Python files.
- [x] 5.4 Run `openspec status --change "target-conditioned-vgn-grasping"` and verify artifacts remain apply-ready.
