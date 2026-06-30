## 1. Message and spec contracts

- [x] 1.1 Add `TSDFGrid` message type with frame id, timestamp, distances, voxel size, truncation distance, origin transform, size, resolution, and optional weights.
- [x] 1.2 Add `ReconstructionStatus` message type with active/paused state, workspace metadata, accepted frame count, dropped frame count, latest integration timestamp, and latest error/status text.
- [x] 1.3 Add `GraspCandidate` and `GraspCandidateArray` message types with pose, jaw width, score, optional id, header, and `to_pose_array()` compatibility.
- [x] 1.4 Add or update serialization/encode/decode helpers and type annotations for new messages.
- [x] 1.5 Add unit tests for `TSDFGrid` shape metadata, voxel-to-frame coordinate semantics, and `GraspCandidateArray.to_pose_array()` ordering.

## 2. Scene reconstruction module

- [x] 2.1 Create a generic `SceneReconstructionModule` consuming `depth_image` and `depth_camera_info` streams plus TF lookups.
- [x] 2.2 Implement workspace configuration using user-facing frame, center, and size while storing TSDF grid origin as the min-corner frame transform.
- [x] 2.3 Implement continuous depth integration with configurable ingest throttle and fixed `reconstruction_fps` publication.
- [x] 2.4 Publish `pointcloud`, `tsdf`, and `status` streams derived from the same accepted depth observations.
- [x] 2.5 Add RPC controls: `reset_scene`, `pause_integration`, `resume_integration`, `set_workspace`, `snapshot_scene`, and `get_reconstruction_status`.
- [x] 2.6 Track and publish dropped-frame status when camera info or TF is missing or stale.
- [x] 2.7 Add unit tests for lifecycle controls, publication throttling, missing-transform handling, and workspace origin conversion.

## 3. TSDF grasp generation module

- [x] 3.1 Add `TSDFGraspGenSpec.generate_grasps_from_tsdf(tsdf: TSDFGrid) -> GraspCandidateArray | None`.
- [x] 3.2 Implement `VGNGraspGenModule` with lazy VGN imports and clear errors for missing optional dependency or model weights.
- [x] 3.3 Subscribe to latest `TSDFGrid` stream and expose RPC generation from latest TSDF.
- [x] 3.4 Convert VGN voxel-coordinate predictions into `TSDFGrid` frame poses using voxel size and grid origin semantics.
- [x] 3.5 Transform grasp candidates into `world` frame and report a clear failure when the transform is unavailable.
- [x] 3.6 Publish primary `GraspCandidateArray` output and optional PoseArray compatibility output.
- [x] 3.7 Add unit tests for lazy import failure, VGN output conversion, world-frame transform behavior, and no-pointcloud-required inference path.

## 4. Wiring and validation

- [x] 4.1 Add opt-in blueprint wiring for depth camera → scene reconstruction → VGN grasp generation without changing existing pick-and-place behavior.
- [x] 4.2 Add smoke tests or lightweight integration tests using synthetic TSDF data and stubbed VGN outputs.
- [x] 4.3 Run targeted pytest for new messages, reconstruction module, and grasp generation module.
- [x] 4.4 Run `openspec status --change "vgn-tsdf-scene-reconstruction"` and verify artifacts remain apply-ready.
