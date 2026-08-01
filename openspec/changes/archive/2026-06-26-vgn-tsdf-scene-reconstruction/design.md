## Context

DimOS already has depth-capable camera streams (`depth_image`, `depth_camera_info`, TF, and optional `pointcloud`) and a grasp-generation orchestration seam (`GraspingModule` + `GraspGenSpec`). What is missing is a native learned grasp backend and the scene reconstruction representation required by VGN.

Official VGN reconstructs a TSDF from depth images, camera intrinsics, and camera poses, then runs inference on a grid shaped like `(1, 40, 40, 40)`. It does not build its canonical input from a pointcloud alone. DimOS should therefore model TSDF as a reusable scene product produced beside pointclouds, not as a private VGN adapter detail.

## Goals / Non-Goals

**Goals:**
- Introduce a reusable scene reconstruction module that consumes depth observations and publishes pointcloud, TSDF, and status outputs.
- Introduce a `TSDFGrid` message aligned with VGN/Open3D grid semantics.
- Introduce TSDF-native grasp generation with VGN as the first backend.
- Preserve existing pointcloud-oriented `GraspGenSpec` and existing heuristic pick behavior.
- Publish grasp outputs as scored gripper candidates in `world` frame, with PoseArray compatibility.
- Keep high-rate/large sensor and reconstruction payloads on streams; use RPC for control and lifecycle.

**Non-Goals:**
- Executing robot picks or closing the gripper using VGN output.
- Supporting cluttered bin-picking quality in the first pass.
- Making Viser the primary visualization path in this change.
- Replacing `ObjectSceneRegistrationModule` or semantic object registration.
- Creating a pointcloud-only approximation as the primary VGN path.

## Decisions

### Scene reconstruction is generic, not VGN-private

Create a `SceneReconstructionModule` that consumes depth streams and TF and publishes reconstructed scene products. VGN consumes the TSDF product.

Alternatives considered:
- Put TSDF integration inside `VGNGraspGenModule`: simpler initially, but hides a reusable scene representation and couples reconstruction lifecycle to one model.
- Build TSDF from `PointCloud2`: easier to wire to existing pointcloud APIs, but loses camera-ray/free-space information needed for faithful TSDF integration.

### TSDFGrid uses min-corner grid semantics

`TSDFGrid` carries `distances` shaped `(1, X, Y, Z)`, `voxel_size`, `truncation_distance`, `origin`, `size`, `resolution`, optional `weights`, `frame_id`, and `ts`. Voxel `[0,0,0]` is located at the grid origin/min corner. Workspace RPCs can accept a user-friendly center and convert internally.

Alternatives considered:
- Center-origin grids: friendlier for humans but adds conversion friction and ambiguity versus VGN/Open3D output.
- VGN-specific wrapper object: less reusable and harder to stream across DimOS modules.

### Reconstruction continuously ingests but publishes at a fixed output rate

The reconstruction module may ingest at camera rate or a configured throttle, but publishes `pointcloud`, `tsdf`, and `status` at `reconstruction_fps` such as 2 Hz. RPCs control lifecycle: reset, pause/resume integration, set workspace, snapshot, and status.

Alternatives considered:
- One-shot RPC carrying depth/camera/TF payloads: easier to reason about for one request, but sends large pickled payloads through RPC and fights DimOS stream architecture.
- Fully manual start/stop-only reconstruction: faithful to scan workflows, but less useful as a general scene product producer.

### TSDF grasp generation gets a new spec

Add `TSDFGraspGenSpec.generate_grasps_from_tsdf(tsdf: TSDFGrid) -> GraspCandidateArray | None`. Keep the existing pointcloud `GraspGenSpec` unchanged for pointcloud-based backends.

Alternatives considered:
- Change `GraspGenSpec` to accept TSDF: would break the existing seam and mix two different input contracts.
- Add optional pointcloud to TSDF generation spec: VGN does not require it for inference, so it should remain a sibling visualization/collision/debug output.

### Grasp candidates are richer than PoseArray

Introduce `GraspCandidate` and `GraspCandidateArray` with pose, jaw width, score, and optional id. Publish this as primary output and provide `to_pose_array()` compatibility.

Alternatives considered:
- Only publish `PoseArray`: insufficient for VGN score, gripper width, visualization, ranking, and later execution.

### VGN outputs world-frame candidates in v1

The VGN module transforms voxel/grid-frame grasps through `TSDFGrid.origin` and TF into `world`, then publishes `GraspCandidateArray.header.frame_id = "world"`. If the transform is unavailable, it returns no result and logs/raises a clear error.

Alternatives considered:
- Configurable target frame: more flexible, but unnecessary for the first integration and creates more validation combinations.

## Risks / Trade-offs

- VGN import triggers ROS visualization side effects in non-ROS environments → lazy-import VGN inside the backend and isolate or shim visualization imports rather than importing `vgn.detection` at module import time.
- TSDF coordinate conventions can silently produce shifted grasps → require unit tests for voxel-to-world conversion and frame id/origin semantics.
- Reconstruction payloads are large → stream them and avoid RPC transport for TSDF/depth arrays.
- VGN model artifacts may not be present → fail with a clear backend status and installation/model-path message.
- Depth/camera/TF synchronization can be brittle → align by timestamps where available and expose reconstruction status with accepted/dropped frame counts.
- `CONTEXT.md` currently has merge conflict markers → do not update glossary/docs there until conflicts are resolved.

## Migration Plan

1. Add new message/spec/module types without changing existing grasp or pick paths.
2. Wire VGN into new opt-in blueprints/demos only.
3. Keep `GraspingModule` pointcloud flow and `PickAndPlaceModule` heuristic flow untouched.
4. Rollback is removing the opt-in modules/blueprints while leaving existing behavior unchanged.

## Open Questions

- Exact location/name for the reconstruction package: `dimos/perception/reconstruction` versus `dimos/navigation` or another namespace.
- Whether `TSDFGrid.weights` should be required immediately or optional until a downstream consumer needs confidence/observation counts.
- Exact model artifact discovery convention for VGN weights.
