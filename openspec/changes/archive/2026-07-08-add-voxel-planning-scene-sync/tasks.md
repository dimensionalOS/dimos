## 1. TF-Derived Camera-Pose Odometry

- [x] 1.1 Add a reusable TF pose source module with configurable `target_frame`, `source_frame`, TF tolerance, and fixed publish rate.
- [x] 1.2 Publish pose-only `Odometry` with `frame_id=target_frame`, `child_frame_id=source_frame`, zero/default twist, and pose copied from the TF lookup.
- [x] 1.3 Add unit tests for successful TF lookup, missing/stale TF behavior, frame ids, and fixed-rate lifecycle behavior.

## 2. PointCloud Self-Filtering

- [x] 2.1 Add a reusable `PointCloudSelfFilter` module that consumes `PointCloud2` and publishes filtered `PointCloud2`.
- [x] 2.2 Support TF-anchored primitive exclusion regions for at least sphere and box shapes.
- [x] 2.3 Preserve the input cloud frame and timestamp while using TF internally to transform exclusion regions into the cloud frame.
- [x] 2.4 Add configurable behavior for missing exclusion-region TF, including diagnostics.
- [x] 2.5 Add unit tests for sphere filtering, box filtering, frame preservation, and missing-TF behavior.

## 3. RoboPlan Octree Planning Projection

- [x] 3.1 Extend manipulation planning models/enums/configuration to represent a pointcloud-map or octree dynamic obstacle projection without breaking existing obstacle types.
- [x] 3.2 Extend `RoboPlanWorld` to create RoboPlan/Coal octree collision geometry from a `PointCloud2` point set and configured resolution.
- [x] 3.3 Implement stable-id replace-on-update behavior for the planning voxel-map octree obstacle.
- [x] 3.4 Ensure octree obstacles remain dynamic scene state after RoboPlan world finalization and are not baked into generated URDF/SRDF assets.
- [x] 3.5 Add backend tests for octree add, replace, remove, resolution handling, and collision-check participation.

## 4. Manipulation Planning Map Input

- [x] 4.1 Add a `planning_voxel_map: In[PointCloud2]` or equivalent input to the manipulation planning module boundary.
- [x] 4.2 Cache the latest planning voxel map and expose a pre-plan sync path that applies it to the RoboPlan world as the stable octree obstacle.
- [x] 4.3 Add world/backend config for `octree_resolution`, defaulting to the voxel mapper resolution in demo wiring when no explicit override is provided.
- [x] 4.4 Add tests that planning sync replaces stale map obstacles and does not append duplicate octrees.

## 5. Demo Wiring

- [x] 5.1 Add or derive an xArm MuJoCo manipulation/Viser demo blueprint that wires `MujocoSimModule.pointcloud -> PointCloudSelfFilter -> RayTracingVoxelMap.lidar`.
- [x] 5.2 Wire `TfPoseSource.odometry -> RayTracingVoxelMap.odometry` using `world -> wrist_camera_color_optical_frame`.
- [x] 5.3 Wire `RayTracingVoxelMap.global_map -> ManipulationModule.planning_voxel_map` for RoboPlan collision checking.
- [x] 5.4 Configure demo self-filter regions for wrist/gripper geometry, including `link7` and `link_tcp` anchored primitives.
- [x] 5.5 Include Viser in the final demo stack with simulation and manipulation planning.
- [x] 5.6 Render the planning pointcloud/voxel-map layer directly in Viser and update it from the latest `global_map`.
- [x] 5.7 Keep existing agentic grasp demo blueprints unchanged; this demo is for manual manipulation planning verification only.

## 6. Validation and Documentation

- [x] 6.1 Add blueprint or integration tests that verify the voxel-backed demo includes the TF pose source, pointcloud self-filter, Rust ray-tracing mapper, manipulation module, and RoboPlan octree path.
- [x] 6.2 Add a smoke or gated validation path that confirms a published `global_map` reaches the planning voxel-map input before manual manipulation planning.
- [x] 6.3 Add validation that the final demo includes Viser and exposes the planning map visualization layer.
- [x] 6.4 Document the demo wiring, Viser map visualization, key configuration values, and known v1 limitations: primitive self-filtering, no Scene Registry integration, no required workspace crop, and out-of-band voxel resolution.
- [x] 6.5 Run targeted unit tests for new modules and existing affected manipulation/RoboPlan demo tests.
