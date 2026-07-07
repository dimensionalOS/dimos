## 1. TF-Derived Camera-Pose Odometry

- [ ] 1.1 Add a reusable TF pose source module with configurable `target_frame`, `source_frame`, TF tolerance, and fixed publish rate.
- [ ] 1.2 Publish pose-only `Odometry` with `frame_id=target_frame`, `child_frame_id=source_frame`, zero/default twist, and pose copied from the TF lookup.
- [ ] 1.3 Add unit tests for successful TF lookup, missing/stale TF behavior, frame ids, and fixed-rate lifecycle behavior.

## 2. PointCloud Self-Filtering

- [ ] 2.1 Add a reusable `PointCloudSelfFilter` module that consumes `PointCloud2` and publishes filtered `PointCloud2`.
- [ ] 2.2 Support TF-anchored primitive exclusion regions for at least sphere and box shapes.
- [ ] 2.3 Preserve the input cloud frame and timestamp while using TF internally to transform exclusion regions into the cloud frame.
- [ ] 2.4 Add configurable behavior for missing exclusion-region TF, including diagnostics.
- [ ] 2.5 Add unit tests for sphere filtering, box filtering, frame preservation, and missing-TF behavior.

## 3. RoboPlan Octree Planning Projection

- [ ] 3.1 Extend manipulation planning models/enums/configuration to represent a pointcloud-map or octree dynamic obstacle projection without breaking existing obstacle types.
- [ ] 3.2 Extend `RoboPlanWorld` to create RoboPlan/Coal octree collision geometry from a `PointCloud2` point set and configured resolution.
- [ ] 3.3 Implement stable-id replace-on-update behavior for the planning voxel-map octree obstacle.
- [ ] 3.4 Ensure octree obstacles remain dynamic scene state after RoboPlan world finalization and are not baked into generated URDF/SRDF assets.
- [ ] 3.5 Add backend tests for octree add, replace, remove, resolution handling, and collision-check participation.

## 4. Manipulation Planning Map Input

- [ ] 4.1 Add a `planning_voxel_map: In[PointCloud2]` or equivalent input to the manipulation planning module boundary.
- [ ] 4.2 Cache the latest planning voxel map and expose a pre-plan sync path that applies it to the RoboPlan world as the stable octree obstacle.
- [ ] 4.3 Add world/backend config for `octree_resolution`, defaulting to the voxel mapper resolution in demo wiring when no explicit override is provided.
- [ ] 4.4 Add tests that planning sync replaces stale map obstacles and does not append duplicate octrees.

## 5. Demo Wiring

- [ ] 5.1 Add or derive an xArm MuJoCo manipulation/Viser demo blueprint that wires `MujocoSimModule.pointcloud -> PointCloudSelfFilter -> RayTracingVoxelMap.lidar`.
- [ ] 5.2 Wire `TfPoseSource.odometry -> RayTracingVoxelMap.odometry` using `world -> wrist_camera_color_optical_frame`.
- [ ] 5.3 Wire `RayTracingVoxelMap.global_map -> ManipulationModule.planning_voxel_map` for RoboPlan collision checking.
- [ ] 5.4 Configure demo self-filter regions for wrist/gripper geometry, including `link7` and `link_tcp` anchored primitives.
- [ ] 5.5 Include Viser in the final demo stack with simulation and manipulation planning.
- [ ] 5.6 Render the planning pointcloud/voxel-map layer directly in Viser and update it from the latest `global_map`.
- [ ] 5.7 Keep existing agentic grasp demo blueprints unchanged; this demo is for manual manipulation planning verification only.

## 6. Validation and Documentation

- [ ] 6.1 Add blueprint or integration tests that verify the voxel-backed demo includes the TF pose source, pointcloud self-filter, Rust ray-tracing mapper, manipulation module, and RoboPlan octree path.
- [ ] 6.2 Add a smoke or gated validation path that confirms a published `global_map` reaches the planning voxel-map input before manual manipulation planning.
- [ ] 6.3 Add validation that the final demo includes Viser and exposes the planning map visualization layer.
- [ ] 6.4 Document the demo wiring, Viser map visualization, key configuration values, and known v1 limitations: primitive self-filtering, no Scene Registry integration, no required workspace crop, and out-of-band voxel resolution.
- [ ] 6.5 Run targeted unit tests for new modules and existing affected manipulation/RoboPlan demo tests.
