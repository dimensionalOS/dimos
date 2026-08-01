## Context

The xArm MuJoCo manipulation stack already has the ingredients needed to observe a scene: `MujocoSimModule` publishes wrist RGBD pointclouds, the manipulation stack publishes link TF for configured links, and `MujocoSimModule` publishes the wrist camera extrinsic relative to `link7`. However, RoboPlan planning currently consumes only robot state and explicitly registered obstacles. Pointcloud observations are not projected into the manipulation planning world, so a planned trajectory can still collide with simulated clutter.

The v1 goal is a direct demo path, not the full Scene Registry architecture. The direct path should still preserve the future boundary: an accumulated global map is one scene-like payload that can later become a `SceneEntity(payload=PointCloud2)` and be projected by `WorldMonitor` into the planning world.

## Goals / Non-Goals

**Goals:**
- Produce camera-pose odometry for a wrist-mounted RGBD pointcloud from the existing TF tree.
- Remove observed robot/gripper points before they enter accumulated mapping.
- Accumulate filtered RGBD observations with the Rust ray-tracing voxel mapper.
- Feed the mapper's `global_map` into manipulation planning as a RoboPlan octree collision obstacle.
- Launch a final integrated demo containing MuJoCo simulation, manipulation planning, Viser, voxel mapping, and RoboPlan octree planning projection.
- Render the planning pointcloud/voxel-map layer directly in Viser so users can inspect the same map being projected into RoboPlan.
- Replace the octree obstacle from the latest full `global_map` on planning-scene sync or pre-plan use.
- Keep configuration simple: octree resolution is world/backend config and defaults to the voxel mapper resolution when practical.

**Non-Goals:**
- Do not implement the full Scene Registry in v1.
- Do not add memory2-backed scene deltas or registry list/watch behavior in this change.
- Do not add exact robot-mesh self-filtering; v1 uses TF-anchored primitive exclusion regions.
- Do not require workspace cropping before octree conversion.
- Do not change grasp target semantic exclusion policy unless it is needed after the voxel planning path is working.

## Decisions

### Use a generic TF-to-pose odometry source

Add a reusable module that queries `self.tf.get(target_frame, source_frame, ts, tolerance)` and publishes pose-only `Odometry` with `frame_id=target_frame` and `child_frame_id=source_frame`.

Rationale: the Rust ray-tracing mapper already expects an `odometry` input, and the existing xArm demo should provide a composed TF chain:

```text
world -> link7                         from ManipulationModule tf_extra_links
link7 -> wrist_camera_color_optical    from MujocoSimModule base_frame_id="link7"
```

Alternatives considered:
- Publish camera-pose odometry directly from `MujocoSimModule`. This is simpler for simulation but does not generalize to real wrist cameras.
- Add TF lookup into the Rust mapper. This couples mapping to DimOS TF and adds another input mode to the native module.

### Publish TF-derived odometry at a fixed rate in v1

The adapter publishes at a configured fixed rate, with a rate equal to or higher than the camera pointcloud rate.

Rationale: fixed-rate output is enough for the demo and avoids timestamp-synchronized wiring complexity.

Future improvement: trigger lookup from each pointcloud timestamp and emit one matching odometry sample per cloud.

### Add a generic pre-map pointcloud self-filter

Add a reusable `PointCloudSelfFilter` that removes points inside configured primitive exclusion regions anchored to TF frames. It preserves the input pointcloud frame and only uses TF internally to express exclusion regions in the cloud frame.

Rationale: the wrist RGBD camera observes gripper geometry. Filtering before accumulation prevents robot-owned points from becoming persistent environment belief.

Alternatives considered:
- Post-map filtering before RoboPlan conversion. This is easier near planning but leaves self-points in the accumulated map and visualization.
- Full robot-model self-filtering. This is more accurate but too large for v1.

### Use the Rust mapper `global_map` for planning

Feed the filtered pointcloud into `RayTracingVoxelMap`, then use its full accumulated `global_map` output for RoboPlan collision.

Rationale: single RGBD frames are partial and cluttered; the accumulated global map is the intended stable belief for planning. `local_map` remains optional/debug-only.

### Do not require workspace cropping in v1

The initial RoboPlan octree projection consumes the latest filtered `global_map` directly.

Rationale: faraway occupied points are usually ignored by arm collision queries, and an unconditional crop policy adds complexity before there is evidence it is needed.

Future improvement: add optional workspace crop or stale-point filtering if octree build time, false positives, or visualization clutter become problems.

### Represent the map as one replace-on-update octree obstacle

Extend RoboPlan world support so the planning voxel map becomes one obstacle with a stable id such as `planning_voxel_map`. When a new map is synced, rebuild or replace the octree obstacle from the latest point set.

Rationale: the Rust mapper publishes full map snapshots, not voxel deltas, and upstream RoboPlan supports `OcTree`/Coal octree collision geometry.

Alternatives considered:
- Convert clusters to boxes. This is simpler but loses shape fidelity and requires clustering policy.
- Convert to mesh. This is heavier than the native octree path and less aligned with MoveIt/RoboPlan occupancy-map patterns.

### Keep voxel resolution out-of-band for v1

Carry voxel/octree resolution in mapper and RoboPlan world/backend config. The RoboPlan octree resolution defaults to the mapper voxel size when the demo can pass it through, with explicit override available.

Rationale: existing voxel modules publish `PointCloud2`, not a typed voxel-grid message containing resolution. This matches current DimOS practice while leaving a future improvement to define a typed voxel-map payload carrying resolution and occupancy metadata inline.

### Visualize the planning map directly in Viser

The final demo renders the accumulated planning map in Viser, not only in Rerun or Meshcat. The visualization layer consumes the same `global_map` stream that feeds manipulation planning and updates the displayed pointcloud/voxel-map layer when that stream updates.

Rationale: the demo's purpose is to prove planning-scene synchronization. Users need to see the map that drives collision checking and verify that planned arm motion avoids it.

Alternatives considered:
- Visualize only the robot and planned path. This hides the critical planning input and makes failures hard to diagnose.
- Visualize only semantic objects from simulation/perception. This does not prove the voxel/octree collision map is present.

### Provide a single integrated demo launch

The final demo entrypoint launches simulation, manipulation planning, Viser, TF-pose odometry, pointcloud self-filtering, Rust ray-tracing voxel mapping, and RoboPlan octree projection together.

Rationale: the expected user experience is one stack where the user can observe the map in Viser and then plan around it. Partial demos are useful for tests but not sufficient as the final acceptance path.

## Risks / Trade-offs

- TF chain not complete → Add tests or smoke diagnostics that verify `world -> wrist_camera_color_optical_frame` can be queried before mapping starts.
- Fixed-rate odometry can be stale during fast wrist motion → Use a rate higher than pointcloud publish rate; add timestamp-synchronized mode later.
- Primitive self-filter removes real nearby object points → Keep regions conservative and configurable; prefer demo defaults over hardcoded geometry.
- Primitive self-filter misses some robot geometry → Accept as v1 limitation; upgrade to robot-model self-filtering only if necessary.
- Global map can contain stale clutter → Rely on ray-tracing mapper clearing behavior first; add crop/staleness controls later if needed.
- Octree replacement may be expensive for large maps → Use reasonable voxel/octree resolution; optimize or cache only after profiling.
- RoboPlan Python bindings may expose octree APIs differently than upstream docs → Isolate octree creation in RoboPlan-world backend code and test it independently.

## Migration Plan

1. Add reusable TF and pointcloud utility modules without changing existing demo behavior.
2. Add RoboPlan octree obstacle support behind backend/world config.
3. Add a new or derived xArm MuJoCo manipulation/Viser demo blueprint that wires pointcloud self-filtering, TF-pose odometry, Rust voxel mapping, and manipulation planning map input.
4. Add Viser rendering for the planning map layer and include it in the final integrated demo stack.
5. Keep existing agentic grasp demos unchanged while validating the new voxel planning path manually through the manipulation/Viser demo.
6. Later, replace direct `global_map -> ManipulationModule` wiring with `SceneEntity(payload=PointCloud2) -> WorldMonitor planning projection` when Scene Registry lands.

Rollback is straightforward: run the existing demo blueprint without the new self-filter, voxel mapper, TF-pose source, or planning voxel-map input.

## Open Questions

- Exact default primitive self-filter dimensions for xArm gripper/wrist frames.
- Exact default voxel and octree resolution for the demo after empirical performance checks.
- Exact blueprint name for the final integrated manipulation/Viser demo.
