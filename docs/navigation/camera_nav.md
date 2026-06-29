# Camera Depth Navigation

A pipeline that turns any supported stereo depth camera into a persistent, world-frame occupancy map for the DimOS navigation stack. The depth streamer is the only camera-specific component; the map, carving, and all filters are shared across cameras.

---

## Where it fits in DimOS

```
Stereo camera (ZED Mini / RealSense / ...)
        │
        ▼
  zed_depth_costmap.py
        │  PointCloud2  (world frame)
        ▼
  CostMapper
        │  OccupancyGrid
        ▼
  ReplanningAStarPlanner
```

This is the only part of the navigation stack that changes. Everything from CostMapper onward is unmodified.

The goal is to unify this with the lidar-based navigation stack into a single system. Performance will be benchmarked against lidar ground truth to measure occupancy accuracy and identify where the camera pipeline falls short.

---

## Streams

| Stream | Type | Direction |
|--------|------|-----------|
| Depth + confidence frames | Camera SDK (`sl.MEASURE.DEPTH`, `sl.MEASURE.CONFIDENCE`) | In |
| VIO pose (rotation, translation) | Camera SDK (`sl.Pose`) | In |
| `world/map` | `PointCloud2` | Out → CostMapper |

---

## Requirements

The system shall:

1. Accept a depth stream from any stereo camera and publish a `PointCloud2` in the world frame.
2. Place each depth frame into the world frame using VIO pose from the camera's own tracking.
3. Accumulate voxel evidence across frames and export only voxels confirmed in ≥ 2 independent observations.
4. Confirm free space by ray casting: decrement evidence for any voxel on the clear path between the camera and each observed surface.
5. Exclude the floor without relying on absolute world-Z, which drifts with VIO translation.

**Design constraint:** adding a new stereo camera requires changes only to the depth streamer. [`VoxelAccumulator`](/dimos/navigation/camera_nav/zed_depth_costmap.py), `carve()`, `_filter_isolated`, and the floor exclusion filter are camera-agnostic.

---

## Pipeline

```
Stereo camera
     │
     ▼  SDK pre-filter                     ~ Iterating
     │  confidence ≥ 65, remove saturated areas, fixed exposure
     │
     ▼  Backprojection                     ✓ Implemented
     │  (u, v, depth) → camera_link → world frame via VIO pose
     │
     ▼  Spatial isolation                  ~ Iterating
     │  ≥ 3 hits per 5 cm voxel — rejects flying pixels and lighting artifacts
     │
     ▼  Height band                        ✓ Implemented
     │  h_rel ∈ [−1.4, +0.5] m camera-relative
     │
     ▼  Floor exclusion                    ~ Iterating
     │  discard rays with Z-component < −0.30 (≈17° below horizontal)
     │
     ▼  Free-space carving                 ~ Iterating
     │  decrement evidence along each ray to the surface
     │
     ▼  VoxelAccumulator                   ✓ Implemented
     │  8 cm voxels, export at min_obs = 2
     │
     ▼  PointCloud2 → CostMapper           ~ In progress
```

---

## Scenarios

**VIO translation drift — observed: `cam_z = 1.93 m`, actual ≈ 0.8 m**
Even when VIO translation is unreliable, floor exclusion shall still hold. The filter uses the ray's Z-component, not absolute world-Z, so translation error cancels. `stable` count shall not drop and no floor voxels shall appear in `world/map`.

**Bright overhead lighting**
When the robot operates under bright LED ceiling panels, `clean` shall remain close to `sure` and no voxels above `h_rel = +0.5 m` shall appear in `world/map`.

---

## Known limitations

**Lighting.** Bright or inconsistent overhead light introduces saturated pixels and sparse specular reflections that appear as floating obstacles. Current mitigations — fixed exposure, `remove_saturated_areas` (ZED only), and spatial isolation — reduce the problem but have not fully resolved it. The spatial isolation threshold is the primary lever being tuned.

---

## Supported cameras

| Camera | Depth | State |
|--------|-------|-------|
| ZED Mini | Stereo NEURAL (metric) | Active — iterating on floor exclusion, carving, and lighting |
| Intel RealSense | Stereo (metric) | Planned |
| Monocular (Depth Anything v2) | Neural estimation (relative scale) | Future consideration — not accurate enough for navigation |

---

## What's next

| Step | Description |
|------|-------------|
| Reduce noise and sharpen free/occupied space accuracy | Continue iterating on spatial isolation, floor exclusion, and free-space carving |
| Test in the navigation pipeline | Wire `world/map` into CostMapper and validate end-to-end on ZED Mini — confirm the planner receives correct free/occupied classification while the robot moves |
| Port to RealSense | Implement the RealSense depth streamer; calibrate IR disparity confidence threshold; validate the same pipeline stages |
| Run on Flowbase | Integrate the module into the DimOS Flowbase system and run on the robot |
| Benchmark against lidar | Compare occupancy accuracy against lidar ground truth to measure where the camera pipeline falls short and guide further improvements |

---

## What didn't work

| Attempt | Result |
|---------|--------|
| Floor filter on absolute world-Z (`xyz[:, 2] > 0.05`) | VIO drift put `cam_z` at 1.93 m. All geometry fell below the threshold. Map went to zero. |
| `filter_occluded` — ray sampling at 33 % and 67 % depth | Only catches a blocking surface at exactly those fractions — rare. Also falsely rejected valid observations near the camera. Removed. |
| `texture_confidence_threshold = 80` | Smooth surfaces have low texture scores but are real obstacles. Too much valid geometry rejected. Lowered to 65. |
| Carving without same-frame surface exclusion | `obs` oscillated 0 → 1 every frame and never reached `min_obs = 2`. Map stayed permanently empty. |
