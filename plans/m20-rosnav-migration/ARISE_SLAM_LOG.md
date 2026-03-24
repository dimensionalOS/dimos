# M20 ARISE SLAM Integration Log

**Goal**: Replace FAST_LIO with ARISE SLAM for M20 autonomous navigation. ARISE supports multi-floor mapping needed for CATL pilot.

**Date started**: 2026-03-24
**Branch**: `integration/m20-rosnav-migration`

---

## Context

FAST_LIO / fastlio2 blocked by message type incompatibility (fastlio2 hardcoded to Livox CustomMsg, see ROSNAV_MIGRATION_LOG.md Finding #29). ARISE SLAM's Velodyne mode accepts standard sensor_msgs/PointCloud2 — our bridge already publishes this.

---

## Finding #1: RSAIRY is 192-channel, Rings 0-191 (2026-03-24)

**Verified from live hardware** (not just spec sheets).

Subscribed to `/bridge/LIDAR_POINTS` inside the container and extracted ring values from a real frame:

- **192-channel RSAIRY** (not 96 as initially assumed)
- 164 unique ring values out of 192 (28 rings empty — normal for hemispherical FOV)
- Ring range: 0-191
- Front lidar: rings 0-95, back lidar: rings 96-191 (offset, not overlapping)
- 77K points per frame, 26 bytes/point, 10Hz
- Time field: absolute seconds (float64) — needs conversion to relative

**Source**: Direct hardware measurement via Python subscriber in container + background research confirming 192-channel variant from rs_driver decoder source.

---

## Finding #2: ARISE SLAM N_SCANS Validation Blocks 192 (2026-03-24)

`featureExtraction.cpp:64`:
```cpp
if (config_.N_SCANS != 16 && config_.N_SCANS != 32 && config_.N_SCANS != 64 && config_.N_SCANS != 4)
```

Only allows 4, 16, 32, 64. Array at line 584 is indexed by ring value — ring >= N_SCANS causes OOB crash.

**Why these specific numbers**: Lines 1441-1480 have hardcoded Velodyne vertical angle-to-scanID formulas (VLP-16: 2deg spacing, VLP-32: 1.33deg, VLP-64: split formula). These only run when `provide_point_time: 0` (legacy mode). With `provide_point_time: 1` (our mode), ring values come directly from the driver — no angle math.

**Proper fix**: Add 192 to the validation. Requires nav image rebuild (~8hrs on arm64 emulation).

**Quick workaround**: Remap rings in ros2_pub to fit in 64 bins: `new_ring = ring * 64 / 192`. Groups 3 original rings per bin. No image rebuild — bridge rebuilds in minutes on NOS.

---

## Finding #3: QoS Mismatch — ARISE Uses RELIABLE (2026-03-24)

ARISE subscriber at `featureExtraction.cpp:76`:
```cpp
create_subscription<PointCloud2>(LASER_TOPIC, 2, ...)
```

Default rclcpp QoS(2) = RELIABLE. Our bridge uses `SensorDataQoS()` = BEST_EFFORT. Incompatible — ARISE won't receive data.

**Fix**: Change ros2_pub to RELIABLE QoS. Done in local source, needs bridge rebuild.

---

## Finding #4: Time Field Must Be Relative (2026-03-24)

Our bridge outputs absolute seconds (float64, e.g., 1774388151.300). ARISE's `PointcloudXYZITR.time` is float32. `pcl::fromROSMsg` truncates f64→f32 — at 1.77e9, float32 resolution is ~128 seconds. ALL sub-second timing destroyed. Deskewing breaks.

**Fix**: In ros2_pub, subtract first point's time from all points before publishing. Relative offsets (0.0-0.1s) fit float32 perfectly. Done in local source, needs bridge rebuild.

---

## Finding #5: Dual-Lidar Merge is Safe (2026-03-24)

Confirmed from hardware data + rs_driver source:
- rsdriver merges with `send_separately: false`
- Front lidar: rings 0-95, back lidar: rings 96-191 (OFFSET, not overlapping)
- Points within each ring are from a single physical lidar, in azimuth order
- Feature extraction (PCA line fit on NeighborWidth neighbors) works correctly — spatial adjacency preserved within each ring

With ring remapping to 64 bins:
- Bins 0-31 = front lidar (groups of 3 original rings)
- Bins 32-63 = back lidar (groups of 3 original rings)
- No cross-lidar mixing within bins
- ~1200 points per bin (plenty for feature extraction)

---

## Current Plan

### Phase 1: Quick Test (no image rebuild)
1. [x] Verify ring values from hardware (Finding #1)
2. [ ] Update ros2_pub: remap rings to 0-63, make time relative, change QoS to RELIABLE
3. [ ] Rebuild bridge on NOS (minutes, not hours)
4. [ ] Create ARISE SLAM M20 config (YAML only)
5. [ ] Start nav container with LOCALIZATION_METHOD=arise_slam using existing image
6. [ ] Test: does ARISE produce /state_estimation and /registered_scan?

### Phase 2: Full Support (image rebuild)
7. [ ] Patch ARISE to accept N_SCANS=192 (one-line change)
8. [ ] Rebuild nav image with ARISE patch
9. [ ] Remove ring remapping from bridge (use native 192 rings)
10. [ ] Tune feature extraction parameters for RSAIRY scan pattern

---

## Key Files

| File | Purpose |
|------|---------|
| `dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/ros2_pub.cpp` | Bridge publisher (ring remap + time fix + QoS) |
| `dimos/robot/deeprobotics/m20/docker/arise_slam_m20.yaml` | M20 ARISE SLAM config |
| `docker/navigation/.../arise_slam_mid360/src/FeatureExtraction/featureExtraction.cpp` | ARISE feature extraction (N_SCANS validation) |
| `docker/navigation/.../arise_slam_mid360/config/livox_mid360.yaml` | Base ARISE config |
| `docker/navigation/.../arise_slam_mid360/include/.../point_os.h` | PointcloudXYZITR definition |
