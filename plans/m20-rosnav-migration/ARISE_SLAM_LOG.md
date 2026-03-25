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
2. [x] Update ros2_pub: remap rings to 0-63, make time relative (f32), RELIABLE QoS
3. [x] Rebuild bridge on NOS (drdds_recv + ros2_pub)
4. [x] Create ARISE SLAM M20 config (arise_slam_m20.yaml)
5. [x] Start nav container with LOCALIZATION_METHOD=arise_slam
6. [x] Add IMU to bridge (Finding #6) — separate poll thread at 200Hz
7. [x] Fix entrypoint: explicitly pass use_fastlio2:=false for arise_slam
8. **BLOCKED**: laser_mapping_node lifecycle never activates (Finding #7)

### Phase 1b: Fix Lifecycle (CURRENT BLOCKER)
9. [ ] Fix laser_mapping lifecycle activation — options:
   a. Mount patched launch file with longer timer delays (30s instead of 1.5s)
   b. Add retry script in entrypoint that calls `ros2 lifecycle set` in a loop
   c. Patch ARISE nodes to auto-configure on startup (bypass lifecycle)
10. [ ] Verify ARISE produces /state_estimation and /registered_scan

### Phase 2: Full Support (image rebuild)
11. [ ] Patch ARISE to accept N_SCANS=192 (one-line change)
12. [ ] Rebuild nav image with ARISE patch
13. [ ] Remove ring remapping from bridge (use native 192 rings)
14. [ ] Tune feature extraction parameters for RSAIRY scan pattern

---

## Finding #6: IMU Bridge Required — Cross-Version DDS Unreliable (2026-03-24)

Yesense `/IMU` was previously directly readable (Finding #16 in ROSNAV_MIGRATION_LOG). After NOS reboot + startup ordering changes, cross-version DDS discovery (FastDDS 2.14 host ↔ 2.6 container) stopped working.

Adding `initialPeersList` to force discovery breaks ARISE's lifecycle node management.

**Fix**: Re-added IMU to the POSIX SHM bridge:
- drdds_recv: subscribes `/IMU` via DrDDSChannel, writes to IMU SHM (64 slots × 256B)
- ros2_pub: separate IMU poll thread (1ms sleep, ~200Hz), publishes `/bridge/IMU`
- ARISE config: `imu_topic: /bridge/IMU`

---

## Finding #7: ARISE Lifecycle Activation Race Condition (2026-03-24)

**Current blocker.** The ARISE launch file (`arize_slam.launch.py`) uses timed lifecycle transitions:
- `TimerAction(period=0.5)` → configure feature_extraction
- `TimerAction(period=1.5)` → configure laser_mapping
- `TimerAction(period=2.5)` → configure imu_preintegration

These fire too early — DDS discovery on the ARM64 NOS takes longer than 1.5s. `laser_mapping_node`'s lifecycle service isn't discoverable in time, so the configure event fails silently. `feature_extraction` and `imu_preintegration` sometimes succeed (race).

Additionally: the baked-in `/ros2_ws/config/fastdds.xml` in the Docker image had incorrect transport descriptor ordering (Finding #18), causing XML parse errors that also prevented lifecycle activation. Fixed by mounting our `fastdds_m20.xml` over it.

**Root cause**: ROS2 lifecycle transitions via `EmitEvent(ChangeState(...))` use DDS service calls, which require DDS discovery to complete first. On slow ARM64, 1.5s is insufficient.

**Fix**: Mounted patched launch file (`arize_slam_m20.launch.py`) with timers increased to 5/10/15s. Also enabled stderr on laser_mapping_node (was commented out — Codex caught this). With 10s delay, all three nodes configure and activate successfully.

---

## Finding #8: Velodyne Processing Pipeline Never Called (2026-03-24)

**NEW BLOCKER after lifecycle fix.** All three ARISE nodes configure + activate, but no SLAM output (`/state_estimation`, `/registered_scan`). The lidar buffer fills to 50, frames drop, refills — infinite loop.

**Root cause**: `undistortionAndscanregistration()` is **commented out** in the Velodyne handler (`laserCloudHandler`, line 1587):
```cpp
//undistortionAndscanregistration();
```

This function is the entire processing pipeline — IMU/lidar sync, point cloud undistortion, feature extraction, and publishing features to `laser_mapping_node`. Without it, lidar frames are buffered but never processed.

The function IS called in the **Livox handler** (line 1393), gated behind `IMU_INIT==true`. But the Velodyne handler has no equivalent — it just buffers and returns.

**Verified against upstream**: Both `jizhang-cmu/autonomy_stack_diablo_setup` and `autonomy_stack_mecanum_wheel_platform` have the same commented-out call. **ARISE SLAM's Velodyne mode has never worked out of the box in any version of this code.**

The `else` branch at line 1398-1404 (non-LIVOX path in the Livox handler) also has it commented out, with only buffer cleanup running.

**Proposed fix**: Mirror the Livox handler's IMU init + processing logic in the Velodyne handler:
1. Check if IMU buffer has enough data (200 × imuPeriod = 1 second)
2. Run `imu_Init->imuInit(imuBuf)` to calibrate gravity
3. Set `IMU_INIT = true`
4. Call `undistortionAndscanregistration()`
5. Clean lidar buffer after processing

**Research results** (2026-03-24): SuperOdom (author's cleaned-up version) fixes this by calling `undistortionAndFeatureExtraction()` from ALL lidar callbacks with `IMU_INIT || imuBuf.empty()` guard. No other fork (12+ checked) has fixed this. AutonomyStackGO1 uses Livox only. ground_based_autonomy_basic uses LOAM, not ARISE.

**Fix applied**: Mirrored SuperOdom pattern — added IMU init + processing call with `IMU_INIT || imuBuf.empty()` fallback to Velodyne handler.

**Result**: ARISE SLAM producing `/state_estimation` (nav_msgs/Odometry) and `/registered_scan` from M20 RoboSense RSAIRY dual lidars. **WORKING as of 2026-03-24.**

---

### Phase 1b: Fix Lifecycle (RESOLVED)

Patched launch file with 5/10/15s timers. All three nodes now configure + activate.

### Phase 1c: Fix Velodyne Processing (RESOLVED)

9. [x] Research how other forks handle the Velodyne processing trigger
10. [x] Apply the correct fix (SuperOdom pattern: IMU init + processing in lidar callback)
11. [x] Rebuild nav image on Mac with ARISE patch (foxglove disabled on arm64)
12. [x] Transfer to NOS via WiFi, deploy
13. [x] **VERIFIED**: ARISE produces /state_estimation and /registered_scan

### Phase 1d: Connect dimos ROSNav to ARISE (DONE)

14. [x] Update rosnav_module.py: topic subscriptions conditional on localization_method
        - arise_slam → /state_estimation + /registered_scan
        - fastlio → /Odometry + /cloud_registered
15. [x] Update M20ROSNavConfig: localization_method → arise_slam, LOCALIZATION_METHOD env → arise_slam
16. [x] Add volume mounts: ARISE config, patched launch file, M20 local_planner config
17. [x] Create local_planner_m20.yaml: M20 vehicle dimensions (0.85x0.45), conservative speeds (0.3 m/s)

### Finding #9: ros2_pub DDS Discovery Requires spin() (2026-03-24)

ros2_pub's main loop was `while(ok()) sleep(1)` — no executor spinning. Without `rclcpp::spin()`, DDS discovery callbacks never fire, so the node is invisible to other ROS2 nodes. `ros2 node list` doesn't show it, `ros2 topic info` shows publisher count 0.

**Fix**: replaced sleep loop with `rclcpp::spin()` in a thread. Publisher still works via dedicated poll threads.

### Finding #10: docker exec DDS Discovery Broken with --ipc host (2026-03-24)

Processes started via `docker exec` cannot join the container's DDS domain. Host's drdds_recv (FastDDS 2.14) creates SHM port files in `/dev/shm/fastrtps_port*` that the container's FastDDS 2.6 can't read/lock. Any new DDS participant fails with `Failed init_port fastrtps_port*: open_and_lock_file failed`.

**Workaround**: start ros2_pub from the entrypoint (same process context as system launch), NOT via `docker exec`. Entrypoint-started processes CAN discover each other.

### Finding #11: IMU/Lidar Timestamp Desync — ARISE Drops All Frames (2026-03-24)

ARISE feature_extraction reports "All lidar data is more recent than all IMU data" and drops every frame. drdds_recv timestamps show lidar and IMU are properly synced on NOS wall clock (~0.5s phase offset, normal).

**Root cause (found 2026-03-24):** The SuperOdom-pattern fix (Finding #8) called `undistortionAndscanregistration()` from `laserCloudHandler` (the lidar callback). But `synchronize_measurements()` requires `meas_end_time > lidar_end_time`, where `lidar_end_time = lidar_start_time + last_point_relative_time` (~+100ms for 10Hz). When the lidar callback fires, IMU data has NOT yet accumulated past the frame's end time — the latest IMU sample is at approximately the lidar frame's START time, not 100ms past it. So the check `meas_end_time <= lidar_end_time` always fails.

The Livox handler avoids this by triggering processing from the **IMU callback**, gated by `imu_timestamp > lidar_first_time + LIDAR_MESSAGE_TIME + 0.05`. This guarantees IMU extends past the lidar frame before sync is attempted.

**Fix:** Extended the Livox IMU-callback processing path to include `SensorType::VELODYNE` (`if(LIVOX || VELODYNE)`). Removed the processing trigger from `laserCloudHandler`. The lidar handler now only buffers frames; the IMU handler triggers processing when timing is satisfied.

**Additional fix (Codex review):** The Livox processing path had `lidarBuf.clean()` after `undistortionAndscanregistration()`. For LIVOX this is correct (publishTopic doesn't clean). For VELODYNE, `feature_extraction()` (called inside undistortionAndscanregistration) already cleans the processed frame — the explicit clean would drop the NEXT buffered frame, dropping every other scan. Fixed by guarding the explicit clean with `if (LIVOX)`.

**Status**: Fix applied. Requires nav image rebuild to deploy.

### Known Issues

- **Entrypoint `/IMU` check**: waits for `/IMU` via ros2 topic list, but ARISE uses `/bridge/IMU`. See Finding #6.
- **FAST_LIO fallback remap**: entrypoint remaps `/cloud_registered` → `/registered_scan`, conflicts with rosnav_module's `/cloud_registered` subscription for fastlio mode. Not an issue since ARISE is active.
- **ros2_pub not baked into nav image**: must build inside container on first start (`colcon build --packages-select drdds_bridge --cmake-args -DBUILD_ROS2_PUB=ON`). Use `docker commit` to persist.

### Next Steps

18. [ ] Rebuild nav image with Finding #11 fix (IMU-triggered processing for Velodyne)
19. [ ] Deploy to NOS and verify ARISE produces /state_estimation + /registered_scan
20. [ ] Test navigation on robot: send a goal, verify robot plans + drives
21. [ ] Fix foxglove-bridge on arm64 (corrupts /opt/ros/humble/setup.bash)
22. [ ] Tune ARISE feature extraction for RSAIRY scan pattern (if needed)
23. [ ] Phase 2: patch N_SCANS=192 to use native ring values (remove 64-bin remap)

---

## Key Files

| File | Purpose |
|------|---------|
| `dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/ros2_pub.cpp` | Bridge: lidar (ring remap, time f32, RELIABLE) + IMU (separate 200Hz thread) |
| `dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/drdds_recv.cpp` | Bridge: drdds subscriber for lidar + IMU → POSIX SHM |
| `dimos/robot/deeprobotics/m20/docker/arise_slam_m20.yaml` | M20 ARISE config (velodyne, scan_line=64, /bridge/IMU) |
| `dimos/robot/deeprobotics/m20/docker/fastdds_m20.xml` | FastDDS config (32MB SHM, no initialPeersList) |
| `dimos/navigation/rosnav/entrypoint.sh` | Fixed: use_fastlio2:=false for arise_slam |
| `docker/navigation/.../arise_slam_mid360/launch/arize_slam.launch.py` | Lifecycle launch (NEEDS TIMER FIX) |
