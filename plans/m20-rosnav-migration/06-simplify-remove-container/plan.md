# Plan: Remove ROSNav Docker Container

**Goal**: Simplify M20 nav stack by running everything natively on NOS via nix. No Docker container.

**Status**: Planning

---

## Current Architecture (Docker-based)

```
drdds_recv (host) → POSIX SHM → ros2_pub (container) → ROS2 topics
  → ARISE SLAM (container, C++)
  → TerrainAnalysis + LocalPlanner + PathFollower (container, C++)
  → ROSNav bridge (container, ROS2 → LCM)
  → PGO + CmdVelMux + ClickToGoal (host, Python)
  → M20Connection → /NAV_CMD
```

## Target Architecture (all native)

```
drdds_recv (host) → POSIX SHM → DrddsLidarBridge (host, Python, LCM)
  → AriseSLAM (host, NativeModule via nix)
  → smart_nav() (host, NativeModules via nix)
      TerrainAnalysis, LocalPlanner, PathFollower, SimplePlanner,
      PGO, CmdVelMux, ClickToGoal
  → M20Connection → nav_cmd_pub → /NAV_CMD
```

## What changes

### Delete
- `dimos/robot/deeprobotics/m20/rosnav_docker.py` — M20ROSNav Docker module
- `dimos/robot/deeprobotics/m20/docker/Dockerfile.nav` — nav container image
- `dimos/robot/deeprobotics/m20/docker/m20_entrypoint.sh` — container entrypoint
- `dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/ros2_pub.cpp` — no longer needed
- `dimos/navigation/rosnav/` — ROSNav module (if no other robot uses it)
- Docker container management in `deploy.sh` (push/pull/start/stop/dev/shell)
- Bridge commands in `deploy.sh` that start ros2_pub inside container

### Keep
- `drdds_recv` — still needed to read drdds SHM from rsdriver/yesense on host
- `nav_cmd_pub` (pybind11) — still needed to publish /NAV_CMD via drdds
- `deploy.sh` bridge-start — still needed for drdds_recv startup ordering
- `M20Connection` — camera, velocity control, odometry

### Create
- **`DrddsLidarBridge` module** — Python dimos Module that reads POSIX SHM
  (written by drdds_recv) and publishes `Out[PointCloud2]` (lidar) +
  `Out[Imu]` (IMU) via LCM. Replaces ros2_pub but outputs to dimos streams
  instead of ROS2 topics. This is the M20 equivalent of the `Mid360` module
  in the G1 blueprint.
  - Options: pure Python SHM reader, or pybind11 wrapper around existing C++
  - Must handle: ring remapping (192→64), time field f64→f32 relative,
    IMU polling at ~200Hz

### Modify
- **`m20_smartnav.py`** — replace `m20_ros_nav()` + manual Python modules with:
  ```python
  DrddsLidarBridge.blueprint(),  # replaces Mid360 in G1 example
  AriseSLAM.blueprint(
      mount=M20.lidar_mount_offset,
      scan_voxel_size=0.1,
      max_range=50.0,
  ),
  smart_nav(
      use_simple_planner=True,
      vehicle_height=M20.height_clearance,
      terrain_analysis={
          "obstacle_height_threshold": 0.01,
          "ground_height_threshold": 0.01,
      },
      path_follower={
          "two_way_drive": False,
      },
      simple_planner={
          "cell_size": 0.3,
          "obstacle_height_threshold": 0.20,
          "inflation_radius": 0.4,
          "lookahead_distance": 2.0,
          "replan_rate": 5.0,
          "replan_cooldown": 2.0,
      },
  ),
  ```
- **`m20_rosnav.py`** — same treatment or delete (it's the older blueprint)

## Prerequisites

1. **Install nix on NOS** — required for NativeModule C++ compilation
2. **Verify AriseSLAM NativeModule builds on ARM64** — the G1 example says
   "WARNING: This is how AriseSLAM should be used, but it is untested"
3. **M20 AriseSLAM config** — need arise_slam_m20.yaml equivalent as
   NativeModule config (N_SCANS=64, ring remapping, IMU topic, etc.)
4. **M20 lidar mount transform** — equivalent of G1.internal_odom_offsets
5. **DrddsLidarBridge implementation** — the new Python SHM reader module

## Resolved questions

- **AriseSLAM config**: accepts same YAML params as the ROS2 node (confirmed by Jeff)
- **AriseSLAM on ARM64**: building via nix should work (confirmed by Jeff)
- **Ring remapping**: DrddsLidarBridge handles 192→64 remapping (not AriseSLAM)
- **DrddsLidarBridge impl**: pybind11 wrapper — hot path (10Hz, ~100K pts/frame),
  Python struct unpacking would be too slow. Same pattern as nav_cmd_pub.
  Essentially ros2_pub.cpp gutted of ROS2 code, exposing `read_lidar()` /
  `read_imu()` that return numpy arrays for Python to wrap into dimos messages.
- **SimplePlanner**: confirmed by Jeff as the planner for M20 (not FarPlanner)
- **M20 lidar mount offset**: needs to be created (no existing M20 config class)

## Critical issues (from Codex review)

1. **PointCloud2 `ring` + `time` fields** — dimos PointCloud2 serialization only
   preserves XYZ + intensity. ARISE SLAM requires `ring` (for scan line grouping)
   and `time` (for undistortion). The pybind11 bridge MUST output a PointCloud2
   with all 6 fields (x, y, z, intensity, ring, time). If ring/time are lost
   during LCM transport, ARISE will silently produce garbage or crash.
   - Current ros2_pub.cpp explicitly writes these fields (lines 113-143)
   - dimos PointCloud2.py may need extension to preserve arbitrary fields

2. **Dual odometry conflict** — M20Connection publishes dead-reckoning `odometry`
   (from UDP), and AriseSLAM also outputs `odometry`. Both would collide on the
   same LCM transport key `(odometry, Odometry)`.
   - Fix: disable M20Connection's dead-reckoning when AriseSLAM is active
     (e.g. `enable_dead_reckoning=False` flag), or remap one of them
   - The `lidar` → `raw_points` remap is also required (same as G1 example)

3. **Navigation mode gating** — M20Connection only switches to `NAVIGATION` usage
   mode (required for `/NAV_CMD` to move the robot) when `enable_ros=True`.
   With `enable_ros=False`, it stays in `REGULAR` mode (UDP teleop only).
   - Fix: add `enable_nav_mode=True` flag independent of rclpy availability,
     or detect that nav_cmd_pub is available and switch automatically

## Open questions

- Exact SHM struct layout from drdds_recv — need to match the reader
- Does dimos PointCloud2 LCM encoding support `ring` + `time` fields, or
  do we need to extend it?
- n_workers: currently 2 (constrained by NOS RAM). With nix + NativeModules,
  will memory usage increase?

## Migration path

Phase 0 — Spike (prove the data path works before rewriting everything):
1. Implement DrddsLidarBridge pybind11 module with ring/time preservation
2. Verify AriseSLAM produces valid output from bridge data (standalone test)
3. Confirm PointCloud2 LCM transport preserves all 6 fields end-to-end

Phase 1 — Infrastructure:
4. Install nix on NOS
5. Build AriseSLAM + SmartNav NativeModules on NOS (verify ARM64)
6. Fix M20Connection nav mode gating (issue no.3 above)
7. Fix odometry conflict (issue no.2 above)
8. Create M20 config class with lidar mount offset + height clearance

Phase 2 — Integration:
9. Create new m20_smartnav blueprint using smart_nav() + AriseSLAM
10. Add lidar → raw_points remap
11. Test alongside existing Docker setup (keep old blueprint as fallback)
12. Verify SLAM output matches Docker-based ARISE
13. Verify navigation (WASD teleop + click-to-goal)

Phase 3 — Cleanup:
14. Delete Docker artifacts once native path is validated
15. Update deploy.sh (remove container commands, keep bridge-start)
16. Update PR description and documentation
