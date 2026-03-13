# M20 ROSNav Migration — Codebase Context

## 1. Module Architecture

**Core (`dimos/core/module.py`):**
- `Module` extends `ModuleBase`, implements RPC + RxPy reactive streams
- Typed stream ports: `In[T]` (subscribe), `Out[T]` (publish)
- `@rpc` decorator exposes methods as Remote Procedure Calls via LCM multicast
- Modules declare specs via class inheritance (e.g., `spec.Nav`, `spec.Pointcloud`)

**Lifecycle:** Instantiation → `.start()` → Subscription setup → `.stop()` (cleanup)

**Blueprint Composition (`dimos/core/blueprints.py`):**
- `autoconnect()` scans module types, finds matching In/Out stream names
- `_get_transport_for()` picks LCM/pLCM based on `lcm_encode` availability
- Topics auto-named: `/{stream_name}` if unique, else random ID

---

## 2. DockerModule Infrastructure

**Classes (`dimos/core/docker_runner.py`, 515 lines):**
- `DockerModule`: Host-side proxy for containers; spawns `StandaloneModuleRunner` inside
- `DockerModuleConfig`: Configuration dataclass for Docker lifecycle, networking, resources
- `StandaloneModuleRunner`: Container-side runner; loads module from JSON payload

**Lifecycle:**
1. Host calls `DockerModule.start()` → builds `docker run` command
2. Sends JSON payload: `{"module_path": "pkg.ClassName", "args": [], "kwargs": {...}}`
3. Entrypoint decodes payload → imports module → instantiates
4. Container RPC server listens on LCM; host polls via `rpc.call_sync()` until ready
5. Graceful stop: RPC call `stop()`, then `docker stop` (30s grace period)

**Config:**
- `docker_network_mode`: "host" (required for LCM multicast + DDS)
- `docker_gpus`, `docker_env`, `docker_volumes`, `docker_devices`
- `docker_restart_policy`: "on-failure:3" (default)

**Image Building (`dimos/core/docker_build.py`, 121 lines):**
- Appends DIMOS_FOOTER to Dockerfiles
- Footer installs dimos from source, creates entrypoint
- BuildKit cache mounts for fast rebuilds

---

## 3. ROSNav Reference Implementation

**ROSNav Class (`dimos/navigation/rosnav.py`, ~410 lines):**
- Extends `Module`, `NavigationInterface`, `spec.Nav`, `spec.GlobalPointcloud`, `spec.Pointcloud`, `spec.LocalPlanner`

**Input Ports (LCM from dimos):**
- `goal_req: In[PoseStamped]` — goal from dimos system

**Output Ports (LCM to dimos):**
- `pointcloud: Out[PointCloud2]` — lidar output
- `global_map: Out[PointCloud2]` — voxel grid output
- `cmd_vel: Out[Twist]` — velocity commands
- `goal_reached: Out[Bool]`
- `path: Out[NavPath]`
- `odom: Out[PoseStamped]`

**ROS Input Ports (ROSTransport from nav container):**
- `ros_goal_reached: In[Bool]` — /goal_reached
- `ros_cmd_vel: In[TwistStamped]` — /cmd_vel
- `ros_registered_scan: In[PointCloud2]` — /registered_scan
- `ros_global_map: In[PointCloud2]` — /terrain_map_ext
- `ros_path: In[Path]`, `ros_tf: In[TFMessage]`

**ROS Output Ports (ROSTransport to nav container):**
- `ros_goal_pose: Out[PoseStamped]` — /goal_pose
- `ros_cancel_goal: Out[Bool]`, `ros_soft_stop: Out[Int8]`, `ros_joy: Out[Joy]`

**Bridging Pattern:**
- ROS topics consumed via `ROSTransport` (bridges from rclpy DDS)
- dimos topics published via `LCMTransport` (LCM multicast)
- RxPY subjects for sampling/throttling
- Goal navigation: blocks in `navigate_to()` waiting for `/goal_reached`

**Docker Config (`dimos/navigation/rosnav_docker.py`):**
- `docker_image: str = "dimos_autonomy_stack:humble"`
- `docker_network_mode: str = "host"`
- `docker_shm_size: str = "4g"`
- Volumes mount: dimos source (live), fastdds.xml, entrypoint, sim data

---

## 4. M20 Current Architecture

### M20Connection (`dimos/robot/deeprobotics/m20/connection.py`)
- Extends `Module`, `spec.Camera`, `spec.Pointcloud`, `spec.Lidar`, `spec.IMU`, `spec.Odometry`
- **Dual operating modes:**
  - Navigation Mode (ROS): rclpy available, `/NAV_CMD` DDS for velocity, `/ODOM`/`/IMU`/`/ALIGNED_POINTS` from ROS
  - Regular Mode (fallback): no rclpy, UDP velocity, dead-reckoning odometry, CycloneDDS lidar
- Streams: `cmd_vel: In[Twist]`, `camera: Out[Image]`, `imu: Out[Imu]`, `odometry: Out[Odometry]`, `pointcloud: Out[PointCloud2]`

### M20ROSSensors (`dimos/robot/deeprobotics/m20/ros_sensors.py`)
- Wraps RawROS for subscriptions to `/ODOM`, `/tf`, `/ALIGNED_POINTS`, `/IMU`, `/MOTION_INFO`
- Vendor messages: `NavCmd`, `MotionStatus`/`MotionInfo` from drdds v1.0.4
- Foxy vs. Humble detection: checks for `meta` field (Humble) vs `header` (Foxy)
- Publishes velocity to `/NAV_CMD` when dimos sends cmd_vel

### Current Docker Setup
- **Dockerfile** (`dimos/robot/deeprobotics/m20/docker/Dockerfile`, 69 lines): Base `ghcr.io/aphexcx/m20-deps:latest` (5.72GB), drdds rebuild for Python 3.10, dimos source install
- **entrypoint.sh** (104 lines): Sources Humble, sets FastDDS, LCM multicast routing, kills Foxy daemon, waits for `/ODOM`+`/IMU`, lidar health check
- **launch_nos.py** (125 lines): Blueprint composition — m20_connection → voxel_mapper → cost_mapper → a_star_planner → frontier_explorer. NOS-tuned config (n_workers=2, voxel_size=0.05, rerun memory_limit=512MB)
- **deploy.sh** (600 lines): `push`/`pull`/`dev`/`start`/`stop`/`logs`/`shell`/`status`. SSH ControlMaster, NAT rules, service conflict management, lidar health recovery

---

## 5. G1 ROSNav Reference

**G1 Blueprint (`dimos/robot/unitree/g1/blueprints/basic/unitree_g1_basic.py`, 31 lines):**
```python
unitree_g1_basic_sim_ros = autoconnect(
    unitree_g1_primitive_no_cam,
    ros_nav(mode="unity_sim"),
)
```
- Composition: `unitree_g1_primitive_no_nav` + `g1_connection()` + `ros_nav()`
- `ros_nav()` is blueprint factory for ROSNav module
- Demonstrates the pattern M20 should follow

---

## 6. Navigation Stack

**ReplanningAStarPlanner (`dimos/navigation/replanning_a_star/module.py`):**
- Input: `odom: In[PoseStamped]`, `global_costmap: In[OccupancyGrid]`, `goal_request: In[PoseStamped]`
- Output: `cmd_vel: Out[Twist]`, `path: Out[Path]`, `navigation_costmap: Out[OccupancyGrid]`, `goal_reached: Out[Bool]`
- Config: max_linear_speed=0.55, max_angular_speed=0.55, control_frequency=10
- Uses `GlobalPlanner` internally with `make_navigation_map()` for path planning

**NavigationInterface (`dimos/navigation/base.py`, 74 lines):**
- Abstract: `set_goal()`, `get_state()`, `is_goal_reached()`, `cancel_goal()`
- States: IDLE, FOLLOWING_PATH, RECOVERY

---

## 7. Transport Layer

**Implementations (`dimos/core/transport.py`):**
- `LCMTransport(topic, type)` — type-safe LCM (uses lcm_encode)
- `pLCMTransport(topic)` — serialization-based LCM
- `ROSTransport(topic, type)` — ROS2 bridge via DimosROS (rclpy)
- `SHMTransport(topic)` — shared memory (fast same-host)
- `JpegLcmTransport` — compressed LCM for images

**Auto-routing:** `autoconnect()` in blueprints scans module stream names, picks transport type based on message serialization support.

---

## 8. Nav Container (`docker/navigation/Dockerfile`)

Multi-stage build: ROS2 Humble + CMU nav stack (FASTLIO2, FAR planner, base autonomy, arise_slam).

**Entrypoints:**
- `dimos/navigation/entrypoint.sh` (493 lines) — container orchestration
- `dimos/navigation/dimos_module_entrypoint.sh` — module-specific entrypoint

---

## 9. Mapping Modules

**VoxelGridMapper (`dimos/mapping/voxels.py`):**
- Input: `lidar: In[PointCloud2]`, Output: `global_map: Out[PointCloud2]`
- Open3D VoxelBlockGrid on CUDA/CPU
- Config: voxel_size=0.05, publish_interval=1.0, height range

**CostMapper (`dimos/mapping/costmapper.py`, 86 lines):**
- Input: `global_map: In[PointCloud2]`, Output: `global_costmap: Out[OccupancyGrid]`
- Algo: height_cost (terrain slope gradient → 0-100 cost)
- M20 config: HeightCostConfig(max_height=0.7, resolution=0.05, ignore_noise=0.05, can_climb=0.25, smoothing=5.0)

---

## 10. Data Flow Summary

```
M20 Robot Hardware
├─ 2x RSAIRY lidars → multicast → rsdriver (AOS+GOS) → /lidar_points DDS
├─ lio_perception (AOS) → /ALIGNED_POINTS, /ODOM, /IMU
└─ NOS receives via ROS2 DDS

Current NOS Pipeline (all in Humble Docker):
  M20Connection (ROS mode) → /ALIGNED_POINTS → VoxelGridMapper → CostMapper → A*Planner → cmd_vel → /NAV_CMD

Target ROSNav Pipeline (host + Humble container):
  Host: M20Connection (UDP+cam) → LCM → VoxelGridMapper → CostMapper → WebVis
  Container: FASTLIO2 (SLAM) → FAR planner → base autonomy → ROSNav bridge → LCM ↔ host
```

---

## 11. M20-Specific Constraints

- **NOS hardware**: RK3588 SoC, 4 cores, ~2GB usable RAM, 115.3GB eMMC
- **SSH**: Sudo password is `'` (single quote), ProxyJump through AOS WiFi
- **Service conflicts**: height_map_nav (AOS), planner (NOS), dimos-mac-bridge (NOS), ros2 Foxy daemon
- **Lidar boot-order**: rsdriver starts before network → empty /ALIGNED_POINTS, entrypoint handles recovery
- **NOS needs**: uv + Python 3.10 (currently only Foxy's Python 3.8) for host-side dimos
- **drdds rebuild**: Mandatory for Python 3.10 ABI compatibility
