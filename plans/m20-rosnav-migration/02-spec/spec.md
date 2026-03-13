# M20 ROSNav Migration — Design Specification

**Created:** 2026-03-13
**Status:** Validated
**Brainstorming Mode:** With scope questions (3x3 multimodal matrix)

---

## Overview

This project migrates the DeepRobotics M20 quadruped from a monolithic Docker architecture to the ROSNav host+container pattern already proven on the Unitree G1. Currently, ALL of dimos runs inside a single ROS2 Humble Docker container on the M20's NOS compute board. The target architecture splits this: dimos runs natively on the NOS host (via uv/Python 3.10), while the CMU navigation stack (FASTLIO2, FAR planner, base_autonomy) runs in a separate Humble Docker container managed by DockerModule. This architectural migration is the prerequisite for multi-level autonomous navigation.

The end user for THIS project is the Houmanoids engineering team, operating the robot through the dimos viewer (Rerun fork) and the existing command center. The broader business context is a CATL factory pilot involving patrol and inspection across multiple floors connected by stairs and ramps. However, the CATL-facing product layer (Houdini fleet management integration, Chinese language UI, production SLAs) is a separate epic. This project delivers the robotics infrastructure: reliable single-floor autonomy in Phase 1, relocalization and map persistence in Phase 2, and validated multi-level stair/ramp traversal in Phase 3.

The M20's key differentiator is quadruped stair-climbing. No wheeled AMR can traverse stairs. This makes the ROSNav migration strategically important: it unlocks a capability class that competitors cannot match, while aligning the M20's software architecture with the proven dimos pattern used across the rest of the fleet.

---

## Scope Questions & Answers

### Summary
- Questions addressed: 147 (all tiers: P0 through P3)
- Auto-answered from codebase: 95
- Human decisions (brainstorming dialogue): 52
- Key insight: project scope is dimos multi-level nav infrastructure, not CATL-facing product

### Key Decisions Table

| # | Question | Answer | How Decided |
|---|----------|--------|-------------|
| 2 | What does "multi-floor transition" mean at CATL? | Ramps AND staircases -- continuous traversal. No elevators at CATL, only stairs connecting levels. | Human (site knowledge) |
| 4 | Does Phase 1 deliver user-visible value? | Yes. Currently ZERO autonomy on M20 (only manual teleop). Phase 1 delivers autonomous single-floor navigation. dimos IS the autonomy layer. | Human (current state clarification) |
| 5 | Automatic or manual floor transitions? | Continuous traversal (stairs/ramps), not discrete elevator-based switching. No user intervention needed at transition points. | Human (follows from Q2) |
| 8 | Will existing maps carry over? | No. Start fresh with FASTLIO2. No migration of manufacturer lio_perception maps. | Human (clean break) |
| 9 | Who is the primary end-user? | Houmanoids engineering team, via dimos viewer (Rerun fork) + command center. | Human |
| 10 | What job is the robot doing during the pilot? | Patrol + inspection at CATL factory. | Human |
| 12 | What is explicitly NOT being solved? | Fleet management, multi-robot, automated inspection analysis, elevator integration, OTA updates, Chinese language in dimos UI, production SLAs, Houdini integration, command center new features. | Human (scope recalibration) |
| 14 | Mission completion rate threshold? | Goal is zero-intervention autonomous operation. Achieved iteratively through edge-case fixing during development. | Human |
| 16 | Acceptable operator interventions per shift? | Zero (target). This is iterative -- fix edge cases as they surface. | Human |
| 18 | Behavior when factory layout has changed? | Dynamic replanning around obstacles. CMU base_autonomy + FAR planner handle local avoidance. | Human |
| 25 | What is the "factory pilot" in business terms? | Demo / proof of concept. | Human |
| 28 | Safety/compliance requirements? | Standard e-stop. M20 has hardware e-stop (HES button). | Human |
| 53 | Where does the monitoring view live? | Dimos viewer (Rerun fork) sufficient for debugging. No command center additions needed. | Human |
| 64 | Environmental conditions at CATL? | Dust + reflective surfaces. Tuning challenges, not blockers. | Human |
| 69 | arise_slam porting: research or engineering? | Uncertain difficulty. Phased approach validated. FASTLIO2 for Phase 1, arise_slam is Phase 2 with explicit risk budget. | Human |

### Deferred Questions

The following are explicitly deferred to the Houdini integration epic or post-demo work:

| # | Question | Deferred To |
|---|----------|-------------|
| 13 | AMR competitor benchmarking | Houdini epic (commercial positioning) |
| 24 | Multi-floor visual representation | Phase 2+ (UI for floor context) |
| 27 | Uptime SLA | Post-demo (production hardening) |
| 40 | Chinese language support | Houdini epic (CATL-facing product) |
| 52 | Shift handoff flow | Post-demo (operational process) |
| 65 | Promises to pilot stakeholders | Houdini epic (business alignment) |
| 80 | System update mechanism | Post-demo (OTA is out of scope) |
| 86 | Multi-user command conflict | Houdini epic (fleet management) |
| 89 | Training materials | Post-demo (CATL onboarding) |
| 95 | Role-based information density | Houdini epic (CATL supervisors) |
| 103 | Patrol route setup | Post-Phase 1 (requires basic nav first) |
| 128 | Role-based permissions | Houdini epic (multi-user access) |

---

## Design

### Architecture Overview

The ROSNav pattern splits the system into two runtimes: dimos runs natively on the NOS host, while the CMU navigation stack runs in a ROS2 Humble Docker container. Communication between them uses LCM multicast (goals, velocity commands, state) while the container subscribes to DDS topics from the robot's existing sensor pipeline.

```
M20 Hardware
├── AOS (10.21.31.103 / wlan0: 10.21.41.1) ──── ROS2 Foxy
│   ├── rsdriver ── reads 2x RSAIRY lidars via multicast
│   │   ├── Front: 224.10.10.201:6691
│   │   └── Back:  224.10.10.202:6692
│   │   └── Publishes merged /lidar_points (DDS)
│   └── lio_perception ── DISABLED in Phase 1 (no longer needed)
│       └── Was: /ALIGNED_POINTS, /ODOM, /LIO_ODOM
│
├── GOS (10.21.31.104, 5G) ──── ROS2 Foxy
│   ├── rsdriver ── redundant copy, same config
│   └── m20-relay
│
└── NOS (10.21.31.106) ──── Compute Board
    │
    ├── HOST (uv / Python 3.10 / dimos native)
    │   ├── M20Connection (UDP velocity + camera)
    │   ├── VoxelGridMapper (Open3D voxel grid)
    │   ├── CostMapper (height cost → OccupancyGrid)
    │   ├── WebsocketVisModule (command center)
    │   ├── RerunBridge (dimos viewer)
    │   └── WavefrontExplorer (frontier exploration)
    │
    │       LCM multicast ↕ (goals, cmd_vel, odom, pointcloud, state)
    │
    └── DOCKER CONTAINER (ROS2 Humble / DockerModule)
        ├── FASTLIO2 (LiDAR-Inertial SLAM)
        │   └── Subscribes: /lidar_points (DDS from rsdriver)
        │   └── Subscribes: /IMU (DDS from AOS)
        ├── FAR Planner (visibility-graph route planning)
        ├── base_autonomy (local obstacle avoidance)
        └── ROSNav Bridge Module (ROS2 ↔ LCM translation)
            ├── ROS Input:  /registered_scan, /cmd_vel, /goal_reached, /terrain_map_ext, /tf
            ├── ROS Output: /goal_pose, /cancel_goal, /soft_stop, /joy
            ├── LCM Output: odom, pointcloud, cmd_vel, path, goal_reached, global_map
            └── LCM Input:  goal_req
```

### Phase 1: ROSNav Pattern + FASTLIO2

**Goal:** Architectural migration to host+container split, with working FASTLIO2 SLAM. This delivers the first-ever autonomous navigation on the M20.

#### 1.1 NOS Host Setup (`deploy.sh setup`)

Install uv and Python 3.10 on NOS (currently only has Foxy's Python 3.8):

- Install `uv` via official installer
- Create Python 3.10 venv: `uv venv --python 3.10`
- Install dimos from source into the venv
- Rebuild `drdds-ros2-msgs` for Python 3.10 ABI compatibility (mandatory for `/NAV_CMD` publishing)
- Add to `deploy.sh` as a new `setup` subcommand (one-time operation)

NOS constraints to respect:
- RK3588 SoC, 4 cores, ~2GB usable RAM + 4GB swap (hardened with `/var/opt/robot/data/swapfile`)
- sshd OOM protection already applied (`OOMScoreAdjust=-1000`)
- Docker storage at `/var/opt/robot/data/docker` (46GB free)
- SSH via ProxyJump: `ssh -o ProxyJump=user@10.21.41.1 user@10.21.31.106`
- NOS sudo password: `'` (single quote) -- extreme quoting difficulty through SSH layers
- NOS added to docker group -- no sudo needed for docker commands

#### 1.2 M20ROSNav DockerModule Configuration

New file: `dimos/robot/deeprobotics/m20/rosnav_docker.py`

Based on the existing `dimos/navigation/rosnav_docker.py` (ROSNavConfig), with M20-specific overrides:

```python
@dataclass
class M20ROSNavConfig(ROSNavConfig):
    docker_image: str = "ghcr.io/aphexcx/m20-nav:latest"
    docker_network_mode: str = "host"          # Required for LCM multicast + DDS
    docker_shm_size: str = "1g"                # NOS RAM-constrained (was 4g for G1)
    docker_memory: str = "1.5g"                # Fail predictably before OOM-killing host
    docker_restart_policy: str = "on-failure:3" # Auto-recover from crashes

    # M20-specific SLAM config
    localization_method: str = "fastlio"        # Phase 1: fastlio, Phase 2: arise_slam
    lidar_topic: str = "/lidar_points"          # rsdriver merged dual-lidar DDS topic
    imu_topic: str = "/IMU"                     # 200Hz IMU from AOS
    nav_cmd_topic: str = "/NAV_CMD"             # M20 velocity command topic (drdds NavCmd)

    # Robot physical parameters
    robot_width: float = 0.45                   # 430mm + 10mm margin per side
    lidar_height: float = 0.47                  # 47cm from ground in agile stance
```

Key adaptations from the G1 reference:
- Reduced `docker_shm_size` from 4g to 1g (NOS has only 2GB usable RAM)
- Added `docker_memory` limit to prevent OOM-killing the host
- M20's lidar input is `/lidar_points` (merged dual RSAIRY), not a single Livox topic
- Velocity output is `/NAV_CMD` (drdds NavCmd format), not standard `/cmd_vel`
- ROS_DOMAIN_ID=42 to isolate from robot's Foxy stack on domain 0

#### 1.3 ROSNav Bridge Adaptations for M20

The existing `dimos/navigation/rosnav.py` (864 lines) handles the ROS2-to-LCM bridge. M20-specific adaptations needed:

**Velocity command translation:** The ROSNav bridge outputs standard `Twist` (linear.x, angular.z). M20 requires `drdds NavCmd` format with absolute m/s values on `/NAV_CMD`. The bridge must:
1. Receive `cmd_vel: Out[Twist]` from the CMU nav stack
2. Convert to NavCmd format (forward velocity, lateral velocity, yaw rate)
3. Publish to `/NAV_CMD` via DDS (requires drdds-ros2-msgs on PYTHONPATH)

**Lidar input:** ROSNav subscribes to `/registered_scan` from FASTLIO2 (already registered/deskewed). No M20-specific changes needed here -- FASTLIO2 handles the RSAIRY scan pattern internally.

**Odom output:** FASTLIO2 produces fused LiDAR-Inertial odometry. This replaces the current lio_perception `/ODOM` topic. The ROSNav bridge publishes this as `odom: Out[PoseStamped]` to the host via LCM.

#### 1.4 M20Connection Changes

Current `M20Connection` (`dimos/robot/deeprobotics/m20/connection.py`) has dual modes:
- Navigation Mode (ROS): uses rclpy, `/NAV_CMD` DDS for velocity, `/ODOM`/`/IMU`/`/ALIGNED_POINTS` from ROS
- Regular Mode (fallback): no rclpy, UDP velocity, dead-reckoning odometry

**Changes for ROSNav pattern:**
- Remove the ROS path entirely (no more rclpy in M20Connection)
- Keep UDP protocol for direct robot communication (motion commands, status)
- Keep camera stream (RTSP/HTTP from M20 cameras)
- Velocity commands now flow: ROSNav bridge (LCM) -> M20Connection (UDP or DDS /NAV_CMD)
- Sensor data no longer comes through M20Connection -- FASTLIO2 subscribes directly to DDS topics

The host-side M20Connection becomes a thin robot interface:
- `cmd_vel: In[Twist]` -- receives velocity from ROSNav, translates to M20 protocol
- `camera: Out[Image]` -- publishes camera frames
- Robot status monitoring (battery, gait state, error codes)

#### 1.5 New Blueprint: `m20_rosnav.py`

New file: `dimos/robot/deeprobotics/m20/blueprints/m20_rosnav.py`

Following the G1 pattern (`unitree_g1_basic.py`):

```python
# Host-side modules (no nav, no ROS)
m20_primitive = autoconnect(
    m20_connection(mode="udp"),           # UDP velocity + camera only
    voxel_grid_mapper(voxel_size=0.05),   # Open3D voxel mapping
    cost_mapper(config=M20HeightCostConfig),
    websocket_vis_module(),
    rerun_bridge(memory_limit="512MB"),
)

# Full ROSNav composition
m20_rosnav = autoconnect(
    m20_primitive,
    ros_nav(mode="m20", config=M20ROSNavConfig),  # DockerModule
)
```

Blueprint composition via `autoconnect()`:
- Scans module `In[T]`/`Out[T]` ports for matching stream names
- Picks LCM transport for cross-process communication
- ROSNav DockerModule is treated as a regular Module by the blueprint system

#### 1.6 Lidar Pipeline: rsdriver -> /lidar_points -> FASTLIO2

The lidar data flow for Phase 1:

```
RSAIRY Front (224.10.10.201:6691) ─┐
                                    ├─ rsdriver (AOS) ─── /lidar_points (DDS) ─── FASTLIO2 (container)
RSAIRY Back  (224.10.10.202:6692) ─┘                                                   │
                                                                                        ├─ /registered_scan (registered pointcloud)
                                                                                        ├─ /odom (fused LIO odometry)
                                                                                        └─ /tf (coordinate transforms)
```

Key points:
- rsdriver already merges both lidars (`send_separately: false`). No separate fusion step needed.
- FASTLIO2 has native RoboSense support via `robosense_fast_lio` fork
- FASTLIO2 subscribes directly to `/lidar_points` DDS + `/IMU` DDS inside the container
- The container uses `--network host` so DDS discovery finds rsdriver's topics
- lio_perception on AOS is disabled (no longer needed; FASTLIO2 replaces it)

#### 1.7 deploy.sh Changes

The existing `deploy.sh` (600 lines) manages the M20 Docker lifecycle. Changes needed:

**New subcommands:**
- `setup` -- one-time NOS host setup (uv, Python 3.10, dimos install, drdds rebuild)
- `dev` -- modified to sync dimos source to NOS host (not just container)

**Modified subcommands:**
- `start` -- launches host dimos first, then DockerModule manages the nav container
- `stop` -- stops dimos host (which stops DockerModule, which stops container)
- `status` -- shows both host dimos health and container health
- `logs` -- aggregates host + container logs

**Preserved functionality:**
- SSH ControlMaster for connection reuse
- NAT rules for network routing
- Service conflict management (auto-disables height_map_nav, planner, dimos-mac-bridge)
- Lidar health recovery logic

#### 1.8 Disabling lio_perception

lio_perception on AOS is no longer needed once FASTLIO2 runs in the container. To disable:
- Stop the lio_perception service on AOS (via deploy.sh or manual SSH)
- This frees CPU on AOS for rsdriver
- rsdriver continues publishing `/lidar_points` DDS -- FASTLIO2 consumes this directly
- `/ODOM`, `/ALIGNED_POINTS`, `/LIO_ODOM` topics will no longer be published (expected)

### Phase 2: arise_slam Port to RoboSense

**Goal:** Enable loop closure, relocalization, and map persistence. This is the prerequisite for multi-level navigation.

**Risk acknowledgment:** arise_slam_mid360 is tuned for Livox Mid-360 (non-repetitive scanning pattern). Porting to RoboSense RSAIRY (mechanical spinning lidar, fundamentally different point distribution) may be a research problem rather than a straightforward engineering task. The three-model question synthesis flagged this as a material risk (Opus called it potentially research-level; GPT and Gemini treated it as engineering). The phased approach mitigates this: Phase 1 delivers value with FASTLIO2 regardless of Phase 2 outcome.

#### 2.1 arise_slam Porting Work

- Port `arise_slam_mid360` feature extraction to work with RSAIRY scan pattern
- RSAIRY produces dense, uniformly-distributed point clouds (mechanical spinning)
- Livox Mid-360 produces sparse, non-repetitive point clouds (solid-state)
- Feature extraction (edge/planar points) tuning is the primary porting effort
- The pose graph and loop closure logic should transfer without modification

#### 2.2 Capabilities Unlocked

- **Map persistence:** Save SLAM maps to disk, reload on reboot (no more starting from 0,0,0)
- **Relocalization:** Robot can determine its position in a previously-mapped environment
- **Loop closure:** Correct accumulated drift when revisiting known areas
- **Multi-session mapping:** Build maps incrementally across multiple sessions
- **Runtime switching:** `LOCALIZATION_METHOD=arise_slam` vs `fastlio` (same container, switchable)

#### 2.3 Validation Criteria

- Relocalization after robot power cycle in a mapped environment: < 10 seconds
- Loop closure triggers correctly when revisiting areas after > 100m traversal
- Map saving/loading round-trips without corruption
- Feature extraction produces stable features from RSAIRY scans (quantified by match rate)

### Phase 3: Multi-Level Validation

**Goal:** Prove end-to-end multi-level navigation via stairs and ramps in a lab/office test environment.

#### 3.1 Staircase Traversal

The M20's quadruped locomotion can climb stairs. The navigation stack must:

- Detect staircase regions in the costmap (steep gradient, regular step pattern)
- Plan paths that include staircase traversal when the goal is on a different level
- Adjust velocity commands during stair climbing (slower speed, tighter control)
- Maintain SLAM tracking during elevation changes (3D pose graph, not planar)

#### 3.2 Ramp Traversal

Ramps are simpler than stairs (continuous slope, no step discontinuities):

- CostMapper's height_cost function already handles slope gradients (0-100 cost)
- `can_climb=0.25` threshold determines traversable slopes
- FAR planner should route through ramps when they offer the shortest path
- Velocity reduction on slopes (proportional to grade)

#### 3.3 Map Layer Architecture

For multi-level environments:

- Each floor is a separate SLAM map layer (arise_slam pose graph segment)
- Floor transitions (stairs/ramps) are encoded as connections between layers
- The robot's current floor is determined by elevation from the active SLAM pose
- Cross-floor planning: route through the map graph (floor A -> staircase -> floor B)

#### 3.4 Validation Environment

- Lab/office building with stairs connecting levels
- Controlled environment for iterative testing
- Success criteria: autonomous navigation from floor A to floor B via staircase, arriving at the correct goal position, without human intervention

### Key Components

| Component | Location | Responsibility | Phase |
|-----------|----------|----------------|-------|
| M20Connection | `dimos/robot/deeprobotics/m20/connection.py` | UDP velocity, camera, robot status | 1 (modify) |
| M20ROSNavConfig | `dimos/robot/deeprobotics/m20/rosnav_docker.py` | M20-specific DockerModule config | 1 (new) |
| m20_rosnav blueprint | `dimos/robot/deeprobotics/m20/blueprints/m20_rosnav.py` | Host+container composition | 1 (new) |
| ROSNav bridge | `dimos/navigation/rosnav.py` | ROS2-to-LCM translation | 1 (adapt) |
| ROSNavConfig | `dimos/navigation/rosnav_docker.py` | Base DockerModule config | 1 (extend) |
| DockerModule | `dimos/core/docker_runner.py` | Container lifecycle management | 1 (use as-is) |
| FASTLIO2 | Container: CMU nav stack | LiDAR-Inertial SLAM | 1 |
| FAR Planner | Container: CMU nav stack | Visibility-graph route planning | 1 |
| base_autonomy | Container: CMU nav stack | Local obstacle avoidance | 1 |
| VoxelGridMapper | `dimos/mapping/voxels.py` | 3D voxel mapping (host) | 1 (use as-is) |
| CostMapper | `dimos/mapping/costmapper.py` | Height cost -> OccupancyGrid | 1 (use as-is) |
| deploy.sh | `dimos/robot/deeprobotics/m20/docker/deploy.sh` | Deployment lifecycle | 1 (modify) |
| arise_slam | Container: CMU nav stack | Loop closure + relocalization | 2 |
| Multi-level map manager | TBD | Floor layer management | 3 |

### Data Flow

End-to-end data flow from lidars to velocity commands:

```
STEP 1: Sensor Data Acquisition
  RSAIRY Front Lidar (224.10.10.201:6691) ──┐
  RSAIRY Back Lidar  (224.10.10.202:6692) ──┤
                                             └── rsdriver (AOS, pinned cores 4-7)
                                                  └── /lidar_points (DDS, merged point cloud)
  IMU (200Hz) ── /IMU (DDS)

STEP 2: SLAM (inside Docker container)
  /lidar_points + /IMU ── FASTLIO2
                           ├── /registered_scan (deskewed, registered point cloud)
                           ├── Fused odometry (iEKF state estimate)
                           └── /tf (coordinate transforms)

STEP 3: Planning (inside Docker container)
  /registered_scan ── FAR Planner
                       ├── Builds visibility graph
                       └── Produces global route

  Global route + /registered_scan ── base_autonomy
                                      ├── Local obstacle avoidance
                                      ├── Speed adaptation
                                      └── /cmd_vel (TwistStamped)

STEP 4: Bridge (inside Docker container)
  ROSNav Bridge Module:
    ROS /cmd_vel ──── LCM cmd_vel ──── Host
    ROS /registered_scan ── LCM pointcloud ── Host
    ROS /goal_reached ──── LCM goal_reached ── Host
    ROS /terrain_map_ext ── LCM global_map ── Host
    Host LCM goal_req ──── ROS /goal_pose

STEP 5: Host Processing
  LCM pointcloud ── VoxelGridMapper ── CostMapper ── OccupancyGrid
                                                       └── WebsocketVisModule (port 7779)
                                                       └── RerunBridge (dimos viewer)

STEP 6: Velocity Command Execution
  LCM cmd_vel ── M20Connection
                  └── /NAV_CMD (drdds NavCmd, DDS) ── M20 Motion Controller
                      OR
                  └── UDP axis commands ── M20 Motion Controller (fallback teleop)

STEP 7: Goal Setting
  Dimos Viewer / Command Center ── goal_req (LCM)
                                    └── ROSNav Bridge ── /goal_pose (ROS2)
                                                          └── FAR Planner
```

### Error Handling

#### Container Crash

- **Detection:** DockerModule polls container health via `rpc.call_sync()`. If container stops responding, DockerModule detects failure.
- **Recovery:** `docker_restart_policy: "on-failure:3"` auto-restarts up to 3 times. After 3 failures, DockerModule reports permanent failure.
- **Safety:** When container dies, no `/cmd_vel` is published. M20 motion controller has built-in watchdog -- if no velocity commands arrive for > 500ms, the robot stops. The robot will NOT continue driving blindly.
- **Host behavior:** Host dimos continues running. VoxelGridMapper, CostMapper, and visualization keep working with stale data. The host reports "Navigation Unavailable" state.

#### SLAM Divergence

- **Detection:** FASTLIO2 publishes a covariance estimate. When uncertainty exceeds threshold, the registered scan quality degrades visibly (points scatter). Monitor the iEKF innovation ratio.
- **Causes:** Featureless environments (empty warehouse), rapid motion beyond IMU-coupling limits, sensor occlusion (dirt on lidar lens).
- **Recovery:** Stop navigation, switch to teleop, drive to a feature-rich area, SLAM will reconverge. In Phase 2, arise_slam's relocalization can recover from complete loss.
- **User feedback:** dimos viewer shows "Localization Degraded" (high uncertainty) or "Localization Lost" (no valid pose). Distinct from "Navigating" state.

#### Lidar Failure

- **Single lidar failure:** rsdriver continues publishing `/lidar_points` from the surviving lidar. FASTLIO2 receives a half-density cloud. Navigation degrades (reduced field of view) but continues.
- **Both lidars fail:** `/lidar_points` stops publishing. FASTLIO2 loses input. Container detects stale input (topic timeout). Robot stops.
- **Recovery:** Existing entrypoint.sh lidar health recovery logic (checks topic freshness, restarts rsdriver if needed). Deploy.sh has lidar health monitoring.
- **Detection:** Monitor `/lidar_points` publish rate. Expected: 10Hz. Below 5Hz: degraded. Zero: failure.

#### Network Loss (NOS to AOS/GOS)

- **Symptom:** DDS topics from AOS (`/lidar_points`, `/IMU`) stop arriving at NOS.
- **Impact:** FASTLIO2 loses input. Same as both-lidar failure from nav stack perspective.
- **Recovery:** DDS auto-reconnects when network restores. FASTLIO2 will need to reconverge (may lose localization if the outage was long).
- **NOS to operator WiFi loss:** Robot continues autonomous operation. Operator reconnects and sees current state. No data loss -- robot operates independently.

#### Stair Traversal Edge Cases (Phase 3)

- **Mid-stair stop:** Robot can hold position on stairs (quadruped stance is stable). Resume navigation continues from current position.
- **Stair detection failure:** If costmap does not correctly identify stairs, the robot may attempt to traverse at too high a speed. Velocity limits during elevation changes mitigate this.
- **SLAM loss on stairs:** Stairwells are often geometrically repetitive (perceptual aliasing). arise_slam's loop closure helps. FASTLIO2 (Phase 1) may struggle here -- this is a known Phase 1 limitation.
- **Fall detection:** M20 reports gait state via `/MOTION_INFO`. If the robot detects instability (via IMU), the motion controller engages self-protection before the nav stack reacts.

#### OOM on NOS

- **Prevention:** Docker container memory-limited to 1.5GB (`--memory=1.5g`). Container OOM is isolated -- does not kill host dimos or sshd.
- **Host OOM:** sshd has OOM protection (`OOMScoreAdjust=-1000`). 4GB swap provides buffer. If host dimos is OOM-killed, the container loses its coordinator but the robot stops (no velocity commands).
- **Monitoring:** Host reports RAM usage via diagnostics. Warning at >80% usage.

### Integration Points

#### DockerModule Pattern (from `dimos/core/docker_runner.py`)

The M20 ROSNav container follows the exact DockerModule lifecycle:

1. Host calls `DockerModule.start()` with M20ROSNavConfig
2. DockerModule builds `docker run` command with host networking, memory limits, volume mounts
3. Container starts, `StandaloneModuleRunner` loads the ROSNav module from JSON payload
4. Container RPC server listens on LCM; host polls until container reports ready
5. Graceful stop: RPC `stop()`, then `docker stop` (30s grace period)

Volume mounts for M20:
- dimos source (live sync for development): `/path/to/dimos:/dimos`
- FastDDS config: `fastdds.xml` (DDS tuning for NOS network)
- SLAM maps (Phase 2): `/var/opt/robot/data/maps:/maps`

#### G1 Reference

The G1 ROSNav implementation (`dimos/robot/unitree/g1/blueprints/basic/unitree_g1_basic.py`) is the reference:

```python
unitree_g1_basic_sim_ros = autoconnect(
    unitree_g1_primitive_no_cam,
    ros_nav(mode="unity_sim"),
)
```

The M20 follows this pattern exactly. The `ros_nav()` factory function creates a ROSNav DockerModule, which `autoconnect()` wires to the host modules by matching stream port types.

#### Existing dimos Modules (unchanged)

- `VoxelGridMapper` (`dimos/mapping/voxels.py`): Receives pointcloud from ROSNav bridge, builds 3D voxel grid. No changes needed.
- `CostMapper` (`dimos/mapping/costmapper.py`): Receives voxel grid, produces OccupancyGrid with height cost. M20-specific config already exists (`HeightCostConfig(max_height=0.7, resolution=0.05, can_climb=0.25)`).
- `RerunBridge`: Publishes to dimos viewer. No changes needed.
- `WebsocketVisModule`: Serves command center on port 7779. No changes needed.

#### Transport Layer

- **Host-to-container:** LCM multicast (requires `--network host` on container)
- **Container-to-robot DDS:** FastDDS with ROS_DOMAIN_ID=42 (isolated from Foxy domain 0)
- **Host-to-robot:** UDP for direct M20 motion commands, DDS for `/NAV_CMD` via drdds
- **Auto-routing:** `autoconnect()` selects LCMTransport for cross-process streams, based on `lcm_encode` availability on message types

#### drdds Interoperability

M20 uses DeepRobotics' proprietary `drdds` DDS implementation alongside standard ROS2 DDS:
- `/NAV_CMD` uses drdds `NavCmd` message type (requires `drdds-ros2-msgs` on PYTHONPATH)
- Standard ROS2 topics (`/lidar_points`, `/IMU`) use FastDDS
- Both coexist on the same network; drdds messages have their own type definitions
- The drdds package must be rebuilt for Python 3.10 ABI compatibility (host) and is already available in the Humble container

---

## Out of Scope

| Item | Rationale |
|------|-----------|
| Fleet management / multi-robot | Houdini integration epic. This project is single-robot infrastructure. |
| Automated inspection analysis | Capture-only for pilot. Analysis is a separate product feature. |
| Elevator integration | No elevators at CATL. Only stairs and ramps. |
| OTA updates | deploy.sh manual deployment is sufficient for engineering team. |
| Chinese language in dimos UI | Handled in Houdini epic (CATL-facing product layer). |
| Production SLAs | This is a demo/proof-of-concept. Production hardening is post-demo. |
| Load/unload automation | Not part of patrol/inspection mission. |
| Houdini integration | Separate epic. dimos viewer + command center are sufficient for this project. |
| Command center new features | No UI additions needed. Existing visualization is sufficient for engineering team. |
| Multi-user access control | Single operator (engineering team). RBAC deferred to Houdini epic. |
| Rollback mechanism | Clean break from old architecture. deploy.sh can redeploy old image if needed. |

---

## Open Questions

### Phase 1 (resolve before implementation)

1. **rsdriver DDS QoS settings:** What QoS profile does rsdriver use for `/lidar_points`? The container's FASTLIO2 subscriber must match (reliability, durability, history depth). Discoverable from rsdriver config or `ros2 topic info -v`.

2. **FASTLIO2 RoboSense fork validation:** The `robosense_fast_lio` fork claims RSAIRY support. Has it been tested with dual-lidar merged input, or only single-lidar? If only single-lidar, the merged point cloud may need scan-level separation metadata.

3. **IMU topic format:** Does the M20's `/IMU` topic use standard `sensor_msgs/Imu` or a drdds custom type? FASTLIO2 expects standard ROS2 IMU messages.

4. **NOS Python 3.10 availability:** Can uv install Python 3.10 on the RK3588 (aarch64)? If not available via uv's managed Pythons, we may need to build from source or use a prebuilt binary.

5. **Docker container image size:** The G1 nav container image is large (multi-stage build with CMU nav stack). NOS has 46GB free on `/var/opt/robot/data/docker`. What is the expected image size? If > 10GB, initial pull time over the M20's network will be significant.

6. **Concurrent rsdriver instances:** Both AOS and GOS run rsdriver with identical configs, both publishing `/lidar_points` to DDS. Does the container receive duplicate point clouds? Need to verify DDS discovery behavior with two publishers on the same topic, or configure FASTLIO2 to subscribe to only the AOS instance.

### Phase 2 (resolve before starting)

7. **arise_slam RSAIRY feature extraction:** What specific changes are needed to arise_slam_mid360's feature extraction for mechanical spinning lidar? This is the key risk item. Recommendation: allocate a 2-week spike before committing to Phase 2 timeline.

8. **Map format and storage:** Where are arise_slam maps stored? What is the format? How large are typical factory floor maps? NOS has limited storage.

### Phase 3 (resolve before starting)

9. **Stair detection in costmap:** Does CostMapper's height_cost function correctly identify stairs (regular step pattern with steep gradient)? Or does it treat stairs as obstacles (lethal cost)?

10. **M20 stair-climbing gait:** What M20 gait mode is used for stair climbing? Does the nav stack need to command a gait transition, or does the M20 motion controller handle it autonomously based on terrain?

11. **arise_slam elevation tracking:** Does arise_slam's pose graph correctly handle 3D elevation changes, or is it planar (2D + heading)?

---

## Next Steps

### Immediate (Phase 1 kickoff)

1. **Resolve Open Questions 1-6** -- SSH to robot, inspect DDS topics, verify FASTLIO2 fork, test Python 3.10 install on NOS
2. **Create M20ROSNavConfig** -- New dataclass extending ROSNavConfig with M20 parameters
3. **Create m20_rosnav blueprint** -- Compose host modules + DockerModule following G1 pattern
4. **Modify M20Connection** -- Strip ROS path, keep UDP + camera
5. **Build M20 nav container image** -- FASTLIO2 + CMU nav stack, push to ghcr.io
6. **Update deploy.sh** -- Add `setup` subcommand, modify `dev`/`start`/`stop` for host+container
7. **Disable lio_perception** -- Stop service on AOS, verify rsdriver continues alone
8. **End-to-end test** -- Single-floor autonomous navigation in lab environment

### Phase 2 gate

- Phase 1 delivers reliable single-floor autonomy (measured by mission completion without intervention)
- arise_slam RSAIRY spike completed (2-week time-boxed exploration)
- Go/no-go decision based on spike results

### Phase 3 gate

- Phase 2 delivers relocalization after power cycle
- Map persistence and loading validated
- Stair-climbing gait integration tested in isolation (M20 climbs stairs via teleop with SLAM tracking)

---

## Appendix: Source Files

### Scope Questions & Triage
- Questions backlog (147 questions, 4 priority tiers): `plans/m20-rosnav-migration/01-scope/questions.md`
- Question triage (95 auto-answered, 52 branch points): `plans/m20-rosnav-migration/01-scope/question-triage.md`
- Codebase context extraction: `plans/m20-rosnav-migration/01-scope/context.md`

### Migration Plan
- Detailed plan with hardware topology and SLAM research: `~/.claude/plans/goofy-discovering-sunbeam.md`

### Key Codebase Files
- M20Connection: `dimos/robot/deeprobotics/m20/connection.py`
- M20 ROS sensors: `dimos/robot/deeprobotics/m20/ros_sensors.py`
- M20 current Dockerfile: `dimos/robot/deeprobotics/m20/docker/Dockerfile`
- M20 current launch: `dimos/robot/deeprobotics/m20/docker/launch_nos.py`
- M20 deploy script: `dimos/robot/deeprobotics/m20/docker/deploy.sh`
- M20 current entrypoint: `dimos/robot/deeprobotics/m20/docker/entrypoint.sh`
- ROSNav bridge module: `dimos/navigation/rosnav.py`
- ROSNav Docker config: `dimos/navigation/rosnav_docker.py`
- DockerModule lifecycle: `dimos/core/docker_runner.py`
- Docker image builder: `dimos/core/docker_build.py`
- Nav container Dockerfile: `docker/navigation/Dockerfile`
- Nav container entrypoint: `dimos/navigation/entrypoint.sh`
- G1 ROSNav blueprint (reference): `dimos/robot/unitree/g1/blueprints/basic/unitree_g1_basic.py`
- Module base class: `dimos/core/module.py`
- Blueprint autoconnect: `dimos/core/blueprints.py`
- VoxelGridMapper: `dimos/mapping/voxels.py`
- CostMapper: `dimos/mapping/costmapper.py`
- ReplanningAStarPlanner: `dimos/navigation/replanning_a_star/module.py`
- NavigationInterface: `dimos/navigation/base.py`
- Transport layer: `dimos/core/transport.py`

### External References
- M20 official dev guide: `~/gt/houmanoids_www/crew/nell/relay/docs/m20-official-software-development-guide.md`
- rsdriver config (robot): `/opt/robot/share/node_driver/config/config.yaml`
- lio_perception script (robot): `/opt/robot/share/lio_perception/scripts/lio_ddsnode.sh`
- FASTLIO2 RoboSense fork: `github.com/RuanJY/robosense_fast_lio`
- CMU nav stack: `ros-navigation-autonomy-stack` repository (FAR planner, base_autonomy, arise_slam)
