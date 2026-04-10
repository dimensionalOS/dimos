# m20-rosnav-migration - Implementation Plan

**Created:** 2026-03-13
**Status:** Reviewed
**Source Spec:** plans/m20-rosnav-migration/02-spec/spec.md

---

## Overview

This plan migrates the DeepRobotics M20 from a monolithic Docker architecture (all of dimos inside a single ROS2 Humble container on NOS) to the ROSNav host+container pattern proven on the G1. After migration, dimos runs natively on the NOS host (uv/Python 3.10), while the CMU navigation stack (FASTLIO2, FAR planner, base_autonomy) runs in a separate Humble Docker container managed by DockerModule. This delivers the first-ever autonomous navigation on the M20.

The implementation follows the G1 ROSNav pattern exactly: a `ROSNavConfig` dataclass defines the container, `autoconnect()` wires host modules to the DockerModule via LCM, and `deploy.sh` manages the deployment lifecycle. The key M20-specific adaptations are: ROS_DOMAIN_ID=0 (to match rsdriver on AOS), memory-constrained Docker settings for NOS (1.5GB limit), and stripped-down volume mounts (no X11/Unity paths).

Phase 1 (this plan) delivers single-floor autonomous navigation. Phases 2 (arise_slam) and 3 (multi-level stair/ramp traversal) are separate plans gated on Phase 1 completion.

---

## Architecture Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Where M20ROSNavConfig lives | `dimos/robot/deeprobotics/m20/rosnav_docker.py` | Per-robot config pattern; G1 doesn't have one because it uses defaults |
| Blueprint location | `dimos/robot/deeprobotics/m20/blueprints/rosnav/m20_rosnav.py` | Follows `blueprints/{variant}/` subdirectory pattern from `m20_minimal` |
| ROSNav import | `from dimos.navigation.rosnav_docker import ros_nav` | DockerModule-backed version; NOT `rosnav.py` (container-side rclpy) |
| `__post_init__` strategy | Call `super().__post_init__()` then replace `docker_volumes` | Base adds X11/Unity/ros_tcp_endpoint mounts that don't exist on NOS |
| ROS_DOMAIN_ID | `"0"` | Must match rsdriver on AOS; base default is `"42"` |
| Memory limits | `--memory=1.5g` via `docker_extra_args` | No `docker_memory` field exists on DockerModuleConfig |
| Stream conflict resolution | Pass `enable_ros=False, enable_lidar=False` to M20Connection | Prevents M20Connection from producing data on `pointcloud`/`lidar`/`odom` ports (skips rclpy + CycloneDDS init). Port declarations remain on the class — use `.remappings()` if autoconnect wiring conflicts arise. |
| Nav container image | Reuse `docker/navigation/Dockerfile` (already arm64 + FASTLIO2) | No new Dockerfile needed; push to `ghcr.io/aphexcx/m20-nav:latest` |
| NOS host launcher | Replace `launch_nos.py` content (same file) | Avoid creating parallel launcher; `deploy.sh` already references this file |
| Velocity translation | M20Connection receives `cmd_vel: In[Twist]` from ROSNav, routes to M20VelocityController | Existing `_on_cmd_vel` handler already does Twist→NavCmd translation |

---

## Shared Abstractions

### M20ROSNavConfig

- **Name:** `M20ROSNavConfig`
- **Location:** `dimos/robot/deeprobotics/m20/rosnav_docker.py`
- **Purpose:** M20-specific DockerModuleConfig extending ROSNavConfig with NOS resource limits, ROS_DOMAIN_ID=0, FASTLIO2 mode, and minimal volume mounts
- **Consumers:** m20_rosnav blueprint (Phase 1), deploy.sh setup (Phase 1), future Phase 2 arise_slam config

This is the only shared abstraction. All other components are either existing (ROSNav, DockerModule, VoxelGridMapper, CostMapper) or task-specific.

---

## Phased Delivery

### Phase 0: Discovery (Pre-Implementation)

**Objective:** Resolve Open Questions 1-6 from the spec before any implementation begins. These are binary blockers — if any answer is "no," the downstream implementation plan changes.

**Prerequisites:** SSH access to M20 NOS (via ProxyJump through AOS wlan0)

#### Tasks

**0.1 Inspect DDS topics and QoS on NOS**

- **What:** SSH to NOS, enumerate DDS topics, capture QoS profiles for `/lidar_points`, `/IMU`, and `/NAV_CMD`.
- **Key details:**
  - `ros2 topic info -v /lidar_points` — confirm message type is `sensor_msgs/PointCloud2`, get QoS (reliability, durability, history depth)
  - `ros2 topic info -v /IMU` — confirm message type is `sensor_msgs/Imu` (NOT a drdds custom type). If custom, ROSNav bridge needs a translator.
  - `ros2 topic echo /IMU --once` — capture a sample message to verify field layout
  - Document QoS profiles — FASTLIO2 subscribers must match or use `BEST_EFFORT`
- **Resolves:** OQ1 (rsdriver DDS QoS), OQ3 (IMU message type)
- **Exit criteria:**
  - [ ] QoS profiles documented for `/lidar_points` and `/IMU`
  - [ ] IMU message type confirmed as `sensor_msgs/Imu` or custom type identified with adaptation plan

**0.2 Review robosense_fast_lio fork for dual-lidar support**

- **What:** Review the `robosense_fast_lio` fork source code to confirm it handles merged dual-lidar point clouds (front+back merged by rsdriver with `send_separately: false`).
- **Key details:**
  - Clone `github.com/RuanJY/robosense_fast_lio` and inspect point cloud preprocessing
  - Confirm it doesn't assume single-origin points (merged cloud has points from 2 origins)
  - Check if any scan deskewing assumes single-lidar rotation pattern
  - If fork only supports single lidar, document adaptation needed
- **Resolves:** OQ2 (FASTLIO2 fork dual-lidar support)
- **Exit criteria:**
  - [ ] Confirmed fork handles merged multi-lidar clouds, OR adaptation plan documented

**0.3 Test Python 3.10 installation on NOS aarch64**

- **What:** SSH to NOS, attempt to install Python 3.10 via uv on the RK3588 aarch64 platform.
- **Key details:**
  - `curl -LsSf https://astral.sh/uv/install.sh | sh`
  - `uv python install 3.10`
  - If uv doesn't have a prebuilt 3.10 for aarch64, try `uv python install 3.10 --build` or fall back to deadsnakes PPA / build from source
  - Test `python3.10 -c "import sys; print(sys.platform, sys.version)"` to confirm
- **Resolves:** OQ4 (Python 3.10 on aarch64)
- **Exit criteria:**
  - [ ] Python 3.10 runs on NOS, OR alternative installation method documented

**0.4 Estimate nav container image size**

- **What:** Build the nav container image for arm64 and measure its size to verify it fits on NOS storage.
- **Key details:**
  - `docker buildx build --platform linux/arm64 -t m20-nav-test .` from `docker/navigation/`
  - `docker images m20-nav-test` — check compressed and uncompressed size
  - Compare against NOS free storage (46GB on `/var/opt/robot/data/docker`)
  - If image >15GB, investigate multi-stage optimization or `--squash`
- **Resolves:** OQ6 (container image size)
- **Exit criteria:**
  - [ ] Image size documented and confirmed to fit NOS storage

#### Phase 0 Exit Criteria
- [ ] All 4 discovery tasks completed with documented answers
- [ ] No "red flag" findings that require spec revision
- [ ] Implementation can proceed with Phase 1

---

### Phase 1: M20ROSNavConfig + Blueprint

**Objective:** Create the config dataclass, blueprint, and launcher so that `m20_rosnav` can be imported and blueprint tests pass.

**Prerequisites:** Phase 0 complete (all discovery questions resolved)

#### Tasks

**1.1 Create M20ROSNavConfig dataclass**

- **What:** New dataclass extending `ROSNavConfig` with M20-specific Docker settings, sensor topics, and robot parameters.
- **Files:**
  - Create: `dimos/robot/deeprobotics/m20/rosnav_docker.py` — M20ROSNavConfig dataclass
- **Key details:**
  - Extend `ROSNavConfig` from `dimos.navigation.rosnav_docker`
  - Override `docker_image` to `"ghcr.io/aphexcx/m20-nav:latest"`
  - Override `docker_shm_size` to `"1g"` (from 8g default)
  - Add `"--memory=1.5g"` and `"--memory-swap=1.5g"` to `docker_extra_args`
  - Set `docker_env` with `ROS_DOMAIN_ID=0`, `LOCALIZATION_METHOD=fastlio`, `MODE=hardware`, `USE_ROUTE_PLANNER=true`, `USE_RVIZ=false`
  - In `__post_init__`: call `super().__post_init__()`, then replace `self.docker_volumes` with minimal NOS list (dimos source, fastdds.xml, entrypoint script only — no X11, no Unity mesh, no ros_tcp_endpoint patch)
  - After replacing volumes, re-set `self.docker_env["ROS_DOMAIN_ID"] = "0"` (super may have modified it)
  - Add M20 physical parameters: `robot_width=0.45`, `lidar_height=0.47`
  - Add sensor topic fields: `lidar_topic="/lidar_points"`, `imu_topic="/IMU"`, `nav_cmd_topic="/NAV_CMD"`
- **Acceptance criteria:**
  - [ ] `M20ROSNavConfig()` instantiates without error
  - [ ] `config.docker_env["ROS_DOMAIN_ID"] == "0"`
  - [ ] `config.docker_extra_args` contains `"--memory=1.5g"`
  - [ ] `config.docker_volumes` does NOT contain X11 or Unity paths
  - [ ] `config.docker_env["LOCALIZATION_METHOD"] == "fastlio"`
- **Dependencies:** None

**1.2 Create m20_rosnav blueprint**

- **What:** Blueprint composing M20Connection (UDP-only) + VoxelGridMapper + CostMapper + visualization + ROSNav DockerModule, following the G1 `unitree_g1_basic_sim_ros` pattern.
- **Files:**
  - Create: `dimos/robot/deeprobotics/m20/blueprints/rosnav/__init__.py` — empty package init
  - Create: `dimos/robot/deeprobotics/m20/blueprints/rosnav/m20_rosnav.py` — blueprint definition
- **Key details:**
  - Import `ros_nav` from `dimos.navigation.rosnav_docker` (NOT `dimos.navigation.rosnav`)
  - Import `m20_connection` from `dimos.robot.deeprobotics.m20.connection`
  - Construct M20Connection with `enable_ros=False, enable_lidar=False, lidar_height=0.47`
  - `enable_ros=False` prevents rclpy init on host (no ROS on host)
  - `enable_lidar=False` skips CycloneDDS lidar initialization (FASTLIO2 handles lidar in the container)
  - **Note:** These flags suppress runtime data production but do NOT remove class-level `Out[T]` port declarations from M20Connection. The ports (`pointcloud`, `lidar`, `odom`) still exist on the class. `autoconnect()` may still attempt to wire them by name+type. Use `.remappings()` if stream name conflicts arise between M20Connection and ROSNav
  - Include `voxel_mapper`, `cost_mapper` with M20 HeightCostConfig, `websocket_vis`, `rerun_bridge`
  - Use `ros_nav(config=M20ROSNavConfig())` as the nav module
  - Set `.global_config(n_workers=2, robot_model="deeprobotics_m20", robot_ip="10.21.33.103", robot_width=0.45, robot_rotation_diameter=0.6)`
  - Handle platform-specific transports (SHM on Mac for color_image, LCM on Linux) following `m20_minimal.py` pattern
  - Export `m20_rosnav` in `__all__`
- **Acceptance criteria:**
  - [ ] `from dimos.robot.deeprobotics.m20.blueprints.rosnav.m20_rosnav import m20_rosnav` succeeds
  - [ ] Blueprint can be instantiated without hardware (import-time only, no rclpy required)
  - [ ] No stream name conflicts between M20Connection and ROSNav ports
  - [ ] Verify M20Connection `cmd_vel: In[Twist]` is wired by `autoconnect` to ROSNav `cmd_vel: Out[Twist]` (velocity translation pipeline)
- **Dependencies:** Task 1.1 (M20ROSNavConfig must exist), Phase 0 (discovery answers may affect config)

**1.3 Register blueprint in all_blueprints.py**

- **What:** Regenerate the auto-generated blueprint registry so `m20_rosnav` is discovered and covered by `test_all_blueprints.py`.
- **Files:**
  - Modify (auto-generated): `dimos/robot/all_blueprints.py` — regenerated by test
- **Key details:**
  - `all_blueprints.py` is auto-generated — its header says "Run `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate"
  - Do NOT manually edit this file. Run: `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate the registry
  - The test scans for blueprints and auto-discovers `m20_rosnav` from the new file
  - Note: No existing M20 blueprints are currently registered (first M20 entry)
- **Acceptance criteria:**
  - [ ] `pytest dimos/robot/test_all_blueprints_generation.py` passes and regenerates the file
  - [ ] `pytest dimos/robot/test_all_blueprints.py -k m20_rosnav` passes (import test)
- **Dependencies:** Task 1.2

**1.4 Replace launch_nos.py with ROSNav host launcher**

- **What:** Replace the current monolithic Docker launcher with a minimal host-side launcher that runs the m20_rosnav blueprint natively on NOS.
- **Files:**
  - Modify: `dimos/robot/deeprobotics/m20/docker/launch_nos.py` — replace contents entirely
- **Key details:**
  - Import `m20_rosnav` blueprint
  - Call `blueprint.build()` to create coordinator
  - Handle SIGINT/SIGTERM for graceful shutdown
  - The DockerModule inside the blueprint handles the nav container lifecycle — no manual `docker run`
  - Keep the file name `launch_nos.py` since `deploy.sh` already references it
- **Acceptance criteria:**
  - [ ] `python -c "from dimos.robot.deeprobotics.m20.docker.launch_nos import *"` succeeds
  - [ ] Script has signal handling for graceful shutdown
- **Dependencies:** Task 1.2

#### Phase 1 Exit Criteria
- [ ] `m20_rosnav` blueprint imports cleanly on Mac (no hardware required)
- [ ] Blueprint registered in `all_blueprints.py` and passes import test
- [ ] `M20ROSNavConfig` produces correct Docker settings (domain 0, 1.5g memory, fastlio mode)
- [ ] `launch_nos.py` updated to use the new blueprint

---

### Phase 2: deploy.sh + NOS Host Setup

**Objective:** Add the `setup` subcommand for one-time NOS host provisioning, and modify existing subcommands to work with the host+container split.

**Prerequisites:** Phase 1 — blueprint and config exist so deploy.sh can reference them

#### Tasks

**2.1 Add `setup` subcommand to deploy.sh**

- **What:** New deploy.sh subcommand that provisions NOS for host-side dimos execution: installs uv, creates Python 3.10 venv, installs dimos, rebuilds drdds bindings.
- **Files:**
  - Modify: `dimos/robot/deeprobotics/m20/docker/deploy.sh` — add `setup)` case
- **Key details:**
  - SSH to NOS via existing ControlMaster pattern
  - Install uv: `curl -LsSf https://astral.sh/uv/install.sh | sh`
  - Create venv: `uv venv --python 3.10 /opt/dimos/venv`
  - Activate venv and install dimos: `uv pip install -e /opt/dimos/src`
  - Rebuild drdds-ros2-msgs for Python 3.10 ABI: run `build_drdds_bindings.sh` in the venv
  - **drdds PYTHONPATH**: Install drdds into the venv's site-packages (preferred) or add drdds path to the venv's activation script. This is critical — M20Connection publishes `/NAV_CMD` via drdds, so the host dimos process MUST be able to `import drdds`.
  - This is a one-time operation — idempotent (can be re-run safely)
  - NOS sudo password handling: use existing `SUDO_PASS="${SUDO_PASS:-"'"}"` pattern
  - NOS is aarch64 (RK3588) — verify uv can install Python 3.10 for this arch
- **Acceptance criteria:**
  - [ ] `deploy.sh setup` completes on NOS without error
  - [ ] Python 3.10 is available at `/opt/dimos/venv/bin/python3`
  - [ ] `pip list` in the venv shows dimos installed
  - [ ] `python -c "import drdds"` succeeds in the venv (PYTHONPATH correctly configured)
- **Dependencies:** None (can run in parallel with Phase 1 tasks, but placed here because it's deployment-focused)

**2.2 Add `ensure_lio_disabled` function**

- **What:** Replace `ensure_lio_enabled()` with `ensure_lio_disabled()` that stops lio_perception on AOS and verifies rsdriver is still publishing `/lidar_points`.
- **Files:**
  - Modify: `dimos/robot/deeprobotics/m20/docker/deploy.sh` — replace function
- **Key details:**
  - SSH to AOS (10.21.31.103) via NOS ProxyJump
  - Stop lio_perception: `systemctl stop lio_perception` or `kill` the lio_ddsnode process
  - Verify rsdriver still publishes `/lidar_points` (independent of lio_perception)
  - This frees CPU on AOS (fallback for SLAM offload per spec)
  - Keep `ensure_lio_enabled()` as `ensure_lio_enabled_legacy()` for rollback
- **Acceptance criteria:**
  - [ ] After running, lio_perception is not running on AOS
  - [ ] `/lidar_points` DDS topic still has publishers (rsdriver continues)
  - [ ] Function is idempotent (safe to call multiple times)
- **Dependencies:** None

**2.3 Adapt entrypoint.sh for host-side use**

- **What:** Modify the M20 Docker entrypoint script for the new architecture where dimos runs on the host and only the nav stack runs in the container.
- **Files:**
  - Modify: `dimos/robot/deeprobotics/m20/docker/entrypoint.sh` — adapt for ROSNav mode
- **Key details:**
  - Currently waits for `/ODOM` and checks `/ALIGNED_POINTS` (both from lio_perception) — wrong for ROSNav mode
  - Keep `/IMU` wait (still needed — IMU comes from robot hardware, not lio_perception)
  - Replace `/ALIGNED_POINTS` check with `/lidar_points` check (from rsdriver)
  - Remove lio_perception restart logic (no longer relevant — FASTLIO2 is in the nav container)
  - Keep rsdriver health verification
  - This script is used during `deploy.sh start` on NOS
- **Acceptance criteria:**
  - [ ] entrypoint.sh waits for `/IMU` and `/lidar_points` (not `/ODOM` or `/ALIGNED_POINTS`)
  - [ ] No references to lio_perception remain
  - [ ] Script exits cleanly if required topics are not available within timeout
- **Dependencies:** None

**2.4 Modify `start` subcommand**

- **What:** In ROSNav mode, `start` launches dimos natively on the NOS host instead of running a monolithic Docker container. The DockerModule inside dimos manages the nav container.
- **Files:**
  - Modify: `dimos/robot/deeprobotics/m20/docker/deploy.sh` — modify `start)` case
- **Key details:**
  - Activate the venv: `source /opt/dimos/venv/bin/activate`
  - Run `launch_nos.py` on NOS host (not inside Docker)
  - The blueprint's DockerModule handles pulling and starting the nav container
  - Call `ensure_lio_disabled` before starting
  - **Verify drdds is importable**: `python -c "import drdds"` before launching (fail fast if PYTHONPATH misconfigured)
  - Use `nohup` or `systemd` unit for daemon mode
  - **Rollback**: No `--legacy` flag — use git to check out the pre-migration commit instead (less maintenance burden than dual code paths)
- **Acceptance criteria:**
  - [ ] `deploy.sh start` launches dimos on the NOS host
  - [ ] DockerModule automatically starts the nav container
  - [ ] `python -c "import drdds"` check passes before launch
- **Dependencies:** Task 2.1 (setup must have been run), Task 2.3 (entrypoint.sh adapted), Phase 1 (blueprint must exist)

**2.5 Modify `stop`, `status`, `dev` subcommands**

- **What:** Update remaining subcommands for host+container split.
- **Files:**
  - Modify: `dimos/robot/deeprobotics/m20/docker/deploy.sh` — modify `stop)`, `status)`, `dev)` cases
- **Key details:**
  - `stop`: Kill host dimos process (which stops DockerModule, which stops container).
  - `status`: Show host dimos process status + nav container status + rsdriver status. Replace lio_perception checks with rsdriver/FASTLIO2 checks.
  - `dev`: rsync dimos source to NOS host path (not just container volume). The DockerModule volume mount picks up changes from the host path.
  - `logs`: Aggregate host dimos logs + container logs (docker logs)
- **Acceptance criteria:**
  - [ ] `deploy.sh stop` cleanly stops both host dimos and nav container
  - [ ] `deploy.sh status` shows health of host, container, and rsdriver
  - [ ] `deploy.sh dev` syncs source changes to NOS
- **Dependencies:** Task 2.4

#### Phase 2 Exit Criteria
- [ ] `deploy.sh setup` provisions NOS with Python 3.10 + dimos + drdds (with drdds importable)
- [ ] entrypoint.sh adapted for host-side use (waits for `/IMU` + `/lidar_points`)
- [ ] `deploy.sh start` launches host dimos + nav container
- [ ] `deploy.sh stop` cleanly shuts down both
- [ ] `deploy.sh status` shows correct health for all components
- [ ] lio_perception disabled on AOS, rsdriver continues

---

### Phase 3: Nav Container Image Build + Push

**Objective:** Build the M20-specific nav container image from the existing Dockerfile (already arm64 + FASTLIO2 capable) and push to ghcr.io.

**Prerequisites:** Phase 1 — M20ROSNavConfig references the image tag

#### Tasks

**3.1 Build and push M20 nav container image**

- **What:** Build `docker/navigation/Dockerfile` for arm64, tag as `ghcr.io/aphexcx/m20-nav:latest`, and push.
- **Files:**
  - No file changes — uses existing `docker/navigation/Dockerfile`
  - Optionally modify: `docker/navigation/build.sh` — add M20-specific build target
- **Key details:**
  - The existing Dockerfile is multi-arch (amd64/arm64) and already includes FASTLIO2 + arise_slam + FAR planner + base_autonomy
  - Build for arm64: `docker buildx build --platform linux/arm64 -t ghcr.io/aphexcx/m20-nav:latest --push .`
  - Verify image size — if >10GB, initial pull on NOS will be slow over 5G but is one-time
  - The `LOCALIZATION_METHOD=fastlio` env var in M20ROSNavConfig controls which SLAM runs at runtime
  - arise_slam is included but dormant in Phase 1
- **Acceptance criteria:**
  - [ ] `docker pull ghcr.io/aphexcx/m20-nav:latest` succeeds on NOS (arm64)
  - [ ] Container starts with `LOCALIZATION_METHOD=fastlio` and FASTLIO2 initializes
  - [ ] Container starts with `ROS_DOMAIN_ID=0` and discovers `/lidar_points` DDS topic from rsdriver
- **Dependencies:** None (can build while Phase 1/2 are in progress)

**3.2 Create M20-specific fastdds.xml**

- **What:** DDS discovery configuration for the M20 network topology — NOS discovering topics from AOS rsdriver, with peer filtering to exclude GOS.
- **Files:**
  - Create: `dimos/robot/deeprobotics/m20/docker/fastdds_m20.xml` — M20-specific DDS config
- **Key details:**
  - Configure DDS peer list to include only AOS (10.21.31.103), exclude GOS (10.21.31.104)
  - This resolves the dual-rsdriver problem — container only sees AOS's `/lidar_points`
  - Reference existing `docker/navigation/config/fastdds.xml` for format
  - M20ROSNavConfig volume mount should map this file to `/ros2_ws/config/fastdds.xml`
- **Acceptance criteria:**
  - [ ] DDS discovery finds AOS rsdriver topics only
  - [ ] GOS rsdriver topics are not discovered
  - [ ] FASTLIO2 receives `/lidar_points` from AOS at 10Hz
- **Dependencies:** None

#### Phase 3 Exit Criteria
- [ ] Nav container image available at `ghcr.io/aphexcx/m20-nav:latest` for arm64
- [ ] M20-specific fastdds.xml filters to AOS-only DDS peers
- [ ] Container can discover and subscribe to rsdriver topics

---

### Phase 4: End-to-End Integration Test

**Objective:** Validate the complete data flow from lidars to velocity commands on the real M20 robot.

**Prerequisites:** Phases 1-3 complete, robot accessible via SSH

#### Tasks

**4.1 Verify DDS topic connectivity**

- **What:** SSH to NOS, start the nav container manually, verify it discovers `/lidar_points` and `/IMU` from AOS.
- **Files:** No file changes
- **Key details:**
  - SSH to NOS via ProxyJump
  - Pull the nav container: `docker pull ghcr.io/aphexcx/m20-nav:latest`
  - Start container with `--network host` and `ROS_DOMAIN_ID=0`
  - Inside container: `ros2 topic list` should show `/lidar_points` and `/IMU`
  - `ros2 topic hz /lidar_points` should show ~10Hz
  - `ros2 topic hz /IMU` should show ~200Hz
- **Acceptance criteria:**
  - [ ] Container discovers AOS DDS topics on domain 0
  - [ ] `/lidar_points` rate is ~10Hz
  - [ ] `/IMU` rate is ~200Hz
- **Dependencies:** Phase 3 (container image must be available)

**4.2 Verify FASTLIO2 SLAM initialization**

- **What:** Start FASTLIO2 inside the container and verify it produces odometry and registered scans.
- **Files:** No file changes
- **Key details:**
  - The `dimos_module_entrypoint.sh` launches FASTLIO2 when `LOCALIZATION_METHOD=fastlio`
  - Check `/registered_scan` topic is publishing
  - Check FASTLIO2's fused odometry output
  - Verify scan quality: points should be coherent (not scattered)
  - Monitor iEKF convergence in FASTLIO2 logs
- **Acceptance criteria:**
  - [ ] FASTLIO2 produces registered point clouds
  - [ ] Fused odometry is reasonable (robot stationary → near-zero drift)
  - [ ] No SLAM divergence after 60 seconds
- **Dependencies:** Task 4.1

**4.3 Full pipeline test with deploy.sh**

- **What:** Run `deploy.sh setup` then `deploy.sh start` and verify the complete data flow.
- **Files:** No file changes
- **Key details:**
  - Run `deploy.sh setup` (one-time NOS provisioning)
  - Run `deploy.sh start` (launches host dimos + DockerModule starts nav container)
  - Verify host dimos connects to nav container via LCM
  - Send a test goal via dimos viewer or command center
  - Verify FAR planner produces a path
  - Verify `cmd_vel` reaches M20Connection and translates to `/NAV_CMD` — confirm `/NAV_CMD` DDS topic receives velocity commands during autonomous navigation
  - Verify the full velocity pipeline: ROSNav `cmd_vel: Out[Twist]` → autoconnect → M20Connection `cmd_vel: In[Twist]` → `_on_cmd_vel` → M20VelocityController → `/NAV_CMD` DDS
  - Verify robot moves (slowly, in a clear area)
  - Run `deploy.sh status` to check all components healthy
  - Run `deploy.sh stop` to verify clean shutdown
- **Acceptance criteria:**
  - [ ] Robot completes a patrol loop (sequence of waypoints) and returns to start
  - [ ] No human intervention required during patrol
  - [ ] `deploy.sh status` shows all components healthy during operation
  - [ ] `deploy.sh stop` cleanly shuts down everything
  - [ ] NOS memory stays under 80% during operation
- **Dependencies:** Tasks 4.1, 4.2, all of Phase 2

#### Phase 4 Exit Criteria
- [ ] Complete data flow verified: lidar → FASTLIO2 → FAR planner → cmd_vel → M20 motion
- [ ] Autonomous patrol loop completed without human intervention
- [ ] deploy.sh lifecycle works: setup → start → status → stop
- [ ] No OOM, no crashes, no SLAM divergence during 30-minute test

---

## Cross-Cutting Concerns

### Error Handling

Follow existing M20Connection patterns:
- Import guards with `_ROS_AVAILABLE` / `_BRIDGE_AVAILABLE` flags for optional deps
- `try/except` with `logger.warning()` for non-fatal initialization failures
- Each `stop()` cleanup operation in its own `try/except` block
- Use `from dimos.utils.logging_config import setup_logger` in navigation modules, `logging.getLogger(__name__)` in robot modules

Container crash recovery is handled by DockerModule:
- `docker_restart_policy: "on-failure:3"` is a native field on `DockerModuleConfig` with the correct default — no override needed in `docker_extra_args`
- M20 motion controller watchdog stops robot if no velocity commands arrive for >500ms

### Testing Strategy

**Unit tests (Phase 1):**
- `M20ROSNavConfig` defaults and `__post_init__` behavior
- Blueprint import test via `test_all_blueprints.py` (automatic after registration)
- Module compliance via `test_modules.py` (automatic for any Module subclass)

**Integration tests (Phase 4):**
- DDS topic discovery on real hardware
- FASTLIO2 SLAM convergence
- End-to-end patrol loop

**No new test files needed** — existing auto-scan tests (`test_all_blueprints.py`, `test_modules.py`) cover the new blueprint and config. The config dataclass can be tested inline or with a small test in `dimos/robot/deeprobotics/tests/`.

### Migration

No data migration needed. This is a clean architectural split:
- No maps to migrate (starting fresh with FASTLIO2 per spec decision Q8)
- No configuration migration (new config from scratch)
- **Rollback**: `deploy.sh stop`, then `git checkout <pre-migration-commit>` and use the old monolithic deploy. No `--legacy` flag — git rollback is simpler and avoids dual-codepath maintenance.

### Spec Deviations

| Deviation | Spec Says | Plan Does | Rationale |
|-----------|-----------|-----------|-----------|
| ROS path removal | §1.4: "Remove the ROS path entirely (no more rclpy in M20Connection)" | Keep M20Connection unchanged; suppress via `enable_ros=False, enable_lidar=False` | Flag approach preserves `m20_minimal` blueprint which still uses the ROS path for teleop mode. Clean removal would break existing functionality. |
| Blueprint path | §1.5: `dimos/robot/deeprobotics/m20/blueprints/m20_rosnav.py` | `blueprints/rosnav/m20_rosnav.py` | Follows codebase convention — existing `basic/`, `agentic/`, `smart/` subdirectory pattern. |
| SLAM state UX | Spec requires "Localization Degraded" / "Localization Lost" in dimos viewer | Deferred to Phase 2 | Phase 1 focuses on getting FASTLIO2 working. Covariance monitoring and state propagation to the viewer require arise_slam's richer state model. |
| Network loss handling | Spec §Error Handling: DDS reconnect, FASTLIO2 reconvergence | Not explicitly implemented | DDS handles reconnection automatically. FASTLIO2 continues integrating IMU during brief network drops. Validated implicitly during Task 4.3 end-to-end testing. Explicit network-loss testing deferred to Phase 2. |

---

## Technical Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Python 3.10 not available via uv on RK3588 aarch64 | M | H | Build from source or use deadsnakes PPA. Test during `deploy.sh setup` before committing. |
| drdds-ros2-msgs won't build for Python 3.10 | M | H | Has existing `build_drdds_bindings.sh` script. If fails, fall back to UDP-only velocity (no /NAV_CMD). |
| Nav container image too large for NOS storage | L | M | 46GB free. If image >15GB, use `--squash` or multi-stage optimization. |
| NOS OOM during operation | M | H | Docker `--memory=1.5g` isolates container. 4GB swap on host. Fallback: offload SLAM to AOS. |
| FASTLIO2 doesn't converge with merged dual-lidar cloud | L | H | robosense_fast_lio fork claims support. If fails, try single-lidar or tune iEKF parameters. |
| Stream name conflicts in autoconnect | M | M | Mitigated by `enable_ros=False, enable_lidar=False` on M20Connection. Verify with import test. |
| DDS peer filtering doesn't exclude GOS rsdriver | L | M | fastdds_m20.xml explicit peer list. Verify with `ros2 topic info` on NOS. |

---

## Phase Gates

### Phase 1 → Phase 2 Gate (arise_slam)
- **Criteria:** Phase 1 complete (single-floor autonomous nav working with FASTLIO2)
- **Decision:** 2-week arise_slam spike — port `arise_slam_mid360` to RoboSense RSAIRY, validate loop closure + relocalization
- **Go/No-Go:** If arise_slam can't handle RSAIRY scan pattern within 2 weeks, evaluate alternatives (KISS-ICP, hdl_graph_slam)

### Phase 2 → Phase 3 Gate (Multi-Floor for the factory pilot)
- **Criteria:** Relocalization validated (robot reboots, finds itself on saved map), map persistence works, arise_slam stable
- **Decision:** Stair-climbing gait tested, elevator transition logic validated, multi-floor path planning functional
- **Go/No-Go:** Must pass all criteria before pilot factory deployment

*These gates are defined in the spec. Separate plans will be created for Phases 2 and 3.*

---

## Spec Coverage Matrix

| Spec Section | Plan Section | Phase |
|-------------|-------------|-------|
| 1.1 NOS Host Setup | Phase 2, Task 2.1 (deploy.sh setup) | 2 |
| 1.2 M20ROSNav DockerModule Config | Phase 1, Task 1.1 (M20ROSNavConfig) | 1 |
| 1.3 ROSNav Bridge Adaptations | No code changes needed — existing `_on_cmd_vel` handles Twist→NavCmd. Verified via Task 1.2 (autoconnect wiring) + Task 4.3 (end-to-end `/NAV_CMD` test) | 1, 4 |
| 1.4 M20Connection Changes | Phase 1, Task 1.2 (enable_ros=False in blueprint) | 1 |
| 1.5 New Blueprint | Phase 1, Task 1.2 (m20_rosnav blueprint) | 1 |
| 1.6 Lidar Pipeline | Phase 3, Task 3.2 (fastdds_m20.xml) + Phase 4 (verification) | 3-4 |
| 1.7 deploy.sh Changes | Phase 2, Tasks 2.1-2.4 | 2 |
| 1.8 Disabling lio_perception | Phase 2, Task 2.2 (ensure_lio_disabled) | 2 |
| Resolved Gap 1: Acceptance criteria | Phase 4, Task 4.3 (patrol loop) | 4 |
| Resolved Gap 2: DDS domain ID | Phase 1, Task 1.1 (ROS_DOMAIN_ID=0 in config) | 1 |
| Resolved Gap 3: NOS memory budget | Phase 1, Task 1.1 (--memory=1.5g) | 1 |
| Resolved Gap 4: Concurrent rsdriver | Phase 3, Task 3.2 (fastdds_m20.xml peer filtering) | 3 |
| Resolved Gap 5: Velocity translation | Phase 1, Task 1.2 (M20Connection cmd_vel routing) | 1 |
| Data Flow (Steps 1-7) | Phase 4, Task 4.3 (end-to-end verification) | 4 |
| Error Handling: Container crash | Handled by native `docker_restart_policy` field (inherited default `"on-failure:3"`) | 1 |
| Error Handling: SLAM divergence | Phase 4, Task 4.2 (FASTLIO2 convergence test) | 4 |
| Error Handling: Lidar failure | Phase 2, Task 2.2 (rsdriver health check) | 2 |
| Error Handling: OOM | Phase 1, Task 1.1 (memory limits) | 1 |

---

## Appendix: Key File Paths

### New Files

| Path | Phase | Purpose |
|------|-------|---------|
| `dimos/robot/deeprobotics/m20/rosnav_docker.py` | 1 | M20ROSNavConfig dataclass |
| `dimos/robot/deeprobotics/m20/blueprints/rosnav/__init__.py` | 1 | Package init |
| `dimos/robot/deeprobotics/m20/blueprints/rosnav/m20_rosnav.py` | 1 | m20_rosnav blueprint |
| `dimos/robot/deeprobotics/m20/docker/fastdds_m20.xml` | 3 | M20-specific DDS discovery config |

### Modified Files

| Path | Phase | Changes |
|------|-------|---------|
| `dimos/robot/all_blueprints.py` | 1 | Regenerated by `test_all_blueprints_generation.py` to include m20-rosnav |
| `dimos/robot/deeprobotics/m20/docker/launch_nos.py` | 1 | Replace with ROSNav host launcher |
| `dimos/robot/deeprobotics/m20/docker/entrypoint.sh` | 2 | Adapt for host-side use: wait for `/IMU` + `/lidar_points`, remove lio_perception refs |
| `dimos/robot/deeprobotics/m20/docker/deploy.sh` | 2 | Add setup, modify start/stop/status/dev, add ensure_lio_disabled |

### Referenced (Unchanged) Files

| Path | Purpose |
|------|---------|
| `dimos/navigation/rosnav_docker.py` | ROSNavConfig base class + ROSNav DockerModule |
| `dimos/core/docker_runner.py` | DockerModule lifecycle management |
| `dimos/core/blueprints.py` | autoconnect() wiring |
| `dimos/navigation/dimos_module_entrypoint.sh` | Container entrypoint (already supports FASTLIO2) |
| `docker/navigation/Dockerfile` | Nav container image (already arm64 + FASTLIO2) |
| `dimos/mapping/voxels.py` | VoxelGridMapper (used as-is) |
| `dimos/mapping/costmapper.py` | CostMapper (used as-is) |
| `dimos/robot/deeprobotics/m20/connection.py` | M20Connection (used as-is, enable_ros=False) |
| `dimos/robot/deeprobotics/m20/velocity_controller.py` | Twist→NavCmd translation |
| `dimos/robot/deeprobotics/m20/blueprints/basic/m20_minimal.py` | Reference pattern |
