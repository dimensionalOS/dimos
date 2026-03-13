# Beads Draft: m20-rosnav-migration

**Generated:** 2026-03-13
**Source:** plans/m20-rosnav-migration/03-plan/plan.md
**Plan review status:** Reviewed (3 P0, 8 P1, 6 P2 — all resolved)

---

## Structure

### Feature Epic: m20-rosnav-migration

**Type:** epic
**Priority:** P1
**Description:** Migrate M20 from monolithic Docker architecture to ROSNav host+container pattern. dimos runs natively on NOS host (uv/Python 3.10), CMU nav stack (FASTLIO2, FAR planner, base_autonomy) runs in a Humble Docker container managed by DockerModule. Delivers first autonomous navigation on M20.

---

### Sub-Epic: Phase 0 — Discovery (Pre-Implementation)

**Type:** epic
**Priority:** P1
**Parent:** Feature epic
**Description:** Resolve Open Questions 1-6 from the spec before any implementation begins. These are binary blockers — if any answer is "no," the downstream implementation plan changes. Requires SSH access to M20 NOS (via ProxyJump through AOS wlan0).

#### Issue: Inspect DDS topics and QoS on NOS (0.1)

**Type:** task
**Priority:** P1
**Parent:** Phase 0
**Dependencies:** None
**Description:**
SSH to NOS, enumerate DDS topics, capture QoS profiles for `/lidar_points`, `/IMU`, and `/NAV_CMD`.

- `ros2 topic info -v /lidar_points` — confirm message type is `sensor_msgs/PointCloud2`, get QoS (reliability, durability, history depth)
- `ros2 topic info -v /IMU` — confirm message type is `sensor_msgs/Imu` (NOT a drdds custom type). If custom, ROSNav bridge needs a translator.
- `ros2 topic echo /IMU --once` — capture a sample message to verify field layout
- Document QoS profiles — FASTLIO2 subscribers must match or use `BEST_EFFORT`
- Resolves: OQ1 (rsdriver DDS QoS), OQ3 (IMU message type)

SSH to NOS via: `ssh -o ProxyJump=user@10.21.41.1 user@10.21.31.106`

**Acceptance Criteria:**
- [ ] QoS profiles documented for `/lidar_points` and `/IMU`
- [ ] IMU message type confirmed as `sensor_msgs/Imu` or custom type identified with adaptation plan

#### Issue: Review robosense_fast_lio fork for dual-lidar support (0.2)

**Type:** task
**Priority:** P1
**Parent:** Phase 0
**Dependencies:** None
**Description:**
Review the `robosense_fast_lio` fork source code to confirm it handles merged dual-lidar point clouds (front+back merged by rsdriver with `send_separately: false`).

- Clone `github.com/RuanJY/robosense_fast_lio` and inspect point cloud preprocessing
- Confirm it doesn't assume single-origin points (merged cloud has points from 2 origins)
- Check if any scan deskewing assumes single-lidar rotation pattern
- If fork only supports single lidar, document adaptation needed
- Resolves: OQ2 (FASTLIO2 fork dual-lidar support)

**Acceptance Criteria:**
- [ ] Confirmed fork handles merged multi-lidar clouds, OR adaptation plan documented

#### Issue: Test Python 3.10 installation on NOS aarch64 (0.3)

**Type:** task
**Priority:** P1
**Parent:** Phase 0
**Dependencies:** None
**Description:**
SSH to NOS, attempt to install Python 3.10 via uv on the RK3588 aarch64 platform.

- `curl -LsSf https://astral.sh/uv/install.sh | sh`
- `uv python install 3.10`
- If uv doesn't have a prebuilt 3.10 for aarch64, try `uv python install 3.10 --build` or fall back to deadsnakes PPA / build from source
- Test `python3.10 -c "import sys; print(sys.platform, sys.version)"` to confirm
- Resolves: OQ4 (Python 3.10 on aarch64)

**Acceptance Criteria:**
- [ ] Python 3.10 runs on NOS, OR alternative installation method documented

#### Issue: Estimate nav container image size (0.4)

**Type:** task
**Priority:** P1
**Parent:** Phase 0
**Dependencies:** None
**Description:**
Build the nav container image for arm64 and measure its size to verify it fits on NOS storage.

- `docker buildx build --platform linux/arm64 -t m20-nav-test .` from `docker/navigation/`
- `docker images m20-nav-test` — check compressed and uncompressed size
- Compare against NOS free storage (46GB on `/var/opt/robot/data/docker`)
- If image >15GB, investigate multi-stage optimization or `--squash`
- Resolves: OQ6 (container image size)

**Acceptance Criteria:**
- [ ] Image size documented and confirmed to fit NOS storage

---

### Sub-Epic: Phase 1 — M20ROSNavConfig + Blueprint

**Type:** epic
**Priority:** P1
**Parent:** Feature epic
**Description:** Create the config dataclass, blueprint, and launcher so that `m20_rosnav` can be imported and blueprint tests pass. This is the core architectural work — everything else builds on these files.

#### Issue: Create M20ROSNavConfig dataclass (1.1)

**Type:** task
**Priority:** P1
**Parent:** Phase 1
**Dependencies:** None
**Description:**
New dataclass extending `ROSNavConfig` with M20-specific Docker settings, sensor topics, and robot parameters.

**File:** Create `dimos/robot/deeprobotics/m20/rosnav_docker.py`

Key details:
- Extend `ROSNavConfig` from `dimos.navigation.rosnav_docker`
- Override `docker_image` to `"ghcr.io/aphexcx/m20-nav:latest"`
- Override `docker_shm_size` to `"1g"` (from 8g default)
- Add `"--memory=1.5g"` and `"--memory-swap=1.5g"` to `docker_extra_args`
- Do NOT add `--restart` to `docker_extra_args` — `docker_restart_policy` is a native field on `DockerModuleConfig` with default `"on-failure:3"` already
- Set `docker_env` with `ROS_DOMAIN_ID=0`, `LOCALIZATION_METHOD=fastlio`, `MODE=hardware`, `USE_ROUTE_PLANNER=true`, `USE_RVIZ=false`
- In `__post_init__`: call `super().__post_init__()`, then replace `self.docker_volumes` with minimal NOS volume tuples (format: `(host_path, container_path, mode)`):
  - `("/opt/dimos/src", "/opt/dimos/src", "rw")` — dimos source (live editable)
  - `(str(Path(__file__).parent / "docker" / "fastdds_m20.xml"), "/ros2_ws/config/fastdds.xml", "ro")` — M20 DDS config
  - `(str(Path(__file__).parent / "docker" / "entrypoint.sh"), "/opt/dimos/entrypoint.sh", "ro")` — entrypoint
  No X11, Unity mesh, or ros_tcp_endpoint patch volumes.
- After replacing volumes, re-set `self.docker_env["ROS_DOMAIN_ID"] = "0"` as a defensive measure (base `__post_init__` doesn't currently modify it, but this guards against future changes)
- Add M20 physical parameters: `robot_width=0.45`, `lidar_height=0.47`
- Add sensor topic fields: `lidar_topic="/lidar_points"`, `imu_topic="/IMU"`, `nav_cmd_topic="/NAV_CMD"`

Reference: `dimos/navigation/rosnav_docker.py` lines 98-250 for `ROSNavConfig` base class. Note `__post_init__` at line 147 adds X11/Unity/ros_tcp_endpoint volumes that must be replaced.

**Acceptance Criteria:**
- [ ] `M20ROSNavConfig()` instantiates without error
- [ ] `config.docker_env["ROS_DOMAIN_ID"] == "0"`
- [ ] `config.docker_extra_args` contains `"--memory=1.5g"`
- [ ] `config.docker_volumes` does NOT contain X11 or Unity paths
- [ ] `config.docker_env["LOCALIZATION_METHOD"] == "fastlio"`

#### Issue: Create m20_rosnav blueprint (1.2)

**Type:** task
**Priority:** P1
**Parent:** Phase 1
**Dependencies:** Task 1.1 (Create M20ROSNavConfig dataclass)
**Description:**
Blueprint composing M20Connection (UDP-only) + VoxelGridMapper + CostMapper + visualization + ROSNav DockerModule, following the G1 `unitree_g1_basic_sim_ros` pattern.

**Files:**
- Create `dimos/robot/deeprobotics/m20/blueprints/rosnav/__init__.py` — empty package init
- Create `dimos/robot/deeprobotics/m20/blueprints/rosnav/m20_rosnav.py` — blueprint definition

Key details:
- Import `ros_nav` from `dimos.navigation.rosnav_docker` (NOT `dimos.navigation.rosnav`)
- Import `m20_connection` from `dimos.robot.deeprobotics.m20.connection`
- Construct M20Connection with `enable_ros=False, enable_lidar=False, lidar_height=0.47`
- `enable_ros=False` prevents rclpy init on host (no ROS on host)
- `enable_lidar=False` skips CycloneDDS lidar initialization (FASTLIO2 handles lidar in the container)
- **Note:** These flags suppress runtime data production but do NOT remove class-level `Out[T]` port declarations from M20Connection. The ports (`pointcloud`, `lidar`, `odom`) still exist on the class. `autoconnect()` may still attempt to wire them by name+type. If autoconnect raises a "multiple producers for stream name X" error, call `.remappings({"pointcloud": "m20_pointcloud"})` on M20Connection to rename the conflicting port.
- `dimos.navigation.rosnav` (container-side rclpy module) is a real file but is the WRONG import — it's meant to run inside the container, not on the host. Always use `dimos.navigation.rosnav_docker` which provides the `ros_nav` factory function for host-side use via DockerModule.
- Include `voxel_mapper`, `cost_mapper` with M20 HeightCostConfig, `websocket_vis`, `rerun_bridge`
- Use `ros_nav(config=M20ROSNavConfig())` as the nav module
- Set `.global_config(n_workers=2, robot_model="deeprobotics_m20", robot_ip="10.21.33.103", robot_width=0.45, robot_rotation_diameter=0.6)`
- Handle platform-specific transports: copy the `_mac_transports` dict and `_transports_base` conditional from `m20_minimal.py` lines 29-39 into the new blueprint
- Export `m20_rosnav` in `__all__`

Reference patterns:
- `dimos/robot/deeprobotics/m20/blueprints/basic/m20_minimal.py` (96 lines) — M20 blueprint pattern
- `dimos/robot/unitree/g1/blueprints/basic/unitree_g1_basic_sim_ros.py` (29 lines) — G1 ROSNav pattern: `autoconnect(unitree_g1_primitive_no_cam, ros_nav(mode="unity_sim"))`

**Acceptance Criteria:**
- [ ] `from dimos.robot.deeprobotics.m20.blueprints.rosnav.m20_rosnav import m20_rosnav` succeeds
- [ ] Blueprint can be instantiated without hardware (import-time only, no rclpy required)
- [ ] No stream name conflicts between M20Connection and ROSNav ports
- [ ] Verify M20Connection `cmd_vel: In[Twist]` is wired by `autoconnect` to ROSNav `cmd_vel: Out[Twist]` (velocity translation pipeline)

#### Issue: Register blueprint in all_blueprints.py (1.3)

**Type:** task
**Priority:** P1
**Parent:** Phase 1
**Dependencies:** Task 1.2 (Create m20_rosnav blueprint)
**Description:**
Regenerate the auto-generated blueprint registry so `m20_rosnav` is discovered and covered by `test_all_blueprints.py`.

**File:** Modify (auto-generated) `dimos/robot/all_blueprints.py`

Key details:
- `all_blueprints.py` is auto-generated — its header says "Run `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate"
- Do NOT manually edit this file. Run: `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate the registry
- The test scans for blueprints and auto-discovers `m20_rosnav` from the new file
- Note: No existing M20 blueprints are currently registered (first M20 entry)

**Acceptance Criteria:**
- [ ] `pytest dimos/robot/test_all_blueprints_generation.py` passes and regenerates the file
- [ ] `pytest dimos/robot/test_all_blueprints.py -k m20_rosnav` passes (import test)

#### Issue: Replace launch_nos.py with ROSNav host launcher (1.4)

**Type:** task
**Priority:** P1
**Parent:** Phase 1
**Dependencies:** Task 1.2 (Create m20_rosnav blueprint)
**Description:**
Replace the current monolithic Docker launcher with a minimal host-side launcher that runs the m20_rosnav blueprint natively on NOS.

**File:** Modify `dimos/robot/deeprobotics/m20/docker/launch_nos.py` — replace contents entirely

Key details:
- The existing `launch_nos.py` has a `main()` function with argparse and signal handling — replace the body but keep the same structure
- Import `m20_rosnav` blueprint
- Call `blueprint.build()` to create coordinator
- Handle SIGINT/SIGTERM for graceful shutdown
- The DockerModule inside the blueprint handles the nav container lifecycle — no manual `docker run`
- Keep the file name `launch_nos.py` since `deploy.sh` already references it
- Wrap startup in `if __name__ == "__main__"` guard so imports don't trigger execution

**Acceptance Criteria:**
- [ ] `python -c "import ast; ast.parse(open('dimos/robot/deeprobotics/m20/docker/launch_nos.py').read())"` passes (syntax valid, no side effects on import)
- [ ] Script has signal handling for graceful shutdown

---

### Sub-Epic: Phase 2 — deploy.sh + NOS Host Setup

**Type:** epic
**Priority:** P2
**Parent:** Feature epic
**Description:** Add the `setup` subcommand for one-time NOS host provisioning, and modify existing subcommands to work with the host+container split.

#### Issue: Add `setup` subcommand to deploy.sh (2.1)

**Type:** task
**Priority:** P2
**Parent:** Phase 2
**Dependencies:** None
**Description:**
New deploy.sh subcommand that provisions NOS for host-side dimos execution: installs uv, creates Python 3.10 venv, installs dimos, rebuilds drdds bindings.

**File:** Modify `dimos/robot/deeprobotics/m20/docker/deploy.sh` — add `setup)` case

Key details:
- SSH to NOS via existing ControlMaster pattern
- Install uv: `curl -LsSf https://astral.sh/uv/install.sh | sh`
- Create venv: `uv venv --python 3.10 /opt/dimos/venv`
- Activate venv and install dimos: `uv pip install -e /opt/dimos/src`
- **drdds bindings for host**: `build_drdds_bindings.sh` requires ROS2 Humble (`colcon`, `ament_cmake`) which is NOT available on the NOS host (Foxy). Two approaches:
  1. **(Preferred)** Build drdds inside the nav container (which has Humble), then copy the pre-built artifacts to the host: `docker cp <container>:/opt/drdds/lib/python3/site-packages/drdds /opt/dimos/venv/lib/python3.10/site-packages/`
  2. **(Alternative)** Run `build_drdds_bindings.sh` inside a temporary Humble container mounted to the host venv's site-packages
- **drdds PYTHONPATH**: Install drdds into the venv's site-packages so that `import drdds` works. This is critical — M20Connection publishes `/NAV_CMD` via drdds, so the host dimos process MUST be able to `from drdds.msg import NavCmd`.
- This is a one-time operation — idempotent (can be re-run safely)
- NOS sudo password handling: use existing `SUDO_PASS="${SUDO_PASS:-"'"}"` pattern
- NOS is aarch64 (RK3588) — verify uv can install Python 3.10 for this arch

**Acceptance Criteria:**
- [ ] `deploy.sh setup` completes on NOS without error
- [ ] Python 3.10 is available at `/opt/dimos/venv/bin/python3`
- [ ] `pip list` in the venv shows dimos installed
- [ ] `python -c "import drdds"` succeeds in the venv (PYTHONPATH correctly configured)

#### Issue: Add `ensure_lio_disabled` function (2.2)

**Type:** task
**Priority:** P2
**Parent:** Phase 2
**Dependencies:** None
**Description:**
Replace `ensure_lio_enabled()` with `ensure_lio_disabled()` that stops lio_perception on AOS and verifies rsdriver is still publishing `/lidar_points`.

**File:** Modify `dimos/robot/deeprobotics/m20/docker/deploy.sh` — replace function

Key details:
- Use the existing `aos_ssh()` and `aos_sudo()` helpers in deploy.sh (lines ~149-155) to SSH to AOS (10.21.31.103) — do NOT construct SSH commands manually (would break Tailscale remote mode)
- Stop lio_perception: `systemctl stop lio_perception` or `kill` the lio_ddsnode process
- Verify rsdriver still publishes `/lidar_points` (independent of lio_perception)
- This frees CPU on AOS (fallback for SLAM offload per spec)
- Remove `ensure_lio_enabled()` — rollback is via git checkout of the pre-migration commit, not dual code paths

**Acceptance Criteria:**
- [ ] After running, lio_perception is not running on AOS
- [ ] `/lidar_points` DDS topic still has publishers (rsdriver continues)
- [ ] Function is idempotent (safe to call multiple times)

#### Issue: Adapt entrypoint.sh for ROSNav architecture (2.3)

**Type:** task
**Priority:** P2
**Parent:** Phase 2
**Dependencies:** None
**Description:**
Modify the M20 Docker entrypoint script for the new ROSNav architecture. This script runs inside the nav container (invoked by DockerModule, not directly by deploy.sh). It checks DDS topic availability before the nav stack starts.

**File:** Modify `dimos/robot/deeprobotics/m20/docker/entrypoint.sh`

Key details:
- Currently waits for `/ODOM` and checks `/ALIGNED_POINTS` (both from lio_perception) — wrong for ROSNav mode
- Keep `/IMU` wait (still needed — IMU comes from robot hardware, not lio_perception)
- Replace `/ALIGNED_POINTS` check with `/lidar_points` check (from rsdriver)
- Remove lio_perception restart logic (no longer relevant — FASTLIO2 is in the nav container)
- Keep rsdriver health verification
- **Caller chain**: `deploy.sh start` → `launch_nos.py` → `blueprint.build()` → DockerModule → `docker run` → this entrypoint
- Note: `deploy.sh dev` currently copies this file into the container via `docker cp` (line ~471 of deploy.sh). If the file's container path changes, update Task 2.5's `dev` command accordingly.

**Acceptance Criteria:**
- [ ] entrypoint.sh waits for `/IMU` and `/lidar_points` (not `/ODOM` or `/ALIGNED_POINTS`)
- [ ] No references to lio_perception remain
- [ ] Script exits cleanly if required topics are not available within timeout

#### Issue: Modify `start` subcommand (2.4)

**Type:** task
**Priority:** P2
**Parent:** Phase 2
**Dependencies:** Task 1.4 (Replace launch_nos.py with ROSNav host launcher)
**Description:**
In ROSNav mode, `start` launches dimos natively on the NOS host instead of running a monolithic Docker container. The DockerModule inside dimos manages the nav container.

**File:** Modify `dimos/robot/deeprobotics/m20/docker/deploy.sh` — modify `start)` case

Key details:
- Activate the venv: `source /opt/dimos/venv/bin/activate`
- Run `launch_nos.py` on NOS host (not inside Docker)
- The blueprint's DockerModule handles pulling and starting the nav container
- Call `ensure_lio_disabled` before starting
- **Verify drdds is importable**: `python -c "import drdds"` before launching (fail fast if PYTHONPATH misconfigured)
- Daemon mode: `nohup python3 /opt/dimos/src/dimos/robot/deeprobotics/m20/docker/launch_nos.py > /var/log/dimos-nos.log 2>&1 &` and record PID in `/var/run/dimos-nos.pid` for use by `stop`
- **Rollback**: No `--legacy` flag — use git to check out the pre-migration commit instead (less maintenance burden than dual code paths)

**Acceptance Criteria:**
- [ ] `deploy.sh start` launches dimos on the NOS host
- [ ] DockerModule automatically starts the nav container
- [ ] `python -c "import drdds"` check passes before launch

#### Issue: Modify `stop`, `status`, `dev` subcommands (2.5)

**Type:** task
**Priority:** P2
**Parent:** Phase 2
**Dependencies:** None
**Description:**
Update remaining subcommands for host+container split.

**File:** Modify `dimos/robot/deeprobotics/m20/docker/deploy.sh` — modify `stop)`, `status)`, `dev)` cases

Key details:
- `stop`: Find host dimos PID via `cat /var/run/dimos-nos.pid` or `pgrep -f launch_nos.py`, send SIGTERM, wait up to 10s for DockerModule to stop the container, then verify with `docker ps --filter name=m20-nav`.
- `status`: Check host dimos via `pgrep -f launch_nos.py`, nav container via `docker ps --filter name=m20-nav`, rsdriver via `ros2 topic hz /lidar_points --once` (1 message timeout). Replace lio_perception checks with rsdriver/FASTLIO2 checks.
- `dev`: rsync dimos source to NOS host path (not just container volume). The DockerModule volume mount picks up changes from the host path.
- `logs`: Aggregate host dimos logs + container logs (docker logs)

**Acceptance Criteria:**
- [ ] `deploy.sh stop` cleanly stops both host dimos and nav container
- [ ] `deploy.sh status` shows health of host, container, and rsdriver
- [ ] `deploy.sh dev` syncs source changes to NOS

---

### Sub-Epic: Phase 3 — Nav Container Image Build + Push

**Type:** epic
**Priority:** P2
**Parent:** Feature epic
**Description:** Build the M20-specific nav container image from the existing Dockerfile (already arm64 + FASTLIO2 capable) and push to ghcr.io.

#### Issue: Build and push M20 nav container image (3.1)

**Type:** task
**Priority:** P2
**Parent:** Phase 3
**Dependencies:** None
**Description:**
Build `docker/navigation/Dockerfile` for arm64, tag as `ghcr.io/aphexcx/m20-nav:latest`, and push.

**Files:** No code file changes — uses existing `docker/navigation/Dockerfile`. Optionally modify `docker/navigation/build.sh` to add M20-specific build target.

Key details:
- The existing Dockerfile is multi-arch (amd64/arm64) and already includes FASTLIO2 + arise_slam + FAR planner + base_autonomy
- Build for arm64: `docker buildx build --platform linux/arm64 -t ghcr.io/aphexcx/m20-nav:latest --push .`
- Verify image size — if >10GB, initial pull on NOS will be slow over 5G but is one-time
- The `LOCALIZATION_METHOD=fastlio` env var in M20ROSNavConfig controls which SLAM runs at runtime
- arise_slam is included but dormant in Phase 1

**Acceptance Criteria:**
- [ ] `docker pull ghcr.io/aphexcx/m20-nav:latest` succeeds on NOS (arm64)
- [ ] Container starts with `LOCALIZATION_METHOD=fastlio` and FASTLIO2 initializes
- [ ] Container starts with `ROS_DOMAIN_ID=0` and discovers `/lidar_points` DDS topic from rsdriver

#### Issue: Create M20-specific fastdds.xml (3.2)

**Type:** task
**Priority:** P2
**Parent:** Phase 3
**Dependencies:** Task 1.1 (Create M20ROSNavConfig dataclass)
**Description:**
DDS discovery configuration for the M20 network topology — NOS discovering topics from AOS rsdriver, with peer filtering to exclude GOS.

**File:** Create `dimos/robot/deeprobotics/m20/docker/fastdds_m20.xml`

Key details:
- Use `<initialPeersList>` with only AOS unicast address to suppress multicast discovery of GOS. The existing `docker/navigation/config/fastdds.xml` uses `discoveryProtocol: SIMPLE` (multicast-based) which would discover both boards. Add an `<initialPeersList>` section with `<locator><udpv4><address>10.21.31.103</address></udpv4></locator>` and set `<metatrafficUnicastLocatorList>` to disable multicast discovery.
- This resolves the dual-rsdriver problem — container only sees AOS's `/lidar_points`
- Reference existing `docker/navigation/config/fastdds.xml` for base format (transport descriptors, buffer sizes)
- The volume mount for this file is configured in `M20ROSNavConfig.__post_init__` (Task 1.1) as `(str(Path(__file__).parent / "docker" / "fastdds_m20.xml"), "/ros2_ws/config/fastdds.xml", "ro")`. Verify Task 1.1 includes this mount and uses the same host path.

**Acceptance Criteria:**
- [ ] DDS discovery finds AOS rsdriver topics only
- [ ] GOS rsdriver topics are not discovered
- [ ] FASTLIO2 receives `/lidar_points` from AOS at 10Hz

---

### Sub-Epic: Phase 4 — End-to-End Integration Test

**Type:** epic
**Priority:** P2
**Parent:** Feature epic
**Description:** Validate the complete data flow from lidars to velocity commands on the real M20 robot. Requires Phases 1-3 complete and robot accessible via SSH.

#### Issue: Verify DDS topic connectivity (4.1)

**Type:** task
**Priority:** P2
**Parent:** Phase 4
**Dependencies:** Task 3.1 (Build and push M20 nav container image), Task 3.2 (Create M20-specific fastdds.xml)
**Description:**
SSH to NOS, start the nav container manually, verify it discovers `/lidar_points` and `/IMU` from AOS.

No file changes.

Key details:
- SSH to NOS via ProxyJump
- Pull the nav container: `docker pull ghcr.io/aphexcx/m20-nav:latest`
- Start container with `--network host` and `ROS_DOMAIN_ID=0`
- Inside container: `ros2 topic list` should show `/lidar_points` and `/IMU`
- `ros2 topic hz /lidar_points` should show ~10Hz
- `ros2 topic hz /IMU` should show ~200Hz

**Acceptance Criteria:**
- [ ] Container discovers AOS DDS topics on domain 0
- [ ] `/lidar_points` rate is ~10Hz
- [ ] `/IMU` rate is ~200Hz

#### Issue: Verify FASTLIO2 SLAM initialization (4.2)

**Type:** task
**Priority:** P2
**Parent:** Phase 4
**Dependencies:** Task 4.1 (Verify DDS topic connectivity)
**Description:**
Start FASTLIO2 inside the container and verify it produces odometry and registered scans.

No file changes.

Key details:
- The `dimos_module_entrypoint.sh` launches FASTLIO2 when `LOCALIZATION_METHOD=fastlio`
- Check `/registered_scan` topic is publishing
- Check FASTLIO2's fused odometry output
- Verify scan quality: points should be coherent (not scattered)
- Monitor iEKF convergence in FASTLIO2 logs

**Acceptance Criteria:**
- [ ] FASTLIO2 produces registered point clouds
- [ ] Fused odometry is reasonable (robot stationary → near-zero drift)
- [ ] No SLAM divergence after 60 seconds

#### Issue: Full pipeline test with deploy.sh (4.3)

**Type:** task
**Priority:** P2
**Parent:** Phase 4
**Dependencies:** Task 4.2 (Verify FASTLIO2 SLAM initialization), Task 2.5 (Modify `stop`, `status`, `dev` subcommands)
**Description:**
Run `deploy.sh setup` then `deploy.sh start` and verify the complete data flow.

No file changes.

Key details:
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

**Acceptance Criteria:**
- [ ] Robot completes a patrol loop (sequence of waypoints) and returns to start
- [ ] No human intervention required during patrol
- [ ] `deploy.sh status` shows all components healthy during operation
- [ ] `deploy.sh stop` cleanly shuts down everything
- [ ] NOS memory stays under 80% during operation

---

## Dependencies

| Blocked Task | Blocked By | Reason |
|-------------|------------|--------|
| Task 1.2 (Create m20_rosnav blueprint) | Task 1.1 (Create M20ROSNavConfig dataclass) | Blueprint needs M20ROSNavConfig to instantiate ROSNav |
| Task 1.3 (Register blueprint in all_blueprints.py) | Task 1.2 (Create m20_rosnav blueprint) | Blueprint must exist to be discovered by registry regeneration |
| Task 1.4 (Replace launch_nos.py) | Task 1.2 (Create m20_rosnav blueprint) | Launcher imports the blueprint |
| Task 2.4 (Modify `start` subcommand) | Task 1.4 (Replace launch_nos.py) | Start calls `launch_nos.py` which must use the new blueprint |
| Task 3.2 (Create M20-specific fastdds.xml) | Task 1.1 (Create M20ROSNavConfig dataclass) | fastdds_m20.xml path must match M20ROSNavConfig volume mount definition |
| Task 4.1 (Verify DDS topic connectivity) | Task 3.1 (Build and push M20 nav container image) | Container image must be available to test DDS connectivity |
| Task 4.1 (Verify DDS topic connectivity) | Task 3.2 (Create M20-specific fastdds.xml) | GOS-exclusion acceptance criterion requires fastdds.xml peer filtering |
| Task 4.2 (Verify FASTLIO2 SLAM initialization) | Task 4.1 (Verify DDS topic connectivity) | DDS must work before SLAM can receive data |
| Task 4.3 (Full pipeline test) | Task 4.2 (Verify FASTLIO2 SLAM initialization) | Pipeline requires working SLAM |
| Task 4.3 (Full pipeline test) | Task 2.5 (Modify `stop`, `status`, `dev`) | Pipeline test runs all deploy.sh subcommands — Plan explicitly requires "all of Phase 2" |

**Reading this table:** each row means the "Blocked Task" cannot start until "Blocked By" completes. This matches `bd dep add` argument order: `bd dep add <blocked-task-id> <blocked-by-id>`.

## Coverage Matrix

| Plan Task | Bead Title | Sub-Epic |
|-----------|------------|----------|
| 0.1 Inspect DDS topics and QoS on NOS | Inspect DDS topics and QoS on NOS | Phase 0: Discovery |
| 0.2 Review robosense_fast_lio fork | Review robosense_fast_lio fork for dual-lidar support | Phase 0: Discovery |
| 0.3 Test Python 3.10 on NOS aarch64 | Test Python 3.10 installation on NOS aarch64 | Phase 0: Discovery |
| 0.4 Estimate nav container image size | Estimate nav container image size | Phase 0: Discovery |
| 1.1 Create M20ROSNavConfig dataclass | Create M20ROSNavConfig dataclass | Phase 1: M20ROSNavConfig + Blueprint |
| 1.2 Create m20_rosnav blueprint | Create m20_rosnav blueprint | Phase 1: M20ROSNavConfig + Blueprint |
| 1.3 Register blueprint in all_blueprints.py | Register blueprint in all_blueprints.py | Phase 1: M20ROSNavConfig + Blueprint |
| 1.4 Replace launch_nos.py | Replace launch_nos.py with ROSNav host launcher | Phase 1: M20ROSNavConfig + Blueprint |
| 2.1 Add `setup` subcommand | Add `setup` subcommand to deploy.sh | Phase 2: deploy.sh + NOS Host Setup |
| 2.2 Add `ensure_lio_disabled` function | Add `ensure_lio_disabled` function | Phase 2: deploy.sh + NOS Host Setup |
| 2.3 Adapt entrypoint.sh | Adapt entrypoint.sh for host-side use | Phase 2: deploy.sh + NOS Host Setup |
| 2.4 Modify `start` subcommand | Modify `start` subcommand | Phase 2: deploy.sh + NOS Host Setup |
| 2.5 Modify `stop`, `status`, `dev` | Modify `stop`, `status`, `dev` subcommands | Phase 2: deploy.sh + NOS Host Setup |
| 3.1 Build and push nav container image | Build and push M20 nav container image | Phase 3: Nav Container Image Build + Push |
| 3.2 Create M20-specific fastdds.xml | Create M20-specific fastdds.xml | Phase 3: Nav Container Image Build + Push |
| 4.1 Verify DDS topic connectivity | Verify DDS topic connectivity | Phase 4: End-to-End Integration Test |
| 4.2 Verify FASTLIO2 SLAM initialization | Verify FASTLIO2 SLAM initialization | Phase 4: End-to-End Integration Test |
| 4.3 Full pipeline test | Full pipeline test with deploy.sh | Phase 4: End-to-End Integration Test |

**Plan tasks:** 18
**Beads mapped:** 18
**Coverage:** 100%

## Summary

- Feature epic: 1
- Sub-epics (phases): 5
- Issues (tasks): 18
- Blocker dependencies: 10
- Items ready immediately (no blockers): 11 (Tasks 0.1, 0.2, 0.3, 0.4, 1.1, 2.1, 2.2, 2.3, 2.5, 3.1)

### Review Passes Applied
- **Pass 1 (Completeness):** PASS — 0 fixes needed
- **Pass 2 (Dependencies):** FAIL → FIXED — removed 2 false blockers (2.1→2.4, 2.4→2.5), added 4 missing blockers (1.4→2.4, 1.1→3.2, 3.2→4.1, 2.5→4.3)
- **Pass 3 (Clarity):** FAIL → FIXED — added volume tuple details (1.1), clarified remappings guidance (1.2), fixed import-safety AC (1.4), documented drdds Humble dependency (2.1), resolved legacy contradiction (2.2), clarified entrypoint caller chain (2.3), specified daemon/stop mechanism (2.4/2.5), added DDS discovery guidance (3.2)
