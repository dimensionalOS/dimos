# M20 ROSNav Migration Log

**Goal**: Replace M20's proprietary drdds-based navigation stack with FAST_LIO (ROS2 Humble) + dimos ROSNav, enabling autonomous navigation on the M20 quadruped.

**Date started**: 2026-03-12
**Last updated**: 2026-03-18
**Branch**: `integration/m20-rosnav-migration`

---

## Context

The M20 robot uses Deep Robotics' proprietary "drdds" SDK (a wrapper around eProsima FastDDS 2.14) for all inter-process DDS communication. drdds publishers are **invisible to ROS2's rmw_fastrtps layer** — DDS endpoint discovery partially works (topic names visible) but publisher matching fails. This means rclpy and rclcpp subscribers cannot receive data from rsdriver or yesense, even on the same host.

The migration requires a C++ bridge (`drdds_recv` + `ros2_pub`) to translate drdds topics into standard ROS2 Humble topics that FAST_LIO can consume.

---

## Architecture: drdds-to-ROS2 Bridge

```
                    NOS Host                          │  Nav Container (Humble)
                                                      │
  rsdriver ──drdds──► drdds_recv ──POSIX SHM──►──────►│──► ros2_pub ──ROS2──► FAST_LIO
  (lidar)         /LIDAR/POINTS   ring buffer         │  /bridge/LIDAR_POINTS    │
                                                      │                          ▼
  yesense ──drdds──► drdds_recv ──POSIX SHM──►──────►│──► ros2_pub ──ROS2──►  /Odometry
  (IMU)            /IMU           ring buffer         │  /bridge/IMU          (10Hz output)
```

**Why two processes?** drdds links against FastDDS 2.14 (`/usr/local/lib/libfastrtps.so.2.14`). ROS2 Humble links against FastDDS 2.6 (`/opt/ros/humble/lib/libfastrtps.so`). Symbol conflicts prevent coexistence in one process. The POSIX shared memory ring buffer bridges them with zero-copy semantics.

**Container requirements**: `--ipc host` (shares `/dev/shm`), `--network host`, `--privileged`

---

## Finding #1: drdds Topic Names (2026-03-12)

rsdriver's config YAML says `dds_send_point_cloud_topic: /lidar_points`, but the **actual DDS topic name** hardcoded in the binary is `/LIDAR/POINTS`. Verified via `strings /opt/robot/share/node_driver/bin/rslidar | grep "^/"`.

Similarly, IMU topic is `/IMU` (not `/imu/data` from config YAML).

**Lesson**: Always check the binary for hardcoded topic names — config YAML values may be defaults overridden by code.

---

## Finding #2: DrDDSSubscriber vs DrDDSChannel (2026-03-17)

`DrDDSSubscriber<T>(callback, topic, domain, prefix)` alone does NOT match rsdriver's publisher. rsdriver uses `DrDDSChannel<T>` which creates both a publisher and subscriber internally with different DDS participant registration.

`DrDDSChannel<T>(callback, topic, domain)` works — it creates matching pub+sub endpoints that rsdriver can discover.

```cpp
// DOES NOT WORK:
DrDDSSubscriber<PointCloud2PubSubType> sub(cb, "/LIDAR/POINTS", 0, "");
// GetMatchedCount() == 0, no data

// WORKS:
DrDDSChannel<PointCloud2PubSubType> ch(cb, "/LIDAR/POINTS", 0);
// GetMatchedCount() == 1, data flows at 10Hz
```

**Note**: `DrDDSSubscriber` with empty prefix DOES get `GetMatchedCount() == 1`, but this may be from DDS endpoint matching without compatible data transport. Only `DrDDSChannel` actually receives data.

---

## Finding #3: DrDDSManager::Init Signatures (2026-03-18)

Two Init methods exist with very different behavior:

```cpp
// Simple Init — creates mLocalParticipant_
static void Init(int domain_id, const std::string network_name = "");

// Multi-domain Init — creates mMultiParticipant_
static void Init(std::vector<int> domain_id, std::string module_id,
                 std::string node_name, bool local_host = false,
                 bool enable_config = false, bool enable_discovery = false);
```

| Process | Init Used | Verified Via |
|---------|-----------|-------------|
| rsdriver | Multi-domain | `nm -D rslidar \| grep Init` |
| handler | Simple | `nm -D handler \| grep Init` |
| lio_perception | Simple | `nm -D lio_ddsnode \| grep Init` |
| drdds_recv (working) | Multi-domain | Code |

**Critical parameters for drdds_recv**:
```cpp
DrDDSManager::Init(vector<int>{0}, "drdds_bridge", "drdds_recv", false, false, false);
```
- `local_host=false` — do NOT set to true (changes transport)
- `enable_config=false` — setting to true causes segfault (looks up XML profiles)
- `enable_discovery=false` — setting to true disables self-matching

---

## Finding #4: rsdriver Uses SHM-Only Transport (2026-03-18)

rsdriver has **zero UDP sockets** (`ss -ulnp | grep $(pgrep rslidar)` returns empty). It communicates entirely via FastDDS Shared Memory transport, using:
- `/dev/shm/fastrtps_*` — SHM segment files for participant discovery and data
- `/dev/shm/fast_datasharing_*` — FastDDS Data Sharing zero-copy buffers (1.3MB each for PointCloud2)
- `/dev/shm/fastrtps_port*` — SHM port files for transport

All SHM files are root-owned (rsdriver runs as root via systemd).

The drdds_recv bridge also has UDP sockets (on 10.21.33.106, eth0), but these are unused for rsdriver communication. All actual data transfer happens through SHM.

---

## Finding #5: CRITICAL — SHM Discovery Startup Ordering (2026-03-18)

**The most important finding of this investigation.**

FastDDS SHM-only transport has a **startup ordering requirement**: the subscriber must be running BEFORE the publisher starts its DDS participant. If the publisher starts first, the subscriber cannot discover it — endpoints match (GetMatchedCount > 0) but data never flows.

This means `drdds_recv` must start BEFORE `rsdriver` and `yesense`.

**Correct startup sequence**:
```bash
# 1. Stop publishers
sudo systemctl stop rsdriver yesense

# 2. Clean ALL stale SHM segments
sudo rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* \
           /dev/shm/sem.fastrtps_* /dev/shm/drdds_bridge_*

# 3. Start subscriber FIRST (must run as root)
sudo nohup /opt/drdds_bridge/lib/drdds_bridge/drdds_recv > /tmp/drdds_recv.log 2>&1 &
sleep 3

# 4. Start rsdriver (has 30s sleep in startup script)
sudo systemctl start rsdriver
sleep 35

# 5. Start yesense (for IMU)
sudo systemctl start yesense
sleep 5

# 6. Verify
grep "lidar_msgs" /tmp/drdds_recv.log  # should show lidar_msgs > 0 AND imu_msgs > 0
```

**Symptoms of wrong ordering**: `GetMatchedCount() == 1` (self-match from DrDDSChannel) but `lidar_msgs == 0`. `GetParticipants() == 0` (no remote participant discovery). rsdriver logs "send success" but no subscriber receives data.

**Root cause**: FastDDS 2.14's SHM discovery mechanism only announces new participants to already-running processes. A subscriber that starts after the publisher will never learn about the publisher's participant. This appears to be a known limitation/bug in FastDDS SHM transport (see [FastDDS Issue #5053](https://github.com/eProsima/Fast-DDS/issues/5053)).

**Debugging breadcrumbs that led to discovery**:
- rsdriver has zero UDP sockets → SHM-only transport
- Our bridge had UDP sockets on wrong interface (eth0 / 10.21.33.106 instead of eth1 / 10.21.31.106)
- Even with correct interface, no data flowed
- `DrDDSManager::GetParticipants()` always returned 0
- Codex suggested Data Sharing transport incompatibility
- Starting bridge BEFORE rsdriver (out of desperation) suddenly worked

---

## Finding #6: NOS Has Dual NICs (2026-03-18)

NOS has two Ethernet interfaces with different IPs:
- **eth0**: 10.21.33.106 (internal LAN)
- **eth1**: 10.21.31.106 (robot control LAN, used by SSH/NAT)

The system FastDDS config (`/opt/robot/fastdds.xml`) whitelists `10.21.31.106` (eth1). But DrDDSManager::Init ignores this XML and binds to eth0 by default. This caused confusion during debugging but is ultimately irrelevant since rsdriver doesn't use UDP.

---

## Finding #7: drdds QoS Configuration Files (2026-03-18)

Deep Robotics ships QoS configs at `/opt/drdds/dr_qos/`:

| File | Purpose |
|------|---------|
| `drqos.xml` | Main QoS profiles — RELIABLE reliability + 50ms deadline for PointCloud2 |
| `participant.xml` | Sample participant config (not used in production) |
| `publisher_subscriber.xml` | Pub/sub QoS |
| `topic.xml` | Topic QoS |
| `datawriter_datareader.xml` | DataWriter/DataReader QoS |

The path `/opt/drdds/dr_qos/drqos.xml` is **hardcoded in libdrdds.so** (`strings libdrdds.so | grep drqos`). It's loaded when `enable_config=true` in `DrDDSManager::Init()`.

Setting `enable_config=true` with our bridge causes a segfault because the XML parser fails to find topic profiles for `sensor_msgs::msg::dds_::PointCloud2_` (the profiles exist but the DomainParticipantFactory requires `FASTRTPS_DEFAULT_PROFILES_FILE` env var to load them).

**Lesson**: Do NOT use `enable_config=true` — leave it false and let the defaults work.

---

## Finding #8: Sample drdds-ros2 Code on Robot (2026-03-18)

Deep Robotics ships sample code at `/opt/robot/samples/drdds-ros2/`:
- `src/lidar_sub/lidar_sub.cpp` — Standard rclcpp subscriber for `/LIDAR/POINTS`
- `src/joints/` — Joint control sample
- `src/service/` — Service sample

The lidar_sub sample is a plain ROS2 Foxy subscriber (NOT using drdds SDK). It demonstrates the intended ROS2 interface but **does not work** because rsdriver's drdds publisher is invisible to ROS2's rmw layer. This confirms the bridge is necessary.

---

## Finding #9: PointCloud2 Field Layout from rsdriver (2026-03-18)

rsdriver produces PointCloud2 with these fields (verified via `ros2 topic echo`):

| Field | Offset | Datatype | Size |
|-------|--------|----------|------|
| x | 0 | FLOAT32 (7) | 4B |
| y | 4 | FLOAT32 (7) | 4B |
| z | 8 | FLOAT32 (7) | 4B |
| intensity | 12 | FLOAT32 (7) | 4B |
| ring | 16 | UINT16 (4) | 2B |
| timestamp | 18 | FLOAT64 (8) | 8B |

**Point step: 26 bytes**. Typical frame: 70K-115K points, 1.8-3.0 MB per message.

The bridge remaps `timestamp` → `time` in the field name (ros2_pub.cpp line 105) for FAST_LIO compatibility.

---

## Finding #10: FAST_LIO Configuration for RSAIRY Lidars (2026-03-18)

Working FAST_LIO config (`/ros2_ws/src/fast_lio/config/robosense.yaml`):

```yaml
common:
    lid_topic: "/bridge/LIDAR_POINTS"
    imu_topic: "/IMU"           # Direct from yesense (readable after bridge restart ordering)
    time_sync_en: false
preprocess:
    lidar_type: 5               # Default — pcl::PointXYZI, no per-point timestamp
    scan_line: 32
    scan_rate: 10
    timestamp_unit: 2           # Microseconds (unused with lidar_type 5)
    blind: 0.5                  # Min distance filter
mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
```

**Key**: `lidar_type: 5` (default/generic) works. Types 1 (VELO16) and 2 (OUST64) have incompatible field expectations for RSAIRY.

**IMU topic**: `/IMU` from yesense is directly readable by ROS2 Humble — no bridge needed. Verified 2026-03-18: stopped drdds_recv entirely, `/IMU` still readable at 200Hz inside container. Yesense's drdds publisher is ROS2-compatible (unlike rsdriver's). IMU removed from bridge code.

---

## Finding #11: FAST_LIO "lidar loop back" Timestamp Issue (2026-03-18)

FAST_LIO repeatedly logs "lidar loop back, clear buffer" when:
1. ros2_pub restarts and reads stale SHM data with old timestamps
2. Multiple FAST_LIO instances run simultaneously (each logs to the same file)

**Fix**: Always kill old ros2_pub and FAST_LIO instances before starting new ones:
```bash
docker exec dimos-nav bash -c 'killall ros2_pub fastlio_mapping 2>/dev/null'
```

With a clean restart sequence (ros2_pub first, wait 5s, then FAST_LIO), timestamps are monotonic and FAST_LIO processes normally.

**Verified**: drdds_recv timestamps are strictly monotonic (~1.0s apart at 10Hz). The timestamp field from rsdriver uses steady-clock-like seconds that increase reliably.

---

## Finding #12: ros2_pub Must Unset FASTRTPS_DEFAULT_PROFILES_FILE (2026-03-18)

The nav container has `FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds.xml` set. This XML has parsing errors that prevent ROS2 DDS from working properly. ros2_pub must be started with the env var unset:

```bash
docker exec -d dimos-nav bash -c 'unset FASTRTPS_DEFAULT_PROFILES_FILE && \
    source /opt/ros/humble/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    ros2 run drdds_bridge ros2_pub'
```

Similarly, any `ros2 topic` CLI commands inside the container need the same env unset.

---

## Finding #13: pkill -f Kills SSH Sessions (2026-03-18)

Running `pkill -f drdds_recv` over SSH kills the SSH session itself because the SSH command line contains "drdds_recv" as an argument and `pkill -f` matches the full command line of ALL processes including SSH.

**Workaround**: Use `killall drdds_recv` or `pgrep -x drdds_recv | xargs kill`.

---

## Verified Working Pipeline (2026-03-18)

| Component | Process | Host | Rate | Status |
|-----------|---------|------|------|--------|
| Lidar multicast | RSAIRY×2 | hardware | 10Hz | Working |
| rsdriver | rslidar | NOS (root) | 10Hz pub | Working |
| yesense | yesense_driver | NOS (root) | 200Hz pub | Working |
| drdds_recv | drdds_recv | NOS (root) | 10Hz lidar + 200Hz IMU | Working |
| ros2_pub | ros2_pub | container | ~9Hz lidar + IMU | Working |
| FAST_LIO | fastlio_mapping | container | ~10Hz odometry | Working |

**Data rates measured**:
- drdds_recv: 115K pts/frame, 3.0 MB/frame at 10Hz (dual RSAIRY merged)
- ros2_pub: /bridge/LIDAR_POINTS at 9Hz, /bridge/IMU at 200Hz
- FAST_LIO: /Odometry at 10Hz

**Resource usage**:
- drdds_recv: minimal CPU (callback-driven)
- ros2_pub: low CPU (SHM poll at 2kHz + ROS2 publish)
- FAST_LIO: 79% CPU on ARM64 (acceptable for SLAM)

---

## Finding #14: Bridge Startup Breaks charge_manager and reflective_column (2026-03-18)

The bridge startup sequence (Finding #5) cleans all `/dev/shm/fast_datasharing_*` segments. This breaks `charge_manager` and `reflective_column` because they hold references to rsdriver's old Data Sharing segments.

**Symptom**: charge_manager logs `Failed to open segment fast_datasharing_... No such file or directory`. Return-to-charger button does nothing.

**Fix**: After the bridge startup sequence completes, restart both charging services:
```bash
sudo systemctl restart reflective_column charge_manager
```

**Note**: The service name is `reflective_column` (not `reflective_column_node`). Its binary lives at `/opt/robot/share/charge_manager/bin/reflective_column_node`.

---

## Remaining Work

### Immediate Next Steps

1. **Install systemd service for reboot persistence**: Run `./deploy.sh bridge-install` to install `drdds-bridge.service` on NOS. This ensures drdds_recv starts before rsdriver/yesense on boot, and restarts charge_manager/reflective_column after rsdriver is up.

2. **FAST_LIO odometry verification**: Check that `/Odometry` produces reasonable pose estimates (not drifting wildly). The "No point, skip this scan!" warning on first scan may indicate field format issues — some scans might be empty.

3. **Rebuild nav container image**: ros2_pub was rebuilt inside the running container. Need to update the Dockerfile and push a new image so the fix persists across container restarts.

### FAST_LIO → dimos Integration

4. **Connect dimos to FAST_LIO odometry**: dimos needs `/Odometry` for costmap building and path planning. The M20Connection class must subscribe to FAST_LIO's output instead of lio_perception's `/ODOM`.

5. **Point cloud for costmap**: FAST_LIO publishes `/cloud_registered` and `/cloud_registered_body`. dimos costmap builder needs one of these for obstacle detection.

### ARISE-SLAM Investigation

6. **Evaluate ARISE-SLAM vs FAST_LIO**: Per the epic, ARISE-SLAM is the target SLAM system. Need to build and test ARISE-SLAM with the same bridge infrastructure.

7. **ARISE-SLAM field requirements**: Check if ARISE-SLAM needs different PointCloud2 field layout or lidar_type configuration than FAST_LIO.

---

## Key Files

| File | Location | Purpose |
|------|----------|---------|
| drdds_recv.cpp | `dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/drdds_recv.cpp` | drdds subscriber → SHM writer |
| ros2_pub.cpp | `dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/ros2_pub.cpp` | SHM reader → ROS2 publisher |
| shm_transport.h | `dimos/robot/deeprobotics/m20/docker/drdds_bridge/include/shm_transport.h` | Lock-free SPSC ring buffer |
| deploy.sh | `dimos/robot/deeprobotics/m20/docker/deploy.sh` | bridge-build/start/stop/logs commands |
| build_host.sh | `dimos/robot/deeprobotics/m20/docker/drdds_bridge/build_host.sh` | Builds drdds_recv on NOS host |
| robosense.yaml | (in container) `/ros2_ws/src/fast_lio/config/robosense.yaml` | FAST_LIO config |
| drqos.xml | (on NOS) `/opt/drdds/dr_qos/drqos.xml` | drdds QoS profiles |
| fastdds.xml | (on NOS) `/opt/robot/fastdds.xml` | System FastDDS transport config |
| plan.md | `plans/m20-rosnav-migration/05-drdds-bridge/plan.md` | Original bridge implementation plan |
