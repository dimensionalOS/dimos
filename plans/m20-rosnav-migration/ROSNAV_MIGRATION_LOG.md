# M20 ROSNav Migration Log

**Goal**: Replace M20's proprietary drdds-based navigation stack with FAST_LIO (ROS2 Humble) + dimos ROSNav, enabling autonomous navigation on the M20 quadruped.

**Date started**: 2026-03-12
**Last updated**: 2026-03-22
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

## Finding #12: FastDDS SHM Segment Too Small — Root Cause of Frame Drops (2026-03-19)

**This was the root cause of the ~20% frame drop issue.**

FastDDS 2.6 default SHM `segment_size` is **512KB**. Our lidar messages are **~3MB**. Messages can't fit in SHM, so FastDDS falls back to **UDP transport**, fragmenting each 3MB message into ~48 UDP datagrams. With BEST_EFFORT QoS, losing **any single fragment kills the entire message**. At 10Hz × 30MB/s, socket buffer overflow causes ~20% fragment loss.

**Fix**: Set `FASTRTPS_DEFAULT_PROFILES_FILE` to XML profiles with large SHM segments on BOTH sides:

Publisher (`ros2_pub_fastdds.xml`):
```xml
<transport_descriptor>
    <transport_id>shm_large</transport_id>
    <type>SHM</type>
    <segment_size>16777216</segment_size>       <!-- 16MB -->
    <maxMessageSize>4194304</maxMessageSize>    <!-- 4MB -->
</transport_descriptor>
```

Subscriber (container `fastdds.xml` for FAST_LIO):
```xml
<transport_descriptor>
    <transport_id>shm_large</transport_id>
    <type>SHM</type>
    <segment_size>33554432</segment_size>       <!-- 32MB -->
    <maxMessageSize>4194304</maxMessageSize>    <!-- 4MB -->
</transport_descriptor>
```

**XML syntax gotcha**: FastDDS 2.6 uses `<maxMessageSize>` (camelCase). FastDDS 2.14 uses `<max_message_size>` (snake_case). Using the wrong case causes silent parse errors.

**Result**: 10Hz bridge rate, 10Hz FAST_LIO odometry. Zero frame drops.

---

## Finding #13: pkill -f Kills SSH Sessions (2026-03-18)

Running `pkill -f drdds_recv` over SSH kills the SSH session itself because the SSH command line contains "drdds_recv" as an argument and `pkill -f` matches the full command line of ALL processes including SSH.

**Workaround**: Use `killall drdds_recv` or `pgrep -x drdds_recv | xargs kill`.

---

## Verified Working Pipeline (2026-03-19, final)

| Component | Process | Host | Rate | Status |
|-----------|---------|------|------|--------|
| Lidar multicast | RSAIRY×2 | hardware | 10Hz | Working |
| rsdriver | rslidar | NOS (root) | 10Hz pub | Working |
| yesense | yesense_driver | NOS (root) | 200Hz pub | Working (ROS2-readable directly) |
| drdds_recv | drdds_recv | NOS (root) | 10Hz lidar | Working (lidar only, IMU removed) |
| ros2_pub | ros2_pub | container | **10Hz** lidar | Working (**zero frame drops**) |
| /IMU | yesense → FAST_LIO | direct | 200Hz | Working (no bridge needed) |
| FAST_LIO | fastlio_mapping | container | **10Hz** odometry | Working |

**Data rates measured**:
- drdds_recv: 115K pts/frame, 3.0 MB/frame at 10Hz
- ros2_pub: /bridge/LIDAR_POINTS at **9.99Hz** (zero loss)
- FAST_LIO: /Odometry at **9.99Hz**

**Resource usage**:
- drdds_recv: minimal CPU (callback-driven)
- ros2_pub: 12.5% CPU (semaphore-driven, no polling)
- FAST_LIO: 25-79% CPU on ARM64

**Performance optimization journey**:

| Change | Bridge Rate | Key Insight |
|--------|------------|-------------|
| Initial (RELIABLE QoS, 4 slots) | 6 Hz | ACK blocking |
| + BEST_EFFORT QoS | 8 Hz | Eliminated ACK waits |
| + 16 SHM ring buffer slots | 8 Hz | More headroom |
| + Dedicated poll thread | 8 Hz | Bypassed executor scheduling |
| + Semaphore notification | 8 Hz | Eliminated 1kHz CPU polling |
| + Timing instrumentation | 8 Hz | Revealed publish() is only 570us! |
| + spin() → sleep() | 8 Hz | Freed a wasted CPU core |
| **+ 16MB FastDDS SHM segment** | **10 Hz** | **Eliminated UDP fragmentation** |

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

## Finding #15: publish() is Fast — UDP Fragmentation Was the Real Bottleneck (2026-03-19)

Timing instrumentation revealed `publish()` takes only **~570us** in steady state (first frame: 10ms from cold caches). The ~80ms we assumed was the FastDDS CDR serialization overhead was actually **UDP fragmentation + socket buffer contention** — not serialization.

```
[drdds_bridge] lidar #101 copy=938us pub=818us data=3104842B pts=119417
[drdds_bridge] lidar #301 copy=557us pub=556us data=1994330B pts=76705
[drdds_bridge] lidar #501 copy=2669us pub=554us data=1992198B pts=76623
```

**Lesson**: Always instrument before optimizing. We spent time on thread architectures, poll rates, and queue designs when the actual fix was a 2-line XML config change.

---

## Finding #16: Yesense IMU Is Directly ROS2-Readable (2026-03-19)

Verified by stopping drdds_recv entirely — `/IMU` from yesense continues at 200Hz inside the container. Yesense's drdds publisher uses a transport configuration that IS compatible with ROS2's rmw_fastrtps (unlike rsdriver which is SHM-only and invisible).

This allowed removing all IMU code from the bridge (drdds_recv and ros2_pub), simplifying both significantly. FAST_LIO subscribes to `/IMU` directly from yesense.

---

## Finding #17: FastDDS 2.6 vs 2.14 XML Syntax Differences (2026-03-19)

FastDDS XML element naming changed between versions:

| Element | FastDDS 2.6 (Humble) | FastDDS 2.14 (drdds host) |
|---------|---------------------|--------------------------|
| Max message size | `<maxMessageSize>` | `<max_message_size>` |
| SHM transport type | `SHM` | `SHM` |
| Segment size | `<segment_size>` | `<segment_size>` |

Using the wrong case causes silent parse errors — the element is ignored and default values are used. This caused our first SHM XML attempt to fail without obvious error.

---

## Finding #18: Nav Container Rebuilt with All Fixes Baked In (2026-03-19)

Rebuilt `ghcr.io/aphexcx/m20-nav:latest` with all bridge fixes:
- FAST_LIO config: `lid_topic: /bridge/LIDAR_POINTS`, `lidar_type: 5`
- fastdds.xml: 32MB SHM segment, 4MB `maxMessageSize` (camelCase), transport_descriptors BEFORE participant (FastDDS 2.6 parses sequentially)
- `ros2_pub_fastdds.xml` copied into container at `/ros2_ws/src/drdds_bridge/config/`
- ros2_pub built with semaphore notification, hardcoded RSAIRY fields, dedicated poll thread

**Important**: `transport_descriptors` block MUST appear before `participant` in the XML — FastDDS 2.6 errors on forward references to transport IDs.

---

## Finding #19: ROSNav Wired to FAST_LIO Topics (2026-03-19)

Connected the dimos navigation pipeline to FAST_LIO:
- `ros_registered_scan` transport: `/registered_scan` → `/cloud_registered` (FAST_LIO's global-frame point cloud)
- Added `ros_odometry: In[Odometry]` port subscribed to FAST_LIO `/Odometry`
- Extracts `PoseStamped` and publishes on `odom: Out[PoseStamped]` for websocket_vis
- Blueprint `m20_rosnav.py` updated with odom LCM transport

Full pipeline:
```
FAST_LIO /cloud_registered → ROSNav → VoxelGridMapper → CostMapper
FAST_LIO /Odometry → ROSNav → odom → websocket_vis
```

---

## Finding #20: Merged jeff/fix/rosnav3 — ROSNav Restructured (2026-03-20)

Merged Jeff's `jeff/fix/rosnav3` branch (586 files changed). Key changes:
- `rosnav.py` → `dimos/navigation/rosnav/rosnav_module.py` (package restructure)
- Docker module init + RPC timeout fixes
- Multi-arch Dockerfile (amd64: desktop-full, arm64: ros-base)
- CI cleanup, websocket_vis updates

15 merge conflicts resolved. All non-M20 files accepted Jeff's version. Dockerfile kept our SHM config (16MB segment) while taking Jeff's multi-arch base images.

Ported our topic remaps to the new module location:
- `/registered_scan` → `/cloud_registered` (FAST_LIO's global-frame scan)
- `/state_estimation` → `/Odometry` (FAST_LIO odometry output)

Added missing `__init__.py` to `dimos/navigation/rosnav/` (Jeff's merge didn't include it, causing ImportError).

---

## Finding #21: time Field Removal Was a Misdiagnosis (2026-03-20)

The per-point `time` field was removed from ros2_pub to "fix" FAST_LIO's "lidar loop back" error. This was wrong:

- FAST_LIO with `lidar_type: 5` reads only x,y,z,intensity via `pcl::fromROSMsg` — it **completely ignores** extra fields like `time` and `ring`
- The actual fix was restarting yesense (IMU) — FAST_LIO needs IMU data during init
- The `time` field removal happened in the same deploy cycle, so the fix was incorrectly attributed to it

Re-added all 6 fields (x, y, z, intensity, ring, time). Both FAST_LIO and ARISE-SLAM now work with the same bridge output.

**Lesson**: Don't attribute fixes to coincidental changes. The field removal had zero effect on FAST_LIO — only the restart sequence mattered.

---

## Finding #22: ARISE-SLAM Compatibility Assessment (2026-03-20)

ARISE-SLAM (`arise_slam_mid360`) supports three sensor types: `velodyne`, `ouster`, `livox`. No native `robosense` option.

**Velodyne mode** is the best candidate for RSAIRY lidars:
- `PointcloudXYZITR` expects: x(f32), y(f32), z(f32), intensity(f32), time(f32), ring(u16)
- Our bridge output has: x(f32), y(f32), z(f32), intensity(f32), ring(u16), time(f64)
- `pcl::fromROSMsg` maps by **name** not position — field ordering doesn't matter
- time datatype mismatch (f64 vs f32) may need investigation

**Topic mapping** (ARISE-SLAM ↔ FAST_LIO):

| | ARISE-SLAM | FAST_LIO |
|---|---|---|
| Lidar input | `/lidar/scan` | `/bridge/LIDAR_POINTS` |
| IMU input | `/imu/data` | `/IMU` |
| Odom output | `/state_estimation` | `/Odometry` |
| Scan output | `/registered_scan` | `/cloud_registered` |
| scan_line | 4 (Livox) → 32 (RSAIRY) | 32 |
| sensor type | `velodyne` | `lidar_type: 5` |

An M20-specific ARISE-SLAM config with `sensor: "velodyne"`, `scan_line: 32`, and M20 topic names is needed for Phase 2.

---

## Finding #23: Architecture Clarification — dimos Runs Natively, Not in Container (2026-03-22)

**Critical realization**: The `ghcr.io/aphexcx/m20-nos:latest` container (dimos-m20) is the **OLD architecture** where everything ran in a single Docker container. The ROSNav migration target is:

```
NOS Host (native, Python 3.10 via uv):
  dimos (m20_rosnav blueprint)
    ├── M20Connection (UDP velocity + camera)
    ├── VoxelGridMapper
    ├── CostMapper
    ├── WebsocketVisModule
    └── ROSNav (DockerModule → manages dimos-nav container)

dimos-nav Container (ROS2 Humble):
  ├── FAST_LIO → /Odometry, /cloud_registered
  ├── FAR Planner
  ├── base_autonomy
  └── ROSNav bridge (ROS2 ↔ LCM, runs via entrypoint.sh)
```

**Key changes from old architecture**:
- No `dimos-m20` container at all
- dimos runs natively on NOS with uv/Python 3.10
- ROSNav is a DockerModule that manages the dimos-nav container
- NOS currently only has Python 3.8 (Foxy) — needs Python 3.10 via uv

**Jeff confirmed** (2026-03-22): base off `jeff/fix/rosnav3`, specifically mimic `rosnav_module.py` at commit d37401a6. ROSNav as DockerModule is the correct pattern.

---

## Finding #24: Docker Socket Required for DockerModule (2026-03-21)

When testing with the old m20-nos container, the ROSNav DockerModule failed because:
1. `/var/run/docker.sock` was not mounted (fixed in deploy.sh)
2. Docker CLI binary was not installed in the container

These are moot for the native host architecture (Finding #23), but documented for reference. When dimos runs natively on NOS, it has direct access to Docker.

---

## Remaining Work

### Completed

1. ~~**Install systemd service for reboot persistence**~~: DONE.
2. ~~**Rebuild nav container image**~~: DONE. All fixes baked into `ghcr.io/aphexcx/m20-nav:latest`.
3. ~~**Persist container fastdds.xml fix**~~: DONE. Baked into Dockerfile.
4. ~~**Connect dimos to FAST_LIO odometry**~~: DONE. ROSNav subscribes to `/Odometry`.
5. ~~**Point cloud for costmap**~~: DONE. ROSNav subscribes to `/cloud_registered`.
6. ~~**Merge jeff/fix/rosnav3**~~: DONE. 586 files, 15 conflicts resolved.
7. ~~**Port topic remaps to new module**~~: DONE. `/cloud_registered` + `/Odometry`.
8. ~~**Re-add time field to bridge**~~: DONE. All 6 fields preserved.
9. ~~**ARISE-SLAM compatibility assessment**~~: DONE. Velodyne mode likely works with `scan_line: 32`.
10. ~~**Architecture clarification**~~: DONE. dimos runs natively, not in container.

### Phase 1.1: NOS Host Setup (BLOCKED — robot offline)

11. **Install uv on NOS**: `curl -LsSf https://astral.sh/uv/install.sh | sh`
12. **Create Python 3.10 venv**: `uv venv --python 3.10`
13. **Install dimos from source**: `uv pip install -e /path/to/dimos`
14. **Rebuild drdds-ros2-msgs for Python 3.10 ABI**: Required for `/NAV_CMD` publishing.
15. **Add `deploy.sh setup` subcommand**: One-time NOS host setup automation.

### Phase 1.2: M20ROSNavConfig

16. **Fix M20ROSNavConfig inheritance**: Currently extends `ModuleConfig`, should extend `ROSNavConfig` from `rosnav_module.py` for proper DockerModule behavior.
17. **Update blueprint to pass M20 config**: `ros_nav(config=M20ROSNavConfig())` instead of `ros_nav()` with default G1 settings.

### Phase 1.3: End-to-End Test

18. **Run launch_nos.py natively on NOS**: Verify dimos → DockerModule → dimos-nav → FAST_LIO → odometry → costmap.

### ARISE-SLAM (Phase 2)

19. **Create M20 ARISE-SLAM config**: `sensor: "velodyne"`, `scan_line: 32`, M20 topic names.
20. **Test time field compatibility**: Verify f64→f32 conversion works in pcl::fromROSMsg.

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
| ros2_pub_fastdds.xml | `dimos/robot/deeprobotics/m20/docker/drdds_bridge/config/ros2_pub_fastdds.xml` | Publisher SHM profile (16MB segment) |
| fastdds.xml | (in container) `/ros2_ws/config/fastdds.xml` | Subscriber SHM profile (32MB segment, 4MB maxMessageSize) |
| drdds-bridge.service | `dimos/robot/deeprobotics/m20/docker/drdds_bridge/drdds-bridge.service` | Systemd service (starts before rsdriver) |
| plan.md | `plans/m20-rosnav-migration/05-drdds-bridge/plan.md` | Original bridge implementation plan |
| rosnav_module.py | `dimos/navigation/rosnav/rosnav_module.py` | ROSNav module (Jeff's restructured version) |
| m20_rosnav.py | `dimos/robot/deeprobotics/m20/blueprints/rosnav/m20_rosnav.py` | M20 ROSNav blueprint |
| rosnav_docker.py | `dimos/robot/deeprobotics/m20/rosnav_docker.py` | M20ROSNavConfig (needs fix — see #16) |
| launch_nos.py | `dimos/robot/deeprobotics/m20/docker/launch_nos.py` | Host-side launcher for native dimos |
| spec.md | `plans/m20-rosnav-migration/02-spec/spec.md` | Full migration spec (Phase 1.1-1.8) |
