# M20 LiDAR Data Flow Investigation Log

**Goal**: Get dimos running on Mac receiving all topics (ODOM, ALIGNED_POINTS, IMU, MOTION_INFO, tf) via mac_bridge so auto exploration can be tested.

**Date**: 2026-02-21
**Dog WiFi IP**: 10.21.41.1 (AOS wlan0)
**SSH**: `ssh user@10.21.41.1` (AOS), `ssh -J user@10.21.41.1 user@10.21.31.104` (GOS), `ssh -J user@10.21.41.1 user@10.21.31.106` (NOS)
**sudo password on all hosts**: `'` (single quote)

---

## Architecture (3-host M20)

| Host | IP (eth, 10.21.31.x) | Role |
|------|----------------------|------|
| AOS  | 10.21.31.103 (eth1), 10.21.33.103 (eth0), 10.21.41.1 (wlan0) | Motor control, camera, lio_perception, rsdriver, yesense |
| GOS  | 10.21.31.104 (eth0), 192.168.0.x (eth2/5G) | ROS2 bridge, mac_bridge, rsdriver, NAT gateway |
| NOS  | 10.21.31.106 (eth1), 10.21.33.106 (eth0) | Localization/SLAM, rsdriver, yesense, handler |

## IMU (Inertial Measurement Unit)

The Yesense IMU is a physical sensor that measures:
- **Accelerometer**: linear acceleration (x, y, z) — detects movement and gravity
- **Gyroscope**: angular velocity (roll, pitch, yaw) — detects rotation
- **Orientation**: fused quaternion (computed on-chip)

It runs at **200Hz** — much faster than LiDAR (10Hz). The perception stack needs IMU because LiDAR alone gives point clouds but can't tell you how the robot moved *between* scans. IMU fills those gaps at 200Hz. Sensor fusion (LIO = Lidar-Inertial Odometry) combines both to produce accurate pose estimates. Without IMU, localization can't fuse poses and lio_perception can't even start.

## Process Reference

### Sensor Drivers

| Process | Binary | Host(s) | Description |
|---|---|---|---|
| **rslidar** | `rslidar` | AOS, GOS, NOS | Receives raw LiDAR data from hardware via multicast UDP (224.10.10.201/202). Publishes `/LIDAR/POINTS` (PointCloud2) via DDS at 10Hz. Each host runs its own instance — 3 LiDAR sensors on the robot. Started by `rsdriver_run.sh` which does `sleep 30` then `taskset -c 4,5,6,7 chrt 45 ./rslidar`. Working dir: `/opt/robot/share/node_driver/bin`. Loads `/opt/robot/fastdds.xml` (SHM + loopback UDP only). |
| **yesense** | `yesense_node` | AOS, NOS | Reads the Yesense IMU hardware over serial (`/dev/ttyS9`, 115200 baud). Publishes `/IMU` (sensor_msgs/Imu, 200Hz), `/IMU_DATA` (drdds/ImuData), `/IMU_DATA_10HZ`, `/IMU_YESENSE` via DDS. Two physical IMU sensors, one per host. Started by `yesense_run.sh`. Working dir: `/opt/robot/share/node_driver/bin`. On AOS: loads fastdds.xml (loopback only). On NOS: does NOT load fastdds.xml (publishes on all interfaces). |

### Perception Stack

| Process | Binary | Host(s) | Description |
|---|---|---|---|
| **localization** | `localization_ddsnode` | NOS | Graph-based SLAM/localization (v3.3.1). Subscribes to `/LIDAR/POINTS` + `/IMU`, fuses them to produce **`/ODOM`** (pose) and likely `/ALIGNED_POINTS` (registered point cloud). Loads a pre-built map from `/var/opt/robot/data/maps/active/` (full_cloud.pcd + occ_grid). Also launches `map_server` as a sibling process. Config: `/opt/robot/share/localization/conf/params.yaml`. `init_pose: [0,0,0]`, `auto_reloc: false`. Started with `chrt 48 taskset -c 4,5,6,7`. Does NOT load fastdds.xml (uses default all-interface transports). **Currently drifting badly after reboot — see Session 3.** |
| **lio_perception** | `lio_ddsnode` | AOS | Lidar-Inertial Odometry (v1.0.11). Subscribes to `/LIDAR/POINTS` + `/IMU_YESENSE`, produces **`/LIO_ODOM`** and `/LIO_ALIGNED_POINTS`. Lighter than NOS localization — no map, real-time odometry only. Runs a UDP sender on port 30100. Started with `chrt 35 taskset -c 4,5,6,7`. Does NOT load fastdds.xml. **Note: `/LIO_ODOM` is not visible via rclpy topic discovery on GOS.** |

### Navigation

| Process | Binary | Host(s) | Description |
|---|---|---|---|
| **handler** | `handler` | NOS | Navigation/motion controller. Subscribes to `/ODOM`, `/ALIGNED_POINTS`, `/IMU`, and point clouds. Runs DWA obstacle avoidance. Publishes `/NAV_CMD` to control the robot. Reports sensor frequencies every 1s (e.g. `IMU=200 Cloud=0 Odom=0`). Config: `/opt/robot/share/handler/conf/params.yaml`. Runs on CPUs 0-3 with SCHED_OTHER (normal scheduling) — the only perception-adjacent process NOT on CPUs 4-7 with real-time scheduling. |
| **planner** | `planner` | NOS | Path planner. Currently **DISABLED** (per user request). |

### Infrastructure

| Process | Binary | Host(s) | Description |
|---|---|---|---|
| **cpunode** | `cpu_node` | AOS, GOS, NOS | System monitor — reports CPU/memory/temperature stats. Not involved in data flow. |
| **rte_checker** | `rte_checker` | NOS | Runtime error checker. Monitors system health. |
| **charge_manager** | `charge_manager` | NOS | Battery/charging management. |
| **reflective_column_node** | `reflective_column_node` | NOS | Detects reflective columns (landmark-based localization aid). |
| **multicast-relay** | `multicast-relay` | NOS | Bridges LiDAR multicast packets between eth0 (10.21.31.x) and eth1 (10.21.33.x) on NOS. |

### Our Code (dimos)

| Process | Binary | Host(s) | Description |
|---|---|---|---|
| **mac_bridge** | `mac_bridge.py` | GOS | rclpy (ROS2 Foxy) subscriber that forwards DDS topics over TCP:9731 to Mac. Subscribes to `/ODOM`, `/ALIGNED_POINTS`, `/IMU`, `/tf`, `/MOTION_INFO`. Runs as `user`. Uses `rclpy.spin_once()` polling in a thread (fixed from `SingleThreadedExecutor.spin()` which didn't work with drdds). Systemd service: `dimos-mac-bridge.service`. |
| **m20-relay** | `relay.main` | GOS | Houdini relay — browser-to-ROS2 bridge for teleop UI. Uses rclpy with `spin_once()` polling. Subscribes to `/IMU`, `/BATTERY_DATA`, `/JOINTS_DATA_10HZ`, `/MOTION_INFO`. Publishes `/JOINTS_CMD`. Runs from `/opt/m20-relay-versions/venv/`. Node IS discoverable. Handles battery/motor telemetry, SDK mode, manual joint control. NOT involved in autonomy data flow. |

### Scheduling Summary

| Process | CPUs | Scheduler | Priority | User |
|---|---|---|---|---|
| rslidar | 4-7 | SCHED_RR | 45 | root |
| yesense | 4-7 | SCHED_RR | 61 | root |
| localization | 4-7 | SCHED_RR | 48 | root |
| lio_perception | 4-7 | SCHED_RR | 35 | root |
| handler | **0-3** | SCHED_OTHER | 0 | root |
| mac_bridge | all | SCHED_OTHER | 0 | **user** |
| m20-relay | all | SCHED_OTHER | 0 | **user** |

All Deep Robotics processes run as root. Sensor/perception processes run on CPUs 4-7 with real-time scheduling (SCHED_RR). Handler is the exception — CPUs 0-3 with normal scheduling. Our processes (mac_bridge, m20-relay) run as `user`. Note: SHM permission mismatch was initially suspected as the GOS root cause but was **disproven** — the Houdini relay runs as user and receives DDS data fine (see Session 2 findings).

## Data Pipeline (expected)

```
LiDAR hardware → multicast UDP (224.10.10.201/202) → rslidar
rslidar → DDS /LIDAR/POINTS → localization/lio_perception
localization → DDS /ODOM, /ALIGNED_POINTS → cross-host DDS → GOS ROS2 topics
yesense → DDS /IMU → localization (+ handler for freq reporting)
GOS ROS2 topics → mac_bridge (TCP:9731) → SSH tunnel → Mac dimos
```

## Symptom

After robot reboot (to fix tilted SLAM), ALL sensor data stopped flowing:
- Handler on NOS: `Cloud=0, Odom=0, IMU=200` (only IMU works)
- Localization on NOS: logs `[Subscription] Subscription matched (1 total)` then nothing
- lio_perception on AOS: logs `[Subscription] Subscription matched (1 total)` then nothing
- ALL ROS2 topics on GOS: 0 Hz (no data on any topic)
- rslidar on all 3 hosts: running, "send success size: ~80KB" at 10Hz

## Completed Fixes (persisted across reboots)

1. **NOS clock fixed** — was stuck at Aug 2024, now chrony synced to NTP
2. **chrony enabled on all 3 hosts** — all clocks within 3 seconds
3. **NOS internet routing** — added default route via GOS (10.21.31.104) in `/etc/netplan/config.yaml`
4. **multicast-relay.service enabled on NOS** — bridges LiDAR multicast eth0↔eth1
5. **planner.service disabled on NOS** — per user request
6. **Notion docs updated** — NOS config, chrony setup, network diagram

## Key Finding #1: Both Perception Processes Stuck

Both localization (NOS) and lio_perception (AOS) get exactly **1 DDS subscription match** and never produce output:

- **NOS localization**: subscribes to `/LIDAR/POINTS` + `/IMU`. Matches 1. Outputs nothing.
- **AOS lio_perception**: subscribes to `/LIDAR/POINTS` + `/IMU_YESENSE`. Matches 1. Outputs nothing.

## Key Finding #2: rslidar Publisher Invisible to Subscribers

From `ros2 topic info /LIDAR/POINTS -v` on each host:

| Perspective | Publishers | Subscribers |
|-------------|-----------|-------------|
| GOS         | 1 (GID 01.0f.9f.ad, GOS's rslidar) | 1 (NOS localization) |
| AOS         | **0** | 3 (lio_perception, localization, handler) |
| NOS         | **0** | 3 (same) |

**rslidar's DDS publisher is NOT visible from AOS or NOS!** Despite "send success" in logs.

## Key Finding #3: rslidar Uses Restricted DDS Transport

Socket analysis on NOS (`ss -ulnp`):

**rslidar** (PID 5039):
- `239.255.0.1:7400` — multicast discovery ✓
- `127.0.0.1:7400` — loopback discovery only
- `127.0.0.1:7430` — **data port on LOOPBACK ONLY**
- `127.0.0.1:7431` — **data port on LOOPBACK ONLY**
- LiDAR multicast ports (224.10.10.x) — raw hardware data

**localization_ddsnode** (PID 4903):
- `239.255.0.1:7400` — multicast discovery ✓
- `10.21.31.106:7400`, `10.21.33.106:7400`, `127.0.0.1:7400` — ALL interfaces
- `10.21.31.106:7422`, `10.21.33.106:7422`, `127.0.0.1:7422` — ALL interfaces
- `10.21.31.106:7423`, `10.21.33.106:7423`, `127.0.0.1:7423` — ALL interfaces

**rslidar binds data ports ONLY on 127.0.0.1.** This means it likely loads `/opt/robot/fastdds.xml` which restricts to `interfaceWhiteList: 127.0.0.1` + SHM.

Localization does NOT load fastdds.xml (default transport, all interfaces).

## Key Finding #4: fastdds.xml Config

All 3 hosts have identical `/opt/robot/fastdds.xml`:
```xml
<interfaceWhiteList>
    <address>127.0.0.1</address>
</interfaceWhiteList>
<useBuiltinTransports>false</useBuiltinTransports>
```
This restricts DDS UDP to loopback + SHM only.

**rslidar loads this file** (proven by loopback-only data ports).
**localization does NOT load this file** (has data ports on all interfaces).

## Key Finding #5: Topic Name Red Herring

The `dds_send_point_cloud_topic: /lidar_points` in rsdriver config is NOT used for DDS publishing. rslidar uses the hardcoded `/LIDAR/POINTS` name. Confirmed by: GOS ROS2 shows `/LIDAR/POINTS` topic (not `/lidar_points`).

## Key Finding #6: All Hosts Run rsdriver

All 3 hosts (AOS, GOS, NOS) independently run rsdriver+rslidar. They all receive the same raw LiDAR multicast from hardware. All publish "send success" independently.

## Key Finding #7: Package Versions Match

All hosts: drddslib 1.1.4 (Nov 11 2025), drdds-ros2-msgs 1.0.4 (Nov 14 2025).
Binary dates: rslidar Nov 25, localization_ddsnode Oct 22, lio_ddsnode Oct 22.

## Key Finding #8: Localization IS Receiving LiDAR — But NOT IMU (BREAKTHROUGH)

The 57MB localization log (`/var/opt/robot/log/2026_0221/localization.2026_0221.log`) reveals that **localization IS actively processing LiDAR data at 10Hz**, contradicting the earlier theory that DDS transport is completely broken.

**Startup sequence** (first instance at 01:15:12):
```
01:15:12.623 — LOCALIZATION version: 3.3.1
01:15:12.625 — Initial pose: (0,0,0), (1,0,0,0) (origin)
01:15:12.626 — Map loaded successfully
01:15:12.671 — [monitor] Input source: LiDAR has new data ← LiDAR arrives 45ms after start
01:15:12.677 — [localization] This frame IMU data too few: 0 ← NO IMU
01:15:12.998 — [monitor] Input source: IMU has new data ← IMU detected once
01:15:13 → forever — IMU data per frame: ALWAYS 0
01:15:15.126 — [monitor] Localization lost ← can't converge without IMU
```

**IMU statistics across entire log**:
- Frames with 0 IMU samples: **25,603**
- Frames with 1 IMU sample: **5** (only at very start of first instance)
- Frames with 2+ IMU samples: **0**
- "IMU has new data input" events: **13** (about 2 per restart, 7 restarts total)
- "IMU data timed out" events: multiple multi-second timeouts

**Localization restarts 7 times** in the log (at 01:15, 01:16, 01:20, 01:24, 01:25, 01:33, 01:39). Each restart: loads map successfully, gets LiDAR within 1s, gets 0 IMU per frame, enters "lost" state, eventually gets restarted.

**Map matching errors**: Every frame has matching error (~0.19) and inlier ratio (~0.50) that exceed thresholds (`matching_error_th: 0.25`, `inlier_ratio_th: 0.95`). Without IMU motion prediction, the scan-to-map matching can't converge.

**Config** (`/opt/robot/share/localization/conf/params.yaml`):
- `input_lidar_topic: "/LIDAR/POINTS"` — receiving data ✓
- `input_imu_topic: "/IMU"` — receiving 0 data ✗
- `lidar_use_system_time: false` — uses hardware timestamps
- `imu_use_system_time: false` — uses hardware timestamps
- Map: `/var/opt/robot/data/maps/active/` (252×358 @ 0.1m/cell, dated 2025-11-06)

## Key Finding #9: Handler Gets IMU=200 But Localization Gets 0 — Same Host!

Both handler (PID 4982) and localization (PID 4903) run on NOS. Both subscribe to `/IMU` using `DrDDSSubscriber<sensor_msgs::msg::ImuPubSubType>` (confirmed via binary strings). Yesense (PID 4828) publishes `/IMU` at 200Hz on NOS.

| Consumer | Topic | Type | Result |
|----------|-------|------|--------|
| handler | /IMU | sensor_msgs::msg::Imu | **200 Hz** ✓ |
| localization | /IMU | sensor_msgs::msg::Imu | **0 per frame** ✗ |

All three processes share SHM segments:
- `fastrtps_c7269f2270249d40` — yesense owns, both localization and handler map it
- `fastrtps_022d9bc63f54a732` — localization owns, yesense maps it
- `fastrtps_58ec3ee7b8f9ef06` — handler owns, yesense maps it

SHM infrastructure appears correctly connected. The failure is specific to yesense→localization IMU delivery.

**Transport config comparison on NOS:**

| Process | fastdds.xml? | Data port interfaces | SHM |
|---------|-------------|---------------------|-----|
| rslidar | YES | 127.0.0.1 only | ✓ |
| yesense | NO | ALL (10.21.31.106, 10.21.33.106, 127.0.0.1) | ✓ |
| localization | NO | ALL (same as yesense) | ✓ |
| handler | NO (partial?) | 10.21.33.106 + 127.0.0.1 | ✓ |

Yesense and localization have IDENTICAL transport configurations (default FastDDS, all interfaces). Yet data doesn't flow between them for IMU.

**Paradox**: rslidar→localization works (LiDAR at 10Hz) DESPITE rslidar having a DIFFERENT transport config (loopback-only). But yesense→localization fails DESPITE having the SAME transport config.

## Key Finding #10: AOS lio_perception Has Same Pattern

AOS yesense (PID 1676) has **loopback-only data ports** (127.0.0.1:7414, 7415) — it DOES load fastdds.xml, unlike NOS yesense.

AOS lio_perception (PID 14827) uses 10.21.33.103 + 127.0.0.1 (not all interfaces).

AOS lio_perception log has only 15 lines (3 initializations, no errors, no IMU warnings). This means either it doesn't log verbosely, or it's not receiving any data at all.

**AOS lio_perception was producing output yesterday** — confirmed by lio_perception log existing for 2026-02-20 with clean init messages and user seeing aligned points in Rerun.

## Key Finding #11: ALL ROS2 Topics on GOS Show 0 Hz

```
/IMU: 0 Hz           /LIDAR/POINTS: 0 Hz      /ALIGNED_POINTS: 0 Hz
/LIO_ALIGNED_POINTS: 0 Hz   /LIO_ODOM: 0 Hz   /ODOM: 0 Hz
/MOTION_INFO: 0 Hz    /tf: 0 Hz               /NAV_CMD: 0 Hz
```

DDS **discovery** works (topics visible, publishers enumerated), but **data delivery** fails across the board.

From GOS `ros2 topic info`:
- `/IMU`: 1 publisher (NOS yesense, GID 01.0f.67.42.dc.12), 3 subscribers
- `/LIDAR/POINTS`: 1 publisher (GOS rslidar, GID 01.0f.9f.ad.c5.09), 1 subscriber

Even GOS's OWN rslidar publisher can't deliver data to ROS2 subscribers on the same host.

## Key Finding #12: No NOS Localization Logs From Previous Days

```
$ find /var/opt/robot/log -name 'localization*' -type f
/var/opt/robot/log/2026_0221/localization.2026_0221.log  ← today only
```

NOS had no localization logs before today. The NOS clock was stuck at Aug 2024, and the 2024_* log directories only contain yesense logs.

**Implication**: NOS localization may have NEVER been working. The aligned points the user saw yesterday likely came from AOS lio_perception (`/LIO_ALIGNED_POINTS`), not NOS localization (`/ALIGNED_POINTS`).

## Key Finding #13: Real-Time Scheduling May Starve DDS Threads

Localization runs with `chrt 48 taskset -c 4,5,6,7` (SCHED_RR priority 48, pinned to cores 4-7). This applies to ALL threads including DDS receive threads.

The computation-heavy point cloud matching may monopolize CPU time on the 4 pinned cores, preventing DDS callback threads from firing for IMU delivery. LiDAR may work via a different mechanism (DataSharing/polling) that doesn't depend on callback threads.

Handler runs WITHOUT chrt/taskset — its DDS threads have normal scheduling and receive IMU at 200Hz.

## Updated Working Theory

The original "DDS transport mismatch" theory was **partially wrong**. The real picture:

1. **LiDAR delivery works** — rslidar→localization via SHM on same host ✓
2. **IMU delivery fails** — yesense→localization fails despite matching transport config ✗
3. **IMU delivery to handler works** — yesense→handler at 200Hz ✓
4. **Cross-host DDS data delivery fails** — all topics show 0 Hz from GOS ✗
5. **DDS discovery works everywhere** — topics and endpoints are visible ✓

Most likely root cause combination:
- **NOS localization IMU failure**: Real-time scheduling (`chrt 48`) starving DDS IMU callback thread, OR a DDS subscription matching bug specific to the localization↔yesense pair
- **GOS 0 Hz everywhere**: DDS transport negotiation failure between drdds publishers (SHM/loopback) and ROS2 rmw_fastrtps subscribers (default transport) — discovery succeeds via multicast but unicast data delivery fails
- **AOS lio_perception stuck**: AOS yesense is loopback-only (loads fastdds.xml), similar pattern to NOS

## What Changed Between Yesterday and Today?

**CRITICAL CONTEXT**: dimos WAS working yesterday (2026-02-20). The user was receiving aligned points via the mac_bridge and had the robot autonomously exploring (frontier exploration), though with costmap issues (ceiling points registering as obstacles at z≈4.08m). The Rerun 3D view showed a populated voxel map with the robot navigating.

**This means the mac_bridge code is proven — it DID receive DDS data and forward it to Mac successfully.** The issue is that the reboot broke the DDS state, not a fundamental code or transport problem.

The reboot changed:
1. New SHM segment names (new PIDs → new segment hashes)
2. New DDS GUIDs for all participants
3. Service startup order may differ (mac_bridge may have started before DDS publishers were ready)
4. NOS clock now correct (was wrong yesterday — chrony fix)
5. DDS discovery state reset — previous endpoint matching lost

**Yesterday's working path was**: mac_bridge on GOS subscribed to DDS topics, received data, forwarded to Mac over TCP. The exact topics received (whether from NOS localization or AOS lio_perception) worked. The reboot destroyed the DDS discovery state that enabled this.

**Most likely fix**: Simply restarting mac_bridge AFTER all DR services are fully up (especially after rslidar's 30-second sleep) should re-trigger DDS discovery and restore data flow. The mac_bridge may have started too early in the boot sequence, before rslidar/yesense endpoints were advertised.

## Key Finding #14: SMOKING GUN — GOS SHM Permission Mismatch

**This is the root cause of the GOS 0 Hz problem.**

On GOS, rslidar runs as **root** but mac_bridge runs as **user** (per systemd `User=user`):

```
rslidar (root) → SHM segments: rw-r--r-- (644) root:root
mac_bridge (user) → SHM segments: rw-r--r-- (644) user:user
```

**They share ZERO SHM segments** (verified via lsof):
- rslidar maps: `fastrtps_53ca2651a9a5274f` (52MB, root:root), `port7419` (root:root)
- mac_bridge maps: `fastrtps_7128dd968c063cf5` (557KB, user:user), `port7412`, `port7413` (user:user)

FastDDS transport matching selects SHM preferentially when both endpoints support it. Both rslidar (explicit SHM in fastdds.xml) and mac_bridge (rclpy default includes SHM) support SHM. FastDDS selects SHM → but mac_bridge (user) can't WRITE to rslidar's (root) SHM segments (644 permissions, others=r-- only) → data delivery fails silently. **FastDDS does NOT fall back to UDP after SHM is selected.**

UDP fallback would also fail independently: rslidar's UDP is bound to `127.0.0.1` only (from fastdds.xml). mac_bridge (rclpy defaults) may not advertise 127.0.0.1 as a unicast locator.

**Proof — on NOS, SHM works** because both rslidar and localization run as root:
```
rslidar (root, PID 5039) → SHM segments root:root 644
localization (root, PID 4903) → both root → can write to rslidar's segments → LiDAR flows ✓
```

## Key Finding #15: Process Scheduling Summary

| Process | Host | CPUs | Sched | Prio | User |
|---------|------|------|-------|------|------|
| rslidar | GOS | 4-7 | SCHED_RR | 45 | root |
| mac_bridge | GOS | all | SCHED_OTHER | 0 | **user** |
| m20-relay | GOS | all | SCHED_OTHER | 0 | **user** |
| rslidar | NOS | 4-7 | SCHED_RR | 45 | root |
| yesense | NOS | 4-7 | SCHED_RR | 61 | root |
| localization | NOS | 4-7 | SCHED_RR | 48 | root |
| handler | NOS | 0-3 | SCHED_OTHER | 0 | root |
| rslidar | AOS | 4-7 | SCHED_RR | 45 | root |
| yesense | AOS | 4-7 | SCHED_RR | 61 | root |
| lio_perception | AOS | 4-7 | SCHED_RR | 35 | root |

All DR robot processes run as root. Only OUR processes (mac_bridge, m20-relay) run as user.

## Key Finding #16: GOS fastdds.xml Config

Location: `/opt/robot/fastdds.xml` (same on all hosts)

```xml
<interfaceWhiteList>
    <address>127.0.0.1</address>
</interfaceWhiteList>
<segment_size>500000000</segment_size>
<useBuiltinTransports>false</useBuiltinTransports>
```

drdds processes load this (loopback + SHM only). rclpy processes do NOT load this (use default all-interface UDP + SHM).

## Key Finding #17: mac_bridge DDS Environment

No `FASTRTPS_DEFAULT_PROFILES_FILE`, no `RMW_IMPLEMENTATION` set. Only:
```
LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib
PYTHONPATH=...rclpy/drdds site-packages...
ROS_DOMAIN_ID=0
```

## Key Finding #18: GOS Firewall Clear

Tailscale installed but only filters Tailscale CGNAT traffic (100.64.0.0/10). INPUT/OUTPUT policies are ACCEPT. No firewall blocking DDS UDP traffic.

---

## ROOT CAUSE ANALYSIS

### Problem 1: GOS 0 Hz (all topics) — REVISED: NOT SHM permissions
**Original theory**: FastDDS SHM permission mismatch between root (rslidar/drdds) and user (mac_bridge/rclpy). **DISPROVEN** — the Houdini relay runs as user and receives DDS data fine. Running mac_bridge as root also didn't help.

**Revised root cause**: Our `dimos_mac_bridge` rclpy node is NOT being discovered by other DDS participants (doesn't appear in `ros2 node list`). The Houdini relay's `m20_relay` node IS discovered using identical rclpy on the same host. Difference may be in Python interpreter (venv vs system), spin model (spin_once polling vs executor.spin() in thread), or other environmental factors.

### Problem 2: NOS localization "lost" (no ODOM/ALIGNED_POINTS output) — PARTIALLY SOLVED
**Root cause**: DDS IS delivering IMU to localization (monitor reports "IMU has new data input"). But localization's ALGORITHM counts 0 IMU per LiDAR frame. The issue is in the APPLICATION layer, not DDS transport.

**Evidence**:
- SHM infrastructure correctly connected: yesense segment `c7269f2270249d40` is mapped by both localization AND handler
- All processes run as root on NOS — no SHM permission issue
- DDS monitor in localization reports "IMU has new data input" at each restart
- Despite this, algorithm reports "imu数据量过少: 0" (too few IMU: 0) for every frame
- Handler on same host receives IMU at 200Hz from same yesense publisher

**Timeline (restart 2)**:
- 01:16:22.958 — Localization reinitializes
- 01:16:23.268 — Monitor: "IMU has new data input" (DDS delivering ✓)
- 01:16:45.263 — First LiDAR frame (22s after restart, rsdriver has `sleep 30` in startup script)
- 01:16:45.266 — "imu数据量过少: 0" (algorithm sees 0 IMU in frame ✗)
- From here: permanently 0 IMU per frame

**Most likely cause**: Timestamp mismatch between IMU and LiDAR data within localization's processing pipeline. Localization collects IMU samples whose timestamps fall within the current LiDAR frame's time window. If IMU timestamps use a different time base (e.g., Unix time vs monotonic/uptime), no samples align with the LiDAR window. LiDAR frame timestamps in the log are ~2587-5890 (looks like uptime seconds). IMU may use a different format.

**This is a bug/limitation in the proprietary localization binary** — we can't fix it directly. The NOS clock was wrong (Aug 2024) until we fixed it with chrony. The timestamp mismatch may be related to the clock correction.

**NOT the cause**: CPU/RT scheduling starvation (disproven — IMU drops to 0 twelve seconds BEFORE LiDAR processing even starts)

### Problem 3: AOS lio_perception stuck — SOLVED
**Root cause**: AOS yesense and lio_perception have **ZERO shared SHM segments**. DDS SHM negotiation failed between them.

**Evidence**:
- yesense (PID 1676) owns: `fastrtps_923182e0b2958c22` (52MB), port 7415
- lio_perception (PID 14827) maps: `f86a3f94eb6a46a2`, `71b8a1781727ece5`, `8ba3fa7311563254` — NONE from yesense
- rslidar ↔ lio_perception: share segments `f86a3f94eb6a46a2` and `71b8a1781727ece5` → LiDAR SHM works
- yesense ↔ lio_perception: share NOTHING → IMU doesn't flow → LIO can't run

**Contrast with NOS**: NOS yesense DOES share SHM with localization (segment `c7269f2270249d40` mapped by both). But NOS yesense uses DEFAULT transports (all interfaces), while AOS yesense loads fastdds.xml (127.0.0.1 only).

**Why rslidar↔lio_perception SHM works but yesense↔lio_perception doesn't**: Both rslidar and yesense load fastdds.xml. But rslidar SHM works with lio_perception while yesense SHM doesn't. The difference may be in DDS discovery timing, endpoint QoS matching, or a FastDDS bug affecting small-message DataSharing vs large-message SHM transport negotiation.

**lio_perception log**: Only initialization, zero processing output (unlike NOS localization which actively processes LiDAR).
Without IMU, LIO (Lidar-Inertial Odometry) cannot start at all — it needs both sensors from the first frame.

---

## PROPOSED FIXES (ordered by priority)

### Fix 1: GOS same-host data flow (SIMPLEST, highest confidence)
**Change**: Run mac_bridge as root — edit `dimos-mac-bridge.service`, change `User=user` → `User=root`.
**Why it works**: Both rslidar and mac_bridge are root → SHM segments accessible → FastDDS SHM delivers data.
**Risk**: Low. Only affects mac_bridge process. Security concern is minimal (mac_bridge already has full network access).
**Fixes**: GOS rslidar → mac_bridge for `/LIDAR/POINTS` ✓. Also fixes cross-host reception since rclpy default transport includes all-interface UDP.

### Fix 2: NOS localization IMU (UNCERTAIN — may be a proprietary bug)
**What we know**: DDS delivers IMU, but localization's algorithm discards it. Likely a timestamp mismatch in the binary.
**Possible actions**:
- a) Restart all NOS services (`systemctl restart localization handler yesense rsdriver`) — fresh DDS connections might fix matching
- b) Check if `imu_use_system_time: true` in localization config fixes the timestamp alignment
- c) Contact Deep Robotics support — this appears to be a bug in their localization binary
- d) **Bypass NOS localization entirely** — use AOS lio_perception (which was the working path yesterday)
**Risk**: Options a/b are low risk (reversible). Option d is the pragmatic approach.

### Fix 3: AOS lio_perception (ALTERNATIVE PATH — restart services)
**Change**: Restart ALL AOS sensor services in correct order: `systemctl restart yesense rsdriver lio_perception`
**Why**: yesense↔lio_perception SHM negotiation failed. Restarting may re-trigger proper DDS discovery. AOS lio_perception was yesterday's working path for `/LIO_ALIGNED_POINTS` and `/LIO_ODOM`.
**Risk**: Low. Service restart is standard and all services auto-recover.
**Note**: If restart doesn't fix SHM negotiation, may need to set `FASTRTPS_DEFAULT_PROFILES_FILE` for lio_perception to use the same fastdds.xml as yesense, ensuring compatible transport config.

### Fix 4: fastdds.xml interface whitelist (BROADER FIX)
**Change**: Add `10.21.31.x` to interfaceWhiteList on all hosts:
```xml
<interfaceWhiteList>
    <address>127.0.0.1</address>
    <address>10.21.31.104</address>  <!-- GOS -->
</interfaceWhiteList>
```
**Why**: Enables cross-host DDS data delivery even for drdds processes.
**Risk**: Low-medium. Changes DDS transport config for all DR robot processes.

### Fix 5: Bypass rclpy (NUCLEAR OPTION)
**Change**: Rewrite mac_bridge.py to use drdds Python bindings directly instead of rclpy.
**Why**: Eliminates all rclpy↔drdds transport negotiation issues.
**Risk**: High effort. Only needed if Fix 1 doesn't solve it.

## Service Status Summary (after all fixes)

| Service | AOS | GOS | NOS |
|---------|-----|-----|-----|
| rsdriver | ✓ send_success | ✓ send_success | ✓ send_success |
| yesense | ✓ running | — | ✓ 200Hz (handler sees it) |
| localization | — | — | STUCK (1 match, no output) |
| lio_perception | STUCK (1 match, no output) | — | — |
| handler | — | — | Cloud=0 Odom=0 IMU=200 |
| cpunode | ✓ running | ✓ running | ✓ running |
| mac_bridge | — | ✓ running (no data) | — |
| m20_relay | — | ✓ running | — |
| multicast-relay | — | — | ✓ enabled |
| chrony | ✓ synced | ✓ synced | ✓ synced |
| planner | — | — | DISABLED |

## Network/DDS Config

- FastDDS multicast group 239.255.0.1 joined on all hosts, all relevant interfaces
- DDS domain 0 (port 7400 = default)
- `ROS_LOCALHOST_ONLY=0` on GOS
- `/opt/robot/fastdds.xml` identical on all hosts (127.0.0.1 + SHM only)

---

## Session 2: Fix Application Results (2026-02-21 ~17:09 CST)

Applied Fix 1 and Fix 2. Results invalidated the GOS SHM permission theory.

### Fix 1 Applied: mac_bridge → root (REVERTED — didn't help)

Changed `User=user` → `User=root` in `dimos-mac-bridge.service`, restarted. Service came up as root (PID 8486).

**Result**: No improvement. `dimos_mac_bridge` node still NOT visible in `ros2 node list`. mac_bridge creates its own SHM segment (`fastrtps_c7acc787efa686d4`, 557KB, root:root) but does NOT map rslidar's segments (53ca, 5b6a, 6a6c — 52MB each, root:root).

Reverted to `User=user` since the relay works as user.

### Fix 2 Applied: AOS service restart (DONE)

```bash
sudo systemctl restart yesense rsdriver lio_perception
```

All three services now active. Have not yet verified if lio_perception is producing output (need to wait 30s+ for rsdriver sleep, then check logs).

### Key Finding #19: `ros2 topic list` Discovers Everything, `ros2 topic hz` Gets Zero

```
$ ros2 topic list   → 40+ topics visible (all hosts)
$ ros2 topic hz /ALIGNED_POINTS → 0 (local GOS topic from rslidar!)
$ ros2 topic hz /IMU → 0 (NOS topic)
$ ros2 topic hz /ODOM → 0 (NOS topic)
$ ros2 topic hz /MOTION_INFO → 0
```

Even with `FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml` set — still 0 data. DDS discovery protocol (participant/endpoint discovery) works across drdds↔rclpy. But actual data delivery does not.

This applies to ALL rclpy tools including `ros2 topic hz` (which creates a temporary subscriber). Not specific to our mac_bridge.

### Key Finding #20: Topic Endpoint Details

From `ros2 topic info -v`:

| Topic | Publisher | Type | Subscribers |
|---|---|---|---|
| `/ALIGNED_POINTS` | `_CREATED_BY_BARE_DDS_APP_` (rslidar GOS) | sensor_msgs/PointCloud2 | **0** — our mac_bridge NOT subscribed |
| `/ODOM` | `_CREATED_BY_BARE_DDS_APP_` (handler NOS) | nav_msgs/Odometry | 1 (drdds process, not us) |
| `/IMU` | `_CREATED_BY_BARE_DDS_APP_` (yesense NOS) | sensor_msgs/Imu | 2 (1 drdds + 1 `m20_relay`) |

All drdds publishers appear as `_CREATED_BY_BARE_DDS_APP_` with no ROS2 namespace. QoS: RELIABLE, VOLATILE, AUTOMATIC liveliness. These should be compatible with rclpy default QoS.

### Key Finding #21: `dimos_mac_bridge` Node Not Discoverable (CRITICAL)

```
$ ros2 node list
/m20_relay          ← Houdini relay only; our node MISSING
```

Our mac_bridge (PID 8486/10102) IS running, logs "ROS adapter started: subscriptions active", but the DDS participant is NOT discovered by other nodes. This explains why data doesn't flow — the node is invisible.

### Key Finding #22: Houdini Relay Uses rclpy AND IT WORKS

The Houdini relay (`/opt/m20-relay-versions/venv/bin/python -m relay.main`) is a Python process that also uses rclpy:

```python
# From /opt/m20-relay-versions/v0.2.0/src/relay/ros_bridge.py
import rclpy
rclpy.init()
self._node = rclpy.create_node("m20_relay")
self._node.create_subscription(StdImu, "/IMU", self._std_imu_callback, 10)
```

It subscribes to: `/IMU` (sensor_msgs/Imu), `/BATTERY_DATA`, `/JOINTS_DATA_10HZ`, `/MOTION_INFO`.
It publishes to: `/JOINTS_CMD`.
Service client: `/SDK_MODE`.

**The relay IS visible in `ros2 node list` and presumably receives data.** This proves rclpy↔drdds data flow IS possible on GOS.

### Key Finding #23: FastRTPS Version

Our mac_bridge loads `libfastrtps.so.2.1.4` (from `/opt/ros/foxy/lib/`). drdds uses FastRTPS/FastDDS 2.14 (built into drdds binaries). The relay likely uses the same Foxy FastRTPS since it's in a Python venv with system rclpy.

### Key Finding #24: What Makes the Relay Work but Our Bridge Not?

| Property | Houdini relay (works) | Our mac_bridge (broken) |
|---|---|---|
| User | user | user (after revert) |
| Python | `/opt/m20-relay-versions/venv/bin/python` | `/usr/bin/python3` |
| Spin model | `rclpy.spin_once(node, timeout=0.001)` polling in event loop | `SingleThreadedExecutor.spin()` in daemon thread |
| QoS | Default (integer `10`) | Explicit `QoSProfile(RELIABLE, KEEP_LAST, VOLATILE, depth=10)` |
| Node name | `m20_relay` | `dimos_mac_bridge` |
| PYTHONPATH | venv site-packages | `/opt/ros/foxy/lib/python3.8/site-packages:/opt/drdds/lib/python3.8/site-packages` |
| Discovered? | YES (`ros2 node list`) | **NO** |

**Top hypotheses for why our node isn't discovered**:
1. **Python interpreter difference**: The relay uses a venv Python that might have different rclpy/FastRTPS shared libs linked
2. **SingleThreadedExecutor vs spin_once**: Maybe `spin()` in a daemon thread blocks discovery protocol responses
3. **Environment/rclpy initialization**: The relay may have additional env vars or rclpy config from its venv
4. **Two rclpy.init() in same process space**: Unlikely since they're separate PIDs, but worth checking

### Resolution: spin_once() fix — ALL DATA FLOWING

**Root cause confirmed**: `SingleThreadedExecutor.spin()` in a daemon thread does NOT work with ROS2 Foxy + drdds on GOS. The executor model fails to properly participate in DDS discovery/data exchange. Replacing with `rclpy.spin_once(node, timeout_sec=0.01)` polling in a thread (matching the Houdini relay's working pattern) fixes everything.

**Verification steps that confirmed this**:
1. Minimal `spin_once` test script received IMU and ALIGNED_POINTS immediately
2. `spin_once` in a separate thread also works (200Hz IMU, 10Hz LiDAR confirmed)
3. Full mac_bridge with spin_once fix deployed, TCP client on Mac receives ALL topics:

```
IMU:         155 Hz   ✓ (expected ~177Hz, some loss over TCP acceptable)
LIDAR:       9.1 Hz   ✓ (expected ~10Hz, 722-734 points per frame)
ODOM:        9.1 Hz   ✓
TF:          9.0 Hz   ✓
MOTION_INFO: 18.3 Hz  ✓
```

**Fix applied**: `dimos/robot/deeprobotics/m20/mac_bridge.py` — removed `SingleThreadedExecutor`, replaced `_spin()` with `_spin_loop()` using `rclpy.spin_once()`.

---

## RECOMMENDED ACTION PLAN (updated after Session 2)

**Goal**: Get dimos on Mac receiving ODOM + ALIGNED_POINTS + IMU via mac_bridge for auto exploration testing.

**The SHM permission theory (Fix 1: run as root) was wrong.** The Houdini relay runs as user and receives DDS data fine. The real issue is that our `dimos_mac_bridge` rclpy node is not being discovered by DDS — likely a **DDS startup ordering issue** from the reboot, not a code problem. dimos worked yesterday with the same mac_bridge code.

**Step 0** (GOS, simplest — try restart now that all services are up):
```bash
# mac_bridge may have started before DDS publishers were ready (rslidar has sleep 30)
# Just restart it now that everything is running
ssh -J user@10.21.41.1 user@10.21.31.104
echo "'" | sudo -S systemctl restart dimos-mac-bridge
sleep 5
# Check if node appears now
source /opt/ros/foxy/setup.bash
export PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/opt/drdds/lib/python3.8/site-packages
ros2 node list   # should show /dimos_mac_bridge alongside /m20_relay
ros2 topic hz /ALIGNED_POINTS  # should show >0 Hz
```

**Step 1** (if Step 0 still shows no data — diagnose rclpy):
```bash
# 1a. Check relay's rclpy library path vs system
/opt/m20-relay-versions/venv/bin/python -c "import rclpy; print(rclpy.__file__)"
/usr/bin/python3 -c "import rclpy; print(rclpy.__file__)"

# 1b. Minimal spin_once subscriber test
/usr/bin/python3 -c "
import rclpy
from sensor_msgs.msg import Imu
rclpy.init()
node = rclpy.create_node('test_sub')
got = [False]
def cb(msg): got[0] = True; print('GOT IMU')
node.create_subscription(Imu, '/IMU', cb, 10)
for _ in range(50):
    rclpy.spin_once(node, timeout_sec=0.1)
    if got[0]: break
node.destroy_node()
rclpy.shutdown()
print('Result:', 'DATA' if got[0] else 'NO DATA')
"
```

**Step 2** (GOS — if spin model matters, fix mac_bridge):
Replace `SingleThreadedExecutor.spin()` in a daemon thread with `rclpy.spin_once()` polling (matching the relay's working pattern).

**Step 3** (AOS — verify lio_perception after restart):
AOS services were already restarted in Session 2. Check if lio_perception produces output:
```bash
ssh user@10.21.41.1
journalctl -u lio_perception -n 30 --no-pager
```

**Step 4** (verify on Mac):
Connect from Mac with SSH tunnel, check rerun for data streams.

**Step 5** (if rclpy itself can't receive — bypass with drdds native):
Write mac_bridge using drdds Python bindings directly instead of rclpy. Nuclear option.

**Step 6** (NOS localization — low priority):
Proprietary binary bug. Pragmatic path is using AOS lio_perception output.

### Pre-reboot known issue: costmap ceiling obstacle

When dimos was working (2026-02-20), the 3D voxel map in Rerun showed the ceiling registering as obstacles in the costmap at z≈4.08m. The costmap needs a max height filter to ignore points above robot-relevant height (e.g., z > 2.0m). This is a separate issue to address after data flow is restored.

---

## Session 3: Data Flowing But Odometry Drifting (2026-02-21 ~17:40 CST)

dimos launched successfully on Mac. TCP bridge connected, all modules deployed, Rerun viewer showing data. But odometry is wildly unstable — robot position jumping around even while sitting still.

### Findings

#### Key Finding #25: ODOM Drift Is At The Source, Not The Bridge

Sampled raw `/ODOM` directly on GOS via rclpy (bypassing mac_bridge entirely). **Robot is sitting still (docked/standing) yet ODOM shows massive drift**:

```
ODOM stats (100 samples in 10.0s = 10.0 Hz):
  X: mean=-2.7985 std=0.3402 min=-3.4451 max=-2.4348 range=1.0103
  Y: mean=-1.5891 std=0.7570 min=-2.1766 max=-0.2706 range=1.9060
  Z: mean=-2.2588 std=0.0213 min=-2.2925 max=-2.0983 range=0.1942
  Jumps (>5cm): 30 out of 99 transitions (30.3%)
  Jump magnitudes: mean=0.1154m max=0.3351m
  Total drift (first to last): 1.9683m
```

**30% of consecutive samples have >5cm jumps. 2m total drift in 10 seconds while stationary.** This is NOT a bridge issue — the raw ROS2 topic on GOS already has this drift.

#### Key Finding #26: `/ODOM` Comes From NOS Localization, NOT AOS lio_perception

**This was a critical misconception in earlier analysis.** The two perception systems publish to DIFFERENT topics:

| System | Host | Output Topic | Status |
|---|---|---|---|
| **NOS localization** (`localization_ddsnode`) | NOS (10.21.31.106) | **`/ODOM`** | Running, 28% CPU, produces output but drifts |
| **AOS lio_perception** (`lio_ddsnode`) | AOS (10.21.31.103) | **`/LIO_ODOM`** | Running, 14.6% CPU, topic NOT visible via rclpy |

From NOS localization config (`/opt/robot/share/localization/conf/params.yaml`):
```yaml
output_odom_topic: "/ODOM"        # ← This is what mac_bridge subscribes to
```

From AOS lio_perception config (`/opt/robot/share/lio_perception/conf/params.yaml`):
```yaml
output_odom_topic: "/LIO_ODOM"    # ← Different topic, NOT visible in ros2 topic list
```

**`/LIO_ODOM` does NOT appear in `ros2 topic list` on GOS.** Either lio_perception doesn't publish it to DDS properly, or rclpy can't discover it.

#### Key Finding #27: NOS Localization Has Map But May Not Be Localized

NOS localization runs with a pre-built map:
```
/var/opt/robot/data/maps/active/
  full_cloud.pcd      (1.8MB — pre-built point cloud map)
  occ_grid.pgm        (90KB — occupancy grid, 252×358 @ 0.1m/cell)
  occ_grid.yaml       (161B — map metadata)
  blocks/             (directory — chunked map data for fast_icp)
```

Config details:
```yaml
init_pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]  # Starts at origin
auto_reloc: false           # NO automatic relocalization
registration_cov: 0.0001   # Registration covariance
matching_error_th: 0.25     # Match error threshold
inlier_ratio_th: 0.95       # Required inlier ratio (very strict)
fast_icp:
  maximum_iterations: 30
  inner_distance: 0.5
```

**Problem**: After reboot, localization starts at `init_pose: [0,0,0]` (map origin). If the robot's actual position doesn't match the map origin, scan-to-map matching starts from a wrong pose. With `auto_reloc: false`, it can't automatically relocalize. The strict `inlier_ratio_th: 0.95` means poor initial matching → noisy/jumpy pose estimates.

**This was the same pattern identified in Session 1** (Finding #8): localization logs showed "matching_error ~0.19 with inlier_ratio ~0.50" — far below the 0.95 threshold. The scan matching never converges because the initial pose is wrong and there's no relocalization.

#### Key Finding #28: IMU Is Healthy

IMU data on GOS is clean (sampled directly via rclpy):
```
IMU: 474 msgs in 3.0s = 157.9 Hz
  accel=(0.129,-0.103,9.803) |a|=9.804   ← correct gravity magnitude
  gyro=(0.0010,0.0015,-0.0017)           ← near-zero (robot stationary)
```

IMU sensor is working perfectly. The drift is in the SLAM, not the IMU.

#### Key Finding #29: ALIGNED_POINTS Source Uncertain

`/ALIGNED_POINTS` appears in the topic list (10.8 Hz, 310-358 points/frame, frame_id="map", point_step=48). But:
- NOS localization config has NO explicit `/ALIGNED_POINTS` output topic
- AOS lio_perception config outputs `/LIO_ALIGNED_POINTS` (different name, doesn't appear in topic list)

Possible explanations:
1. NOS localization publishes `/ALIGNED_POINTS` internally even though it's not in the yaml config
2. AOS lio_perception remaps `/LIO_ALIGNED_POINTS` → `/ALIGNED_POINTS` at the binary level
3. There's another process producing it

**Point count dropped from ~720 to ~320** compared to earlier TCP bridge tests. The lower count may be due to poor localization → fewer matched/aligned points.

#### Key Finding #30: Full Topic List From GOS

55+ DDS topics visible from GOS (via rclpy). Notable ones:
```
/ODOM                        ← NOS localization (drifting)
/ALIGNED_POINTS              ← Source uncertain (NOS or AOS)
/IMU                         ← NOS yesense (157Hz, healthy)
/LIDAR/POINTS                ← Local GOS rslidar
/MOTION_INFO                 ← AOS motor control
/tf                          ← Transform frames
/FULL_CLOUD_MAP              ← NOS localization (global map)
/LOCATION_STATUS             ← NOS localization health (drdds type, unreadable via rclpy)
/LOCATION_STATUS/MATCHING_ERROR  ← NOS localization quality metric
/LOC_BODY_POINTS             ← NOS localization body-frame points
```

### Root Cause: NOS Localization Not Converged After Reboot

The odometry drift is caused by NOS localization (`localization_ddsnode`) failing to properly localize against its pre-built map after the robot reboot:

1. After reboot, localization starts at `init_pose: [0,0,0]` (map origin)
2. Robot is NOT at map origin → initial scan-to-map matching fails
3. `auto_reloc: false` → no automatic relocalization attempt
4. Without proper localization, ICP matching oscillates between local minima
5. Result: 1-2m drift per 10 seconds, 30% of transitions have >5cm jumps, even while stationary

**This is consistent with Session 1 findings** where localization logs showed matching_error ~0.19 (above 0.25 threshold) with inlier_ratio ~0.50 (far below 0.95 threshold). The SLAM never converges.

**Yesterday it worked because** either:
- NOS localization had been running long enough to converge (the map was built from this environment)
- The robot was positioned near the map origin at startup
- A manual relocalization was triggered at some point

### What about AOS lio_perception as alternative?

AOS lio_perception is simpler (no prior map, pure LiDAR-inertial odometry). It publishes to `/LIO_ODOM` which is currently NOT visible via rclpy. Options:
1. **Make mac_bridge subscribe to `/LIO_ODOM`** instead of/in addition to `/ODOM`
2. **Fix NOS localization** by providing a correct initial pose or enabling auto-relocalization
3. **Use both** — subscribe to both and pick the one with better quality

### Proposed Next Steps

1. **Quick test: Check raw `/LIO_ODOM` data quality on AOS**
   - If lio_perception produces stable odometry, it's a better source than the broken NOS localization
   - Need to check if `/LIO_ODOM` is even visible from GOS over DDS

2. **Fix NOS localization initial pose**
   - Option A: Change `init_pose` in `/opt/robot/share/localization/conf/params.yaml` to the robot's actual location
   - Option B: Set `auto_reloc: true` so it relocates automatically on startup
   - Option C: Publish a manual relocalization pose to `/initialpose` (topic exists in the list)

3. **Subscribe to `/LIO_ODOM` in mac_bridge as fallback**
   - Update mac_bridge.py to subscribe to both `/ODOM` and `/LIO_ODOM`
   - Use whichever produces better data

4. **NOS localization IMU issue** (Finding #8) — this was the deeper bug where localization gets 0 IMU per frame. Even with correct init_pose, localization can't converge without IMU. This is likely the real reason it's drifting — it's doing scan-matching only, no inertial prediction, producing noisy results.

#### Key Finding #31: AOS SHM Fixed After Session 2 Restart — But lio_perception Still Silent

After the AOS service restart in Session 2, the SHM segments are NOW properly shared:

```
lio_perception (PID 135111) ↔ yesense (PID 135029):
  e8f7b26a07c7f7c0 — yesense owns (17:09), lio maps ✓
  b60838afe85f56be — shared (17:09) ✓

lio_perception (PID 135111) ↔ rslidar (PID 135310):
  84efce70cb99c7b9 — lio owns (17:10), rslidar maps ✓
  b60838afe85f56be — shared (17:09) ✓
```

**This fixes Finding #10** (AOS lio↔yesense shared ZERO segments). The Session 2 restart re-triggered DDS discovery and SHM negotiation succeeded. But lio_perception's log (`1576 bytes total`) contains ONLY initialization messages (4 restarts, no processing output):

```
[17:09:57] LIO version: 1.0.11
[17:09:57] 里程计节点完成初始化 (odometry node initialized)
[17:09:57] UDP server listening on port 30100
           ← no further output in 38+ minutes
```

At 11% CPU, lio_perception may be processing data but not logging it (lio_perception has no verbose logging mode that we know of). Or it may be stuck in initialization despite having SHM access. **We cannot determine from logs alone whether `/LIO_ODOM` is being published.**

### Status After Session 3

| Component | Status | Notes |
|---|---|---|
| mac_bridge (GOS→Mac TCP) | **WORKING** | spin_once fix deployed, all 5 topics flowing |
| SSH tunnel (Mac→GOS) | **WORKING** | localhost:9731 → GOS:9731 via AOS |
| dimos on Mac | **RUNNING** | PID 75355, blueprint built, Rerun showing data |
| TCP data flow | **VERIFIED** | IMU 155Hz, LIDAR 9.1Hz, ODOM 9.1Hz, TF 9Hz, MOTION_INFO 18.3Hz |
| Odometry quality | **BROKEN** | 1.97m drift/10s while stationary, source: NOS localization |
| Costmap | AFFECTED | Drift causes costmap to jump around (shows unstable ego) |
| NOS localization | **DRIFTING** | Not converged after reboot, possible IMU integration failure |
| AOS lio_perception | UNKNOWN | Running but `/LIO_ODOM` not visible from GOS via rclpy |
| IMU | **HEALTHY** | 158Hz, correct gravity, near-zero gyro |

---

## Session 4: LIO Enable Discovery & Stable Odometry (2026-02-22 ~08:00 CST)

This session resolved the odometry drift by switching from NOS localization to AOS lio_perception, discovering the critical `/LIO_ENABLE` service requirement, and establishing a reliable Mac bridge pipeline.

### Key Finding #32: `/opt/robot/fastdds.xml` Is NOT Loaded By Any Process

Spent significant time investigating DDS transport config. Definitive findings:

- `libfastrtps.so.2.14` checks for `DEFAULT_FASTRTPS_PROFILES.xml` in CWD and `FASTRTPS_DEFAULT_PROFILES_FILE` env var
- `libdrdds.so` hardcodes loading `/opt/drdds/dr_qos/drqos.xml` (QoS settings only)
- **No process sets `FASTRTPS_DEFAULT_PROFILES_FILE`** env var (verified via `/proc/<pid>/environ`)
- rslidar's CWD is `/opt/robot/share/node_driver/bin` — no XML there
- lio's CWD is `/` (from systemd) — no XML there either
- rslidar creates 52MB SHM segments (default FastDDS size), NOT 500MB as specified in `fastdds.xml`
- **Conclusion: `/opt/robot/fastdds.xml` is dead config — never loaded by any process**

### Key Finding #33: libdrdds.so Programmatically Configures Transport

All drdds binaries get transport config from `libdrdds.so` (27MB, `/usr/local/lib/`):

| Process | UDP Bindings | SHM |
|---|---|---|
| rslidar | 127.0.0.1 + multicast 239.255.0.1 | ✓ 52MB |
| yesense | 127.0.0.1 + multicast 239.255.0.1 | ✓ 52MB |
| lio_ddsnode | 127.0.0.1 + 10.21.33.103 + multicast | ✓ 52MB |
| basic_server | ALL interfaces | ✓ 52MB |

**Transport is set programmatically by libdrdds, NOT via XML config.** This explains why modifying `fastdds.xml` or setting `FASTRTPS_DEFAULT_PROFILES_FILE` had no effect.

### Key Finding #34: basic_server Relays Topics Cross-Host

`basic_server` on AOS binds to ALL interfaces and acts as a DDS relay:
- Subscribes to local topics on loopback/SHM
- Re-publishes on all interfaces (10.21.31.103, 10.21.33.103, etc.)
- This is how topics from AOS reach GOS/NOS

### Key Finding #35: BREAKTHROUGH — `/LIO_ENABLE` Service Required

**THIS WAS THE PRIMARY BLOCKER.** lio_ddsnode initializes but WAITS for an explicit enable command before processing sensor data.

Evidence:
- Binary `/opt/robot/share/lio_perception/bin/lio_command` sends integer commands to `/LIO_ENABLE` DDS service
- lio log shows "LIO command server started." on init — it's waiting, not processing
- Journal shows "Subscription matched" events but no data processing until enable

```bash
# Enable command:
sudo /opt/robot/share/lio_perception/bin/lio_command 1
# Output: "调用成功，res: 1" (Call successful, response: 1)
```

After enabling:
```
[08:38:02.199] LIO started
[08:38:02.200] Restarting LidarOdometry...
[08:38:02.200] LIO restarted
[08:38:02.223] 本帧雷达无对应有效imu数据 (no valid IMU for this frame)
[08:38:02.223] 本帧点云为空 (empty point cloud)  ← repeated ~10x during init
[08:38:03.345] Finish init first scan!  ← LIO initialized successfully
```

### Key Finding #36: lio_perception params.yaml Already Modified

From a previous session, the lio_perception output topics were renamed:
```yaml
output_odom_topic: "/ODOM"              # was "/LIO_ODOM"
output_aligned_cloud_topic: "/ALIGNED_POINTS"  # was "/LIO_ALIGNED_POINTS"
```
Backup at `params.yaml.dimos_bak`. This means lio_perception publishes to the same topic names that the mac_bridge subscribes to — no bridge changes needed.

### Key Finding #37: LIO Odometry Is Rock Stable

After enabling LIO, sampled `/ODOM` data on GOS:

```
Drift test (robot stationary, 35s, 273 samples):
  Max drift from first position: 0.58cm
  Start: (+0.0002, +0.0145, +0.0003)
  End:   (-0.0000, +0.0139, +0.0013)
  PASS: Drift < 5cm — LIO odometry is stable
```

**Comparison**: NOS localization was drifting 2m/10s. LIO drifts 0.6cm/35s. ~300x improvement.

### Key Finding #38: SSH Tunnels Cause Stale TCP Connections

SSH tunnels (`ssh -L 9731:...`) don't properly propagate TCP FIN when the Mac client disconnects. The GOS bridge sees the TCP connection as still alive (heartbeats previously sent = "authenticated"). New connections are rejected for up to 15s (heartbeat timeout).

**Fix applied (bridge-side)**: Removed "authenticated client" protection. New connections always kick existing ones. Single-client use case doesn't need protection.

**Fix applied (client-side)**: Added magic byte resynchronization to `FrameProtocol.read_frame()`. If first bytes aren't magic, scans forward byte-by-byte until magic found. Handles starting mid-stream after reconnect.

### Key Finding #39: iptables NAT Is Better Than SSH Tunnel

Mac (10.21.41.29) can't directly reach GOS (10.21.31.104) — different subnets. AOS has IP forwarding enabled (`ip_forward=1`).

**Solution**: iptables DNAT on AOS forwards WiFi traffic to GOS:
```bash
iptables -t nat -A PREROUTING -i wlan0 -p tcp --dport 9731 -j DNAT --to-destination 10.21.31.104:9731
iptables -t nat -A POSTROUTING -p tcp -d 10.21.31.104 --dport 9731 -j MASQUERADE
```

Saved to `/etc/iptables/rules.v4` via `iptables-persistent` (survives reboot).

**Advantages over SSH tunnel**: Proper TCP close propagation, no encryption overhead, transparent to bridge connection detection.

### Changes Made This Session

| Location | File | Change |
|---|---|---|
| AOS | `lio_ddsnode.sh` | Auto-enables LIO: starts lio_ddsnode in background, waits 5s, runs `lio_command 1` with retry |
| AOS | `lio_ddsnode.sh.orig` | Backup of original script |
| AOS | `/etc/iptables/rules.v4` | NAT PREROUTING: WiFi:9731 → GOS:9731 |
| GOS | `mac_bridge.py` | Always-kick: new connections always replace existing (fixes stale SSH tunnel connections) |
| Mac | `mac_bridge_client.py` | Magic byte resync in `FrameProtocol.read_frame()` |
| Mac | `mac_bridge.py` (local) | Same always-kick fix |

### Network Path (Final)

```
Mac (10.21.41.29) ──WiFi──→ AOS wlan0 (10.21.41.1:9731)
  ──iptables DNAT──→ AOS eth2 (10.21.31.103)
  ──wired──→ GOS (10.21.31.104:9731) mac_bridge
  ──ROS2/DDS──→ AOS (loopback) basic_server relay
  ──DDS──→ lio_ddsnode → /ODOM, /ALIGNED_POINTS
```

### Status After Session 4

| Component | Status | Notes |
|---|---|---|
| mac_bridge (GOS→Mac TCP) | **WORKING** | Always-kick fix deployed |
| iptables NAT (Mac→GOS) | **WORKING** | Persistent, no SSH tunnel needed |
| lio_perception (AOS) | **WORKING** | Auto-enables on boot, stable odometry |
| LIO odometry quality | **STABLE** | 0.6cm drift/35s (was 2m/10s from NOS) |
| TCP data flow | **VERIFIED** | ODOM 10Hz, LIDAR 10Hz (2350pts), IMU 180Hz, MI 15Hz |
| NAV_CMD (Mac→Robot) | **VERIFIED** | Zero-velocity commands sent successfully |
| lio auto-enable | **AUTOMATED** | `lio_ddsnode.sh` runs `lio_command 1` after 5s |
| NOS localization | BYPASSED | Using AOS lio_perception instead |
| Client resync | IMPLEMENTED | Handles mid-stream magic byte alignment |

### False Leads This Session

1. **`/opt/robot/fastdds.xml` transport config** — Spent time investigating XML config differences. The file is never loaded by any process. libdrdds sets transport programmatically.
2. **`FASTRTPS_DEFAULT_PROFILES_FILE` env var** — Setting this created extra DDS participants but didn't fix the input issue. The actual blocker was the `/LIO_ENABLE` service call.
3. **SHM segment analysis** — Deep-dove into data sharing segments and permission errors. These were symptoms of cleaning `/dev/shm` without restarting basic_server, not root causes.

---

## Session 5: Rerun Fix & Full dimos Launch (2026-02-22 ~01:10 CST)

Resolved the Rerun gRPC crash that prevented dimos from launching. Full navigation stack now running on Mac via the TCP bridge.

### Key Finding #40: Rerun gRPC Crash in Subprocess Workers

dimos launches modules in Dask worker subprocesses. `rr.init("dimos")` calls `flush_and_cleanup_orphaned_recordings()` which fails in subprocess workers:

```
RuntimeError: gRPC connection to rerun+http://127.0.0.1:9876/proxy gracefully disconnected
```

- Rerun 0.29.2 SDK and viewer versions match
- `rr.init('test')` works from the main process but fails in Dask workers
- Killing/restarting Rerun, cleaning temp state — no effect
- The error occurs in `dimos/visualization/rerun/bridge.py:257` inside `RerunBridgeModule.start()`

### Key Finding #41: `DIMOS_VIEWER_BACKEND=none` Env Var Doesn't Work

`GlobalConfig` uses `pydantic_settings.BaseSettings` but has NO `env_prefix` configured:

```python
# dimos/core/global_config.py
model_config = SettingsConfigDict(
    env_file=".env",
    env_file_encoding="utf-8",
    extra="ignore",
    # NO env_prefix — env vars like DIMOS_VIEWER_BACKEND are NOT read
)
```

Setting `DIMOS_VIEWER_BACKEND=none` has no effect. The valid `ViewerBackend` values are: `"rerun"`, `"rerun-web"`, `"foxglove"`, `"none"`.

### Key Finding #42: CRITICAL — `viewer_backend` Match Evaluates at Import Time

In `dimos/robot/deeprobotics/m20/blueprints/basic/m20_minimal.py` (line 61), the match statement runs at **module import time**:

```python
match global_config.viewer_backend:   # ← evaluates when m20_minimal.py is imported
    case "rerun":
        from dimos.visualization.rerun.bridge import rerun_bridge
        with_vis = autoconnect(_transports_base, rerun_bridge(**rerun_config))
    case _:
        with_vis = _transports_base   # ← fallback: no visualization
```

Calling `.global_config(viewer_backend="none")` AFTER importing the blueprint has no effect — `rerun_bridge` is already baked into the blueprint graph. The config must be set BEFORE importing.

### Fix: Set `viewer_backend` Before Import

Updated `launch_m20_smart.py`:

```python
# Disable Rerun BEFORE importing blueprints — the m20_minimal match statement
# evaluates at import time, so viewer_backend must be set first.
from dimos.core.global_config import global_config
global_config.update(viewer_backend="none")

from dimos.robot.deeprobotics.m20.blueprints.smart.m20_smart import m20_smart
# ... rest of imports
```

This causes the `case _:` fallback to fire, which uses `_transports_base` (no visualization module).

### Successful Launch

All 6 modules deployed to Dask workers:

| Module | Worker | Status |
|---|---|---|
| WebsocketVisModule | 3 | Running (port 7779) |
| VoxelGridMapper | 5 | Running (CPU:0) |
| CostMapper | 4 | Running |
| ReplanningAStarPlanner | 1 | Running |
| WavefrontFrontierExplorer | 2 | Running |
| M20Connection | 0 | Running (Mac Bridge) |

All transports wired: odom, lidar, global_map, global_costmap, cmd_vel, goal_request, explore_cmd, path, etc.

Web UI (Command Center): http://localhost:7779 — serving and accepting websocket connections.

### Changes Made This Session

| Location | File | Change |
|---|---|---|
| Mac | `launch_m20_smart.py` | Set `global_config.update(viewer_backend="none")` before blueprint import; removed Rerun reference from print |

### Status After Session 5

| Component | Status | Notes |
|---|---|---|
| dimos on Mac | **RUNNING** | All 6 modules deployed, no Rerun |
| Web UI (Command Center) | **RUNNING** | http://localhost:7779 |
| VoxelGridMapper | **RUNNING** | Building 3D voxel map from LiDAR |
| CostMapper | **RUNNING** | Generating occupancy grids |
| ReplanningAStarPlanner | **RUNNING** | Ready for path planning |
| WavefrontFrontierExplorer | **RUNNING** | Ready for autonomous exploration |
| M20Connection (Mac Bridge) | **RUNNING** | Connected to GOS via iptables NAT |
| lio_perception (AOS) | **RUNNING** | Auto-enabled, stable odometry |
| mac_bridge (GOS) | **RUNNING** | TCP:9731 forwarding all topics |
| iptables NAT (AOS) | **PERSISTENT** | WiFi:9731 → GOS:9731 |
| Rerun visualization | DISABLED | Bypassed with `viewer_backend="none"` |

### False Leads This Session

1. **`DIMOS_VIEWER_BACKEND=none` env var** — pydantic-settings has no `env_prefix`, env vars are ignored
2. **Rerun SDK/viewer version mismatch** — Both 0.29.2, versions matched; the issue was subprocess gRPC lifecycle
3. **Rerun process state** — Killed/restarted Rerun multiple times, cleared temp files; the subprocess issue persisted regardless of viewer state

---

## Session 6: Robot Not Moving — Velocity Command Pipeline Broken (2026-02-22 ~03:08 CST)

Re-enabled Rerun (`viewer_backend="rerun"`), verified native Rerun viewer working. Full navigation stack running — planner finding paths, Rerun showing 3D map + path visualization. But robot physically not moving.

### Key Finding #43: Robot Not Responding To Any Velocity Commands

Robot doesn't move despite:
- ReplanningAStarPlanner actively computing paths and entering `initial_rotation` → `path_following` states
- Local planner emitting `cmd_vel` Twist messages
- Keyboard control (web UI "Start Keyboard Control") also not working
- Exploration mode also not working

**Keyboard control failure is the critical clue** — it rules out planner/navigation issues. The problem is in the velocity command pipeline itself.

### Key Finding #44: Robot State Is Correct But Gait Wrong

Read MOTION_INFO directly from GOS bridge:
```
state=17 (RLControl) — correct, robot is in RL control mode
gait=0x1001 (Basic/Standard) — WRONG, should be 0x3002 (FlatAgile) for navigation
height=0.5697 — standing (~57cm)
vel_x=0.003, vel_y=-0.004, vel_yaw=0.001 — near zero (sensor noise)
```

Per the M20 Software Development Guide:
- **Section 1.2.5**: UDP axis command (Type=2, Cmd=21) is **"only supported in Regular Mode"**
- **Section 2.3.1**: `/NAV_CMD` ROS2 topic is **"only effective when the robot is in navigation mode"** and **"depends on the `basic_server` service"**
- **Section 1.2.4**: "When switching gaits, the robot will automatically switch to the corresponding motion mode"

The robot is in RL Control (state=17) with Standard gait (0x1001). dimos sends velocity through `/NAV_CMD` (bridge path), which requires Navigation mode + Agile gait. The gait switch to 0x3002 didn't take effect.

### Key Finding #45: NAV_CMD Through Bridge Does Not Move Robot

Direct test: connected to bridge, sent NAV_CMD frames with yaw_vel=0.3 at 20Hz for 2 seconds. MOTION_INFO vel_yaw remained at 0.0006 (sensor noise). **The robot completely ignores /NAV_CMD.**

### Key Finding #46: Gait Switch Via UDP Not Taking Effect

Sent gait switch command (Type=2, Cmd=23, GaitParam=0x3002) via UDP:
1. From Mac to AOS WiFi (10.21.41.1:30000) — gait stayed at 0x1001
2. From AOS itself to AOS wired (10.21.31.103:30000) — gait stayed at 0x1001
3. Sent in sequence: heartbeat → UsageMode.NAVIGATION → GaitParam=0x3002 — still no change

**The robot is rejecting or ignoring gait switch commands** regardless of source.

### Key Finding #47: UDP Communication Otherwise Healthy

Heartbeat → response test: AOS responds with full status reports (46 responses in 3s):
- Type=1002, Cmd=4: Motion Control Status (10Hz)
- Type=1002, Cmd=5: Device State (2Hz)
- Type=1002, Cmd=6: Basic Status (2Hz)
- Type=1002, Cmd=3: Abnormal Status (2Hz)

UDP port 30000 is listening on 0.0.0.0 on AOS. Communication works fine — just the gait switch command is ignored.

### Key Finding #48: AOS Services Running But height_map_nav Active

```
basic_server.service     — active (running) since 08:33 CST
rl_deploy.service        — active (running) since 06:05 CST
height_map_nav.service   — active (running), Cloud=0 Odom=0 IMU=0
```

`height_map_nav` is the robot's **built-in planner** — the dev guide warns of conflicts when publishing to `/NAV_CMD`. However, it reports `Cloud=0 Odom=0 IMU=0` so it's idle (no sensor data).

Failed services: `dr_wifi.service` (failed), `isc-dhcp-server` (failed). WiFi works despite dr_wifi failure.

### Key Finding #49: Teleop Controller Also Broken

User reported: "on the teleop controller I'm not seeing any battery levels or joint parameters reported." This suggests the broader control interface is in a degraded state — not just our velocity commands.

### Key Finding #50: Robot Battery Depleted

During investigation, robot ran out of battery. Session paused pending recharge and fresh boot.

### Root Cause Analysis

**Primary issue**: The robot's gait is stuck at 0x1001 (Basic/Standard Motion Mode) and cannot be switched to 0x3002 (Flat/Agile Motion Mode). This makes `/NAV_CMD` velocity commands ineffective (they require Agile mode). UDP axis commands also wouldn't work (they require Regular mode, not RL Control state).

**Contributing factors**:
1. Gait switch UDP commands are being silently ignored by AOS — possibly `basic_server` or `rl_deploy` in a degraded state
2. The teleop controller also lost status reporting — broader AOS control stack issue
3. `height_map_nav` (built-in planner) is running but starved of data — unclear if it interferes

**The velocity command pipeline in dimos is structurally correct** — the chain from local planner → cmd_vel → velocity controller → M20MacBridgeClient.publish_nav_cmd() → TCP → GOS mac_bridge → ROS2 /NAV_CMD is verified working. The issue is at the robot firmware/service level.

### Velocity Command Pipeline (Documented)

```
LocalPlanner._loop()
  → cmd_vel.on_next(Twist)              [RxPy Subject]
  → LCM transport                        [inter-process, Dask workers]
  → M20Connection._on_cmd_vel(twist)     [subscribes in start()]
  → M20VelocityController.set_twist()    [stores target + timestamp]
  → _control_loop() at 20Hz              [daemon thread]
  → _publish_control()
    → if nav_cmd_publish: bridge path    [absolute m/s]
      → M20MacBridgeClient.publish_nav_cmd(vx, vy, vyaw)
      → JSON encode → FrameProtocol.encode(MSG_NAV_CMD)
      → socket.sendall() to GOS:9731
      → GOS mac_bridge receives → publishes to /NAV_CMD ROS2 topic
    → else: UDP fallback                 [normalized [-1,1]]
      → M20Protocol.send_velocity() to AOS:30000
```

**Known silent failure point**: `M20MacBridgeClient._send_frame()` returns silently if `_connected` is not set (line 286-287). No logging on failure.

### Proposed Next Steps (After Recharge)

1. **Fresh boot** — battery depletion + recharge = clean restart of all services
2. **Verify gait switching works after fresh boot** — send GaitParam=0x3002 immediately
3. **Check `basic_server` journal** for errors during gait switch attempts
4. **If gait switching works**: restart dimos, verify robot moves with keyboard control first, then test navigation
5. **If gait switching still fails**: investigate `rl_deploy` and `basic_server` service health more deeply; may need Deep Robotics support

### Changes Made This Session

| Location | File | Change |
|---|---|---|
| Mac | `launch_m20_smart.py` | Set `viewer_backend="rerun"` (re-enabled Rerun) |
| Mac | `INVESTIGATION_LOG.md` | Added Session 5 + Session 6 findings |

### Status After Session 6

| Component | Status | Notes |
|---|---|---|
| dimos on Mac | STOPPED | Killed for investigation; need restart after recharge |
| Rerun visualization | **WORKING** | Native viewer showing 3D map, paths, goals |
| Planner/Navigation | COMPUTING | Finding paths, entering states, but robot not moving |
| Velocity pipeline (dimos) | **VERIFIED** | Full chain: planner → velocity ctrl → bridge → GOS → /NAV_CMD |
| Gait switching | **BROKEN** | 0x3002 command ignored by AOS, stuck at 0x1001 |
| /NAV_CMD | **IGNORED** | Robot doesn't respond (wrong gait/mode) |
| UDP velocity | **N/A** | Only works in Regular mode; robot in RL Control |
| Teleop controller | **DEGRADED** | No battery/joint data reported |
| Robot battery | **DEPLETED** | Recharging required |
| lio_perception | WORKING | Was producing stable odometry before battery died |
| mac_bridge (GOS) | WORKING | TCP data flowing to Mac |
| iptables NAT | PERSISTENT | Survives reboot |

---

## IMPORTANT REMINDERS (read every session)

1. **NEVER use SSH passwords** — SSH keys are set up on all hosts (AOS, GOS, NOS). Use key-based auth only. If keys are lost after a reboot, re-push them using `sshpass` once, then use keys thereafter.

2. **Always re-read the M20 Software Development Guide** when debugging M20 systems: `/Users/afik_cohen/gt/houmanoids_www/crew/nell/relay/docs/m20-official-software-development-guide.md`. It contains critical info about:
   - DDS topic dependencies (e.g. /LIDAR/POINTS depends on multicast-relay.service on NOS)
   - Service dependencies (e.g. /NAV_CMD depends on basic_server, conflicts with height_map_nav)
   - Velocity mode requirements (/NAV_CMD only works in Navigation Mode with Agile gait 0x3002)
   - State machine transitions and gait codes

3. **After fresh boot**, the following must happen:
   - SSH keys may need re-pushing to GOS/NOS
   - iptables NAT on AOS persists (saved to /etc/iptables/rules.v4)
   - lio_perception auto-sends lio_command 1 via modified lio_ddsnode.sh
   - multicast-relay.service on NOS may crash on boot (race condition with interface) — verify and restart if needed
   - height_map_nav (built-in planner) on AOS must be STOPPED to avoid /NAV_CMD conflicts

---

## Session 7: Fresh Boot After Battery Recharge (2026-02-22)

### Finding #51: DDS Data Not Flowing After Fresh Boot
**Problem**: Robot freshly rebooted. All services active (basic_server, rl_deploy, height_map_nav, lio_perception on AOS; dimos-mac-bridge on GOS; rsdriver on NOS). But height_map_nav shows `Cloud=0 Odom=0 IMU=0` — DDS sensor data from NOS not reaching AOS.

**Root cause investigation**:
- NOS multicast-relay.service crashed at boot with `OSError: [Errno 19] No such device` (race condition — interface not ready when service starts). Eventually restarted and became active, but DDS discovery may have been disrupted.
- lio_ddsnode on AOS showed `receive: 0` initially, then `receive: 1` after restart but immediately `Subscription unmatched (1 total)`.
- AOS has 3 ethernet interfaces: eth0 (10.21.33.103/24), eth1 (10.21.32.103/24), eth2 (10.21.31.103/24). NOS has eth0 (10.21.33.106/24) and eth1 (10.21.31.106/24). DDS multicast can flow on either shared subnet.

**M20 dev guide confirms**: "/LIDAR/POINTS topic depends on the multicast-relay.service service" (Section 2.1)

### Finding #52: height_map_nav Must Be Stopped
**M20 dev guide Section 2.3.1**: "When publishing this topic [/NAV_CMD], potential conflicts may arise with the robot's built-in planner service. Please refer to Appendix 2 to disable the planner service."

height_map_nav is the built-in planner. It conflicts with dimos publishing /NAV_CMD. Must be stopped.

### Finding #53: SSH Keys Setup for All Hosts
Pushed SSH keys (ed25519) from AOS to GOS and NOS. Mac key also pushed to GOS. Verified passwordless auth AOS→GOS, AOS→NOS. Keys survive across sessions but may be lost after robot firmware update.

### Finding #54: Bridge Data Flow Partially Restored
After restarting dimos-mac-bridge on GOS:
- **IMU**: ~1858 frames/10s (185Hz) — WORKING
- **MOTION_INFO**: ~200 frames/10s (20Hz) — WORKING
- **ODOM**: 0 — NOT WORKING (depends on lio_ddsnode SLAM)
- **TF**: 0 — NOT WORKING (depends on lio_ddsnode SLAM)
- **LIDAR**: 0 — NOT WORKING (depends on lio_ddsnode SLAM)

IMU and MOTION_INFO flow through drdds directly (from yesense_node and rl_deploy on AOS to GOS bridge). These bypass lio_ddsnode.

### Finding #55: Gait Switch Works on Fresh Boot!
On fresh boot with proper sequence:
1. Send STAND (Type=2, Cmd=22, MotionParam=1) → State changes to 1 (Stand)
2. Send NAVIGATION mode (Type=1101, Cmd=5, Mode=1)
3. Send AGILE_FLAT gait (Type=2, Cmd=23, GaitParam=0x3002)
4. Result: **State=17 (RLControl), Gait=0x3002, Mode=1 (Navigation)** ✓

Previous session's gait-stuck-at-0x1001 was likely due to the robot being in a degraded state (low battery / service issues).

### Finding #56: lio_ddsnode Not Receiving Sensor Data
**Critical unresolved issue**: lio_ddsnode (LIO SLAM) on AOS starts up, receives lio_command 1 successfully, but NEVER receives LiDAR or IMU data. This means no /ODOM, /tf, or /ALIGNED_POINTS ROS2 topics are published.

- rslidar (PID 2752) runs on AOS producing `msg_lidar id: 0/1` at high frequency
- yesense_node (PID 1675) runs on AOS
- lio_ddsnode (PID 12171) runs on AOS — same host!
- lio_ddsnode log shows only the lio_command service interaction, never any sensor data receipt
- Multiple restarts of lio_perception don't help
- drdds communication between AOS→GOS works fine (MOTION_INFO, IMU bridge correctly)
- The issue is specifically lio_ddsnode not subscribing to / not receiving from rslidar/yesense

**Resolved**: See Finding #57 below. lio_ddsnode WAS receiving data (log shows "Finish init first scan!"). The issue was DDS transport — lio's output couldn't reach GOS.

### Finding #57: BREAKTHROUGH — lio_ddsnode Output Not Reaching GOS Due to DDS Subnet Mismatch (RESOLVED)

**Problem**: lio_ddsnode on AOS initializes successfully ("Finish init first scan!"), is processing at 37.8% CPU, but `/ODOM` and `/ALIGNED_POINTS` don't reach GOS. `ros2 topic info /ODOM -v` on GOS shows **Publisher count: 0**.

**Root cause**: lio_ddsnode's DDS transport (configured by libdrdds.so) only uses `127.0.0.1 + 10.21.33.103` (eth0). GOS is on `10.21.31.104` (different subnet). DDS discovery via multicast works on the 10.21.31.x subnet but lio's data/metatraffic locators only point to 10.21.33.103, which GOS cannot reach.

IMU and MOTION_INFO work because basic_server (which publishes them) binds to ALL interfaces including 10.21.31.103. basic_server subscribes to /ODOM internally but does NOT re-publish it.

**Two-part fix**:

1. **fastdds_lio.xml** on AOS — adds `10.21.31.103` (eth2) as a `defaultUnicastLocatorList` entry + `userTransport`:
   ```xml
   <defaultUnicastLocatorList>
     <locator><udpv4><address>10.21.31.103</address></udpv4></locator>
   </defaultUnicastLocatorList>
   ```
   This makes lio_ddsnode advertise its data endpoints on eth2 (GOS's subnet).
   File: `/opt/robot/share/lio_perception/conf/fastdds_lio.xml`

2. **GOS route** — adds `10.21.33.0/24 via 10.21.31.103` so GOS can reach lio's metatraffic on AOS eth0:
   ```
   ip route add 10.21.33.0/24 via 10.21.31.103
   ```
   Persisted in `/etc/netplan/config.yaml` on GOS.

3. **lio_ddsnode.sh** on AOS — exports `FASTRTPS_DEFAULT_PROFILES_FILE` and uses `sudo -E` to pass it to lio_ddsnode:
   ```bash
   export FASTRTPS_DEFAULT_PROFILES_FILE=$SCRIPT_PATH/../conf/fastdds_lio.xml
   sudo -E chrt 35 taskset -c 4,5,6,7 $SCRIPT_PATH/../bin/lio_ddsnode &
   ```

**Important**: `metatrafficUnicastLocatorList` REPLACES default metatraffic locators in FastDDS. Setting it to only 10.21.31.103 breaks local lio_command discovery. Only use `defaultUnicastLocatorList` (data locators) — leave metatraffic to libdrdds defaults.

**Result after fix**: ALL topics flowing through bridge:
- ODOM: 9.2 Hz (stable, near-zero drift while stationary)
- LIDAR: 9.7 Hz (~2000 points/frame)
- IMU: 177 Hz
- MOTION_INFO: 19 Hz

### Finding #58: Topic Name Config vs Actual DDS Names (re-confirmed)
rslidar config says `dds_send_point_cloud_topic: /lidar_points` and `dds_send_imu_data_topic: /imu/data`, but these are NOT the actual DDS topic names. rslidar hardcodes `/LIDAR/POINTS` and yesense publishes `/IMU_YESENSE`. This was already established in Finding #5 (Session 1) — a known red herring.

### Changes Made This Session (Session 7 continued)

| Location | File | Change |
|---|---|---|
| AOS | `/opt/robot/share/lio_perception/conf/fastdds_lio.xml` | Added defaultUnicastLocatorList + userTransport for 10.21.31.103 (eth2) |
| AOS | `/opt/robot/share/lio_perception/scripts/lio_ddsnode.sh` | Added FASTRTPS_DEFAULT_PROFILES_FILE env var, sudo -E |
| GOS | `/etc/netplan/config.yaml` | Added route 10.21.33.0/24 via 10.21.31.103 |

### Current State (14:28 CST)
| Component | Status | Notes |
|-----------|--------|-------|
| Robot state | RLControl (17) | Standing, Agile Flat gait, Navigation mode |
| UDP protocol | WORKING | Heartbeat, stand, gait switch all work |
| lio_ddsnode SLAM | **WORKING** | Processing at 37.8% CPU, Finish init first scan! |
| ODOM | **FLOWING** | 9.2 Hz, stable (near-zero while stationary) |
| LIDAR | **FLOWING** | 9.7 Hz, ~2000 points/frame |
| IMU | **FLOWING** | 177 Hz |
| MOTION_INFO | **FLOWING** | 19 Hz |
| mac_bridge (GOS) | **WORKING** | All 5 topic types forwarding to Mac |
| iptables NAT | WORKING | Persistent across reboot |
| GOS route | **PERSISTENT** | 10.21.33.0/24 via 10.21.31.103 in netplan |
| height_map_nav | STOPPED | Disabled to avoid /NAV_CMD conflicts |

---

## Session 8: Exploration Failure + lio Subscription Match Root Cause (~18:00-19:15 CST, Feb 22)

### Summary
After Session 7's working state, exploration was triggered. Robot spun in circles, lio lost tracking and stopped publishing ODOM. After multiple lio restarts, only 1 subscription match (instead of the needed 2). Root cause identified: **DDS participant port collision between cpu_node and lio_ddsnode on 127.0.0.1:7412**. Fix found: restart lio BEFORE cpu_node to control port assignment order.

### Finding #59: lio Lost Tracking During Exploration
After triggering exploration, the robot spun in circles. lio_ddsnode lost tracking (likely due to rapid rotation exceeding SLAM tracking limits). ODOM went to 0 Hz on both AOS and GOS. After stopping exploration via command center, robot stopped spinning but lio never recovered — it needed a restart.

### Finding #60: Persistent 1-Match Problem After lio Restart
After restarting lio_ddsnode (multiple times), the log consistently showed:
```
[Subscription] Subscription matched  (1 total)
```
Only 1 match instead of the expected 2 (rslidar for /LIDAR/POINTS + yesense for /IMU_YESENSE). Without IMU data from yesense, lio cannot produce ODOM output.

### Finding #61: Full DDS Participant Port Map (ROOT CAUSE)
Using `sudo ss -ulnp | grep ':741[0-9]'`, mapped all DDS participants on AOS:

| Participant ID | Metatraffic Port | Process | Interfaces |
|---|---|---|---|
| 0 | 7410/7411 | gps_node (PID 49211) | 127.0.0.1 + 10.21.33.103 |
| 1 | 7412/7413 | **cpu_node** (127.0.0.1) + **lio_ddsnode** (10.21.31.103 + 10.21.33.103) | **SPLIT - collision!** |
| 2 | 7414/7415 | yesense_node | 127.0.0.1 ONLY |
| 3 | 7416/7417 | ctrlmcu_node | 127.0.0.1 + 10.21.33.103 |
| 32 | 7474/7475 | rslidar | 127.0.0.1 ONLY |
| 33 | 7476/7477 | lio_ddsnode (2nd participant) | 127.0.0.1 + 10.21.33.103 |

**The critical finding**: cpu_node (PID 1669, started at boot ~13:41) holds `127.0.0.1:7412`. When lio_ddsnode restarts later, it tries Participant 1 (port 7412) but can only bind 10.21.33.103:7412 and 10.21.31.103:7412. The 127.0.0.1:7412 binding fails silently because cpu_node already has it.

**DDS Port Formula** (RTPS spec): metatraffic_unicast = 7410 + 2 × participantId

### Finding #62: Why yesense→lio SEDP Fails
1. yesense (Participant 2) advertises metatraffic at 127.0.0.1:7414
2. lio Participant 1 advertises metatraffic at 10.21.33.103:7412 + 10.21.31.103:7412 (NO 127.0.0.1)
3. yesense's transport is loopback-only (interfaceWhiteList: 127.0.0.1)
4. yesense cannot reach 10.21.33.103 — it can only send to 127.0.0.1 addresses
5. SEDP endpoint exchange fails → /IMU_YESENSE publisher never matches lio's subscriber
6. lio's one match is from basic_server (which has all-interface transports at 0.0.0.0)

Confirmed via tcpdump: yesense sends packets to 127.0.0.1:7412, but those go to cpu_node (not lio).

### Finding #63: Why Session 7 Worked After Reboot
After a full AOS reboot, ALL services restart simultaneously. The startup order (from systemd service configs):
- yesense: `ExecStartPre=/bin/sleep 5`
- cpunode: `ExecStartPre=/bin/sleep 5`
- lio_ddsnode.sh: `sleep 10` at script start

After reboot, services race for participant IDs. The exact ID assignment depends on who binds first. In Session 7's fresh boot, lio likely got 127.0.0.1:7412 before cpu_node (or the timing was different). After a manual lio-only restart, cpu_node already holds 127.0.0.1:7412, so lio always loses.

### Finding #64: participantID XML Setting Broke Participant Init
Attempted to set `<participantID>50</participantID>` in the FastDDS XML profile to force lio to use ports 7510/7511 (avoiding cpu_node's 7412). Result:
- lio DID bind 127.0.0.1:7510, 10.21.33.103:7510, 10.21.31.103:7510 ✓
- BUT lio was completely silent — zero sends in 10 seconds
- FastDDS version is 2.14.2 (from `libfastrtps.so.2.14.2`)
- The `participantID` XML element may not be valid in this version, causing broken participant initialization
- **Reverted immediately**

### Finding #65: Restart Order Fix — WORKING
The correct fix: stop cpu_node systemd service → start lio_ddsnode → restart cpu_node.

Steps:
1. `sudo systemctl stop cpunode` — frees 127.0.0.1:7412
2. Kill any running lio_ddsnode processes
3. Start lio_ddsnode directly (with FASTRTPS_DEFAULT_PROFILES_FILE set, skipping the shell script's sleep 10)
4. Wait for lio to bind ports (~3-5 seconds)
5. `sudo systemctl start cpunode` — cpu_node gets a different Participant ID
6. Restart yesense for fresh SEDP discovery

**Result**: lio bound ALL three interfaces on port 7412:
```
UNCONN  10.21.31.103:7412  lio_ddsnode ✓
UNCONN  10.21.33.103:7412  lio_ddsnode ✓
UNCONN     127.0.0.1:7412  lio_ddsnode ✓  ← KEY FIX
```

lio_command succeeded after 4 retries ("调用成功, res: 1"). lio at 29% CPU, sending 12KB+ data frames to GOS (10.21.31.104). 1980 sends in 5 seconds confirmed.

**CRITICAL**: cpunode has `Restart=on-failure` in systemd, so must use `systemctl stop` not `kill`.

### Finding #66: libdrdds Uses XML Default Profile for Transports
Evidence that the XML `is_default_profile="true"` participant profile IS applied to libdrdds's `create_participant()` call:
- Without XML: lio only binds eth0 (10.21.33.103)
- With XML adding eth2_udp transport: lio also binds 10.21.31.103
- With XML adding loopback_udp transport AND cpu_node stopped: lio also binds 127.0.0.1

This confirms libdrdds calls `create_participant()` with `PARTICIPANT_QOS_DEFAULT`, which picks up the XML's transport config.

### Finding #67: lio Creates Two DDS Participants
lio_ddsnode creates two separate DDS participants via libdrdds:
- **Participant 1** (or whatever ID it gets): Used for sensor subscriptions (/LIDAR/POINTS, /IMU_YESENSE) and data publishing (/ODOM, /ALIGNED_POINTS). This is the critical one.
- **Participant 33** (or next available ID): Purpose unclear — possibly for the LIO command service (DrDDSServerChannel). Has loopback binding regardless.

The sensor subscriptions are on Participant 1, so Participant 1 MUST have loopback connectivity for yesense matching.

### Changes Made This Session

| Location | File | Change |
|---|---|---|
| AOS | `/opt/robot/share/lio_perception/conf/fastdds_lio.xml` | v3: loopback_udp + eth2_udp transports (no participantID) |
| AOS | Services | Restart order: stop cpunode → start lio → start cpunode |
| AOS | cpu_node | Temporarily stopped to free port 7412, then restarted |
| AOS | yesense | Restarted for fresh DDS discovery |
| AOS | lio_ddsnode | Started directly (bypassing shell script sleep 10) |

### Current State (04:50 CST, Feb 23)

| Component | Status | Notes |
|---|---|---|
| lio_ddsnode | **ACTIVE** | 29% CPU, 2 subscription matches, sending data to GOS |
| lio_command | **SUCCEEDED** | "调用成功, res: 1" after 4 retries |
| cpu_node | **RUNNING** | Restarted with different Participant ID |
| yesense | **RUNNING** | Restarted, exchanging SEDP with lio |
| rslidar | **RUNNING** | Never restarted (PID 88288 since 17:48) |
| Data to GOS | **FLOWING** | 856B + 65KB packets to 10.21.31.104 via eth2 |
| mac_bridge (GOS) | **RUNNING** | Service active 14h, no client connected |
| ODOM on GOS | **UNVERIFIED** | ros2 topic hz /ODOM timed out, needs verification |
| "Finish init first scan!" | **NOT SEEN** | Likely stdout buffering issue (nohup to file) |

### Next Steps
1. ~~Verify ODOM frequency on GOS via ros2 commands~~ → Done (Session 10)
2. ~~Connect Mac client to mac_bridge and verify end-to-end data flow~~ → In progress (Session 10)
3. ~~Make the restart-order fix permanent~~ → Superseded: removed all custom XML, vendor defaults work
4. ~~Update lio_ddsnode.sh to use `stdbuf -oL`~~ → Not needed: restored vendor lio_ddsnode.sh

---

## Session 10 — NOS Migration (Feb 23-24, 2026)

### Summary
Resolved lio_command failure root cause, migrated mac_bridge from GOS to NOS for cleaner DDS architecture. All infrastructure verified end-to-end.

### Finding #68: ANY Custom FastDDS XML Breaks lio_command
Tested multiple XML configurations this session:
1. **Minimal eth2-only XML** (no useBuiltinTransports) → lio_command fails all 10 retries
2. **Full XML** (SHM + loopback + eth2, useBuiltinTransports=false) → lio_command fails all 10 retries
3. **Full XML + `-E` flag on lio_command** → Progress! `Publisher matched:1`, `Subscription matched (2 total)`. But service call **HUNG** — lio_command at 17.7% CPU, 575MB RAM (SHM segment). Never completed. `receive: 1` never appeared.

**Root cause**: libdrdds.so programmatically configures DDS transport in C++ (Finding #33). Setting `FASTRTPS_DEFAULT_PROFILES_FILE` creates mixed transport stacks — the XML transports conflict with libdrdds's hardcoded transports. Even when both lio_ddsnode and lio_command load the same XML, the mixed config causes DDS service calls to discover peers but hang on data exchange.

**Solution**: Don't touch lio's DDS config at all. Vendor defaults are the only working configuration.

### Finding #69: Vendor Default lio_command Works Perfectly
After removing all custom XML and restoring `lio_ddsnode.sh`:
```
LIO command server started.
[Subscription] Subscription matched (1 total)    ← rslidar
Publisher matched:1
[Subscription] Subscription matched (2 total)
receive: 1
调用成功，res: 1                                   ← SUCCESS
[Subscription] Subscription unmatched (1 total)   ← lio_command exited
```
Two initial retries (`第0次尝试调用失败`, `第1次尝试调用失败`) are normal — lio_command starts discovery before lio_ddsnode's service endpoint is fully ready.

### Finding #70: NOS Has Full DDS + ROS2 Stack
NOS (10.21.33.106 eth0 / 10.21.31.106 eth1) verified:
- Python 3.8.10, ROS2 Foxy, rclpy, rmw_fastrtps_cpp
- drdds vendor types: NavCmd, MotionInfo — `from drdds.msg import NavCmd, MotionInfo` works
- eth0 (10.21.33.106) on same L2 as AOS eth0 (10.21.33.103) — 0.2ms ping
- All AOS DDS topics visible via `ros2 topic list` (50+ topics)
- `/ODOM` data confirmed flowing: `ros2 topic echo /ODOM` received position/orientation data

**Note**: `ros2 topic hz` returns nothing for bare DDS topics (libdrdds publishers) despite data flowing. Use `ros2 topic echo` to verify, or rely on rclpy subscriptions (which work fine — this is what mac_bridge uses).

### Finding #71: /ODOM QoS = RELIABLE
`ros2 topic info /ODOM -v` on NOS showed:
- Publisher: `_CREATED_BY_BARE_DDS_APP_` (libdrdds)
- QoS: `RELIABLE`, `VOLATILE`, `AUTOMATIC` liveliness
- Same QoS for /IMU, /MOTION_INFO, /ALIGNED_POINTS

mac_bridge.py uses `qos_profile_sensor_data` (BEST_EFFORT) for subscriptions — but this works because FastDDS allows BEST_EFFORT subscribers to match RELIABLE publishers (with potential data loss). For this use case it's fine.

### Finding #72: /tf Not Published on M20
`ros2 topic info /tf` returns "Unknown topic '/tf'" on NOS. mac_bridge.py subscribes to `/tf` but handles its absence gracefully (no callback = no data forwarded). This matches previous sessions — M20 doesn't publish standard ROS2 tf.

### Changes Made This Session

| Location | File | Change | Purpose |
|---|---|---|---|
| AOS | `/opt/robot/share/lio_perception/conf/fastdds_lio.xml` | Renamed to `.disabled_final` | Remove custom XML that broke lio_command |
| AOS | `/opt/robot/share/lio_perception/scripts/lio_ddsnode.sh` | Restored to vendor defaults + lio_command auto-enable | Remove FASTRTPS export, -E flags; keep Session 4 lio_command logic |
| AOS | iptables NAT PREROUTING | DNAT 9731 → 10.21.31.106 (was 10.21.31.104) | Route Mac traffic to NOS instead of GOS |
| AOS | iptables NAT POSTROUTING | MASQUERADE for 10.21.31.106:9731 (was 10.21.31.104) | Return traffic for NOS NAT |
| AOS | `/etc/iptables/rules.v4` | Persisted new NAT rules | Survive reboot |
| NOS | `/opt/dimos/mac_bridge.py` | Deployed (copied from GOS) | TCP bridge server |
| NOS | `/etc/systemd/system/dimos-mac-bridge.service` | Created + enabled | Auto-start bridge with correct LD_LIBRARY_PATH |

### NOS systemd Service
```ini
[Unit]
Description=dimos Mac Bridge (ROS2 → TCP)
After=network-online.target
StartLimitIntervalSec=300
StartLimitBurst=5

[Service]
Type=simple
ExecStart=/bin/bash -c 'source /opt/ros/foxy/setup.bash && /usr/bin/python3 /opt/dimos/mac_bridge.py --port 9731'
Restart=on-failure
RestartSec=3
Environment=LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/drdds/lib
Environment=PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/opt/drdds/lib/python3.8/site-packages
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
StandardOutput=journal
StandardError=journal
User=root

[Install]
WantedBy=multi-user.target
```

**Critical**: `LD_LIBRARY_PATH` must include `/opt/drdds/lib` — without it, `from drdds.msg import NavCmd, MotionInfo` fails silently and `/NAV_CMD` publish + `/MOTION_INFO` subscribe are disabled.

### Restored lio_ddsnode.sh (on AOS)
```bash
#!/bin/bash
sleep 10
SCRIPT_PATH=$(dirname "$(readlink -f "$0")")
echo "启动drdds节点"
sudo mkdir -p /var/opt/robot/data/maps/active

sudo chrt 35 taskset -c 4,5,6,7 $SCRIPT_PATH/../bin/lio_ddsnode &
LIO_PID=$!

sleep 5

echo "Enabling LIO via lio_command..."
sudo $SCRIPT_PATH/../bin/lio_command 1
if [ $? -ne 0 ]; then
    echo "lio_command failed, retrying in 3s..."
    sleep 3
    sudo $SCRIPT_PATH/../bin/lio_command 1 || echo "lio_command retry failed"
fi

wait $LIO_PID
```
Changes from vendor original: Added lio_command auto-enable logic (from Session 4). Removed: FASTRTPS_DEFAULT_PROFILES_FILE export, -E flags, custom comments.

### Network Path (Final)
```
Mac (WiFi) ──TCP 9731──→ AOS wlan0 (10.21.41.1)
                              │ DNAT → 10.21.31.106:9731
                              ↓
                         AOS eth2 (10.21.31.103)
                              │ forward + MASQUERADE
                              ↓
                         NOS eth1 (10.21.31.106) ← TCP arrives here
                         NOS mac_bridge :9731
                         NOS eth0 (10.21.33.106) ← DDS subscriptions
                              │ same L2 broadcast domain
                              ↓
                         AOS eth0 (10.21.33.103) ← lio_ddsnode publishes
```

### Current State (03:05 CST, Feb 24)

| Component | Status | Notes |
|---|---|---|
| lio_ddsnode (AOS) | **ACTIVE** | Vendor defaults, lio_command succeeded |
| lio_command (AOS) | **SUCCEEDED** | `调用成功，res: 1` after 2 retries |
| mac_bridge (NOS) | **RUNNING** | drdds enabled, TCP 9731 listening, systemd enabled |
| mac_bridge (GOS) | **RUNNING** | Still active, will disable after burn-in |
| NAT (AOS) | **UPDATED** | DNAT 9731 → NOS (10.21.31.106), persisted |
| TCP path Mac→NOS | **VERIFIED** | `nc -z 10.21.41.1 9731` succeeds, connection logged on NOS |
| /ODOM data | **FLOWING** | Confirmed via `ros2 topic echo` on NOS |
| dimos client test | **PENDING** | Next: run `launch_m20_smart.py` from Mac |

### Rollback Commands (if NOS bridge fails)
```bash
# On AOS — revert NAT to GOS
ssh user@10.21.41.1
echo "'" | sudo -S bash -c '
iptables -t nat -D PREROUTING -p tcp --dport 9731 -j DNAT --to-destination 10.21.31.106:9731
iptables -t nat -D POSTROUTING -p tcp -d 10.21.31.106 --dport 9731 -j MASQUERADE
iptables -t nat -A PREROUTING -p tcp --dport 9731 -j DNAT --to-destination 10.21.31.104:9731
iptables -t nat -A POSTROUTING -p tcp -d 10.21.31.104 --dport 9731 -j MASQUERADE
iptables-save > /etc/iptables/rules.v4
'
```

### Next Steps
1. ~~Run `python launch_m20_smart.py` from Mac~~ → Done, point cloud confirmed on Rerun
2. ~~Send velocity command from Mac~~ → Done, robot moved (explored autonomously)
3. ~~Monitor 5+ minutes for stability~~ → Partial, see Finding #73
4. ~~Disable GOS mac_bridge after burn-in~~ → Done
5. ~~Update `deploy_gos.sh` to support NOS target~~ → Done, renamed to `deploy_nos.sh`

### Finding #73: height_map_nav Conflicts with dimos /NAV_CMD
During first full e2e test, robot started exploring autonomously via dimos. When approaching a table, operator stopped exploration. After that:
- Point cloud stopped updating in Rerun
- Robot began halting/spinning in place
- Stop button on dimos command center had no effect

**Root cause**: `height_map_nav.service` was running on AOS. This is the vendor's built-in planner which also publishes to `/NAV_CMD`. Both dimos and height_map_nav were fighting over velocity commands — dimos sent stop (0,0,0) but height_map_nav immediately overwrote it.

**Fix**: `sudo systemctl stop height_map_nav && sudo systemctl disable height_map_nav` on AOS. This is explicitly documented in dev guide Section 2.3.1: *"Must disable height_map_nav to avoid /NAV_CMD conflicts."*

**Point cloud loss**: Likely caused by lio losing tracking during rapid deceleration (table collision) or TCP backpressure from high-frequency data. Needs restart of lio_perception and bridge to recover.

### Changes Made (continued)

| Location | File/Service | Change | Purpose |
|---|---|---|---|
| AOS | `height_map_nav.service` | Stopped + disabled | Eliminate /NAV_CMD conflict with dimos |
| Local | `deploy_gos.sh` → `deploy_nos.sh` | Renamed, updated for NOS | SSH jump host, NOS defaults, fixed systemd service |
| GOS | `dimos-mac-bridge.service` | Stopped + disabled | Replaced by NOS bridge |

### Current State (07:50 CST, Feb 24)

| Component | Status | Notes |
|---|---|---|
| lio_ddsnode (AOS) | **ACTIVE** | Vendor defaults, lio_command succeeded |
| mac_bridge (NOS) | **RUNNING** | drdds enabled, TCP 9731, systemd enabled |
| mac_bridge (GOS) | **DISABLED** | Stopped + disabled |
| height_map_nav (AOS) | **DISABLED** | Stopped + disabled to avoid /NAV_CMD conflict |
| NAT (AOS) | **PERSISTED** | DNAT 9731 → NOS (10.21.31.106) |
| dimos client (Mac) | **STOPPED** | Killed after conflict incident, ready to relaunch |

### Operational Checklist (before running dimos)
1. Ensure `height_map_nav` is stopped on AOS: `ssh user@10.21.41.1 "echo \"'\" | sudo -S systemctl is-active height_map_nav"`
2. Ensure `lio_perception` is active and lio_command succeeded
3. Ensure `dimos-mac-bridge` is running on NOS
4. Launch: `uv run python launch_m20_smart.py`

---

## Session 11 — Upstream Spec Adoption & Deploy Hardening (Feb 24, 2026)

### Context
After Session 10 (NOS migration), rebased `feature/m20-integration` onto latest `origin/dev` (37 new upstream commits). Needed to adopt new spec protocols and harden the deploy script.

### Finding #74: New Upstream Spec Protocols
37 upstream commits introduced three new spec protocols in `dimos/spec/perception.py`:
- `spec.Lidar` → `lidar: Out[PointCloud2]`
- `spec.IMU` → `imu: Out[Imu]`
- `spec.Odometry` → `odometry: Out[OdometryMsg]`

Also new message types: `dimos.msgs.sensor_msgs.Imu` (with covariances) and enhanced `dimos.msgs.nav_msgs.Odometry` (with `PoseWithCovariance`, `TwistWithCovariance`, convenience properties like `.vx`, `.yaw`).

No breaking changes to M20, but our code was using raw/legacy types where proper dimos types now exist.

### Finding #75: DDS Participant Startup Order Conflict (Root Cause)
**Critical discovery**: The NOS bridge's ROS2 DDS participant interferes with `lio_command`'s DDS service call on AOS.

**Symptoms**: `lio_command` hangs at `"Publisher matched:1"` indefinitely when bridge is running on NOS. lio_command succeeded immediately when bridge was stopped.

**Root cause**: FastDDS service discovery. The bridge creates a DDS participant on the shared L2 network (NOS eth0 10.21.33.106 ↔ AOS eth0 10.21.33.103). lio_command uses a DDS service to enable lio, and the extra participant confuses the service matching.

**Required startup order**:
1. `systemctl restart lio_perception` on AOS (wait for `调用成功，res: 1`)
2. `systemctl start dimos-mac-bridge` on NOS (only after lio is enabled)

### Changes Made

| Location | File | Change | Purpose |
|---|---|---|---|
| Local | `connection.py` | Added `spec.Lidar`, `spec.IMU`, `spec.Odometry` to class; added `imu: Out[Imu]`, `odometry: Out[OdometryMsg]` outputs; wired streams in `_start_ros_path()` | Adopt upstream spec protocols |
| Local | `ros_sensors.py` | Added `_ros_imu_to_dimos()`, `_ros_odom_to_dimos()` conversions; typed Subject/Observable to `DimosImu`/`DimosOdometry` | Proper dimos message types with covariances |
| NOS | `mac_bridge.py` | Extended `_on_odom()` with twist velocities + child_frame_id; `_on_imu()` with frame_id | Backward-compatible serialization for new fields |
| Local | `mac_bridge_client.py` | Added `_decode_odometry()`, updated `_decode_imu()` to return `DimosImu`; added `odometry_stream()` | Typed decoding, full Odometry with twist |
| Local | `deploy_nos.sh` | Added `ensure_lio_enabled()` with 3-signal health check; `lio_publishing_data()` data flow check | Automate startup order, prevent DDS conflict |

### Three-Signal lio Health Check (`deploy_nos.sh`)
1. **`pgrep -f lio_ddsnode`** on AOS — is the DDS node process running?
2. **`! pgrep -f 'lio_command 1'`** on AOS — has the enable sequence completed?
3. **`ros2 topic echo /ODOM --once`** on NOS — is data actually flowing over shared L2?

Fast path: if all 3 healthy, skip restart. Otherwise restart lio_perception and poll up to 45s.

**Note**: The `/ODOM` echo check timed out during deploy testing (45s) despite lio being healthy. Likely a ROS2 Foxy DDS discovery timing issue on NOS — the bridge wasn't running yet so no DDS participant to relay. The check is conservative (proceeds anyway with a warning). AOS process checks confirmed lio was healthy.

### Backward Compatibility (Bridge Protocol)
- New JSON fields added to odom: `lx`, `ly`, `lz`, `ax`, `ay`, `az`, `child_frame_id`
- New JSON field added to IMU: `frame_id`
- Client decoders use `.get()` with defaults — handles both old and new bridge format gracefully
- Old bridge running on NOS before redeploy still works with new client code

### End-to-End Test Result
Deployed updated bridge via `deploy_nos.sh --mac-bridge`, launched `uv run python launch_m20_smart.py`:
- All modules deployed (7 modules, 6 workers)
- New transports registered: `odometry` (`/odometry#nav_msgs.Odometry`), `imu` (`/imu#sensor_msgs.Imu`)
- Bridge connected, data flowing
- Frontier exploration started autonomously — 9 frontiers detected, goal published, robot navigating
- Robot replanned once (stuck detection), then continued exploring

### Current State (19:22 CST, Feb 24)

| Component | Status | Notes |
|---|---|---|
| lio_ddsnode (AOS) | **ACTIVE** | lio_command completed, /ODOM flowing |
| mac_bridge (NOS) | **RUNNING** | Updated with twist + frame_id serialization |
| height_map_nav (AOS) | **DISABLED** | Still disabled from Session 10 |
| dimos (Mac) | **RUNNING** | Autonomous exploration active, all spec streams wired |

### Commits
- `d8e4db742` — Adopt upstream spec protocols (Lidar, IMU, Odometry) and harden deploy

---

## Session 12: Full dimos on NOS in Humble Docker Container (Feb 25, 2026)

### Goal
Run the full dimos stack (navigation, mapping, planning, web UI) directly on NOS inside a ROS2 Humble Docker container, eliminating the Mac-side TCP bridge entirely.

### Architecture Change
```
BEFORE: Mac (dimos full stack) → TCP bridge → NOS (mac_bridge.py) → ROS2 Foxy DDS
AFTER:  NOS Docker (dimos full stack) → direct rclpy → ROS2 Humble DDS
        Mac → http://10.21.41.1:7779/command-center (thin client only)
```

### Why Docker?
M20 runs ROS2 Foxy natively (Python 3.8, Ubuntu 20.04). dimos requires Python 3.10+ (Humble). Deep Robotics provided `ysc_ros2_backup.tar.gz` — ARM64 Ubuntu 22.04 + ROS2 Humble + drdds vendor messages. This Docker container bridges the version gap without modifying the host OS.

### Key Findings

1. **NOS hardware**: 4 cores, 15GB RAM, 18GB disk, ARM64 — identical to GOS and AOS
2. **Vendor image**: `192.168.102.241:8881/ysc_docker_hub/ysc_jammy_ros2:arm64.1.0.3` (4.39GB), Python 3.10.12, ROS2 Humble, drdds vendor messages pre-installed
3. **Private apt sources**: Vendor image has apt sources pointing to `192.168.102.241:25193` (unreachable). Must replace with `ports.ubuntu.com` before installing packages
4. **docker build OOM**: `docker build` on NOS sends entire build context to daemon, saturating 4-core ARM64. Solution: build image interactively with `docker exec` + `docker commit` instead
5. **NumPy 2.x vs system matplotlib**: System matplotlib (from Ubuntu packages) compiled against NumPy 1.x. Installing NumPy 2.2.6 in venv causes `_ARRAY_API not found`. Fix: install matplotlib 3.10+ in venv to override system version
6. **langchain-core missing**: dimos core module imports `langchain_core.tools` — not in the dep list. Added interactively
7. **LCM autoconf prompt blocks**: `configure_system()` prompts `input("Apply changes? [y/N]:")` which gets EOFError in non-interactive container, causing SystemExit(1). Fix: set `CI=1` env var to skip the check (sysctl/multicast configured in entrypoint instead)
8. **FastDDS XML parse error**: `max_message_size` is not valid for SharedMemory transport descriptor in this FastRTPS version. Non-fatal — DDS falls back to defaults
9. **lio /ODOM check from NOS**: The deploy.sh lio health check uses `ros2 topic echo /ODOM --once` from NOS via Foxy setup.bash. This often times out due to Foxy-Humble DDS version mismatch. lio is actually healthy — the check is unreliable cross-version

### Files Created
| File | Purpose |
|---|---|
| `dimos/robot/deeprobotics/m20/docker/Dockerfile` | Extends pre-built deps image with dimos source |
| `dimos/robot/deeprobotics/m20/docker/deploy.sh` | Build/start/stop/logs/shell/status lifecycle |
| `dimos/robot/deeprobotics/m20/docker/entrypoint.sh` | Sources Humble, configures DDS + LCM, waits for topics |
| `dimos/robot/deeprobotics/m20/docker/launch_nos.py` | Direct ROS2 mode launch (no bridge) |
| `dimos/robot/deeprobotics/m20/docker/fastdds.xml` | DDS transport config (UDP + SHM) |
| `dimos/robot/deeprobotics/m20/docker/docker-compose.yml` | Container config (host network, host IPC) |

### Docker Image Build Process (for NOS ARM64)
Standard `docker build` OOMs on NOS. Instead:
1. Load vendor image: `docker load -i ysc_ros2_backup.tar.gz`
2. Tag: `docker tag <id> dimos/m20-humble-base:latest`
3. Start builder: `docker run -d --name deps-builder --network host dimos/m20-humble-base:latest sleep 3600`
4. Fix apt sources (replace private DR repos with ports.ubuntu.com)
5. Install system deps: `python3-venv python3-dev libturbojpeg0-dev libgl1 libglib2.0-0 git curl`
6. Create venv: `python3 -m venv --system-site-packages /opt/dimos-venv`
7. Install pip deps in batches (numpy, scipy, dask, rerun-sdk, etc.)
8. Commit: `docker commit deps-builder dimos/m20-deps:latest`
9. Start new builder from deps image, copy dimos source via `docker cp`
10. `pip install -e /opt/dimos/src` + copy entrypoint/fastdds/launch_nos
11. Commit final: `docker commit --change 'ENTRYPOINT/CMD' builder dimos-m20-nos:latest`

### Container Runtime Requirements
- `--privileged` — required for sysctl and ip route (LCM multicast)
- `--network host` — DDS discovery on shared L2 with AOS
- `--ipc host` — shared memory DDS transport
- `-e CI=1` — skip interactive LCM autoconf prompt
- `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp` — DDS middleware

### Current State (10:55 CST, Feb 25)

| Component | Status | Notes |
|---|---|---|
| dimos-m20 container (NOS) | **RUNNING** | All modules deployed, web UI on port 7779 |
| WebsocketVisModule | **DEPLOYED** | Port 7779, GPS goal tracking enabled |
| M20Connection | **DEPLOYED** | odometry, IMU, pointcloud, color_image, camera_info |
| ReplanningAStarPlanner | **DEPLOYED** | Navigation ready |
| WavefrontFrontierExplorer | **DEPLOYED** | Exploration ready |
| lio_perception (AOS) | **ACTIVE** | /ODOM, /IMU flowing |
| height_map_nav (AOS) | **STOPPED** | Conflicts with /NAV_CMD |
| planner (NOS) | **STOPPED** | Conflicts with velocity commands |
| NAT rules (AOS) | **ACTIVE** | 7779→NOS, 9876→NOS |
| viewer_backend | **none** | Disabled for initial testing (Rerun crashed dask workers) |

### Commits
- `557d99734` — Add Docker deployment for running full dimos on NOS in Humble container
- `2b42b8ac8` — Fix deploy.sh argument parsing: command before host/user
- `0727bf228` — Fix Docker deployment: pre-built deps image, LCM autoconf, viewer disable

### Finding #10: NAT rules not persisted across power cycles
The `deploy.sh ensure_nat()` uses `aos_sudo` to add iptables rules, but the rules added during `deploy.sh start` silently failed (likely due to sudo piping issues with `aos_sudo`). NAT rules had to be added manually:
```bash
ssh user@10.21.41.1 "printf \"'\\n\" | sudo -S iptables -t nat -A PREROUTING -i wlan0 -p tcp --dport 7779 -j DNAT --to-destination 10.21.31.106:7779"
ssh user@10.21.41.1 "printf \"'\\n\" | sudo -S iptables -t nat -A PREROUTING -i wlan0 -p tcp --dport 9876 -j DNAT --to-destination 10.21.31.106:9876"
```
After adding rules manually, `curl http://10.21.31.106:7779/command-center` from AOS returned HTTP 200, and the web UI loaded from Mac at `http://10.21.41.1:7779/command-center`.

### Current State (11:05 CST, Feb 25)

| Component | Status | Notes |
|---|---|---|
| dimos-m20 container (NOS) | **RUNNING** | All modules deployed, stable for 6+ min |
| Web UI (Mac) | **ACCESSIBLE** | http://10.21.41.1:7779/command-center — HTTP 200 |
| NAT rules (AOS) | **ACTIVE** | 7779→NOS, 9876→NOS (manually added) |

---

## Session 13 — Docker Debugging: Rerun, Resource Optimization, Data Flow (Feb 25)

### Finding #11: Web UI black screen — lio needs navigation mode

After power cycle, web UI loaded (HTTP 200) but showed black screen with no map. Keyboard control didn't work. Root cause: lio_command ran but the robot wasn't in navigation mode. /ODOM topic existed in DDS discovery but had no data. After user switched robot to navigation mode via controller, /ODOM data started flowing at 10Hz. The entrypoint topic wait passed (`/ODOM: OK`, `/IMU: OK`).

**Lesson**: lio_perception produces /ODOM only when robot is in navigation mode. DDS topic discovery lists topics even when no data flows — `ros2 topic list` is insufficient, must verify with `ros2 topic echo`.

### Finding #12: FastDDS XML parse error — `max_message_size` invalid for SHM

The `max_message_size` element in the SharedMemory transport descriptor caused FastDDS XML parse errors. Removed it; `segment_size` is the only valid SHM sizing parameter. Non-fatal (DDS falls back to defaults) but generates noise in logs.

### Finding #13: Rerun viewer enabled successfully with CI=1 fix

Previous attempt to enable `viewer_backend="rerun-web"` crashed dask workers via `SystemExit(1)` from LCM autoconf's interactive `input()` prompt. With `-e CI=1` env var, the autoconf is skipped entirely. RerunBridgeModule now deploys cleanly.

Rerun serves on two ports:
- **9876**: gRPC data server (for `rerun --connect rerun+http://HOST:9876/proxy`)
- **9090**: Web viewer (serves HTML page with embedded viewer)

Both require NAT rules on AOS. Port 9090 was initially missed.

### Finding #14: drdds-ros2-msgs not available in Humble container

The vendor Docker image has drdds as a C++ DDS library (`/opt/drdds/`, `/usr/local/lib/cmake/drdds`) but NO Python ROS2 message bindings. The Python bindings exist only at `/opt/drdds/lib/python3.8/site-packages` on the Foxy system — compiled for Python 3.8, incompatible with Python 3.10 (Humble).

**Impact**: `/NAV_CMD` publisher and `/MOTION_INFO` subscription disabled. M20VelocityController falls back to UDP protocol for velocity commands (normalized [-1,1] range instead of absolute m/s). Robot obstacle avoidance is bypassed in this mode.

### Finding #15: NOS resource contention — 383% CPU on 4 cores

After power cycle, NOS was running at load average 27 on 4 cores. Root causes identified and fixed:

| Process | CPU% | Action | Reason |
|---------|------|--------|--------|
| ros2_daemon (Foxy) | 78% | Killed | Foxy/Humble DDS discovery storm — daemon thrashes trying to reconcile Humble type hashes |
| mac_bridge.py | 56% | Killed | Replaced by Docker container |
| multicast-relay | 27% | Stopped + disabled | dimos uses DDS not multicast; /LIDAR/POINTS relay not needed on NOS |
| rsdriver (rslidar) | 18% | Stopped + disabled | Lidar driver; lio runs on AOS, NOS doesn't need raw lidar |
| yesense | 7% | Stopped + disabled | /IMU comes from main controller independently |
| charge_manager | 10% | **Kept** | Needed for autonomous charging |
| reflective_column | 2.4% | **Kept** | Needed for charging dock localization |
| handler | 14% | **Kept** | Robot state management, essential |

Total freed: ~186% CPU. Load average dropped from 27 to ~8.

**Foxy ros2_daemon thrashing explained**: With `--network host`, the Humble container's DDS participants share multicast with the Foxy host. Humble uses a newer DDS type system with type hashes that Foxy can't resolve. Every 3-second lease announcement triggers the Foxy daemon to attempt type resolution, fail, queue for retry, repeat. 78% CPU doing nothing useful.

**Prevention**: `entrypoint.sh` now sets `ROS2_DAEMON_TIMEOUT=0` and runs `ros2 daemon stop`. `deploy.sh check_conflicts()` kills mac_bridge, ros2_daemon, and stops unneeded services.

**Per DR official docs**: "After each OTA update, the auto-start settings of all services will be reset to factory defaults" — services need re-disabling after firmware updates.

### Finding #16: Rerun hitchiness — gRPC backpressure over WiFi

Despite 10Hz ROS2 data and clean rclpy pipeline (SingleThreadedExecutor, 100Hz spin, 1:1 synchronous callbacks, no batching), Rerun showed data arriving in ~1-second bursts. Root cause: **Rerun gRPC channel backpressure**.

Container logs:
```
re_grpc_server broadcast: Sender has been blocked for over 5 seconds waiting for space in channel
log_channel(SDK): Sender has been blocked for over 5 seconds waiting for space in channel
batcher_output: Sender has been blocked for over 5 seconds waiting for space in channel
```

Data path: NOS container → gRPC → AOS NAT → WiFi → Mac native Rerun viewer. The WiFi link can't sustain the combined bandwidth of point clouds + color images + maps + odometry. Rerun SDK queues data, channel fills, sender blocks 5+ seconds, data gets through in bursts when channel drains.

**M20ROSSensors architecture** (confirmed no batching):
- rclpy: `SingleThreadedExecutor`, `spin_once(timeout_sec=0.01)` = 100Hz spin in background thread
- Callbacks: synchronous 1:1, ROS msg → `Subject.on_next()` → LCM publish
- QoS: RELIABLE, KEEP_LAST(10), VOLATILE
- No batching, no throttling at any layer

**Not fixable without wired connection** — this is a WiFi bandwidth limitation. Potential mitigations: downsample point clouds before Rerun logging, reduce image resolution, or connect Mac directly to NOS via ethernet.

### Commits
- `557d99734` — Add Docker deployment for running full dimos on NOS in Humble container
- `2b42b8ac8` — Fix deploy.sh argument parsing: command before host/user
- `0727bf228` — Fix Docker deployment: pre-built deps image, LCM autoconf, viewer disable
- (uncommitted) — FastDDS XML fix, Rerun enabled, 3 dask workers, deploy.sh service management, entrypoint daemon prevention

### Current State (20:30 CST, Feb 25)

| Component | Status | Notes |
|---|---|---|
| dimos-m20 container (NOS) | **RUNNING** | 3 dask workers, all modules deployed |
| Rerun viewer (Mac native) | **WORKING** | Point cloud, costmap, color_image, odom visible — hitchy due to WiFi bandwidth |
| Web UI (Mac) | **ACCESSIBLE** | http://10.21.41.1:7779/command-center |
| Color camera | **WORKING** | First time! Didn't work over mac_bridge TCP |
| NAT rules (AOS) | **ACTIVE** | 7779, 9876, 9090 → NOS |
| NOS load average | **~8** (was 27) | After stopping 5 unnecessary services |
| drdds /NAV_CMD | **DISABLED** | No Humble Python bindings; UDP fallback active |

### Next Steps
- Test keyboard control via web UI (UDP fallback path)
- Test navigation: send goal from web UI → verify robot moves
- Test exploration: start autonomous exploration from web UI
- Investigate wired connection option for smoother Rerun streaming
- Consider building drdds Python bindings for Humble to enable /NAV_CMD
- Commit pending changes
