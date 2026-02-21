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
