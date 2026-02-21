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
| **localization** | `localization_ddsnode` | NOS | Graph-based SLAM/localization (v3.3.1). Subscribes to `/LIDAR/POINTS` + `/IMU`, fuses them to produce `/ODOM` (pose) and `/ALIGNED_POINTS` (registered point cloud). Loads a pre-built map from `/var/opt/robot/data/maps/active/occ_grid.yaml`. Also launches `map_server` as a sibling process. Config: `/opt/robot/share/localization/conf/params.yaml`. Started with `chrt 48 taskset -c 4,5,6,7`. Does NOT load fastdds.xml (uses default all-interface transports). |
| **lio_perception** | `lio_ddsnode` | AOS | Lidar-Inertial Odometry (v1.0.11). Subscribes to `/LIDAR/POINTS` + `/IMU`, produces `/LIO_ODOM` and `/LIO_ALIGNED_POINTS`. Lighter than NOS localization — no map, real-time odometry only. Runs a UDP sender on port 30100. Started with `chrt 35 taskset -c 4,5,6,7`. Does NOT load fastdds.xml. |

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
| **mac_bridge** | `mac_bridge.py` | GOS | rclpy (ROS2 Foxy) subscriber that forwards DDS topics over TCP:9731 to Mac. Subscribes to `/ODOM`, `/ALIGNED_POINTS`, `/IMU`, `/tf`, `/MOTION_INFO`. Runs as `user` (not root) — this is the cause of the GOS SHM permission bug. Systemd service: `dimos-mac-bridge.service`. |
| **m20-relay** | `relay.main` | GOS | Browser-to-ROS2 bridge for Houdini teleop UI. Handles battery status, manual control. NOT involved in autonomy data flow. |

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

All Deep Robotics processes run as root. Sensor/perception processes run on CPUs 4-7 with real-time scheduling (SCHED_RR). Handler is the exception — CPUs 0-3 with normal scheduling. Our processes (mac_bridge, m20-relay) run as `user`, which causes the GOS SHM permission mismatch.

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

The reboot changed:
1. New SHM segment names (new PIDs → new segment hashes)
2. New DDS GUIDs
3. Service startup order may differ
4. NOS clock now correct (was wrong yesterday)
5. Possibly different FastDDS SHM initialization state

**Yesterday's working path was likely**: AOS lio_perception → `/LIO_ALIGNED_POINTS` → cross-host DDS → GOS → mac_bridge. This path may have worked due to a lucky DDS state that the reboot destroyed.

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

### Problem 1: GOS 0 Hz (all topics) — SOLVED
**Root cause**: FastDDS SHM permission mismatch between root (rslidar/drdds) and user (mac_bridge/rclpy). FastDDS selects SHM transport, SHM fails silently, no UDP fallback.

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

## RECOMMENDED ACTION PLAN (for when user returns)

**Goal**: Get dimos on Mac receiving ODOM + ALIGNED_POINTS + IMU via mac_bridge for auto exploration testing.

**Step 1** (GOS, highest confidence, 1 min):
```bash
ssh -J user@10.21.41.1 user@10.21.31.104
echo "'" | sudo -S sed -i 's/User=user/User=root/' /etc/systemd/system/dimos-mac-bridge.service
echo "'" | sudo -S systemctl daemon-reload
echo "'" | sudo -S systemctl restart dimos-mac-bridge
```
This fixes the SHM permission mismatch and should immediately deliver `/LIDAR/POINTS` from GOS rslidar to mac_bridge.

**Step 2** (AOS, medium confidence, 2 min):
```bash
ssh user@10.21.41.1
echo "'" | sudo -S systemctl restart yesense rsdriver lio_perception
# Wait 60s for rsdriver sleep 30 + startup
echo "'" | sudo -S journalctl -u lio_perception -n 30 --no-pager
```
Restart AOS services to re-trigger DDS discovery. If lio_perception gets IMU this time, it will produce `/LIO_ODOM` and `/LIO_ALIGNED_POINTS`.

**Step 3** (verify on Mac):
```bash
# From Mac, with SSH tunnel already set up:
python launch_m20_smart.py  # bridge_host="127.0.0.1"
# Check rerun for ODOM, ALIGNED_POINTS, LIDAR data
```

**Step 4** (if Step 2 fails — NOS fallback):
Try toggling `imu_use_system_time: true` in NOS localization config, or restart all NOS services. This is lower confidence since the issue appears to be in the localization binary's timestamp handling.

**Step 5** (if all else fails — bypass entirely):
Write a simple drdds subscriber on GOS (Python 3.8 + drdds) that reads `/IMU` and `/LIDAR/POINTS` directly via drdds (same transport as rslidar/yesense), bypassing rclpy entirely.
