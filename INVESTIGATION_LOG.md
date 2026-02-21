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

## Working Theory: DDS Transport Mismatch

rslidar loads fastdds.xml → uses UDP on 127.0.0.1 + SHM.
localization/lio_perception use default FastDDS → UDP on all interfaces + SHM + built-in transports.

For DDS endpoints to match, they need compatible transport. If rslidar publishes via SHM and localization subscribes via SHM, they SHOULD match on the same host. But if there's a transport negotiation issue (e.g., rslidar advertises loopback-only unicast locator, localization tries to reach it, fails because of routing), matching would fail.

## What Changed Between Yesterday and Today?

**Unknown.** The user confirms data was flowing yesterday (ALIGNED_POINTS visible in Rerun). Today nothing works. Configs, binaries, and packages are all dated weeks/months ago.

Possibilities:
1. Robot reboot changed something in DDS shared memory state
2. A boot-time service ordering dependency broke
3. The clock being wrong when services first started left persistent DDS state
4. SHM cleanup (done on NOS) affected something
5. An OTA update happened between sessions

## Next Steps to Investigate

1. **Confirm rslidar loads fastdds.xml** — check with `strace -e openat` or check `lsof` for open file handles
2. **Check SHM communication** — if rslidar and localization both use SHM, check if SHM segments are accessible
3. **Try removing fastdds.xml temporarily** — would force rslidar to use default transport (all interfaces), matching localization
4. **Check if removing SHM transport restriction helps** — modify fastdds.xml to allow all interfaces
5. **Check GOS data flow** — GOS rslidar publisher IS visible locally; does the mac_bridge on GOS receive /LIDAR/POINTS?

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
