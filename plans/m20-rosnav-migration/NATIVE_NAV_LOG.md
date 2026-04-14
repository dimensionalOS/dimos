# M20 Native Navigation Log

**Goal**: Run the entire M20 nav stack natively on NOS without Docker containers or ROS.
**Date started**: 2026-04-10
**Branch**: `feat/deeprobotics-m20-nav`

---

## Status: FULLY WORKING — ROS-free WASD teleop verified

Full native pipeline running on NOS, zero Docker, zero ROS:
- DrddsLidarBridge (C++ NativeModule) → lidar + IMU via POSIX SHM → LCM
- AriseSLAM (C++ NativeModule) → registered_scan + odometry
- SmartNav (C++ NativeModules: TerrainAnalysis, LocalPlanner, PathFollower, SimplePlanner + Python: PGO, CmdVelMux, ClickToGoal)
- NavCmdPub (C++ NativeModule) → raw FastDDS `rt/NAV_CMD` → AOS basic_server → motors
- RerunBridge + dimos-viewer → WASD teleop verified working

**No Docker. No ROS. No rclpy. No bridges. No TCP hops.**

---

## Finding no.1: Nix on Kernel 5.10 — fchmodat2 Workaround (2026-04-10)

See `06-simplify-remove-container/nix-arm64-kernel510-workaround.md` for full details.

Nix's coreutils (glibc 2.42) uses `fchmodat2` syscall which kernel 5.10 doesn't have.
Fixed by overriding `unpackPhase` and `fixupPhase` to use host coreutils (`/usr/bin/cp`, `/usr/bin/chmod`).
Also requires `/nix` bind-mounted to ext4 (root is overlayfs).

---

## Finding no.2: drdds /NAV_CMD is Local-Only — Can't Cross NOS→AOS (2026-04-10)

**Resolved by Finding no.5.**

The drdds SDK hardcodes SHM-only transport. Cross-board drdds communication is unsupported.
`DrDDSManager::Init` ignores participant-level XML config for transport — only QoS is configurable.

Tried and failed: `drqos.xml` modifications, `FASTRTPS_DEFAULT_PROFILES_FILE` env var,
`enable_config=true`, `enable_discovery=true`. None enable UDP transport from NOS to AOS.

---

## Finding no.3: DrDDSChannel Matches but Robot Doesn't Move (2026-04-11)

Using `DrDDSChannel` (pub+sub pair) on AOS achieves `matched_count=1` with `basic_server`.
Confirmed NOT a self-match. Data reaches basic_server. But robot doesn't move.

**Root cause discovered in Finding no.4**: DrDDSChannel publishes on DDS topic `/NAV_CMD`,
but basic_server subscribes via ROS2 on DDS topic `rt/NAV_CMD`. Different topics!

---

## Finding no.4: ROS2 Topic Name Convention — rt/ Prefix (2026-04-12)

**The breakthrough insight.**

ROS2's `rmw_fastrtps` prefixes ALL DDS topic names with `rt/`. When rclpy publishes
to `/NAV_CMD`, the actual DDS topic is `rt/NAV_CMD`. When drdds publishes to `/NAV_CMD`,
the DDS topic is literally `/NAV_CMD` — no prefix.

`basic_server` subscribes to `/NAV_CMD` via ROS2 (confirmed by `ros2 topic list` showing
`/NAV_CMD` on AOS). Its actual DDS subscription is on `rt/NAV_CMD`.

This explains everything:
- drdds `DrDDSChannel("/NAV_CMD")` → DDS topic `/NAV_CMD` → doesn't match basic_server
- rclpy `create_publisher(NavCmd, "/NAV_CMD")` → DDS topic `rt/NAV_CMD` → matches!
- `matched_count=1` from DrDDSChannel was matching a different drdds subscriber, not basic_server

**Confirmed by Codex (GPT-5.4)** with references to rmw_fastrtps Foxy source code:
- Topic prefix: `rmw_fastrtps_shared_cpp/src/namespace_prefix.cpp`
- Type name: `<pkg>::msg::dds_::<Name>_` (e.g. `drdds::msg::dds_::NavCmd_`)
- QoS: RELIABLE, VOLATILE, KEEP_LAST(10)

---

## Finding no.5: Raw FastDDS Publisher Works — No ROS Needed (2026-04-12)

**The final solution.** Created `NavCmdPub` — a C++ NativeModule that uses the FastDDS 2.14
API directly (NOT drdds wrapper, NOT rclpy) to publish velocity commands.

Key configuration:
- **DDS topic**: `rt/NAV_CMD` (ROS2 convention, matches basic_server)
- **DDS type name**: `drdds::msg::dds_::NavCmd_` (overridden via `setName()`)
- **QoS**: RELIABLE, VOLATILE, KEEP_LAST(10) (matches rmw_fastrtps defaults)
- **Participant**: `initialPeersList` pointing to `10.21.33.103` (AOS on the subnet
  where basic_server announces DDS discovery)
- **Host route**: `10.21.33.103/32 via 10.21.31.103` on NOS (Codex suggestion)

The NavCmdPub subscribes to `cmd_vel` on LCM (from CmdVelMux) and publishes to
`rt/NAV_CMD` via FastDDS at 10Hz. `matched_count=1` with AOS basic_server in <1 second.

**WASD teleop verified working.** Robot moves with keyboard control from dimos-viewer.

### DDS Subnet Discovery (Codex-assisted)

AOS has three Ethernet interfaces:
- `eth0`: 10.21.33.103 (AOS↔GOS subnet — DDS discovery here)
- `eth1`: 10.21.32.103
- `eth2`: 10.21.31.103 (AOS↔NOS subnet — no DDS discovery)

`basic_server` announces DDS metatraffic on `127.0.0.1 + 10.21.33.103` only.
NOS (10.21.31.106) can't reach 10.21.33.103 without a host route.

Fix: `sudo ip route add 10.21.33.103/32 via 10.21.31.103` on NOS.
This lets NOS's FastDDS participant discover basic_server via the 10.21.33.x subnet.

---

## Final Architecture

```
NOS (10.21.31.106) — everything native, no Docker, no ROS
├── drdds_recv (host service)
│   └── Reads /LIDAR/POINTS + /IMU via drdds SHM from rsdriver/yesense
│   └── Writes to POSIX SHM in /dev/shm
│
├── DrddsLidarBridge (C++ NativeModule, nix)
│   └── Reads POSIX SHM → publishes lidar + IMU to LCM
│   └── Ring remapping (192→64) + time relativization in C++ hot path
│
├── AriseSLAM (C++ NativeModule, nix)
│   └── LCM raw_points + imu → registered_scan + odometry
│
├── SmartNav (C++ + Python NativeModules, nix)
│   ├── TerrainAnalysis (C++) → terrain_map
│   ├── LocalPlanner (C++) → path
│   ├── PathFollower (C++) → nav_cmd_vel
│   ├── SimplePlanner (Python) → goal_path
│   ├── PGO (Python) → corrected_odometry, global_map
│   ├── CmdVelMux (Python) → cmd_vel
│   └── ClickToGoal (Python) → waypoints
│
├── NavCmdPub (C++ NativeModule, cmake)
│   └── LCM cmd_vel → FastDDS rt/NAV_CMD → AOS basic_server
│
├── M20Connection (Python)
│   └── Camera (RTSP), robot state (UDP heartbeat, gait, mode)
│
└── RerunBridge + WebsocketVis
    └── dimos-viewer with WASD teleop

AOS (10.21.31.103) — stock firmware, no modifications
└── basic_server → ctrlmcu → motors
```

## NOS Prerequisites After Reboot

```bash
sudo ip link set lo multicast on
sudo ip route add 224.0.0.0/4 dev lo
sudo ip route add 10.21.33.103/32 via 10.21.31.103
sudo mount --bind /var/opt/robot/data/nix /nix
```

## Files

| File | Purpose |
|------|---------|
| `dimos/robot/deeprobotics/m20/drdds_bridge/cpp/main.cpp` | DrddsLidarBridge: SHM → LCM |
| `dimos/robot/deeprobotics/m20/drdds_bridge/cpp/nav_cmd_pub.cpp` | NavCmdPub: LCM → FastDDS rt/NAV_CMD |
| `dimos/robot/deeprobotics/m20/drdds_bridge/module.py` | Python NativeModule wrappers |
| `dimos/robot/deeprobotics/m20/blueprints/rosnav/m20_smartnav_native.py` | The working blueprint |
| `dimos/robot/deeprobotics/m20/connection.py` | Camera + robot state |
| `dimos/robot/deeprobotics/m20/blueprints/rosnav/m20_rerun.py` | Rerun visualization config |
