# M20 Native Navigation Log

**Goal**: Run the entire M20 nav stack natively on NOS without Docker containers.
**Date started**: 2026-04-10
**Branch**: `feat/deeprobotics-m20-nav`

---

## Status: WORKING (except velocity commands to AOS)

Full native pipeline verified on NOS:
- DrddsLidarBridge (C++ NativeModule) → lidar + IMU via POSIX SHM → LCM
- AriseSLAM (C++ NativeModule) → registered_scan + odometry
- SmartNav (TerrainAnalysis + LocalPlanner + PathFollower + SimplePlanner + PGO + CmdVelMux + ClickToGoal)
- RerunBridge + WebsocketVis → dimos-viewer with points + camera
- WASD teleop reaches CmdVelMux

**Blocker**: `/NAV_CMD` velocity commands can't reach AOS motor controller.

---

## Finding no.1: Nix on Kernel 5.10 — fchmodat2 Workaround (2026-04-10)

See `06-simplify-remove-container/nix-arm64-kernel510-workaround.md` for full details.

Nix's coreutils (glibc 2.42) uses `fchmodat2` syscall which kernel 5.10 doesn't have. 
Fixed by overriding `unpackPhase` and `fixupPhase` to use host coreutils (`/usr/bin/cp`, `/usr/bin/chmod`).
Also requires `/nix` bind-mounted to ext4 (root is overlayfs).

---

## Finding no.2: drdds /NAV_CMD is Local-Only — Can't Cross NOS→AOS (2026-04-10)

**Current blocker.**

The M20's motor controller (`ctrlmcu`, PID 1673) runs on AOS (10.21.31.103) and subscribes to `/NAV_CMD` via drdds. Our `nav_cmd_pub` runs on NOS (10.21.31.106) and publishes via drdds. `matched_count` is always 0.

**Root cause**: drdds uses FastDDS SHM transport by default. SHM is local-only — it cannot cross machine boundaries. The QoS config (`/opt/drdds/dr_qos/drqos.xml`) has NavCmd reader/writer profiles but the participant profile has no explicit transport configuration (no UDP peer lists, no multicast setup).

**Why it worked before**: The M20's own `height_map_nav` planner runs ON AOS (same machine as `ctrlmcu`), so SHM works. We're the first to try publishing `/NAV_CMD` from NOS.

**Why the Docker version worked**: When dimos ran inside a Docker container, it used ROS2 Humble's FastDDS with a configured `fastdds.xml` that had UDP transport. The ROS2 stack handled cross-board DDS discovery properly.

**Options**:
1. Run a `nav_cmd_pub` bridge ON AOS that receives commands over TCP/UDP from NOS
2. Configure drdds to use UDP transport for cross-board `/NAV_CMD` discovery
3. Use the M20's UDP axis command protocol (only works in Regular Mode, not Navigation Mode)

**Investigation results for Option 2** (2026-04-10):

Tried multiple approaches to configure drdds UDP transport:
1. Modified `/opt/drdds/dr_qos/drqos.xml` on BOTH NOS and AOS with `initialPeersList` pointing to each other — no effect. `DrDDSManager::Init` appears to create its own participant that ignores the XML participant profile.
2. Set `FASTRTPS_DEFAULT_PROFILES_FILE` env var — no effect. drdds doesn't use FastDDS's default XML mechanism.
3. Changed `enable_config=true` in `DrDDSManager::Init` — loads the QoS profiles (data_writer/reader) but still doesn't apply participant-level transport config.
4. tcpdump confirms AOS sends UDP discovery announcements TO NOS, but NOS's drdds participant doesn't respond — it's SHM-only.

**Conclusion**: The drdds SDK hardcodes SHM-only transport in its participant creation. The XML config only controls QoS (reliability, deadlines) for data readers/writers, not the transport layer. Cross-board drdds communication appears to be unsupported by the SDK.

**Recommended fix**: Option 1 — deploy a tiny bridge on AOS:
- A small C++ process on AOS that listens on a TCP/UDP socket for velocity commands from NOS
- Publishes to `/NAV_CMD` locally via drdds (same-machine SHM works)
- M20Connection on NOS sends velocity commands over TCP/UDP instead of drdds
- This is essentially the reverse of how `drdds_recv` bridges lidar data

Alternatively, ask Deep Robotics if there's a way to enable UDP transport in drdds, or if the UDP protocol (Type=2, Cmd=21) works in Navigation Mode (the docs say it doesn't, but maybe newer firmware supports it).

---

## Finding no.3: DrDDSChannel Matches but Robot Doesn't Move (2026-04-11)

**Partial resolution of Finding no.2.** Using `DrDDSChannel` (pub+sub pair) instead of
`DrDDSPublisher` alone achieves `matched_count=1` with `basic_server` on AOS. Confirmed
NOT a self-match (matched_count=0 when services stopped, 1 when running).

Built a TCP bridge on AOS (`nav_cmd_bridge.cpp`):
- NOS sends 12-byte velocity (3x float32) over TCP to AOS port 9740
- Bridge publishes to `/NAV_CMD` via DrDDSChannel locally on AOS
- Data confirmed received and published (bridge logs show velocity values)

**But the robot doesn't move.** Despite `matched_count=1` and correct message format,
`/NAV_CMD` velocity commands have no effect on locomotion.

**Key insight from investigation logs:** The ONLY time `/NAV_CMD` successfully moved the
robot was via `mac_bridge.py` on GOS using **rclpy** (`self._node.create_publisher(NavCmd,
"/NAV_CMD", qos)`), NOT via drdds's `DrDDSPublisher` or `DrDDSChannel`. ROS2's
rmw_fastrtps creates DDS entities differently from drdds's wrapper — the subscriber
(likely in `basic_server`) may only accept messages from ROS2-created publishers.

**rl_deploy is the balance controller** — stopping it causes the robot to collapse.
It must remain running. It does NOT subscribe to `/NAV_CMD` directly.

**UDP velocity works** — `M20Protocol.send_velocity()` in Regular Mode moves the robot.
The motor controller is functional; the issue is purely in the `/NAV_CMD` DDS path.

**Next steps:**
1. Ask Jeff/Deep Robotics: does `/NAV_CMD` require a ROS2 publisher (via rmw_fastrtps)
   rather than a raw drdds publisher? The mac_bridge used rclpy and it worked.
2. If so, we need a lightweight rclpy publisher somewhere (GOS has Foxy, NOS has Foxy
   in /opt/ros/foxy). Could be a tiny Python script on AOS using rclpy.
3. Alternative: investigate if `basic_server` forwards `/NAV_CMD` to ctrlmcu or if
   it needs height_map_nav as an intermediary.
