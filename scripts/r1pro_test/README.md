# R1 Pro DiMOS Integration — Setup & Connection Guide

## Overview

This directory contains test scripts for validating DiMOS connectivity to the
Galaxea R1 Pro humanoid robot over ethernet. The robot runs ROS2 Humble on a
Jetson Orin (Ubuntu 22.04 / L4T). The laptop runs Ubuntu 24.04 with ROS2 Jazzy.

---

## Network Setup

### Physical Connection
- Connect laptop to robot via ethernet cable
- Robot ethernet port: `eth1` on the robot

### Robot IP (persistent after netplan config)
- Robot `eth1`: `192.168.123.150/24`
- Laptop ethernet (`enxf8e43bb7046c`): `192.168.123.100/24`

### Set laptop ethernet IP (if not already set)
```bash
sudo ip addr add 192.168.123.100/24 dev enxf8e43bb7046c
```

### SSH into robot
```bash
ssh nvidia@192.168.123.150
# password: nvidia
```

### Make robot IP persistent across reboots (already done)
Edit `/etc/netplan/50-cloud-init.yaml` on the robot, add `192.168.123.150/24`
to eth1 addresses:
```yaml
eth1:
  dhcp4: true
  addresses: [192.168.2.150/24, 192.168.123.150/24]
```
Then: `sudo netplan apply`

---

## Robot Startup Procedure

Run these commands on the robot via SSH every session:

```bash
# Step 1: Start CAN bus driver
bash ~/can.sh

# Step 2: Launch full robot stack (ros2_discovery, mobiman, hdas, tools)
cd ~/galaxea/install/startup_config/share/startup_config/script
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/
```

Wait ~30 seconds. The arms will open and close during HDAS initialization —
this is normal and confirms arm hardware is healthy.

```bash
# Step 3: Verify all topics are up (use --no-daemon, the daemon is unreliable)
source ~/galaxea/install/setup.bash
export ROS_DOMAIN_ID=41
ros2 topic list --no-daemon | grep hdas | head -5
# Expected: /hdas/feedback_arm_left, /hdas/feedback_arm_right, etc.
```

### Robot tmux sessions
| Session | Purpose |
|---|---|
| `ros_discovery` | FastDDS discovery server on port 11811 (for VR/WiFi, not needed for ethernet) |
| `mobiman` | Main motion control stack |
| `hdas` | Hardware abstraction — arms, chassis, torso, grippers |
| `tools` | Utilities |

Check session health: `tmux attach -t hdas` (Ctrl+B D to detach)

---

## Laptop Setup (every session)

```bash
cd ~/Downloads/dimos

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=41
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/scripts/r1pro_test/fastdds_r1pro.xml
```

Tip: add these to a shell script `scripts/r1pro_test/env.sh` and `source` it.

---

## Verification Tests

Run in order after startup:

```bash
# Test 1: Topic discovery (70 topics expected, ~10s)
python3 scripts/r1pro_test/test_01_topic_discovery.py

# Test 2: Read live arm joint data (safe, read-only)
python3 scripts/r1pro_test/test_02_read_arm_feedback.py

# Test 4: Arm hold command (safe, no movement expected)
python3 scripts/r1pro_test/test_04_arm_joint_command.py

# Test 3: Chassis movement (WARNING: robot moves ~5cm)
python3 scripts/r1pro_test/test_03_chassis_command.py
```

### Test status
| Test | Status | Notes |
|---|---|---|
| 01 topic discovery | PASS | 70 topics visible |
| 02 arm feedback | PASS | 7-joint positions/velocities/efforts streaming |
| 03 chassis command | Pending | Message type fixed (TwistStamped not Twist) |
| 04 arm hold command | Pending | DDS discovery wait added |
| 05 DiMOS ROS layer | Pending | Requires jeff/fix/rosnav3 branch |

---

## Key Topics

| Topic | Type | Direction |
|---|---|---|
| `/hdas/feedback_arm_left` | `sensor_msgs/JointState` | robot → laptop |
| `/hdas/feedback_arm_right` | `sensor_msgs/JointState` | robot → laptop |
| `/hdas/feedback_chassis` | `sensor_msgs/JointState` | robot → laptop |
| `/hdas/feedback_torso` | `sensor_msgs/JointState` | robot → laptop |
| `/motion_target/target_speed_chassis` | `geometry_msgs/TwistStamped` | laptop → robot |
| `/motion_target/target_joint_state_arm_left` | `sensor_msgs/JointState` | laptop → robot |
| `/motion_target/target_joint_state_arm_right` | `sensor_msgs/JointState` | laptop → robot |
| `/motion_target/target_joint_state_torso` | `sensor_msgs/JointState` | laptop → robot |
| `/motion_target/target_position_gripper_left` | `sensor_msgs/JointState` | laptop → robot |
| `/motion_target/target_position_gripper_right` | `sensor_msgs/JointState` | laptop → robot |

---

## Challenges & How We Solved Them

### 1. Finding the robot's IP
Robot had no known IP when connected via ethernet. Used `tcpdump` and `arp -a`
to discover it. Robot's `eth1` had no IPv4 assigned by default — manually
assigned `192.168.123.150/24` with `sudo ip addr add`, then made it persistent
via netplan.

### 2. ROS2 topic discovery failing across machines
**Root causes found (in order):**

**a) `ROS_LOCALHOST_ONLY=1` set in robot's `~/.bashrc`**
The robot was configured to only accept local DDS connections. Changed to
`ROS_LOCALHOST_ONLY=0` in `~/.bashrc` so tmux sessions (which source bashrc)
inherit the correct setting.

**b) CycloneDDS ↔ FastDDS EDP incompatibility**
Tried CycloneDDS on the laptop (ROS2 Jazzy default) thinking it would
interoperate with FastDDS on the robot (ROS2 Humble). Peer discovery (PDP)
worked — tcpdump confirmed packets flowing both ways — but endpoint discovery
(EDP) failed silently. Topics never appeared.

Fix: switch laptop to FastDDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) to match
the robot.

**c) FastDDS using wrong network interface on laptop**
Laptop has WiFi (`192.168.1.68`), ethernet (`192.168.123.100`), and Tailscale
(`100.78.x.x`). FastDDS multicast was going out the wrong interface.

Fix: `fastdds_r1pro.xml` — a FastDDS profile that binds to `192.168.123.100`
(ethernet) and sets `192.168.123.150:17650` as explicit unicast peer. This
bypasses multicast entirely.

**d) `interfaceWhiteList` renamed in FastDDS 3.x (Jazzy)**
The original XML used `<interfaceWhiteList>` which is FastDDS 2.x syntax.
FastDDS 3.x (shipped with Jazzy) renamed it to `allowlist`. The element was
silently ignored, so interface restriction never applied.

Fix: switched from transport-level interface restriction to locator-based
config (`metatrafficUnicastLocatorList`, `defaultUnicastLocatorList`,
`initialPeersList`) which works in both FastDDS 2.x and 3.x.

**e) Robot's FastDDS discovery server (port 11811)**
The robot runs a FastDDS discovery server via `start_discover_server.sh`.
Initially thought we needed to use `ROS_DISCOVERY_SERVER` to connect to it.
Investigation revealed the mobiman/hdas nodes do NOT connect to the discovery
server — they use standard multicast. The discovery server is for VR/WiFi
remote control only. Using `ROS_DISCOVERY_SERVER` on either side broke topic
visibility.

**f) HDAS process crashing (exit code -9)**
After restarting the robot stack, HDAS sometimes crashes on startup. Cause:
HDAS needs ~30 seconds to initialize and communicate with the arm motors over
CAN. If you check topics too early, only chassis topics appear. The arm
open/close cycle during boot confirms hardware is healthy. Always wait for this
before checking topics.

### 3. Wrong message types in test scripts
`test_03_chassis_command.py` was publishing `geometry_msgs/Twist` but the robot
expects `geometry_msgs/TwistStamped` on `/motion_target/target_speed_chassis`.
The robot silently ignored the wrong type. Fixed to use `TwistStamped` with
a populated header timestamp.

### 4. ROS2 daemon unreliable on robot
The ros2 daemon on the robot has slow discovery and often shows only 2 topics
(`/parameter_events`, `/rosout`) even when 70+ topics are active. Always use
`ros2 topic list --no-daemon` on the robot for accurate results.

---

## Robot Architecture Notes

- **Platform**: Jetson Orin (aarch64), Ubuntu 22.04, L4T (Jetpack)
- **ROS2**: Humble, FastDDS (rmw_fastrtps_cpp)
- **ROS_DOMAIN_ID**: 41
- **CAN bus**: arms and torso communicate via CAN (`can.sh` starts the driver)
- **HDAS**: Hardware abstraction layer — publishes all sensor feedback, receives
  all motion commands
- **mobiman**: Motion manager — handles kinematics, IK, safety limits
- **Custom message package**: `hdas_msg` — used for motor control, BMS, LED,
  version info. Standard ROS2 types used for joint states and geometry.

---

## Next Steps

- [ ] Complete test 03 (chassis movement) and test 04 (arm hold)
- [ ] Build `R1ProAdapter` in DiMOS following the G1 adapter pattern
- [ ] Implement `adapter.py` wrapping the ROS2 pub/sub layer
- [ ] Integrate with DiMOS control coordinator
- [ ] Test end-to-end DiMOS command → robot movement
