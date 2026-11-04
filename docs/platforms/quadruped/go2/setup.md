# Unitree Go2 Setup

Full autonomous navigation and mapping on a real Go2. No ROS required.

## Requirements

- Unitree Go2 Pro or Air (stock firmware 1.1.7+, no jailbreak needed)
- Ubuntu 22.04/24.04 with CUDA GPU (recommended), or macOS (experimental)
- Python 3.12

## Install

Use the [official installer](/docs/installation/index.md), then follow its environment activation command:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

## Run on Your Go2

### First-time setup, connecting to wifi, finding robot IP

Use [go2tool](https://github.com/jeff-hykin/go2tool) to put the robot on wifi over Bluetooth and find its IP. Skip if the robot is already on your network and you know its IP.

1. Power on the Go2. It advertises over BLE immediately.

2. Optionally make sure the robot is detected:

```bash
uvx go2tool discover
```

3. Provision wifi (one-time per network):

```bash
uvx go2tool connect-wifi --ssid <wifi> --password <password>
```

Scans BLE and connects to the only robot it finds, or prompts you to pick if there are several.

4. Find the robot's IP:

```bash
uvx go2tool discover
```

Prints `SOURCE NAME IP MAC SERIAL` for every robot it sees over BLE and LAN. Export the IP:

```bash
export ROBOT_IP=<discovered_ip>
```

### Pre-flight checks

1. Robot is reachable and low latency `<10ms`, 0% packet loss
```bash
ping $ROBOT_IP
```

2. Built-in obstacle avoidance is on. (dimOS handles path planning, but the onboard obstacle avoidance provides an extra safety layer around tight spots). `unitree-go2` turns it off; add `--go2webrtc.obstacle-avoidance true` to keep it on.

### Ready to run dimOS

```bash
export ROBOT_IP=<YOUR_GO2_IP>
dimos run unitree-go2
```

That's it. dimOS connects via WebRTC (no jailbreak required), starts the full navigation stack, and opens the Rerun viewer.

### What's Running

| Module | What It Does |
|--------|-------------|
| **Go2WebRtc** | WebRTC connection to the robot (Rust native module). Streams LiDAR, video, odometry, joint state |
| **UrdfManagerNative** | Poses the Go2 URDF from the joint state onto tf |
| **RayTracingVoxelMap** | 3D voxel map with raycast clearing of dynamic objects |
| **MLSPlannerNative** | Multi-level surface global path planning |
| **LocalPlannerNative** / **TrajectoryFollowerNative** | Local planning over the live local map, and tracking the planned path |
| **MovementManager** | Merges clicked goals, navigation and teleop velocity into `cmd_vel` |
| **LocalMapRelocalization** | Relocalizes against a premap (off until `--localmaprelocalization.map-file <map>`) |
| **RerunBridge** | 3D visualization |

### Send Goals

Click on the map in the Rerun viewer to set a navigation goal.
