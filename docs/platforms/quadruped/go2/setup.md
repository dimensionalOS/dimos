# Unitree Go2 — Setup

Full autonomous navigation, mapping, and agentic control on a real Go2 — no ROS required.

## Requirements

- Unitree Go2 Pro or Air (stock firmware 1.1.7+, no jailbreak needed)
- Ubuntu 22.04/24.04 with CUDA GPU (recommended), or macOS (experimental)
- Python 3.12

## Install

First, install system dependencies for your platform:
- [Ubuntu](/docs/installation/ubuntu.md)
- [macOS](/docs/installation/osx.md)
- [Nix](/docs/installation/nix.md)

Then install DimOS:

```bash
uv venv --python "3.12"
source .venv/bin/activate
uv pip install 'dimos[base,unitree]'
```

## Run on Your Go2

### First-time setup, connecting to wifi, finding robot IP

Use `dimos go2tool` to provision wifi and find the robot's IP. Skip if the robot is already on your network and you know its IP.

1. Power on the Go2 — it advertises over BLE immediately.

2. Provision wifi (one-time per network):

optionally use discover to make sure robot is detected

```bash
dimos go2tool discover
```

configure wifi

```bash
dimos go2tool connect-wifi --ssid <wifi> --password <password>
```

Scans BLE and connects to the only robot it finds, or prompts you to pick if there are several.

3. Find the robot's IP:

```bash
dimos go2tool discover
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

2. Built-in obstacle avoidance is on. (DimOS handles path planning, but the onboard obstacle avoidance provides an extra safety layer around tight spots)

### Ready to run DimOS

```bash
export ROBOT_IP=<YOUR_GO2_IP>
dimos run unitree-go2
```

That's it. DimOS connects via WebRTC (no jailbreak required), starts the full navigation stack, and opens the command center in your browser.

### What's Running

| Module | What It Does |
|--------|-------------|
| **GO2Connection** | WebRTC connection to the robot — streams LiDAR, video, odometry |
| **VoxelGridMapper** | Builds a 3D voxel map using column-carving (CUDA accelerated) |
| **CostMapper** | Converts 3D map → 2D costmap via terrain slope analysis |
| **ReplanningAStarPlanner** | Continuous A* path planning with dynamic replanning |
| **WavefrontFrontierExplorer** | Autonomous exploration of unmapped areas |
| **RerunBridge** | 3D visualization in browser |
| **WebsocketVis** | Command center at localhost:7779 |

### Send Goals

From the command center ([localhost:7779](http://localhost:7779)):
- Click on the map to set navigation goals
- Toggle autonomous exploration
- Monitor robot pose, costmap, and planned path

## Agentic Control

Natural language control with an LLM agent that understands physical space:

```bash
export OPENAI_API_KEY=<YOUR_KEY>
export ROBOT_IP=<YOUR_GO2_IP>
dimos run unitree-go2-agentic
```

Then use the human CLI to talk to the agent:

```bash
humancli
> explore the space
```

The agent subscribes to camera, LiDAR, and spatial memory streams — it sees what the robot sees.
