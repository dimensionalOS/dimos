# Unitree Go2 Setup

Full autonomous navigation, mapping, and agentic control on a real Go2. No ROS required.

## Requirements

- Unitree Go2 Pro or Air (stock firmware 1.1.7+, no jailbreak needed)
- Ubuntu 22.04/24.04/26.04 with CUDA GPU (recommended), or macOS (experimental)
- Python 3.12

## Install

Use the [official installer](/docs/installation/index.md), then follow its environment activation command:

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

## Run on Your Go2

### First-time setup, connecting to wifi, finding robot IP

Use `dimos go2tool` to provision wifi and find the robot's IP. Skip if the robot is already on your network and you know its IP.

1. Power on the Go2. It advertises over BLE immediately.

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

Prints `SOURCE NAME IP MAC SERIAL` for robots it sees over BLE and LAN. BLE rows
identify nearby robots but do not contain an IP. Match the full `SERIAL` on the
intended robot's BLE row (or its label) to a LAN row, then export that row's IP.
Do not select the first Unitree MAC or an unrelated LAN row when several robots
share the network.

```bash
export ROBOT_IP=<discovered_ip>
```

If the scan reports `No LAN robots found.` or no LAN match for your BLE serial,
the robot may still be reachable by IP. LAN discovery depends on UDP multicast
to `231.1.1.1:10131` and replies on UDP port `10134`. Check that the laptop and
robot are on the same LAN, that the firewall permits these packets, and that the
Wi-Fi network permits multicast between clients. Ask the network administrator
to check multicast filtering and client isolation, or retry on a network that
permits discovery. These are possible causes, not a diagnosis of the network.

You can isolate the LAN scan with `dimos go2tool discover --lan -t 30`. A longer
timeout cannot fix blocked multicast. DimOS currently has no direct IP-to-serial
lookup when discovery receives no reply. If you obtain an IP through the Unitree
app or network administrator, verify that it belongs to the intended serial
before setting `ROBOT_IP`. A successful ping alone does not identify the robot.

### Pre-flight checks

1. Robot is reachable and low latency `<10ms`, 0% packet loss
```bash
ping $ROBOT_IP
```

2. Built-in obstacle avoidance is on. (dimOS handles path planning, but the onboard obstacle avoidance provides an extra safety layer around tight spots)

### Ready to run dimOS

```bash
export ROBOT_IP=<YOUR_GO2_IP>
dimos run unitree-go2
```

That's it. dimOS connects via WebRTC (no jailbreak required), starts the full navigation stack, and opens the Rerun viewer.

### What's Running

| Module | What It Does |
|--------|-------------|
| **GO2Connection** | WebRTC connection to the robot. Streams LiDAR, video, odometry |
| **VoxelGridMapper** | Builds a 3D voxel map using column-carving (CUDA accelerated) |
| **CostMapper** | Converts 3D map → 2D costmap via terrain slope analysis |
| **ReplanningAStarPlanner** | Continuous A* path planning with dynamic replanning |
| **WavefrontFrontierExplorer** | Autonomous exploration of unmapped areas |
| **RerunBridge** | 3D visualization in browser |

### Send Goals

Click on the map in the Rerun viewer to set navigation goals.

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

The agent subscribes to camera, LiDAR, and spatial memory streams. It sees what the robot sees.
