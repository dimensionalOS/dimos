# Unitree Go2 — Getting Started

The Unitree Go2 is DimOS's primary reference platform. Full autonomous navigation, mapping, and agentic control — no ROS required.

## Requirements

- Unitree Go2 Pro or Air (stock firmware 1.1.7+, no jailbreak needed)
- Ubuntu 22.04/24.04 with CUDA GPU (recommended), or macOS (experimental)
- Python 3.12

## Install

First, install system dependencies for your platform:
- [Ubuntu](../../../installation/ubuntu.md)
- [macOS](../../../installation/osx.md)
- [Nix](../../../installation/nix.md)

Then install DimOS:

```bash
uv venv --python "3.12"
source .venv/bin/activate
uv pip install dimos[base,unitree]
```

## Try It — No Hardware Needed

```bash
# Replay a recorded Go2 navigation session
# First run downloads ~2.4 GB of LiDAR/video data from LFS
dimos --replay run unitree-go2
```

Opens the command center at [localhost:7779](http://localhost:7779) with Rerun 3D visualization — watch the Go2 map and navigate an office in real time.

## Run on Your Go2

```bash
export ROBOT_IP=<YOUR_GO2_IP>
dimos run unitree-go2
```

That's it. DimOS connects via WebRTC (no jailbreak required), starts the full navigation stack, and opens the command center.

> **Tip:** Keep the Unitree built-in obstacle avoidance enabled on the robot for now. DimOS handles path planning, but the onboard obstacle avoidance provides an extra safety layer.

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

## MuJoCo Simulation

```bash
uv pip install dimos[base,unitree,sim]
dimos --simulation run unitree-go2
```

Full navigation stack in MuJoCo — same code, simulated robot.

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

## Available Blueprints

| Blueprint | Description |
|-----------|-------------|
| `unitree-go2-basic` | Connection + visualization (no navigation) |
| `unitree-go2` | Full navigation stack |
| `unitree-go2-agentic` | Navigation + LLM agent |
| `unitree-go2-agentic-ollama` | Agent with local Ollama models |
| `unitree-go2-agentic-mcp` | Agent with MCP tool access |
| `unitree-go2-spatial` | Navigation + spatial memory |
| `unitree-go2-detection` | Navigation + object detection |
| `unitree-go2-ros` | ROS 2 bridge mode |

## Deep Dive

- [Navigation Stack](../../../capabilities/navigation/native/index.md) — column-carving voxel mapping, costmap generation, A* planning
- [Visualization](../../../usage/visualization.md) — Rerun, Foxglove, performance tuning
- [Data Streams](../../../usage/data_streams/) — RxPY streams, backpressure, quality filtering
- [Transports](../../../usage/transports/index.md) — LCM, SHM, DDS
- [Blueprints](../../../usage/blueprints.md) — composing modules
