# Unitree G1 — Getting Started

The Unitree G1 is a humanoid robot platform with full-body locomotion, arm gesture control, and agentic capabilities — no ROS required for basic operation.

## Requirements

- Unitree G1 (stock firmware)
- Ubuntu 22.04/24.04 with CUDA GPU (recommended), or macOS (experimental)
- Python 3.12
- ZED camera (mounted at chest height) for perception blueprints
- ROS 2 for navigation (the G1 navigation stack uses ROS nav)

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

## MuJoCo Simulation

No hardware? Start with simulation:

```bash
uv pip install 'dimos[base,unitree,sim]'
dimos --simulation run unitree-g1-basic-sim
```

This runs the G1 in MuJoCo with the native A* navigation stack — same blueprint structure, simulated robot. Opens the command center at [localhost:7779](http://localhost:7779) with Rerun 3D visualization.

## Run on Your G1

```bash
export ROBOT_IP=<YOUR_G1_IP>
dimos run unitree-g1-basic
```

DimOS connects via WebRTC, starts the ROS navigation stack, and opens the command center.

### What's Running

| Module | What It Does |
|--------|-------------|
| **G1Connection** | WebRTC connection to the robot — streams video, odometry |
| **Webcam** | ZED camera capture (stereo left, 15 fps) |
| **VoxelGridMapper** | Builds a 3D voxel map using column-carving (CUDA accelerated) |
| **CostMapper** | Converts 3D map → 2D costmap via terrain slope analysis |
| **WavefrontFrontierExplorer** | Autonomous exploration of unmapped areas |
| **RerunBridge** | 3D visualization in browser |
| **WebsocketVis** | Command center at localhost:7779 |

### Send Goals

From the command center ([localhost:7779](http://localhost:7779)):
- Click on the map to set navigation goals
- Toggle autonomous exploration
- Monitor robot pose, costmap, and planned path

## Agentic Control

Natural language control with an LLM agent that understands physical space and can command arm gestures:

```bash
export OPENAI_API_KEY=<YOUR_KEY>
export ROBOT_IP=<YOUR_G1_IP>
dimos run unitree-g1-agentic
```

Then use the human CLI:

```bash
humancli
> wave hello
> explore the room
> give me a high five
```

The agent subscribes to camera and spatial memory streams and has access to G1-specific skills including arm gestures and movement modes.

### Arm Gestures

The G1 agent can perform expressive arm gestures:

| Gesture | Description |
|---------|-------------|
| Handshake | Perform a handshake gesture with the right hand |
| HighFive | Give a high five with the right hand |
| Hug | Perform a hugging gesture with both arms |
| HighWave | Wave with the hand raised high |
| Clap | Clap hands together |
| FaceWave | Wave near the face level |
| LeftKiss | Blow a kiss with the left hand |
| ArmHeart | Make a heart shape with both arms overhead |
| RightHeart | Make a heart gesture with the right hand |
| HandsUp | Raise both hands up in the air |
| RightHandUp | Raise only the right hand up |
| Reject | Make a rejection or "no" gesture |
| CancelAction | Cancel any current arm action and return to neutral |

### Movement Modes

| Mode | Description |
|------|-------------|
| WalkMode | Normal walking |
| WalkControlWaist | Walking with waist control |
| RunMode | Running |

## Keyboard Teleop

Direct keyboard control via a pygame-based joystick:

```bash
export ROBOT_IP=<YOUR_G1_IP>
dimos run unitree-g1-joystick
```

## Available Blueprints

| Blueprint | Description |
|-----------|-------------|
| `unitree-g1-basic` | Connection + ROS navigation + visualization |
| `unitree-g1-basic-sim` | Simulation with A* navigation |
| `unitree-g1` | Navigation + perception + spatial memory |
| `unitree-g1-sim` | Simulation with perception + spatial memory |
| `unitree-g1-agentic` | Full stack with LLM agent and G1 skills |
| `unitree-g1-agentic-sim` | Agentic stack in simulation |
| `unitree-g1-full` | Agentic + SHM image transport + keyboard teleop |
| `unitree-g1-joystick` | Navigation + keyboard teleop |
| `unitree-g1-detection` | Navigation + YOLO person detection and tracking |
| `unitree-g1-shm` | Navigation + perception with shared memory image transport |
| `uintree-g1-primitive-no-nav` | Sensors + visualization only (no navigation, base for custom blueprints) |

### Blueprint Hierarchy

Blueprints compose incrementally:

```
primitive (sensors + vis)
├── basic (+ connection + navigation)
│   ├── basic-sim (sim connection + A* nav)
│   ├── joystick (+ keyboard teleop)
│   └── detection (+ YOLO person tracking)
├── perceptive (+ spatial memory + object tracking)
│   ├── sim (sim variant)
│   └── shm (+ shared memory transport)
└── agentic (+ LLM agent + G1 skills)
    ├── agentic-sim (sim variant)
    └── full (+ SHM + keyboard teleop)
```

## Deep Dive

- [Navigation Stack](/docs/capabilities/navigation/readme.md) — path planning and autonomous exploration
- [Visualization](/docs/usage/visualization.md) — Rerun, Foxglove, performance tuning
- [Data Streams](/docs/usage/data_streams) — RxPY streams, backpressure, quality filtering
- [Transports](/docs/usage/transports/index.md) — LCM, SHM, DDS
- [Blueprints](/docs/usage/blueprints.md) — composing modules
- [Agents](/docs/capabilities/agents/readme.md) — LLM agent framework
