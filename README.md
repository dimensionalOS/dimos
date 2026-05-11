<div align="center">

<img width="1000" alt="banner_bordered_trimmed" src="https://github.com/user-attachments/assets/64f13b39-da06-4f58-add0-cfc44f04db4e" />

<h2>The Agentive Operating System for Physical Space</h2>

[![Discord](https://img.shields.io/discord/1341146487186391173?style=flat-square&logo=discord&logoColor=white&label=Discord&color=5865F2)](https://discord.gg/dimos)
[![Stars](https://img.shields.io/github/stars/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/stargazers)
[![Forks](https://img.shields.io/github/forks/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/fork)
[![Contributors](https://img.shields.io/github/contributors/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/graphs/contributors)
![Nix](https://img.shields.io/badge/Nix-flakes-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![NixOS](https://img.shields.io/badge/NixOS-supported-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![CUDA](https://img.shields.io/badge/CUDA-supported-76B900?style=flat-square&logo=nvidia&logoColor=white)
[![Docker](https://img.shields.io/badge/Docker-ready-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)

<a href="https://trendshift.io/repositories/23169" target="_blank"><img src="https://trendshift.io/api/badge/repositories/23169" alt="dimensionalOS%2Fdimos | Trendshift" style="width: 250px; height: 55px;" width="250" height="55"/></a>

<big><big>

[Hardware](#hardware) •
[Installation](#installation) •
[Agent CLI & MCP](#agent-cli-and-mcp) •
[Blueprints](#blueprints) •
[Development](#development)

⚠️ **Pre-Release Beta** ⚠️

</big></big>

</div>

# Intro

Dimensional is the modern operating system for generalist robotics. We are setting the next-generation SDK standard, integrating with the majority of robot manufacturers.

With a simple install and no ROS required, build physical applications entirely in python that run on any humanoid, quadruped, or drone.

Dimensional is agent native -- "vibecode" your robots in natural language and build (local & hosted) multi-agent systems that work seamlessly with your hardware. Agents run as native modules — subscribing to any embedded stream, from perception (lidar, camera) and spatial memory down to control loops and motor drivers.
<table>
  <tr>
    <td align="center" width="50%">
      <a href="docs/capabilities/navigation/native/index.md"><img src="assets/readme/navigation.gif" alt="Navigation" width="100%"></a>
    </td>
    <td align="center" width="50%">
      <img src="assets/readme/perception.png" alt="Perception" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <h3><a href="docs/capabilities/navigation/native/index.md">Navigation and Mapping</a></h3>
      SLAM, dynamic obstacle avoidance, route planning, and autonomous exploration — via both DimOS native and ROS<br><a href="https://x.com/stash_pomichter/status/2010471593806545367">Watch video</a>
    </td>
    <td align="center" width="50%">
      <h3>Perception</h3>
      Detectors, 3d projections, VLMs, Audio processing
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <a href="docs/capabilities/agents/readme.md"><img src="assets/readme/agentic_control.gif" alt="Agents" width="100%"></a>
    </td>
    <td align="center" width="50%">
      <img src="assets/readme/spatial_memory.gif" alt="Spatial Memory" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <h3><a href="docs/capabilities/agents/readme.md">Agentive Control, MCP</a></h3>
      "hey Robot, go find the kitchen"<br><a href="https://x.com/stash_pomichter/status/2015912688854200322">Watch video</a>
    </td>
    <td align="center" width="50%">
      <h3>Spatial Memory</a></h3>
      Spatio-temporal RAG, Dynamic memory, Object localization and permanence<br><a href="https://x.com/stash_pomichter/status/1980741077205414328">Watch video</a>
    </td>
  </tr>
</table>


# Hardware

<table>
  <tr>
    <td align="center" width="20%">
      <h3>Quadruped</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
    <td align="center" width="20%">
      <h3>Humanoid</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
    <td align="center" width="20%">
      <h3>Arm</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
    <td align="center" width="20%">
      <h3>Drone</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
    <td align="center" width="20%">
      <h3>Misc</h3>
      <img width="245" height="1" src="assets/readme/spacer.png">
    </td>
  </tr>

  <tr>
    <td align="center" width="20%">
      🟩 <a href="docs/platforms/quadruped/go2/index.md">Unitree Go2 pro/air</a><br>
      🟥 <a href="dimos/robot/unitree/b1">Unitree B1</a><br>
    </td>
    <td align="center" width="20%">
      🟨 <a href="docs/platforms/humanoid/g1/index.md">Unitree G1</a><br>
    </td>
    <td align="center" width="20%">
      🟨 <a href="docs/capabilities/manipulation/readme.md">Xarm</a><br>
      🟨 <a href="docs/capabilities/manipulation/readme.md">AgileX Piper</a><br>
    </td>
    <td align="center" width="20%">
      🟧 <a href="dimos/robot/drone/README.md">MAVLink</a><br>
      🟧 <a href="dimos/robot/drone/README.md">DJI Mavic</a><br>
    </td>
    <td align="center" width="20%">
      🟥 <a href="https://github.com/dimensionalOS/openFT-sensor">Force Torque Sensor</a><br>
    </td>
  </tr>
</table>
<br>
<div align="right">
🟩 stable 🟨 beta 🟧 alpha 🟥 experimental

</div>

> [!IMPORTANT]
> 🤖 Direct your favorite Agent (OpenClaw, Claude Code, etc.) to [AGENTS.md](AGENTS.md) and our [CLI and MCP](#agent-cli-and-mcp) interfaces to start building powerful Dimensional applications.

# Installation

## Interactive Install

```sh skip
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
```

> See [`scripts/install.sh --help`](scripts/install.sh) for non-interactive and advanced options.

## Manual System Install

To set up your system dependencies, follow one of these guides:

- 🟩 [Ubuntu 22.04 / 24.04](docs/installation/ubuntu.md)
- 🟩 [NixOS / General Linux](docs/installation/nix.md)
- 🟧 [macOS](docs/installation/osx.md)

> Full system requirements, tested configs, and dependency tiers: [docs/requirements.md](docs/requirements.md)

## Python Install

### Quickstart

```bash
uv venv --python "3.12"
source .venv/bin/activate
uv pip install 'dimos[unitree]'

# Replay a recorded quadruped session (no hardware needed)
# NOTE: First run will show a black rerun window while ~75 MB downloads from LFS
dimos --replay run unitree-go2
```

```bash
# Add perception (object detection, VLMs — heavy dependencies, needs to download GBs)
uv pip install 'dimos[unitree,perception]'

# Add simulation support
uv pip install 'dimos[unitree,sim]'

# Run quadruped in MuJoCo simulation
dimos --simulation run unitree-go2

# Run humanoid in simulation
dimos --simulation run unitree-g1-sim
```

```bash
# Control a real robot (Unitree quadruped over WebRTC)
export ROBOT_IP=<YOUR_ROBOT_IP>
dimos run unitree-go2
```

# Featured Runfiles

| Run command | What it does |
|-------------|-------------|
| `dimos --replay run unitree-go2` | Quadruped navigation replay — SLAM, costmap, A* planning |
| `dimos --replay --replay-db go2_bigoffice run unitree-go2-memory` | Quadruped temporal memory replay |
| `dimos --simulation run unitree-go2-agentic` | Quadruped agentic + MCP server in simulation |
| `dimos --simulation run unitree-g1` | Humanoid in MuJoCo simulation |
| `dimos --replay run drone-basic` | Drone video + telemetry replay |
| `dimos --replay run drone-agentic` | Drone + LLM agent with flight skills (replay) |
| `dimos run demo-camera` | Webcam demo — no hardware needed |
| `dimos run keyboard-teleop-xarm7` | Keyboard teleop with mock xArm7 (requires `dimos[manipulation]` extra) |
| `dimos --simulation run unitree-go2-agentic-ollama` | Quadruped agentic with local LLM (requires [Ollama](https://ollama.com) + `ollama serve`) |

> Full blueprint docs: [docs/usage/blueprints.md](docs/usage/blueprints.md)

# Agent CLI and MCP

The `dimos` CLI manages the full lifecycle — run blueprints, inspect state, interact with agents, and call skills via MCP.

```bash
dimos run unitree-go2-agentic --daemon   # Start in background
dimos status                              # Check what's running
dimos log -f                              # Follow logs
dimos agent-send "explore the room"       # Send agent a command
dimos mcp list-tools                      # List available MCP skills
dimos mcp call relative_move --arg forward=0.5  # Call a skill directly
dimos stop                                # Shut down
```

> Full CLI reference: [docs/usage/cli.md](docs/usage/cli.md)


# Usage

## Use DimOS as a Library

See below a simple robot connection module that sends streams of continuous `cmd_vel` to the robot and receives `color_image` to a simple `Listener` module. DimOS Modules are subsystems on a robot that communicate with other modules using standardized messages.

```py skip
import threading, time, numpy as np
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.sensor_msgs import Image, ImageFormat

class RobotConnection(Module):
    cmd_vel: In[Twist]
    color_image: Out[Image]

    @rpc
    def start(self):
        threading.Thread(target=self._image_loop, daemon=True).start()

    def _image_loop(self):
        while True:
            img = Image.from_numpy(
                np.zeros((120, 160, 3), np.uint8),
                format=ImageFormat.RGB,
                frame_id="camera_optical",
            )
            self.color_image.publish(img)
            time.sleep(0.2)

class Listener(Module):
    color_image: In[Image]

    @rpc
    def start(self):
        self.color_image.subscribe(lambda img: print(f"image {img.width}x{img.height}"))

if __name__ == "__main__":
    autoconnect(
        RobotConnection.blueprint(),
        Listener.blueprint(),
    ).build().loop()
```

## Blueprints

Blueprints are instructions for how to construct and wire modules. We compose them with
`autoconnect(...)`, which connects streams by `(name, type)` and returns a `Blueprint`.

Blueprints can be composed, remapped, and have transports overridden if `autoconnect()` fails due to conflicting variable names or `In[]` and `Out[]` message types.

A blueprint example that connects the image stream from a robot to an MCP-backed LLM agent for reasoning and action execution.
```py skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree.go2.connection import go2_connection
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer

blueprint = autoconnect(
    go2_connection(),
    McpServer.blueprint(),
    McpClient.blueprint(),
).transports({("color_image", Image): LCMTransport("/color_image", Image)})

# Run the blueprint
if __name__ == "__main__":
    blueprint.build().loop()
```

## Library API

- [Modules](docs/usage/modules.md)
- [LCM](docs/usage/lcm.md)
- [Blueprints](docs/usage/blueprints.md)
- [Transports](docs/usage/transports/index.md) — LCM, SHM, DDS, ROS 2
- [Data Streams](docs/usage/data_streams/README.md)
- [Configuration](docs/usage/configuration.md)
- [Visualization](docs/usage/visualization.md)

## Demos

<img src="assets/readme/dimos_demo.gif" alt="DimOS Demo" width="100%">

# Development

## Develop on DimOS

```sh skip
export GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/dimensionalOS/dimos.git
cd dimos

uv sync --extra all

# Run fast test suite
uv run pytest dimos
```


## Multi Language Support

Python is our glue and prototyping language, but we support many languages via LCM interop.

Check our language interop examples:
- [C++](examples/language-interop/cpp/)
- [Lua](examples/language-interop/lua/)
- [TypeScript](examples/language-interop/ts/)
