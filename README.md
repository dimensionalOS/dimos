
<div align="center">

<img width="1000" alt="banner_bordered_trimmed" src="https://github.com/user-attachments/assets/15283d94-ad95-42c9-abd5-6565a222a837" />

<h2>The Agentive Operating System for Generalist Robotics</h2>

[![Discord](https://img.shields.io/discord/1341146487186391173?style=flat-square&logo=discord&logoColor=white&label=Discord&color=5865F2)](https://discord.gg/dimos)
[![Stars](https://img.shields.io/github/stars/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/stargazers)
[![Forks](https://img.shields.io/github/forks/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/fork)
[![Contributors](https://img.shields.io/github/contributors/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/graphs/contributors)
![Nix](https://img.shields.io/badge/Nix-flakes-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![NixOS](https://img.shields.io/badge/NixOS-supported-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![CUDA](https://img.shields.io/badge/CUDA-supported-76B900?style=flat-square&logo=nvidia&logoColor=white)
[![Docker](https://img.shields.io/badge/Docker-ready-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)

<big><big>

[Hardware](#hardware) •
[Installation](#installation) •
[Development](#development) •
[Multi Language](#multi-language-support) •
[ROS](#ros-interop)

⚠️ **Alpha Pre-Release: Expect Breaking Changes** ⚠️

</big></big>

</div>

# Intro

Dimensional is the modern operating system for generalist robotics. We are setting the next-generation SDK standard, integrating with the majority of robot manufacturers.

With a simple install and no ROS required, build physical applications entirely in python that run on any humanoid, quadruped, or drone.

Dimensional is agent native -- "vibecode" your robots in natural language and build (local & hosted) multi-agent systems that work seamlessly with your hardware. Agents run as native modules — subscribing to any embedded stream, from perception (lidar, camera) and spatial memory down to control loops and motor drivers.
<table>
  <tr>
    <td align="center" width="50%">
      <a href="docs/capabilities/navigation/readme.md"><img src="assets/readme/navigation.gif" alt="Navigation" width="100%"></a>
    </td>
    <td align="center" width="50%">
      <a href="docs/capabilities/perception/readme.md"><img src="assets/readme/perception.png" alt="Perception" width="100%"></a>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <h3><a href="docs/capabilities/navigation/readme.md">Navigation and Mapping</a></h3>
      SLAM, dynamic obstacle avoidance, route planning, and autonomous exploration — via both DimOS native and ROS<br><a href="https://x.com/stash_pomichter/status/2010471593806545367">Watch video</a>
    </td>
    <td align="center" width="50%">
      <h3><a href="docs/capabilities/perception/readme.md">Perception</a></h3>
      Detectors, 3d projections, VLMs, Audio processing
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <a href="docs/capabilities/agents/readme.md"><img src="assets/readme/agentic_control.gif" alt="Agents" width="100%"></a>
    </td>
    <td align="center" width="50%">
      <img src="assets/readme/spatial_memory.gif" alt="Spatial Memory" width="100%"></a>
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
      🟨 <a href="docs/todo.md">Unitree G1</a><br>
    </td>
    <td align="center" width="20%">
      🟥 <a href="docs/todo.md">Xarm</a><br>
      🟥 <a href="docs/todo.md">AgileX Piper</a><br>
    </td>
    <td align="center" width="20%">
      🟥 <a href="dimos/robot/drone">Mavlink</a><br>
      🟥 <a href="dimos/robot/drone">DJI SDK</a><br>
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

# Installation

## System Install

To set up your system dependencies, follow one of these guides:

- 🟩 [Ubuntu 22.04 / 24.04](docs/installation/ubuntu.md)
- 🟩 [NixOS / General Linux](docs/installation/nix.md)
- 🟧 [macOS](docs/installation/osx.md)

## Python Install

### Quickstart

```bash
uv venv --python "3.12"
source .venv/bin/activate
uv pip install dimos[base,unitree]

# Replay a recorded Go2 session (no hardware needed)
# NOTE: First run will show a black rerun window while ~2.4 GB downloads from LFS
dimos --replay run unitree-go2
```

```bash
# Install with simulation support
uv pip install dimos[base,unitree,sim]

# Run Go2 in MuJoCo simulation
dimos --simulation run unitree-go2

# Run G1 humanoid in simulation
dimos --simulation run unitree-g1-sim
```

```bash
# Control a real robot (Unitree Go2 over WebRTC)
export ROBOT_IP=<YOUR_ROBOT_IP>
dimos run unitree-go2
```

# Usage

## Use DimOS as a Library

See below a simple robot connection module that sends streams of continuous `cmd_vel` to the robot and receives `color_image` to a simple `Listener` module. DimOS Modules are subsystems on a robot that communicate with other modules using standardized messages.

```py
import threading, time, numpy as np
from dimos.core import In, Module, Out, rpc, autoconnect
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

A blueprint example that connects the image stream from a robot to an LLM Agent for reasoning and action execution.
```py
from dimos.core import autoconnect, LCMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree.go2.connection import go2_connection
from dimos.agents.agent import agent

blueprint = autoconnect(
    go2_connection(),
    agent(),
).transports({("color_image", Image): LCMTransport("/color_image", Image)})

# Run the blueprint
if __name__ == "__main__":
    blueprint.build().loop()
```

## Library API

- [Modules](docs/usage/modules.md)
- [LCM](docs/usage/lcm.md)
- [Blueprints](docs/usage/blueprints.md)
- [Transports](docs/usage/transports/index.md)
- [Data Streams](docs/usage/data_streams/README.md)
- [Configuration](docs/usage/configuration.md)
- [Visualization](docs/usage/visualization.md)

## Demos

<img src="assets/readme/dimos_demo.gif" alt="DimOS Demo" width="100%">

# Development

## Develop on DimOS

```sh
export GIT_LFS_SKIP_SMUDGE=1
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

uv sync --all-extras --no-extra dds

# Run fast test suite
uv run pytest dimos
```

## Multi Language Support

Python is our glue and prototyping language, but we support many languages via LCM interop.

Check our language interop examples:
- [C++](examples/language-interop/cpp/)
- [Lua](examples/language-interop/lua/)
- [TypeScript](examples/language-interop/ts/)

## ROS interop

For researchers, we can talk to ROS directly via [ROS Transports](docs/usage/transports/index.md), or host dockerized ROS deployments as first-class DimOS modules, allowing you easy installation and portability
