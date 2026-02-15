
<div align="center">

![Dimensional](assets/dimensional-dark.svg#gh-dark-mode-only)
![Dimensional](assets/dimensional-light.svg#gh-light-mode-only)

<h2>Operating System for Generalist Robotics</h2>

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

</big></big>

</div>

# Intro

Our goal is to provide an easy-to-install, modern framework for general robotics. You should be up and running in minutes.

<table>
  <tr>
    <td align="center" width="50%">
      <img src="assets/readme/navigation.gif" alt="Navigation" width="100%">
    </td>
    <td align="center" width="50%">
      <img src="assets/readme/perception.png" alt="Det" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <h3>Navigation and Mapping</h3>
      ros and non ros<br><a href="#">Watch video</a>
    </td>
    <td align="center" width="50%">
      <h3>Perception</h3>
      bla bla perception<br><a href="#">Watch video</a>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="assets/readme/agents.png" alt="Agents" width="100%">
    </td>
    <td align="center" width="50%">
      <img src="assets/readme/lidar.gif" alt="Sensor Integrations" width="100%">
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <h3>Agentic Control, MCP</h3>
      "hey Claude, go to the kitchen"<br><a href="https://x.com/lesh_bla/status/2014904935088062503">Watch video</a>
    </td>
    <td align="center" width="50%">
      <h3>Sensor Integrations</h3>
      <a href="">Livox Mid360</a>, <a href="">Zed</a>
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
      🟩 <a href="docs/hardware/dog/go2/index.md">Unitree Go2 pro/air</a><br>
    </td>
    <td align="center" width="20%">
      🟧 <a href="docs/hardware/dog/go2/index.md">Unitree G1</a><br>
    </td>
    <td align="center" width="20%">
      🟨 <a href="docs/hardware/dog/go2/index.md">Xarm</a><br>
      🟨 <a href="docs/hardware/dog/go2/index.md">Piper</a><br>
    </td>
    <td align="center" width="20%">
      🟧 <a href="docs/hardware/dog/go2/index.md">Mavlink</a><br>
      🟥 <a href="docs/hardware/dog/go2/index.md">DJI SDK</a><br>
    </td>
    <td align="center" width="20%">
      🟨 <a href="docs/hardware/force-torque/index.md">Force Torque Sensor</a><br>
    </td>
  </tr>
</table>
<br>
<div align="right">
🟩 stable 🟨 beta 🟧 alpha 🟥 experimental

</div>

[New Hardware Integration Guide](docs/hardware/integration_guide.md)

# Installation

### Step 1: System Install

To set up your system dependencies, follow one of these guides:

- 🟩 [NixOS / General Linux](docs/installation/nix.md)
- 🟥 [macOS](docs/installation/osx.md)
- 🟩 [Ubuntu 24.04](docs/installation/ubuntu.md)
- 🟨 [Docker](docs/installation/docker.md)

### Step 2: Python Install

Now install the Python package.

Check the [hardware section](#hardware) above to choose the right `extras` and `blueprint` for your platform.

**Use DimOS as a library / UI**

```bash
pip install dimos[your_platform]
dimos run your_platform_blueprint
```

**Develop DimOS**

```bash
git clone https://github.com/dimensionalOS/dimos/ dimos
cd dimos
uv venv -p 3.12
source .venv/bin/activate
uv sync --all-extras
mypy dimos
pytest dimos
dimos --replay run unitree-go2
```

### Step 3 - Profit

<img src="assets/readme/dimos_demo.gif" alt="DimOS Demo" width="100%">

# Development

## API

- [Modules](docs/api/modules.md)
- [LCM](docs/api/lcm.md)
- [Blueprints](docs/api/blueprints.md)
- [Transports](docs/api/transports.md)
- [Data Streams](docs/api/data_streams/README.md)
- [Configuration](docs/api/configuration.md)
- [Visualization](docs/api/visualization.md)

## Multi Language Support

Python is our glue and prototyping language, but we support many languages via LCM interop.

Check our language interop examples:
- [C++](examples/language-interop/cpp/)
- [Lua](examples/language-interop/lua/)
- [TypeScript](examples/language-interop/ts/)

## ROS interop

For researchers, we can talk to ROS directly via [ROS Transports](docs/api/transports.md), or host dockerized ROS deployments as first-class DimOS modules, allowing you easy installation and portability
