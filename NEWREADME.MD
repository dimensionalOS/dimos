
<div align="center">

![Dimensional](assets/dimensional-dark.svg#gh-dark-mode-only)
![Dimensional](assets/dimensional-light.svg#gh-light-mode-only)
<h2>Operating System for Generalist Robotics</h2>

<big>

[Features](#the-dimensional-framework) •
[Installation](#installation) •
[Documentation](#documentation) •
[Development](#development) •
[Contributing](#contributing)

</big>

</div>

# Intro

Our goal is to provide an easy to install and run modern framework for general robotics, you should be up and running in minutes.

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
      Livox Mid360, Zed
    </td>
  </tr>
</table>


# Hardware

<table>
  <tr>
    <td align="center" width="25%">
      <h3>Dog</h3>
      <img width="200" height="1" src="">
      <a href="docs/hardware/dog/go2/index.md">Unitree Go2 pro/air</a><br>
    </td>
    <td align="center" width="25%">
      <h3>Humanoid</h3>
      <img width="200" height="1" src="">
      <a href="docs/hardware/dog/go2/index.md">Unitree G1</a><br>
    </td>
    <td align="center" width="25%">
      <h3>Arm</h3>
      <img width="200" height="1" src="">
      <a href="docs/hardware/dog/go2/index.md">Xarm</a><br>
      <a href="docs/hardware/dog/go2/index.md">Piper</a><br>
    </td>
    <td align="center" width="25%">
      <h3>Drone</h3>
      <img width="200" height="1" src="">
      <a href="docs/hardware/dog/go2/index.md">Mavlink</a><br>
      <a href="docs/hardware/dog/go2/index.md">DJI SDK</a><br>
    </td>
  </tr>
</table>

# Installation

## Step 1 - System Install

to setup your system dependencies follow one of these:

- [NixOS / General Linux](docs/installation/nix.md)
- [macOS](docs/installation/osx.md)
- [Ubuntu 24.04](docs/installation/ubuntu.md)


## Step 2 - Python Install

We are ready to jump into Python

```sh
pip install dimos[your_platform]
dimos run your_platform_blueprint
```

## Step 3 - Profit!

![DimOS Demo](assets/readme/dimos_demo.gif)

# API

[Blueprints](docs/api/blueprints.md) •
[Modules](docs/api/modules.md) •
[Transports](docs/api/transports.md) •
[Data Streams](docs/api/data_streams/README.md) •
[LCM](docs/api/lcm.md) •
[Configuration](docs/api/configuration.md) •
[Visualization](docs/api/visualization.md)

# Multi Language Support

Python is our glue and prototyping language, but we support many languages, and ROS interop

check our [examples/language-interop](examples/language-interop/)

# ROS interop

for researchers we can talk to ros directly via [ROS Transports](docs/api/transports.md), or we can host ROS deployments as first class Dimos Modules
