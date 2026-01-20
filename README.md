<div align="center">
   <img width="1000" alt="banner_bordered_trimmed" src="https://github.com/user-attachments/assets/15283d94-ad95-42c9-abd5-6565a222a837" /> </a>
    <h4 align="center">The Agentive Operating System for Generalist Robotics</h4>


<br>

[![Discord](https://img.shields.io/discord/1341146487186391173?style=flat-square&logo=discord&logoColor=white&label=Discord&color=5865F2)](https://discord.gg/8m6HMArf)
[![Stars](https://img.shields.io/github/stars/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/stargazers)
[![Forks](https://img.shields.io/github/forks/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/fork)
[![Contributors](https://img.shields.io/github/contributors/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/graphs/contributors)
<br>
![Nix](https://img.shields.io/badge/Nix-flakes-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![NixOS](https://img.shields.io/badge/NixOS-supported-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![CUDA](https://img.shields.io/badge/CUDA-12.x-76B900?style=flat-square&logo=nvidia&logoColor=white)
[![Docker](https://img.shields.io/badge/Docker-ready-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)

<p align="center">
  <a href="#how-does-dimensional-work">Key Features</a> •
  <a href="#how-do-i-get-started">How To Use</a> •
  <a href="#contributing--building-from-source">Contributing</a> •
  <a href="#license">License</a>
</p>

</div>

> \[!NOTE]
>
> **Active Beta: Expect Breaking Changes**

# What is Dimensional?

DimOS is both a language-agnostic framework and a Python-first library for robot control. It has optional ROS integration and is designed to let AI agents invoke tools (skills), directly access sensor and state data, and generate complex emergent behaviors.

The python library comes with a rich set of integrations; visualizers, spatial reasoners, planners, simulators (mujoco, Isaac Sim, etc.), robot state/action primitives, and more.

# How do I get started?

### Installation

Supported/tested matrix:

| Platform | Status | Tested | Required System deps |
| --- | --- | --- | --- |
| Linux | supported | Ubuntu 22.04, 24.04 | See below |
| macOS | experimental beta | not CI-tested | `brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python` |

Note: macOS is usable but expect inconsistent/flaky behavior (rather than hard errors/crashes).

```sh
sudo apt-get update
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev
# install uv for python
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
```

Option 1: Install in a virtualenv

```sh

uv venv && . .venv/bin/activate
uv pip install 'dimos[base,unitree]'
# replay recorded data to test that the system is working
# IMPORTANT: First replay run will show a black rerun window while 2.4 GB downloads from LFS
dimos --replay run unitree-go2
```

Option 2: Run without installing

```sh
uvx --from 'dimos[base,unitree]' dimos --replay run unitree-go2
```

<!-- command for testing pre launch: `GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" uv pip install 'dimos[unitree] @ git+ssh://git@github.com/dimensionalOS/dimos.git@dev'` -->

### Dimensional Usage

#### Control a robot in a simulation (no robot required)


```sh
export DISPLAY=:1 # Or DISPLAY=:0 if getting GLFW/OpenGL X11 errors
# ignore the warp warnings
dimos --viewer-backend rerun-web --simulation run unitree-go2
```

#### Control a real robot (Unitree Go2 over WebRTC)

```sh
export ROBOT_IP=<YOUR_ROBOT_IP>
dimos --viewer-backend rerun-web run unitree-go2
```

After running dimOS open http://localhost:7779 to control robot movement.

#### Dimensional Agents

> \[!NOTE]
>
> **Experimental Beta: Potential unstoppable robot sentience**

```sh
export OPENAI_API_KEY=<your private key>
dimos --viewer-backend rerun-web run unitree-go2-agentic
```

After running that, open a new terminal and run the following to start giving instructions to the agent.
```sh
# activate the venv in this new terminal
source .venv/bin/activate

# then tell the agent "explore the room"
# then tell it to go to something, ex: "go to the door"
humancli
```

## Using DimOS as a Library

### Documentation & Concepts

If you you need more information on how DimOS works, check out the following links:

- [Modules](/docs/concepts/modules.md): The building blocks of DimOS, modules run in parallel and are singleton python classes.
- [Streams](/docs/api/sensor_streams/README.md): How modules communicate, a Pub / Sub system.
- [Blueprints](/docs/concepts/blueprints.md): a way to group modules together and define their connections to each other.
- [RPC](/docs/concepts/blueprints.md#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- [Skills](/docs/concepts/blueprints.md#defining-skills): An RPC function, except it can be called by an AI agent (a tool for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools.

### Simple DimOS Application

While `dimos run unitree-go2` is a good demo, we can recreate it with a few lines of code.

Save and run the following to get behavior like `dimos run unitree-go2`:

```python
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import autoconnect, basic, cost_mapper, voxel_mapper, replanning_a_star_planner, wavefront_frontier_explorer

nav = autoconnect(
    basic,
    voxel_mapper(voxel_size=0.3),
    cost_mapper(),
).global_config(simulation=True)

nav.build().loop()
```

## Contributing / Building From Source

We welcome contributions! Open up the [Development Guide](/docs/development/README.md) to see how to hack on DimOS and make PR's and  our [Bounty List](https://docs.google.com/spreadsheets/d/1tzYTPvhO7Lou21cU6avSWTQOhACl5H8trSvhtYtsk8U/edit?usp=sharing) for open requests for contributions. If you would like to suggest a feature or sponsor a bounty, open an issue.


### Monitoring & Debugging

In addition to rerun logging, DimOS comes with a number of monitoring tools:
- Run `lcmspy` to see how fast messages are being published on streams.
- Run `skillspy` to see how skills are being called, how long they are running, which are active, etc.
- Run `agentspy` to see the agent's status over time.
- If you suspect there is a bug within DimOS itself, you can enable extreme logging by prefixing the dimos command with `DIMOS_LOG_LEVEL=DEBUG RERUN_SAVE=1 `. Ex: `DIMOS_LOG_LEVEL=DEBUG RERUN_SAVE=1 dimos --replay run unitree-go2`

# License

DimOS is licensed under the Apache License, Version 2.0.
