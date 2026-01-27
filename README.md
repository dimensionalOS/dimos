<div align="center">
   <img width="1000" alt="banner_bordered_trimmed" src="https://github.com/user-attachments/assets/15283d94-ad95-42c9-abd5-6565a222a837" /> </a>
    <h4 align="center">Program Atoms</h4>
    <h4 align="center">The Agentive Operating System for Generalist Robotics</h4>


<br>

[![Discord](https://img.shields.io/discord/1341146487186391173?style=flat-square&logo=discord&logoColor=white&label=Discord&color=5865F2)](https://discord.gg/dimos)
[![Stars](https://img.shields.io/github/stars/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/stargazers)
[![Forks](https://img.shields.io/github/forks/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/fork)
[![Contributors](https://img.shields.io/github/contributors/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/graphs/contributors)
<br>
![Nix](https://img.shields.io/badge/Nix-flakes-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![NixOS](https://img.shields.io/badge/NixOS-supported-5277C3?style=flat-square&logo=NixOS&logoColor=white)
![CUDA](https://img.shields.io/badge/CUDA-12.x-76B900?style=flat-square&logo=nvidia&logoColor=white)
[![Docker](https://img.shields.io/badge/Docker-ready-2496ED?style=flat-square&logo=docker&logoColor=white)](https://www.docker.com/)

<p align="center">
  <a href="#the-dimensional-framework">Features</a> •
  <a href="#installation">Installation</a> •
  <a href="#documentation">Documentation</a> •
  <a href="#development">Development</a> •
  <a href="#contributing">Contributing</a>
</p>

</div>

> \[!NOTE]
>
> ⚠️ **Alpha Pre-Release: Expect Breaking Changes** ⚠️

# The Dimensional Framework

Dimensional is the open-source, universal operating system for generalist robotics. On DimOS, developers
can design, build, and run physical ("dimensional") applications that run on any humanoid, quadruped,
drone, or wheeled embodiment.

**Programming physical robots is now as simple as programming digital software**: Composable, Modular, Repeatable.

Core Features:
- **Navigation:** Production navigation stack for any robot with lidar: SLAM, terrain analysis, collision
  avoidance, route planning, exploration.
- **Dashboard:** The DimOS command center gives developers the tooling to debug, visualize, compose, and
  test dimensional applications in real-time. Control your robot via waypoint, agent query, keyboard,
  VR, more.
- **Modules:** Standalone components (equivalent to ROS nodes) that publish and subscribe to typed
  In/Out streams that communicate over DimOS transports. The primary components of Dimensional.
- **Agents (experimental):** DimOS agents understand physical space, subscribe to sensor streams, and call
  **physical** tools. Emergence appears when agents have physical agency.
- **MCP (experimental):** Vibecode robots by giving your AI editor (Cursor, Claude Code) MCP access to run
  physical commands (move forward 1 meter, jump, etc.).
- **Manipulation (unreleased)** Classical (OMPL, IK, GraspGen), Agentive (TAMP), and VLA-native manipulation stack runs out-of-the-box on any DimOS supported arm embodiment.
- **Transport/Middleware:** DimOS native Python transport supports LCM, DDS, and SHM, plus ROS 2.
- **Robot integrations:** We integrate with the majority of hardware OEMs and are moving fast to cover
  them all. Supported and/or immediate roadmap:

  | Category | Platforms |
  | --- | --- |
  | Quadrupeds | Unitree Go2, Unitree B1, AGIBOT D1 Max/Pro, Dobot Rover |
  | Drones | DJI Mavic 2, Holybro x500 |
  | Humanoids | Unitree G1, Booster K1, AGIBOT X2, ABIBOT A2 |
  | Arms | OpenARMs, xARM 6/7, AgileX Piper, HighTorque Pantera |

# Getting Started

## Installation

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

### Test Installation

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

# The Dimensional Library

### Modules

Modules are subsystems on a robot that operate autonomously and communicate with other subsystems using standardized messages. See below a simple robot connection module that sends streams of continuous `cmd_vel` to the robot and recieves `color_image` to a simple `Listener` module.

```py
import threading, time, numpy as np
from dimos.core import In, Module, Out, rpc
from dimos.core.blueprints import autoconnect
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import ImageFormat

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

### Blueprints

Blueprints are how robots are constructed on Dimensional; instructions for how to construct and wire modules. You compose them with
`autoconnect(...)`, which connects streams by `(name, type)` and returns a `ModuleBlueprintSet`.

Blueprints can be composed, remapped, and have transports overridden if `autoconnect()` fails due to conflicting variable names or `In[]` and `Out[]` message types.

A blueprint example that connects the image stream from a robot to an LLM Agent for reasoning and action execution.
```py
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree.connection.go2 import go2_connection
from dimos.agents.agent import llm_agent

blueprint = autoconnect(
    go2_connection(),
    llm_agent(),
).transports({("color_image", Image): LCMTransport("/color_image", Image)})

# Run the blueprint
blueprint.build().loop()
```

# Development

```sh
GIT_LFS_SKIP_SMUDGE=1 git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos
```

Then pick one of two development paths:

Option A: Devcontainer
```sh
./bin/dev
```

Option B: Editable install with uv
```sh
uv venv && . .venv/bin/activate
uv pip install -e '.[base,dev]'
```

For system deps, Nix setups, and testing, see `/docs/development/README.md`.

### Monitoring & Debugging

DimOS comes with a number of monitoring tools:
- Run `lcmspy` to see how fast messages are being published on streams.
- Run `skillspy` to see how skills are being called, how long they are running, which are active, etc.
- Run `agentspy` to see the agent's status over time.
- If you suspect there is a bug within DimOS itself, you can enable extreme logging by prefixing the dimos command with `DIMOS_LOG_LEVEL=DEBUG RERUN_SAVE=1 `. Ex: `DIMOS_LOG_LEVEL=DEBUG RERUN_SAVE=1 dimos --replay run unitree-go2`


# Documentation

Concepts:
- [Modules](/docs/concepts/modules.md): The building blocks of DimOS, modules run in parallel and are singleton python classes.
- [Streams](/docs/api/sensor_streams/index.md): How modules communicate, a Pub / Sub system.
- [Blueprints](/dimos/core/README_BLUEPRINTS.md): a way to group modules together and define their connections to each other.
- [RPC](/dimos/core/README_BLUEPRINTS.md#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- [Skills](/dimos/core/README_BLUEPRINTS.md#defining-skills): An RPC function, except it can be called by an AI agent (a tool for an AI).

## Contributing

We welcome contributions! See our [Bounty List](https://docs.google.com/spreadsheets/d/1tzYTPvhO7Lou21cU6avSWTQOhACl5H8trSvhtYtsk8U/edit?usp=sharing) for open requests for contributions. If you would like to suggest a feature or sponsor a bounty, open an issue.
