<div align="center">
   <img width="1000" alt="banner_bordered_trimmed" src="https://github.com/user-attachments/assets/15283d94-ad95-42c9-abd5-6565a222a837" /> </a>
    <h4 align="center">The Open-Source Framework for Robotic Intelligence</h4>


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
  <a href="#how-does-dimos-work-conceptually">Key Features</a> •
  <a href="#how-do-i-get-started">How To Use</a> •
  <a href="#contributing--building-from-source">Contributing</a> • <a href="#license">License</a>
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

- Linux is supported, with tests being performed on Ubuntu 22.04 and 24.04
- MacOS support is in beta, you're welcome to try it *but expect inconsistent/flakey behavior (rather than errors/crashing)*
    - instead of the apt-get command below run: `brew install gnu-sed gcc portaudio git-lfs libjpeg-turbo python`

```sh
sudo apt-get update
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev
# install uv for python
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"

#
# NOTE!!! the first time, you're going to have an empty/black rerun window for a while
#
# the command needs to download the replay file (2.4gb), which takes a bit

# OPTION 1: install dimos in a virtualenv
uv venv && . .venv/bin/activate
uv pip install 'dimos[base,unitree]'
dimos --replay run unitree-go2

# OPTION 2: if you want to test out dimos without installing run:
uvx --from 'dimos[base,unitree]' dimos --replay run unitree-go2
```

<!-- command for testing pre launch: `GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" uv pip install 'dimos[unitree] @ git+ssh://git@github.com/dimensionalOS/dimos.git@dev'` -->

### Usage

#### Control a robot in a simulation (no robot required)

```bash
export DISPLAY=:1 # Or DISPLAY=:0 if getting GLFW/OpenGL X11 errors
# ignore the warp warnings
dimos --simulation run unitree-go2
# open http://localhost:7779/command-center in your browser to control the robot movement
```

#### Get it working on a physical robot!

```sh
export ROBOT_IP=PUT_YOUR_IP_ADDR_HERE
dimos run unitree-go2
```

#### Have it controlled by AI!

WARNING: This is a demo showing the **connection** between AI and robotic control -- not a demo of a super-intelligent AI. Be ready to physically prevent your robot from taking dumb physical actions.

```sh
export OPENAI_API_KEY=<your private key>
dimos run unitree-go2-agentic
```

After running that, open a new terminal and run the following to start giving instructions to the agent.
```sh
# activate the venv in this new terminal
source .venv/bin/activate

# Note: after running the next command, WAIT for the agent to connect
# (this will take a while the first time)
# then tell the agent "explore the room"
# then tell it to go to something, ex: "go to the door"
humancli
```

# How do I use it as a library?


Simple camera activation (save this as a python file and run it):

```py
from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule

if __name__ == "__main__":
    autoconnect(
        CameraModule.blueprint()
    ).build().loop()
```

Write your own custom module:

```py
from dimos.core.blueprints import autoconnect
from dimos.core import In, Out, Module
from dimos.core.core import rpc
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.msgs.sensor_msgs import Image

from reactivex.disposable import Disposable

class Listener(Module):
    # the CameraModule has an Out[Image] named "color_image"
    # this module will only receive those messages if
    # the names ("color_image") match, otherwise autoconnect
    # will not be able to connect one module to another
    color_image: In[Image] = None
    grayscale_image: Out[Image] = None

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.count = 0

    @rpc
    def start(self) -> None:
        super().start()
        def callback_func(img: Image) -> None:
            self.count += 1
            print(f"got frame {self.count}")
            print(f"img.data.shape: {img.data.shape}")
            self.grayscale_image.publish(img.to_grayscale())

        unsubscribe_func = self.color_image.subscribe(callback_func)
        # disposables will be called when the module is stopped
        self._disposables.add(Disposable(
            unsubscribe_func
        ))

    @rpc
    def stop(self) -> None:
        super().stop()

if __name__ == "__main__":
    autoconnect(
        Listener.blueprint(),
        CameraModule.blueprint(),
    ).build().loop()
```

#### Note: Many More Examples in the [Examples Folder](./examples)



### How does that example work?

- Every module represents one process: all modules run in parallel (python multiprocessing). Because of this **modules should only save/modify data on themselves** Don't mutate or share global vars inside a module.
- At the top of this module definition, the In/Out **streams** are defining a pub-sub system. This module expects someone somewhere to give it a color image. The module is going to publish a grayscale image.
- The `autoconnect` ties everything together:
  - The CameraModule has an output of `color_image`
  - The Listener has an input of `color_image`
  - Autoconnect puts them together, and checks that their types are compatible (both are of type `Image`)
- What about `@rpc`?
   - If you want a method to be called by another module (not just an internal method) then add the `@rpc` decorator AND make sure BOTH the arguments and return value of the method are json-serializable.
   - The start/stop methods always need to be an rpc because they are called externally.


# How does DimOS work conceptually?

There are several tools:
- [Modules](/docs/concepts/modules.md): The building blocks of DimOS, modules run in parallel and are defined in python as classes.
- [Streams](/docs/api/sensor_streams/index.md): How modules communicate, a Pub / Sub system.
- [Blueprints](/dimos/core/README_BLUEPRINTS.md): a way to group modules together and define their connections to each other.
- [RPC](/dimos/core/README_BLUEPRINTS.md#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- [Skills](/dimos/core/README_BLUEPRINTS.md#defining-skills): Pretty much an RPC, call but it can be called by an AI agent (they're tools for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools.

## Contributing / Building From Source

For development, we optimize for flexibility—whether you love Docker, Nix, or have nothing but **notepad.exe** and a dream, you’re good to go. Open up the [Development Guide](/docs/development/README.md) to see the extra steps for setting up development environments.

We welcome contributions! See our [Bounty List](https://docs.google.com/spreadsheets/d/1tzYTPvhO7Lou21cU6avSWTQOhACl5H8trSvhtYtsk8U/edit?usp=sharing) for open requests for contributions. If you would like to suggest a feature or sponsor a bounty, open an issue.

# License

DimOS is licensed under the Apache License, Version 2.0. And will always be free and open source.
