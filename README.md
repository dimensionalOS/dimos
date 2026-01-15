<div align="center">
    <img width="900" height="205" alt="banner_bordered_trimmed" src="https://github.com/user-attachments/assets/e7220790-eed9-44c2-8fab-d61a4c034828" /> </a>
    <h4 align="center">The Open-Source Framework for Robotic Intelligence</h4>

<br>

[![Discord](https://img.shields.io/discord/1341146487186391173?style=flat-square&logo=discord&logoColor=white&label=Discord&color=5865F2)](https://discord.gg/8m6HMArf) 
[![Stars](https://img.shields.io/github/stars/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/stargazers)
[![Forks](https://img.shields.io/github/forks/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/fork)
[![Contributors](https://img.shields.io/github/contributors/dimensionalOS/dimos?style=flat-square)](https://github.com/dimensionalOS/dimos/graphs/contributors)

<p align="center">
  <a href="#how-does-dimos-work-conceptually">Key Features</a> •
  <a href="#how-do-i-get-started">How To Use</a> •
  <a href="#contributing--building-from-source">Contributing</a> •
  <a href="#acknowledgments">Credits</a> •
  <a href="#license">License</a>
</p>

</div>

# What is Dimensional?
> \[!NOTE]
>
> **Active Beta: Expect Breaking Changes**

DimOS is both a specification based (any programming language) framework and a python-first library for controlling robots. DimOS works with (and without) [ROS](https://www.ros.org/), with design that enables AI agents to call tools/functions (skills), read sensor/state data directly, and generate complex emergent behaviors. 

The python library comes with a rich set of integrations; visualization, spatial reasoners, planners, simulators (mujoco, Isaac Sim, etc.), robot state/action primitives, and more.

# How do I get started?

### Installation

#### Details / Requirements

- Linux, tested on Ubuntu 22.04, 24.04
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

# OPTION 1: use without installing dimos
uvx --from 'dimos[base,unitree]' dimos --replay run unitree-go2

# OPTION 2: install dimos in a virtualenv
uv venv && . .venv/bin/activate
uv pip install 'dimos[base,unitree]'
dimos --replay run unitree-go2
```

<!-- command for testing pre launch: `GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" uv pip install 'dimos[unitree] @ git+ssh://git@github.com/dimensionalOS/dimos.git@dev'` -->

#### Get it working on a real physical robot!

```sh
export ROBOT_IP=PUT_YOUR_IP_ADDR_HERE
dimos run unitree-go2
```

#### Get it working in an interactive simulation!

```bash
export DISPLAY=:1 # Or DISPLAY=:0 if getting GLFW/OpenGL X11 errors
# ignore the warp warnings
dimos --simulation run unitree-go2
# open http://localhost:7779/command-center in your browser to control the robot movement
```

#### Have it controlled by AI!

```bash
export OPENAI_API_KEY=<your private key>
dimos run unitree-go2-agentic
# open the following in a different terminal tab to tell it where to go
# Warning!: make sure to watch the bot, this is a pre-release it will run into stuff
#           and get tangled/trapped
# ex: tell it to explore the room, then tell it to go to where it can see a door
humancli
```

# How do I use it as a library?


Simple camera activation (save this as a python file and run it):

```py
from dimos.core.blueprints import autoconnect
from dimos.hardware.camera.module import CameraModule

if __name__ == "__main__":
    autoconnect(
        CameraModule.blueprint()
    ).build().loop()
    print("Webcam pipeline running. Press Ctrl+C to stop.")
```

Write your own custom module:

```py
from dimos.core.blueprints import autoconnect
from dimos.core import In, Module, pSHMTransport
from dimos.core.core import rpc
from dimos.hardware.camera.module import CameraModule
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

### Note: Many More Examples in the [Examples Folder](./examples)


# How does DimOS work conceptually?

There are several tools:
- [Modules](/docs/concepts/modules.md): The building blocks of DimOS, modules run in parallel and are defined in python as classes.
- [Streams](/docs/api/sensor_streams/index.md): How modules communicate, a Pub / Sub system.
- [Blueprints](/docs/concepts/blueprints.md): a way to group modules together and define their connections to each other
- [RPC](/docs/concepts/blueprints.md#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data)
- [Skills](/docs/concepts/blueprints.md#defining-skills): Pretty much an RPC, call but it can be called by an AI agent (they're tools for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools

## Contributing / Building From Source

For development, we optimize for flexibility—whether you love Docker, Nix, or have nothing but **notepad.exe** and a dream, you’re good to go. Open up the [Development Guide](/docs/development/README.md) to see the extra steps for setting up development environments.

We welcome contributions! See our [Bounty List](https://docs.google.com/spreadsheets/d/1tzYTPvhO7Lou21cU6avSWTQOhACl5H8trSvhtYtsk8U/edit?usp=sharing) for open requests for contributions. If you would like to suggest a feature or sponsor a bounty, open an issue.

# Acknowledgments

Huge thanks to!
- The Roboverse Community and their unitree-specific help. Check out their [Discord](https://discord.gg/HEXNMCNhEh).
- @abizovnuralem for his work on the [Unitree Go2 ROS2 SDK](https://github.com/abizovnuralem/go2_ros2_sdk) we integrate with for DimOS.
- @legion1581 for his work on the [Unitree Go2 WebRTC Connect](https://github.com/legion1581/go2_webrtc_connect) from which we've pulled the ```Go2WebRTCConnection``` class and other types for seamless WebRTC-only integration with DimOS.
- @tfoldi for the webrtc_req integration via Unitree Go2 ROS2 SDK, which allows for seamless usage of Unitree WebRTC control primitives with DimOS.

# License

DimOS is licensed under the Apache License, Version 2.0. And will always be free and open source.
