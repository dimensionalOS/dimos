<!-- COMMENTED OUT IMAGES CAUSE THEY DON'T RENDER ON PYPI (fix in 24hrs) -->
<!-- ![Screenshot 2025-02-18 at 16-31-22 DimOS Terminal](/assets/dimos_terminal.png)

<div align="center">
  <table>
    <tr>
      <td width="80%">
        <img src="./assets/dimos_interface.gif" alt="dimOS interface" width="100%">
        <p align="center"><em>A simple two-shot PlanningAgent</em></p>
      </td>
      <td width="20%">
        <img src="./assets/simple_demo_small.gif" alt="3rd person POV" width="100%">
        <p align="center"><em>3rd person POV</em></p>
      </td>
    </tr>
  </table>
</div> -->

# The Dimensional Framework
*The universal framework for AI-native generalist robotics*

## What is Dimensional?

#### Warning: This is a pre-release version

Dimensional is an open-source framework for adding customized general intelligence to robots. DimOS allows AI agents to call tools/functions (skills), read sensor/state data directly, and use them to produce robust emergent behavior. DimOS is both specification based (use any programming language) and a python-first library that works well with (and without) [ROS](https://www.ros.org/). The python library comes with a rich set of integrations; spatial reasoners, planners, simulators (mujoco, Isaac Sim, etc.), robot state/action primitives, and more.

# How do I get started?

## Installation

#### Details / Requirements

- Linux (MacOS support is in beta, you're welcome to roughly follow these steps, which will run on MacOS)

```sh
sudo apt-get update
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev
# install uv for python
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"
# create & activate a virtualenv (needed for dimos)
uv venv && . .venv/bin/activate
# install dimos
uv pip install 'dimos[base,unitree]'
#
# NOTE!!! you're going to have an empty/black rerun window for a while the first time
#
# this needs to download the replay file (2.4gb), which will take a while the first time
dimos --replay run unitree-go2
```

<!-- command for testing pre launch: `GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" uv pip install 'dimos[unitree] @ git+ssh://git@github.com/dimensionalOS/dimos.git@jeff_readme_go2'` -->

#### Get it working on a real physical robot!

```sh
export ROBOT_IP=PUT_YOUR_IP_ADDR_HERE
dimos run unitree-go2
```

<!-- TOOD: get it working with keyboard controls -->

#### Get it working in an interactive simulation!

```bash
export DISPLAY=:1 # Or DISPLAY=:0 if getting GLFW/OpenGL X11 errors
# ignore the warp warnings
dimos --simulation run unitree-go2
# open http://localhost:7779/command-center in your browser to control the robot movement
```
<!-- FIXME switch to pygame -->
<!-- TODO: figure out why there is a Max retreis exceeded with url: /offer -->


#### Have it controlled by AI!

```bash
export OPENAI_API_KEY=<your private key>
dimos run unitree-go2-agentic
# open the following in a different terminal tab
humancli
```
<!-- TODO pre-launch: figure out libav.h264:no frame! -->
<!-- TODO pre-launch: figure out libav.h264:non-existing PPS 0 referenced -->

<!-- TODO: figure out (warning) ERROR:root:An error occurred: HTTPConnectionPool(host='10.0.0.152', port=8081): Max retries exceeded with url: /offer (Caused by NewConnectionError("HTTPConnection(host='10.0.0.152', port=8081): Failed to establish a new connection: [Errno 111] Connection refused"))
ERROR:root:An error occurred with the old method: Failed to receive SDP Answer: No response
-->

<!-- TODO: cuda install -->
<!-- TODO: issac sim -->
<!-- LATER: ROS -->


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

### Note: Many More Examples in the [Examples Folder](https://github.com/dimensionalOS/dimos/tree/main/examples)


# How does DimOS work conceptually?

There are several tools:
- Modules: parallel processing, several modules run. Modules are usually python classes but its possible to use C++, Rust, Typescript, etc to write a module.
- Streams: How modules communicate, a Pub / Sub system.
- Blueprints: a way to group modules together and define their connections to each other
- RPC: how one module can call a method on another module (arguments get serialized to JSON-like binary data)
- Skills: Pretty much an RPC, call but it can be called by an AI agent (they're tools for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools
