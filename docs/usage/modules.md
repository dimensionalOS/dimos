# DimOS Modules

Modules are subsystems on a robot that operate autonomously and communicate with other subsystems using standardized messages.

Some examples of modules are:

- Webcam (outputs image)
- Navigation (inputs a map and a target, outputs a path)
- Detection (takes an image and a vision model like YOLO, outputs a stream of detections)

Below is an example of a structure for controlling a robot. Black blocks represent modules, and colored lines are connections and message types. It's okay if this doesn't make sense now. It will by the end of this document.

> **Prerequisite:** Blueprint visualization (both SVG export and the Rerun Graph tab) requires Graphviz:
> ```bash
> sudo apt install graphviz   # Ubuntu/Debian
> brew install graphviz        # macOS
> ```

```python output=assets/go2_nav.svg
from dimos.core.introspection import to_svg
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import nav
to_svg(nav, "assets/go2_nav.svg")
```

<!--Result:-->
![output](assets/go2_nav.svg)

## Camera Module

Let's learn how to build stuff like the above, starting with a simple camera module.

```python session=camera_module_demo output=assets/camera_module.svg
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.core.introspection import to_svg
to_svg(CameraModule.module_info(), "assets/camera_module.svg")
```

<!--Result:-->
![output](assets/camera_module.svg)

We can also print Module I/O quickly to the console via the `.io()` call. We will do this from now on.

```python session=camera_module_demo ansi=false
print(CameraModule.io())
```

<!--Result:-->
```
┌┴─────────────┐
│ CameraModule │
└┬─────────────┘
 ├─ color_image: Image
 ├─ camera_info: CameraInfo
 │
 ├─ RPC start()
 ├─ RPC stop()
 │
 ├─ Skill take_a_picture
```

We can see that the camera module outputs two streams:

- `color_image` with [sensor_msgs.Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) type
- `camera_info` with [sensor_msgs.CameraInfo](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) type

It offers two RPC calls: `start()` and `stop()` (lifecycle methods).

It also exposes an agentic [skill](/docs/usage/blueprints.md#defining-skills) called `take_a_picture` (more on skills in the Blueprints guide).

We can start this module and explore the output of its streams in real time (this will use your webcam).

```python session=camera_module_demo ansi=false
import time

camera = CameraModule()
camera.start()
# Now this module runs in our main loop in a thread. We can observe its outputs.

print(camera.color_image)

camera.color_image.subscribe(print)
time.sleep(0.5)
camera.stop()
```

<!--Result:-->
```
Out color_image[Image] @ CameraModule
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:16)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:16)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:17)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:17)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:17)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:17)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:17)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:17)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:17)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2025-12-31 15:54:17)
```


## Connecting modules

Let's load a standard 2D detector module and hook it up to a camera.

```python ansi=false session=detection_module
from dimos.perception.detection.module2D import Detection2DModule, Config
print(Detection2DModule.io())
```

<!--Result:-->
```
 ├─ image: Image
┌┴──────────────────┐
│ Detection2DModule │
└┬──────────────────┘
 ├─ detections: Detection2DArray
 ├─ annotations: ImageAnnotations
 ├─ detected_image_0: Image
 ├─ detected_image_1: Image
 ├─ detected_image_2: Image
 │
 ├─ RPC set_transport(stream_name: str, transport: Transport) -> bool
 ├─ RPC start() -> None
 ├─ RPC stop() -> None
```

<!-- TODO: add easy way to print config -->

Looks like the detector just needs an image input and outputs some sort of detection and annotation messages. Let's connect it to a camera.

```python ansi=false
import time
from dimos.perception.detection.module2D import Detection2DModule, Config
from dimos.hardware.sensors.camera.module import CameraModule

camera = CameraModule()
detector = Detection2DModule()

detector.image.connect(camera.color_image)

camera.start()
detector.start()

detector.detections.subscribe(print)
time.sleep(3)
detector.stop()
camera.stop()
```

<!--Result:-->
```
Detection(Person(1))
Detection(Person(1))
Detection(Person(1))
Detection(Person(1))
```

## Distributed Execution

As we build module structures, we'll quickly want to utilize all cores on the machine (which Python doesn't allow as a single process) and potentially distribute modules across machines or even the internet.

For this, we use `dimos.core` and DimOS transport protocols.

Defining message exchange protocols and message types also gives us the ability to write models in faster languages.

## Restarting a module

While iterating on a module it's often convenient to edit its source file
and pick up the changes without tearing down the whole coordinator. The
`restart_module` call stops a single deployed module, reloads its source
via `importlib.reload`, then redeploys it onto a fresh worker process while
keeping its stream transports and reconnecting any other modules that held
a reference to it.

```python
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import GlobalConfig
from dimos.hardware.sensors.camera.module import CameraModule

coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
coordinator.start()
coordinator.load_module(CameraModule)

# ... edit CameraModule source on disk ...

coordinator.restart_module(CameraModule)
```

## Async modules (lock-free state)

Modules contain a per-instance asyncio loop on a daemon thread (`self._loop`). It is possible to write modules using only `async def` methods so that everything runs on the same thread and you don't need to use locks. The module's auto-bound input handlers, async `@rpc` methods, and `process_observable` callbacks all run on `self._loop`, and each handler subscription is serialized through a dedicated dispatcher task.

### Auto-bound input handlers

For every declared `x: In[T]`, if the module defines `async def handle_x(self, msg: T)`, the handler is automatically subscribed at `start()` and dispatched onto `self._loop`. Subscriptions are cleaned up at `stop()`.

```python
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist


class MovementManager(Module):
    clicked_point: In[PointStamped]
    nav_cmd_vel: In[Twist]
    tele_cmd_vel: In[Twist]

    cmd_vel: Out[Twist]
    goal: Out[PointStamped]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # No lock needed. `_teleop_active` is only mutated on `self._loop`.
        self._teleop_active = False

    async def handle_clicked_point(self, msg: PointStamped) -> None:
        self.goal.publish(msg)

    async def handle_nav_cmd_vel(self, msg: Twist) -> None:
        if not self._teleop_active:
            self.cmd_vel.publish(msg)

    async def handle_tele_cmd_vel(self, msg: Twist) -> None:
        self._teleop_active = True
        self.cmd_vel.publish(msg)
```

Each handler runs in a per-handler dispatcher task on `self._loop`. Handlers are serialized: only one invocation of `handle_x` runs at a time. If messages arrive faster than the handler can process them, intermediate messages are dropped — only the most recent unprocessed message is kept (LATEST policy). The handler is guaranteed to eventually run with the most recently published value.

### Async `@rpc` methods

`@rpc` works on both sync and `async def` methods. When applied to an async method, the call site dispatches automatically:

- From another thread (the RPC dispatcher, sync test code, a sync `@rpc` on the same module), the call blocks until the coroutine completes on `self._loop`.
- From inside the loop (another async `@rpc`, a `handle_*`, or a `process_observable` callback), it returns the coroutine so the caller can `await` it.

```python
class NameModule(Module):
    @rpc
    async def say_hello(self, name: str) -> str:
        return f"Hello {name}, from {self._my_name}"

    @rpc
    async def set_my_name(self, new_name: str) -> None:
        self._my_name = new_name
```

Async and sync `@rpc` methods are interchangeable for cross-module linking. Both are discovered via `Module.rpcs` and served through the same RPC machinery. A module ref or RPC client doesn't care whether the underlying method is sync or async.

When the consumer types a module ref using a Spec that declares `async def`, the proxy automatically exposes those methods as awaitables: `await self._name_module.say_hello(name)`.

```python
class NameSpec(Spec, Protocol):
    async def say_hello(self, name: str) -> str: ...
    async def set_my_name(self, new_name: str) -> None: ...


class StartModule(Module):
    _name_module: NameSpec

    async def code():
        await self._name_module.set_my_name("John")
        print(await self._name_module.say_hello("Bill"))
```

`NameModule` is async. But if you need to call it from a sync module, you just need to create a `SyncNameSpec`:

```python
class SyncNameSpec(Spec, Protocol):
    def say_hello(self, name: str) -> str: ...
    def set_my_name(self, new_name: str) -> None: ...
```

This will match with `NameModule`. You can call it synchronously from your module, but it will run in the `self._loop` async loop in the `NameModule` module.

The reverse is also true: you can call a sync module from async code.

### `spawn`: schedule a long-running coroutine from sync code

When you need to start a long-running async task from `start()` (e.g., a timer loop), use `self.spawn(coro)` instead of `asyncio.run_coroutine_threadsafe(coro, self._loop)`. The helper wires up a done-callback that surfaces unhandled exceptions to the module logger. bare `run_coroutine_threadsafe` silently stores the exception on the returned Future, where it disappears unless the user remembers to read `.result()`.

```python
@rpc
def start(self) -> None:
    super().start()
    self._timer_future = self.spawn(self._timer_loop())

async def _timer_loop(self) -> None:
    while True:
        await asyncio.sleep(1.0)
        ...

@rpc
def stop(self) -> None:
    if self._timer_future is not None:
        self._timer_future.cancel()
    super().stop()
```

### `process_observable`: async subscriptions to arbitrary observables

Sometimes you have rxpy observables which you need to run inside `self._loop`. You can do this with `self.process_observable(observable, async_handler)` .

```python
@rpc
def start(self) -> None:
    super().start()
    fast = self.foo.observable().pipe(ops.filter(lambda v: v > threshold))
    self.process_observable(fast, self._on_fast_foo)

async def _on_fast_foo(self, v: int) -> None:
    ...
```

### `main()`: combined setup/teardown

When a module owns a resource that needs construction at startup *and* explicit cleanup at shutdown, define `async def main(self)` as an **async generator with exactly one `yield`**. Code before `yield` runs at `start()`, code after `yield` runs at `stop()`.

```python
class PersonFollowSkillContainer(Module):
    async def main(self) -> AsyncIterator[None]:
        # setup
        self._vl_model = create("qwen")

        yield

        # teardown
        self._vl_model.stop()
```

Compared to splitting the same work across `__init__` / `start()` / `stop()`, `main()` keeps the construction-and-destruction of each resource visually adjacent.

## Blueprints

A blueprint is a predefined structure of interconnected modules. You can include blueprints or modules in new blueprints.

A basic Unitree Go2 blueprint looks like what we saw before.

```python  session=blueprints output=assets/go2_agentic.svg
from dimos.core.introspection import to_svg
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import agentic

to_svg(agentic, "assets/go2_agentic.svg")
```

<!--Result:-->
![output](assets/go2_agentic.svg)


To see more information on how to use Blueprints, see [Blueprints](/docs/usage/blueprints.md).
