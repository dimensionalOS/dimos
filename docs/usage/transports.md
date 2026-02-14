# Transports

Transports connect **module streams** across **process boundaries** and/or **networks**.

* **Module**: a running component (e.g., camera, mapping, nav).
* **Stream**: a unidirectional flow of messages owned by a module (one broadcaster → many receivers).
* **Topic**: the name/identifier used by a transport or pubsub backend.
* **Message**: payload carried on a stream (often `dimos.msgs.*`, but can be bytes / images / pointclouds / etc.).

Each edge in the graph is a **transported stream** (potentially different protocols). Each node is a **module**:

![go2_nav](assets/go2_nav.svg)

## What the transport layer guarantees (and what it doesn’t)

Modules **don’t** know or care *how* data moves. They just:

* emit messages (broadcast)
* subscribe to messages (receive)

A transport is responsible for the mechanics of delivery (IPC, sockets, Redis, ROS 2, etc.).

**Important:** delivery semantics depend on the backend:

* Some are **best-effort** (e.g., UDP multicast / LCM): loss can happen.
* Some can be **reliable** (e.g., TCP-backed, Redis, some DDS configs) but may add latency/backpressure.

So: treat the API as uniform, but pick a backend whose semantics match the task.

---

## Benchmarks

Quick view on performance of our pubsub backends:

```sh skip
python -m pytest -svm tool -k "not bytes" dimos/protocol/pubsub/benchmark/test_benchmark.py
```

![Benchmark results](assets/pubsub_benchmark.png)

---

## Abstraction layers

<details><summary>Pikchr</summary>

```pikchr output=assets/abstraction_layers.svg fold
color = white
fill = none
linewid = 0.5in
boxwid = 1.0in
boxht = 0.4in

# Boxes with labels
B: box "Blueprints" rad 10px
arrow
M: box "Modules" rad 5px
arrow
T: box "Transports" rad 5px
arrow
P: box "PubSub" rad 5px

# Descriptions below
text "robot configs" at B.s + (0.1, -0.2in)
text "camera, nav" at M.s + (0, -0.2in)
text "LCM, SHM, ROS" at T.s + (0, -0.2in)
text "pub/sub API" at P.s + (0, -0.2in)
```

</details>

<!--Result:-->
![output](assets/abstraction_layers.svg)

We’ll go through these layers top-down.

---

## Using transports with blueprints

See [Blueprints](blueprints.md) for the blueprint API.

From [`unitree/go2/blueprints/__init__.py`](/dimos/robot/unitree/go2/blueprints/__init__.py).

Example: rebind a few streams from the default `LCMTransport` to `ROSTransport` (defined at [`transport.py`](/dimos/core/transport.py#L226)) so you can visualize in **rviz2**.

```python skip
nav = autoconnect(
    basic,
    voxel_mapper(voxel_size=0.1),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
).global_config(n_dask_workers=6, robot_model="unitree_go2")

ros = nav.transports(
    {
        ("lidar", PointCloud2): ROSTransport("lidar", PointCloud2),
        ("global_map", PointCloud2): ROSTransport("global_map", PointCloud2),
        ("odom", PoseStamped): ROSTransport("odom", PoseStamped),
        ("color_image", Image): ROSTransport("color_image", Image),
    }
)
```

---

## Using transports with modules

Each **stream** on a module can use a different transport. Set `.transport` on the stream **before starting** modules.

```python ansi=false
import time

from dimos.core import In, Module, start
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.msgs.sensor_msgs import Image


class ImageListener(Module):
    image: In[Image]

    def start(self):
        super().start()
        self.image.subscribe(lambda img: print(f"Received: {img.shape}"))


if __name__ == "__main__":
    # Start local cluster and deploy modules to separate processes
    dimos = start(2)

    camera = dimos.deploy(CameraModule, frequency=2.0)
    listener = dimos.deploy(ImageListener)

    # Choose a transport for the stream (example: LCM typed channel)
    camera.color_image.transport = LCMTransport("/camera/rgb", Image)

    # Connect listener input to camera output
    listener.image.connect(camera.color_image)

    camera.start()
    listener.start()

    time.sleep(2)
    dimos.stop()
```

<!--Result:-->

```
Initialized dimos local cluster with 2 workers, memory limit: auto
2026-01-24T13:17:50.190559Z [info     ] Deploying module.                                            [dimos/core/__init__.py] module=CameraModule
2026-01-24T13:17:50.218466Z [info     ] Deployed module.                                             [dimos/core/__init__.py] module=CameraModule worker_id=1
2026-01-24T13:17:50.229474Z [info     ] Deploying module.                                            [dimos/core/__init__.py] module=ImageListener
2026-01-24T13:17:50.250199Z [info     ] Deployed module.                                             [dimos/core/__init__.py] module=ImageListener worker_id=0
Received: (480, 640, 3)
Received: (480, 640, 3)
Received: (480, 640, 3)
```

See [Modules](modules.md) for more on module architecture.

---

## Inspecting LCM traffic (CLI)

`lcmspy` shows topic frequency/bandwidth stats:

![lcmspy](assets/lcmspy.png)

`dimos topic echo /topic` listens on typed channels like `/topic#pkg.Msg` and decodes automatically:

```sh skip
Listening on /camera/rgb (inferring from typed LCM channels like '/camera/rgb#pkg.Msg')... (Ctrl+C to stop)
Image(shape=(480, 640, 3), format=RGB, dtype=uint8, dev=cpu, ts=2026-01-24 20:28:59)
```

---

## Implementing a transport

At the stream layer, a transport is implemented by subclassing `Transport` (see [`core/stream.py`](/dimos/core/stream.py#L83)) and implementing:

* `broadcast(...)`
* `subscribe(...)`

Your `Transport.__init__` args can be anything meaningful for your backend:

* `(ip, port)`
* a shared-memory segment name
* a filesystem path
* a Redis channel

Encoding is an implementation detail, but we encourage using LCM-compatible message types when possible.

### Encoding helpers

Many of our message types provide `lcm_encode` / `lcm_decode` for compact, language-agnostic binary encoding (often faster than pickle). For details, see [LCM](/docs/usage/lcm.md).

---

## PubSub transports

Even though transport can be anything (TCP connection, unix socket) for now all our transport backends implement the `PubSub` interface.

* `publish(topic, message)`
* `subscribe(topic, callback) -> unsubscribe`

```python
from dimos.protocol.pubsub.spec import PubSub
import inspect

print(inspect.getsource(PubSub.publish))
print(inspect.getsource(PubSub.subscribe))
```

<!--Result:-->
```python
    @abstractmethod
    def publish(self, topic: TopicT, message: MsgT) -> None:
        """Publish a message to a topic."""
        ...

    @abstractmethod
    def subscribe(
        self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
    ) -> Callable[[], None]:
        """Subscribe to a topic with a callback. returns unsubscribe function"""
        ...
```

Topic/message types are flexible: bytes, JSON, or our ROS-compatible [LCM](/docs/usage/lcm.md) types. We also have pickle-based transports for arbitrary Python objects.

### LCM (UDP multicast)

LCM is UDP multicast. It’s very fast on a robot LAN, but it’s **best-effort** (packets can drop).
For local emission it autoconfigures system in a way in which it's more robust and faster then other more common protocols like ROS, DDS

```python
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.msgs.geometry_msgs import Vector3

lcm = LCM(autoconf=True)
lcm.start()

received = []
topic = Topic("/robot/velocity", Vector3)

lcm.subscribe(topic, lambda msg, t: received.append(msg))
lcm.publish(topic, Vector3(1.0, 0.0, 0.5))

import time
time.sleep(0.1)

print(f"Received velocity: x={received[0].x}, y={received[0].y}, z={received[0].z}")
lcm.stop()
```

<!--Result:-->
```
Received velocity: x=1.0, y=0.0, z=0.5
```

### Shared memory (IPC)

Shared memory is highest performance, but only works on the **same machine**.

```python
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory

shm = PickleSharedMemory(prefer="cpu")
shm.start()

received = []
shm.subscribe("test/topic", lambda msg, topic: received.append(msg))
shm.publish("test/topic", {"data": [1, 2, 3]})

import time
time.sleep(0.1)

print(f"Received: {received}")
shm.stop()
```

<!--Result:-->
```
Received: [{'data': [1, 2, 3]}]
```

### DDS Transport

For network communication, DDS uses the Data Distribution Service (DDS) protocol:

```python session=dds_demo ansi=false
from dataclasses import dataclass
from cyclonedds.idl import IdlStruct

from dimos.protocol.pubsub.impl.ddspubsub import DDS, Topic

@dataclass
class SensorReading(IdlStruct):
    value: float

dds = DDS()
dds.start()

received = []
sensor_topic = Topic(name="sensors/temperature", data_type=SensorReading)

dds.subscribe(sensor_topic, lambda msg, t: received.append(msg))
dds.publish(sensor_topic, SensorReading(value=22.5))

import time
time.sleep(0.1)

print(f"Received: {received}")
dds.stop()
```

<!--Result:-->
```
Received: [SensorReading(value=22.5)]
```

---

## A minimal transport: `Memory`

The simplest toy backend is `Memory` (single process). Start from there when implementing a new pubsub backend.

```python
from dimos.protocol.pubsub.memory import Memory

bus = Memory()
received = []

unsubscribe = bus.subscribe("sensor/data", lambda msg, topic: received.append(msg))

bus.publish("sensor/data", {"temperature": 22.5})
bus.publish("sensor/data", {"temperature": 23.0})

print(f"Received {len(received)} messages:")
for msg in received:
    print(f"  {msg}")

unsubscribe()
```

<!--Result:-->
```
Received 2 messages:
  {'temperature': 22.5}
  {'temperature': 23.0}
```

See [`memory.py`](/dimos/protocol/pubsub/impl/memory.py) for the complete source.

---

## Encode/decode mixins

Transports often need to serialize messages before sending and deserialize after receiving.

`PubSubEncoderMixin` at [`pubsub/spec.py`](/dimos/protocol/pubsub/spec.py#L95) provides a clean way to add encoding/decoding to any pubsub implementation.

### Available mixins

| Mixin                | Encoding        | Use case                           |
|----------------------|-----------------|------------------------------------|
| `PickleEncoderMixin` | Python pickle   | Any Python object, Python-only     |
| `LCMEncoderMixin`    | LCM binary      | Cross-language (C/C++/Python/Go/…) |
| `JpegEncoderMixin`   | JPEG compressed | Image data, reduces bandwidth      |

`LCMEncoderMixin` is especially useful: you can use LCM message definitions with *any* transport (not just UDP multicast). See [LCM](/docs/usage/lcm.md) for details.

### Creating a custom mixin

```python session=jsonencoder no-result
from dimos.protocol.pubsub.spec import PubSubEncoderMixin
import json

class JsonEncoderMixin(PubSubEncoderMixin[str, dict, bytes]):
    def encode(self, msg: dict, topic: str) -> bytes:
        return json.dumps(msg).encode("utf-8")

    def decode(self, msg: bytes, topic: str) -> dict:
        return json.loads(msg.decode("utf-8"))
```

Combine with a pubsub implementation via multiple inheritance:

```python session=jsonencoder no-result
from dimos.protocol.pubsub.memory import Memory

class MyJsonPubSub(JsonEncoderMixin, Memory):
    pass
```

Swap serialization by changing the mixin:

```python session=jsonencoder no-result
from dimos.protocol.pubsub.spec import PickleEncoderMixin

class MyPicklePubSub(PickleEncoderMixin, Memory):
    pass
```

---

## Testing and benchmarks

### Spec tests

See [`pubsub/test_spec.py`](/dimos/protocol/pubsub/test_spec.py) for the grid tests your new backend should pass.

### Benchmarks

Add your backend to benchmarks to compare in context:

```sh skip
python -m pytest -svm tool -k "not bytes" dimos/protocol/pubsub/benchmark/test_benchmark.py
```

---

# Available transports

| Transport      | Use case                            | Cross-process | Network | Notes                                |
|----------------|-------------------------------------|---------------|---------|--------------------------------------|
| `Memory`       | Testing only, single process        | No            | No      | Minimal reference impl               |
| `SharedMemory` | Multi-process on same machine       | Yes           | No      | Highest throughput (IPC)             |
| `LCM`          | Robot LAN broadcast (UDP multicast) | Yes           | Yes     | Best-effort; can drop packets on LAN |
| `Redis`        | Network pubsub via Redis server     | Yes           | Yes     | Central broker; adds hop             |
| `ROS`          | ROS 2 topic communication           | Yes           | Yes     | Integrates with RViz/ROS tools       |
| `DDS`          | Cyclone DDS without ROS (WIP)       | Yes           | Yes     | WIP                                  |
