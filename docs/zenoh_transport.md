# Zenoh Transport Guide

## Overview

Two Zenoh transport classes are available:

| Transport | Use Case | Serialization |
|-----------|----------|---------------|
| `ZenohTransport` | Image, PointCloud2 (high-bandwidth) | Native binary (10x faster decode) |
| `pZenohTransport` | Arbitrary Python objects | Pickle |

## Quick Start

```python
from dimos.core import Module, In, Out, rpc
from dimos.core.transport import ZenohTransport, pZenohTransport
from dimos.msgs.sensor_msgs import Image


class Camera(Module):
    color: Out[Image]
    depth: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        # publish images...
        self.color.publish(color_image)
        self.depth.publish(depth_image)


def deploy(dimos):
    cam = dimos.deploy(Camera)

    # ZenohTransport requires msg_type for native encoding
    cam.color.transport = ZenohTransport("camera/color", Image)
    cam.depth.transport = ZenohTransport("camera/depth", Image)

    cam.start()
    return cam
```

## QoS Configuration

For high-bandwidth streams (images, pointclouds), use best-effort with drop:

```python
from dimos.core.transport import ZenohTransport, ZenohQoS, Reliability, CongestionControl
from dimos.msgs.sensor_msgs import Image

qos = ZenohQoS(
    reliability=Reliability.BEST_EFFORT,
    congestion_control=CongestionControl.DROP,
)

cam.color.transport = ZenohTransport("camera/color", Image, qos=qos)
cam.depth.transport = ZenohTransport("camera/depth", Image, qos=qos)
```

## QoS Options

| Parameter | Options | Use Case |
|-----------|---------|----------|
| `reliability` | `RELIABLE` (default), `BEST_EFFORT` | `BEST_EFFORT` for sensors |
| `congestion_control` | `BLOCK` (default), `DROP` | `DROP` for real-time |
| `priority` | `REAL_TIME`, `DATA` (default) | `REAL_TIME` for cmd_vel |
| `express` | `True`, `False` (default) | `True` for lowest latency |

## Subscribing

```python
class Processor(Module):
    color: In[Image]
    depth: In[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.color.subscribe(self._on_color)
        self.depth.subscribe(self._on_depth)

    def _on_color(self, img: Image) -> None:
        print(f"Got color image: {img.shape}")

    def _on_depth(self, img: Image) -> None:
        print(f"Got depth image: {img.shape}")


def deploy(dimos):
    proc = dimos.deploy(Processor)

    proc.color.transport = ZenohTransport("camera/color", Image)
    proc.depth.transport = ZenohTransport("camera/depth", Image)

    proc.start()
    return proc
```

## pZenohTransport (Pickle)

For arbitrary Python objects without native encoding:

```python
from dimos.core.transport import pZenohTransport

# Works with any picklable object
my_stream.transport = pZenohTransport("topic/name")
```

## Topic Names

Zenoh normalizes ROS-style topics automatically:
- `/camera/color` → `camera/color`
- `camera//depth/` → `camera/depth`
