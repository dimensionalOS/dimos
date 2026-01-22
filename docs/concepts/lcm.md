
# LCM Messages

[LCM (Lightweight Communications and Marshalling)](https://github.com/lcm-proj/lcm) is a message-passing system with bindings for many languages (C, C++, Python, Java, Lua, Go). While LCM includes a UDP multicast transport, its real power is the message definition format - classes that can encode themselves to a compact binary representation.

Dimos uses LCM message definitions for all inter-module communication. Because messages serialize to binary, they can be sent over any transport - not just LCM's UDP multicast, but also shared memory, Redis, WebSockets, or any other channel.

## dimos-lcm Package

The `dimos-lcm` package provides base message types that mirror [ROS message definitions](https://docs.ros.org/en/melodic/api/sensor_msgs/html/index.html):

```python session=lcm_demo ansi=false
from dimos_lcm.geometry_msgs import Vector3 as LCMVector3
from dimos_lcm.sensor_msgs.PointCloud2 import PointCloud2 as LCMPointCloud2

# LCM messages can encode to binary
msg = LCMVector3()
msg.x, msg.y, msg.z = 1.0, 2.0, 3.0

binary = msg.lcm_encode()
print(f"Encoded to {len(binary)} bytes: {binary.hex()}")

# And decode back
decoded = LCMVector3.lcm_decode(binary)
print(f"Decoded: x={decoded.x}, y={decoded.y}, z={decoded.z}")
```

<!--Result:-->
```
Encoded to 24 bytes: 000000000000f03f00000000000000400000000000000840
Decoded: x=1.0, y=2.0, z=3.0
```

## Dimos Message Overlays

Dimos subclasses the base LCM types to add Python-friendly features while preserving binary compatibility. For example, `dimos.msgs.geometry_msgs.Vector3` extends the LCM base with:

- Multiple constructor overloads (from tuples, numpy arrays, etc.)
- Math operations (`+`, `-`, `*`, `/`, dot product, cross product)
- Conversions to numpy, quaternions, etc.

```python session=lcm_demo ansi=false
from dimos.msgs.geometry_msgs import Vector3

# Rich constructors
v1 = Vector3(1, 2, 3)
v2 = Vector3([4, 5, 6])
v3 = Vector3(v1)  # copy

# Math operations
print(f"v1 + v2 = {(v1 + v2).to_tuple()}")
print(f"v1 dot v2 = {v1.dot(v2)}")
print(f"v1 x v2 = {v1.cross(v2).to_tuple()}")
print(f"|v1| = {v1.length():.3f}")

# Still encodes to LCM binary
binary = v1.lcm_encode()
print(f"LCM encoded: {len(binary)} bytes")
```

<!--Result:-->
```
v1 + v2 = (5.0, 7.0, 9.0)
v1 dot v2 = 32.0
v1 x v2 = (-3.0, 6.0, -3.0)
|v1| = 3.742
LCM encoded: 24 bytes
```

## PointCloud2 with Open3D

A more complex example is `PointCloud2`, which wraps Open3D point clouds while maintaining LCM binary compatibility:

```python session=lcm_demo ansi=false
import numpy as np
from dimos.msgs.sensor_msgs import PointCloud2

# Create from numpy
points = np.random.rand(100, 3).astype(np.float32)
pc = PointCloud2.from_numpy(points, frame_id="camera")

print(f"PointCloud: {len(pc)} points, frame={pc.frame_id}")
print(f"Center: {pc.center}")

# Access as Open3D (for visualization, processing)
o3d_cloud = pc.pointcloud
print(f"Open3D type: {type(o3d_cloud).__name__}")

# Encode to LCM binary (for transport)
binary = pc.lcm_encode()
print(f"LCM encoded: {len(binary)} bytes")

# Decode back
pc2 = PointCloud2.lcm_decode(binary)
print(f"Decoded: {len(pc2)} points")
```

<!--Result:-->
```
PointCloud: 100 points, frame=camera
Center: ↗ Vector (Vector([0.49166839, 0.50896413, 0.48393918]))
Open3D type: PointCloud
LCM encoded: 1716 bytes
Decoded: 100 points
```

## Transport Independence

Since LCM messages encode to bytes, you can use them over any transport:

```python session=lcm_demo ansi=false
from dimos.msgs.geometry_msgs import Vector3
from dimos.protocol.pubsub.memory import Memory
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory

# Same message works with any transport
msg = Vector3(1, 2, 3)

# In-memory (same process)
memory = Memory()
received = []
memory.subscribe("velocity", lambda m, t: received.append(m))
memory.publish("velocity", msg)
print(f"Memory transport: received {received[0]}")

# The LCM binary can also be sent raw over any byte-oriented channel
binary = msg.lcm_encode()
# send over WebSocket, Redis, TCP, file, etc.
decoded = Vector3.lcm_decode(binary)
print(f"Raw binary transport: decoded {decoded}")
```

<!--Result:-->
```
Memory transport: received ↗ Vector (Vector([1. 2. 3.]))
Raw binary transport: decoded ↗ Vector (Vector([1. 2. 3.]))
```

## Available Message Types

Dimos provides overlays for common message types:

| Package | Messages |
|---------|----------|
| `geometry_msgs` | `Vector3`, `Quaternion`, `Pose`, `Twist`, `Transform` |
| `sensor_msgs` | `Image`, `PointCloud2`, `CameraInfo`, `LaserScan` |
| `nav_msgs` | `Odometry`, `Path`, `OccupancyGrid` |
| `vision_msgs` | `Detection2D`, `Detection3D`, `BoundingBox2D` |

Base LCM types (without Dimos extensions) are available in `dimos_lcm.*`.

## Creating Custom Message Types

To create a new message type:

1. Define the LCM message in `.lcm` format (or use existing `dimos_lcm` base)
2. Create a Python overlay that subclasses the LCM type
3. Add `lcm_encode()` and `lcm_decode()` methods if custom serialization is needed

See [`PointCloud2.py`](/dimos/msgs/sensor_msgs/PointCloud2.py) and [`Vector3.py`](/dimos/msgs/geometry_msgs/Vector3.py) for examples.
