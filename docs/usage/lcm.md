# LCM transport and CDR messages

dimOS uses [LCM](https://github.com/lcm-proj/lcm) as a raw UDP multicast
transport for local inter-process communication. Typed streams carry generated
CDR messages. LCM and Zenoh use the same message bytes; choosing a transport
does not change a message's schema or codec.

Message definitions are ROS2 `.msg` files. The standalone generator produces
Python, C++ and Rust value types without requiring ROS. Python classes/codecs use rosbags; C++ uses upstream ROSIDL/Fast CDR and
native Rust types use `re_cdr`.
See [add and use a message](/docs/development/messages.md) for the complete
local-message user story and the three language examples.

## Encode a generated value

```python session=cdr_transport_demo ansi=false
from dimos_generated.geometry_msgs.msg import Vector3
from dimos_message_build.registry import encode, decode

message = Vector3(x=1.0, y=2.0, z=3.0)
payload = encode(message)
decoded = decode(payload, Vector3)
assert (decoded.x, decoded.y, decoded.z) == (1.0, 2.0, 3.0)
print(message.__msgtype__)
print(f"Decoded: x={decoded.x}, y={decoded.y}, z={decoded.z}")
```

```results
geometry_msgs/msg/Vector3
Decoded: x=1.0, y=2.0, z=3.0
```

Generated values expose ROS-shaped fields and codec/schema metadata. They do
not provide rich vector operators, implicit NumPy constructors or viewer
methods. Geometry, timestamps, arrays and visualization use explicit external
helpers. For example, add vectors with NumPy and explicitly construct the
resulting wire value:

```python session=cdr_transport_demo ansi=false
import numpy as np

first = np.array([message.x, message.y, message.z])
second = np.array([4.0, 5.0, 6.0])
summed = first + second
result = Vector3(x=summed[0], y=summed[1], z=summed[2])
assert (result.x, result.y, result.z) == (5.0, 7.0, 9.0)
print(f"dot={float(first @ second)}")
```

```results
dot=32.0
```

## Point clouds and explicit array helpers

```python session=cdr_transport_demo ansi=false
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz

points = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
cloud = pointcloud_from_xyz(points, header=Header(stamp=Time(sec=0, nanosec=0), frame_id="camera"))
roundtrip = decode(encode(cloud), PointCloud2)
np.testing.assert_array_equal(pointcloud_xyz(roundtrip), points)
print(f"PointCloud: {roundtrip.width * roundtrip.height} points")
print(f"Frame: {roundtrip.header.frame_id}")
```

```results
PointCloud: 2 points
Frame: camera
```

The generated `PointCloud2` preserves fields, offsets, row/point strides and
endianness. [Point-cloud helpers](/dimos/msgs/pointcloud.py) provide explicit
array/Open3D conversion and geometry operations. Borrowed numeric buffers are
read-only; request a copy before mutable processing.

## Typed routing and transport independence

A typed LCM channel has a qualified type suffix such as
`/velocity#geometry_msgs/msg/Vector3`. The type selects the CDR decoder; the
payload contains no LCM fingerprint. Prefer `LCMTransport` or the configured
transport factory for [module streams](/docs/usage/modules.md), so publication
and subscription use the same routing convention. Raw-byte LCM remains available
when an application owns its payload contract.

Python objects can still be passed in-process without a wire codec:

```python session=cdr_transport_demo ansi=false
from dimos.protocol.pubsub.impl.memory import Memory

memory = Memory()
received = []
unsubscribe = memory.subscribe("velocity", lambda value, topic: received.append(value))
try:
    memory.publish("velocity", message)
    assert received[0] is message
    print(f"In-process value: {received[0].x}, {received[0].y}, {received[0].z}")
finally:
    unsubscribe()
```

```results
In-process value: 1.0, 2.0, 3.0
```

For inter-process exchange, use the registry’s `encode(message)` and
`decode(payload, MessageType)` functions. Separate Python-object serialization paths retain their
own contracts; CDR is the typed-message representation.

## Available types and custom packages

| Generated package | Examples |
| --- | --- |
| `geometry_msgs` | `Vector3`, `Quaternion`, `Pose`, `PoseStamped`, `Twist`, `TransformStamped` |
| `sensor_msgs` | `Image`, `CompressedImage`, `PointCloud2`, `CameraInfo`, `LaserScan` |
| `nav_msgs` | `Odometry`, `Path`, `OccupancyGrid` |
| `vision_msgs` | `Detection2D`, `Detection3D`, `BoundingBox2D` |
| `dimos_msgs` | dimOS-owned custom message definitions |

Import built-in values from `dimos_generated.<package>.msg`. Application-local
`.msg` files can generate their own Python package, CMake target and Cargo crate.
Installed Python packages expose schemas through the `dimos.messages` entry
point, without editing a handwritten type registry. Follow the
[message authoring guide](/docs/development/messages.md) and
[native module guide](/docs/usage/native_modules.md).

The CDR cutover deliberately breaks the previous message API and wire format.
Old LCM-generated payloads and historical typed recordings are not accepted by
the new codec. Preserve old data and use the original compatible checkout to
inspect it, or create a new recording with generated messages. New MCAP files
carry `cdr` channels and complete `ros2msg` schemas for viewer inspection.
