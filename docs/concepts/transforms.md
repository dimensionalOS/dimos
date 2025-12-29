# Transforms

Transforms describe the spatial relationship between coordinate frames in a robotics system. DimOS uses a transform system inspired by [ROS tf2](http://wiki.ros.org/tf2) to track how different parts of a robot (sensors, joints, end effectors) relate to each other in 3D space.

## Core Concepts

A **transform** represents the translation and rotation from one coordinate frame (the parent) to another (the child). For example, a camera mounted on a robot has a transform describing its position and orientation relative to the robot's base.

Transforms form a **tree structure** where frames are connected by parent-child relationships:

```
world
  └── base_link
        ├── camera_link
        │     └── camera_optical
        └── lidar_link
```

## The Transform Class

The `Transform` class at [`Transform.py`](/dimos/msgs/geometry_msgs/Transform.py#L21) represents a spatial transformation with:

- `frame_id` - The parent frame name
- `child_frame_id` - The child frame name
- `translation` - A `Vector3` (x, y, z) offset
- `rotation` - A `Quaternion` (x, y, z, w) orientation
- `ts` - Timestamp for temporal lookups

```python
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion

# Camera 0.5m forward and 0.3m up from base, no rotation
camera_transform = Transform(
    translation=Vector3(0.5, 0.0, 0.3),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # Identity rotation
    frame_id="base_link",
    child_frame_id="camera_link",
)
print(camera_transform)
```

<!--Result:-->
```
Transform:
 base_link -> camera_link Translation: → Vector Vector([0.5 0.  0.3])
  Rotation: Quaternion(0.000000, 0.000000, 0.000000, 1.000000)
```

### Transform Operations

Transforms can be composed and inverted:

```python
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion

# Create two transforms
t1 = Transform(
    translation=Vector3(1.0, 0.0, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="base_link",
    child_frame_id="camera_link",
)
t2 = Transform(
    translation=Vector3(0.0, 0.5, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="camera_link",
    child_frame_id="end_effector",
)

# Compose: base_link -> camera -> end_effector
t3 = t1 + t2
print(f"Composed: {t3.frame_id} -> {t3.child_frame_id}")
print(f"Translation: ({t3.translation.x}, {t3.translation.y}, {t3.translation.z})")

# Inverse: if t goes A -> B, -t goes B -> A
t_inverse = -t1
print(f"Inverse: {t_inverse.frame_id} -> {t_inverse.child_frame_id}")
```

<!--Result:-->
```
Composed: base_link -> end_effector
Translation: (1.0, 0.5, 0.0)
Inverse: camera_link -> base_link
```

### Converting to Matrix Form

For integration with libraries like NumPy or OpenCV:

```python
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion

t = Transform(
    translation=Vector3(1.0, 2.0, 3.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
)
matrix = t.to_matrix()
print("4x4 transformation matrix:")
print(matrix)
```

<!--Result:-->
```
4x4 transformation matrix:
[[1. 0. 0. 1.]
 [0. 1. 0. 2.]
 [0. 0. 1. 3.]
 [0. 0. 0. 1.]]
```

## Frame IDs in Modules

Modules in DimOS automatically get a `frame_id` property. This is controlled by two config options in [`core/module.py`](/dimos/core/module.py#L78):

- `frame_id` - The base frame name (defaults to the class name)
- `frame_id_prefix` - Optional prefix for namespacing

```python
from dimos.core import Module, ModuleConfig
from dataclasses import dataclass

@dataclass
class MyModuleConfig(ModuleConfig):
    frame_id: str = "sensor_link"
    frame_id_prefix: str | None = None

class MySensorModule(Module[MyModuleConfig]):
    default_config = MyModuleConfig

# With default config:
sensor = MySensorModule()
print(f"Default frame_id: {sensor.frame_id}")

# With prefix (useful for multi-robot scenarios):
sensor2 = MySensorModule(frame_id_prefix="robot1")
print(f"With prefix: {sensor2.frame_id}")
```

<!--Result:-->
```
Default frame_id: sensor_link
With prefix: robot1/sensor_link
```

## The TF Service

Every module has access to `self.tf`, a transform service that:

- **Publishes** transforms to the system
- **Looks up** transforms between any two frames
- **Buffers** historical transforms for temporal queries

The TF service is implemented in [`tf.py`](/dimos/protocol/tf/tf.py) and is lazily initialized on first access.

### Publishing Transforms

```python
from dimos.core import Module
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion

class CameraModule(Module):
    def publish_transform(self):
        camera_link = Transform(
            translation=Vector3(0.5, 0.0, 0.3),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
        )
        # Publish one or more transforms
        self.tf.publish(camera_link)

# Demo the module structure
print(f"CameraModule defined with publish_transform method")
```

<!--Result:-->
```
CameraModule defined with publish_transform method
```

### Looking Up Transforms

```python
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion
import time

# Create a transform buffer directly for demo
tf = MultiTBuffer()

# Add some transforms
t1 = Transform(
    translation=Vector3(1.0, 0.0, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="base_link",
    child_frame_id="camera_link",
    ts=time.time(),
)
t2 = Transform(
    translation=Vector3(0.0, 0.0, 0.1),
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="camera_link",
    child_frame_id="camera_optical",
    ts=time.time(),
)
tf.receive_transform(t1, t2)

# Look up direct transform
result = tf.get("base_link", "camera_link")
print(f"base_link -> camera_link: translation=({result.translation.x}, {result.translation.y}, {result.translation.z})")

# Look up chained transform (automatically composes t1 + t2)
result = tf.get("base_link", "camera_optical")
print(f"base_link -> camera_optical: translation=({result.translation.x:.2f}, {result.translation.y:.2f}, {result.translation.z:.2f})")

# Look up inverse (automatically inverts)
result = tf.get("camera_link", "base_link")
print(f"camera_link -> base_link: translation=({result.translation.x}, {result.translation.y}, {result.translation.z})")
```

<!--Result:-->
```
base_link -> camera_link: translation=(1.0, 0.0, 0.0)
base_link -> camera_optical: translation=(1.00, 0.00, 0.10)
camera_link -> base_link: translation=(-1.0, -0.0, -0.0)
```

## Example: Camera Module

The [`hardware/camera/module.py`](/dimos/hardware/camera/module.py) demonstrates a complete transform setup. The camera publishes two transforms:

1. `base_link -> camera_link` - Where the camera is mounted on the robot
2. `camera_link -> camera_optical` - The optical frame convention (Z forward, X right, Y down)

This creates the transform chain:

```
base_link -> camera_link -> camera_optical
```

## Transform Buffers

The TF service maintains a temporal buffer of transforms (default 10 seconds) allowing queries at past timestamps:

```python
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion
import time

tf = MultiTBuffer()

# Simulate transforms at different times
for i in range(5):
    t = Transform(
        translation=Vector3(float(i), 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="camera_link",
        ts=time.time() + i * 0.1,
    )
    tf.receive_transform(t)

# Query the latest transform
result = tf.get("base_link", "camera_link")
print(f"Latest transform: x={result.translation.x}")
print(f"Buffer has {len(tf.buffers)} transform pair(s)")
print(tf)
```

<!--Result:-->
```
Latest transform: x=4.0
Buffer has 1 transform pair(s)
MultiTBuffer(1 buffers):
  TBuffer(base_link -> camera_link, 5 msgs, 0.40s [2025-12-29 12:17:01 - 2025-12-29 12:17:01])
```

This is essential for sensor fusion where you need to know where the camera was when an image was captured, not where it is now.

## Further Reading

For the mathematical foundations of transforms and coordinate frames, the ROS documentation provides excellent background:

- [ROS tf2 Concepts](http://wiki.ros.org/tf2)
- [ROS REP 103 - Standard Units and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [ROS REP 105 - Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)

See also:
- [Modules](/docs/concepts/modules/index.md) for understanding the module system
- [Configuration](/docs/concepts/configuration.md) for module configuration patterns
