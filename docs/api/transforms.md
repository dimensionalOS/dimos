# Transforms

## The Problem: Everything Measures from Its Own Perspective

Imagine your robot has an RGB-D camera—a camera that captures both color images and depth (distance to each pixel). These are common in robotics: Intel RealSense, Microsoft Kinect, and similar sensors.

The camera spots a coffee mug at pixel (320, 240), and the depth sensor says it's 1.2 meters away. You want the robot arm to pick it up—but the arm doesn't understand pixels or camera-relative distances. It needs coordinates in its own workspace: "move to position (0.8, 0.3, 0.1) meters from my base."

To convert camera measurements to arm coordinates, you need to know:
- The camera's intrinsic parameters (focal length, sensor size) to convert pixels to a 3D direction
- The depth value to get the full 3D position relative to the camera
- Where the camera is mounted relative to the arm, and at what angle

This chain of conversions—(pixels + depth) → 3D point in camera frame → robot coordinates—is what **transforms** handle.

```
world
  └── robot_base
        ├── camera_link
        │     └── camera_optical ← mug at (0.3, 0.1, 1.2)m from camera
        └── arm_base
              └── gripper ← needs target here to pick up mug
```

Each arrow in this tree is a transform. To get the mug's position in gripper coordinates, you chain transforms through their common parent: camera → robot_base → arm → gripper.

## What's a Coordinate Frame?

A **coordinate frame** is simply a point of view—an origin point and a set of axes (X, Y, Z) from which you measure positions and orientations.

Think of it like giving directions:
- **GPS** says you're at 37.7749° N, 122.4194° W
- The **coffee shop floor plan** says "table 5 is 3 meters from the entrance"
- Your **friend** says "I'm two tables to your left"

These all describe positions in the same physical space, but from different reference points. Each is a coordinate frame.

In a robot:
- The **camera** measures in pixels, or in meters relative to its lens
- The **LIDAR** measures distances from its own mounting point
- The **robot arm** thinks in terms of its base or end-effector position
- The **world** has a fixed coordinate system everything lives in

Each sensor, joint, and reference point has its own frame.

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
base_link -> camera_link
  Translation: → Vector Vector([0.5 0.  0.3])
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
from dimos.protocol.tf import TF
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion
import time

# Create a TF service (autostart=False to skip pubsub for demo)
tf = TF(autostart=False)

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

print(
   "Look up direct transform:\n",
   tf.get("base_link", "camera_link"),
)

print(
   "Look up chained transform (automatically composes t1 + t2)\n",
   tf.get("base_link", "camera_optical")
)

print(
   "Look up inverse (automatically inverts)\n",
   tf.get("camera_link", "base_link")
)
```

<!--Result:-->
```
Look up direct transform:
 base_link -> camera_link
  Translation: → Vector Vector([1. 0. 0.])
  Rotation: Quaternion(0.000000, 0.000000, 0.000000, 1.000000)
Look up chained transform (automatically composes t1 + t2)
 base_link -> camera_optical
  Translation: → Vector Vector([1.  0.  0.1])
  Rotation: Quaternion(-0.500000, 0.500000, -0.500000, 0.500000)
Look up inverse (automatically inverts)
 camera_link -> base_link
  Translation: ← Vector Vector([-1. -0. -0.])
  Rotation: Quaternion(-0.000000, -0.000000, -0.000000, 1.000000)
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
from dimos.protocol.tf import TF
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion
import time

tf = TF(autostart=False)

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
LCMTF(1 buffers):
  TBuffer(base_link -> camera_link, 5 msgs, 0.40s [2025-12-29 12:26:10 - 2025-12-29 12:26:10])
```

This is essential for sensor fusion where you need to know where the camera was when an image was captured, not where it is now.

## Further Reading

For a visual introduction to transforms and coordinate frames:
- [Coordinate Transforms (YouTube)](https://www.youtube.com/watch?v=NGPn9nvLPmg)

For the mathematical foundations, the ROS documentation provides detailed background:

- [ROS tf2 Concepts](http://wiki.ros.org/tf2)
- [ROS REP 103 - Standard Units and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [ROS REP 105 - Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)

See also:
- [Modules](/docs/concepts/modules/index.md) for understanding the module system
- [Configuration](/docs/concepts/configuration.md) for module configuration patterns
