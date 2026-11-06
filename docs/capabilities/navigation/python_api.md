# Navigation from Python

Drive a mobile robot from a script connected to a running blueprint. Connect with
`Dimos.connect()`, call the navigation skills on `app.skills`, and read the robot's
sensors with `app.peek_stream`. See the [Python API guide](/docs/usage/python-api.md)
for connecting and discovery.

```python skip
from dimos.porcelain.dimos import Dimos

app = Dimos.connect()
try:
    print(app.skills)          # the skills the running modules offer
    print(app.list_streams())  # the streams they publish
finally:
    app.stop()
```

Disconnecting does not stop the robot or the blueprint.

## Unitree Go2

Blueprints with `unitree-skill-container` (for example `unitree-go2` in DimSim or on the
robot) offer a blocking `move_to`:

```python skip
print(app.skills.move_to(x=3.2, y=-0.5))              # world (3.2, -0.5)
print(app.skills.move_to(x=3.2, y=-0.5, degrees=90))  # ...and face +y on arrival
print(app.skills.move_to(x=2, y=-1, relative=True))   # 2 m forward, 1 m right
print(app.skills.move_to(degrees=-90, relative=True)) # turn right 90 degrees in place
```

World coordinates are metres in the odometry frame, +x east and +y north; `degrees` is
the heading to face on arrival (0 = +x, 90 = +y). With `relative=True`, x is forward and
y is left of where the robot stands, and `degrees` is a turn. `move_to` returns once the
robot arrives or gives up, with the outcome and where it ended up.

`app.skills.execute_sport_command(name)` runs a Unitree sport command; an unknown name
returns the closest matches. `app.skills.wait(seconds)` and `app.skills.current_time()`
also exist.

| Stream | Type | Contents |
|---|---|---|
| `odom` | `PoseStamped` | base pose in the world frame |
| `odometry` | `Odometry` | base pose and velocity |
| `color_image` | `Image` | front camera |
| `camera_info` | `CameraInfo` | camera intrinsics (`K`) |
| `lidar` | `PointCloud2` | lidar points |

## Habitat

The `habitat-nav` blueprint plans with `MLSPlannerNative`. Its `go_to` starts a goal and
returns at once, so poll the robot's position to learn when it stops:

```python skip
import math
import time

print(app.skills.go_to(x=4.0, y=1.5))  # "Navigating. ..."
last = None
while True:
    time.sleep(1.0)
    pose = app.peek_stream("odometry", 2.0)
    if pose is None:
        continue
    here = (pose.x, pose.y)
    if last is not None and math.dist(here, last) < 0.02:
        break  # stopped: arrived, or short of the goal
    last = here
print(here, math.dist(here, (4.0, 1.5)))
```

A new `go_to` replaces the goal under way; `app.skills.stop_navigation()` cancels it.
The robot can stop short of an unreachable point. With `observe-skill`,
`app.skills.observe()` returns the current camera frame.

| Stream | Type | Contents |
|---|---|---|
| `odometry` | `Odometry` | base pose in the world frame |
| `color_image` | `Image` | camera RGB |
| `depth_image` | `Image` | camera depth, float32 metres |
| `camera_info` | `CameraInfo` | camera intrinsics (`K`) |
| `registered_scan` | `PointCloud2` | lidar points in the world frame |

## Cameras

`peek_stream` returns the next message or `None` if none arrives in time. Save a frame
to look at it:

```python skip
frame = app.peek_stream("color_image", 2.0)
if frame is None:
    raise RuntimeError("no camera frame within 2 s")
frame.save("view.jpg")
```
