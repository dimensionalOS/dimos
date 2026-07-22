# Obstacle repulsion

Reactive obstacle avoidance as a memory2 transform: take a lidar frame
(straight from the sensor, already world-frame), collapse it into a costmap
(same algos the `CostMapper` module uses), then sum a potential-field force
that pushes the robot away from nearby obstacle cells.

The output is a world-frame `Twist` — linear only, a directional push. It
doesn't turn the robot; rotation will be a separate transform, and the push
gets mixed with the local planner's velocity downstream.

<details>
<summary>Python</summary>

```python title="Python" fold session=repel output=none
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vis import color
from dimos.memory2.vis.space.elements import Box3D, Polyline
from dimos.memory2.vis.space.space import Space
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.obstacle_avoidance.repulsion import ObstacleRepulsion
from dimos.utils.data import get_data
```

</details>

we work against a recorded go2 walk through an office

```python title="Python" session=repel
store = SqliteStore(path=get_data("go2_hongkong_office.db"))
print(store.streams.lidar.summary())
```

```results
Stream("lidar"): 4235 items, 2026-05-06 08:12:09 — 2026-05-06 08:21:27 (558.0s, 7.59 Hz, 1.84 GiB)
```

## single frame

pick a lidar frame and run it through the transform internals: first the
costmap, then the repulsive force at the robot's position

```python title="Python" session=repel
lidar = store.streams.lidar.to_list()

obs = lidar[1000]

repel = ObstacleRepulsion()

costmap = repel.costmap(obs.data)
force = repel.force(obs.data, obs.pose.position)

print(costmap)
print(force)
```

```results
▦ OccupancyGrid[world] 159x168 (8.0x8.4m @ 20cm res) Origin: (6.82, 0.52) ▣ 6.4% □ 41.2% ◌ 52.4%
↗ Vector Vector([ 0.0409786  -0.00295715  0.        ])
```

the force is a sum over all obstacle cells within `influence_radius` (1.5m
default) of the robot, each pushing away with `gain * (1/d - 1/R) / d²`,
weighted by cell cost. let's draw the costmap, the robot (box is the go2's
actual footprint), and the suggested force as a line

```python title="Python" session=repel output=none
GO2_SIZE = Vector3(0.70, 0.31, 0.40)

def robot_box(obs, color=color.green):
    return Box3D(center=obs.pose, size=GO2_SIZE, color=color)

def force_line(obs, force, scale=1, color=color.red):
    """Suggested push drawn as a line from the robot's position."""
    p = obs.pose.position
    tip = PoseStamped(
        ts=obs.ts,
        position=(p.x + scale * force.x, p.y + scale * force.y, p.z),
    )
    return Polyline(msg=Path(poses=[obs.pose_stamped, tip]), color=color, width=0.05)

drawing = Space()
drawing.add(costmap)
drawing.add(robot_box(obs))
drawing.add(force_line(obs, force))
drawing.to_svg("assets/repulsion_single.svg")
```

![output](assets/repulsion_single.svg)

## short walks

pick any starting frames, take 20 frames from each, run them through the
transform, and draw every pose with the push we'd apply, over a map merged
from the walk's frames

```python title="Python" session=repel output=none
from functools import reduce

def render_walk(idx, count=20):
    frames = lidar[idx : idx + count]
    pushes = list(ObstacleRepulsion()(iter(frames)))

    drawing = Space()
    drawing.add(reduce(lambda a, b: a + b, (o.data for o in frames)).voxel_downsample(0.05))

    for obs, push in zip(frames, pushes):
        drawing.add(robot_box(obs))
        drawing.add(force_line(obs, push.data.linear))

    drawing.to_svg(f"assets/repulsion_walk_{idx}.svg")

for idx in [100, 500, 1000, 1200, 1400, 2000, 2300, 2500, 2700, 3000]:
    render_walk(idx)
```

![output](assets/repulsion_walk_100.svg)

![output](assets/repulsion_walk_500.svg)

![output](assets/repulsion_walk_1000.svg)

![output](assets/repulsion_walk_1200.svg)

![output](assets/repulsion_walk_1400.svg)

![output](assets/repulsion_walk_2000.svg)

![output](assets/repulsion_walk_2300.svg)

![output](assets/repulsion_walk_2500.svg)

![output](assets/repulsion_walk_2700.svg)

![output](assets/repulsion_walk_3000.svg)

## rotation

a directional push isn't enough for an elongated body: to pass a tight
corridor or corner the body has to *rotate* so it doesn't clip walls.
`ObstacleTorque` is a separate transform for that — it samples the same
repulsive field at the four corners of the footprint and sums the rigid-body
torque `τ = Σ r × F` about the center. in a corridor the leading corner feels
its wall more than the trailing one, so the body gets rotated parallel to the
walls; a symmetric pinch (doorway) cancels to zero and lets the push carry
the robot through.

to visualize a rotation we draw a **ghost box**: the footprint re-drawn at
the yaw the torque nudges the body toward. where the field wants no rotation
the ghost sits exactly on the robot; near walls it visibly twists away.

```python title="Python" session=repel output=none
from dimos.msgs.geometry_msgs.Pose import Pose as GeoPose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.navigation.obstacle_avoidance.repulsion import ObstacleTorque

def torque_ghost(obs, tau, scale=1.0, color=color.red):
    """Footprint rotated by the suggested torque (clamped to ±0.6 rad)."""
    rot = max(-0.6, min(0.6, scale * tau))
    pose = GeoPose(
        obs.pose.position,
        Quaternion.from_euler(Vector3(0.0, 0.0, obs.pose.yaw + rot)),
    )
    return Box3D(center=pose, size=GO2_SIZE, color=color, opacity=0.6)

def render_rotation_walk(idx, count=20):
    frames = lidar[idx : idx + count]
    twists = list(ObstacleTorque()(iter(frames)))

    drawing = Space()
    drawing.add(reduce(lambda a, b: a + b, (o.data for o in frames)).voxel_downsample(0.05))

    for obs, tw in zip(frames, twists):
        drawing.add(robot_box(obs))
        drawing.add(torque_ghost(obs, tw.data.angular.z, scale=2.0))

    drawing.to_svg(f"assets/torque_walk_{idx}.svg")

for idx in [100, 500, 1000, 1200, 1400, 2000, 2300, 2500, 2700, 3000]:
    render_rotation_walk(idx)
```

![output](assets/torque_walk_100.svg)

![output](assets/torque_walk_500.svg)

![output](assets/torque_walk_1000.svg)

![output](assets/torque_walk_1200.svg)

![output](assets/torque_walk_1400.svg)

![output](assets/torque_walk_2000.svg)

![output](assets/torque_walk_2300.svg)

![output](assets/torque_walk_2500.svg)

![output](assets/torque_walk_2700.svg)

![output](assets/torque_walk_3000.svg)
