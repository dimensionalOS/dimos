# Path obstacle avoidance

Global path planning against dataset costmaps. We take the existing A*
(`min_cost_astar` — a plain function: costmap, goal, start → `Path`), run it
standalone over global maps built from short walks of the
`go2_hongkong_office` dataset, and later correct the resulting path with the
repulsion field from [obstacle_avoidance](/docs/capabilities/navigation/obstacle_avoidance.md) so the
robot's footprint corners never touch obstacles.

<details>
<summary>Python</summary>

```python title="Python" fold session=pathavoid output=none
from functools import reduce

import numpy as np

from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vis import color
from dimos.memory2.vis.space.elements import Point, Polyline
from dimos.memory2.vis.space.space import Space
from dimos.msgs.geometry_msgs.Point import Point as GeoPoint
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data
```

</details>

same recording as the repulsion doc

```python title="Python" session=pathavoid
store = SqliteStore(path=get_data("go2_hongkong_office.db"))
lidar = store.streams.lidar.to_list()
print(store.streams.lidar.summary())
```

```results
Stream("lidar"): 4235 items, 2026-05-06 08:12:09 — 2026-05-06 08:21:27 (558.0s, 7.59 Hz, 1.84 GiB)
```

## corner to corner

for each starting frame we merge 20 lidar frames into a global map, collapse
it into a costmap, pick two free cells as far apart as possible (min/max of
x+y — opposite corners of the mapped area), and run A* between them

```python title="Python" session=pathavoid output=none
SPOTS = [100, 300, 500, 700, 1000, 1200, 1400, 1600, 1800, 2000,
         2100, 2300, 2500, 2700, 2900, 3000, 3200, 3400, 3600, 3800]

_costmaps = {}

def walk_costmap(idx, count=20):
    if idx not in _costmaps:
        frames = lidar[idx : idx + count]
        merged = reduce(lambda a, b: a + b, (o.data for o in frames)).voxel_downsample(0.05)
        _costmaps[idx] = height_cost_occupancy(merged)
    return _costmaps[idx]

def far_apart_free_cells(costmap, max_cost=10):
    rows, cols = np.nonzero((costmap.grid >= 0) & (costmap.grid < max_cost))
    xs = costmap.origin.position.x + (cols + 0.5) * costmap.resolution
    ys = costmap.origin.position.y + (rows + 0.5) * costmap.resolution
    i0, i1 = np.argmin(xs + ys), np.argmax(xs + ys)
    return (float(xs[i0]), float(ys[i0])), (float(xs[i1]), float(ys[i1]))

def render_path(idx):
    costmap = walk_costmap(idx)
    start, goal = far_apart_free_cells(costmap)
    path = min_cost_astar(costmap, goal, start)

    drawing = Space()
    drawing.add(costmap)
    if path is not None:
        drawing.add(Polyline(msg=path, color=color.red, width=0.06))
    drawing.add(Point(msg=GeoPoint(*start, 0.0), color=color.green, radius=0.15, label="start"))
    drawing.add(Point(msg=GeoPoint(*goal, 0.0), color=color.blue, radius=0.15, label="goal"))
    drawing.to_svg(f"assets/astar_walk_{idx}.svg")

for idx in SPOTS:
    render_path(idx)
```

![output](assets/astar_walk_100.svg)

![output](assets/astar_walk_300.svg)

![output](assets/astar_walk_500.svg)

![output](assets/astar_walk_700.svg)

![output](assets/astar_walk_1000.svg)

![output](assets/astar_walk_1200.svg)

![output](assets/astar_walk_1400.svg)

![output](assets/astar_walk_1600.svg)

![output](assets/astar_walk_1800.svg)

![output](assets/astar_walk_2000.svg)

![output](assets/astar_walk_2100.svg)

![output](assets/astar_walk_2300.svg)

![output](assets/astar_walk_2500.svg)

![output](assets/astar_walk_2700.svg)

![output](assets/astar_walk_2900.svg)

![output](assets/astar_walk_3000.svg)

![output](assets/astar_walk_3200.svg)

![output](assets/astar_walk_3400.svg)

![output](assets/astar_walk_3600.svg)

![output](assets/astar_walk_3800.svg)

## correcting the path

A* plans through grid cells — it happily hugs walls and cuts corners tighter
than the robot's body. `correct_path` fixes that: an elastic-band pass that
pushes every waypoint along the repulsive field from
[obstacle_avoidance](/docs/capabilities/navigation/obstacle_avoidance.md) (capped per iteration so it can't
jump across thin walls) while a smoothing term keeps the path taut. Endpoints
stay fixed, and every corrected waypoint carries the desired body yaw (the
path heading).

we draw the raw A* path (blue), the corrected path (red), and the go2
footprint at the desired orientation along it — green where
`footprint_clear` says the body fits, red where the spot is impossible to
navigate at that size

```python title="Python" session=pathavoid output=none
from dimos.mapping.occupancy.path_resampling import simple_resample_path
from dimos.memory2.vis.space.elements import Box3D
from dimos.msgs.geometry_msgs.Pose import Pose as GeoPose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.obstacle_avoidance.path_correction import correct_path, footprint_clear

GO2_SIZE = Vector3(0.70, 0.31, 0.40)

def render_corrected(idx):
    costmap = walk_costmap(idx)
    start, goal = far_apart_free_cells(costmap)
    path = min_cost_astar(costmap, goal, start)

    drawing = Space()
    drawing.add(costmap)
    if path is not None:
        corrected = correct_path(path, costmap)
        drawing.add(Polyline(msg=path, color=color.blue, width=0.04, opacity=0.5))
        drawing.add(Polyline(msg=corrected, color=color.red, width=0.06))
        # correction leaves waypoints unevenly spaced — resample at a
        # fixed 10cm so the footprint boxes sweep the path uniformly
        boxes = simple_resample_path(corrected, corrected.poses[-1], 0.1)
        for pose in boxes.poses:
            fits = footprint_clear(costmap, pose)
            drawing.add(Box3D(
                center=GeoPose(pose.position, pose.orientation),
                size=GO2_SIZE,
                color=color.green if fits else color.red,
                opacity=0.9,
            ))
    drawing.add(Point(msg=GeoPoint(*start, 0.0), color=color.green, radius=0.15, label="start"))
    drawing.add(Point(msg=GeoPoint(*goal, 0.0), color=color.blue, radius=0.15, label="goal"))
    drawing.to_svg(f"assets/corrected_walk_{idx}.svg")

for idx in SPOTS:
    render_corrected(idx)
```

![output](assets/corrected_walk_100.svg)

![output](assets/corrected_walk_300.svg)

![output](assets/corrected_walk_500.svg)

![output](assets/corrected_walk_700.svg)

![output](assets/corrected_walk_1000.svg)

![output](assets/corrected_walk_1200.svg)

![output](assets/corrected_walk_1400.svg)

![output](assets/corrected_walk_1600.svg)

![output](assets/corrected_walk_1800.svg)

![output](assets/corrected_walk_2000.svg)

![output](assets/corrected_walk_2100.svg)

![output](assets/corrected_walk_2300.svg)

![output](assets/corrected_walk_2500.svg)

![output](assets/corrected_walk_2700.svg)

![output](assets/corrected_walk_2900.svg)

![output](assets/corrected_walk_3000.svg)

![output](assets/corrected_walk_3200.svg)

![output](assets/corrected_walk_3400.svg)

![output](assets/corrected_walk_3600.svg)

![output](assets/corrected_walk_3800.svg)
