# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The agent guide served as ``PointCloud2.agent_encode_legend()``."""

from __future__ import annotations


def legend() -> str:
    """How an agent script measures a ``PointCloud2`` with this package."""
    return (
        r"""
# Point-cloud measurement API

`cloud.agent_encode()` is a first look: `frame_id`, `ts`, `num_points`, `bounds_m`
(min and max corner), `centroid_m` and `overview`. Measure more in Python, where `cloud`
is the `PointCloud2` you were given. Every operation is a small frozen class run with
`.run(cloud)`; nothing is cached between scripts.

```python
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.experimental.agent_encode.pointcloud.queries.select import Select
from dimos.experimental.agent_encode.pointcloud.queries.bounds import Bounds
from dimos.experimental.agent_encode.pointcloud.queries.nearest import Nearest
from dimos.experimental.agent_encode.pointcloud.queries.sweep import Sweep
from dimos.experimental.agent_encode.pointcloud.queries.spacing import Spacing
from dimos.experimental.agent_encode.pointcloud.grid.count import Count
from dimos.experimental.agent_encode.pointcloud.grid.z_min import ZMin
from dimos.experimental.agent_encode.pointcloud.grid.z_max import ZMax
from dimos.experimental.agent_encode.pointcloud.grid.z_percentile import ZPercentile
from dimos.experimental.agent_encode.pointcloud.grid.occupancy import Occupancy
from dimos.experimental.agent_encode.pointcloud.image.camera import CameraView
from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Arrow, Line
from dimos.experimental.agent_encode.pointcloud.overview import Overview
```

## Conventions

- Metres in the cloud's frame: x east, y north, z up; every z is absolute. Yaw 0 faces
  +x and turns toward +y; pitch up is positive.
- No returns, and NaN in a grid, mean unobserved, never free.
- A `Grid` is top-down cells over x, y. `grid < 0.3`, `<=`, `>`, `>=` give a mask of
  1/0/no data (there is no `==`); masks combine with `&`, `|`, `~`, grids with `+`, `-`.
  `&` binds tighter than `>`: write `(a > 1) & b`. Grids combine over the cells both
  cover, at the finer cell when one `cell_m` is a whole multiple of the other.
- Every `z=(lo, hi)` is absolute, both ends included; `None` leaves an end open.
- Images: pixel (0, 0) is top-left, u right, v down. Print an image, look at its
  `path`, then re-run the same view in a later script to `pick` from it.
- Errors are `ValueError`/`TypeError` saying what to change.

## API

| Call | Use |
|---|---|
| `Box(center=(x,y,z), size=(sx,sy,sz), yaw_deg=0)` | Solid box, full extents, turned about z. |
| `Cylinder(center=(x,y), radius, z=(lo,hi))` | Vertical, the full height by default. A body-sized radius is the robot. |
| `Select(z=(lo,hi), inside=shapes, outside=shapes)` | The returns in the band (ends included), inside every `inside`, outside every `outside`: a `PointCloud2`, so any query runs on it. |
| `Bounds()` | `((x0,y0,z0),(x1,y1,z1))` of the returns, or None. |
| `Nearest(to=(x,y))` / `(x,y,z)` | `Hit` with `point_m`, `distance_m` (horizontal for 2 coordinates), or None. |
| `Sweep(shape, heading_deg=, max_m=1.0, step_m=0.05)` or `direction=(dx,dy,dz)` | Moves a body-sized shape: `distance_m` to first contact (None: clear for `max_m`), `point_m`, `start_inside` (returns inside before moving give 0; start the shape just ahead of the robot). |
| `Spacing()` | Median gap between neighbouring returns; cells finer than this leave holes. |
| `Count(cell_m, area=None)` | Returns per cell (0 is observed-empty). `area=None` covers every return. |
| `ZMin` / `ZMax(cell_m, area=None)` | Lowest / highest z per cell; NaN without returns. |
| `ZPercentile(q, cell_m, min_count=4, area=None)` | q-th percentile of z per cell; q=10 follows the lower surface. |
| `Occupancy(z=(lo,hi), cell_m, area=None)` | 1 with a return in the band, 0 with only returns below it, NaN otherwise. |
| `grid.at((x,y))`, `grid.near((x,y), radius)` | The containing cell's value; (min, max) within radius. None: no data. |
| `grid.window(((x0,y0),(x1,y1)))` | Cell values in an area, rows north to south, at most 4096 cells. |
| `mask.distance()` | Metres from each cell to the nearest true cell: clearance. |
| `mask.regions(gap=0, measure=None)` | Connected true cells, largest first; `gap` joins regions that many cells apart, `measure=grid` adds each region's `stats`. |
| `regions[i]`, `regions[:n]`, `regions.near((x,y), radius)` | `Region` with `id`, `cell_count`, `area_m2`, `centroid`, `bounds`, `stats`; `near` lists regions touching a disc. |
| `regions.gap(a, b)` | `Gap` with `distance_m` between two regions' closest cells, edge to edge, and its ends `from_m`, `to_m`: the width of the opening between two obstacles. |
| `grid.image(value_range=None, draw=())` | Top-down picture of any grid, north up, grey lines on whole metres, at most 1024 cells a side. A mask is white 0, black 1, light grey no data; other grids are coloured over `value_range`, dark grey no data. The map of a band, with the robot on it: `Occupancy(z=(lo,hi), cell_m).run(cloud).image(draw=(Arrow(p.x, p.y, math.degrees(p.yaw)),))`. |
| `CameraView(pose=(x,y,z,yaw_deg,pitch_deg), fov_deg=90, size=(768,480), max_depth=None)` | Perspective depth image, near red to far blue; black is no return. |
| `img.pixel((x,y,z))`, `img.world((u,v))` | World to pixel and back; None where nothing is shown. |
| `img.pick(uv=(u,v), radius_px=0)` or `rect=(u,v,w,h)` or `polygon=` | The returns under pixels, as a `PointCloud2`. |
| `draw=(...)` on any image | Shapes, `Hit`, sweep results, regions, gaps, clouds, `Line(((x,y,z), ...))` and `Arrow(x, y, yaw_deg, z=None)` (a pose; odometry yaw is radians: `Arrow(p.x, p.y, math.degrees(p.yaw), p.z)`); 2D points and `z=None` draw at the cloud's lowest z; `img.drawn` lists colours. |
| `Overview()` | The same numbers as `agent_encode()["overview"]`. |

`overview.lower_surface.reference_z_m` is the median 10th-percentile z per cell: a
reference height, not a verified floor. `structure` regions are cells with returns
0.15 to 1.0 m above it; they enclose returns and are not rooms or solid objects. `relief`
lists lower-surface patches over 0.15 m from it; a few cells is weak evidence, and no
patch does not prove level ground.

## Example: can the robot get from start to goal?

```python
start, goal, cell, clearance = (0.0, 0.0), (2.0, 1.0), 0.1, 0.3
(x0, y0, _), (x1, y1, _) = Bounds().run(cloud)
area = ((x0, y0), (x1, y1))  # a grid cut short closes off routes around
body = Cylinder(center=start, radius=0.35)
obstacles = Select(z=(0.15, 1.0), outside=body).run(cloud)
blocked = Count(cell, area).run(obstacles) > 0
seen = Count(cell, area).run(cloud) > 0
free = (blocked.distance() > clearance) & seen
regions = free.regions()
# a place's own cell often fails clearance, so look ~1.5 cells around it
here = {r.id for r in regions.near(start, radius=1.5 * cell)}
there = {r.id for r in regions.near(goal, radius=1.5 * cell)}
print("connected" if here & there else "not connected")
print(regions)  # the largest regions first
print(free.image(draw=(Line((start, goal)),)))  # look at the file it names
```
""".strip()
        + "\n"
    )
