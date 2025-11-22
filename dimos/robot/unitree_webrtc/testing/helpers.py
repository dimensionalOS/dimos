# Copyright 2025 Dimensional Inc.
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

from collections.abc import Callable, Iterable
import time
from typing import Protocol

import open3d as o3d

color1 = [1, 0.706, 0]
color2 = [0, 0.651, 0.929]
color3 = [0.8, 0.196, 0.6]
color4 = [0.235, 0.702, 0.443]
color = [color1, color2, color3, color4]


# benchmarking function can return int, which will be applied to the time.
#
# (in case there is some preparation within the fuction and this time needs to be subtracted
# from the benchmark target)
def benchmark(calls: int, targetf: Callable[[], int | None]) -> float:
    start = time.time()
    timemod = 0
    for _ in range(calls):
        res = targetf()
        if res is not None:
            timemod += res
    end = time.time()
    return (end - start + timemod) * 1000 / calls


O3dDrawable = (
    o3d.geometry.Geometry
    | o3d.geometry.LineSet
    | o3d.geometry.TriangleMesh
    | o3d.geometry.PointCloud
)


class ReturnsDrawable(Protocol):
    def o3d_geometry(self) -> O3dDrawable: ...


Drawable = O3dDrawable | ReturnsDrawable


def show3d(*components: Iterable[Drawable], title: str = "open3d") -> o3d.visualization.Visualizer:
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title)
    for component in components:
        # our custom drawable components should return an open3d geometry
        if hasattr(component, "o3d_geometry"):
            vis.add_geometry(component.o3d_geometry)
        else:
            vis.add_geometry(component)

    opt = vis.get_render_option()
    opt.background_color = [0, 0, 0]
    opt.point_size = 10
    vis.poll_events()
    vis.update_renderer()
    return vis
