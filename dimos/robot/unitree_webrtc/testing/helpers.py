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
from typing import Any, Protocol

import open3d as o3d  # type: ignore[import-untyped]
from reactivex.observable import Observable

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
    def o3d_geometry(self) -> O3dDrawable: ...  # type: ignore[valid-type]


Drawable = O3dDrawable | ReturnsDrawable


def show3d(*components: Iterable[Drawable], title: str = "open3d") -> o3d.visualization.Visualizer:  # type: ignore[valid-type]
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


def multivis(*vis: o3d.visualization.Visualizer) -> None:
    while True:
        for v in vis:
            v.poll_events()
            v.update_renderer()


def show3d_stream(
    geometry_observable: Observable[Any],
    clearframe: bool = False,
    title: str = "open3d",
) -> o3d.visualization.Visualizer:
    """
    Visualize a stream of geometries using Open3D. The first geometry initializes the visualizer.
    Subsequent geometries update the visualizer. If no new geometry, just poll events.
    geometry_observable: Observable of objects with .o3d_geometry or Open3D geometry
    """
    import queue
    import threading
    import time
    from typing import Any

    q: queue.Queue[Any] = queue.Queue()
    stop_flag = threading.Event()

    def on_next(geometry: O3dDrawable) -> None:  # type: ignore[valid-type]
        q.put(geometry)

    def on_error(e: Exception) -> None:
        print(f"Visualization error: {e}")
        stop_flag.set()

    def on_completed() -> None:
        print("Geometry stream completed")
        stop_flag.set()

    subscription = geometry_observable.subscribe(
        on_next=on_next,
        on_error=on_error,
        on_completed=on_completed,
    )

    def geom(geometry: Drawable) -> O3dDrawable:  # type: ignore[valid-type]
        """Extracts the Open3D geometry from the given object."""
        return geometry.o3d_geometry if hasattr(geometry, "o3d_geometry") else geometry  # type: ignore[attr-defined, no-any-return]

    # Wait for the first geometry
    first_geometry = None
    while first_geometry is None and not stop_flag.is_set():
        try:
            first_geometry = q.get(timeout=100)
        except queue.Empty:
            print("No geometry received to visualize.")
            return

    scene_geometries = []
    first_geom_obj = geom(first_geometry)

    scene_geometries.append(first_geom_obj)

    vis = show3d(first_geom_obj, title=title)

    try:
        while not stop_flag.is_set():
            try:
                geometry = q.get_nowait()
                geom_obj = geom(geometry)
                if clearframe:
                    scene_geometries = []
                    vis.clear_geometries()

                    vis.add_geometry(geom_obj)
                    scene_geometries.append(geom_obj)
                else:
                    if geom_obj in scene_geometries:
                        print("updating existing geometry")
                        vis.update_geometry(geom_obj)
                    else:
                        print("new geometry")
                        vis.add_geometry(geom_obj)
                        scene_geometries.append(geom_obj)
            except queue.Empty:
                pass
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("closing visualizer...")
        stop_flag.set()
        vis.destroy_window()
        subscription.dispose()

    return vis
