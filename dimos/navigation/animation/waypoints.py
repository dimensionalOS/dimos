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

"""Click camera waypoints onto a premap; they persist to a JSON file that can be edited live."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

import numpy as np
import reactivex as rx
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype

logger = setup_logger()

WAYPOINTS_FILE = Path(__file__).parent / "waypoints.json"
WAYPOINT_COLOR = (255, 0, 255)
CAMERA_COLOR = (0, 200, 255)
# Camera height at the first and last waypoint, m; clicked heights are ignored.
HEIGHT = (0.6, 15.0)


def load_waypoints(path: Path) -> list[list[float]]:
    return json.loads(path.read_text()) if path.exists() else []


def save_waypoints(path: Path, points: list[list[float]]) -> None:
    path.write_text(json.dumps(points, indent=1))


def pending_path(path: Path) -> Path:
    """Where the edit CLI leaves what the next click should do: ``{"op": ..., "index": n}``."""
    return path.with_name(path.name + ".next")


def apply_click(
    points: list[list[float]], point: list[float], pending: dict[str, Any] | None
) -> int:
    """Place the click per the pending edit (append without one); returns its index."""
    if pending is None:
        points.append(point)
        return len(points) - 1
    index = int(pending["index"])
    if pending["op"] == "place":
        points[index] = point
    else:
        points.insert(index, point)
    return index


def spline(waypoints: NDArray[np.float64], progress: NDArray[np.float64]) -> NDArray[np.float64]:
    """Points at ``progress`` (0..1) along a chord-length spline through the waypoints.

    PCHIP rather than a cubic spline: it never overshoots, so no waves between points.
    """
    from scipy.interpolate import PchipInterpolator

    chord = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1))]
    keep = np.r_[True, np.diff(chord) > 1e-6]
    return np.asarray(PchipInterpolator(chord[keep], waypoints[keep])(progress * chord[-1]))


def camera_curve(
    waypoints: NDArray[np.float64], height: tuple[float, float] = HEIGHT, n: int = 20000
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """The camera's path: the spline, climbing linearly with ground distance. Also the distances."""
    dense = spline(waypoints, np.linspace(0.0, 1.0, n))
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(dense[:, :2], axis=0), axis=1))]
    dense[:, 2] = height[0] + (height[1] - height[0]) * arc / arc[-1]
    return dense, arc


def render_camera_path(msg: PointCloud2) -> Archetype:
    import rerun as rr

    return rr.LineStrips3D([msg.points_f32()], colors=[CAMERA_COLOR], radii=-1.5)


def render_waypoints(msg: PointCloud2) -> list[tuple[str, Archetype]]:
    """Numbered dots, plus the spline the camera flies through them."""
    import rerun as rr

    points = msg.points_f32()
    curve = (
        spline(points.astype(np.float64), np.linspace(0.0, 1.0, 500)) if len(points) > 1 else points
    )
    return [
        (
            "world/waypoints",
            rr.Points3D(
                points,
                labels=[str(i) for i in range(len(points))],
                colors=[WAYPOINT_COLOR],
                radii=-6.0,
                show_labels=True,
            ),
        ),
        ("world/waypoints/line", rr.LineStrips3D([curve], colors=[WAYPOINT_COLOR], radii=-1.0)),
    ]


class WaypointPickerConfig(ModuleConfig):
    map_file: str = "recording_go2_mid360_2026-05-29_4-45pm-PST_corrected"  # premap stem or path
    map_voxel_size: float = 0.05  # downsample the premap for the viewer; 0 keeps it as is
    waypoints_file: str = str(WAYPOINTS_FILE)
    reload_s: float = 1.0  # how often the file is checked for outside edits


class WaypointPicker(Module):
    """Publish the premap, append every viewer click as a waypoint, republish on file edits."""

    config: WaypointPickerConfig
    clicked_point: In[PointStamped]
    global_map: Out[PointCloud2]
    waypoints: Out[PointCloud2]
    camera_path: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self._path = Path(self.config.waypoints_file).absolute()
        self._mtime = -1.0

        premap = PointCloud2.lcm_decode(
            resolve_named_path(self.config.map_file, ".pc2.lcm").read_bytes()
        )
        if self.config.map_voxel_size > 0:
            premap = premap.voxel_downsample(self.config.map_voxel_size)
        # No frame: logged straight under world/, the same space clicks come back in.
        premap.frame_id = ""
        self.global_map.publish(premap)
        logger.info(f"premap {len(premap)} points, waypoints in {self._path}")

        self.register_disposable(Disposable(self.clicked_point.subscribe(self._on_click)))
        self.register_disposable(rx.interval(self.config.reload_s).subscribe(self._poll))

    def _on_click(self, msg: PointStamped) -> None:
        points = load_waypoints(self._path)
        pending_file = pending_path(self._path)
        pending = json.loads(pending_file.read_text()) if pending_file.exists() else None
        point = [round(v, 3) for v in (msg.x, msg.y, msg.z)]
        index = apply_click(points, point, pending)
        save_waypoints(self._path, points)
        pending_file.unlink(missing_ok=True)
        logger.info(f"waypoint {index}: {point} ({pending['op'] if pending else 'append'})")

    def _poll(self, _: Any) -> None:
        mtime = self._path.stat().st_mtime if self._path.exists() else 0.0
        if mtime == self._mtime:
            return
        self._mtime = mtime
        points = np.array(load_waypoints(self._path), dtype=np.float32).reshape(-1, 3)
        self.waypoints.publish(PointCloud2.from_numpy(points, frame_id="", timestamp=time.time()))
        if len(points) > 1:
            # ponytail: premap frame, so off by the relocalization fix's z (about 0.6 m on door)
            curve, _ = camera_curve(points.astype(np.float64), n=500)
            self.camera_path.publish(
                PointCloud2.from_numpy(curve, frame_id="", timestamp=time.time())
            )
