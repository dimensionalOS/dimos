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

"""Read lidar+odometry recordings into world-frame frames and a trajectory."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import PointCloud2
import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry import pose_matrix
from dimos.msgs.pointcloud import pointcloud_xyz
from dimos.navigation.nav_3d.evaluator.metrics import arc_lengths

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path

    from numpy.typing import NDArray


@dataclass
class Frame:
    ts: float
    points: NDArray[np.float32]
    origin: tuple[float, float, float]


@dataclass
class Trajectory:
    """Odometry poses by time, sensor-level (T, 3) float32. Views are memoized."""

    ts: NDArray[np.float64]
    positions: NDArray[np.float32]
    _arcs: NDArray[np.float64] | None = None
    _foot: tuple[float, NDArray[np.float32]] | None = None

    def arc_lengths(self) -> NDArray[np.float64]:
        """Cumulative walked distance at each pose, starting at 0."""
        if self._arcs is None:
            self._arcs = arc_lengths(self.positions)
        return self._arcs

    def foot(self, robot_height: float) -> NDArray[np.float32]:
        """Poses dropped to foot level, the frame cases and paths are given in."""
        if self._foot is None or self._foot[0] != robot_height:
            drop = self.positions - np.array([0.0, 0.0, robot_height], dtype=np.float32)
            self._foot = (robot_height, drop)
        return self._foot[1]


def iter_world_frames(
    db_path: Path,
    lidar_stream: str,
    odom_stream: str,
    align_tol: float = 0.05,
) -> Iterator[Frame]:
    """Lidar frames placed in the world by their pose. Clouds are sensor-frame."""
    store = SqliteStore(path=str(db_path))
    with store:
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        odom = store.stream(odom_stream, Odometry).order_by("ts")
        for pair_obs in lidar.align(odom, tolerance=align_tol):
            lidar_obs, odom_obs = pair_obs.data
            if lidar_obs.data.header.frame_id == "world":
                raise ValueError(
                    f"{db_path}: stream {lidar_stream!r} has pre-registered world-frame "
                    "clouds; this legacy format is not supported for evaluation"
                )
            o = odom_obs.data
            mat = pose_matrix(o.pose.pose)
            rot = mat[:3, :3].astype(np.float32)
            trans = mat[:3, 3].astype(np.float32)
            pts = pointcloud_xyz(lidar_obs.data).astype(np.float32) @ rot.T + trans
            yield Frame(
                ts=lidar_obs.ts,
                points=pts,
                origin=(
                    float(o.pose.pose.position.x),
                    float(o.pose.pose.position.y),
                    float(o.pose.pose.position.z),
                ),
            )


def load_trajectory(db_path: Path, odom_stream: str) -> Trajectory:
    store = SqliteStore(path=str(db_path))
    ts: list[float] = []
    positions: list[tuple[float, float, float]] = []
    with store:
        for obs in store.stream(odom_stream, Odometry).order_by("ts"):
            o = obs.data
            ts.append(obs.ts)
            positions.append(
                (
                    float(o.pose.pose.position.x),
                    float(o.pose.pose.position.y),
                    float(o.pose.pose.position.z),
                )
            )
    if not positions:
        raise ValueError(f"{db_path}: no odometry in stream {odom_stream!r}")
    return Trajectory(
        ts=np.asarray(ts, dtype=np.float64),
        positions=np.asarray(positions, dtype=np.float32),
    )
