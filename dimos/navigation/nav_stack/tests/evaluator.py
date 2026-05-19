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

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any

import numpy as np
from reactivex import interval
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class Scene:
    """Hand-crafted test world for the planner."""

    voxels: np.ndarray  # (N, 3) float32 world-frame coordinates of occupied voxel centers
    voxel_size: float
    start_position: tuple[float, float, float]
    start_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    goal_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    goal_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    name: str = "scene"


def _flat_floor(
    voxel_size: float,
    extent: tuple[float, float, float, float],
    z: float = 0.0,
    holes: list[tuple[float, float, float, float]] | None = None,
) -> np.ndarray:
    """Single-layer floor at height ``z`` over ``extent=(xmin, xmax, ymin, ymax)``,
    with rectangular ``holes`` cut out (e.g. footprints of objects sitting on it)."""
    xmin, xmax, ymin, ymax = extent
    xs = np.arange(xmin, xmax + voxel_size, voxel_size)
    ys = np.arange(ymin, ymax + voxel_size, voxel_size)
    fx, fy = np.meshgrid(xs, ys, indexing="ij")
    mask = np.ones(fx.shape, dtype=bool)
    for hx_min, hx_max, hy_min, hy_max in holes or []:
        mask &= ~((fx >= hx_min) & (fx <= hx_max) & (fy >= hy_min) & (fy <= hy_max))
    return np.stack([fx[mask], fy[mask], np.full(int(mask.sum()), z)], axis=1)


def _box_shell(
    voxel_size: float,
    bounds: tuple[float, float, float, float, float, float],
    include_bottom: bool = False,
) -> np.ndarray:
    """Hollow axis-aligned box: top face + 4 side walls. No interior.

    ``bounds=(xmin, xmax, ymin, ymax, zmin, zmax)``. ``include_bottom`` defaults
    False since boxes sitting on a floor occlude their bottom face from lidar.
    """
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    xs = np.arange(xmin, xmax + voxel_size, voxel_size)
    ys = np.arange(ymin, ymax + voxel_size, voxel_size)
    zs = np.arange(zmin, zmax + voxel_size, voxel_size)

    def _grid(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
        ga, gb, gc = np.meshgrid(a, b, c, indexing="ij")
        return np.stack([ga.ravel(), gb.ravel(), gc.ravel()], axis=1)

    faces = [
        _grid(xs, ys, np.array([zmax])),  # top
        _grid(np.array([xmin]), ys, zs),  # -x wall
        _grid(np.array([xmax]), ys, zs),  # +x wall
        _grid(xs, np.array([ymin]), zs),  # -y wall
        _grid(xs, np.array([ymax]), zs),  # +y wall
    ]
    if include_bottom:
        faces.append(_grid(xs, ys, np.array([zmin])))
    return np.concatenate(faces, axis=0)


def default_scene(voxel_size: float = 0.1) -> Scene:
    """Lidar-realistic shell scene: floor with a box obstacle (shell only).

    The robot starts at x=-3 and must reach x=+3. A 1m x 1m x 0.3m box sits in
    the middle. Voxels are only on observed surfaces (lidar shells): floor
    everywhere except under the box's footprint, box top + 4 side walls, no
    interior or bottom. A stub straight-line path clips through the box; the
    real planner should route around it.
    """
    box = (-0.5, 0.5, -0.5, 0.5, 0.0, 0.3)
    floor = _flat_floor(
        voxel_size,
        extent=(-5.0, 5.0, -5.0, 5.0),
        holes=[(box[0], box[1], box[2], box[3])],
    )
    box_voxels = _box_shell(voxel_size, box)
    voxels = np.concatenate([floor, box_voxels], axis=0).astype(np.float32)

    return Scene(
        voxels=voxels,
        voxel_size=voxel_size,
        start_position=(-3.0, 0.0, 0.5),
        goal_position=(3.0, 0.0, 0.5),
        name="default_floor_with_box_shell",
    )


class EvaluatorConfig(ModuleConfig):
    world_frame: str = "world"
    body_frame: str = "body"
    publish_rate: float = 1.0  # Hz — map + odom republish for late subscribers
    goal_delay: float = 2.0  # s — wait before publishing goal so planner is ready


class Evaluator(Module):
    """Publishes a synthetic scene and evaluates the planner's returned path.

    Outputs the three inputs a planner expects (global_map, odometry, goal);
    subscribes to the planner's path output and logs basic metrics.
    """

    config: EvaluatorConfig

    global_map: Out[PointCloud2]
    odometry: Out[Odometry]
    goal: Out[PoseStamped]
    path: In[Path]

    def __init__(self, scene: Scene | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._scene: Scene = scene if scene is not None else default_scene()
        self._start_time: float = 0.0
        self._goal_published: bool = False
        self._lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self._start_time = time.time()
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        tick = interval(1.0 / self.config.publish_rate).subscribe(self._tick)
        self.register_disposable(Disposable(tick))
        logger.info("Evaluator started with scene=%s", self._scene.name)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _tick(self, _: Any) -> None:
        self._publish_map()
        self._publish_odom()
        if not self._goal_published and time.time() - self._start_time >= self.config.goal_delay:
            self._publish_goal()
            self._goal_published = True

    def _publish_map(self) -> None:
        cloud = PointCloud2.from_numpy(
            points=self._scene.voxels,
            frame_id=self.config.world_frame,
            timestamp=time.time(),
        )
        self.global_map.publish(cloud)

    def _publish_odom(self) -> None:
        x, y, z = self._scene.start_position
        qx, qy, qz, qw = self._scene.start_orientation
        odom = Odometry(
            ts=time.time(),
            frame_id=self.config.world_frame,
            child_frame_id=self.config.body_frame,
            pose=Pose(position=Vector3(x, y, z), orientation=Quaternion(qx, qy, qz, qw)),
        )
        self.odometry.publish(odom)

    def _publish_goal(self) -> None:
        x, y, z = self._scene.goal_position
        qx, qy, qz, qw = self._scene.goal_orientation
        goal = PoseStamped(
            ts=time.time(),
            frame_id=self.config.world_frame,
            position=Vector3(x, y, z),
            orientation=Quaternion(qx, qy, qz, qw),
        )
        self.goal.publish(goal)
        logger.info("Evaluator published goal at %s", self._scene.goal_position)

    def _on_path(self, path: Path) -> None:
        n = len(path.poses)
        if n == 0:
            logger.warning("Evaluator received empty path")
            return
        total_xy = 0.0
        total_z = 0.0
        for a, b in zip(path.poses, path.poses[1:], strict=False):
            dx = b.position.x - a.position.x
            dy = b.position.y - a.position.y
            dz = b.position.z - a.position.z
            total_xy += (dx * dx + dy * dy) ** 0.5
            total_z += abs(dz)
        logger.info(
            "Evaluator received path: %d poses, xy_len=%.2fm, z_traveled=%.2fm",
            n,
            total_xy,
            total_z,
        )
