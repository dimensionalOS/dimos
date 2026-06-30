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

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.object_scene_registration import _to_registered_object


@dataclass(slots=True)
class _ObjectWithPointcloud:
    object_id: str
    name: str
    center: Vector3
    size: Vector3
    frame_id: str
    ts: float
    pointcloud: PointCloud2 | None


def _pointcloud(points: np.ndarray, frame_id: str = "world") -> PointCloud2:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return PointCloud2(pointcloud=pcd, frame_id=frame_id, ts=12.5)


def test_registered_object_bounds_ignore_isolated_pointcloud_outlier() -> None:
    cluster = np.array(
        [
            [-0.02, -0.02, -0.02],
            [-0.02, 0.02, 0.02],
            [0.02, -0.02, 0.02],
            [0.02, 0.02, -0.02],
        ]
        * 8,
        dtype=np.float64,
    )
    points = np.vstack([cluster, np.array([[1.0, 1.0, 1.0]], dtype=np.float64)])
    obj = _ObjectWithPointcloud(
        object_id="obj-1",
        name="orange",
        center=Vector3(0.5, 0.5, 0.5),
        size=Vector3(1.0, 1.0, 1.0),
        frame_id="fallback",
        ts=12.5,
        pointcloud=_pointcloud(points, frame_id="camera"),
    )

    registered = _to_registered_object(obj)  # type: ignore[arg-type]

    assert registered.object_id == "obj-1"
    assert registered.name == "orange"
    assert registered.frame_id == "camera"
    assert registered.ts == 12.5
    assert abs(registered.center.x) < 0.08
    assert abs(registered.center.y) < 0.08
    assert abs(registered.center.z) < 0.08
    assert registered.size.x < 0.2
    assert registered.size.y < 0.2
    assert registered.size.z < 0.2


def test_registered_object_bounds_fall_back_to_object_metadata_without_pointcloud() -> None:
    obj = _ObjectWithPointcloud(
        object_id="obj-2",
        name="sphere",
        center=Vector3(0.4, -0.1, 0.2),
        size=Vector3(0.08, 0.07, 0.06),
        frame_id="world",
        ts=22.0,
        pointcloud=None,
    )

    registered = _to_registered_object(obj)  # type: ignore[arg-type]

    assert registered.object_id == "obj-2"
    assert registered.center == obj.center
    assert registered.size == obj.size
    assert registered.frame_id == "world"
    assert registered.ts == 22.0
