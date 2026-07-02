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

import numpy as np

from dimos.mapping.utils.cli.world_registration import WorldRegistrar
from dimos.memory2.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def _cloud(frame_id: str) -> PointCloud2:
    """Single point at the origin in ``frame_id``."""
    return PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32), frame_id=frame_id)


def _world_from_lidar(dx: float, ts: float) -> TFMessage:
    return TFMessage(
        Transform(
            translation=Vector3(dx, 0.0, 0.0), frame_id="world", child_frame_id="lidar", ts=ts
        )
    )


def test_world_frame_cloud_passes_through_untouched() -> None:
    with MemoryStore() as store:
        registrar = WorldRegistrar(store)
        cloud = _cloud("world")
        assert registrar.register_cloud(cloud, ts=1.0) is cloud
        assert registrar.skipped == 0


def test_non_world_cloud_is_registered_via_tf() -> None:
    with MemoryStore() as store:
        store.stream("tf", TFMessage).append(_world_from_lidar(1.0, ts=5.0), ts=5.0)
        registrar = WorldRegistrar(store)

        registered = registrar.register_cloud(_cloud("lidar"), ts=5.0)
        assert registered is not None
        points = registered.pointcloud_tensor.point["positions"].numpy()
        # world <- lidar shifts the origin point +1.0 along x.
        assert points[0][0] == 1.0
        assert registrar.skipped == 0


def test_non_world_cloud_skipped_without_tf_stream() -> None:
    with MemoryStore() as store:
        registrar = WorldRegistrar(store)
        assert registrar.register_cloud(_cloud("lidar"), ts=1.0) is None
        assert registrar.skipped == 1


def test_non_world_cloud_skipped_for_unknown_frame() -> None:
    with MemoryStore() as store:
        store.stream("tf", TFMessage).append(_world_from_lidar(1.0, ts=5.0), ts=5.0)
        registrar = WorldRegistrar(store)
        assert registrar.register_cloud(_cloud("camera"), ts=5.0) is None
        assert registrar.skipped == 1
