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

from collections.abc import Iterator
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.mapping.ray_tracing.utils.loaded_map import first_loaded_map, place_loaded_map
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.messages.geometry_msgs.msg import TransformStamped
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.msgs.geometry import transform_from_matrix
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_seconds

pytestmark = [pytest.mark.skipif_aarch64, pytest.mark.skipif_macos]

T0 = 1_700_000_000.0


@pytest.fixture
def store(tmp_path: Path) -> Iterator[SqliteStore]:
    with SqliteStore(path=str(tmp_path / "r.db")) as s:
        yield s


def _cloud(x: float, ts: float) -> PointCloud2:
    return pointcloud_from_xyz(
        np.array([[x, 0, 0]], dtype=np.float32),
        header=Header(frame_id="map", stamp=time_from_seconds(ts)),
    )


def _world_to_map(yaw_deg: float, x: float, y: float, ts: float) -> TransformStamped:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    m[0, 3], m[1, 3] = x, y
    return TransformStamped(
        header=Header(frame_id="world", stamp=time_from_seconds(ts)),
        child_frame_id="map",
        transform=transform_from_matrix(m),
    )


def test_first_loaded_map_is_none_for_a_missing_or_empty_stream(store: SqliteStore) -> None:
    assert first_loaded_map(store, "loaded_map") is None
    store.stream("loaded_map", PointCloud2)
    assert first_loaded_map(store, "loaded_map") is None


def test_first_loaded_map_is_the_earliest_by_ts(store: SqliteStore) -> None:
    stream = store.stream("loaded_map", PointCloud2)
    stream.append(_cloud(2.0, T0 + 1.0), ts=T0 + 1.0)
    stream.append(_cloud(1.0, T0), ts=T0)

    first = first_loaded_map(store, "loaded_map")

    assert first is not None
    assert first.ts == T0
    np.testing.assert_allclose(pointcloud_xyz(first.data), [[1.0, 0.0, 0.0]])


def test_place_loaded_map_moves_points_into_the_world(store: SqliteStore) -> None:
    store.stream("tf", TFMessage).append(
        TFMessage(transforms=[_world_to_map(90.0, 1.0, 2.0, T0)]), ts=T0, pose=None
    )
    tf = StreamTF(store.stream("tf", TFMessage))
    points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
    loaded = store.stream("loaded_map", PointCloud2).append(
        pointcloud_from_xyz(points, header=Header(frame_id="map", stamp=time_from_seconds(T0))),
        ts=T0,
    )

    placed = place_loaded_map(loaded, tf, "world", T0)

    np.testing.assert_allclose(placed, [[1.0, 3.0, 0.0], [0.0, 2.0, 0.0]], atol=1e-6)


def test_place_loaded_map_raises_without_a_transform(store: SqliteStore) -> None:
    store.stream("tf", TFMessage)
    tf = StreamTF(store.stream("tf", TFMessage))
    loaded = store.stream("loaded_map", PointCloud2).append(_cloud(1.0, T0), ts=T0)

    with pytest.raises(RuntimeError, match="world->map"):
        place_loaded_map(loaded, tf, "world", T0)
