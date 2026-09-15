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

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.mapping.ray_tracing.utils.loaded_map import first_loaded_map, place_loaded_map
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

pytestmark = [pytest.mark.skipif_aarch64, pytest.mark.skipif_macos]

T0 = 1_700_000_000.0


def _cloud(x: float, ts: float) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.array([[x, 0.0, 0.0]], dtype=np.float32), frame_id="map", timestamp=ts
    )


def _world_to_map(yaw_deg: float, x: float, y: float, ts: float) -> Transform:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    m[0, 3], m[1, 3] = x, y
    tf = Transform.from_matrix(m, frame_id="world", child_frame_id="map")
    tf.ts = ts
    return tf


def test_first_loaded_map_is_none_without_the_stream(tmp_path: Path) -> None:
    with SqliteStore(path=str(tmp_path / "r.db")) as store:
        assert first_loaded_map(store, "loaded_map") is None


def test_first_loaded_map_is_the_earliest_by_ts(tmp_path: Path) -> None:
    with SqliteStore(path=str(tmp_path / "r.db")) as store:
        stream = store.stream("loaded_map", PointCloud2)
        stream.append(_cloud(2.0, T0 + 5.0), ts=T0 + 5.0)
        stream.append(_cloud(1.0, T0 + 1.0), ts=T0 + 1.0)

        obs = first_loaded_map(store, "loaded_map")

        assert obs is not None
        assert obs.ts == T0 + 1.0
        np.testing.assert_allclose(obs.data.points_f32(), [[1.0, 0.0, 0.0]])


def test_place_loaded_map_moves_points_into_the_world(tmp_path: Path) -> None:
    with SqliteStore(path=str(tmp_path / "r.db")) as store:
        store.stream("tf", TFMessage).append(
            TFMessage(_world_to_map(90.0, 1.0, 2.0, T0)), ts=T0, pose=None
        )
        tf = StreamTF(store.stream("tf", TFMessage))
        points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        loaded = store.stream("loaded_map", PointCloud2).append(
            PointCloud2.from_numpy(points, frame_id="map", timestamp=T0), ts=T0
        )

        placed = place_loaded_map(loaded, tf, "world", T0)

        np.testing.assert_allclose(placed, [[1.0, 3.0, 0.0], [0.0, 2.0, 0.0]], atol=1e-6)


def test_place_loaded_map_raises_without_a_transform(tmp_path: Path) -> None:
    with SqliteStore(path=str(tmp_path / "r.db")) as store:
        store.stream("tf", TFMessage)
        tf = StreamTF(store.stream("tf", TFMessage))
        loaded = store.stream("loaded_map", PointCloud2).append(_cloud(1.0, T0), ts=T0)

        with pytest.raises(RuntimeError, match="world->map"):
            place_loaded_map(loaded, tf, "world", T0)
