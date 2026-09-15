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


from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.mapping.relocalization.lidar.replay import (
    LOADED_MAP_STREAM,
    fix_error,
    place_premap,
    write_loaded_map,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _fix(yaw_deg: float, x: float = 0.0, y: float = 0.0) -> Transform:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    m[0, 3], m[1, 3] = x, y
    return Transform.from_matrix(m, frame_id="odom", child_frame_id="map")


def test_place_premap_applies_the_fix_to_every_point() -> None:
    pts = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
    placed = place_premap(pts, _fix(90.0, x=1.0))
    np.testing.assert_allclose(placed, [[1, 1, 0], [0, 0, 0]], atol=1e-6)


def test_fix_error_wraps_yaw_and_measures_translation() -> None:
    dyaw, dt = fix_error(_fix(179.0), _fix(-179.0, x=3.0, y=4.0))
    assert abs(dyaw - (-2.0)) < 1e-6
    assert abs(dt - 5.0) < 1e-6


@pytest.mark.skipif_aarch64
@pytest.mark.skipif_macos
def test_write_loaded_map_writes_once(tmp_path: Path) -> None:
    pts = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
    premap = PointCloud2.from_numpy(pts, frame_id="map")
    fix = _fix(90.0, x=1.0)
    with SqliteStore(path=str(tmp_path / "r.db")) as store:
        assert write_loaded_map(store, premap, fix, 5.0, "odom") is True
        stream = store.stream(LOADED_MAP_STREAM, PointCloud2)
        assert stream.count() == 1
        written = stream.first()
        assert written.data.frame_id == "odom"
        np.testing.assert_allclose(written.data.points_f32(), place_premap(pts, fix), atol=1e-6)

        assert write_loaded_map(store, premap, fix, 6.0, "odom") is False
        assert stream.count() == 1
