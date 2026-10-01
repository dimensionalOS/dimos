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
from types import SimpleNamespace

import numpy as np
import pytest
from pytest_mock import MockerFixture
from scipy.spatial.transform import Rotation

from dimos.mapping.ray_tracing.utils.loaded_map import LOADED_MAP_STREAM
from dimos.mapping.relocalization.lidar.relocalize import LidarRelocalizer, RelocAttempt
from dimos.mapping.relocalization.lidar.replay import (
    _recorded_fix,
    fix_error,
    replay,
    write_loaded_map,
)
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def _fix(yaw_deg: float, x: float = 0.0, y: float = 0.0) -> Transform:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    m[0, 3], m[1, 3] = x, y
    return Transform.from_matrix(m, frame_id="odom", child_frame_id="map")


def _edge(parent: str, child: str, ts: float, x: float = 0.0) -> Transform:
    return Transform(frame_id=parent, child_frame_id=child, translation=Vector3(x, 0.0, 0.0), ts=ts)


def test_fix_error_wraps_yaw_and_measures_translation() -> None:
    dyaw, dist_m = fix_error(_fix(179.0), _fix(-179.0, x=3.0, y=4.0))
    assert abs(dyaw - (-2.0)) < 1e-6
    assert abs(dist_m - 5.0) < 1e-6


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
        assert written.ts == 5.0
        np.testing.assert_allclose(written.data.points_f32(), [[1, 1, 0], [0, 0, 0]], atol=1e-6)

        assert write_loaded_map(store, premap, fix, 6.0, "odom") is False
        assert stream.count() == 1


@pytest.mark.skipif_aarch64
@pytest.mark.skipif_macos
def test_recorded_fix_is_the_first_world_to_map_edge(tmp_path: Path) -> None:
    with SqliteStore(path=str(tmp_path / "r.db")) as store:
        assert _recorded_fix(store, "odom", "map") is None
        tf = store.stream("tf", TFMessage)
        for edge in (
            _edge("odom", "base_link", 1.0),
            _edge("odom", "map", 2.0, x=2.0),
            _edge("odom", "map", 3.0, x=3.0),
        ):
            tf.append(TFMessage(edge), ts=edge.ts, pose=None)

        recorded = _recorded_fix(store, "odom", "map")
        assert recorded is not None
        assert recorded.translation.x == 2.0
        assert _recorded_fix(store, "odom", "nowhere") is None


@pytest.mark.skipif_aarch64
@pytest.mark.skipif_macos
def test_replay_spaces_attempts_and_stops_after_the_fix(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    pytest.importorskip("dimos_voxel_ray_tracing")
    fix = _fix(90.0, x=1.0)
    attempts = [
        RelocAttempt(None, SimpleNamespace(fitness=0.2)),
        RelocAttempt(fix, SimpleNamespace(fitness=0.9)),
    ]
    mocker.patch.object(LidarRelocalizer, "_prepare", return_value=None)
    attempt = mocker.patch.object(LidarRelocalizer, "attempt", side_effect=attempts)
    span = np.arange(0.0, 1.0, 0.05)
    points = np.array([(x, y, 0.5) for x in span for y in span], dtype=np.float32)
    premap = PointCloud2.from_numpy(np.eye(3, dtype=np.float32), frame_id="map")

    with SqliteStore(path=str(tmp_path / "r.db")) as store:
        lidar = store.stream("lidar", PointCloud2)
        tf = store.stream("tf", TFMessage)
        for i in range(6):
            ts = 10.0 + i * 0.5
            tf.append(TFMessage(_edge("odom", "lidar", ts)), ts=ts, pose=None)
            lidar.append(PointCloud2.from_numpy(points, frame_id="lidar", timestamp=ts), ts=ts)
        result = replay(
            store,
            lidar_stream="lidar",
            premap=premap,
            preset="mid360",
            world_frame="odom",
            reloc_interval=1.0,
            min_local_points=1,
            voxel_size=0.1,
            after_s=0.5,
            from_time=None,
            to_time=None,
        )

    assert attempt.call_count == 2
    assert [a.t_s for a in result.attempts] == [0.0, 1.0]
    assert [a.fix for a in result.attempts] == [None, fix]
    assert result.fix is fix
    assert result.fix_ts == 11.0
    assert result.recorded is None
